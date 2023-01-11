/*
 * Copyright (c) 2013 Johannes Berg <johannes@sipsolutions.net>
 *
 *  This file is free software: you may copy, redistribute and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation, either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  This file is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/if_vlan.h>
#include <linux/mdio.h>
#include <linux/aer.h>
#include <linux/bitops.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <net/ip6_checksum.h>
#include <linux/crc32.h>
#include "alx.h"
#include "hw.h"
#include "reg.h"

static const char alx_drv_name[] = "alx";

static void alx_free_txbuf(struct alx_tx_queue *txq, int entry)
{
	struct alx_buffer *txb = &txq->bufs[entry];

	if (dma_unmap_len(txb, size)) {
		dma_unmap_single(txq->dev,
				 dma_unmap_addr(txb, dma),
				 dma_unmap_len(txb, size),
				 DMA_TO_DEVICE);
		dma_unmap_len_set(txb, size, 0);
	}

	if (txb->skb) {
		dev_kfree_skb_any(txb->skb);
		txb->skb = NULL;
	}
}

static int alx_refill_rx_ring(struct alx_priv *alx, gfp_t gfp)
{
	struct alx_rx_queue *rxq = alx->qnapi[0]->rxq;
	struct sk_buff *skb;
	struct alx_buffer *cur_buf;
	dma_addr_t dma;
	u16 cur, next, count = 0;

	next = cur = rxq->write_idx;
	if (++next == alx->rx_ringsz)
		next = 0;
	cur_buf = &rxq->bufs[cur];

	while (!cur_buf->skb && next != rxq->read_idx) {
		struct alx_rfd *rfd = &rxq->rfd[cur];

		/*
		 * When DMA RX address is set to something like
		 * 0x....fc0, it will be very likely to cause DMA
		 * RFD overflow issue.
		 *
		 * To work around it, we apply rx skb with 64 bytes
		 * longer space, and offset the address whenever
		 * 0x....fc0 is detected.
		 */
		skb = __netdev_alloc_skb(alx->dev, alx->rxbuf_size + 64, gfp);
		if (!skb)
			break;

		if (((unsigned long)skb->data & 0xfff) == 0xfc0)
			skb_reserve(skb, 64);

		dma = dma_map_single(&alx->hw.pdev->dev,
				     skb->data, alx->rxbuf_size,
				     DMA_FROM_DEVICE);
		if (dma_mapping_error(&alx->hw.pdev->dev, dma)) {
			dev_kfree_skb(skb);
			break;
		}

		/* Unfortunately, RX descriptor buffers must be 4-byte
		 * aligned, so we can't use IP alignment.
		 */
		if (WARN_ON(dma & 3)) {
			dev_kfree_skb(skb);
			break;
		}

		cur_buf->skb = skb;
		dma_unmap_len_set(cur_buf, size, alx->rxbuf_size);
		dma_unmap_addr_set(cur_buf, dma, dma);
		rfd->addr = cpu_to_le64(dma);

		cur = next;
		if (++next == alx->rx_ringsz)
			next = 0;
		cur_buf = &rxq->bufs[cur];
		count++;
	}

	if (count) {
		/* flush all updates before updating hardware */
		wmb();
		rxq->write_idx = cur;
		alx_write_mem16(&alx->hw, ALX_RFD_PIDX, cur);
	}

	return count;
}

static struct alx_tx_queue *alx_tx_queue_mapping(struct alx_priv *alx,
						 struct sk_buff *skb)
{
	unsigned int r_idx = skb->queue_mapping;

	if (r_idx >= alx->num_txq)
		r_idx = r_idx % alx->num_txq;

	return alx->qnapi[r_idx]->txq;
}

static struct netdev_queue *alx_get_tx_queue(const struct alx_tx_queue *txq)
{
	return netdev_get_tx_queue(txq->netdev, txq->queue_idx);
}

static inline int alx_tpd_avail(struct alx_tx_queue *txq)
{
	if (txq->write_idx >= txq->read_idx)
		return txq->count + txq->read_idx - txq->write_idx - 1;
	return txq->read_idx - txq->write_idx - 1;
}

static bool alx_clean_tx_irq(struct alx_tx_queue *txq)
{
	struct alx_priv *alx;
	struct netdev_queue *tx_queue;
	u16 hw_read_idx, sw_read_idx;
	unsigned int total_bytes = 0, total_packets = 0;
	int budget = ALX_DEFAULT_TX_WORK;

	alx = netdev_priv(txq->netdev);
	tx_queue = alx_get_tx_queue(txq);

	sw_read_idx = txq->read_idx;
	hw_read_idx = alx_read_mem16(&alx->hw, txq->c_reg);

	if (sw_read_idx != hw_read_idx) {
		while (sw_read_idx != hw_read_idx && budget > 0) {
			struct sk_buff *skb;

			skb = txq->bufs[sw_read_idx].skb;
			if (skb) {
				total_bytes += skb->len;
				total_packets++;
				budget--;
			}

			alx_free_txbuf(txq, sw_read_idx);

			if (++sw_read_idx == txq->count)
				sw_read_idx = 0;
		}
		txq->read_idx = sw_read_idx;

		netdev_tx_completed_queue(tx_queue, total_packets, total_bytes);
	}

	if (netif_tx_queue_stopped(tx_queue) && netif_carrier_ok(alx->dev) &&
	    alx_tpd_avail(txq) > txq->count / 4)
		netif_tx_wake_queue(tx_queue);

	return sw_read_idx == hw_read_idx;
}

static void alx_schedule_link_check(struct alx_priv *alx)
{
	schedule_work(&alx->link_check_wk);
}

static void alx_schedule_reset(struct alx_priv *alx)
{
	schedule_work(&alx->reset_wk);
}

static int alx_clean_rx_irq(struct alx_rx_queue *rxq, int budget)
{
	struct alx_priv *alx;
	struct alx_rrd *rrd;
	struct alx_buffer *rxb;
	struct sk_buff *skb;
	u16 length, rfd_cleaned = 0;
	int work = 0;

	alx = netdev_priv(rxq->netdev);

	while (work < budget) {
		rrd = &rxq->rrd[rxq->rrd_read_idx];
		if (!(rrd->word3 & cpu_to_le32(1 << RRD_UPDATED_SHIFT)))
			break;
		rrd->word3 &= ~cpu_to_le32(1 << RRD_UPDATED_SHIFT);

		if (ALX_GET_FIELD(le32_to_cpu(rrd->word0),
				  RRD_SI) != rxq->read_idx ||
		    ALX_GET_FIELD(le32_to_cpu(rrd->word0),
				  RRD_NOR) != 1) {
			alx_schedule_reset(alx);
			return work;
		}

		rxb = &rxq->bufs[rxq->read_idx];
		dma_unmap_single(rxq->dev,
				 dma_unmap_addr(rxb, dma),
				 dma_unmap_len(rxb, size),
				 DMA_FROM_DEVICE);
		dma_unmap_len_set(rxb, size, 0);
		skb = rxb->skb;
		rxb->skb = NULL;

		if (rrd->word3 & cpu_to_le32(1 << RRD_ERR_RES_SHIFT) ||
		    rrd->word3 & cpu_to_le32(1 << RRD_ERR_LEN_SHIFT)) {
			rrd->word3 = 0;
			dev_kfree_skb_any(skb);
			goto next_pkt;
		}

		length = ALX_GET_FIELD(le32_to_cpu(rrd->word3),
				       RRD_PKTLEN) - ETH_FCS_LEN;
		skb_put(skb, length);
		skb->protocol = eth_type_trans(skb, rxq->netdev);

		skb_checksum_none_assert(skb);
		if (alx->dev->features & NETIF_F_RXCSUM &&
		    !(rrd->word3 & (cpu_to_le32(1 << RRD_ERR_L4_SHIFT) |
				    cpu_to_le32(1 << RRD_ERR_IPV4_SHIFT)))) {
			switch (ALX_GET_FIELD(le32_to_cpu(rrd->word2),
					      RRD_PID)) {
			case RRD_PID_IPV6UDP:
			case RRD_PID_IPV4UDP:
			case RRD_PID_IPV4TCP:
			case RRD_PID_IPV6TCP:
				skb->ip_summed = CHECKSUM_UNNECESSARY;
				break;
			}
		}

		napi_gro_receive(&rxq->np->napi, skb);
		work++;

next_pkt:
		if (++rxq->read_idx == rxq->count)
			rxq->read_idx = 0;
		if (++rxq->rrd_read_idx == rxq->count)
			rxq->rrd_read_idx = 0;

		if (++rfd_cleaned > ALX_RX_ALLOC_THRESH)
			rfd_cleaned -= alx_refill_rx_ring(alx, GFP_ATOMIC);
	}

	if (rfd_cleaned)
		alx_refill_rx_ring(alx, GFP_ATOMIC);

	return work;
}

static int alx_poll(struct napi_struct *napi, int budget)
{
	struct alx_napi *np = container_of(napi, struct alx_napi, napi);
	struct alx_priv *alx = np->alx;
	struct alx_hw *hw = &alx->hw;
	unsigned long flags;
	bool tx_complete = true;
	int work = 0;

	if (np->txq)
		tx_complete = alx_clean_tx_irq(np->txq);
	if (np->rxq)
		work = alx_clean_rx_irq(np->rxq, budget);

	if (!tx_complete || work == budget)
		return budget;

	napi_complete_done(&np->napi, work);

	/* enable interrupt */
	if (alx->hw.pdev->msix_enabled) {
		alx_mask_msix(hw, np->vec_idx, false);
	} else {
		spin_lock_irqsave(&alx->irq_lock, flags);
		alx->int_mask |= ALX_ISR_TX_Q0 | ALX_ISR_RX_Q0;
		alx_write_mem32(hw, ALX_IMR, alx->int_mask);
		spin_unlock_irqrestore(&alx->irq_lock, flags);
	}

	alx_post_write(hw);

	return work;
}

static bool alx_intr_handle_misc(struct alx_priv *alx, u32 intr)
{
	struct alx_hw *hw = &alx->hw;

	if (intr & ALX_ISR_FATAL) {
		netif_warn(alx, hw, alx->dev,
			   "fatal interrupt 0x%x, resetting\n", intr);
		alx_schedule_reset(alx);
		return true;
	}

	if (intr & ALX_ISR_ALERT)
		netdev_warn(alx->dev, "alert interrupt: 0x%x\n", intr);

	if (intr & ALX_ISR_PHY) {
		/* suppress PHY interrupt, because the source
		 * is from PHY internal. only the internal status
		 * is cleared, the interrupt status could be cleared.
		 */
		alx->int_mask &= ~ALX_ISR_PHY;
		alx_write_mem32(hw, ALX_IMR, alx->int_mask);
		alx_schedule_link_check(alx);
	}

	return false;
}

static irqreturn_t alx_intr_handle(struct alx_priv *alx, u32 intr)
{
	struct alx_hw *hw = &alx->hw;

	spin_lock(&alx->irq_lock);

	/* ACK interrupt */
	alx_write_mem32(hw, ALX_ISR, intr | ALX_ISR_DIS);
	intr &= alx->int_mask;

	if (alx_intr_handle_misc(alx, intr))
		goto out;

	if (intr & (ALX_ISR_TX_Q0 | ALX_ISR_RX_Q0)) {
		napi_schedule(&alx->qnapi[0]->napi);
		/* mask rx/tx interrupt, enable them when napi complete */
		alx->int_mask &= ~ALX_ISR_ALL_QUEUES;
		alx_write_mem32(hw, ALX_IMR, alx->int_mask);
	}

	alx_write_mem32(hw, ALX_ISR, 0);

 out:
	spin_unlock(&alx->irq_lock);
	return IRQ_HANDLED;
}

static irqreturn_t alx_intr_msix_ring(int irq, void *data)
{
	struct alx_napi *np = data;
	struct alx_hw *hw = &np->alx->hw;

	/* mask interrupt to ACK chip */
	alx_mask_msix(hw, np->vec_idx, true);
	/* clear interrupt status */
	alx_write_mem32(hw, ALX_ISR, np->vec_mask);

	napi_schedule(&np->napi);

	return IRQ_HANDLED;
}

static irqreturn_t alx_intr_msix_misc(int irq, void *data)
{
	struct alx_priv *alx = data;
	struct alx_hw *hw = &alx->hw;
	u32 intr;

	/* mask interrupt to ACK chip */
	alx_mask_msix(hw, 0, true);

	/* read interrupt status */
	intr = alx_read_mem32(hw, ALX_ISR);
	intr &= (alx->int_mask & ~ALX_ISR_ALL_QUEUES);

	if (alx_intr_handle_misc(alx, intr))
		return IRQ_HANDLED;

	/* clear interrupt status */
	alx_write_mem32(hw, ALX_ISR, intr);

	/* enable interrupt again */
	alx_mask_msix(hw, 0, false);

	return IRQ_HANDLED;
}

static irqreturn_t alx_intr_msi(int irq, void *data)
{
	struct alx_priv *alx = data;

	return alx_intr_handle(alx, alx_read_mem32(&alx->hw, ALX_ISR));
}

static irqreturn_t alx_intr_legacy(int irq, void *data)
{
	struct alx_priv *alx = data;
	struct alx_hw *hw = &alx->hw;
	u32 intr;

	intr = alx_read_mem32(hw, ALX_ISR);

	if (intr & ALX_ISR_DIS || !(intr & alx->int_mask))
		return IRQ_NONE;

	return alx_intr_handle(alx, intr);
}

static const u16 txring_header_reg[] = {ALX_TPD_PRI0_ADDR_LO,
					ALX_TPD_PRI1_ADDR_LO,
					ALX_TPD_PRI2_ADDR_LO,
					ALX_TPD_PRI3_ADDR_LO};

static void alx_init_ring_ptrs(struct alx_priv *alx)
{
	struct alx_hw *hw = &alx->hw;
	u32 addr_hi = ((u64)alx->descmem.dma) >> 32;
	struct alx_napi *np;
	int i;

	for (i = 0; i < alx->num_napi; i++) {
		np = alx->qnapi[i];
		if (np->txq) {
			np->txq->read_idx = 0;
			np->txq->write_idx = 0;
			alx_write_mem32(hw,
					txring_header_reg[np->txq->queue_idx],
					np->txq->tpd_dma);
		}

		if (np->rxq) {
			np->rxq->read_idx = 0;
			np->rxq->write_idx = 0;
			np->rxq->rrd_read_idx = 0;
			alx_write_mem32(hw, ALX_RRD_ADDR_LO, np->rxq->rrd_dma);
			alx_write_mem32(hw, ALX_RFD_ADDR_LO, np->rxq->rfd_dma);
		}
	}

	alx_write_mem32(hw, ALX_TX_BASE_ADDR_HI, addr_hi);
	alx_write_mem32(hw, ALX_TPD_RING_SZ, alx->tx_ringsz);

	alx_write_mem32(hw, ALX_RX_BASE_ADDR_HI, addr_hi);
	alx_write_mem32(hw, ALX_RRD_RING_SZ, alx->rx_ringsz);
	alx_write_mem32(hw, ALX_RFD_RING_SZ, alx->rx_ringsz);
	alx_write_mem32(hw, ALX_RFD_BUF_SZ, alx->rxbuf_size);

	/* load these pointers into the chip */
	alx_write_mem32(hw, ALX_SRAM9, ALX_SRAM_LOAD_PTR);
}

static void alx_free_txring_buf(struct alx_tx_queue *txq)
{
	int i;

	if (!txq->bufs)
		return;

	for (i = 0; i < txq->count; i++)
		alx_free_txbuf(txq, i);

	memset(txq->bufs, 0, txq->count * sizeof(struct alx_buffer));
	memset(txq->tpd, 0, txq->count * sizeof(struct alx_txd));
	txq->write_idx = 0;
	txq->read_idx = 0;

	netdev_tx_reset_queue(alx_get_tx_queue(txq));
}

static void alx_free_rxring_buf(struct alx_rx_queue *rxq)
{
	struct alx_buffer *cur_buf;
	u16 i;

	if (!rxq->bufs)
		return;

	for (i = 0; i < rxq->count; i++) {
		cur_buf = rxq->bufs + i;
		if (cur_buf->skb) {
			dma_unmap_single(rxq->dev,
					 dma_unmap_addr(cur_buf, dma),
					 dma_unmap_len(cur_buf, size),
					 DMA_FROM_DEVICE);
			dev_kfree_skb(cur_buf->skb);
			cur_buf->skb = NULL;
			dma_unmap_len_set(cur_buf, size, 0);
			dma_unmap_addr_set(cur_buf, dma, 0);
		}
	}

	rxq->write_idx = 0;
	rxq->read_idx = 0;
	rxq->rrd_read_idx = 0;
}

static void alx_free_buffers(struct alx_priv *alx)
{
	int i;

	for (i = 0; i < alx->num_txq; i++)
		if (alx->qnapi[i] && alx->qnapi[i]->txq)
			alx_free_txring_buf(alx->qnapi[i]->txq);

	if (alx->qnapi[0] && alx->qnapi[0]->rxq)
		alx_free_rxring_buf(alx->qnapi[0]->rxq);
}

static int alx_reinit_rings(struct alx_priv *alx)
{
	alx_free_buffers(alx);

	alx_init_ring_ptrs(alx);

	if (!alx_refill_rx_ring(alx, GFP_KERNEL))
		return -ENOMEM;

	return 0;
}

static void alx_add_mc_addr(struct alx_hw *hw, const u8 *addr, u32 *mc_hash)
{
	u32 crc32, bit, reg;

	crc32 = ether_crc(ETH_ALEN, addr);
	reg = (crc32 >> 31) & 0x1;
	bit = (crc32 >> 26) & 0x1F;

	mc_hash[reg] |= BIT(bit);
}

static void __alx_set_rx_mode(struct net_device *netdev)
{
	struct alx_priv *alx = netdev_priv(netdev);
	struct alx_hw *hw = &alx->hw;
	struct netdev_hw_addr *ha;
	u32 mc_hash[2] = {};

	if (!(netdev->flags & IFF_ALLMULTI)) {
		netdev_for_each_mc_addr(ha, netdev)
			alx_add_mc_addr(hw, ha->addr, mc_hash);

		alx_write_mem32(hw, ALX_HASH_TBL0, mc_hash[0]);
		alx_write_mem32(hw, ALX_HASH_TBL1, mc_hash[1]);
	}

	hw->rx_ctrl &= ~(ALX_MAC_CTRL_MULTIALL_EN | ALX_MAC_CTRL_PROMISC_EN);
	if (netdev->flags & IFF_PROMISC)
		hw->rx_ctrl |= ALX_MAC_CTRL_PROMISC_EN;
	if (netdev->flags & IFF_ALLMULTI)
		hw->rx_ctrl |= ALX_MAC_CTRL_MULTIALL_EN;

	alx_write_mem32(hw, ALX_MAC_CTRL, hw->rx_ctrl);
}

static void alx_set_rx_mode(struct net_device *netdev)
{
	__alx_set_rx_mode(netdev);
}

static int alx_set_mac_address(struct net_device *netdev, void *data)
{
	struct alx_priv *alx = netdev_priv(netdev);
	struct alx_hw *hw = &alx->hw;
	struct sockaddr *addr = data;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	if (netdev->addr_assign_type & NET_ADDR_RANDOM)
		netdev->addr_assign_type ^= NET_ADDR_RANDOM;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	memcpy(hw->mac_addr, addr->sa_data, netdev->addr_len);
	alx_set_macaddr(hw, hw->mac_addr);

	return 0;
}

static int alx_alloc_tx_ring(struct alx_priv *alx, struct alx_tx_queue *txq,
			     int offset)
{
	txq->bufs = kcalloc(txq->count, sizeof(struct alx_buffer), GFP_KERNEL);
	if (!txq->bufs)
		return -ENOMEM;

	txq->tpd = alx->descmem.virt + offset;
	txq->tpd_dma = alx->descmem.dma + offset;
	offset += sizeof(struct alx_txd) * txq->count;

	return offset;
}

static int alx_alloc_rx_ring(struct alx_priv *alx, struct alx_rx_queue *rxq,
			     int offset)
{
	rxq->bufs = kcalloc(rxq->count, sizeof(struct alx_buffer), GFP_KERNEL);
	if (!rxq->bufs)
		return -ENOMEM;

	rxq->rrd = alx->descmem.virt + offset;
	rxq->rrd_dma = alx->descmem.dma + offset;
	offset += sizeof(struct alx_rrd) * rxq->count;

	rxq->rfd = alx->descmem.virt + offset;
	rxq->rfd_dma = alx->descmem.dma + offset;
	offset += sizeof(struct alx_rfd) * rxq->count;

	return offset;
}

static int alx_alloc_rings(struct alx_priv *alx)
{
	int i, offset = 0;

	/* physical tx/rx ring descriptors
	 *
	 * Allocate them as a single chunk because they must not cross a
	 * 4G boundary (hardware has a single register for high 32 bits
	 * of addresses only)
	 */
