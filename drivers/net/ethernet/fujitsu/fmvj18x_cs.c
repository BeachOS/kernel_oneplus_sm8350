/*======================================================================
    fmvj18x_cs.c 2.8 2002/03/23

    A fmvj18x (and its compatibles) PCMCIA client driver

    Contributed by Shingo Fujimoto, shingo@flab.fujitsu.co.jp

    TDK LAK-CD021 and CONTEC C-NET(PC)C support added by 
    Nobuhiro Katayama, kata-n@po.iijnet.or.jp

    The PCMCIA client code is based on code written by David Hinds.
    Network code is based on the "FMV-18x driver" by Yutaka TAMIYA
    but is actually largely Donald Becker's AT1700 driver, which
    carries the following attribution:

    Written 1993-94 by Donald Becker.

    Copyright 1993 United States Government as represented by the
    Director, National Security Agency.
    
    This software may be used and distributed according to the terms
    of the GNU General Public License, incorporated herein by reference.
    
    The author may be reached as becker@scyld.com, or C/O
    Scyld Computing Corporation
    410 Severn Ave., Suite 210
    Annapolis MD 21403
   
======================================================================*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DRV_NAME	"fmvj18x_cs"
#define DRV_VERSION	"2.9"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/ioport.h>
#include <linux/crc32.h>

#include <pcmcia/cistpl.h>
#include <pcmcia/ciscode.h>
#include <pcmcia/ds.h>

#include <linux/uaccess.h>
#include <asm/io.h>

/*====================================================================*/

/* Module parameters */

MODULE_DESCRIPTION("fmvj18x and compatible PCMCIA ethernet driver");
MODULE_LICENSE("GPL");

#define INT_MODULE_PARM(n, v) static int n = v; module_param(n, int, 0)

/* SRAM configuration */
/* 0:4KB*2 TX buffer   else:8KB*2 TX buffer */
INT_MODULE_PARM(sram_config, 0);


/*====================================================================*/
/*
    PCMCIA event handlers
 */
static int fmvj18x_config(struct pcmcia_device *link);
static int fmvj18x_get_hwinfo(struct pcmcia_device *link, u_char *node_id);
static int fmvj18x_setup_mfc(struct pcmcia_device *link);
static void fmvj18x_release(struct pcmcia_device *link);
static void fmvj18x_detach(struct pcmcia_device *p_dev);

/*
    LAN controller(MBH86960A) specific routines
 */
static int fjn_config(struct net_device *dev, struct ifmap *map);
static int fjn_open(struct net_device *dev);
static int fjn_close(struct net_device *dev);
static netdev_tx_t fjn_start_xmit(struct sk_buff *skb,
					struct net_device *dev);
static irqreturn_t fjn_interrupt(int irq, void *dev_id);
static void fjn_rx(struct net_device *dev);
static void fjn_reset(struct net_device *dev);
static void set_rx_mode(struct net_device *dev);
static void fjn_tx_timeout(struct net_device *dev);
static const struct ethtool_ops netdev_ethtool_ops;

/*
    card type
 */
enum cardtype { MBH10302, MBH10304, TDK, CONTEC, LA501, UNGERMANN,
	       XXX10304, NEC, KME
};

/*
    driver specific data structure
*/
struct local_info {
	struct pcmcia_device	*p_dev;
    long open_time;
    uint tx_started:1;
    uint tx_queue;
    u_short tx_queue_len;
    enum cardtype cardtype;
    u_short sent;
    u_char __iomem *base;
};

#define MC_FILTERBREAK 64

/*====================================================================*/
/* 
    ioport offset from the base address 
 */
#define TX_STATUS               0 /* transmit status register */
#define RX_STATUS               1 /* receive status register */
#define TX_INTR                 2 /* transmit interrupt mask register */
#define RX_INTR                 3 /* receive interrupt mask register */
#define TX_MODE                 4 /* transmit mode register */
#define RX_MODE                 5 /* receive mode register */
#define CONFIG_0                6 /* configuration register 0 */
#define CONFIG_1                7 /* configuration register 1 */

#define NODE_ID                 8 /* node ID register            (bank 0) */
#define MAR_ADR                 8 /* multicast address registers (bank 1) */

#define DATAPORT                8 /* buffer mem port registers   (bank 2) */
#define TX_START               10 /* transmit start register */
#define COL_CTRL               11 /* 16 collision control register */
#define BMPR12                 12 /* reserved */
#define BMPR13                 13 /* reserved */
#define RX_SKIP                14 /* skip received packet register */

#define LAN_CTRL               16 /* LAN card control register */

#define MAC_ID               0x1a /* hardware address */
#define UNGERMANN_MAC_ID     0x18 /* UNGERMANN-BASS hardware address */

/* 
    control bits 
 */
#define ENA_TMT_OK           0x80
#define ENA_TMT_REC          0x20
#define ENA_COL              0x04
#define ENA_16_COL           0x02
#define ENA_TBUS_ERR         0x01

#define ENA_PKT_RDY          0x80
#define ENA_BUS_ERR          0x40
#define ENA_LEN_ERR          0x08
#define ENA_ALG_ERR          0x04
#define ENA_CRC_ERR          0x02
#define ENA_OVR_FLO          0x01

/* flags */
#define F_TMT_RDY            0x80 /* can accept new packet */
#define F_NET_BSY            0x40 /* carrier is detected */
#define F_TMT_OK             0x20 /* send packet successfully */
#define F_SRT_PKT            0x10 /* short packet error */
#define F_COL_ERR            0x04 /* collision error */
#define F_16_COL             0x02 /* 16 collision error */
#define F_TBUS_ERR           0x01 /* bus read error */

#define F_PKT_RDY            0x80 /* packet(s) in buffer */
#define F_BUS_ERR            0x40 /* bus read error */
#define F_LEN_ERR            0x08 /* short packet */
#define F_ALG_ERR            0x04 /* frame error */
#define F_CRC_ERR            0x02 /* CRC error */
#define F_OVR_FLO            0x01 /* overflow error */

#define F_BUF_EMP            0x40 /* receive buffer is empty */

#define F_SKP_PKT            0x05 /* drop packet in buffer */

/* default bitmaps */
#define D_TX_INTR  ( ENA_TMT_OK )
#define D_RX_INTR  ( ENA_PKT_RDY | ENA_LEN_ERR \
		   | ENA_ALG_ERR | ENA_CRC_ERR | ENA_OVR_FLO )
#define TX_STAT_M  ( F_TMT_RDY )
#define RX_STAT_M  ( F_PKT_RDY | F_LEN_ERR \
                   | F_ALG_ERR | F_CRC_ERR | F_OVR_FLO )

/* commands */
#define D_TX_MODE            0x06 /* no tests, detect carrier */
#define ID_MATCHED           0x02 /* (RX_MODE) */
#define RECV_ALL             0x03 /* (RX_MODE) */
#define CONFIG0_DFL          0x5a /* 16bit bus, 4K x 2 Tx queues */
#define CONFIG0_DFL_1        0x5e /* 16bit bus, 8K x 2 Tx queues */
#define CONFIG0_RST          0xda /* Data Link Controller off (CONFIG_0) */
#define CONFIG0_RST_1        0xde /* Data Link Controller off (CONFIG_0) */
#define BANK_0               0xa0 /* bank 0 (CONFIG_1) */
#define BANK_1               0xa4 /* bank 1 (CONFIG_1) */
#define BANK_2               0xa8 /* bank 2 (CONFIG_1) */
#define CHIP_OFF             0x80 /* contrl chip power off (CONFIG_1) */
#define DO_TX                0x80 /* do transmit packet */
#define SEND_PKT             0x81 /* send a packet */
#define AUTO_MODE            0x07 /* Auto skip packet on 16 col detected */
#define MANU_MODE            0x03 /* Stop and skip packet on 16 col */
#define TDK_AUTO_MODE        0x47 /* Auto skip packet on 16 col detected */
#define TDK_MANU_MODE        0x43 /* Stop and skip packet on 16 col */
#define INTR_OFF             0x0d /* LAN controller ignores interrupts */
#define INTR_ON              0x1d /* LAN controller will catch interrupts */

#define TX_TIMEOUT		((400*HZ)/1000)

#define BANK_0U              0x20 /* bank 0 (CONFIG_1) */
#define BANK_1U              0x24 /* bank 1 (CONFIG_1) */
#define BANK_2U              0x28 /* bank 2 (CONFIG_1) */

static const struct net_device_ops fjn_netdev_ops = {
	.ndo_open 		= fjn_open,
	.ndo_stop		= fjn_close,
	.ndo_start_xmit 	= fjn_start_xmit,
	.ndo_tx_timeout 	= fjn_tx_timeout,
	.ndo_set_config 	= fjn_config,
	.ndo_set_rx_mode	= set_rx_mode,
	.ndo_set_mac_address 	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

static int fmvj18x_probe(struct pcmcia_device *link)
{
    struct local_info *lp;
    struct net_device *dev;

    dev_dbg(&link->dev, "fmvj18x_attach()\n");

    /* Make up a FMVJ18x specific data structure */
    dev = alloc_etherdev(sizeof(struct local_info));
    if (!dev)
	return -ENOMEM;
    lp = netdev_priv(dev);
    link->priv = dev;
    lp->p_dev = link;
    lp->base = NULL;

    /* The io structure describes IO port mapping */
    link->resource[0]->end = 32;
    link->resource[0]->flags |= IO_DATA_PATH_WIDTH_AUTO;

    /* General socket configuration */
    link->config_flags |= CONF_ENABLE_IRQ;

    dev->netdev_ops = &fjn_netdev_ops;
    dev->watchdog_timeo = TX_TIMEOUT;

    dev->ethtool_ops = &netdev_ethtool_ops;

    return fmvj18x_config(link);
} /* fmvj18x_attach */

/*====================================================================*/

static void fmvj18x_detach(struct pcmcia_device *link)
{
    struct net_device *dev = link->priv;

    dev_dbg(&link->dev, "fmvj18x_detach\n");

    unregister_netdev(dev);

    fmvj18x_release(link);

    free_netdev(dev);
} /* fmvj18x_detach */

/*====================================================================*/

static int mfc_try_io_port(struct pcmcia_device *link)
{
    int i, ret;
    static const unsigned int serial_base[5] =
	{ 0x3f8, 0x2f8, 0x3e8, 0x2e8, 0x0 };

    for (i = 0; i < 5; i++) {
	link->resource[1]->start = serial_base[i];
	link->resource[1]->flags |= IO_DATA_PATH_WIDTH_8;
	if (link->resource[1]->start == 0) {
	    link->resource[1]->end = 0;
	    pr_notice("out of resource for serial\n");
	}
	ret = pcmcia_request_io(link);
	if (ret == 0)
		return ret;
    }
    return ret;
}

static int ungermann_try_io_port(struct pcmcia_device *link)
{
    int ret;
    unsigned int ioaddr;
    /*
	Ungermann-Bass Access/CARD accepts 0x300,0x320,0x340,0x360
	0x380,0x3c0 only for ioport.
    */
    for (ioaddr = 0x300; ioaddr < 0x3e0; ioaddr += 0x20) {
	link->resource[0]->start = ioaddr;
	ret = pcmcia_request_io(link);
	if (ret == 0) {
	    /* calculate ConfigIndex value */
	    link->config_index =
		((link->resource[0]->start & 0x0f0) >> 3) | 0x22;
	    return ret;
	}
    }
    return ret;	/* RequestIO failed */
}

static int fmvj18x_ioprobe(struct pcmcia_device *p_dev, void *priv_data)
{
	return 0; /* strange, but that's what the code did already before... */
}

static int fmvj18x_config(struct pcmcia_device *link)
{
    struct net_device *dev = link->priv;
    struct local_info *lp = netdev_priv(dev);
    int i, ret;
    unsigned int ioaddr;
    enum cardtype cardtype;
    char *card_name = "unknown";
    u8 *buf;
    size_t len;
    u_char buggybuf[32];

    dev_dbg(&link->dev, "fmvj18x_config\n");

    link->io_lines = 5;

    len = pcmcia_get_tuple(link, CISTPL_FUNCE, &buf);
    kfree(buf);

    if (len) {
	/* Yes, I have CISTPL_FUNCE. Let's check CISTPL_MANFID */
	ret = pcmcia_loop_config(link, fmvj18x_ioprobe, NULL);
	if (ret != 0)
		goto failed;

	switch (link->manf_id) {
	case MANFID_TDK:
	    cardtype = TDK;
	    if (link->card_id == PRODID_TDK_GN3410 ||
		link->card_id == PRODID_TDK_NP9610 ||
		link->card_id == PRODID_TDK_MN3200) {
		/* MultiFunction Card */
		link->config_base = 0x800;
		link->config_index = 0x47;
		link->resource[1]->end = 8;
	    }
	    break;
	case MANFID_NEC:
	    cardtype = NEC; /* MultiFunction Card */
	    link->config_base = 0x800;
	    link->config_index = 0x47;
	    link->resource[1]->end = 8;
	    break;
	case MANFID_KME:
	    cardtype = KME; /* MultiFunction Card */
	    link->config_base = 0x800;
	    link->config_index = 0x47;
	    link->resource[1]->end = 8;
	    break;
	case MANFID_CONTEC:
	    cardtype = CONTEC;
	    break;
	case MANFID_FUJITSU:
	    if (link->config_base == 0x0fe0)
		cardtype = MBH10302;
	    else if (link->card_id == PRODID_FUJITSU_MBH10302) 
                /* RATOC REX-5588/9822/4886's PRODID are 0004(=MBH10302),
                   but these are MBH10304 based card. */ 
		cardtype = MBH10304;
	    else if (link->card_id == PRODID_FUJITSU_MBH10304)
		cardtype = MBH10304;
	    else
		cardtype = LA501;
	    break;
	default:
	    cardtype = MBH10304;
	}
    } else {
	/* old type card */
	switch (link->manf_id) {
	case MANFID_FUJITSU:
	    if (link->card_id == PRODID_FUJITSU_MBH10304) {
		cardtype = XXX10304;    /* MBH10304 with buggy CIS */
		link->config_index = 0x20;
	    } else {
		cardtype = MBH10302;    /* NextCom NC5310, etc. */
		link->config_index = 1;
	    }
	    break;
	case MANFID_UNGERMANN:
	    cardtype = UNGERMANN;
	    break;
	default:
	    cardtype = MBH10302;
	    link->config_index = 1;
	}
    }

    if (link->resource[1]->end != 0) {
	ret = mfc_try_io_port(link);
	if (ret != 0) goto failed;
    } else if (cardtype == UNGERMANN) {
	ret = ungermann_try_io_port(link);
	if (ret != 0) goto failed;
    } else { 
	    ret = pcmcia_request_io(link);
	    if (ret)
		    goto failed;
    }
    ret = pcmcia_request_irq(link, fjn_interrupt);
    if (ret)
	    goto failed;
    ret = pcmcia_enable_device(link);
    if (ret)
	    goto failed;

    dev->irq = link->irq;
    dev->base_addr = link->resource[0]->start;

    if (resource_size(link->resource[1]) != 0) {
	ret = fmvj18x_setup_mfc(link);
	if (ret != 0) goto failed;
    }

    ioaddr = dev->base_addr;

    /* Reset controller */
    if (sram_config == 0) 
	outb(CONFIG0_RST, ioaddr + CONFIG_0);
    else
	outb(CONFIG0_RST_1, ioaddr + CONFIG_0);

    /* Power On chip and select bank 0 */
    if (cardtype == MBH10302)
	outb(BANK_0, ioaddr + CONFIG_1);
    else
	outb(BANK_0U, ioaddr + CONFIG_1);
    
    /* Set hardware address */
    switch (cardtype) {
    case MBH10304:
    case TDK:
    case LA501:
    case CONTEC:
    case NEC:
    case KME:
	if (cardtype == MBH10304) {
	    card_name = "FMV-J182";

	    len = pcmcia_get_tuple(link, CISTPL_FUNCE, &buf);
	    if (len < 11) {
		    kfree(buf);
		    goto failed;
	    }
	    /* Read MACID from CIS */
	    for (i = 0; i < 6; i++)
		    dev->dev_addr[i] = buf[i + 5];
	    kfree(buf);
	} else {
	    if (pcmcia_get_mac_from_cis(link, dev))
		goto failed;
	    if( cardtype == TDK ) {
		card_name = "TDK LAK-CD021";
	    } else if( cardtype == LA501 ) {
		card_name = "LA501";
	    } else if( cardtype == NEC ) {
		card_name = "PK-UG-J001";
	    } else if( cardtype == KME ) {
		card_name = "Panasonic";
	    } else {
		card_name = "C-NET(PC)C";
	    }
	}
	break;
    case UNGERMANN:
	/* Read MACID from register */
	for (i = 0; i < 6; i++) 
	    dev->dev_addr[i] = inb(ioaddr + UNGERMANN_MAC_ID + i);
	card_name = "Access/CARD";
	break;
    case XXX10304:
	/* Read MACID from Buggy CIS */
	if (fmvj18x_get_hwinfo(link, buggybuf) == -1) {
	    pr_notice("unable to read hardware net address\n");
	    goto failed;
	}
	for (i = 0 ; i < 6; i++) {
	    dev->dev_addr[i] = buggybuf[i];
	}
	card_name = "FMV-J182";
	break;
    case MBH10302:
    default:
	/* Read MACID from register */
	for (i = 0; i < 6; i++) 
	    dev->dev_addr[i] = inb(ioaddr + MAC_ID + i);
	card_name = "FMV-J181";
	break;
    }

    lp->cardtype = cardtype;
    SET_NETDEV_DEV(dev, &link->dev);

    if (register_netdev(dev) != 0) {
	pr_notice("register_netdev() failed\n");
	goto failed;
    }

    /* print current configuration */
    netdev_info(dev, "%s, sram %s, port %#3lx, irq %d, hw_addr %pM\n",
		card_name, sram_config == 0 ? "4K TX*2" : "8K TX*2",
		dev->base_addr, dev->irq, dev->dev_addr);

    return 0;
    
failed:
    fmvj18x_release(link);
    return -ENODEV;
} /* fmvj18x_config */
/*====================================================================*/

static int fmvj18x_get_hwinfo(struct pcmcia_device *link, u_char *node_id)
{
    u_char __iomem *base;
    int i, j;

    /* Allocate a small memory window */
    link->resource[2]->flags |= WIN_DATA_WIDTH_8|WIN_MEMORY_TYPE_AM|WIN_ENABLE;
    link->resource[2]->start = 0; link->resource[2]->end = 0;
    i = pcmcia_request_window(link, link->resource[2], 0);
    if (i != 0)
	return -1;

    base = ioremap(link->resource[2]->start, resource_size(link->resource[2]));
    if (!base) {
