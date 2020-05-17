// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6368 Ethernet Switch Controller Driver
 *
 * Copyright (C) 2020 Álvaro Fernández Rojas <noltari@gmail.com>
 * Copyright (C) 2015 Jonas Gorski <jonas.gorski@gmail.com>
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 */

#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/of_clk.h>
#include <linux/of_net.h>
#include <linux/platform_data/b53.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>

/* default number of descriptor */
#define ENETSW_DEF_RX_DESC			64
#define ENETSW_DEF_TX_DESC			32
#define ENETSW_DEF_CPY_BREAK			128

/* maximum burst len for dma (4 bytes unit) */
#define ENETSW_DMA_MAXBURST			8

/* Number of ports */
#define ENETSW_MAX_PORT				8
#define ENETSW_RGMII_PORT0			4

/* Port traffic control */
#define ENETSW_PTCTRL_REG(x)			(0x0 + (x))
#define ENETSW_PTCTRL_RXDIS_MASK		(1 << 0)
#define ENETSW_PTCTRL_TXDIS_MASK		(1 << 1)

/* Switch mode register */
#define ENETSW_SWMODE_REG			(0xb)
#define ENETSW_SWMODE_FWD_EN_MASK		(1 << 1)

/* IMP override Register */
#define ENETSW_IMPOV_REG			(0xe)
#define ENETSW_IMPOV_FORCE_MASK			(1 << 7)
#define ENETSW_IMPOV_TXFLOW_MASK		(1 << 5)
#define ENETSW_IMPOV_RXFLOW_MASK		(1 << 4)
#define ENETSW_IMPOV_1000_MASK			(1 << 3)
#define ENETSW_IMPOV_100_MASK			(1 << 2)
#define ENETSW_IMPOV_FDX_MASK			(1 << 1)
#define ENETSW_IMPOV_LINKUP_MASK		(1 << 0)

/* Port override Register */
#define ENETSW_PORTOV_REG(x)			(0x58 + (x))
#define ENETSW_PORTOV_ENABLE_MASK		(1 << 6)
#define ENETSW_PORTOV_TXFLOW_MASK		(1 << 5)
#define ENETSW_PORTOV_RXFLOW_MASK		(1 << 4)
#define ENETSW_PORTOV_1000_MASK			(1 << 3)
#define ENETSW_PORTOV_100_MASK			(1 << 2)
#define ENETSW_PORTOV_FDX_MASK			(1 << 1)
#define ENETSW_PORTOV_LINKUP_MASK		(1 << 0)

/* Port RGMII control register */
#define ENETSW_RGMII_CTRL_REG(x)		(0x60 + (x))
#define ENETSW_RGMII_CTRL_GMII_CLK_EN		(1 << 7)
#define ENETSW_RGMII_CTRL_MII_OVERRIDE_EN	(1 << 6)
#define ENETSW_RGMII_CTRL_MII_MODE_MASK		(3 << 4)
#define ENETSW_RGMII_CTRL_RGMII_MODE		(0 << 4)
#define ENETSW_RGMII_CTRL_MII_MODE		(1 << 4)
#define ENETSW_RGMII_CTRL_RVMII_MODE		(2 << 4)
#define ENETSW_RGMII_CTRL_TIMING_SEL_EN		(1 << 0)

/* Port RGMII timing register */
#define ENETSW_RGMII_TIMING_REG(x)		(0x68 + (x))

/* MDIO control register */
#define ENETSW_MDIOC_REG			(0xb0)
#define ENETSW_MDIOC_EXT_MASK			(1 << 16)
#define ENETSW_MDIOC_REG_SHIFT			20
#define ENETSW_MDIOC_PHYID_SHIFT		25
#define ENETSW_MDIOC_RD_MASK			(1 << 30)
#define ENETSW_MDIOC_WR_MASK			(1 << 31)

/* MDIO data register */
#define ENETSW_MDIOD_REG			(0xb4)

/* Global Management Configuration Register */
#define ENETSW_GMCR_REG				(0x200)
#define ENETSW_GMCR_RST_MIB_MASK		(1 << 0)

/* MIB register */
#define ENETSW_MIB_REG(x)			(0x2800 + (x) * 4)
#define ENETSW_MIB_REG_COUNT			47

/* Jumbo control register port mask register */
#define ENETSW_JMBCTL_PORT_REG			(0x4004)

/* Jumbo control mib good frame register */
#define ENETSW_JMBCTL_MAXSIZE_REG		(0x4008)

/* MIB Counters register definitions */
#define ENETSW_MIB_TX_ALL_OCT			0
#define ENETSW_MIB_TX_DROP_PKTS			2
#define ENETSW_MIB_TX_QOS_PKTS			3
#define ENETSW_MIB_TX_BRDCAST			4
#define ENETSW_MIB_TX_MULT			5
#define ENETSW_MIB_TX_UNI			6
#define ENETSW_MIB_TX_COL			7
#define ENETSW_MIB_TX_1_COL			8
#define ENETSW_MIB_TX_M_COL			9
#define ENETSW_MIB_TX_DEF			10
#define ENETSW_MIB_TX_LATE			11
#define ENETSW_MIB_TX_EX_COL			12
#define ENETSW_MIB_TX_PAUSE			14
#define ENETSW_MIB_TX_QOS_OCT			15
#define ENETSW_MIB_RX_ALL_OCT			17
#define ENETSW_MIB_RX_UND			19
#define ENETSW_MIB_RX_PAUSE			20
#define ENETSW_MIB_RX_64			21
#define ENETSW_MIB_RX_65_127			22
#define ENETSW_MIB_RX_128_255			23
#define ENETSW_MIB_RX_256_511			24
#define ENETSW_MIB_RX_512_1023			25
#define ENETSW_MIB_RX_1024_1522			26
#define ENETSW_MIB_RX_OVR			27
#define ENETSW_MIB_RX_JAB			28
#define ENETSW_MIB_RX_ALIGN			29
#define ENETSW_MIB_RX_CRC			30
#define ENETSW_MIB_RX_GD_OCT			31
#define ENETSW_MIB_RX_DROP			33
#define ENETSW_MIB_RX_UNI			34
#define ENETSW_MIB_RX_MULT			35
#define ENETSW_MIB_RX_BRDCAST			36
#define ENETSW_MIB_RX_SA_CHANGE			37
#define ENETSW_MIB_RX_FRAG			38
#define ENETSW_MIB_RX_OVR_DISC			39
#define ENETSW_MIB_RX_SYM			40
#define ENETSW_MIB_RX_QOS_PKTS			41
#define ENETSW_MIB_RX_QOS_OCT			42
#define ENETSW_MIB_RX_1523_2047			44
#define ENETSW_MIB_RX_2048_4095			45
#define ENETSW_MIB_RX_4096_8191			46
#define ENETSW_MIB_RX_8192_9728			47

struct bcm6368_enetsw_mib_counters {
	u64 tx_gd_octets;
	u32 tx_gd_pkts;
	u32 tx_all_octets;
	u32 tx_all_pkts;
	u32 tx_unicast;
	u32 tx_brdcast;
	u32 tx_mult;
	u32 tx_64;
	u32 tx_65_127;
	u32 tx_128_255;
	u32 tx_256_511;
	u32 tx_512_1023;
	u32 tx_1024_max;
	u32 tx_1523_2047;
	u32 tx_2048_4095;
	u32 tx_4096_8191;
	u32 tx_8192_9728;
	u32 tx_jab;
	u32 tx_drop;
	u32 tx_ovr;
	u32 tx_frag;
	u32 tx_underrun;
	u32 tx_col;
	u32 tx_1_col;
	u32 tx_m_col;
	u32 tx_ex_col;
	u32 tx_late;
	u32 tx_def;
	u32 tx_crs;
	u32 tx_pause;
	u64 rx_gd_octets;
	u32 rx_gd_pkts;
	u32 rx_all_octets;
	u32 rx_all_pkts;
	u32 rx_brdcast;
	u32 rx_unicast;
	u32 rx_mult;
	u32 rx_64;
	u32 rx_65_127;
	u32 rx_128_255;
	u32 rx_256_511;
	u32 rx_512_1023;
	u32 rx_1024_max;
	u32 rx_jab;
	u32 rx_ovr;
	u32 rx_frag;
	u32 rx_drop;
	u32 rx_crc_align;
	u32 rx_und;
	u32 rx_crc;
	u32 rx_align;
	u32 rx_sym;
	u32 rx_pause;
	u32 rx_cntrl;
};

#define DMA_CHAN_WIDTH			0x10

/* Controller Configuration Register */
#define DMA_CFG_REG			(0x0)
#define DMA_CFG_EN_SHIFT		0
#define DMA_CFG_EN_MASK			(1 << DMA_CFG_EN_SHIFT)
#define DMA_CFG_FLOWCH_MASK(x)		(1 << ((x >> 1) + 1))

/* Flow Control Descriptor Low Threshold register */
#define DMA_FLOWCL_REG(x)		(0x4 + (x) * 6)

/* Flow Control Descriptor High Threshold register */
#define DMA_FLOWCH_REG(x)		(0x8 + (x) * 6)

/* Flow Control Descriptor Buffer Alloca Threshold register */
#define DMA_BUFALLOC_REG(x)		(0xc + (x) * 6)
#define DMA_BUFALLOC_FORCE_SHIFT	31
#define DMA_BUFALLOC_FORCE_MASK		(1 << DMA_BUFALLOC_FORCE_SHIFT)

/* Channel Configuration register */
#define DMAC_CHANCFG_REG		(0x0)
#define DMAC_CHANCFG_EN_SHIFT		0
#define DMAC_CHANCFG_EN_MASK		(1 << DMAC_CHANCFG_EN_SHIFT)
#define DMAC_CHANCFG_PKTHALT_SHIFT	1
#define DMAC_CHANCFG_PKTHALT_MASK	(1 << DMAC_CHANCFG_PKTHALT_SHIFT)
#define DMAC_CHANCFG_BUFHALT_SHIFT	2
#define DMAC_CHANCFG_BUFHALT_MASK	(1 << DMAC_CHANCFG_BUFHALT_SHIFT)
#define DMAC_CHANCFG_CHAINING_SHIFT	2
#define DMAC_CHANCFG_CHAINING_MASK	(1 << DMAC_CHANCFG_CHAINING_SHIFT)
#define DMAC_CHANCFG_WRAP_EN_SHIFT	3
#define DMAC_CHANCFG_WRAP_EN_MASK	(1 << DMAC_CHANCFG_WRAP_EN_SHIFT)
#define DMAC_CHANCFG_FLOWC_EN_SHIFT	4
#define DMAC_CHANCFG_FLOWC_EN_MASK	(1 << DMAC_CHANCFG_FLOWC_EN_SHIFT)

/* Interrupt Control/Status register */
#define DMAC_IR_REG			(0x4)
#define DMAC_IR_BUFDONE_MASK		(1 << 0)
#define DMAC_IR_PKTDONE_MASK		(1 << 1)
#define DMAC_IR_NOTOWNER_MASK		(1 << 2)

/* Interrupt Mask register */
#define DMAC_IRMASK_REG			(0x8)

/* Maximum Burst Length */
#define DMAC_MAXBURST_REG		(0xc)

/* Ring Start Address register */
#define DMAS_RSTART_REG			(0x0)

/* State Ram Word 2 */
#define DMAS_SRAM2_REG			(0x4)

/* State Ram Word 3 */
#define DMAS_SRAM3_REG			(0x8)

/* State Ram Word 4 */
#define DMAS_SRAM4_REG			(0xc)

struct bcm6368_enetsw_desc {
	u32 len_stat;
	u32 address;
};

/* control */
#define DMADESC_LENGTH_SHIFT		16
#define DMADESC_LENGTH_MASK		(0xfff << DMADESC_LENGTH_SHIFT)
#define DMADESC_OWNER_MASK		(1 << 15)
#define DMADESC_EOP_MASK		(1 << 14)
#define DMADESC_SOP_MASK		(1 << 13)
#define DMADESC_ESOP_MASK		(DMADESC_EOP_MASK | DMADESC_SOP_MASK)
#define DMADESC_WRAP_MASK		(1 << 12)
#define DMADESC_USB_NOZERO_MASK 	(1 << 1)
#define DMADESC_USB_ZERO_MASK		(1 << 0)

/* status */
#define DMADESC_UNDER_MASK		(1 << 9)
#define DMADESC_APPEND_CRC		(1 << 8)
#define DMADESC_OVSIZE_MASK		(1 << 4)
#define DMADESC_RXER_MASK		(1 << 2)
#define DMADESC_CRC_MASK		(1 << 1)
#define DMADESC_OV_MASK			(1 << 0)
#define DMADESC_ERR_MASK		(DMADESC_UNDER_MASK | \
					 DMADESC_OVSIZE_MASK | \
					 DMADESC_RXER_MASK | \
					 DMADESC_CRC_MASK | \
					 DMADESC_OV_MASK)

struct bcm6368_enetsw_port {
	bool used;
	int phy_id;

	bool bypass_link;
	int force_speed;
	bool force_duplex_full;

	const char *name;
};

struct bcm6368_enetsw {
	void __iomem *base;
	void __iomem *dma_base;
	void __iomem *dma_chan;
	void __iomem *dma_sram;

	struct device **pm;
	struct device_link **link_pm;
	int num_pms;

	struct clk **clock;
	unsigned int num_clocks;

	struct reset_control **reset;
	unsigned int num_resets;

	int copybreak;

	bool rgmii_timing;
	bool rgmii_override;

	int irq_rx;
	int irq_tx;

	/* hw view of rx & tx dma ring */
	dma_addr_t rx_desc_dma;
	dma_addr_t tx_desc_dma;

	/* allocated size (in bytes) for rx & tx dma ring */
	unsigned int rx_desc_alloc_size;
	unsigned int tx_desc_alloc_size;

	struct napi_struct napi;

	/* dma channel id for rx */
	int rx_chan;

	/* number of dma desc in rx ring */
	int rx_ring_size;

	/* cpu view of rx dma ring */
	struct bcm6368_enetsw_desc *rx_desc_cpu;

	/* current number of armed descriptor given to hardware for rx */
	int rx_desc_count;

	/* next rx descriptor to fetch from hardware */
	int rx_curr_desc;

	/* next dirty rx descriptor to refill */
	int rx_dirty_desc;

	/* size of allocated rx skbs */
	unsigned int rx_skb_size;

	/* list of skb given to hw for rx */
	struct sk_buff **rx_skb;

	/* used when rx skb allocation failed, so we defer rx queue
	 * refill */
	struct timer_list rx_timeout;

	/* lock rx_timeout against rx normal operation */
	spinlock_t rx_lock;

	/* dma channel id for tx */
	int tx_chan;

	/* number of dma desc in tx ring */
	int tx_ring_size;

	/* maximum dma burst size */
	int dma_maxburst;

	/* cpu view of rx dma ring */
	struct bcm6368_enetsw_desc *tx_desc_cpu;

	/* number of available descriptor for tx */
	int tx_desc_count;

	/* next tx descriptor avaiable */
	int tx_curr_desc;

	/* next dirty tx descriptor to reclaim */
	int tx_dirty_desc;

	/* list of skb given to hw for tx */
	struct sk_buff **tx_skb;

	/* lock used by tx reclaim and xmit */
	spinlock_t tx_lock;

	/* stats */
	struct bcm6368_enetsw_mib_counters mib;

	/* network device reference */
	struct net_device *net_dev;

	/* platform device reference */
	struct platform_device *pdev;

	/* maximum hardware transmit/receive size */
	unsigned int hw_mtu;

	/* port mapping for switch devices */
	int num_ports;
	struct bcm6368_enetsw_port used_ports[ENETSW_MAX_PORT];
	int sw_port_link[ENETSW_MAX_PORT];

	/* platform device for associated switch */
	struct platform_device *b53_device;

	/* used to poll switch port state */
	struct timer_list swphy_poll;
	spinlock_t enetsw_mdio_lock;

	/* dma channel enable mask */
	u32 dma_chan_en_mask;

	/* dma channel interrupt mask */
	u32 dma_chan_int_mask;

	/* dma channel width */
	unsigned int dma_chan_width;

	/* dma descriptor shift value */
	unsigned int dma_desc_shift;
};

/* IO helpers to access switch registers */
static inline u32 enetsw_readl(struct bcm6368_enetsw *priv, u32 off)
{
	return __raw_readl(priv->base + off);
}

static inline void enetsw_writel(struct bcm6368_enetsw *priv,
				 u32 val, u32 off)
{
	__raw_writel(val, priv->base + off);
}

static inline u16 enetsw_readw(struct bcm6368_enetsw *priv, u32 off)
{
	return __raw_readw(priv->base + off);
}

static inline void enetsw_writew(struct bcm6368_enetsw *priv,
				 u16 val, u32 off)
{
	__raw_writew(val, priv->base + off);
}

static inline u8 enetsw_readb(struct bcm6368_enetsw *priv, u32 off)
{
	return __raw_readb(priv->base + off);
}

static inline void enetsw_writeb(struct bcm6368_enetsw *priv,
				 u8 val, u32 off)
{
	__raw_writeb(val, priv->base + off);
}

static inline void dma_writel(struct bcm6368_enetsw *priv, u32 val, u32 off)
{
	__raw_writel(val, priv->dma_base + off);
}

static inline u32 dma_readl(struct bcm6368_enetsw *priv, u32 off, int chan)
{
	return __raw_readl(priv->dma_chan + off + chan * priv->dma_chan_width);
}

static inline void dmac_writel(struct bcm6368_enetsw *priv, u32 val,
				    u32 off, int chan)
{
	__raw_writel(val, priv->dma_chan + off + chan * priv->dma_chan_width);
}

static inline void dmas_writel(struct bcm6368_enetsw *priv, u32 val,
				    u32 off, int chan)
{
	__raw_writel(val, priv->dma_sram + off + chan * priv->dma_chan_width);
}

/*
 * refill rx queue
 */
static int bcm6368_enetsw_refill_rx(struct net_device *dev)
{
	struct bcm6368_enetsw *priv;

	priv = netdev_priv(dev);

	while (priv->rx_desc_count < priv->rx_ring_size) {
		struct bcm6368_enetsw_desc *desc;
		struct sk_buff *skb;
		dma_addr_t p;
		int desc_idx;
		u32 len_stat;

		desc_idx = priv->rx_dirty_desc;
		desc = &priv->rx_desc_cpu[desc_idx];

		if (!priv->rx_skb[desc_idx]) {
			skb = netdev_alloc_skb(dev, priv->rx_skb_size);
			if (!skb)
				break;
			priv->rx_skb[desc_idx] = skb;
			p = dma_map_single(&priv->pdev->dev, skb->data,
					   priv->rx_skb_size,
					   DMA_FROM_DEVICE);
			desc->address = p;
		}

		len_stat = priv->rx_skb_size << DMADESC_LENGTH_SHIFT;
		len_stat |= DMADESC_OWNER_MASK;
		if (priv->rx_dirty_desc == priv->rx_ring_size - 1) {
			len_stat |= (DMADESC_WRAP_MASK >>
				     priv->dma_desc_shift);
			priv->rx_dirty_desc = 0;
		} else {
			priv->rx_dirty_desc++;
		}
		wmb();
		desc->len_stat = len_stat;

		priv->rx_desc_count++;

		/* tell dma engine we allocated one buffer */
		dma_writel(priv, 1, DMA_BUFALLOC_REG(priv->rx_chan));
	}

	/* If rx ring is still empty, set a timer to try allocating
	 * again at a later time. */
	if (priv->rx_desc_count == 0 && netif_running(dev)) {
		dev_warn(&priv->pdev->dev, "unable to refill rx ring\n");
		priv->rx_timeout.expires = jiffies + HZ;
		add_timer(&priv->rx_timeout);
	}

	return 0;
}

/*
 * timer callback to defer refill rx queue in case we're OOM
 */
static void bcm6368_enetsw_refill_rx_timer(struct timer_list *t)
{
	struct bcm6368_enetsw *priv = from_timer(priv, t, rx_timeout);
	struct net_device *dev = priv->net_dev;

	spin_lock(&priv->rx_lock);
	bcm6368_enetsw_refill_rx(dev);
	spin_unlock(&priv->rx_lock);
}

/*
 * extract packet from rx queue
 */
static int bcm6368_enetsw_receive_queue(struct net_device *dev, int budget)
{
	struct bcm6368_enetsw *priv;
	struct device *kdev;
	int processed;

	priv = netdev_priv(dev);
	kdev = &priv->pdev->dev;
	processed = 0;

	/* don't scan ring further than number of refilled
	 * descriptor */
	if (budget > priv->rx_desc_count)
		budget = priv->rx_desc_count;

	do {
		struct bcm6368_enetsw_desc *desc;
		struct sk_buff *skb;
		int desc_idx;
		u32 len_stat;
		unsigned int len;

		desc_idx = priv->rx_curr_desc;
		desc = &priv->rx_desc_cpu[desc_idx];

		/* make sure we actually read the descriptor status at
		 * each loop */
		rmb();

		len_stat = desc->len_stat;

		/* break if dma ownership belongs to hw */
		if (len_stat & DMADESC_OWNER_MASK)
			break;

		processed++;
		priv->rx_curr_desc++;
		if (priv->rx_curr_desc == priv->rx_ring_size)
			priv->rx_curr_desc = 0;
		priv->rx_desc_count--;

		/* if the packet does not have start of packet _and_
		 * end of packet flag set, then just recycle it */
		if ((len_stat & (DMADESC_ESOP_MASK >> priv->dma_desc_shift))
		    != (DMADESC_ESOP_MASK >> priv->dma_desc_shift)) {
			dev->stats.rx_dropped++;
			continue;
		}

		/* valid packet */
		skb = priv->rx_skb[desc_idx];
		len = (len_stat & DMADESC_LENGTH_MASK)
		      >> DMADESC_LENGTH_SHIFT;
		/* don't include FCS */
		len -= 4;

		if (len < priv->copybreak) {
			struct sk_buff *nskb;

			nskb = napi_alloc_skb(&priv->napi, len);
			if (!nskb) {
				/* forget packet, just rearm desc */
				dev->stats.rx_dropped++;
				continue;
			}

			dma_sync_single_for_cpu(kdev, desc->address,
						len, DMA_FROM_DEVICE);
			memcpy(nskb->data, skb->data, len);
			dma_sync_single_for_device(kdev, desc->address,
						   len, DMA_FROM_DEVICE);
			skb = nskb;
		} else {
			dma_unmap_single(&priv->pdev->dev, desc->address,
					 priv->rx_skb_size, DMA_FROM_DEVICE);
			priv->rx_skb[desc_idx] = NULL;
		}

		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, dev);
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += len;
		netif_receive_skb(skb);
	} while (--budget > 0);

	if (processed || !priv->rx_desc_count) {
		bcm6368_enetsw_refill_rx(dev);

		/* kick rx dma */
		dmac_writel(priv, priv->dma_chan_en_mask,
			    DMAC_CHANCFG_REG, priv->rx_chan);
	}

	return processed;
}

/*
 * try to or force reclaim of transmitted buffers
 */
static int bcm6368_enetsw_tx_reclaim(struct net_device *dev, int force)
{
	struct bcm6368_enetsw *priv;
	int released;

	priv = netdev_priv(dev);
	released = 0;

	while (priv->tx_desc_count < priv->tx_ring_size) {
		struct bcm6368_enetsw_desc *desc;
		struct sk_buff *skb;

		/* We run in a bh and fight against start_xmit, which
		 * is called with bh disabled  */
		spin_lock(&priv->tx_lock);

		desc = &priv->tx_desc_cpu[priv->tx_dirty_desc];

		if (!force && (desc->len_stat & DMADESC_OWNER_MASK)) {
			spin_unlock(&priv->tx_lock);
			break;
		}

		/* ensure other field of the descriptor were not read
		 * before we checked ownership */
		rmb();

		skb = priv->tx_skb[priv->tx_dirty_desc];
		priv->tx_skb[priv->tx_dirty_desc] = NULL;
		dma_unmap_single(&priv->pdev->dev, desc->address, skb->len,
				 DMA_TO_DEVICE);

		priv->tx_dirty_desc++;
		if (priv->tx_dirty_desc == priv->tx_ring_size)
			priv->tx_dirty_desc = 0;
		priv->tx_desc_count++;

		spin_unlock(&priv->tx_lock);

		if (desc->len_stat & DMADESC_UNDER_MASK)
			dev->stats.tx_errors++;

		dev_kfree_skb(skb);
		released++;
	}

	if (netif_queue_stopped(dev) && released)
		netif_wake_queue(dev);

	return released;
}

/*
 * poll func, called by network core
 */
static int bcm6368_enetsw_poll(struct napi_struct *napi, int budget)
{
	struct bcm6368_enetsw *priv;
	struct net_device *dev;
	int rx_work_done;

	priv = container_of(napi, struct bcm6368_enetsw, napi);
	dev = priv->net_dev;

	/* ack interrupts */
	dmac_writel(priv, priv->dma_chan_int_mask,
			 DMAC_IR_REG, priv->rx_chan);
	dmac_writel(priv, priv->dma_chan_int_mask,
			 DMAC_IR_REG, priv->tx_chan);

	/* reclaim sent skb */
	bcm6368_enetsw_tx_reclaim(dev, 0);

	spin_lock(&priv->rx_lock);
	rx_work_done = bcm6368_enetsw_receive_queue(dev, budget);
	spin_unlock(&priv->rx_lock);

	if (rx_work_done >= budget) {
		/* rx queue is not yet empty/clean */
		return rx_work_done;
	}

	/* no more packet in rx/tx queue, remove device from poll
	 * queue */
	napi_complete_done(napi, rx_work_done);

	/* restore rx/tx interrupt */
	dmac_writel(priv, priv->dma_chan_int_mask,
			 DMAC_IRMASK_REG, priv->rx_chan);
	dmac_writel(priv, priv->dma_chan_int_mask,
			 DMAC_IRMASK_REG, priv->tx_chan);

	return rx_work_done;
}

/*
 * rx/tx dma interrupt handler
 */
static irqreturn_t bcm6368_enetsw_isr_dma(int irq, void *dev_id)
{
	struct net_device *dev;
	struct bcm6368_enetsw *priv;

	dev = dev_id;
	priv = netdev_priv(dev);

	/* mask rx/tx interrupts */
	dmac_writel(priv, 0, DMAC_IRMASK_REG, priv->rx_chan);
	dmac_writel(priv, 0, DMAC_IRMASK_REG, priv->tx_chan);

	napi_schedule(&priv->napi);

	return IRQ_HANDLED;
}

/*
 * tx request callback
 */
static netdev_tx_t
bcm6368_enetsw_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct bcm6368_enetsw *priv;
	struct bcm6368_enetsw_desc *desc;
	u32 len_stat;
	netdev_tx_t ret;

	priv = netdev_priv(dev);

	/* lock against tx reclaim */
	spin_lock(&priv->tx_lock);

	/* make sure  the tx hw queue  is not full,  should not happen
	 * since we stop queue before it's the case */
	if (unlikely(!priv->tx_desc_count)) {
		netif_stop_queue(dev);
		dev_err(&priv->pdev->dev, "xmit called with no tx desc "
			"available?\n");
		ret = NETDEV_TX_BUSY;
		goto out_unlock;
	}

	/* pad small packets */
	if (skb->len < 64) {
		int needed = 64 - skb->len;
		char *data;

		if (unlikely(skb_tailroom(skb) < needed)) {
			struct sk_buff *nskb;

			nskb = skb_copy_expand(skb, 0, needed, GFP_ATOMIC);
			if (!nskb) {
				ret = NETDEV_TX_BUSY;
				goto out_unlock;
			}
			dev_kfree_skb(skb);
			skb = nskb;
		}
		data = skb_put_zero(skb, needed);
	}

	/* point to the next available desc */
	desc = &priv->tx_desc_cpu[priv->tx_curr_desc];
	priv->tx_skb[priv->tx_curr_desc] = skb;

	/* fill descriptor */
	desc->address = dma_map_single(&priv->pdev->dev, skb->data, skb->len,
				       DMA_TO_DEVICE);

	len_stat = (skb->len << DMADESC_LENGTH_SHIFT) & DMADESC_LENGTH_MASK;
	len_stat |= (DMADESC_ESOP_MASK >> priv->dma_desc_shift) |
		    DMADESC_APPEND_CRC | DMADESC_OWNER_MASK;

	priv->tx_curr_desc++;
	if (priv->tx_curr_desc == priv->tx_ring_size) {
		priv->tx_curr_desc = 0;
		len_stat |= (DMADESC_WRAP_MASK >> priv->dma_desc_shift);
	}
	priv->tx_desc_count--;

	/* dma might be already polling, make sure we update desc
	 * fields in correct order */
	wmb();
	desc->len_stat = len_stat;
	wmb();

	/* kick tx dma */
	dmac_writel(priv, priv->dma_chan_en_mask, DMAC_CHANCFG_REG,
		    priv->tx_chan);

	/* stop queue if no more desc available */
	if (!priv->tx_desc_count)
		netif_stop_queue(dev);

	dev->stats.tx_bytes += skb->len;
	dev->stats.tx_packets++;
	ret = NETDEV_TX_OK;

out_unlock:
	spin_unlock(&priv->tx_lock);
	return ret;
}

/*
 * disable dma in given channel
 */
static void bcm6368_enetsw_disable_dma(struct bcm6368_enetsw *priv, int chan)
{
	int limit;

	dmac_writel(priv, 0, DMAC_CHANCFG_REG, chan);

	limit = 1000;
	do {
		u32 val;

		val = dma_readl(priv, DMAC_CHANCFG_REG, chan);
		if (!(val & DMAC_CHANCFG_EN_MASK))
			break;
		udelay(1);
	} while (limit--);
}

/*
 * ethtool callbacks
 */
struct bcm6368_enetsw_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
	int mib_reg;
};

#define GEN_STAT(m) sizeof(((struct bcm6368_enetsw *)0)->m),		\
		     offsetof(struct bcm6368_enetsw, m)
#define DEV_STAT(m) sizeof(((struct net_device_stats *)0)->m),		\
		     offsetof(struct net_device_stats, m)

/*
 * adjust mtu, can't be called while device is running
 */
static int bcm6368_enetsw_change_mtu(struct net_device *dev, int new_mtu)
{
	struct bcm6368_enetsw *priv = netdev_priv(dev);
	int actual_mtu = new_mtu;

	if (netif_running(dev))
		return -EBUSY;

	/* add ethernet header + vlan tag size */
	actual_mtu += VLAN_ETH_HLEN + VLAN_HLEN;

	/*
	 * setup maximum size before we get overflow mark in
	 * descriptor, note that this will not prevent reception of
	 * big frames, they will be split into multiple buffers
	 * anyway
	 */
	priv->hw_mtu = actual_mtu;

	/*
	 * align rx buffer size to dma burst len, account FCS since
	 * it's appended
	 */
	priv->rx_skb_size = ALIGN(actual_mtu + ETH_FCS_LEN,
				  priv->dma_maxburst * 4);

	dev->mtu = new_mtu;
	return 0;
}

struct b53_platform_data bcm63xx_b53_pdata = {
	.chip_id = 0x6300,
	.big_endian = 1,
};

struct platform_device bcm63xx_b53_dev = {
	.name = "b53-switch",
	.id = -1,
	.dev = {
		.platform_data = &bcm63xx_b53_pdata,
	},
};

static int bcm6368_enetsw_register(struct bcm6368_enetsw *priv, u16 port_mask)
{
	int ret;

	bcm63xx_b53_pdata.regs = priv->base;
	bcm63xx_b53_pdata.enabled_ports = port_mask;
	bcm63xx_b53_pdata.alias = priv->net_dev->name;

	ret = platform_device_register(&bcm63xx_b53_dev);
	if (!ret)
		priv->b53_device = &bcm63xx_b53_dev;

	return ret;
}

static void bcm6368_enetsw_unregister(struct bcm6368_enetsw *priv)
{
	if (priv->b53_device)
		platform_device_unregister(&bcm63xx_b53_dev);

	priv->b53_device = NULL;
}

/*
 * switch mii access callbacks
 */
static int bcm6368_enetsw_mdio_read(struct bcm6368_enetsw *priv, int ext,
				    int phy_id, int location)
{
	u32 reg;
	int ret;

	spin_lock_bh(&priv->enetsw_mdio_lock);
	enetsw_writel(priv, 0, ENETSW_MDIOC_REG);

	reg = ENETSW_MDIOC_RD_MASK |
		(phy_id << ENETSW_MDIOC_PHYID_SHIFT) |
		(location << ENETSW_MDIOC_REG_SHIFT);

	if (ext)
		reg |= ENETSW_MDIOC_EXT_MASK;

	enetsw_writel(priv, reg, ENETSW_MDIOC_REG);
	udelay(50);
	ret = enetsw_readw(priv, ENETSW_MDIOD_REG);
	spin_unlock_bh(&priv->enetsw_mdio_lock);
	return ret;
}

static void bcm6368_enetsw_mdio_write(struct bcm6368_enetsw *priv, int ext,
				      int phy_id, int location, uint16_t data)
{
	u32 reg;

	spin_lock_bh(&priv->enetsw_mdio_lock);
	enetsw_writel(priv, 0, ENETSW_MDIOC_REG);

	reg = ENETSW_MDIOC_WR_MASK |
		(phy_id << ENETSW_MDIOC_PHYID_SHIFT) |
		(location << ENETSW_MDIOC_REG_SHIFT);

	if (ext)
		reg |= ENETSW_MDIOC_EXT_MASK;

	reg |= data;

	enetsw_writel(priv, reg, ENETSW_MDIOC_REG);
	udelay(50);
	spin_unlock_bh(&priv->enetsw_mdio_lock);
}

static inline int bcm6368_enetsw_port_is_rgmii(int portid)
{
	return portid >= ENETSW_RGMII_PORT0;
}

/*
 * enet sw PHY polling
 */
static void swphy_poll_timer(struct timer_list *t)
{
	struct bcm6368_enetsw *priv = from_timer(priv, t, swphy_poll);
	unsigned int i;

	for (i = 0; i < priv->num_ports; i++) {
		struct bcm6368_enetsw_port *port;
		int val, j, up, advertise, lpa, speed, duplex, media;
		int external_phy = bcm6368_enetsw_port_is_rgmii(i);
		u8 override;

		port = &priv->used_ports[i];
		if (!port->used)
			continue;

		if (port->bypass_link)
			continue;

		/* dummy read to clear */
		for (j = 0; j < 2; j++)
			val = bcm6368_enetsw_mdio_read(priv, external_phy,
						       port->phy_id,
						       MII_BMSR);

		if (val == 0xffff)
			continue;

		up = (val & BMSR_LSTATUS) ? 1 : 0;
		if (!(up ^ priv->sw_port_link[i]))
			continue;

		priv->sw_port_link[i] = up;

		/* link changed */
		if (!up) {
			dev_info(&priv->pdev->dev, "link DOWN on %s\n",
				 port->name);
			enetsw_writeb(priv, ENETSW_PORTOV_ENABLE_MASK,
				      ENETSW_PORTOV_REG(i));
			enetsw_writeb(priv, ENETSW_PTCTRL_RXDIS_MASK |
				      ENETSW_PTCTRL_TXDIS_MASK,
				      ENETSW_PTCTRL_REG(i));
			continue;
		}

		advertise = bcm6368_enetsw_mdio_read(priv, external_phy,
						     port->phy_id,
						     MII_ADVERTISE);

		lpa = bcm6368_enetsw_mdio_read(priv, external_phy,
					       port->phy_id, MII_LPA);

		/* figure out media and duplex from advertise and LPA values */
		media = mii_nway_result(lpa & advertise);
		duplex = (media & ADVERTISE_FULL) ? 1 : 0;

		if (media & (ADVERTISE_100FULL | ADVERTISE_100HALF))
			speed = 100;
		else
			speed = 10;

		if (val & BMSR_ESTATEN) {
			advertise = bcm6368_enetsw_mdio_read(priv,
							     external_phy,
							     port->phy_id,
							     MII_CTRL1000);

			lpa = bcm6368_enetsw_mdio_read(priv, external_phy,
						       port->phy_id,
						       MII_STAT1000);

			if (advertise & (ADVERTISE_1000FULL |
					 ADVERTISE_1000HALF)
			    && lpa & (LPA_1000FULL | LPA_1000HALF)) {
				speed = 1000;
				duplex = (lpa & LPA_1000FULL);
			}
		}

		dev_info(&priv->pdev->dev,
			 "link UP on %s, %dMbps, %s-duplex\n",
			 port->name, speed, duplex ? "full" : "half");

		override = ENETSW_PORTOV_ENABLE_MASK |
			   ENETSW_PORTOV_LINKUP_MASK;

		if (speed == 1000)
			override |= ENETSW_PORTOV_1000_MASK;
		else if (speed == 100)
			override |= ENETSW_PORTOV_100_MASK;
		if (duplex)
			override |= ENETSW_PORTOV_FDX_MASK;

		enetsw_writeb(priv, override, ENETSW_PORTOV_REG(i));
		enetsw_writeb(priv, 0, ENETSW_PTCTRL_REG(i));
	}

	priv->swphy_poll.expires = jiffies + HZ;
	add_timer(&priv->swphy_poll);
}

/*
 * open callback, allocate dma rings & buffers and start rx operation
 */
static int bcm6368_enetsw_open(struct net_device *dev)
{
	struct bcm6368_enetsw *priv;
	struct device *kdev;
	int i, ret;
	unsigned int size;
	void *p;
	u32 val;

	priv = netdev_priv(dev);
	kdev = &priv->pdev->dev;

	/* mask all interrupts and request them */
	dmac_writel(priv, 0, DMAC_IRMASK_REG, priv->rx_chan);
	dmac_writel(priv, 0, DMAC_IRMASK_REG, priv->tx_chan);

	ret = request_irq(priv->irq_rx, bcm6368_enetsw_isr_dma,
			  0, dev->name, dev);
	if (ret)
		goto out_freeirq;

	if (priv->irq_tx != -1) {
		ret = request_irq(priv->irq_tx, bcm6368_enetsw_isr_dma,
				  0, dev->name, dev);
		if (ret)
			goto out_freeirq_rx;
	}

	/* allocate rx dma ring */
	size = priv->rx_ring_size * sizeof(struct bcm6368_enetsw_desc);
	p = dma_alloc_coherent(kdev, size, &priv->rx_desc_dma, GFP_KERNEL);
	if (!p) {
		dev_err(kdev, "cannot allocate rx ring %u\n", size);
		ret = -ENOMEM;
		goto out_freeirq_tx;
	}

	memset(p, 0, size);
	priv->rx_desc_alloc_size = size;
	priv->rx_desc_cpu = p;

	/* allocate tx dma ring */
	size = priv->tx_ring_size * sizeof(struct bcm6368_enetsw_desc);
	p = dma_alloc_coherent(kdev, size, &priv->tx_desc_dma, GFP_KERNEL);
	if (!p) {
		dev_err(kdev, "cannot allocate tx ring\n");
		ret = -ENOMEM;
		goto out_free_rx_ring;
	}

	memset(p, 0, size);
	priv->tx_desc_alloc_size = size;
	priv->tx_desc_cpu = p;

	priv->tx_skb = kzalloc(sizeof(struct sk_buff *) * priv->tx_ring_size,
			       GFP_KERNEL);
	if (!priv->tx_skb) {
		dev_err(kdev, "cannot allocate rx skb queue\n");
		ret = -ENOMEM;
		goto out_free_tx_ring;
	}

	priv->tx_desc_count = priv->tx_ring_size;
	priv->tx_dirty_desc = 0;
	priv->tx_curr_desc = 0;
	spin_lock_init(&priv->tx_lock);

	/* init & fill rx ring with skbs */
	priv->rx_skb = kzalloc(sizeof(struct sk_buff *) * priv->rx_ring_size,
			       GFP_KERNEL);
	if (!priv->rx_skb) {
		dev_err(kdev, "cannot allocate rx skb queue\n");
		ret = -ENOMEM;
		goto out_free_tx_skb;
	}

	priv->rx_desc_count = 0;
	priv->rx_dirty_desc = 0;
	priv->rx_curr_desc = 0;

	/* disable all ports */
	for (i = 0; i < priv->num_ports; i++) {
		enetsw_writeb(priv, ENETSW_PORTOV_ENABLE_MASK,
			      ENETSW_PORTOV_REG(i));
		enetsw_writeb(priv, ENETSW_PTCTRL_RXDIS_MASK |
			      ENETSW_PTCTRL_TXDIS_MASK,
			      ENETSW_PTCTRL_REG(i));

		priv->sw_port_link[i] = 0;
	}

	/* enable external ports */
	for (i = ENETSW_RGMII_PORT0; i < priv->num_ports; i++) {
		u8 rgmii_ctrl;

		if (!priv->used_ports[i].used)
			continue;

		rgmii_ctrl = enetsw_readb(priv, ENETSW_RGMII_CTRL_REG(i));
		rgmii_ctrl |= ENETSW_RGMII_CTRL_GMII_CLK_EN;
		if (priv->rgmii_timing)
			rgmii_ctrl |= ENETSW_RGMII_CTRL_TIMING_SEL_EN;
		if (priv->rgmii_override)
			rgmii_ctrl |= ENETSW_RGMII_CTRL_MII_OVERRIDE_EN;
		enetsw_writeb(priv, rgmii_ctrl, ENETSW_RGMII_CTRL_REG(i));
	}

	/* initialize flow control buffer allocation */
	dma_writel(priv, DMA_BUFALLOC_FORCE_MASK | 0,
		   DMA_BUFALLOC_REG(priv->rx_chan));

	if (bcm6368_enetsw_refill_rx(dev)) {
		dev_err(kdev, "cannot allocate rx skb queue\n");
		ret = -ENOMEM;
		goto out;
	}

	/* write rx & tx ring addresses */
	dmas_writel(priv, priv->rx_desc_dma,
		    DMAS_RSTART_REG, priv->rx_chan);
	dmas_writel(priv, priv->tx_desc_dma,
		    DMAS_RSTART_REG, priv->tx_chan);

	/* clear remaining state ram for rx & tx channel */
	dmas_writel(priv, 0, DMAS_SRAM2_REG, priv->rx_chan);
	dmas_writel(priv, 0, DMAS_SRAM2_REG, priv->tx_chan);
	dmas_writel(priv, 0, DMAS_SRAM3_REG, priv->rx_chan);
	dmas_writel(priv, 0, DMAS_SRAM3_REG, priv->tx_chan);
	dmas_writel(priv, 0, DMAS_SRAM4_REG, priv->rx_chan);
	dmas_writel(priv, 0, DMAS_SRAM4_REG, priv->tx_chan);

	/* set dma maximum burst len */
	dmac_writel(priv, priv->dma_maxburst,
		    DMAC_MAXBURST_REG, priv->rx_chan);
	dmac_writel(priv, priv->dma_maxburst,
		    DMAC_MAXBURST_REG, priv->tx_chan);

	/* set flow control low/high threshold to 1/3 / 2/3 */
	val = priv->rx_ring_size / 3;
	dma_writel(priv, val, DMA_FLOWCL_REG(priv->rx_chan));
	val = (priv->rx_ring_size * 2) / 3;
	dma_writel(priv, val, DMA_FLOWCH_REG(priv->rx_chan));

	/* all set, enable mac and interrupts, start dma engine and
	 * kick rx dma channel
	 */
	wmb();
	dma_writel(priv, DMA_CFG_EN_MASK, DMA_CFG_REG);
	dmac_writel(priv, DMAC_CHANCFG_EN_MASK,
		    DMAC_CHANCFG_REG, priv->rx_chan);

	/* watch "packet transferred" interrupt in rx and tx */
	dmac_writel(priv, DMAC_IR_PKTDONE_MASK,
		    DMAC_IR_REG, priv->rx_chan);
	dmac_writel(priv, DMAC_IR_PKTDONE_MASK,
		    DMAC_IR_REG, priv->tx_chan);

	/* make sure we enable napi before rx interrupt  */
	napi_enable(&priv->napi);

	dmac_writel(priv, DMAC_IR_PKTDONE_MASK,
		    DMAC_IRMASK_REG, priv->rx_chan);
	dmac_writel(priv, DMAC_IR_PKTDONE_MASK,
		    DMAC_IRMASK_REG, priv->tx_chan);

	netif_carrier_on(dev);
	netif_start_queue(dev);

	/* apply override config for bypass_link ports here. */
	for (i = 0; i < priv->num_ports; i++) {
		struct bcm6368_enetsw_port *port;
		u8 override;
		port = &priv->used_ports[i];
		if (!port->used)
			continue;

		if (!port->bypass_link)
			continue;

		override = ENETSW_PORTOV_ENABLE_MASK |
			   ENETSW_PORTOV_LINKUP_MASK;

		switch (port->force_speed) {
		case 1000:
			override |= ENETSW_PORTOV_1000_MASK;
			break;
		case 100:
			override |= ENETSW_PORTOV_100_MASK;
			break;
		case 10:
			break;
		default:
			pr_warn("invalid forced speed on port %s: assume 10\n",
			       port->name);
			break;
		}

		if (port->force_duplex_full)
			override |= ENETSW_PORTOV_FDX_MASK;

		enetsw_writeb(priv, override, ENETSW_PORTOV_REG(i));
		enetsw_writeb(priv, 0, ENETSW_PTCTRL_REG(i));
	}

	/* start phy polling timer */
	timer_setup(&priv->swphy_poll, swphy_poll_timer, 0);
	mod_timer(&priv->swphy_poll, jiffies);

	return 0;

out:
	for (i = 0; i < priv->rx_ring_size; i++) {
		struct bcm6368_enetsw_desc *desc;

		if (!priv->rx_skb[i])
			continue;

		desc = &priv->rx_desc_cpu[i];
		dma_unmap_single(kdev, desc->address, priv->rx_skb_size,
				 DMA_FROM_DEVICE);
		kfree_skb(priv->rx_skb[i]);
	}
	kfree(priv->rx_skb);

out_free_tx_skb:
	kfree(priv->tx_skb);

out_free_tx_ring:
	dma_free_coherent(kdev, priv->tx_desc_alloc_size,
			  priv->tx_desc_cpu, priv->tx_desc_dma);

out_free_rx_ring:
	dma_free_coherent(kdev, priv->rx_desc_alloc_size,
			  priv->rx_desc_cpu, priv->rx_desc_dma);

out_freeirq_tx:
	if (priv->irq_tx != -1)
		free_irq(priv->irq_tx, dev);

out_freeirq_rx:
	free_irq(priv->irq_rx, dev);

out_freeirq:
	return ret;
}

/* stop callback */
static int bcm6368_enetsw_stop(struct net_device *dev)
{
	struct bcm6368_enetsw *priv;
	struct device *kdev;
	int i;

	priv = netdev_priv(dev);
	kdev = &priv->pdev->dev;

	del_timer_sync(&priv->swphy_poll);
	netif_stop_queue(dev);
	napi_disable(&priv->napi);
	del_timer_sync(&priv->rx_timeout);

	/* mask all interrupts */
	dmac_writel(priv, 0, DMAC_IRMASK_REG, priv->rx_chan);
	dmac_writel(priv, 0, DMAC_IRMASK_REG, priv->tx_chan);

	/* disable dma & mac */
	bcm6368_enetsw_disable_dma(priv, priv->tx_chan);
	bcm6368_enetsw_disable_dma(priv, priv->rx_chan);

	/* force reclaim of all tx buffers */
	bcm6368_enetsw_tx_reclaim(dev, 1);

	/* free the rx skb ring */
	for (i = 0; i < priv->rx_ring_size; i++) {
		struct bcm6368_enetsw_desc *desc;

		if (!priv->rx_skb[i])
			continue;

		desc = &priv->rx_desc_cpu[i];
		dma_unmap_single_attrs(kdev, desc->address, priv->rx_skb_size,
				       DMA_FROM_DEVICE,
				       DMA_ATTR_SKIP_CPU_SYNC);
		kfree_skb(priv->rx_skb[i]);
	}

	/* free remaining allocated memory */
	kfree(priv->rx_skb);
	kfree(priv->tx_skb);
	dma_free_coherent(kdev, priv->rx_desc_alloc_size,
			  priv->rx_desc_cpu, priv->rx_desc_dma);
	dma_free_coherent(kdev, priv->tx_desc_alloc_size,
			  priv->tx_desc_cpu, priv->tx_desc_dma);
	if (priv->irq_tx != -1)
		free_irq(priv->irq_tx, dev);
	free_irq(priv->irq_rx, dev);

	return 0;
}

/* try to sort out phy external status by walking the used_port field
 * in the bcm6368_enetsw structure. in case the phy address is not
 * assigned to any physical port on the switch, assume it is external
 * (and yell at the user).
 */
static int bcm6368_enetsw_phy_is_external(struct bcm6368_enetsw *priv,
					  int phy_id)
{
	int i;

	for (i = 0; i < priv->num_ports; ++i) {
		if (!priv->used_ports[i].used)
			continue;
		if (priv->used_ports[i].phy_id == phy_id)
			return bcm6368_enetsw_port_is_rgmii(i);
	}

	printk_once(KERN_WARNING "bcm63xx_enet: could not find a used port "
		    "with phy_id %i, assuming phy is external\n", phy_id);

	return 1;
}

/* can't use bcm6368_enetsw_mdio_read directly as we need to sort out
 * external/internal status of the given phy_id first.
 */
static int bcm6368_enetsw_mii_mdio_read(struct net_device *dev, int phy_id,
					int location)
{
	struct bcm6368_enetsw *priv;

	priv = netdev_priv(dev);
	return bcm6368_enetsw_mdio_read(priv,
			bcm6368_enetsw_phy_is_external(priv, phy_id),
			phy_id, location);
}

/* can't use bcm6368_enetsw_mdio_write directly as we need to sort out
 * external/internal status of the given phy_id first.
 */
static void bcm6368_enetsw_mii_mdio_write(struct net_device *dev, int phy_id,
					  int location, int val)
{
	struct bcm6368_enetsw *priv;

	priv = netdev_priv(dev);
	bcm6368_enetsw_mdio_write(priv,
			bcm6368_enetsw_phy_is_external(priv, phy_id),
			phy_id, location, val);
}

static int bcm6368_enetsw_ioctl(struct net_device *dev, struct ifreq *rq,
				int cmd)
{
	struct mii_if_info mii;

	mii.dev = dev;
	mii.mdio_read = bcm6368_enetsw_mii_mdio_read;
	mii.mdio_write = bcm6368_enetsw_mii_mdio_write;
	mii.phy_id = 0;
	mii.phy_id_mask = 0x3f;
	mii.reg_num_mask = 0x1f;
	return generic_mii_ioctl(&mii, if_mii(rq), cmd, NULL);
}

static const struct net_device_ops bcm6368_enetsw_ops = {
	.ndo_open = bcm6368_enetsw_open,
	.ndo_stop = bcm6368_enetsw_stop,
	.ndo_start_xmit = bcm6368_enetsw_start_xmit,
	.ndo_change_mtu = bcm6368_enetsw_change_mtu,
	.ndo_do_ioctl = bcm6368_enetsw_ioctl,
};

static const struct bcm6368_enetsw_stats bcm6368_enetsw_gstrings_stats[] = {
	{ "rx_packets", DEV_STAT(rx_packets), -1 },
	{ "tx_packets",	DEV_STAT(tx_packets), -1 },
	{ "rx_bytes", DEV_STAT(rx_bytes), -1 },
	{ "tx_bytes", DEV_STAT(tx_bytes), -1 },
	{ "rx_errors", DEV_STAT(rx_errors), -1 },
	{ "tx_errors", DEV_STAT(tx_errors), -1 },
	{ "rx_dropped",	DEV_STAT(rx_dropped), -1 },
	{ "tx_dropped",	DEV_STAT(tx_dropped), -1 },

	{ "tx_good_octets", GEN_STAT(mib.tx_gd_octets), ENETSW_MIB_RX_GD_OCT },
	{ "tx_unicast", GEN_STAT(mib.tx_unicast), ENETSW_MIB_RX_BRDCAST },
	{ "tx_broadcast", GEN_STAT(mib.tx_brdcast), ENETSW_MIB_RX_BRDCAST },
	{ "tx_multicast", GEN_STAT(mib.tx_mult), ENETSW_MIB_RX_MULT },
	{ "tx_64_octets", GEN_STAT(mib.tx_64), ENETSW_MIB_RX_64 },
	{ "tx_65_127_oct", GEN_STAT(mib.tx_65_127), ENETSW_MIB_RX_65_127 },
	{ "tx_128_255_oct", GEN_STAT(mib.tx_128_255), ENETSW_MIB_RX_128_255 },
	{ "tx_256_511_oct", GEN_STAT(mib.tx_256_511), ENETSW_MIB_RX_256_511 },
	{ "tx_512_1023_oct", GEN_STAT(mib.tx_512_1023),
	  ENETSW_MIB_RX_512_1023},
	{ "tx_1024_1522_oct", GEN_STAT(mib.tx_1024_max),
	  ENETSW_MIB_RX_1024_1522 },
	{ "tx_1523_2047_oct", GEN_STAT(mib.tx_1523_2047),
	  ENETSW_MIB_RX_1523_2047 },
	{ "tx_2048_4095_oct", GEN_STAT(mib.tx_2048_4095),
	  ENETSW_MIB_RX_2048_4095 },
	{ "tx_4096_8191_oct", GEN_STAT(mib.tx_4096_8191),
	  ENETSW_MIB_RX_4096_8191 },
	{ "tx_8192_9728_oct", GEN_STAT(mib.tx_8192_9728),
	  ENETSW_MIB_RX_8192_9728 },
	{ "tx_oversize", GEN_STAT(mib.tx_ovr), ENETSW_MIB_RX_OVR },
	{ "tx_oversize_drop", GEN_STAT(mib.tx_ovr), ENETSW_MIB_RX_OVR_DISC },
	{ "tx_dropped",	GEN_STAT(mib.tx_drop), ENETSW_MIB_RX_DROP },
	{ "tx_undersize", GEN_STAT(mib.tx_underrun), ENETSW_MIB_RX_UND },
	{ "tx_pause", GEN_STAT(mib.tx_pause), ENETSW_MIB_RX_PAUSE },

	{ "rx_good_octets", GEN_STAT(mib.rx_gd_octets),
	  ENETSW_MIB_TX_ALL_OCT },
	{ "rx_broadcast", GEN_STAT(mib.rx_brdcast), ENETSW_MIB_TX_BRDCAST },
	{ "rx_multicast", GEN_STAT(mib.rx_mult), ENETSW_MIB_TX_MULT },
	{ "rx_unicast", GEN_STAT(mib.rx_unicast), ENETSW_MIB_TX_MULT },
	{ "rx_pause", GEN_STAT(mib.rx_pause), ENETSW_MIB_TX_PAUSE },
	{ "rx_dropped", GEN_STAT(mib.rx_drop), ENETSW_MIB_TX_DROP_PKTS },
};

#define BCM6368_ENETSW_STATS_LEN (sizeof(bcm6368_enetsw_gstrings_stats) / \
				  sizeof(struct bcm6368_enetsw_stats))

static void bcm6368_enetsw_get_strings(struct net_device *netdev,
				       u32 stringset, u8 *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < BCM6368_ENETSW_STATS_LEN; i++) {
			memcpy(data + i * ETH_GSTRING_LEN,
			       bcm6368_enetsw_gstrings_stats[i].stat_string,
			       ETH_GSTRING_LEN);
		}
		break;
	}
}

static int bcm6368_enetsw_get_sset_count(struct net_device *netdev,
					 int string_set)
{
	switch (string_set) {
	case ETH_SS_STATS:
		return BCM6368_ENETSW_STATS_LEN;
	default:
		return -EINVAL;
	}
}

static void bcm6368_enetsw_get_drvinfo(struct net_device *netdev,
				       struct ethtool_drvinfo *drvinfo)
{
	strncpy(drvinfo->driver, "bcm6368-enetsw", 32);
	strncpy(drvinfo->bus_info, "bcm6368", 32);
}

static void bcm6368_enetsw_get_ethtool_stats(struct net_device *netdev,
					     struct ethtool_stats *stats,
					     u64 *data)
{
	struct bcm6368_enetsw *priv;
	int i;

	priv = netdev_priv(netdev);

	for (i = 0; i < BCM6368_ENETSW_STATS_LEN; i++) {
		const struct bcm6368_enetsw_stats *s;
		u32 lo, hi;
		char *p;
		int reg;

		s = &bcm6368_enetsw_gstrings_stats[i];

		reg = s->mib_reg;
		if (reg == -1)
			continue;

		lo = enetsw_readl(priv, ENETSW_MIB_REG(reg));
		p = (char *)priv + s->stat_offset;

		if (s->sizeof_stat == sizeof(u64)) {
			hi = enetsw_readl(priv, ENETSW_MIB_REG(reg + 1));
			*(u64 *)p = ((u64)hi << 32 | lo);
		} else {
			*(u32 *)p = lo;
		}
	}

	for (i = 0; i < BCM6368_ENETSW_STATS_LEN; i++) {
		const struct bcm6368_enetsw_stats *s;
		char *p;

		s = &bcm6368_enetsw_gstrings_stats[i];

		if (s->mib_reg == -1)
			p = (char *)&netdev->stats + s->stat_offset;
		else
			p = (char *)priv + s->stat_offset;

		data[i] = (s->sizeof_stat == sizeof(u64)) ?
			*(u64 *)p : *(u32 *)p;
	}
}

static void bcm6368_enetsw_get_ringparam(struct net_device *dev,
					 struct ethtool_ringparam *ering)
{
	struct bcm6368_enetsw *priv;

	priv = netdev_priv(dev);

	/* rx/tx ring is actually only limited by memory */
	ering->rx_max_pending = 8192;
	ering->tx_max_pending = 8192;
	ering->rx_mini_max_pending = 0;
	ering->rx_jumbo_max_pending = 0;
	ering->rx_pending = priv->rx_ring_size;
	ering->tx_pending = priv->tx_ring_size;
}

static int bcm6368_enetsw_set_ringparam(struct net_device *dev,
					struct ethtool_ringparam *ering)
{
	struct bcm6368_enetsw *priv;
	int was_running;

	priv = netdev_priv(dev);

	was_running = 0;
	if (netif_running(dev)) {
		bcm6368_enetsw_stop(dev);
		was_running = 1;
	}

	priv->rx_ring_size = ering->rx_pending;
	priv->tx_ring_size = ering->tx_pending;

	if (was_running) {
		int err;

		err = bcm6368_enetsw_open(dev);
		if (err)
			dev_close(dev);
	}

	return 0;
}

static const struct ethtool_ops bcm6368_enetsw_ethtool_ops = {
	.get_strings = bcm6368_enetsw_get_strings,
	.get_sset_count = bcm6368_enetsw_get_sset_count,
	.get_ethtool_stats = bcm6368_enetsw_get_ethtool_stats,
	.get_drvinfo = bcm6368_enetsw_get_drvinfo,
	.get_ringparam = bcm6368_enetsw_get_ringparam,
	.set_ringparam = bcm6368_enetsw_set_ringparam,
};

static int bcm6368_enetsw_probe(struct platform_device *pdev)
{
	struct bcm6368_enetsw *priv;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *of_ports, *of_port;
	struct net_device *ndev;
	struct resource *res;
	const void *mac;
	u16 port_mask = BIT(8);
	unsigned i, num_ports = 0;
	int ret;
	u8 val;

	ndev = alloc_etherdev(sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);

	priv->num_pms = of_count_phandle_with_args(node, "power-domains",
						   "#power-domain-cells");
	if (priv->num_pms > 1) {
		priv->pm = devm_kcalloc(dev, priv->num_pms,
					sizeof(struct device *), GFP_KERNEL);
		if (!priv->pm)
			return -ENOMEM;

		priv->link_pm = devm_kcalloc(dev, priv->num_pms,
					     sizeof(struct device_link *),
					     GFP_KERNEL);
		if (!priv->link_pm)
			return -ENOMEM;

		for (i = 0; i < priv->num_pms; i++) {
			priv->pm[i] = genpd_dev_pm_attach_by_id(dev, i);
			if (IS_ERR(priv->pm[i])) {
				dev_err(dev, "error getting pm %d\n", i);
				return -EINVAL;
			}

			priv->link_pm[i] = device_link_add(dev, priv->pm[i],
				DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME |
				DL_FLAG_RPM_ACTIVE);
		}
	}

	pm_runtime_enable(dev);
	pm_runtime_no_callbacks(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_disable(dev);
		dev_info(dev, "PM prober defer: ret=%d\n", ret);
		return -EPROBE_DEFER;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "enetsw");
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	priv->dma_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->dma_base))
		return PTR_ERR(priv->dma_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "dma-channels");
	priv->dma_chan = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->dma_chan))
		return PTR_ERR(priv->dma_chan);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma-sram");
	priv->dma_sram = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->dma_sram))
		return PTR_ERR(priv->dma_sram);

	priv->irq_rx = platform_get_irq_byname(pdev, "rx");
	if (!priv->irq_rx)
		return -ENODEV;

	priv->irq_tx = platform_get_irq_byname(pdev, "tx");
	if (!priv->irq_tx)
		return -ENODEV;
	else if (priv->irq_tx < 0)
		priv->irq_tx = -1;

	if (device_property_read_u32(dev, "dma-rx", &priv->rx_chan))
		return -ENODEV;

	if (device_property_read_u32(dev, "dma-tx", &priv->tx_chan))
		return -ENODEV;

	priv->rx_ring_size = ENETSW_DEF_RX_DESC;
	priv->tx_ring_size = ENETSW_DEF_TX_DESC;

	priv->dma_maxburst = ENETSW_DMA_MAXBURST;

	priv->copybreak = ENETSW_DEF_CPY_BREAK;

	priv->dma_chan_en_mask = DMAC_CHANCFG_EN_MASK;
	priv->dma_chan_int_mask = DMAC_IR_PKTDONE_MASK;
	priv->dma_chan_width = DMA_CHAN_WIDTH;

	if (device_property_read_bool(dev, "brcm,rgmii-override"))
		priv->rgmii_override = true;
	if (device_property_read_bool(dev, "brcm,rgmii-timing"))
		priv->rgmii_timing = true;

	if (device_property_read_u32(dev, "brcm,num-ports",
				     &priv->num_ports))
		priv->num_ports = ENETSW_MAX_PORT;

	mac = of_get_mac_address(node);
	if (!IS_ERR_OR_NULL(mac)) {
		memcpy(ndev->dev_addr, mac, ETH_ALEN);
		dev_info(dev, "mtd mac %pM\n", ndev->dev_addr);
	} else {
		random_ether_addr(ndev->dev_addr);
		dev_info(dev, "random mac %pM\n", ndev->dev_addr);
	}

	of_ports = of_get_child_by_name(node, "ports");
	if (!of_ports) {
		dev_err(dev, "no ports child node found\n");
		return -EINVAL;
	}

	for_each_available_child_of_node(of_ports, of_port) {
		struct bcm6368_enetsw_port *port;
		struct device_node *of_fixlnk;
		u32 reg;

		ret = of_property_read_u32(of_port, "reg", &reg);
		if (ret)
			return -EINVAL;

		if (reg > ENETSW_MAX_PORT)
			continue;

		port = &priv->used_ports[reg];

		if (of_property_read_s32(of_port, "brcm,phy-id",
					 &port->phy_id))
			continue;

		if (of_property_read_bool(of_port, "brcm,bypass-link"))
			port->bypass_link = true;

		of_fixlnk = of_get_child_by_name(of_port, "fixed-link");
		if (of_fixlnk) {
			if (of_property_read_bool(of_fixlnk, "full-duplex"))
				port->force_duplex_full = true;

			of_property_read_s32(of_fixlnk, "speed",
					     &port->force_speed);
		}

		if (of_property_read_string(of_port, "label", &port->name))
			continue;

		port->used = true;
	}

	ret = bcm6368_enetsw_change_mtu(ndev, ndev->mtu);
	if (ret)
		goto out;

	priv->num_clocks = of_clk_get_parent_count(node);
	if (priv->num_clocks) {
		priv->clock = devm_kcalloc(dev, priv->num_clocks,
					   sizeof(struct clk *), GFP_KERNEL);
		if (!priv->clock)
			return -ENOMEM;
	}
	for (i = 0; i < priv->num_clocks; i++) {
		priv->clock[i] = of_clk_get(node, i);
		if (IS_ERR(priv->clock[i])) {
			dev_err(dev, "error getting clock %d\n", i);
			return -EINVAL;
		}

		ret = clk_prepare_enable(priv->clock[i]);
		if (ret) {
			dev_err(dev, "error enabling clock %d\n", i);
			return ret;
		}
	}

	priv->num_resets = of_count_phandle_with_args(node, "resets",
						      "#reset-cells");
	if (priv->num_resets) {
		priv->reset = devm_kcalloc(dev, priv->num_resets,
					   sizeof(struct reset_control *),
					   GFP_KERNEL);
		if (!priv->reset)
			return -ENOMEM;
	}
	for (i = 0; i < priv->num_resets; i++) {
		priv->reset[i] = devm_reset_control_get_by_index(dev, i);
		if (IS_ERR(priv->reset[i])) {
			dev_err(dev, "error getting reset %d\n", i);
			return -EINVAL;
		}

		ret = reset_control_reset(priv->reset[i]);
		if (ret) {
			dev_err(dev, "error performing reset %d\n", i);
			return ret;
		}
	}

	spin_lock_init(&priv->rx_lock);

	timer_setup(&priv->rx_timeout, bcm6368_enetsw_refill_rx_timer, 0);

	/* register netdevice */
	ndev->netdev_ops = &bcm6368_enetsw_ops;
	netif_napi_add(ndev, &priv->napi, bcm6368_enetsw_poll, 16);
	ndev->ethtool_ops = &bcm6368_enetsw_ethtool_ops;
	SET_NETDEV_DEV(ndev, dev);

	spin_lock_init(&priv->enetsw_mdio_lock);

	ret = register_netdev(ndev);
	if (ret)
		goto out_disable_clk;

	netif_carrier_off(ndev);
	platform_set_drvdata(pdev, ndev);
	priv->pdev = pdev;
	priv->net_dev = ndev;

	/* reset mib */
	val = enetsw_readb(priv, ENETSW_GMCR_REG);
	val |= ENETSW_GMCR_RST_MIB_MASK;
	enetsw_writeb(priv, val, ENETSW_GMCR_REG);
	mdelay(1);
	val &= ~ENETSW_GMCR_RST_MIB_MASK;
	enetsw_writeb(priv, val, ENETSW_GMCR_REG);
	mdelay(1);

	/* force CPU port state */
	val = enetsw_readb(priv, ENETSW_IMPOV_REG);
	val |= ENETSW_IMPOV_FORCE_MASK | ENETSW_IMPOV_LINKUP_MASK;
	enetsw_writeb(priv, val, ENETSW_IMPOV_REG);

	/* enable switch forward engine */
	val = enetsw_readb(priv, ENETSW_SWMODE_REG);
	val |= ENETSW_SWMODE_FWD_EN_MASK;
	enetsw_writeb(priv, val, ENETSW_SWMODE_REG);

	/* enable jumbo on all ports */
	enetsw_writel(priv, 0x1ff, ENETSW_JMBCTL_PORT_REG);
	enetsw_writew(priv, 9728, ENETSW_JMBCTL_MAXSIZE_REG);

	for (i = 0; i < priv->num_ports; i++) {
		struct bcm6368_enetsw_port *port = &priv->used_ports[i];

		if (!port->used)
			continue;

		num_ports++;
		port_mask |= BIT(i);
	}

	/* only register if there is more than one external port */
	if (num_ports > 1)
		bcm6368_enetsw_register(priv, port_mask);

	return 0;

out_disable_clk:
	for (i = 0; i < priv->num_resets; i++)
		reset_control_assert(priv->reset[i]);

	for (i = 0; i < priv->num_clocks; i++)
		clk_disable_unprepare(priv->clock[i]);

out:
	free_netdev(ndev);

	return ret;
}

static int bcm6368_enetsw_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct bcm6368_enetsw *priv = netdev_priv(ndev);
	unsigned int i;

	unregister_netdev(ndev);

	bcm6368_enetsw_unregister(priv);

	pm_runtime_put_sync(dev);
	for (i = 0; priv->pm && i < priv->num_pms; i++) {
		dev_pm_domain_detach(priv->pm[i], true);
		device_link_del(priv->link_pm[i]);
	}

	for (i = 0; i < priv->num_resets; i++)
		reset_control_assert(priv->reset[i]);

	for (i = 0; i < priv->num_clocks; i++)
		clk_disable_unprepare(priv->clock[i]);

	free_netdev(ndev);

	return 0;
}

static const struct of_device_id bcm6368_enetsw_of_match[] = {
	{ .compatible = "brcm,bcm6368-enetsw", },
	{ },
};

static struct platform_driver bcm6368_enetsw_driver = {
	.probe	= bcm6368_enetsw_probe,
	.remove	= bcm6368_enetsw_remove,
	.driver = {
		.name = "bcm6368-enetsw",
		.of_match_table = of_match_ptr(bcm6368_enetsw_of_match),
	},
};

int __init bcm6368_enetsw_init(void)
{
	int ret = platform_driver_register(&bcm6368_enetsw_driver);
	if (ret)
		pr_err("bcm6368-enetsw: Error registering platform driver!\n");
	return ret;
}
late_initcall(bcm6368_enetsw_init);
