// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6348 Ethernet Controller Driver
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

#define BCM6348_ENET_MACS	2

struct bcm6348_mac {
	struct bcm6348_enet *enet;
	void __iomem *base;

	struct clk **clock;
	unsigned int num_clocks;

	int irq_rx;
	int irq_tx;

	int rx_chan;

	int tx_chan;
};

struct bcm6348_enet {
	void __iomem *dma_base;
	void __iomem *dma_chan;
	void __iomem *dma_sram;

	struct reset_control **reset;
	unsigned int num_resets;

	static int num_mac;
	struct bcm6348_mac *macs[BCM6348_ENET_MACS];
};

static int bcm6348_mac_probe(struct bcm6348_enet *enet, struct device_node *child)
{
	struct bcm6348_mac *mac;
	struct net_device *ndev;
	unsigned i;

	ndev = alloc_etherdev(sizeof(*mac));
	if (!ndev)
		return -ENOMEM;

	mac = netdev_priv(ndev);
	mac->enet = enet;
	enet->mac[priv->num_mac] = mac;
	priv->num_mac++;

	/* TODO */

	mac->num_clocks = of_clk_get_parent_count(node);
	if (mac->num_clocks) {
		mac->clock = devm_kcalloc(dev, mac->num_clocks,
					   sizeof(struct clk *), GFP_KERNEL);
		if (!mac->clock)
			return -ENOMEM;
	}
	for (i = 0; i < mac->num_clocks; i++) {
		mac->clock[i] = of_clk_get(node, i);
		if (IS_ERR(mac->clock[i])) {
			dev_err(dev, "error getting clock %d\n", i);
			return -EINVAL;
		}

		ret = clk_prepare_enable(mac->clock[i]);
		if (ret) {
			dev_err(dev, "error enabling clock %d\n", i);
			return ret;
		}
	}

	return 0;
}

static int bcm6348_enet_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *child;
	struct bcm6348_enet *enet;
	struct resource *res;
	unsigned i;

	enet = devm_kzalloc(dev, sizeof(*enet), GFP_KERNEL);
	if (!enet)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	enet->dma_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(enet->dma_base))
		return PTR_ERR(enet->dma_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "dma-channels");
	enet->dma_chan = devm_ioremap_resource(dev, res);
	if (IS_ERR(enet->dma_chan))
		return PTR_ERR(enet->dma_chan);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma-sram");
	enet->dma_sram = devm_ioremap_resource(dev, res);
	if (IS_ERR(enet->dma_sram))
		return PTR_ERR(enet->dma_sram);

	enet->num_resets = of_count_phandle_with_args(node, "resets",
						      "#reset-cells");
	if (enet->num_resets) {
		enet->reset = devm_kcalloc(dev, enet->num_resets,
					   sizeof(struct reset_control *),
					   GFP_KERNEL);
		if (!enet->reset)
			return -ENOMEM;
	}
	for (i = 0; i < enet->num_resets; i++) {
		enet->reset[i] = devm_reset_control_get_by_index(dev, i);
		if (IS_ERR(enet->reset[i])) {
			dev_err(dev, "error getting reset %d\n", i);
			return -EINVAL;
		}

		ret = reset_control_reset(enet->reset[i]);
		if (ret) {
			dev_err(dev, "error performing reset %d\n", i);
			return ret;
		}
	}

	for_each_available_child_of_node(node, child) {
		if (enet->num_mac >= BCM6348_ENET_MACS)
			break;

		if (!of_device_is_compatible(node, "brcm,bcm6348-mac"))
			continue;

		ret = bcm6348_mac_probe(enet, child);
		if (ret)
			return ret;
	}

	return 0;
}

static int bcm6348_enet_remove(struct platform_device *pdev)
{
	/* TODO */

	return 0;
}

static const struct of_device_id bcm6348_enet_of_match[] = {
	{ .compatible = "brcm,bcm6348-enet", },
	{ },
};

static struct platform_driver bcm6348_enet_driver = {
	.probe	= bcm6348_enet_probe,
	.remove	= bcm6348_enet_remove,
	.driver = {
		.name = "bcm6348-enet",
		.of_match_table = of_match_ptr(bcm6348_enet_of_match),
	},
};

int __init bcm6348_enet_init(void)
{
	int ret = platform_driver_register(&bcm6348_enet_driver);
	if (ret)
		pr_err("bcm6348-enet: Error registering platform driver!\n");
	return ret;
}
late_initcall(bcm6348_enet_init);
