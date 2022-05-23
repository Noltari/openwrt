// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6328 PCIe Controller Driver
 *
 * Copyright (C) 2020 Álvaro Fernández Rojas <noltari@gmail.com>
 * Copyright (C) 2015 Jonas Gorski <jonas.gorski@gmail.com>
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#include "../pci.h"

#define SERDES_PCIE_EXD_EN		BIT(15)
#define SERDES_PCIE_EN			BIT(0)

#define PCIE_BUS_BRIDGE			0
#define PCIE_BUS_DEVICE			1

#define PCIE_CONFIG2_REG		0x408
#define CONFIG2_BAR1_SIZE_EN		1
#define CONFIG2_BAR1_SIZE_MASK		0xf

#define PCIE_IDVAL3_REG			0x43c
#define IDVAL3_CLASS_CODE_MASK		0xffffff
#define IDVAL3_SUBCLASS_SHIFT		8
#define IDVAL3_CLASS_SHIFT		16

#define PCIE_DLSTATUS_REG		0x1048
#define DLSTATUS_PHYLINKUP		(1 << 13)

#define PCIE_BRIDGE_OPT1_REG		0x2820
#define OPT1_RD_BE_OPT_EN		(1 << 7)
#define OPT1_RD_REPLY_BE_FIX_EN		(1 << 9)
#define OPT1_PCIE_BRIDGE_HOLE_DET_EN	(1 << 11)
#define OPT1_L1_INT_STATUS_MASK_POL	(1 << 12)

#define PCIE_BRIDGE_OPT2_REG		0x2824
#define OPT2_UBUS_UR_DECODE_DIS		(1 << 2)
#define OPT2_TX_CREDIT_CHK_EN		(1 << 4)
#define OPT2_CFG_TYPE1_BD_SEL		(1 << 7)
#define OPT2_CFG_TYPE1_BUS_NO_SHIFT	16
#define OPT2_CFG_TYPE1_BUS_NO_MASK	(0xff << OPT2_CFG_TYPE1_BUS_NO_SHIFT)

#define PCIE_BRIDGE_BAR0_BASEMASK_REG	0x2828
#define BASEMASK_REMAP_EN		(1 << 0)
#define BASEMASK_SWAP_EN		(1 << 1)
#define BASEMASK_MASK_SHIFT		4
#define BASEMASK_MASK_MASK		(0xfff << BASEMASK_MASK_SHIFT)
#define BASEMASK_BASE_SHIFT		20
#define BASEMASK_BASE_MASK		(0xfff << BASEMASK_BASE_SHIFT)

#define PCIE_BRIDGE_BAR0_REBASE_ADDR_REG 0x282c
#define REBASE_ADDR_BASE_SHIFT		20
#define REBASE_ADDR_BASE_MASK		(0xfff << REBASE_ADDR_BASE_SHIFT)

#define PCIE_BRIDGE_RC_INT_MASK_REG	0x2854
#define PCIE_RC_INT_A			(1 << 0)
#define PCIE_RC_INT_B			(1 << 1)
#define PCIE_RC_INT_C			(1 << 2)
#define PCIE_RC_INT_D			(1 << 3)

#define PCIE_DEVICE_OFFSET		0x8000

struct bcm6328_pcie {
	struct device *dev;
	void __iomem *base;
	int irq;
	struct regmap *serdes;
	struct device **pm;
	struct device_link **link_pm;
	unsigned int num_pms;
	struct clk *clk;
	struct reset_control *reset;
	struct reset_control *reset_ext;
	struct reset_control *reset_core;
	struct reset_control *reset_hard;
};

/*
 * swizzle 32bits data to return only the needed part
 */
static inline int postprocess_read(u32 data, int where, unsigned int size)
{
	u32 ret = 0;

	switch (size) {
	case 1:
		ret = (data >> ((where & 3) << 3)) & 0xff;
		break;
	case 2:
		ret = (data >> ((where & 3) << 3)) & 0xffff;
		break;
	case 4:
		ret = data;
		break;
	}

	return ret;
}

static inline int preprocess_write(u32 orig_data, u32 val, int where,
			    unsigned int size)
{
	u32 ret = 0;

	switch (size) {
	case 1:
		ret = (orig_data & ~(0xff << ((where & 3) << 3))) |
		      (val << ((where & 3) << 3));
		break;
	case 2:
		ret = (orig_data & ~(0xffff << ((where & 3) << 3))) |
		      (val << ((where & 3) << 3));
		break;
	case 4:
		ret = val;
		break;
	}

	return ret;
}

static inline int bcm6328_pcie_can_access(struct pci_bus *bus, int devfn)
{
	struct bcm6328_pcie *pcie = bus->sysdata;

	switch (bus->number) {
	case PCIE_BUS_BRIDGE:
		return PCI_SLOT(devfn) == 0;
	case PCIE_BUS_DEVICE:
		if (PCI_SLOT(devfn) == 0)
			return __raw_readl(pcie->base + PCIE_DLSTATUS_REG)
			       & DLSTATUS_PHYLINKUP;
		fallthrough;
	default:
		return false;
	}
}

static int bcm6328_pcie_read(struct pci_bus *bus, unsigned int devfn,
			     int where, int size, u32 *val)
{
	struct bcm6328_pcie *pcie = bus->sysdata;
	u32 data;
	u32 reg = where & ~3;

	if (!bcm6328_pcie_can_access(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == PCIE_BUS_DEVICE)
		reg += PCIE_DEVICE_OFFSET;

	data = __raw_readl(pcie->base + reg);
	*val = postprocess_read(data, where, size);

	return PCIBIOS_SUCCESSFUL;
}

static int bcm6328_pcie_write(struct pci_bus *bus, unsigned int devfn,
			      int where, int size, u32 val)
{
	struct bcm6328_pcie *pcie = bus->sysdata;
	u32 data;
	u32 reg = where & ~3;

	if (!bcm6328_pcie_can_access(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == PCIE_BUS_DEVICE)
		reg += PCIE_DEVICE_OFFSET;

	data = __raw_readl(pcie->base + reg);
	data = preprocess_write(data, val, where, size);
	__raw_writel(data, pcie->base + reg);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops bcm6328_pcie_ops = {
	.read = bcm6328_pcie_read,
	.write = bcm6328_pcie_write,
};

static void bcm6328_pcie_reset(struct bcm6328_pcie *pcie)
{
	regmap_write_bits(pcie->serdes, 0,
			  SERDES_PCIE_EXD_EN | SERDES_PCIE_EN,
			  SERDES_PCIE_EXD_EN | SERDES_PCIE_EN);

	reset_control_assert(pcie->reset);
	reset_control_assert(pcie->reset_core);
	reset_control_assert(pcie->reset_ext);
	if (pcie->reset_hard) {
		reset_control_assert(pcie->reset_hard);
		mdelay(10);
		reset_control_deassert(pcie->reset_hard);
	}
	mdelay(10);

	reset_control_deassert(pcie->reset_core);
	reset_control_deassert(pcie->reset);
	mdelay(10);

	reset_control_deassert(pcie->reset_ext);
	mdelay(200);
}

static void bcm6328_pcie_setup(struct pci_host_bridge *host, struct bcm6328_pcie *pcie)
{
	struct resource *mem_res = NULL;
	struct resource_entry *win, *tmp;
	u32 val;

	resource_list_for_each_entry_safe(win, tmp, &host->windows) {
		struct resource *res = win->res;

		switch (resource_type(res)) {
		case IORESOURCE_MEM:
			mem_res = res;
			break;
		}
	}

	val = __raw_readl(pcie->base + PCIE_BRIDGE_OPT1_REG);
	val |= OPT1_RD_BE_OPT_EN;
	val |= OPT1_RD_REPLY_BE_FIX_EN;
	val |= OPT1_PCIE_BRIDGE_HOLE_DET_EN;
	val |= OPT1_L1_INT_STATUS_MASK_POL;
	__raw_writel(val, pcie->base + PCIE_BRIDGE_OPT1_REG);

	val = __raw_readl(pcie->base + PCIE_BRIDGE_RC_INT_MASK_REG);
	val |= PCIE_RC_INT_A;
	val |= PCIE_RC_INT_B;
	val |= PCIE_RC_INT_C;
	val |= PCIE_RC_INT_D;
	__raw_writel(val, pcie->base + PCIE_BRIDGE_RC_INT_MASK_REG);

	val = __raw_readl(pcie->base + PCIE_BRIDGE_OPT2_REG);
	/* enable credit checking and error checking */
	val |= OPT2_TX_CREDIT_CHK_EN;
	val |= OPT2_UBUS_UR_DECODE_DIS;
	/* set device bus/func for the pcie device */
	val |= (PCIE_BUS_DEVICE << OPT2_CFG_TYPE1_BUS_NO_SHIFT);
	val |= OPT2_CFG_TYPE1_BD_SEL;
	__raw_writel(val, pcie->base + PCIE_BRIDGE_OPT2_REG);

	/* setup class code as bridge */
	val = __raw_readl(pcie->base + PCIE_IDVAL3_REG);
	val &= ~IDVAL3_CLASS_CODE_MASK;
	val |= (PCI_CLASS_BRIDGE_PCI << IDVAL3_SUBCLASS_SHIFT);
	__raw_writel(val, pcie->base + PCIE_IDVAL3_REG);

	/* disable bar1 size */
	val = __raw_readl(pcie->base + PCIE_CONFIG2_REG);
	val &= ~CONFIG2_BAR1_SIZE_MASK;
	__raw_writel(val, pcie->base + PCIE_CONFIG2_REG);

	if (mem_res) {
		/* set bar0 to little endian */
		val = (mem_res->start >> 20)
			  << BASEMASK_BASE_SHIFT;
		val |= (mem_res->end >> 20) << BASEMASK_MASK_SHIFT;
		val |= BASEMASK_REMAP_EN;
		__raw_writel(val, pcie->base + PCIE_BRIDGE_BAR0_BASEMASK_REG);

		val = (mem_res->start >> 20)
			  << REBASE_ADDR_BASE_SHIFT;
		__raw_writel(val, pcie->base + PCIE_BRIDGE_BAR0_REBASE_ADDR_REG);
	}
}

static int bcm6328_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct bcm6328_pcie *pcie;
	struct pci_host_bridge *host;
	struct resource *res;
	int ret;
	LIST_HEAD(resources);

	host = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!host)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(host);
	pcie->dev = dev;
	platform_set_drvdata(pdev, pcie);

	pm_runtime_enable(dev);
	pm_runtime_no_callbacks(dev);

	pcie->num_pms = of_count_phandle_with_args(np, "power-domains",
						   "#power-domain-cells");
	if (pcie->num_pms > 1) {
		unsigned int i;

		pcie->pm = devm_kcalloc(dev, pcie->num_pms,
					sizeof(struct device *), GFP_KERNEL);
		if (!pcie->pm)
			return -ENOMEM;

		pcie->link_pm = devm_kcalloc(dev, pcie->num_pms,
					     sizeof(struct device_link *),
					     GFP_KERNEL);
		if (!pcie->link_pm)
			return -ENOMEM;

		for (i = 0; i < pcie->num_pms; i++) {
			pcie->pm[i] = genpd_dev_pm_attach_by_id(dev, i);
			if (IS_ERR(pcie->pm[i])) {
				dev_err(dev, "error getting pm %d\n", i);
				return -EINVAL;
			}

			pcie->link_pm[i] = device_link_add(dev, pcie->pm[i],
				DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME |
				DL_FLAG_RPM_ACTIVE);
		}
	}

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_disable(dev);
		dev_info(dev, "PM prober defer: ret=%d\n", ret);
		return -EPROBE_DEFER;
	}

	of_pci_check_probe_only();

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pcie->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->base))
		return PTR_ERR(pcie->base);

	pcie->irq = platform_get_irq(pdev, 0);
	if (!pcie->irq)
		return -ENODEV;

	pcie->serdes = syscon_regmap_lookup_by_phandle(np, "brcm,serdes");
	if (IS_ERR(pcie->serdes))
		return PTR_ERR(pcie->serdes);

	pcie->reset = devm_reset_control_get(dev, "pcie");
	if (IS_ERR(pcie->reset))
		return PTR_ERR(pcie->reset);

	pcie->reset_ext = devm_reset_control_get(dev, "pcie-ext");
	if (IS_ERR(pcie->reset_ext))
		return PTR_ERR(pcie->reset_ext);

	pcie->reset_core = devm_reset_control_get(dev, "pcie-core");
	if (IS_ERR(pcie->reset_core))
		return PTR_ERR(pcie->reset_core);

	pcie->reset_hard = devm_reset_control_get_optional(dev, "pcie-hard");
	if (IS_ERR(pcie->reset_hard))
		return PTR_ERR(pcie->reset_hard);

	pcie->clk = devm_clk_get(dev, "pcie");
	if (IS_ERR(pcie->clk))
		return PTR_ERR(pcie->clk);

	ret = clk_prepare_enable(pcie->clk);
	if (ret) {
		dev_err(dev, "could not enable clock\n");
		return ret;
	}

	bcm6328_pcie_reset(pcie);
	bcm6328_pcie_setup(host, pcie);

	host->ops = &bcm6328_pcie_ops;
	host->sysdata = pcie;

	ret = pci_host_probe(host);
	if (ret) {
		dev_err(dev, "error probbing PCI\n");
		return -EINVAL;
	}

	return 0;
}

static const struct of_device_id bcm6328_pcie_of_match[] = {
	{ .compatible = "brcm,bcm6328-pcie", },
	{ /* sentinel */ }
};

static struct platform_driver bcm6328_pcie_driver = {
	.probe = bcm6328_pcie_probe,
	.driver	= {
		.name = "bcm6328-pcie",
		.of_match_table = bcm6328_pcie_of_match,
	},
};

int __init bcm6328_pcie_init(void)
{
	int ret = platform_driver_register(&bcm6328_pcie_driver);

	if (ret)
		pr_err("pci-bcm6328: Error registering platform driver!\n");

	return ret;
}
late_initcall_sync(bcm6328_pcie_init);
