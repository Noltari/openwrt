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
#include <linux/module.h>
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

#define PCIE_BUS_BRIDGE			1
#define PCIE_BUS_DEVICE			2

#define WLAN_OCP_DEV_NUM		1
#define WLAN_OCP_DEV_SLOT		0
#define WLAN_OCP_PCI_HDR_DW_LEN		64
#define WLAN_OCP_PCI_ID			0x435f14e4
#define WLAN_OCP_RES_SIZE		0x2000

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

#define BCM_6362_WLAN_CHIPCOMMON_BASE	(0xb0004000)

#define BCM6362_PCIE_MEM2_BASE		0xa0000000
#define BCM6362_PCIE_MEM2_SIZE		(0x01000000-0x100000)

#define BCM6362_PCI_MEM_BASE		(BCM6362_PCIE_MEM2_BASE + \
					 BCM6362_PCIE_MEM2_SIZE)
#define BCM6362_PCI_MEM_SIZE		0x00100000

#define BCM6362_CB_MEM_BASE		(BCM6362_PCI_MEM_BASE + \
					 BCM6362_PCI_MEM_SIZE)
#define BCM6362_CB_MEM_SIZE		0x01000000

#define BCM6362_PCI_IO_BASE		(BCM6362_CB_MEM_BASE + \
					 BCM6362_CB_MEM_SIZE)
#define BCM6362_PCI_IO_SIZE		0x00010000

#define BCM6362_PCI_IO_BASE_PA		BCM6362_PCI_IO_BASE
#define BCM6362_PCI_IO_END_PA		(BCM6362_PCI_IO_BASE_PA + \
					 BCM6362_PCI_IO_SIZE - 1)

#define BCM6362_PCI_MEM_BASE_PA		BCM6362_PCI_MEM_BASE
#define BCM6362_PCI_MEM_END_PA		(BCM6362_PCI_MEM_BASE_PA + \
					 BCM6362_PCI_MEM_SIZE - 1)

struct bcm6328_pcie {
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

static struct bcm6328_pcie bcm6328_pcie;

extern int bmips_pci_irq;
extern int bmips_wlan_irq;

static void bcm6362_pci_fixup(struct pci_dev *dev)
{
	switch (PCI_SLOT(dev->devfn)) {
	case WLAN_OCP_DEV_SLOT:
		if (((dev->device << 16) | dev->vendor) == WLAN_OCP_PCI_ID) {
			pr_info("resource[0].start=%08x dev->resource[0].end=%08x\n", dev->resource[0].start, dev->resource[0].end);
			pr_info("resource[1].start=%08x dev->resource[1].end=%08x\n", dev->resource[1].start, dev->resource[1].end);
			pr_info("resource[2].start=%08x dev->resource[2].end=%08x\n", dev->resource[2].start, dev->resource[2].end);
			pr_info("resource[3].start=%08x dev->resource[3].end=%08x\n", dev->resource[3].start, dev->resource[3].end);

			pr_info("bcm6362_pci_fixup: devfn=%x dev=%x vendor=%x fixup=%x\n", dev->devfn, dev->device, dev->vendor, (dev->device << 16) | dev->vendor);
			dev->resource[0].start = BCM_6362_WLAN_CHIPCOMMON_BASE;
			dev->resource[0].end = BCM_6362_WLAN_CHIPCOMMON_BASE + WLAN_OCP_RES_SIZE - 1;
		}
		break;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, bcm6362_pci_fixup);

/*
 * swizzle 32bits data to return only the needed part
 */
static int postprocess_read(u32 data, int where, unsigned int size)
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

static int preprocess_write(u32 orig_data, u32 val, int where,
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

u32 bcm6362_wlan_soft_config_space[WLAN_OCP_DEV_NUM][WLAN_OCP_PCI_HDR_DW_LEN] = {
	{
		WLAN_OCP_PCI_ID, 0x00100006, 0x02800000, 0x00000010,
		BCM_6362_WLAN_CHIPCOMMON_BASE, 0x00000000, 0x00000000, 0x00000000,
		0x00000000, 0x00000000, 0x00000000, 0x051314e4,
		0x00000000, 0x00000040, 0x00000000, 0x0000010f,
		0xce035801, 0x00004008, 0x0080d005, 0x00000000,
		0x00000000, 0x00000000, 0x00784809, 0x00000010,
		0x00000000, 0x00000000, 0x00000000, 0x00000000,
		0x00000000, 0x00000000, 0x00000000, 0x00000000,
		0x18001000, 0x00000000, 0xffffffff, 0x00000003,
		0x00000000, 0x00000100, 0x00000000, 0x00000000,
		0x00000000, 0x00000000, 0x00010000, 0x18101000,
		0x00000000, 0x00000000, 0x00000000, 0x00000000,
		0x00000000, 0x00000000, 0x00000000, 0x00000000,
		0x00010010, 0x00288fa0, 0x00190100, 0x00176c11,
		0x30110040, 0x00000000, 0x00000000, 0x00000000,
		0x00000000, 0x00000000, 0x00000000, 0x00000000,
	},
};

static int bcm6362_wlan_pci_read(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 *val)
{
	u32 data;

#if 0
	pr_info("bcm6362_wlan_pci_read: devfn=%x where=%x size=%x\n", devfn, where, size);
#endif

	if (PCI_SLOT(devfn) != WLAN_OCP_DEV_SLOT)
		return PCIBIOS_SUCCESSFUL;

	if (where >= 256)
		data = 0xffffffff;
	else
		data = bcm6362_wlan_soft_config_space[PCI_SLOT(devfn) - WLAN_OCP_DEV_SLOT][where / 4];

	*val = postprocess_read(data, where, size);

	switch (size) {
	case 4:
		/* Special case for reading PCI device range */
		if ((where >= PCI_BASE_ADDRESS_0) && (where <= PCI_BASE_ADDRESS_5)) {
			if (data == 0xffffffff) {
				if (where == PCI_BASE_ADDRESS_0)
					*val = 0xFFFF0000;
				else
					*val = 0;
			}
		}
		break;
	}

#if 0
	pr_info("bcm6362_wlan_pci_read: devfn=%x where=%x size=%x val=%x\n", devfn, where, size, *val);
#endif

	return PCIBIOS_SUCCESSFUL;
}

static int bcm6362_wlan_pci_write(struct pci_bus *bus, unsigned int devfn,
				  int where, int size, u32 val)
{
	u32 data;

#if 0
	pr_info("bcm6362_wlan_pci_write: devfn=%x where=%x size=%x val=%x\n", devfn, where, size, val);
#endif

	if (PCI_SLOT(devfn) != WLAN_OCP_DEV_SLOT)
		return PCIBIOS_SUCCESSFUL;

	if (where >= 256)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	data = bcm6362_wlan_soft_config_space[PCI_SLOT(devfn) - WLAN_OCP_DEV_SLOT][where / 4];
	data = preprocess_write(data, val, where, size);
	bcm6362_wlan_soft_config_space[PCI_SLOT(devfn) - WLAN_OCP_DEV_SLOT][where / 4] = data;

	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops bcm6362_wlan_pci_ops = {
	.read = bcm6362_wlan_pci_read,
	.write = bcm6362_wlan_pci_write
};

static int bcm6328_pcie_can_access(struct pci_bus *bus, int devfn)
{
	struct bcm6328_pcie *priv = &bcm6328_pcie;

	switch (bus->number) {
	case PCIE_BUS_BRIDGE:
		return PCI_SLOT(devfn) == 0;
	case PCIE_BUS_DEVICE:
		if (PCI_SLOT(devfn) == 0)
			return __raw_readl(priv->base + PCIE_DLSTATUS_REG)
			       & DLSTATUS_PHYLINKUP;
		fallthrough;
	default:
		return false;
	}
}

static int bcm6328_pcie_read(struct pci_bus *bus, unsigned int devfn,
			     int where, int size, u32 *val)
{
	struct bcm6328_pcie *priv = &bcm6328_pcie;
	u32 data;
	u32 reg = where & ~3;

	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (!bcm6328_pcie_can_access(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == PCIE_BUS_DEVICE)
		reg += PCIE_DEVICE_OFFSET;

	data = __raw_readl(priv->base + reg);
	*val = postprocess_read(data, where, size);

	return PCIBIOS_SUCCESSFUL;
}

static int bcm6328_pcie_write(struct pci_bus *bus, unsigned int devfn,
			      int where, int size, u32 val)
{
	struct bcm6328_pcie *priv = &bcm6328_pcie;
	u32 data;
	u32 reg = where & ~3;

	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (!bcm6328_pcie_can_access(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == PCIE_BUS_DEVICE)
		reg += PCIE_DEVICE_OFFSET;

	data = __raw_readl(priv->base + reg);
	data = preprocess_write(data, val, where, size);
	__raw_writel(data, priv->base + reg);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops bcm6328_pcie_ops = {
	.read = bcm6328_pcie_read,
	.write = bcm6328_pcie_write,
};

static struct resource bcm6328_pcie_io_resource;
static struct resource bcm6328_pcie_mem_resource;
static struct resource bcm6328_pcie_busn_resource;

static struct pci_controller bcm6328_pcie_controller = {
	.pci_ops = &bcm6328_pcie_ops,
	.io_resource = &bcm6328_pcie_io_resource,
	.mem_resource = &bcm6328_pcie_mem_resource,
};

static struct resource bcm6362_wlan_pci_io_resource = {
	.name = "bcm6362 WLAN PCI IO space",
	.start = BCM6362_PCI_IO_BASE_PA,
	.end = BCM6362_PCI_IO_END_PA,
	.flags = IORESOURCE_IO
};

static struct resource bcm6362_wlan_pci_mem_resource = {
	.name = "bcm6362 WLAN PCI memory space",
	.start = BCM6362_PCI_MEM_BASE_PA,
	.end = BCM6362_PCI_MEM_END_PA,
	.flags = IORESOURCE_MEM
};

struct pci_controller bcm6362_wlan_pci_controller = {
	.pci_ops = &bcm6362_wlan_pci_ops,
	.io_resource = &bcm6362_wlan_pci_io_resource,
	.mem_resource = &bcm6362_wlan_pci_mem_resource,
};

static void bcm6328_pcie_reset(struct bcm6328_pcie *priv)
{
	regmap_write_bits(priv->serdes, 0,
			  SERDES_PCIE_EXD_EN | SERDES_PCIE_EN,
			  SERDES_PCIE_EXD_EN | SERDES_PCIE_EN);

	reset_control_assert(priv->reset);
	reset_control_assert(priv->reset_core);
	reset_control_assert(priv->reset_ext);
	if (priv->reset_hard) {
		reset_control_assert(priv->reset_hard);
		mdelay(10);
		reset_control_deassert(priv->reset_hard);
	}
	mdelay(10);

	reset_control_deassert(priv->reset_core);
	reset_control_deassert(priv->reset);
	mdelay(10);

	reset_control_deassert(priv->reset_ext);
	mdelay(200);
}

static void bcm6328_pcie_setup(struct bcm6328_pcie *priv)
{
	u32 val;

	val = __raw_readl(priv->base + PCIE_BRIDGE_OPT1_REG);
	val |= OPT1_RD_BE_OPT_EN;
	val |= OPT1_RD_REPLY_BE_FIX_EN;
	val |= OPT1_PCIE_BRIDGE_HOLE_DET_EN;
	val |= OPT1_L1_INT_STATUS_MASK_POL;
	__raw_writel(val, priv->base + PCIE_BRIDGE_OPT1_REG);

	val = __raw_readl(priv->base + PCIE_BRIDGE_RC_INT_MASK_REG);
	val |= PCIE_RC_INT_A;
	val |= PCIE_RC_INT_B;
	val |= PCIE_RC_INT_C;
	val |= PCIE_RC_INT_D;
	__raw_writel(val, priv->base + PCIE_BRIDGE_RC_INT_MASK_REG);

	val = __raw_readl(priv->base + PCIE_BRIDGE_OPT2_REG);
	/* enable credit checking and error checking */
	val |= OPT2_TX_CREDIT_CHK_EN;
	val |= OPT2_UBUS_UR_DECODE_DIS;
	/* set device bus/func for the pcie device */
	val |= (PCIE_BUS_DEVICE << OPT2_CFG_TYPE1_BUS_NO_SHIFT);
	val |= OPT2_CFG_TYPE1_BD_SEL;
	__raw_writel(val, priv->base + PCIE_BRIDGE_OPT2_REG);

	/* setup class code as bridge */
	val = __raw_readl(priv->base + PCIE_IDVAL3_REG);
	val &= ~IDVAL3_CLASS_CODE_MASK;
	val |= (PCI_CLASS_BRIDGE_PCI << IDVAL3_SUBCLASS_SHIFT);
	__raw_writel(val, priv->base + PCIE_IDVAL3_REG);

	/* disable bar1 size */
	val = __raw_readl(priv->base + PCIE_CONFIG2_REG);
	val &= ~CONFIG2_BAR1_SIZE_MASK;
	__raw_writel(val, priv->base + PCIE_CONFIG2_REG);

	/* set bar0 to little endian */
	val = (bcm6328_pcie_mem_resource.start >> 20)
	      << BASEMASK_BASE_SHIFT;
	val |= (bcm6328_pcie_mem_resource.end >> 20) << BASEMASK_MASK_SHIFT;
	val |= BASEMASK_REMAP_EN;
	__raw_writel(val, priv->base + PCIE_BRIDGE_BAR0_BASEMASK_REG);

	val = (bcm6328_pcie_mem_resource.start >> 20)
	      << REBASE_ADDR_BASE_SHIFT;
	__raw_writel(val, priv->base + PCIE_BRIDGE_BAR0_REBASE_ADDR_REG);
}

static int bcm6328_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct bcm6328_pcie *priv = &bcm6328_pcie;
	struct resource *res;
	unsigned int i;
	int ret;
	LIST_HEAD(resources);

	pm_runtime_enable(dev);
	pm_runtime_no_callbacks(dev);

	priv->num_pms = of_count_phandle_with_args(np, "power-domains",
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

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_disable(dev);
		dev_info(dev, "PM prober defer: ret=%d\n", ret);
		return -EPROBE_DEFER;
	}

	of_pci_check_probe_only();

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->irq = platform_get_irq(pdev, 0);
	if (!priv->irq)
		return -ENODEV;

	bmips_pci_irq = priv->irq;

	priv->serdes = syscon_regmap_lookup_by_phandle(np, "brcm,serdes");
	if (IS_ERR(priv->serdes))
		return PTR_ERR(priv->serdes);

	priv->reset = devm_reset_control_get(dev, "pcie");
	if (IS_ERR(priv->reset))
		return PTR_ERR(priv->reset);

	priv->reset_ext = devm_reset_control_get(dev, "pcie-ext");
	if (IS_ERR(priv->reset_ext))
		return PTR_ERR(priv->reset_ext);

	priv->reset_core = devm_reset_control_get(dev, "pcie-core");
	if (IS_ERR(priv->reset_core))
		return PTR_ERR(priv->reset_core);

	priv->reset_hard = devm_reset_control_get_optional(dev, "pcie-hard");
	if (IS_ERR(priv->reset_hard))
		return PTR_ERR(priv->reset_hard);

	priv->clk = devm_clk_get(dev, "pcie");
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(dev, "could not enable clock\n");
		return ret;
	}

	pci_load_of_ranges(&bcm6328_pcie_controller, np);
	if (!bcm6328_pcie_mem_resource.start)
		return -EINVAL;

	of_pci_parse_bus_range(np, &bcm6328_pcie_busn_resource);
	pci_add_resource(&resources, &bcm6328_pcie_busn_resource);

	bcm6328_pcie_reset(priv);
	register_pci_controller(&bcm6362_wlan_pci_controller);
	bcm6328_pcie_setup(priv);

	register_pci_controller(&bcm6328_pcie_controller);

	return 0;
}

static const struct of_device_id bcm6328_pcie_of_match[] = {
	{ .compatible = "brcm,bcm6328-pcie", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bcm6328_pcie_of_match);

static struct platform_driver bcm6328_pcie_driver = {
	.probe = bcm6328_pcie_probe,
	.driver	= {
		.name = "bcm6328-pcie",
		.of_match_table = bcm6328_pcie_of_match,
	},
};
module_platform_driver(bcm6328_pcie_driver);

#if 0
0x00    uint32 CcIdA;                               /* CC desc A */
0x04    uint32 CcIdB;                               /* CC desc B */
0x08    uint32 CcAddr;                              /* CC base addr */
0x0c    uint32 MacIdA;                              /* MAC desc A */
0x10    uint32 MacIdB;                              /* MAC desc B */
0x14    uint32 MacAddr;                             /* MAC base addr */
0x18    uint32 ShimIdA;                             /* SHIM desc A */
0x1c    uint32 ShimIdB;                             /* SHIM desc B */
0x20    uint32 ShimAddr;                            /* SHIM addr */
0x24    uint32 ShimEot;                             /* EOT */
0x28    uint32 CcControl;                           /* CC control */                                                
0x2c    uint32 CcStatus;                            /* CC status */                                                
0x30    uint32 MacControl;                          /* MAC control */                                                
0x34    uint32 MacStatus;                           /* MAC status */    
0x38    uint32 ShimMisc;                            /* SHIM control registers */
0x3c    uint32 ShimStatus;                          /* SHIM status */
#endif

#if 0
0x00    uint32 ShimMisc;                            /* SHIM control registers */
0x04    uint32 ShimStatus;                          /* SHIM status */       
0x08    uint32 CcControl;                           /* CC control */
0x0c    uint32 CcStatus;                            /* CC status */
0x10    uint32 MacControl;                          /* MAC control */
0x14    uint32 MacStatus;                           /* MAC status */
0x18    uint32 CcIdA;                               /* CC desc A */
0x1c    uint32 CcIdB;                               /* CC desc B */
0x20    uint32 CcAddr;                              /* CC base addr */
0x24    uint32 MacIdA;                              /* MAC desc A */
0x28    uint32 MacIdB;                              /* MAC desc B */
0x2c    uint32 MacAddr;                             /* MAC base addr */
0x30    uint32 ShimIdA;                             /* SHIM desc A */
0x34    uint32 ShimIdB;                             /* SHIM desc B */
0x38    uint32 ShimAddr;                            /* SHIM addr */
0x3c    uint32 ShimEot;                             /* EOT */     
#endif

#define CHIP_REV_ADDR			((void __iomem *) 0xb0000000)
#define  REV_REVID_SHIFT		0
#define  REV_REVID_MASK			(0xff << REV_REVID_SHIFT)

#define WLAN_SHIM_MISC			0x00
#define WLAN_SHIM_MISC_A0		0x38
#define WLAN_SHIM_FORCE_CLOCKS_ON	BIT(2)
#define WLAN_SHIM_MACRO_DISABLE		BIT(1)
#define WLAN_SHIM_MACRO_SOFT_RESET	BIT(0)

#define WLAN_SHIM_STATUS		0x04
#define WLAN_SHIM_STATUS_A0		0x3c

#define WLAN_CC_CONTROL			0x08
#define WLAN_CC_CONTROL_A0		0x28

#define WLAN_CC_STATUS			0x0c
#define WLAN_CC_STATUS_A0		0x2c

#define WLAN_MAC_CONTROL		0x10
#define WLAN_MAC_CONTROL_A0		0x30
#define SICF_FGC			BIT(1)
#define SICF_CLOCK_EN			BIT(0)

#define WLAN_MAC_STATUS			0x14
#define WLAN_MAC_STATUS_A0		0x34

#define WLAN_CC_ID_A			0x18
#define WLAN_CC_ID_A_A0			0x00

#define WLAN_CC_ID_B			0x1c
#define WLAN_CC_ID_B_A0			0x04

#define WLAN_CC_ADDR			0x20
#define WLAN_CC_ADDR_A0			0x08

#define WLAN_MAC_ID_A			0x24
#define WLAN_MAC_ID_A_A0		0x0c

#define WLAN_MAC_ID_B			0x28
#define WLAN_MAC_ID_B_A0		0x10

#define WLAN_MAC_ADDR			0x2c
#define WLAN_MAC_ADDR_A0		0x14

#define WLAN_SHIM_ID_A			0x30
#define WLAN_SHIM_ID_A_A0		0x18

#define WLAN_SHIM_ID_B			0x34
#define WLAN_SHIM_ID_B_A0		0x1c

#define WLAN_SHIM_ADDR			0x38
#define WLAN_SHIM_ADDR_A0		0x20

#define WLAN_SHIM_EOT			0x3c
#define WLAN_SHIM_EOT_A0		0x24

#define bcm_wlan_shim_writel(v, o) __raw_writel((v), (wlan_shim) + (o))

static int bcm6362_wlan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *wlan_shim;
	struct reset_control *reset_shim;
	struct reset_control *reset_ubus;
	struct resource *res;
	struct clk *clk_ocp;
	u32 chip_rev;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wlan_shim = devm_ioremap_resource(dev, res);
	if (IS_ERR(wlan_shim))
		return PTR_ERR(wlan_shim);

	bmips_wlan_irq = platform_get_irq(pdev, 0);
	if (!bmips_wlan_irq)
		return -ENODEV;

	reset_shim = devm_reset_control_get(dev, "wlan-shim");
	if (IS_ERR(reset_shim))
		return PTR_ERR(reset_shim);

	reset_ubus = devm_reset_control_get(dev, "wlan-ubus");
	if (IS_ERR(reset_ubus))
		return PTR_ERR(reset_ubus);

	clk_ocp = devm_clk_get(dev, "wlan-ocp");
	if (IS_ERR(clk_ocp))
		return PTR_ERR(clk_ocp);

	ret = clk_prepare_enable(clk_ocp);
	if (ret) {
		dev_err(dev, "could not enable clock\n");
		return ret;
	}

	chip_rev = __raw_readl(CHIP_REV_ADDR);
	dev_info(dev, "bcm6362-wlan: detected chip 0x%x\n", chip_rev);

	mdelay(10);
	reset_control_assert(reset_shim);
	reset_control_assert(reset_ubus);
	mdelay(1);
	reset_control_deassert(reset_shim);
	reset_control_deassert(reset_ubus);
	mdelay(1);

	if ((chip_rev & REV_REVID_MASK) == 0xa0)
	{
		bcm_wlan_shim_writel((WLAN_SHIM_FORCE_CLOCKS_ON |
				WLAN_SHIM_MACRO_SOFT_RESET),
				WLAN_SHIM_MISC_A0);
		mdelay(1);
		bcm_wlan_shim_writel((SICF_FGC | SICF_CLOCK_EN),
				WLAN_MAC_CONTROL_A0);
		wmb();
		bcm_wlan_shim_writel(WLAN_SHIM_FORCE_CLOCKS_ON,
				WLAN_SHIM_MISC_A0);
		wmb();
		bcm_wlan_shim_writel(WLAN_SHIM_FORCE_CLOCKS_ON,
				WLAN_SHIM_MISC_A0);
		wmb();
		bcm_wlan_shim_writel(0, WLAN_SHIM_MISC_A0);
		wmb();
		bcm_wlan_shim_writel(SICF_CLOCK_EN, WLAN_MAC_CONTROL_A0);
	} else {
		bcm_wlan_shim_writel((WLAN_SHIM_FORCE_CLOCKS_ON |
				WLAN_SHIM_MACRO_SOFT_RESET),
				WLAN_SHIM_MISC);
		mdelay(1);
		bcm_wlan_shim_writel((SICF_FGC | SICF_CLOCK_EN),
				WLAN_MAC_CONTROL);
		wmb();
		bcm_wlan_shim_writel(WLAN_SHIM_FORCE_CLOCKS_ON,
				WLAN_SHIM_MISC);
		wmb();
		bcm_wlan_shim_writel(WLAN_SHIM_FORCE_CLOCKS_ON,
				WLAN_SHIM_MISC);
		wmb();
		bcm_wlan_shim_writel(0, WLAN_SHIM_MISC);
		wmb();
		bcm_wlan_shim_writel(SICF_CLOCK_EN, WLAN_MAC_CONTROL);
	}

	dev_info(dev, "bcm6362-wlan: setup done!\n");

	return 0;
}

static const struct of_device_id bcm6362_wlan_of_match[] = {
	{ .compatible = "brcm,bcm6362-wlan", },
	{ .compatible = "brcm,bcm63268-wlan", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bcm6362_wlan_of_match);

static struct platform_driver bcm6362_wlan_driver = {
	.probe = bcm6362_wlan_probe,
	.driver	= {
		.name = "bcm6362-wlan",
		.of_match_table = bcm6362_wlan_of_match,
	},
};

int __init bcm6362_wlan_init(void)
{
	int ret = platform_driver_register(&bcm6362_wlan_driver);
	if (ret)
		pr_err("bcm6362-wlan: error registering platform driver!\n");
	return ret;
}
rootfs_initcall(bcm6362_wlan_init);

MODULE_AUTHOR("Álvaro Fernández Rojas <noltari@gmail.com>");
MODULE_DESCRIPTION("BCM6328 PCIe Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bcm6328-pcie");
