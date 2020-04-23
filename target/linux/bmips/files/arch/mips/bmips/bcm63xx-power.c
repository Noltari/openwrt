// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM63xx Power Domain Controller Driver
 *
 * Copyright (C) 2020 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/of.h>
#include <linux/of_device.h>

struct bcm63xx_power_dev {
	struct generic_pm_domain genpd;
	struct bcm63xx_power *power;
	uint32_t mask;
};

struct bcm63xx_power {
	void __iomem *base;
	spinlock_t lock;
	struct bcm63xx_power_dev *dev;
	struct genpd_onecell_data genpd_data;
	struct generic_pm_domain **genpd;
};

struct bcm63xx_power_data {
	const char * const name;
	uint8_t bit;
	unsigned int flags;
};

static int bcm63xx_power_get_state(struct bcm63xx_power_dev *pmd, bool *is_on)
{
	struct bcm63xx_power *power = pmd->power;

	if (!pmd->mask) {
		*is_on = false;
		return -EINVAL;
	}

	*is_on = !(__raw_readl(power->base) & pmd->mask);

	return 0;
}

static int bcm63xx_power_set_state(struct bcm63xx_power_dev *pmd, bool on)
{
	struct bcm63xx_power *power = pmd->power;
	unsigned long flags;
	uint32_t val;

	if (!pmd->mask)
		return -EINVAL;

	spin_lock_irqsave(&power->lock, flags);
	val = __raw_readl(power->base);
	if (on)
		val &= ~pmd->mask;
	else
		val |= pmd->mask;
	__raw_writel(val, power->base);
	spin_unlock_irqrestore(&power->lock, flags);

	return 0;
}

static int bcm63xx_power_on(struct generic_pm_domain *genpd)
{
	struct bcm63xx_power_dev *pmd = container_of(genpd,
		struct bcm63xx_power_dev, genpd);

	return bcm63xx_power_set_state(pmd, true);
}

static int bcm63xx_power_off(struct generic_pm_domain *genpd)
{
	struct bcm63xx_power_dev *pmd = container_of(genpd,
		struct bcm63xx_power_dev, genpd);

	return bcm63xx_power_set_state(pmd, false);
}

static int __init bcm63xx_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	const struct bcm63xx_power_data *entry, *table;
	struct bcm63xx_power *power;
	unsigned int ndom;
	uint8_t max_bit = 0;
	int ret;

	power = devm_kzalloc(dev, sizeof(*power), GFP_KERNEL);
	if (!power)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	power->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(power->base))
		return PTR_ERR(power->base);

	table = of_device_get_match_data(dev);
	if (!table)
		return -EINVAL;

	power->genpd_data.num_domains = 0;
	ndom = 0;
	for (entry = table; entry->name; entry++) {
		max_bit = max(max_bit, entry->bit);
		ndom++;
	}

	if (!ndom)
		return -ENODEV;

	power->genpd_data.num_domains = max_bit + 1;

	power->dev = devm_kcalloc(dev, power->genpd_data.num_domains,
				  sizeof(struct bcm63xx_power_dev),
				  GFP_KERNEL);
	if (!power->dev)
		return -ENOMEM;

	power->genpd = devm_kcalloc(dev, power->genpd_data.num_domains,
				    sizeof(struct generic_pm_domain *),
				    GFP_KERNEL);
	if (!power->genpd)
		return -ENOMEM;

	power->genpd_data.domains = power->genpd;

	ndom = 0;
	for (entry = table; entry->name; entry++) {
		struct bcm63xx_power_dev *pmd = &power->dev[ndom];
		bool is_on;

		pmd->power = power;
		pmd->mask = BIT(entry->bit);
		pmd->genpd.name = entry->name;
		pmd->genpd.flags = entry->flags;

		ret = bcm63xx_power_get_state(pmd, &is_on);
		if (ret)
			dev_warn(dev, "unable to get current state for %s\n",
				 pmd->genpd.name);

		pmd->genpd.power_on = bcm63xx_power_on;
		pmd->genpd.power_off = bcm63xx_power_off;

		pm_genpd_init(&pmd->genpd, NULL, !is_on);
		power->genpd[entry->bit] = &pmd->genpd;

		ndom++;
	}

	spin_lock_init(&power->lock);

	ret = of_genpd_add_provider_onecell(np, &power->genpd_data);
	if (ret) {
		dev_err(dev, "failed to register genpd driver: %d\n", ret);
		return ret;
	}

	dev_info(dev, "registered %u power domains\n", ndom);

	return 0;
}

static const struct bcm63xx_power_data bcm6318_power_domains[] = {
	{
		.name = "pcie",
		.bit = 0,
	}, {
		.name = "usb",
		.bit = 1,
	}, {
		.name = "ephy0",
		.bit = 2,
	}, {
		.name = "ephy1",
		.bit = 3,
	}, {
		.name = "ephy2",
		.bit = 4,
	}, {
		.name = "ephy3",
		.bit = 5,
	}, {
		.name = "ldo2p5",
		.bit = 6,
		.flags = GENPD_FLAG_ALWAYS_ON,
	}, {
		.name = "ldo2p9",
		.bit = 7,
		.flags = GENPD_FLAG_ALWAYS_ON,
	}, {
		.name = "sw1p0",
		.bit = 8,
		.flags = GENPD_FLAG_ALWAYS_ON,
	}, {
		.name = "pad",
		.bit = 9,
		.flags = GENPD_FLAG_ALWAYS_ON,
	}, {
		/* sentinel */
	},
};

static const struct bcm63xx_power_data bcm6328_power_domains[] = {
	{
		.name = "adsl2-mips",
		.bit = 0,
	}, {
		.name = "adsl2-phy",
		.bit = 1,
	}, {
		.name = "adsl2-afe",
		.bit = 2,
	}, {
		.name = "sar",
		.bit = 3,
	}, {
		.name = "pcm",
		.bit = 4,
	}, {
		.name = "usbd",
		.bit = 5,
	}, {
		.name = "usbh",
		.bit = 6,
	}, {
		.name = "pcie",
		.bit = 7,
	}, {
		.name = "robosw",
		.bit = 8,
	}, {
		.name = "ephy",
		.bit = 9,
	}, {
		/* sentinel */
	},
};

static const struct bcm63xx_power_data bcm6362_power_domains[] = {
	{
		.name = "sar",
		.bit = 0,
	}, {
		.name = "ipsec",
		.bit = 1,
	}, {
		.name = "mips",
		.bit = 2,
	}, {
		.name = "dect",
		.bit = 3,
	}, {
		.name = "usbh",
		.bit = 4,
	}, {
		.name = "usbd",
		.bit = 5,
	}, {
		.name = "robosw",
		.bit = 6,
	}, {
		.name = "pcm",
		.bit = 7,
	}, {
		.name = "periph",
		.bit = 8,
	}, {
		.name = "adsl-phy",
		.bit = 9,
	}, {
		.name = "gmii-pads",
		.bit = 10,
	}, {
		.name = "fap",
		.bit = 11,
	}, {
		.name = "pcie",
		.bit = 12,
	}, {
		.name = "wlan-pads",
		.bit = 13,
	}, {
		/* sentinel */
	},
};

static const struct bcm63xx_power_data bcm63268_power_domains[] = {
	{
		.name = "sar",
		.bit = 0,
	}, {
		.name = "ipsec",
		.bit = 1,
	}, {
		.name = "mips",
		.bit = 2,
	}, {
		.name = "dect",
		.bit = 3,
	}, {
		.name = "usbh",
		.bit = 4,
	}, {
		.name = "usbd",
		.bit = 5,
	}, {
		.name = "robosw",
		.bit = 6,
	}, {
		.name = "pcm",
		.bit = 7,
	}, {
		.name = "periph",
		.bit = 8,
	}, {
		.name = "vdsl-phy",
		.bit = 9,
	}, {
		.name = "vdsl-mips",
		.bit = 10,
	}, {
		.name = "fap",
		.bit = 11,
	}, {
		.name = "pcie",
		.bit = 12,
	}, {
		.name = "wlan-pads",
		.bit = 13,
	}, {
		/* sentinel */
	},
};

static const struct of_device_id bcm63xx_power_of_match[] = {
	{
		.compatible = "brcm,bcm6318-power-controller",
		.data = &bcm6318_power_domains,
	}, {
		.compatible = "brcm,bcm6328-power-controller",
		.data = &bcm6328_power_domains,
	}, {
		.compatible = "brcm,bcm6362-power-controller",
		.data = &bcm6362_power_domains,
	}, {
		.compatible = "brcm,bcm63268-power-controller",
		.data = &bcm63268_power_domains,
	}, {
		/* sentinel */
	}
};

static struct platform_driver bcm63xx_power_driver = {
	.probe  = bcm63xx_power_probe,
	.driver = {
		.name = "bcm63xx-power-controller",
		.of_match_table = bcm63xx_power_of_match,
	},
};
builtin_platform_driver(bcm63xx_power_driver);
