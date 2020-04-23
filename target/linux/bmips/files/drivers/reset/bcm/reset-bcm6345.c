// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6345 Reset Controller Driver
 *
 * Copyright (C) 2020 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>

#define BCM6345_RESET_NUM		32
#define BCM6345_RESET_SLEEP_MIN_US	10000
#define BCM6345_RESET_SLEEP_MAX_US	20000

struct bcm6345_reset {
	struct reset_controller_dev rcdev;
	void __iomem *base;
	spinlock_t lock;
};

static int bcm6345_reset_update(struct bcm6345_reset *bcm6345_reset,
				unsigned long id, bool assert)
{
	uint32_t val;

	val = __raw_readl(bcm6345_reset->base);
	if (assert)
		val &= ~BIT(id);
	else
		val |= BIT(id);
	__raw_writel(val, bcm6345_reset->base);

	return 0;
}

static int bcm6345_reset_assert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct bcm6345_reset *bcm6345_reset =
		container_of(rcdev, struct bcm6345_reset, rcdev);
	unsigned long flags;

	spin_lock_irqsave(&bcm6345_reset->lock, flags);
	bcm6345_reset_update(bcm6345_reset, id, true);
	spin_unlock_irqrestore(&bcm6345_reset->lock, flags);

	return 0;
}

static int bcm6345_reset_deassert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	struct bcm6345_reset *bcm6345_reset =
		container_of(rcdev, struct bcm6345_reset, rcdev);
	unsigned long flags;

	spin_lock_irqsave(&bcm6345_reset->lock, flags);
	bcm6345_reset_update(bcm6345_reset, id, false);
	spin_unlock_irqrestore(&bcm6345_reset->lock, flags);

	return 0;
}

static int bcm6345_reset_reset(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct bcm6345_reset *bcm6345_reset =
		container_of(rcdev, struct bcm6345_reset, rcdev);
	unsigned long flags;

	spin_lock_irqsave(&bcm6345_reset->lock, flags);
	usleep_range(BCM6345_RESET_SLEEP_MIN_US,
		     BCM6345_RESET_SLEEP_MAX_US);
	bcm6345_reset_update(bcm6345_reset, id, true);
	usleep_range(BCM6345_RESET_SLEEP_MIN_US,
		     BCM6345_RESET_SLEEP_MAX_US);
	bcm6345_reset_update(bcm6345_reset, id, false);
	spin_unlock_irqrestore(&bcm6345_reset->lock, flags);

	return 0;
}

static int bcm6345_reset_status(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct bcm6345_reset *bcm6345_reset =
		container_of(rcdev, struct bcm6345_reset, rcdev);

	return !(__raw_readl(bcm6345_reset->base) & BIT(id));
}

static struct reset_control_ops bcm6345_reset_ops = {
	.assert = bcm6345_reset_assert,
	.deassert = bcm6345_reset_deassert,
	.reset = bcm6345_reset_reset,
	.status = bcm6345_reset_status,
};

static int __init bcm6345_reset_probe(struct platform_device *pdev)
{
	struct bcm6345_reset *bcm6345_reset;
	struct resource *res;
	int err;

	bcm6345_reset = devm_kzalloc(&pdev->dev,
				     sizeof(*bcm6345_reset), GFP_KERNEL);
	if (!bcm6345_reset)
		return -ENOMEM;

	platform_set_drvdata(pdev, bcm6345_reset);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bcm6345_reset->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bcm6345_reset->base))
		return PTR_ERR(bcm6345_reset->base);

	spin_lock_init(&bcm6345_reset->lock);
	bcm6345_reset->rcdev.ops = &bcm6345_reset_ops;
	bcm6345_reset->rcdev.owner = THIS_MODULE;
	bcm6345_reset->rcdev.of_node = pdev->dev.of_node;
	bcm6345_reset->rcdev.of_reset_n_cells = 1;
	bcm6345_reset->rcdev.nr_resets = BCM6345_RESET_NUM;

	err = devm_reset_controller_register(&pdev->dev,
					     &bcm6345_reset->rcdev);
	if (err)
		return err;

	return 0;
}

static const struct of_device_id bcm6345_reset_of_match[] = {
	{ .compatible = "brcm,bcm6345-reset" },
	{ },
};

static struct platform_driver bcm6345_reset_driver = {
	.probe = bcm6345_reset_probe,
	.driver	= {
		.name = "bcm6345-reset",
		.of_match_table = bcm6345_reset_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(bcm6345_reset_driver);
