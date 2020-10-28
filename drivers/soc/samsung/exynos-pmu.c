/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS - CPU PMU(Power Management Unit) support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/smp.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <soc/samsung/exynos-pmu.h>
#include <linux/mod_devicetable.h>

/**
 * "pmureg" has the mapped base address of PMU(Power Management Unit)
 */
static struct regmap *pmureg;
static void __iomem *pmu_alive;
static spinlock_t update_lock;

/* Atomic operation for PMU_ALIVE registers. (offset 0~0x3FFF)
   When the targer register can be accessed by multiple masters,
   This functions should be used. */
static inline void exynos_pmu_set_bit_atomic(unsigned int offset, unsigned int val)
{
	__raw_writel(val, pmu_alive + (offset | 0xc000));
}

static inline void exynos_pmu_clr_bit_atomic(unsigned int offset, unsigned int val)
{
	__raw_writel(val, pmu_alive + (offset | 0x8000));
}

/**
 * No driver refers the "pmureg" directly, through the only exported API.
 */
int exynos_pmu_read(unsigned int offset, unsigned int *val)
{
	return regmap_read(pmureg, offset, val);
}

int exynos_pmu_write(unsigned int offset, unsigned int val)
{
	return regmap_write(pmureg, offset, val);
}

int exynos_pmu_update(unsigned int offset, unsigned int mask, unsigned int val)
{
	int i;
	unsigned long flags;

	if (offset > 0x3fff) {
		return regmap_update_bits(pmureg, offset, mask, val);
	} else {
		spin_lock_irqsave(&update_lock, flags);
		for (i = 0; i < 32; i++) {
			if (mask & (1 << i)) {
				if (val & (1 << i))
					exynos_pmu_set_bit_atomic(offset, i);
				else
					exynos_pmu_clr_bit_atomic(offset, i);
			}
		}
		spin_unlock_irqrestore(&update_lock, flags);
		return 0;
	}
}

struct regmap *exynos_get_pmu_regmap(void)
{
	return pmureg;
}

EXPORT_SYMBOL_GPL(exynos_get_pmu_regmap);
EXPORT_SYMBOL(exynos_pmu_read);
EXPORT_SYMBOL(exynos_pmu_write);
EXPORT_SYMBOL(exynos_pmu_update);

#ifdef CONFIG_CP_PMUCAL
#define PMU_CP_STAT		0x0038
int exynos_check_cp_status(void)
{
	unsigned int val;

	exynos_pmu_read(PMU_CP_STAT, &val);

	return val;
}
#endif

static int exynos_pmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;

	pmureg = syscon_regmap_lookup_by_phandle(dev->of_node,
						"samsung,syscon-phandle");
	if (IS_ERR(pmureg)) {
		pr_err("Fail to get regmap of PMU\n");
		return PTR_ERR(pmureg);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmu_alive");
	pmu_alive = devm_ioremap_resource(dev, res);
	if (IS_ERR(pmu_alive)) {
		pr_err("Failed to get address of PMU_ALIVE\n");
		return PTR_ERR(pmu_alive);
	}
	spin_lock_init(&update_lock);
	return 0;
}

static const struct of_device_id of_exynos_pmu_match[] = {
	{ .compatible = "samsung,exynos-pmu", },
	{ },
};

static const struct platform_device_id exynos_pmu_ids[] = {
	{ "exynos-pmu", },
	{ }
};

static struct platform_driver exynos_pmu_driver = {
	.driver = {
		.name = "exynos-pmu",
		.owner = THIS_MODULE,
		.of_match_table = of_exynos_pmu_match,
	},
	.probe		= exynos_pmu_probe,
	.id_table	= exynos_pmu_ids,
};

int __init exynos_pmu_init(void)
{
	return platform_driver_register(&exynos_pmu_driver);
}
subsys_initcall(exynos_pmu_init);
