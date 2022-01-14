/*
 * Copyright 2020 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clocksource.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <asm/mach/time.h>

/* Registers */
#define TICCNT_0		0x44
#define TICCON_0		0x38
#define CURTICCNT_0		0x90

/* Register Field Values */
#define TICCNT_FULL_RANGE	0xFFFFFFFF
#define TICCKSEL_32768_HZ	(0x0 << 1)
#define TICEN_START		0x1
#define TICEN_CLEAR		0x0

struct s3c_rtc_dev {
	struct device *device;
	void __iomem *base;
	struct timespec64 persistent_ts;
	u64 cycles;
	unsigned int mult;
	unsigned int shift;
};

/* This must be globals because of the persistent clock function signature */
static struct s3c_rtc_dev *global_dev;

static void __s3c_rtc_read_persistent_clock64(struct s3c_rtc_dev *dev, struct timespec64 *ts)
{
	unsigned long long nsecs;
	u64 last_cycles, delta_cycles;

	last_cycles = dev->cycles;

	/* Read new cycle count */
	dev->cycles = readl(dev->base + CURTICCNT_0);

	dev_dbg(dev->device, "cycle count = 0x%08x\n", dev->cycles);

	/* Compute delta and explicitly handle wraps beyond 32 bits */
	if (dev->cycles < last_cycles)
		delta_cycles = 0x100000000ull + dev->cycles - last_cycles;
	else
		delta_cycles = dev->cycles - last_cycles;

	/* Find the nanosecond delta, and update the persistent clock */
	nsecs = clocksource_cyc2ns(delta_cycles, dev->mult, dev->shift);
	timespec64_add_ns(&dev->persistent_ts, nsecs);
	*ts = dev->persistent_ts;
}

static void s3c_rtc_read_persistent_clock64(struct timespec64 *ts)
{
	if (global_dev) {
		__s3c_rtc_read_persistent_clock64(global_dev, ts);
	} else {
		ts->tv_sec = 0;
		ts->tv_nsec = 0;
	}
}

static int s3c_rtc_probe(struct platform_device *pdev)
{
	struct s3c_rtc_dev *dev;
	struct resource *res;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);
	dev->device = &pdev->dev;

	/* get the memory region */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->base))
		return PTR_ERR(dev->base);

	/* Stop and clear count */
	writel(TICEN_CLEAR, dev->base + TICCON_0);

	/* Set counter to use full 32 bit range */
	writel(TICCNT_FULL_RANGE, dev->base + TICCNT_0);

	/* Configure counter to run at 32768 Hz and start */
	writel(TICCKSEL_32768_HZ | TICEN_START, dev->base + TICCON_0);

	/* Determine values for tick conversion to nanoseconds */
	clocks_calc_mult_shift(&dev->mult, &dev->shift, 32768, NSEC_PER_SEC, 120000);

	/* Update hacky global pointer for use by s3c_rtc_read_persistent_clock64() */
	global_dev = dev;

	/* Register s3c_rtc_read_persistent_clock64() as persistent clock */
	register_persistent_clock(s3c_rtc_read_persistent_clock64);

	return 0;
}

static int s3c_rtc_remove(struct platform_device *pdev)
{
	global_dev = NULL;
	return 0;
}

static const struct of_device_id s3c_rtc_dt_match[] = {
	{ .compatible = "samsung,exynos8-rtc" },
	{ },
};
MODULE_DEVICE_TABLE(of, s3c_rtc_dt_match);

static struct platform_driver s3c_rtc_driver = {
	.driver	= {
		.name	= "s3c-rtc",
		.of_match_table	= of_match_ptr(s3c_rtc_dt_match),
	},
	.probe = s3c_rtc_probe,
	.remove = s3c_rtc_remove,
};
module_platform_driver(s3c_rtc_driver);

MODULE_AUTHOR("Tim Kryger <tkryger@google.com>");
MODULE_DESCRIPTION("Samsung S3C RTC Driver (Persistent Clock Only)");
MODULE_LICENSE("GPL v2");
