// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CHUB IF Driver Exynos specific code
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 * Authors:
 *	 Sukwon Ryu <sw.ryoo@samsung.com>
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>

#ifdef CONFIG_SENSOR_DRV
#include "main.h"
#endif
#include "comms.h"
#include "chub.h"
#include "ipc_chub.h"
#include "chub_dbg.h"
#include "chub_exynos.h"

#include <soc/samsung/cal-if.h>

int contexthub_soc_poweron(struct contexthub_ipc_info *chub)
{
	int ret;

	/* pmu reset-release on CHUB */
	ret = cal_chub_reset_release();

	return ret;
}

int contexthub_core_reset(struct contexthub_ipc_info *chub)
{
	nanohub_err("%s: core reset is not implemented except exynos9610\n", __func__);
	return -EINVAL;
}

int contexthub_disable_pin(struct contexthub_ipc_info *chub)
{
	return 0;
}

int contexthub_get_qch_base(struct contexthub_ipc_info *chub)
{
	return 0;
}

static struct clk *contexthub_devm_clk_prepare(struct device *dev, const char *name)
{
	struct clk *clk = NULL;
	int ret;

	clk = devm_clk_get(dev, name);
	if (IS_ERR(clk)) {
		nanohub_dev_err(dev, "Failed to get clock %s\n", name);
		goto error;
	}

	ret = clk_prepare(clk);
	if (ret < 0) {
		nanohub_dev_err(dev, "Failed to prepare clock %s\n", name);
		goto error;
	}

	ret = clk_enable(clk);
	if (ret < 0) {
		nanohub_dev_err(dev, "Failed to enable clock %s\n", name);
		goto error;
	}

error:
	return clk;
}

int contexthub_set_clk(struct contexthub_ipc_info *chub)
{
	struct clk *clk;

	clk = contexthub_devm_clk_prepare(chub->dev, "chub_bus");
	if (!clk)
		return -ENODEV;
	chub->clkrate = clk_get_rate(clk);
	if (!chub->clkrate) {
		dev_info(chub->dev, "clk not set, default %lu\n", chub->clkrate);
		chub->clkrate = 400000000;
	}
	return 0;
}

int contexthub_get_clock_names(struct contexthub_ipc_info *chub)
{
	return 0;
}
