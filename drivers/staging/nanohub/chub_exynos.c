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

#ifdef CONFIG_CHRE_SENSORHUB_HAL
#include "main.h"
#endif
#include "comms.h"
#include "chub.h"
#include "ipc_chub.h"
#include "chub_dbg.h"
#include "chub_exynos.h"

#include <soc/samsung/cal-if.h>

int contexthub_soc_poweron(struct contexthub_ipc_info *ipc)
{
	int ret;

	/* pmu reset-release on CHUB */
	ret = cal_chub_reset_release();

	return ret;
}

int contexthub_core_reset(struct contexthub_ipc_info *ipc)
{
	nanohub_err("%s: core reset is not implemented except exynos9610\n", __func__);
	return -EINVAL;
}

int contexthub_disable_pin(struct contexthub_ipc_info *ipc)
{
	return 0;
}

int contexthub_get_qch_base(struct contexthub_ipc_info *ipc)
{
	return 0;
}

int contexthub_set_clk(struct contexthub_ipc_info *ipc)
{
	struct clk *clk;

	clk = contexthub_devm_clk_prepare(ipc->dev, "chub_bus");
	if (!clk)
		return -ENODEV;
	ipc->clkrate = clk_get_rate(clk);
	if (!ipc->clkrate) {
		dev_info(ipc->dev, "clk not set, default %lu\n", ipc->clkrate);
		ipc->clkrate = 400000000;
	}
	return 0;
}

int contexthub_get_clock_names(struct contexthub_ipc_info *ipc)
{
	return 0;
}
