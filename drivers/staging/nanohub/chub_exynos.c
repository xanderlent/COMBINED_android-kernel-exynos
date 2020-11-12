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

int contexthub_blk_poweron(struct contexthub_ipc_info *chub)
{
	/* only for 9110, which is power down defaut */
#if defined(CONFIG_SOC_EXYNOS9110)
	int val, ret;
	int trycnt = 0;

	if ((IS_ERR_OR_NULL(chub->pmu_osc_rco)) ||
	    (IS_ERR_OR_NULL(chub->pmu_chub_ctrl)) ||
	    (IS_ERR_OR_NULL(chub->pmu_chub_reset_stat))){
		nanohub_err("%s: pmu registers are not completed\n", __func__);
		return -1;
	}
	/* pmu MUX Unset */
	val = __raw_readl(chub->pmu_osc_rco + REG_CTRL_REFCLK_PMU);
	__raw_writel((val & ~(0x1 << 4)), chub->pmu_osc_rco + REG_CTRL_REFCLK_PMU);

	val = __raw_readl(chub->pmu_osc_rco + REG_CTRL_REFCLK_CHUB_VTS);
	__raw_writel((val & ~(0x1 << 4)), chub->pmu_osc_rco + REG_CTRL_REFCLK_CHUB_VTS);

	/* CHUB Block Reset Release */
	val = __raw_readl(chub->pmu_chub_ctrl);
	__raw_writel((val | (0x1 << 9)), chub->pmu_chub_ctrl);

	/* Check Reset Sequence Status */
	do {
		msleep(WAIT_TIMEOUT_MS / 1000);
		val = __raw_readl(chub->pmu_chub_reset_stat);
		val = (val >> 12) & 0x7;
		if (++trycnt > WAIT_TRY_CNT)
		break;
	} while (val != 0x5);

	/* pmu MUX Set */
	val = __raw_readl(chub->pmu_osc_rco + REG_CTRL_REFCLK_PMU);
	__raw_writel((val | (0x1 << 4)), chub->pmu_osc_rco + REG_CTRL_REFCLK_PMU);

	val = __raw_readl(chub->pmu_osc_rco + REG_CTRL_REFCLK_CHUB_VTS);
	__raw_writel((val | (0x1 << 4)), chub->pmu_osc_rco + REG_CTRL_REFCLK_CHUB_VTS);

	ret = exynos_smc(SMC_CMD_CONN_IF,
			 (uint64_t)EXYNOS_CHUB << 32 | EXYNOS_SET_CONN_TZPC, 0, 0);
	if (ret) {
		nanohub_err("%s: exynos_smc failed\n", __func__);
		return ret;
	}
        /* pmu rtc_control Set */
	val = __raw_readl(chub->pmu_rtc_ctrl);
	__raw_writel((val | (0x1 << 0)), chub->pmu_rtc_ctrl);

	/* Set CMU_CHUB CHUB_BUS as 49.152Mhz CLK_RCO_VTS in FW */
	chub->clkrate = 24576000 * 2;
	nanohub_info("%s clk selection of CMU_CHUB is %lu.\n", __func__, chub->clkrate);
#endif
	return 0;
}

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
