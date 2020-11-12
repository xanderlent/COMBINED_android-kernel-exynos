/*
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *
 * Boojin Kim <boojin.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifdef CONFIG_EXYNOS_ITMON
#include <soc/samsung/exynos-itmon.h>
#endif

#ifdef CONFIG_CHRE_SENSORHUB_HAL
#include "main.h"
#endif
#include "comms.h"
#include "chub.h"
#include "ipc_chub.h"
#include "chub_dbg.h"

#if defined(CONFIG_SOC_S5E9815)
#define PHYS_SRAM_BASE 0x11300000
#endif

/*	PMU CHUB_CPU registers */
#if defined(CONFIG_SOC_EXYNOS9810)
#define REG_CHUB_RESET_CHUB_CONFIGURATION	(0x0)
#define REG_CHUB_CPU_STATUS			(0x0)
#define REG_CHUB_CPU_OPTION			(0x4)
#define ENABLE_SYSRESETREQ			BIT(4)
#define CHUB_RESET_RELEASE_VALUE		(0x10000000)

#elif defined(CONFIG_SOC_EXYNOS9610)
#define REG_CHUB_CPU_CONFIGURATION	(0x0)
#define REG_CHUB_CPU_STATUS			(0x4)
#define REG_CHUB_CPU_OPTION			(0x8)
#define ENABLE_SYSRESETREQ			BIT(9)
#define CHUB_RESET_RELEASE_VALUE	(0x8000)
#define REG_CHUB_CPU_STATUS_BIT_STANDBYWFI	(28)
#define REG_CHUB_RESET_CHUB_OPTION	(0x8)

#elif defined(CONFIG_SOC_EXYNOS9630)
/* for 9630, clk should be : 360000000, 180000000, 120000000 */
#define CHUB_DLL_CLK				360000000
#define REG_CHUB_CPU_CONFIGURATION	(0x0)
#define REG_CHUB_CPU_STATUS			(0x4)
#define REG_CHUB_CPU_OPTION			(0xc)
#define ENABLE_SYSRESETREQ			BIT(9)
#define CHUB_RESET_RELEASE_VALUE		(0x1)
#else
#define REG_CHUB_CPU_STATUS			(0x0)
#define REG_CHUB_CPU_OPTION			(0x0)
#define ENABLE_SYSRESETREQ			BIT(0)
#define CHUB_RESET_RELEASE_VALUE		(0x0)
#endif

#define REG_CHUB_CPU_DURATION			(0x8)
#define REG_CHUB_CPU_STATUS_BIT_STANDBYWFI	(28)

/*	CMU CHUB_QCH registers	*/
#if defined(CONFIG_SOC_EXYNOS9610)
#define REG_QCH_CON_CM4_SHUB_QCH (0x8)
#define IGNORE_FORCE_PM_EN BIT(2)
#define CLOCK_REQ BIT(1)
#define ENABLE BIT(0)
#elif defined(CONFIG_SOC_EXYNOS9630)
#define REG_GPH2_CON				(0x0)
#define REG_GPH2_DAT				(0x4)
#endif

int contexthub_soc_poweron(struct contexthub_ipc_info *ipc);
int contexthub_disable_pin(struct contexthub_ipc_info *ipc);
int contexthub_disable_pin(struct contexthub_ipc_info *ipc);
int contexthub_get_qch_base(struct contexthub_ipc_info *ipc);
int contexthub_set_clk(struct contexthub_ipc_info *ipc);
int contexthub_get_clock_names(struct contexthub_ipc_info *ipc);
int contexthub_core_reset(struct contexthub_ipc_info *ipc);
