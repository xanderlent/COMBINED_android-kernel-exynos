/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * CHUB IF Driver Exynos specific code
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 * Authors:
 *	 Sukwon Ryu <sw.ryoo@samsung.com>
 *
 */

#ifdef CONFIG_SENSOR_DRV
#include "main.h"
#endif
#include "comms.h"
#include "chub.h"
#include "ipc_chub.h"
#include "chub_dbg.h"

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
#elif defined(CONFIG_SOC_EXYNOS9110)
#define REG_CHUB_RESET_CHUB_CONFIGURATION	(0x0)
#define REG_CHUB_RESET_CHUB_STATUS		(0x4)
#define REG_CHUB_RESET_CHUB_OPTION		(0x8)
#define CHUB_RESET_RELEASE_VALUE		(0x20008000)
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

#if defined(CONFIG_SOC_EXYNOS9110)
#define REG_PMU_OSC_RCO			(0xb1c)
#define REG_PMU_RTC_CTRL		(0x73c)
#define REG_PMU_CHUB_CTRL		(0x7c)
#define REG_PMU_CHUB_RESET_STATUS	(0x504)
#define REG_CTRL_REFCLK_PMU		(0x0)
#define REG_CTRL_REFCLK_CHUB_VTS	(0x8)
#define REG_MAILBOX_ISSR0 (0x80)
#define REG_MAILBOX_ISSR1 (0x84)
#define REG_MAILBOX_ISSR2 (0x88)
#define REG_MAILBOX_ISSR3 (0x8c)
#else
#define REG_MAILBOX_ISSR0 (0x100)
#define REG_MAILBOX_ISSR1 (0x104)
#define REG_MAILBOX_ISSR2 (0x108)
#define REG_MAILBOX_ISSR3 (0x10c)
#endif

int contexthub_blk_poweron(struct contexthub_ipc_info *chub);
int contexthub_soc_poweron(struct contexthub_ipc_info *chub);
int contexthub_disable_pin(struct contexthub_ipc_info *chub);
int contexthub_disable_pin(struct contexthub_ipc_info *chub);
int contexthub_get_qch_base(struct contexthub_ipc_info *chub);
int contexthub_set_clk(struct contexthub_ipc_info *chub);
int contexthub_get_clock_names(struct contexthub_ipc_info *chub);
int contexthub_core_reset(struct contexthub_ipc_info *chub);
