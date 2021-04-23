/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS - PMU(Power Management Unit) support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __EXYNOS_POWERMODE_H
#define __EXYNOS_POWERMODE_H __FILE__
#include <soc/samsung/cal-if.h>

extern int exynos_prepare_sys_powerdown(enum sys_powerdown mode);
extern void exynos_wakeup_sys_powerdown(enum sys_powerdown mode, bool early_wakeup);
extern void exynos_prepare_cp_call(void);
extern void exynos_wakeup_cp_call(bool early_wakeup);
extern int exynos_rtc_wakeup(void);

static char *sys_powerdown_str[NUM_SYS_POWERDOWN] = {
	"SICD",
	"SICD_CPD",
	"AFTR",
	"STOP",
	"LPD",
	"LPA",
	"ALPA",
	"DSTOP",
	"SLEEP",
	"SLEEP_VTS_ON",
	"SLEEP_AUD_ON",
	"FAPO",
};

static inline char* get_sys_powerdown_str(int mode)
{
	return sys_powerdown_str[mode];
}

/**
 * Functions for cpuidle driver
 */
extern int exynos_cpu_pm_enter(unsigned int cpu, int index);
extern void exynos_cpu_pm_exit(unsigned int cpu, int enter_failed);

#define MAX_CLUSTER		2

/**
  IDLE_IP control
 */
#define for_each_idle_ip(num)					\
        for ((num) = 0; (num) < NUM_IDLE_IP; (num)++)

#define for_each_syspwr_mode(mode)				\
	for ((mode) = 0; (mode) < NUM_SYS_POWERDOWN; (mode)++)

#define for_each_cluster(id)					\
	for ((id) = 0; (id) < MAX_CLUSTER; (id)++)

/**
 * external driver APIs
 */
#ifdef CONFIG_SERIAL_SAMSUNG
extern void s3c24xx_serial_fifo_wait(void);
#else
static inline void s3c24xx_serial_fifo_wait(void) { }
#endif

#ifdef CONFIG_PINCTRL_EXYNOS
extern u32 exynos_eint_wake_mask_array[2];
#else
/* Mask EINT0 - EINT13 and GPM20 - GPM25 */
u32 exynos_eint_wake_mask_array[2] = {0x00003fff, 0x00003f00};
#endif

/* SUPPORT HOTPLUG */
#ifdef CONFIG_HOTPLUG_CPU
extern int exynos_hotplug_in_callback(unsigned int cpu);
extern int exynos_hotplug_out_callback(unsigned int cpu);
#else
static inline int exynos_hotplug_in_callback(unsigned int cpu) { return 0; }
static inline int exynos_hotplug_out_callback(unsigned int cpu) { return 0; }
#endif

#endif /* __EXYNOS_POWERMODE_H */
