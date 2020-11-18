
/*
 * Copyright (c) 2018 Park Bumgyu, Samsung Electronics Co., Ltd <bumgyu.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_CPUPM_H
#define __EXYNOS_CPUPM_H __FILE__

/**
  IDLE_IP control
 */
#define IDLE_IP_REG_SIZE		32
#define IDLE_IP_MAX_INDEX		127
#define IDLE_IP_FIX_INDEX_COUNT		2
#define IDLE_IP_MAX_CONFIGURABLE_INDEX	(IDLE_IP_MAX_INDEX - IDLE_IP_FIX_INDEX_COUNT)

enum exynos_idle_ip {
	IDLE_IP0,
	IDLE_IP1,
	IDLE_IP2,
	IDLE_IP3,
	NUM_IDLE_IP,
};

#ifdef CONFIG_EXYNOS_CPUPM
int exynos_get_idle_ip_index(const char *name);
void exynos_get_idle_ip_list(char *(*idle_ip_list)[IDLE_IP_REG_SIZE]);
void exynos_update_ip_idle_status(int index, int idle);
#else
static inline int exynos_get_idle_ip_index(const char *name) { return 0; }
static inline void exynos_get_idle_ip_list(char *(*idle_ip_list)[IDLE_IP_REG_SIZE]) { return; }
static inline void exynos_update_ip_idle_status(int index, int idle) { return; }
#endif
#endif /* __EXYNOS_CPUPM_H */
