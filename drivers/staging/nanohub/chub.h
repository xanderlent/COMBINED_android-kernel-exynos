/*
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *
 * Boojin Kim <boojin.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __CONTEXTHUB_IPC_H_
#define __CONTEXTHUB_IPC_H_

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/pm_wakeup.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/platform_data/nanohub.h>
#include <linux/sched/clock.h>
#include <linux/sched/signal.h>
#include "ipc_chub.h"
#include "chub_log.h"

#include <soc/samsung/exynos-s2mpu.h>
#include <soc/samsung/sysevent.h>
#include <soc/samsung/memlogger.h>
#include <soc/samsung/imgloader.h>

#define WAIT_TRY_CNT (3)
#define RESET_WAIT_TRY_CNT (10)
#define WAIT_CHUB_MS (100)
#define WAIT_CHUB_DFS_SCAN_MS_MAX (120000)
#define WAIT_CHUB_DFS_SCAN_MS (2000)

#define MAX_USI_CNT (15)

#ifndef EXUNOS_SET_CONN_TZPC
#define EXYNOS_CHUB 2ull
#define EXYNOS_SET_CONN_TZPC 0
#endif

#define MEMLOGGER_KMSG
#define LOG_TAG "contexthub: "
extern struct memlog_obj *memlog_printf_chub;
extern struct contexthub_ipc_info *chub_info;
#ifndef MEMLOGGER_KMSG
#define nanohub_debug(fmt, ...) \
	        pr_debug(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define nanohub_info(fmt, ...) \
	        pr_info(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define nanohub_warn(fmt, ...) \
	        pr_warn(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define nanohub_err(fmt, ...) \
	        pr_err(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#else
void chub_printf(int level, int fw_idx, const char *fmt, ...);
#define nanohub_debug(fmt, ...)			chub_printf('D', 0, fmt, ##__VA_ARGS__)
#define nanohub_info(fmt, ...)			chub_printf('I', 0, fmt, ##__VA_ARGS__)
#define nanohub_warn(fmt, ...)			chub_printf('W', 0, fmt, ##__VA_ARGS__)
#define nanohub_err(fmt, ...)			chub_printf('E', 0, fmt, ##__VA_ARGS__)
#define nanohub_dev_debug(dev, fmt, ...)	chub_printf('D', 0, fmt, ##__VA_ARGS__)
#define nanohub_dev_info(dev, fmt, ...)		chub_printf('I', 0, fmt, ##__VA_ARGS__)
#define nanohub_dev_warn(dev, fmt, ...)		chub_printf('W', 0, fmt, ##__VA_ARGS__)
#define nanohub_dev_err(dev, fmt, ...)		chub_printf('E', 0, fmt, ##__VA_ARGS__)
#endif

/* utils for nanohub main */
#define wait_event_interruptible_timeout_locked(q, cond, tmo)		\
({									\
	long __ret = (tmo);						\
	DEFINE_WAIT(__wait);						\
	if (!(cond)) {							\
		for (;;) {						\
			__wait.flags &= ~WQ_FLAG_EXCLUSIVE;		\
			if (list_empty(&__wait.entry))			\
				__add_wait_queue_entry_tail(&(q), &__wait);	\
			set_current_state(TASK_INTERRUPTIBLE);		\
			if ((cond))					\
				break;					\
			if (signal_pending(current)) {			\
				__ret = -ERESTARTSYS;			\
				break;					\
			}						\
			spin_unlock_irq(&(q).lock);			\
			__ret = schedule_timeout(__ret);		\
			spin_lock_irq(&(q).lock);			\
			if (!__ret) {					\
				if ((cond))				\
					__ret = 1;			\
				break;					\
			}						\
		}							\
		__set_current_state(TASK_RUNNING);			\
		if (!list_empty(&__wait.entry))				\
			list_del_init(&__wait.entry);			\
		else if (__ret == -ERESTARTSYS &&			\
			 /*reimplementation of wait_abort_exclusive() */\
			 waitqueue_active(&(q)))			\
			__wake_up_locked_key(&(q), TASK_INTERRUPTIBLE,	\
			NULL);						\
	} else {							\
		__ret = 1;						\
	}								\
	__ret;								\
})

enum mailbox_event {
	MAILBOX_EVT_UTC_MAX = IPC_DEBUG_UTC_MAX,
	MAILBOX_EVT_DUMP_STATUS = IPC_DEBUG_DUMP_STATUS,
	MAILBOX_EVT_DFS_GOVERNOR = IPC_DEBUG_DFS_GOVERNOR,
	MAILBOX_EVT_WAKEUP,
	MAILBOX_EVT_WAKEUP_CLR,
	MAILBOX_EVT_ENABLE_IRQ,
	MAILBOX_EVT_DISABLE_IRQ,
	MAILBOX_EVT_RESET_EVT_START,
	MAILBOX_EVT_INIT_IPC,
	MAILBOX_EVT_POWER_ON,
	MAILBOX_EVT_CHUB_ALIVE,
	MAILBOX_EVT_SHUTDOWN,
	MAILBOX_EVT_RESET,
	MAILBOX_EVT_RT_LOGLEVEL,
	MAILBOX_EVT_MAX,
};

enum chub_status {
	CHUB_ST_NO_POWER,
	CHUB_ST_POWER_ON,
	CHUB_ST_RUN,
	CHUB_ST_SHUTDOWN,
	CHUB_ST_NO_RESPONSE,
	CHUB_ST_ERR,
	CHUB_ST_HANG,
	CHUB_ST_RESET_FAIL,
};

struct read_wait {
	atomic_t cnt;
	atomic_t flag;
	wait_queue_head_t event;
};

struct chub_alive {
	atomic_t flag;
	wait_queue_head_t event;
};

#ifdef USE_EXYNOS_LOG
#define CHUB_DBG_DIR "/data/exynos/log/chub"
#else
#define CHUB_DBG_DIR "/data"
#endif

#define CHUB_RESET_THOLD (3)
#define CHUB_RESET_THOLD_MINOR (5)

enum chub_err_type {
	CHUB_ERR_NONE,
	CHUB_ERR_ITMON, /* ITMON by CHUB */
	CHUB_ERR_S2MPU, /* S2MPU by CHUB */
	CHUB_ERR_FW_FAULT, /* CSP_FAULT by CHUB */
	CHUB_ERR_FW_WDT, /* watchdog by CHUB */
	CHUB_ERR_FW_REBOOT,
	CHUB_ERR_ISR, /* 5 */
	CHUB_ERR_CHUB_NO_RESPONSE,
	CHUB_ERR_CRITICAL, /* critical errors */
	CHUB_ERR_EVTQ_ADD, /* write event error */
	CHUB_ERR_EVTQ_EMTPY, /* isr empty event */
	CHUB_ERR_EVTQ_WAKEUP, /* 10 */
	CHUB_ERR_EVTQ_WAKEUP_CLR,
	CHUB_ERR_WRITE_FAIL,
	CHUB_ERR_MAJER,	/* majer errors */
	CHUB_ERR_READ_FAIL,
	CHUB_ERR_CHUB_ST_ERR,	/* 15: chub_status ST_ERR */
	CHUB_ERR_COMMS_WAKE_ERR,
	CHUB_ERR_FW_ERROR,
	CHUB_ERR_NEED_RESET, /* reset errors */
	CHUB_ERR_NANOHUB_LOG,
	CHUB_ERR_COMMS_NACK, /* 20: ap comms error */
	CHUB_ERR_COMMS_BUSY,
	CHUB_ERR_COMMS_UNKNOWN,
	CHUB_ERR_COMMS,
	CHUB_ERR_RESET_CNT,
	CHUB_ERR_KERNEL_PANIC,
	CHUB_ERR_MAX,
};

struct contexthub_baaw_info {
	unsigned int baaw_p_apm_chub_start;
	unsigned int baaw_p_apm_chub_end;
	unsigned int baaw_p_apm_chub_remap;
	unsigned int baaw_p_cmgp_chub_start;
	unsigned int baaw_p_cmgp_chub_end;
	unsigned int baaw_p_cmgp_chub_remap;
	unsigned int baaw_d_chub_start;
	unsigned int baaw_d_chub_end;
	unsigned int baaw_d_chub_remap;
};

struct contexthub_symbol_addr {
	unsigned int base;
	unsigned int size;
	unsigned int offset;
	unsigned int length;
};

struct contexthub_symbol_table {
	unsigned int size;
	unsigned int count;
	unsigned int name_offset;
	unsigned int reserved;
	struct contexthub_symbol_addr symbol[0];
};

#define CHUB_IRQ_PIN_MAX (5)
struct contexthub_ipc_info {
	u32 cur_err;
	int err_cnt[CHUB_ERR_MAX];
	struct device *dev;
	struct nanohub_data *data;
	struct nanohub_platform_data *pdata;
	struct ipc_info *chub_ipc;
	wait_queue_head_t wakeup_wait;
	struct work_struct debug_work;
	struct work_struct log_work;
	int log_work_reqcnt;
	spinlock_t logout_lock;
	struct read_wait read_lock;
	struct chub_alive chub_alive_lock;
	struct chub_alive poweron_lock;
	struct chub_alive reset_lock;
	void __iomem *sram;
	u32 sram_size;
	void __iomem *mailbox;
	void __iomem *chub_dumpgpr;
	void __iomem *chub_baaw;
	void __iomem *chub_baaw_d;
	void __iomem *chub_dump_cmu;
	void __iomem *chub_dump_sys;
	void __iomem *chub_dump_wdt;
	void __iomem *chub_dump_timer;
	void __iomem *chub_dump_pwm;
	void __iomem *chub_dump_rtc;
	void __iomem *usi_array[MAX_USI_CNT];
	int usi_cnt;
	struct contexthub_baaw_info baaw_info;
	struct ipc_map_area *ipc_map;
	struct log_buffer_info *log_info;
	struct log_buffer_info *dd_log;
	struct LOG_BUFFER *dd_log_buffer;
	struct runtimelog_buf chub_rt_log;
	unsigned long clkrate;
	atomic_t log_work_active;
	atomic_t irq1_apInt;
	atomic_t wakeup_chub;
	atomic_t in_use_ipc;
	int irq_mailbox;
	int irq_wdt;
	bool irq_wdt_disabled;
	int utc_run;
	int powermode;
	atomic_t chub_shutdown;
	atomic_t chub_status;
	atomic_t chub_sleep;
	u32 wakeup_by_chub_cnt;
	atomic_t in_reset;
	int block_reset;
	bool sel_os;
	bool os_load;
	u8 num_os;
	char os_name[MAX_FILE_LEN];
	struct imgloader_desc chub_img_desc[3];
	u32 chub_dfs_gov;
	struct notifier_block itmon_nb;
	struct s2mpufd_notifier_block s2mpu_nb;
	struct wakeup_source *ws;
	struct wakeup_source *ws_reset;
	struct contexthub_symbol_table *symbol_table;
	struct sysevent_desc sysevent_desc;
	struct sysevent_device *sysevent_dev;
#ifdef CONFIG_CONTEXTHUB_DEBUG
	struct work_struct utc_work;
#endif
};

#define SENSOR_VARIATION 10

#define IPC_HW_WRITE_DUMPGPR_CTRL(base, val) \
	__raw_writel((val), (base) + REG_CHUB_DUMPGPR_CTRL)
#define IPC_HW_READ_DUMPGPR_PCR(base) \
	__raw_readl((base) + REG_CHUB_DUMPGPR_PCR)

#define READ_CHUB_USI_CONF(base) \
	__raw_readl((base) + USI_REG_USI_CONFIG)

/*	CHUB BAAW Registers : CHUB BASE + 0x100000 */
#define REG_BAAW_D_CHUB0 (0x0)
#define REG_BAAW_D_CHUB1 (0x4)
#define REG_BAAW_D_CHUB2 (0x8)
#define REG_BAAW_D_CHUB3 (0xc)
#define BAAW_VAL_MAX (4)
#define BAAW_RW_ACCESS_ENABLE 0x80000003

#define IPC_MAX_TIMEOUT (0xffffff)
#define INIT_CHUB_VAL (-1)

#define IPC_HW_WRITE_BAAW_CHUB0(base, val) \
	__raw_writel((val), (base) + REG_BAAW_D_CHUB0)
#define IPC_HW_WRITE_BAAW_CHUB1(base, val) \
	__raw_writel((val), (base) + REG_BAAW_D_CHUB1)
#define IPC_HW_WRITE_BAAW_CHUB2(base, val) \
	__raw_writel((val), (base) + REG_BAAW_D_CHUB2)
#define IPC_HW_WRITE_BAAW_CHUB3(base, val) \
	__raw_writel((val), (base) + REG_BAAW_D_CHUB3)

static inline struct wakeup_source *chub_wake_lock_init(struct device *dev, const char *name)
{
	struct wakeup_source *ws = NULL;

	ws = wakeup_source_register(dev, name);
	if (ws == NULL) {
		nanohub_err("%s: wakelock register fail\n", name);
		return NULL;
	}

	return ws;
}

static inline void chub_wake_lock_destroy(struct wakeup_source *ws)
{
	if (ws == NULL) {
		nanohub_err("wakelock unregister fail\n");
		return;
	}

	wakeup_source_unregister(ws);
}

static inline void chub_wake_lock(struct wakeup_source *ws)
{
	if (ws == NULL) {
		nanohub_err("wakelock fail\n");
		return;
	}

	__pm_stay_awake(ws);
}

static inline void chub_wake_lock_timeout(struct wakeup_source *ws, long timeout)
{
	if (ws == NULL) {
		nanohub_err("wakelock timeout fail\n");
		return;
	}

	__pm_wakeup_event(ws, jiffies_to_msecs(timeout));
}

static inline void chub_wake_unlock(struct wakeup_source *ws)
{
	if (ws == NULL) {
		nanohub_err("wake unlock fail\n");
		return;
	}

	__pm_relax(ws);
}

static inline int chub_wake_lock_active(struct wakeup_source *ws)
{
	if (ws == NULL) {
		nanohub_err("wake unlock fail\n");
		return 0;
	}

	return ws->active;
}

struct clk *contexthub_devm_clk_prepare(struct device *dev, const char *name);
int contexthub_ipc_write_event(struct contexthub_ipc_info *data,
				enum mailbox_event event);
int contexthub_ipc_read(struct contexthub_ipc_info *ipc,
				uint8_t *rx, int max_length, int timeout);
int contexthub_ipc_write(struct contexthub_ipc_info *ipc,
				uint8_t *tx, int length, int timeout);
int contexthub_poweron(struct contexthub_ipc_info *data);
int contexthub_download_image(struct contexthub_ipc_info *data, enum ipc_region reg);
int contexthub_reset(struct contexthub_ipc_info *ipc, bool force_load, enum chub_err_type err);
int contexthub_wakeup(struct contexthub_ipc_info *data, int evt);
void chub_wake_event(struct chub_alive *event);
int contexthub_get_sensortype(struct contexthub_ipc_info *ipc, char *buf);
void contexthub_handle_debug(struct contexthub_ipc_info *ipc, enum chub_err_type err);
int contexthub_sync_memlogger(void);
int contexthub_get_token(struct contexthub_ipc_info *ipc);
void contexthub_put_token(struct contexthub_ipc_info *ipc);
#endif
