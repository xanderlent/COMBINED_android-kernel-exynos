/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * CHUB IF Driver Log
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 * Authors:
 *      Boojin Kim <boojin.kim@samsung.com>
 *      Sukwon Ryu <sw.ryoo@samsung.com>
 *
 */

#ifndef __CHUB_LOG_H_
#define __CHUB_LOG_H_

#include <linux/device.h>
#include <soc/samsung/memlogger.h>

struct memlogs {
	struct memlog *memlog_chub;
	struct memlog_obj *memlog_file_chub;
	struct memlog_obj *memlog_obj_chub;
	struct memlog_obj *memlog_arr_file_chub;
	struct memlog_obj *memlog_arr_chub;
	struct memlog_obj *memlog_sram_file_chub;
	struct memlog_obj *memlog_sram_chub;
	struct memlog_obj *memlog_printf_file_chub;
	struct memlog_obj *memlog_printf_chub;
};

struct log_buffer_info {
	struct list_head list;
	struct device *dev;
	struct file *filp;
	int id;
	bool file_created;
	struct mutex lock;	/* logbuf access lock */
	ssize_t log_file_index;
	char save_file_name[64];
	struct LOG_BUFFER *log_buffer;
	struct runtimelog_buf *rt_log;
	bool sram_log_buffer;
	bool support_log_save;
};

struct LOG_BUFFER {
	volatile u32 index_writer;
	volatile u32 index_reader;
	volatile u32 size;
	volatile u32 full;
	char buffer[0];
};

#define MEMLOGGER_KMSG

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

int contexthub_sync_memlogger(void *chub_p);
void contexthub_log_active_work(void *chub);
int contexthub_log_init(void *chub_p);
#endif /* __CHUB_LOG_H_ */
