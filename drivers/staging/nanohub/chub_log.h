/*
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __CHUB_LOG_H_
#define __CHUB_LOG_H_

#include <linux/device.h>
#include "ipc_chub.h"

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

struct log_buffer_info *log_register_buffer(struct device *dev, int id,
					    struct runtimelog_buf *rt_log,
					    char *name, bool sram);

#ifdef CONFIG_CONTEXTHUB_DEBUG
void log_dump_all(int err);
#else
#define log_dump_all(err) do {} while (0)
#endif

void log_printf(const char *format, ...);
#endif /* __CHUB_LOG_H_ */
