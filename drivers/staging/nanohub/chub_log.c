/*
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/iommu.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "chub_log.h"
#include "chub_dbg.h"
#include "ipc_chub.h"

#ifdef CONFIG_CONTEXTHUB_DEBUG
#define SIZE_OF_BUFFER (SZ_128K + SZ_128K)
#else
#define SIZE_OF_BUFFER (SZ_128K)
#endif

#define S_IRWUG (0660)
#define DEFAULT_FLUSH_MS (1000)

static struct dentry *dbg_root_dir __read_mostly;
static LIST_HEAD(log_list_head);
static struct log_buffer_info *print_info;
u32 auto_log_flush_ms;

static void log_memcpy(struct log_buffer_info *info,
		       struct log_kernel_buffer *kernel_buffer,
		       const char *src, size_t size)
{
	size_t left_size = SIZE_OF_BUFFER - kernel_buffer->index;

	dev_dbg(info->dev, "%s(%zu)\n", __func__, size);
	if (size > SIZE_OF_BUFFER) {
		dev_warn(info->dev,
			 "flush size (%zu, %zu) is bigger than kernel buffer size (%d)",
			 size, left_size, SIZE_OF_BUFFER);
		size = SIZE_OF_BUFFER;
	}

	if (left_size < size) {
		if (info->sram_log_buffer)
			memcpy_fromio(kernel_buffer->buffer + kernel_buffer->index, src, left_size);
		else
			memcpy(kernel_buffer->buffer + kernel_buffer->index, src, left_size);

		src += left_size;
		size -= left_size;

		kernel_buffer->index = 0;
		kernel_buffer->wrap = true;
	}

	if (info->sram_log_buffer)
		memcpy_fromio(kernel_buffer->buffer + kernel_buffer->index, src, size);
	else
		memcpy(kernel_buffer->buffer + kernel_buffer->index, src, size);

	kernel_buffer->index += size;
}

void log_flush(struct log_buffer_info *info)
{
	struct LOG_BUFFER *buffer = info->log_buffer;
	struct log_kernel_buffer *kernel_buffer = &info->kernel_buffer;
	unsigned int index_writer = buffer->index_writer;

	/* check logbuf index dueto sram corruption */
	if ((buffer->index_reader >= buffer->size)
		|| (buffer->index_writer >= buffer->size)) {
		dev_err(info->dev, "%s(%d): offset is corrupted. index_writer=%u, index_reader=%u, size=%u\n",
			__func__, info->id, buffer->index_writer, buffer->index_reader, buffer->size);

		return;
	}

	if (buffer->index_reader == index_writer)
		return;

	dev_dbg(info->dev,
		"%s(%d): index_writer=%u, index_reader=%u, size=%u\n", __func__,
		info->id, index_writer, buffer->index_reader, buffer->size);

	mutex_lock(&info->lock);

	if (buffer->index_reader > index_writer) {
		log_memcpy(info, kernel_buffer,
			   buffer->buffer + buffer->index_reader,
			   buffer->size - buffer->index_reader);
		buffer->index_reader = 0;
	}
	log_memcpy(info, kernel_buffer,
		   buffer->buffer + buffer->index_reader,
		   index_writer - buffer->index_reader);
	buffer->index_reader = index_writer;

	wmb();
	mutex_unlock(&info->lock);

	kernel_buffer->updated = true;
	wake_up_interruptible(&kernel_buffer->wq);
}

static void log_flush_all_work_func(struct work_struct *work);
static DECLARE_DEFERRABLE_WORK(log_flush_all_work, log_flush_all_work_func);

void log_flush_all(void)
{
	struct log_buffer_info *info;

	list_for_each_entry(info, &log_list_head, list) {
		if (!info) {
			pr_warn("%s: fails get info\n", __func__);
			return;
		}
		log_flush(info);
	}
}

static void log_flush_all_work_func(struct work_struct *work)
{
	log_flush_all();

	if (auto_log_flush_ms)
		schedule_delayed_work(&log_flush_all_work,
				      msecs_to_jiffies(auto_log_flush_ms));
}

void log_schedule_flush_all(void)
{
	schedule_delayed_work(&log_flush_all_work, msecs_to_jiffies(3000));
}

static int log_file_open(struct inode *inode, struct file *file)
{
	struct log_buffer_info *info = inode->i_private;

	dev_dbg(info->dev, "%s\n", __func__);

	file->private_data = inode->i_private;
	info->log_file_index = -1;

	return 0;
}

static ssize_t log_file_read(struct file *file, char __user *buf, size_t count,
			     loff_t *ppos)
{
	struct log_buffer_info *info = file->private_data;
	struct log_kernel_buffer *kernel_buffer = &info->kernel_buffer;
	size_t end, size;
	bool first = (info->log_file_index < 0);
	int result;

	dev_dbg(info->dev, "%s(%zu, %lld)\n", __func__, count, *ppos);

	mutex_lock(&info->lock);

	if (info->log_file_index < 0) {
		info->log_file_index =
		    likely(kernel_buffer->wrap) ? kernel_buffer->index : 0;
	}

	do {
		end = ((info->log_file_index < kernel_buffer->index) ||
		       ((info->log_file_index == kernel_buffer->index) &&
			!first)) ? kernel_buffer->index : SIZE_OF_BUFFER;
		size = min(end - info->log_file_index, count);
		if (size == 0) {
			mutex_unlock(&info->lock);
			if (file->f_flags & O_NONBLOCK) {
				dev_dbg(info->dev, "non block\n");
				return -EAGAIN;
			}
			kernel_buffer->updated = false;

			result = wait_event_interruptible(kernel_buffer->wq,
				kernel_buffer->updated);
			if (result != 0) {
				dev_dbg(info->dev, "interrupted\n");
				return result;
			}
			mutex_lock(&info->lock);
		}
	} while (size == 0);

	dev_dbg(info->dev, "start=%zd, end=%zd size=%zd\n",
		info->log_file_index, end, size);
	if (copy_to_user
	    (buf, kernel_buffer->buffer + info->log_file_index, size)) {
		mutex_unlock(&info->lock);
		return -EFAULT;
	}

	info->log_file_index += size;
	if (info->log_file_index >= SIZE_OF_BUFFER)
		info->log_file_index = 0;

	mutex_unlock(&info->lock);

	dev_dbg(info->dev, "%s: size = %zd\n", __func__, size);

	return size;
}

static unsigned int log_file_poll(struct file *file, poll_table *wait)
{
	struct log_buffer_info *info = file->private_data;
	struct log_kernel_buffer *kernel_buffer = &info->kernel_buffer;

	dev_dbg(info->dev, "%s\n", __func__);

	poll_wait(file, &kernel_buffer->wq, wait);
	return POLLIN | POLLRDNORM;
}

static const struct file_operations log_fops = {
	.open = log_file_open,
	.read = log_file_read,
	.poll = log_file_poll,
	.llseek = generic_file_llseek,
	.owner = THIS_MODULE,
};

static struct dentry *chub_dbg_get_root_dir(void)
{
	if (!dbg_root_dir)
		dbg_root_dir = debugfs_create_dir("nanohub", NULL);

	return dbg_root_dir;
}

static ssize_t chub_log_flush_show(struct device *kobj,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", auto_log_flush_ms);
}

static ssize_t chub_log_flush_save(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	long event;
	int err;

	err = kstrtol(&buf[0], 10, &event);
	if (!err) {
		if (!event) {
			log_flush_all();
		} else {
			/* update log_flush time */
			auto_log_flush_ms = event;
			pr_err("%s: set flush ms:%d\n", __func__, auto_log_flush_ms);
			schedule_delayed_work(&log_flush_all_work, msecs_to_jiffies(auto_log_flush_ms));
		}
		return count;
	} else {
		pr_err("%s: fails event:%d, err:%d\n", __func__, event, err);
		return 0;
	}
}

static struct device_attribute attributes[] = {
	/* flush sram-logbuf to dram */
	__ATTR(flush_log, 0664, chub_log_flush_show, chub_log_flush_save),
};

struct log_buffer_info *log_register_buffer(struct device *dev, int id,
					    struct LOG_BUFFER *buffer,
					    char *name, bool sram)
{
	struct log_buffer_info *info = vmalloc(sizeof(*info));
	int i;
	int ret;

	if (!info)
		return NULL;

	mutex_init(&info->lock);
	info->id = id;
	info->file_created = false;
	info->kernel_buffer.buffer = vzalloc(SIZE_OF_BUFFER);
	info->kernel_buffer.buffer_size = SIZE_OF_BUFFER;
	info->kernel_buffer.index = 0;
	info->kernel_buffer.index_reader = 0;
	info->kernel_buffer.index_writer = 0;
	info->kernel_buffer.wrap = false;
	init_waitqueue_head(&info->kernel_buffer.wq);
	info->dev = dev;
	info->log_buffer = buffer;

	/* HACK: clang make error
	buffer->index_reader = 0;
	buffer->index_writer = 0;
	*/
	info->save_file_name[0] = '\0';
	info->filp = NULL;

	dev_info(dev, "%s with %p buffer size %d. %p kernel buffer size %d\n",
		 __func__, buffer->buffer, buffer->size,
		 info->kernel_buffer.buffer, SIZE_OF_BUFFER);

	debugfs_create_file(name, S_IRWUG, chub_dbg_get_root_dir(), info,
			    &log_fops);

	list_add_tail(&info->list, &log_list_head);

	if (sram) {
		info->sram_log_buffer = true;
		info->support_log_save = true;

		/* add device files */
		for (i = 0, ret = 0; i < ARRAY_SIZE(attributes); i++) {
			ret = device_create_file(dev, &attributes[i]);
			if (ret)
				dev_warn(dev, "Failed to create file: %s\n",
					 attributes[i].attr.name);
		}
	} else {
		print_info = info;
		info->sram_log_buffer = false;
		info->support_log_save = false;
	}

	return info;
}

void log_printf(const char *format, ...)
{
	struct LOG_BUFFER *buffer;
	int size;
	va_list args;

	if (print_info) {
		char tmp_buf[512];
		char *buffer_index = tmp_buf;

		buffer = print_info->log_buffer;

		va_start(args, format);
		size = vsprintf(tmp_buf, format, args);
		va_end(args);

		size++;
		if (buffer->index_writer + size > buffer->size) {
			int left_size = buffer->size - buffer->index_writer;

			memcpy(&buffer->buffer[buffer->index_writer],
			       buffer_index, left_size);
			buffer->index_writer = 0;
			buffer_index += left_size;
		}
		memcpy(&buffer->buffer[buffer->index_writer], buffer_index,
		       size - (buffer_index - tmp_buf));
		buffer->index_writer += size - (buffer_index - tmp_buf);

	}
}