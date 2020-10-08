// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CHUB IF Driver Log
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 * Authors:
 *      Boojin Kim <boojin.kim@samsung.com>
 *      Sukwon Ryu <sw.ryoo@samsung.com>
 *
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
#define SIZE_OF_BUFFER (SZ_1M)
#else
#define SIZE_OF_BUFFER (SZ_128K)
#endif

#define S_IRWUG (0660)
#define DEFAULT_FLUSH_MS (1000)

static struct dentry *dbg_root_dir __read_mostly;
static LIST_HEAD(log_list_head);
static struct log_buffer_info *print_info;
u32 auto_log_flush_ms;

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
	struct runtimelog_buf *rt_buf = info->rt_log;
	size_t end, size;
	bool first = (info->log_file_index < 0);
	int result;

	dev_dbg(info->dev, "%s(%zu, %lld)\n", __func__, count, *ppos);

	mutex_lock(&info->lock);

	if (info->log_file_index < 0) {
		info->log_file_index =
		    likely(rt_buf->wrap) ? rt_buf->write_index : 0;
	}

	do {
		end = ((info->log_file_index < rt_buf->write_index) ||
		       ((info->log_file_index == rt_buf->write_index) &&
			!first)) ? rt_buf->write_index : rt_buf->buffer_size;
		size = min(end - info->log_file_index, count);
		if (size == 0) {
			mutex_unlock(&info->lock);
			if (file->f_flags & O_NONBLOCK) {
				dev_dbg(info->dev, "non block\n");
				return -EAGAIN;
			}
			rt_buf->updated = false;

			result = wait_event_interruptible(rt_buf->wq,
							  rt_buf->updated);
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
	    (buf, rt_buf->buffer + info->log_file_index, size)) {
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

	dev_dbg(info->dev, "%s\n", __func__);

	poll_wait(file, &info->rt_log->wq, wait);
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

struct log_buffer_info *log_register_buffer(struct device *dev, int id,
					    struct runtimelog_buf *rt_log,
					    char *name, bool sram)
{
	struct log_buffer_info *info = vmalloc(sizeof(*info));

	if (!info || !rt_log) {
		dev_warn(dev, "%s nullptr", __func__);
		return NULL;
	}

	mutex_init(&info->lock);
	info->id = id;
	info->file_created = false;
	init_waitqueue_head(&rt_log->wq);
	info->dev = dev;
	info->rt_log = rt_log;

	/* HACK: clang make error
	buffer->index_reader = 0;
	buffer->index_writer = 0;
	*/
	info->save_file_name[0] = '\0';
	info->filp = NULL;

	dev_info(dev, "%s with %p buffer size %d.\n",
		 __func__, rt_log->buffer, rt_log->buffer_size);

	debugfs_create_file(name, S_IRWUG, chub_dbg_get_root_dir(), info,
			    &log_fops);

	list_add_tail(&info->list, &log_list_head);

	print_info = info;
	info->sram_log_buffer = false;
	info->support_log_save = false;

	return info;
}
