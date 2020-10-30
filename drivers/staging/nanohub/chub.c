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

#include <linux/module.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/iio/iio.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/random.h>
#include <linux/rtc.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/timekeeping.h>
#include <linux/of_gpio.h>
#include <linux/fcntl.h>
#include <uapi/linux/sched/types.h>

#ifdef CONFIG_EXYNOS_ITMON
#include <soc/samsung/exynos-itmon.h>
#endif

#include "chub.h"
#include "ipc_chub.h"
#include "chub_dbg.h"
#include "chub_log.h"
#include "chub_exynos.h"

#include <soc/samsung/cal-if.h>
#include <soc/samsung/exynos-smc.h>
#include <soc/samsung/exynos-el3_mon.h>
#include <soc/samsung/exynos-s2mpu.h>
#include <soc/samsung/sysevent.h>
#include <soc/samsung/memlogger.h>

#define WAIT_TIMEOUT_MS (1000)
enum { CHUB_ON, CHUB_OFF };
enum { C2A_ON, C2A_OFF };

struct contexthub_ipc_info *chub_info;
static struct memlog *memlog_chub;
struct memlog_obj *memlog_sram_file_chub;
struct memlog_obj *memlog_sram_chub;
struct memlog_obj *memlog_printf_file_chub;
struct memlog_obj *memlog_printf_chub;

const char *os_image[SENSOR_VARIATION] = {
	"os.checked_0.bin",
	"os.checked_1.bin",
	"os.checked_2.bin",
	"os.checked_3.bin",
	"os.checked_4.bin",
	"os.checked_5.bin",
	"os.checked_6.bin",
	"os.checked_7.bin",
	"os.checked_8.bin",
};

static DEFINE_MUTEX(reset_mutex);
static DEFINE_MUTEX(pmu_shutdown_mutex);
static DEFINE_MUTEX(log_mutex);
static DEFINE_MUTEX(wt_mutex);

void chub_printf(int level, int fw_idx, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;
	struct logbuf_output log;
	int n;

	memset(&log, 0, sizeof(struct logbuf_output));
	log.timestamp = ktime_get_boottime_ns() / 1000;
	log.level = level;
	log.size = fw_idx;
	log.buf = kzalloc(240, GFP_KERNEL);
	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;

	memlog_write_printf(memlog_printf_chub, MEMLOG_LEVEL_ERR, "%pV", &vaf);
	pr_info("nanohub: %pV", &vaf);
	n = snprintf(log.buf, 239, "%pV", &vaf);
	log.buf[238] = '\n';
	ipc_logbuf_printf(&log, strlen(log.buf));
	kfree(log.buf);
	va_end(args);
}
EXPORT_SYMBOL(chub_printf);

void chub_wake_event(struct chub_alive *event)
{
	atomic_set(&event->flag, 1);
	wake_up_interruptible_sync(&event->event);
}

static int chub_wait_event(struct chub_alive *event, int timeout)
{
	atomic_set(&event->flag, 0);
	return wait_event_interruptible_timeout(event->event,
						 atomic_read(&event->flag),
						 msecs_to_jiffies(timeout));
}

static inline int contexthub_read_in_reset(struct contexthub_ipc_info *ipc)
{
	return atomic_read(&ipc->in_reset);
}

static inline void contexthub_set_in_reset(struct contexthub_ipc_info *ipc, bool i)
{
	return atomic_set(&ipc->in_reset, i);
}

static inline int contexthub_read_token(struct contexthub_ipc_info *ipc)
{
	if (contexthub_read_in_reset(ipc))
		return -EINVAL;

	return atomic_read(&ipc->in_use_ipc);
}

int contexthub_get_token(struct contexthub_ipc_info *ipc)
{
	if (contexthub_read_in_reset(ipc))
		return -EINVAL;

	atomic_inc(&ipc->in_use_ipc);
	return 0;
}

void contexthub_put_token(struct contexthub_ipc_info *ipc)
{
	atomic_dec(&ipc->in_use_ipc);
}

static void contexthub_reset_token(struct contexthub_ipc_info *ipc)
{
	atomic_set(&ipc->in_use_ipc, 0);
}

/* host interface functions */
int contexthub_is_run(struct contexthub_ipc_info *ipc)
{
	if (!ipc->powermode)
		return 1;

#ifdef CONFIG_CHRE_SENSORHUB_HAL
	return nanohub_irq1_fired(ipc->data);
#else
	return 1;
#endif
}

static inline void contexthub_notify_host(struct contexthub_ipc_info *ipc)
{
#ifdef CONFIG_CHRE_SENSORHUB_HAL
	nanohub_handle_irq1(ipc->data);
#else
	/* TODO */
#endif
}

#ifdef CONFIG_CHRE_SENSORHUB_HAL
/* by nanohub kernel RxBufStruct. packet header is 10 + 2 bytes to align */
struct rxbuf {
	u8 pad;
	u8 pre_preamble;
	u8 buf[PACKET_SIZE_MAX];
	u8 post_preamble;
};

static int nanohub_mailbox_open(void *data)
{
	return 0;
}

static void nanohub_mailbox_close(void *data)
{
	(void)data;
}

static int nanohub_mailbox_write(void *data, uint8_t *tx, int length,
				 int timeout)
{
	struct nanohub_data *ipc = data;

	return contexthub_ipc_write(ipc->pdata->mailbox_client, tx, length, timeout);
}

static int nanohub_mailbox_read(void *data, uint8_t *rx, int max_length,
				int timeout)
{
	struct nanohub_data *ipc = data;

	return contexthub_ipc_read(ipc->pdata->mailbox_client, rx, max_length, timeout);
}

void nanohub_mailbox_comms_init(struct nanohub_comms *comms)
{
	comms->seq = 1;
	comms->timeout_write = 544;
	comms->timeout_ack = 272;
	comms->timeout_reply = 512;
	comms->open = nanohub_mailbox_open;
	comms->close = nanohub_mailbox_close;
	comms->write = nanohub_mailbox_write;
	comms->read = nanohub_mailbox_read;
}
#endif

static int contexthub_read_process(uint8_t *rx, u8 *raw_rx, u32 size)
{
#ifdef CONFIG_CHRE_SENSORHUB_HAL
	struct rxbuf *rxstruct;
	struct nanohub_packet *packet;

	rxstruct = (struct rxbuf *)raw_rx;
	packet = (struct nanohub_packet *)&rxstruct->pre_preamble;
	memcpy_fromio(rx, (void *)packet, size);

	return NANOHUB_PACKET_SIZE(packet->len);
#else
	memcpy_fromio(rx, (void *)raw_rx, size);
	return size;
#endif
}

static void cipc_func_sleep(unsigned int ms)
{
    msleep(ms);
}

static void cipc_func_memcpy(void *dst, void *src, int size, int dst_io, int src_io)
{
	if (dst_io && src_io) {
		nanohub_info("%s: error: not support yet\n", __func__);
		return;
	}
	if (dst_io) {
		memcpy_toio(dst, src, size);
		return;
	}
	if (src_io) {
		memcpy_fromio(dst, src, size);
		return;
	}
	memcpy(dst, src, size);
}

void cipc_func_memset(void *buf, int val, int size, int io)
{
	if (io)
	    memset_io(buf, val, size);
	else
		memset(buf, val, size);
}

static DEFINE_SPINLOCK(ipc_evt_lock);
static DEFINE_SPINLOCK(ipc_data_lock);

static void cipc_func_get_evtlock(unsigned long *flag)
{
	spin_lock_irqsave(&ipc_evt_lock, *flag);
}

static void cipc_func_get_datalock(unsigned long *flag)
{
	spin_lock_irqsave(&ipc_data_lock, *flag);
}

static void cipc_func_put_evtlock(unsigned long *flag)
{
	spin_unlock_irqrestore(&ipc_evt_lock, *flag);
}

static void cipc_func_put_datalock(unsigned long *flag)
{
	spin_unlock_irqrestore(&ipc_data_lock, *flag);
}

static int cipc_func_strncmp(char *dst, char *src, int size)
{
    return strncmp(dst, src, size);
}

static void cipc_func_strncpy(char *dst, char *src, int size, int dst_io, int src_io)
{
	if (!dst_io && !src_io)
		strncpy(dst, (const char *)src, (size_t)size);
	else
		cipc_func_memcpy(dst, src, size, dst_io, src_io);
}

static void cipc_func_logout(const char *str, ...)
{
	va_list vl;

	va_start(vl, str);
	printk(KERN_INFO, str, vl);
	va_end(vl);
}

void cipc_func_handle_irq(int evt, void *priv)
{
	int err;
	int dbg;
	int lock;
	struct contexthub_ipc_info *ipc = priv;

	switch (evt) {
	case IRQ_EVT_C2A_DEBUG:
		if (contexthub_get_token(ipc)) {
			nanohub_dev_warn(ipc->dev, "%s: get token\n", __func__);
			return;
		}
		dbg = ipc_read_value(IPC_VAL_C2A_DEBUG);
		if (dbg >= IPC_DEBUG_CHUB_FAULT) {
			err = (dbg == IPC_DEBUG_CHUB_FAULT) ? CHUB_ERR_FW_FAULT : CHUB_ERR_FW_ERROR;
			ipc_write_value(IPC_VAL_C2A_DEBUG, 0);
			contexthub_put_token(ipc);
			nanohub_dev_err(ipc->dev, "%s: c2a_debug: debug:%d, err:%d\n", __func__, dbg, err);
			contexthub_handle_debug(ipc, err);
		} else {
			nanohub_dev_err(ipc->dev, "%s: c2a_debug: debug:%d\n", __func__, dbg);
			ipc_write_value(IPC_VAL_C2A_DEBUG, 0);
		}
		break;
	case IRQ_EVT_C2A_INT:
		if (atomic_read(&ipc->irq1_apInt) == C2A_OFF) {
			atomic_set(&ipc->irq1_apInt, C2A_ON);
			contexthub_notify_host(ipc);
		}
		break;
	case IRQ_EVT_C2A_INTCLR:
		atomic_set(&ipc->irq1_apInt, C2A_OFF);
		break;
	case IRQ_EVT_C2A_LOG:
		break;
	case CIPC_REG_DATA_CHUB2AP_BATCH:
		/* handle batch data */
		cipc_loopback_test(CIPC_REG_DATA_CHUB2AP_BATCH, 0);
		break;
	default:
		atomic_inc(&ipc->read_lock.cnt);
		/* TODO: requered.. ? */
		spin_lock(&ipc->read_lock.event.lock);
		lock = atomic_read(&ipc->read_lock.flag);
		spin_unlock(&ipc->read_lock.event.lock);
		if (lock)
			wake_up_interruptible_sync(&ipc->read_lock.event);
		break;
	};
	if (ipc_logbuf_filled() && !atomic_read(&ipc->log_work_active)) {
		ipc->log_work_reqcnt++; /* debug */
		atomic_set(&ipc->log_work_active, 1);
		schedule_work(&ipc->log_work);
	}
}

struct cipc_funcs cipc_func = {
    .mbusywait = cipc_func_sleep,
    .memcpy = cipc_func_memcpy,
    .memset = cipc_func_memset,
    .getlock_evt = cipc_func_get_evtlock,
    .putlock_evt = cipc_func_put_evtlock,
    .getlock_data = cipc_func_get_datalock,
    .putlock_data = cipc_func_put_datalock,
    .strncmp = cipc_func_strncmp,
    .strncpy = cipc_func_strncpy,
    .print = cipc_func_logout,
};

#ifdef IPC_DEBUG
static void ipc_test(struct contexthub_ipc_info *chub)
{
	u64 test_val = 0xbb000bb00;
	int i;

	nanohub_info("%s: start: ipc write\n", __func__);
	for (i = 0; i < IPC_VAL_C2A_SENSORID; i++) {
		ipc_write_value(i, test_val);
		if (test_val != ipc_read_value(i))
			nanohub_info("%s: ipc_write_value(%d) fail: %llx <-> %llx\n",
				__func__, i, test_val, ipc_read_value(i));
		else
			nanohub_info("%s: ipc_write_value(%d) pass: %llx <-> %llx\n",
				__func__, i, test_val, ipc_read_value(i));
	}

	nanohub_info("%s: start: ipc hw write\n", __func__);
	test_val = 0xbb00;
	for (i = IPC_VAL_HW_BOOTMODE; i < IPC_VAL_HW_DEBUG; i++) {
		ipc_write_hw_value(i, test_val);
		if (test_val != ipc_read_hw_value(i))
			nanohub_info("%s: ipc_write_value(%d) fail: %llx <-> %llx\n",
				__func__, i, test_val, ipc_read_hw_value(i));
		else
			nanohub_info("%s: ipc_write_value(%d) pass: %llx <-> %llx\n",
				__func__, i, test_val, ipc_read_hw_value(i));
	}
}
#endif

static int contexthub_notifier(struct contexthub_notifier_block *nb)
{
	nanohub_dev_info("%s called!: subsys:%s, start_off:%x, end_off:%x",
		__func__, nb->subsystem, nb->start_off, nb->end_off);
	return 0;
}

struct contexthub_notifier_block chub_cipc_nb = {
	"CHUB", 0, 0, CHUB_FW_ST_INVAL, contexthub_notifier
};

struct contexthub_notifi_info {
	char name[IPC_NAME_MAX];
	enum ipc_owner id;
	struct contexthub_notifier_block *nb;
};

static struct contexthub_notifi_info cipc_noti[IPC_OWN_MAX] = {
	{"EMP", 0, NULL},
	{"CHUB", IPC_OWN_CHUB, NULL},
	{"AP", IPC_OWN_AP, NULL},
	{"GNSS", IPC_OWN_GNSS, NULL},
	{"ABOX", IPC_OWN_ABOX, NULL},
	{"VTS", IPC_OWN_VTS, NULL},
};

static int contexthub_notifier_call(struct contexthub_ipc_info *chub, enum CHUB_STATE state)
{
	int ret = 0;
	int i;
	u32 cipc_start_offset = 0;
	u32 cipc_size = 0;
	u32 ipc_base_offset = 0;
	u32 cipc_base_offset = 0;
	bool find = 0;

	nanohub_dev_info(chub->dev, "%s enters: state:%d\n", __func__, state);
	if (state == CHUB_FW_ST_OFF || state == CHUB_FW_ST_ON) {
		for (i = IPC_OWN_HOST; i < IPC_OWN_MAX; i++) {
			/* Check Notifier call pointer exist */
			if (cipc_noti[i].nb == NULL)
				continue;

			if (cipc_noti[i].nb->state) {
				nanohub_dev_info(chub->dev, "%s (%d:%s) call notifier call on chub-reset\n",
					__func__, i, cipc_noti[i].name);
				cipc_noti[i].nb->state = state;
				ret =  cipc_noti[i].nb->notifier_call(cipc_noti[i].nb);
				if (ret)
					nanohub_dev_info(chub->dev, "%s (%d:%s) fails to notifier ret[%x] --\n", __func__, i, cipc_noti[i].name, ret);
			}
		}
		return 0;
	}

	/* find the cipc base of sussystem */
	for (i = IPC_OWN_HOST; i < IPC_OWN_MAX; i++) {
		nanohub_dev_info(chub->dev, "%s (%d:%s) start to find start_off\n",
			__func__, i, cipc_noti[i].name);
		if ((i == IPC_OWN_MASTER) || (i == IPC_OWN_HOST)) {
			nanohub_dev_info(chub->dev, "%s (%d:%s) skip by master & host\n",
				__func__, i, cipc_noti[i].name);
			continue;
		}
		if (!cipc_noti[i].nb) {
			nanohub_dev_info(chub->dev, "%s (%d:%s) skip by no notifier\n",
				__func__, i, cipc_noti[i].name);
			continue;
		}

		cipc_get_offset_owner(i, &cipc_start_offset, &cipc_size);
		if (cipc_start_offset) {
			nanohub_dev_info(chub->dev, "%s (%d:%s) get start_off:+%x, size:%d\n",
				__func__, i, cipc_noti[i].name, cipc_start_offset, cipc_size);

			cipc_base_offset = cipc_get_base(CIPC_REG_CIPC_BASE) - chub->sram;
			ipc_base_offset = ipc_get_base(IPC_REG_IPC) - chub->sram;
			/* offset check: offset should be 4KB align. And it's bigger than cipc_base. and it's smaller than ipc_end */
			if (((cipc_start_offset % IPC_ALIGN_SIZE) == 0) && ((cipc_size % IPC_ALIGN_SIZE) == 0) &&
				(cipc_base_offset <= cipc_start_offset) &&
				((cipc_start_offset + cipc_size) <= (ipc_base_offset + ipc_get_size(IPC_REG_IPC)))) {
				cipc_noti[i].nb->start_off = cipc_start_offset;
				cipc_noti[i].nb->end_off = cipc_start_offset + cipc_size;
				nanohub_dev_info(chub->dev, "%s (%d:%s) fill notifier:start_off:+%x, end_off:+%x\n",
					__func__, i, cipc_noti[i].name, cipc_noti[i].nb->start_off, cipc_noti[i].nb->end_off);
				find = 1;
			} else {
				nanohub_dev_err(chub->dev, "%s (%d:%s) invalid start_off:+%x(align:%d), end_off:+%x(align:%d), cipc_base_off:+%x, ipc_end_off:+%x\n",
					__func__, i, cipc_noti[i].name,
					cipc_start_offset, cipc_start_offset % IPC_ALIGN_SIZE,
					cipc_start_offset + cipc_size, (cipc_start_offset + cipc_size) % IPC_ALIGN_SIZE,
					cipc_base_offset, ipc_base_offset + ipc_get_size((IPC_REG_IPC)));
			}
		}
	}

	/* call cipc notifiers */
	if (find) {
		for (i = IPC_OWN_HOST; i < IPC_OWN_MAX; i++) {
			/* Check Notifier call pointer exist */
			if (cipc_noti[i].nb == NULL) {
				nanohub_dev_info(chub->dev, "%s (%d:%s) doesn't have notifier call\n",
					__func__, i, cipc_noti[i].name);
				continue;
			}
			nanohub_dev_info(chub->dev, "%s (%d:%s) start_off:+%x, end_off:+%x\n",
				__func__, i, cipc_noti[i].name, cipc_noti[i].nb->start_off, cipc_noti[i].nb->end_off);
			if (cipc_noti[i].nb->start_off) {
				cipc_noti[i].nb->state = state;
				ret =  cipc_noti[i].nb->notifier_call(cipc_noti[i].nb);
				if (ret)
					nanohub_dev_info(chub->dev, "%s (%d:%s) fails to notifier ret[%x] --\n", __func__, i, cipc_noti[i].name, ret);
			}
		}
	}
	return ret;
}

int contexthub_notifier_register(struct contexthub_notifier_block *nb)
{
	int index;

	if (!nb) {
		pr_err("%s: subsystem notifier block is NULL pointer\n", __func__);
		return -EINVAL;
	}
	if (!nb->subsystem || !nb->notifier_call) {
		pr_err("%s: subsystem is NULL pointer\n", __func__);
		return -EINVAL;
	}

	for (index = IPC_OWN_HOST; index < IPC_OWN_MAX; index++) {
		if (!strncmp(cipc_noti[index].name, nb->subsystem, IPC_NAME_MAX))
			break;
	}

	if (index >= IPC_OWN_MAX) {
		pr_err("%s: can't find subsystem:%s\n",
		__func__, nb->subsystem);
		return -EINVAL;
	}

	nb->start_off = 0;
	nb->end_off = 0;
	cipc_noti[index].nb = nb;
	pr_info("%s: (%s) register successful!\n", __func__, nb->subsystem);
	return 0;
}
EXPORT_SYMBOL(contexthub_notifier_register);

int contexthub_sync_memlogger(void)
{
	int ret = 0;

	if (memlog_printf_chub)
		ret |= memlog_sync_to_file(memlog_printf_chub);

	return ret;
}

static int contexthub_chub_ipc_init(struct contexthub_ipc_info *chub)
{
	u32 cipc_start_offset = 0;
	int cipc_size;
	int ret;

	nanohub_dev_info(chub->dev, "%s : chub_ipc\n", __func__);
	cipc_func.priv = chub;
	if (!chub->chub_ipc) {
		chub->chub_ipc = ipc_init(IPC_OWN_AP, IPC_SRC_MB0, chub->sram, chub->mailbox, &cipc_func);
		if (!chub->chub_ipc) {
			nanohub_dev_err(chub->dev, "%s: ipc_init failed\n", __func__);
			return -EINVAL;
		}

		nanohub_dev_info(chub->dev, "%s: ipc_init success\n", __func__);
		ret = cipc_register(chub->mailbox, CIPC_USER_CHUB2AP, CIPC_USER_AP2CHUB, cipc_func_handle_irq, chub, &cipc_start_offset, &cipc_size);
		if (ret) {
			CSP_PRINTF_ERROR("%s: c2a cipc_request fails\n", __func__);
			return -EINVAL;
		}
		nanohub_dev_info(chub->dev, "%s: cipc_register success: offset:+%x, size:%d\n", __func__, cipc_start_offset, cipc_size);
	} else {
		nanohub_dev_info(chub->dev, "%s : with reset\n", __func__);
		ipc_reset_map();
	}
	return 0;
}

static int contexthub_ipc_drv_init(struct contexthub_ipc_info *chub)
{
	int ret = 0;

	ret = contexthub_chub_ipc_init(chub);
	if (ret)
		nanohub_err("%s: fails in contexthub_ipc_drv_init. ret:%d\n", __func__, ret);
	else {
		ret = chub_dbg_init(chub, chub->chub_rt_log.buffer, chub->chub_rt_log.buffer_size);
		if (ret)
			nanohub_err("%s: fails in chub_dbg_init. ret:%d\n", __func__, ret);
	}
	return ret;
}

#ifdef PACKET_LOW_DEBUG
static void debug_dumpbuf(unsigned char *buf, int len)
{
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET, 16, 1, buf, len,
		       false);
}
#endif

/* simple alive check function : don't use ipc map */
static bool contexthub_lowlevel_alive(struct contexthub_ipc_info *ipc)
{
	int ret;
	struct ipc_info *chub_ipc = ipc->chub_ipc;

	atomic_set(&ipc->chub_alive_lock.flag, 0);
	ipc_hw_gen_interrupt(ipc->mailbox, chub_ipc->opp_mb_id, IRQ_NUM_CHUB_ALIVE);
	ret = chub_wait_event(&ipc->chub_alive_lock, 200);
	nanohub_dev_info(ipc->dev, "%s done: ret:%d\n", __func__, ret);
	return atomic_read(&ipc->chub_alive_lock.flag);
}

/* contexhub slient reset support */
void contexthub_handle_debug(struct contexthub_ipc_info *ipc,
	enum chub_err_type err)
{
	int thold = (err < CHUB_ERR_CRITICAL) ? 1 :
		   ((err < CHUB_ERR_MAJER) ? CHUB_RESET_THOLD : CHUB_RESET_THOLD_MINOR);

	/* update error count */
	ipc->err_cnt[err]++;

	nanohub_dev_info(ipc->dev, "%s: err:%d(cnt:%d), cur_err:%d, thold:%d, chub_status:%d\n",
		__func__, err, ipc->err_cnt[err], ipc->cur_err, thold, atomic_read(&ipc->chub_status));

	/* set chub_status to CHUB_ST_ERR. request silent reset */
	if ((ipc->err_cnt[err] >= thold) && (atomic_read(&ipc->chub_status) != CHUB_ST_ERR)) {
		atomic_set(&ipc->chub_status, CHUB_ST_ERR);
		nanohub_dev_info(ipc->dev, "%s: err:%d(cnt:%d), enter error status\n",
			__func__, err, ipc->err_cnt[err]);
		/* handle err */
		ipc->cur_err = err;
		schedule_work(&ipc->debug_work);
	}
}

static int contexthub_alive_check(struct contexthub_ipc_info *ipc)
{
	int trycnt = 0;

	do {
		msleep(WAIT_CHUB_MS);
		contexthub_ipc_write_event(ipc, MAILBOX_EVT_CHUB_ALIVE);
		if (++trycnt > WAIT_TRY_CNT)
			break;
	} while ((atomic_read(&ipc->chub_status) != CHUB_ST_RUN));

	if (atomic_read(&ipc->chub_status) == CHUB_ST_RUN) {
		nanohub_dev_info(ipc->dev, "%s done. contexthub status is %d\n",
				__func__, atomic_read(&ipc->chub_status));
		return 0;
	} else {
		nanohub_dev_warn(ipc->dev, "%s failed. contexthub status is %d\n",
				__func__, atomic_read(&ipc->chub_status));

		if (!atomic_read(&ipc->in_reset)) {
			atomic_set(&ipc->chub_status, CHUB_ST_NO_RESPONSE);
			contexthub_handle_debug(ipc, CHUB_ERR_CHUB_NO_RESPONSE);
		} else {
			nanohub_dev_info(ipc->dev, "%s: skip to handle debug in reset\n", __func__);
		}
		return -ETIMEDOUT;
	}
}

static void contexthub_select_os(struct contexthub_ipc_info *ipc)
{
	int ret;
	u8 val = (u8)ipc_read_hw_value(IPC_VAL_HW_DEBUG);
	if (!val) {
		nanohub_dev_warn(ipc->dev, "%s os number is invalid\n");
		val = 1;
	}
	ipc->sel_os = true;
	ipc->num_os = val;

	strcpy(ipc->os_name, os_image[val]);
	nanohub_dev_info(ipc->dev, "%s selected os_name = %s\n", __func__, ipc->os_name);

	if (IS_ENABLED(CONFIG_EXYNOS_IMGLOADER))
		ret = imgloader_boot(&ipc->chub_img_desc[val + 1]);
	else
		contexthub_download_image(ipc, IPC_REG_OS);

	ipc_write_hw_value(IPC_VAL_HW_BOOTMODE, ipc->os_load);
	ipc_write_hw_value(IPC_VAL_HW_DEBUG, READY_TO_GO);
	contexthub_alive_check(ipc);
	nanohub_dev_info(ipc->dev, "%s done: wakeup interrupt\n", __func__);
	chub_wake_event(&ipc->poweron_lock);
}

static DEFINE_MUTEX(chub_err_mutex);
static ssize_t chub_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static void handle_debug_work_func(struct work_struct *work)
{
	struct contexthub_ipc_info *ipc =
	    container_of(work, struct contexthub_ipc_info, debug_work);

	/* 1st work: os select on booting */
	if ((atomic_read(&ipc->chub_status) == CHUB_ST_POWER_ON) && (ipc->sel_os == false)) {
		contexthub_select_os(ipc);
		return;
	}

	mutex_lock(&chub_err_mutex);
	/* 2nd work: slient reset */
	if (ipc->cur_err) {
		int ret;
		int err = ipc->cur_err;

		ipc->cur_err = 0;
		nanohub_dev_info(ipc->dev, "%s: request silent reset. err:%d, status:%d, in-reset:%d\n",
			__func__, err, __raw_readl(&ipc->chub_status), __raw_readl(&ipc->in_reset));
		ret = chub_reset(ipc->dev, NULL, NULL, 0);
		if (ret) {
			nanohub_dev_warn(ipc->dev, "%s: fails to reset:%d. status:%d\n",
				__func__, ret, __raw_readl(&ipc->chub_status));
		} else {
			ipc->cur_err = 0;
			nanohub_dev_info(ipc->dev, "%s: chub reset! should be recovery\n",
				__func__);
		}
	}
	mutex_unlock(&chub_err_mutex);
}

static void handle_log_work_func(struct work_struct *work)
{
	struct contexthub_ipc_info *ipc =
	    container_of(work, struct contexthub_ipc_info, log_work);
	int retrycnt = 0;

retry:
	if (contexthub_get_token(ipc)) {
		chub_wait_event(&ipc->reset_lock, WAIT_TIMEOUT_MS * 2);
		if (!retrycnt) {
			retrycnt++;
			goto retry;
		}
		atomic_set(&ipc->log_work_active, 0);
		return;
	}
	ipc_logbuf_flush_on(1);
	mutex_lock(&log_mutex);
	if (ipc_logbuf_outprint(&ipc->chub_rt_log, 100))
		ipc->err_cnt[CHUB_ERR_NANOHUB_LOG]++;
	mutex_unlock(&log_mutex);
	ipc_logbuf_flush_on(0);
	contexthub_put_token(ipc);
	chub_wake_event(&ipc->reset_lock);
	atomic_set(&ipc->log_work_active, 0);
}

static inline void clear_err_cnt(struct contexthub_ipc_info *ipc, enum chub_err_type err)
{
	if (ipc->err_cnt[err])
		ipc->err_cnt[err] = 0;
}

int contexthub_ipc_read(struct contexthub_ipc_info *ipc, uint8_t *rx, int max_length,
				int timeout)
{
	unsigned long flag;
	int size = 0;
	int ret = 0;
	void *rxbuf;
	u64 time = 0; /* for debug */
	bool wait_event_err = false;
	bool restart_err = false;

	if (__raw_readl(&ipc->chub_status) != CHUB_ST_RUN) {
		nanohub_dev_warn(ipc->dev, "%s: chub isn't run:%d\n",
				__func__, __raw_readl(&ipc->chub_status));
		contexthub_handle_debug(ipc, CHUB_ERR_CHUB_ST_ERR);
		return 0;
	} else {
		clear_err_cnt(ipc, CHUB_ERR_CHUB_ST_ERR);
	}

	if (!atomic_read(&ipc->read_lock.cnt)) {
		time = sched_clock();

		spin_lock_irqsave(&ipc->read_lock.event.lock, flag);
		atomic_inc(&ipc->read_lock.flag);
		ret =
			wait_event_interruptible_timeout_locked(ipc->read_lock.event,
								atomic_read(&ipc->read_lock.cnt),
								msecs_to_jiffies(timeout));
		atomic_dec(&ipc->read_lock.flag);
		spin_unlock_irqrestore(&ipc->read_lock.event.lock, flag);
		if (ret < 0) {
			wait_event_err = true;
			if (ret == (-ERESTARTSYS))
				restart_err = true;
			nanohub_dev_warn(ipc->dev,
				 "fails to get read ret:%d timeout:%d, err:%d\n",
				 ret, timeout, restart_err);
		}
	}

	if (contexthub_get_token(ipc)) {
		nanohub_dev_warn(ipc->dev, "no-active: read fails\n");
		return 0;
	}

	if (atomic_read(&ipc->read_lock.cnt)) {
		rxbuf = cipc_read_data(CIPC_REG_DATA_CHUB2AP, &size);
		if (size > 0)
			ret = contexthub_read_process(rx, rxbuf, size);
		atomic_dec(&ipc->read_lock.cnt);
	} else {
		if (ipc->err_cnt[CHUB_ERR_READ_FAIL])
			nanohub_dev_info(ipc->dev, "%s: read timeout(%d): c2aq_cnt:%d, recv_cnt:%d during %lld ns\n",
				__func__, ipc->err_cnt[CHUB_ERR_READ_FAIL],
				cipc_get_remain_qcnt(CIPC_REG_DATA_CHUB2AP), atomic_read(&ipc->read_lock.cnt),
				sched_clock() - time);
		if (cipc_get_remain_qcnt(CIPC_REG_DATA_CHUB2AP)) {
			rxbuf = cipc_read_data(CIPC_REG_DATA_CHUB2AP, &size);
			if (size > 0)
				ret = contexthub_read_process(rx, rxbuf, size);
		} else {
			ret = -EINVAL;
		}
	}

	if (wait_event_err) /* add debug log for wait_event_err */
		nanohub_dev_info(ipc->dev, "%s: read in wait event err, ret:%d\n", __func__, ret);

	if (ret < 0) {
		if (ipc->err_cnt[CHUB_ERR_READ_FAIL]) {
			nanohub_err("%s: fails to read data: ret:%d, len:%d errcnt:%d, err:%d\n",
				__func__, ret, ipc->err_cnt[CHUB_ERR_READ_FAIL], restart_err);
			ipc_dump();
			if (restart_err == false)
				contexthub_handle_debug(ipc, CHUB_ERR_READ_FAIL);
		} else
			ipc->err_cnt[CHUB_ERR_READ_FAIL]++;
	} else {
		clear_err_cnt(ipc, CHUB_ERR_READ_FAIL);
	}
	contexthub_put_token(ipc);
	return ret;
}

int contexthub_ipc_write(struct contexthub_ipc_info *ipc,
				uint8_t *tx, int length, int timeout)
{
	int ret;

	if (__raw_readl(&ipc->chub_status) != CHUB_ST_RUN) {
		nanohub_dev_warn(ipc->dev, "%s: chub isn't run:%d\n",
				__func__, __raw_readl(&ipc->chub_status));
		contexthub_handle_debug(ipc, CHUB_ERR_CHUB_ST_ERR);
		return 0;
	}  else {
		clear_err_cnt(ipc, CHUB_ERR_CHUB_ST_ERR);
	}

	if (contexthub_get_token(ipc)) {
		nanohub_dev_warn(ipc->dev, "no-active: write fails\n");
		return 0;
	}

	mutex_lock(&wt_mutex);
	ret = cipc_write_data(CIPC_REG_DATA_AP2CHUB, tx, length);
	mutex_unlock(&wt_mutex);
	contexthub_put_token(ipc);
	if (ret) {
		nanohub_err("%s: fails to write data: ret:%d, len:%d errcnt:%d\n",
			__func__, ret, length, ipc->err_cnt[CHUB_ERR_WRITE_FAIL]);
		contexthub_handle_debug(ipc, CHUB_ERR_WRITE_FAIL);
		length = 0;
	} else {
		clear_err_cnt(ipc, CHUB_ERR_WRITE_FAIL);
	}
	return length;
}

static void check_rtc_time(void)
{
	struct rtc_device *chub_rtc = rtc_class_open(CONFIG_RTC_SYSTOHC_DEVICE);
	struct rtc_device *ap_rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	struct rtc_time chub_tm, ap_tm;
	time64_t chub_t, ap_t;

	rtc_read_time(ap_rtc, &chub_tm);
	rtc_read_time(chub_rtc, &ap_tm);

	chub_t = rtc_tm_sub(&chub_tm, &ap_tm);

	if (chub_t) {
		nanohub_info("nanohub %s: diff_time: %llu\n", __func__, chub_t);
		rtc_set_time(chub_rtc, &ap_tm);
	};

	chub_t = rtc_tm_to_time64(&chub_tm);
	ap_t = rtc_tm_to_time64(&ap_tm);
}

static int contexthub_hw_reset(struct contexthub_ipc_info *ipc,
				 enum mailbox_event event)
{
	int ret = 0;
	int i;
	bool first_poweron = false;

	nanohub_dev_info(ipc->dev, "%s. status:%d\n", __func__, __raw_readl(&ipc->chub_status));

	/* clear ipc value */
	atomic_set(&ipc->wakeup_chub, CHUB_OFF);
	atomic_set(&ipc->irq1_apInt, C2A_OFF);
	atomic_set(&ipc->read_lock.cnt, 0);
	atomic_set(&ipc->read_lock.flag, 0);
	atomic_set(&ipc->log_work_active, 0);

	/* chub err init */
	for (i = 0; i < CHUB_ERR_COMMS; i++)
		ipc->err_cnt[i] = 0;

	ipc_write_hw_value(IPC_VAL_HW_BOOTMODE, ipc->os_load);
	ipc_set_chub_clk((u32)ipc->clkrate);
	ipc->chub_rt_log.loglevel = CHUB_RT_LOG_DUMP_PRT;
	ipc_set_chub_bootmode(BOOTMODE_COLD, ipc->chub_rt_log.loglevel);

	switch (event) {
	case MAILBOX_EVT_POWER_ON:
#ifdef NEED_TO_RTC_SYNC
		check_rtc_time();
#endif
		if (atomic_read(&ipc->chub_status) == CHUB_ST_NO_POWER) {
			if (ipc->sel_os == false)
				first_poweron = true;

			atomic_set(&ipc->chub_status, CHUB_ST_POWER_ON);

			/* enable Dump gpr */
			IPC_HW_WRITE_DUMPGPR_CTRL(ipc->chub_dumpgpr, 0x1);

			ret = contexthub_soc_poweron(ipc);
		} else {
			ret = -EINVAL;
			nanohub_dev_warn(ipc->dev,
				 "fails to contexthub power on. Status is %d\n",
				 atomic_read(&ipc->chub_status));
		}
		break;
	case MAILBOX_EVT_RESET:
		ret = cal_chub_reset_release();
		break;
	default:
		break;
	}
	if (!IS_ENABLED(CONFIG_EXYNOS_IMGLOADER)) {
		i = exynos_request_fw_stage2_ap("CHUB");
		if (i)
			nanohub_dev_err(ipc->dev, "%s fw stage2 fail %d\n", __func__, i);
	}
	/* don't send alive with first poweron of multi-os */
	if (first_poweron) {
		nanohub_dev_info(ipc->dev, "%s -> os select\n", __func__);
		return 0;
	}
	if (ret)
		return ret;
	else {
		/* wait active */
		nanohub_dev_info(ipc->dev, "%s: alive check\n", __func__);
		return contexthub_alive_check(ipc);
	}
}

static void contexthub_set_baaw(struct contexthub_ipc_info *chub)
{
	/* BAAW-D-CHUB for CHUB to access CHIP ID. 1 window is used */
	if (chub->chub_baaw_d) {
		nanohub_info("%s BAAW_D_CHUB start 0x%lx end 0x%lx remap 0x%lx\n", __func__,
				chub->baaw_info.baaw_d_chub_start,
				chub->baaw_info.baaw_d_chub_end,
				chub->baaw_info.baaw_d_chub_remap);
		IPC_HW_WRITE_BAAW_CHUB0(chub->chub_baaw_d,
					chub->baaw_info.baaw_d_chub_start);
		IPC_HW_WRITE_BAAW_CHUB1(chub->chub_baaw_d,
					chub->baaw_info.baaw_d_chub_end);
		IPC_HW_WRITE_BAAW_CHUB2(chub->chub_baaw_d,
					chub->baaw_info.baaw_d_chub_remap);
		IPC_HW_WRITE_BAAW_CHUB3(chub->chub_baaw_d, BAAW_RW_ACCESS_ENABLE);
	}
}
#define os_name_idx (11)

int contexthub_get_sensortype(struct contexthub_ipc_info *ipc, char *buf)
{
	struct sensor_map *sensor_map;
	struct sensor_map_pack *pack = (struct sensor_map_pack *) buf;
	int trycnt = 0;
	int ret;
	int i;

	if (atomic_read(&ipc->chub_status) != CHUB_ST_RUN) {
		nanohub_dev_warn(ipc->dev, "%s :fails chub isn't active, status:%d, inreset:%d\n",
			__func__, atomic_read(&ipc->chub_status), contexthub_read_in_reset(ipc));
		contexthub_handle_debug(ipc, CHUB_ERR_CHUB_ST_ERR);
		return -EINVAL;
	}  else {
		clear_err_cnt(ipc, CHUB_ERR_CHUB_ST_ERR);
	}


	do {
		ret = contexthub_get_token(ipc);
		if (ret && (++trycnt < WAIT_TRY_CNT)) {
			nanohub_dev_warn(ipc->dev, "%s fails to get token: try:%d\n", __func__, trycnt);
			msleep(WAIT_CHUB_MS);
		}
	} while (ret);

	if (ret) {
		nanohub_dev_warn(ipc->dev, "%s fails to get token\n", __func__);
		return -EINVAL;
	}

	sensor_map = ipc_get_base(IPC_REG_IPC_SENSORINFO);
	if (ipc_have_sensor_info(sensor_map)) {
		pack->num_os = ipc->os_name[os_name_idx] - '0';
		memcpy(&pack->sensormap, sensor_map, sizeof(struct sensor_map));

		nanohub_dev_info(ipc->dev, "%s: get sensorinfo: (os:%d, size:%d, %d / %d %d), index:%d, map:%s\n",
			__func__, pack->num_os, ipc_get_size(IPC_REG_IPC_SENSORINFO), sizeof(struct sensor_map_pack),
			sizeof(pack->magic), sizeof(pack->num_os), sensor_map->index, (char *)&pack->sensormap);

		for (i = 0; i < MAX_PHYSENSOR_NUM; i++)
			nanohub_dev_info(ipc->dev, "%s2: %d/%d - %d: %s,%s\n",
			__func__, i, pack->sensormap.index,
			pack->sensormap.sinfo[i].sensortype,
			pack->sensormap.sinfo[i].name,
			pack->sensormap.sinfo[i].vendorname);
	}
	contexthub_put_token(ipc);

	return sizeof(struct sensor_map_pack);
}

void contexthub_ipc_status_reset(struct contexthub_ipc_info *ipc)
{
	/* clear ipc value */
	atomic_set(&ipc->wakeup_chub, CHUB_OFF);
	atomic_set(&ipc->irq1_apInt, C2A_OFF);
	atomic_set(&ipc->read_lock.cnt, 0x0);
	atomic_set(&ipc->log_work_active, 0);
}

int contexthub_reset_prepare(struct contexthub_ipc_info *ipc) {
	int ret;
	/* pmu call reset-release_config */
	ret = cal_chub_reset_release_config();
	if (ret) {
		nanohub_err("%s: reset release cfg fail\n", __func__);
		return ret;
	}
	/* tzpc setting */
	ret = exynos_smc(SMC_CMD_CONN_IF,
		(EXYNOS_CHUB << 32) |
		EXYNOS_SET_CONN_TZPC, 0, 0);
	if (ret) {
		nanohub_err("%s: TZPC setting fail\n",
			__func__);
		return -EINVAL;
	}
	nanohub_dev_info(ipc->dev, "%s: tzpc and baaw set\n", __func__);
	contexthub_set_baaw(ipc);

	/* baaw config */
	contexthub_chub_ipc_init(ipc);

	return ret;
}

int contexthub_ipc_write_event(struct contexthub_ipc_info *ipc,
				enum mailbox_event event)
{
	u32 val;
	int ret = 0;
	int need_ipc = 0;

	switch (event) {
	case MAILBOX_EVT_INIT_IPC:
		ret = contexthub_ipc_drv_init(ipc);
		break;
	case MAILBOX_EVT_POWER_ON:
		ret = contexthub_hw_reset(ipc, event);
		break;
	case MAILBOX_EVT_RESET:
		if (atomic_read(&ipc->chub_shutdown)) {
			ret = contexthub_hw_reset(ipc, event);
		} else {
			nanohub_dev_err(ipc->dev,
				"contexthub status isn't shutdown. fails to reset: %d, %d\n",
					atomic_read(&ipc->chub_shutdown),
					atomic_read(&ipc->chub_status));
			ret = -EINVAL;
		}
		break;
	case MAILBOX_EVT_SHUTDOWN:
		/* assert */
		if (ipc->block_reset) {
			/* pmu call assert */
			ret = cal_chub_reset_assert();
			if (ret) {
				nanohub_err("%s: reset assert fail\n", __func__);
				return ret;
			}

			/* pmu call reset-release_config */
			ret = contexthub_reset_prepare(ipc);
			if (ret) {
				nanohub_err("%s: reset prepare fail\n", __func__);
				return ret;
			}
		} else {
			ret = contexthub_core_reset(ipc);
		}
		atomic_set(&ipc->chub_shutdown, 1);
		atomic_set(&ipc->chub_status, CHUB_ST_SHUTDOWN);
		break;
	case MAILBOX_EVT_CHUB_ALIVE:
		pr_info("%s alive check", __func__);
		ipc_write_hw_value(IPC_VAL_HW_AP_STATUS, AP_WAKE);
		val = contexthub_lowlevel_alive(ipc);
		if (val) {
			atomic_set(&ipc->chub_status, CHUB_ST_RUN);
			nanohub_dev_info(ipc->dev, "%s : chub is alive\n", __func__);
		} else if (ipc->sel_os == true) {
			nanohub_dev_err(ipc->dev,
				"%s : chub isn't alive, status:%d, inreset:%d\n",
				__func__, atomic_read(&ipc->chub_status), contexthub_read_in_reset(ipc));
			ret = -EINVAL;
		}
		break;
	case MAILBOX_EVT_ENABLE_IRQ:
	case MAILBOX_EVT_DISABLE_IRQ:
		nanohub_dev_warn(ipc->dev, "%s not support: %d\n", event);
		ret = -EINVAL;
		break;
	default:
		need_ipc = 1;
		break;
	}

	if (need_ipc) {
		if (atomic_read(&ipc->chub_status) != CHUB_ST_RUN) {
			nanohub_dev_warn(ipc->dev, "%s event:%d/%d fails chub isn't active, status:%d, inreset:%d\n",
				__func__, event, MAILBOX_EVT_MAX, atomic_read(&ipc->chub_status), contexthub_read_in_reset(ipc));
			contexthub_handle_debug(ipc, CHUB_ERR_CHUB_ST_ERR);
			return -EINVAL;
		}  else {
			clear_err_cnt(ipc, CHUB_ERR_CHUB_ST_ERR);
		}

		if (contexthub_get_token(ipc))
			return -EINVAL;

		/* handle ipc */
		switch (event) {
		case MAILBOX_EVT_RT_LOGLEVEL:
			ipc_logbuf_loglevel(ipc->chub_rt_log.loglevel, 1);
			break;
		case MAILBOX_EVT_DUMP_STATUS:
			/* dump nanohub kernel status */
			nanohub_dev_info(ipc->dev, "Request to dump chub fw status\n");
			ipc_write_value(IPC_VAL_A2C_DEBUG, MAILBOX_EVT_DUMP_STATUS);
			ret = cipc_add_evt(CIPC_REG_EVT_AP2CHUB, IRQ_EVT_A2C_DEBUG);
			break;
		case MAILBOX_EVT_WAKEUP_CLR:
			if (atomic_read(&ipc->wakeup_chub) == CHUB_ON) {
				ret = cipc_add_evt(CIPC_REG_EVT_AP2CHUB, IRQ_EVT_A2C_WAKEUP_CLR);
				if (ret >= 0) {
					atomic_set(&ipc->wakeup_chub, CHUB_OFF);
					clear_err_cnt(ipc, CHUB_ERR_EVTQ_WAKEUP_CLR);
				} else {
					nanohub_dev_warn(ipc->dev, "%s: fails to set wakeup. ret:%d, err:%d",
						__func__, ret, ipc->err_cnt[CHUB_ERR_EVTQ_WAKEUP_CLR]);
					contexthub_handle_debug(ipc, CHUB_ERR_EVTQ_WAKEUP_CLR);
				}
			}
			break;
		case MAILBOX_EVT_WAKEUP:
			if (atomic_read(&ipc->wakeup_chub) == CHUB_OFF) {
				ret = cipc_add_evt(CIPC_REG_EVT_AP2CHUB, IRQ_EVT_A2C_WAKEUP);
				if (ret >= 0) {
					atomic_set(&ipc->wakeup_chub, CHUB_ON);
					clear_err_cnt(ipc, CHUB_ERR_EVTQ_WAKEUP);
				} else {
					nanohub_dev_warn(ipc->dev, "%s: fails to set wakeupclr. ret:%d, err:%d",
						__func__, ret, ipc->err_cnt[CHUB_ERR_EVTQ_WAKEUP]);
					contexthub_handle_debug(ipc, CHUB_ERR_EVTQ_WAKEUP);
				}
			}
			break;
		case MAILBOX_EVT_DFS_GOVERNOR:
			ipc_write_value(IPC_VAL_A2C_DEBUG, (u32)event);
			ret = cipc_add_evt(CIPC_REG_EVT_AP2CHUB, IRQ_EVT_A2C_DEBUG);
			break;
		default:
			/* handle ipc utc */
			if ((int)event < IPC_DEBUG_UTC_MAX) {
				ipc->utc_run = event;
				if ((int)event == IPC_DEBUG_UTC_TIME_SYNC)
					check_rtc_time();
				ipc_write_value(IPC_VAL_A2C_DEBUG, (u32)event);
				ret = cipc_add_evt(CIPC_REG_EVT_AP2CHUB, IRQ_EVT_A2C_DEBUG);
			}
			break;
		}
		contexthub_put_token(ipc);

		if (ret < 0) {
			contexthub_handle_debug(ipc, CHUB_ERR_EVTQ_ADD);
			nanohub_dev_warn(ipc->dev, "%s: errcnt: evtq:%d, wake:%d, clr:%d\n",
				__func__, ipc->err_cnt[CHUB_ERR_EVTQ_ADD],
				ipc->err_cnt[CHUB_ERR_EVTQ_WAKEUP],
				ipc->err_cnt[CHUB_ERR_EVTQ_WAKEUP_CLR]);
		} else
			clear_err_cnt(ipc, CHUB_ERR_EVTQ_ADD);
	}
	return ret;
}

int contexthub_poweron(struct contexthub_ipc_info *ipc)
{
	int ret = 0;
	struct device *dev = ipc->dev;

	if (!atomic_read(&ipc->chub_status)) {
		memset_io(ipc->sram, 0, ipc->sram_size);
		if (IS_ENABLED(CONFIG_EXYNOS_IMGLOADER))
			ret = imgloader_boot(&ipc->chub_img_desc[0]);
		else
			ret = contexthub_download_image(ipc, IPC_REG_BL);

		if (ret) {
			nanohub_dev_warn(dev, "fails to download bootloader\n");
			return ret;
		}

		ret = contexthub_ipc_write_event(ipc, MAILBOX_EVT_INIT_IPC);
		if (ret) {
			dev_warn(dev, "fails to init ipc\n");
			return ret;
		}

		if (!strcmp(ipc->os_name, os_image[0]) || ipc->os_name[0] != 'o') {
			nanohub_dev_info(dev, "OS name not saved: %s\n", ipc->os_name);
			ipc->sel_os = false;
		} else {
			nanohub_dev_info(dev, "OS name saved: %s\n", ipc->os_name);
			ipc->sel_os = true;
		}

		if (IS_ENABLED(CONFIG_EXYNOS_IMGLOADER))
			imgloader_boot(&ipc->chub_img_desc[ipc->num_os + 1]);
		else
			ret = contexthub_download_image(ipc, IPC_REG_OS);

		if (ret) {
			nanohub_dev_warn(dev, "fails to download kernel\n");
			return ret;
		}

		if (ipc->chub_dfs_gov) {
			nanohub_dev_info(dev, "%s: set dfs gov:%d\n", __func__, ipc->chub_dfs_gov);
			ipc_set_dfs_gov(ipc->chub_dfs_gov);
		}
		nanohub_dev_info(dev, "%s: get dfs gov:%d\n", __func__, ipc_get_dfs_gov());

		ret = contexthub_ipc_write_event(ipc, MAILBOX_EVT_POWER_ON);
		if (ret) {
			dev_warn(dev, "fails to poweron\n");
			return ret;
		}
		if (atomic_read(&ipc->chub_status) == CHUB_ST_RUN) {
			 /* chub driver directly gets alive event without multi-os */
			nanohub_dev_info(dev, "%s: contexthub power-on\n", __func__);
		} else {
			if (ipc->sel_os)  /* without multi-os */
				nanohub_dev_warn(dev, "contexthub failed to power-on\n");
			else {  /* with multi-os */
				nanohub_dev_info(dev, "%s: wait for multi-os poweron\n", __func__);
				ret = chub_wait_event(&ipc->poweron_lock, WAIT_TIMEOUT_MS * 2);
				nanohub_dev_info(dev, "%s: multi-os poweron %s, status:%d, ret:%d, flag:%d\n", __func__,
					atomic_read(&ipc->chub_status) == CHUB_ST_RUN ? "success" : "fails",
					atomic_read(&ipc->chub_status), ret, ipc->poweron_lock.flag);
			}
		}
	} else
	/* CHUB already went through poweron sequence */
		return -EINVAL;

	if (ipc->sysevent_dev && sysevent_get("CHB") == PTR_ERR)
		nanohub_warn("%s sysevent_get fail!", __func__);

	contexthub_notifier_call(ipc, CHUB_FW_ST_POWERON);
#ifdef IPC_DEBUG
	ipc_test(chub);
#endif

	nanohub_info("%s done!\n", __func__);

	return 0;
}

static int contexthub_download_and_check_image(struct contexthub_ipc_info *ipc, enum ipc_region reg)
{
	u32 *fw = vmalloc(ipc_get_size(reg));
	int ret = 0;

	if (!fw)
		return contexthub_download_image(ipc, reg);

	memcpy_fromio(fw, ipc_get_base(reg), ipc_get_size(reg));
	ret = contexthub_download_image(ipc, reg);
	if (ret) {
		nanohub_dev_err(ipc->dev, "%s: download bl(%d) fails\n", __func__, reg == IPC_REG_BL);
		goto out;
	}

	ret = memcmp(fw, ipc_get_base(reg), ipc_get_size(reg));
	if (ret) {
		int i;
		u32 *fw_image = (u32 *)ipc_get_base(reg);

		nanohub_dev_err(ipc->dev, "%s: fw(%lx) doens't match with size %d\n",
			__func__, (unsigned long)ipc_get_base(reg), ipc_get_size(reg));
		for (i = 0; i < ipc_get_size(reg) / 4; i++)
			if (fw[i] != fw_image[i]) {
				nanohub_dev_err(ipc->dev, "fw[%d] %x -> wrong %x\n", i, fw_image[i], fw[i]);
				print_hex_dump(KERN_CONT, "before:", DUMP_PREFIX_OFFSET, 16, 1, &fw[i], 64, false);
				print_hex_dump(KERN_CONT, "after:", DUMP_PREFIX_OFFSET, 16, 1, &fw_image[i], 64, false);
				ret = -EINVAL;
				break;
			}
	}
out:
	nanohub_dev_info(ipc->dev, "%s: download and checked bl(%d) ret:%d \n", __func__, reg == IPC_REG_BL, ret);
	vfree(fw);
	return ret;
}

#define CHUB_RESET_WAIT_TIME_MS (300)
int contexthub_reset(struct contexthub_ipc_info *ipc, bool force_load, enum chub_err_type err)
{
	int ret = 0;
	int trycnt = 0;
	bool irq_disabled;

	if (ipc->sysevent_dev)
		sysevent_put((void *) ipc->sysevent_dev);

	/* debug dump */
	memlog_do_dump(memlog_sram_chub, MEMLOG_LEVEL_EMERG);
	chub_dbg_dump_hw(ipc, err);

	mutex_lock(&reset_mutex);
	nanohub_dev_info(ipc->dev, "%s: force:%d, status:%d, in-reset:%d, err:%d, user:%d\n",
		__func__, force_load, atomic_read(&ipc->chub_status),
		contexthub_read_in_reset(ipc), err, contexthub_read_token(ipc));
	if (!force_load && (atomic_read(&ipc->chub_status) == CHUB_ST_RUN)) {
		mutex_unlock(&reset_mutex);
		nanohub_dev_info(ipc->dev, "%s: out status:%d\n", __func__, atomic_read(&ipc->chub_status));
		return 0;
	}

	/* disable mailbox interrupt to prevent sram access during chub reset */
	disable_irq(ipc->irq_mailbox);
	irq_disabled = true;

	atomic_inc(&ipc->in_reset);
	chub_wake_lock(ipc->ws_reset);
	contexthub_notifier_call(ipc, CHUB_FW_ST_OFF);

	/* wait for ipc free */
	while (atomic_read(&ipc->in_use_ipc)) {
		chub_wait_event(&ipc->reset_lock, CHUB_RESET_WAIT_TIME_MS);
		if (++trycnt > RESET_WAIT_TRY_CNT) {
			nanohub_dev_info(ipc->dev, "%s: can't get lock. in_use_ipc: %d\n",
				__func__, contexthub_read_token(ipc));
			ret = -EINVAL;
			goto out;
		}
		nanohub_dev_info(ipc->dev, "%s: wait for ipc user free: %d\n",
			__func__, contexthub_read_token(ipc));
	}
	if (!trycnt)
		msleep(CHUB_RESET_WAIT_TIME_MS); /* wait for subsystem release */

	contexthub_set_in_reset(ipc, true);

	nanohub_dev_info(ipc->dev, "%s: start reset status:%d\n", __func__, atomic_read(&ipc->chub_status));

	if (!ipc->block_reset) {
		/* core reset */
		cipc_add_evt(CIPC_REG_EVT_AP2CHUB, IRQ_EVT_A2C_SHUTDOWN);
		msleep(100);	/* wait for shut down time */
	}

	/* shutdown */
	mutex_lock(&pmu_shutdown_mutex);
	atomic_set(&ipc->chub_shutdown, 0);
	nanohub_dev_info(ipc->dev, "%s: shutdown\n", __func__);
	ret = contexthub_ipc_write_event(ipc, MAILBOX_EVT_SHUTDOWN);
	if (ret) {
		nanohub_dev_err(ipc->dev, "%s: shutdown fails, ret:%d\n", __func__, ret);
		mutex_unlock(&pmu_shutdown_mutex);
		goto out;
	}
	/* deassert and config */
	nanohub_dev_info(ipc->dev, "%s: out shutdown\n", __func__);
	mutex_unlock(&pmu_shutdown_mutex);

	/* image download */
	if (ipc->block_reset || force_load) {
		if (IS_ENABLED(CONFIG_EXYNOS_IMGLOADER))
			ret = imgloader_boot(&ipc->chub_img_desc[0]);
		else
			ret = contexthub_download_image(ipc, IPC_REG_BL);
		if (ret) {
			nanohub_dev_err(ipc->dev, "%s: download bl fails\n", __func__);
			ret = -EINVAL;
			goto out;
		}

		if (force_load) {/* can use new binary */
			if (IS_ENABLED(CONFIG_EXYNOS_IMGLOADER))
				ret = imgloader_boot(&ipc->chub_img_desc[ipc->num_os + 1]);
			else
				ret = contexthub_download_image(ipc, IPC_REG_OS);
		} else /* use previous binary */
			ret = contexthub_download_and_check_image(ipc, IPC_REG_OS);
		if (ret) {
			nanohub_dev_err(ipc->dev, "%s: download os fails\n", __func__);
			ret = -EINVAL;
			goto out;
		}
	}

	/* enable mailbox interrupt to get 'alive' event */
	enable_irq(ipc->irq_mailbox);
	irq_disabled = false;

	/* reset release */
	ret = contexthub_ipc_write_event(ipc, MAILBOX_EVT_RESET);
	if (ret) {
	        nanohub_dev_err(ipc->dev, "%s: chub reset fail! (ret:%d)\n", __func__, ret);
	} else {
	        nanohub_dev_info(ipc->dev, "%s: chub reset done! (cnt:%d)\n",
	                __func__, ipc->err_cnt[CHUB_ERR_RESET_CNT]);
	        ipc->err_cnt[CHUB_ERR_RESET_CNT]++;
			contexthub_reset_token(ipc);
	}
out:
	if (ret) {
		atomic_set(&ipc->chub_status, CHUB_ST_RESET_FAIL);
		if (irq_disabled)
			enable_irq(ipc->irq_mailbox);
		nanohub_dev_err(ipc->dev, "%s: chub reset fail! should retry to reset (ret:%d), irq_disabled:%d\n",
			__func__, ret, irq_disabled);
	}
	msleep(100); /* wakeup delay */
	chub_wake_event(&ipc->reset_lock);
	contexthub_set_in_reset(ipc, false);
	contexthub_notifier_call(ipc, CHUB_FW_ST_ON);
	enable_irq(ipc->irq_wdt);

	mutex_unlock(&reset_mutex);

	if (ipc->sysevent_dev && sysevent_get("CHB") == PTR_ERR)
		nanohub_warn("%s sysevent_get fail!", __func__);

	if (memlog_printf_chub)
		memlog_sync_to_file(memlog_printf_chub);

	return ret;
}

static int contexthub_verify_symtable(struct contexthub_ipc_info *chub, void *data, size_t size)
{
	u32 magic, symbol_size, offset = 0;

	if (!chub || !chub->sel_os || chub->symbol_table != NULL)
		return -1;

	magic = *(u32 *)(data + size - 4);
	if (magic != 0x626d7973) {
		offset = 528;
		magic = *(u32 *)(data + size - offset - 4);
	}

	nanohub_dev_info(chub->dev, "read magic : %x\n", magic);
	if (magic == 0x626d7973) {
		symbol_size = *(u32 *)(data + size - offset - 8);
		nanohub_dev_info(chub->dev, "symbol table size : %d\n", symbol_size);

		if (symbol_size < size - offset) {
			chub->symbol_table = (struct contexthub_symbol_table *) kmalloc(symbol_size, GFP_KERNEL);
			if (chub->symbol_table) {
				memcpy(chub->symbol_table, data + size - symbol_size - offset - 8, symbol_size);
				nanohub_dev_info(chub->dev, "Load symbol table Done!!!\n");
				}
		}
	} else {
		chub->symbol_table = NULL;
	}

	return 0;
}

int contexthub_download_image(struct contexthub_ipc_info *ipc, enum ipc_region reg)
{
	const struct firmware *entry;
	int ret = 0;
	void *chub_addr = NULL;

	if (reg == IPC_REG_BL) {
		nanohub_dev_info(ipc->dev, "%s: download bl\n", __func__);
		ret = request_firmware(&entry, "bl.unchecked.bin", ipc->dev);
		chub_addr = ipc->sram;
	}
	else if (reg == IPC_REG_OS) {
		nanohub_dev_info(ipc->dev, "%s: download %s\n", __func__, ipc->os_name);
		ret = request_firmware(&entry, ipc->os_name, ipc->dev);
		chub_addr = ipc_get_base(reg);
	}
	else {
		nanohub_dev_err(ipc->dev, "%s: invalid reg:%d\n", __func__, reg);
		return -EINVAL;
	}

	if (ret) {
		nanohub_dev_err(ipc->dev, "%s, bl(%d) request_firmware failed\n",
			reg == IPC_REG_BL, __func__);
		release_firmware(entry);
		return ret;
	}
	memcpy_toio(chub_addr, entry->data, entry->size);
	nanohub_dev_info(ipc->dev, "%s: bl:%d, bin(size:%d) on %lx\n",
		 __func__, reg == IPC_REG_BL, (int)entry->size, (unsigned long)ipc_get_base(reg));

	if (reg == IPC_REG_BL)
		ret = exynos_verify_subsystem_fw("CHUB", 0,
				PHYS_SRAM_BASE, entry->size, SZ_4K);
	else if (reg == IPC_REG_OS)
		ret = exynos_verify_subsystem_fw("CHUB", 1,
				PHYS_SRAM_BASE + SZ_4K,
				entry->size, SZ_1M - SZ_4K);
	else
		nanohub_dev_err(ipc->dev, "%s: invalid reg:%d\n", __func__, reg);

	if (ret) {
		nanohub_dev_err(ipc->dev, "%s: verify fail!:%d\n", __func__, ret);
		return -1;
	}

	if ( reg == IPC_REG_OS) {
		contexthub_verify_symtable(ipc, (void *) entry->data, entry->size);
	}

	release_firmware(entry);

	return 0;
}

static irqreturn_t contexthub_irq_handler(int irq, void *data)
{
	struct contexthub_ipc_info *ipc = data;
	struct cipc_info *cipc;
	enum ipc_mb_id rcv_mb_id;
	unsigned int status;
	int start_index;
	int irq_num;
	int ret;

	cipc = ipc->chub_ipc->cipc;
	rcv_mb_id = cipc->user_info[CIPC_USER_CHUB2AP].map_info.dst.mb_id;
	status = ipc_hw_read_int_status_reg_all(ipc->mailbox, rcv_mb_id);
	start_index = ipc_hw_start_bit(rcv_mb_id);
	irq_num = IRQ_NUM_CHUB_ALIVE + start_index;

	if(atomic_read(&ipc->chub_sleep)) {
		nanohub_dev_info(ipc->dev,
			"%s wakeup_by_me: status:0x%x\n", __func__, status);
		ipc_dump();
		ipc->wakeup_by_chub_cnt++;
	}

	/* chub alive interrupt handle */
	if (status & (1 << irq_num)) {
		status &= ~(1 << irq_num);
		ipc_hw_clear_int_pend_reg(ipc->mailbox, rcv_mb_id, irq_num);
		if (atomic_read(&ipc->chub_status) == CHUB_ST_POWER_ON && ipc->sel_os == false) {
			schedule_work(&ipc->debug_work);
			return IRQ_HANDLED;
		}

		if (ipc_read_hw_value(IPC_VAL_HW_AP_STATUS) == CHUB_REBOOT_REQ) {
			nanohub_dev_err(ipc->dev," chub sends to request reboot\n");
			contexthub_handle_debug(ipc, CHUB_ERR_FW_REBOOT);
		} else {
			/* set wakeup flag for chub_alive_lock */
			chub_wake_event(&ipc->chub_alive_lock);
		}
	}

	if (contexthub_get_token(ipc)) {
		nanohub_dev_err(ipc->dev, "%s: in reset irq_status:%d\n", __func__, status);
		ipc_hw_clear_all_int_pend_reg(ipc->mailbox, rcv_mb_id);
		return IRQ_HANDLED;
	}

	irq_num = IRQ_NUM_CHUB_LOG + start_index;
	if (status & (1 << irq_num)) {
		status &= ~(1 << irq_num);
		ipc_hw_clear_int_pend_reg(ipc->mailbox, rcv_mb_id, irq_num);
		cipc_func_handle_irq(IRQ_EVT_C2A_LOG, ipc);
	}

	if (status) {
		ret = cipc_irqhandler(CIPC_USER_CHUB2AP, status);
		if (ret)
			contexthub_handle_debug(ipc, CHUB_ERR_ISR);
	}
	contexthub_put_token(ipc);
	return IRQ_HANDLED;
}

static irqreturn_t contexthub_irq_wdt_handler(int irq, void *data)
{
	struct contexthub_ipc_info *ipc = data;

	nanohub_dev_info(ipc->dev, "%s called\n", __func__);
	disable_irq_nosync(ipc->irq_wdt);
	ipc->irq_wdt_disabled = 1;
	contexthub_handle_debug(ipc, CHUB_ERR_FW_WDT);

	return IRQ_HANDLED;
}

struct clk *contexthub_devm_clk_prepare(struct device *dev, const char *name)
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

static void __iomem *get_iomem(struct platform_device *pdev,
                const char *name, u32 *size)
{
	struct resource *res;
	void __iomem *ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (IS_ERR_OR_NULL(res)) {
		nanohub_dev_err(&pdev->dev, "Failed to get %s\n", name);
		return ERR_PTR(-EINVAL);
	}

	ret = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ret)) {
		nanohub_dev_err(&pdev->dev, "fails to get %s\n", name);
		return ERR_PTR(-EINVAL);
	}

	if (size)
		*size = resource_size(res);
	nanohub_dev_info(&pdev->dev, "%s: %s is mapped on with size of %zu\n",
		__func__, name, (size_t)resource_size(res));

	return ret;
}

static int contexthub_ipc_hw_init(struct platform_device *pdev,
					 struct contexthub_ipc_info *chub)
{
	int ret;
	const char *os;
	const char *resetmode;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	/*chub_usi_array*/
	int reg_cnt;
	const char *reg_names[50];
	int j;

	if (!node) {
		nanohub_dev_err(dev, "driver doesn't support non-dt\n");
		return -ENODEV;
	}

	/* get os type from dt */
	os = of_get_property(node, "os-type", NULL);
	if (!os || !strcmp(os, "none") || !strcmp(os, "pass")) {
		nanohub_dev_err(dev, "no use contexthub\n");
		chub->os_load = 0;
		return -ENODEV;
	} else {
		chub->os_load = 1;
		strcpy(chub->os_name, os);
	}

	/* get resetmode from dt */
	resetmode = of_get_property(node, "reset-mode", NULL);
	if (!resetmode || !strcmp(resetmode, "block"))
		chub->block_reset = 1;
	else
		chub->block_reset = 0;

	/* get mailbox interrupt */
	chub->irq_mailbox = irq_of_parse_and_map(node, 0);
	if (chub->irq_mailbox < 0) {
		nanohub_dev_err(dev, "failed to get irq:%d\n", chub->irq_mailbox);
		return -EINVAL;
	}
	nanohub_dev_info(dev, "%s: chub irq mailbox %d\n", __func__, chub->irq_mailbox);

	/* request irq handler */
	ret = devm_request_irq(dev, chub->irq_mailbox, contexthub_irq_handler,
			       IRQ_TYPE_LEVEL_HIGH, dev_name(dev), chub);
	if (ret) {
		nanohub_dev_err(dev, "failed to request irq:%d, ret:%d\n",
			chub->irq_mailbox, ret);
		return ret;
	}

	/* get wdt interrupt optionally */
	chub->irq_wdt = irq_of_parse_and_map(node, 1);
	if (chub->irq_wdt > 0) {
		/* request irq handler */
		ret = devm_request_irq(dev, chub->irq_wdt, contexthub_irq_wdt_handler,
					IRQ_TYPE_LEVEL_HIGH, dev_name(dev), chub);
		if (ret) {
			nanohub_dev_err(dev, "failed to request wdt irq:%d, ret:%d\n",
				chub->irq_wdt, ret);
			return ret;
		}
		chub->irq_wdt_disabled = 0;
	} else {
		nanohub_dev_info(dev, "don't use wdt irq:%d\n", chub->irq_wdt);
	}
	nanohub_dev_info(dev, "%s: chub irq wdt %d\n", __func__, chub->irq_wdt);

	/* get MAILBOX SFR */
	chub->mailbox = get_iomem(pdev, "mailbox", NULL);
	if (IS_ERR(chub->mailbox))
		return PTR_ERR(chub->mailbox);

	/* get SRAM base */
	chub->sram = get_iomem(pdev, "sram", &chub->sram_size);
	if (IS_ERR(chub->sram))
		return PTR_ERR(chub->sram);

	/* get chub gpr base */
	chub->chub_dumpgpr = get_iomem(pdev, "dumpgpr", NULL);
	if (IS_ERR(chub->chub_dumpgpr))
		return PTR_ERR(chub->chub_dumpgpr);

	chub->chub_baaw_d = get_iomem(pdev, "chub_baaw_d", NULL);
	if (IS_ERR(chub->chub_baaw_d))
		return PTR_ERR(chub->chub_baaw_d);

	contexthub_get_qch_base(chub);

	/* get chub cmu base */
	chub->chub_dump_cmu = get_iomem(pdev, "chub_dump_cmu", NULL);
	if (IS_ERR(chub->chub_dump_cmu))
		return PTR_ERR(chub->chub_dump_cmu);

	/* get chub sys base */
	chub->chub_dump_sys = get_iomem(pdev, "chub_dump_sys", NULL);
	if (IS_ERR(chub->chub_dump_sys))
		return PTR_ERR(chub->chub_dump_sys);

	/* get chub wdt base */
	chub->chub_dump_wdt = get_iomem(pdev, "chub_dump_wdt", NULL);
	if (IS_ERR(chub->chub_dump_wdt))
		return PTR_ERR(chub->chub_dump_wdt);

	/* get chub timer base */
	chub->chub_dump_timer = get_iomem(pdev, "chub_dump_timer", NULL);
	if (IS_ERR(chub->chub_dump_timer))
		return PTR_ERR(chub->chub_dump_timer);

	/* get chub pwm base */
	chub->chub_dump_pwm = get_iomem(pdev, "chub_dump_pwm", NULL);
	if (IS_ERR(chub->chub_dump_pwm))
		return PTR_ERR(chub->chub_dump_pwm);

	/* get chub rtc base */
	chub->chub_dump_rtc = get_iomem(pdev, "chub_dump_rtc", NULL);
	if (IS_ERR(chub->chub_dump_rtc))
		return PTR_ERR(chub->chub_dump_rtc);

	/*get usi_array*/
	chub->usi_cnt = 0;
	reg_cnt = of_property_count_strings(node, "reg-names");
	of_property_read_string_array(node, "reg-names", reg_names, reg_cnt);
#ifdef	CONFIG_CONTEXTHUB_DEBUG
	for (j = 0; j < reg_cnt; j++) {
		if (strstr(reg_names[j], "usi"))
			nanohub_dev_info(&pdev->dev,
				 "%s: usi reg name %s\n",
				 __func__, reg_names[j]);
	}
#endif
	for (j = 0; j < reg_cnt; j++) {
		if (strstr(reg_names[j], "usi")) {
			if (chub->usi_cnt < MAX_USI_CNT) {
				chub->usi_array[chub->usi_cnt]
				    = get_iomem(pdev, reg_names[j], NULL);
				if (IS_ERR(chub->usi_array[chub->usi_cnt])) {
					nanohub_dev_err(&pdev->dev,
						"driver failed to get memorry %s\n",
						reg_names[j]);
					return PTR_ERR(
					    chub->usi_array[chub->usi_cnt]);
				}
				chub->usi_cnt++;
			} else {
				nanohub_dev_err(&pdev->dev,
					"usi regs more than MAX Seeting Value(15), please know that!\n");
			}
		}
	}
	/* get addresses information to set BAAW */
	if (of_property_read_u32_index
		(node, "baaw,baaw-p-apm-chub", 0,
		 &chub->baaw_info.baaw_p_apm_chub_start)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-p-apm-chub, start\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-p-apm-chub", 1,
		 &chub->baaw_info.baaw_p_apm_chub_end)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-p-apm-chub, end\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-p-apm-chub", 2,
		 &chub->baaw_info.baaw_p_apm_chub_remap)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-p-apm-chub, remap\n");
		return -ENODEV;
	}

	/* get addresses information to set BAAW */
	if (of_property_read_u32_index
		(node, "baaw,baaw-p-cmgp-chub", 0,
		 &chub->baaw_info.baaw_p_cmgp_chub_start)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-p-apm-chub, start\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-p-cmgp-chub", 1,
		 &chub->baaw_info.baaw_p_cmgp_chub_end)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-p-apm-chub, end\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-p-cmgp-chub", 2,
		 &chub->baaw_info.baaw_p_cmgp_chub_remap)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-p-apm-chub, remap\n");
		return -ENODEV;
	}

	/* get addresses information to set BAAW D */
	if (of_property_read_u32_index
		(node, "baaw,baaw-d-chub", 0,
		 &chub->baaw_info.baaw_d_chub_start)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-d-chub, start\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-d-chub", 1,
		 &chub->baaw_info.baaw_d_chub_end)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-d-chub, end\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-d-chub", 2,
		 &chub->baaw_info.baaw_d_chub_remap)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-d-chub, remap\n");
		return -ENODEV;
	}

	contexthub_disable_pin(chub);
	contexthub_set_clk(chub);
	contexthub_get_clock_names(chub);
	return 0;
}

static ssize_t chub_poweron(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);
	int ret = contexthub_poweron(ipc);

	return ret < 0 ? ret : count;
}

static ssize_t chub_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);
	int ret;

#ifdef CONFIG_CHRE_SENSORHUB_HAL
	ret = nanohub_hw_reset(ipc->data);
#else
	ret = contexthub_reset(ipc, 1, CHUB_ERR_NONE);
#endif

	return ret < 0 ? ret : count;
}

static struct device_attribute attributes[] = {
	__ATTR(poweron, 0220, NULL, chub_poweron),
	__ATTR(reset, 0220, NULL, chub_reset),
};

#ifdef CONFIG_EXYNOS_ITMON
static int chub_itmon_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	struct contexthub_ipc_info *data = container_of(nb, struct contexthub_ipc_info, itmon_nb);
	struct itmon_notifier *itmon_data = nb_data;

	if (itmon_data && itmon_data->master &&
		((!strncmp("CM4_SHUB_CD", itmon_data->master, sizeof("CM4_SHUB_CD") - 1)) ||
		(!strncmp("CM4_SHUB_P", itmon_data->master, sizeof("CM4_SHUB_P") - 1)) ||
		(!strncmp("PDMA_SHUB", itmon_data->master, sizeof("PDMA_SHUB") - 1)) ||
		(!strncmp("CM4_CHUB", itmon_data->master, sizeof("CM4_CHUB") - 1)) ||
		(!strncmp("PDMA_CHUB", itmon_data->master, sizeof("PDMA_CHUB") - 1)) ||
		(!strncmp("CHUB", itmon_data->master, sizeof("CHUB") - 1))
		)) {
		nanohub_dev_info(data->dev, "%s: chub(%s) itmon detected: action:%d!!\n",
			__func__, itmon_data->master, action);
		contexthub_handle_debug(data, CHUB_ERR_ITMON);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}
#endif

static int contexthub_panic_handler(struct notifier_block *nb,
                               unsigned long action, void *data)
{
	memlog_do_dump(memlog_sram_chub, MEMLOG_LEVEL_EMERG);
	chub_dbg_dump_ram(CHUB_ERR_KERNEL_PANIC);
	return NOTIFY_OK;
}

static struct notifier_block chub_panic_notifier = {
        .notifier_call  = contexthub_panic_handler,
        .next           = NULL,
        .priority       = 0     /* priority: INT_MAX >= x >= 0 */
};

int contexthub_sysevent_shutdown(const struct sysevent_desc *desc, bool force_stop)
{
	(void) desc;
	nanohub_info("%s [%d]\n", __func__, force_stop);

	return 0;
}

int contexthub_sysevent_powerup(const struct sysevent_desc *desc)
{
	(void) desc;
	nanohub_info("%s\n", __func__);
	return 0;
}

int contexthub_sysevent_ramdump(int tmp, const struct sysevent_desc *desc)
{
	(void) desc;
	(void) tmp;
	nanohub_info("%s\n", __func__);
	return 0;
}

void contexthub_sysevent_crash_shutdown(const struct sysevent_desc *desc)
{
	(void) desc;
	nanohub_info("%s\n", __func__);
}

static int contexthub_memlog_init(struct device *dev)
{
	int ret;

	ret = memlog_register("CHB", dev, &memlog_chub);
	if (!memlog_chub) {
		nanohub_dev_err(dev, "memlog chub desc registration fail");
		return -1;
	}

	memlog_printf_file_chub = memlog_alloc_file(memlog_chub, "log-fil", SZ_512K, SZ_1M, 1000, 3);
	dev_info(dev, "memlog printf file chub %s\n",
				memlog_printf_file_chub ? "pass" : "fail");
	if (memlog_printf_file_chub) {
		memlog_obj_set_sysfs_mode(memlog_printf_file_chub, true);
		memlog_printf_chub = memlog_alloc_printf(memlog_chub, SZ_512K,
							memlog_printf_file_chub, "log-mem", 0);
	}
	dev_info(dev, "memlog printf chub %s\n",
				memlog_printf_chub ? "pass" : "fail");

	memlog_sram_file_chub = memlog_alloc_file(memlog_chub, "srm-fil", SZ_1M, SZ_1M, 1000, 3);
	if (memlog_sram_file_chub)
		memlog_sram_chub = memlog_alloc_dump(memlog_chub, SZ_1M,
					PHYS_SRAM_BASE, false,
					memlog_sram_file_chub, "srm-dmp");


	return 0;
}

static int contexthub_s2mpu_notifier(struct s2mpufd_notifier_block *nb, struct s2mpufd_notifier_info *nb_data)
{
	struct contexthub_ipc_info *data = container_of(nb, struct contexthub_ipc_info, s2mpu_nb);
	struct s2mpufd_notifier_info *s2mpu_data = nb_data;

	(void) data;
	(void) s2mpu_data;

	nanohub_info("%s called!\n", __func__);

	contexthub_handle_debug(data, CHUB_ERR_S2MPU);
	return S2MPUFD_NOTIFY_OK;
}


#define MAX_FIRMWARE_NUM 3
int contexthub_imgloader_mem_setup(struct imgloader_desc *desc, const u8 *metadata, size_t size,
		                phys_addr_t *fw_phys_base, size_t *fw_bin_size, size_t *fw_mem_size)
{
	void *addr = NULL;
	struct contexthub_ipc_info *chub = (struct contexthub_ipc_info *) desc->data;

	if (desc->fw_id == 0) {
		*fw_phys_base = PHYS_SRAM_BASE;
		*fw_bin_size = size;
		*fw_mem_size = SZ_4K;
		addr = chub->sram;
	} else {
		*fw_phys_base = PHYS_SRAM_BASE + SZ_4K;
		*fw_bin_size = size;
		*fw_mem_size = SZ_1M - SZ_4K;
		addr = ipc_get_base(IPC_REG_OS);
	}

	memcpy_toio(addr, metadata, size);
	nanohub_info("%s chub image %s loaded by imgloader\n", __func__, desc->fw_name);

	contexthub_verify_symtable(chub, (void *) metadata, size);

	return 0;
}

int contexthub_imgloader_verify_fw(struct imgloader_desc *desc, phys_addr_t fw_phys_base,
                size_t fw_bin_size, size_t fw_mem_size)
{
        uint64_t ret64 = 0;

        ret64 = exynos_verify_subsystem_fw(desc->name, desc->fw_id,
                        fw_phys_base, fw_bin_size,
                        ALIGN(fw_mem_size, SZ_4K));
        if (ret64) {
                nanohub_dev_warn(desc->dev, "Failed F/W verification, ret=%llu\n", ret64);
                return -EIO;
        }

        ret64 = exynos_request_fw_stage2_ap(desc->name);
        if (ret64) {
                nanohub_dev_warn(desc->dev, "Failed F/W verification to S2MPU, ret=%llu\n", ret64);
                return -EIO;
        }

        return 0;
}


struct imgloader_ops contexthub_imgloader_ops = {
        .mem_setup = contexthub_imgloader_mem_setup,
	.verify_fw = contexthub_imgloader_verify_fw
};

static int contexthub_imgloader_init(struct contexthub_ipc_info *chub)
{
	int ret = 0;
#if IS_ENABLED(CONFIG_EXYNOS_IMGLOADER)
	int i;

	for (i = 0 ; i < MAX_FIRMWARE_NUM ; i++) {
		chub->chub_img_desc[i].dev = chub->dev;
		chub->chub_img_desc[i].owner = THIS_MODULE;
		chub->chub_img_desc[i].ops = &contexthub_imgloader_ops;
		chub->chub_img_desc[i].data = (void *) chub;
		chub->chub_img_desc[i].name = "CHUB";
		chub->chub_img_desc[i].s2mpu_support = true;
		chub->chub_img_desc[i].fw_name = i ? os_image[i - 1] : "bl.unchecked.bin";
		chub->chub_img_desc[i].fw_id = i ? 1 : 0;

		ret |= imgloader_desc_init(&chub->chub_img_desc[i]);
	}
#endif
	return ret;
}

static int contexthub_ipc_probe(struct platform_device *pdev)
{
	struct contexthub_ipc_info *chub;
	int need_to_free = 0;
	int ret = 0;
	int i;
#ifdef CONFIG_CHRE_SENSORHUB_HAL
	struct iio_dev *iio_dev;
#endif
	/* register memlog at 1st to log chub */
	ret = contexthub_memlog_init(&pdev->dev);
	if (ret)
		dev_err(&pdev->dev, "memlog not registered...\n");

	chub = chub_dbg_get_memory(pdev->dev.of_node, DBG_NANOHUB_DD_AREA);
	if (!chub) {
		chub =
		    devm_kzalloc(&pdev->dev, sizeof(struct contexthub_ipc_info),
				 GFP_KERNEL);
		need_to_free = 1;
	}
	if (IS_ERR(chub)) {
		dev_err(&pdev->dev, "%s failed to get ipc memory\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	chub_info = chub;
	chub->dev = &pdev->dev;

	chub->log_info = log_register_buffer(chub->dev, 0,
					     &chub->chub_rt_log,
					     "log", 0);
	/* parse dt and hw init */
	ret = contexthub_ipc_hw_init(pdev, chub);
	if (ret) {
		nanohub_dev_err(&pdev->dev, "%s failed to get init hw with ret %d\n",
			__func__, ret);
		goto err;
	}

#ifdef CONFIG_CHRE_SENSORHUB_HAL
	/* nanohub probe */
	iio_dev = nanohub_probe(&pdev->dev, NULL);
	if (IS_ERR(iio_dev))
		goto err;

	/* set wakeup irq number on nanohub driver */
	chub->data = iio_priv(iio_dev);
	nanohub_mailbox_comms_init(&chub->data->comms);
	chub->pdata = chub->data->pdata;
	chub->pdata->mailbox_client = chub;
	chub->data->irq1 = chub->irq_mailbox;
	chub->data->irq2 = 0;
#endif

	chub->chub_rt_log.loglevel = 0;
	spin_lock_init(&chub->logout_lock);
	contexthub_reset_token(chub);
	atomic_set(&chub->chub_status, CHUB_ST_NO_POWER);
	contexthub_set_in_reset(chub, 0);
	chub->powermode = 0; /* updated by fw bl */
	chub->cur_err = 0;
	for (i = 0; i < CHUB_ERR_MAX; i++)
		chub->err_cnt[i] = 0;
	platform_set_drvdata(pdev, chub);
	contexthub_set_baaw(chub);

	for (i = 0, ret = 0; i < ARRAY_SIZE(attributes); i++) {
		ret = device_create_file(chub->dev, &attributes[i]);
		if (ret)
			nanohub_dev_warn(chub->dev, "Failed to create file: %s\n",
				 attributes[i].attr.name);
	}
	init_waitqueue_head(&chub->poweron_lock.event);
	init_waitqueue_head(&chub->reset_lock.event);
	init_waitqueue_head(&chub->read_lock.event);
	init_waitqueue_head(&chub->chub_alive_lock.event);
	atomic_set(&chub->poweron_lock.flag, 0);
	atomic_set(&chub->chub_alive_lock.flag, 0);
	INIT_WORK(&chub->debug_work, handle_debug_work_func);
	INIT_WORK(&chub->log_work, handle_log_work_func);
	chub->log_work_reqcnt = 0;
#ifdef CONFIG_EXYNOS_ITMON
	chub->itmon_nb.notifier_call = contexthub_itmon_notifier;
	itmon_notifier_chain_register(&chub->itmon_nb);
#endif
	atomic_notifier_chain_register(&panic_notifier_list, &chub_panic_notifier);
	contexthub_notifier_register(&chub_cipc_nb);

	chub->s2mpu_nb.notifier_call = contexthub_s2mpu_notifier;
	chub->s2mpu_nb.subsystem = "CHUB";
	s2mpufd_notifier_call_register(&chub->s2mpu_nb);

	/* init fw runtime log */
	chub->chub_rt_log.buffer = vzalloc(SZ_512K * 4);
	if (!chub->chub_rt_log.buffer) {
		ret = -ENOMEM;
		goto err;
	}
	chub->chub_rt_log.buffer_size = SZ_512K * 4;
	chub->chub_rt_log.write_index = 0;
	chub->chub_rt_log.wrap = false;

	chub->ws_reset = chub_wake_lock_init(&pdev->dev, "chub_reboot");
	nanohub_dev_info(chub->dev, "%s with %s FW and %lu clk is done\n",
					__func__, chub->os_name, chub->clkrate);

	if (IS_ENABLED(CONFIG_EXYNOS_SYSTEM_EVENT)) {
		/* sysevent register */
		chub->sysevent_desc.name = "CHB";
		strcpy(chub->sysevent_desc.fw_name, "os.checked_1.bin");
		chub->sysevent_desc.owner = THIS_MODULE;
		chub->sysevent_desc.shutdown = contexthub_sysevent_shutdown;
		chub->sysevent_desc.powerup = contexthub_sysevent_powerup;
		chub->sysevent_desc.ramdump = contexthub_sysevent_ramdump;
		chub->sysevent_desc.crash_shutdown = contexthub_sysevent_crash_shutdown;
		chub->sysevent_desc.dev = &pdev->dev;

		chub->sysevent_dev = sysevent_register(&chub->sysevent_desc);

		if (IS_ERR(chub->sysevent_dev)) {
			ret = PTR_ERR(chub->sysevent_dev);
			nanohub_dev_err(&pdev->dev, "%s: failed to register sysevent:%d\n", __func__, ret);
		} else
			nanohub_dev_info(&pdev->dev, "%s: success to register sysevent\n", __func__);
	}
	contexthub_imgloader_init(chub);

	return 0;
err:
	if (chub)
		if (need_to_free)
			devm_kfree(&pdev->dev, chub);

	nanohub_dev_err(&pdev->dev, "%s is fail with ret %d\n", __func__, ret);
	return ret;
}

static int contexthub_ipc_remove(struct platform_device *pdev)
{
	struct contexthub_ipc_info *chub = platform_get_drvdata(pdev);

	chub_wake_lock_destroy(chub->ws_reset);
	return 0;
}

static int contexthub_suspend(struct device *dev)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);
	struct ipc_info *chub_ipc = ipc->chub_ipc;

	if (atomic_read(&ipc->chub_status) != CHUB_ST_RUN)
		return 0;

	nanohub_dev_info(dev, "%s: irq-pend:ap:%x,chub:%x\n", __func__,
		ipc_hw_read_int_status_reg_all(ipc->mailbox, chub_ipc->my_mb_id),
		ipc_hw_read_int_status_reg_all(ipc->mailbox, chub_ipc->opp_mb_id));
	ipc_write_hw_value(IPC_VAL_HW_AP_STATUS, AP_SLEEP);
	ipc_hw_gen_interrupt(ipc->mailbox, chub_ipc->opp_mb_id, IRQ_NUM_CHUB_ALIVE);
	atomic_set(&ipc->chub_sleep, 1);
	return 0;
}

static int contexthub_resume(struct device *dev)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);
	struct ipc_info *chub_ipc = ipc->chub_ipc;

	if (atomic_read(&ipc->chub_status) != CHUB_ST_RUN)
		return 0;

	nanohub_dev_info(dev, "%s: irq-pend:ap:%x,chub:%x\n", __func__,
		ipc_hw_read_int_status_reg_all(ipc->mailbox, chub_ipc->my_mb_id),
		ipc_hw_read_int_status_reg_all(ipc->mailbox, chub_ipc->opp_mb_id));
	ipc_write_hw_value(IPC_VAL_HW_AP_STATUS, AP_WAKE);
	ipc_hw_gen_interrupt(ipc->mailbox, chub_ipc->opp_mb_id, IRQ_NUM_CHUB_ALIVE);
	atomic_set(&ipc->chub_sleep, 0);
	return 0;
}

static SIMPLE_DEV_PM_OPS(contexthub_pm_ops, contexthub_suspend, contexthub_resume);

static const struct of_device_id contexthub_ipc_match[] = {
	{.compatible = "samsung,exynos-nanohub"},
	{},
};

static struct platform_driver samsung_contexthub_ipc_driver = {
	.probe = contexthub_ipc_probe,
	.remove = contexthub_ipc_remove,
	.driver = {
		   .name = "nanohub-ipc",
		   .owner = THIS_MODULE,
		   .of_match_table = contexthub_ipc_match,
		   .pm = &contexthub_pm_ops,
	},
};

int nanohub_mailbox_init(void)
{
	nanohub_init();
	return platform_driver_register(&samsung_contexthub_ipc_driver);
}

static void __exit nanohub_mailbox_cleanup(void)
{
	nanohub_cleanup();
	platform_driver_unregister(&samsung_contexthub_ipc_driver);
}

module_init(nanohub_mailbox_init);
module_exit(nanohub_mailbox_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Exynos contexthub mailbox Driver");
MODULE_AUTHOR("Boojin Kim <boojin.kim@samsung.com>");
