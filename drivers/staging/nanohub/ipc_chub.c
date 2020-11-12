/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *
 * Boojin Kim <boojin.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "ipc_chub.h"

#if defined(CHUB_IPC)
#if defined(SEOS)
#include <seos.h>
#include <errno.h>
#include <cmsis.h>
#elif defined(EMBOS)
#include <Device.h>
#define EINVAL 22
#endif
#include <mailboxDrv.h>
#include <csp_common.h>
#include <csp_printf.h>
#include <string.h>
#include <string.h>

#include <mailboxOS.h>
#include <plat/dfs/dfs.h>
#include <cmu.h>
#elif defined(AP_IPC)
#include <linux/delay.h>
#include <linux/io.h>
#include "chub.h"
#endif

extern struct contexthub_ipc_info *chub_info;
struct ipc_info ipc;

#if defined(CHUB_IPC)
#define NAME_PREFIX "ipc"
#else
#define NAME_PREFIX "nanohub-ipc"
#endif

#define SENSORMAP_MAGIC	"SensorMap"
#define MAX_ACTIVE_SENSOR_NUM (10)
bool ipc_have_sensor_info(struct sensor_map *sensor_map)
{
	if (sensor_map)
		if (!strncmp
		    (SENSORMAP_MAGIC, sensor_map->magic,
		     sizeof(SENSORMAP_MAGIC)))
			return true;
	return false;
}

void *ipc_get_base(enum ipc_region area)
{
	return ipc.ipc_addr[area].base;
}

u32 ipc_get_size(enum ipc_region area)
{
	return ipc.ipc_addr[area].size;
}

static u32 ipc_get_offset(enum ipc_region area)
{
	return (u32) (ipc.ipc_addr[area].base - ipc.sram_base);
}

static u32 ipc_get_offset_addr(void *addr)
{
	return (u32) (addr - ipc.sram_base);
}

static void ipc_set_info(enum ipc_region area, void *base, int size, char *name)
{
	ipc.ipc_addr[area].base = base;
	ipc.ipc_addr[area].size = size;
	memset(ipc.ipc_addr[area].name, 0, IPC_NAME_MAX);
	strncpy(ipc.ipc_addr[area].name, name, IPC_NAME_MAX - 1);
}

/* API for bootmode and runtimelog */
void ipc_set_chub_bootmode(u16 bootmode, u16 rtlog)
{
	struct chub_bootargs *map = ipc_get_base(IPC_REG_BL_MAP);

	map->bootmode = bootmode;
	map->runtimelog = rtlog;
}

u16 ipc_get_chub_bootmode(void)
{
	struct chub_bootargs *map = ipc_get_base(IPC_REG_BL_MAP);

	return map->bootmode;
}

u16 ipc_get_chub_rtlogmode(void)
{
	struct chub_bootargs *map = ipc_get_base(IPC_REG_BL_MAP);

	return map->runtimelog;
}

/* API for ap sleep */
void ipc_set_ap_wake(u16 wake)
{
	struct ipc_map_area *map = ipc_get_base(IPC_REG_IPC);

	map->wake = wake;
}

u16 ipc_get_ap_wake(void)
{
	struct ipc_map_area *map = ipc_get_base(IPC_REG_IPC);

	return map->wake;
}

struct sampleTimeTable *ipc_get_dfs_sampleTime(void)
{

	struct ipc_map_area *map = ipc_get_base(IPC_REG_IPC);

	return (struct sampleTimeTable *)map->persist.dfs.sampleTime;
}

u32 ipc_get_dfs_gov(void)
{
	struct ipc_map_area *map = ipc_get_base(IPC_REG_IPC);

	return map->persist.dfs.governor;
}

void ipc_set_dfs_gov(uint32_t gov)
{
	struct ipc_map_area *map = ipc_get_base(IPC_REG_IPC);

	map->persist.dfs.governor = gov;
}

void ipc_set_dfs_numSensor(uint32_t num)
{
	struct ipc_map_area *map = ipc_get_base(IPC_REG_IPC);

	map->persist.dfs.numSensor = num;
}

u32 ipc_get_dfs_numSensor(void)
{
	struct ipc_map_area *map = ipc_get_base(IPC_REG_IPC);

	return map->persist.dfs.numSensor;
}

static void ipc_print_logbuf(void)
{
	struct ipc_logbuf *logbuf = &ipc.ipc_map->logbuf;

	CSP_PRINTF_INFO
	    ("%s: channel: eq:%d, dq:%d, size:%d, full:%d, dbg_full_cnt:%d, err(overwrite):%d, ipc-reqcnt:%d, fw:%d\n",
	     __func__, logbuf->eq, logbuf->dq, logbuf->size, logbuf->full,
	     logbuf->dbg_full_cnt, logbuf->errcnt, logbuf->reqcnt,
	     logbuf->fw_num);
}

void ipc_dump(void)
{
	if (strncmp(CHUB_IPC_MAGIC, ipc.ipc_map->magic, sizeof(CHUB_IPC_MAGIC))) {
		CSP_PRINTF_INFO("%s: [%s]: ipc crash\n", NAME_PREFIX, __func__);
		return;
	}

	CSP_PRINTF_INFO("[%s]: magic:%s\n", __func__, ipc.ipc_map->magic);
	ipc_print_logbuf();
	cipc_dump(CIPC_USER_AP2CHUB);
	cipc_dump(CIPC_USER_CHUB2AP);
}

#ifdef AP_IPC
#define LOGFILE_NUM_SIZE (32)
extern struct memlog_obj *memlog_printf_chub;

u32 ipc_get_chub_mem_size(void)
{
	return ipc.ipc_addr[IPC_REG_DUMP].size;
}

void ipc_set_chub_clk(u32 clk)
{
	struct chub_bootargs *map = ipc_get_base(IPC_REG_BL_MAP);

	map->chubclk = clk;
}

u8 ipc_get_sensor_info(u8 type)
{
	int i;
	struct sensor_map *ipc_sensor_map =
	    ipc_get_base(IPC_REG_IPC_SENSORINFO);

	CSP_PRINTF_INFO("%s: senstype:%d, index:%d\n", __func__, type,
			ipc_sensor_map->index);
	if (ipc_have_sensor_info(ipc_sensor_map)) {
		for (i = 0; i < ipc_sensor_map->index; i++)
			if (ipc_sensor_map->sinfo[i].senstype == type) {
				CSP_PRINTF_INFO
				    ("%s: find: senstype:%d, id:%d\n", __func__,
				     type, ipc_sensor_map->sinfo[i].chipid);
				return ipc_sensor_map->sinfo[i].chipid;
			}
	}
	return 0;
}

void ipc_logbuf_flush_on(bool on)
{
	struct ipc_logbuf *logbuf;

	if (ipc.ipc_map) {
		logbuf = &ipc.ipc_map->logbuf;
		logbuf->flush_active = on;
	}
}

bool ipc_logbuf_filled(void)
{
	struct ipc_logbuf *logbuf;

	if (ipc.ipc_map) {
		logbuf = &ipc.ipc_map->logbuf;
		return logbuf->eq != logbuf->dq;
	}
	return 0;
}

int ipc_logbuf_printf(struct logbuf_content *log, u32 len)
{
	u32 lenout = 0;
	struct runtimelog_buf *rt_buf = &chub_info->chub_rt_log;
	int dst;

	if (!log || !chub_info || !rt_buf || !rt_buf->buffer) {
		pr_info("%s nullptr!", __func__);
		return -EINVAL;
	}
	if (rt_buf->write_index + len + LOGFILE_NUM_SIZE >= rt_buf->buffer_size)
		rt_buf->write_index = 0;
	dst = rt_buf->write_index; //for race condition
	if (dst + len + LOGFILE_NUM_SIZE >= rt_buf->buffer_size) {
		pr_info("%s index err!", __func__);
		return -EINVAL;
	}

	lenout = snprintf(rt_buf->buffer + dst, LOGFILE_NUM_SIZE + len, "%10d:[%c][%6u.%06u]%c %s\n",
			log->size, log->size ? 'F' : 'K', (log->timestamp) / 1000000,
			(log->timestamp) % 1000000, log->level, log->buf);
	rt_buf->write_index += lenout;
	if (lenout != (LOGFILE_NUM_SIZE + len))
		pr_warn("%s: %s: size-n mismatch: %d -> %d\n",
			NAME_PREFIX, __func__, LOGFILE_NUM_SIZE + len, lenout);
	return 0;
}

int ipc_logbuf_outprint(struct runtimelog_buf *rt_buf, u32 loop)
{
	if (ipc.ipc_map) {
		u8 level = MEMLOG_LEVEL_DEBUG;
		struct logbuf_content *log;
		struct ipc_logbuf *logbuf = &ipc.ipc_map->logbuf;
		u32 eq;
		u32 len;

retry:
		eq = logbuf->eq;
		if (eq >= LOGBUF_NUM || logbuf->dq >= LOGBUF_NUM) {
			CSP_PRINTF_ERROR("%s: index  eq:%d, dq:%d\n", __func__, eq,
			       logbuf->dq);
			logbuf->eq = logbuf->dq = 0;

			if (logbuf->eq != 0 || logbuf->dq != 0) {
				__raw_writel(0, &logbuf->eq);
				__raw_writel(0, &logbuf->dq);
				CSP_PRINTF_ERROR("%s: index after: eq:%d, dq:%d\n",
				       __func__, __raw_readl(&logbuf->eq),
				       __raw_readl(&logbuf->dq));
			}
			return -1;
		}

		if (logbuf->full) {
			logbuf->full = 0;
			logbuf->dq = (eq + 1) % LOGBUF_NUM;
		}

		while (eq != logbuf->dq) {
			log = &logbuf->log[logbuf->dq];
			len = strlen((char *)log);

			if (len > 0 && len <= LOGBUF_DATA_SIZE) {
				char buf[120];
				memset(buf, 0, 120);
				if (log->level || log->timestamp) {
					pr_info("nanohub:FW:[%6llu.%06llu]%c %s",
						(log->timestamp) / 1000000,
						(log->timestamp) % 1000000,
						log->level, (char *)log->buf);
					if (memlog_printf_chub)
						memlog_write_printf(memlog_printf_chub, MEMLOG_LEVEL_ERR,
								    "FW:[%6llu.%06llu]%c %s",
								    (log->timestamp) / 1000000,
								    (log->timestamp) % 1000000,
								    log->level, (char *)log->buf);
				} else {
					pr_info("FW:                  %s", (char *)log->buf);
					if (memlog_printf_chub)
						memlog_write_printf(memlog_printf_chub,
								    MEMLOG_LEVEL_ERR,
								    "FW:                  %s",
								    (char *)log->buf);
				}

				if (rt_buf)
					ipc_logbuf_printf(log, len);
			} else {
				CSP_PRINTF_ERROR("%s: %s: size err:%d, eq:%d, dq:%d\n",
				       NAME_PREFIX, __func__, len, eq,
				       logbuf->dq);
			}
			logbuf->dq = (logbuf->dq + 1) % LOGBUF_NUM;
		}
		msleep(10);
		if ((eq != logbuf->eq) && loop) {
			loop--;
			goto retry;
		}

		if (logbuf->flush_req)
			logbuf->flush_req = 0;
	}
	return 0;
}

#define ipc_logbuf_inbase(a) ((void)0)
#define ipc_logbuf_req_flush(a) ((void)0)

void ipc_reset_map(void)
{
	ipc_hw_clear_all_int_pend_reg(ipc.mb_base, IPC_SRC_MB0);
	ipc_hw_clear_all_int_pend_reg(ipc.mb_base, IPC_DST_MB1);
	ipc_hw_set_mcuctrl(ipc.mb_base, 0x1);
	memset_io(ipc_get_base(IPC_REG_LOG), 0, ipc_get_size(IPC_REG_LOG));
	memset_io(cipc_get_base(CIPC_REG_AP2CHUB), 0, cipc_get_size(CIPC_REG_AP2CHUB));
	memset_io(cipc_get_base(CIPC_REG_CHUB2AP), 0, cipc_get_size(CIPC_REG_CHUB2AP));
	memset_io(cipc_get_base(CIPC_REG_ABOX2CHUB), 0, cipc_get_size(CIPC_REG_ABOX2CHUB));
	memset_io(cipc_get_base(CIPC_REG_CHUB2ABOX), 0, cipc_get_size(CIPC_REG_CHUB2ABOX));
	cipc_reset_map();
}
#else
#include <os/inc/trylock.h>
static TRYLOCK_DECL_STATIC(ipcLockLog) = TRYLOCK_INIT_STATIC();

u32 ipc_get_chub_clk(void)
{
	struct chub_bootargs *map = ipc_get_base(IPC_REG_BL_MAP);

	return map->chubclk;
}

void ipc_set_sensor_info(u8 type, const char *name, const char *vendor,
			 u8 senstype, u8 id)
{
	struct sensor_map *ipc_sensor_map =
	    ipc_get_base(IPC_REG_IPC_SENSORINFO);

	if (ipc_have_sensor_info(ipc_sensor_map)) {
		if (ipc_sensor_map->index >= MAX_PHYSENSOR_NUM) {
			CSP_PRINTF_ERROR("%s: invalid index:%d\n", __func__,
					 ipc_sensor_map->index);
			return;
		}
		if (name) {
			ipc_sensor_map->sinfo[ipc_sensor_map->index].sensortype = type;
			strncpy(ipc_sensor_map->
				sinfo[ipc_sensor_map->index].name, name,
				MAX_SENSOR_NAME);
			strncpy(ipc_sensor_map->
				sinfo[ipc_sensor_map->index].vendorname, vendor,
				MAX_SENSOR_VENDOR_NAME);
		}
		if (senstype) {
			int i;

			for (i = 0; i < MAX_SENSOR_NUM; i++)
				if (ipc_sensor_map->sinfo[i].sensortype == type) {
					ipc_sensor_map->sinfo[i].chipid = id;
					ipc_sensor_map->sinfo[i].senstype =
					    senstype;
					return;
				}
		}
		CSP_PRINTF_ERROR("sensortype: %d: u:%d, t:%d, n:%s, v:%s\n",
				 ipc_sensor_map->index, type,
				 ipc_sensor_map->sinfo[ipc_sensor_map->index].sensortype,
				 ipc_sensor_map->sinfo[ipc_sensor_map->index].name,
				 ipc_sensor_map->sinfo[ipc_sensor_map->index].vendorname);
		ipc_sensor_map->index++;
	}
}

void *ipc_logbuf_inbase(bool force)
{
	if (ipc.ipc_map) {
		struct ipc_logbuf *logbuf = &ipc.ipc_map->logbuf;

		if (logbuf->eq >= LOGBUF_NUM || logbuf->dq >= LOGBUF_NUM)
			return NULL;

		if (force || logbuf->loglevel) {
			struct logbuf_content *log;
			int index;

			/* check the validataion of ipc index */
			if (logbuf->eq >= LOGBUF_NUM
			    || logbuf->dq >= LOGBUF_NUM)
				return NULL;

			if (!trylockTryTake(&ipcLockLog))
				return NULL;

			if (logbuf->full)	/* logbuf is full overwirte */
				logbuf->dbg_full_cnt++;

			index = logbuf->eq;
			logbuf->eq = (logbuf->eq + 1) % LOGBUF_NUM;
			if (logbuf->eq == logbuf->dq)
				logbuf->full = 1;
			trylockRelease(&ipcLockLog);

			log = &logbuf->log[index];
			memset(log, 0, sizeof(struct logbuf_content));
			return log;
		}
	}
	return NULL;
}

struct logbuf_content *ipc_logbuf_get_curlogbuf(struct logbuf_content *log)
{
	int i;

	for (i = 0; i < log->newline; i++) {
		if (log->nextaddr)
			log = (struct logbuf_content *)log->nextaddr;
		else
			break;
	}

	return log;
}

void ipc_logbuf_set_req_num(struct logbuf_content *log)
{
	struct ipc_logbuf *logbuf = &ipc.ipc_map->logbuf;

	log->size = logbuf->fw_num++;
}

void ipc_logbuf_req_flush(struct logbuf_content *log)
{
	if (log) {
		struct ipc_logbuf *logbuf = &ipc.ipc_map->logbuf;

		if (logbuf->eq >= LOGBUF_NUM || logbuf->dq >= LOGBUF_NUM)
			return;

		/* debug check overwrite */
		if (log->nextaddr && log->newline) {
			struct logbuf_content *nextlog =
			    ipc_logbuf_get_curlogbuf(log);

			nextlog->size = logbuf->fw_num++;
		} else {
			log->size = logbuf->fw_num++;
		}

		if (ipc.ipc_map) {
			if (!logbuf->flush_req && !logbuf->flush_active) {
#ifdef USE_LOG_FLUSH_TRSHOLD
				u32 eq = logbuf->eq;
				u32 dq = logbuf->dq;
				u32 logcnt =
				    (eq >= dq) ? (eq - dq) : (eq + (logbuf->size - dq));

				if (((ipc_get_ap_wake() == AP_WAKE)
				     && (logcnt > LOGBUF_FLUSH_THRESHOLD))
				    || log->error) {
					if (!logbuf->flush_req) {
						logbuf->flush_req = 1;
						ipc_hw_gen_interrupt(ipc.mb_base,
							ipc.opp_mb_id, IRQ_NUM_CHUB_LOG);
					}
				}
#else
				if ((ipc_get_ap_wake() == AP_WAKE)
				    || log->error) {
					if (!logbuf->flush_req) {
						logbuf->flush_req = 1;
						logbuf->reqcnt++;
						ipc_hw_gen_interrupt(ipc.mb_base,
							ipc.opp_mb_id, IRQ_NUM_CHUB_LOG);
					}
				}
#endif
			}
		}
	}
}

#define ipc_logbuf_outprint(a) ((void)0, 0)

#if defined(LOCAL_POWERGATE)
u32 *ipc_get_chub_psp(void)
{
	struct chub_bootargs *map = ipc_get_base(IPC_REG_BL_MAP);

	return &(map->psp);
}

u32 *ipc_get_chub_msp(void)
{
	struct chub_bootargs *map = ipc_get_base(IPC_REG_BL_MAP);

	return &(map->msp);
}
#endif

struct ipc_info *ipc_get_info(void)
{
	return &ipc;
}
#endif

enum ipc_fw_loglevel ipc_logbuf_loglevel(enum ipc_fw_loglevel loglevel, int set)
{
	if (ipc.ipc_map) {
		struct ipc_logbuf *logbuf = &ipc.ipc_map->logbuf;

		if (set)
			logbuf->loglevel = (u8) loglevel;
		return (enum ipc_fw_loglevel)logbuf->loglevel;
	}

	return 0;
}

void ipc_write_value(enum ipc_value_id id, u64 val)
{
	if (ipc.ipc_map)
		ipc.ipc_map->value[id] = val;
}

u64 ipc_read_value(enum ipc_value_id id)
{
	if (ipc.ipc_map)
		return ipc.ipc_map->value[id];
	return 0;
}

void ipc_write_hw_value(enum ipc_value_id id, u64 val)
{
	u32 low = val & 0xffffffff;
	u32 high;

	switch (id) {
	case IPC_VAL_HW_BOOTMODE:
		ipc_hw_write_shared_reg(ipc.mb_base, low, SR_BOOT_MODE);
		break;
	case IPC_VAL_HW_AP_STATUS:
		ipc_hw_write_shared_reg(ipc.mb_base, low, SR_CHUB_ALIVE);
		break;
	case IPC_VAL_HW_DEBUG:
		high = val >> 32;
		ipc_hw_write_shared_reg(ipc.mb_base, low, SR_DEBUG_VAL_LOW);
		ipc_hw_write_shared_reg(ipc.mb_base, high, SR_DEBUG_VAL_HIGH);
		break;
	default:
		CSP_PRINTF_ERROR("%s: invalid id:%d\n", __func__, id);
		break;
	};
}

u64 ipc_read_hw_value(enum ipc_value_id id)
{
	u64 val = 0;
	u32 low;
	u64 high;

	switch (id) {
	case IPC_VAL_HW_BOOTMODE:
		val = ipc_hw_read_shared_reg(ipc.mb_base, SR_BOOT_MODE);
		break;
	case IPC_VAL_HW_AP_STATUS:
		val = ipc_hw_read_shared_reg(ipc.mb_base, SR_3);
		break;
	case IPC_VAL_HW_DEBUG:
		low = ipc_hw_read_shared_reg(ipc.mb_base, SR_DEBUG_VAL_LOW);
		high = ipc_hw_read_shared_reg(ipc.mb_base, SR_DEBUG_VAL_HIGH);
		val = low | (high << 32);
		break;
	default:
		CSP_PRINTF_ERROR("%s: invalid id:%d\n", __func__, id);
		break;
	};
	return val;
}

static void *ipc_set_map(char *sram_base)
{
	struct chub_bootargs *map =
	    (struct chub_bootargs *)(sram_base + MAP_INFO_OFFSET);

#ifdef AP_IPC
	if (strncmp(OS_UPDT_MAGIC, map->magic, sizeof(OS_UPDT_MAGIC))) {
		CSP_PRINTF_ERROR("%s: %s: %p has wrong magic key: %s -> %s\n",
				 NAME_PREFIX, __func__, map, OS_UPDT_MAGIC,
				 map->magic);
		return 0;
	}

	if (map->ipc_version != IPC_VERSION) {
		CSP_PRINTF_ERROR
		    ("%s: %s: ipc_version doesn't match: AP %d, Chub: %d\n",
		     NAME_PREFIX, __func__, IPC_VERSION, map->ipc_version);
		return 0;
	}

	CSP_PRINTF_INFO("%s enter: version:%x, bootarg_size:%d\n",
		__func__, map->ipc_version, sizeof(struct chub_bootargs));
	if (sizeof(struct chub_bootargs) > MAP_INFO_MAX_SIZE) {
		CSP_PRINTF_ERROR
		    ("%s: %s: chub_bootargs size bigger than max %d > %d", NAME_PREFIX,
		     __func__, sizeof(struct chub_bootargs), MAP_INFO_MAX_SIZE);
		return 0;
	}

	if (sizeof(struct ipc_map_area) > CIPC_START_OFFSET) {
		CSP_PRINTF_ERROR
		    ("%s: %s: ipc_map_area size bigger than max %d > %d", NAME_PREFIX,
		     __func__, sizeof(struct ipc_map_area), CIPC_START_OFFSET);
		return 0;
	}
#endif

	ipc.sram_base = sram_base;

	ipc_set_info(IPC_REG_BL, sram_base, map->bl_end - map->bl_start, "bl");
	ipc_set_info(IPC_REG_BL_MAP, map, sizeof(struct chub_bootargs),
		     "bl_map");
	ipc_set_info(IPC_REG_OS, sram_base + map->code_start,
		     map->code_end - map->code_start, "os");
	ipc_set_info(IPC_REG_IPC, sram_base + map->ipc_start,
		     map->ipc_end - map->ipc_start, "ipc");
	ipc_set_info(IPC_REG_RAM, sram_base + map->ram_start,
		     map->ram_end - map->ram_start, "ram");
	ipc_set_info(IPC_REG_DUMP, sram_base + map->dump_start,
		     map->dump_end - map->dump_start, "dump");

	ipc.ipc_map = ipc.ipc_addr[IPC_REG_IPC].base;

#ifdef AP_IPC
	if (ipc_get_size(IPC_REG_IPC) < sizeof(struct ipc_map_area)) {
		CSP_PRINTF_INFO
		    ("%s: fails. ipc size (0x%x) should be increase to 0x%x\n",
		     __func__, ipc_get_size(IPC_REG_IPC),
		     sizeof(struct ipc_map_area));
		return 0;
	}
	memset_io(ipc_get_base(IPC_REG_IPC), 0, ipc_get_size(IPC_REG_IPC));
#endif

	ipc.ipc_map->logbuf.size = LOGBUF_TOTAL_SIZE;
	ipc_set_info(IPC_REG_LOG, &ipc.ipc_map->logbuf,
		     sizeof(struct ipc_logbuf), "log");
	ipc_set_info(IPC_REG_PERSISTBUF, &ipc.ipc_map->persist,
		     sizeof(struct chub_persist), "persist");
	ipc_set_info(IPC_REG_IPC_SENSORINFO, &ipc.ipc_map->sensormap,
		     sizeof(struct sensor_map), "sensorinfo");

#ifdef CHUB_IPC
	strncpy(&ipc.ipc_map->magic[0], CHUB_IPC_MAGIC, sizeof(CHUB_IPC_MAGIC));

#ifdef SEOS
	ipc.ipc_map->logbuf.loglevel = ipc_get_chub_rtlogmode();
	if (!ipc_have_sensor_info(&ipc.ipc_map->sensormap)) {
		CSP_PRINTF_INFO("%s:ipc set sensormap and magic:%p\n", __func__,
				&ipc.ipc_map->sensormap);
		memset(&ipc.ipc_map->sensormap, 0, sizeof(struct sensor_map));
		strncpy(&ipc.ipc_map->sensormap.magic[0], SENSORMAP_MAGIC,
			sizeof(SENSORMAP_MAGIC));
	}
#endif
	if ((CHUB_DFS_SIZE * sizeof(u32)) <
	    (sizeof(struct sampleTimeTable) * MAX_SENS_NUM)) {
		CSP_PRINTF_ERROR
		    ("%s: dfs persist size %d should be bigger than %d\n",
		     NAME_PREFIX, CHUB_DFS_SIZE,
		     (sizeof(struct sampleTimeTable) * MAX_SENS_NUM));
		return NULL;
	}
#endif

	CSP_PRINTF_INFO("IPC %s: map info(v%u,bt:%d,rt:%d)\n",
			NAME_PREFIX, map->ipc_version, ipc_get_chub_bootmode(),
			ipc_get_chub_rtlogmode());
	CSP_PRINTF_INFO("bl(+%d %d)\n", ipc_get_offset(IPC_REG_BL),
			ipc_get_size(IPC_REG_BL));
	CSP_PRINTF_INFO("os(+%d %d)\n", ipc_get_offset(IPC_REG_OS),
			ipc_get_size(IPC_REG_OS));
	CSP_PRINTF_INFO("ipc(+%d %d)\n", ipc_get_offset(IPC_REG_IPC),
			ipc_get_size(IPC_REG_IPC));
	CSP_PRINTF_INFO("ram(+%d %d)\n", ipc_get_offset(IPC_REG_RAM),
			ipc_get_size(IPC_REG_RAM));
	CSP_PRINTF_INFO("dump(+%d %d)\n", ipc_get_offset(IPC_REG_DUMP),
			ipc_get_size(IPC_REG_DUMP));
	CSP_PRINTF_INFO("sensormap(+%d %d)\n",
			ipc_get_offset(IPC_REG_IPC_SENSORINFO),
			ipc_get_size(IPC_REG_IPC_SENSORINFO));
	CSP_PRINTF_INFO("persistbuf(+%d %d)\n",
			ipc_get_offset(IPC_REG_PERSISTBUF),
			ipc_get_size(IPC_REG_PERSISTBUF));
	CSP_PRINTF_INFO("log(+%d %d)\n", ipc_get_offset(IPC_REG_LOG),
			ipc_get_size(IPC_REG_LOG));
	return &ipc.ipc_map;
}

struct ipc_info *ipc_init(enum ipc_owner owner, enum ipc_mb_id mb_id,
			  void *sram_base, void *mb_base,
			  struct cipc_funcs *func)
{
	struct cipc_funcs *funcs = func;

#ifdef AP_IPC
	CSP_PRINTF_INFO("%s: owner:%d, mb_id:%d, func:%p\n", __func__, owner,
			mb_id, func);
#endif

	ipc.mb_base = mb_base;
	if (owner == IPC_OWN_CHUB || owner == IPC_OWN_AP) {
		ipc.owner = owner;
		ipc.my_mb_id = mb_id;
		ipc.opp_mb_id =
		    (mb_id == IPC_SRC_MB0) ? IPC_DST_MB1 : IPC_SRC_MB0;
		ipc_set_map(sram_base);
		if (!ipc.ipc_map) {
			CSP_PRINTF_ERROR("%s: ipc_map is NULL.\n", __func__);
			return NULL;
		}
	} else {
		CSP_PRINTF_INFO("%s: invalid onwer:%d\n", __func__, owner);
		return NULL;
	}

#ifdef CHUB_IPC
	ipc.cipc = chub_cipc_init(owner, sram_base);
	if (!ipc.cipc) {
		CSP_PRINTF_ERROR("%s: cipc_init fails\n", __func__);
		return NULL;
	}
#else /* AP_IPC */
	ipc.cipc = cipc_init(owner, sram_base, funcs);
	if (!ipc.cipc) {
		CSP_PRINTF_ERROR("%s: cipc_init fails\n", __func__);
		return NULL;
	}
#endif
	ipc.ipc_map->cipc_base = cipc_get_base(CIPC_REG_CIPC_BASE);
	CSP_PRINTF_INFO("%s: done. sram_base:+%x\n", __func__,
			ipc_get_offset_addr(sram_base));

	return &ipc;
}
