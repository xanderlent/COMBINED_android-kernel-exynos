// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CHUB IF Driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 * Authors:
 *      Boojin Kim <boojin.kim@samsung.com>
 *      Sukwon Ryu <sw.ryoo@samsung.com>
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/random.h>
#include <linux/rtc.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/timekeeping.h>
#include <linux/of_gpio.h>
#include <linux/fcntl.h>
#include <uapi/linux/sched/types.h>
#ifdef CONFIG_EXYNOS_SYSTEM_EVENT
#include <soc/samsung/sysevent.h>
#endif
#ifdef CONFIG_EXYNOS_MEMORY_LOGGER
#include <soc/samsung/memlogger.h>
#endif
#ifdef CONFIG_EXYNOS_ITMON
#include <soc/samsung/exynos-itmon.h>
#endif

#include <soc/samsung/exynos-pmu.h>

#include "chub.h"
#include "ipc_chub.h"
#include "chub_dbg.h"
#include "chub_exynos.h"

void contexthub_check_time(void)
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

static irqreturn_t contexthub_irq_handler(int irq, void *data)
{
	struct contexthub_ipc_info *chub = data;
	struct cipc_info *cipc;
	enum ipc_mb_id rcv_mb_id;
	unsigned int status;
	int start_index;
	int irq_num;
	int ret;

	cipc = chub->chub_ipc->cipc;
	rcv_mb_id = cipc->user_info[CIPC_USER_CHUB2AP].map_info.dst.mb_id;
	status = ipc_hw_read_int_status_reg_all(chub->mailbox, rcv_mb_id);
	start_index = ipc_hw_start_bit(rcv_mb_id);
	irq_num = IRQ_NUM_CHUB_ALIVE + start_index;

	if (atomic_read(&chub->chub_sleep)) {
		nanohub_dev_info(chub->dev, "%s wakeup_by_me: status:0x%x\n",
				 __func__, status);
		ipc_dump();
		chub->wakeup_by_chub_cnt++;
	}

	/* chub alive interrupt handle */
	if (status & (1 << irq_num)) {
		status &= ~(1 << irq_num);
		ipc_hw_clear_int_pend_reg(chub->mailbox, rcv_mb_id, irq_num);
		if ((atomic_read(&chub->chub_status) == CHUB_ST_POWER_ON) && !chub->sel_os) {
			schedule_work(&chub->debug_work);
			return IRQ_HANDLED;
		}

		if (ipc_read_hw_value(IPC_VAL_HW_AP_STATUS) == CHUB_REBOOT_REQ) {
			nanohub_dev_err(chub->dev, "chub sends to request reboot\n");
			contexthub_handle_debug(chub, CHUB_ERR_FW_REBOOT);
		} else {
			/* set wakeup flag for chub_alive_lock */
			chub_wake_event(&chub->chub_alive_lock);
		}
	}

	if (contexthub_get_token(chub)) {
		nanohub_dev_err(chub->dev, "%s: in reset irq_status:%d\n", __func__, status);
		ipc_hw_clear_all_int_pend_reg(chub->mailbox, rcv_mb_id);
		return IRQ_HANDLED;
	}

	irq_num = IRQ_NUM_CHUB_LOG + start_index;
	if (status & (1 << irq_num)) {
		status &= ~(1 << irq_num);
		ipc_hw_clear_int_pend_reg(chub->mailbox, rcv_mb_id, irq_num);
		contexthub_log_active_work(chub);
	}

	if (status) {
		ret = cipc_irqhandler(CIPC_USER_CHUB2AP, status);
		if (ret)
			contexthub_handle_debug(chub, CHUB_ERR_ISR);
	}
	contexthub_put_token(chub);
	return IRQ_HANDLED;
}

static irqreturn_t contexthub_irq_wdt_handler(int irq, void *data)
{
	struct contexthub_ipc_info *chub = data;

	nanohub_dev_info(chub->dev, "%s called\n", __func__);
	disable_irq_nosync(chub->irq_wdt);
	chub->irq_wdt_disabled = 1;
	contexthub_handle_debug(chub, CHUB_ERR_FW_WDT);

	return IRQ_HANDLED;
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
	nanohub_dev_info(&pdev->dev, "%s: %s is mapped on 0x%lx with size of %zu\n",
			 __func__, name, ret, (size_t)resource_size(res));

	return ret;
}

static int contexthub_dt_init(struct platform_device *pdev,
					 struct contexthub_ipc_info *chub)
{
	int ret;
	const char *os;
	const char *resetmode;
	const char *dfs;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	/*chub_usi_array*/
	int reg_cnt;
	const char *reg_names[50];
	int i, j;
	struct resource *res;

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

	dfs = of_get_property(node, "chub-dfs-gov", NULL);
	nanohub_info("%s: dfs %s\n", __func__, dfs);
	if (dfs && !strcmp(dfs, "enabled"))
		chub->chub_dfs_gov = true;
	else
		chub->chub_dfs_gov = false;

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

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (IS_ERR_OR_NULL(res)) {
		nanohub_dev_err(&pdev->dev, "Failed to get %s\n", "sram");
		return -EINVAL;
	}
	chub->sram_phys = res->start;

	/* get chub gpr base */
	chub->chub_dumpgpr = get_iomem(pdev, "dumpgpr", NULL);
	if (IS_ERR(chub->chub_dumpgpr))
		return PTR_ERR(chub->chub_dumpgpr);

	chub->chub_baaw = get_iomem(pdev, "chub_baaw_p_apm", NULL);
	if (IS_ERR(chub->chub_baaw))
		nanohub_dev_warn(&pdev->dev, "%s not available", "chub_baaw_p_apm");

	chub->chub_baaw_d = get_iomem(pdev, "chub_baaw_d", NULL);
	if (IS_ERR(chub->chub_baaw_d))
		nanohub_dev_warn(&pdev->dev, "%s not available", "chub_baaw_d");

	contexthub_get_qch_base(chub);

	/* get chub cmu base */
	chub->chub_dump_cmu = get_iomem(pdev, "chub_dump_cmu", NULL);
	if (IS_ERR(chub->chub_dump_cmu))
		nanohub_dev_warn(&pdev->dev, "%s not available", "chub_dump_cmu");

	/* get chub sys base */
	chub->chub_dump_sys = get_iomem(pdev, "chub_dump_sys", NULL);
	if (IS_ERR(chub->chub_dump_sys))
		nanohub_dev_warn(&pdev->dev, "%s not available", "chub_dump_sys");

	/* get chub wdt base */
	chub->chub_dump_wdt = get_iomem(pdev, "chub_dump_wdt", NULL);
	if (IS_ERR(chub->chub_dump_wdt))
		nanohub_dev_warn(&pdev->dev, "%s not available", "chub_dump_wdt");

	/* get chub timer base */
	chub->chub_dump_timer = get_iomem(pdev, "chub_dump_timer", NULL);
	if (IS_ERR(chub->chub_dump_timer))
		nanohub_dev_warn(&pdev->dev, "%s not available", "chub_dump_timer");

	/* get chub pwm base */
	chub->chub_dump_pwm = get_iomem(pdev, "chub_dump_pwm", NULL);
	if (IS_ERR(chub->chub_dump_pwm))
		nanohub_dev_warn(&pdev->dev, "%s not available", "chub_dump_pwm");

	/* get chub rtc base */
	chub->chub_dump_rtc = get_iomem(pdev, "chub_dump_rtc", NULL);
	if (IS_ERR(chub->chub_dump_rtc))
		nanohub_dev_warn(&pdev->dev, "%s not available", "chub_dump_rtc");

	/*get usi_array*/
	chub->usi_cnt = 0;
	reg_cnt = of_property_count_strings(node, "reg-names");
	of_property_read_string_array(node, "reg-names", reg_names, reg_cnt);
	for (j = 0; j < reg_cnt; j++) {
		if (strstr(reg_names[j], "usi"))
			nanohub_dev_info(&pdev->dev,
				 "%s: usi reg name %s\n",
				 __func__, reg_names[j]);
	}
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

	/* get pmu osc rco */
	chub->pmu_osc_rco = get_iomem(pdev, "osc_rco", NULL);
	if (IS_ERR(chub->pmu_osc_rco))
		nanohub_dev_warn(dev, "fails to get pmu_osc_rco\n");

	/* get pmu rtc control */
	chub->pmu_rtc_ctrl = get_iomem(pdev, "rtc_ctrl", NULL);
	if (IS_ERR(chub->pmu_rtc_ctrl))
		nanohub_dev_warn(dev, "fails to get pmu_rtc_ctrl\n");

	/* get pmu chub control base */
	chub->pmu_chub_ctrl = get_iomem(pdev, "chub_ctrl", NULL);
	if (IS_ERR(chub->pmu_chub_ctrl))
		nanohub_dev_warn(dev, "fails to get pmu_chub_ctrl\n");

	/* get pmu chub reset release status */
	chub->pmu_chub_reset_stat = get_iomem(pdev, "chub_reset_status", NULL);
	if (IS_ERR(chub->pmu_chub_reset_stat))
		nanohub_dev_warn(dev, "fails to get pmu_chub_reset_stat\n");

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
			"driver failed to get baaw-p-cmgp-chub, start\n");
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-p-cmgp-chub", 1,
		 &chub->baaw_info.baaw_p_cmgp_chub_end)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-p-cmgp-chub, end\n");
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-p-cmgp-chub", 2,
		 &chub->baaw_info.baaw_p_cmgp_chub_remap)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-p-cmgp-chub, remap\n");
	}

	/* get addresses information to set BAAW D */
	if (of_property_read_u32_index
		(node, "baaw,baaw-d-chub", 0,
		 &chub->baaw_info.baaw_d_chub_start)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-d-chub, start\n");
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-d-chub", 1,
		 &chub->baaw_info.baaw_d_chub_end)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-d-chub, end\n");
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-d-chub", 2,
		 &chub->baaw_info.baaw_d_chub_remap)) {
		nanohub_dev_err(&pdev->dev,
			"driver failed to get baaw-d-chub, remap\n");
	}
	/* disable chub irq list (for sensor irq) */
	of_property_read_u32(node, "chub-irq-pin-len", &chub->irq_pin_len);
	if (chub->irq_pin_len) {
		if (chub->irq_pin_len > sizeof(chub->irq_pins)) {
			nanohub_dev_err(&pdev->dev, "failed to get irq pin length %d, %d\n",
				chub->irq_pin_len, sizeof(chub->irq_pins));
			chub->irq_pin_len = 0;
			return -ENODEV;
		}

		dev_info(&pdev->dev, "get chub irq_pin len:%d\n", chub->irq_pin_len);
		for (i = 0; i < chub->irq_pin_len; i++) {
			chub->irq_pins[i] = of_get_named_gpio(node, "chub-irq-pin", i);
			if (!gpio_is_valid(chub->irq_pins[i])) {
				nanohub_dev_err(&pdev->dev, "get invalid chub irq_pin:%d\n",
					chub->irq_pins[i]);
				return -EINVAL;
			}
			nanohub_dev_info(&pdev->dev, "get chub irq_pin:%d\n", chub->irq_pins[i]);
		}
	}
	contexthub_disable_pin(chub);
	//contexthub_set_clk(chub);
	contexthub_get_clock_names(chub);

	return 0;
}


#ifdef CONFIG_EXYNOS_ITMON
static int contexthub_itmon_notifier(struct notifier_block *nb,
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
#ifdef CONFIG_EXYNOS_MEMORY_LOGGER
	struct contexthub_ipc_info *chub = container_of(nb, struct contexthub_ipc_info, panic_nb);

	memlog_do_dump(chub->mlog.memlog_sram_chub, MEMLOG_LEVEL_EMERG);
#endif
	chub_dbg_dump_ram(CHUB_ERR_KERNEL_PANIC);
	return NOTIFY_OK;
}

#ifdef CONFIG_EXYNOS_SYSTEM_EVENT
static int contexthub_sysevent_shutdown(const struct sysevent_desc *desc, bool force_stop)
{
	(void) desc;
	nanohub_info("%s [%d]\n", __func__, force_stop);

	return 0;
}

static int contexthub_sysevent_powerup(const struct sysevent_desc *desc)
{
	(void) desc;
	nanohub_info("%s\n", __func__);
	return 0;
}

static int contexthub_sysevent_ramdump(int tmp, const struct sysevent_desc *desc)
{
	(void) desc;
	(void) tmp;
	nanohub_info("%s\n", __func__);
	return 0;
}

static void contexthub_sysevent_crash_shutdown(const struct sysevent_desc *desc)
{
	(void) desc;
	nanohub_info("%s\n", __func__);
}
#endif
/* CIPC Notification */
static int contexthub_notifier(struct contexthub_notifier_block *nb)
{
	nanohub_info("%s called!: subsys:%s, start_off:%x, end_off:%x",
			 __func__, nb->subsystem, nb->start_off, nb->end_off);
	return 0;
}

static struct contexthub_notifi_info cipc_noti[IPC_OWN_MAX] = {
	{"EMP", 0, NULL},
	{"CHUB", IPC_OWN_CHUB, NULL},
	{"AP", IPC_OWN_AP, NULL},
	{"GNSS", IPC_OWN_GNSS, NULL},
	{"ABOX", IPC_OWN_ABOX, NULL},
	{"VTS", IPC_OWN_VTS, NULL},
};

int contexthub_notifier_call(struct contexthub_ipc_info *chub,
			     enum CHUB_STATE state)
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
			if (!cipc_noti[i].nb)
				continue;

			if (cipc_noti[i].nb->state) {
				nanohub_dev_info(chub->dev,
						 "%s (%d:%s) call notifier call on chub-reset\n",
						 __func__, i, cipc_noti[i].name);
				cipc_noti[i].nb->state = state;
				ret = cipc_noti[i].nb->notifier_call(cipc_noti[i].nb);
				if (ret)
					nanohub_dev_info(chub->dev,
							 "%s (%d:%s) fails to notifier ret[%x] --\n",
							 __func__, i, cipc_noti[i].name, ret);
			}
		}
		return 0;
	}

	/* find the cipc base of sussystem */
	for (i = IPC_OWN_HOST; i < IPC_OWN_MAX; i++) {
		nanohub_dev_info(chub->dev,
				 "%s (%d:%s) start to find start_off\n",
				 __func__, i, cipc_noti[i].name);
		if ((i == IPC_OWN_MASTER) || (i == IPC_OWN_HOST)) {
			nanohub_dev_info(chub->dev,
					 "%s (%d:%s) skip by master & host\n",
					 __func__, i, cipc_noti[i].name);
			continue;
		}
		if (!cipc_noti[i].nb) {
			nanohub_dev_info(chub->dev,
					 "%s (%d:%s) skip by no notifier\n",
					 __func__, i, cipc_noti[i].name);
			continue;
		}

		cipc_get_offset_owner(i, &cipc_start_offset, &cipc_size);
		if (cipc_start_offset) {
			nanohub_dev_info(chub->dev,
					 "%s (%d:%s) get start_off:+%x, size:%d\n",
					 __func__, i, cipc_noti[i].name,
					 cipc_start_offset, cipc_size);

			cipc_base_offset = cipc_get_base(CIPC_REG_CIPC_BASE) - chub->sram;
			ipc_base_offset = ipc_get_base(IPC_REG_IPC) - chub->sram;
			/* offset check: offset should be 4KB align. And it's bigger than cipc_base. and it's smaller than ipc_end */
			if (((cipc_start_offset % IPC_ALIGN_SIZE) == 0) &&
			    ((cipc_size % IPC_ALIGN_SIZE) == 0) &&
			    (cipc_base_offset <= cipc_start_offset) &&
			    ((cipc_start_offset + cipc_size) <=
			    (ipc_base_offset + ipc_get_size(IPC_REG_IPC)))) {
				cipc_noti[i].nb->start_off = cipc_start_offset;
				cipc_noti[i].nb->end_off = cipc_start_offset + cipc_size;
				nanohub_dev_info(chub->dev,
						 "%s (%d:%s) fill notifier:start_off:+%x, end_off:+%x\n",
						 __func__, i, cipc_noti[i].name,
						 cipc_noti[i].nb->start_off,
						 cipc_noti[i].nb->end_off);
				find = 1;
			} else {
				nanohub_dev_err(chub->dev,
						"%s (%d:%s) invalid start_off:+%x(align:%d), end_off:+%x(align:%d), cipc_base_off:+%x, ipc_end_off:+%x\n",
						__func__, i, cipc_noti[i].name,
						cipc_start_offset, cipc_start_offset % IPC_ALIGN_SIZE,
						cipc_start_offset + cipc_size,
						(cipc_start_offset + cipc_size) % IPC_ALIGN_SIZE,
						cipc_base_offset, ipc_base_offset + ipc_get_size((IPC_REG_IPC)));
			}
		}
	}

	/* call cipc notifiers */
	if (find) {
		for (i = IPC_OWN_HOST; i < IPC_OWN_MAX; i++) {
			/* Check Notifier call pointer exist */
			if (!cipc_noti[i].nb) {
				nanohub_dev_info(chub->dev,
						 "%s (%d:%s) doesn't have notifier call\n",
						 __func__, i, cipc_noti[i].name);
				continue;
			}
			nanohub_dev_info(chub->dev,
					 "%s (%d:%s) start_off:+%x, end_off:+%x\n",
					 __func__, i, cipc_noti[i].name,
					 cipc_noti[i].nb->start_off,
					 cipc_noti[i].nb->end_off);
			if (cipc_noti[i].nb->start_off) {
				cipc_noti[i].nb->state = state;
				ret = cipc_noti[i].nb->notifier_call(cipc_noti[i].nb);
				if (ret)
					nanohub_dev_info(chub->dev,
							 "%s (%d:%s) fails to notifier ret[%x] --\n",
							 __func__, i,
							 cipc_noti[i].name,
							 ret);
			}
		}
	}
	return ret;
}

int contexthub_notifier_register(struct contexthub_notifier_block *nb)
{
	int index;

	if (!nb) {
		pr_err("%s: subsystem notifier block is NULL pointer\n",
		       __func__);
		return -EINVAL;
	}
	if (!nb->subsystem || !nb->notifier_call) {
		pr_err("%s: subsystem is NULL pointer\n", __func__);
		return -EINVAL;
	}

	for (index = IPC_OWN_HOST; index < IPC_OWN_MAX; index++) {
		if (!strncmp
		    (cipc_noti[index].name, nb->subsystem, IPC_NAME_MAX))
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

static int contexthub_ipc_probe(struct platform_device *pdev)
{
	struct contexthub_ipc_info *chub;
	int need_to_free = 0;
	int ret = 0;
	int i;

	chub = chub_dbg_get_memory(pdev->dev.of_node);
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

	chub->dev = &pdev->dev;
	dev_info(chub->dev, "%s chub probe", __func__);
	/* parse dt and hw init */
	ret = contexthub_dt_init(pdev, chub);
	if (ret) {
		nanohub_dev_err(&pdev->dev, "%s failed to get init hw with ret %d\n",
			__func__, ret);
		goto err;
	}

	ret = contexthub_blk_poweron(chub);
	if (ret) {
		nanohub_dev_err(&pdev->dev, "%s block poweron failed\n", __func__);
		goto err;
	}

	spin_lock_init(&chub->logout_lock);
	atomic_set(&chub->chub_status, CHUB_ST_NO_POWER);
	chub->powermode = 0; /* updated by fw bl */
	chub->cur_err = 0;
	for (i = 0; i < CHUB_ERR_MAX; i++)
		chub->err_cnt[i] = 0;
	platform_set_drvdata(pdev, chub);
#ifdef CONFIG_EXYNOS_ITMON
	/* itmon notifier */
	chub->itmon_nb.notifier_call = contexthub_itmon_notifier;
	itmon_notifier_chain_register(&chub->itmon_nb);
#endif
	/* panic notifier */
	chub->panic_nb.notifier_call = contexthub_panic_handler;
	atomic_notifier_chain_register(&panic_notifier_list, &chub->panic_nb);

	/* chub power & reset notifier */
	chub->chub_cipc_nb.subsystem = "CHUB";
	chub->chub_cipc_nb.notifier_call = contexthub_notifier;
	contexthub_notifier_register(&chub->chub_cipc_nb);

#ifdef CONFIG_EXYNOS_SYSTEM_EVENT
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
		nanohub_dev_err(&pdev->dev, "%s: failed to register sysevent:%d\n",
				__func__, ret);
		goto err;
	}
#endif
	ret = contexthub_log_init(chub);
	if (ret) {
		nanohub_dev_err(&pdev->dev, "%s log init is fail with ret %d\n", __func__, ret);
		goto err;
	}

	ret = contexthub_ipc_if_init(chub);
	if (ret) {
		nanohub_dev_err(&pdev->dev, "%s ipc_if init is fail with ret %d\n", __func__, ret);
		goto err;
	}

	ret = contexthub_bootup_init(chub);
	if (ret) {
		nanohub_dev_err(&pdev->dev, "%s bootup init is fail with ret %d\n", __func__, ret);
		goto err;
	}

	nanohub_dev_info(&pdev->dev, "%s is done. ret:%d, sensor_drv:%s\n",
			__func__, ret, chub->data ? "on" : "off");
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

static int contexthub_alive_noirq(struct contexthub_ipc_info *ipc)
{
	int cnt = 100;
	int start_index;
	unsigned int status;
	int irq_num = IRQ_NUM_CHUB_ALIVE + start_index;
	struct cipc_info *cipc;
	enum ipc_mb_id rcv_mb_id;
	void *base;

	cipc = ipc->chub_ipc->cipc;
	rcv_mb_id = cipc->user_info[CIPC_USER_CHUB2AP].map_info.dst.mb_id;
	status = ipc_hw_read_int_status_reg_all(ipc->mailbox, rcv_mb_id);
	start_index = ipc_hw_start_bit(rcv_mb_id);
	irq_num = IRQ_NUM_CHUB_ALIVE + start_index;
	base = cipc->user_info[CIPC_USER_CHUB2AP].mb_base;

	ipc_write_hw_value(IPC_VAL_HW_AP_STATUS, AP_WAKE);
	ipc_hw_gen_interrupt(ipc->mailbox, ipc->chub_ipc->opp_mb_id, IRQ_NUM_CHUB_ALIVE);

	atomic_set(&ipc->chub_alive_lock.flag, 0);
	while(cnt--) {
		mdelay(1);
		status = ipc_hw_read_int_status_reg_all(base, rcv_mb_id);
		if (status & (1 << irq_num)) {
			ipc_hw_clear_int_pend_reg(base, rcv_mb_id, irq_num);
			atomic_set(&ipc->chub_alive_lock.flag, 1);
			return 0;
		}
	}
	return -1;
}

static int contexthub_resume_noirq(struct device *dev)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);

 	if (atomic_read(&ipc->chub_status) != CHUB_ST_RUN)
		return 0;

	contexthub_alive_noirq(ipc);
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static int contexthub_suspend_noirq(struct device *dev)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);

	if (atomic_read(&ipc->chub_status) != CHUB_ST_RUN)
	return 0;

	dev_info(dev, "%s: send ap sleep\n", __func__);
	ipc_write_hw_value(IPC_VAL_HW_AP_STATUS, AP_SLEEP);
	ipc_hw_gen_interrupt(ipc->mailbox, ipc->chub_ipc->opp_mb_id, IRQ_NUM_CHUB_ALIVE);
	dev_info(dev, "%s: out\n", __func__);
	return 0;
}

static int contexthub_suspend(struct device *dev)
{
	struct contexthub_ipc_info *chub = dev_get_drvdata(dev);
	struct ipc_info *chub_ipc = chub->chub_ipc;

	if (atomic_read(&chub->chub_status) != CHUB_ST_RUN)
		return 0;

#ifdef CONFIG_CONTEXTHUB_SENSOR_DEBUG
#ifdef ALIVE_WORK
	cancel_delayed_work(&chub->sensor_alive_work);
	chub->sensor_alive_work_run = false;
#endif
#endif

	nanohub_dev_info(dev, "%s: irq-pend:ap:%x,chub:%x\n", __func__,
		ipc_hw_read_int_status_reg_all(chub->mailbox, chub_ipc->my_mb_id),
		ipc_hw_read_int_status_reg_all(chub->mailbox, chub_ipc->opp_mb_id));
	atomic_set(&chub->chub_sleep, 1);
	return 0;
}

static int contexthub_resume(struct device *dev)
{
	struct contexthub_ipc_info *chub = dev_get_drvdata(dev);
	struct ipc_info *chub_ipc = chub->chub_ipc;

	if (atomic_read(&chub->chub_status) != CHUB_ST_RUN)
		return 0;
#ifdef CONFIG_CONTEXTHUB_SENSOR_DEBUG
#ifdef ALIVE_WORK
	if (chub->sensor_alive_work_run == false) {
		dev_info(dev, "%s: schedule sensor_alive_work\n", __func__);
		schedule_delayed_work(&chub->sensor_alive_work, msecs_to_jiffies(1000));
	}
#endif
#endif
	nanohub_dev_info(dev, "%s: irq-pend:ap:%x,chub:%x\n", __func__,
		ipc_hw_read_int_status_reg_all(chub->mailbox, chub_ipc->my_mb_id),
		ipc_hw_read_int_status_reg_all(chub->mailbox, chub_ipc->opp_mb_id));
	atomic_set(&chub->chub_sleep, 0);
	return 0;
}

static const struct dev_pm_ops contexthub_pm_ops = {
	.suspend = contexthub_suspend,
	.suspend_noirq = contexthub_suspend_noirq,
	.resume = contexthub_resume,
	.resume_noirq = contexthub_resume_noirq,
};

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

int contexthub_init(void)
{
#ifdef CONFIG_SENSOR_DRV
	nanohub_init();
#endif
	return platform_driver_register(&samsung_contexthub_ipc_driver);
}

static void __exit contexthub_cleanup(void)
{
#ifdef CONFIG_SENSOR_DRV
	nanohub_cleanup();
#endif
	platform_driver_unregister(&samsung_contexthub_ipc_driver);
}

module_init(contexthub_init);
module_exit(contexthub_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Exynos contexthub mailbox Driver");
MODULE_AUTHOR("Boojin Kim <boojin.kim@samsung.com>");
