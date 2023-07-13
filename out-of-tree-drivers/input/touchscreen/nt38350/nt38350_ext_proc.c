/*
 * Copyright (C) 2020 Novatek, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */


#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "nt38350.h"

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"

#if GOOGLE_TOUCH_EXT_PROC
#define NVT_RESET "nvt_reset"
#define NVT_SCAN_MODE "nvt_scan_mode"
#define NVT_SENSING "nvt_sensing"
#define NVT_HOPPING_FREQ "nvt_hopping_freq"
#define NVT_TRIMID "nvt_trimid"
#define NVT_HISTORY_EVENT "nvt_history_event"
#endif

#define BUS_TRANSFER_LENGTH  64

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB

#define XDATA_SECTOR_SIZE   256

static uint8_t xdata_tmp[2048] = {0};
static int32_t xdata[2048] = {0};

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;

#if GOOGLE_TOUCH_EXT_PROC
static struct proc_dir_entry *NVT_proc_reset_entry;
static struct proc_dir_entry *NVT_proc_scan_mode_entry;
static struct proc_dir_entry *NVT_proc_sensing_entry;
static struct proc_dir_entry *NVT_proc_hopping_freq_entry;
static struct proc_dir_entry *NVT_proc_trimid_entry;
static struct proc_dir_entry *NVT_proc_history_event_entry;
#endif
/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void nvt_change_mode(uint8_t mode)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

	if (mode == NORMAL_MODE) {
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
		msleep(20);
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t nvt_get_fw_pipe(void)
{
	uint8_t buf[8]= {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	//---read fw status---
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

	//NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[BUS_TRANSFER_LENGTH + 1] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	//---set xdata sector address & length---
	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = ts->x_num * ts->y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

	//read xdata : step 1
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		//---change xdata index---
		nvt_set_page(I2C_FW_Address, head_addr + XDATA_SECTOR_SIZE * i);

		//---read xdata by BUS_TRANSFER_LENGTH
		for (j = 0; j < (XDATA_SECTOR_SIZE / BUS_TRANSFER_LENGTH); j++) {
			//---read data---
			buf[0] = BUS_TRANSFER_LENGTH * j;
			CTP_I2C_READ(ts->client, I2C_FW_Address, buf, BUS_TRANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < BUS_TRANSFER_LENGTH; k++) {
				xdata_tmp[XDATA_SECTOR_SIZE * i + BUS_TRANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04X\n", buf[k+1], (XDATA_SECTOR_SIZE*i + BUS_TRANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (head_addr+XDATA_SECTOR_SIZE*i));
	}

	//read xdata : step2
	if (residual_len != 0) {
		//---change xdata index---
		nvt_set_page(I2C_FW_Address, xdata_addr + data_len - residual_len);

		//---read xdata by BUS_TRANSFER_LENGTH
		for (j = 0; j < (residual_len / BUS_TRANSFER_LENGTH + 1); j++) {
			//---read data---
			buf[0] = BUS_TRANSFER_LENGTH * j;
			CTP_I2C_READ(ts->client, I2C_FW_Address, buf, BUS_TRANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < BUS_TRANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) + BUS_TRANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04x\n", buf[k+1], ((dummy_len+data_len-residual_len) + BUS_TRANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (xdata_addr+data_len-residual_len));
	}

	//---remove placeholder data and 2bytes-to-1data---
	for (i = 0; i < (data_len / 2); i++) {
		xdata[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
	}

#if TOUCH_KEY_NUM > 0
	//read button xdata : step3
	//---change xdata index---
	nvt_set_page(I2C_FW_Address, xdata_btn_addr);

	//---read data---
	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, (TOUCH_KEY_NUM * 2 + 1));

	//---2bytes-to-1data---
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[ts->x_num * ts->y_num + i] = (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR);
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
*******************************************************/
void nvt_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = ts->x_num;
    *m_y_num = ts->y_num;
    memcpy(buf, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", ts->fw_ver, ts->x_num, ts->y_num, ts->max_button_num);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < ts->y_num; i++) {
		for (j = 0; j < ts->x_num; j++) {
			seq_printf(m, "%d, ", xdata[i * ts->x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%d, ", xdata[ts->x_num * ts->y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

#if GOOGLE_TOUCH_EXT_PROC
#define CMD_CHANGE_SCAN 0x1F
#define CMD_DOZE_MODE 0x30
#define CMD_GESTURE_MODE 0x31
#define CMD_FDM_MODE 0x32
#define CMD_HOPPING_FREQ1 0x41
#define CMD_HOPPING_FREQ2 0x42
#define CMD_ACTVE_MODE 0x66
#define INIT_MODE 0x00
#define ACTVE_MODE 0x40

#define SCAN_MODE_STATUS_ADDR 0x20AA0

int32_t nvt_customizeCmd_store(uint8_t u8Cmd, uint8_t u8SubCmd, uint16_t len) {
	uint8_t buf[4] = {0};
	int32_t ret = 0;
	int32_t i, retry = 5;


	//---set xdata index to EVENT BUF ADDR---
	ret = nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	if (ret < 0) {
		NVT_ERR("Set event buffer index fail!\n");
		return ret;
	}

	for(i = 0; i < retry; i++){
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		buf[2] = u8SubCmd;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, len);

		msleep(20);

		//---read cmd status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(i == retry)) {
		NVT_ERR("send Cmd 0x%02X, 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, u8SubCmd, buf[1]);
		ret = -1;
	} else {
		NVT_LOG("send Cmd 0x%02X, 0x%02X success, tried %d times\n", u8Cmd, u8SubCmd, i);
	}

	return  ret;
}


uint8_t nvt_customizeCmd_show(int32_t start_addr) {
	uint8_t buf[4] = {0};
	uint8_t result = 0;

	//---set xdata index to start_addr---
	nvt_set_page(I2C_FW_Address, start_addr);

	//---read cmd status---
	buf[0] = start_addr & 0xFF;
	buf[1] = 0xFF;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
	result = buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR);

	return  result;
}


/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_reset. (Read only)

return:
	Reset OK!
*******************************************************/
static int nvt_reset_show(struct seq_file *sfile, void *v) {
       seq_printf(sfile, "Reset OK!\n");
       return 0;
}

static int32_t nvt_reset_open(struct inode *inode, struct file *file)
{
	NVT_LOG("++\n");

	//---Reset IC---
	nvt_bootloader_reset();

	NVT_LOG("--\n");

	return single_open(file, &nvt_reset_show, NULL);
}

static const struct file_operations nvt_reset_fops = {
	.owner = THIS_MODULE,
	.open = nvt_reset_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_scan_mode.

return:
Read:
	fw current scan mode
Write:
	1 : tp enter active mode
	2 : tp enter doze mode
	3 : tp enter standby mode finger on
	4 : tp enter standby mode finger off
*******************************************************/
static int nvt_scan_mode_show(struct seq_file *sfile, void *v) {
	uint8_t scan_mode;

	scan_mode = nvt_customizeCmd_show(SCAN_MODE_STATUS_ADDR);

	switch (scan_mode) {
		case INIT_MODE:
			seq_printf(sfile, "Initial value\n");
			break;
		case ACTVE_MODE:
			seq_printf(sfile, "Active mode\n");
			break;
		case CMD_DOZE_MODE:
			seq_printf(sfile, "Doze mode\n");
			break;
		case CMD_GESTURE_MODE:
			seq_printf(sfile, "Standby mode finger on\n");
			break;
		case CMD_FDM_MODE:
			seq_printf(sfile, "Standby mode finger off\n");
			break;
		case CMD_HOPPING_FREQ1:
			seq_printf(sfile, "Active mode & Hopping Freq1\n");
			break;
		case CMD_HOPPING_FREQ2:
			seq_printf(sfile, "Active mode & Hopping Freq2\n");
			break;
		default:
			seq_printf(sfile, "%02x Invalid!\n", scan_mode);
			break;
	}

    return 0;
}

static int32_t nvt_scan_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, &nvt_scan_mode_show, NULL);
}

static ssize_t nvt_scan_mode_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int input = 0;
	char cmd[128] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if(count > 2) {
		NVT_ERR("input value error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if(copy_from_user(cmd, buffer, count)) {
		NVT_ERR("copy value from user error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &input)) {
		NVT_ERR("kstrtouint error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_NORMAL_RUN);

	switch (input) {
		case 1: //Active mode
			nvt_customizeCmd_store(CMD_ACTVE_MODE,0x00,2);
			break;
		case 2://Doze mode
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_DOZE_MODE,3);
			break;
		case 3://Standby mode finger on
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_GESTURE_MODE,3);
			break;
		case 4://Standby mode finger off
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_FDM_MODE,3);
			break;
		default:
			NVT_ERR("%u not supported\n", input);
			break;

	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return count;

}
static const struct file_operations nvt_scan_mode_fops = {
	.owner = THIS_MODULE,
	.open = nvt_scan_mode_open,
	.write = nvt_scan_mode_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};


/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_hopping_freq. (read/write)

return:
Read:
	fw current scan mode
Write:
	1 : hopping active frequency 1
	2 : hopping active frequency 2
*******************************************************/
static int nvt_hopping_freq_show(struct seq_file *sfile, void *v) {
	uint8_t scan_mode;

	scan_mode = nvt_customizeCmd_show(SCAN_MODE_STATUS_ADDR);

	switch (scan_mode) {
		case INIT_MODE:
			seq_printf(sfile, "Initial value\n");
			break;
		case ACTVE_MODE:
			seq_printf(sfile, "Active mode\n");
			break;
		case CMD_DOZE_MODE:
			seq_printf(sfile, "Doze mode\n");
			break;
		case CMD_GESTURE_MODE:
			seq_printf(sfile, "Standby mode finger on\n");
			break;
		case CMD_FDM_MODE:
			seq_printf(sfile, "Standby mode finger off\n");
			break;
		case CMD_HOPPING_FREQ1:
			seq_printf(sfile, "Active mode & Hopping Freq1\n");
			break;
		case CMD_HOPPING_FREQ2:
			seq_printf(sfile, "Active mode & Hopping Freq2\n");
			break;
		default:
			seq_printf(sfile, "%02x Invalid!\n", scan_mode);
			break;
	}

    return 0;
}

static int32_t nvt_hopping_freq_open(struct inode *inode, struct file *file)
{

	return single_open(file, &nvt_hopping_freq_show, NULL);
}

static ssize_t nvt_hopping_freq_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int input = 0;
	char cmd[128] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if(count > 2) {
		NVT_ERR("input value error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if(copy_from_user(cmd, buffer, count)) {
		NVT_ERR("copy value from user error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &input)) {
		NVT_ERR("kstrtouint error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_NORMAL_RUN);

	switch (input) {
		case 1:
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_HOPPING_FREQ1,3);
			break;
		case 2:
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_HOPPING_FREQ2,3);
			break;
		default:
			NVT_ERR("%u not supported\n", input);
			break;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return count;
}
static const struct file_operations nvt_hopping_freq_fops = {
	.owner = THIS_MODULE,
	.open = nvt_hopping_freq_open,
	.write = nvt_hopping_freq_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_sensing. (write only)

return:
	0 : sensing off, touch enter power down
	1 : sensing on
*******************************************************/
static ssize_t nvt_sensing_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int input = 0;
	char cmd[4] = {0};
	uint8_t buf[4] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if(count > 2) {
		NVT_ERR("input value error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if(copy_from_user(cmd, buffer, count)) {
		NVT_ERR("copy value from user error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &input)) {
		NVT_ERR("kstrtouint error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	switch (input) {
		case 0:
			//---set xdata index to EVENT BUF ADDR---
			nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

			buf[0] = EVENT_MAP_HOST_CMD;
			buf[1] = 0x12;
			CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
			NVT_LOG("Touch Enter PD\n");
			break;
		case 1:
			nvt_bootloader_reset();
			break;
		default:
			NVT_ERR("%u not supported\n", input);
			break;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return count;
}
static const struct file_operations nvt_sensing_fops = {
	.write = nvt_sensing_write,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_trimid. (read only)

return:
	novatek chip id
*******************************************************/
static int nvt_trimid_show(struct seq_file *sfile, void *v) {
	   seq_printf(sfile, "%s\n", ts->trimid);
       return 0;
}

static int32_t nvt_trimid_open(struct inode *inode, struct file *file)
{
	return single_open(file, &nvt_trimid_show, NULL);
}

static const struct file_operations nvt_trimid_fops = {
	.owner = THIS_MODULE,
	.open = nvt_trimid_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_history_event. (read/write)
return:
Read:
	fw history event log
	@0x209C8 1 bytes
	@0x20A00 64 bytes
	@0x20A50 64 bytes
Write:
	0:inital fw history log
*******************************************************/
#define HISTORY_EVENT_ADDR1 0x20A00
#define HISTORY_EVENT_ADDR2 0x20A50
#define HISTORY_WDT_ADDR 0x209C8
static int32_t c_history_event_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	seq_printf(m, "Addr:[0x%5X]\n", HISTORY_WDT_ADDR);
	seq_printf(m, "%2X\n", xdata_tmp[0]);

	seq_printf(m, "Addr:[0x%5X]\n", HISTORY_EVENT_ADDR1);
	for (i = 0; i < 2; i++) {
		if(i == 1) {
			seq_printf(m, "Addr:[0x%5X]\n", HISTORY_EVENT_ADDR2);
		}
		for (j = 0; j < 64; j++) {
			seq_printf(m, "%2X, ", xdata_tmp[1 + i * 64 + j]);
		}
		seq_puts(m, "\n");
	}

	seq_printf(m, "\n\n");
	return 0;
}

const struct seq_operations nvt_history_event_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_history_event_show
};

static int32_t nvt_history_event_open(struct inode *inode, struct file *file)
{
	int32_t i = 0;
	int32_t j = 0;
	uint8_t buf[BUS_TRANSFER_LENGTH + 1] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	//---read WDT log---
	nvt_set_page(I2C_FW_Address, HISTORY_WDT_ADDR);
	buf[0] = HISTORY_WDT_ADDR & 0xFF;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
	xdata_tmp[j++] = buf[1];

	//---read history event buffer 1---
	nvt_set_page(I2C_FW_Address, HISTORY_EVENT_ADDR1);
	buf[0] = HISTORY_EVENT_ADDR1 & 0xFF;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, BUS_TRANSFER_LENGTH + 1);

	//---copy buf to xdata_tmp---
	for (i = 0; i < BUS_TRANSFER_LENGTH; i++) {
		xdata_tmp[j++] = buf[i + 1];
	}

	//---read history event buffer 2---
	buf[0] = HISTORY_EVENT_ADDR2 & 0xFF;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, BUS_TRANSFER_LENGTH + 1);

	//---copy buf to xdata_tmp---
	for (i = 0; i < BUS_TRANSFER_LENGTH; i++) {
		xdata_tmp[j++] = buf[i + 1];
	}

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_history_event_seq_ops);
}

static ssize_t nvt_hitstory_event_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int input = 0;
	char cmd[4] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if(count > 2) {
		NVT_ERR("input value error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if(copy_from_user(cmd, buffer, count)) {
		NVT_ERR("copy value from user error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &input)) {
		NVT_ERR("kstrtouint error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	switch (input) {
		case 0:
			nvt_bootloader_reset();
			break;
		default:
			NVT_ERR("%u not supported\n", input);
			break;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return count;
}

static const struct file_operations nvt_history_event_fops = {
	.owner = THIS_MODULE,
	.write = nvt_hitstory_event_write,
	.open = nvt_history_event_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif
/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
int32_t nvt_extra_proc_init(void)
{
	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL,&nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_FW_VERSION);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_FW_VERSION);
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL,&nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_BASELINE);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_BASELINE);
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL,&nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_RAW);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_RAW);
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL,&nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_DIFF);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_DIFF);
	}

#if GOOGLE_TOUCH_EXT_PROC
	NVT_proc_reset_entry = proc_create(NVT_RESET, 0444, NULL,&nvt_reset_fops);
	if (NVT_proc_reset_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_RESET);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_RESET);
	}

	NVT_proc_scan_mode_entry = proc_create(NVT_SCAN_MODE, 0644, NULL,&nvt_scan_mode_fops);
	if (NVT_proc_scan_mode_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_SCAN_MODE);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_SCAN_MODE);
	}

	NVT_proc_hopping_freq_entry = proc_create(NVT_HOPPING_FREQ, 0644, NULL,&nvt_hopping_freq_fops);
	if (NVT_proc_hopping_freq_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_HOPPING_FREQ);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_HOPPING_FREQ);
	}

	NVT_proc_sensing_entry = proc_create(NVT_SENSING, 0222, NULL,&nvt_sensing_fops);
	if (NVT_proc_sensing_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_SENSING);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_SENSING);
	}

	NVT_proc_trimid_entry = proc_create(NVT_TRIMID, 0444, NULL,&nvt_trimid_fops);
	if (NVT_proc_trimid_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_TRIMID);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_TRIMID);
	}

	NVT_proc_history_event_entry = proc_create(NVT_HISTORY_EVENT, 0644, NULL,&nvt_history_event_fops);
	if (NVT_proc_history_event_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_HISTORY_EVENT);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_HISTORY_EVENT);
	}
#endif
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	deinitial function.

return:
	n.a.
*******************************************************/
void nvt_extra_proc_deinit(void)
{
	if (NVT_proc_fw_version_entry != NULL) {
		remove_proc_entry(NVT_FW_VERSION, NULL);
		NVT_proc_fw_version_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_FW_VERSION);
	}

	if (NVT_proc_baseline_entry != NULL) {
		remove_proc_entry(NVT_BASELINE, NULL);
		NVT_proc_baseline_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_BASELINE);
	}

	if (NVT_proc_raw_entry != NULL) {
		remove_proc_entry(NVT_RAW, NULL);
		NVT_proc_raw_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_RAW);
	}

	if (NVT_proc_diff_entry != NULL) {
		remove_proc_entry(NVT_DIFF, NULL);
		NVT_proc_diff_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_DIFF);
	}
#if GOOGLE_TOUCH_EXT_PROC
	if (NVT_proc_reset_entry != NULL) {
		remove_proc_entry(NVT_RESET, NULL);
		NVT_proc_reset_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_RESET);
	}

	if (NVT_proc_scan_mode_entry != NULL) {
		remove_proc_entry(NVT_SCAN_MODE, NULL);
		NVT_proc_scan_mode_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_SCAN_MODE);
	}

	if (NVT_proc_hopping_freq_entry != NULL) {
		remove_proc_entry(NVT_HOPPING_FREQ, NULL);
		NVT_proc_hopping_freq_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_HOPPING_FREQ);
	}

	if (NVT_proc_sensing_entry != NULL) {
		remove_proc_entry(NVT_SENSING, NULL);
		NVT_proc_sensing_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_SENSING);
	}

	if (NVT_proc_trimid_entry != NULL) {
		remove_proc_entry(NVT_TRIMID, NULL);
		NVT_proc_trimid_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_TRIMID);
	}

	if (NVT_proc_history_event_entry != NULL) {
		remove_proc_entry(NVT_HISTORY_EVENT, NULL);
		NVT_proc_history_event_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_HISTORY_EVENT);
	}
#endif
}
#endif
