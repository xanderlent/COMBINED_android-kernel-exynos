/* drivers/input/misc/ots_pat9126/pat9126_linux_driver.c
 *
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 */

#include <linux/input.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include "pat9126.h"
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <asm/uaccess.h>

#include <linux/notifier.h>
#include <linux/reboot.h>
#include<linux/time.h>
#include <linux/rtc.h>
#include <linux/string.h>

// TODO(b/192250990): Do not re-enable PM ops before removing the call to
// 'flush_scheduled_work' from pat9126_suspend.
//#define PAT9126_PM_OPS

struct pixart_pat9126_data {
	struct i2c_client *client;
	struct input_dev *input;
	int irq;
	u32	irq_flags;
	struct device *pat9126_device;
	u32 press_keycode;
	bool press_en;
	bool inverse_x;
	bool inverse_y;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct delayed_work polling_work;
	struct delayed_work resume_work;
};

/*IRQ Flags*/
bool is_initialized = false;

/*Persist Flags*/
extern char *saved_command_line;

struct mutex irq_mutex;
int en_irq_cnt = 0; /*Calculate times of enable irq*/
int dis_irq_cnt = 0;/*Calculate times of disable irq*/

struct rtc_time pat9126_tm;

struct rw_reg_info {
	char flag; /*R/W char*/
	long w_addr;
	long r_addr;
	long r_data;
};

struct rw_reg_info pat9126_reg_info;

#if defined(PAT9126_PM_OPS)
/* Declaration of suspend and resume functions */
static int pat9126_suspend(struct device *dev);
static int pat9126_resume(struct device *dev);
#else
static int pat9126_enable_irq_wake(struct device *dev);
static int pat9126_disable_irq_wake(struct device *dev);
#endif

static int pat9126_write(struct i2c_client *client, u8 addr, u8 data)
{
	u8 buf[BUF_SIZE];
	struct device *dev = &client->dev;

	buf[0] = addr;
	buf[1] = data;

	/* Returns negative errno, or else the number of bytes written. */
	if (i2c_master_send(client, buf, BUF_SIZE) < 0) {
		dev_err(dev, "%s Failed: writing to reg 0x%x\n", __func__, addr);
		return -EIO;
	}

	return 0;
}

static int pat9126_read(struct i2c_client *client, u8 addr, u8 *data)
{
	struct device *dev = &client->dev;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &addr,
		}, {
		 .addr = client->addr,
		 .flags = 1,
		 .len = 1,
		 .buf = data,
		}
	};

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		dev_err(dev, "%s Failed: writing to reg 0x%x\n", __func__,
			addr);
		return -EIO;
	}

	return 0;
}

static int pat9126_write_verified(struct i2c_client *client, u8 address, u8 data)
{
	int i, ret;
	u8 read_value;

	for (i = 0; i < PAT9150_I2C_RETRY; i++) {
		ret = pat9126_write(client, address, data);
		if (ret < 0)
			return ret;
		ret = pat9126_read(client, address, &read_value);
		if (ret < 0)
			return ret;

		if (read_value == data)
			return 0;

		msleep(1);
	}

	return -EIO;
}

void delay(int ms)
{
	msleep(ms);
}

bool pat9126_sensor_init(struct i2c_client *client)
{
	u8 id;
	int ret = 0;

	/*
	 * Read sensor_pid in address 0x00 to check if the
	 * serial link is valid, read value should be 0x31.
	 */
	pat9126_read(client, PIXART_PAT9126_PRODUCT_ID1_REG, &id);
	if (id != PIXART_PAT9126_SENSOR_ID) {
		return false;
	}

	/*
	 * PAT9126 sensor recommended settings:
	 * switch to bank0, not allowed to perform pat9126_write_verified
	 */
	pat9126_write(client, PIXART_PAT9126_SELECT_BANK_REG,
			PIXART_PAT9126_BANK0);
	/*
	 * software reset (i.e. set bit7 to 1).
	 * It will reset to 0 automatically
	 * so perform OTS_RegWriteRead is not allowed.
	 */
	pat9126_write(client, PIXART_PAT9126_CONFIG_REG,
			PIXART_PAT9126_RESET);

	/* delay 1ms */
	usleep_range(RESET_DELAY_US, RESET_DELAY_US + 1);

	/* disable write protect */
	if (pat9126_write_verified(client, PIXART_PAT9126_WRITE_PROTECT_REG,
				PIXART_PAT9126_DISABLE_WRITE_PROTECT) < 0)
		return false;
	/* set X-axis resolution (depends on application) */
	if (pat9126_write_verified(client, PIXART_PAT9126_SET_CPI_RES_X_REG,
		PIXART_PAT9126_CPI_RESOLUTION_X) < 0)
		return false;
	/* set Y-axis resolution (depends on application) */
	if (pat9126_write_verified(client, PIXART_PAT9126_SET_CPI_RES_Y_REG,
				PIXART_PAT9126_CPI_RESOLUTION_Y) < 0)
		return false;
	/* set 12-bit X/Y data format (depends on application) */
	if (pat9126_write_verified(client, PIXART_PAT9126_ORIENTATION_REG,
				PIXART_PAT9126_MOTION_DATA_LENGTH) < 0)
		return false;
	/* ONLY for VDD=VDDA=1.7~1.9V: for power saving */
	if (pat9126_write_verified(client, PIXART_PAT9126_VOLTAGE_SEGMENT_SEL_REG,
				PIXART_PAT9126_LOW_VOLTAGE_SEGMENT) < 0)
		return false;

	pat9126_write_verified(client, PIXART_PAT9126_SENSOR_MODE_SELECT_REG,
			PIXART_PAT9126_SENSOR_SET_MODE2);

	pat9126_write_verified(client, PIXART_PAT9126_AE_ENABLE, PIXART_PAT9126_AE_ENABLE_VAL);
	pat9126_write_verified(client, PIXART_PAT9126_NY_MIN, PIXART_PAT9126_NY_MIN_VAL);

	ret = pat9126_enable_mot(client);
	if (ret < 0) {
		pr_err("[PAT9126]: Enable Motion FAIL.");
	}

	/* enable write protect */
	pat9126_write_verified(client, PIXART_PAT9126_WRITE_PROTECT_REG,
			PIXART_PAT9126_ENABLE_WRITE_PROTECT);
	return ret;
}

int pat9126_disable_mot(struct i2c_client *client, int16_t detect_freq)
{
	uint8_t tmp_1 = 0;
	uint8_t sensor_pid = 0;

	pat9126_read(client, PIXART_PAT9126_PRODUCT_ID1_REG, &sensor_pid);
	if (sensor_pid != PIXART_PAT9126_SENSOR_ID) {
		return (-1);
	}

	pat9126_write_verified(client, PIXART_PAT9126_SENSOR_MODE_SELECT_REG,
		PIXART_PAT9126_SENSOR_DEFAULT_MODE2); // Set motion to open drain
	pat9126_read(client, PIXART_PAT9126_SENSOR_MODE_SELECT_REG, &tmp_1);
	pr_debug("[PAT9126]: Open drain mode motion: 0x%2x. \n", tmp_1);

	/*Switch to bank1*/
	pat9126_write(client, PIXART_PAT9126_SELECT_BANK_REG,
		PIXART_PAT9126_SELECT_BANK_VAL2);

	pat9126_write_verified(client, PIXART_PAT9126_BANK_FTWK,
		PIXART_PAT9126_BANK_FTWK_VAL1);

	pat9126_write_verified(client, PIXART_PAT9126_BANK_FTWK_D2,
		PIXART_PAT9126_BANK_FTWK_D2_VAL1);

	pat9126_write_verified(client, PIXART_PAT9126_BANK_CTB,
		PIXART_PAT9126_BANK_CTB_VAL1);

	pat9126_write_verified(client, PIXART_PAT9126_BANK_HI_SAD_K,
		PIXART_PAT9126_BANK_HI_SAD_K_VAL1);

	pat9126_write(client, PIXART_PAT9126_SELECT_BANK_REG,
		PIXART_PAT9126_SELECT_BANK_VAL1);

	delay(1);				  /* delay 1ms */

	pat9126_write_verified(client, PIXART_PAT9126_SLEEP2_MODE_FREQ_REG,
		detect_freq);

	pat9126_write(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG,
		PIXART_PAT9126_FORCE_ENTER_SLEEP2_MODE);

	return 0;
}

int pat9126_enable_mot(struct i2c_client *client)
{
	uint8_t tmp_1 = 0;
	uint8_t sensor_pid = 0;

	pat9126_read(client, PIXART_PAT9126_PRODUCT_ID1_REG, &sensor_pid);
	if (sensor_pid != PIXART_PAT9126_SENSOR_ID) {
		return (-1);
	}

	pat9126_write_verified(client, PIXART_PAT9126_SENSOR_MODE_SELECT_REG,
			PIXART_PAT9126_SENSOR_SET_MODE2); // Set motion to drive mode
	pat9126_read(client, PIXART_PAT9126_SENSOR_MODE_SELECT_REG, &tmp_1);
	pr_debug("[PAT9126]: Drive mode motion: 0x%2x. \n", tmp_1);

	delay(1);				  /* delay 1ms */

	/*Read Register for Pulling Up Motion IRQ*/
	pat9126_read(client, PIXART_PAT9126_MOTION_STATUS_REG, &tmp_1);
	pat9126_read(client, PIXART_PAT9126_DELTA_X_LO_REG, &tmp_1);
	pat9126_read(client, PIXART_PAT9126_DELTA_Y_LO_REG, &tmp_1);
	pat9126_read(client, PIXART_PAT9126_DELTA_XY_HI_REG, &tmp_1);

	/*Write Register for Active Mode*/
	/*Switch to bank1*/
	pat9126_write(client, PIXART_PAT9126_SELECT_BANK_REG,
		PIXART_PAT9126_SELECT_BANK_VAL2);

	pat9126_write_verified(client, PIXART_PAT9126_BANK_FTWK,
		PIXART_PAT9126_BANK_FTWK_DEFAULT_VAL);

	pat9126_write_verified(client, PIXART_PAT9126_BANK_FTWK_D2,
		PIXART_PAT9126_BANK_FTWK_D2_DEFAULT_VAL);

	pat9126_write_verified(client, PIXART_PAT9126_BANK_CTB,
		PIXART_PAT9126_BANK_CTB_DEFAULT_VAL);

	pat9126_write_verified(client, PIXART_PAT9126_BANK_HI_SAD_K,
		PIXART_PAT9126_BANK_HI_SAD_K_DEFAULT_VAL);

	pat9126_write(client, PIXART_PAT9126_SELECT_BANK_REG,
		PIXART_PAT9126_SELECT_BANK_VAL1);

	delay(1);				  /* delay 1ms */

	pat9126_write_verified(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG,
		PIXART_PAT9126_WAKEUP_MODE);
	pat9126_read(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG, &tmp_1);
	pr_debug("[PAT9126]: Enable sleep1 and disable sleep2 mode: 0x%2x. \n", tmp_1);
	return 0;
}

/* Read motion */
void pat9126_read_motion(struct i2c_client *client, int16_t *dx16, int16_t *dy16)
{
	int8_t deltaX_l = 0, deltaY_l = 0, deltaXY_h = 0;
	int16_t deltaX_h = 0, deltaY_h = 0;
	uint8_t motion = 0;

	pat9126_read(client, PIXART_PAT9126_MOTION_STATUS_REG, &motion);
	pr_debug("[pat9126]: Motion BIT: 0x%2x\n", motion);

	if (motion & PIXART_PAT9126_VALID_MOTION_DATA) {
		pat9126_read(client, PIXART_PAT9126_DELTA_X_LO_REG, &deltaX_l);
		pat9126_read(client, PIXART_PAT9126_DELTA_Y_LO_REG, &deltaY_l);
		pat9126_read(client, PIXART_PAT9126_DELTA_XY_HI_REG, &deltaXY_h);

		deltaX_h = (deltaXY_h & 0xF0);
		deltaX_h <<= 4;
		/* 12-bit data convert to 16-bit */
		if (deltaX_h & 0x800)
			deltaX_h |= 0xf000;

		deltaY_h = (deltaXY_h & 0xF);
		deltaY_h <<= 8;
		/* 12-bit data convert to 16-bit */
		if (deltaY_h & 0x800)
			deltaY_h |= 0xf000;
	}

	*dx16 = deltaX_h | deltaX_l;
	*dy16 = deltaY_h | deltaY_l;
}

static void pat9126_work_handler(struct work_struct *work)
{
	int16_t delta_x = 0, delta_y = 0;
	struct delayed_work *dw = to_delayed_work(work);
	struct pixart_pat9126_data *data = \
			container_of(dw, struct pixart_pat9126_data, polling_work);
	struct input_dev *ipdev = data->input;
	struct device *dev = &data->client->dev;

	/* check if MOTION bit is set or not */
	pat9126_read_motion(data->client, &delta_x, &delta_y);
	pr_debug("[PAT9126]delta_x: %d, delta_y: %d\n", delta_x, delta_y);

	/* Inverse x depending upon the device orientation */
	delta_x = (data->inverse_x) ? -delta_x : delta_x;
	/* Inverse y depending upon the device orientation */
	delta_y = (data->inverse_y) ? -delta_y : delta_y;

	dev_dbg(dev, "delta_x = 0x%2x, delta_y = 0x%2x\n",
				delta_x, delta_y);

	if (delta_x != 0) {
		/* Send delta_x as REL_WHEEL for rotation */
		input_report_rel(ipdev, REL_WHEEL, (s8) delta_x);
		input_sync(ipdev);
	}
	enable_irq(data->client->irq);
	pm_relax(dev);
}

static irqreturn_t pat9126_irq(int irq, void *dev_data)
{
	struct pixart_pat9126_data *data = dev_data;
	struct device *dev = &data->client->dev;
	bool result = false;

	disable_irq_nosync(irq);
	pm_stay_awake(dev);
	if (!work_pending(&data->work)) {
		result = schedule_delayed_work(&data->polling_work, msecs_to_jiffies(10));
		if (result == false) {
			/* queue_work fail */
			pr_err("%s:queue_work fail.\n",__func__);
		}
	} else {
		/* work pending */
		pr_err("%s:queue_work pending.\n",__func__);
	}

	return IRQ_HANDLED;
}

static ssize_t pat9126_sensitivity_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count) {
	u8 sensitivity;

	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	if (!kstrtou8(buf, 0, &sensitivity)) {
		dev_dbg(dev, "RES_X: %d, 0x%x\n", sensitivity, sensitivity);
		pat9126_write(client, PIXART_PAT9126_SET_CPI_RES_X_REG, sensitivity);
	}

	return count;
}

static ssize_t pat9126_sensitivity_show(struct device *dev,
					struct device_attribute *attr,
					char *buf) {
	int count = 0;
	uint8_t tmp;

	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	pat9126_read(client, PIXART_PAT9126_SET_CPI_RES_X_REG, &tmp);
	count += sprintf(buf, "0x%2x\n", tmp);

	return count;
}

static ssize_t pat9126_id_show(struct device *dev,
					struct device_attribute *attr,
					char *buf) {
	int count = 0;
	uint8_t maj, min;

	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	pat9126_read(client, PIXART_PAT9126_PRODUCT_ID1_REG, &maj);
	pat9126_read(client, PIXART_PAT9126_PRODUCT_ID2_REG, &min);
	count += sprintf(buf, "0x%2x 0x%2x\n", maj, min);

	return count;
}

static ssize_t pat9126_max_sleep_level_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count) {
	u8 tmp, sleep_mode;

	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	pat9126_read(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG, &sleep_mode);

	if (kstrtou8(buf, 0, &tmp)) {
		return count;
	}

	dev_dbg(dev, "Max sleep level: %u\n", tmp);

	sleep_mode &= ~SLEEP_MODES_ENABLED_MASK;

	if (tmp == 0) {
		sleep_mode |= SLEEP_MODES_ENABLED_ZERO;
	} else if (tmp == 1) {
		sleep_mode |= SLEEP_MODES_ENABLED_ONE;
	} else if (tmp == 2) {
		sleep_mode |= SLEEP_MODES_ENABLED_TWO;
	}

	pat9126_write(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG, sleep_mode);

	return count;
}

static ssize_t pat9126_max_sleep_level_show(struct device *dev,
					struct device_attribute *attr,
					char *buf) {
	int count = 0;
	uint8_t sleep_mode;

	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	pat9126_read(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG, &sleep_mode);

	switch (sleep_mode & SLEEP_MODES_ENABLED_MASK) {
		case SLEEP_MODES_ENABLED_TWO:
			count += sprintf(buf, "2\n");
			break;
		case SLEEP_MODES_ENABLED_ONE:
			count += sprintf(buf, "1\n");
			break;
		default:
			/* Note: this case covers '01' and '00' which both
			   correspond to zero sleep modes */
			count += sprintf(buf, "0\n");
			break;
	}

	return count;
}

static ssize_t pat9126_sleep_level_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count) {
	u8 tmp, sleep_mode;

	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	pat9126_read(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG, &sleep_mode);

	if (kstrtou8(buf, 0, &tmp)) {
		return count;
	}

	dev_dbg(dev, "Setting sleep level: %u\n", tmp);

	if (tmp == 0) {
		sleep_mode &= ~SLEEP_MODE_MASK;
		sleep_mode |= SLEEP_MODE_WAKE;
	} else if (tmp == 1) {
		/* Can't switch to sleep mode #1 if disabled */
		if ((sleep_mode & SLEEP_MODES_ENABLED_MASK) != SLEEP_MODES_ENABLED_ZERO) {
			sleep_mode &= ~SLEEP_MODE_MASK;
			sleep_mode |= SLEEP_MODE_ONE;
		}
	} else if (tmp == 2) {
		/* Can't switch to sleep mode #2 if disabled */
		if ((sleep_mode & SLEEP_MODES_ENABLED_MASK) == SLEEP_MODES_ENABLED_TWO) {
			sleep_mode &= ~SLEEP_MODE_MASK;
			sleep_mode |= SLEEP_MODE_TWO;
		}
	}

	pat9126_write(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG, sleep_mode);

	return count;
}

static int pat9126_pd_write(struct device *dev, u8 val) {
	u8 config;
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	pat9126_read(client, PIXART_PAT9126_CONFIG_REG, &config);
	config &= ~POWER_DOWN_ENABLE_BIT;

	if (val) {
		config |= POWER_DOWN_ENABLE_BIT;
	}

	pat9126_write(client, PIXART_PAT9126_CONFIG_REG, config);
	return 0;
}

static ssize_t pat9126_pd_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count) {
	u8 tmp;

	if (!kstrtou8(buf, 0, &tmp)) {
		dev_dbg(dev, "power down: %d\n", (tmp ? 1 : 0));
		pat9126_pd_write(dev, tmp);
	} else {
		dev_warn(dev, "failed to parse sysfs arg: '%s'\n", buf);
	}

	return count;
}

static ssize_t pat9126_pd_show(struct device *dev,
					struct device_attribute *attr,
					char *buf) {
	int count = 0;
	uint8_t tmp;

	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	pat9126_read(client, PIXART_PAT9126_CONFIG_REG, &tmp);
	tmp = ((POWER_DOWN_ENABLE_BIT & tmp) ? 1 : 0);
	count += sprintf(buf, "0x%x\n", tmp);

	return count;
}

static void pat9126_complete_resume(struct work_struct *work) {
    struct delayed_work *dw = to_delayed_work(work);
	struct pixart_pat9126_data *data =
			container_of(dw, struct pixart_pat9126_data, resume_work);
	struct device *dev = &data->client->dev;

	printk(KERN_DEBUG "[PAT9126]%s, start\n", __func__);
	if (is_initialized) {
		printk(KERN_DEBUG "[PAT9126]%s: Already initialized. \n", __func__);
	} else {
		printk(KERN_DEBUG "[PAT9126]%s: Not initialize yet. \n", __func__);
	}

	pat9126_pd_write(dev, 0);
	delay(3); // To ensure accuracy, datasheet recommends 3ms delay here

	if (is_initialized) {
		if (en_irq_cnt == 0){
			enable_irq(data->client->irq);
			en_irq_cnt++;
			dis_irq_cnt = 0;
		}
		else {
			dev_info(dev, "Already in wake state \n");
		}
	}
}

static DEVICE_ATTR
	(crown_sensitivity, S_IRUGO | S_IWUSR | S_IWGRP, pat9126_sensitivity_show, pat9126_sensitivity_store);

static DEVICE_ATTR
	(id, S_IRUGO, pat9126_id_show, NULL);

static DEVICE_ATTR
	(max_sleep_level, S_IRUGO | S_IWUSR | S_IWGRP, pat9126_max_sleep_level_show, pat9126_max_sleep_level_store);

static DEVICE_ATTR
	(sleep_level, S_IWUSR | S_IWGRP, NULL, pat9126_sleep_level_store);

static DEVICE_ATTR
	(pd, S_IRUGO | S_IWUSR | S_IWGRP, pat9126_pd_show, pat9126_pd_store);

static struct attribute *pat9126_attr_list[] = {
	&dev_attr_crown_sensitivity.attr,
	&dev_attr_id.attr,
	&dev_attr_max_sleep_level.attr,
	&dev_attr_sleep_level.attr,
	&dev_attr_pd.attr,
	NULL,
};

static struct attribute_group pat9126_attr_grp = {
	.attrs = pat9126_attr_list,
};

static int pat9126_parse_dt(struct device *dev,
		struct pixart_pat9126_data *data)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int ret;

	data->inverse_x = of_property_read_bool(np, "pixart,inverse-x");
	data->inverse_y = of_property_read_bool(np, "pixart,inverse-y");
	data->press_en = of_property_read_bool(np, "pixart,press-enabled");
	if (data->press_en) {
		ret = of_property_read_u32(np, "pixart,press-keycode",
						&temp_val);
		if (!ret) {
			data->press_keycode = temp_val;
		} else {
			dev_err(dev, "Unable to parse press-keycode\n");
			return ret;
		}
	}

	return 0;
}

static int pat9126_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct pixart_pat9126_data *data;
	struct input_dev *input;
	struct device *dev = &client->dev;

	ret = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE);
	if (ret < 0) {
		dev_err(dev, "I2C not supported\n");
		return -ENXIO;
	}

	data = devm_kzalloc(dev, sizeof(struct pixart_pat9126_data),
			GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	ret = pat9126_parse_dt(dev, data);
	if (ret) {
		dev_err(dev, "DT parsing failed, errno:%d\n", ret);
		return ret;
	}
	data->client = client;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "Failed to alloc input device\n");
		return -ENOMEM;
	}

	input_set_capability(input, EV_REL, REL_WHEEL);
	if (data->press_en)
		input_set_capability(input, EV_KEY, data->press_keycode);

	i2c_set_clientdata(client, data);
	input_set_drvdata(input, data);
	input->name = PAT9126_DEV_NAME;

	data->input = input;
	ret = input_register_device(data->input);
	if (ret < 0) {
		dev_err(dev, "Failed to register input device\n");
		return ret;
	}

	ret = pinctrl_pm_select_default_state(dev);
	if (ret < 0)
		dev_err(dev, "Could not set pin to active state %d\n", ret);

	usleep_range(DELAY_BETWEEN_REG_US, DELAY_BETWEEN_REG_US + 1);

	/*
		* Initialize pixart sensor after some delay, when vdd
		* regulator is enabled
	*/
	if (pat9126_sensor_init(data->client) < 0) {
		ret = -ENODEV;
		dev_err(dev, "Failed to initialize sensor %d\n", ret);
		return ret;
	}
	INIT_DELAYED_WORK(&data->polling_work, pat9126_work_handler);
	INIT_DELAYED_WORK(&data->resume_work, pat9126_complete_resume);
	if (en_irq_cnt == 0) {
		pr_err("[PAT9126]: Probe Enable Irq. \n");
		ret = devm_request_threaded_irq(dev, client->irq, NULL, pat9126_irq,
				 IRQF_ONESHOT | IRQF_TRIGGER_LOW,
				"pixart_pat9126_irq", data);
		is_initialized = true;

		if (ret) {
			dev_err(dev, "Req irq %d failed, errno:%d\n", client->irq, ret);
			goto err_request_threaded_irq;
		}
		en_irq_cnt++;
	} else
		en_irq_cnt = 0;

	ret = sysfs_create_group(&client->dev.kobj, &pat9126_attr_grp);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group, errno:%d\n", ret);
		goto err_sysfs_create;
	}

	return 0;

err_sysfs_create:
err_request_threaded_irq:
	cancel_work_sync(&data->work);
	destroy_workqueue(data->workqueue);

	return ret;
}

static int pat9126_i2c_remove(struct i2c_client *client)
{
	return 0;
}

#if defined(PAT9126_PM_OPS)
static int pat9126_suspend(struct device *dev)
{
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);

	flush_scheduled_work();

	printk(KERN_DEBUG "[PAT9126]%s, start\n", __func__);
	if(is_initialized) {
		printk(KERN_DEBUG "[PAT9126]%s: Already initialized. \n", __func__);

		if (dis_irq_cnt == 0){
			disable_irq(data->client->irq);
			dis_irq_cnt++;
		}
		else {
			dev_info(dev, "Already in suspend state\n");
			return 0;
		}
		en_irq_cnt = 0;
	} else {
		printk(KERN_DEBUG "[PAT9126]%s: Not initialize yet. \n", __func__);
	}

	pat9126_pd_write(dev, 1);
	return 0;
}

static int pat9126_resume(struct device *dev)
{
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);

	if (!schedule_delayed_work(&data->resume_work, msecs_to_jiffies(10))) {
		pr_err("Failed to schedule delayed resume\n");
		pat9126_complete_resume(&data->resume_work.work);
	}

	return 0;
}
#else
static int pat9126_enable_irq_wake(struct device *dev)
{
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);

	enable_irq_wake(data->client->irq);

	return 0;
}

static int pat9126_disable_irq_wake(struct device *dev)
{
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);

	disable_irq_wake(data->client->irq);

	return 0;
}
#endif

static const struct i2c_device_id pat9126_device_id[] = {
	{PAT9126_DEV_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, pat9126_device_id);

static const struct dev_pm_ops pat9126_pm_ops = {
#if defined(PAT9126_PM_OPS)
	.suspend = pat9126_suspend,
	.resume = pat9126_resume
#else
	.suspend = pat9126_enable_irq_wake,
	.resume = pat9126_disable_irq_wake
#endif
};

static const struct of_device_id pixart_pat9126_match_table[] = {
	{ .compatible = "pixart,pat9126",},
	{ },
};

static struct i2c_driver pat9126_i2c_driver = {
	.driver = {
		   .name = PAT9126_DEV_NAME,
		   .owner = THIS_MODULE,
		   .pm = &pat9126_pm_ops,
		   .of_match_table = pixart_pat9126_match_table,
		   },
	.probe = pat9126_i2c_probe,
	.remove = pat9126_i2c_remove,
	.id_table = pat9126_device_id,
};
module_i2c_driver(pat9126_i2c_driver);

MODULE_AUTHOR("pixart");
MODULE_DESCRIPTION("pixart pat9126 driver");
MODULE_LICENSE("GPL");
