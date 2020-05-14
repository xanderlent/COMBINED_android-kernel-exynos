/* drivers/input/misc/ots_pat9150/pat9150_linux_driver.c
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
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include "pat9150.h"

struct pixart_pat9150_data {
	struct i2c_client *client;
	struct input_dev *input;
	u32 press_keycode;
	u32 irq_gpio;
	bool press_en;
	bool inverse_x;
};

/* Declaration of suspend and resume functions */
static int pat9150_suspend(struct device *dev);
static int pat9150_resume(struct device *dev);

static int pat9150_read(struct i2c_client *client, u8 addr, u8 *data)
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

static int pat9150_write(struct i2c_client *client, u8 addr, u8 data)
{
	u8 buf[BUF_SIZE];
	struct device *dev = &client->dev;

	buf[0] = addr;
	buf[1] = data;

	/* Returns negative errno, or else the number of bytes written. */
	if (i2c_master_send(client, buf, BUF_SIZE) < 0) {
		dev_err(dev, "%s Failed: writing to reg 0x%x\n", __func__,
			addr);
		return -EIO;
	}

	return 0;
}

static int pat9150_write_verified(struct i2c_client *client, u8 addr, u8 data)
{
	int i, ret;
	u8 read_value;

	for (i = 0; i < PAT9150_I2C_RETRY; i++) {
		ret = pat9150_write(client, addr, data);
		if (ret < 0)
			return ret;

		ret = pat9150_read(client, addr, &read_value);
		if (ret < 0)
			return ret;

		if (read_value == data)
			return 0;

		msleep(1);
	}

	return -EIO;
}

static bool pat9150_sensor_init(struct i2c_client *client)
{
	u8 id;
	/*
	 * Read sensor_pid in address 0x00 to check if the
	 * serial link is valid, read value should be 0x31.
	 */
	if (pat9150_read(client, PIXART_PAT9150_PRODUCT_ID1_REG, &id) < 0)
		return false;

	if (id != PIXART_PAT9150_SENSOR_ID)
		return false;

	/*
	 * PAT9150 sensor recommended settings:
	 * switch to bank0, not allowed to perform pat9150_write_verified
	 */
	if (pat9150_write(client, PIXART_PAT9150_SELECT_BANK_REG,
                      PIXART_PAT9150_BANK0) < 0)
		return false;

	/*
	 * software reset (i.e. set bit7 to 1).
	 * It will reset to 0 automatically
	 * so perform OTS_RegWriteRead is not allowed.
	 */
	if (pat9150_write(client, PIXART_PAT9150_RESET_REG,
                      PIXART_PAT9150_RESET) < 0)
		return false;

	/* delay 1ms */
	usleep_range(RESET_DELAY_US, RESET_DELAY_US + 1);

	// Application note indicates that we should write and then
	// read each of these settings to confirm that they were
	// set successfully.
	if (pat9150_write_verified(client, PIXART_PAT9150_WRITE_PROTECT_REG,
			   PIXART_PAT9150_DISABLE_WRITE_PROTECT) < 0)
		return false;

	if (pat9150_write_verified(client, PIXART_PAT9150_MOTION_PIN_MODE_REG,
			   PIXART_PAT9150_MOTION_PIN_MODE_DRIVE) < 0)
		return false;

	// Set X resolution.
	// Copied value from the r10 driver.
	if (pat9150_write_verified(client, PIXART_PAT9150_SET_CPI_RES_X_REG,
			   PIXART_PAT9150_CPI_RESOLUTION_X) < 0)
		return false;

	if (pat9150_write(client, PIXART_PAT9150_POWER_DOWN_ENH_REG, 0) < 0)
		return false;

	if (pat9150_write(client, PIXART_PAT9150_SLEEP1_REG,
			   PIXART_PAT9150_SLEEP1_FREQ_ENM) < 0)
		return false;

	if (pat9150_write(client, PIXART_PAT9150_SLEEP2_REG,
			   PIXART_PAT9150_SLEEP3_FREQ_ENM) < 0)
		return false;

	if (pat9150_write(client, PIXART_PAT9150_SLEEP3_REG,
			   PIXART_PAT9150_SLEEP2_FREQ_ENM) < 0)
		return false;

	/* enable write protect */
	if (pat9150_write_verified(client, PIXART_PAT9150_WRITE_PROTECT_REG,
			   PIXART_PAT9150_ENABLE_WRITE_PROTECT) < 0)
		return false;

	return true;
}

static irqreturn_t pat9150_irq(int irq, void *dev_data)
{
	s16 delta_x;
	u16 unsigned_x;
	u8 low_x, high_x, motion;
	irqreturn_t ret = IRQ_NONE;
	struct pixart_pat9150_data *data = dev_data;
	struct input_dev *ipdev = data->input;
	struct device *dev = &data->client->dev;

	if (pat9150_read(data->client, PIXART_PAT9150_MOTION_STATUS_REG,
			  &motion) < 0)
		return IRQ_NONE;

	while (motion & PIXART_PAT9150_VALID_MOTION_DATA) {
		if (pat9150_read(data->client,
				  PIXART_PAT9150_DELTA_X_LO_REG, &low_x) < 0)
			return ret;

		if (pat9150_read(data->client,
				  PIXART_PAT9150_DELTA_X_HI_REG, &high_x) < 0)
			return ret;

		ret = IRQ_HANDLED;
		unsigned_x = high_x;
		unsigned_x <<= 8;
		unsigned_x |= low_x;
		delta_x = *(s16 *) &unsigned_x;

		/* Inverse x depending upon the device orientation */
		delta_x = (data->inverse_x) ? -delta_x : delta_x;

		dev_dbg(dev, "motion = %x, delta_x = %x\n", motion, delta_x);

		if (delta_x != 0) {
			/* Send delta_x as REL_WHEEL for rotation */
			input_report_rel(ipdev, REL_WHEEL, delta_x);
			input_sync(ipdev);
		}

		usleep_range(PIXART_SAMPLING_PERIOD_US_MIN,
			     PIXART_SAMPLING_PERIOD_US_MAX);

		if (pat9150_read(data->client,
				  PIXART_PAT9150_MOTION_STATUS_REG, &motion) < 0)
			return ret;
	}

	return ret;
}

static int pat9150_power_on(struct pixart_pat9150_data *data)
{
	int ret = 0;
	struct device *dev = &data->client->dev;

	if (!pat9150_sensor_init(data->client)) {
		ret = -ENODEV;
		dev_err(dev, "Failed to initialize sensor %d\n", ret);
		return ret;
	}

	return 0;
}

static int pat9150_parse_dt(struct device *dev,
			    struct pixart_pat9150_data *data)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int ret;

	data->irq_gpio =
	    of_get_named_gpio_flags(np, "pixart,irq-gpio", 0, &temp_val);
	if (data->irq_gpio < 0)
		return data->irq_gpio;

	data->inverse_x = of_property_read_bool(np, "pixart,inverse-x");
	ret = of_property_read_u32(np, "pixart,press-keycode", &temp_val);
	if (ret) {
		data->press_en = false;
	} else {
		data->press_en = true;
		data->press_keycode = temp_val;
	}

	return 0;
}

static int pat9150_configure_gpio(struct device *dev,
				  struct pixart_pat9150_data *data)
{
	int ret = 0;

	if (gpio_is_valid(data->irq_gpio)) {
		ret = gpio_request(data->irq_gpio, "pixart_irq_gpio");
		if (ret) {
			dev_err(dev, "[ots] failed to get irq gpio\n");
			goto free_gpio;
		}

		ret = gpio_direction_input(data->irq_gpio);
		if (ret) {
			dev_err(dev,
				"[ots] failed to set irq gpio direction\n");
			goto free_gpio;
		}
	}

	return 0;

free_gpio:
	if (gpio_is_valid(data->irq_gpio))
		gpio_free(data->irq_gpio);
	return ret;
}

static int pat9150_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int ret = 0;
	struct pixart_pat9150_data *data;
	struct input_dev *input;
	struct device *dev = &client->dev;
	int irq;

	ret = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE);
	if (ret < 0) {
		dev_err(dev, "I2C not supported\n");
		return -ENXIO;
	}

	data = devm_kzalloc(dev, sizeof(struct pixart_pat9150_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	ret = pat9150_parse_dt(dev, data);
	if (ret) {
		dev_err(dev, "DT parsing failed, errno:%d\n", ret);
		return ret;
	}
	data->client = client;

	ret = pat9150_configure_gpio(dev, data);
	if (ret) {
		dev_err(dev, "failed to configure gpio:%d\n", ret);
		return ret;
	}

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
	input->name = PAT9150_DEV_NAME;

	data->input = input;
	ret = input_register_device(data->input);
	if (ret < 0) {
		dev_err(dev, "Failed to register input device\n");
		return ret;
	}

	ret = pat9150_power_on(data);
	if (ret) {
		dev_err(dev, "Failed to power-on the sensor %d\n", ret);
		return ret;
	}

	irq = gpio_to_irq(data->irq_gpio);
	ret =
	    devm_request_threaded_irq(dev, irq, NULL, pat9150_irq,
				      IRQF_ONESHOT | IRQF_TRIGGER_LOW,
				      "pixart_pat9150_irq", data);
	if (ret) {
		dev_err(dev, "Req irq %d failed, errno:%d\n", client->irq, ret);
		return ret;
	}

	return 0;
}

static int pat9150_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int pat9150_suspend(struct device *dev)
{
	struct pixart_pat9150_data *data =
	    (struct pixart_pat9150_data *)dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	if (pat9150_write(client, PIXART_PAT9150_POWER_DOWN_ENH_REG, 1) < 0) {
		dev_err(dev, "Failed to write 1 to POWER_DOWN\n");
	}

	disable_irq(client->irq);
	return 0;
}

static int pat9150_resume(struct device *dev)
{
	int ret;
	struct pixart_pat9150_data *data =
	    (struct pixart_pat9150_data *)dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	ret = pinctrl_pm_select_default_state(dev);
	if (ret < 0)
		dev_err(dev, "Could not set pin to active state %d\n", ret);

	ret = pat9150_power_on(data);
	if (ret) {
		dev_err(dev, "Failed to power-on the sensor %d\n", ret);
		return ret;
	}

	enable_irq(client->irq);

	ret = pat9150_write(client, PIXART_PAT9150_POWER_DOWN_ENH_REG, 0);
	if (ret) {
		dev_err(dev, "Failed to write 0 to POWER_DOWN\n");
		return ret;
	}

	return 0;
}

static const struct i2c_device_id pat9150_device_id[] = {
	{ PAT9150_DEV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, pat9150_device_id);

static const struct dev_pm_ops pat9150_pm_ops = {
	.suspend = pat9150_suspend,
	.resume = pat9150_resume
};

static const struct of_device_id pixart_pat9150_match_table[] = {
	{.compatible = "pixart,pat9150", },
	{ },
};

static struct i2c_driver pat9150_i2c_driver = {
	.driver = {
		   .name = PAT9150_DEV_NAME,
		   .owner = THIS_MODULE,
		   .pm = &pat9150_pm_ops,
		   .of_match_table = pixart_pat9150_match_table,
		    },
	.probe = pat9150_i2c_probe,
	.remove = pat9150_i2c_remove,
	.id_table = pat9150_device_id,
};

module_i2c_driver(pat9150_i2c_driver);

MODULE_AUTHOR("pixart");
MODULE_DESCRIPTION("pixart pat9150 driver");
MODULE_LICENSE("GPL");
