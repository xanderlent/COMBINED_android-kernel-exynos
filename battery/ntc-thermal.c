/*
 * NTC thermal driver
 *
 * Copyright 2020 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/iio/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

#define NTC_AUTOSUSPEND_DELAY		100 /* autosuspend delay 100ms */

struct ntc_device;

struct ntc_sensor {
	struct ntc_device *ntcdev;
	struct thermal_zone_device *tz_dev;
	struct iio_channel *channel;
};

struct adc_temp_pair {
	s32 temp;
	s32 adc;
};

struct ntc_device {
	struct adc_temp_pair *lookup_table;
	int nlookup_table;
	int reference_voltage;
	struct device *dev;
	struct gpio_desc *enable_gpio;
	struct mutex gpiolock;
	u32 num_sensors;
	struct ntc_sensor *sensors;
};

static int ntc_thermal_adc_to_temp(struct ntc_device *ntcdev, s64 val)
{
	int temp, adc_x, adc_y, temp_x, temp_y;
	int i;

	for (i = 0; i < ntcdev->nlookup_table; i++) {
		if (val >= ntcdev->lookup_table[i].adc) {
			break;
		}
	}

	if (i == 0) {
		temp = ntcdev->lookup_table[0].temp;
	} else if (i >= (ntcdev->nlookup_table - 1)) {
		temp = ntcdev->lookup_table[ntcdev->nlookup_table - 1].temp;
	} else {
		// Do interpolation.
		adc_x = ntcdev->lookup_table[i-1].adc;
		adc_y = ntcdev->lookup_table[i].adc;

		temp_x = ntcdev->lookup_table[i-1].temp;
		temp_y = ntcdev->lookup_table[i].temp;

		temp = temp_x + mult_frac(temp_y - temp_x, val - adc_x, adc_y - adc_x);
	}

	return temp;
}

static int ntc_thermal_get_temp(void *data, int *temp)
{
	struct ntc_sensor *ntc_sensor = data;
	struct ntc_device *ntcdev = ntc_sensor->ntcdev;
	int val;
	s64 long_val;
	int ret;

	mutex_lock(&ntcdev->gpiolock);
	pm_runtime_get_sync(ntcdev->dev);
	ret = iio_read_channel_raw(ntc_sensor->channel, &val);

	pm_runtime_mark_last_busy(ntcdev->dev);
	pm_runtime_put_autosuspend(ntcdev->dev);
	mutex_unlock(&ntcdev->gpiolock);

	if (ret < 0) {
		dev_err(ntcdev->dev, "IIO channel read failed %d\n", ret);
		return ret;
	}
	long_val = val;
	long_val = mult_frac(long_val, ntcdev->reference_voltage, 0xfff);
	*temp = ntc_thermal_adc_to_temp(ntcdev, long_val);

	if (*temp == ntcdev->lookup_table[ntcdev->nlookup_table - 1].temp) {
		dev_err(ntcdev->dev, "IIO channel returned unusual temperature %d\n", *temp);
		// Treat maximum temperature as a NTC error
		return -EINVAL;
	}

	return 0;
}

static const struct thermal_zone_of_device_ops ntc_thermal_ops = {
	.get_temp = ntc_thermal_get_temp,
};

static int ntc_thermal_read_linear_lookup_table(struct device *dev,
						 struct ntc_device *ntcdev)
{
	struct device_node *np = dev->of_node;
	int ntable;
	int ret;

	ntable = of_property_count_elems_of_size(np, "temperature-lookup-table",
						 sizeof(u32));
	if (ntable < 0) {
		dev_err(dev, "Lookup table is not provided\n");
		return ntable;
	}

	if (ntable % 2) {
		dev_err(dev, "Pair of temperature vs ADC read value missing\n");
		return -EINVAL;
	}

	ntcdev->lookup_table = devm_kzalloc(dev, sizeof(*ntcdev->lookup_table) *
					 ntable, GFP_KERNEL);
	if (!ntcdev->lookup_table)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "temperature-lookup-table",
					 (u32 *)ntcdev->lookup_table, ntable);
	if (ret < 0) {
		dev_err(dev, "Failed to read temperature lookup table: %d\n",
			ret);
		return ret;
	}

	ntcdev->nlookup_table = ntable / 2;

	return 0;
}

static int ntc_thermal_probe(struct platform_device *pdev)
{
	struct ntc_device *ntc_device;
	struct ntc_sensor *ntc_sensor;
	int ret, id, nchan;
	struct iio_channel *channels;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "Only DT based supported\n");
		return -ENODEV;
	}

	channels = devm_iio_channel_get_all(&pdev->dev);
	if (IS_ERR(channels)) {
		ret = PTR_ERR(channels);
		dev_err(&pdev->dev, "IIO channels not found: %d\n", ret);
		return ret;
	}

	nchan = 0;
	while (channels[nchan].channel != NULL) {
		// Count channels
		nchan++;
	}
	// Allocate a nti struct and create a thermal zone for each channel
	ntc_device = devm_kzalloc(&pdev->dev, sizeof(*ntc_device), GFP_KERNEL);
	if (!ntc_device)
		return -ENOMEM;
	platform_set_drvdata(pdev, ntc_device);
	ntc_device->sensors = devm_kzalloc(&pdev->dev, sizeof(*ntc_sensor) * (nchan+1), GFP_KERNEL);
	if (!ntc_device->sensors)
		return -ENOMEM;
	ret = ntc_thermal_read_linear_lookup_table(&pdev->dev, ntc_device);
	if (ret < 0)
		return ret;

	ntc_device->num_sensors = nchan;
	ntc_device->dev = &pdev->dev;
	ntc_device->enable_gpio = devm_gpiod_get(&pdev->dev, "enable", GPIOD_OUT_LOW);
	mutex_init(&ntc_device->gpiolock);
	ret = of_property_read_u32(pdev->dev.of_node, "ref-voltage", &ntc_device->reference_voltage);
	if (ret) {
		ntc_device->reference_voltage = 1800000;
	}

	pm_runtime_enable(ntc_device->dev);
	pm_runtime_set_autosuspend_delay(ntc_device->dev, NTC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(ntc_device->dev);

	for (id = 0; id < ntc_device->num_sensors; id++) {
		ntc_sensor = &ntc_device->sensors[id];
		ntc_sensor->ntcdev = ntc_device;
		ntc_sensor->channel = &channels[id];
		ntc_sensor->tz_dev = devm_thermal_zone_of_sensor_register(&pdev->dev, id,
						      ntc_sensor, &ntc_thermal_ops);
		if (IS_ERR(ntc_sensor->tz_dev)) {
			ret = PTR_ERR(ntc_sensor->tz_dev);
			dev_err(&pdev->dev, "Thermal zone sensor register failed: %d\n",
				ret);
			return ret;
		}
	}
	dev_info(&pdev->dev, "NTC driver probe completed.\n");

	return 0;
}

static int ntc_runtime_suspend(struct device *dev) {
	struct ntc_device *ntc_device = dev_get_drvdata(dev);
	// Make sure we don't set it to 0 when something else is still reading it
	mutex_lock(&ntc_device->gpiolock);
	gpiod_set_value(ntc_device->enable_gpio, 0);
	mutex_unlock(&ntc_device->gpiolock);
	return 0;
}

static int ntc_runtime_resume(struct device *dev) {
	struct ntc_device *ntc_device = dev_get_drvdata(dev);
	gpiod_set_value(ntc_device->enable_gpio, 1);
	/* Wait for ADC to stabilize, takes ~50ms */
	usleep_range(50000, 60000);
	return 0;
}

static const struct of_device_id of_ntc_thermal_match[] = {
	{ .compatible = "google,ntc-thermal", },
	{},
};
MODULE_DEVICE_TABLE(of, of_ntc_thermal_match);

static const struct dev_pm_ops ntc_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(ntc_runtime_suspend, ntc_runtime_resume, NULL)
};

static struct platform_driver ntc_thermal_driver = {
	.driver = {
		.name = "ntc-thermal",
		.of_match_table = of_ntc_thermal_match,
		.pm = &ntc_pm_ops,
	},
	.probe = ntc_thermal_probe,
};

module_platform_driver(ntc_thermal_driver);

MODULE_DESCRIPTION("ADC NTC thermal driver for R11 using IIO framework with DT");
MODULE_LICENSE("GPL v2");
