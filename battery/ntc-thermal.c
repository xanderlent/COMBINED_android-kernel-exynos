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

	gpiod_set_value(ntcdev->enable_gpio, 1);
	usleep_range(10, 20);

	ret = iio_read_channel_raw(ntc_sensor->channel, &val);
	pr_info("adc raw read value: %d\n", val);
	if (ret < 0) {
		dev_err(ntcdev->dev, "IIO channel read failed %d\n", ret);
		return ret;
	}
	long_val = val;
	long_val = mult_frac(long_val, ntcdev->reference_voltage, 0xfff);
	pr_info("adc voltage: %llu\n", long_val);
	*temp = ntc_thermal_adc_to_temp(ntcdev, long_val);

	gpiod_set_value(ntcdev->enable_gpio, 0);
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
	ntc_device->enable_gpio = gpiod_get(&pdev->dev, "enable", GPIOD_OUT_LOW);
	ret = of_property_read_u32(pdev->dev.of_node, "ref-voltage", &ntc_device->reference_voltage);
	if (ret) {
		ntc_device->reference_voltage = 1800000;
	}

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

static const struct of_device_id of_ntc_thermal_match[] = {
	{ .compatible = "google,ntc-thermal", },
	{},
};
MODULE_DEVICE_TABLE(of, of_ntc_thermal_match);

static struct platform_driver ntc_thermal_driver = {
	.driver = {
		.name = "ntc-thermal",
		.of_match_table = of_ntc_thermal_match,
	},
	.probe = ntc_thermal_probe,
};

module_platform_driver(ntc_thermal_driver);

MODULE_DESCRIPTION("ADC NTC thermal driver for R11 using IIO framework with DT");
MODULE_LICENSE("GPL v2");
