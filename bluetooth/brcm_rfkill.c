/*
* Copyright (C) 2018 BROADCOM Inc.
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/slab.h>
#include <linux/of_device.h>

static struct gpio_desc *bt_power;

static int bluetooth_set_power(void *data, bool blocked)
{
	pr_debug("bluetooth_set_power: start_block = %d\n", blocked);

	if (!blocked) {
		gpiod_direction_output(bt_power, 0);
		mdelay(10);
		pr_debug("bluetooth_set_power: power up = %d\n", blocked);
		gpiod_direction_output(bt_power, 1);
		mdelay(150);
	} else {
		gpiod_direction_output(bt_power, 0);
		pr_debug("bluetooth_set_power: power down = %d\n", blocked);
		pr_debug("bluetooth_set_power: start_block = %d\n", blocked);
		mdelay(10);
	}
	pr_debug("bluetooth_set_power: end_block = %d\n", blocked);

	return 0;
}

static struct rfkill_ops rfkill_bluetooth_ops = {
	.set_block = bluetooth_set_power,
};

static int rfkill_bluetooth_probe(struct platform_device *pdev)
{

	struct rfkill *rfkill;
	int ret;
	bool default_state = true;
	pr_info("rfkill_bluetooth_probe in\n");

	rfkill = rfkill_alloc("bt_power", &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&rfkill_bluetooth_ops,NULL);

	/* configure power as output mode */
	bt_power = devm_gpiod_get(&pdev->dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(bt_power)) {
		dev_err(&pdev->dev, "unable to retrieve 'power' gpio\n");
		return PTR_ERR(bt_power);
	}

	/* force Bluetooth off during init to allow for user control */
	rfkill_init_sw_state(rfkill, 0);

	ret = rfkill_register(rfkill);

	if (ret) {
		pr_debug("rfkill register failed=%d\n", ret);
		rfkill_destroy(rfkill);
		return ret;
	}

	platform_set_drvdata(pdev, rfkill);

	bluetooth_set_power(NULL, default_state);
	return 0;
}

static int rfkill_bluetooth_remove(struct platform_device *pdev)
{

	struct rfkill *rfkill;
	pr_debug("rfkill_bluetooth_remove in11\n");

	rfkill = platform_get_drvdata(pdev);
	if (rfkill)
		rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id rfkill_of_match[] = {
	{.compatible = "brcm,rfkill"},
	{ }
};
MODULE_DEVICE_TABLE(of, rfkill_of_match);

static struct platform_driver rfkill_bluetooth_driver = {
	.probe	= rfkill_bluetooth_probe,
	.remove	= rfkill_bluetooth_remove,
	.driver	= {
		.name			= "rfkill",
		.owner			= THIS_MODULE,
		.of_match_table	= rfkill_of_match,
	},
};

static int __init rfkill_bluetooth_init(void)
{
	int ret;
	pr_debug("rfkill_bluetooth_init156\n");
	ret=platform_driver_register(&rfkill_bluetooth_driver);
	pr_debug("rfkill_bluetooth_init56 =%d\n",ret);
	return ret;
}

static void __exit rfkill_bluetooth_exit(void)
{
	pr_debug("rfkill_bluetooth_exit\n");
	platform_driver_unregister(&rfkill_bluetooth_driver);
}
module_init(rfkill_bluetooth_init);
module_exit(rfkill_bluetooth_exit);
MODULE_DESCRIPTION("bluetooth rfkill");
MODULE_LICENSE("GPL");
