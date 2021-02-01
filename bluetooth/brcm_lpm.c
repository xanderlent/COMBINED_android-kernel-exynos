/*
* Copyright (C) 2016 Spreadtrum Communications Inc.
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/gpio/consumer.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/serial_core.h>
#include <linux/spinlock.h>
#include <linux/termios.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <linux/of_device.h>

#define VERSION			"1.1"
#define PROC_DIR		"bluetooth/sleep"
#define BT_PORT_ID		0
#define POLARITY_LOW	0
#define POLARITY_HIGH	1

/* Macros for handling sleep work */
#define bluesleep_rx_busy()		schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_busy()		schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_rx_idle()		schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_idle()		schedule_delayed_work(&sleep_workqueue, 0)

/* 1 second timeout */
/* after 10S DEV pin change,the BT can sleep */
#define TX_TIMER_INTERVAL		1

/* state variable names and bit positions */
#define BT_PROTO		0x01
#define BT_TXDATA		0x02
#define BT_ASLEEP		0x04
#define BT_EXT_WAKE		0x08
#define BT_SUSPEND		0x10

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 1,
	DEBUG_BTWAKE = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};

static int debug_mask = DEBUG_USER_STATE;
static bool is_bt_stopped;
static bool has_lpm_enabled;

static struct platform_device *bluesleep_uart_dev;
static struct bluesleep_info *bsi;
struct workqueue_struct *hostwake_work_queue;
struct proc_dir_entry *bluetooth_dir, *sleep_dir;

struct bluesleep_info {
	struct gpio_desc *host_wake;
	struct gpio_desc *ext_wake;
	unsigned int host_wake_irq;
	struct uart_port *uport;
	struct wake_lock bt_wakelock;
	struct wake_lock host_wakelock;
	int irq_polarity;
	int has_ext_wake;
};

/* module usage */
static atomic_t open_count = ATOMIC_INIT(1);

/* Global state flags */
static unsigned long flags;

/* Lock for state transitions */
static spinlock_t rw_lock;

static void bluesleep_sleep_work(struct work_struct *work);
DECLARE_DELAYED_WORK(sleep_workqueue, bluesleep_sleep_work);

/* Transmission timer */
static void bluesleep_tx_timer_expire(struct timer_list *data);
static DEFINE_TIMER(tx_timer, bluesleep_tx_timer_expire);

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

/* Local functions */
static void hsuart_power(int on)
{
	pr_info("hsuart_power");
	if (test_bit(BT_SUSPEND, &flags))
		return;
	pr_info("uart power %d\n", on);
}

/* @return 1 if the Host can go to sleep, 0 otherwise */
int bluesleep_can_sleep(void)
{
	/* check if WAKE_BT_GPIO and BT_WAKE_GPIO are both deasserted */
	pr_debug("bt_wake %d, host_wake %d, uport %p",
		gpiod_get_value(bsi->ext_wake),
		gpiod_get_value(bsi->host_wake),
		bsi->uport);
	return ((gpiod_get_value(bsi->host_wake) != bsi->irq_polarity) &&
		(test_bit(BT_EXT_WAKE, &flags)) &&
		(bsi->uport != NULL));
}

/*
 * Starts the Sleep-Mode Protocol on the Host.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int bluesleep_start(void)
{
	int retval;
	unsigned long irq_flags;

	pr_info("bluesleep_start beg\n");
	is_bt_stopped = 0;
	spin_lock_irqsave(&rw_lock, irq_flags);

	if (test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		pr_info("bluesleep_start beg1\n");
		return 0;
	}

	spin_unlock_irqrestore(&rw_lock, irq_flags);
	if (!atomic_dec_and_test(&open_count)) {
		atomic_inc(&open_count);
		pr_info("bluesleep_start beg2\n");
		return -EBUSY;
	}

	/* start the timer */
	mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));

	/* assert BT_WAKE */
	if (bsi->has_ext_wake == 1)
		gpiod_set_value(bsi->ext_wake, 1);
	clear_bit(BT_EXT_WAKE, &flags);
	retval = enable_irq_wake(bsi->host_wake_irq);
	if (retval < 0) {
		pr_info("Couldn't enable BT_HOST_WAKE\n"
			"as wakeup interrupt\n");
		goto fail;
	}
	set_bit(BT_PROTO, &flags);
	pr_debug("bluesleep_start end\n");
	return 0;

fail:
	del_timer(&tx_timer);
	atomic_inc(&open_count);

	return retval;
}

/*
 * Stops the Sleep-Mode Protocol on the Host.
 */
static void bluesleep_stop(void)
{
	unsigned long irq_flags;

	pr_info("bluesleep_stop\n");
	spin_lock_irqsave(&rw_lock, irq_flags);

	if (!test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return;
	}

	/* assert BT_WAKE */
	if (bsi->has_ext_wake == 1)
		gpiod_set_value(bsi->ext_wake, 1);
	clear_bit(BT_EXT_WAKE, &flags);
	del_timer(&tx_timer);
	clear_bit(BT_PROTO, &flags);

	if (test_bit(BT_ASLEEP, &flags)) {
		clear_bit(BT_ASLEEP, &flags);
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		hsuart_power(1);
	} else {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
	}

	atomic_inc(&open_count);

	if (disable_irq_wake(bsi->host_wake_irq))
		pr_info("Couldn't disable hostwake IRQ wakeup mode\n");
	wake_unlock(&bsi->host_wakelock);
	wake_unlock(&bsi->bt_wakelock);
	is_bt_stopped = 1;
}

void bluesleep_sleep_wakeup(void)
{
	pr_info("bluesleep_sleep_wakeup");
	if (test_bit(BT_ASLEEP, &flags)) {
		pr_info("waking up...");
		/* Start the timer */
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));
		if (bsi->has_ext_wake == 1)
			gpiod_set_value(bsi->ext_wake, 1);
		clear_bit(BT_EXT_WAKE, &flags);
		clear_bit(BT_ASLEEP, &flags);
		/* Activating UART */
		hsuart_power(1);
	}
}

/*
 * @brief@ main sleep work handling function which update the flags
 * and activate and deactivate UART ,check FIFO.
 */
static void bluesleep_sleep_work(struct work_struct *work)
{
	pr_info("bluesleep_sleep_work");
	if (bluesleep_can_sleep()) {
		pr_info("can sleep...");
		/* already asleep, this is an error case */
		if (test_bit(BT_ASLEEP, &flags)) {
			pr_info("already asleep");
			return;
		}

		if (bsi->uport->ops->tx_empty(bsi->uport)) {
			pr_info("going to sleep...");
			set_bit(BT_ASLEEP, &flags);
			/* Deactivating UART */
			hsuart_power(0);
			/* UART clk is not turned off immediately. */
			/* Release wakelock after 500 ms. */
		} else {
			pr_info("tx buffer is not empty, modify timer...");
			/* lgh add end */
			mod_timer(&tx_timer,
				jiffies + (TX_TIMER_INTERVAL * HZ));
			return;
		}
	} else if (test_bit(BT_EXT_WAKE, &flags)
		&& !test_bit(BT_ASLEEP, &flags)) {
		pr_info("can not sleep, bt_wake %d",
			gpiod_get_value(bsi->ext_wake));
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));
		gpiod_set_value(bsi->ext_wake, 1);
		clear_bit(BT_EXT_WAKE, &flags);
	} else {
		bluesleep_sleep_wakeup();
	}
}

/*
 * A workqueue that runs in workqueue context and reads the value
 * of the HOST_WAKE GPIO pin and further defer the work.
 * @param work Not used.
 */
static irqreturn_t bluesleep_hostwake_thread_irq(int irq, void *dev_id)
{
	struct bluesleep_info *bsi = (struct bluesleep_info *)dev_id;

	pr_info("hostwake line change");
	pr_info("irq_polarity=%d\n", bsi->irq_polarity);
	pr_info("bsi->host_wake=%d\n", gpiod_get_value(bsi->host_wake));

	spin_lock(&rw_lock);
	if ((gpiod_get_value(bsi->host_wake)) == (bsi->irq_polarity))
		bluesleep_rx_busy();
	else
		bluesleep_rx_idle();

	spin_unlock(&rw_lock);

	return IRQ_HANDLED;
}

static ssize_t bluesleep_write_proc_lpm(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	char b;

	pr_debug("bluesleep_write_proc_lpm\n");
	if (count < 1)
		return -EINVAL;

	if (copy_from_user(&b, buffer, 1))
		return -EFAULT;
	pr_debug("bluesleep_write_proc_lpm_b=%d\n", b);
	if (b == '0') {
		/* HCI_DEV_UNREG */
		bluesleep_stop();
		has_lpm_enabled = false;
		bsi->uport = NULL;
	} else {
		/* HCI_DEV_REG */
		if (!has_lpm_enabled) {
			has_lpm_enabled = true;
			/* if bluetooth started, start bluesleep */
			bluesleep_start();
		}
	}

	return count;
}

static int lpm_proc_show(struct seq_file *m, void *v)
{
	pr_debug("bluesleep_read_proc_lpm\n");
	seq_puts(m, "unsupported to read\n");

	return 0;
}

static int bluesleep_open_proc_lpm(struct inode *inode, struct file *file)
{
	return single_open(file, lpm_proc_show, PDE_DATA(inode));
}

static ssize_t bluesleep_write_proc_btwrite(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	char b;

	pr_debug("bluesleep_write_proc_btwrite");
	if (count < 1)
		return -EINVAL;
	pr_debug("bluesleep_write_proc_btwrite11");
	if (is_bt_stopped == 1)
		return count;
	pr_debug("bluesleep_write_proc_btwrite22");
	if (copy_from_user(&b, buffer, 1))
		return -EFAULT;
	pr_debug("bluesleep_write_proc_btwrite=%c", b);

	/* HCI_DEV_WRITE */
	if (b == '1')
	{
		if (bsi->has_ext_wake == 1)
			gpiod_set_value(bsi->ext_wake, 1);
		set_bit(BT_EXT_WAKE, &flags);
		set_bit(BT_TXDATA, &flags);
	} else {
		if (bsi->has_ext_wake == 1)
			gpiod_set_value(bsi->ext_wake, 0);
		clear_bit(BT_EXT_WAKE, &flags);
		clear_bit(BT_TXDATA, &flags);
	}

	return count;
}

static int btwrite_proc_show(struct seq_file *m, void *v)
{
	pr_debug("bluesleep_read_proc_lpm\n");
	seq_puts(m, "unsupported to read\n");

	return 0;
}

static int bluesleep_open_proc_btwrite(struct inode *inode, struct file *file)
{
	return single_open(file, btwrite_proc_show, PDE_DATA(inode));
}

/*
 * Handles transmission timer expiration.
 * @param data Not used.
 */
static void bluesleep_tx_timer_expire(struct timer_list *data)
{
	pr_debug("bluesleep_tx_timer_expire\n");
	gpiod_set_value(bsi->ext_wake, 1);
	if (bsi->uport == NULL){
		pr_debug("bluesleep_tx_timer_expire is NULL\n");
		return;
	}
	if (bsi->uport->ops->tx_empty(bsi->uport)) {
		pr_debug("empty");
		gpiod_set_value(bsi->ext_wake, 0);
		wake_unlock(&bsi->bt_wakelock);
	} else {
		pr_debug("not empty");
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL*HZ));
	}
}

/*
 * Schedules a workqueue to run when receiving an interrupt on the
 * <code>HOST_WAKE</code> GPIO pin.
 * @param irq Not used.
 * @param dev_id Not used.
 */
static irqreturn_t bluesleep_hostwake_isr(int irq, void *dev_id)
{
	/* schedule a tasklet to handle the change in the host wake line */
	int ext_wake, host_wake;

	pr_debug("bluesleep_hostwake_isr\n");
	ext_wake = gpiod_get_value(bsi->ext_wake);
	host_wake = gpiod_get_value(bsi->host_wake);
	pr_debug("ext_wake : %d, host_wake : %d", ext_wake, host_wake);
	if (host_wake == 0) {
		wake_lock_timeout(&bsi->host_wakelock, HZ*1);
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
	} else {
		wake_lock(&bsi->host_wakelock);
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
	}

	if (host_wake == 0)
		pr_debug("bluesleep_hostwake_isr: Register workqueue\n");

	return IRQ_HANDLED;
}

/*
 * Write the <code>BT_WAKE</code> GPIO pin value via the proc interface.
 * @param file Not used.
 * @param buffer The buffer to read from.
 * @param count The number of bytes to be written.
 * @param data Not used.
 * @return On success, the number of bytes written. On error, -1, and
 * <code>errno</code> is set appropriately.
 */
static ssize_t bluepower_write_proc_btwake(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	char *buf;

	pr_debug("bluepower_write_proc_btwake\n11111");
	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {
		if (bsi->has_ext_wake == 1)
		{
			gpiod_set_value(bsi->ext_wake, 0);
			pr_debug("bluepower_write_proc_btwake down\n");
		}
		clear_bit(BT_EXT_WAKE, &flags);
	} else if (buf[0] == '1') {
		if (bsi->has_ext_wake == 1)
		{
			gpiod_set_value(bsi->ext_wake, 1);
			pr_debug("bluepower_write_proc_btwake up\n");
		}
		set_bit(BT_EXT_WAKE, &flags);
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);

	return count;
}

static int btwake_proc_show(struct seq_file *m, void *v)
{
	unsigned int btwake;

	btwake = test_bit(BT_EXT_WAKE, &flags) ? 1 : 0;
	seq_printf(m, "%u\n", btwake);

	return 0;
}

static int bluepower_open_proc_btwake(struct inode *inode, struct file *file)
{
	return single_open(file, btwake_proc_show, PDE_DATA(inode));
}

static int hostwake_proc_show(struct seq_file *m, void *v)
{
	unsigned int hostwake;

	hostwake = gpiod_get_value(bsi->host_wake);
	seq_printf(m, "%u\n", hostwake);

	return 0;
}

static int bluepower_open_proc_hostwake(struct inode *inode, struct file *file)
{
	return single_open(file, hostwake_proc_show, PDE_DATA(inode));
}

static int bluesleep_proc_show(struct seq_file *m, void *v)
{
	unsigned int asleep;

	asleep = test_bit(BT_ASLEEP, &flags) ? 1 : 0;
	seq_printf(m, "%u\n", asleep);

	return 0;
}

static int bluesleep_open_proc_asleep(struct inode *inode, struct file *file)
{
	return single_open(file, bluesleep_proc_show, PDE_DATA(inode));
}

static ssize_t bluesleep_write_proc_proto(struct file *filp,
	const char __user *buff, size_t count, loff_t *pos)
{
	char proto;

	pr_debug("bluesleep_write_proc_proto\n");
	if (count > 0) {
		if (copy_from_user(&proto, buff, 1))
			return -EFAULT;
		pr_debug("write proto %c", proto);
		if (proto == '0')
			bluesleep_stop();
		else
			bluesleep_start();
	}

	return count;
}

static int proto_proc_show(struct seq_file *m, void *v)
{
	unsigned int proto;

	proto = test_bit(BT_PROTO, &flags) ? 1 : 0;
	seq_printf(m, "%u\n", proto);

	return 0;
}

static int bluesleep_open_proc_proto(struct inode *inode, struct file *file)
{
	return single_open(file, proto_proc_show, PDE_DATA(inode));
}

void bluesleep_setup_uart_port(struct platform_device *uart_dev)
{
	bluesleep_uart_dev = uart_dev;
}

static const struct file_operations lpm_proc_btwake_fops = {
	.owner = THIS_MODULE,
	.open = bluepower_open_proc_btwake,
	.read = seq_read,
	.write = bluepower_write_proc_btwake,
	.release = single_release,
};

static const struct file_operations lpm_proc_hostwake_fops = {
	.owner = THIS_MODULE,
	.open = bluepower_open_proc_hostwake,
	.read = seq_read,
	.release = single_release,
};

static const struct file_operations lpm_proc_proto_fops = {
	.owner = THIS_MODULE,
	.open = bluesleep_open_proc_proto,
	.read = seq_read,
	.write = bluesleep_write_proc_proto,
	.release = single_release,
};

static const struct file_operations lpm_proc_asleep_fops = {
	.owner = THIS_MODULE,
	.open = bluesleep_open_proc_asleep,
	.read = seq_read,
	.release = single_release,
};

static const struct file_operations lpm_proc_lpm_fops = {
	.owner = THIS_MODULE,
	.open = bluesleep_open_proc_lpm,
	.read = seq_read,
	.write = bluesleep_write_proc_lpm,
	.release = single_release,
};

static const struct file_operations lpm_proc_btwrite_fops = {
	.owner = THIS_MODULE,
	.open = bluesleep_open_proc_btwrite,
	.read = seq_read,
	.write = bluesleep_write_proc_btwrite,
	.release = single_release,
};

static int bluesleep_probe(struct platform_device *pdev)
{
	int ret;
	struct proc_dir_entry *ent;
	pr_info("bluesleep_probe in\n");

	bsi = devm_kzalloc(&pdev->dev, sizeof(struct bluesleep_info), GFP_KERNEL);
	if (!bsi)
		return -ENOMEM;

	/* configure host_wake as input */
	bsi->host_wake = devm_gpiod_get(&pdev->dev, "host-wake", GPIOD_IN);
	if (IS_ERR(bsi->host_wake)) {
		dev_err(&pdev->dev, "unable to retrieve 'host-wake' gpio\n");
		return PTR_ERR(bsi->host_wake);
	}

	/* configure ext_wake as output mode */
	if (debug_mask & DEBUG_BTWAKE)
		pr_info("BT WAKE: set to wake\n");
	bsi->ext_wake = devm_gpiod_get(&pdev->dev, "dev-wake", GPIOD_OUT_LOW);
	if (IS_ERR(bsi->ext_wake)) {
		bsi->has_ext_wake = 0;
		dev_err(&pdev->dev, "unable to retrieve 'dev-wake' gpio\n");
		ret = PTR_ERR(bsi->ext_wake);
		goto free_bt_host_wake;
	}
	else {
		bsi->has_ext_wake = 1;
	}
	clear_bit(BT_EXT_WAKE, &flags);

	/* configure host_wake_irq as input */
	pr_info("allocat irq hostwake = %p\n", bsi->host_wake);
	bsi->host_wake_irq = gpiod_to_irq(bsi->host_wake);
	pr_info("irq = bsi->host_wake_irq %d", bsi->host_wake_irq);
	if (bsi->host_wake_irq < 0) {
		pr_err("couldn't find host_wake irq\n");
		ret = -ENODEV;
		goto free_bt_ext_wake;
	}

	/* low edge (falling edge) */
	bsi->irq_polarity = POLARITY_HIGH;

	wake_lock_init(&bsi->bt_wakelock, WAKE_LOCK_SUSPEND, "bluesleep1");
	wake_lock_init(&bsi->host_wakelock, WAKE_LOCK_SUSPEND, "bluesleep2");
	clear_bit(BT_SUSPEND, &flags);

	pr_info("host_wake_irq %d, polarity %d",
			bsi->host_wake_irq,
			bsi->irq_polarity);

	ret = devm_request_threaded_irq(&pdev->dev, bsi->host_wake_irq,
		bluesleep_hostwake_isr, bluesleep_hostwake_thread_irq,
		(IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND),
		"host-wake-gpios", bsi);

	if (ret < 0) {
		pr_info("Couldn't request host-wake-gpios\n");
		goto free_bt_ext_wake;
	}

	flags = 0;
	spin_lock_init(&rw_lock);
	timer_setup(&tx_timer, bluesleep_tx_timer_expire, 0);

	if (bsi == NULL)
		return 0;

	bluetooth_dir = proc_mkdir("bluetooth", NULL);
	if (bluetooth_dir == NULL) {
		pr_err("Unable to create /proc/bluetooth directory");
		return -ENOMEM;
	}

	sleep_dir = proc_mkdir("sleep", bluetooth_dir);
	if (sleep_dir == NULL) {
		pr_err("Unable to create /proc/%s directory", PROC_DIR);
		return -ENOMEM;
	}

	/* Creating read/write entry */
	/* read/write */
	ent = proc_create("btwake", S_IRUGO | S_IWUSR | S_IWGRP,
		sleep_dir, &lpm_proc_btwake_fops);
	/* only read */
	ent = proc_create("hostwake", S_IRUGO,
		sleep_dir, &lpm_proc_hostwake_fops);
	/* read/write */
	ent = proc_create("proto", S_IRUGO | S_IWUSR | S_IWGRP,
		sleep_dir, &lpm_proc_proto_fops);
	/* only read */
	ent = proc_create("asleep", S_IRUGO,
		sleep_dir, &lpm_proc_asleep_fops);
	/* read/write */
	ent = proc_create("lpm", S_IRUGO | S_IWUSR | S_IWGRP,
		sleep_dir, &lpm_proc_lpm_fops);
	/* read/write */
	ent = proc_create("btwrite", S_IRUGO | S_IWUSR | S_IWGRP,
		sleep_dir, &lpm_proc_btwrite_fops);
	if (ent == NULL) {
		pr_err("Unable to create /proc/%s/btwake entry",
			PROC_DIR);
		ret = -ENOMEM;
		goto fail;
	}

	/* assert bt wake */
	if (bsi->has_ext_wake == 1)
		gpiod_set_value(bsi->ext_wake, 1);
	clear_bit(BT_EXT_WAKE, &flags);
	pr_info("a=%d,b=%p\n", bsi->has_ext_wake, bsi->ext_wake);
	return ret;

free_bt_ext_wake:
	gpiod_put(bsi->ext_wake);
free_bt_host_wake:
	gpiod_put(bsi->host_wake);

	return ret;

fail:
	remove_proc_entry("btwrite", sleep_dir);
	remove_proc_entry("lpm", sleep_dir);
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);

	return ret;
}

static int bluesleep_remove(struct platform_device *pdev)
{
	pr_info("bluesleep_remove");

	remove_proc_entry("btwrite", sleep_dir);
	remove_proc_entry("lpm", sleep_dir);
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);

	/* assert bt wake */
	if (bsi->has_ext_wake == 1)
		gpiod_set_value(bsi->ext_wake, 1);
	clear_bit(BT_EXT_WAKE, &flags);
	if (test_bit(BT_PROTO, &flags)) {
		if (disable_irq_wake(bsi->host_wake_irq))
			pr_info("Couldn't disable hostwake\n"
				"IRQ wakeup mode\n");
		free_irq(bsi->host_wake_irq, NULL);
		del_timer(&tx_timer);
		if (test_bit(BT_ASLEEP, &flags))
			hsuart_power(1);
	}

	free_irq(bsi->host_wake_irq, NULL);
	gpiod_put(bsi->host_wake);
	gpiod_put(bsi->ext_wake);
	wake_lock_destroy(&bsi->bt_wakelock);
	wake_lock_destroy(&bsi->host_wakelock);

	return 0;
}

static int bluesleep_resume(struct platform_device *pdev)
{
	pr_info("bluesleep_resume\n");
	if (test_bit(BT_SUSPEND, &flags)) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("bluesleep resuming\n");
		if ((bsi->uport != NULL) &&
			(gpiod_get_value(bsi->host_wake) == bsi->irq_polarity)) {
			if (debug_mask & DEBUG_SUSPEND)
				pr_info("bluesleep\n"
					"resume from BT event\n");
		}
		clear_bit(BT_SUSPEND, &flags);
	}

	return 0;
}

static int bluesleep_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("bluesleep suspending\n");
	set_bit(BT_SUSPEND, &flags);

	return 0;
}

static struct of_device_id bluesleep_match_table[] = {
	{ .compatible = "brcm,bluesleep" },
	{}
};
MODULE_DEVICE_TABLE(of, bluesleep_match_table);

static struct platform_driver bluesleep_driver = {
	.probe = bluesleep_probe,
	.remove = bluesleep_remove,
	.suspend = bluesleep_suspend,
	.resume = bluesleep_resume,
	.driver = {
		.name = "bluesleep",
		.owner = THIS_MODULE,
		.of_match_table = bluesleep_match_table,
	},
};

/*
 * Initializes the module.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int __init bluesleep_init(void)
{
	int ret = 0;

	pr_info("BlueSleep Mode Driver Ver %s", VERSION);
	ret = platform_driver_register(&bluesleep_driver);

	return ret;
}

/* Cleans up the module. */
static void __exit bluesleep_exit(void)
{
	platform_driver_unregister(&bluesleep_driver);
}

module_init(bluesleep_init);
module_exit(bluesleep_exit);
MODULE_DESCRIPTION("Bluetooth Sleep Mode Driver ver %s " VERSION);
MODULE_LICENSE("GPL");
