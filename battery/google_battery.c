/*
 * Copyright 2020 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/version.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/muic/muic.h>
#include <linux/alarmtimer.h>
#include <linux/thermal.h>
#include <linux/debugfs.h>
#include <linux/timekeeping.h>

#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#define FAKE_BAT_LEVEL	50
#define DEFAULT_ALARM_INTERVAL	10
#define SLEEP_ALARM_INTERVAL	300
#define RESUME_DELAY_MS         300
#define INITIAL_VOUT_BOOST_MV	100
#define DEFAULT_CHARGE_STOP_LEVEL	100
#define DEFAULT_CHARGE_START_LEVEL	0

static char *bat_status_str[] = {
	"Unknown",
	"Charging",
	"Discharging",
	"Not-charging",
	"Full"
};

static char *health_str[] = {
	"Unknown",
	"Good",
	"Overheat",
	"Dead",
	"OverVoltage",
	"UnspecFailure",
	"Cold",
	"WatchdogTimerExpire",
	"SafetyTimerExpire",
	"UnderVoltage",
};

static char *power_supply_type_str[] = {
	// Copied from drivers/power/supply/power_supply_sysfs.c
	"Unknown", "Battery", "UPS", "Mains", "USB",
	"USB_DCP", "USB_CDP", "USB_ACA", "USB_C",
	"USB_PD", "USB_PD_DRP", "OTG", "HV_Mains",
	"Prepare_TA", "SMART_NOTG", "Wireless"
};

static enum power_supply_property google_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
};

static enum power_supply_property charging_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
};

static enum power_supply_property wireless_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};
enum battery_voltage_mode {
	BATTERY_VOLTAGE_AVERAGE = 0,
	BATTERY_VOLTAGE_OCV,
};

enum battery_current_mode {
	BATTERY_CURRENT_UA = 0,
	BATTERY_CURRENT_MA,
};

enum battery_charger_mode {
	BAT_CHG_MODE_CHARGING = 0,
	BAT_CHG_MODE_CHARGING_OFF,
	BAT_CHG_MODE_BUCK_OFF,
};

typedef struct battery_platform_data {
	char *charger_name;
	char *fuelgauge_name;
	char *wlc_name;
	char *bat_tz_name;
	char *wlc_tz_name;
	char *wlc_cdev_name;

	/* charging profile */
	unsigned int num_temp_limits;
	unsigned int num_volt_limits;
	s32 *temp_limits;
	s32 *volt_limits;
	s32 *cc_limits;
	int cv_headroom;
	int topoff_current;
	int default_current;
	int wlc_vout_headroom;
	int wlc_vout_min;
	int wlc_vout_max;

	/* full check */
	unsigned int full_check_count;
	unsigned int chg_recharge_vdrop;
	unsigned int chg_recharge_soc;
	int full_check_condition_soc;

	int temp_hysteresis;
	int voltage_hysteresis;
	/* Initial maximum raw SOC */
	unsigned int max_rawsoc;
	/* Max battery charge (in uAh) when battery is full */
	unsigned int charge_full_design;

	/* battery */
	char *vendor;
	int battery_type;
	void *battery_data;
} battery_platform_data_t;

struct battery_info {
	struct device *dev;
	battery_platform_data_t *pdata;

	struct power_supply *psy_battery;
	struct power_supply_desc psy_battery_desc;
	struct power_supply *psy_usb;
	struct power_supply_desc psy_usb_desc;
	struct power_supply *psy_wireless;
	struct power_supply_desc psy_wireless_desc;

	struct mutex iolock;

	struct wake_lock monitor_wake_lock;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
	struct wake_lock vbus_wake_lock;

	struct alarm monitor_alarm;
	unsigned int monitor_alarm_interval;

	int temp_index;
	int voltage_index;
	int charging_current;
	int topoff_current;
	int power_supply_type;
	int float_voltage;

#if defined(CONFIG_MUIC_NOTIFIER)
	struct notifier_block batt_nb;
#endif

	int full_check_cnt;

	/* charging */
	bool is_recharging;
	bool wlc_connected;
	bool wlc_authentic;
	struct mutex wlc_state_lock;
	int wlc_vout_setting;
	int charge_start_level;
	int charge_stop_level;

	bool battery_valid;
	int status;
	int health;

	int voltage_now;
	int voltage_avg;

	unsigned int rawsoc; /* True SOC */
	unsigned int capacity; /* True SOC, scaled down to 100% as max_rawsoc */
	unsigned int soc_fg; /* SOC scaled by FG driver */
	unsigned int soc_spoof; /* UI spoofed SOC */
	unsigned int soc_spoof_full;
	unsigned int max_rawsoc; /* Last known max raw SOC */

	unsigned int charge_full_design;  /* designed battery capacity (uAh) */
	unsigned int charge_full;         /* last known full battery capacity (uAh) */

	int current_now;        /* current (mA) */
	int wlc_current;	/* current (mA) */
	int wlc_voltage;	/* voltage (mV) */

#if defined(CONFIG_MUIC_NOTIFIER)
	struct notifier_block cable_check;
#endif

	/* temperature check */
	int temperature;
	int wlc_temperature;
	/* thermistor voltage reading from the wlc chip */
	int wlc_thermistor;
	int temp_high;
	int temp_high_recovery;
	int temp_low;
	int temp_low_recovery;

	struct thermal_cooling_device *tcd;
	struct thermal_zone_device *wlc_tzd;
	int thermal_level;

	/* DEBUG props */
	struct dentry *debug_root;
	int charging_state_override;
	int charging_current_override;

};

static int calculate_cc_index(struct battery_info *battery, int temp_index, int voltage_index) {
	return (temp_index - 1) * (battery->pdata->num_volt_limits) + voltage_index;
}

static int set_charging_current(struct battery_info *battery, int charging_current) {
	union power_supply_propval value;
	struct power_supply *psy;
	int ret = 0;
	dev_info(battery->dev, "%s: charging_current(%d to %d)\n", __func__,
		battery->charging_current, charging_current);
	value.intval = charging_current;
	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy) {
		return -EINVAL;
	}
	ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property CURRENT_NOW\n", __func__);
	power_supply_put(psy);
	battery->charging_current = charging_current;
	return ret;
}

static int check_charging_current(struct battery_info *battery)
{
	union power_supply_propval value;
	struct power_supply *psy;
	int ret = 0;
	int t_i, v_i, cc_i, check_cc_i, charging_current, topoff_current, volt_threshold;
	int temp_diff, volt_diff;
	// Calculate temp index
	for (t_i = 0; t_i < battery->pdata->num_temp_limits; t_i++) {
		if (battery->temperature < battery->pdata->temp_limits[t_i]) {
			// temp is between limits[t_i - 1] and limits[t_i]
			break;
		}
	}
	// Calculate voltage index, do not pass final index
	for (v_i = 0; v_i < battery->pdata->num_volt_limits - 1; v_i++) {
		volt_threshold = battery->pdata->volt_limits[v_i] - battery->pdata->cv_headroom;
		if (battery->voltage_now < volt_threshold) {
			break;
		}
	}
	if (t_i == 0 || t_i >= battery->pdata->num_temp_limits) {
		battery->temp_index = t_i;
		// temp outside limits, no charge; handled by check_health
		return 0;
	}
	if (t_i == battery->temp_index && v_i == battery->voltage_index) {
		// Want to stay in same CC index as last check
		return 0;
	}
	// Check temperature hysteresis if previous temp index set
	if (battery->temp_index >= 0) {
		// Calculate initial estimate of new charging current
		cc_i = calculate_cc_index(battery, t_i, v_i);
		charging_current = battery->pdata->cc_limits[cc_i];
		// Calculate charging current if we keep current temp index
		check_cc_i = calculate_cc_index(battery, battery->temp_index, v_i);
		// Apply hysteresis only when new current is greater
		if (charging_current > battery->pdata->cc_limits[check_cc_i]) {
			if (t_i > battery->temp_index)
				temp_diff = battery->temperature - battery->pdata->temp_limits[battery->temp_index];
			else
				temp_diff = battery->pdata->temp_limits[t_i] - battery->temperature;
			if (temp_diff < battery->pdata->temp_hysteresis) {
				t_i = battery->temp_index;
				dev_dbg(battery->dev, "Temp hysteresis, keeping temp_index %d\n", t_i);
			}
		}
	}

	// Check voltage hysteresis if previous volt index set and temp index stayed same
	if (battery->voltage_index >= 0 && t_i == battery->temp_index) {
		// Higher voltage threshold always correspond to lower current
		volt_diff = battery->pdata->volt_limits[v_i] - battery->voltage_now;
		if (v_i < battery->voltage_index && volt_diff < battery->pdata->voltage_hysteresis) {
			v_i = battery->voltage_index;
			dev_dbg(battery->dev, "Voltage hysteresis, keeping volt_index %d\n", v_i);
		}
	}

	// Check if the charging current would be 0 at the specified voltage index
	while (v_i > 0) {
		check_cc_i = calculate_cc_index(battery, t_i, v_i);
		if (battery->pdata->cc_limits[check_cc_i] == 0) {
			v_i = v_i - 1;
			dev_dbg(battery->dev, "0 Charging current, moving down volt_index to %d\n", v_i);
		} else {
			break;
		}
	}

	if (t_i == battery->temp_index && v_i == battery->voltage_index) {
		// Want to stay in same CC index as last check
		return 0;
	}
	dev_dbg(battery->dev, "%s: temp_index(%d to %d), volt_index(%d to %d)\n", __func__,
			battery->temp_index, t_i, battery->voltage_index, v_i);
	battery->temp_index = t_i;
	// Calculate final new CC index
	cc_i = calculate_cc_index(battery, battery->temp_index, v_i);
	charging_current = battery->pdata->cc_limits[cc_i];

	if (battery->charging_current_override) {
		if (battery->charging_current_override <= charging_current) {
			charging_current = battery->charging_current_override;
		} else {
			dev_info(battery->dev, "Charging current override too high. t=%d, v=%d, override=%d, max current=%d\n",
					battery->temperature, battery->voltage_now, battery->charging_current_override, battery->charging_current);
		}
	}
	// Actually change charger parameters
	mutex_lock(&battery->iolock);
	// Update charging current if necessary
	if (battery->charging_current != charging_current) {
		ret = set_charging_current(battery, charging_current);
		if (ret < 0)
			goto charger_fail;
	}
	// Update Float Voltage if necessary
	if (battery->voltage_index != v_i) {
		dev_dbg(battery->dev, "%s: voltage_index(%d to %d)\n", __func__,
			battery->voltage_index, v_i);
		value.intval = battery->pdata->volt_limits[v_i];
		psy = power_supply_get_by_name(battery->pdata->charger_name);
		if (!psy) {
			ret = -EINVAL;
			goto charger_fail;
		}
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property VOLTAGE_MAX\n", __func__);
		power_supply_put(psy);
		battery->voltage_index = v_i;
		battery->float_voltage = battery->pdata->volt_limits[v_i];
	}

	// Update topoff current if necessary
	topoff_current = battery->pdata->topoff_current;
	if (battery->topoff_current != topoff_current) {
		value.intval = topoff_current;
		psy = power_supply_get_by_name(battery->pdata->charger_name);
		if (!psy) {
			ret = -EINVAL;
			goto charger_fail;
		}
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CURRENT_FULL, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property CURRENT_FULL\n", __func__);
		power_supply_put(psy);
		battery->topoff_current = topoff_current;
	}

charger_fail:
	mutex_unlock(&battery->iolock);
	return ret;
}

static int check_wlc_vout(struct battery_info *battery)
{
	union power_supply_propval value;
	struct power_supply *psy;
	int ret = 0;
	int wlc_vout;
	wlc_vout = battery->voltage_now + battery->pdata->wlc_vout_headroom;
	if (wlc_vout < battery->pdata->wlc_vout_min) {
		wlc_vout = battery->pdata->wlc_vout_min;
	}
	if (battery->wlc_vout_setting == -1) {
		// need a higher vout on first charge because battery voltage
		// measurement starts out inaccurate
		wlc_vout += INITIAL_VOUT_BOOST_MV;
	}
	if (wlc_vout > battery->pdata->wlc_vout_max) {
		wlc_vout = battery->pdata->wlc_vout_max;
	}
	if (wlc_vout != battery->wlc_vout_setting) {
		psy = power_supply_get_by_name(battery->pdata->wlc_name);
		if (!psy)
			return -EINVAL;
		value.intval = wlc_vout;
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_ENERGY_AVG, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute WLC VOUT property\n", __func__);
		power_supply_put(psy);
		battery->wlc_vout_setting = wlc_vout;
	}
	return 0;
}

static int set_wlc_online(struct battery_info *battery)
{
	union power_supply_propval value;
	struct power_supply *psy;
	int ret;
	// Inform the WLC driver that we are receiving power
	// It needs this to differentiate digital ping from actual power transfer
	psy = power_supply_get_by_name(battery->pdata->wlc_name);
	if (!psy)
		return -EINVAL;
	if (battery->power_supply_type == POWER_SUPPLY_TYPE_WIRELESS) {
		value.intval = 1;
		battery->wlc_vout_setting = -1; // target vout should be recalculated and resent
	} else {
		value.intval = 0;
	}
	ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute WLC ONLINE property\n", __func__);
	power_supply_put(psy);
	return 0;
}


/*
 * set_charger_mode(): charger_mode must have one of following values.
 * 1. BAT_CHG_MODE_CHARGING
 *	Charger on.
 *	Supply power to system & battery both.
 * 2. BAT_CHG_MODE_CHARGING_OFF
 *	Buck mode. Stop battery charging.
 *	But charger supplies system power.
 * 3. BAT_CHG_MODE_BUCK_OFF
 *	All off. Charger is completely off.
 *	Do not supply power to battery & system both.
 */

static int set_charger_mode(
		struct battery_info *battery,
		int charger_mode)
{
	union power_supply_propval val;
	struct power_supply *psy;
		int ret;

	if (charger_mode !=	BAT_CHG_MODE_CHARGING)
		battery->full_check_cnt = 0;

	val.intval = charger_mode;

	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy)
		return -EINVAL;
	ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

	power_supply_put(psy);
	return 0;
}

static int set_wlc_status(struct battery_info *battery,
		int status)
{
	union power_supply_propval val;
	struct power_supply *psy;
	int ret;
	val.intval = status;
	psy = power_supply_get_by_name(battery->pdata->wlc_name);
	if (!psy)
		return -EINVAL;
	ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (ret < 0)
		pr_err("%s: Fail to execute WLC status property\n", __func__);

	power_supply_put(psy);
	return 0;
}

static int set_battery_status(struct battery_info *battery,
		int status)
{
	union power_supply_propval value;
	struct power_supply *psy;
		int ret;

	dev_info(battery->dev, "%s: current status = %s, new status = %s\n", __func__,
			bat_status_str[battery->status], bat_status_str[status]);

	if (battery->charging_state_override && status == POWER_SUPPLY_STATUS_CHARGING) {
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	if (battery->status == status)
		return 0;

	switch (status) {
	case POWER_SUPPLY_STATUS_CHARGING:
		/* notify charger cable type */
		value.intval = battery->power_supply_type;

		psy = power_supply_get_by_name(battery->pdata->charger_name);
		if (!psy)
			return -EINVAL;
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
		power_supply_put(psy);

		set_charger_mode(battery, BAT_CHG_MODE_CHARGING);
		battery->charging_current = battery->pdata->default_current;
		break;

	case POWER_SUPPLY_STATUS_DISCHARGING:

		/* notify charger cable type, this is the power supply type */
		value.intval = battery->power_supply_type;

		psy = power_supply_get_by_name(battery->pdata->charger_name);
		if (!psy)
			return -EINVAL;
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
		// Re-enable charging if it was disabled
		set_charger_mode(battery, BAT_CHG_MODE_CHARGING);
		power_supply_put(psy);
		break;

	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		// charger in unhealthy state
		set_charger_mode(battery, BAT_CHG_MODE_BUCK_OFF);
		set_wlc_status(battery, status);
		break;

	case POWER_SUPPLY_STATUS_FULL:
		// battery full
		set_charger_mode(battery, BAT_CHG_MODE_CHARGING_OFF);
		set_wlc_status(battery, status);
		break;
	}

	if (status != POWER_SUPPLY_STATUS_CHARGING) {
		// Set charging current to default and reset internal state keeping
		if (battery->charging_current != battery->pdata->default_current)
			ret = set_charging_current(battery, battery->pdata->default_current);
		battery->temp_index = -1;
		battery->voltage_index = -1;
		battery->charging_current = 0;
		battery->topoff_current = 0;
	}
	/* battery status update */
	battery->status = status;
	value.intval = battery->status;

	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy)
		return -EINVAL;
	ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
	power_supply_put(psy);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

	return 0;
}

static void set_bat_status_by_cable(struct battery_info *battery)
{
	if (battery->power_supply_type == POWER_SUPPLY_TYPE_BATTERY) {
		// Power is disconnected
		if (!battery->wlc_connected) {
			// Physical charger disconnected
			battery->is_recharging = false;
			set_battery_status(battery, POWER_SUPPLY_STATUS_DISCHARGING);
		} else {
			// Physical WLC charger is still connected but lost power
			if (battery->status != POWER_SUPPLY_STATUS_FULL) {
				// If battery full, keep tracking that
				set_battery_status(battery, POWER_SUPPLY_STATUS_DISCHARGING);
			}
		}
	} else {
		// Power is connected
		if (battery->wlc_connected) {
			if (battery->status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
					battery->status == POWER_SUPPLY_STATUS_FULL) {
				// Shut wireless charger back down
				set_wlc_status(battery, battery->status);
			}
		}
		if (battery->status != POWER_SUPPLY_STATUS_FULL) {
			// Not fully charged
			set_battery_status(battery, POWER_SUPPLY_STATUS_CHARGING);
		}
	}

	return;
}

static int battery_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct battery_info *battery =  power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battery->status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = battery->health;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battery->power_supply_type;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battery->battery_valid;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battery->voltage_now * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = battery->voltage_avg * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battery->temperature;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = (battery->rawsoc * (battery->charge_full_design / 100)) / 100;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!battery->battery_valid)
			val->intval = FAKE_BAT_LEVEL;
		else {
			val->intval = battery->soc_spoof;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = battery->charge_full_design;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = battery->charge_full;
		break;
	default:
		ret = -ENODATA;
	}
	return ret;
}

static int battery_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct battery_info *battery = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		set_battery_status(battery, val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		battery->health = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		battery->power_supply_type = val->intval;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*
 * USB charger operations
 */
static int usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct battery_info *battery =  power_supply_get_drvdata(psy);

	if (psp != POWER_SUPPLY_PROP_ONLINE && psp != POWER_SUPPLY_PROP_PRESENT)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	switch (battery->power_supply_type) {
	case POWER_SUPPLY_TYPE_MAINS:
	case POWER_SUPPLY_TYPE_UNKNOWN:
	case POWER_SUPPLY_TYPE_PREPARE_TA:
	case POWER_SUPPLY_TYPE_HV_MAINS:
		val->intval = 1;
		break;
	default:
		val->intval = 0;
		break;
	}

	return 0;
}

static int wireless_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct battery_info *battery =  power_supply_get_drvdata(psy);

	switch(psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
		if (battery->wlc_authentic && (battery->power_supply_type == POWER_SUPPLY_TYPE_WIRELESS || battery->wlc_connected))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (battery->wlc_authentic)
			val->intval = 300;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

#if defined(CONFIG_MUIC_NOTIFIER)
static int battery_handle_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	muic_attached_dev_t attached_dev = *(muic_attached_dev_t *)data;
	const char *cmd;
	int power_supply_type;
	struct battery_info *battery =
		container_of(nb, struct battery_info, batt_nb);

	switch (action) {
	case MUIC_NOTIFY_CMD_DETACH:
	case MUIC_NOTIFY_CMD_LOGICALLY_DETACH:
		cmd = "DETACH";
		power_supply_type = POWER_SUPPLY_TYPE_BATTERY;
		break;
	case MUIC_NOTIFY_CMD_ATTACH:
	case MUIC_NOTIFY_CMD_LOGICALLY_ATTACH:
		cmd = "ATTACH";
		if (attached_dev == ATTACHED_DEV_WIRELESS_TA_MUIC) {
			power_supply_type = POWER_SUPPLY_TYPE_WIRELESS;
		} else {
			power_supply_type = POWER_SUPPLY_TYPE_MAINS;
		}
		break;
	default:
		cmd = "ERROR";
		power_supply_type = POWER_SUPPLY_TYPE_BATTERY;
		break;
	}

	dev_info(battery->dev, "%s: current_power_supply_type(%s) former power_supply_type(%s) battery_valid(%d)\n",
			__func__,
			power_supply_type_str[power_supply_type],
			power_supply_type_str[battery->power_supply_type],
			battery->battery_valid);
	if (battery->battery_valid == false)
		dev_info(battery->dev, "%s: Battery is disconnected\n", __func__);

	battery->power_supply_type = power_supply_type;

	mutex_lock(&battery->wlc_state_lock);

	if (battery->wlc_connected) {
		set_wlc_online(battery);
	}
	set_bat_status_by_cable(battery);
	mutex_unlock(&battery->wlc_state_lock);

	dev_dbg(battery->dev,
			"%s: Status(%s), Health(%s), Power Supply Type(%s), Recharging(%d))"
			"\n", __func__,
			bat_status_str[battery->status],
			health_str[battery->health],
			power_supply_type_str[battery->power_supply_type],
			battery->is_recharging
		  );

	power_supply_changed(battery->psy_battery);
	alarm_cancel(&battery->monitor_alarm);
	wake_lock(&battery->monitor_wake_lock);
	queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 0);
	return 0;
}
#endif

// Scale input to a percentage of full scale with rounding
static int google_battery_percent_scale(int input, int full_scale) {
	int val = (input * 100 + (full_scale-1)) / full_scale;
	if (val > 100)
		return 100;
	else
		return val;
}

static void get_battery_capacity(struct battery_info *battery)
{

	union power_supply_propval value;
	struct power_supply *psy;
	int ret;
	unsigned int raw_soc = 0;
	unsigned int new_soc = 0;

	psy = power_supply_get_by_name(battery->pdata->fuelgauge_name);
	if (psy == NULL) {
		pr_err("%s : cannot find fuelgauge!\n", __func__);
		return;
	}

	//read rawsoc only
	value.intval = 1;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &value);
	if (ret < 0) {
		pr_err("%s: Fail to execute capacity property\n", __func__);
		return;
	}
	raw_soc = value.intval;

	// Read scaled soc
	value.intval = 0;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &value);
	if (ret < 0) {
		pr_err("%s: Fail to execute scaled capacity property\n", __func__);
		return;
	}
	battery->soc_fg = value.intval;

	if (raw_soc > battery->max_rawsoc)
		battery->max_rawsoc = raw_soc;

	battery->rawsoc = raw_soc;

	battery->capacity = google_battery_percent_scale(raw_soc, battery->max_rawsoc);
	if (battery->status == POWER_SUPPLY_STATUS_FULL ||
			raw_soc > battery->soc_spoof_full + 100) {
		if (battery->charge_stop_level == DEFAULT_CHARGE_STOP_LEVEL) {
			battery->soc_spoof_full = raw_soc - 100;
		} else {
			// Do not spoof SOC if we are lowering the charge level
			battery->soc_spoof_full = battery->max_rawsoc;
		}
	}

	// new_soc = real capacity * 100% / fake full capacity, rounded
	new_soc = google_battery_percent_scale(raw_soc, battery->soc_spoof_full);

	if (battery->wlc_connected || battery->status == POWER_SUPPLY_STATUS_CHARGING ||
		battery->status == POWER_SUPPLY_STATUS_FULL	|| new_soc < battery->soc_spoof ||
		battery->soc_spoof == 0) {
		// Don't let capacity increase during discharge
		battery->soc_spoof = new_soc;

	}

	power_supply_put(psy);
}

static int get_battery_info(struct battery_info *battery)
{
	union power_supply_propval value;
	struct power_supply *psy;
	struct thermal_zone_device *tzd;
	int temp;
	int ret;
	unsigned int old_capacity = battery->capacity;
	unsigned int old_spoof = battery->soc_spoof;

	/*Get fuelgauge psy*/
	psy = power_supply_get_by_name(battery->pdata->fuelgauge_name);
	if (!psy)
		return -EINVAL;

	/* Get voltage and current value */
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	else
		battery->voltage_now = value.intval;

	// Selection of voltage reading mode
	value.intval = BATTERY_VOLTAGE_AVERAGE;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_AVG, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	else
		battery->voltage_avg = value.intval;

	// Selection of current reading mode
	value.intval = BATTERY_CURRENT_MA;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	else
		battery->current_now = value.intval;

	/* Get temperature info */
	tzd = thermal_zone_get_zone_by_name(battery->pdata->bat_tz_name);
	if (!tzd) {
		pr_err("%s: Fail to get battery thermal zone\n", __func__);
		battery->temperature = -1;
	} else {
		ret = thermal_zone_get_temp(tzd, &temp);
		if (ret < 0) {
			pr_err("%s: Fail to get temp from thermal zone\n", __func__);
			battery->temperature = -1;
		} else {
			// Supposed to be represented in 1/10 degrees celsius
			battery->temperature = temp / 100;
		}
	}

	power_supply_put(psy);
	get_battery_capacity(battery);

	if (old_capacity != battery->capacity || old_spoof != battery->soc_spoof) {
		// Log data about battery state on % changes
		dev_info(battery->dev, "%s: Time(%lu), SOC(%u), FG SOC(%u), UI SOC(%u), rawsoc(%d), max_rawsoc(%u), soc_spoof_full(%u), voltage(%u), current(%d), temp(%u), status(%s)\n",
				__func__,
				ktime_get_real_ns() / 1000000000, battery->capacity, battery->soc_fg,
				battery->soc_spoof, battery->rawsoc, battery->max_rawsoc,
				battery->soc_spoof_full, battery->voltage_now, battery->current_now,
				battery->temperature, bat_status_str[battery->status]);
	}

	return 0;
}

static int get_battery_health(struct battery_info *battery)
{
	union power_supply_propval value;
	int health = POWER_SUPPLY_HEALTH_UNKNOWN;
	struct power_supply *psy;
	int ret;

	/* Get health status from charger */
	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy)
		return -EINVAL;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_HEALTH, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	else
		health = value.intval;
	power_supply_put(psy);

	return health;
}

static int get_temperature_health(struct battery_info *battery)
{
	int health = POWER_SUPPLY_HEALTH_UNKNOWN;

	switch (battery->health) {
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		if (battery->temperature < battery->temp_high_recovery)
			health = POWER_SUPPLY_HEALTH_GOOD;
		else
			health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case POWER_SUPPLY_HEALTH_COLD:
		if (battery->temperature > battery->temp_low_recovery)
			health = POWER_SUPPLY_HEALTH_GOOD;
		else
			health = POWER_SUPPLY_HEALTH_COLD;
		break;
	case POWER_SUPPLY_HEALTH_GOOD:
	default:
		if (battery->temperature > battery->temp_high)
			health = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (battery->temperature < battery->temp_low)
			health = POWER_SUPPLY_HEALTH_COLD;
		else
			health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	}

	return health;
}

static void check_health(struct battery_info *battery)
{
	int battery_health = 0;
	int temperature_health = 0;

	battery_health = get_battery_health(battery);
	temperature_health = get_temperature_health(battery);

	if (battery_health >= 0)

	/* If battery & temperature both are normal,			 *
	 *	set battery->health GOOD and recover battery->status */
	if (battery_health == POWER_SUPPLY_HEALTH_GOOD &&
		temperature_health == POWER_SUPPLY_HEALTH_GOOD) {
		battery->health = POWER_SUPPLY_HEALTH_GOOD;
		mutex_lock(&battery->wlc_state_lock);
		if (battery->status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			set_bat_status_by_cable(battery);
		}
		mutex_unlock(&battery->wlc_state_lock);
		return;
	}

	switch (battery_health) {
	case POWER_SUPPLY_HEALTH_OVERVOLTAGE:
	case POWER_SUPPLY_HEALTH_UNDERVOLTAGE:
	case POWER_SUPPLY_HEALTH_UNKNOWN:
		battery->health = battery_health;
		goto abnormal_health;
	default:
		break;
	}
	switch (temperature_health) {
	case POWER_SUPPLY_HEALTH_OVERHEAT:
	case POWER_SUPPLY_HEALTH_COLD:
	case POWER_SUPPLY_HEALTH_UNKNOWN:
		battery->health = temperature_health;
		goto abnormal_health;
	default:
		break;
	}

	pr_err("%s: Abnormal case of temperature & battery health.\n", __func__);
	return;

abnormal_health:
	if (battery->status != POWER_SUPPLY_STATUS_NOT_CHARGING) {
		battery->is_recharging = false;
		/* Take the wakelock during 10 seconds	*
		 * when not_charging status is detected */
		wake_lock_timeout(&battery->vbus_wake_lock, HZ * 10);
		set_battery_status(battery, POWER_SUPPLY_STATUS_NOT_CHARGING);
	}
}

static void check_charging_full(
		struct battery_info *battery)
{
	union power_supply_propval value;
	struct power_supply *psy;
	int ret;
	unsigned int raw_soc = 0;
	int recharge_vcell;
	bool spoof_full = false;
	if (!battery->wlc_connected) {
		if ((battery->status == POWER_SUPPLY_STATUS_DISCHARGING) ||
				(battery->status == POWER_SUPPLY_STATUS_NOT_CHARGING)) {
			dev_dbg(battery->dev,
					"%s: No Need to Check Full-Charged\n", __func__);
			return;
		}
	}

	/* Recharging check */
	if (battery->status == POWER_SUPPLY_STATUS_FULL) {
		recharge_vcell = battery->float_voltage - battery->pdata->chg_recharge_vdrop;
		if (battery->capacity < battery->charge_start_level ||
				(battery->charge_stop_level == DEFAULT_CHARGE_STOP_LEVEL &&
				(battery->voltage_now < recharge_vcell && battery->capacity <= battery->pdata->chg_recharge_soc))) {
			dev_info(battery->dev, "%s: Recharging start: SOC %d\n", __func__, battery->capacity);
			mutex_lock(&battery->wlc_state_lock);
			if (battery->wlc_connected && battery->power_supply_type == POWER_SUPPLY_TYPE_BATTERY) {
				set_battery_status(battery, POWER_SUPPLY_STATUS_DISCHARGING);
			} else {
				set_battery_status(battery, POWER_SUPPLY_STATUS_CHARGING);
			}
			battery->is_recharging = true;
			mutex_unlock(&battery->wlc_state_lock);
		}
	}

	/* Full charged check */
	if (battery->capacity > battery->charge_stop_level) {
		// Spoof fullness state
		spoof_full = true;
	} else {
		psy = power_supply_get_by_name(battery->pdata->charger_name);
		if (!psy) {
			pr_err("%s: Could not get charger psy\n", __func__);
			return;
		}
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
		if (ret < 0) {
			pr_err("%s: Fail to execute property\n", __func__);
			return;
		}
		power_supply_put(psy);

		if ((battery->status != value.intval
					&& value.intval == POWER_SUPPLY_STATUS_FULL)) {
			if (battery->capacity
					< battery->pdata->full_check_condition_soc) {
				dev_info(battery->dev, "%s: below full check SOC.\n", __func__);
				return;
			}

			battery->full_check_cnt++;
			dev_info(battery->dev, "%s: Full Check Cnt (%d)\n",
					__func__, battery->full_check_cnt);
		} else if (battery->full_check_cnt != 0) {
			/* Reset full check cnt when it is out of full condition */
			battery->full_check_cnt = 0;
			dev_info(battery->dev, "%s: Reset Full Check Cnt\n", __func__);
		}
	}

	/* If full charged, turn off charging. */
	if ((spoof_full && battery->status != POWER_SUPPLY_STATUS_FULL) ||
			battery->full_check_cnt >= battery->pdata->full_check_count) {
		if (spoof_full)
			dev_info(battery->dev, "Charge stop level is set at %d, spoof fullness\n", battery->charge_stop_level);
		battery->full_check_cnt = 0;
		mutex_lock(&battery->wlc_state_lock);
		battery->is_recharging = false;
		set_battery_status(battery, POWER_SUPPLY_STATUS_FULL);
		mutex_unlock(&battery->wlc_state_lock);
		dev_info(battery->dev, "%s: Full charged, charger off\n", __func__);

		if (!spoof_full) {
			/* Read raw SOC value from the fuel gauge */
			psy = power_supply_get_by_name(battery->pdata->fuelgauge_name);
			if (!psy)
				return;
			value.intval = 1;
			ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &value);
			if (ret < 0)
				pr_err("%s: Fail to get capacity property\n", __func__);
			raw_soc = value.intval;
			/* Max battery charge is ( (raw_soc/100) * charge_full_design ) / 100
			 * where raw_soc is in 0.01% units and charge_full_design is in uAh.
			 * switching around to prevent int overflow and maintain precision:
			 * (raw_soc * ( charge_full_design / 100 )) /100 */
			battery->charge_full = (raw_soc * (battery->charge_full_design / 100)) / 100;

			battery->max_rawsoc = raw_soc;

			/* Let fuelgauge update charge_full */
			value.intval = battery->charge_full;
			ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGE_FULL, &value);
			if (ret < 0)
				pr_err("%s: Fail to set charge_full property\n", __func__);
			power_supply_put(psy);
		}
	}
}

static void get_wlc_info(struct battery_info* battery) {
	int ret;
	union power_supply_propval value;
	struct power_supply *wlc_psy;
	int temp;

	if (battery->wlc_tzd) {
		ret = thermal_zone_get_temp(battery->wlc_tzd, &temp);
		if (ret < 0) {
			pr_err("%s: Fail to get temp from thermal zone\n", __func__);
			battery->wlc_temperature = -1;
		} else {
			// Keep unit consistency with battery temperature measurement
			// It is 1/10 of a degree C
			battery->wlc_temperature = temp / 100;
		}
	} else {
		// No WLC temperature support
		battery->wlc_temperature = -1;
	}

	battery->wlc_current = 0;
	battery->wlc_voltage = 0;
	battery->wlc_thermistor = 0;
	if (battery->wlc_connected && battery->status == POWER_SUPPLY_STATUS_CHARGING) {

		wlc_psy = power_supply_get_by_name(battery->pdata->wlc_name);
		if (!wlc_psy) {
			pr_err("Failed to get WLC PSY\n");
			return;
		}

		ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_CURRENT_AVG, &value);
		if (ret < 0) {
			pr_err("%s: Fail to execute property\n", __func__);
		} else {
			// Convert uA to mA
			battery->wlc_current = value.intval / 1000;
		}

		ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_ENERGY_AVG, &value);
		if (ret < 0) {
			pr_err("%s: Fail to execute property\n", __func__);
		} else {
			// Conver uA to mA
			battery->wlc_voltage = value.intval / 1000;
		}

		ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_TEMP_AMBIENT, &value);
		if (ret < 0) {
			pr_err("%s: Fail to execute property\n", __func__);
		} else {
			battery->wlc_thermistor = value.intval;
		}
		power_supply_put(wlc_psy);
	}
}

static void battery_external_power_changed(struct power_supply *psy) {
	union power_supply_propval value;
	struct power_supply *wlc_psy;
	int ret;
	struct battery_info *battery = power_supply_get_drvdata(psy);
	mutex_lock(&battery->wlc_state_lock);
	wlc_psy = power_supply_get_by_name(battery->pdata->wlc_name);
	if (!wlc_psy) {
		pr_err("Failed to get WLC PSY\n");
		goto external_power_changed_fail;
	}
	ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_PRESENT, &value);
	if (ret < 0) {
		pr_err("%s: Fail to execute WLC present property\n", __func__);
	} else {
		if (value.intval) {
			if (!battery->wlc_connected)
				dev_info(battery->dev, "WLC connected\n");
			battery->wlc_connected = true;
			ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_AUTHENTIC, &value);
			if (ret < 0) {
				pr_err("%s: Fail to execute WLC auth property\n", __func__);
			}
			if (value.intval) {
				battery->wlc_authentic = true;
			} else {
				dev_info(battery->dev, "WLC inauthentic\n");
				battery->wlc_authentic = false;
			}
		} else {
			if (battery->power_supply_type == POWER_SUPPLY_TYPE_BATTERY) {
				battery->is_recharging = false;
				set_battery_status(battery, POWER_SUPPLY_STATUS_DISCHARGING);
			}
			battery->wlc_connected = false;
			dev_info(battery->dev, "WLC disconnected\n");
		}
	}
	if (battery->psy_battery)
		power_supply_changed(battery->psy_battery);
external_power_changed_fail:
	power_supply_put(wlc_psy);
	mutex_unlock(&battery->wlc_state_lock);
}

static void bat_monitor_work(struct work_struct *work)
{
	struct battery_info *battery =
		container_of(work, struct battery_info, monitor_work.work);
	union power_supply_propval value;
	struct power_supply *psy;
	struct thermal_zone_device *tzd;
	int old_charging_current, old_temp_index, old_capacity, old_health;
	int ret;

	tzd = thermal_zone_get_zone_by_name(battery->pdata->wlc_tz_name);
	if (tzd) {
		enum thermal_device_mode mode;
		tzd->ops->get_mode(tzd, &mode);
		if (mode == THERMAL_DEVICE_DISABLED) {
			tzd->ops->set_mode(tzd, THERMAL_DEVICE_ENABLED);
		}
	} else {
		pr_err("%s: Fail to get thermal zone device: %s\n",
		       __func__, battery->pdata->wlc_tz_name);
	}

	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy) {
		pr_err("%s: Fail to get charger psy\n", __func__);
		goto continue_monitor;
	}
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &value);
	if (ret < 0) {
		pr_err("%s: Fail to execute property\n", __func__);
	} else {
		if (!value.intval) {
			battery->battery_valid = false;
			dev_dbg(battery->dev, "%s: There is no battery, skip monitoring.\n", __func__);
			goto continue_monitor;
		} else
			battery->battery_valid = true;
	}
	old_capacity = battery->soc_spoof;
	old_health = battery->health;
	old_temp_index = battery->temp_index;
	get_battery_info(battery);

	check_health(battery);
	if (battery->status == POWER_SUPPLY_STATUS_CHARGING) {
		old_charging_current = battery->charging_current;
		ret = check_charging_current(battery);
		if (battery->charging_current != old_charging_current) {
			mdelay(250);
		}
		if (battery->wlc_connected) {
			check_wlc_vout(battery);
		}
	}
	get_wlc_info(battery);
	if (ret < 0)
		pr_err("%s: Fail to check charging current\n", __func__);
	check_charging_full(battery);

	if (old_capacity != battery->soc_spoof ||
			old_health != battery->health ||
			old_temp_index != battery->temp_index) {
		power_supply_changed(battery->psy_battery);
	}

continue_monitor:
	dev_info(battery->dev,
			"%s: Time=%lu, Battery sts=%s, vbat=%d, ichg=%d, ifg=%d, soc=%d, tbat=%d, tchg=%d, wlc=%s, iwlc=%d, twlc=%d, vwlc=%d\n",
			__func__,
			ktime_get_real_ns() / 1000000000,
			bat_status_str[battery->status],
			battery->voltage_now,
			battery->charging_current,
			battery->current_now,
			battery->capacity,
			battery->temperature,
			battery->wlc_temperature,
			battery->wlc_connected ? "a" : "d",
			battery->wlc_current,
			battery->wlc_thermistor,
			battery->wlc_voltage
			);

	power_supply_put(psy);
	alarm_cancel(&battery->monitor_alarm);
	alarm_start_relative(&battery->monitor_alarm, ktime_set(battery->monitor_alarm_interval, 0));
	wake_unlock(&battery->monitor_wake_lock);
}

static int google_battery_parse_dt(struct device *dev,
		struct battery_info *battery)
{
	struct device_node *np = of_find_node_by_name(NULL, "battery");
	battery_platform_data_t *pdata = battery->pdata;
	int ret = 0;
	u32 temp;
	u32 array_size;

	if (!np) {
		pr_info("%s np NULL(battery)\n", __func__);
		return -1;
	}
	ret = of_property_read_string(np,
			"battery,vendor", (char const **)&pdata->vendor);
	if (ret)
		pr_info("%s: Vendor is empty\n", __func__);

	ret = of_property_read_string(np,
			"battery,charger_name", (char const **)&pdata->charger_name);
	if (ret) {
		pr_info("%s: Charger name is empty\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_string(np,
			"battery,fuelgauge_name", (char const **)&pdata->fuelgauge_name);
	if (ret) {
		pr_info("%s: Fuelgauge name is empty\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_string(np,
			"battery,wlc_name", (char const **)&pdata->wlc_name);
	if (ret)
		pr_info("%s: WLC name is empty\n", __func__);

	ret = of_property_read_string(np,
			"battery,bat_tz_name", (char const **)&pdata->bat_tz_name);
	if (ret) {
		pr_info("%s: Battery Thermal Zone name is empty\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_string(np,
			"battery,wlc_tz_name", (char const **)&pdata->wlc_tz_name);
	if (ret)
		pr_info("%s: WLC Thermal Zone name is empty\n", __func__);

	ret = of_property_read_string(np,
			"battery,wlc_cdev_name", (char const **)&pdata->wlc_cdev_name);
	if (ret)
		pr_info("%s: WLC Cooling device name is empty\n", __func__);

	pdata->num_temp_limits =
	    of_property_count_elems_of_size(np, "battery,chg_temp_limits",
					    sizeof(u32));
	if (pdata->num_temp_limits <= 0) {
		pr_err("%s: cannot read chg_temp_limits, num=%d\n", __func__, pdata->num_temp_limits);
		return -EINVAL;
	}
	pdata->temp_limits = devm_kzalloc(dev, sizeof(s32) * pdata->num_temp_limits,
			GFP_KERNEL);
	if (!pdata->temp_limits)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "battery,chg_temp_limits",
					 (u32 *)pdata->temp_limits,
					 pdata->num_temp_limits);
	if (ret < 0) {
		pr_err("%s: cannot read chg_temp_limits table, ret=%d\n", __func__,
			 ret);
		return ret;
	}

	pdata->num_volt_limits =
	    of_property_count_elems_of_size(np, "battery,chg_volt_limits",
					    sizeof(u32));
	if (pdata->num_volt_limits <= 0) {
		pr_err("%s: cannot read chg_volt_limits, num=%d\n", __func__, pdata->num_temp_limits);
		return -EINVAL;
	}
	pdata->volt_limits = devm_kzalloc(dev, sizeof(s32) * pdata->num_volt_limits,
			GFP_KERNEL);
	if (!pdata->volt_limits)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "battery,chg_volt_limits",
					 (u32 *)pdata->volt_limits,
					 pdata->num_volt_limits);
	if (ret < 0) {
		pr_err("%s: cannot read chg_volt_limits table, ret=%d\n", __func__,
			 ret);
		return -EINVAL;
	}

	array_size = (pdata->num_temp_limits - 1) * pdata->num_volt_limits;
	pdata->cc_limits = devm_kzalloc(dev, sizeof(s32) * array_size, GFP_KERNEL);

	if (!pdata->cc_limits)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "battery,chg_cc_limits",
					 (u32 *)pdata->cc_limits, array_size);
	if (ret < 0) {
		pr_err("%s: cannot read chg_cc_limits table, ret=%d\n", __func__,
			 ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "battery,chg_cv_headroom",
			&pdata->cv_headroom);
	if (ret) {
		pr_debug("%s : CV headroom is empty\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "battery,topoff_current",
			&pdata->topoff_current);
	if (ret) {
		pr_debug("%s : Topoff current is empty\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "battery,default_current",
			&pdata->default_current);
	if (ret) {
		pr_debug("%s : Default current is empty\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "battery,wlc_vout_headroom",
			&pdata->wlc_vout_headroom);
	if (ret) {
		pr_debug("%s : WLC VOUT headroom is empty\n", __func__);
		pdata->wlc_vout_headroom = 200;
	}

	ret = of_property_read_u32(np, "battery,wlc_vout_min",
			&pdata->wlc_vout_min);
	if (ret) {
		pr_debug("%s : WLC VOUT min is empty\n", __func__);
		pdata->wlc_vout_min = 4200;
	}

	ret = of_property_read_u32(np, "battery,wlc_vout_max",
			&pdata->wlc_vout_max);
	if (ret) {
		pr_debug("%s : WLC VOUT max is empty\n", __func__);
		pdata->wlc_vout_min = 5000;
	}

	ret = of_property_read_u32(np, "battery,full_check_condition_soc",
			&pdata->full_check_condition_soc);
	if (ret) {
		pr_debug("%s : Full check condition soc is empty\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "battery,temp_hysteresis", &temp);
	if (ret) {
		pr_info("%s : temp_high is empty\n", __func__);
		pdata->temp_hysteresis = 10;
	} else
		pdata->temp_hysteresis = (int)temp;

	ret = of_property_read_u32(np, "battery,voltage_hysteresis", &temp);
	if (ret) {
		pr_info("%s : voltage_hysteresis is empty\n", __func__);
		pdata->voltage_hysteresis = 100;
	} else
		pdata->voltage_hysteresis = (int)temp;

	pr_info("%s : num_temp_limits(%d), num_volt_limits(%d), temp_hysteresis(%d), voltage_hysteresis(%d)\n",
			__func__,
			pdata->num_temp_limits, pdata->num_volt_limits,
			pdata->temp_hysteresis, pdata->voltage_hysteresis);

	ret = of_property_read_u32(np, "battery,full_check_count",
			&pdata->full_check_count);
	if (ret)
		pr_info("%s : full_check_count is empty\n", __func__);

	ret = of_property_read_u32(np, "battery,chg_recharge_vdrop",
			&pdata->chg_recharge_vdrop);
	if (ret) {
		pr_info("%s : chg_recharge_vdrop is empty\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "battery,chg_recharge_soc",
			&pdata->chg_recharge_soc);
	if (ret) {
		pr_info("%s : chg_recharge_soc is empty\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "battery,max_rawsoc",
			&pdata->max_rawsoc);
	if (ret) {
		pr_info("%s : max_rawsoc is empty\n", __func__);
		pdata->max_rawsoc = 10000;
	}

	ret = of_property_read_u32(np, "battery,charge_full_design",
				   &pdata->charge_full_design);
	if (ret) {
		pr_info("%s : charge_full_design is empty\n", __func__);
		pdata->charge_full_design = 300000;
	}

	pr_info("%s:DT parsing is done, vendor : %s\n",
			__func__, pdata->vendor);
	return ret;
}

static const struct of_device_id google_battery_match_table[] = {
	{ .compatible = "google,battery",},
	{},
};
MODULE_DEVICE_TABLE(of, google_battery_match_table);

static enum alarmtimer_restart bat_monitor_alarm(
	struct alarm *alarm, ktime_t now)
{
	struct battery_info *battery = container_of(alarm,
				struct battery_info, monitor_alarm);

	wake_lock(&battery->monitor_wake_lock);
	queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 0);

	return ALARMTIMER_NORESTART;
}

static int get_max_charge_cntl_limit(struct thermal_cooling_device *tcd, unsigned long *lvl) {
	*lvl = 3;
	return 0;
}

static int get_cur_charge_cntl_limit(struct thermal_cooling_device *tcd, unsigned long *lvl) {
	struct battery_info *battery = (struct battery_info *)tcd->devdata;
	*lvl = battery->thermal_level;
	return 0;
}

static int set_charge_cntl_limit(struct thermal_cooling_device *tcd, unsigned long lvl) {
	struct battery_info *battery = (struct battery_info *) tcd->devdata;
	bool first_trigger = false;
	int trigger_level = 2;
	if (lvl < 0 || lvl > 2)
		return -EINVAL;
	if (lvl == 0 && battery->thermal_level == 0) {
		// No update needed
		return 0;
	}
	if (battery->voltage_index == battery->pdata->num_volt_limits - 1) {
		dev_info(battery->dev, "Currently in last voltage stage, lower thermal throttle trip\n");
		trigger_level = 1;
	}
	if (lvl == trigger_level && battery->thermal_level < trigger_level) {
		// Transition from 0->1
		first_trigger = true;
		dev_info(battery->dev, "Received cooling device limit: %lu. Last recorded temp: %d\n", lvl, battery->wlc_temperature);
	}
	battery->thermal_level = lvl;

	if (!battery->wlc_connected) {
		// Exit early; only apply thermal throttling if WLC is connected
		if (first_trigger)
			dev_info(battery->dev, "WLC not connected, won't throttle\n");
		return 0;
	}

	if (battery->status == POWER_SUPPLY_STATUS_CHARGING && battery->thermal_level == trigger_level) {
		dev_info(battery->dev, "%s: Temperature high: disable charging.\n", __func__);
		// Turn off WLC
		set_wlc_status(battery, POWER_SUPPLY_STATUS_NOT_CHARGING);
		// Immediately run a battery monitoring iteration
		alarm_cancel(&battery->monitor_alarm);
		wake_lock(&battery->monitor_wake_lock);
		queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 0);
	}
	return 0;
}

static const struct thermal_cooling_device_ops wlc_cooling_ops = {
	.get_max_state = get_max_charge_cntl_limit,
	.get_cur_state = get_cur_charge_cntl_limit,
	.set_cur_state = set_charge_cntl_limit,
};

static int charging_override_get(void *data, u64 *val)
{
	struct battery_info *battery = data;
	*val = battery->charging_state_override;
	return 0;
}

static int charging_override_set(void *data, u64 val)
{
	struct battery_info *battery = data;
	if (val != 0 && val != 1) {
		return -EINVAL;
	}
	battery->charging_state_override = val;
	set_bat_status_by_cable(battery);
	return 0;
}

static int current_override_get(void *data, u64 *val)
{
	struct battery_info *battery = data;
	if (battery->charging_current < battery->charging_current_override) {
		dev_info(battery->dev, "Set current over safe charging current");
		*val = battery->charging_current;
	} else {
		*val = battery->charging_current_override;
	}
	return 0;
}

static int current_override_set(void *data, u64 val)
{
	struct battery_info *battery = data;
	if (val < 0 || val > 300) {
		return -EINVAL;
	}
	battery->charging_current_override = val;
	// Reset current tracking indices so that current will be recalculated
	battery->temp_index = -1;
	battery->voltage_index = -1;
	battery->charging_current = -1;
	alarm_cancel(&battery->monitor_alarm);
	wake_lock(&battery->monitor_wake_lock);
	queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 0);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(charging_debug_ops, charging_override_get, charging_override_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(current_debug_ops, current_override_get, current_override_set, "%lld\n");

static ssize_t show_charge_stop_level(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct battery_info *battery = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", battery->charge_stop_level);
}

static ssize_t set_charge_stop_level(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_info *battery = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (!battery->psy_battery) {
		dev_err(battery->dev, "battery psy is not ready");
		return -ENODATA;
	}

	if ((val == battery->charge_stop_level) ||
	    (val <= battery->charge_start_level) ||
	    (val > DEFAULT_CHARGE_STOP_LEVEL))
		return count;

	battery->charge_stop_level = val;
	if (battery->psy_battery)
		power_supply_changed(battery->psy_battery);

	return count;
}
static DEVICE_ATTR(charge_stop_level, 0660, show_charge_stop_level,
					    set_charge_stop_level);

static ssize_t
show_charge_start_level(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct battery_info *battery = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", battery->charge_start_level);
}

static ssize_t set_charge_start_level(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct battery_info *battery = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (!battery->psy_battery) {
		dev_err(battery->dev, "battery psy is not ready");
		return -ENODATA;
	}

	if ((val == battery->charge_start_level) ||
	    (val >= battery->charge_stop_level) ||
	    (val < DEFAULT_CHARGE_START_LEVEL))
		return count;

	battery->charge_start_level = val;
	if (battery->psy_battery)
		power_supply_changed(battery->psy_battery);

	return count;
}


static DEVICE_ATTR(charge_start_level, 0660,
		   show_charge_start_level, set_charge_start_level);


static int battery_create_fs_entries(struct battery_info *battery)
{
	struct dentry *ent;
	int ret = 0;

#ifdef CONFIG_DEBUG_FS
	battery->debug_root = debugfs_create_dir("google-battery", NULL);
	if (!battery->debug_root) {
		dev_err(battery->dev, "Couldn't create debug dir\n");
		ret = -ENOENT;
	} else {
		battery->charging_state_override = 0;
		ent = debugfs_create_file("charging_disable", S_IFREG | S_IWUSR | S_IRUGO,
				battery->debug_root, battery, &charging_debug_ops);
		if (!ent) {
			dev_err(battery->dev, "Couldn't create charging debug file\n");
			ret = -ENOENT;
		}
		battery->charging_current_override = 0;
		ent = debugfs_create_file("charging_current", S_IFREG | S_IWUSR | S_IRUGO,
				battery->debug_root, battery, &current_debug_ops);
		if (!ent) {
			dev_err(battery->dev, "Couldn't create current debug file\n");
			ret = -ENOENT;
		}
	}
#endif
	ret = device_create_file(battery->dev, &dev_attr_charge_stop_level);
	if (ret != 0) {
		pr_err("Failed to create charge_stop_level files, ret=%d\n",
		       ret);
	}

	ret = device_create_file(battery->dev, &dev_attr_charge_start_level);
	if (ret != 0) {
		pr_err("Failed to create charge_start_level files, ret=%d\n",
		       ret);
	}
	return ret;
}

static int google_battery_probe(struct platform_device *pdev)
{
	struct battery_info *battery;
	struct power_supply_config psy_cfg = {};
	union power_supply_propval value;
	int ret = 0, temp = 0;
	struct power_supply *psy;
	struct device_node *cooling_node = NULL;
#ifndef CONFIG_OF
	int i;
#endif

	pr_info("%s: Google battery driver loading\n", __func__);

	/* Allocate necessary device data structures */
	battery = kzalloc(sizeof(*battery), GFP_KERNEL);
	if (!battery)
		return -ENOMEM;

	pr_info("%s: battery is allocated\n", __func__);

	battery->pdata = devm_kzalloc(&pdev->dev, sizeof(*(battery->pdata)),
			GFP_KERNEL);
	if (!battery->pdata) {
		ret = -ENOMEM;
		goto err_bat_free;
	}

	pr_info("%s: pdata is allocated\n", __func__);

	/* Get device/board dependent configuration data from DT */
	temp = google_battery_parse_dt(&pdev->dev, battery);
	if (temp) {
		pr_info("%s: google_battery_parse_dt(&pdev->dev, battery) == %d\n", __func__, temp);
		dev_err(&pdev->dev, "%s: Failed to get battery dt\n", __func__);
		ret = -EINVAL;
		goto err_parse_dt_nomem;
	}

	pr_info("%s: DT parsing is done\n", __func__);

	/* Set driver data */
	platform_set_drvdata(pdev, battery);
	battery->dev = &pdev->dev;

	mutex_init(&battery->iolock);

	wake_lock_init(&battery->monitor_wake_lock, WAKE_LOCK_SUSPEND,
			"sec-battery-monitor");
	wake_lock_init(&battery->vbus_wake_lock, WAKE_LOCK_SUSPEND,
			"sec-battery-vbus");

	/* Inintialization of battery information */
	battery->status = POWER_SUPPLY_STATUS_DISCHARGING;
	battery->health = POWER_SUPPLY_HEALTH_GOOD;

	battery->charging_current = -1;
	battery->temp_index = -1;
	battery->voltage_index = -1;
	battery->topoff_current = -1;

	battery->temp_high = battery->pdata->temp_limits[battery->pdata->num_temp_limits - 1];
	battery->temp_high_recovery = battery->temp_high - battery->pdata->temp_hysteresis;
	battery->temp_low = battery->pdata->temp_limits[0];
	battery->temp_low_recovery = battery->temp_low + battery->pdata->temp_hysteresis;

	battery->max_rawsoc = battery->pdata->max_rawsoc;
	battery->charge_full_design = battery->pdata->charge_full_design;
	battery->charge_full = battery->charge_full_design;
	battery->charge_start_level = DEFAULT_CHARGE_START_LEVEL;
	battery->charge_stop_level = DEFAULT_CHARGE_STOP_LEVEL;

	battery->is_recharging = false;
	battery->power_supply_type = POWER_SUPPLY_TYPE_BATTERY;

	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy) {
		pr_err("%s: Fail to get charger psy\n", __func__);
		return -EINVAL;
	}
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &value);
	if (ret < 0) {
		pr_err("%s: Fail to execute property\n", __func__);
	} else {
		if (!value.intval)
			battery->battery_valid = false;
		else
			battery->battery_valid = true;
	}
	power_supply_put(psy);

	/* Register battery as "POWER_SUPPLY_TYPE_BATTERY" */
	battery->psy_battery_desc.name = "battery";
	battery->psy_battery_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	battery->psy_battery_desc.get_property =  battery_get_property;
	battery->psy_battery_desc.set_property =  battery_set_property;
	battery->psy_battery_desc.external_power_changed =  battery_external_power_changed;
	battery->psy_battery_desc.properties = google_battery_props;
	battery->psy_battery_desc.num_properties =  ARRAY_SIZE(google_battery_props);

	/* Register an AC power supply so charging registers correctly */
	battery->psy_usb_desc.name = "usb";
	battery->psy_usb_desc.type = POWER_SUPPLY_TYPE_MAINS;
	battery->psy_usb_desc.properties = charging_props;
	battery->psy_usb_desc.num_properties = ARRAY_SIZE(charging_props);
	battery->psy_usb_desc.get_property = usb_get_property;

	/* Register a wireless power supply for WLC power */
	battery->psy_wireless_desc.name = "wireless";
	battery->psy_wireless_desc.type = POWER_SUPPLY_TYPE_WIRELESS;
	battery->psy_wireless_desc.properties = wireless_props;
	battery->psy_wireless_desc.num_properties = ARRAY_SIZE(wireless_props);
	battery->psy_wireless_desc.get_property = wireless_get_property;

	/* Initialize work queue for periodic polling thread */
	battery->monitor_wqueue =
		create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!battery->monitor_wqueue) {
		dev_err(battery->dev,
				"%s: Fail to Create Workqueue\n", __func__);
		goto err_irr;
	}

	/* Init work & alarm for monitoring */
	INIT_DELAYED_WORK(&battery->monitor_work, bat_monitor_work);
	alarm_init(&battery->monitor_alarm, ALARM_BOOTTIME, bat_monitor_alarm);
	battery->monitor_alarm_interval = DEFAULT_ALARM_INTERVAL;

	/* Register power supply to framework */
	psy_cfg.drv_data = battery;

	battery->psy_battery = power_supply_register(&pdev->dev, &battery->psy_battery_desc, &psy_cfg);
	if (IS_ERR(battery->psy_battery)) {
		pr_err("%s: Failed to Register psy_battery\n", __func__);
		ret = PTR_ERR(battery->psy_battery);
		goto err_workqueue;
	}
	pr_info("%s: Registered battery as power supply\n", __func__);

	battery->psy_usb = power_supply_register(&pdev->dev, &battery->psy_usb_desc, &psy_cfg);
	if (IS_ERR(battery->psy_usb)) {
		pr_err("%s: Failed to Register psy_usb\n", __func__);
		ret = PTR_ERR(battery->psy_usb);
		goto err_unreg_battery;
	}
	pr_info("%s: Registered USB as power supply\n", __func__);

	battery->psy_wireless = power_supply_register(&pdev->dev, &battery->psy_wireless_desc, &psy_cfg);
	if (IS_ERR(battery->psy_wireless)) {
		pr_err("%s: Failed to Register psy_wireless\n", __func__);
		ret = PTR_ERR(battery->psy_wireless);
		goto err_unreg_usb;
	}
	pr_info("%s: Registered Wireless as power supply\n", __func__);

	/* check fuelgauge availability */
	psy = power_supply_get_by_name(battery->pdata->fuelgauge_name);
	if (!psy) {
		ret = -EINVAL;
		goto err_unreg_wireless;
	}
	power_supply_put(psy);
	// Assume lower bound of charge full spoofing
	battery->soc_spoof_full = (battery->pdata->chg_recharge_soc - 1) * 100;
	/* Initialize battery level */
	get_battery_info(battery);

	set_charging_current(battery, battery->pdata->default_current);
#if defined(CONFIG_MUIC_NOTIFIER)
	pr_info("%s: Register MUIC notifier\n", __func__);
	muic_notifier_register(&battery->batt_nb, battery_handle_notification,
			MUIC_NOTIFY_DEV_CHARGER);
#endif

	/* Set up wlc related variable */
	battery->wlc_connected = false;
	mutex_init(&battery->wlc_state_lock);

	/* Set up WLC cooling device */
	if (battery->pdata->wlc_cdev_name) {
		cooling_node = of_find_node_by_name(NULL, battery->pdata->wlc_cdev_name);
		if (!cooling_node) {
			pr_err("No %s OF node for cooling device\n", battery->pdata->wlc_cdev_name);
		} else {
			battery->tcd = thermal_of_cooling_device_register(cooling_node, battery->pdata->wlc_cdev_name, battery, &wlc_cooling_ops);
			battery->thermal_level = 0;
			battery->wlc_tzd = thermal_zone_get_zone_by_name(battery->pdata->wlc_tz_name);
			if (battery->wlc_tzd) {
				battery->wlc_tzd->ops->set_mode(battery->wlc_tzd, THERMAL_DEVICE_ENABLED);
			}
		}
	}

	battery_create_fs_entries(battery);

	/* Kick off monitoring thread */
	pr_info("%s: start battery monitoring work\n", __func__);
	queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 5*HZ);

	dev_info(battery->dev, "%s: Battery driver is loaded\n", __func__);
	return 0;
err_unreg_wireless:
	power_supply_unregister(battery->psy_wireless);
err_unreg_usb:
	power_supply_unregister(battery->psy_usb);
err_unreg_battery:
	power_supply_unregister(battery->psy_battery);
err_workqueue:
	destroy_workqueue(battery->monitor_wqueue);
err_irr:
	wake_lock_destroy(&battery->monitor_wake_lock);
	wake_lock_destroy(&battery->vbus_wake_lock);
	mutex_destroy(&battery->iolock);
err_parse_dt_nomem:
	kfree(battery->pdata);
err_bat_free:
	kfree(battery);

	return ret;
}

static int google_battery_remove(struct platform_device *pdev)
{
	return 0;
}

#if defined CONFIG_PM
static int google_battery_prepare(struct device *dev)
{
	struct battery_info *battery = dev_get_drvdata(dev);
	struct thermal_zone_device *tzd;

	/* If charger is connected, monitoring is required*/
	if (battery->power_supply_type == POWER_SUPPLY_TYPE_BATTERY) {
		alarm_cancel(&battery->monitor_alarm);
		battery->monitor_alarm_interval = SLEEP_ALARM_INTERVAL;
		dev_info(battery->dev, "%s: Increase battery monitoring interval -> %d\n",
				__func__, battery->monitor_alarm_interval);
		alarm_start_relative(&battery->monitor_alarm,
			ktime_set(battery->monitor_alarm_interval, 0));
	}

	/* Disable the thermal device to avoid a temperature read on-resume
	   This cuts 50-70ms from the critical path */
	tzd = thermal_zone_get_zone_by_name(battery->pdata->wlc_tz_name);
	if (tzd) {
		tzd->ops->set_mode(tzd, THERMAL_DEVICE_DISABLED);
	} else {
		pr_err("%s: Fail to get thermal zone device: %s\n",
		       __func__, battery->pdata->wlc_tz_name);
	}
	return 0;
}

static int google_battery_suspend(struct device *dev)
{
	return 0;
}

static int google_battery_resume(struct device *dev)
{
	return 0;
}

static void google_battery_complete(struct device *dev)
{
	struct battery_info *battery = dev_get_drvdata(dev);

	if (battery->monitor_alarm_interval != DEFAULT_ALARM_INTERVAL) {
		battery->monitor_alarm_interval = DEFAULT_ALARM_INTERVAL;
		dev_info(battery->dev, "%s: Recover battery monitoring interval -> %d\n",
			__func__, battery->monitor_alarm_interval);
		alarm_cancel(&battery->monitor_alarm);
		wake_lock(&battery->monitor_wake_lock);
		/* We need to delay long enough here to avoid re-enabling the thermal device
		   before resume is complete. If we don't, we will block resume for 50ms while
		   we wait for the thermistor reading to stabilize. */
		queue_delayed_work(battery->monitor_wqueue,
		                   &battery->monitor_work,
		                   msecs_to_jiffies(RESUME_DELAY_MS));
	}
}

#else
#define google_battery_prepare NULL
#define google_battery_suspend NULL
#define google_battery_resume NULL
#define google_battery_complete NULL
#endif

static const struct dev_pm_ops google_battery_pm_ops = {
	.prepare = google_battery_prepare,
	.suspend = google_battery_suspend,
	.resume = google_battery_resume,
	.complete = google_battery_complete,
};

static struct platform_driver google_battery_driver = {
	.driver         = {
		.name   = "google-battery",
		.owner  = THIS_MODULE,
		.pm     = &google_battery_pm_ops,
		.of_match_table = google_battery_match_table,
	},
	.probe          = google_battery_probe,
	.remove     = google_battery_remove,
};

static int __init google_battery_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&google_battery_driver);
	return ret;
}
late_initcall(google_battery_init);

static void __exit google_battery_exit(void)
{
	platform_driver_unregister(&google_battery_driver);
}
module_exit(google_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alice Sheng <alicesheng@google.com>");
MODULE_DESCRIPTION("Google Battery driver");
