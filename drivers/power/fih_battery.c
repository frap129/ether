/*
 * Driver for fih battery feature.
 *
 * Copyright (C) 2014 FIH co.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author:  PinyCHWu <pinychwu@fih-foxconn.com>
 *	    November 2014
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/of.h>
#endif
#include <linux/power/fih_battery.h>

#define MINI_UPDATE_PERIOD(X) do {if (X<60) X=60;} while (0)
//This should be the same order for the items of battery and charger info in power_info structure.
static enum power_supply_property fih_bat_info_props[] = {
	//battery part
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	//charger part
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL
};

//This should be the same order for the items of usb info in power_info structure.
static enum power_supply_property fih_usb_info_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW
};

//This should be the same order for the items of dc info in power_info structure.
static enum power_supply_property fih_dc_info_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
};

//This should be the same order for the items of bms info in power_info structure.
static enum power_supply_property fih_bms_info_props[] = {
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
};

//This should be the same order for the items of usb-parallel info in power_info structure.
static enum power_supply_property fih_usbp_info_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

struct power_info {
	//battery info
	int			bat_present;
	int			bat_status;
	int			bat_health;
	unsigned int		bat_cap;
	unsigned int		bat_vol_mV;
	int			bat_cur_mA;
	int			bat_temp_0p1C;

	//charger info
	int			chg_enable;
	int			chg_cur_max_mA;
	int			chg_input_cur_max_mA;
	int			chg_sys_lvl;

	//usb info
	int			usb_present;
	int			usb_online;
	int			usb_scope;
	int			usb_type;
	int			usb_vol_mV;

	//dc info
	int			dc_present;
	int			dc_online;
	int			dc_chg_enable;
	int			dc_input_cur_max_mA;

	//bms info
	int			bms_batr_ohm;
	int			bms_idr_ohm;

	//usbp info
	int			usbp_present;
	int			usbp_status;
	int			usbp_chg_enable;
	int			usbp_chg_cur_max_mA;
};

union data_union {
	int			info_array[26]; //The size should be same for struct power_info
	struct power_info	p_info;
};

struct fih_chip {
	struct device 		*dev;
	struct mutex		read_lock;

	struct delayed_work	monitor_work;
	unsigned long		update_time;
	int			int_flag;

	//fih battery power supply and battery/usb power supply data.
	struct power_supply	fih_bat;
	struct power_supply	*bat_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*dc_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*usbp_psy;
	union data_union	data;

	//QC charger report value without fih handle.
	int			qcchg_status;
	int			qcchg_health;
	int			qcchg_cap;

	int			fake_cap_en;
	int			fake_temp_en;
	int			otp_status;
	int			smooth_flag;
};

#define FIH_BAT_NONE_FLAG (0)
#define FIH_BAT_NOTIFY_FLAG (1)
#define FIH_BAT_SHUTDOWN_FLAG (2)

#define FIH_OTP_NONE	(0)
#define FIH_OTP_HOT	(1)
#define FIH_OTP_COLD	(2)

#define FIH_BAT_MONITOR_DELAY	(HZ * 10)
#define FIH_BAT_START_DELAY	(HZ)
#define FIH_BAT_EXTERNAL_DELAY	(1)
#define SHOW_INFO_THERSHOLD	3
#define REG_DUMP_THERSHOLD	10

static unsigned int cache_time = 1000;

/* ---------------------- Export functions ------------------------- */
static struct fih_chip* get_fih_chip_by_psy(void)
{
	struct power_supply *psy = NULL;
	struct fih_chip *chip = NULL;

	psy = power_supply_get_by_name(FIH_BATTERY_DEV_NAME);
	if (!psy) {
		pr_debug("[%s] No fih-battery power-supply\n", __func__);
		return NULL;
	}

	chip = container_of(psy, struct fih_chip, fih_bat);
	return chip;
}

static void fih_battery_finetune_capacity(struct fih_chip *chip)
{
	struct power_info *p = NULL;

	if (!chip) {
		pr_debug("[%s] fih_chip is null\n", __func__);
		return;
	}

	p = &chip->data.p_info;

	p->bat_cap = chip->qcchg_cap;
}

int fih_battery_fake_capacity(int cap)
{
	struct fih_chip *chip = NULL;

	chip = get_fih_chip_by_psy();
	if (!chip) {
		pr_debug("[%s] fih_chip is null\n", __func__);
		return -1;
	}

	if (cap >= 0) {
		chip->fake_cap_en = 1;
	} else {
		chip->fake_cap_en = 0;
	}

	return 0;
}
EXPORT_SYMBOL(fih_battery_fake_capacity);

int fih_battery_fake_temp(int fake_temp_en)
{
	struct fih_chip *chip = NULL;

	chip = get_fih_chip_by_psy();
	if (!chip) {
		pr_debug("[%s] fih_chip is null\n", __func__);
		return -1;
	}

	chip->fake_temp_en = fake_temp_en;

	return 0;
}
EXPORT_SYMBOL(fih_battery_fake_temp);

int fih_battery_status(int status)
{
	struct fih_chip *chip = NULL;
	struct power_info *p = NULL;

	chip = get_fih_chip_by_psy();
	if (!chip) {
		pr_debug("[%s] fih_chip is null\n", __func__);
		return status;
	}

	p = &chip->data.p_info;
	chip->qcchg_status = status;

	if (chip->otp_status) {
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else if (p->bat_cap >= 100) {
		return POWER_SUPPLY_STATUS_FULL;
	}

	return status;
}
EXPORT_SYMBOL(fih_battery_status);

int fih_battery_health(int health)
{
	struct fih_chip *chip = NULL;

	chip = get_fih_chip_by_psy();
	if (!chip) {
		pr_debug("[%s] fih_chip is null\n", __func__);
		return health;
	}

	chip->qcchg_health = health;

	if (chip->otp_status == FIH_OTP_HOT) {
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (chip->otp_status == FIH_OTP_COLD) {
		return POWER_SUPPLY_HEALTH_COLD;
	}

	return health;
}
EXPORT_SYMBOL(fih_battery_health);

int fih_battery_capacity(int capacity)
{
	struct fih_chip *chip = NULL;
	struct power_info *p = NULL;

	chip = get_fih_chip_by_psy();
	if (!chip) {
		pr_debug("[%s] fih_chip is null\n", __func__);
		return capacity;
	}

	p = &chip->data.p_info;
	if (chip->fake_cap_en)
		return capacity;

	chip->qcchg_cap = capacity;
	fih_battery_finetune_capacity(chip);

	return p->bat_cap;
}
EXPORT_SYMBOL(fih_battery_capacity);

/* ---------------------- Power supply functions ------------------------- */
static enum power_supply_property fih_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int fih_battery_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct fih_chip *chip = container_of(psy, struct fih_chip, fih_bat);

	if (!chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int fih_battery_set_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct fih_chip *chip = container_of(psy, struct fih_chip, fih_bat);

	if (!chip)
		return -EINVAL;

	if (!chip->bat_psy)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void fih_external_power_changed(struct power_supply *psy)
{
	struct fih_chip *chip = container_of(psy, struct fih_chip, fih_bat);

	if (!chip)
		return;

	pr_debug("[%s]\n", __func__);
	chip->int_flag = FIH_BAT_NOTIFY_FLAG;
	cancel_delayed_work(&chip->monitor_work);
	schedule_delayed_work(&chip->monitor_work, FIH_BAT_EXTERNAL_DELAY);
}

/* ----------------------------- monitor & irq handler ------------------------ */
static int check_u2m_props(enum power_supply_property psp)
{
	switch(psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return 1;
	default:
		break;
	}
	return 0;
}

static int fih_read_power_info(struct fih_chip *chip, char *psy_name)
{
	struct power_supply *psy = NULL;
	union power_supply_propval ret = {0,};
	int i = 0, j = 0, prop_size = 0;
	enum power_supply_property *info_props = NULL;

	if (!strcmp(psy_name, "battery")) {
		psy = chip->bat_psy;
		j = 0;
		prop_size = ARRAY_SIZE(fih_bat_info_props);
		info_props = fih_bat_info_props;
	} else if (!strcmp(psy_name, "usb")) {
		psy = chip->usb_psy;
		j = ARRAY_SIZE(fih_bat_info_props);
		prop_size = ARRAY_SIZE(fih_usb_info_props);
		info_props = fih_usb_info_props;
	} else if (!strcmp(psy_name, "dc")) {
		psy = chip->dc_psy;
		j = ARRAY_SIZE(fih_bat_info_props) + ARRAY_SIZE(fih_usb_info_props);
		prop_size = ARRAY_SIZE(fih_dc_info_props);
		info_props = fih_dc_info_props;
	} else if (!strcmp(psy_name, "bms")) {
		psy = chip->bms_psy;
		j = ARRAY_SIZE(fih_bat_info_props) + ARRAY_SIZE(fih_usb_info_props) + ARRAY_SIZE(fih_dc_info_props);
		prop_size = ARRAY_SIZE(fih_bms_info_props);
		info_props = fih_bms_info_props;
	} else if (!strcmp(psy_name, "usb-parallel")) {
		psy = chip->usbp_psy;
		j = ARRAY_SIZE(fih_bat_info_props) + ARRAY_SIZE(fih_usb_info_props) + ARRAY_SIZE(fih_dc_info_props) + ARRAY_SIZE(fih_bms_info_props);
		prop_size = ARRAY_SIZE(fih_usbp_info_props);
		info_props = fih_usbp_info_props;
	} else {
		pr_err("[%s] No such %s power-supply prop info\n", __func__, psy_name);
		return -EINVAL;
	}

	if (psy == NULL) {
		pr_debug("No such %s power-supply\n", psy_name);
		return -EINVAL;
	}

	for (i = 0; i < prop_size; i++, j++, ret.intval = 0) {
		if (!psy->get_property(psy, info_props[i], &ret)) {
			if (check_u2m_props(info_props[i]))
				chip->data.info_array[j] = (ret.intval / 1000);
			else
				chip->data.info_array[j] = ret.intval;
		} else {
			pr_err("[%s] Get %s props %d failed\n", __func__, psy_name, info_props[i]);
		}
	}

	return 0;
}

extern int fih_get_usbin_voltage_now(void);
extern void fih_set_alert_info(u8 is_alert, bool byte_location, u8 val);
extern void fih_check_fastchg_current_comp(int temp);
extern int sensor_get_temp(uint32_t sensor_id, long *temp);
static void fih_update_status(struct fih_chip *chip, int soc_int_bit)
{
	static struct power_info old_p = {
		//battery part
		-1, -1, -1, 0, 0, 0, -99999,
		//charger part
		-1, 0, 0, 0,
		//usb part
		-1, -1, 0, 0, 0,
		//dc part
		-1, -1, -1, 0,
		//bms part
		0, 0,
		//usb-parallel
		-1, -1, -1, -1
	};
	static int old_qcchg_cap = -1;
	static int show_info = 0;
	static int err_counter = 0;
	struct power_info *p = &chip->data.p_info;
	int usbin_vol = 0;
	long msm_therm = 0;
	long pmic = 0;

	if ((!soc_int_bit) && chip->update_time && time_before(jiffies, chip->update_time +  msecs_to_jiffies(cache_time)))
		return;

	chip->update_time = jiffies;

	mutex_lock(&chip->read_lock);
	if (fih_read_power_info(chip, "battery")) {
		pr_err("[%s] battery power-supply not ready\n", __func__);
		goto read_failed;
	}

	if (fih_read_power_info(chip, "usb")) {
		pr_err("[%s] usb power-supply not ready\n", __func__);
	}

	if (fih_read_power_info(chip, "bms")) {
		pr_err("[%s] dc power-supply not ready\n", __func__);
	}

	if (soc_int_bit == FIH_BAT_NONE_FLAG)
		show_info++;
	else if (soc_int_bit == FIH_BAT_SHUTDOWN_FLAG)
		show_info = SHOW_INFO_THERSHOLD;

	if ((old_p.bat_present != p->bat_present) || (old_p.bat_status != p->bat_status) || (old_p.bat_health != p->bat_health) ||
	(old_qcchg_cap != chip->qcchg_cap) || (old_p.bat_cap != p->bat_cap) || (abs(old_p.bat_temp_0p1C - p->bat_temp_0p1C) >= 10) ||
	(old_p.chg_enable != p->chg_enable) || (old_p.chg_cur_max_mA !=  p->chg_cur_max_mA) ||
	(old_p.usb_present != p->usb_present) || (old_p.usb_online != p->usb_online) ||
	(old_p.dc_present != p->dc_present) || (old_p.dc_online != p->dc_online) || (old_p.dc_chg_enable != p->dc_chg_enable) ||
	(old_p.usbp_present != p->usbp_present) || (old_p.usbp_status != p->usbp_status) || (old_p.usbp_chg_enable != p->usbp_chg_enable)) {
		old_p = *p;
		old_qcchg_cap = chip->qcchg_cap;
		show_info = SHOW_INFO_THERSHOLD;
		power_supply_changed(&chip->fih_bat);
	}

	usbin_vol = fih_get_usbin_voltage_now();

	if (show_info >= SHOW_INFO_THERSHOLD) {
		show_info = 0;
		sensor_get_temp(15, &msm_therm);
		sensor_get_temp(19, &pmic);
		pr_info("[%s] PRE: %d, STS: %d/%d, HEL: %d/%d, CAP: %d/%d/%d, VOL: %d, CUR: %d, TMP: %d/%d, CEN: %d, CUM: %d, ICM: %d, "
			"STL: %d, UPR: %d, UON: %d, UTP: %d, UVL: %d, IDR: %d, MTM: %ld, PMIC: %ld\n", __func__,
		p->bat_present, chip->qcchg_status, p->bat_status, chip->qcchg_health, p->bat_health, chip->qcchg_cap, p->bat_cap, chip->fake_cap_en,
		p->bat_vol_mV, p->bat_cur_mA, p->bat_temp_0p1C, chip->fake_temp_en, p->chg_enable, p->chg_cur_max_mA,
		p->chg_input_cur_max_mA, p->chg_sys_lvl, p->usb_present, p->usb_online, p->usb_type, usbin_vol, p->bms_idr_ohm, msm_therm, pmic);

		fih_check_fastchg_current_comp(p->bat_temp_0p1C);

		if((p->usb_type != 0) && (p->chg_cur_max_mA <= 0))
		{
			err_counter++;
		}
		else
		{
			if(err_counter - 1 < 0)
				err_counter = 0;
			else
				err_counter--;
		}

		if(err_counter >= REG_DUMP_THERSHOLD)
		{
			fih_set_alert_info(BIT(0), 1, BIT(4));
			err_counter = 0;
		}

	}

read_failed:
	mutex_unlock(&chip->read_lock);
	return;
}

static void battery_monitor_work(struct work_struct *work)
{
	struct fih_chip *chip = container_of(work, struct fih_chip, monitor_work.work);
	int int_bit = chip->int_flag;

	if (int_bit == FIH_BAT_NOTIFY_FLAG) {
		pr_debug("[%s] This is nofity interrupt\n", __func__);
	}
	chip->int_flag = FIH_BAT_NONE_FLAG;
	fih_update_status(chip, int_bit);

	schedule_delayed_work(&chip->monitor_work, FIH_BAT_MONITOR_DELAY);
}

/* ----------------------------- Driver functions ----------------------------- */
#ifdef CONFIG_OF
static void parse_dts_to_pdata(struct fih_platform_data *pdata, struct device_node *dev_node)
{
	return;
}
#endif

static int fih_battery_probe(struct platform_device *pdev)
{
	struct fih_platform_data *pdata = pdev->dev.platform_data;
	struct fih_chip *chip = NULL;
	struct power_supply *bat_psy = NULL, *usb_psy = NULL, *dc_psy = NULL, *bms_psy = NULL, *usbp_psy = NULL;
	int retval = 0;
#ifdef CONFIG_OF
	struct device_node *dev_node = pdev->dev.of_node;
	int pdata_alloc = 0;
#endif

	pr_info("[%s]\n", __func__);

	bat_psy = power_supply_get_by_name("battery");
	if (!bat_psy) {
		pr_err("Battery supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("USB supply not found, deferring probe\n");
		bat_psy = NULL;
		return -EPROBE_DEFER;
	}

	dc_psy = power_supply_get_by_name("dc");
	if (!dc_psy) {
		pr_err("DC supply not found, deferring probe\n");
		bat_psy = usb_psy = NULL;
		return -EPROBE_DEFER;
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!dc_psy) {
		pr_err("DC supply not found, deferring probe\n");
		bat_psy = usb_psy = dc_psy = NULL;
		return -EPROBE_DEFER;
	}

	usbp_psy = power_supply_get_by_name("usb-parallel");
	if (!dc_psy) {
		pr_err("DC supply not found, deferring probe\n");
		bat_psy = usb_psy = dc_psy = bms_psy = NULL;
		return -EPROBE_DEFER;
	}

	//original chip data
	chip = (struct fih_chip*)kzalloc(sizeof(struct fih_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("[%s] Failed to allocate chip data\n", __func__);
		retval = -ENOMEM;
		goto err_mem;
	}

#ifdef CONFIG_OF
	if (!pdata && !dev_node) {
		pr_err("[%s] No pdata and dts\n", __func__);
		goto err_pdata;
	} else if (!pdata) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			goto err_pdata;
		pdata_alloc = 1;
	}

	if (dev_node)
		parse_dts_to_pdata(pdata, dev_node);
#else
	if (!pdata) {
		pr_err("[%s] No pdata\n", __func__);
		goto err_pdata;
	}
#endif

	//chip data init
	chip->dev = &pdev->dev;
	chip->int_flag = FIH_BAT_NONE_FLAG;
	chip->bat_psy = bat_psy;
	chip->usb_psy = usb_psy;
	chip->dc_psy = dc_psy;
	chip->bms_psy = bms_psy;
	chip->usbp_psy = usbp_psy;
	mutex_init(&chip->read_lock);
	platform_set_drvdata(pdev, chip);
	INIT_DELAYED_WORK(&chip->monitor_work, battery_monitor_work);

	//init power supply property
	chip->fih_bat.name		= FIH_BATTERY_DEV_NAME;
	chip->fih_bat.type		= POWER_SUPPLY_TYPE_UNKNOWN;
	chip->fih_bat.properties	= fih_battery_props;
	chip->fih_bat.num_properties	= ARRAY_SIZE(fih_battery_props);
	chip->fih_bat.get_property	= fih_battery_get_prop;
	chip->fih_bat.set_property	= fih_battery_set_prop;
	chip->fih_bat.external_power_changed= fih_external_power_changed;

	retval = power_supply_register(chip->dev, &chip->fih_bat); //create battery sysfs
	if (retval) {
		pr_err("[%s] Failed to register battery:(%d)\n", __func__, retval);
		goto err_psy;
	}

	schedule_delayed_work(&chip->monitor_work, FIH_BAT_START_DELAY);

#ifdef CONFIG_OF
	if (pdata_alloc)
		kfree(pdata);
#endif

	return 0;

err_psy:
	platform_set_drvdata(pdev, NULL);
#ifdef CONFIG_OF
	if (pdata_alloc)
		kfree(pdata);
#endif
err_pdata:
	kfree(chip);
err_mem:
	bat_psy = usb_psy = dc_psy = NULL;
	return retval;
}

static int fih_battery_remove(struct platform_device *pdev)
{
	struct fih_chip *chip = platform_get_drvdata(pdev);
	pr_info("[%s]\n", __func__);

	cancel_delayed_work(&chip->monitor_work);
	power_supply_unregister(&chip->fih_bat);
	platform_set_drvdata(pdev, NULL);
	kfree(chip);
	return 0;
}

static void fih_battery_shutdown(struct platform_device *pdev)
{
	struct fih_chip *chip = platform_get_drvdata(pdev);
	pr_info("[%s]\n", __func__);

	cancel_delayed_work(&chip->monitor_work);
	flush_delayed_work(&chip->monitor_work);

	fih_update_status(chip, FIH_BAT_SHUTDOWN_FLAG);
}

#ifdef CONFIG_PM
static int fih_battery_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct fih_chip *chip = platform_get_drvdata(pdev);
	cancel_delayed_work(&chip->monitor_work);
	flush_delayed_work(&chip->monitor_work);
	return 0;
}

static int fih_battery_resume(struct platform_device *pdev)
{
	struct fih_chip *chip = platform_get_drvdata(pdev);
	schedule_delayed_work(&chip->monitor_work, HZ / 10);
	return 0;
}
#else
#define fih_battery_suspend	NULL
#define fih_battery_resume	NULL
#endif

#ifdef CONFIG_OF
static const struct of_device_id fih_battery_match[] = {
	{.compatible = "fih,battery"},
	{}
};
#endif

static struct platform_driver fih_battery_driver = {
	.driver = {
		.name = FIH_BATTERY_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = fih_battery_match,
#endif
	},
	.probe		= fih_battery_probe,
	.remove		= fih_battery_remove,
	.suspend	= fih_battery_suspend,
	.resume		= fih_battery_resume,
	.shutdown	= fih_battery_shutdown,
};

static int __init fih_battery_init(void)
{
	return platform_driver_register(&fih_battery_driver);
}

static void __exit fih_battery_exit(void)
{
	platform_driver_unregister(&fih_battery_driver);
}

module_init(fih_battery_init);
module_exit(fih_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("PinyCHWu <pinychwu@fih-foxconn.com>");
MODULE_DESCRIPTION("fih battery driver");
