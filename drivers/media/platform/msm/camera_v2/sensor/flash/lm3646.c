/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_led_flash.h"

#define FLASH_NAME "ti,lm3646"

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

//#define CREATE_VIRTUAL_FILE
/**
 * All current values are in [uA]
 */
#define FLASH_REG_MIN		(0)
#define FLASH_REG_OFFSET	(1)
#define FLASH_REG_MAX		(127)
#define FLASH_CURRENT_MIN	(0)
#define FLASH_CURRENT_OFFSET	(23040)
#define FLASH_CURRENT_MAX	(1499600)
#define FLASH_CURRENT_STEP	(((FLASH_CURRENT_MAX - FLASH_CURRENT_OFFSET) + (FLASH_REG_MAX - FLASH_REG_OFFSET) / 2) / (FLASH_REG_MAX - FLASH_REG_OFFSET))
#define FLASH_CURRENT(reg)	((reg) < FLASH_REG_OFFSET ? FLASH_CURRENT_MIN : ((reg) - FLASH_REG_OFFSET) * FLASH_CURRENT_STEP + FLASH_CURRENT_OFFSET)
#define FLASH_REG(_i_)		((_i_) < FLASH_CURRENT_OFFSET ? 0 : ((_i_) - FLASH_CURRENT_OFFSET + FLASH_CURRENT_STEP / 2) / FLASH_CURRENT_STEP + FLASH_REG_OFFSET)

#define TORCH_REG_MIN		(0)
#define TORCH_REG_OFFSET	(1)
#define TORCH_REG_MAX		(127)
#define TORCH_CURRENT_MIN	(0)
#define TORCH_CURRENT_OFFSET	(2530)
#define TORCH_CURRENT_MAX	(187100)
#define TORCH_CURRENT_STEP	(((TORCH_CURRENT_MAX - TORCH_CURRENT_OFFSET) + (TORCH_REG_MAX - TORCH_REG_OFFSET) / 2) / (TORCH_REG_MAX - TORCH_REG_OFFSET))
#define TORCH_CURRENT(reg)	((reg) < TORCH_REG_OFFSET ? TORCH_CURRENT_MIN : ((reg) - TORCH_REG_OFFSET) * TORCH_CURRENT_STEP + TORCH_CURRENT_OFFSET)
#define TORCH_REG(_i_)		((_i_) < TORCH_CURRENT_OFFSET ? 0 : ((_i_) - TORCH_CURRENT_OFFSET + TORCH_CURRENT_STEP / 2) / TORCH_CURRENT_STEP + TORCH_REG_OFFSET)

#define MAX_FLASH_REG_MIN	(0)
#define MAX_FLASH_REG_MAX	(15)
#define MAX_FLASH_CURRENT_MIN	(93350)
#define MAX_FLASH_CURRENT_MAX	(1499600)
#define MAX_FLASH_CURRENT_STEP	(((MAX_FLASH_CURRENT_MAX - MAX_FLASH_CURRENT_MIN) + (MAX_FLASH_REG_MAX - MAX_FLASH_REG_MIN) / 2) / (MAX_FLASH_REG_MAX - MAX_FLASH_REG_MIN))
#define MAX_FLASH_CURRENT(reg)	((reg) < MAX_FLASH_REG_MIN ? MAX_FLASH_CURRENT_MIN : (reg) * MAX_FLASH_CURRENT_STEP + MAX_FLASH_CURRENT_MIN)
#define MAX_FLASH_REG(_i_)	((_i_) < MAX_FLASH_CURRENT_MIN ? 0 : ((_i_) - MAX_FLASH_CURRENT_MIN + MAX_FLASH_CURRENT_STEP / 2) / MAX_FLASH_CURRENT_STEP + MAX_FLASH_REG_MIN)

#define MAX_TORCH_REG_MIN	(0)
#define MAX_TORCH_REG_MAX	(7)
#define MAX_TORCH_CURRENT_MIN	(23040)
#define MAX_TORCH_CURRENT_MAX	(187100)
#define MAX_TORCH_CURRENT_STEP	(((MAX_TORCH_CURRENT_MAX - MAX_TORCH_CURRENT_MIN) + (MAX_TORCH_REG_MAX - MAX_TORCH_REG_MIN) / 2) / (MAX_TORCH_REG_MAX - MAX_TORCH_REG_MIN))
#define MAX_TORCH_CURRENT(reg)	((reg) < MAX_TORCH_REG_MIN ? MAX_TORCH_CURRENT_MIN : (reg) * MAX_TORCH_CURRENT_STEP + MAX_TORCH_CURRENT_MIN)
#define MAX_TORCH_REG(_i_)	((_i_) < MAX_TORCH_CURRENT_MIN ? 0 : ((_i_) - MAX_TORCH_CURRENT_MIN + MAX_TORCH_CURRENT_STEP / 2) / MAX_TORCH_CURRENT_STEP + MAX_TORCH_REG_MIN)

#define MAX_REG(_t_, _f_)	(((MAX_TORCH_REG(_t_) & 0x7)<<4) | (MAX_FLASH_REG(_f_) & 0xf))
/**
 * All ramp up values are in [usec]
 */
#define FLASH_RAMPUP_REG_MIN    (0)
#define FLASH_RAMPUP_REG_MAX    (7)
#define FLASH_RAMPUP_TIME_MIN   (256)
#define FLASH_RAMPUP_TIME_MAX   (FLASH_RAMPUP_TIME_MIN << FLASH_RAMPUP_REG_MAX)
#define FLASH_RAMPUP_TIME(reg)  (FLASH_RAMPUP_TIME_MIN << (reg))
#define FLASH_RAMPUP_REG(_t_)   (log2((_t_) / FLASH_RAMPUP_TIME_MIN))
/**
 * All time out values are in [msec]
 */
#define FLASH_TIMEOUT_REG_MIN   (0)
#define FLASH_TIMEOUT_REG_MAX   (7)
#define FLASH_TIMEOUT_TIME_MIN  (50)
#define FLASH_TIMEOUT_TIME_MAX  (400)
#define FLASH_TIMEOUT_TIME_STEP (((FLASH_TIMEOUT_TIME_MAX - FLASH_TIMEOUT_TIME_MIN) + (FLASH_TIMEOUT_REG_MAX - FLASH_TIMEOUT_REG_MIN) / 2) / (FLASH_TIMEOUT_REG_MAX - FLASH_TIMEOUT_REG_MIN))
#define FLASH_TIMEOUT_TIME(reg) ((reg) < FLASH_TIMEOUT_REG_MIN ? FLASH_TIMEOUT_TIME_MIN : (reg) * FLASH_TIMEOUT_TIME_STEP + FLASH_TIMEOUT_TIME_MIN)
#define FLASH_TIMEOUT_REG(_t_)  ((_t_) < FLASH_TIMEOUT_TIME_MIN ? 0 : ((_t_) - FLASH_TIMEOUT_TIME_MIN + FLASH_TIMEOUT_TIME_STEP / 2) / FLASH_TIMEOUT_TIME_STEP + FLASH_TIMEOUT_REG_MIN)
/**
 * Maximum current supported by HW serup in [uA]
 */
#define MAX_CURRENT_FLASH	(1218350)
#define MAX_CURRENT_TORCH	(163660)

#define LED_AMBER		(0x00)
#define LED_COOL		(0x01)
#define LED1_SELECT		LED_COOL

#define REG_SILICON_REVISION	(0x00)
#define	REG_ENABLE		(0x01)
#define REG_FLASH_TIME_CTRL	(0x04)
#define	REG_MAX_I_CTRL		(0x05)
#define REG_LED1_FLASH_I_CTRL	(0x06)
#define REG_LED1_TORCH_I_CTRL	(0x07)

#define	TORCH_I_SHIFT		(4)
#define	FLASH_I_SHIFT		(0)
/**
 * REG_ENABLE modes
 */
#define MODE_FLASH		(0xE3)
#define MODE_TORCH		(0xE2)
#define MODES_STASNDBY		(0xE0)
/**
 * REG_FLASH_TIME_CTRL reg IVFM_MODULATION bit values
 */
#define IVFM_MODULATION_DOWN	(0x00)
#define IVFM_MODULATION_UP_DOWN	(0x01)
/**
 * REG_FLASH_TIME_CTRL reg STROBE_USAGE bit values
 */
#define STROBE_USAGE_LEVEL	(0x00)
#define STROBE_USAGE_EDGE	(0x01)

/**
 * @note: Absolute maximum current limits
 * Values below are defined by HW limitations!
 * Torch - REG_MAX_I_CTRL[6:4] = 0x6 (163.66[mA])
 * Flash - REG_MAX_I_CTRL[3:0] = 0xC (1218.35[mA])
 */
#define MAX_I_LIMIT		(0x6C)
/**
 * @note: Absolute maximum torch current limit
 * Value below is defined by HW limitations!
 * REG_LED1_TORCH_I_CTRL[6:0] = 0x6F (163.66[mA])
 */
#define TORCH_I_LIMIT		(0x6F)
/**
 * @note: Absolute maximum flash current limit
 * Value below is defined by HW limitations!
 * REG_LED1_FLASH_I_CTRL[6:0] = 0x67 (1218.35[mA])
 */
#define FLASH_I_LIMIT		(0x67)

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3646_i2c_driver;
static struct msm_led_flash_ctrl_t *P_fctrl;

#ifdef CREATE_VIRTUAL_FILE
static struct class *FlashLED_class;
#endif

static struct msm_camera_i2c_reg_array lm3646_init_array[] = {
	{REG_ENABLE, MODES_STASNDBY},
	{REG_FLASH_TIME_CTRL, (uint8_t)((IVFM_MODULATION_DOWN<<7) | (STROBE_USAGE_EDGE<<6) | (FLASH_RAMPUP_REG_MIN<<3) | FLASH_TIMEOUT_REG_MAX)},
};

static struct msm_camera_i2c_reg_array lm3646_off_array[] = {
	{REG_ENABLE, MODES_STASNDBY},
};

static struct msm_camera_i2c_reg_array lm3646_release_array[] = {
	{REG_ENABLE, MODES_STASNDBY},
};

static struct msm_camera_i2c_reg_array lm3646_low_array[] = {
	{REG_MAX_I_CTRL, MAX_I_LIMIT},
	{REG_LED1_TORCH_I_CTRL, TORCH_I_LIMIT},
	{REG_ENABLE, MODE_TORCH},
};

static struct msm_camera_i2c_reg_array lm3646_high_array[] = {
	{REG_MAX_I_CTRL, MAX_I_LIMIT},
	{REG_LED1_FLASH_I_CTRL, FLASH_I_LIMIT},
	{REG_ENABLE, MODE_FLASH},
};

static void __exit msm_flash_lm3646_i2c_remove(void)
{
	i2c_del_driver(&lm3646_i2c_driver);
	return;
}

static const struct of_device_id lm3646_trigger_dt_match[] = {
	{.compatible = FLASH_NAME, .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, lm3646_trigger_dt_match);

static const struct i2c_device_id lm3646_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int msm_flash_lm3646_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	pr_err("msm_flash_lm3646_i2c_probe ++");
	if (!id) {
		pr_err("msm_flash_lm3646_i2c_probe: id is NULL");
		id = lm3646_i2c_id;
	}

	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver lm3646_i2c_driver = {
	.id_table = lm3646_i2c_id,
	.probe  = msm_flash_lm3646_i2c_probe,
	.remove = __exit_p(msm_flash_lm3646_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3646_trigger_dt_match,
	},
};

void lm3646_pre_init_attribute(struct msm_led_flash_ctrl_t *fctrl)
{
	P_fctrl = fctrl;
}

static int lm3646_flash_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc;
	uint8_t i;
	uint32_t max_current = 0; // sum of LED1 and LED2 currents [uA]
	uint32_t max_reg; // sum of LED1 and LED2 currents register value
	uint32_t led1_current; // LED1 current [uA]
	uint32_t led1_reg; // sum of LED1 current register value
	struct msm_camera_i2c_reg_array *reg_array;

	for (i=0;i<fctrl->torch_num_sources;i++) {
		max_current += fctrl->torch_op_current[i];
	}
	max_current *= 1000; // convert [mA] to [uA]
	led1_current = fctrl->torch_op_current[LED1_SELECT] * 1000; // convert [mA] to [uA]
	if (0 == max_current) {
              max_current = 120 * 1000;
              led1_current = max_current;
	}
	max_reg = MAX_REG(max_current, 0);
	led1_reg = TORCH_REG(led1_current);
	reg_array = fctrl->reg_setting->low_setting->reg_setting;
	// set MAX current
	reg_array[0].reg_addr = REG_MAX_I_CTRL;
	reg_array[0].reg_data = max_reg;
	// set LED1 current
	reg_array[1].reg_addr = REG_LED1_TORCH_I_CTRL;
	reg_array[1].reg_data = led1_reg;
	// enable Torch
	reg_array[2].reg_addr = REG_ENABLE;
	reg_array[2].reg_data = MODE_TORCH;
	// apply Torch settings
	rc = msm_flash_led_low(fctrl);
	CDBG("%s: max= %7d[uA](0x%02x), led1= %7d[uA](0x%4x)\n", __func__, max_current, max_reg, led1_current, led1_reg);

	return(rc);
}

static int lm3646_flash_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc;
	uint8_t i;
	uint32_t max_current = 0; // sum of LED1 and LED2 currents [uA]
	uint32_t max_reg; // sum of LED1 and LED2 currents register value
	uint32_t led1_current; // LED1 current [uA]
	uint32_t led1_reg; // sum of LED1 current register value
	struct msm_camera_i2c_reg_array *reg_array;

	for (i=0;i<fctrl->flash_num_sources;i++) {
		max_current += fctrl->flash_op_current[i];
	}
	max_current *= 1000; // convert [mA] to [uA]
	max_reg = MAX_REG(0, max_current);
	led1_current = fctrl->flash_op_current[LED1_SELECT] * 1000; // convert [mA] to [uA]
	led1_reg = FLASH_REG(led1_current);
	reg_array = fctrl->reg_setting->high_setting->reg_setting;
	// set MAX current
	reg_array[0].reg_addr = REG_MAX_I_CTRL;
	reg_array[0].reg_data = max_reg;
	// set LED1 current
	reg_array[1].reg_addr = REG_LED1_FLASH_I_CTRL;
	reg_array[1].reg_data = led1_reg;
	// enable Flash
	reg_array[2].reg_addr = REG_ENABLE;
	reg_array[2].reg_data = MODE_FLASH;
	// apply Flash settings
	rc = msm_flash_led_high(fctrl);
	CDBG("%s: max= %7d[uA](0x%02x), led1= %7d[uA](0x%4x)", __func__, max_current, max_reg, led1_current, led1_reg);

	return(rc);
}

#ifdef CREATE_VIRTUAL_FILE
static ssize_t FlashLED1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned long FlashLED_en = 33;
	pr_err("Virtual file lm3646");
	
     return sprintf(buf, "%lu\n", FlashLED_en);
}

static ssize_t FlashLED1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;

	if(!strncmp(buf, "on", 2))
	{
		pr_err("FlashLED_store on");
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_MAX_I_CTRL, 0x7F, MSM_CAMERA_I2C_BYTE_DATA);
		
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_LED1_FLASH_I_CTRL, 0x7F, MSM_CAMERA_I2C_BYTE_DATA);
		
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_ENABLE, MODE_FLASH, MSM_CAMERA_I2C_BYTE_DATA);

		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	if(!strncmp(buf, "off", 3))
	{
		pr_err("FlashLED_store off");
		rc = P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			P_fctrl->flash_i2c_client,
			P_fctrl->reg_setting->off_setting);
	}
	if(!strncmp(buf, "torch", 5))
	{
		pr_err("FlashLED_store torch");
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_MAX_I_CTRL, 0x7F, MSM_CAMERA_I2C_BYTE_DATA);
		
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_LED1_TORCH_I_CTRL, 0x7F, MSM_CAMERA_I2C_BYTE_DATA);
		
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_ENABLE, MODE_TORCH, MSM_CAMERA_I2C_BYTE_DATA);

		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

       return count;
}

static ssize_t FlashLED2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned long FlashLED_en = 33;
	pr_err("Virtual file lm3646");
	
     return sprintf(buf, "%lu\n", FlashLED_en);
}

static ssize_t FlashLED2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;

	if(!strncmp(buf, "on", 2))
	{
		pr_err("FlashLED_store on");
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_MAX_I_CTRL, 0x7F, MSM_CAMERA_I2C_BYTE_DATA);
		
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_LED1_FLASH_I_CTRL, 0, MSM_CAMERA_I2C_BYTE_DATA);
		
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_ENABLE, MODE_FLASH, MSM_CAMERA_I2C_BYTE_DATA);

		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	if(!strncmp(buf, "off", 3))
	{
		pr_err("FlashLED_store off");
		rc = P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			P_fctrl->flash_i2c_client,
			P_fctrl->reg_setting->off_setting);
	}
	if(!strncmp(buf, "torch", 5))
	{
		pr_err("FlashLED_store torch");
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_MAX_I_CTRL, 0x7F, MSM_CAMERA_I2C_BYTE_DATA);
		
		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_LED1_TORCH_I_CTRL, 0, MSM_CAMERA_I2C_BYTE_DATA);

		P_fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			P_fctrl->flash_i2c_client, REG_ENABLE, MODE_TORCH, MSM_CAMERA_I2C_BYTE_DATA);

		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

       return count;
}


static struct device_attribute led_device_attributes[] = {
        __ATTR(flash_on_off, 0664, FlashLED1_show, FlashLED1_store),
		__ATTR(flash_on_off2, 0664, FlashLED2_show, FlashLED2_store),
        __ATTR_NULL,
};

static int create_class_led_file(void)
{
    FlashLED_class = class_create(THIS_MODULE, "camera");
        if (IS_ERR(FlashLED_class)) {
                pr_err("Unable to create camera class; errno = %ld\n", PTR_ERR(FlashLED_class));
                return PTR_ERR(FlashLED_class);
        }
        
        FlashLED_class->dev_attrs = led_device_attributes;
    device_create(FlashLED_class, NULL, 0, NULL, "led");
        return 0;
}
#endif

static int msm_flash_lm3646_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;

	pr_err("msm_flash_lm3646_platform_probe ++");

	match = of_match_device(lm3646_trigger_dt_match, &pdev->dev);
	if (!match)
	{
		pr_err("msm_flash_lm3646_platform_probe fail, rc %d", -EFAULT);
		return -EFAULT;
	}

#ifdef CREATE_VIRTUAL_FILE
	create_class_led_file();	// create virtual file for FTM test.
#endif
	return msm_flash_probe(pdev, match->data);
}

static struct platform_driver lm3646_platform_driver = {
	.probe = msm_flash_lm3646_platform_probe,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3646_trigger_dt_match,
	},
};

static int __init msm_flash_lm3646_init_module(void)
{
	int32_t rc = 0;

	pr_err("msm_flash_lm3646_init_module ++");

	rc = platform_driver_register(&lm3646_platform_driver);
	if (fctrl.pdev != NULL && rc == 0) {
		pr_err("lm3646 platform_driver_register success");
		return rc;
	} else if (rc != 0) {
		pr_err("lm3646 platform_driver_register failed");
		return rc;
	} else {
		rc = i2c_add_driver(&lm3646_i2c_driver);
		if (!rc)
			pr_err("lm3646 i2c_add_driver success");
	}
	return rc;
}

static void __exit msm_flash_lm3646_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&lm3646_platform_driver);
	else
		i2c_del_driver(&lm3646_i2c_driver);
}

static struct msm_camera_i2c_client lm3646_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3646_init_setting = {
	.reg_setting = lm3646_init_array,
	.size = ARRAY_SIZE(lm3646_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3646_off_setting = {
	.reg_setting = lm3646_off_array,
	.size = ARRAY_SIZE(lm3646_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3646_release_setting = {
	.reg_setting = lm3646_release_array,
	.size = ARRAY_SIZE(lm3646_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3646_low_setting = {
	.reg_setting = lm3646_low_array,
	.size = ARRAY_SIZE(lm3646_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3646_high_setting = {
	.reg_setting = lm3646_high_array,
	.size = ARRAY_SIZE(lm3646_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm3646_regs = {
	.init_setting = &lm3646_init_setting,
	.off_setting = &lm3646_off_setting,
	.low_setting = &lm3646_low_setting,
	.high_setting = &lm3646_high_setting,
	.release_setting = &lm3646_release_setting,
};

static struct msm_flash_fn_t lm3646_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = lm3646_flash_led_low,
	.flash_led_high = lm3646_flash_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3646_i2c_client,
	.reg_setting = &lm3646_regs,
	.func_tbl = &lm3646_func_tbl,
	/* Flash */
	.flash_trigger_name = {
		[LED_AMBER] = "flash amber LED",
		[LED_COOL] = "flash cool LED",
	},
	.flash_num_sources =  2,
	.flash_op_current = {
		[LED_AMBER] = 0,
		[LED_COOL] = 0,
	},
	/* Torch */
	.torch_trigger_name = {
		[LED_AMBER] = "torch amber LED",
		[LED_COOL] = "torch cool LED",
	},
	.torch_num_sources = 2,
	.torch_op_current = {
		[LED_AMBER] = 0,
		[LED_COOL] = 0,
	},

	.flash_device_type = MSM_CAMERA_I2C_DEVICE,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_lm3646_init_module);
module_exit(msm_flash_lm3646_exit_module);
MODULE_DESCRIPTION("lm3646 FLASH");
MODULE_LICENSE("GPL v2");
