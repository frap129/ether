/* * 
 * * Copyright (C) 2015 TI Inc.
 * */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/export.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>

#include <asm-generic/bitops/non-atomic.h>
#include <linux/spinlock.h>
#include "hd3ss3220_platform_i2c.h"
#include <linux/regulator/consumer.h>


#define HD3SS3220_DRIVER_NAME "hd3ss3220"
#define COMPATIBLE_NAME "ti,hd3ss3220"

#define CSR_REG_08 	0x08
#define CSR_REG_09 	0x09
#define CSR_REG_0A 	0x0A
#define CSR_REG_45 	0x45

#define GPIO_USBPD_INT 		 9


enum {
	INT_INDEX = 0,
};
enum {
	MODE_SELECT_DEFAULT =0,
	MODE_SELECT_UFP,
	MODE_SELECT_DFP,
	MODE_SELECT_DRP,
};

struct gpio hd3ss3220_gpio[]={
	{GPIO_USBPD_INT, GPIOF_IN, "USBPD_intr"},
};

/*  NBQ - AlbertWu - [NBQ-1056] - [USB] Judge HW ID in HD3SS3220 USB type C driver. */
#define	HD3SS3220_DEVICE_I2C_ADDR_LOW	0x60
#define HD3SS3220_DEVICE_I2C_ADDR_HIGH	0x61
/*  NBQ - AlbertWu - [NBQ-1608] - [USB]  Configure New HD3SS3220 Chip */
#define	NEW_HD3SS3220_DEVICE_I2C_ADDR_LOW	0x47
#define NEW_HD3SS3220_DEVICE_I2C_ADDR_HIGH	0x67
/* end  NBQ - AlbertWu - [NBQ-1608] */
uint8_t HD3SS3220_device_ID;
/* end  NBQ - AlbertWu - [NBQ-1056] */

/*  NBQ - AlbertWu - [NBQ-1487] - [USB] Enable regulator L31. */
int nbq_hw_id;
/*  NBQ - AlbertWu - [NBQ-1608] - [USB]  Configure New HD3SS3220 Chip */
/*  NBQ - AlbertWu - [NBQ-1724] - [USB]  Get new HW ID to configure */
int Prj_info;
/* end  NBQ - AlbertWu - [NBQ-1724] */


static int  set_regulator_ldo31_configured(struct regulator *regulator)
{
	int ret;
	
	ret=regulator_disable(regulator);	
	if (ret < 0)
		return ret;
	ret = regulator_enable(regulator);
	return ret;
}

/* end  NBQ - AlbertWu - [NBQ-1608] */
/* end  NBQ - AlbertWu - [NBQ-1487] */



uint8_t check_USB_typeC_mode(void)
{
	uint8_t  temp_reg,mode;

	temp_reg = hd3ss3220_platform_rd_reg8(CSR_REG_0A);
	temp_reg &= 0x30;
	mode = ( temp_reg >> 4 );
	pr_info("\n mode=%d",mode);
	return mode;
}

void select_USB_typeC_mode(uint8_t mode)
{
	uint8_t temp_reg;

	temp_reg = hd3ss3220_platform_rd_reg8(CSR_REG_0A);
	temp_reg &=~0x30;
	switch(mode){
		case MODE_SELECT_DEFAULT :
			temp_reg |= 0x00;
			hd3ss3220_platform_wr_reg8(CSR_REG_0A, temp_reg);
			pr_info("Select DEFAULT mode \n");
			break;		
		case MODE_SELECT_UFP :
			temp_reg |= 0x10;
			hd3ss3220_platform_wr_reg8(CSR_REG_0A, temp_reg);
			pr_info("Select UFP mode \n");
			break;
		case MODE_SELECT_DFP: 
			temp_reg |= 0x20;
			hd3ss3220_platform_wr_reg8(CSR_REG_0A, temp_reg);
			pr_info("Select DFP mode \n");
			break;
		case MODE_SELECT_DRP:
			temp_reg |= 0x30;
			hd3ss3220_platform_wr_reg8(CSR_REG_0A, temp_reg);
			pr_info("Select DRP mode \n");
			break;
		default:
			printk("Invalid mode\n");
			break;
	}
	pr_info("\n temp_reg=0x%x",temp_reg);
}


static irqreturn_t usbpd_irq_handler(int irq, void *data)
{
	uint8_t USB_typeC_mode;
/*  NBQ - AlbertWu - [NBQM-880] - [USB] Add HD3ss3220 DVT2 device tree to NBQM project. */
	//uint8_t p_data[12]={0x00},i;
	//pr_info("[%s]",__FUNCTION__);
/* end  NBQ - AlbertWu - [NBQM-880] */


	select_USB_typeC_mode(MODE_SELECT_UFP);
	
	USB_typeC_mode=check_USB_typeC_mode();
	pr_info("\n USB_typeC_mode=%x",USB_typeC_mode);
/*  NBQ - AlbertWu - [NBQM-880] - [USB] Add HD3ss3220 DVT2 device tree to NBQM project. */
/*
	msleep(300);	
	hd3ss3220_platform_block_read8(0x00, p_data, 12);
	
	for(i=0x00;i<=0x0A;i++)
	{
		pr_info("\n p_data[%d]=0x%x \n",i,p_data[i]);
	}
*/
/* end  NBQ - AlbertWu - [NBQM-880] */
	
	return  IRQ_HANDLED;
}


static int hd3ss3220_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int value;

	value = of_get_named_gpio_flags(np, "ti-hd3ss3220,irq-gpio", 0, NULL);
	if (value >= 0){
		hd3ss3220_gpio[INT_INDEX].gpio = value;
		pr_info("##hd3ss3220 int ping :%d",hd3ss3220_gpio[INT_INDEX].gpio);
	}
	else
		return -ENODEV;

	return 0;
}


static int hd3ss3220_i2c_remove(struct i2c_client *client)
{

	gpio_free_array(hd3ss3220_gpio,
			ARRAY_SIZE(hd3ss3220_gpio));

	pr_info("driver unloaded.\n");

	return 0;
}

static int hd3ss3220_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	int ret=0, rc =0 ;

	struct regulator *regulator_vdd;
	int irq_qpio;
	dev_dbg(&client->dev, "Enter\n");

	pr_info("\n=======================================\n");
	pr_info("\n --- hd3ss3220 Driver v1.0 --- \n");	
	pr_info("\n=======================================\n");

	if (client->dev.of_node) {
		ret = hd3ss3220_parse_dt(&client->dev);
		if (ret) {
			return -ENODEV;
		}
	}
	else
		return -ENODEV;

	ret = gpio_request_array(hd3ss3220_gpio, ARRAY_SIZE(hd3ss3220_gpio));

	if(ret < 0){
		dev_err(&client->dev, "gpio_request_array failed");
		return -EINVAL;
	}
	
	irq_qpio= gpio_to_irq(hd3ss3220_gpio[INT_INDEX].gpio);
	usbpd_pf_i2c_init(client->adapter);
	
	regulator_vdd = regulator_get(&client->dev,"hd3ss3220vdd");
	if (IS_ERR(regulator_vdd)) {
	    pr_err("%s: Failed to get hd3ss3220vdd\n", __func__);
	    ret = PTR_ERR(regulator_vdd);
	    regulator_put(regulator_vdd);
	    return ret;
	}
	ret = regulator_enable(regulator_vdd);
	if (ret) {	
	      pr_err("%s: Failed to enable vdd-supply\n",__func__);
	      regulator_disable(regulator_vdd);
	      regulator_put(regulator_vdd);
	      return ret;
 	}
	pr_info("hd3ss3220 power on .....");
/*  NBQ - AlbertWu - [NBQ-1724] - [USB]  Get new HW ID to configure */
/*  NBQ - AlbertWu - [NBQ-1608] - [USB] . Configure New HD3SS3220 Chip */
	//if(Prj_info == FIH_PRJ_NBQ && nbq_hw_id == FIH_REV_EVT_PRE1_5){
		pr_info("\n [Enable regulator]");
		regulator_vdd = regulator_get(&client->dev,"usb_redriver"); // Need to check L31 on EVT1C
		if (IS_ERR(regulator_vdd)) {
		    pr_err("%s: Failed to get USB 3.0 re-driver \n", __func__);
		    ret = PTR_ERR(regulator_vdd);
		    regulator_put(regulator_vdd);
		    return ret;
		}
	
		ret = regulator_enable(regulator_vdd);
		if (ret) {	
		      pr_err("%s: Failed to enable USB 3.0 re-driver\n",__func__);
		      regulator_disable(regulator_vdd);
		      regulator_put(regulator_vdd);
		      return ret;
	 	}
	//}
/* end  NBQ - AlbertWu - [NBQ-1608] */
/* end  NBQ - AlbertWu - [NBQ-1724] */

/*  NBQ - AlbertWu - [NBQ-1487] - [USB] Enable regulator L31. */
	pr_info("NBQ_HW_ID : %x",nbq_hw_id);
/*  NBQ - AlbertWu - [NBQ-1608] - [USB] . Configure New HD3SS3220 Chip */
	if( (Prj_info == FIH_PRJ_VZW) || (Prj_info == FIH_PRJ_NBQ && nbq_hw_id >= FIH_REV_EVT1C) )
		set_regulator_ldo31_configured(regulator_vdd);	
/* end  NBQ - AlbertWu - [NBQ-1608] */

/* end  NBQ - AlbertWu - [NBQ-1487] */
	pr_info("USB 3.0 re-driver power on .....");

	rc = request_threaded_irq(irq_qpio, NULL, usbpd_irq_handler, 
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, HD3SS3220_DRIVER_NAME, 
			&client->dev);			

	if (rc < 0) {
		dev_err(&client->dev, "request_threaded_irq failed, status\n");
		return -1;
	}
	


	pr_info("Driver is intialised properly \n");
	return 0;
}
/*  NBQ - AlbertWu - [NBQ-1056] - [USB] Judge HW ID in HD3SS3220 USB type C driver. */
/* static struct i2c_device_id hd3ss3220_i2c_id[] = {
	{ HD3SS3220_DRIVER_NAME, 0x60},
	{}
}; */

static struct i2c_device_id hd3ss3220_i2c_id1[] = {
	{ HD3SS3220_DRIVER_NAME, HD3SS3220_DEVICE_I2C_ADDR_LOW},
	{}
};
static struct i2c_device_id hd3ss3220_i2c_id2[] = {
	{ HD3SS3220_DRIVER_NAME, HD3SS3220_DEVICE_I2C_ADDR_HIGH},
	{}
};	

/* end  NBQ - AlbertWu - [NBQ-1056] */
/*  NBQ - AlbertWu - [NBQ-1608] - [USB]  Configure New HD3SS3220 Chip */
static struct i2c_device_id hd3ss3220_i2c_id3[] = {
	{ HD3SS3220_DRIVER_NAME, NEW_HD3SS3220_DEVICE_I2C_ADDR_HIGH},
	{}
};	
/* end  NBQ - AlbertWu - [NBQ-1608] */
static const struct of_device_id hd3ss3220_match_table[] = {
	{.compatible = COMPATIBLE_NAME},
	{}
};

static struct i2c_driver hd3ss3220_i2c_driver = {
	.driver = {
		.name = HD3SS3220_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = hd3ss3220_match_table,
	},
	.probe = hd3ss3220_i2c_probe,
	.remove =  hd3ss3220_i2c_remove,
/*  NBQ - AlbertWu - [NBQ-1056] - [USB] Judge HW ID in HD3SS3220 USB type C driver. */	
	.id_table = hd3ss3220_i2c_id1, //hd3ss3220_i2c_id
/* end  NBQ - AlbertWu - [NBQ-1056] */
};


/*  NBQ - AlbertWu - [NBQ-1724] - [USB]  Get new HW ID to configure */
void check_HW_ID_to_configure(void)
{
	Prj_info = fih_hwid_fetch(FIH_HWID_PRJ);
	pr_info("Prj info :%d \n",Prj_info);
	nbq_hw_id = fih_hwid_fetch(FIH_HWID_REV);
	if(Prj_info == FIH_PRJ_NBQ)
	{
		if(nbq_hw_id <= FIH_REV_EVT_PRE1_5) {
			pr_info("FIH_REV_EVT_PRE1_5 \n");
			HD3SS3220_device_ID = HD3SS3220_DEVICE_I2C_ADDR_LOW;
			hd3ss3220_i2c_driver.id_table=hd3ss3220_i2c_id1;
		} else if(nbq_hw_id == FIH_REV_EVT1C) {
			pr_info("FIH_REV_EVT1C \n");
			HD3SS3220_device_ID = HD3SS3220_DEVICE_I2C_ADDR_HIGH;
			hd3ss3220_i2c_driver.id_table=hd3ss3220_i2c_id2;
		} else if(nbq_hw_id >= FIH_REV_DVT) {
			pr_info("FIH_REV_DVT \n");
			HD3SS3220_device_ID = NEW_HD3SS3220_DEVICE_I2C_ADDR_HIGH;
			hd3ss3220_i2c_driver.id_table=hd3ss3220_i2c_id3;
		} else {
			pr_info("HW ID:%d \n",nbq_hw_id);
			HD3SS3220_device_ID = NEW_HD3SS3220_DEVICE_I2C_ADDR_HIGH;
			hd3ss3220_i2c_driver.id_table=hd3ss3220_i2c_id3;
		}
	}
	else if(Prj_info == FIH_PRJ_VZW )
	{
		if(nbq_hw_id <= FIH_REV_EVT) {
			pr_info("FIH_REV_EVT \n");
			HD3SS3220_device_ID = NEW_HD3SS3220_DEVICE_I2C_ADDR_HIGH;
			hd3ss3220_i2c_driver.id_table=hd3ss3220_i2c_id3;
		} else {
			pr_info("HW ID:%d \n",nbq_hw_id);
			HD3SS3220_device_ID = NEW_HD3SS3220_DEVICE_I2C_ADDR_HIGH;
			hd3ss3220_i2c_driver.id_table=hd3ss3220_i2c_id3;
		}
	}
	else {
		pr_info("Unknow Prj info... \n");
	}
}

static int __init hd3ss3220_init(void)
{
	int ret = 0;

	printk("[FTS]: %s\n", __func__);
/*	NBQ - AlbertWu - [NBQ-1724] - [USB]	Get new HW ID to configure */
	check_HW_ID_to_configure();
/* end	NBQ - AlbertWu - [NBQ-1724] */

	ret = i2c_add_driver(&hd3ss3220_i2c_driver);
	if (ret < 0) {
		pr_err("[FTS]: %s:failed to add driver, error %d\n",
			__func__, ret);
		return ret;
	}
	return ret;
}

static void __exit hd3ss3220_exit(void)
{
	printk("[FTS]: %s\n", __func__);
	i2c_del_driver(&hd3ss3220_i2c_driver);
}

module_init(hd3ss3220_init);
module_exit(hd3ss3220_exit);


MODULE_DESCRIPTION("TI HD3SS3220  switch driver");
MODULE_AUTHOR("TI HD3SS3220 <http://www.ti.com/>");
MODULE_LICENSE("GPL");
