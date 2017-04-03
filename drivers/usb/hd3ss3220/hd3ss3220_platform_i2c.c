/* * 
 * * Copyright (C) 2014 Silicon Image Inc.
 * *
 * * Author: [Praveen Vuppala <praveen.vuppala@siliconimage.com>]
 * *
 * * No part of this work may be reproduced, modified, distributed, transmitted,
 * * transcribed, or translated into any language or computer format, in any form
 * * or by any means without written permission of
 * * Silicon Image, Inc., 1140 East Arques Avenue, Sunnyvale, California 94085
 * */


#include <linux/list.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include "hd3ss3220_platform_i2c.h"

/*  NBQ - AlbertWu - [NBQ-1056] - [USB] Judge HW ID in HD3SS3220 USB type C driver. */
//#define HD3SS3220_DRV_DEVICE_I2C_ADDR 0x60
extern uint8_t HD3SS3220_device_ID;
/* end  NBQ - AlbertWu - [NBQ-1056] */
#define I2C_RETRY_MAX 10
static struct i2c_adapter *i2c_bus;

/*This should be in the i2c layer*/
void usbpd_pf_i2c_init(struct i2c_adapter *adapter)
{
	i2c_bus = adapter;
}
//EXPORT_SYMBOL(usbpd_pf_i2c_init);

static inline int hd3ss3220_platform_read_i2c_block(struct i2c_adapter *i2c_bus, uint8_t
		device_id, uint8_t offset, uint16_t count, uint8_t *values)
{
	struct i2c_msg msg[2];
	int ret;
	int i;

	msg[0].flags = 0;
	msg[0].addr = device_id;
	msg[0].buf = &offset;
	msg[0].len = 1;

	msg[1].flags = I2C_M_RD;
	msg[1].addr = device_id;
	msg[1].buf = values;
	msg[1].len = count;

	for (i = 0; i < I2C_RETRY_MAX; i++) {
		ret = i2c_transfer(i2c_bus, msg, 2);
		if (ret != 2) {
			pr_err("I2c(R) retry 0x%02x:0x%02x\n" ,device_id, offset);
			ret = -EIO;
			msleep(20);
		} else {
			break;
		}
	}

	return ret;
}

static inline int hd3ss3220_platform_write_i2c_block(struct i2c_adapter *i2c_bus, 
		uint8_t device_id, uint8_t offset, uint16_t count, const uint8_t *values) 
{
	struct i2c_msg msg;
	u8 *buffer;
	int ret; 
	int i;

	buffer = kmalloc(count + 1, GFP_KERNEL);
	if (!buffer) {
		printk(KERN_ERR "%s:%d buffer allocation failed\n",
				__func__, __LINE__);
		return -ENOMEM;
	}
	buffer[0] = offset;
	memmove(&buffer[1], values, count); 

	msg.flags = 0;
	msg.addr = device_id;
	msg.buf = buffer;
	msg.len = count + 1;

	for (i = 0; i < I2C_RETRY_MAX; i++) {
		ret = i2c_transfer(i2c_bus, &msg, 1);
		if(ret != 1) {
			pr_err("I2ci(W) retry 0x%02x:0x%02x: 0x%02x \n",device_id, offset,*values);
			ret = -EIO;
			msleep(20);
		}
		else {
			ret = 0;
			break;
		}
	}

	kfree(buffer);

	return ret;
}

void hd3ss3220_platform_wr_reg8(const uint8_t addr,
		const uint8_t val)
{
	hd3ss3220_platform_block_write8(addr, &val, 1);
}
EXPORT_SYMBOL(hd3ss3220_platform_wr_reg8);
uint8_t hd3ss3220_platform_rd_reg8(const uint8_t addr)
{
	uint8_t val = 0;
	hd3ss3220_platform_block_read8(addr, &val, 1);
	return val;
}
EXPORT_SYMBOL(hd3ss3220_platform_rd_reg8);

void hd3ss3220_platform_set_bit8(const uint8_t addr,
		const uint8_t   mask)
{
	uint8_t val;
	val = hd3ss3220_platform_rd_reg8(addr) ;
	//printk("reading:  %X writing: ",val);
	val = (val & ((uint8_t)~mask)) | mask;
	//printk("%X\n",val);
	hd3ss3220_platform_wr_reg8(addr, val);
}
EXPORT_SYMBOL(hd3ss3220_platform_set_bit8);

void hd3ss3220_platform_clr_bit8(const uint8_t addr,
		const uint8_t mask)
{
	uint8_t val;
	val = hd3ss3220_platform_rd_reg8(addr) ;
	val = (val & ((uint8_t)~mask));
	hd3ss3220_platform_wr_reg8(addr, val);
}
EXPORT_SYMBOL(hd3ss3220_platform_clr_bit8);

void hd3ss3220_platform_put_bit8(const uint8_t addr,
		const uint8_t mask, const uint8_t val)
{
	uint8_t temp;
	uint8_t write_val;
	temp = hd3ss3220_platform_rd_reg8(addr);
	temp = (temp & ((uint8_t)~mask));
	write_val = temp | (mask & val);
	hd3ss3220_platform_wr_reg8(addr, write_val);
}
EXPORT_SYMBOL(hd3ss3220_platform_put_bit8);

void hd3ss3220_platform_fifo_read8(const uint8_t addr,
		uint8_t *p_data, uint16_t size)
{
	uint16_t i;
	for (i = 0; i < size; i++) {
		p_data[i] = hd3ss3220_platform_rd_reg8(addr);
		;
	}
}
EXPORT_SYMBOL(hd3ss3220_platform_fifo_read8);

void hd3ss3220_platform_fifo_write8(const uint8_t addr,
		const uint8_t *p_data, uint16_t size)
{
	uint16_t i;
	for (i = 0; i < size; i++) {
		hd3ss3220_platform_wr_reg8(addr, *(p_data+i));
		;
	}
}
EXPORT_SYMBOL(hd3ss3220_platform_fifo_write8);

void hd3ss3220_platform_block_write8(const uint8_t addr,
		const uint8_t *p_data, uint16_t size)

{
	int ret = 0;
/*  NBQ - AlbertWu - [NBQ-1056] - [USB] Judge HW ID in HD3SS3220 USB type C driver. */
//ret = hd3ss3220_platform_write_i2c_block(i2c_bus,HD3SS3220_DRV_DEVICE_I2C_ADDR, addr, size, p_data);
ret = hd3ss3220_platform_write_i2c_block(i2c_bus,HD3SS3220_device_ID, addr, size, p_data);
/* end  NBQ - AlbertWu - [NBQ-1056] */
	/*Note: Customer can implement their error handling. Below is an example*/
}
EXPORT_SYMBOL(hd3ss3220_platform_block_write8);

void hd3ss3220_platform_block_read8(const uint8_t addr, 
		uint8_t *p_data, uint16_t size)
{
	int ret = 0;
/*  NBQ - AlbertWu - [NBQ-1056] - [USB] Judge HW ID in HD3SS3220 USB type C driver. */	
	//ret  = hd3ss3220_platform_read_i2c_block(i2c_bus, HD3SS3220_DRV_DEVICE_I2C_ADDR,addr, size, p_data);
	ret  = hd3ss3220_platform_read_i2c_block(i2c_bus, HD3SS3220_device_ID,addr, size, p_data);	
/* end  NBQ - AlbertWu - [NBQ-1056] */
}
EXPORT_SYMBOL(hd3ss3220_platform_block_read8);

