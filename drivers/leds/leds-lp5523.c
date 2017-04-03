/*
 * lp5523.c - LP5523 LED Driver
 *
 * Copyright (C) 2010 Nokia Corporation
 * Copyright (C) 2012 Texas Instruments
 *
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *          Milo(Woogyom) Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h> //NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver.
#include <linux/platform_data/leds-lp55xx.h>
#include <linux/slab.h>

#include "leds-lp55xx-common.h"
#include <fih/hwid.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>

#define LP5523_PROGRAM_LENGTH		32
#define LP5523_MAX_LEDS			9
#define LP5523_CUSTOMER_LEDS			4 //  NBQ - MaoyiChou - [NBQ-798] - [Cloud LED] Implement leds on/off for lp5523.

/* Registers */
#define LP5523_REG_ENABLE		0x00
#define LP5523_REG_OP_MODE		0x01
#define LP5523_REG_ENABLE_LEDS_MSB	0x04
#define LP5523_REG_ENABLE_LEDS_LSB	0x05
#define LP5523_REG_LED_PWM_BASE		0x16
#define LP5523_REG_LED_CURRENT_BASE	0x26
#define LP5523_REG_CONFIG		0x36
#define LP5523_REG_STATUS		0x3A
#define LP5523_REG_RESET		0x3D
#define LP5523_REG_LED_TEST_CTRL	0x41
#define LP5523_REG_LED_TEST_ADC		0x42
/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
#define LP5523_REG_CH1_PROG_START	0x4C
#define LP5523_REG_CH2_PROG_START	0x4D
#define LP5523_REG_CH3_PROG_START	0x4E
/* end  NBQ - MaoyiChou - [NBQ-775] */
#define LP5523_REG_PROG_PAGE_SEL	0x4F
#define LP5523_REG_PROG_MEM		0x50

/* Bit description in registers */
#define LP5523_ENABLE			0x40
#define LP5523_AUTO_INC			0x40
#define LP5523_PWR_SAVE			0x20
#define LP5523_PWM_PWR_SAVE		0x04
#define LP5523_CP_AUTO			0x18
#define LP5523_AUTO_CLK			0x02

#define LP5523_EN_LEDTEST		0x80
#define LP5523_LEDTEST_DONE		0x80
#define LP5523_RESET			0xFF
#define LP5523_ADC_SHORTCIRC_LIM	80
#define LP5523_EXT_CLK_USED		0x08
#define LP5523_ENG_STATUS_MASK		0x07 //NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver.

/* Memory Page Selection */
#define LP5523_PAGE_ENG1		0
#define LP5523_PAGE_ENG2		1
#define LP5523_PAGE_ENG3		2
/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
#define LP5523_PAGE_MUX1		3
#define LP5523_PAGE_MUX2		4
#define LP5523_PAGE_MUX3		5
/* end  NBQ - MaoyiChou - [NBQ-775] */

/* Program Memory Operations */
#define LP5523_MODE_ENG1_M		0x30	/* Operation Mode Register */
#define LP5523_MODE_ENG2_M		0x0C
#define LP5523_MODE_ENG3_M		0x03
#define LP5523_LOAD_ENG1		0x10
#define LP5523_LOAD_ENG2		0x04
#define LP5523_LOAD_ENG3		0x01

#define LP5523_ENG1_IS_LOADING(mode)	\
	((mode & LP5523_MODE_ENG1_M) == LP5523_LOAD_ENG1)
#define LP5523_ENG2_IS_LOADING(mode)	\
	((mode & LP5523_MODE_ENG2_M) == LP5523_LOAD_ENG2)
#define LP5523_ENG3_IS_LOADING(mode)	\
	((mode & LP5523_MODE_ENG3_M) == LP5523_LOAD_ENG3)

#define LP5523_EXEC_ENG1_M		0x30	/* Enable Register */
#define LP5523_EXEC_ENG2_M		0x0C
#define LP5523_EXEC_ENG3_M		0x03
#define LP5523_EXEC_M			0x3F
#define LP5523_RUN_ENG1			0x20
#define LP5523_RUN_ENG2			0x08
#define LP5523_RUN_ENG3			0x02

#define LED_ACTIVE(mux, led)		(!!(mux & (0x0001 << led)))  //NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver.

enum lp5523_chip_id {
	LP5523,
	LP55231,
};

/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
static bool allowAccess = true;

static int lp5523_init_program_engine(struct lp55xx_chip *chip);
/* end  NBQ - MaoyiChou - [NBQ-775] */

static inline void lp5523_wait_opmode_done(void)
{
	usleep_range(1000, 2000);
}

static void lp5523_set_led_current(struct lp55xx_led *led, u8 led_current)
{
	led->led_current = led_current;
	lp55xx_write(led->chip, LP5523_REG_LED_CURRENT_BASE + led->chan_nr,
		led_current);
}

static int lp5523_post_init_device(struct lp55xx_chip *chip)
{
	int ret;

	ret = lp55xx_write(chip, LP5523_REG_ENABLE, LP5523_ENABLE);
	if (ret)
		return ret;

	/* Chip startup time is 500 us, 1 - 2 ms gives some margin */
	usleep_range(1000, 2000);

	ret = lp55xx_write(chip, LP5523_REG_CONFIG,
			    LP5523_AUTO_INC | LP5523_PWR_SAVE |
			    LP5523_CP_AUTO | LP5523_AUTO_CLK |
			    LP5523_PWM_PWR_SAVE);
	if (ret)
		return ret;

	/* turn on all leds */
	ret = lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_MSB, 0x01);
	if (ret)
		return ret;

/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
	ret = lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_LSB, 0xff);
	if (ret)
		return ret;

	return lp5523_init_program_engine(chip);
/* end  NBQ - MaoyiChou - [NBQ-775] */
}

static void lp5523_load_engine(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5523_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5523_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5523_MODE_ENG3_M,
	};

	u8 val[] = {
		[LP55XX_ENGINE_1] = LP5523_LOAD_ENG1,
		[LP55XX_ENGINE_2] = LP5523_LOAD_ENG2,
		[LP55XX_ENGINE_3] = LP5523_LOAD_ENG3,
	};

/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
	lp55xx_update_bits(chip, LP5523_REG_OP_MODE, mask[idx], val[idx]);

	lp5523_wait_opmode_done();
}

static void lp5523_load_engine_and_select_page(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 page_sel[] = {
		[LP55XX_ENGINE_1] = LP5523_PAGE_ENG1,
		[LP55XX_ENGINE_2] = LP5523_PAGE_ENG2,
		[LP55XX_ENGINE_3] = LP5523_PAGE_ENG3,
	};

	lp5523_load_engine(chip);
/* end  NBQ - MaoyiChou - [NBQ-775] */

	lp55xx_write(chip, LP5523_REG_PROG_PAGE_SEL, page_sel[idx]);
}

static void lp5523_stop_engine(struct lp55xx_chip *chip)
{
	lp55xx_write(chip, LP5523_REG_OP_MODE, 0);
	lp5523_wait_opmode_done();
}

static void lp5523_turn_off_channels(struct lp55xx_chip *chip)
{
	int i;

	for (i = 0; i < LP5523_MAX_LEDS; i++)
		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, 0);
}

static void lp5523_run_engine(struct lp55xx_chip *chip, bool start)
{
	int ret;
	u8 mode;
	u8 exec;

	/* stop engine */
	if (!start) {
		lp5523_stop_engine(chip);
		lp5523_turn_off_channels(chip);
		return;
	}

	/*
	 * To run the engine,
	 * operation mode and enable register should updated at the same time
	 */

	ret = lp55xx_read(chip, LP5523_REG_OP_MODE, &mode);
	if (ret)
		return;

	ret = lp55xx_read(chip, LP5523_REG_ENABLE, &exec);
	if (ret)
		return;

	/* change operation mode to RUN only when each engine is loading */
	if (LP5523_ENG1_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG1_M) | LP5523_RUN_ENG1;
		exec = (exec & ~LP5523_EXEC_ENG1_M) | LP5523_RUN_ENG1;
	}

	if (LP5523_ENG2_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG2_M) | LP5523_RUN_ENG2;
		exec = (exec & ~LP5523_EXEC_ENG2_M) | LP5523_RUN_ENG2;
	}

	if (LP5523_ENG3_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG3_M) | LP5523_RUN_ENG3;
		exec = (exec & ~LP5523_EXEC_ENG3_M) | LP5523_RUN_ENG3;
	}

	lp55xx_write(chip, LP5523_REG_OP_MODE, mode);
	lp5523_wait_opmode_done();

	lp55xx_update_bits(chip, LP5523_REG_ENABLE, LP5523_EXEC_M, exec);
}

/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
static int lp5523_init_program_engine(struct lp55xx_chip *chip)
{
	int i;
	int j;
	int ret;
	u8 status;
	/* one pattern per engine setting LED MUX start and stop addresses */
	static const u8 pattern[][LP5523_PROGRAM_LENGTH] =  {
		{ 0x9c, 0x30, 0x9c, 0xb0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x40, 0x9c, 0xc0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x50, 0x9c, 0xd0, 0x9d, 0x80, 0xd8, 0x00, 0},
	};

	/* hardcode 32 bytes of memory for each engine from program memory */
	ret = lp55xx_write(chip, LP5523_REG_CH1_PROG_START, 0x01);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_CH2_PROG_START, 0x10);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_CH3_PROG_START, 0x20);
	if (ret)
		return ret;

	/* write LED MUX address space for each engine */
	for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++) {
		chip->engine_idx = i;
		lp5523_load_engine_and_select_page(chip);

		for (j = 0; j < LP5523_PROGRAM_LENGTH; j++) {
			ret = lp55xx_write(chip, LP5523_REG_PROG_MEM + j,
					pattern[i - 1][j]);
			if (ret)
				goto out;
		}
	}

	lp5523_run_engine(chip, true);

	/* Let the programs run for couple of ms and check the engine status */
	usleep_range(3000, 6000);
	lp55xx_read(chip, LP5523_REG_STATUS, &status);
	status &= LP5523_ENG_STATUS_MASK;

	if (status != LP5523_ENG_STATUS_MASK) {
		dev_err(&chip->cl->dev,
			"cound not configure LED engine, status = 0x%.2x\n",
			status);
		ret = -1;
	}

out:
	lp5523_stop_engine(chip);
	return ret;
}
/* end  NBQ - MaoyiChou - [NBQ-775] */

static int lp5523_update_program_memory(struct lp55xx_chip *chip,
					const u8 *data, size_t size)
{
	u8 pattern[LP5523_PROGRAM_LENGTH] = {0};
	unsigned cmd;
	char c[3];
	int nrchars;
	int ret;
/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
	int offset = 0;
	int i = 0;
/* end  NBQ - MaoyiChou - [NBQ-775] */

	while ((offset < size - 1) && (i < LP5523_PROGRAM_LENGTH)) {
		/* separate sscanfs because length is working only for %s */
		ret = sscanf(data + offset, "%2s%n ", c, &nrchars);
		if (ret != 1)
			goto err;

		ret = sscanf(c, "%2x", &cmd);
		if (ret != 1)
			goto err;

		pattern[i] = (u8)cmd;
		offset += nrchars;
		i++;
	}

	/* Each instruction is 16bit long. Check that length is even */
	if (i % 2)
		goto err;

/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
	mutex_lock(&chip->lock);

	for (i = 0; i < LP5523_PROGRAM_LENGTH; i++) {
		ret = lp55xx_write(chip, LP5523_REG_PROG_MEM + i, pattern[i]);
		if (ret) {
			mutex_unlock(&chip->lock);
			return -EINVAL;
		}
	}

	mutex_unlock(&chip->lock);

	return size;
/* end  NBQ - MaoyiChou - [NBQ-775] */

err:
	dev_err(&chip->cl->dev, "wrong pattern format\n");
	return -EINVAL;
}

static void lp5523_firmware_loaded(struct lp55xx_chip *chip)
{
	const struct firmware *fw = chip->fw;

	if (fw->size > LP5523_PROGRAM_LENGTH) {
		dev_err(&chip->cl->dev, "firmware data size overflow: %zu\n",
			fw->size);
		return;
	}

	/*
	 * Program momery sequence
	 *  1) set engine mode to "LOAD"
	 *  2) write firmware data into program memory
	 */

	lp5523_load_engine_and_select_page(chip); //NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver.
	lp5523_update_program_memory(chip, fw->data, fw->size);
}

/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
static ssize_t show_engine_mode(struct device *dev,
				struct device_attribute *attr,
				char *buf, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	enum lp55xx_engine_mode mode = chip->engines[nr - 1].mode;

	switch (mode) {
	case LP55XX_ENGINE_RUN:
		return sprintf(buf, "run\n");
	case LP55XX_ENGINE_LOAD:
		return sprintf(buf, "load\n");
	case LP55XX_ENGINE_DISABLED:
	default:
		return sprintf(buf, "disabled\n");
	}
}
show_mode(1)
show_mode(2)
show_mode(3)

static ssize_t store_engine_mode(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_engine *engine = &chip->engines[nr - 1];

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;

	if (!strncmp(buf, "run", 3)) {
		lp5523_run_engine(chip, true);
		engine->mode = LP55XX_ENGINE_RUN;
	} else if (!strncmp(buf, "load", 4)) {
		lp5523_stop_engine(chip);
		lp5523_load_engine(chip);
		engine->mode = LP55XX_ENGINE_LOAD;
	} else if (!strncmp(buf, "disabled", 8)) {
		lp5523_stop_engine(chip);
		engine->mode = LP55XX_ENGINE_DISABLED;
	}

	mutex_unlock(&chip->lock);

	return len;
}
store_mode(1)
store_mode(2)
store_mode(3)

static int lp5523_mux_parse(const char *buf, u16 *mux, size_t len)
{
	u16 tmp_mux = 0;
	int i;

	len = min_t(int, len, LP5523_MAX_LEDS);

	for (i = 0; i < len; i++) {
		switch (buf[i]) {
		case '1':
			tmp_mux |= (1 << i);
			break;
		case '0':
			break;
		case '\n':
			i = len;
			break;
		default:
			return -1;
		}
	}
	*mux = tmp_mux;

	return 0;
}

static void lp5523_mux_to_array(u16 led_mux, char *array)
{
	int i, pos = 0;
	for (i = 0; i < LP5523_MAX_LEDS; i++)
		pos += sprintf(array + pos, "%x", LED_ACTIVE(led_mux, i));

	array[pos] = '\0';
}

static ssize_t show_engine_leds(struct device *dev,
			    struct device_attribute *attr,
			    char *buf, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	char mux[LP5523_MAX_LEDS + 1];

	lp5523_mux_to_array(chip->engines[nr - 1].led_mux, mux);

	return sprintf(buf, "%s\n", mux);
}
show_leds(1)
show_leds(2)
show_leds(3)

static int lp5523_load_mux(struct lp55xx_chip *chip, u16 mux, int nr)
{
	struct lp55xx_engine *engine = &chip->engines[nr - 1];
	int ret;
	u8 mux_page[] = {
		[LP55XX_ENGINE_1] = LP5523_PAGE_MUX1,
		[LP55XX_ENGINE_2] = LP5523_PAGE_MUX2,
		[LP55XX_ENGINE_3] = LP5523_PAGE_MUX3,
	};

	lp5523_load_engine(chip);

	ret = lp55xx_write(chip, LP5523_REG_PROG_PAGE_SEL, mux_page[nr]);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_PROG_MEM , (u8)(mux >> 8));
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_PROG_MEM + 1, (u8)(mux));
	if (ret)
		return ret;

	engine->led_mux = mux;
	return 0;
}

static ssize_t store_engine_leds(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_engine *engine = &chip->engines[nr - 1];
	u16 mux = 0;
	ssize_t ret;

	if (lp5523_mux_parse(buf, &mux, len))
		return -EINVAL;

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;
	ret = -EINVAL;

	if (engine->mode != LP55XX_ENGINE_LOAD)
		goto leave;

	if (lp5523_load_mux(chip, mux, nr))
		goto leave;

	ret = len;
leave:
	mutex_unlock(&chip->lock);
	return ret;
}
store_leds(1)
store_leds(2)
store_leds(3)

static ssize_t store_engine_load(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;
	lp5523_load_engine_and_select_page(chip);

	mutex_unlock(&chip->lock);

	return lp5523_update_program_memory(chip, buf, len);
}
store_load(1)
store_load(2)
store_load(3)
/* end  NBQ - MaoyiChou - [NBQ-775] */

static ssize_t lp5523_selftest(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;
	int i, ret, pos = 0;
	u8 status, adc, vdd;

	mutex_lock(&chip->lock);

	ret = lp55xx_read(chip, LP5523_REG_STATUS, &status);
	if (ret < 0)
		goto fail;

	/* Check that ext clock is really in use if requested */
	if (pdata->clock_mode == LP55XX_CLOCK_EXT) {
		if  ((status & LP5523_EXT_CLK_USED) == 0)
			goto fail;
	}

	/* Measure VDD (i.e. VBAT) first (channel 16 corresponds to VDD) */
	lp55xx_write(chip, LP5523_REG_LED_TEST_CTRL, LP5523_EN_LEDTEST | 16);
	usleep_range(3000, 6000); /* ADC conversion time is typically 2.7 ms */
	ret = lp55xx_read(chip, LP5523_REG_STATUS, &status);
	if (ret < 0)
		goto fail;

	if (!(status & LP5523_LEDTEST_DONE))
		usleep_range(3000, 6000); /* Was not ready. Wait little bit */

	ret = lp55xx_read(chip, LP5523_REG_LED_TEST_ADC, &vdd);
	if (ret < 0)
		goto fail;

	vdd--;	/* There may be some fluctuation in measurement */

	for (i = 0; i < LP5523_MAX_LEDS; i++) {
		/* Skip non-existing channels */
		if (pdata->led_config[i].led_current == 0)
			continue;

		/* Set default current */
		lp55xx_write(chip, LP5523_REG_LED_CURRENT_BASE + i,
			pdata->led_config[i].led_current);

		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, 0xff);
		/* let current stabilize 2 - 4ms before measurements start */
		usleep_range(2000, 4000);
		lp55xx_write(chip, LP5523_REG_LED_TEST_CTRL,
			     LP5523_EN_LEDTEST | i);
		/* ADC conversion time is 2.7 ms typically */
		usleep_range(3000, 6000);
		ret = lp55xx_read(chip, LP5523_REG_STATUS, &status);
		if (ret < 0)
			goto fail;

		if (!(status & LP5523_LEDTEST_DONE))
			usleep_range(3000, 6000);/* Was not ready. Wait. */

		ret = lp55xx_read(chip, LP5523_REG_LED_TEST_ADC, &adc);
		if (ret < 0)
			goto fail;

		if (adc >= vdd || adc < LP5523_ADC_SHORTCIRC_LIM)
			pos += sprintf(buf + pos, "LED %d FAIL\n", i);

		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, 0x00);

		/* Restore current */
		lp55xx_write(chip, LP5523_REG_LED_CURRENT_BASE + i,
			led->led_current);
		led++;
	}
	if (pos == 0)
		pos = sprintf(buf, "OK\n");
	goto release_lock;
fail:
	pos = sprintf(buf, "FAIL\n");

release_lock:
	mutex_unlock(&chip->lock);

	return pos;
}

/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
static ssize_t lp5523_rwtest(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_device_config *cfg = chip->cfg;
	int ret, pos = 0;
	u8 addr = cfg->enable.addr;
	u8 val  = cfg->enable.val;

	mutex_lock(&chip->lock);

	ret = lp55xx_write(chip, addr, val);
	if (ret)
		goto fail;

	usleep_range(1000, 2000);

	ret = lp55xx_read(chip, addr, &val);
	if (ret)
		goto fail;

	if (val != cfg->enable.val)
		pos += sprintf(buf + pos, "TEST FAIL\n");

	if (pos == 0)
		pos = sprintf(buf, "OK\n");
	goto release_lock;

fail:
	pos = sprintf(buf, "FAIL\n");

release_lock:
	mutex_unlock(&chip->lock);

	return pos;
}
/* end  NBQ - MaoyiChou - [NBQ-775] */

static void lp5523_led_brightness_work(struct work_struct *work)
{
	struct lp55xx_led *led = container_of(work, struct lp55xx_led,
					      brightness_work);
	struct lp55xx_chip *chip = led->chip;

/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
	if (led->brightness == LED_OFF)	{
		mutex_lock(&chip->lock);
		allowAccess = false;
		// set output off control bit
		lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_MSB, 0x0);
		lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_LSB, 0x0);
		mutex_unlock(&chip->lock);
	} else {
		mutex_lock(&chip->lock);
		allowAccess = true;
        mutex_unlock(&chip->lock);
	}
}

static ssize_t store_led_control(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;

	char junk;
	unsigned value[LP5523_MAX_LEDS];
	int res, i;

	if (allowAccess == false) return len;

	res = sscanf(buf, "%d %d %d %d %d %d %d %d %d %c", &value[0], &value[1], &value[2], &value[3], &value[4], &value[5], &value[6], &value[7], &value[8], &junk);

	mutex_lock(&chip->lock);

	for (i = 0; i < LP5523_MAX_LEDS; i++)
		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, value[i]);

	// set output on off control bit
	res = lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_LSB, 0xff);
	res = lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_MSB, 0x01);

	mutex_unlock(&chip->lock);

	return len;
}

static ssize_t show_led_pattern(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;

	return sprintf(buf, "%d\n", chip->led_patterns);
}

static ssize_t store_led_pattern(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;

	int res, i, j;
	char junk;
	unsigned mode;

	typedef const u8 leds_fw_array[LP5523_PROGRAM_LENGTH];

	struct leds_pattern_fw_setting {
		leds_fw_array *engine_fw_array;
		unsigned short size;
		u8 * engine_start_addr;
		unsigned short num;
	};

	struct leds_engine_info {
		u8 prog_start;
		u8 mode_mask;
		u8 mode_value;
	};

	static leds_fw_array alwayon_engine_array[] = {
		{ 0x01, 0x0F, 0x9C, 0x00, 0x40, 0xFF }
	};

	static leds_fw_array pulse_right_engine_array[] = {
		{ 0x00, 0x0F, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x0E, 0x00, 0x07, 0x00, 0x03, 0x00, 0x01, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x06, 0x00, 0x03, 0x00, 0x01, 0x9C, 0x01, 0x9C, 0x8B, 0x04, 0x50, 0x9D, 0x80},
		{ 0xA2, 0x82, 0x05, 0x78, 0x9D, 0x80, 0xA2, 0x05, 0xA0, 0x02 }
	};

	static leds_fw_array pulse_left_engine_array[] = {
		{ 0x00, 0x0F, 0x00, 0x01, 0x00, 0x03, 0x00, 0x07, 0x00, 0x0E, 0x00, 0x0C, 0x00, 0x08, 0x00, 0x01, 0x00, 0x03, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x08, 0x9C, 0x01, 0x9C, 0x8B, 0x04, 0x50, 0x9D, 0x80},
		{ 0xA2, 0x82, 0x05, 0x78, 0x9D, 0x80, 0xA2, 0x05, 0xA0, 0x02 }
	};

	static leds_fw_array pulse_forth_engine_array[] = {
		{ 0x00, 0x0F, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x0E, 0x00, 0x07, 0x00, 0x03, 0x00, 0x01, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x06, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x07, 0x00, 0x0E},
		{ 0x00, 0x0C, 0x00, 0x08, 0x00, 0x01, 0x00, 0x03, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x08, 0x9C, 0x01, 0x9C, 0x8B, 0x04, 0x50, 0x9D, 0x80, 0xA2, 0x82, 0x05, 0x78, 0x9D, 0x80, 0xA2, 0x05, 0x9C, 0x0C},
		{ 0x9C, 0x96, 0xA0, 0x82, 0xA0}
	};

	static leds_fw_array sparkle_engine_array[] = {
		{ 0x9D, 0x01, 0x40, 0x14, 0x9D, 0x04, 0x40, 0x14, 0x9D, 0x01, 0x08, 0xDC, 0x4C, 0x00, 0x9D, 0x04, 0x06, 0xDC, 0x9D, 0x01, 0x0D, 0xDC, 0x46, 0x00, 0x9D, 0x04, 0x07, 0xDC, 0x54, 0x00, 0xA0, 0x04},
		{ 0x00, 0x00, 0x9D, 0x02, 0x40, 0x14, 0x04, 0xDC, 0x46, 0x00, 0x0B, 0xDC, 0x54, 0x00, 0xA0, 0x02, 0x00, 0x00, 0x9D, 0x03, 0x40, 0x14, 0x0E, 0xDC, 0x4C, 0x00, 0x09, 0xDC, 0x5A, 0x00, 0xA0, 0x02}
	};

	static u8 common_engine_start_addr[] = {0x01};

	static u8 pulse_right_engine_start_addr[] = {0x0C};

	static u8 pulse_left_engine_start_addr[] = {0x0C};

	static u8 pulse_forth_engine_start_addr[] = {0x17};

	static u8 sparkle_engine_start_addr[] = {0x00, 0x11, 0x19};

	static struct leds_pattern_fw_setting leds_pattern[] = {
		{alwayon_engine_array, ARRAY_SIZE(alwayon_engine_array), common_engine_start_addr, ARRAY_SIZE(common_engine_start_addr)},
		{pulse_right_engine_array, ARRAY_SIZE(pulse_right_engine_array), pulse_right_engine_start_addr, ARRAY_SIZE(pulse_right_engine_start_addr)},
		{pulse_left_engine_array, ARRAY_SIZE(pulse_left_engine_array), pulse_left_engine_start_addr, ARRAY_SIZE(pulse_left_engine_start_addr)},
		{pulse_forth_engine_array, ARRAY_SIZE(pulse_forth_engine_array), pulse_forth_engine_start_addr, ARRAY_SIZE(pulse_forth_engine_start_addr)},
		{sparkle_engine_array, ARRAY_SIZE(sparkle_engine_array), sparkle_engine_start_addr, ARRAY_SIZE(sparkle_engine_start_addr)},
	};

	static struct leds_engine_info engine_info[] = {
		{LP5523_REG_CH1_PROG_START, LP5523_MODE_ENG1_M, LP5523_LOAD_ENG1},
		{LP5523_REG_CH2_PROG_START, LP5523_MODE_ENG2_M, LP5523_LOAD_ENG2},
		{LP5523_REG_CH3_PROG_START, LP5523_MODE_ENG3_M, LP5523_LOAD_ENG3}
	};


	res = sscanf(buf, "%d %s", &mode, &junk);

	mutex_lock(&chip->lock);

	chip->led_patterns = mode;

	if (mode == 0)
	{
		// To stop running pattern
		lp5523_run_engine(chip, false);
		lp5523_stop_engine(chip);
	} else if (mode == 116) {
		// This project do NOT support that, do nothing
		dev_err(&chip->cl->dev, "NOT support that, do nothing! \n");
	} else if (mode <= ARRAY_SIZE(leds_pattern)) {
		// shift for pattern array
		mode = mode - 1;

		for (j = 0 ; j < leds_pattern[mode].num; j++)
		{
			// 1. Specify program start address
			res = lp55xx_write(chip, engine_info[j].prog_start, leds_pattern[mode].engine_start_addr[j]);

			// 2. Load engine
			lp55xx_update_bits(chip, LP5523_REG_OP_MODE, engine_info[j].mode_mask, engine_info[j].mode_value);
			lp5523_wait_opmode_done();
		}

		for (j = 0; j < leds_pattern[mode].size; j++)
		{
			// 3. Select the program memory page
			lp55xx_write(chip, LP5523_REG_PROG_PAGE_SEL, j);

			// 4. Write pattern data into selected area
			for (i = 0; i < LP5523_PROGRAM_LENGTH; i++) {
				res = lp55xx_write(chip, LP5523_REG_PROG_MEM + i,
					leds_pattern[mode].engine_fw_array[j][i]);
			}
		}

		// 5. Run engine
		lp5523_run_engine(chip, true);
	}

	mutex_unlock(&chip->lock);

	return len;
}

/*  NBQ - MaoyiChou - [NBQ-798] - [FTM] Implement cloud led command.*/
static ssize_t store_leds_on_off(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;

	unsigned value[2];
	u8 pwm_value[LP5523_CUSTOMER_LEDS], val, i, led_pos, led_pwm;
	int res;


	mutex_lock(&chip->lock);
	res = sscanf(buf, "%d %d", &value[0], &value[1]);
	led_pos = value[0];
	led_pwm = value[1];

	for (i = 0; i < LP5523_CUSTOMER_LEDS; i++){
		res = lp55xx_read(chip, LP5523_REG_LED_PWM_BASE + i, &val);
		pwm_value[i] = val;
	}
	pwm_value[led_pos-1] = led_pwm;


	for (i = 0; i < LP5523_CUSTOMER_LEDS; i++)
		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, pwm_value[i]);

	// set output on control bit
	res = lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_LSB, 0x0f);

	mutex_unlock(&chip->lock);

	return len;
}
/* end  NBQ - MaoyiChou - [NBQ-798] */

static ssize_t store_load_firmware_run(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;

	#define PROG_MEM_PAGE 6
	#define TEST_FILE_PATH "/data/FIH-Nextbit.hex"
	struct file  *fp;
	char val[650];
	mm_segment_t fs;
	long l;

	u8 pattern[LP5523_PROGRAM_LENGTH*6] = {0};
	unsigned cmd;
	char c[3];
	int nrchars;
	int ret = 1;
	int offset = 0;
	int i = 0, j = 0, k = 0, engine_number = 0;
	unsigned mode;
	u8 engine_start_addr[3] = {0};
	ssize_t rlen;

	struct leds_engine_info {
		u8 prog_start;
		u8 mode_mask;
		u8 mode_value;
		u8 page;
	};

	static u8 alwayon_engine_array[] = {0x01, 0x0F, 0x9C, 0x00, 0x40, 0xFF};

	static u8 common_engine_start_addr[] = {0x01};

	static struct leds_engine_info engine_info[] = {
		{LP5523_REG_CH1_PROG_START, LP5523_MODE_ENG1_M, LP5523_LOAD_ENG1, LP5523_PAGE_ENG1},
		{LP5523_REG_CH2_PROG_START, LP5523_MODE_ENG2_M, LP5523_LOAD_ENG2, LP5523_PAGE_ENG2},
		{LP5523_REG_CH3_PROG_START, LP5523_MODE_ENG3_M, LP5523_LOAD_ENG3, LP5523_PAGE_ENG3}
	};

	sscanf(buf, "%d", &mode);
	//dev_err(&chip->cl->dev, "mode =%d\n", mode);

	mutex_lock(&chip->lock);

	//dev_err(&chip->cl->dev, "size =%zd, %zd\n", ARRAY_SIZE(alwayon_engine_array), ARRAY_SIZE(common_engine_start_addr));
	//dev_err(&chip->cl->dev, "size =%zd, %zd\n", strlen(alwayon_engine_array), strlen(common_engine_start_addr));
	if (mode == 0)
	{
		// To stop running pattern
		lp5523_run_engine(chip, false);
		lp5523_stop_engine(chip);
	}else{
		fs = get_fs();
		set_fs(get_ds());
		fp = filp_open(TEST_FILE_PATH, O_RDONLY,S_IRUSR);
		if(IS_ERR(fp)){
			//dev_err(&chip->cl->dev, "open /data/FIH-Nextbit.hex failed = %ld\n", PTR_ERR(fp));
			engine_number = 1;
			memcpy(pattern, alwayon_engine_array, ARRAY_SIZE(alwayon_engine_array));
			memcpy(engine_start_addr, common_engine_start_addr, ARRAY_SIZE(common_engine_start_addr));
		}else{
			l = i_size_read(fp->f_path.dentry->d_inode);
			//dev_err(&chip->cl->dev, "l len is %ld\n", l);
			memset(val, 0, sizeof(val));
			fp->f_op->read(fp,val,l,&fp->f_pos);
			//dev_err(&chip->cl->dev, "read data is\n%s\n", val);

			i = 0;
			while ((offset < l - 1) && (i < LP5523_PROGRAM_LENGTH*6)) {
				/* separate sscanfs because length is working only for %s */
				ret = sscanf(val + offset, "%2s%n ", c, &nrchars);
				if (ret != 1)
					break;
				ret = sscanf(c, "%2x", &cmd);
				if (ret != 1)
					break;
				pattern[i] = (u8)cmd;
				//dev_err(&chip->cl->dev, "pattern[%d]=0x%x\n", i, pattern[i]);
				offset += nrchars;
				i++;
			}

			i = 0;
			offset = 0;
			while (1){
				memset(val, 0, sizeof(val));
				fp->f_op->llseek(fp, 600+offset+2, SEEK_SET);
				rlen = fp->f_op->read(fp,val,2,&fp->f_pos);
				//dev_err(&chip->cl->dev, "rlen=%zd\n", rlen);
				//dev_err(&chip->cl->dev, "read data is\n%s\n", val);
				offset+= 15;
				if (rlen > 0){
					ret = sscanf(val, "%2s ", c);
					if (ret != 1)
						break;
					ret = sscanf(c, "%2x", &cmd);
					if (ret != 1)
						break;
					engine_start_addr[i] = (u8)cmd;
					//dev_err(&chip->cl->dev, "engine_start_addr[%d]=0x%x\n", i, engine_start_addr[i]);
					i++;
				}
				else
					break;
				engine_number = i;
			}

			filp_close(fp,NULL);
		}

		set_fs(fs);

		if (ret != 1){
			dev_err(&chip->cl->dev, "data format error!\n");
			return len;
		}

		//dev_err(&chip->cl->dev, "engine_number=%d \n", engine_number);
		for (j = 0 ; j < engine_number; j++)
		{
			//dev_err(&chip->cl->dev, "engine_start_addr[%d]=0x%x \n", j, engine_start_addr[j]);
			// 1. Specify program start address
			lp55xx_write(chip, engine_info[j].prog_start, engine_start_addr[j]);

			// 2. Load program to SRAM
			lp55xx_update_bits(chip, LP5523_REG_OP_MODE, engine_info[j].mode_mask, engine_info[j].mode_value);
			lp5523_wait_opmode_done();
		}
		// 3. Select the program memory page
		for (i = 0; i < PROG_MEM_PAGE; i++){
			lp55xx_write(chip, LP5523_REG_PROG_PAGE_SEL, i);
			// 4. Write pattern data into page area
			for (j = 0; j < LP5523_PROGRAM_LENGTH; j++) {
				//dev_err(&chip->cl->dev, "pattern[%d]=0x%x \n", k, pattern[k]);
				lp55xx_write(chip, LP5523_REG_PROG_MEM + j, pattern[k++]);
			}
		}
		// 5. Run engine
		lp5523_run_engine(chip, true);

	}

	mutex_unlock(&chip->lock);

	return len;
}


static LP55XX_DEV_ATTR_RW(engine1_mode, show_engine1_mode, store_engine1_mode);
static LP55XX_DEV_ATTR_RW(engine2_mode, show_engine2_mode, store_engine2_mode);
static LP55XX_DEV_ATTR_RW(engine3_mode, show_engine3_mode, store_engine3_mode);
static LP55XX_DEV_ATTR_RW(engine1_leds, show_engine1_leds, store_engine1_leds);
static LP55XX_DEV_ATTR_RW(engine2_leds, show_engine2_leds, store_engine2_leds);
static LP55XX_DEV_ATTR_RW(engine3_leds, show_engine3_leds, store_engine3_leds);
static LP55XX_DEV_ATTR_WO(engine1_load, store_engine1_load);
static LP55XX_DEV_ATTR_WO(engine2_load, store_engine2_load);
static LP55XX_DEV_ATTR_WO(engine3_load, store_engine3_load);
static LP55XX_DEV_ATTR_RO(selftest, lp5523_selftest);
static LP55XX_DEV_ATTR_RO(rwtest, lp5523_rwtest);
static LP55XX_DEV_ATTR_WO(led_control, store_led_control);
static LP55XX_DEV_ATTR_RW(led_pattern, show_led_pattern, store_led_pattern);
static LP55XX_DEV_ATTR_WO(leds_on_off, store_leds_on_off); //  NBQ - MaoyiChou - [NBQ-798] - [Cloud LED] Implement leds on/off for lp5523.
static LP55XX_DEV_ATTR_WO(load_firmware_run, store_load_firmware_run);


static struct attribute *lp5523_attributes[] = {
	&dev_attr_engine1_mode.attr,
	&dev_attr_engine2_mode.attr,
	&dev_attr_engine3_mode.attr,
	&dev_attr_engine1_load.attr,
	&dev_attr_engine2_load.attr,
	&dev_attr_engine3_load.attr,
	&dev_attr_engine1_leds.attr,
	&dev_attr_engine2_leds.attr,
	&dev_attr_engine3_leds.attr,
	&dev_attr_selftest.attr,
	&dev_attr_rwtest.attr,
	&dev_attr_led_control.attr,
	&dev_attr_led_pattern.attr,
	&dev_attr_leds_on_off.attr, //  NBQ - MaoyiChou - [NBQ-798] - [Cloud LED] Implement leds on/off for lp5523.
	&dev_attr_load_firmware_run.attr,
	NULL,
};
/* end  NBQ - MaoyiChou - [NBQ-775] */

static const struct attribute_group lp5523_group = {
	.attrs = lp5523_attributes,
};

/* Chip specific configurations */
static struct lp55xx_device_config lp5523_cfg = {
	.reset = {
		.addr = LP5523_REG_RESET,
		.val  = LP5523_RESET,
	},
	.enable = {
		.addr = LP5523_REG_ENABLE,
		.val  = LP5523_ENABLE,
	},
	.max_channel  = LP5523_MAX_LEDS,
	.post_init_device   = lp5523_post_init_device,
	.brightness_work_fn = lp5523_led_brightness_work,
	.set_led_current    = lp5523_set_led_current,
	.firmware_cb        = lp5523_firmware_loaded,
	.run_engine         = lp5523_run_engine,
	.dev_attr_group     = &lp5523_group,
};

/*  NBQ - MaoyiChou - [NBQ-1102] - [Cloud LED] Add the LP5523 power manager.*/
#ifdef CONFIG_PM
static int lp5523_suspend(struct device *dev)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int ret = 0;


	if (chip->led_patterns == 0)
	{
		mutex_lock(&chip->lock);
		ret = lp55xx_pwr_on(chip, false);
		if (ret)
			dev_err(&chip->cl->dev, "lp5523 suspend power fail\n");
		mutex_unlock(&chip->lock);
	}
	return ret;
}

static int lp5523_resume(struct device *dev)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
/*FIH, Hubert, 20160115, fix the brightness changed problem {*/
	struct lp55xx_platform_data *pdata = chip->pdata;
	struct lp55xx_device_config *cfg = chip->cfg;
	int num_channels = pdata->num_channels;
	struct lp55xx_led *each;
	u8 led_current;
	int i;
/*} FIH, Hubert, 20160115, fix the brightness changed problem*/
	int ret = 0;

	if (chip->led_patterns == 0)
	{
		mutex_lock(&chip->lock);
		ret = lp55xx_pwr_on(chip, true);
		if (ret)
			dev_err(&chip->cl->dev, "lp5523 resume power fail\n");
		ret = lp55xx_init_device(chip);
		if (ret)
			dev_err(&chip->cl->dev, "lp5523 resume device fail\n");
		mutex_unlock(&chip->lock);
/*FIH, Hubert, 20160115, fix the brightness changed problem {*/
		for (i = 0; i < num_channels; i++)
		{
			if (pdata->led_config[i].led_current == 0)
				continue;
			led_current = pdata->led_config[i].led_current;
			each = led + i;
			if (cfg->set_led_current)
			cfg->set_led_current(each, led_current);
		}
/*} FIH, Hubert, 20160115, fix the brightness changed problem*/
	}
	return ret;
}

static const struct dev_pm_ops lp5523_pm_ops = {
	.suspend = lp5523_suspend,
	.resume = lp5523_resume,
};
#else
static int lp5523_suspend(struct device *dev)
{
	return 0;
}

static int lp5523_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops lp5523_pm_ops = {
};
#endif
/* end  NBQ - MaoyiChou - [NBQ-1102] */

static int lp5523_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct lp55xx_chip *chip;
	struct lp55xx_led *led;
/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
	struct lp55xx_platform_data *pdata;
	struct device_node *np = client->dev.of_node;

	if (!dev_get_platdata(&client->dev)) {
		if (np) {
			ret = lp55xx_of_populate_pdata(&client->dev, np);
			if (ret < 0)
				return ret;
		} else {
			dev_err(&client->dev, "no platform data\n");
			return -EINVAL;
		}
	}
	pdata = dev_get_platdata(&client->dev);
/* end  NBQ - MaoyiChou - [NBQ-775] */

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	led = devm_kzalloc(&client->dev,
			sizeof(*led) * pdata->num_channels, GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	chip->cl = client;
	chip->pdata = pdata;
	chip->cfg = &lp5523_cfg;

	mutex_init(&chip->lock);

	i2c_set_clientdata(client, led);

/*  NBQ - MaoyiChou - [NBQ-1102] - [Cloud LED] Add the LP5523 power manager.*/
	ret = lp55xx_pwr_init(chip, true);
	if (ret)
		goto err_init;
	ret = lp55xx_pwr_on(chip, true);
	if (ret)
		goto err_init;
/* end  NBQ - MaoyiChou - [NBQ-1102] */
	ret = lp55xx_init_device(chip);
	if (ret)
		goto err_init;

	dev_info(&client->dev, "%s Programmable led chip found\n", id->name);

	ret = lp55xx_register_leds(led, chip);
	if (ret)
		goto err_register_leds;

	ret = lp55xx_register_sysfs(chip);
	if (ret) {
		dev_err(&client->dev, "registering sysfs failed\n");
		goto err_register_sysfs;
	}

	return 0;

err_register_sysfs:
	lp55xx_unregister_leds(led, chip);
err_register_leds:
	lp55xx_deinit_device(chip);
err_init:
	return ret;
}

static int lp5523_remove(struct i2c_client *client)
{
	struct lp55xx_led *led = i2c_get_clientdata(client);
	struct lp55xx_chip *chip = led->chip;

	lp5523_stop_engine(chip);
	lp55xx_unregister_sysfs(chip);
	lp55xx_unregister_leds(led, chip);
	lp55xx_deinit_device(chip);

	return 0;
}

static const struct i2c_device_id lp5523_id[] = {
	{ "lp5523",  LP5523 },
	{ "lp55231", LP55231 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, lp5523_id);

/*  NBQ - MaoyiChou - [NBQ-775] - [Cloud LED] Implement LP5523 driver. */
#ifdef CONFIG_OF
static const struct of_device_id of_lp5523_leds_match[] = {
	{ .compatible = "national,lp5523", },
	{},
};

MODULE_DEVICE_TABLE(of, of_lp5523_leds_match);
#endif
/* end  NBQ - MaoyiChou - [NBQ-775] */

static struct i2c_driver lp5523_driver = {
	.driver = {
		.name	= "lp5523x",
		.of_match_table = of_match_ptr(of_lp5523_leds_match), //MaoyiChou,2015/6/24, Implement 9-channel led driver
/*  NBQ - MaoyiChou - [NBQ-1102] - [Cloud LED] Add the LP5523 power manager.*/
#ifdef CONFIG_PM
		.pm = &lp5523_pm_ops,
#endif
/* end  NBQ - MaoyiChou - [NBQ-1102] */
	},
	.probe		= lp5523_probe,
	.remove		= lp5523_remove,
	.id_table	= lp5523_id,
};

module_i2c_driver(lp5523_driver);

MODULE_AUTHOR("Mathias Nyman <mathias.nyman@nokia.com>");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>");
MODULE_DESCRIPTION("LP5523 LED engine");
MODULE_LICENSE("GPL");
