/*
 * This file is part of the AP3223, AP3212C and AP3216C sensor driver.
 * AP3426 is combined proximity and ambient light sensor.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 *	    Templeton Tsai <templeton.tsai@dyna-image.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3426.c
 *
 * Summary:
 *	AP3426 device driver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 02/02/12 YC       1. Modify irq function to seperate two interrupt routine.
 *                   2. Fix the index of reg array error in em write.
 * 02/22/12 YC	     3. Merge AP3426 and AP3216C into the same driver. (ver 1.8)
 * 03/01/12 YC       Add AP3212C into the driver. (ver 1.8)
 * 07/25/14 John     Ver.2.1 , ported for Nexus 7
 * 08/21/14 Templeton AP3426 Ver 1.0, ported for Nexus 7
 * 07/29/15 Vic      ported for Chrome Arkham project (kernel 3.14)
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include "ap3223.h"

#define AP3223_DRV_NAME		"ap3223"
#define DRIVER_VERSION		"1"


#define PL_TIMER_DELAY 2000
#define POLLING_MODE 0

#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s, args...) \
{pr_info("AP3223: func [%s], line [%d], ", __func__, __LINE__); \
pr_info(s, ## args); }
#else
#define LDBG(s, args...) {}
#endif
static void plsensor_work_handler(struct work_struct *w);
#if POLLING_MODE
static void pl_timer_callback(unsigned long pl_data);
#endif
static int ap3223_set_phthres(struct i2c_client *client, int val);
static int ap3223_set_plthres(struct i2c_client *client, int val);

struct ap3223_data {
	struct i2c_client *client;
	u8 reg_cache[AP3223_NUM_CACHABLE_REGS];	/* TO-DO */
	u8 power_state_before_suspend;
	int irq;
	int hsensor_enable;
	struct input_dev *psensor_input_dev;
	struct input_dev *lsensor_input_dev;
	struct input_dev *hsensor_input_dev;
	struct workqueue_struct *plsensor_wq;
	struct work_struct plsensor_work;
#if POLLING_MODE
	struct timer_list pl_timer;
#endif
};

static struct ap3223_data *ap3223_data_g;
/* AP3223 register */
static u8 ap3223_reg[AP3223_NUM_CACHABLE_REGS] = {
	0x00, 0x01, 0x02, 0x06, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	0x10, 0x14, 0x1A, 0x1B, 0x1C, 0x1D,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2A, 0x2B,
	0x2C, 0x2D, 0x30, 0x32
};

#define INIT_ARKHAM
/* platform related register initialization shold be done in user space
 * during boot up, * we temporally add it here for factory testing */
#ifdef INIT_ARKHAM
/* arkham initial value */
static u8 init_reg[] = {
	0x0, 0x3,
	0x10, 0x20,
	0x1a, 0xe8,
	0x1b, 0x3,
	0x1c, 0xd0,
	0x1d, 0x7,
	0x20, 0x4,
	0x21, 0x3,
	0x22, 0x1,
	0x23, 0x3,
	0x25, 0xb,
	0x2a, 0x64,
	0x2c, 0xf4,
	0x2d, 0x0
};
#endif

/* AP3223 range */
static int ap3223_range[4] = { 65535, 16383, 4095, 1023 };



static u8 *reg_array;
static int *range;
static int reg_num;

static int cali;
static int misc_ps_opened;
static int misc_ls_opened;

static DEFINE_MUTEX(ap3223_lock);
static DEFINE_MUTEX(ap3223_ls_lock);
static DEFINE_MUTEX(ap3223_ps_lock);
static DEFINE_MUTEX(ap3223_heartbeat_lock);
#define ADD_TO_IDX(addr, idx)	{ \
	int i; \
	for (i = 0; i < reg_num; i++) { \
		if (addr == reg_array[i]) { \
			idx = i; \
			break; \
		} \
	} \
}


/*
 * register access helpers
 */

static int __ap3223_read_reg(struct i2c_client *client,
			     u32 reg, u8 mask, u8 shift)
{
	struct ap3223_data *data = i2c_get_clientdata(client);
	u8 idx = 0xff;

	ADD_TO_IDX(reg, idx)
	    return (data->reg_cache[idx] & mask) >> shift;
}

static int __ap3223_write_reg(struct i2c_client *client,
			      u32 reg, u8 mask, u8 shift, u8 val)
{
	struct ap3223_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;
	u8 idx = 0xff;

	ADD_TO_IDX(reg, idx)
	    if (idx >= reg_num)
		return -EINVAL;

	tmp = data->reg_cache[idx];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		data->reg_cache[idx] = tmp;

	return ret;
}

/*
 * internally used functions
 */

/* range */
static int ap3223_get_range(struct i2c_client *client)
{
	u8 idx = __ap3223_read_reg(client, AP3223_REG_ALS_CONF,
				   AP3223_ALS_RANGE_MASK,
				   AP3223_ALS_RANGE_SHIFT);
	return range[idx];
}

static int ap3223_set_range(struct i2c_client *client, int range)
{
	return __ap3223_write_reg(client, AP3223_REG_ALS_CONF,
				  AP3223_ALS_RANGE_MASK,
				  AP3223_ALS_RANGE_SHIFT, range);
}


static int ap3223_set_ir_data(struct i2c_client *client, int en)
{
	int ret = 0;

	if (en == 9) {
		ret = __ap3223_write_reg(client, AP3223_REG_SYS_CONF,
					 AP3223_REG_SYS_CONF_MASK,
					 AP3223_REG_SYS_CONF_SHIFT,
					 AP3223_SYS_DEV_RESET);
		mdelay(200);
		ret = __ap3223_write_reg(client, AP3223_REG_PS_CONF,
					 AP3223_REG_PS_CONF_MASK,
					 AP3223_REG_PS_CONF_SHIFT, 0);
		ret =
		    __ap3223_write_reg(client, AP3223_REG_PS_DC_1,
				       AP3223_REG_PS_DC_1_MASK,
				       AP3223_REG_PS_DC_1_SHIFT, 0);
		ret =
		    __ap3223_write_reg(client, AP3223_REG_PS_DC_2,
				       AP3223_REG_PS_DC_2_MASK,
				       AP3223_REG_PS_DC_2_SHIFT, 0);
		ret =
		    __ap3223_write_reg(client, AP3223_REG_PS_LEDD,
				       AP3223_REG_PS_LEDD_MASK,
				       AP3223_REG_PS_LEDD_SHIFT, 1);
		ret =
		    __ap3223_write_reg(client, AP3223_REG_PS_MEAN,
				       AP3223_REG_PS_MEAN_MASK,
				       AP3223_REG_PS_MEAN_SHIFT, 0);
		ret =
		    __ap3223_write_reg(client, AP3223_REG_PS_PERSIS,
				       AP3223_REG_PS_PERSIS_MASK,
				       AP3223_REG_PS_PERSIS_SHIFT, 0);
		ret = ap3223_set_plthres(client, 0);
		ret = ap3223_set_phthres(client, 535);
		ret = __ap3223_write_reg(client, AP3223_REG_SYS_CONF,
					 AP3223_REG_SYS_CONF_MASK,
					 AP3223_REG_SYS_CONF_SHIFT,
					 AP3223_SYS_PS_ENABLE);
	} else if (en == 0) {
		ret = __ap3223_write_reg(client, AP3223_REG_SYS_CONF,
					 AP3223_REG_SYS_CONF_MASK,
					 AP3223_REG_SYS_CONF_SHIFT,
					 AP3223_SYS_DEV_DOWN);
		mdelay(200);
	}

	return ret;
}

/* mode */
static int ap3223_get_mode(struct i2c_client *client)
{
	int ret;

	ret = __ap3223_read_reg(client, AP3223_REG_SYS_CONF,
				AP3223_REG_SYS_CONF_MASK,
				AP3223_REG_SYS_CONF_SHIFT);
	return ret;
}

static int ap3223_set_mode(struct i2c_client *client, int mode)
{
	int ret;

	ret = __ap3223_write_reg(client, AP3223_REG_SYS_CONF,
				 AP3223_REG_SYS_CONF_MASK,
				 AP3223_REG_SYS_CONF_SHIFT, mode);

	return ret;
}

/* ALS low threshold */
static int ap3223_get_althres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3223_read_reg(client, AP3223_REG_ALS_THDL_L,
				AP3223_REG_ALS_THDL_L_MASK,
				AP3223_REG_ALS_THDL_L_SHIFT);
	msb =
	    __ap3223_read_reg(client, AP3223_REG_ALS_THDL_H,
			      AP3223_REG_ALS_THDL_H_MASK,
			      AP3223_REG_ALS_THDL_H_SHIFT);
	return (msb << 8) | lsb;
}

static int ap3223_set_althres(struct i2c_client *client, int val)
{

	int lsb, msb, err;

	msb = val >> 8;
	lsb = val & AP3223_REG_ALS_THDL_L_MASK;

	err = __ap3223_write_reg(client, AP3223_REG_ALS_THDL_L,
				 AP3223_REG_ALS_THDL_L_MASK,
				 AP3223_REG_ALS_THDL_L_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3223_write_reg(client, AP3223_REG_ALS_THDL_H,
				 AP3223_REG_ALS_THDL_H_MASK,
				 AP3223_REG_ALS_THDL_H_SHIFT, msb);

	return err;
}

/* ALS high threshold */
static int ap3223_get_ahthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3223_read_reg(client, AP3223_REG_ALS_THDH_L,
				AP3223_REG_ALS_THDH_L_MASK,
				AP3223_REG_ALS_THDH_L_SHIFT);
	msb =
	    __ap3223_read_reg(client, AP3223_REG_ALS_THDH_H,
			      AP3223_REG_ALS_THDH_H_MASK,
			      AP3223_REG_ALS_THDH_H_SHIFT);
	return (msb << 8) | lsb;
}

static int ap3223_set_ahthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;

	msb = val >> 8;
	lsb = val & AP3223_REG_ALS_THDH_L_MASK;

	err = __ap3223_write_reg(client, AP3223_REG_ALS_THDH_L,
				 AP3223_REG_ALS_THDH_L_MASK,
				 AP3223_REG_ALS_THDH_L_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3223_write_reg(client, AP3223_REG_ALS_THDH_H,
				 AP3223_REG_ALS_THDH_H_MASK,
				 AP3223_REG_ALS_THDH_H_SHIFT, msb);

	return err;
}

/* PX low threshold */
static int ap3223_get_plthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3223_read_reg(client, AP3223_REG_PS_THDL_L,
				AP3223_REG_PS_THDL_L_MASK,
				AP3223_REG_PS_THDL_L_SHIFT);
	msb =
	    __ap3223_read_reg(client, AP3223_REG_PS_THDL_H,
			      AP3223_REG_PS_THDL_H_MASK,
			      AP3223_REG_PS_THDL_H_SHIFT);
	return (msb << 8) | lsb;
}

static int ap3223_set_plthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;

	msb = val >> 8;
	lsb = val & AP3223_REG_PS_THDL_L_MASK;

	err = __ap3223_write_reg(client, AP3223_REG_PS_THDL_L,
				 AP3223_REG_PS_THDL_L_MASK,
				 AP3223_REG_PS_THDL_L_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3223_write_reg(client, AP3223_REG_PS_THDL_H,
				 AP3223_REG_PS_THDL_H_MASK,
				 AP3223_REG_PS_THDL_H_SHIFT, msb);

	return err;
}

/* PX high threshold */
static int ap3223_get_phthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3223_read_reg(client, AP3223_REG_PS_THDH_L,
				AP3223_REG_PS_THDH_L_MASK,
				AP3223_REG_PS_THDH_L_SHIFT);
	msb =
	    __ap3223_read_reg(client, AP3223_REG_PS_THDH_H,
			      AP3223_REG_PS_THDH_H_MASK,
			      AP3223_REG_PS_THDH_H_SHIFT);
	return (msb << 8) | lsb;
}

static int ap3223_set_phthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;

	msb = val >> 8;
	lsb = val & AP3223_REG_PS_THDH_L_MASK;

	err = __ap3223_write_reg(client, AP3223_REG_PS_THDH_L,
				 AP3223_REG_PS_THDH_L_MASK,
				 AP3223_REG_PS_THDH_L_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3223_write_reg(client, AP3223_REG_PS_THDH_H,
				 AP3223_REG_PS_THDH_H_MASK,
				 AP3223_REG_PS_THDH_H_SHIFT, msb);

	return err;
}

/* PX calibration */
static int ap3223_get_pcali(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3223_read_reg(client, AP3223_REG_PS_CAL_L,
				AP3223_REG_PS_CAL_L_MASK,
				AP3223_REG_PS_CAL_L_SHIFT);
	msb =
	    __ap3223_read_reg(client, AP3223_REG_PS_CAL_H,
			      AP3223_REG_PS_CAL_H_MASK,
			      AP3223_REG_PS_CAL_H_SHIFT);
	return (msb << 8) | lsb;
}

static int ap3223_set_pcali(struct i2c_client *client, int val)
{
	int lsb, msb, err;

	msb = val >> 8;
	lsb = val & AP3223_REG_PS_CAL_L_MASK;

	err = __ap3223_write_reg(client, AP3223_REG_PS_CAL_L,
				 AP3223_REG_PS_CAL_L_MASK,
				 AP3223_REG_PS_CAL_L_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3223_write_reg(client, AP3223_REG_PS_CAL_H,
				 AP3223_REG_PS_CAL_H_MASK,
				 AP3223_REG_PS_CAL_H_SHIFT, msb);

	return err;
}

static int ap3223_get_adc_value(struct i2c_client *client)
{
	unsigned int lsb, msb, val;
#ifdef LSC_DBG
	unsigned int tmp, range;
#endif

	lsb = i2c_smbus_read_byte_data(client, AP3223_REG_ALS_DATA_LOW);

	if (lsb < 0)
		return lsb;

	msb = i2c_smbus_read_byte_data(client, AP3223_REG_ALS_DATA_HIGH);

	if (msb < 0)
		return msb;

	range = ap3223_get_range(client);

	tmp = (((msb << 8) | lsb) * range) >> 16;
	tmp = tmp * cali / 100;
	val = tmp;
	return val;
}


static int ap3223_get_object(struct i2c_client *client)
{
	int val;

	val = i2c_smbus_read_byte_data(client, AP3223_OBJ_COMMAND);
	val &= AP3223_OBJ_MASK;

	return val >> AP3223_OBJ_SHIFT;
}

static int ap3223_get_intstat(struct i2c_client *client)
{
	int val;

	val = i2c_smbus_read_byte_data(client, AP3223_REG_SYS_INTSTATUS);
	val &= AP3223_REG_SYS_INT_MASK;

	return val >> AP3223_REG_SYS_INT_SHIFT;
}


static int ap3223_get_px_value(struct i2c_client *client)
{
	int lsb, msb;

	lsb = i2c_smbus_read_byte_data(client, AP3223_REG_PS_DATA_LOW);

	if (lsb < 0)
		return lsb;

	/* LDBG("%s, IR = %d\n", __func__, (u32)(lsb)); */
	msb = i2c_smbus_read_byte_data(client, AP3223_REG_PS_DATA_HIGH);

	if (msb < 0)
		return msb;

	/* LDBG("%s, IR = %d\n", __func__, (u32)(msb)); */

	return (u32) (((msb & AL3223_REG_PS_DATA_HIGH_MASK) << 8) |
		      (lsb & AL3223_REG_PS_DATA_LOW_MASK));
}




#ifdef CONFIG_HAS_EARLYSUSPEND
static int ap3223_lsensor_enable(struct i2c_client *client)
{
	int ret = 0, mode;

	mode = ap3223_get_mode(client);
	if ((mode & AP3223_SYS_ALS_ENABLE) == 0) {
		mode |= AP3223_SYS_ALS_ENABLE;
		ret = ap3223_set_mode(client, mode);
	}

	return ret;
}

static int ap3223_lsensor_disable(struct i2c_client *client)
{
	int ret = 0, mode;

	mode = ap3223_get_mode(client);
	if (mode & AP3223_SYS_ALS_ENABLE) {
		mode &= ~AP3223_SYS_ALS_ENABLE;
		if (mode == AP3223_SYS_DEV_RESET)
			mode = 0;
		ret = ap3223_set_mode(client, mode);
	}

	return ret;
}
#endif

static int ap3223_register_lsensor_device(struct i2c_client *client,
					  struct ap3223_data *data)
{
	struct input_dev *input_dev;
	int rc;

	LDBG("allocating input device lsensor\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev,
			"%s: could not allocate input device for lsensor\n",
			__func__);
		rc = -ENOMEM;
		goto done;
	}
	data->lsensor_input_dev = input_dev;
	input_set_drvdata(input_dev, data);
	input_dev->name = "lightsensor-level";
	input_dev->dev.parent = &client->dev;
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_MISC, 0, 8, 0, 0);

	rc = input_register_device(input_dev);
	if (rc < 0) {
		pr_err("%s: could not register input device for lsensor\n",
		       __func__);
		goto done;
	}
done:
	return rc;
}

static void ap3223_unregister_lsensor_device(struct i2c_client *client,
					     struct ap3223_data *data)
{
	input_unregister_device(data->lsensor_input_dev);
}

static int ap3223_register_heartbeat_sensor_device(struct i2c_client
						   *client,
						   struct ap3223_data
						   *data)
{
	struct input_dev *input_dev;
	int rc;

	LDBG("allocating input device heartbeat sensor\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev,
		"%s: could not allocate input device for heartbeat sensor\n",
			__func__);
		rc = -ENOMEM;
		goto done;
	}
	data->hsensor_input_dev = input_dev;
	input_set_drvdata(input_dev, data);
	input_dev->name = "heartbeat";
	input_dev->dev.parent = &client->dev;
	set_bit(EV_REL, input_dev->evbit);
	input_set_capability(input_dev, EV_REL, ABS_WHEEL);
	rc = input_register_device(input_dev);
	if (rc < 0) {
		pr_err
		("%s: could not register input device for heartbeat sensor\n",
		     __func__);
		goto done;
	}
done:
	return rc;
}

static void ap3223_unregister_heartbeat_device(struct i2c_client *client,
					       struct ap3223_data *data)
{
	input_unregister_device(data->hsensor_input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int ap3223_psensor_enable(struct i2c_client *client)
{
	int ret = 0, mode;

	mode = ap3223_get_mode(client);
	if ((mode & AP3223_SYS_PS_ENABLE) == 0) {
		mode |= AP3223_SYS_PS_ENABLE;
		ret = ap3223_set_mode(client, mode);
	}

	return ret;
}

static int ap3223_psensor_disable(struct i2c_client *client)
{
	int ret = 0, mode;

	mode = ap3223_get_mode(client);
	if (mode & AP3223_SYS_PS_ENABLE) {
		mode &= ~AP3223_SYS_PS_ENABLE;
		if (mode == AP3223_SYS_DEV_RESET)
			mode = AP3223_SYS_DEV_DOWN;
		ret = ap3223_set_mode(client, mode);
	}
	return ret;
}
#endif


static int ap3223_register_psensor_device(struct i2c_client *client,
					  struct ap3223_data *data)
{
	struct input_dev *input_dev;
	int rc;

	LDBG("allocating input device psensor\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev,
			"%s: could not allocate input device for psensor\n",
			__func__);
		rc = -ENOMEM;
		goto done;
	}
	data->psensor_input_dev = input_dev;
	input_set_drvdata(input_dev, data);
	input_dev->name = "proximity";
	input_dev->dev.parent = &client->dev;
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	rc = input_register_device(input_dev);
	if (rc < 0) {
		pr_err("%s: could not register input device for psensor\n",
		       __func__);
		goto done;
	}

	return 0;

done:
	return rc;
}

static void ap3223_unregister_psensor_device(struct i2c_client *client,
					     struct ap3223_data *data)
{
	input_unregister_device(data->psensor_input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend ap3223_early_suspend;
static void ap3223_suspend(struct early_suspend *h)
{

	if (misc_ps_opened)
		ap3223_psensor_disable(ap3223_data_g->client);
	if (misc_ls_opened)
		ap3223_lsensor_disable(ap3223_data_g->client);
}

static void ap3223_resume(struct early_suspend *h)
{

	if (misc_ls_opened)
		ap3223_lsensor_enable(ap3223_data_g->client);
	if (misc_ps_opened)
		ap3223_psensor_enable(ap3223_data_g->client);
}
#endif


/* range */
static ssize_t ap3223_show_range(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct ap3223_data *data = ap3223_data_g;
	return sprintf(buf, "%i\n", ap3223_get_range(data->client));
}

static ssize_t ap3223_store_range(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 3)
		return -EINVAL;

	ret = ap3223_set_range(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}




static ssize_t ap3223_store_ir_data(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	ret = ap3223_set_ir_data(data->client, val);

	if (ret < 0)
		return ret;
#if POLLING_MODE
	ret =
	    mod_timer(&data->pl_timer,
		      jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

	if (ret)
		LDBG("Timer Error\n");
#endif
	return count;
}

/* mode */
static ssize_t ap3223_show_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ap3223_data *data = ap3223_data_g;
	return sprintf(buf, "%d\n", ap3223_get_mode(data->client));
}

static ssize_t ap3223_store_mode(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	ret = ap3223_set_mode(data->client, val);

	if (ret < 0)
		return ret;
#if POLLING_MODE
	LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies);
	ret =
	    mod_timer(&data->pl_timer,
		      jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

	if (ret)
		LDBG("Timer Error\n");
#endif
	return count;
}


static ssize_t ap3223_ls_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long mode;
	int ret;

	LDBG("mode = %s,%s\n", __func__, buf);
	ret = kstrtoul(buf, 10, &mode);
	if (ret)
		return ret;

	mutex_lock(&ap3223_ls_lock);
	if ((mode == AP3223_SYS_ALS_ENABLE)
	    && ap3223_get_mode(data->client) != AP3223_SYS_ALS_ENABLE) {
		ap3223_set_althres(data->client, 1000);
		ap3223_set_ahthres(data->client, 2000);
		misc_ls_opened = 1;
		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
					 AP3223_REG_SYS_CONF_MASK,
					 AP3223_REG_SYS_CONF_SHIFT,
					 AP3223_SYS_ALS_ENABLE);
		if (ret < 0)
			return ret;
	} else {
		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
					 AP3223_REG_SYS_CONF_MASK,
					 AP3223_REG_SYS_CONF_SHIFT,
					 AP3223_SYS_DEV_RESET);
		ret =
		    __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
				       AP3223_REG_SYS_CONF_MASK,
				       AP3223_REG_SYS_CONF_SHIFT,
				       AP3223_SYS_DEV_DOWN);
	}
	mutex_unlock(&ap3223_ls_lock);
#if POLLING_MODE
	LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies);
	ret =
	    mod_timer(&data->pl_timer,
		      jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

	if (ret)
		LDBG("Timer Error\n");
#endif
	return count;
}


static ssize_t ap3223_ps_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long mode;
	int ret;

	LDBG("mode = %s,%s\n", __func__, buf);
	ret = kstrtoul(buf, 10, &mode);
	if (ret)
		return ret;

	mutex_lock(&ap3223_ps_lock);
	if ((mode == AP3223_SYS_PS_ENABLE)
	    && ap3223_get_mode(data->client) != AP3223_SYS_PS_ENABLE) {
		ret = ap3223_set_plthres(data->client, 100);
		ret = ap3223_set_phthres(data->client, 500);
		misc_ps_opened = 1;
		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
					 AP3223_REG_SYS_CONF_MASK,
					 AP3223_REG_SYS_CONF_SHIFT,
					 AP3223_SYS_PS_ENABLE);
		if (ret < 0)
			return ret;


	} else {
		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
					 AP3223_REG_SYS_CONF_MASK,
					 AP3223_REG_SYS_CONF_SHIFT,
					 AP3223_SYS_DEV_RESET);
		ret =
		    __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
				       AP3223_REG_SYS_CONF_MASK,
				       AP3223_REG_SYS_CONF_SHIFT,
				       AP3223_SYS_DEV_DOWN);
	}
	mutex_unlock(&ap3223_ps_lock);
#if POLLING_MODE
	LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies);
	ret =
	    mod_timer(&data->pl_timer,
		      jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

	if (ret)
		LDBG("Timer Error\n");
#endif
	return count;
}

static ssize_t ap3223_hs_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long mode;
	int ret;

	LDBG("mode = %s,%s\n", __func__, buf);
	ret = kstrtoul(buf, 10, &mode);
	if (ret)
		return ret;

	mutex_lock(&ap3223_heartbeat_lock);


	if (mode == 9) {
		data->hsensor_enable = 1;
		ret = __ap3223_write_reg(data->client, AP3223_REG_PS_CONF,
					 AP3223_REG_PS_CONF_MASK,
					 AP3223_REG_PS_CONF_SHIFT, 0);
		ret =
		    __ap3223_write_reg(data->client, AP3223_REG_PS_DC_1,
				       AP3223_REG_PS_DC_1_MASK,
				       AP3223_REG_PS_DC_1_SHIFT, 0);
		ret =
		    __ap3223_write_reg(data->client, AP3223_REG_PS_DC_2,
				       AP3223_REG_PS_DC_2_MASK,
				       AP3223_REG_PS_DC_2_SHIFT, 0);
		ret =
		    __ap3223_write_reg(data->client, AP3223_REG_PS_LEDD,
				       AP3223_REG_PS_LEDD_MASK,
				       AP3223_REG_PS_LEDD_SHIFT, 1);
		ret =
		    __ap3223_write_reg(data->client, AP3223_REG_PS_MEAN,
				       AP3223_REG_PS_MEAN_MASK,
				       AP3223_REG_PS_MEAN_SHIFT, 0);
		ret =
		    __ap3223_write_reg(data->client, AP3223_REG_PS_PERSIS,
				       AP3223_REG_PS_PERSIS_MASK,
				       AP3223_REG_PS_PERSIS_SHIFT, 0);
		ret = ap3223_set_plthres(data->client, 0);
		ret = ap3223_set_phthres(data->client, 535);
		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
					 AP3223_REG_SYS_CONF_MASK,
					 AP3223_REG_SYS_CONF_SHIFT,
					 AP3223_SYS_PS_ENABLE);

		if (ret < 0)
			return ret;
	} else {
		data->hsensor_enable = 0;
		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
					 AP3223_REG_SYS_CONF_MASK,
					 AP3223_REG_SYS_CONF_SHIFT,
					 AP3223_SYS_DEV_RESET);
		ret =
		    __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
				       AP3223_REG_SYS_CONF_MASK,
				       AP3223_REG_SYS_CONF_SHIFT,
				       AP3223_SYS_DEV_DOWN);
	}
	mutex_unlock(&ap3223_heartbeat_lock);
#if POLLING_MODE
	LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies);
	ret =
	    mod_timer(&data->pl_timer,
		      jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

	if (ret)
		LDBG("Timer Error\n");
#endif
	return count;
}

/* lux */
static ssize_t ap3223_show_lux(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct ap3223_data *data = ap3223_data_g;

	/* No LUX data if power down */
	if (ap3223_get_mode(data->client) == AP3223_SYS_DEV_DOWN)
		return sprintf((char *) buf, "%s\n",
			       "Please power up first!");

	return sprintf(buf, "%d\n", ap3223_get_adc_value(data->client));
}



/* Px data */
static ssize_t ap3223_show_pxvalue(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct ap3223_data *data = ap3223_data_g;

	/* No Px data if power down */
	if (ap3223_get_mode(data->client) == AP3223_SYS_DEV_DOWN)
		return -EBUSY;

	return sprintf(buf, "%d\n", ap3223_get_px_value(data->client));
}



/* proximity object detect */
static ssize_t ap3223_show_object(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ap3223_data *data = ap3223_data_g;
	return sprintf(buf, "%d\n", ap3223_get_object(data->client));
}



/* ALS low threshold */
static ssize_t ap3223_show_althres(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct ap3223_data *data = ap3223_data_g;
	return sprintf(buf, "%d\n", ap3223_get_althres(data->client));
}

static ssize_t ap3223_store_althres(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	ret = ap3223_set_althres(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}



/* ALS high threshold */
static ssize_t ap3223_show_ahthres(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct ap3223_data *data = ap3223_data_g;
	return sprintf(buf, "%d\n", ap3223_get_ahthres(data->client));
}

static ssize_t ap3223_store_ahthres(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	ret = ap3223_set_ahthres(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}


/* Px low threshold */
static ssize_t ap3223_show_plthres(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct ap3223_data *data = ap3223_data_g;
	return sprintf(buf, "%d\n", ap3223_get_plthres(data->client));
}

static ssize_t ap3223_store_plthres(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	ret = ap3223_set_plthres(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}


/* Px high threshold */
static ssize_t ap3223_show_phthres(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct ap3223_data *data = ap3223_data_g;
	return sprintf(buf, "%d\n", ap3223_get_phthres(data->client));
}

static ssize_t ap3223_store_phthres(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	ret = ap3223_set_phthres(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}

/* Px calibration */
static ssize_t ap3223_show_pcali(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct ap3223_data *data = ap3223_data_g;
	return sprintf(buf, "%d\n", ap3223_get_pcali(data->client));
}

static ssize_t ap3223_store_pcali(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	ret = ap3223_set_pcali(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}



/* calibration */
static ssize_t ap3223_show_calibration_state(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	return sprintf(buf, "%d\n", cali);
}

static ssize_t ap3223_store_calibration_state(struct device *dev,
					      struct device_attribute
					      *attr, const char *buf,
					      size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	int stdls, lux;
	char tmp[10];

	LDBG("DEBUG ap3223_store_calibration_state..\n");

	/* No LUX data if not operational */
	if (ap3223_get_mode(data->client) == AP3223_SYS_DEV_DOWN) {
		pr_err("Please power up first!");
		return -EINVAL;
	}

	cali = 100;
	sscanf(buf, "%d %s", &stdls, tmp);

	if (!strncmp(tmp, "-setcv", 6)) {
		cali = stdls;
		return count;
	}

	if (stdls < 0) {
		pr_err("Std light source: [%d] < 0 !!!\n", stdls);
		pr_err("Check again, please.\n");
		pr_err("Set calibration factor to 100.\n");
		return -EBUSY;
	}

	lux = ap3223_get_adc_value(data->client);
	cali = stdls * 100 / lux;

	return count;
}


#ifdef LSC_DBG
/* engineer mode */
static ssize_t ap3223_em_read(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct ap3223_data *data = ap3223_data_g;
	int i;
	u8 tmp;

	LDBG("DEBUG ap3223_em_read..\n");

	for (i = 0; i < reg_num; i++) {
		tmp = i2c_smbus_read_byte_data(data->client, reg_array[i]);

		LDBG("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
	}

	return 0;
}

static ssize_t ap3223_em_write(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct ap3223_data *data = ap3223_data_g;
	u32 addr, val, idx = 0;
	int ret = 0;

	LDBG("DEBUG ap3223_em_write..\n");

	sscanf(buf, "%x%x", &addr, &val);

	pr_warn("Write [%x] to Reg[%x]...\n", val, addr);

	ret = i2c_smbus_write_byte_data(data->client, addr, val);
	ADD_TO_IDX(addr, idx)
	    if (!ret)
		data->reg_cache[idx] = val;

	return count;
}
#endif


static struct device_attribute attributes[] = {
	__ATTR(range, S_IWUSR | S_IRUGO, ap3223_show_range,
	       ap3223_store_range),
	__ATTR(mode, 0666, ap3223_show_mode, ap3223_store_mode),
	__ATTR(lsensor, 0666, ap3223_show_mode, ap3223_ls_enable),
	__ATTR(psensor, 0666, ap3223_show_mode, ap3223_ps_enable),
	__ATTR(hsensor, 0666, ap3223_show_mode, ap3223_hs_enable),
	__ATTR(lux, S_IRUGO, ap3223_show_lux, NULL),
	__ATTR(pxvalue, S_IRUGO, ap3223_show_pxvalue, NULL),
	__ATTR(object, S_IRUGO, ap3223_show_object, NULL),
	__ATTR(althres, S_IWUSR | S_IRUGO, ap3223_show_althres,
	       ap3223_store_althres),
	__ATTR(ahthres, S_IWUSR | S_IRUGO, ap3223_show_ahthres,
	       ap3223_store_ahthres),
	__ATTR(plthres, S_IWUSR | S_IRUGO, ap3223_show_plthres,
	       ap3223_store_plthres),
	__ATTR(phthres, S_IWUSR | S_IRUGO, ap3223_show_phthres,
	       ap3223_store_phthres),
	__ATTR(pcalibration, S_IWUSR | S_IRUGO, ap3223_show_pcali,
	       ap3223_store_pcali),
	__ATTR(calibration, S_IWUSR | S_IRUGO,
	       ap3223_show_calibration_state,
	       ap3223_store_calibration_state),
	__ATTR(ir_data, S_IWUGO, NULL, ap3223_store_ir_data),
#ifdef LSC_DBG
	__ATTR(em, S_IWUSR | S_IRUGO, ap3223_em_read, ap3223_em_write),
#endif

};

static int create_sysfs_interfaces(struct ap3223_data *sensor)
{
	int i;
	struct class *ap3223_class = NULL;
	struct device *ap3223_dev = NULL;
	int ret;

	ap3223_class = class_create(THIS_MODULE, "sensors");
	if (IS_ERR(ap3223_class)) {
		ret = PTR_ERR(ap3223_class);
		ap3223_class = NULL;
		LDBG("%s: could not allocate ap3223_class, ret = %d\n",
		     __func__, ret);
		goto ap3223_class_error;
	}

	ap3223_dev = device_create(ap3223_class,
				   NULL, 0, "%s", "di_sensors");

	if (ap3223_dev == NULL)
		goto ap3223_device_error;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(ap3223_dev, attributes + i))
			goto ap3223_create_file_error;

	return 0;

ap3223_create_file_error:
	for (; i >= 0; i--)
		device_remove_file(ap3223_dev, attributes + i);

ap3223_device_error:
	class_destroy(ap3223_class);
ap3223_class_error:
	dev_err(&sensor->client->dev, "%s:Unable to create interface\n",
		__func__);
	return -1;
}

static int ap3223_init_client(struct i2c_client *client)
{
	struct ap3223_data *data = i2c_get_clientdata(client);
	int i;

	LDBG("DEBUG ap3223_init_client..\n");

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < reg_num; i++) {
		int v = i2c_smbus_read_byte_data(client, reg_array[i]);
		if (v < 0)
			return -ENODEV;

		data->reg_cache[i] = v;
	}
	/* set defaults */

	ap3223_set_range(client, AP3223_ALS_RANGE_0);
	ap3223_set_mode(data->client, AP3223_SYS_DEV_DOWN);

	return 0;
}

#if POLLING_MODE
static void pl_timer_callback(unsigned long pl_data)
{
	struct ap3223_data *data;
	int ret = 0;

	data = ap3223_data_g;
	queue_work(data->plsensor_wq, &data->plsensor_work);

	ret =
	    mod_timer(&ap3223_data_g->pl_timer,
		      jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

	if (ret)
		LDBG("Timer Error\n");

}
#endif
static void plsensor_work_handler(struct work_struct *w)
{

	struct ap3223_data *data =
	    container_of(w, struct ap3223_data, plsensor_work);
	u8 int_stat;
	int pxvalue;
	int obj;
	int ret;
	int value;

	int_stat = ap3223_get_intstat(data->client);


	/* ALS int */
	if (int_stat & AP3223_REG_SYS_INT_AMASK) {
		/* LDBG("LS INT Status: %0x\n", int_stat); */

		value = ap3223_get_adc_value(data->client);
		ret =
		    __ap3223_write_reg(data->client,
				       AP3223_REG_SYS_INTSTATUS,
				       AP3223_REG_SYS_INT_AMASK,
				       AP3223_REG_SYS_INT_LS_SHIFT, 0);
		input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
		input_sync(data->lsensor_input_dev);
	}
	/* PX int */
	if (int_stat & AP3223_REG_SYS_INT_PMASK) {
		/* LDBG("PS INT Status: %0x\n", int_stat); */
		obj = ap3223_get_object(data->client);
		pxvalue = ap3223_get_px_value(data->client);

		ret =
		    __ap3223_write_reg(data->client,
				       AP3223_REG_SYS_INTSTATUS,
				       AP3223_REG_SYS_INT_PMASK,
				       AP3223_REG_SYS_INT_PS_SHIFT, 0);
		LDBG("%s\n", obj ? "obj near" : "obj far");
		input_report_abs(data->psensor_input_dev, ABS_DISTANCE,
				 obj);
		input_sync(data->psensor_input_dev);

		if (data->hsensor_enable) {
			LDBG("pxvalue = %d\n", pxvalue);
			input_report_rel(data->hsensor_input_dev,
					 ABS_WHEEL, pxvalue);
			input_sync(data->hsensor_input_dev);
		}

	}

	enable_irq(data->client->irq);
}

/*
 * I2C layer
 */

static irqreturn_t ap3223_irq(int irq, void *data_)
{
	struct ap3223_data *data = data_;

	/* LDBG("interrupt\n"); */
	disable_irq_nosync(data->client->irq);
	queue_work(data->plsensor_wq, &data->plsensor_work);

	return IRQ_HANDLED;
}

static int ap3223_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ap3223_data *data;
	int err = 0;
#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
	u16 int_gpio;
#endif

	LDBG("ap3223_probe\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit_free_gpio;
	}
#ifdef CONFIG_OF
	if (np) {
		int_gpio = of_get_named_gpio(np, "int-gpio", 0);
		if (int_gpio)
			client->irq = gpio_to_irq(int_gpio);
	}
#endif

	reg_array = ap3223_reg;
	range = ap3223_range;
	reg_num = AP3223_NUM_CACHABLE_REGS;

	data = kzalloc(sizeof(struct ap3223_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit_free_gpio;
	}

	data->client = client;
	i2c_set_clientdata(client, data);
	data->irq = client->irq;

	/* initialize the AP3223 chip */
	err = ap3223_init_client(client);
	if (err)
		goto exit_kfree;

	err = ap3223_register_lsensor_device(client, data);
	if (err) {
		dev_err(&client->dev,
			"failed to register_lsensor_device\n");
		goto exit_kfree;
	}

	err = ap3223_register_psensor_device(client, data);
	if (err) {
		dev_err(&client->dev,
			"failed to register_psensor_device\n");
		goto exit_free_ls_device;
	}

	err = ap3223_register_heartbeat_sensor_device(client, data);
	if (err) {
		dev_err(&client->dev,
			"failed to register_heartbeatsensor_device\n");
		goto exit_free_heartbeats_device;
	}

	err = create_sysfs_interfaces(data);
	if (err)
		goto exit_free_ps_device;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ap3223_early_suspend.suspend = ap3223_suspend;
	ap3223_early_suspend.resume = ap3223_resume;
	ap3223_early_suspend.level = 0x02;
	register_early_suspend(&ap3223_early_suspend);
#endif

	err = request_threaded_irq(client->irq, NULL, ap3223_irq,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW
				   | IRQF_ONESHOT, "ap3223", data);
	if (err) {
		dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",
			err, client->irq);
		goto exit_free_ps_device;
	}

	data->plsensor_wq = create_singlethread_workqueue("plsensor_wq");
	if (!data->plsensor_wq) {
		LDBG("%s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&data->plsensor_work, plsensor_work_handler);

#if POLLING_MODE
	LDBG("Timer module installing\n");
	setup_timer(&data->pl_timer, pl_timer_callback, 0);
#endif


	ap3223_data_g = data;
#ifdef INIT_ARKHAM
/* arkham initial value */
	{
		int i;
		for (i = 0; i < sizeof(init_reg); i += 2)
			err =
			    __ap3223_write_reg(client, init_reg[i], 0xff,
					       0, init_reg[i + 1]);
	}
#endif

	dev_info(&client->dev, "Driver version %s enabled\n",
		 DRIVER_VERSION);
	return 0;
err_create_wq_failed:
#if POLLING_MODE
	if (&data->pl_timer != NULL)
		del_timer(&data->pl_timer);
#endif
	if (data->plsensor_wq)
		destroy_workqueue(data->plsensor_wq);
exit_free_ps_device:
	ap3223_unregister_psensor_device(client, data);

exit_free_heartbeats_device:
	ap3223_unregister_heartbeat_device(client, data);

exit_free_ls_device:
	ap3223_unregister_lsensor_device(client, data);

exit_kfree:
	kfree(data);

exit_free_gpio:
	return err;
}

static int ap3223_remove(struct i2c_client *client)
{
	struct ap3223_data *data = i2c_get_clientdata(client);
	free_irq(data->irq, data);

	ap3223_unregister_psensor_device(client, data);
	ap3223_unregister_lsensor_device(client, data);
	ap3223_unregister_heartbeat_device(client, data);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ap3223_early_suspend);
#endif

	ap3223_set_mode(data->client, 0);
	kfree(i2c_get_clientdata(client));

	if (data->plsensor_wq)
		destroy_workqueue(data->plsensor_wq);
#if POLLING_MODE
	if (&data->pl_timer)
		del_timer(&data->pl_timer);
#endif
	return 0;
}

static const struct i2c_device_id ap3223_id[] = {
	{AP3223_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ap3223_id);

static struct i2c_driver ap3223_driver = {
	.driver = {
		   .name = AP3223_DRV_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = ap3223_probe,
	.remove = ap3223_remove,
	.id_table = ap3223_id,
};

static int __init ap3223_init(void)
{
	int ret;

	LDBG("ap3223_init\n");
	reg_num = 0;
	cali = 100;
	misc_ps_opened = 0;
	misc_ls_opened = 0;
	ap3223_data_g = NULL;
	ret = i2c_add_driver(&ap3223_driver);
	return ret;

}

static void __exit ap3223_exit(void)
{
	i2c_del_driver(&ap3223_driver);
}

MODULE_AUTHOR("Templeton Tsai Dyna-Image Corporation.");
MODULE_DESCRIPTION("AP3223 driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(ap3223_init);
module_exit(ap3223_exit);
