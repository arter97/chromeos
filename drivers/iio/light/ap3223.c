/*
 * Copyright (C) 2015 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Note about the original authors:
 *
 * This driver is based on the original driver for AP3223 that was distributed
 * by Dyna Image. That driver was using input framework. The driver from Dyna
 * image is not available online anywhere, so there is no reference for it here.
 * However, that driver is also GPLv2.
 * The following is part of the header found in that file
 * (The GPL notice from the original header is removed)
 *
 * >> This file is part of the AP3223, AP3212C and AP3216C sensor driver.
 * >> AP3426 is combined proximity and ambient light sensor.
 * >> AP3216C is combined proximity, ambient light sensor and IRLED.
 * >>
 * >> Contact: John Huang <john.huang@dyna-image.com>
 * >>	       Templeton Tsai <templeton.tsai@dyna-image.com>
 *
 * Another author initials mentioned in that file was just YC (and no name).
 *
 * Not sure for what kernel version the driver from dyna image was written for.
 * Vic Lee <Vic_Lee@asus.com> made modifications to it to run on 3.14.
 *
 * Datasheet:
 * http://www.dyna-image.com/english/product/optical-sensor-detail.php?cpid=2&dpid=8#doc
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/device.h>

#include "ap3223.h"

#define AP3223_DRV_NAME "ap3223"

#define AP3223_GAIN_65535_LUX_PER_RES_BIT	65535
#define AP3223_GAIN_16383_LUX_PER_RES_BIT	16383
#define AP3223_GAIN_4095_LUX_PER_RES_BIT	4095
#define AP3223_GAIN_1023_LUX_PER_RES_BIT	1023

/* Initial Threshold Configuration */
#define AP3223_ALS_LOW_THD_L_INIT		0xE8
#define AP3223_ALS_LOW_THD_H_INIT		0x03
#define AP3223_ALS_HIGH_THD_L_INIT		0xD0
#define AP3223_ALS_HIGH_THD_H_INIT		0x07

#define AP3223_PS_LOW_THD_L_INIT		0x64
#define AP3223_PS_LOW_THD_H_INIT		0x00
#define AP3223_PS_HIGH_THD_L_INIT		0xF4
#define AP3223_PS_HIGH_THD_H_INIT		0x00

struct ap3223_data {
	struct mutex ap3223_mutex;
	struct i2c_client *client;
	struct regmap *regmap;
	int ps_calib;
	bool event_enabled;
};

static const u8 ap3223_initial_reg_conf[] = {
	AP3223_REG_SYS_CTRL, AP3223_SYS_ALS_PS_ENABLE,
	AP3223_REG_SYS_INTCTRL, (AP3223_SYS_INT_PINT_EN |
				 AP3223_SYS_INT_CLEAR_AUTO),
	AP3223_REG_SYS_WAITTIME, AP3223_WAITTIME_SLOT(0),
	AP3223_REG_ALS_GAIN, (AP3223_ALS_RANGE_2 << AP3223_ALS_RANGE_SHIFT),
	AP3223_REG_ALS_THDL_L, AP3223_ALS_LOW_THD_L_INIT,
	AP3223_REG_ALS_THDL_H, AP3223_ALS_LOW_THD_H_INIT,
	AP3223_REG_ALS_THDH_L, AP3223_ALS_HIGH_THD_L_INIT,
	AP3223_REG_ALS_THDH_H, AP3223_ALS_HIGH_THD_H_INIT,
	AP3223_REG_PS_GAIN, (AP3223_PS_GAIN_8 << AP3223_REG_PS_GAIN_SHIFT),
	AP3223_REG_PS_LEDD, AP3223_PS_LED_DRVR_CUR_P_100,
	AP3223_REG_PS_IFORM, AP3223_PS_INT_ALSO_HYST_MODE,
	AP3223_REG_PS_MEAN, AP3223_PS_MEAN_0,
	AP3223_REG_PS_SMARTINT, AP3223_PS_SMARTINT_ENABLE,
	AP3223_REG_PS_INTEGR_TIME, AP3223_PS_INTEGR_TIME_SEL(0x1F),
	AP3223_REG_PS_PERSIST, AP3223_PS_PERSIST_CONV_TIME(0x01),
	AP3223_REG_PS_THDL_L, AP3223_PS_LOW_THD_L_INIT,
	AP3223_REG_PS_THDL_H, AP3223_PS_LOW_THD_H_INIT,
	AP3223_REG_PS_THDH_L, AP3223_PS_HIGH_THD_L_INIT,
	AP3223_REG_PS_THDH_H, AP3223_PS_HIGH_THD_H_INIT
};

static const int ap3223_range[] = {
	AP3223_GAIN_65535_LUX_PER_RES_BIT,
	AP3223_GAIN_16383_LUX_PER_RES_BIT,
	AP3223_GAIN_4095_LUX_PER_RES_BIT,
	AP3223_GAIN_1023_LUX_PER_RES_BIT
};

static bool ap3223_is_writable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AP3223_REG_SYS_CTRL:
	case AP3223_REG_SYS_INTSTATUS:
	case AP3223_REG_SYS_INTCTRL:
	case AP3223_REG_SYS_WAITTIME:
	case AP3223_REG_ALS_GAIN:
	case AP3223_REG_ALS_PERSIST:
	case AP3223_REG_ALS_THDL_L:
	case AP3223_REG_ALS_THDL_H:
	case AP3223_REG_ALS_THDH_L:
	case AP3223_REG_ALS_THDH_H:
	case AP3223_REG_PS_GAIN:
	case AP3223_REG_PS_LEDD:
	case AP3223_REG_PS_IFORM:
	case AP3223_REG_PS_MEAN:
	case AP3223_REG_PS_SMARTINT:
	case AP3223_REG_PS_INTEGR_TIME:
	case AP3223_REG_PS_PERSIST:
	case AP3223_REG_PS_CAL_L:
	case AP3223_REG_PS_CAL_H:
	case AP3223_REG_PS_THDL_L:
	case AP3223_REG_PS_THDL_H:
	case AP3223_REG_PS_THDH_L:
	case AP3223_REG_PS_THDH_H:
		return true;

	default:
		return false;
	}
}

static bool ap3223_is_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AP3223_REG_SYS_CTRL:
	case AP3223_REG_SYS_INTSTATUS:
	case AP3223_REG_SYS_INTCTRL:
	case AP3223_REG_SYS_WAITTIME:
	case AP3223_REG_IR_DATA_LOW:
	case AP3223_REG_IR_DATA_HIGH:
	case AP3223_REG_ALS_DATA_LOW:
	case AP3223_REG_ALS_DATA_HIGH:
	case AP3223_REG_PS_DATA_LOW:
	case AP3223_REG_PS_DATA_HIGH:
	case AP3223_REG_ALS_GAIN:
	case AP3223_REG_ALS_PERSIST:
	case AP3223_REG_ALS_THDL_L:
	case AP3223_REG_ALS_THDL_H:
	case AP3223_REG_ALS_THDH_L:
	case AP3223_REG_ALS_THDH_H:
	case AP3223_REG_PS_GAIN:
	case AP3223_REG_PS_LEDD:
	case AP3223_REG_PS_IFORM:
	case AP3223_REG_PS_MEAN:
	case AP3223_REG_PS_SMARTINT:
	case AP3223_REG_PS_INTEGR_TIME:
	case AP3223_REG_PS_PERSIST:
	case AP3223_REG_PS_CAL_L:
	case AP3223_REG_PS_CAL_H:
	case AP3223_REG_PS_THDL_L:
	case AP3223_REG_PS_THDL_H:
	case AP3223_REG_PS_THDH_L:
	case AP3223_REG_PS_THDH_H:
		return true;

	default:
		return false;
	}
}

static bool ap3223_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AP3223_REG_SYS_CTRL:
	case AP3223_REG_SYS_INTSTATUS:
	case AP3223_REG_IR_DATA_LOW:
	case AP3223_REG_IR_DATA_HIGH:
	case AP3223_REG_ALS_DATA_LOW:
	case AP3223_REG_ALS_DATA_HIGH:
	case AP3223_REG_PS_DATA_LOW:
	case AP3223_REG_PS_DATA_HIGH:
		return true;

	default:
		return false;
	}
}

static const struct regmap_config ap3223_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AP3223_REG_PS_THDH_H,
	.cache_type = REGCACHE_RBTREE,

	.writeable_reg = ap3223_is_writable_reg,
	.readable_reg = ap3223_is_readable_reg,
	.volatile_reg = ap3223_is_volatile_reg,
};

static const struct iio_event_spec ap3223_prox_event_spec[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				BIT(IIO_EV_INFO_ENABLE),
	}
};

#define AP3223_LIGHT_CHANNEL {				\
	.type = IIO_LIGHT,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
}

#define AP3223_PROXIMITY_CHANNEL {			\
	.type = IIO_PROXIMITY,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
			BIT(IIO_CHAN_INFO_CALIBBIAS),	\
	.event_spec = ap3223_prox_event_spec,           \
	.num_event_specs = ARRAY_SIZE(ap3223_prox_event_spec),\
}

static const struct iio_chan_spec ap3223_channels[] = {
	AP3223_LIGHT_CHANNEL,
	AP3223_PROXIMITY_CHANNEL,
};

static int ap3223_read_reg(struct i2c_client *client,
			   u32 reg, u8 mask, u8 shift)
{
	struct ap3223_data *data = iio_priv(i2c_get_clientdata(client));
	unsigned int tmp;

	if (regmap_read(data->regmap, reg, &tmp) < 0)
		return -EINVAL;

	return (tmp & mask) >> shift;
}

static int ap3223_write_reg(struct i2c_client *client,
			    u32 reg, u8 mask, u8 shift, u8 val)
{
	struct ap3223_data *data = iio_priv(i2c_get_clientdata(client));
	int ret = 0;
	unsigned int tmp;

	if (regmap_read(data->regmap, reg, &tmp) < 0)
		return -EINVAL;

	tmp &= ~mask;
	tmp |= val << shift;

	ret = regmap_write(data->regmap, reg, tmp);
	if (ret)
		return -EINVAL;

	return ret;
}

static int ap3223_get_mode(struct i2c_client *client)
{
	return ap3223_read_reg(client, AP3223_REG_SYS_CTRL,
			       AP3223_REG_SYS_CTRL_MASK,
			       AP3223_REG_SYS_CTRL_SHIFT);
}

static int ap3223_set_mode(struct i2c_client *client, int mode)
{
	int err = 0;

	err = ap3223_write_reg(client, AP3223_REG_SYS_CTRL,
			       AP3223_REG_SYS_CTRL_MASK,
			       AP3223_REG_SYS_CTRL_SHIFT, mode);
	if (err < 0)
		dev_err(&client->dev, "Failed to set mode\n");

	return err;
}

static int ap3223_get_range(struct i2c_client *client)
{
	u8 idx = ap3223_read_reg(client, AP3223_REG_ALS_GAIN,
				 AP3223_ALS_RANGE_MASK,
				 AP3223_ALS_RANGE_SHIFT);
	if (idx < 0)
		return -EINVAL;

	return ap3223_range[idx];
}

static int ap3223_set_range(struct i2c_client *client, int range)
{
	int err = 0;

	err = ap3223_write_reg(client, AP3223_REG_ALS_GAIN,
			       AP3223_ALS_RANGE_MASK,
			       AP3223_ALS_RANGE_SHIFT, range);
	if (err < 0)
		dev_err(&client->dev, "Failed to set range\n");

	return err;
}

static int ap3223_get_adc_value(struct i2c_client *client)
{
	int range;
	unsigned int lsb, msb;
	unsigned int tmp;
	struct ap3223_data *data = iio_priv(i2c_get_clientdata(client));

	range = ap3223_get_range(client);
	if (range < 0)
		return range;

	if (regmap_read(data->regmap, AP3223_REG_ALS_DATA_LOW, &lsb) < 0)
		return -EINVAL;

	if (regmap_read(data->regmap, AP3223_REG_ALS_DATA_HIGH, &msb) < 0)
		return -EINVAL;

	tmp = (((msb << 8) | lsb) * range) >> 16;

	return tmp;
}

static int ap3223_get_object(struct i2c_client *client)
{
	return ap3223_read_reg(client, AP3223_REG_SYS_INTSTATUS,
			      AP3223_REG_SYS_INT_OBJ_MASK,
			      AP3223_REG_SYS_INT_OBJ_SHIFT);
}

static int ap3223_clear_irq_flag(struct i2c_client *client)
{
	/*
	 * If AP3223_REG_SYS_INTCTRL:CLR_MNR is set to
	 * AP3223_SYS_INT_CLEAR_AUTO(0), the interrupt flag is automatically
	 * cleared by reading the following data
	 * registers;
	 *
	 * AP3223_REG_ALS_DATA_LOW & AP3223_REG_ALS_DATA_HIGH - for ALS
	 * AP3223_REG_PS_DATA_LOW & AP3223_REG_PS_DATA_HIGH - for PS
	 *
	 * If AP3223_REG_SYS_INTCTRL:CLR_MNR is set to
	 * AP3223_SYS_INT_CLEAR_MANUAL(1), the interrupt flag is cleared
	 * manually. i.e,
	 *
	 * If PS_INT is asserted, it can be cleared after I2C writes to
	 * AP3223_REG_SYS_INTSTATUS with AP3223_REG_SYS_INT_PMASK
	 *
	 * If ALS_INT is asserted, it can be cleared after I2C writes to
	 * AP3223_REG_SYS_INTSTATUS with AP3223_REG_SYS_INT_AMASK
	 */

	struct ap3223_data *data = iio_priv(i2c_get_clientdata(client));
	unsigned int lsb, msb;

	if (regmap_read(data->regmap, AP3223_REG_PS_DATA_LOW, &lsb) < 0)
		return -EINVAL;

	if (regmap_read(data->regmap, AP3223_REG_PS_DATA_HIGH, &msb) < 0)
		return -EINVAL;

	return 0;
}

static int ap3223_sw_reset(struct i2c_client *client)
{
	struct ap3223_data *data = iio_priv(i2c_get_clientdata(client));
	int err = 0;
	int retry = 15;

	for (; retry; retry--) {
		err = regmap_write(data->regmap, AP3223_REG_SYS_CTRL,
				   AP3223_SYS_DEV_RESET);

		if (err >= 0)
			break;

		usleep_range(1000, 2000);
	}

	if (err < 0)
		dev_err(&client->dev, "SW Reset: Failed to set mode\n");

	return err;
}

static int ap3223_init_client(struct i2c_client *client)
{
	struct ap3223_data *data = iio_priv(i2c_get_clientdata(client));

	if (ap3223_set_range(client, AP3223_ALS_RANGE_0) < 0)
		return -EINVAL;

	if (ap3223_set_mode(data->client, AP3223_SYS_DEV_DOWN) < 0)
		return -EINVAL;

	return 0;
}

static int ap3223_set_ps_calib(struct i2c_client *client, int val)
{
	int err;

	err = ap3223_write_reg(client, AP3223_REG_PS_CAL_L,
			       AP3223_REG_PS_CAL_L_MASK,
			       AP3223_REG_PS_CAL_L_SHIFT, val);
	if (err < 0) {
		dev_err(&client->dev, "Failed to set PS Calibration(L)\n");
		return err;
	}

	err = ap3223_write_reg(client, AP3223_REG_PS_CAL_H,
			       AP3223_REG_PS_CAL_H_MASK,
			       AP3223_REG_PS_CAL_H_SHIFT, val >> 8);
	if (err < 0)
		dev_err(&client->dev, "Failed to set PS Calibration(H)\n");

	return err;
}

static int ap3223_set_ps_thres(struct i2c_client *client,
			       u8 address,
			       int val)
{
	int err = -EINVAL;
	/* Shitfs of all the threshold registers are same */
	u8 shift = AP3223_REG_PS_THDH_L_SHIFT;

	/*
	 * AP3223_REG_PS_THDL_L_MASK = AP3223_REG_PS_THDH_L_MASK and
	 * AP3223_REG_PS_THDL_H_MASK = AP3223_REG_PS_THDH_H_MASK
	 */

	err = ap3223_write_reg(client, address, AP3223_REG_PS_THDH_L_MASK,
			       shift, val);
	if (err < 0) {
		dev_err(&client->dev, "Failed to set PS Threshold (L)\n");
		goto out;
	}

	address++;

	err = ap3223_write_reg(client, address, AP3223_REG_PS_THDH_H_MASK,
			       shift, val >> 8);
	if (err < 0)
		dev_err(&client->dev, "Failed to set PS Threshold (H)\n");

out:
	return err;
}

static int ap3223_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct ap3223_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&data->ap3223_mutex);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = ap3223_get_mode(data->client);
			if (ret < 0)
				break;

			ret = ap3223_get_adc_value(data->client);
			if (ret < 0)
				break;

			*val = ret;
			ret = IIO_VAL_INT;
			break;

		case IIO_PROXIMITY:
			*val = ap3223_get_object(data->client);
			if (*val < 0)
				break;

			ret = IIO_VAL_INT;
			break;

		default:
			break;
		}
		break;

	case IIO_CHAN_INFO_CALIBBIAS:
		switch (chan->type) {
		case IIO_PROXIMITY:
			*val = data->ps_calib;
			ret = IIO_VAL_INT;

		default:
			break;
		}
		break;

	default:
		break;
	}

	mutex_unlock(&data->ap3223_mutex);

	return ret;
}

static int ap3223_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ap3223_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&data->ap3223_mutex);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBBIAS:
		switch (chan->type) {
		case IIO_PROXIMITY:
			ret = ap3223_set_ps_calib(data->client, val);
			if (ret >= 0)
				data->ps_calib = val;
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}

	mutex_unlock(&data->ap3223_mutex);

	return ret;
}

static int ap3223_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info, int *val,
				   int *val2)
{
	struct ap3223_data *data = iio_priv(indio_dev);
	int ret = IIO_VAL_INT;

	mutex_lock(&data->ap3223_mutex);

	*val = ap3223_get_object(data->client);
	if (*val < 0)
		ret = -EINVAL;

	mutex_unlock(&data->ap3223_mutex);

	return ret;
}

static int ap3223_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int val, int val2)
{
	struct ap3223_data *data = iio_priv(indio_dev);
	int ret = 0;
	u8 address = (dir == IIO_EV_DIR_RISING) ?
			AP3223_REG_PS_THDH_L : AP3223_REG_PS_THDL_L;

	mutex_lock(&data->ap3223_mutex);
	ret = ap3223_set_ps_thres(data->client, address, val);
	mutex_unlock(&data->ap3223_mutex);

	return ret;
}

static int ap3223_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct ap3223_data *data = iio_priv(indio_dev);
	return data->event_enabled;
}

static int ap3223_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir, int state)
{
	struct ap3223_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->ap3223_mutex);

	if (data->client->irq <= 0) {
		ret = -EINVAL;
		goto ap3223_write_event_exit;
	}

	if (state && data->event_enabled == false) {
		enable_irq(data->client->irq);
		data->event_enabled = true;
	} else if (!state && data->event_enabled == true) {
		disable_irq_nosync(data->client->irq);
		data->event_enabled = false;
	}

ap3223_write_event_exit:
	mutex_unlock(&data->ap3223_mutex);

	return ret;
}

static const struct iio_info ap3223_info = {
	.read_raw		= &ap3223_read_raw,
	.write_raw		= &ap3223_write_raw,
	.read_event_value	= &ap3223_read_event_value,
	.write_event_value	= &ap3223_write_event_value,
	.read_event_config	= &ap3223_read_event_config,
	.write_event_config	= &ap3223_write_event_config,
	.driver_module		= THIS_MODULE,
};

static irqreturn_t ap3223_irq_threaded_fn(int irq, void *ptr)
{
	struct ap3223_data *data = ptr;
	unsigned int int_stat;
	int ev_dir;
	u64 ev_code;
	int ret = -EIO;
	struct iio_dev *indio_dev = i2c_get_clientdata(data->client);

	mutex_lock(&data->ap3223_mutex);

	if (regmap_read(data->regmap, AP3223_REG_SYS_INTSTATUS, &int_stat) >= 0)
		if (ap3223_clear_irq_flag(data->client) >= 0)
			ret = 0;

	mutex_unlock(&data->ap3223_mutex);

	if (ret >= 0 && int_stat & AP3223_REG_SYS_INT_PMASK) {
		if (int_stat & AP3223_REG_SYS_INT_OBJ_MASK)
			ev_dir = IIO_EV_DIR_FALLING;
		else
			ev_dir = IIO_EV_DIR_RISING;

		ev_code = IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, 0,
					       IIO_EV_TYPE_THRESH, ev_dir);
		iio_push_event(indio_dev, ev_code, iio_get_time_ns());
	}

	return IRQ_HANDLED;
}

static int ap3223_init_reg_config(struct i2c_client *client)
{
	struct ap3223_data *data = iio_priv(i2c_get_clientdata(client));
	int i;

	for (i = 0; i < ARRAY_SIZE(ap3223_initial_reg_conf); i += 2) {
		/*
		 * Taking care of transient i2c failures during init.
		 * The delay outside the loop helps to avoid the write failures
		 * completely (never saw a single failure during test). However,
		 * the inner loop is retained to recover from the NACK errors
		 * that were observed before the introduction of the outer
		 * delay.
		 *
		 * The value of 1000us (for all the usleep_range used in this
		 * file), were arrived at by testing the device and verifying
		 * that there were no errors.
		 *
		 * No failure is observed with these delays in about 500+
		 * cycles of reboot and load and unload of modules (without
		 * any gap in between) over 15 hours.
		 */

		int retry = 15;

		usleep_range(1000, 2000);

		for (; retry; retry--) {
			if (regmap_write(data->regmap,
					 ap3223_initial_reg_conf[i],
					 ap3223_initial_reg_conf[i + 1]) >= 0)
				break;

			usleep_range(1000, 2000);
		}

		if (!retry)
			return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_OF
static inline void ap3223_init_of(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	u16 int_gpio = 0;

	if (np) {
		int_gpio = of_get_named_gpio(np, "int-gpio", 0);
		if (gpio_is_valid(int_gpio))
			client->irq = gpio_to_irq(int_gpio);
	}
}
#else
static inline void ap3223_init_of(struct i2c_client *client) {}
#endif /* CONFIG_OF */

static int ap3223_init(struct ap3223_data *data)
{
	struct i2c_client *client = data->client;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto ap3223_init_exit;
	}

	ap3223_init_of(client);

	err = ap3223_sw_reset(client);
	if (err < 0)
		goto ap3223_init_exit;

	err = ap3223_init_client(client);
	if (err < 0)
		goto ap3223_init_exit;

	err = ap3223_init_reg_config(client);
	if (err < 0) {
		dev_err(&client->dev, "Failed to write initial reg config\n");
		goto ap3223_init_exit;
	}

	if (client->irq > 0) {
		err = request_threaded_irq(client->irq, NULL,
					   ap3223_irq_threaded_fn,
					   IRQF_TRIGGER_FALLING |
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   "ap3223", data);
		if (err < 0) {
			dev_err(&client->dev, "Err: %d, could not get IRQ %d\n",
				err, client->irq);
			goto ap3223_init_exit;
		}

		data->event_enabled = true;
	}

	err = regcache_sync(data->regmap);
	if (err < 0) {
		dev_err(&client->dev, "regcache_sync failure");
		goto ap3223_init_exit;
	}

	return 0;

ap3223_init_exit:
	return err;
}

static int ap3223_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ap3223_data *data;
	struct iio_dev *indio_dev;
	int ret;
	struct regmap *regmap;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);

	regmap = devm_regmap_init_i2c(client, &ap3223_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Regmap initialization failed.\n");
		ret = PTR_ERR(regmap);
		return ret;
	}

	i2c_set_clientdata(client, indio_dev);

	data->regmap = regmap;
	data->client = client;
	data->ps_calib = 0;
	data->event_enabled = false;

	mutex_init(&data->ap3223_mutex);
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &ap3223_info;
	indio_dev->name = AP3223_DRV_NAME;
	indio_dev->channels = ap3223_channels;
	indio_dev->num_channels = ARRAY_SIZE(ap3223_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ap3223_init(data);
	if (ret < 0) {
		dev_err(&client->dev, "Chip init failed\n");
		return ret;
	}

	return devm_iio_device_register(&client->dev, indio_dev);
}

static int ap3223_remove(struct i2c_client *client)
{
	struct ap3223_data *data = iio_priv(i2c_get_clientdata(client));

	if (client->irq > 0)
		free_irq(client->irq, data);

	ap3223_sw_reset(data->client);
	ap3223_set_mode(data->client, 0);

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
	.probe          = ap3223_probe,
	.remove         = ap3223_remove,
	.id_table       = ap3223_id,
};

module_i2c_driver(ap3223_driver);

MODULE_AUTHOR("Suresh Rajashekara <sureshraj@google.com>");
MODULE_DESCRIPTION("AP3223 Ambient Light and Proximity Sensor Driver");
MODULE_LICENSE("GPL v2");
