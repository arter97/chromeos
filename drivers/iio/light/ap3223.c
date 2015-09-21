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
 * The driver for AP3223 was originally distributed by dyna image in a
 * different framework (as an input driver). This driver uses code from
 * that driver and converts it to iio framework. The non-iio driver from
 * dyna image is not available online anywhere, so there is no reference
 * for it here. However, that driver is also GPLv2.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
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
	struct i2c_client *client;
	struct regmap *regmap;
};

static const u8 ap3223_initial_reg_conf[] = {
	AP3223_REG_SYS_CTRL, AP3223_SYS_ALS_PS_ENABLE,
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

#define AP3223_LIGHT_CHANNEL {				\
	.type = IIO_LIGHT,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
}

#define AP3223_PROXIMITY_CHANNEL {			\
	.type = IIO_PROXIMITY,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
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

static int ap3223_sw_reset(struct i2c_client *client)
{
	struct ap3223_data *data = iio_priv(i2c_get_clientdata(client));
	int err = 0;

	err = regmap_write(data->regmap, AP3223_REG_SYS_CTRL,
		     AP3223_SYS_DEV_RESET);
	if (err < 0)
		dev_err(&client->dev, "Failed to set mode\n");

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

static int ap3223_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct ap3223_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = ap3223_get_mode(data->client);
			if (ret < 0)
				break;

			*val = ap3223_get_adc_value(data->client);
			if (*val < 0)
				break;

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

	default:
	       break;
	}

	return ret;
}

static const struct iio_info ap3223_info = {
	.driver_module  = THIS_MODULE,
	.read_raw       = ap3223_read_raw,
};

static int ap3223_init_reg_config(struct i2c_client *client)
{
	struct ap3223_data *data = iio_priv(i2c_get_clientdata(client));
	int i;

	for (i = 0; i < ARRAY_SIZE(ap3223_initial_reg_conf); i += 2) {
		if (regmap_write(data->regmap, ap3223_initial_reg_conf[i],
			     ap3223_initial_reg_conf[i + 1]) < 0)
			return -EINVAL;
	}

	return 0;
}

static int ap3223_init(struct ap3223_data *data)
{
	struct i2c_client *client = data->client;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit_ap3223_init;
	}

	err = ap3223_sw_reset(client);
	if (err < 0)
		goto exit_ap3223_init;

	err = ap3223_init_client(client);
	if (err < 0)
		goto exit_ap3223_init;

	err = ap3223_init_reg_config(client);
	if (err < 0) {
		dev_err(&client->dev, "Failed to write initial reg config\n");
		goto exit_ap3223_init;
	}

	err = regcache_sync(data->regmap);

exit_ap3223_init:
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
