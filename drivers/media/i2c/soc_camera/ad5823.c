/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

#include <media/soc_camera.h>
#include <media/v4l2-async.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>

#include <uapi/linux/v4l2-controls.h>

/* address */
#define AD5823_RESET			0x1
#define AD5823_MODE			0x2
#define AD5823_VCM_MOVE_TIME		0x3
#define AD5823_VCM_CODE_MSB		0x4
#define AD5823_VCM_CODE_LSB		0x5
#define AD5823_VCM_THRESHOLD_MSB	0x6
#define AD5823_VCM_THRESHOLD_LSB	0x7
#define AD5823_RING_CTRL		(1 << 2)

#define AD5823_PWR_DEV_OFF		0
#define AD5823_PWR_DEV_ON		1

#define AD5823_ACTUATOR_RANGE		1023
#define AD5823_POS_LOW_DEFAULT		0
#define AD5823_POS_HIGH_DEFAULT		1023
#define AD5823_FOCUS_MACRO		568
#define AD5823_FOCUS_INFINITY		146

#define SETTLETIME_MS			15
#define FOCAL_LENGTH			4.507f
#define FNUMBER				2.8f
#define	AD5823_MOVE_TIME_VALUE		0x43

#define AD5823_MAX_RETRIES		3

struct ad5823 {
	struct v4l2_subdev		subdev;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct v4l2_clk			*clk;

	struct regmap			*regmap;
	struct regulator		*reg;
	int				pwr_dev;
};

static struct ad5823 *to_ad5823(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ad5823, subdev);
}

static int ad5823_set_position(struct ad5823 *priv, u32 position)
{
	int ret = 0;

	ret |= regmap_write(priv->regmap, AD5823_VCM_MOVE_TIME,
		AD5823_MOVE_TIME_VALUE);
	ret |= regmap_write(priv->regmap, AD5823_MODE, 0);
	ret |= regmap_write(priv->regmap, AD5823_VCM_CODE_MSB,
		((position >> 8) & 0x3) | (1 << 2));
	ret |= regmap_write(priv->regmap, AD5823_VCM_CODE_LSB,
		position & 0xFF);

	return ret;
}

static int ad5823_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct ad5823 *priv = to_ad5823(client);

	return soc_camera_set_power(&client->dev, ssdd, priv->clk, on);
}

static int ad5823_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ad5823 *priv =
		container_of(ctrl->handler, struct ad5823, ctrl_handler);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		ret = ad5823_set_position(priv, ctrl->val);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static struct v4l2_subdev_core_ops ad5823_core_ops = {
	.s_power		= ad5823_s_power,
};

static struct v4l2_subdev_ops ad5823_subdev_ops = {
	.core			= &ad5823_core_ops,
};

static const struct v4l2_ctrl_ops ad5823_ctrl_ops = {
	.s_ctrl			= ad5823_s_ctrl,
};

static int ad5823_ctrls_init(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ad5823 *priv = to_ad5823(client);
	int ret;

	v4l2_ctrl_handler_init(&priv->ctrl_handler, 10);

	v4l2_ctrl_new_std(&priv->ctrl_handler, &ad5823_ctrl_ops,
			V4L2_CID_FOCUS_ABSOLUTE,
			AD5823_POS_LOW_DEFAULT,
			AD5823_POS_HIGH_DEFAULT,
			1, 100);

	priv->subdev.ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		ret = priv->ctrl_handler.error;
		goto error;
	}

	ret = ad5823_s_power(&priv->subdev, 1);
	if (ret < 0)
		goto error;

	ret = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (ret < 0) {
		dev_err(&client->dev, "Error %d setting default controls\n",
			ret);
		goto error_power;
	}

	ad5823_s_power(&priv->subdev, 0);

	return 0;

error_power:
	ad5823_s_power(&priv->subdev, 0);
error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return ret;
}

static int ad5823_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ad5823 *priv;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	static struct regmap_config ad5823_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int ret;

	if (!ssdd) {
		dev_err(&client->dev, "AD5823: missing platform data!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_BYTE\n");
		return -EIO;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct ad5823), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->clk = v4l2_clk_get(&client->dev, "mclk");
	if (IS_ERR(priv->clk)) {
		dev_info(&client->dev, "Error %ld getting clock\n",
			 PTR_ERR(priv->clk));
		return -EPROBE_DEFER;
	}

	priv->reg = devm_regulator_get(&client->dev, "power");
	if (IS_ERR_OR_NULL(priv->reg)) {
		dev_err(&client->dev, "couldn't get power regulator, err %ld\n",
			PTR_ERR(priv->reg));
		priv->reg = NULL;
		ret = PTR_ERR(priv->reg);
		goto eclk;
	}

	priv->regmap = devm_regmap_init_i2c(client, &ad5823_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev,
			"Failed to allocate register map: %d\n", ret);
		goto eclk;
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &ad5823_subdev_ops);

	ret = soc_camera_power_init(&client->dev, ssdd);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to initialize soc-camera power: %d\n", ret);
		goto eclk;
	}

	ret = ad5823_ctrls_init(&priv->subdev);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to initialize ctrls: %d\n", ret);
		goto eclk;
	}

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to register subdev: %d\n", ret);
		goto eclk;
	}

	dev_notice(&client->dev, "Finished probe\n");

	return 0;

eclk:
	v4l2_clk_put(priv->clk);

	dev_notice(&client->dev, "Failed probe\n");

	return ret;
}

static int ad5823_remove(struct i2c_client *client)
{
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct ad5823 *priv = to_ad5823(client);

	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	v4l2_clk_put(priv->clk);

	if (ssdd->free_bus)
		ssdd->free_bus(ssdd);

	return 0;
}

static const struct i2c_device_id ad5823_id[] = {
	{ "ad5823", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ad5823_id);

static struct i2c_driver ad5823_i2c_driver = {
	.driver = {
		.name = "ad5823",
	},
	.probe	  = ad5823_probe,
	.remove	  = ad5823_remove,
	.id_table = ad5823_id,
};

module_i2c_driver(ad5823_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Analog Devices AD5823 focuser");
MODULE_AUTHOR("Andrew Chew <achew@nvidia.com>");
MODULE_LICENSE("GPL v2");
