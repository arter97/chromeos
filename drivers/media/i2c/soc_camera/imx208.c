/*
 * Driver for IMX208 CMOS Image Sensor from Sony
 *
 * Copyright (C) 2014, Andrew Chew <achew@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/v4l2-mediabus.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>

#include <media/soc_camera.h>
#include <media/v4l2-async.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>

/* IMX208 supported geometry */
#define IMX208_WIDTH			1920
#define IMX208_HEIGHT			1080

#define IMX208_TABLE_END		0xffff

#define IMX208_ANALOGUE_GAIN_MULTIPLIER	256
#define IMX208_ANALOGUE_GAIN_MIN	(1 * IMX208_ANALOGUE_GAIN_MULTIPLIER)
#define IMX208_ANALOGUE_GAIN_MAX	(16 * IMX208_ANALOGUE_GAIN_MULTIPLIER)
/* In dB */
#define IMX208_DIGITAL_GAIN_MIN		0
#define IMX208_DIGITAL_GAIN_MAX		24
#define IMX208_DIGITAL_GAIN_STEP	6

struct imx208_reg {
	u16 addr;
	u8 val;
};

static const struct imx208_reg mode_1920x1080[] = {
	/* PLL settting */
	{0x0305, 0x04}, /* pre_pll_clk_div[7:0] 0x01 */
	{0x0307, 0x87}, /* pll_multiplier[7:0] 0x2D */
	{0x303C, 0x4B}, /* PLSTATIM[7:0] 0x38 */
	{0x30A4, 0x02}, /* RGPLTD[1:0] 0x02 */
	/* Mode setting */
	{0x0112, 0x0a}, /* ccp_dt_fmt[15:8] 0x0a */
	{0x0113, 0x0a}, /* ccp_dt_fmt[7:0] 0x0a */
	{0x0340, 0x04}, /* frame_length_lines[15:8] 0x04 */
	{0x0341, 0xb0}, /* frame_length_lines[7:0] 0xb0 */
	{0x0342, 0x08}, /* line_length_pck[15:8] 0x08 */
	{0x0343, 0xc8}, /* line_length_pck[7:0] 0xc8 */
	{0x0344, 0x00}, /* x_addr_start[15:8] 0x00 */
	{0x0345, 0x08}, /* x_addr_start[7:0] 0x00 */
	{0x0346, 0x00}, /* y_addr_start[15:8] 0x00 */
	{0x0347, 0x08}, /* y_addr_start[7:0] 0x00 */
	{0x0348, 0x07}, /* x_addr_end[15:8] 0x07 */
	{0x0349, 0x87}, /* x_addr_end[7:0] 0x8f */
	{0x034a, 0x04}, /* y_addr_end[15:8] 0x04 */
	{0x034b, 0x3f}, /* y_addr_end[7:0] 0x47 */
	{0x034c, 0x07}, /* x_output_size[15:8] 0x07 */
	{0x034d, 0x80}, /* x_output_size[7:0] 0x90 */
	{0x034e, 0x04}, /* y_output_size[15:8] 0x04 */
	{0x034f, 0x38}, /* y_output_size[7:0] 0x48 */
	{0x0381, 0x01}, /* x_even_inc[3:0] 0x01 */
	{0x0383, 0x01}, /* x_odd_inc[3:0] 0x01 */
	{0x0385, 0x01}, /* y_even_inc[3:0] 0x01 */
	{0x0387, 0x01}, /* y_odd_inc[3:0] 0x01 */
	{0x3048, 0x00}, /* vmodefds 0x00 */
	{0x304e, 0x0a}, /* vtpxck_div 0x0a */
	{0x3050, 0x02}, /* opsyck_div 0x02 */
	{0x309b, 0x00}, /* rgdafdsumen 0x00 */
	{0x30d5, 0x00}, /* hadden,haddmode 0x00 */
	{0x3301, 0x01}, /* rglanesel 0x01 as non-continuous clock mode */
	{0x3318, 0x61}, /* mipi global timing table 0x61 */
	/* Shutter gain setting */
	{0x0202, 0x01}, /* coarse_integration_time[15:8] 0x01 */
	{0x0203, 0x90}, /* coarse_integration_time[7:0] 0x90 */
	{ IMX208_TABLE_END, 0x00 }
};

static const struct imx208_reg start[] = {
	{0x0100, 0x01}, /* mode select streaming on */
	{ IMX208_TABLE_END, 0x00 }
};

static const struct imx208_reg stop[] = {
	{0x0100, 0x00 }, /* mode select streaming off */
	{ IMX208_TABLE_END, 0x00 }
};

enum {
	TEST_PATTERN_DISABLED,
	TEST_PATTERN_SOLID_BLACK,
	TEST_PATTERN_SOLID_WHITE,
	TEST_PATTERN_SOLID_RED,
	TEST_PATTERN_SOLID_GREEN,
	TEST_PATTERN_SOLID_BLUE,
	TEST_PATTERN_COLOR_BAR,
	TEST_PATTERN_FADE_TO_GREY_COLOR_BAR,
	TEST_PATTERN_PN9,
	TEST_PATTERN_MAX
};
static const char * const tp_qmenu[] = {
	"Disabled",
	"Solid Black",
	"Solid White",
	"Solid Red",
	"Solid Green",
	"Solid Blue",
	"Color Bar",
	"Fade to Grey Color Bar",
	"PN9",
};

/* IMX208 has only one fixed colorspace per pixelcode */
struct imx208_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

#define SIZEOF_I2C_TRANSBUF 32

struct imx208 {
	struct v4l2_subdev		subdev;
	struct v4l2_ctrl_handler	ctrl_handler;
	const struct imx208_datafmt	*fmt;
	struct v4l2_clk			*clk;

	int				hflip;
	int				vflip;
	u8				analogue_gain;
	u8				digital_gain;
	u16				test_pattern;
	u16				test_pattern_solid_color_r;
	u16				test_pattern_solid_color_gr;
	u16				test_pattern_solid_color_b;
	u16				test_pattern_solid_color_gb;
};

static const struct imx208_datafmt imx208_colour_fmts[] = {
	{V4L2_MBUS_FMT_SRGGB10_1X10, V4L2_COLORSPACE_SRGB},
};

static struct imx208 *to_imx208(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct imx208, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct imx208_datafmt *imx208_find_datafmt(
	enum v4l2_mbus_pixelcode code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(imx208_colour_fmts); i++)
		if (imx208_colour_fmts[i].code == code)
			return imx208_colour_fmts + i;

	return NULL;
}

static int reg_write(struct i2c_client *client, const u16 addr, const u8 data)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	u8 tx[3];
	int ret;

	msg.addr = client->addr;
	msg.buf = tx;
	msg.len = 3;
	msg.flags = 0;

	tx[0] = addr >> 8;
	tx[1] = addr & 0xff;
	tx[2] = data;

	ret = i2c_transfer(adap, &msg, 1);

	mdelay(2);

	return ret == 1 ? 0 : -EIO;
}

static int reg_read(struct i2c_client *client, const u16 addr)
{
	u8 buf[2] = {addr >> 8, addr & 0xff};
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = buf,
		}, {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_warn(&client->dev, "Reading register %x from %x failed\n",
			 addr, client->addr);
		return ret;
	}

	return buf[0];
}

static int reg_write_table(struct i2c_client *client,
			   const struct imx208_reg table[])
{
	const struct imx208_reg *reg;
	int ret;

	for (reg = table; reg->addr != IMX208_TABLE_END; reg++) {
		ret = reg_write(client, reg->addr, reg->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* V4L2 subdev video operations */
static int imx208_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx208 *priv = to_imx208(client);
	u8 reg = 0x00;
	int ret;

	if (!enable)
		return reg_write_table(client, stop);

	ret = reg_write_table(client, mode_1920x1080);
	if (ret)
		return ret;

	/* Handle flip/mirror */
	if (priv->hflip)
		reg |= 0x1;
	if (priv->vflip)
		reg |= 0x2;

	ret = reg_write(client, 0x0101, reg);
	if (ret)
		return ret;

	/* Handle analogue gain */
	ret = reg_write(client, 0x0205, priv->analogue_gain);
	if (ret)
		return ret;

	/* Handle digital gain */
	ret = reg_write(client, 0x020E, priv->digital_gain);
	ret |= reg_write(client, 0x0210, priv->digital_gain);
	ret |= reg_write(client, 0x0212, priv->digital_gain);
	ret |= reg_write(client, 0x0214, priv->digital_gain);
	if (ret)
		return ret;

	/* Handle test pattern */
	if (priv->test_pattern) {
		ret = reg_write(client, 0x3282, 0x01);
		ret |= reg_write(client, 0x0600, priv->test_pattern >> 8);
		ret |= reg_write(client, 0x0601, priv->test_pattern & 0xff);
		ret |= reg_write(client, 0x0602,
				 priv->test_pattern_solid_color_r >> 8);
		ret |= reg_write(client, 0x0603,
				 priv->test_pattern_solid_color_r & 0xff);
		ret |= reg_write(client, 0x0604,
				 priv->test_pattern_solid_color_gr >> 8);
		ret |= reg_write(client, 0x0605,
				 priv->test_pattern_solid_color_gr & 0xff);
		ret |= reg_write(client, 0x0606,
				 priv->test_pattern_solid_color_b >> 8);
		ret |= reg_write(client, 0x0607,
				 priv->test_pattern_solid_color_b & 0xff);
		ret |= reg_write(client, 0x0608,
				 priv->test_pattern_solid_color_gb >> 8);
		ret |= reg_write(client, 0x0609,
				 priv->test_pattern_solid_color_gb & 0xff);
	} else {
		ret = reg_write(client, 0x3282, 0x00);
		ret |= reg_write(client, 0x0600, 0x00);
		ret |= reg_write(client, 0x0601, 0x00);
	}
	if (ret)
		return ret;

	return reg_write_table(client, start);
}

static int imx208_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= IMX208_WIDTH;
	a->bounds.height		= IMX208_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int imx208_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct v4l2_rect *rect = &a->c;

	a->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	rect->top	= 0;
	rect->left	= 0;
	rect->width	= IMX208_WIDTH;
	rect->height	= IMX208_HEIGHT;

	return 0;
}

static int imx208_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(imx208_colour_fmts))
		return -EINVAL;

	*code = imx208_colour_fmts[index].code;

	return 0;
}

static int imx208_g_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx208 *priv = to_imx208(client);

	const struct imx208_datafmt *fmt = priv->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->width	= IMX208_WIDTH;
	mf->height	= IMX208_HEIGHT;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int imx208_try_mbus_fmt(struct v4l2_subdev *sd,
			       struct v4l2_mbus_framefmt *mf)
{
	const struct imx208_datafmt *fmt = imx208_find_datafmt(mf->code);

	dev_dbg(sd->v4l2_dev->dev, "%s(%u)\n", __func__, mf->code);

	if (!fmt) {
		mf->code	= imx208_colour_fmts[0].code;
		mf->colorspace	= imx208_colour_fmts[0].colorspace;
	}

	mf->width	= IMX208_WIDTH;
	mf->height	= IMX208_HEIGHT;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int imx208_s_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx208 *priv = to_imx208(client);

	dev_dbg(sd->v4l2_dev->dev, "%s(%u)\n", __func__, mf->code);

	/* MIPI CSI could have changed the format, double-check */
	if (!imx208_find_datafmt(mf->code))
		return -EINVAL;

	imx208_try_mbus_fmt(sd, mf);

	priv->fmt = imx208_find_datafmt(mf->code);

	return 0;
}

static int imx208_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_2_LANE |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

/* V4L2 subdev core operations */
static int imx208_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct imx208 *priv = to_imx208(client);

	return soc_camera_set_power(&client->dev, ssdd, priv->clk, on);
}

/* V4L2 ctrl operations */
static int imx208_s_ctrl_test_pattern(struct v4l2_ctrl *ctrl)
{
	struct imx208 *priv =
		container_of(ctrl->handler, struct imx208, ctrl_handler);

	switch (ctrl->val) {
	case TEST_PATTERN_DISABLED:
		priv->test_pattern = 0x0000;
		break;
	case TEST_PATTERN_SOLID_BLACK:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_SOLID_WHITE:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x03ff;
		priv->test_pattern_solid_color_gr = 0x03ff;
		priv->test_pattern_solid_color_b = 0x03ff;
		priv->test_pattern_solid_color_gb = 0x03ff;
		break;
	case TEST_PATTERN_SOLID_RED:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x03ff;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_SOLID_GREEN:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x03ff;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x03ff;
		break;
	case TEST_PATTERN_SOLID_BLUE:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x03ff;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_COLOR_BAR:
		priv->test_pattern = 0x0002;
		break;
	case TEST_PATTERN_FADE_TO_GREY_COLOR_BAR:
		priv->test_pattern = 0x0003;
		break;
	case TEST_PATTERN_PN9:
		priv->test_pattern = 0x0004;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int imx208_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx208 *priv =
		container_of(ctrl->handler, struct imx208, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
	u8 reg;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		priv->hflip = ctrl->val;
		break;
	case V4L2_CID_VFLIP:
		priv->vflip = ctrl->val;
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		/*
		 * Register value goes from 0 to 240, and the gain setting is
		 * 256 / (256 - reg).  This results in a total gain of 1.0f to
		 * 16.0f.  We multiply the control setting by some big number
		 * IMX208_ANALOGUE_GAIN_MULTIPLIER to make use of the full
		 * resolution of this register.
	 	 */
		priv->analogue_gain =
			256 - ((256 * IMX208_ANALOGUE_GAIN_MULTIPLIER) /
			       ctrl->val);
		break;
	case V4L2_CID_GAIN:
		switch (ctrl->val) {
		case 0:
			priv->digital_gain = 0x01;
			break;
		case 6:
			priv->digital_gain = 0x02;
			break;
		case 12:
			priv->digital_gain = 0x04;
			break;
		case 18:
			priv->digital_gain = 0x08;
			break;
		case 24:
			priv->digital_gain = 0x10;
			break;
		default:
			return -EINVAL;
		}
		break;
	case V4L2_CID_TEST_PATTERN:
		return imx208_s_ctrl_test_pattern(ctrl);
	default:
		return -EINVAL;
	}

	/* If enabled, apply settings immediately */
	reg = reg_read(client, 0x0100);
	if ((reg & 0x3f) == 0x01)
		imx208_s_stream(&priv->subdev, 1);

	return 0;
}

/* Various V4L2 operations tables */
static struct v4l2_subdev_video_ops imx208_subdev_video_ops = {
	.s_stream	= imx208_s_stream,
	.cropcap	= imx208_cropcap,
	.g_crop		= imx208_g_crop,
	.enum_mbus_fmt	= imx208_enum_mbus_fmt,
	.g_mbus_fmt	= imx208_g_mbus_fmt,
	.try_mbus_fmt	= imx208_try_mbus_fmt,
	.s_mbus_fmt	= imx208_s_mbus_fmt,
	.g_mbus_config	= imx208_g_mbus_config,
};

static struct v4l2_subdev_core_ops imx208_subdev_core_ops = {
	.s_power	= imx208_s_power,
};

static struct v4l2_subdev_ops imx208_subdev_ops = {
	.core		= &imx208_subdev_core_ops,
	.video		= &imx208_subdev_video_ops,
};

static const struct v4l2_ctrl_ops imx208_ctrl_ops = {
	.s_ctrl		= imx208_s_ctrl,
};

static int imx208_ctrls_init(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx208 *priv = to_imx208(client);
	int ret;

	v4l2_ctrl_handler_init(&priv->ctrl_handler, 10);

	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx208_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx208_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx208_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN,
			  IMX208_ANALOGUE_GAIN_MIN,
			  IMX208_ANALOGUE_GAIN_MAX,
			  1,
			  IMX208_ANALOGUE_GAIN_MIN);

	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx208_ctrl_ops,
			  V4L2_CID_GAIN,
			  IMX208_DIGITAL_GAIN_MIN,
			  IMX208_DIGITAL_GAIN_MAX,
			  IMX208_DIGITAL_GAIN_STEP,
			  IMX208_DIGITAL_GAIN_MIN);

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler, &imx208_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(tp_qmenu) - 1, 0, 0, tp_qmenu);

	priv->subdev.ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			 priv->ctrl_handler.error);
		ret = priv->ctrl_handler.error;
		goto error;
	}

	ret = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (ret < 0) {
		dev_err(&client->dev, "Error %d setting default controls\n",
			ret);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return ret;
}

static int imx208_video_probe(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	u16 model_id;
	u32 lot_id;
	u16 chip_id;
	int ret;

	ret = imx208_s_power(subdev, 1);
	if (ret < 0)
		return ret;

	/* Check and show model, lot, and chip ID. */
	ret = reg_read(client, 0x0000);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Model ID (high byte)\n");
		goto done;
	}
	model_id = ret << 8;

	ret = reg_read(client, 0x0001);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Model ID (low byte)\n");
		goto done;
	}
	model_id |= ret;

	ret = reg_read(client, 0x0004);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Lot ID (high byte)\n");
		goto done;
	}
	lot_id = ret << 16;

	ret = reg_read(client, 0x0005);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Lot ID (mid byte)\n");
		goto done;
	}
	lot_id |= ret << 8;

	ret = reg_read(client, 0x0006);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Lot ID (low byte)\n");
		goto done;
	}
	lot_id |= ret;

	ret = reg_read(client, 0x000D);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Chip ID (high byte)\n");
		goto done;
	}
	chip_id = ret << 8;

	ret = reg_read(client, 0x000E);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Chip ID (low byte)\n");
		goto done;
	}
	chip_id |= ret;

	if (model_id != 0x0208) {
		dev_err(&client->dev, "Model ID: %x not supported!\n",
			model_id);
		ret = -ENODEV;
		goto done;
	}

	dev_info(&client->dev,
		 "Model ID 0x%04x, Lot ID 0x%06x, Chip ID 0x%04x\n",
		 model_id, lot_id, chip_id);

done:
	imx208_s_power(subdev, 0);

	return ret;
}

static int imx208_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct imx208 *priv;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	int ret;

	if (!ssdd) {
		dev_err(&client->dev, "IMX208: missing platform data!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_BYTE\n");
		return -EIO;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct imx208), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->clk = v4l2_clk_get(&client->dev, "mclk");
	if (IS_ERR(priv->clk)) {
		dev_info(&client->dev, "Error %ld getting clock\n",
			 PTR_ERR(priv->clk));
		return -EPROBE_DEFER;
	}

	priv->fmt = &imx208_colour_fmts[0];

	v4l2_i2c_subdev_init(&priv->subdev, client, &imx208_subdev_ops);

	ret = soc_camera_power_init(&client->dev, ssdd);
	if (ret < 0)
		goto eclk;

	ret = imx208_ctrls_init(&priv->subdev);
	if (ret < 0)
		goto eclk;

	ret = imx208_video_probe(client);
	if (ret < 0)
		goto eclk;

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0)
		goto eclk;

	return 0;

eclk:
	v4l2_clk_put(priv->clk);

	return ret;
}

static int imx208_remove(struct i2c_client *client)
{
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct imx208 *priv = to_imx208(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	v4l2_clk_put(priv->clk);

	if (ssdd->free_bus)
		ssdd->free_bus(ssdd);

	return 0;
}

static const struct i2c_device_id imx208_id[] = {
	{ "imx208", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, imx208_id);

static struct i2c_driver imx208_i2c_driver = {
	.driver = {
		.name = "imx208",
	},
	.probe		= imx208_probe,
	.remove		= imx208_remove,
	.id_table	= imx208_id,
};

module_i2c_driver(imx208_i2c_driver);

MODULE_DESCRIPTION("Sony IMX208 Camera driver");
MODULE_AUTHOR("Guennadi Liakhovetski <g.liakhovetski@gmx.de>");
MODULE_LICENSE("GPL v2");
