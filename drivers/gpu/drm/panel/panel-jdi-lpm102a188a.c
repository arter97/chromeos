/*
 * Copyright (C) 2014 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <drm/panel/panel-jdi-lpm102a188a.h>

#include <video/mipi_display.h>

struct backlight_lp8557 {
	struct backlight_device *bl_dev;
	struct i2c_client *client;

	bool enabled;

	enum lp8557_config_brightness_mode brightness_mode;
	bool auto_detect_leds;
	bool pwm_standby;
	bool current_use_iset;
	enum lp8557_current bl_current;
	bool pwm_use_fset;
	enum lp8557_pgen_frequency pwm_frequency;
	enum lp8557_boost_freq boost_frequency;
	enum lp8557_boost_bcomp boost_bcomp;
	bool boost_use_iset;
	bool boost_use_fset;
	u8 led_enable_mask;
	enum lp8557_step_ramp step_ramp;
	enum lp8557_step_smoothing step_smoothing;
};

struct panel_jdi {
	struct drm_panel base;
	struct backlight_lp8557 lp8557;
	struct mipi_dsi_device *dsi;

	struct regulator *supply;
	struct regulator *ddi_supply;
	int enable_gpio;
	unsigned long enable_gpio_flags;
	int reset_gpio;
	unsigned long reset_gpio_flags;

	const struct drm_display_mode *mode;

	bool enabled;
};

static inline struct panel_jdi *to_panel_jdi(struct drm_panel *panel)
{
	return container_of(panel, struct panel_jdi, base);
}

static int lp8557_backlight_get_intensity(struct backlight_device *bl_dev)
{
	struct backlight_lp8557 *lp8557 = bl_get_data(bl_dev);
	struct i2c_client *client = lp8557->client;
	int val;

	val = i2c_smbus_read_byte_data(client, LP8557_BRIGHTNESS_LOW) >> 4;
	val |= (i2c_smbus_read_byte_data(client, LP8557_BRIGHTNESS_HIGH) << 4);

	return val;
}

static int lp8557_backlight_update_status(struct backlight_device *bl_dev)
{
	struct backlight_lp8557 *lp8557 = bl_get_data(bl_dev);
	struct i2c_client *client = lp8557->client;
	int ret;
	u8 val;

	if (!lp8557->enabled)
		return 0;

	val = LP8557_BRIGHTNESS_LOW_MASK(bl_dev->props.brightness);
	ret = i2c_smbus_write_byte_data(client, LP8557_BRIGHTNESS_LOW, val);
	if (ret) {
		DRM_ERROR("i2c_write brightness low fail, ret=%d\n", ret);
		return ret;
	}

	val = LP8557_BRIGHTNESS_HIGH_MASK(bl_dev->props.brightness);
	ret = i2c_smbus_write_byte_data(client, LP8557_BRIGHTNESS_HIGH, val);
	if (ret) {
		DRM_ERROR("i2c_write brightness high fail, ret=%d\n", ret);
		return ret;
	}

	val = LP8557_COMMAND_ON;
	ret = i2c_smbus_write_byte_data(client, LP8557_COMMAND, val);
	if (ret) {
		DRM_ERROR("i2c_write command fail, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int lp8557_enable_backlight(struct backlight_lp8557 *lp8557)
{
	struct i2c_client *client = lp8557->client;
	int ret;
	u8 val;

	val = LP8557_CONFIG_BRTMODE(lp8557->brightness_mode) |
		(lp8557->auto_detect_leds ? LP8557_CONFIG_AUTO_DETECT_LED : 0) |
		(lp8557->pwm_standby ? LP8557_CONFIG_PWM_STANDBY : 0);
	ret = i2c_smbus_write_byte_data(client, LP8557_CONFIG, val);
	if (ret) {
		DRM_ERROR("i2c_write config fail, ret=%d\n", ret);
		return ret;
	}

	val = LP8557_CURRENT_MAXCURR(lp8557->bl_current) |
		(lp8557->current_use_iset ? LP8557_CURRENT_ISET : 0);
	ret = i2c_smbus_write_byte_data(client, LP8557_CURRENT, val);
	if (ret) {
		DRM_ERROR("i2c_write current fail, ret=%d\n", ret);
		return ret;
	}

	val = LP8557_PGEN_FREQ(lp8557->pwm_frequency) | LP8557_PGEN_MAGIC |
		(lp8557->pwm_use_fset ? LP8557_PGEN_FSET : 0);
	ret = i2c_smbus_write_byte_data(client, LP8557_PGEN, val);
	if (ret) {
		DRM_ERROR("i2c_write pgen fail, ret=%d\n", ret);
		return ret;
	}

	val = LP8557_BOOST_FREQ(lp8557->boost_frequency) |
		LP8557_BOOST_BCOMP(lp8557->boost_bcomp) |
		(lp8557->boost_use_iset ? LP8557_BOOST_BCSET : 0) |
		(lp8557->boost_use_fset ? LP8557_BOOST_BFSET : 0);
	ret = i2c_smbus_write_byte_data(client, LP8557_BOOST, val);
	if (ret) {
		DRM_ERROR("i2c_write boost fail, ret=%d\n", ret);
		return ret;
	}

	val = lp8557->led_enable_mask | LP8557_LED_ENABLE_MAGIC;
	ret = i2c_smbus_write_byte_data(client, LP8557_LED_ENABLE, val);
	if (ret) {
		DRM_ERROR("i2c_write led enable fail, ret=%d\n", ret);
		return ret;
	}

	val = LP8557_STEP_RAMP(lp8557->step_ramp) |
		LP8557_STEP_SMOOTHING(lp8557->step_smoothing);
	ret = i2c_smbus_write_byte_data(client, LP8557_STEP, val);
	if (ret) {
		DRM_ERROR("i2c_write step fail, ret=%d\n", ret);
		return ret;
	}

	lp8557->enabled = true;

	return lp8557_backlight_update_status(lp8557->bl_dev);
}

static int lp8557_disable_backlight(struct backlight_lp8557 *lp8557)
{
	struct i2c_client *client = lp8557->client;
	int ret;

	lp8557->enabled = false;

	ret = i2c_smbus_write_byte_data(client, LP8557_COMMAND, 0);
	if (ret) {
		DRM_ERROR("i2c_write command fail, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static const struct backlight_ops lp8557_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = lp8557_backlight_get_intensity,
	.update_status = lp8557_backlight_update_status,
};

static int panel_jdi_disable(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);
	int ret;

	if (!jdi->enabled)
		return 0;

	ret = lp8557_disable_backlight(&jdi->lp8557);
	if (ret < 0)
		DRM_ERROR("failed to disable backlight: %d\n", ret);

	return ret;
}

static int panel_jdi_unprepare(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);
	int ret;

	if (!jdi->enabled)
		return 0;

	ret = mipi_dsi_dcs_set_display_off(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to set display off: %d\n", ret);

	msleep(50);

	ret = mipi_dsi_dcs_enter_sleep_mode(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to enter sleep mode: %d\n", ret);

	msleep(150);

	gpio_set_value(jdi->reset_gpio,
		(jdi->reset_gpio_flags & GPIO_ACTIVE_LOW) ? 0 : 1);

	usleep_range(1000, 2000);

	gpio_set_value(jdi->enable_gpio,
		(jdi->enable_gpio_flags & GPIO_ACTIVE_LOW) ? 1 : 0);

	usleep_range(2000, 4000);

	regulator_disable(jdi->supply);

	usleep_range(10000, 20000);

	regulator_disable(jdi->ddi_supply);

	msleep(50);

	jdi->enabled = false;

	return 0;
}

struct touch_calibration_dcs_cmd {
	u8 command;
	u8 data[32];
	int data_len;
};

static const struct touch_calibration_dcs_cmd cal_cmds[] = {
	{
		.command = 0xB0,
		.data = { 0x04 },
		.data_len = 0,
	}, {
		.command = 0xED,
		.data = {
			0x4D, 0x00, 0x27, 0x11, 0x00, 0x83, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x4B, 0x00, 0x24, 0x27, 0x00,
			0xC1, 0x00, 0xC0, 0x06, 0x14, 0x00, 0x00, 0x00,
			0xC7, 0x17, 0x18, 0x4E, 0x00, 0x00, 0x00
		},
		.data_len = 31,
	}, {
		.command = 0xEE,
		.data = {
			0x26, 0x00, 0x00, 0x10, 0x00, 0x02, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x24, 0x00, 0x27, 0x27, 0x01,
			0x8B, 0x01, 0x8A, 0x0C, 0x63, 0x00, 0x00, 0x00,
			0xA0, 0x2E, 0x2F, 0x27, 0x00, 0x00, 0x00
		},
		.data_len = 31,
	}, {
		.command = 0xEF,
		.data = {
			0x74, 0x00, 0x00, 0x10, 0x00, 0x02, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x24, 0x00, 0x27, 0x27, 0x00,
			0x7E, 0x00, 0x7D, 0x03, 0xFA, 0x00, 0x00, 0x00,
			0xC7, 0x0F, 0x10, 0xA6, 0x00, 0x00, 0x00
		},
		.data_len = 31,
	},
};

static int panel_jdi_configure_touchscreen(struct panel_jdi *jdi)
{
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(cal_cmds); i++) {
		ret = mipi_dsi_dcs_write(jdi->dsi, cal_cmds[i].command,
				cal_cmds[i].data, cal_cmds[i].data_len);
		if (ret < 0) {
			DRM_ERROR("master touch_cal_cmd failed cmd=%d,ret=%d\n",
				cal_cmds[i].command, ret);
			return ret;
		}

		ret = mipi_dsi_dcs_write(jdi->dsi->slave, cal_cmds[i].command,
				cal_cmds[i].data, cal_cmds[i].data_len);
		if (ret < 0) {
			DRM_ERROR("slave touch_cal_cmd failed cmd=%d,ret=%d\n",
				cal_cmds[i].command, ret);
			return ret;
		}
	}
	return 0;
}

static int panel_jdi_prepare(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);
	int ret;
	u8 data;

	if (jdi->enabled)
		return 0;

	ret = regulator_enable(jdi->supply);
	if (ret < 0) {
		DRM_ERROR("failed to enable supply: %d\n", ret);
		return ret;
	}

	usleep_range(4000, 6000);

	ret = regulator_enable(jdi->ddi_supply);
	if (ret < 0) {
		DRM_ERROR("failed to enable ddi_supply: %d\n", ret);
		return ret;
	}

	usleep_range(2000, 4000);

	gpio_set_value(jdi->enable_gpio,
		(jdi->enable_gpio_flags & GPIO_ACTIVE_LOW) ? 0 : 1);

	msleep(20);

	gpio_set_value(jdi->reset_gpio,
		(jdi->reset_gpio_flags & GPIO_ACTIVE_LOW) ? 1 : 0);

	msleep(40);

	ret = mipi_dsi_dcs_set_column_address(jdi->dsi, 0,
				jdi->mode->crtc_hdisplay / 2);
	if (ret < 0)
		DRM_ERROR("failed to set column address: %d\n", ret);

	ret = mipi_dsi_dcs_set_column_address(jdi->dsi->slave, 0,
				jdi->mode->crtc_hdisplay / 2 - 1);
	if (ret < 0)
		DRM_ERROR("failed to set column address: %d\n", ret);

	ret = mipi_dsi_dcs_set_page_address(jdi->dsi, 0,
				jdi->mode->crtc_vdisplay - 1);
	if (ret < 0)
		DRM_ERROR("failed to set page address: %d\n", ret);

	ret = mipi_dsi_dcs_set_page_address(jdi->dsi->slave, 0,
				jdi->mode->crtc_vdisplay - 1);
	if (ret < 0)
		DRM_ERROR("failed to set page address: %d\n", ret);

	ret = mipi_dsi_dcs_exit_sleep_mode(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to exit sleep mode: %d\n", ret);

	ret = mipi_dsi_dcs_exit_sleep_mode(jdi->dsi->slave);
	if (ret < 0)
		DRM_ERROR("failed to exit sleep mode: %d\n", ret);

	ret = mipi_dsi_dcs_set_tear_on(jdi->dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		DRM_ERROR("failed to set tear on: %d\n", ret);

	ret = mipi_dsi_dcs_set_tear_on(jdi->dsi->slave,
			MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		DRM_ERROR("failed to set tear on: %d\n", ret);

	ret = mipi_dsi_dcs_set_address_mode(jdi->dsi, false, false, false,
			false, false, false, false, false);
	if (ret < 0)
		DRM_ERROR("failed to set address mode: %d\n", ret);

	ret = mipi_dsi_dcs_set_address_mode(jdi->dsi->slave, false, false,
			false, false, false, false, false, false);
	if (ret < 0)
		DRM_ERROR("failed to set address mode: %d\n", ret);

	ret = mipi_dsi_dcs_set_pixel_format(jdi->dsi, 0x77);
	if (ret < 0)
		DRM_ERROR("failed to set pixel format: %d\n", ret);

	ret = mipi_dsi_dcs_set_pixel_format(jdi->dsi->slave, 0x77);
	if (ret < 0)
		DRM_ERROR("failed to set pixel format: %d\n", ret);

	data = 0xFF;
	ret = mipi_dsi_dcs_write(jdi->dsi, 0x51, &data, 1);
	if (ret < 0)
		DRM_ERROR("failed to set 0x51: %d\n", ret);

	data = 0xFF;
	ret = mipi_dsi_dcs_write(jdi->dsi->slave, 0x51, &data, 1);
	if (ret < 0)
		DRM_ERROR("failed to set 0x51: %d\n", ret);

	data = 0x24;
	ret = mipi_dsi_dcs_write(jdi->dsi, 0x53, &data, 1);
	if (ret < 0)
		DRM_ERROR("failed to set 0x53: %d\n", ret);

	data = 0x24;
	ret = mipi_dsi_dcs_write(jdi->dsi->slave, 0x53, &data, 1);
	if (ret < 0)
		DRM_ERROR("failed to set 0x53: %d\n", ret);

	data = 0x00;
	ret = mipi_dsi_dcs_write(jdi->dsi, 0x55, &data, 1);
	if (ret < 0)
		DRM_ERROR("failed to set 0x55: %d\n", ret);

	data = 0x00;
	ret = mipi_dsi_dcs_write(jdi->dsi->slave, 0x55, &data, 1);
	if (ret < 0)
		DRM_ERROR("failed to set 0x55: %d\n", ret);

	ret = mipi_dsi_dcs_set_display_on(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to set display on: %d\n", ret);

	ret = mipi_dsi_dcs_set_display_on(jdi->dsi->slave);
	if (ret < 0)
		DRM_ERROR("failed to set display on: %d\n", ret);

	jdi->enabled = true;

	ret = panel_jdi_configure_touchscreen(jdi);
	if (ret < 0)
		DRM_ERROR("failed to configure touchscreen: %d\n", ret);

	return ret;
}

static int panel_jdi_enable(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);

	return lp8557_enable_backlight(&jdi->lp8557);
}

static const struct drm_display_mode default_mode = {
	.clock = 301620,
	.hdisplay = 1280,
	.hsync_start = 1280 + 80,
	.hsync_end = 1280 + 80 + 80,
	.htotal = 1280 + 80 + 80 + 1280,
	.vdisplay = 800,
	.vsync_start = 800 + 4,
	.vsync_end = 800 + 4 + 4,
	.vtotal = 800 + 4 + 4 + 1000,

	.crtc_hdisplay = 2560,
	.crtc_hsync_start = 2560 + 80,
	.crtc_hsync_end = 2560 + 80 + 80,
	.crtc_htotal = 2560 + 80 + 80 + 80,
	.crtc_vdisplay = 1800,
	.crtc_vsync_start = 1800 + 4,
	.crtc_vsync_end = 1800 + 4 + 4,
	.crtc_vtotal = 1800 + 4 + 4 + 4,

	.vrefresh = 60,
};

static int panel_jdi_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DRM_ERROR( "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 211;
	panel->connector->display_info.height_mm = 148;

	return 1;
}

static const struct drm_panel_funcs panel_jdi_funcs = {
	.prepare = panel_jdi_prepare,
	.enable = panel_jdi_enable,
	.disable = panel_jdi_disable,
	.unprepare = panel_jdi_unprepare,
	.get_modes = panel_jdi_get_modes,
};

static const struct of_device_id jdi_of_match[] = {
	{ .compatible = "jdi,lpm102a188a", },
	{ }
};
MODULE_DEVICE_TABLE(of, jdi_of_match);

static int panel_jdi_enslave(struct mipi_dsi_device *master,
			struct mipi_dsi_device *slave)
{
	int ret;

	ret = mipi_dsi_attach(master);
	if (ret < 0)
		return ret;

	return ret;
}

static int panel_jdi_liberate(struct mipi_dsi_device *master,
			struct mipi_dsi_device *slave)
{
	int ret;

	ret = mipi_dsi_detach(master);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct mipi_dsi_master_ops panel_jdi_master_ops = {
	.enslave = panel_jdi_enslave,
	.liberate = panel_jdi_liberate,
};

static int lp8557_probe_backlight(struct panel_jdi *jdi)
{
	struct mipi_dsi_device *dsi = jdi->dsi;
	struct backlight_lp8557 *lp8557 = &jdi->lp8557;
	struct device_node *np;
	struct backlight_properties backlight_props;
	int ret, i;
	u32 val, led_enable[6];

	np = of_parse_phandle(dsi->dev.of_node, "backlight", 0);
	if (!np) {
		DRM_ERROR("could not find backlight phandle\n");
		return -ENOENT;
	}

	ret = of_property_read_u32(np, "brightness-mode", &val);
	if (ret || val >= LP8557_CONFIG_BRTMODE_MAX) {
		DRM_ERROR("invalid brightness mode %d,%d\n", ret, val);
		return -EINVAL;
	}
	lp8557->brightness_mode = val;

	lp8557->pwm_standby = of_property_read_bool(np, "pwm-standby");

	ret = of_property_read_u32(np, "current", &val);
	if (ret == -EINVAL) {
		lp8557->current_use_iset = true;
	} else if (ret || val >= LP8557_CURRENT_MAX) {
		DRM_ERROR("invalid current %d,%d\n", ret, val);
		return -EINVAL;
	} else {
		lp8557->bl_current = val;
	}

	ret = of_property_read_u32(np, "pwm-frequency", &val);
	if (ret == -EINVAL) {
		lp8557->pwm_use_fset = true;
	} else if (ret || val >= LP8557_PGEN_FREQ_MAX) {
		DRM_ERROR("invalid pwmfreq %d,%d\n", ret, val);
		return -EINVAL;
	} else {
		lp8557->pwm_frequency = val;
	}

	ret = of_property_read_u32(np, "boost-frequency", &val);
	if (ret == -EINVAL) {
		lp8557->boost_use_fset = true;
	} else if (ret || val >= LP8557_BOOST_FREQ_MAX) {
		DRM_ERROR("invalid boost %d,%d\n", ret, val);
		return -EINVAL;
	} else {
		lp8557->boost_frequency = val;
	}

	ret = of_property_read_u32(np, "boost-bcomp", &val);
	if (ret == -EINVAL) {
		lp8557->boost_use_iset = true;
	} else if (ret || val >= LP8557_BOOST_BCOMP_MAX) {
		DRM_ERROR("invalid bcomp %d,%d\n", ret, val);
		return -EINVAL;
	} else {
		lp8557->boost_bcomp = val;
	}

	ret = of_property_read_u32_array(np, "led-enable", led_enable,
			ARRAY_SIZE(led_enable));
	if (ret == -EINVAL) {
		lp8557->auto_detect_leds = true;
	} else if (ret) {
		DRM_ERROR("invalid led-enable %d\n", ret);
		return -EINVAL;
	} else {
		for (i = 0; i < ARRAY_SIZE(led_enable); i++)
			lp8557->led_enable_mask |= led_enable[i] ? 1 << i : 0;
	}

	ret = of_property_read_u32(np, "step-ramp", &val);
	if (ret || val >= LP8557_STEP_RAMP_MAX) {
		DRM_ERROR("invalid step ramp %d,%d\n", ret, val);
		return -EINVAL;
	}
	lp8557->step_ramp = val;

	ret = of_property_read_u32(np, "step-smoothing", &val);
	if (ret || val >= LP8557_STEP_SMOOTHING_MAX) {
		DRM_ERROR("invalid step smoothing %d,%d\n", ret, val);
		return -EINVAL;
	}
	lp8557->step_smoothing = val;

	lp8557->client = of_find_i2c_device_by_node(np);
	if (!lp8557->client) {
		DRM_ERROR("could not get i2c client from bl node\n");
		ret = -ENODEV;
		goto err_node_put;
	}

	memset(&backlight_props, 0, sizeof(struct backlight_properties));
	backlight_props.type = BACKLIGHT_RAW;
	backlight_props.max_brightness = LP8557_MAX_BRIGHTNESS;
	backlight_props.brightness = 0;

	lp8557->bl_dev = backlight_device_register("lp8557_backlight",
				&dsi->dev, lp8557, &lp8557_backlight_ops,
				&backlight_props);
	if (IS_ERR(lp8557->bl_dev)) {
		ret = PTR_ERR(lp8557->bl_dev);
		DRM_ERROR("backlight register failed ret=%d\n", ret);
		goto err_i2c_put;
	}

	of_node_put(np);
	return 0;

err_i2c_put:
	put_device(&lp8557->client->dev);
err_node_put:
	of_node_put(np);

	return ret;
}

static int panel_jdi_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct panel_jdi *jdi;
	enum of_gpio_flags gpio_flags;
	int ret;
	unsigned int value;

	jdi = devm_kzalloc(&dsi->dev, sizeof(*jdi), GFP_KERNEL);
	if (!jdi)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, jdi);
	jdi->mode = &default_mode;
	jdi->dsi = dsi;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = 0;

	dsi->master = mipi_dsi_get_master(dsi);
	if (IS_ERR(dsi->master))
		return PTR_ERR(jdi->dsi->master);

	if (dsi->master) {
		ret = mipi_dsi_attach(dsi);
		if (ret < 0)
			return ret;

		ret = mipi_dsi_enslave(dsi->master, dsi);
		if (ret < 0) {
			DRM_ERROR("mipi_dsi_enslave() failed: %d\n", ret);
			return ret;
		}

		return 0;
	}

	dsi->ops = &panel_jdi_master_ops;

	jdi->supply = devm_regulator_get(&dsi->dev, "power");
	if (IS_ERR(jdi->supply))
		return PTR_ERR(jdi->supply);

	jdi->ddi_supply = devm_regulator_get(&dsi->dev, "ddi");
	if (IS_ERR(jdi->ddi_supply))
		return PTR_ERR(jdi->ddi_supply);

	jdi->enable_gpio = of_get_named_gpio_flags(dsi->dev.of_node,
				"enable-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(jdi->enable_gpio)) {
		DRM_ERROR("enable gpio not found: %d\n", ret);
		return -ENODEV;
	}

	if (gpio_flags & OF_GPIO_ACTIVE_LOW)
		jdi->enable_gpio_flags |= GPIO_ACTIVE_LOW;

	ret = gpio_request(jdi->enable_gpio, "jdi-enable");
	if (ret < 0) {
		DRM_ERROR("Request enable gpio failed: %d\n", ret);
		return ret;
	}

	value = (jdi->enable_gpio_flags & GPIO_ACTIVE_LOW) ? 1 : 0;
	ret = gpio_direction_output(jdi->enable_gpio, value);
	if (ret < 0) {
		DRM_ERROR("Set enable gpio direction failed: %d\n", ret);
		goto err_gpio_enable;
	}

	jdi->reset_gpio = of_get_named_gpio_flags(dsi->dev.of_node,
				"reset-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(jdi->reset_gpio)) {
		DRM_ERROR("reset gpio not found: %d\n", ret);
		ret = -ENODEV;
		goto err_gpio_enable;
	}

	if (gpio_flags & OF_GPIO_ACTIVE_LOW)
		jdi->reset_gpio_flags |= GPIO_ACTIVE_LOW;

	ret = gpio_request(jdi->reset_gpio, "jdi-reset");
	if (ret < 0) {
		DRM_ERROR("Request reset gpio failed: %d\n", ret);
		goto err_gpio_enable;
	}

	value = (jdi->reset_gpio_flags & GPIO_ACTIVE_LOW) ? 0 : 1;
	ret = gpio_direction_output(jdi->reset_gpio, value);
	if (ret < 0) {
		DRM_ERROR("Set enable gpio direction failed: %d\n", ret);
		goto err_gpio_reset;
	}

	ret = lp8557_probe_backlight(jdi);
	if (ret) {
		DRM_ERROR("failed to probe backlight, ret=%d\n", ret);
		goto err_gpio_reset;
	}

	drm_panel_init(&jdi->base);
	jdi->base.dev = &dsi->dev;
	jdi->base.funcs = &panel_jdi_funcs;

	ret = drm_panel_add(&jdi->base);
	if (ret < 0) {
		DRM_ERROR("drm_panel_add failed: %d\n", ret);
		goto err_bl_unregister;
	}

	return 0;

err_bl_unregister:
	backlight_device_unregister(jdi->lp8557.bl_dev);
err_gpio_reset:
	if (gpio_is_valid(jdi->reset_gpio))
		gpio_free(jdi->reset_gpio);
err_gpio_enable:
	if (gpio_is_valid(jdi->enable_gpio))
		gpio_free(jdi->enable_gpio);

	return ret;
}

static int panel_jdi_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct panel_jdi *jdi = mipi_dsi_get_drvdata(dsi);
	struct i2c_client *client = jdi->lp8557.client;
	int ret;

	panel_jdi_disable(&jdi->base);

	drm_panel_detach(&jdi->base);
	drm_panel_remove(&jdi->base);

	backlight_device_unregister(jdi->lp8557.bl_dev);

	put_device(&client->dev);

	if (gpio_is_valid(jdi->reset_gpio))
		gpio_free(jdi->reset_gpio);

	if (gpio_is_valid(jdi->enable_gpio))
		gpio_free(jdi->enable_gpio);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_ERROR("failed to detach from DSI host: %d\n", ret);

	put_device(&dsi->slave->dev);

	return 0;
}

static void panel_jdi_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	struct panel_jdi *jdi = mipi_dsi_get_drvdata(dsi);

	panel_jdi_disable(&jdi->base);
}

static struct mipi_dsi_driver panel_jdi_dsi_driver = {
	.driver = {
		.name = "panel-jdi-lpm102a188a-dsi",
		.of_match_table = jdi_of_match,
	},
	.probe = panel_jdi_dsi_probe,
	.remove = panel_jdi_dsi_remove,
	.shutdown = panel_jdi_dsi_shutdown,
};
module_mipi_dsi_driver(panel_jdi_dsi_driver);
MODULE_AUTHOR("Sean Paul <seanpaul@chromium.org>");
MODULE_DESCRIPTION("DRM Driver for JDI LPM102A188A");
MODULE_LICENSE("GPL and additional rights");
