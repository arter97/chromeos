/*
 * Copyright (c) 2014, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/crc16.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <drm/drm_of.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>
#include <drm/bridge/dw_hdmi.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_vop.h"

#include "../../../../arch/arm/mach-rockchip/efuse.h"

#define GRF_SOC_CON6                    0x025c
#define HDMI_SEL_VOP_LIT                (1 << 4)

struct rockchip_hdmi {
	struct device *dev;
	struct regmap *regmap;
	struct drm_encoder encoder;
};

#define CLK_SLOP(clk)		((clk) / 333)
#define CLK_PLUS_SLOP(clk)	((clk) + CLK_SLOP(clk))

#define to_rockchip_hdmi(x)	container_of(x, struct rockchip_hdmi, x)

static const int dw_hdmi_rates[] = {
	25170732,	/* for 25.175 MHz, 0.017% off */
	25200000,
	27000000,
	28320000,
	30240000,
	31500000,
	32000000,
	33750000,
	36000000,
	40000000,
	49500000,
	50000000,
	54000000,
	57290323,	/* for 57.284 MHz, .011 % off */
	65000000,
	68250000,
	71000000,
	72000000,
	73263158,	/* for 73.25 MHz, .018% off */
	74250000,
	74400000,	/* for 74.44 MHz, .054% off */
	75000000,
	78720000,	/* for 78.75 MHz, .038% off */
	79500000,
	83520000,	/* for 83.5 MHz,  .024% off */
	85500000,
	88695653,	/* for 88.75 MHz, .061% off */
	97714285,	/* for 97.75 MHz, .037% off */
	101052632,	/* for 101.0 MHz, .052% off */
	106500000,
	108000000,
	115500000,
	118666667,	/* for 118.68 MHz, .011% off */
	119000000,
	121714286,	/* for 121.75 MHz, .029% off */
	135000000,
	136800000,	/* for 136.75 MHz, .037% off */
	146250000,
	148500000,
	154000000,
	162000000,
};

/*
 * There are some rates that would be ranged for better clock jitter at
 * Chrome OS tree, like 25.175Mhz would range to 25.170732Mhz. But due
 * to the clock is aglined to KHz in struct drm_display_mode, this would
 * bring some inaccurate error if we still run the compute_n math, so
 * let's just code an const table for it until we can actually get the
 * right clock rate.
 */
static const struct dw_hdmi_audio_tmds_n rockchip_werid_tmds_n_table[] = {
	/* 25170732 for 25.175 MHz */
	{ .tmds = 25171000, .n_32k = 5248, .n_44k1 = 6027, .n_48k = 6560, },
	/* 57290323 for 57.284 MHz */
	{ .tmds = 57291000, .n_32k = 3968, .n_44k1 = 4557, .n_48k = 5952, },
	/* 73263158 for 73.25 MHz */
	{ .tmds = 73264000, .n_32k = 4256, .n_44k1 = 5586, .n_48k = 6080, },
	/* 74400000 for 74.44 MHz */
	{ .tmds = 74400000, .n_32k = 4096, .n_44k1 = 5586, .n_48k = 6144, },
	/* 78720000 for 78.75 MHz */
	{ .tmds = 78720000, .n_32k = 4096, .n_44k1 = 5586, .n_48k = 6144, },
	/* 83520000 for 83.5 MHz */
	{ .tmds = 83520000, .n_32k = 4096, .n_44k1 = 5635, .n_48k = 6144, },
	/* 88695653 for 88.75 MHz */
	{ .tmds = 88696000, .n_32k = 4416, .n_44k1 = 6762, .n_48k = 5888, },
	/* 97714285 for 97.75 MHz */
	{ .tmds = 97715000, .n_32k = 4480, .n_44k1 = 5488, .n_48k = 6272, },
	/* 101052632 for 101.00 MHz */
	{ .tmds = 101053000, .n_32k = 4104, .n_44k1 = 5586, .n_48k = 6156, },
	/* 118666667 for 118.68 MHz */
	{ .tmds = 118667000, .n_32k = 4224, .n_44k1 = 5292, .n_48k = 6336, },
	/* 121714286 for 121.75 MHz */
	{ .tmds = 121715000, .n_32k = 4480, .n_44k1 = 6174, .n_48k = 6272, },
	/* 136800000 for 136.75 MHz */
	{ .tmds = 136800000, .n_32k = 4096, .n_44k1 = 5684, .n_48k = 6144, },
	/* End of table */
	{ .tmds = 0,         .n_32k = 0,    .n_44k1 = 0,    .n_48k = 0, },
};

static const struct dw_hdmi_mpll_config rockchip_mpll_cfg[] = {
	{
		30666000, {
			{ 0x00b3, 0x0000 },
			{ 0x2153, 0x0000 },
			{ 0x40f3, 0x0000 },
		},
	},  {
		36800000, {
			{ 0x00b3, 0x0000 },
			{ 0x2153, 0x0000 },
			{ 0x40a2, 0x0001 },
		},
	},  {
		46000000, {
			{ 0x00b3, 0x0000 },
			{ 0x2142, 0x0001 },
			{ 0x40a2, 0x0001 },
		},
	},  {
		61333000, {
			{ 0x0072, 0x0001 },
			{ 0x2142, 0x0001 },
			{ 0x40a2, 0x0001 },
		},
	},  {
		73600000, {
			{ 0x0072, 0x0001 },
			{ 0x2142, 0x0001 },
			{ 0x4061, 0x0002 },
		},
	},  {
		92000000, {
			{ 0x0072, 0x0001 },
			{ 0x2145, 0x0002 },
			{ 0x4061, 0x0002 },
		},
	},  {
		122666000, {
			{ 0x0051, 0x0002 },
			{ 0x2145, 0x0002 },
			{ 0x4061, 0x0002 },
		},
	},  {
		147200000, {
			{ 0x0051, 0x0002 },
			{ 0x2145, 0x0002 },
			{ 0x4064, 0x0003 },
		},
	},  {
		184000000, {
			{ 0x0051, 0x0002 },
			{ 0x214c, 0x0003 },
			{ 0x4064, 0x0003 },
		},
	},  {
		226666000, {
			{ 0x0040, 0x0003 },
			{ 0x214c, 0x0003 },
			{ 0x4064, 0x0003 },
		},
	},  {
		272000000, {
			{ 0x0040, 0x0003 },
			{ 0x214c, 0x0003 },
			{ 0x5a64, 0x0003 },
		},
	},  {
		340000000, {
			{ 0x0040, 0x0003 },
			{ 0x3b4c, 0x0003 },
			{ 0x5a64, 0x0003 },
		},
	},  {
		600000000, {
			{ 0x1a40, 0x0003 },
			{ 0x3b4c, 0x0003 },
			{ 0x5a64, 0x0003 },
		},
	},  {
		~0UL, {
			{ 0x0000, 0x0000 },
			{ 0x0000, 0x0000 },
			{ 0x0000, 0x0000 },
		},
	}
};

static const struct dw_hdmi_curr_ctrl rockchip_cur_ctr[] = {
	/*      pixelclk     bpp8    bpp10   bpp12 */
	{
		600000000, { 0x0000, 0x0000, 0x0000 },
	},  {
		~0UL,      { 0x0000, 0x0000, 0x0000 },
	},
};

static const struct dw_hdmi_phy_config rockchip_phy_config[] = {
	/*pixelclk   symbol   term   vlev*/
	{ CLK_PLUS_SLOP(74250000),  0x8009, 0x0004, 0x0272},
	{ CLK_PLUS_SLOP(165000000), 0x802b, 0x0004, 0x0209},
	{ CLK_PLUS_SLOP(297000000), 0x8039, 0x0005, 0x028d},
	{ ~0UL,	                    0x0000, 0x0000, 0x0000}
};

static int rockchip_hdmi_parse_dt(struct rockchip_hdmi *hdmi)
{
	struct device_node *np = hdmi->dev->of_node;

	hdmi->regmap = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(hdmi->regmap)) {
		dev_err(hdmi->dev, "Unable to get rockchip,grf\n");
		return PTR_ERR(hdmi->regmap);
	}

	return 0;
}

static enum drm_mode_status
dw_hdmi_rockchip_mode_valid(struct drm_connector *connector,
			    struct drm_display_mode *mode)
{
	int pclk = mode->clock * 1000;
	int num_rates = ARRAY_SIZE(dw_hdmi_rates);
	int i;

	/*
	 * Pixel clocks we support are always < 2GHz and so fit in an
	 * int.  We should make sure source rate does too so we don't get
	 * overflow when we multiply by 1000.
	 */
	if (mode->clock > INT_MAX / 1000)
		return MODE_BAD;

	/* HACK: Modes > 3840x2160 pixels can't work on the VOP; filter them. */
	if (mode->hdisplay > 3840 || mode->vdisplay > 2160)
		return MODE_BAD;

	for (i = 0; i < num_rates; i++) {
		int slop = CLK_SLOP(pclk);

		if ((pclk >= dw_hdmi_rates[i] - slop) &&
		    (pclk <= dw_hdmi_rates[i] + slop))
			return MODE_OK;
	}

	return MODE_BAD;
}

static struct drm_encoder_funcs dw_hdmi_rockchip_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static void dw_hdmi_rockchip_encoder_disable(struct drm_encoder *encoder)
{
}

static bool
dw_hdmi_rockchip_encoder_mode_fixup(struct drm_encoder *encoder,
				    const struct drm_display_mode *mode,
				    struct drm_display_mode *adj_mode)
{
	struct rockchip_hdmi *hdmi = to_rockchip_hdmi(encoder);
	int pclk = adj_mode->clock * 1000;
	int best_diff = INT_MAX;
	int best_clock = 0;
	int slop;
	int i;

	/* Pick the best clock */
	for (i = 0; i < ARRAY_SIZE(dw_hdmi_rates); i++) {
		int diff = dw_hdmi_rates[i] - pclk;

		if (diff < 0)
			diff = -diff;
		if (diff < best_diff) {
			best_diff = diff;
			best_clock = dw_hdmi_rates[i];

			/* Bail early if we're exact */
			if (best_diff == 0)
				return true;
		}
	}

	/* Double check that it's OK */
	slop = CLK_SLOP(pclk);
	if ((pclk >= best_clock - slop) && (pclk <= best_clock + slop)) {
		adj_mode->clock = DIV_ROUND_UP(best_clock, 1000);
		return true;
	}

	/* Shoudn't be here; we should have said rate wasn't valid */
	dev_warn(hdmi->dev, "tried to set invalid rate %d\n", adj_mode->clock);
	return false;
}

static void dw_hdmi_rockchip_encoder_mode_set(struct drm_encoder *encoder,
					      struct drm_display_mode *mode,
					      struct drm_display_mode *adj_mode)
{
}

static void dw_hdmi_rockchip_encoder_commit(struct drm_encoder *encoder)
{
	struct rockchip_hdmi *hdmi = to_rockchip_hdmi(encoder);
	u32 val;
	int mux;

	mux = rockchip_drm_encoder_get_mux_id(hdmi->dev->of_node, encoder);
	if (mux)
		val = HDMI_SEL_VOP_LIT | (HDMI_SEL_VOP_LIT << 16);
	else
		val = HDMI_SEL_VOP_LIT << 16;

	regmap_write(hdmi->regmap, GRF_SOC_CON6, val);
	dev_dbg(hdmi->dev, "vop %s output to hdmi\n",
		(mux) ? "LIT" : "BIG");
}

static void dw_hdmi_rockchip_encoder_prepare(struct drm_encoder *encoder)
{
	rockchip_drm_crtc_mode_config(encoder->crtc, DRM_MODE_CONNECTOR_HDMIA,
				      ROCKCHIP_OUT_MODE_AAAA);
}

static struct drm_encoder_helper_funcs dw_hdmi_rockchip_encoder_helper_funcs = {
	.mode_fixup = dw_hdmi_rockchip_encoder_mode_fixup,
	.mode_set   = dw_hdmi_rockchip_encoder_mode_set,
	.prepare    = dw_hdmi_rockchip_encoder_prepare,
	.commit     = dw_hdmi_rockchip_encoder_commit,
	.disable    = dw_hdmi_rockchip_encoder_disable,
};

static ssize_t hdcp_key_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct dw_hdmi_hdcp_key_1x *key;
	struct device_node *efuse_np;
	struct platform_device *efuse;
	char uid[EFUSE_CHIP_UID_LEN];
	u16 encrypt_seed;
	int retval;

	key = kzalloc(sizeof(*key), GFP_KERNEL);
	if (IS_ERR(key))
		return -ENOMEM;

	/*
	 * The HDCP Key format should look like this: "12345678...",
	 * every two charactor stand for a byte, so the total key size
	 * would be (308 * 2) byte.
	 */
	if (count < (DW_HDMI_HDCP_KEY_LEN * 2))
		goto err_key_format;

	efuse_np = of_parse_phandle(dev->of_node, "rockchip,efuse", 0);
	if (!efuse_np) {
		dev_err(dev, "missing rockchip,efuse property\n");
		goto err_key_format;
	}

	efuse = of_find_device_by_node(efuse_np);
	of_node_put(efuse_np);

	if (!efuse) {
		dev_err(dev, "couldn't find efuse device\n");
		goto err_key_format;
	}

	retval = rockchip_efuse_get_uid(efuse, uid);
	platform_device_put(efuse);

	if (retval) {
		dev_err(dev, "failed to read efuse cpu uid\n");
		goto err_key_format;
	}

	/*
	 * The format of input HDCP Key should be "12345678...".
	 * there is no standard format for HDCP keys, so it is
	 * just made up for this driver.
	 *
	 * The "ksv & device_key & sha" should parsed from input data
	 * buffer, and the "seed" would take the crc16 of cpu uid.
	 */
	retval = hex2bin((u8 *)key, buf, DW_HDMI_HDCP_KEY_LEN);
	if (retval) {
		dev_err(dev, "Failed to decode the input HDCP key format\n");
		goto err_key_format;
	}

	encrypt_seed = crc16(0xFFFF, uid, EFUSE_CHIP_UID_LEN);
	key->seed[0] = encrypt_seed & 0xFF;
	key->seed[1] = (encrypt_seed >> 8) & 0xFF;

	retval = dw_hdmi_config_hdcp_key(dev, key);
	if (retval)
		goto err_key_format;

	kfree(key);

	return DW_HDMI_HDCP_KEY_LEN * 2;

err_key_format:
	kfree(key);

	return -EINVAL;
}
static DEVICE_ATTR(hdcp_key, S_IWUSR, NULL, hdcp_key_store);

static const struct dw_hdmi_plat_data rockchip_hdmi_drv_data = {
	.mode_valid = dw_hdmi_rockchip_mode_valid,
	.mpll_cfg   = rockchip_mpll_cfg,
	.cur_ctr    = rockchip_cur_ctr,
	.phy_config = rockchip_phy_config,
	.dev_type   = RK3288_HDMI,
	.tmds_n_table = rockchip_werid_tmds_n_table,
};

static const struct of_device_id dw_hdmi_rockchip_ids[] = {
	{ .compatible = "rockchip,rk3288-dw-hdmi",
	  .data = &rockchip_hdmi_drv_data
	},
	{},
};
MODULE_DEVICE_TABLE(of, dw_hdmi_rockchip_dt_ids);

static int dw_hdmi_rockchip_bind(struct device *dev, struct device *master,
				 void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct dw_hdmi_plat_data *plat_data;
	const struct of_device_id *match;
	struct drm_device *drm = data;
	struct drm_encoder *encoder;
	struct rockchip_hdmi *hdmi;
	struct resource *iores;
	int irq;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	match = of_match_node(dw_hdmi_rockchip_ids, pdev->dev.of_node);
	plat_data = match->data;
	hdmi->dev = &pdev->dev;
	encoder = &hdmi->encoder;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iores)
		return -ENXIO;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);
	/*
	 * If we failed to find the CRTC(s) which this encoder is
	 * supposed to be connected to, it's because the CRTC has
	 * not been registered yet.  Defer probing, and hope that
	 * the required CRTC is added later.
	 */
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	ret = rockchip_hdmi_parse_dt(hdmi);
	if (ret) {
		dev_err(hdmi->dev, "Unable to parse OF data\n");
		return ret;
	}

	device_create_file(dev, &dev_attr_hdcp_key);

	drm_encoder_helper_add(encoder, &dw_hdmi_rockchip_encoder_helper_funcs);
	drm_encoder_init(drm, encoder, &dw_hdmi_rockchip_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS);

	ret = dw_hdmi_bind(dev, master, data, encoder, iores, irq, plat_data);

	/*
	 * If dw_hdmi_bind() fails we'll never call dw_hdmi_unbind(),
	 * which would have called the encoder cleanup.  Do it manually.
	 */
	if (ret)
		drm_encoder_cleanup(encoder);

	return ret;
}

static void dw_hdmi_rockchip_unbind(struct device *dev, struct device *master,
				    void *data)
{
	return dw_hdmi_unbind(dev, master, data);
}

static const struct component_ops dw_hdmi_rockchip_ops = {
	.bind	= dw_hdmi_rockchip_bind,
	.unbind	= dw_hdmi_rockchip_unbind,
};

static int dw_hdmi_rockchip_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &dw_hdmi_rockchip_ops);
}

static int dw_hdmi_rockchip_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dw_hdmi_rockchip_ops);

	return 0;
}

static int dw_hdmi_rockchip_suspend(struct device *dev)
{
	return dw_hdmi_suspend(dev);
}

static int dw_hdmi_rockchip_resume(struct device *dev)
{
	return dw_hdmi_resume(dev);
}

static const struct dev_pm_ops dw_hdmi_rockchip_pm = {
	.resume_early = dw_hdmi_rockchip_resume,
	.suspend_late = dw_hdmi_rockchip_suspend,
};

static struct platform_driver dw_hdmi_rockchip_pltfm_driver = {
	.probe  = dw_hdmi_rockchip_probe,
	.remove = dw_hdmi_rockchip_remove,
	.driver = {
		.name = "dwhdmi-rockchip",
		.pm = &dw_hdmi_rockchip_pm,
		.of_match_table = dw_hdmi_rockchip_ids,
	},
};

module_platform_driver(dw_hdmi_rockchip_pltfm_driver);

MODULE_AUTHOR("Andy Yan <andy.yan@rock-chips.com>");
MODULE_AUTHOR("Yakir Yang <ykk@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip Specific DW-HDMI Driver Extension");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dwhdmi-rockchip");
