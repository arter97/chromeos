/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __DW_HDMI__
#define __DW_HDMI__

#include <drm/drmP.h>

enum {
	DW_HDMI_RES_8,
	DW_HDMI_RES_10,
	DW_HDMI_RES_12,
	DW_HDMI_RES_MAX,
};

enum {
	DW_HDMI_HDCP_KSV_LEN = 8,
	DW_HDMI_HDCP_SHA_LEN = 20,
	DW_HDMI_HDCP_DPK_LEN = 280,
	DW_HDMI_HDCP_KEY_LEN = 308,
	DW_HDMI_HDCP_SEED_LEN = 2,
};

struct dw_hdmi;

enum dw_hdmi_devtype {
	IMX6Q_HDMI,
	IMX6DL_HDMI,
	RK3288_HDMI,
};

struct dw_hdmi_audio_tmds_n {
	unsigned long tmds;
	unsigned int n_32k;
	unsigned int n_44k1;
	unsigned int n_48k;
};

struct dw_hdmi_mpll_config {
	unsigned long mpixelclock;
	struct {
		u16 cpce;
		u16 gmp;
	} res[DW_HDMI_RES_MAX];
};

struct dw_hdmi_curr_ctrl {
	unsigned long mpixelclock;
	u16 curr[DW_HDMI_RES_MAX];
};

struct dw_hdmi_phy_config {
	unsigned long mpixelclock;
	u16 sym_ctr;    /*clock symbol and transmitter control*/
	u16 term;       /*transmission termination value*/
	u16 vlev_ctr;   /* voltage level control */
};

typedef void (*dw_hdmi_audio_plugged_fn)(struct platform_device *audio_pdev,
					 bool plugged);

struct dw_hdmi_audio_data {
	struct dw_hdmi *dw;

	void (*set_plugged_callback)(struct dw_hdmi *hdmi,
				     dw_hdmi_audio_plugged_fn fn);

	u8 (*read)(struct dw_hdmi *hdmi, int offset);
	void (*write)(struct dw_hdmi *hdmi, u8 val, int offset);
	void (*mod)(struct dw_hdmi *hdmi, u8 data, u8 mask, unsigned reg);

	/* NOTE: enable/disable may be called with IRQs disabled */
	void (*enable)(struct dw_hdmi *hdmi);
	void (*disable)(struct dw_hdmi *hdmi);

	void (*set_sample_rate)(struct dw_hdmi *hdmi, unsigned int rate);
};

struct dw_hdmi_hdcp_key_1x {
	u8 ksv[DW_HDMI_HDCP_KSV_LEN];
	u8 device_key[DW_HDMI_HDCP_DPK_LEN];
	u8 sha1[DW_HDMI_HDCP_SHA_LEN];
	u8 seed[DW_HDMI_HDCP_SEED_LEN];
};

struct dw_hdmi_plat_data {
	enum dw_hdmi_devtype dev_type;
	const struct dw_hdmi_audio_tmds_n *tmds_n_table;
	const struct dw_hdmi_mpll_config *mpll_cfg;
	const struct dw_hdmi_curr_ctrl *cur_ctr;
	const struct dw_hdmi_phy_config *phy_config;
	enum drm_mode_status (*mode_valid)(struct drm_connector *connector,
					   struct drm_display_mode *mode);
};

int dw_hdmi_resume(struct device *dev);
int dw_hdmi_suspend(struct device *dev);
void dw_hdmi_unbind(struct device *dev, struct device *master, void *data);
int dw_hdmi_bind(struct device *dev, struct device *master,
		 void *data, struct drm_encoder *encoder,
		 struct resource *iores, int irq,
		 const struct dw_hdmi_plat_data *plat_data);
int dw_hdmi_config_hdcp_key(struct device *dev,
			    const struct dw_hdmi_hdcp_key_1x *keys);
#endif /* __IMX_HDMI_H__ */
