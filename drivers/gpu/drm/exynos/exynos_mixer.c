/*
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors:
 * Seung-Woo Kim <sw0312.kim@samsung.com>
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * Based on drivers/media/video/s5p-tv/mixer_reg.c
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

#include "regs-mixer.h"
#include "regs-vp.h"

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include <drm/exynos_drm.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_gem.h"
#include "exynos_drm_iommu.h"
#include "exynos_mixer.h"

#define exynos_plane_to_win_idx(ctx, x) (unsigned int)(x - ctx->planes)
#define get_mixer_context(dev)	platform_get_drvdata(to_platform_device(dev))

#define MIXER_WIN_NR		3
#define MIXER_DEFAULT_WIN	0

static const uint32_t plane_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV12MT,
};

enum mixer_version_id {
	MXR_VER_0_0_0_16 = 1 << 0,
	MXR_VER_16_0_33_0 = 1 << 1,
	MXR_VER_128_0_0_184 = 1 << 2,
};

struct mixer_resources {
	int			irq;
	void __iomem		*mixer_regs;
	void __iomem		*vp_regs;
	spinlock_t		reg_slock;
	struct clk		*mixer;
	struct clk		*vp;
	struct clk		*sclk_mixer;
	struct clk		*sclk_hdmi;
	struct clk		*sclk_dac;
};

enum workaround_state {
	WORKAROUND_STATE_INACTIVE,
	WORKAROUND_STATE_INIT,
	WORKAROUND_STATE_ACTIVE,
};

struct mixer_context {
	struct device		*dev;
	struct drm_device	*drm_dev;
	struct exynos_drm_plane	planes[MIXER_WIN_NR];
	bool			plane_updated[MIXER_WIN_NR];
	int			pipe;
	bool			interlace;
	bool			powered;
	bool			vp_enabled;
	u32			previous_dxy;

	struct mixer_resources	mixer_res;
	enum mixer_version_id	mxr_ver;
	wait_queue_head_t	wait_vsync_queue;
	atomic_t		wait_vsync_event;
	atomic_t		vblank_ref;
	enum workaround_state	workaround_state;
};

struct mixer_drv_data {
	enum mixer_version_id	version;
	bool					is_vp_enabled;
};

static const u8 filter_y_horiz_tap8[] = {
	0,	-1,	-1,	-1,	-1,	-1,	-1,	-1,
	-1,	-1,	-1,	-1,	-1,	0,	0,	0,
	0,	2,	4,	5,	6,	6,	6,	6,
	6,	5,	5,	4,	3,	2,	1,	1,
	0,	-6,	-12,	-16,	-18,	-20,	-21,	-20,
	-20,	-18,	-16,	-13,	-10,	-8,	-5,	-2,
	127,	126,	125,	121,	114,	107,	99,	89,
	79,	68,	57,	46,	35,	25,	16,	8,
};

static const u8 filter_y_vert_tap4[] = {
	0,	-3,	-6,	-8,	-8,	-8,	-8,	-7,
	-6,	-5,	-4,	-3,	-2,	-1,	-1,	0,
	127,	126,	124,	118,	111,	102,	92,	81,
	70,	59,	48,	37,	27,	19,	11,	5,
	0,	5,	11,	19,	27,	37,	48,	59,
	70,	81,	92,	102,	111,	118,	124,	126,
	0,	0,	-1,	-1,	-2,	-3,	-4,	-5,
	-6,	-7,	-8,	-8,	-8,	-8,	-6,	-3,
};

static const u8 filter_cr_horiz_tap4[] = {
	0,	-3,	-6,	-8,	-8,	-8,	-8,	-7,
	-6,	-5,	-4,	-3,	-2,	-1,	-1,	0,
	127,	126,	124,	118,	111,	102,	92,	81,
	70,	59,	48,	37,	27,	19,	11,	5,
};

enum exynos_mixer_mode_type {
	EXYNOS_MIXER_MODE_INVALID,
	EXYNOS_MIXER_MODE_SD_NTSC,
	EXYNOS_MIXER_MODE_SD_PAL,
	EXYNOS_MIXER_MODE_HD_720,
	EXYNOS_MIXER_MODE_SXGA,
	EXYNOS_MIXER_MODE_WXGA,
	EXYNOS_MIXER_MODE_UXGA,
	EXYNOS_MIXER_MODE_HD_1080,
};

const char *mixer_mode_type_name(enum exynos_mixer_mode_type type)
{
	switch (type) {
	case EXYNOS_MIXER_MODE_INVALID:
		return "Invalid";
	case EXYNOS_MIXER_MODE_SD_NTSC:
		return "SD_NTSC";
	case EXYNOS_MIXER_MODE_SD_PAL:
		return "SD_PAL";
	case EXYNOS_MIXER_MODE_HD_720:
		return "HD_720";
	case EXYNOS_MIXER_MODE_SXGA:
		return "SXGA";
	case EXYNOS_MIXER_MODE_WXGA:
		return "WXGA";
	case EXYNOS_MIXER_MODE_UXGA:
		return "UXGA";
	case EXYNOS_MIXER_MODE_HD_1080:
		return "HD_1080";
	default:
		return "?";
	}
}

struct mixer_scan_range {
	int min_res[2], max_res[2];
	enum exynos_mixer_mode_type mode_type;
	enum mixer_version_id m_ver;
};

struct mixer_scan_adjustment {
	int res[2], new_res[2];
	enum mixer_version_id m_ver;
};

const struct mixer_scan_range scan_ranges[] = {
	{
		.min_res = { 464, 0 },
		.max_res = { 720, 480 },
		.mode_type = EXYNOS_MIXER_MODE_SD_NTSC,
		.m_ver = MXR_VER_16_0_33_0 | MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 464, 481 },
		.max_res = { 720, 576 },
		.mode_type = EXYNOS_MIXER_MODE_SD_PAL,
		.m_ver = MXR_VER_16_0_33_0 | MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 800, 600 },
		.max_res = { 800, 600 },
		.mode_type = EXYNOS_MIXER_MODE_HD_1080,
		.m_ver = MXR_VER_16_0_33_0 | MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1024, 0 },
		.max_res = { 1280, 720 },
		.mode_type = EXYNOS_MIXER_MODE_HD_720,
		.m_ver = MXR_VER_16_0_33_0 | MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1024, 721 },
		.max_res = { 1280, 1024 },
		.mode_type = EXYNOS_MIXER_MODE_SXGA,
		.m_ver = MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1360, 768 },
		.max_res = { 1360, 768 },
		.mode_type = EXYNOS_MIXER_MODE_WXGA,
		.m_ver = MXR_VER_16_0_33_0 | MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1366, 768 },
		.max_res = { 1366, 768 },
		.mode_type = EXYNOS_MIXER_MODE_WXGA,
		.m_ver = MXR_VER_16_0_33_0 | MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1664, 0 },
		.max_res = { 1920, 1080 },
		.mode_type = EXYNOS_MIXER_MODE_HD_1080,
		.m_ver = MXR_VER_16_0_33_0 | MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1440, 900 },
		.max_res = { 1440, 900 },
		.mode_type = EXYNOS_MIXER_MODE_HD_1080,
		.m_ver = MXR_VER_16_0_33_0 | MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1440, 480 },
		.max_res = { 1440, 480 },
		.mode_type = EXYNOS_MIXER_MODE_WXGA,
		.m_ver = MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1440, 576 },
		.max_res = { 1440, 576 },
		.mode_type = EXYNOS_MIXER_MODE_WXGA,
		.m_ver =  MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1600, 900 },
		.max_res = { 1600, 900 },
		.mode_type = EXYNOS_MIXER_MODE_HD_1080,
		.m_ver = MXR_VER_16_0_33_0 | MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1600, 1200 },
		.max_res = { 1600, 1200 },
		.mode_type = EXYNOS_MIXER_MODE_UXGA,
		.m_ver =  MXR_VER_128_0_0_184,
	},
	{
		.min_res = { 1920, 1200 },
		.max_res = { 1920, 1200 },
		.mode_type = EXYNOS_MIXER_MODE_UXGA,
		.m_ver =  MXR_VER_128_0_0_184,
	},
};

const struct mixer_scan_adjustment scan_adjustments[] = {
	{
		.res = { 1024, 768 },
		.new_res = { 1024, 720 },
		.m_ver = MXR_VER_16_0_33_0,
	},
	{
		.res = { 1280, 800 },
		.new_res = { 1280, 720 },
		.m_ver = MXR_VER_16_0_33_0,
	},
	{
		.res = { 1366, 768 },
		.new_res = { 1360, 768 },
		.m_ver = MXR_VER_16_0_33_0 | MXR_VER_128_0_0_184,
	},
};

static inline u32 vp_reg_read(struct mixer_resources *res, u32 reg_id)
{
	return readl(res->vp_regs + reg_id);
}

static inline void vp_reg_write(struct mixer_resources *res, u32 reg_id,
				 u32 val)
{
	writel(val, res->vp_regs + reg_id);
}

static inline void vp_reg_writemask(struct mixer_resources *res, u32 reg_id,
				 u32 val, u32 mask)
{
	u32 old = vp_reg_read(res, reg_id);

	val = (val & mask) | (old & ~mask);
	writel(val, res->vp_regs + reg_id);
}

static inline u32 mixer_reg_read(struct mixer_resources *res, u32 reg_id)
{
	return readl(res->mixer_regs + reg_id);
}

static inline void mixer_reg_write(struct mixer_resources *res, u32 reg_id,
				 u32 val)
{
	writel(val, res->mixer_regs + reg_id);
}

static inline void mixer_reg_writemask(struct mixer_resources *res,
				 u32 reg_id, u32 val, u32 mask)
{
	u32 old = mixer_reg_read(res, reg_id);

	val = (val & mask) | (old & ~mask);
	writel(val, res->mixer_regs + reg_id);
}

enum exynos_mixer_mode_type exynos_mixer_get_mode_type(
		struct mixer_context *ctx, int width, int height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(scan_ranges); i++) {
		const struct mixer_scan_range *range = &scan_ranges[i];

		if (width >= range->min_res[0] && width <= range->max_res[0]
		 && height >= range->min_res[1] && height <= range->max_res[1]
		 && range->m_ver & ctx->mxr_ver)
			return range->mode_type;
	}
	return EXYNOS_MIXER_MODE_INVALID;
}

static void mixer_adjust_mode(void *ctx, struct drm_connector *connector,
				 struct drm_display_mode *mode_to_adjust)
{
	struct mixer_context *mctx = ctx;
	struct drm_connector_helper_funcs *connector_funcs =
		connector->helper_private;
	int i;

	for (i = 0; i < ARRAY_SIZE(scan_adjustments); i++) {
		const struct mixer_scan_adjustment *adj = &scan_adjustments[i];
		if (adj->res[0] == mode_to_adjust->hdisplay &&
		    adj->res[1] == mode_to_adjust->vdisplay &&
		    (adj->m_ver & mctx->mxr_ver)) {
			struct drm_display_mode *mode;

			/*
			 * Make sure the mode resulting from the adjustment is
			 * not already natively supported. This might cause us
			 * to do something stupid like choose a chopped 1280x800
			 * resolution over native 720p.
			 */
			list_for_each_entry(mode, &connector->modes, head) {
				if (adj->new_res[0] == mode->hdisplay &&
				    adj->new_res[1] == mode->vdisplay &&
				    !(mode->private_flags & EXYNOS_MODE_ADJUSTED)) {
					/* do not compare with invalid modes */
					if (connector_funcs->mode_valid &&
						connector_funcs->mode_valid(
							connector,
							mode) != MODE_OK)
							continue;
					return;
				}
			}
			mode_to_adjust->hdisplay = adj->new_res[0];
			mode_to_adjust->vdisplay = adj->new_res[1];
			mode_to_adjust->private_flags |= EXYNOS_MODE_ADJUSTED;
			return;
		}
	}
}

static bool mixer_mode_fixup(void *ctx, const struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode)
{
	struct mixer_context *mctx = ctx;
	enum exynos_mixer_mode_type mode_type;

	mode_type = exynos_mixer_get_mode_type(mctx, mode->hdisplay,
			mode->vdisplay);

	if (mode_type == EXYNOS_MIXER_MODE_INVALID) {
		DRM_INFO("Mode %dx%d unsupported in mixer, failing modeset\n",
				mode->hdisplay, mode->vdisplay);
		return false;
	}

	return true;
}

static void mixer_regs_dump(struct mixer_context *ctx)
{
#define DUMPREG(reg_id) \
do { \
	DRM_DEBUG_KMS(#reg_id " = %08x\n", \
		(u32)readl(ctx->mixer_res.mixer_regs + reg_id)); \
} while (0)

	DUMPREG(MXR_STATUS);
	DUMPREG(MXR_CFG);
	DUMPREG(MXR_INT_EN);
	DUMPREG(MXR_INT_STATUS);

	DUMPREG(MXR_LAYER_CFG);
	DUMPREG(MXR_VIDEO_CFG);

	DUMPREG(MXR_GRAPHIC0_CFG);
	DUMPREG(MXR_GRAPHIC0_BASE);
	DUMPREG(MXR_GRAPHIC0_SPAN);
	DUMPREG(MXR_GRAPHIC0_WH);
	DUMPREG(MXR_GRAPHIC0_SXY);
	DUMPREG(MXR_GRAPHIC0_DXY);

	DUMPREG(MXR_GRAPHIC1_CFG);
	DUMPREG(MXR_GRAPHIC1_BASE);
	DUMPREG(MXR_GRAPHIC1_SPAN);
	DUMPREG(MXR_GRAPHIC1_WH);
	DUMPREG(MXR_GRAPHIC1_SXY);
	DUMPREG(MXR_GRAPHIC1_DXY);
#undef DUMPREG
}

static void vp_regs_dump(struct mixer_context *ctx)
{
#define DUMPREG(reg_id) \
do { \
	DRM_DEBUG_KMS(#reg_id " = %08x\n", \
		(u32) readl(ctx->mixer_res.vp_regs + reg_id)); \
} while (0)

	DUMPREG(VP_ENABLE);
	DUMPREG(VP_SRESET);
	DUMPREG(VP_SHADOW_UPDATE);
	DUMPREG(VP_FIELD_ID);
	DUMPREG(VP_MODE);
	DUMPREG(VP_IMG_SIZE_Y);
	DUMPREG(VP_IMG_SIZE_C);
	DUMPREG(VP_PER_RATE_CTRL);
	DUMPREG(VP_TOP_Y_PTR);
	DUMPREG(VP_BOT_Y_PTR);
	DUMPREG(VP_TOP_C_PTR);
	DUMPREG(VP_BOT_C_PTR);
	DUMPREG(VP_ENDIAN_MODE);
	DUMPREG(VP_SRC_H_POSITION);
	DUMPREG(VP_SRC_V_POSITION);
	DUMPREG(VP_SRC_WIDTH);
	DUMPREG(VP_SRC_HEIGHT);
	DUMPREG(VP_DST_H_POSITION);
	DUMPREG(VP_DST_V_POSITION);
	DUMPREG(VP_DST_WIDTH);
	DUMPREG(VP_DST_HEIGHT);
	DUMPREG(VP_H_RATIO);
	DUMPREG(VP_V_RATIO);

#undef DUMPREG
}

static inline void vp_filter_set(struct mixer_resources *res,
		int reg_id, const u8 *data, unsigned int size)
{
	/* assure 4-byte align */
	BUG_ON(size & 3);
	for (; size; size -= 4, reg_id += 4, data += 4) {
		u32 val = (data[0] << 24) |  (data[1] << 16) |
			(data[2] << 8) | data[3];
		vp_reg_write(res, reg_id, val);
	}
}

static void vp_default_filter(struct mixer_resources *res)
{
	vp_filter_set(res, VP_POLY8_Y0_LL,
		filter_y_horiz_tap8, sizeof(filter_y_horiz_tap8));
	vp_filter_set(res, VP_POLY4_Y0_LL,
		filter_y_vert_tap4, sizeof(filter_y_vert_tap4));
	vp_filter_set(res, VP_POLY4_C0_LL,
		filter_cr_horiz_tap4, sizeof(filter_cr_horiz_tap4));
}

static void mixer_vsync_set_update(struct mixer_context *ctx, bool enable)
{
	struct mixer_resources *res = &ctx->mixer_res;

	/* block update on vsync */
	mixer_reg_writemask(res, MXR_STATUS, enable ?
			MXR_STATUS_SYNC_ENABLE : 0, MXR_STATUS_SYNC_ENABLE);

	if (ctx->vp_enabled)
		vp_reg_write(res, VP_SHADOW_UPDATE, enable ?
			VP_SHADOW_UPDATE_ENABLE : 0);
}

static void mixer_cfg_scan(struct mixer_context *ctx, unsigned int width,
		unsigned int height)
{
	struct mixer_resources *res = &ctx->mixer_res;
	enum exynos_mixer_mode_type mode_type;
	u32 val;

	/* choosing between interlace and progressive mode */
	val = (ctx->interlace ? MXR_CFG_SCAN_INTERLACE :
				MXR_CFG_SCAN_PROGRASSIVE);

	/* choosing between proper HD and SD mode */
	mode_type = exynos_mixer_get_mode_type(ctx, width, height);
	switch (mode_type) {
	case EXYNOS_MIXER_MODE_SD_NTSC:
		val |= MXR_CFG_SCAN_NTSC | MXR_CFG_SCAN_SD;
		break;
	case EXYNOS_MIXER_MODE_SD_PAL:
		val |= MXR_CFG_SCAN_PAL | MXR_CFG_SCAN_SD;
		break;
	case EXYNOS_MIXER_MODE_HD_720:
		val |= MXR_CFG_SCAN_HD_720 | MXR_CFG_SCAN_HD;
		break;
	case EXYNOS_MIXER_MODE_HD_1080:
	case EXYNOS_MIXER_MODE_WXGA:
		val |= MXR_CFG_SCAN_HD_1080 | MXR_CFG_SCAN_HD;
		break;
	case EXYNOS_MIXER_MODE_SXGA:
	case EXYNOS_MIXER_MODE_UXGA:
		break;
	default:
		DRM_ERROR("Invalid config %dx%d\n", width, height);
		return;
	}

	mixer_reg_writemask(res, MXR_CFG, val, MXR_CFG_SCAN_MASK);
}

unsigned mixer_get_horizontal_offset(unsigned width, unsigned height)
{
	if (width == 800 && height == 600)
		return 0x20;
	else if (width == 1440 && height == 900)
		return 0xe0;
	else if (width == 1600 && height == 900)
		return 0x40;
	else if (width == 1360 && height == 768)
		return 0x130;

	return 0;
}

static void mixer_cfg_rgb_fmt(struct mixer_context *ctx, unsigned int height)
{
	struct mixer_resources *res = &ctx->mixer_res;
	u32 val;

	if (height == 480) {
		val = MXR_CFG_RGB601_0_255;
	} else if (height == 576) {
		val = MXR_CFG_RGB601_0_255;
	} else if (height == 720) {
		val = MXR_CFG_RGB709_16_235;
		mixer_reg_write(res, MXR_CM_COEFF_Y,
				(1 << 30) | (94 << 20) | (314 << 10) |
				(32 << 0));
		mixer_reg_write(res, MXR_CM_COEFF_CB,
				(972 << 20) | (851 << 10) | (225 << 0));
		mixer_reg_write(res, MXR_CM_COEFF_CR,
				(225 << 20) | (820 << 10) | (1004 << 0));
	} else if (height == 1080) {
		val = MXR_CFG_RGB709_16_235;
		mixer_reg_write(res, MXR_CM_COEFF_Y,
				(1 << 30) | (94 << 20) | (314 << 10) |
				(32 << 0));
		mixer_reg_write(res, MXR_CM_COEFF_CB,
				(972 << 20) | (851 << 10) | (225 << 0));
		mixer_reg_write(res, MXR_CM_COEFF_CR,
				(225 << 20) | (820 << 10) | (1004 << 0));
	} else {
		val = MXR_CFG_RGB709_16_235;
		mixer_reg_write(res, MXR_CM_COEFF_Y,
				(1 << 30) | (94 << 20) | (314 << 10) |
				(32 << 0));
		mixer_reg_write(res, MXR_CM_COEFF_CB,
				(972 << 20) | (851 << 10) | (225 << 0));
		mixer_reg_write(res, MXR_CM_COEFF_CR,
				(225 << 20) | (820 << 10) | (1004 << 0));
	}

	mixer_reg_writemask(res, MXR_CFG, val, MXR_CFG_RGB_FMT_MASK);
}

static void mixer_cfg_layer(struct drm_plane *plane, bool enable)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct mixer_context *ctx = exynos_plane->ctx;
	struct mixer_resources *res = &ctx->mixer_res;
	u32 val = enable ? ~0 : 0;

	switch (exynos_plane_to_win_idx(ctx, exynos_plane)) {
	case 0:
		mixer_reg_writemask(res, MXR_CFG, val, MXR_CFG_GRP0_ENABLE);
		break;
	case 1:
		mixer_reg_writemask(res, MXR_CFG, val, MXR_CFG_GRP1_ENABLE);
		break;
	case 2:
		if (ctx->vp_enabled) {
			vp_reg_writemask(res, VP_ENABLE, val, VP_ENABLE_ON);
			mixer_reg_writemask(res, MXR_CFG, val,
				MXR_CFG_VP_ENABLE);
		}
		break;
	}
}

static void mixer_run(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;

	mixer_reg_writemask(res, MXR_STATUS, ~0, MXR_STATUS_REG_RUN);

	mixer_regs_dump(ctx);
}

static int mixer_vp_plane_update(struct drm_plane *plane, struct drm_crtc *crtc,
		struct drm_framebuffer *fb)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct mixer_context *ctx = exynos_plane->ctx;
	struct mixer_resources *res = &ctx->mixer_res;
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);
	dma_addr_t luma_addr[2], chroma_addr[2];
	unsigned long flags;
	unsigned int x_ratio, y_ratio;
	bool tiled_mode = false;
	u32 val;

	/* TODO: single buffer format NV12, NV21 */
	if (fb->pixel_format == DRM_FORMAT_NV12MT) {
		tiled_mode = true;
	} else if (fb->pixel_format != DRM_FORMAT_NV12) {
		DRM_ERROR("pixel format for vp is wrong [%d].\n",
				fb->pixel_format);
		return -EINVAL;
	}

	if (WARN_ON(exynos_drm_fb_get_buf_cnt(exynos_fb) != 2))
		return -EINVAL;

	luma_addr[0] = exynos_drm_fb_buffer(exynos_fb, 0)->dma_addr;
	chroma_addr[0] = exynos_drm_fb_buffer(exynos_fb, 1)->dma_addr;

	/* scaling feature: (src << 16) / dst */
	x_ratio = (exynos_plane->src_w << 16) / exynos_plane->crtc_w;
	y_ratio = (exynos_plane->src_h << 16) / exynos_plane->crtc_h;

	if (crtc->mode.flags & DRM_MODE_FLAG_INTERLACE) {
		ctx->interlace = true;
		if (tiled_mode) {
			luma_addr[1] = luma_addr[0] + 0x40;
			chroma_addr[1] = chroma_addr[0] + 0x40;
		} else {
			luma_addr[1] = luma_addr[0] + fb->width;
			chroma_addr[1] = chroma_addr[0] + fb->height;
		}
	} else {
		ctx->interlace = false;
		luma_addr[1] = 0;
		chroma_addr[1] = 0;
	}

	spin_lock_irqsave(&res->reg_slock, flags);
	mixer_vsync_set_update(ctx, false);

	/* interlace or progressive scan mode */
	val = (ctx->interlace ? ~0 : 0);
	vp_reg_writemask(res, VP_MODE, val, VP_MODE_LINE_SKIP);

	/* setup format */
	val = VP_MODE_NV12;
	val |= (tiled_mode ? VP_MODE_MEM_TILED : VP_MODE_MEM_LINEAR);
	vp_reg_writemask(res, VP_MODE, val, VP_MODE_FMT_MASK);

	/* setting size of input image */
	vp_reg_write(res, VP_IMG_SIZE_Y, VP_IMG_HSIZE(fb->width) |
		VP_IMG_VSIZE(fb->height));
	/* chroma height has to reduced by 2 to avoid chroma distorions */
	vp_reg_write(res, VP_IMG_SIZE_C, VP_IMG_HSIZE(fb->width) |
		VP_IMG_VSIZE(fb->height / 2));

	vp_reg_write(res, VP_SRC_WIDTH, exynos_plane->src_w);
	vp_reg_write(res, VP_SRC_HEIGHT, exynos_plane->src_h);
	vp_reg_write(res, VP_SRC_H_POSITION,
			VP_SRC_H_POSITION_VAL(exynos_plane->src_x));
	vp_reg_write(res, VP_SRC_V_POSITION, exynos_plane->src_y);

	vp_reg_write(res, VP_DST_WIDTH, exynos_plane->crtc_w);
	vp_reg_write(res, VP_DST_H_POSITION, exynos_plane->crtc_x);
	if (ctx->interlace) {
		vp_reg_write(res, VP_DST_HEIGHT, exynos_plane->crtc_h / 2);
		vp_reg_write(res, VP_DST_V_POSITION, exynos_plane->crtc_y / 2);
	} else {
		vp_reg_write(res, VP_DST_HEIGHT, exynos_plane->crtc_h);
		vp_reg_write(res, VP_DST_V_POSITION, exynos_plane->crtc_y);
	}

	vp_reg_write(res, VP_H_RATIO, x_ratio);
	vp_reg_write(res, VP_V_RATIO, y_ratio);

	vp_reg_write(res, VP_ENDIAN_MODE, VP_ENDIAN_MODE_LITTLE);

	/* set buffer address to vp */
	vp_reg_write(res, VP_TOP_Y_PTR, luma_addr[0]);
	vp_reg_write(res, VP_BOT_Y_PTR, luma_addr[1]);
	vp_reg_write(res, VP_TOP_C_PTR, chroma_addr[0]);
	vp_reg_write(res, VP_BOT_C_PTR, chroma_addr[1]);

	mixer_cfg_scan(ctx, crtc->mode.hdisplay,
		crtc->mode.vdisplay);
	mixer_cfg_rgb_fmt(ctx, crtc->mode.vdisplay);
	mixer_cfg_layer(plane, true);
	mixer_run(ctx);

	mixer_vsync_set_update(ctx, true);
	spin_unlock_irqrestore(&res->reg_slock, flags);

	drm_vblank_get(ctx->drm_dev, ctx->pipe);
	atomic_inc(&ctx->vblank_ref);

	vp_regs_dump(ctx);
	return 0;
}

static int mixer_get_layer_update_count(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;
	u32 val;

	val = mixer_reg_read(res, MXR_CFG);

	return (val & MXR_CFG_LAYER_UPDATE_COUNT_MASK) >>
			MXR_CFG_LAYER_UPDATE_COUNT0;
}

static void mixer_layer_update(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;
	mixer_reg_writemask(res, MXR_CFG, ~0, MXR_CFG_LAYER_UPDATE);
}

static int mixer_graph_plane_update(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct mixer_context *ctx = exynos_plane->ctx;
	int win = exynos_plane_to_win_idx(ctx, exynos_plane);
	struct mixer_resources *res = &ctx->mixer_res;
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);
	struct exynos_drm_gem_buf *buffer;
	dma_addr_t dma_addr;
	unsigned long flags;
	u32 val, fmt;

	DRM_DEBUG_KMS("[WIN:%d]\n", win);

	/* setup format */
	if (fb->bits_per_pixel == 16)
		fmt = MXR_GRP_CFG_FORMAT_ARGB4444;
	else if (fb->bits_per_pixel == 32)
		fmt = MXR_GRP_CFG_FORMAT_ARGB8888;
	else
		return WARN_ON(-EINVAL);

	if (WARN_ON(exynos_drm_fb_get_buf_cnt(exynos_fb) > 1))
		return -EINVAL;

	buffer = exynos_drm_fb_buffer(exynos_fb, 0);

	/* converting dma address base and source offset */
	dma_addr = buffer->dma_addr + exynos_plane->src_x
			* (fb->bits_per_pixel >> 3) + exynos_plane->src_y
			* fb->pitches[0];

	if (crtc->mode.flags & DRM_MODE_FLAG_INTERLACE)
		ctx->interlace = true;
	else
		ctx->interlace = false;

	spin_lock_irqsave(&res->reg_slock, flags);
	mixer_vsync_set_update(ctx, false);

	mixer_reg_writemask(res, MXR_GRAPHIC_CFG(win),
		MXR_GRP_CFG_FORMAT_VAL(fmt), MXR_GRP_CFG_FORMAT_MASK);

	/* setup geometry */
	mixer_reg_write(res, MXR_GRAPHIC_SPAN(win), fb->width);

	/* setup display size */
	if (ctx->mxr_ver == MXR_VER_128_0_0_184 && win == MIXER_DEFAULT_WIN)
		mixer_reg_write(res, MXR_RESOLUTION,
			MXR_MXR_RES_HEIGHT(exynos_plane->src_h) |
			MXR_MXR_RES_WIDTH(exynos_plane->src_w));

	mixer_reg_write(res, MXR_GRAPHIC_WH(win),
		MXR_GRP_WH_WIDTH(exynos_plane->crtc_w) |
		MXR_GRP_WH_HEIGHT(exynos_plane->crtc_h));

	/* reset offsets in source image */
	mixer_reg_write(res, MXR_GRAPHIC_SXY(win), 0);

	val = exynos_plane->crtc_x;
	/* Add any applicable horizontal offset to the destination offset */
	if (ctx->mxr_ver == MXR_VER_16_0_33_0)
		val += mixer_get_horizontal_offset(
					crtc->mode.hdisplay,
					crtc->mode.vdisplay);

	/* setup offsets in display image */
	mixer_reg_write(res, MXR_GRAPHIC_DXY(win), MXR_GRP_DXY_DX(val) |
		MXR_GRP_DXY_DY(exynos_plane->crtc_y));

	/* set buffer address to mixer */
	mixer_reg_write(res, MXR_GRAPHIC_BASE(win), dma_addr);

	mixer_cfg_scan(ctx, crtc->mode.hdisplay, crtc->mode.vdisplay);

	mixer_cfg_rgb_fmt(ctx, crtc->mode.vdisplay);
	mixer_cfg_layer(plane, true);

	/* layer update mandatory for mixer 16.0.33.0 */
	if ((ctx->mxr_ver == MXR_VER_16_0_33_0 ||
	    ctx->mxr_ver == MXR_VER_128_0_0_184) &&
	    !ctx->plane_updated[win]) {
		mixer_layer_update(ctx);
		ctx->plane_updated[win] = true;
	}

	mixer_run(ctx);

	mixer_vsync_set_update(ctx, true);
	spin_unlock_irqrestore(&res->reg_slock, flags);

	drm_vblank_get(ctx->drm_dev, ctx->pipe);
	atomic_inc(&ctx->vblank_ref);

	return 0;
}

static void vp_win_reset(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;
	int tries = 100;

	vp_reg_write(res, VP_SRESET, VP_SRESET_PROCESSING);
	for (tries = 100; tries; --tries) {
		/* waiting until VP_SRESET_PROCESSING is 0 */
		if (~vp_reg_read(res, VP_SRESET) & VP_SRESET_PROCESSING)
			break;
		usleep_range(10000, 12000);
	}
	WARN(tries == 0, "failed to reset Video Processor\n");
}

static void mixer_win_reset(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;
	unsigned long flags;
	u32 val; /* value stored to register */

	DRM_DEBUG_KMS("\n");

	spin_lock_irqsave(&res->reg_slock, flags);
	mixer_vsync_set_update(ctx, false);

	mixer_reg_writemask(res, MXR_CFG, MXR_CFG_DST_HDMI, MXR_CFG_DST_MASK);

	/* set output in RGB888 mode */
	mixer_reg_writemask(res, MXR_CFG, MXR_CFG_OUT_RGB888, MXR_CFG_OUT_MASK);

	/* 16 beat burst in DMA */
	mixer_reg_writemask(res, MXR_STATUS, MXR_STATUS_16_BURST,
		MXR_STATUS_BURST_MASK);

	/* setting default layer priority: layer1 > layer0 > video
	 * because typical usage scenario would be
	 * layer1 - OSD
	 * layer0 - framebuffer
	 * video - video overlay
	 */
	val = MXR_LAYER_CFG_GRP1_VAL(3);
	val |= MXR_LAYER_CFG_GRP0_VAL(2);
	if (ctx->vp_enabled)
		val |= MXR_LAYER_CFG_VP_VAL(1);
	mixer_reg_write(res, MXR_LAYER_CFG, val);

	/* setting background color */
	mixer_reg_write(res, MXR_BG_COLOR0, 0x008080);
	mixer_reg_write(res, MXR_BG_COLOR1, 0x008080);
	mixer_reg_write(res, MXR_BG_COLOR2, 0x008080);

	/* setting graphical layers */
	val  = MXR_GRP_CFG_COLOR_KEY_DISABLE; /* no blank key */
	val |= MXR_GRP_CFG_WIN_BLEND_EN;
	val |= MXR_GRP_CFG_ALPHA_VAL(0xff); /* non-transparent alpha */

	/* Don't blend layer 0 onto the mixer background */
	mixer_reg_write(res, MXR_GRAPHIC_CFG(0), val);

	/* Blend layer 1 into layer 0 */
	val |= MXR_GRP_CFG_BLEND_PRE_MUL;
	val |= MXR_GRP_CFG_PIXEL_BLEND_EN;
	mixer_reg_write(res, MXR_GRAPHIC_CFG(1), val);

	/* setting video layers */
	val = MXR_GRP_CFG_ALPHA_VAL(0);
	mixer_reg_write(res, MXR_VIDEO_CFG, val);

	if (ctx->vp_enabled) {
		/* configuration of Video Processor Registers */
		vp_win_reset(ctx);
		vp_default_filter(res);
	}

	/* disable all layers */
	mixer_reg_writemask(res, MXR_CFG, 0, MXR_CFG_GRP0_ENABLE);
	mixer_reg_writemask(res, MXR_CFG, 0, MXR_CFG_GRP1_ENABLE);
	if (ctx->vp_enabled)
		mixer_reg_writemask(res, MXR_CFG, 0, MXR_CFG_VP_ENABLE);

	mixer_vsync_set_update(ctx, true);
	spin_unlock_irqrestore(&res->reg_slock, flags);

	drm_vblank_get(ctx->drm_dev, ctx->pipe);
	atomic_inc(&ctx->vblank_ref);
}

static int mixer_plane_commit(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct mixer_context *mixer_ctx = exynos_plane->ctx;
	int win = exynos_plane_to_win_idx(mixer_ctx, exynos_plane);

	if (!mixer_ctx->powered)
		return 0;

	if (win > 1 && mixer_ctx->vp_enabled)
		return mixer_vp_plane_update(plane, crtc, fb);

	return mixer_graph_plane_update(plane, crtc, fb);
}

int mixer_plane_update(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb, int crtc_x,
		int crtc_y, unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y, uint32_t src_w, uint32_t src_h)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_plane old_plane;
	int ret;

	exynos_plane_copy_state(exynos_plane, &old_plane);

	/* Copy the parameters into the plane so we can restore it later */
	exynos_plane->crtc_x = crtc_x;
	exynos_plane->crtc_y = crtc_y;
	exynos_plane->crtc_w = crtc_w;
	exynos_plane->crtc_h = crtc_h;
	exynos_plane->src_x = src_x >> 16;
	exynos_plane->src_y = src_y >> 16;
	exynos_plane->src_w = src_w >> 16;
	exynos_plane->src_h = src_h >> 16;

	exynos_sanitize_plane_coords(plane, crtc);

	ret = mixer_plane_commit(plane, crtc, fb);

	/* restore old plane on failure*/
	if (ret)
		exynos_plane_copy_state(&old_plane, exynos_plane);

	return ret;
}

static int mixer_update(void *ctx, struct drm_crtc *crtc,
		struct drm_framebuffer *fb)
{
	struct mixer_context *mixer_ctx = ctx;
	struct exynos_drm_plane *exynos_plane =
					&mixer_ctx->planes[MIXER_DEFAULT_WIN];
	struct exynos_drm_plane old_plane;
	struct drm_plane *plane = &exynos_plane->base;
	int ret;

	exynos_plane_copy_state(exynos_plane, &old_plane);

	/* Copy the parameters into the plane so we can restore it later */
	exynos_plane->crtc_x = 0;
	exynos_plane->crtc_y = 0;
	exynos_plane->crtc_w = fb->width - crtc->x;
	exynos_plane->crtc_h = fb->height - crtc->y;
	exynos_plane->src_x = crtc->x;
	exynos_plane->src_y = crtc->y;
	exynos_plane->src_w = exynos_plane->crtc_w;
	exynos_plane->src_h = exynos_plane->crtc_h;

	exynos_sanitize_plane_coords(plane, crtc);

	/* Grab a reference, just as setplane would */
	drm_framebuffer_reference(fb);

	ret = mixer_plane_commit(plane, crtc, fb);
	if (!ret) {
		if (plane->fb)
			drm_framebuffer_unreference(plane->fb);
		plane->fb = fb;
		plane->crtc = crtc;
	} else {
		/* restore old plane on failure */
		exynos_plane_copy_state(&old_plane, exynos_plane);
		drm_framebuffer_unreference(fb);
		DRM_ERROR("fimd plane commit failed %d\n", ret);
	}

	/* Reset workaround state if the resolution is not 800x600 */
	if (fb->width != 800 || fb->height != 600) {
		mixer_ctx->workaround_state = WORKAROUND_STATE_INACTIVE;
		goto done;
	}

	/* Initialize workaround state if not active */
	if (mixer_ctx->workaround_state != WORKAROUND_STATE_ACTIVE)
		mixer_ctx->workaround_state = WORKAROUND_STATE_INIT;

done:
	return ret;
}

/*
 * Schedule a window (hardware overlay) to be disabled at the next vblank.
 * This is useful when disabling multiple windows, for example during suspend.
 */
static void mixer_plane_disable_nowait(struct drm_plane *plane)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct mixer_context *mixer_ctx = exynos_plane->ctx;
	struct mixer_resources *res = &mixer_ctx->mixer_res;
	unsigned long flags;

	if (!mixer_ctx->powered)
		return;

	DRM_DEBUG_KMS("[WIN:%d]\n",
		exynos_plane_to_win_idx(mixer_ctx, exynos_plane));

	spin_lock_irqsave(&res->reg_slock, flags);
	mixer_vsync_set_update(mixer_ctx, false);

	mixer_cfg_layer(plane, false);

	mixer_vsync_set_update(mixer_ctx, true);
	spin_unlock_irqrestore(&res->reg_slock, flags);

	drm_vblank_get(mixer_ctx->drm_dev, mixer_ctx->pipe);
	atomic_inc(&mixer_ctx->vblank_ref);
}

static void mixer_wait_for_vblank(void *ctx)
{
	struct mixer_context *mixer_ctx = ctx;

	if (!mixer_ctx->powered)
		return;

	DRM_DEBUG_KMS("\n");

	drm_vblank_get(mixer_ctx->drm_dev, mixer_ctx->pipe);

	atomic_set(&mixer_ctx->wait_vsync_event, 1);

	/*
	 * wait for MIXER to signal VSYNC interrupt or return after
	 * timeout which is set to 50ms (refresh rate of 20).
	 */
	if (!wait_event_timeout(mixer_ctx->wait_vsync_queue,
				!atomic_read(&mixer_ctx->wait_vsync_event),
				DRM_HZ/20))
		DRM_DEBUG_KMS("vblank wait timed out.\n");

	drm_vblank_put(mixer_ctx->drm_dev, mixer_ctx->pipe);
}

int mixer_plane_disable(struct drm_plane *plane)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct mixer_context *ctx = exynos_plane->ctx;

	DRM_DEBUG_KMS("[WIN:%d]\n", exynos_plane_to_win_idx(ctx, exynos_plane));

	mixer_plane_disable_nowait(plane);

	/* Synchronously wait for window to be disabled */
	mixer_wait_for_vblank(ctx);

	return 0;
}

static const struct drm_plane_funcs mixer_plane_funcs = {
	.update_plane = mixer_plane_update,
	.disable_plane = mixer_plane_disable,
	.destroy = drm_plane_cleanup,
};

static int mixer_initialize(void *ctx, struct drm_crtc *crtc, int pipe)
{
	struct mixer_context *mixer_ctx = ctx;
	int ret, i;

	DRM_DEBUG_KMS("pipe: %d\n", pipe);

	mixer_ctx->drm_dev = crtc->dev;
	mixer_ctx->pipe = pipe;

	if (!is_drm_iommu_supported(mixer_ctx->drm_dev))
		return 0;

	ret = drm_iommu_attach_device(mixer_ctx->drm_dev, mixer_ctx->dev);
	if (ret) {
		DRM_ERROR("Failed to attach iommu device ret=%d\n", ret);
		return ret;
	}

	for (i = 0; i < MIXER_WIN_NR; i++) {
		struct drm_plane *plane = &mixer_ctx->planes[i].base;
		struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);

		ret = drm_plane_init(mixer_ctx->drm_dev, plane,
				1 << mixer_ctx->pipe, &mixer_plane_funcs,
				plane_formats, ARRAY_SIZE(plane_formats),
				i == MIXER_DEFAULT_WIN ? true : false);
		if (ret) {
			DRM_ERROR("Init plane %d failed (ret=%d)\n", i, ret);
			goto err;
		}

		exynos_plane->ctx = mixer_ctx;
	}

	return 0;
err:
	for (i--; i >= 0; i--)
		drm_plane_cleanup(&mixer_ctx->planes[i].base);

	return ret;
}

static void mixer_mgr_remove(void *ctx)
{
	struct mixer_context *mixer_ctx = ctx;

	DRM_DEBUG_KMS("pipe: %d\n", mixer_ctx->pipe);

	if (is_drm_iommu_supported(mixer_ctx->drm_dev))
		drm_iommu_detach_device(mixer_ctx->drm_dev, mixer_ctx->dev);
}

/*
 * This driver only uses one of mixer's interrupts, "Vertical synchronization"
 * which is enabled by Mixer_INTR_EN.INT_EN_VSYNC.
 */
static void mixer_irq_mask(struct mixer_context *ctx, bool enable)
{
	struct mixer_resources *res = &ctx->mixer_res;
	u32 val;

	DRM_DEBUG_KMS("enable: %u\n", enable);

	val = enable ? MXR_INT_EN_VSYNC : 0;
	mixer_reg_write(res, MXR_INT_EN, val);
}

static int mixer_enable_vblank(void *ctx)
{
	struct mixer_context *mixer_ctx = ctx;

	DRM_DEBUG_KMS("powered: %d\n", mixer_ctx->powered);

	if (!mixer_ctx->powered)
		return 0;

	mixer_irq_mask(ctx, true);

	return 0;
}

static void mixer_disable_vblank(void *ctx)
{
	struct mixer_context *mixer_ctx = ctx;

	DRM_DEBUG_KMS("powered: %d\n", mixer_ctx->powered);

	if (!mixer_ctx->powered)
		return;

	mixer_irq_mask(ctx, false);
}

static void mixer_disable_planes(struct mixer_context *ctx)
{
	int i;
	bool wait_for_vblank = false;

	DRM_DEBUG_KMS("\n");

	for (i = 0; i < MIXER_WIN_NR; i++) {
		struct drm_plane *plane = &ctx->planes[i].base;

		if (!plane->fb)
			continue;

		mixer_plane_disable_nowait(plane);
		wait_for_vblank = true;
	}

	/* Synchronously wait for any window disables to complete */
	if (wait_for_vblank)
		mixer_wait_for_vblank(ctx);
}

static void mixer_enable_planes(struct mixer_context *ctx)
{
	int i, ret;

	DRM_DEBUG_KMS("\n");

	for (i = 0; i < MIXER_WIN_NR; i++) {
		struct exynos_drm_plane *exynos_plane = &ctx->planes[i];
		struct drm_plane *plane = &exynos_plane->base;

		if (!plane->fb)
			continue;

		ret = plane->funcs->update_plane(plane, plane->crtc, plane->fb,
			exynos_plane->crtc_x, exynos_plane->crtc_y,
			exynos_plane->crtc_w, exynos_plane->crtc_h,
			exynos_plane->src_x << 16, exynos_plane->src_y << 16,
			exynos_plane->src_w << 16, exynos_plane->src_h << 16);
		if (ret)
			DRM_ERROR("Failed to update plane %d ret=%d\n", i, ret);
	}

	if (ctx->workaround_state != WORKAROUND_STATE_INACTIVE)
		ctx->workaround_state = WORKAROUND_STATE_INIT;
}

static void mixer_poweron(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;

	if (ctx->powered)
		return;

	DRM_DEBUG_KMS("\n");

	pm_runtime_get_sync(ctx->dev);

	clk_prepare_enable(res->mixer);
	if (ctx->vp_enabled) {
		clk_prepare_enable(res->vp);
		clk_prepare_enable(res->sclk_mixer);
	}

	mixer_win_reset(ctx);

	mixer_enable_planes(ctx);

	/*
	 * We don't flip the powered bit until after the window state has been
	 * reset since enabling it earlier will cause iommu faults for those
	 * windows which are not properly setup.
	 */
	ctx->powered = true;

	mixer_irq_mask(ctx, drm_is_vblank_enabled(ctx->drm_dev, ctx->pipe));
	enable_irq(ctx->mixer_res.irq);
}

static void mixer_poweroff(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;

	if (!ctx->powered)
		return;

	DRM_DEBUG_KMS("\n");

	mixer_disable_planes(ctx);

	disable_irq(ctx->mixer_res.irq);

	clk_disable_unprepare(res->mixer);
	if (ctx->vp_enabled) {
		clk_disable_unprepare(res->vp);
		clk_disable_unprepare(res->sclk_mixer);
	}

	pm_runtime_put_sync(ctx->dev);

	ctx->powered = false;
}

static void mixer_dpms(void *ctx, int mode)
{
	struct mixer_context *mixer_ctx = ctx;

	DRM_DEBUG_KMS("[DPMS:%s]\n", drm_get_dpms_name(mode));

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		mixer_poweron(mixer_ctx);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		mixer_poweroff(mixer_ctx);
		break;
	default:
		DRM_DEBUG_KMS("unknown dpms mode: %d\n", mode);
		break;
	}
}

static const struct exynos_drm_manager_ops mixer_manager_ops = {
	.initialize		= mixer_initialize,
	.remove			= mixer_mgr_remove,
	.dpms			= mixer_dpms,
	.adjust_mode		= mixer_adjust_mode,
	.mode_fixup		= mixer_mode_fixup,
	.update			= mixer_update,
	.enable_vblank		= mixer_enable_vblank,
	.disable_vblank		= mixer_disable_vblank,
};

static struct exynos_drm_manager mixer_manager = {
	.type			= EXYNOS_DISPLAY_TYPE_HDMI,
	.ops			= &mixer_manager_ops,
};

int mixer_mode_valid(struct drm_display_mode *mode)
{
	struct mixer_context *mctx = mixer_manager.ctx;
	enum exynos_mixer_mode_type mt;

	mt = exynos_mixer_get_mode_type(mctx, mode->hdisplay, mode->vdisplay);

	return (mt == EXYNOS_MIXER_MODE_INVALID) ? MODE_BAD : MODE_OK;
}

static inline void mixer_update_workaround_state(struct mixer_context *mctx)
{
	struct mixer_resources *res = &mctx->mixer_res;
	u32 val;

	/* The 800x600 workaround just applies to 16.0.33.0 */
	if (mctx->mxr_ver != MXR_VER_16_0_33_0)
		return;

	/* The workaround uses stereoscopic output mode*/
	val = MXR_TVOUT_CFG_STEREOSCOPIC | MXR_TVOUT_CFG_UNKNOWN |
		MXR_TVOUT_CFG_FRAME_FMT_SXS;

	switch (mctx->workaround_state) {
	case WORKAROUND_STATE_INACTIVE:
		val = MXR_TVOUT_CFG_PATH_ONE_PATH; /* Revert back to one path */
		break;
	case WORKAROUND_STATE_INIT:
		val |= MXR_TVOUT_CFG_PATH_ONE_PATH;
		mctx->workaround_state = WORKAROUND_STATE_ACTIVE;
		break;
	case WORKAROUND_STATE_ACTIVE:
		val |= MXR_TVOUT_CFG_PATH_TWO_PATH;
		break;
	}
	mixer_reg_write(res, MXR_TVOUT_CFG, val);
}

static irqreturn_t mixer_irq_handler(int irq, void *arg)
{
	struct mixer_context *ctx = arg;
	struct mixer_resources *res = &ctx->mixer_res;
	u32 val;
	int i;
	unsigned long flags;
	bool finish_pageflip = false;

	WARN_ON(!ctx->powered);

	spin_lock_irqsave(&res->reg_slock, flags);

	/* read interrupt status for handling and clearing flags for VSYNC */
	val = mixer_reg_read(res, MXR_INT_STATUS);

	if (!ctx->drm_dev)
		goto out;

	/* handling VSYNC */
	if (val & MXR_INT_STATUS_VSYNC) {
		drm_handle_vblank(ctx->drm_dev, ctx->pipe);

		if (ctx->mxr_ver == MXR_VER_16_0_33_0 ||
			ctx->mxr_ver == MXR_VER_128_0_0_184) {
			/* Bail out if a layer update is pending */
			if (mixer_get_layer_update_count(ctx))
				goto out;

			mixer_update_workaround_state(ctx);

			for (i = 0; i < MIXER_WIN_NR; i++)
				ctx->plane_updated[i] = false;
		}

		finish_pageflip = true;

		/* set wait vsync event to zero and wake up queue. */
		if (atomic_read(&ctx->wait_vsync_event)) {
			atomic_set(&ctx->wait_vsync_event, 0);
			DRM_WAKEUP(&ctx->wait_vsync_queue);
		}
	}

out:
	/* clear interrupts */
	if (~val & MXR_INT_EN_VSYNC) {
		/* vsync interrupt use different bit for read and clear */
		val &= ~MXR_INT_EN_VSYNC;
		val |= MXR_INT_CLEAR_VSYNC;
	}
	mixer_reg_write(res, MXR_INT_STATUS, val);

	spin_unlock_irqrestore(&res->reg_slock, flags);

	if (atomic_dec_if_positive(&ctx->vblank_ref) >= 0)
		drm_vblank_put(ctx->drm_dev, ctx->pipe);

	if (finish_pageflip)
		exynos_drm_crtc_finish_pageflip(ctx->drm_dev, ctx->pipe);

	return IRQ_HANDLED;
}

static int mixer_resources_init(struct mixer_context *mixer_ctx,
				struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mixer_resources *mixer_res = &mixer_ctx->mixer_res;
	struct resource *res;
	int ret;

	DRM_DEBUG("[PDEV:%s]\n", pdev->name);

	spin_lock_init(&mixer_res->reg_slock);

	mixer_res->mixer = devm_clk_get(dev, "mixer");
	if (IS_ERR_OR_NULL(mixer_res->mixer)) {
		dev_err(dev, "failed to get clock 'mixer'\n");
		return -ENODEV;
	}

	mixer_res->sclk_hdmi = devm_clk_get(dev, "sclk_hdmi");
	if (IS_ERR_OR_NULL(mixer_res->sclk_hdmi)) {
		dev_err(dev, "failed to get clock 'sclk_hdmi'\n");
		return -ENODEV;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "get memory resource failed.\n");
		return -ENXIO;
	}

	mixer_res->mixer_regs = devm_ioremap(&pdev->dev, res->start,
							resource_size(res));
	if (mixer_res->mixer_regs == NULL) {
		dev_err(dev, "register mapping failed.\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(dev, "get interrupt resource failed.\n");
		return -ENXIO;
	}

	ret = devm_request_irq(&pdev->dev, res->start, mixer_irq_handler,
						0, "drm_mixer", mixer_ctx);
	if (ret) {
		dev_err(dev, "request interrupt failed.\n");
		return ret;
	}
	mixer_res->irq = res->start;
	disable_irq(mixer_res->irq);

	return 0;
}

static int vp_resources_init(struct mixer_context *mixer_ctx,
			     struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mixer_resources *mixer_res = &mixer_ctx->mixer_res;
	struct resource *res;

	DRM_DEBUG("[PDEV:%s]\n", pdev->name);

	mixer_res->vp = devm_clk_get(dev, "vp");
	if (IS_ERR_OR_NULL(mixer_res->vp)) {
		dev_err(dev, "failed to get clock 'vp'\n");
		return -ENODEV;
	}
	mixer_res->sclk_mixer = devm_clk_get(dev, "sclk_mixer");
	if (IS_ERR_OR_NULL(mixer_res->sclk_mixer)) {
		dev_err(dev, "failed to get clock 'sclk_mixer'\n");
		return -ENODEV;
	}
	mixer_res->sclk_dac = devm_clk_get(dev, "sclk_dac");
	if (IS_ERR_OR_NULL(mixer_res->sclk_dac)) {
		dev_err(dev, "failed to get clock 'sclk_dac'\n");
		return -ENODEV;
	}

	if (mixer_res->sclk_hdmi)
		clk_set_parent(mixer_res->sclk_mixer, mixer_res->sclk_hdmi);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		dev_err(dev, "get memory resource failed.\n");
		return -ENXIO;
	}

	mixer_res->vp_regs = devm_ioremap(&pdev->dev, res->start,
							resource_size(res));
	if (mixer_res->vp_regs == NULL) {
		dev_err(dev, "register mapping failed.\n");
		return -ENXIO;
	}

	return 0;
}

static struct mixer_drv_data exynos5420_mxr_drv_data = {
	.version = MXR_VER_128_0_0_184,
	.is_vp_enabled = 0,
};

static struct mixer_drv_data exynos5250_mxr_drv_data = {
	.version = MXR_VER_16_0_33_0,
	.is_vp_enabled = 0,
};

static struct mixer_drv_data exynos4210_mxr_drv_data = {
	.version = MXR_VER_0_0_0_16,
	.is_vp_enabled = 1,
};

static struct platform_device_id mixer_driver_types[] = {
	{
		.name		= "s5p-mixer",
		.driver_data	= (unsigned long)&exynos4210_mxr_drv_data,
	}, {
		.name		= "exynos5-mixer",
		.driver_data	= (unsigned long)&exynos5250_mxr_drv_data,
	}, {
		/* end node */
	}
};

static struct of_device_id mixer_match_types[] = {
	{
		.compatible = "samsung,exynos5420-mixer",
		.data	= &exynos5420_mxr_drv_data,
	}, {
		.compatible = "samsung,exynos5250-mixer",
		.data	= &exynos5250_mxr_drv_data,
	}, {
		/* end node */
	}
};

static int mixer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mixer_context *ctx;
	struct mixer_drv_data *drv;
	int ret;

	DRM_DEBUG("[PDEV:%s]\n", pdev->name);

	dev_info(dev, "probe start\n");

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		DRM_ERROR("failed to alloc mixer context.\n");
		return -ENOMEM;
	}

	if (dev->of_node) {
		const struct of_device_id *match;
		match = of_match_node(of_match_ptr(mixer_match_types),
							  pdev->dev.of_node);
		drv = (struct mixer_drv_data *)match->data;
	} else {
		drv = (struct mixer_drv_data *)
			platform_get_device_id(pdev)->driver_data;
	}

	ctx->dev = &pdev->dev;
	ctx->vp_enabled = drv->is_vp_enabled;
	ctx->mxr_ver = drv->version;
	DRM_INIT_WAITQUEUE(&ctx->wait_vsync_queue);
	atomic_set(&ctx->wait_vsync_event, 0);
	atomic_set(&ctx->vblank_ref, 0);

	platform_set_drvdata(pdev, ctx);

	/* acquire resources: regs, irqs, clocks */
	ret = mixer_resources_init(ctx, pdev);
	if (ret) {
		DRM_ERROR("mixer_resources_init failed\n");
		goto fail;
	}

	if (ctx->vp_enabled) {
		/* acquire vp resources: regs, irqs, clocks */
		ret = vp_resources_init(ctx, pdev);
		if (ret) {
			DRM_ERROR("vp_resources_init failed\n");
			goto fail;
		}
	}

	mixer_manager.ctx = ctx;
	exynos_drm_manager_register(&mixer_manager);

	/*
	 * We need to runtime pm to enable/disable sysmmu since it is a child of
	 * this driver. Ideally, this would hang off the drm driver's runtime
	 * operations, but we're not quite there yet.
	 *
	 * Tracked in crbug.com/264312
	 */
	pm_runtime_enable(dev);

	return 0;


fail:
	dev_info(dev, "probe failed\n");
	return ret;
}

static int mixer_remove(struct platform_device *pdev)
{
	DRM_DEBUG("[PDEV:%s]\n", pdev->name);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

struct platform_driver mixer_driver = {
	.driver = {
		.name = "exynos-mixer",
		.owner = THIS_MODULE,
		.of_match_table = mixer_match_types,
	},
	.probe = mixer_probe,
	.remove = mixer_remove,
	.id_table	= mixer_driver_types,
};
