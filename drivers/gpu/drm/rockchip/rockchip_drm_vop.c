/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:Mark Yao <mark.yao@rock-chips.com>
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

#include <drm/drm.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_sync_helper.h>

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/fence.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_fb.h"
#include "rockchip_drm_gem.h"
#include "rockchip_drm_vop.h"

#include <soc/rockchip/dmc-sync.h>

#define VOP_REG(off, _mask, s) \
		{.offset = off, \
		 .mask = _mask, \
		 .shift = s,}

#define __REG_SET_RELAXED(x, off, mask, shift, v) \
		vop_mask_write_relaxed(x, off, (mask) << shift, (v) << shift)
#define __REG_SET_NORMAL(x, off, mask, shift, v) \
		vop_mask_write(x, off, (mask) << shift, (v) << shift)

#define REG_SET(x, base, reg, v, mode) \
		__REG_SET_##mode(x, base + reg.offset, reg.mask, reg.shift, v)

#define VOP_WIN_SET(x, win, name, v) \
		REG_SET(x, win->base, win->phy->name, v, RELAXED)
#define VOP_CTRL_SET(x, name, v) \
		REG_SET(x, 0, (x)->data->ctrl->name, v, NORMAL)

#define VOP_WIN_GET(x, win, name) \
		vop_read_reg(x, win->base, &win->phy->name)

#define VOP_WIN_GET_YRGBADDR(vop, win) \
		vop_readl(vop, win->base + win->phy->yrgb_mst.offset)

#define to_vop(x) container_of(x, struct vop, crtc)
#define to_vop_win(x) container_of(x, struct vop_win, base)

struct vop_win {
	struct drm_plane base;
	const struct vop_win_data *data;
	struct vop *vop;

	struct drm_framebuffer *front_fb;

	bool pending;
	struct drm_framebuffer *pending_fb; /* NULL for pending win disable */
	dma_addr_t pending_yrgb_mst;
	uint32_t pending_dsp_st;
	uint32_t pending_dsp_info;
	struct drm_pending_vblank_event *pending_event;
	struct completion completion;

#ifdef CONFIG_DRM_DMA_SYNC
	unsigned fence_context;
	atomic_t fence_seqno;
	struct fence *fence;
	struct drm_reservation_cb rcb;

	struct fence *pending_fence;
	bool pending_needs_vblank;
#endif
};

struct vop {
	struct drm_crtc crtc;
	struct device *dev;
	struct drm_device *drm_dev;
	bool is_enabled;

	int connector_type;
	int connector_out_mode;

	const struct vop_data *data;

	struct notifier_block dmc_nb;
	struct completion dmc_completion;

	uint32_t *regsbak;
	void __iomem *regs;

	/* physical map length of vop register */
	uint32_t len;

	/* one time only one process allowed to config the register */
	spinlock_t reg_lock;
	/* lock vop irq reg */
	spinlock_t irq_lock;

	unsigned int irq;

	/* vop AHP clk */
	struct clk *hclk;
	/* vop dclk */
	struct clk *dclk;
	/* vop share memory frequency */
	struct clk *aclk;

	/* vop dclk reset */
	struct reset_control *dclk_rst;

	int pipe;

	struct vop_win win[];
};

enum vop_data_format {
	VOP_FMT_ARGB8888 = 0,
	VOP_FMT_RGB888,
	VOP_FMT_RGB565,
	VOP_FMT_YUV420SP = 4,
	VOP_FMT_YUV422SP,
	VOP_FMT_YUV444SP,
};

struct vop_reg_data {
	uint32_t offset;
	uint32_t value;
};

struct vop_reg {
	uint32_t offset;
	uint32_t shift;
	uint32_t mask;
};

struct vop_ctrl {
	struct vop_reg standby;
	struct vop_reg data_blank;
	struct vop_reg gate_en;
	struct vop_reg mmu_en;
	struct vop_reg rgb_en;
	struct vop_reg edp_en;
	struct vop_reg hdmi_en;
	struct vop_reg mipi_en;
	struct vop_reg out_mode;
	struct vop_reg dither_down;
	struct vop_reg dither_up;
	struct vop_reg pin_pol;

	struct vop_reg htotal_pw;
	struct vop_reg hact_st_end;
	struct vop_reg vtotal_pw;
	struct vop_reg vact_st_end;
	struct vop_reg hpost_st_end;
	struct vop_reg vpost_st_end;
};

struct vop_win_phy {
	const uint32_t *data_formats;
	uint32_t nformats;

	struct vop_reg enable;
	struct vop_reg format;
	struct vop_reg act_info;
	struct vop_reg dsp_info;
	struct vop_reg dsp_st;
	struct vop_reg yrgb_mst;
	struct vop_reg uv_mst;
	struct vop_reg yrgb_vir;
	struct vop_reg uv_vir;

	struct vop_reg dst_alpha_ctl;
	struct vop_reg src_alpha_ctl;
};

struct vop_win_data {
	uint32_t base;
	const struct vop_win_phy *phy;
	enum drm_plane_type type;
};

struct vop_data {
	const struct vop_reg_data *init_table;
	unsigned int table_size;
	const struct vop_ctrl *ctrl;
	const struct vop_win_data *win;
	unsigned int win_size;
};

static const uint32_t formats_01[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
};

static const uint32_t formats_234[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
};

static const struct vop_win_phy win01_data = {
	.data_formats = formats_01,
	.nformats = ARRAY_SIZE(formats_01),
	.enable = VOP_REG(WIN0_CTRL0, 0x1, 0),
	.format = VOP_REG(WIN0_CTRL0, 0x7, 1),
	.act_info = VOP_REG(WIN0_ACT_INFO, 0x1fff1fff, 0),
	.dsp_info = VOP_REG(WIN0_DSP_INFO, 0x0fff0fff, 0),
	.dsp_st = VOP_REG(WIN0_DSP_ST, 0x1fff1fff, 0),
	.yrgb_mst = VOP_REG(WIN0_YRGB_MST, 0xffffffff, 0),
	.uv_mst = VOP_REG(WIN0_CBR_MST, 0xffffffff, 0),
	.yrgb_vir = VOP_REG(WIN0_VIR, 0x3fff, 0),
	.uv_vir = VOP_REG(WIN0_VIR, 0x3fff, 16),
	.src_alpha_ctl = VOP_REG(WIN0_SRC_ALPHA_CTRL, 0xff, 0),
	.dst_alpha_ctl = VOP_REG(WIN0_DST_ALPHA_CTRL, 0xff, 0),
};

static const struct vop_win_phy win23_data = {
	.data_formats = formats_234,
	.nformats = ARRAY_SIZE(formats_234),
	.enable = VOP_REG(WIN2_CTRL0, 0x1, 0),
	.format = VOP_REG(WIN2_CTRL0, 0x7, 1),
	.dsp_info = VOP_REG(WIN2_DSP_INFO0, 0x0fff0fff, 0),
	.dsp_st = VOP_REG(WIN2_DSP_ST0, 0x1fff1fff, 0),
	.yrgb_mst = VOP_REG(WIN2_MST0, 0xffffffff, 0),
	.yrgb_vir = VOP_REG(WIN2_VIR0_1, 0x1fff, 0),
	.src_alpha_ctl = VOP_REG(WIN2_SRC_ALPHA_CTRL, 0xff, 0),
	.dst_alpha_ctl = VOP_REG(WIN2_DST_ALPHA_CTRL, 0xff, 0),
};

static const struct vop_win_phy cursor_data = {
	.data_formats = formats_234,
	.nformats = ARRAY_SIZE(formats_234),
	.enable = VOP_REG(HWC_CTRL0, 0x1, 0),
	.format = VOP_REG(HWC_CTRL0, 0x7, 1),
	.dsp_st = VOP_REG(HWC_DSP_ST, 0x1fff1fff, 0),
	.yrgb_mst = VOP_REG(HWC_MST, 0xffffffff, 0),
};

static const struct vop_ctrl ctrl_data = {
	.standby = VOP_REG(SYS_CTRL, 0x1, 22),
	.gate_en = VOP_REG(SYS_CTRL, 0x1, 23),
	.mmu_en = VOP_REG(SYS_CTRL, 0x1, 20),
	.rgb_en = VOP_REG(SYS_CTRL, 0x1, 12),
	.hdmi_en = VOP_REG(SYS_CTRL, 0x1, 13),
	.edp_en = VOP_REG(SYS_CTRL, 0x1, 14),
	.mipi_en = VOP_REG(SYS_CTRL, 0x1, 15),
	.dither_down = VOP_REG(DSP_CTRL1, 0xf, 1),
	.dither_up = VOP_REG(DSP_CTRL1, 0x1, 6),
	.data_blank = VOP_REG(DSP_CTRL0, 0x1, 19),
	.out_mode = VOP_REG(DSP_CTRL0, 0xf, 0),
	.pin_pol = VOP_REG(DSP_CTRL0, 0xf, 4),
	.htotal_pw = VOP_REG(DSP_HTOTAL_HS_END, 0x1fff1fff, 0),
	.hact_st_end = VOP_REG(DSP_HACT_ST_END, 0x1fff1fff, 0),
	.vtotal_pw = VOP_REG(DSP_VTOTAL_VS_END, 0x1fff1fff, 0),
	.vact_st_end = VOP_REG(DSP_VACT_ST_END, 0x1fff1fff, 0),
	.hpost_st_end = VOP_REG(POST_DSP_HACT_INFO, 0x1fff1fff, 0),
	.vpost_st_end = VOP_REG(POST_DSP_VACT_INFO, 0x1fff1fff, 0),
};

static const struct vop_reg_data vop_init_reg_table[] = {
	{SYS_CTRL, 0x00c00000},
	{DSP_CTRL0, 0x00000000},
	{WIN0_CTRL0, 0x00000080},
	{WIN1_CTRL0, 0x00000080},
};

/*
 * Note: rk3288 has a dedicated 'cursor' window, however, that window requires
 * special support to get alpha blending working.  For now, just use overlay
 * window 1 for the drm cursor.
 */
static const struct vop_win_data rk3288_vop_win_data[] = {
	{ .base = 0x00, .phy = &win01_data, .type = DRM_PLANE_TYPE_PRIMARY },
	{ .base = 0x40, .phy = &win01_data, .type = DRM_PLANE_TYPE_CURSOR },
	{ .base = 0x00, .phy = &win23_data, .type = DRM_PLANE_TYPE_OVERLAY },
	{ .base = 0x50, .phy = &win23_data, .type = DRM_PLANE_TYPE_OVERLAY },
	{ .base = 0x00, .phy = &cursor_data, .type = DRM_PLANE_TYPE_OVERLAY },
};

static const struct vop_data rk3288_vop = {
	.init_table = vop_init_reg_table,
	.table_size = ARRAY_SIZE(vop_init_reg_table),
	.ctrl = &ctrl_data,
	.win = rk3288_vop_win_data,
	.win_size = ARRAY_SIZE(rk3288_vop_win_data),
};

static const struct of_device_id vop_driver_dt_match[] = {
	{ .compatible = "rockchip,rk3288-vop",
	  .data = &rk3288_vop },
	{},
};

static inline void vop_writel(struct vop *vop, uint32_t offset, uint32_t v)
{
	writel(v, vop->regs + offset);
	vop->regsbak[offset >> 2] = v;
}

static inline uint32_t vop_readl(struct vop *vop, uint32_t offset)
{
	return readl(vop->regs + offset);
}

static inline uint32_t vop_read_reg(struct vop *vop, uint32_t base,
				    const struct vop_reg *reg)
{
	return (vop_readl(vop, base + reg->offset) >> reg->shift) & reg->mask;
}

static inline void vop_cfg_done(struct vop *vop)
{
	writel(0x01, vop->regs + REG_CFG_DONE);
}

static inline void vop_mask_write(struct vop *vop, uint32_t offset,
				  uint32_t mask, uint32_t v)
{
	if (mask) {
		uint32_t cached_val = vop->regsbak[offset >> 2];

		cached_val = (cached_val & ~mask) | v;
		writel(cached_val, vop->regs + offset);
		vop->regsbak[offset >> 2] = cached_val;
	}
}

static inline void vop_mask_write_relaxed(struct vop *vop, uint32_t offset,
					  uint32_t mask, uint32_t v)
{
	if (mask) {
		uint32_t cached_val = vop->regsbak[offset >> 2];

		cached_val = (cached_val & ~mask) | v;
		writel_relaxed(cached_val, vop->regs + offset);
		vop->regsbak[offset >> 2] = cached_val;
	}
}

static enum vop_data_format vop_convert_format(uint32_t format)
{
	switch (format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return VOP_FMT_ARGB8888;
	case DRM_FORMAT_RGB888:
		return VOP_FMT_RGB888;
	case DRM_FORMAT_RGB565:
		return VOP_FMT_RGB565;
	case DRM_FORMAT_NV12:
		return VOP_FMT_YUV420SP;
	case DRM_FORMAT_NV16:
		return VOP_FMT_YUV422SP;
	case DRM_FORMAT_NV24:
		return VOP_FMT_YUV444SP;
	default:
		DRM_ERROR("unsupport format[%08x]\n", format);
		return -EINVAL;
	}
}

static bool is_alpha_support(uint32_t format)
{
	switch (format) {
	case DRM_FORMAT_ARGB8888:
		return true;
	default:
		return false;
	}
}

static void vop_enable(struct drm_crtc *crtc)
{
	struct vop *vop = to_vop(crtc);
	int ret;

	if (vop->is_enabled)
		return;

	ret = clk_enable(vop->hclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to enable hclk - %d\n", ret);
		return;
	}

	ret = clk_enable(vop->dclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to enable dclk - %d\n", ret);
		goto err_disable_hclk;
	}

	ret = clk_enable(vop->aclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to enable aclk - %d\n", ret);
		goto err_disable_dclk;
	}

	/*
	 * Slave iommu shares power, irq and clock with vop.  It was associated
	 * automatically with this master device via common driver code.
	 * Now that we have enabled the clock we attach it to the shared drm
	 * mapping.
	 */
	ret = rockchip_drm_dma_attach_device(vop->drm_dev, vop->dev);
	if (ret) {
		dev_err(vop->dev, "failed to attach dma mapping, %d\n", ret);
		goto err_disable_aclk;
	}

	/*
	 * At here, vop clock & iommu is enable, R/W vop regs would be safe.
	 */
	vop->is_enabled = true;

	spin_lock(&vop->reg_lock);

	VOP_CTRL_SET(vop, standby, 0);

	spin_unlock(&vop->reg_lock);

	enable_irq(vop->irq);

	drm_vblank_on(vop->drm_dev, vop->pipe);
	rockchip_dmc_get(&vop->dmc_nb);

	return;

err_disable_aclk:
	clk_disable(vop->aclk);
err_disable_dclk:
	clk_disable(vop->dclk);
err_disable_hclk:
	clk_disable(vop->hclk);
}

static void vop_disable(struct drm_crtc *crtc)
{
	struct vop *vop = to_vop(crtc);

	if (!vop->is_enabled)
		return;

	rockchip_dmc_put(&vop->dmc_nb);
	drm_vblank_off(crtc->dev, vop->pipe);

	disable_irq(vop->irq);

	/*
	 * TODO: Since standby doesn't take effect until the next vblank,
	 * when we turn off dclk below, the vop is probably still active.
	 */
	spin_lock(&vop->reg_lock);

	VOP_CTRL_SET(vop, standby, 1);

	spin_unlock(&vop->reg_lock);

	vop->is_enabled = false;
	/*
	 * disable dclk to stop frame scan, so we can safely detach iommu,
	 */
	clk_disable(vop->dclk);

	rockchip_drm_dma_detach_device(vop->drm_dev, vop->dev);

	clk_disable(vop->aclk);
	clk_disable(vop->hclk);
}

/*
 * We only track pending updates that require some action in the vblank irq.
 * That is, updates that do at least one of the following:
 *  - send a vblank event
 *  - disable front_fb
 *  - change enabled front fb
 *
 * We do not queue updates that:
 *  - keep the same fb (e.g., cursor moves)
 *  - enable a disabled window (NULL -> fb)
 * This function should be only called after wait for vop_win completion.
 */
static bool vop_win_update_needs_vblank(struct vop_win *vop_win,
					struct drm_framebuffer *fb,
					struct drm_pending_vblank_event *event)
{
	return event || (vop_win->front_fb && vop_win->front_fb != fb);
}

/*
 * We only need to wait for other owners of dma buffer if it exists
 * and it has synchronization object and if we do not already own
 * this buffer.
 * This function should be only called after wait for vop_win completion.
 */
static bool vop_win_update_needs_sync(struct vop_win *vop_win,
				      struct drm_framebuffer *fb,
				      struct dma_buf *dma_buf)
{
	return (fb && fb != vop_win->front_fb) && (dma_buf && dma_buf->resv);
}

static void vop_win_update(struct vop_win *vop_win)
{
	struct vop *vop = vop_win->vop;
	struct drm_crtc *crtc = &vop->crtc;
	struct drm_device *drm = crtc->dev;
	unsigned long flags;

	DRM_DEBUG_KMS("[PLANE:%u] [FB:%d->%d] update\n",
		      vop_win->base.base.id,
		      vop_win->front_fb ? vop_win->front_fb->base.id : -1,
		      vop_win->pending_fb ? vop_win->pending_fb->base.id : -1);

	if (vop_win->pending_event) {
		spin_lock_irqsave(&drm->event_lock, flags);
		drm_send_vblank_event(drm, vop->pipe, vop_win->pending_event);
		spin_unlock_irqrestore(&drm->event_lock, flags);
		vop_win->pending_event = NULL;
	}

	if (vop_win->front_fb)
		drm_framebuffer_unreference(vop_win->front_fb);
#ifdef CONFIG_DRM_DMA_SYNC
	if (vop_win->pending_fence ||
	    vop_win->front_fb != vop_win->pending_fb) {
		drm_fence_signal_and_put(&vop_win->fence);
		vop_win->fence = vop_win->pending_fence;
		vop_win->pending_fence = NULL;
	}
#endif
	vop_win->front_fb = vop_win->pending_fb;
	vop_win->pending_fb = NULL;
}

static void vop_win_update_commit(struct vop_win *vop_win, bool needs_vblank)
{
	const struct vop_win_data *win = vop_win->data;
	struct vop *vop = vop_win->vop;
	struct drm_framebuffer *fb = vop_win->pending_fb;
	uint32_t val;
	unsigned int y_vir_stride;
	enum vop_data_format format;
	bool is_alpha;

	is_alpha = is_alpha_support(fb->pixel_format);
	format = vop_convert_format(fb->pixel_format);
	y_vir_stride = fb->pitches[0] / (fb->bits_per_pixel >> 3);

	if (needs_vblank) {
		smp_wmb(); /* make sure all pending_* writes are complete */
		vop_win->pending = true;
	} else
		vop_win_update(vop_win);

	spin_lock(&vop->reg_lock);

	VOP_WIN_SET(vop, win, format, format);
	VOP_WIN_SET(vop, win, yrgb_vir, y_vir_stride);
	VOP_WIN_SET(vop, win, yrgb_mst, vop_win->pending_yrgb_mst);
	VOP_WIN_SET(vop, win, act_info, vop_win->pending_dsp_info);
	VOP_WIN_SET(vop, win, dsp_info, vop_win->pending_dsp_info);
	VOP_WIN_SET(vop, win, dsp_st, vop_win->pending_dsp_st);

	/* this completion doesn't need to protect the pending_* vars anymore */
	if (!needs_vblank)
		complete(&vop_win->completion);

	if (is_alpha) {
		VOP_WIN_SET(vop, win, dst_alpha_ctl,
			    DST_FACTOR_M0(ALPHA_SRC_INVERSE));
		val = SRC_ALPHA_EN(1) | SRC_COLOR_M0(ALPHA_SRC_PRE_MUL) |
			SRC_ALPHA_M0(ALPHA_STRAIGHT) |
			SRC_BLEND_M0(ALPHA_PER_PIX) |
			SRC_ALPHA_CAL_M0(ALPHA_NO_SATURATION) |
			SRC_FACTOR_M0(ALPHA_ONE);
		VOP_WIN_SET(vop, win, src_alpha_ctl, val);
	} else {
		VOP_WIN_SET(vop, win, src_alpha_ctl, SRC_ALPHA_EN(0));
	}

	VOP_WIN_SET(vop, win, enable, 1);

	vop_cfg_done(vop);
	spin_unlock(&vop->reg_lock);
}

#ifdef CONFIG_DRM_DMA_SYNC
static void vop_win_update_cb(struct drm_reservation_cb *rcb, void *params)
{
	struct vop_win *vop_win = params;
	bool needs_vblank = vop_win->pending_needs_vblank;

	vop_win->pending_needs_vblank = false;
	vop_win_update_commit(vop_win, needs_vblank);
}

static int vop_win_update_sync(struct vop_win *vop_win,
			       struct reservation_object *resv,
			       bool needs_vblank)
{
	struct fence *fence;
	int ret;

	BUG_ON(vop_win->pending_fence);

	vop_win->pending_needs_vblank = needs_vblank;
	ww_mutex_lock(&resv->lock, NULL);
	ret = reservation_object_reserve_shared(resv);
	if (ret < 0) {
		DRM_ERROR("Reserving space for shared fence failed: %d.\n",
			ret);
		goto err_mutex;
	}
	fence = drm_sw_fence_new(vop_win->fence_context,
				 atomic_add_return(1, &vop_win->fence_seqno));
	if (IS_ERR(fence)) {
		ret = PTR_ERR(fence);
		DRM_ERROR("Failed to create fence: %d.\n", ret);
		goto err_mutex;
	}
	vop_win->pending_fence = fence;
	drm_reservation_cb_init(&vop_win->rcb, vop_win_update_cb, vop_win);
	ret = drm_reservation_cb_add(&vop_win->rcb, resv, false);
	if (ret < 0) {
		DRM_ERROR("Adding reservation to callback failed: %d.\n", ret);
		goto err_fence;
	}
	drm_reservation_cb_done(&vop_win->rcb);
	reservation_object_add_shared_fence(resv, vop_win->pending_fence);
	ww_mutex_unlock(&resv->lock);
	return 0;
err_fence:
	fence_put(vop_win->pending_fence);
	vop_win->pending_fence = NULL;
err_mutex:
	ww_mutex_unlock(&resv->lock);

	return ret;
}
#else
static int vop_win_update_sync(struct vop_win *vop_win,
			       struct reservation_object *resv,
			       bool needs_vblank)
{
	vop_win_update_commit(vop_win, needs_vblank);
	return 0;
}
#endif /* CONFIG_DRM_DMA_SYNC */

static int vop_update_plane_event(struct drm_plane *plane,
				  struct drm_crtc *crtc,
				  struct drm_framebuffer *fb, int crtc_x,
				  int crtc_y, unsigned int crtc_w,
				  unsigned int crtc_h, uint32_t src_x,
				  uint32_t src_y, uint32_t src_w,
				  uint32_t src_h,
				  struct drm_pending_vblank_event *event)
{
	struct vop_win *vop_win = to_vop_win(plane);
	struct vop *vop = to_vop(crtc);
	struct drm_gem_object *obj;
	struct rockchip_gem_object *rk_obj;
	enum vop_data_format format;
	unsigned long offset;
	unsigned int actual_w;
	unsigned int actual_h;
	unsigned int dsp_stx;
	unsigned int dsp_sty;
	dma_addr_t yrgb_mst;
	uint32_t dsp_st;
	uint32_t dsp_info;
	bool visible;
	bool needs_vblank;
	int ret;
	struct drm_rect dest = {
		.x1 = crtc_x,
		.y1 = crtc_y,
		.x2 = crtc_x + crtc_w,
		.y2 = crtc_y + crtc_h,
	};
	struct drm_rect src = {
		/* 16.16 fixed point */
		.x1 = src_x,
		.y1 = src_y,
		.x2 = src_x + src_w,
		.y2 = src_y + src_h,
	};
	const struct drm_rect clip = {
		.x2 = crtc->mode.hdisplay,
		.y2 = crtc->mode.vdisplay,
	};
	bool can_position = plane->type != DRM_PLANE_TYPE_PRIMARY;

	ret = drm_plane_helper_check_update(plane, crtc, fb,
					    &src, &dest, &clip,
					    DRM_PLANE_HELPER_NO_SCALING,
					    DRM_PLANE_HELPER_NO_SCALING,
					    can_position, false, &visible);
	if (ret)
		return ret;

	if (!visible)
		return 0;

	format = vop_convert_format(fb->pixel_format);
	if (format < 0)
		return format;

	obj = rockchip_fb_get_gem_obj(fb, 0);
	if (!obj) {
		DRM_ERROR("fail to get rockchip gem object from framebuffer\n");
		return -EINVAL;
	}

	rk_obj = to_rockchip_obj(obj);

	actual_w = (src.x2 - src.x1) >> 16;
	actual_h = (src.y2 - src.y1) >> 16;
	dsp_info = ((actual_h - 1) << 16) | ((actual_w - 1) & 0xffff);

	crtc_x = max(0, crtc_x);
	crtc_y = max(0, crtc_y);
	dsp_stx = crtc_x + crtc->mode.htotal - crtc->mode.hsync_start;
	dsp_sty = crtc_y + crtc->mode.vtotal - crtc->mode.vsync_start;
	dsp_st = (dsp_sty << 16) | (dsp_stx & 0xffff);

	offset = (src.x1 >> 16) * (fb->bits_per_pixel >> 3);
	offset += (src.y1 >> 16) * fb->pitches[0];
	yrgb_mst = rk_obj->dma_addr + offset;

	wait_for_completion(&vop_win->completion);
	needs_vblank = vop_win_update_needs_vblank(vop_win, fb, event);
	if (needs_vblank) {
		ret = drm_vblank_get(crtc->dev, vop->pipe);
		if (ret) {
			DRM_ERROR("failed to get vblank, %d\n", ret);
			complete(&vop_win->completion);
			return ret;
		}
	}
	drm_framebuffer_reference(fb);
	vop_win->pending_fb = fb;
	vop_win->pending_event = event;
	/*
	 * It may not look like it but the completion does protect the
	 * following 3 variables because it will be signaled only after
	 * yrgb_mst write has been consumed by hardware during vblank.
	 */
	vop_win->pending_yrgb_mst = yrgb_mst;
	vop_win->pending_dsp_st = dsp_st;
	vop_win->pending_dsp_info = dsp_info;

	if (vop_win_update_needs_sync(vop_win, fb, obj->dma_buf)) {
		ret = vop_win_update_sync(vop_win, obj->dma_buf->resv,
					  needs_vblank);
		if (ret) {
			vop_win->pending_fb = NULL;
			vop_win->pending_event = NULL;
			drm_framebuffer_unreference(fb);
			drm_vblank_put(crtc->dev, vop->pipe);
			complete(&vop_win->completion);
		}
	} else {
		vop_win_update_commit(vop_win, needs_vblank);
		ret = 0;
	}

	return ret;
}

static int vop_update_plane(struct drm_plane *plane, struct drm_crtc *crtc,
			    struct drm_framebuffer *fb, int crtc_x, int crtc_y,
			    unsigned int crtc_w, unsigned int crtc_h,
			    uint32_t src_x, uint32_t src_y, uint32_t src_w,
			    uint32_t src_h)
{
	return vop_update_plane_event(plane, crtc, fb, crtc_x, crtc_y, crtc_w,
				      crtc_h, src_x, src_y, src_w, src_h,
				      NULL);
}

static int vop_update_primary_plane(struct drm_crtc *crtc,
				    struct drm_pending_vblank_event *event)
{
	unsigned int crtc_w, crtc_h;

	crtc_w = crtc->primary->fb->width - crtc->x;
	crtc_h = crtc->primary->fb->height - crtc->y;

	return vop_update_plane_event(crtc->primary, crtc, crtc->primary->fb,
				      0, 0, crtc_w, crtc_h, crtc->x << 16,
				      crtc->y << 16, crtc_w << 16,
				      crtc_h << 16, event);
}

static int vop_disable_plane(struct drm_plane *plane)
{
	struct vop_win *vop_win = to_vop_win(plane);
	const struct vop_win_data *win = vop_win->data;
	struct vop *vop;
	int ret;

	if (!plane->crtc)
		return 0;

	vop = to_vop(plane->crtc);

	wait_for_completion(&vop_win->completion);

	/* other pending_* are cleared to NULL already */
	vop_win->pending_yrgb_mst = 0;

	if (vop_win_update_needs_vblank(vop_win, NULL, NULL)) {
		ret = drm_vblank_get(plane->crtc->dev, vop->pipe);
		if (ret) {
			DRM_ERROR("failed to get vblank, %d\n", ret);
			complete(&vop_win->completion);
			return ret;
		}
		smp_wmb(); /* make sure all pending_* writes are complete */
		vop_win->pending = true;
	} else {
		vop_win_update(vop_win);
		complete(&vop_win->completion);
	}

	spin_lock(&vop->reg_lock);
	VOP_WIN_SET(vop, win, enable, 0);
	vop_cfg_done(vop);
	spin_unlock(&vop->reg_lock);

	return 0;
}

static void vop_plane_destroy(struct drm_plane *plane)
{
	vop_disable_plane(plane);
	drm_plane_cleanup(plane);
}

static const struct drm_plane_funcs vop_plane_funcs = {
	.update_plane = vop_update_plane,
	.disable_plane = vop_disable_plane,
	.destroy = vop_plane_destroy,
	.set_property = drm_atomic_plane_set_property
};

int rockchip_drm_crtc_mode_config(struct drm_crtc *crtc,
				  int connector_type,
				  int out_mode)
{
	struct vop *vop = to_vop(crtc);

	vop->connector_type = connector_type;
	vop->connector_out_mode = out_mode;

	return 0;
}

static struct vop *vop_from_pipe(struct drm_device *drm, int pipe)
{
	struct drm_crtc *crtc;
	int i = 0;

	list_for_each_entry(crtc, &drm->mode_config.crtc_list, head)
		if (i++ == pipe)
			return to_vop(crtc);

	return NULL;
}

int rockchip_drm_crtc_enable_vblank(struct drm_device *dev, int pipe)
{
	struct vop *vop = vop_from_pipe(dev, pipe);
	unsigned long flags;

	if (!vop->is_enabled)
		return -EPERM;

	spin_lock_irqsave(&vop->irq_lock, flags);

	vop_mask_write(vop, INTR_CTRL0, FS_INTR_MASK, FS_INTR_EN(1));

	spin_unlock_irqrestore(&vop->irq_lock, flags);

	return 0;
}

void rockchip_drm_crtc_disable_vblank(struct drm_device *dev, int pipe)
{
	struct vop *vop = vop_from_pipe(dev, pipe);
	unsigned long flags;

	if (!vop->is_enabled)
		return;

	spin_lock_irqsave(&vop->irq_lock, flags);
	vop_mask_write(vop, INTR_CTRL0, FS_INTR_MASK, FS_INTR_EN(0));
	spin_unlock_irqrestore(&vop->irq_lock, flags);
}

static void vop_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	DRM_DEBUG_KMS("crtc[%d] mode[%d]\n", crtc->base.id, mode);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		vop_enable(crtc);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		vop_disable(crtc);
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}
}

static void vop_crtc_prepare(struct drm_crtc *crtc)
{
	vop_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
}

static bool vop_crtc_mode_fixup(struct drm_crtc *crtc,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	if (adjusted_mode->htotal == 0 || adjusted_mode->vtotal == 0)
		return false;

	return true;
}

static int vop_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
				  struct drm_framebuffer *old_fb)
{
	int ret;

	crtc->x = x;
	crtc->y = y;

	ret = vop_update_primary_plane(crtc, NULL);
	if (ret < 0) {
		DRM_ERROR("fail to update plane\n");
		return ret;
	}

	return 0;
}

static int vop_crtc_mode_set(struct drm_crtc *crtc,
			     struct drm_display_mode *mode,
			     struct drm_display_mode *adjusted_mode,
			     int x, int y, struct drm_framebuffer *fb)
{
	struct vop *vop = to_vop(crtc);
	u16 hsync_len = adjusted_mode->hsync_end - adjusted_mode->hsync_start;
	u16 hdisplay = adjusted_mode->hdisplay;
	u16 htotal = adjusted_mode->htotal;
	u16 hact_st = adjusted_mode->htotal - adjusted_mode->hsync_start;
	u16 hact_end = hact_st + hdisplay;
	u16 vdisplay = adjusted_mode->vdisplay;
	u16 vtotal = adjusted_mode->vtotal;
	u16 vsync_len = adjusted_mode->vsync_end - adjusted_mode->vsync_start;
	u16 vact_st = adjusted_mode->vtotal - adjusted_mode->vsync_start;
	u16 vact_end = vact_st + vdisplay;
	int ret;
	uint32_t val;
	struct vop_win *vop_win = to_vop_win(crtc->primary);

	/*
	 * Wait for any pending updates to complete before full mode set.
	 *
	 * There is a funny quirk during full mode_set.  Full mode_set does:
	 * - disable dclk
	 * - set some mode register
	 * - call mode_set_base to setup the framebuffer
	 * - reset the "dclk" domain (immediately applies register updates)
	 * - enables dclk
	 *
	 * The call to mode_set_base will wait for the completion; however,
	 * since the call is made with dclk disabled, if there actually was any
	 * pending update it will never complete.  Therefore, we must wait for
	 * the completion first before disabling dclk, and complete it again so
	 * it is available for mode_set_base.
	 */
	wait_for_completion(&vop_win->completion);
	complete(&vop_win->completion);
	/*
	 * disable dclk to stop frame scan, so that we can safe config mode and
	 * enable iommu.
	 */
	clk_disable(vop->dclk);

	switch (vop->connector_type) {
	case DRM_MODE_CONNECTOR_LVDS:
		VOP_CTRL_SET(vop, rgb_en, 1);
		break;
	case DRM_MODE_CONNECTOR_eDP:
		VOP_CTRL_SET(vop, edp_en, 1);
		break;
	case DRM_MODE_CONNECTOR_HDMIA:
		VOP_CTRL_SET(vop, hdmi_en, 1);
		break;
	default:
		DRM_ERROR("unsupport connector_type[%d]\n",
			  vop->connector_type);
		return -EINVAL;
	};
	VOP_CTRL_SET(vop, out_mode, vop->connector_out_mode);

	val = 0x8;
	val |= (adjusted_mode->flags & DRM_MODE_FLAG_NHSYNC) ? 0 : 1;
	val |= (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC) ? 0 : (1 << 1);
	VOP_CTRL_SET(vop, pin_pol, val);

	VOP_CTRL_SET(vop, htotal_pw, (htotal << 16) | hsync_len);
	val = hact_st << 16;
	val |= hact_end;
	VOP_CTRL_SET(vop, hact_st_end, val);
	VOP_CTRL_SET(vop, hpost_st_end, val);

	VOP_CTRL_SET(vop, vtotal_pw, (vtotal << 16) | vsync_len);
	val = vact_st << 16;
	val |= vact_end;
	VOP_CTRL_SET(vop, vact_st_end, val);
	VOP_CTRL_SET(vop, vpost_st_end, val);

	ret = vop_crtc_mode_set_base(crtc, x, y, fb);
	if (ret)
		return ret;

	/*
	 * reset dclk, take all mode config affect, so the clk would run in
	 * correct frame.
	 */
	reset_control_assert(vop->dclk_rst);
	usleep_range(10, 20);
	reset_control_deassert(vop->dclk_rst);

	clk_set_rate(vop->dclk, adjusted_mode->clock * 1000);
	ret = clk_enable(vop->dclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to enable dclk - %d\n", ret);
		return ret;
	}

	return 0;
}

static void vop_crtc_commit(struct drm_crtc *crtc)
{
}

static const struct drm_crtc_helper_funcs vop_crtc_helper_funcs = {
	.dpms = vop_crtc_dpms,
	.prepare = vop_crtc_prepare,
	.mode_fixup = vop_crtc_mode_fixup,
	.mode_set = vop_crtc_mode_set,
	.mode_set_base = vop_crtc_mode_set_base,
	.commit = vop_crtc_commit,
};

static int vop_crtc_page_flip(struct drm_crtc *crtc,
			      struct drm_framebuffer *fb,
			      struct drm_pending_vblank_event *event,
			      uint32_t page_flip_flags)
{
	struct vop *vop = to_vop(crtc);
	struct drm_framebuffer *old_fb = crtc->primary->fb;
	int ret;

	/* when the page flip is requested, crtc should be on */
	if (!vop->is_enabled) {
		DRM_DEBUG("page flip request rejected because crtc is off.\n");
		return 0;
	}

	crtc->primary->fb = fb;

	ret = vop_update_primary_plane(crtc, event);
	if (ret)
		crtc->primary->fb = old_fb;

	return ret;
}

static void vop_crtc_destroy(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
}

static const struct drm_crtc_funcs vop_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.page_flip = vop_crtc_page_flip,
	.destroy = vop_crtc_destroy,
	.set_property = drm_atomic_crtc_set_property
};

static bool vop_win_pending_is_complete_fb(struct vop_win *vop_win)
{
	dma_addr_t yrgb_mst;

	/* check yrgb_mst to tell if pending_fb is now front */
	/* No locking needed because this is read only access */
	yrgb_mst = VOP_WIN_GET_YRGBADDR(vop_win->vop, vop_win->data);

	return yrgb_mst == vop_win->pending_yrgb_mst;
}

static bool vop_win_pending_is_complete_disable(struct vop_win *vop_win)
{
	/* check enable bit to tell if plane is now disabled */
	/* No locking needed because this is read only access */
	return VOP_WIN_GET(vop_win->vop, vop_win->data, enable) == 0;
}

static int dmc_notify(struct notifier_block *nb,
		      unsigned long action,
		      void *data)
{
	struct vop *vop = container_of(nb, struct vop, dmc_nb);
	int ret;

	reinit_completion(&vop->dmc_completion);
	ret = drm_vblank_get(vop->drm_dev, vop->pipe);
	if (ret < 0) {
		DRM_ERROR("failed to get vblank for dmc sync %d\n", ret);
		return NOTIFY_BAD;
	}

	if (!wait_for_completion_timeout(&vop->dmc_completion,
					 msecs_to_jiffies(50))) {
		DRM_ERROR("dmc wait for vblank completion timed out\n");
		drm_vblank_put(vop->drm_dev, vop->pipe);
		return NOTIFY_BAD;
	}
	drm_vblank_put(vop->drm_dev, vop->pipe);

	return NOTIFY_STOP;
}

static bool vop_win_pending_is_complete(struct vop_win *vop_win)
{
	if (vop_win->pending_fb)
		return vop_win_pending_is_complete_fb(vop_win);
	else
		return vop_win_pending_is_complete_disable(vop_win);
}

static void vop_win_process_pending(struct vop_win *vop_win)
{
	struct vop *vop = vop_win->vop;
	struct drm_crtc *crtc = &vop->crtc;

	if (!vop_win->pending)
		return;

	/* Ensure vop_win->pending_* are read after ->pending */
	smp_rmb();

	if (!vop_win_pending_is_complete(vop_win))
		return;

	DRM_DEBUG_KMS("[PLANE:%u] [FB:%d->%d] complete\n",
		      vop_win->base.base.id,
		      vop_win->front_fb ? vop_win->front_fb->base.id : -1,
		      vop_win->pending_fb ? vop_win->pending_fb->base.id : -1);

	drm_vblank_put(crtc->dev, vop->pipe);

	vop_win_update(vop_win);
	vop_win->pending = false;

	complete(&vop_win->completion);
}

static irqreturn_t vop_isr_thread(int irq, void *data)
{
	struct vop *vop = data;
	unsigned int i;

	for (i = 0; i < vop->data->win_size; i++)
		vop_win_process_pending(&vop->win[i]);

	return IRQ_HANDLED;
}

static irqreturn_t vop_isr(int irq, void *data)
{
	struct vop *vop = data;
	uint32_t intr0_reg, active_irqs;
	unsigned long flags;

	/*
	 * INTR_CTRL0 register has interrupt status, enable and clear bits, we
	 * must hold irq_lock to avoid a race with enable/disable_vblank().
	*/
	spin_lock_irqsave(&vop->irq_lock, flags);
	intr0_reg = vop_readl(vop, INTR_CTRL0);
	active_irqs = intr0_reg & INTR_MASK;
	/* Clear all active interrupt sources */
	if (active_irqs)
		vop_writel(vop, INTR_CTRL0,
			   intr0_reg | (active_irqs << INTR_CLR_SHIFT));
	spin_unlock_irqrestore(&vop->irq_lock, flags);

	/* This is expected for vop iommu irqs, since the irq is shared */
	if (!active_irqs)
		return IRQ_NONE;

	/* Only Frame Start Interrupt is enabled; other irqs are spurious. */
	if (!(active_irqs & FS_INTR)) {
		DRM_ERROR("Unknown VOP IRQs: %#02x\n", active_irqs);
		return IRQ_NONE;
	}

	drm_handle_vblank(vop->drm_dev, vop->pipe);
	if (!completion_done(&vop->dmc_completion))
		complete(&vop->dmc_completion);

	return IRQ_WAKE_THREAD;
}

static int vop_create_crtc(struct vop *vop)
{
	const struct vop_data *vop_data = vop->data;
	struct device *dev = vop->dev;
	struct drm_device *drm_dev = vop->drm_dev;
	struct drm_plane *primary, *cursor, *plane;
	struct drm_crtc *crtc = &vop->crtc;
	struct device_node *port;
	int ret;
	int i;

	/*
	 * Create drm_plane for primary and cursor planes first, since we need
	 * to pass them to drm_crtc_init_with_planes, which sets the
	 * "possible_crtcs" to the newly initialized crtc.
	 */
	for (i = 0; i < vop_data->win_size; i++) {
		struct vop_win *vop_win = &vop->win[i];
		const struct vop_win_data *win_data = vop_win->data;

		if (win_data->type != DRM_PLANE_TYPE_PRIMARY &&
		    win_data->type != DRM_PLANE_TYPE_CURSOR)
			continue;

		ret = drm_universal_plane_init(vop->drm_dev, &vop_win->base,
					       0, &vop_plane_funcs,
					       win_data->phy->data_formats,
					       win_data->phy->nformats,
					       win_data->type);
		if (ret) {
			DRM_ERROR("failed to initialize plane\n");
			goto err_cleanup_planes;
		}

		plane = &vop_win->base;
		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			primary = plane;
		else if (plane->type == DRM_PLANE_TYPE_CURSOR)
			cursor = plane;
	}

	ret = drm_crtc_init_with_planes(drm_dev, crtc, primary, cursor,
					&vop_crtc_funcs);
	if (ret)
		return ret;

	drm_crtc_helper_add(crtc, &vop_crtc_helper_funcs);

	/*
	 * Create drm_planes for overlay windows with possible_crtcs restricted
	 * to the newly created crtc.
	 */
	for (i = 0; i < vop_data->win_size; i++) {
		struct vop_win *vop_win = &vop->win[i];
		const struct vop_win_data *win_data = vop_win->data;
		unsigned long possible_crtcs = 1 << crtc->index;

		if (win_data->type != DRM_PLANE_TYPE_OVERLAY)
			continue;

		ret = drm_universal_plane_init(vop->drm_dev, &vop_win->base,
					       possible_crtcs,
					       &vop_plane_funcs,
					       win_data->phy->data_formats,
					       win_data->phy->nformats,
					       win_data->type);
		if (ret) {
			DRM_ERROR("failed to initialize overlay plane\n");
			goto err_cleanup_crtc;
		}
	}

	port = of_get_child_by_name(dev->of_node, "port");
	if (!port) {
		DRM_ERROR("no port node found in %s\n",
			  dev->of_node->full_name);
		goto err_cleanup_crtc;
	}

	vop->dmc_nb.notifier_call = dmc_notify;
	init_completion(&vop->dmc_completion);

	crtc->port = port;
	vop->pipe = crtc->index;

	return 0;

err_cleanup_crtc:
	drm_crtc_cleanup(crtc);
err_cleanup_planes:
	list_for_each_entry(plane, &drm_dev->mode_config.plane_list, head)
		drm_plane_cleanup(plane);
	return ret;
}

static void vop_destroy_crtc(struct vop *vop)
{
	struct drm_crtc *crtc = &vop->crtc;

	of_node_put(crtc->port);
	drm_crtc_cleanup(crtc);
}

static int vop_initial(struct vop *vop)
{
	const struct vop_data *vop_data = vop->data;
	const struct vop_reg_data *init_table = vop_data->init_table;
	struct reset_control *ahb_rst;
	int i, ret;

	vop->hclk = devm_clk_get(vop->dev, "hclk_vop");
	if (IS_ERR(vop->hclk)) {
		dev_err(vop->dev, "failed to get hclk source\n");
		return PTR_ERR(vop->hclk);
	}
	vop->aclk = devm_clk_get(vop->dev, "aclk_vop");
	if (IS_ERR(vop->aclk)) {
		dev_err(vop->dev, "failed to get aclk source\n");
		return PTR_ERR(vop->aclk);
	}
	vop->dclk = devm_clk_get(vop->dev, "dclk_vop");
	if (IS_ERR(vop->dclk)) {
		dev_err(vop->dev, "failed to get dclk source\n");
		return PTR_ERR(vop->dclk);
	}

	ret = clk_prepare(vop->hclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to prepare hclk\n");
		return ret;
	}

	ret = clk_prepare(vop->dclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to prepare dclk\n");
		goto err_unprepare_hclk;
	}

	ret = clk_prepare(vop->aclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to prepare aclk\n");
		goto err_unprepare_dclk;
	}

	/*
	 * enable hclk, so that we can config vop register.
	 */
	ret = clk_enable(vop->hclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to prepare aclk\n");
		goto err_unprepare_aclk;
	}
	/*
	 * do hclk_reset, reset all vop registers.
	 */
	ahb_rst = devm_reset_control_get(vop->dev, "ahb");
	if (IS_ERR(ahb_rst)) {
		dev_err(vop->dev, "failed to get ahb reset\n");
		ret = PTR_ERR(ahb_rst);
		goto err_disable_hclk;
	}
	reset_control_assert(ahb_rst);
	usleep_range(10, 20);
	reset_control_deassert(ahb_rst);

	memcpy(vop->regsbak, vop->regs, vop->len);

	for (i = 0; i < vop_data->table_size; i++)
		vop_writel(vop, init_table[i].offset, init_table[i].value);

	for (i = 0; i < vop_data->win_size; i++) {
		const struct vop_win_data *win = &vop_data->win[i];
		VOP_WIN_SET(vop, win, enable, 0);
	}

	vop_cfg_done(vop);

	/*
	 * do dclk_reset, let all config take affect.
	 */
	vop->dclk_rst = devm_reset_control_get(vop->dev, "dclk");
	if (IS_ERR(vop->dclk_rst)) {
		dev_err(vop->dev, "failed to get dclk reset\n");
		ret = PTR_ERR(vop->dclk_rst);
		goto err_unprepare_aclk;
	}
	reset_control_assert(vop->dclk_rst);
	usleep_range(10, 20);
	reset_control_deassert(vop->dclk_rst);

	clk_disable(vop->hclk);

	vop->is_enabled = false;

	return 0;

err_disable_hclk:
	clk_disable(vop->hclk);
err_unprepare_aclk:
	clk_unprepare(vop->aclk);
err_unprepare_dclk:
	clk_unprepare(vop->dclk);
err_unprepare_hclk:
	clk_unprepare(vop->hclk);
	return ret;
}

/*
 * Initialize the vop->win array elements.
 */
static void vop_win_init(struct vop *vop)
{
	const struct vop_data *vop_data = vop->data;
	unsigned int i;

	for (i = 0; i < vop_data->win_size; i++) {
		struct vop_win *vop_win = &vop->win[i];
		const struct vop_win_data *win_data = &vop_data->win[i];

		vop_win->data = win_data;
		vop_win->vop = vop;
		init_completion(&vop_win->completion);
		/* completion is initially available */
		complete(&vop_win->completion);
#ifdef CONFIG_DRM_DMA_SYNC
		vop_win->fence_context = fence_context_alloc(1);
		atomic_set(&vop_win->fence_seqno, 0);
#endif
	}
}

static int vop_bind(struct device *dev, struct device *master, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct of_device_id *of_id;
	const struct vop_data *vop_data;
	struct drm_device *drm_dev = data;
	struct vop *vop;
	struct resource *res;
	size_t alloc_size;
	int ret;

	of_id = of_match_device(vop_driver_dt_match, dev);
	vop_data = of_id->data;
	if (!vop_data)
		return -ENODEV;

	/* Allocate vop struct and its vop_win array */
	alloc_size = sizeof(*vop) + sizeof(*vop->win) * vop_data->win_size;
	vop = devm_kzalloc(dev, alloc_size, GFP_KERNEL);
	if (!vop)
		return -ENOMEM;

	vop->dev = dev;
	vop->data = vop_data;
	vop->drm_dev = drm_dev;
	dev_set_drvdata(dev, vop);

	vop_win_init(vop);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vop->len = resource_size(res);
	vop->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(vop->regs))
		return PTR_ERR(vop->regs);

	vop->regsbak = devm_kzalloc(dev, vop->len, GFP_KERNEL);
	if (!vop->regsbak)
		return -ENOMEM;

	ret = vop_initial(vop);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot initial vop dev - err %d\n", ret);
		return ret;
	}

	vop->irq = platform_get_irq(pdev, 0);
	if (vop->irq < 0) {
		dev_err(dev, "cannot find irq for vop\n");
		return vop->irq;
	}

	spin_lock_init(&vop->reg_lock);
	spin_lock_init(&vop->irq_lock);

	ret = devm_request_threaded_irq(dev, vop->irq, vop_isr, vop_isr_thread,
					IRQF_SHARED, dev_name(dev), vop);
	if (ret) {
		dev_err(dev, "cannot request irq%d - err %d\n", vop->irq, ret);
		return ret;
	}

	/* IRQ is initially disabled; it gets enabled in power_on */
	disable_irq(vop->irq);

	ret = vop_create_crtc(vop);
	if (ret)
		return ret;

	pm_runtime_enable(&pdev->dev);
	return 0;
}

static void vop_unbind(struct device *dev, struct device *master, void *data)
{
	struct vop *vop = dev_get_drvdata(dev);

	pm_runtime_disable(dev);
	vop_destroy_crtc(vop);
}

static const struct component_ops vop_component_ops = {
	.bind = vop_bind,
	.unbind = vop_unbind,
};

static int vop_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (!dev->of_node) {
		dev_err(dev, "can't find vop devices\n");
		return -ENODEV;
	}

	return component_add(dev, &vop_component_ops);
}

static int vop_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &vop_component_ops);

	return 0;
}

struct platform_driver vop_platform_driver = {
	.probe = vop_probe,
	.remove = vop_remove,
	.driver = {
		.name = "rockchip-vop",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(vop_driver_dt_match),
	},
};

module_platform_driver(vop_platform_driver);

MODULE_AUTHOR("Mark Yao <mark.yao@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP VOP Driver");
MODULE_LICENSE("GPL v2");
