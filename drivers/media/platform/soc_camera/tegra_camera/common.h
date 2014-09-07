/*
 * Copyright (c) 2012-2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_CAMERA_COMMON_H_
#define _TEGRA_CAMERA_COMMON_H_

#include <linux/videodev2.h>
#include <linux/host1x.h>

#include <media/videobuf2-dma-contig.h>
#include <media/soc_camera.h>
#include <dt-bindings/media/tegra_camera.h>

#define TEGRA_CSI_NUM_LANES	9

struct tegra_camera_port {
	int			port;
	int			lanes[TEGRA_CSI_NUM_LANES];
	int			num_lanes;
	int			clk_lane;
	bool			continuous_clk;
	bool			flip_h;
	bool			flip_v;

	/* Some values derived from lanes, num_lanes, and clk_lane */
	bool			cila_used;
	bool			cilb_used;
	bool			cilc_used;
	bool			cild_used;
	bool			cile_used;
};

/* Buffer for one video frame */
struct tegra_camera_buffer {
	struct vb2_buffer		vb; /* v4l buffer must be first */
	struct list_head		queue;
	struct soc_camera_device	*icd;
	int				output_channel;

	/*
	 * Various buffer addresses shadowed so we don't have to recalculate
	 * per frame. These are calculated during videobuf_prepare.
	 */
	dma_addr_t			buffer_addr;
	dma_addr_t			buffer_addr_u;
	dma_addr_t			buffer_addr_v;
	dma_addr_t			start_addr;
	dma_addr_t			start_addr_u;
	dma_addr_t			start_addr_v;
};

static inline struct tegra_camera_buffer *to_tegra_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct tegra_camera_buffer, vb);
}

struct tegra_camera;

struct tegra_camera_clk {
	const char	*name;
	struct clk	*clk;
	u32		freq;
};

struct tegra_camera_ops {
	int (*init)(struct tegra_camera *cam);
	void (*deinit)(struct tegra_camera *cam);
	int (*clock_start)(struct tegra_camera *cam);
	void (*clock_stop)(struct tegra_camera *cam);
	int (*activate)(struct tegra_camera *cam);
	void (*deactivate)(struct tegra_camera *cam);
	bool (*port_is_valid)(struct tegra_camera_port *cam_port);
	int (*capture_frame)(struct tegra_camera *cam,
			     struct tegra_camera_buffer *buf);
};

struct tegra_camera {
	struct soc_camera_host		ici;
	struct host1x_client		client;
	struct platform_device		*pdev;

	struct tegra_camera_port	ports[TEGRA_CAMERA_PORT_NUM];

	struct regulator		*reg;

	struct clk			*clk;
	struct reset_control		*rst;

	struct tegra_camera_clk		*clks;
	int				num_clks;

	struct tegra_camera_ops_old	*ops;

	void __iomem			*reg_base;
	spinlock_t			videobuf_queue_lock;
	struct list_head		capture;
	struct vb2_buffer		*active;
	struct vb2_alloc_ctx		*alloc_ctx;
	enum v4l2_field			field;
	int				sequence_a;
	int				sequence_b;

	struct work_struct		work;
	struct mutex			work_mutex;

	/* syncpts */
	struct host1x_syncpt		*syncpt_id_csi_a;
	struct host1x_syncpt		*syncpt_id_csi_b;
	struct host1x_syncpt		*syncpt_id_vip;

	/* syncpt values */
	u32				syncpt_csi_a;
	u32				syncpt_csi_b;
	u32				syncpt_vip;

	/* Debug */
	int				num_frames;
	int				enable_refcnt;

	/* Test Pattern Generator mode */
	int				tpg_mode;

	int				sof;
};

extern struct platform_driver tegra_camera_driver;
void tegra_camera_register_ops(struct tegra_camera_ops *ops);

#endif
