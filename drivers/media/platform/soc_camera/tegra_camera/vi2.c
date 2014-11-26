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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/regulator/consumer.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>

#include <soc/tegra/pmc.h>

#include "common.h"

#define TC_VI_REG_RD(dev, offset) readl(dev->reg_base + offset)
#define TC_VI_REG_WT(dev, offset, val) writel(val, dev->reg_base + offset)

#define TEGRA_SYNCPT_CSI_WAIT_TIMEOUT			200
#define TEGRA_SYNCPT_RETRY_COUNT			10

#define TEGRA_VI_CFG_VI_INCR_SYNCPT			0x000
#define TEGRA_VI_CFG_VI_INCR_SYNCPT_CNTRL		0x004
#define TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR		0x008
#define TEGRA_VI_CFG_CTXSW				0x020
#define TEGRA_VI_CFG_INTSTATUS				0x024
#define TEGRA_VI_CFG_PWM_CONTROL			0x038
#define TEGRA_VI_CFG_PWM_HIGH_PULSE			0x03c
#define TEGRA_VI_CFG_PWM_LOW_PULSE			0x040
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_A			0x044
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_B			0x048
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_C			0x04c
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_D			0x050
#define TEGRA_VI_CFG_VGP1				0x064
#define TEGRA_VI_CFG_VGP2				0x068
#define TEGRA_VI_CFG_VGP3				0x06c
#define TEGRA_VI_CFG_VGP4				0x070
#define TEGRA_VI_CFG_VGP5				0x074
#define TEGRA_VI_CFG_VGP6				0x078
#define TEGRA_VI_CFG_INTERRUPT_MASK			0x08c
#define TEGRA_VI_CFG_INTERRUPT_TYPE_SELECT		0x090
#define TEGRA_VI_CFG_INTERRUPT_POLARITY_SELECT		0x094
#define TEGRA_VI_CFG_INTERRUPT_STATUS			0x098
#define TEGRA_VI_CFG_VGP_SYNCPT_CONFIG			0x0ac
#define TEGRA_VI_CFG_VI_SW_RESET			0x0b4
#define TEGRA_VI_CFG_CG_CTRL				0x0b8
#define TEGRA_VI_CFG_VI_MCCIF_FIFOCTRL			0x0e4
#define TEGRA_VI_CFG_TIMEOUT_WCOAL_VI			0x0e8
#define TEGRA_VI_CFG_DVFS				0x0f0
#define TEGRA_VI_CFG_RESERVE				0x0f4
#define TEGRA_VI_CFG_RESERVE_1				0x0f8

#define TEGRA_VI_CSI_0_SW_RESET				0x100
#define TEGRA_VI_CSI_0_SINGLE_SHOT			0x104
#define TEGRA_VI_CSI_0_SINGLE_SHOT_STATE_UPDATE		0x108
#define TEGRA_VI_CSI_0_IMAGE_DEF			0x10c
#define TEGRA_VI_CSI_0_RGB2Y_CTRL			0x110
#define TEGRA_VI_CSI_0_MEM_TILING			0x114
#define TEGRA_VI_CSI_0_CSI_IMAGE_SIZE			0x118
#define TEGRA_VI_CSI_0_CSI_IMAGE_SIZE_WC		0x11c
#define TEGRA_VI_CSI_0_CSI_IMAGE_DT			0x120
#define TEGRA_VI_CSI_0_SURFACE0_OFFSET_MSB		0x124
#define TEGRA_VI_CSI_0_SURFACE0_OFFSET_LSB		0x128
#define TEGRA_VI_CSI_0_SURFACE1_OFFSET_MSB		0x12c
#define TEGRA_VI_CSI_0_SURFACE1_OFFSET_LSB		0x130
#define TEGRA_VI_CSI_0_SURFACE2_OFFSET_MSB		0x134
#define TEGRA_VI_CSI_0_SURFACE2_OFFSET_LSB		0x138
#define TEGRA_VI_CSI_0_SURFACE0_BF_OFFSET_MSB		0x13c
#define TEGRA_VI_CSI_0_SURFACE0_BF_OFFSET_LSB		0x140
#define TEGRA_VI_CSI_0_SURFACE1_BF_OFFSET_MSB		0x144
#define TEGRA_VI_CSI_0_SURFACE1_BF_OFFSET_LSB		0x148
#define TEGRA_VI_CSI_0_SURFACE2_BF_OFFSET_MSB		0x14c
#define TEGRA_VI_CSI_0_SURFACE2_BF_OFFSET_LSB		0x150
#define TEGRA_VI_CSI_0_SURFACE0_STRIDE			0x154
#define TEGRA_VI_CSI_0_SURFACE1_STRIDE			0x158
#define TEGRA_VI_CSI_0_SURFACE2_STRIDE			0x15c
#define TEGRA_VI_CSI_0_SURFACE_HEIGHT0			0x160
#define TEGRA_VI_CSI_0_ISPINTF_CONFIG			0x164
#define TEGRA_VI_CSI_0_ERROR_STATUS			0x184
#define TEGRA_VI_CSI_0_ERROR_INT_MASK			0x188
#define TEGRA_VI_CSI_0_WD_CTRL				0x18c
#define TEGRA_VI_CSI_0_WD_PERIOD			0x190

#define TEGRA_VI_CSI_1_SW_RESET				0x200
#define TEGRA_VI_CSI_1_SINGLE_SHOT			0x204
#define TEGRA_VI_CSI_1_SINGLE_SHOT_STATE_UPDATE		0x208
#define TEGRA_VI_CSI_1_IMAGE_DEF			0x20c
#define TEGRA_VI_CSI_1_RGB2Y_CTRL			0x210
#define TEGRA_VI_CSI_1_MEM_TILING			0x214
#define TEGRA_VI_CSI_1_CSI_IMAGE_SIZE			0x218
#define TEGRA_VI_CSI_1_CSI_IMAGE_SIZE_WC		0x21c
#define TEGRA_VI_CSI_1_CSI_IMAGE_DT			0x220
#define TEGRA_VI_CSI_1_SURFACE0_OFFSET_MSB		0x224
#define TEGRA_VI_CSI_1_SURFACE0_OFFSET_LSB		0x228
#define TEGRA_VI_CSI_1_SURFACE1_OFFSET_MSB		0x22c
#define TEGRA_VI_CSI_1_SURFACE1_OFFSET_LSB		0x230
#define TEGRA_VI_CSI_1_SURFACE2_OFFSET_MSB		0x234
#define TEGRA_VI_CSI_1_SURFACE2_OFFSET_LSB		0x238
#define TEGRA_VI_CSI_1_SURFACE0_BF_OFFSET_MSB		0x23c
#define TEGRA_VI_CSI_1_SURFACE0_BF_OFFSET_LSB		0x240
#define TEGRA_VI_CSI_1_SURFACE1_BF_OFFSET_MSB		0x244
#define TEGRA_VI_CSI_1_SURFACE1_BF_OFFSET_LSB		0x248
#define TEGRA_VI_CSI_1_SURFACE2_BF_OFFSET_MSB		0x24c
#define TEGRA_VI_CSI_1_SURFACE2_BF_OFFSET_LSB		0x250
#define TEGRA_VI_CSI_1_SURFACE0_STRIDE			0x254
#define TEGRA_VI_CSI_1_SURFACE1_STRIDE			0x258
#define TEGRA_VI_CSI_1_SURFACE2_STRIDE			0x25c
#define TEGRA_VI_CSI_1_SURFACE_HEIGHT0			0x260
#define TEGRA_VI_CSI_1_ISPINTF_CONFIG			0x264
#define TEGRA_VI_CSI_1_ERROR_STATUS			0x284
#define TEGRA_VI_CSI_1_ERROR_INT_MASK			0x288
#define TEGRA_VI_CSI_1_WD_CTRL				0x28c
#define TEGRA_VI_CSI_1_WD_PERIOD			0x290

#define TEGRA_CSI_CSI_CAP_CIL				0x808
#define TEGRA_CSI_CSI_CAP_CSI				0x818
#define TEGRA_CSI_CSI_CAP_PP				0x828
#define TEGRA_CSI_INPUT_STREAM_A_CONTROL		0x838
#define TEGRA_CSI_PIXEL_STREAM_A_CONTROL0		0x83c
#define TEGRA_CSI_PIXEL_STREAM_A_CONTROL1		0x840
#define TEGRA_CSI_PIXEL_STREAM_A_GAP			0x844
#define TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND		0x848
#define TEGRA_CSI_PIXEL_STREAM_A_EXPECTED_FRAME		0x84c
#define TEGRA_CSI_CSI_PIXEL_PARSER_A_INTERRUPT_MASK	0x850
#define TEGRA_CSI_CSI_PIXEL_PARSER_A_STATUS		0x854
#define TEGRA_CSI_CSI_SW_SENSOR_A_RESET			0x858
#define TEGRA_CSI_INPUT_STREAM_B_CONTROL		0x86c
#define TEGRA_CSI_PIXEL_STREAM_B_CONTROL0		0x870
#define TEGRA_CSI_PIXEL_STREAM_B_CONTROL1		0x874
#define TEGRA_CSI_PIXEL_STREAM_B_GAP			0x878
#define TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND		0x87c
#define TEGRA_CSI_PIXEL_STREAM_B_EXPECTED_FRAME		0x880
#define TEGRA_CSI_CSI_PIXEL_PARSER_B_INTERRUPT_MASK	0x884
#define TEGRA_CSI_CSI_PIXEL_PARSER_B_STATUS		0x888
#define TEGRA_CSI_CSI_SW_SENSOR_B_RESET			0x88c
#define TEGRA_CSI_PHY_CIL_COMMAND			0x908
#define TEGRA_CSI_CIL_PAD_CONFIG0			0x90c

#define TEGRA_CSI_CILA_PAD_CONFIG0			0x92c
#define TEGRA_CSI_CILA_PAD_CONFIG1			0x930
#define TEGRA_CSI_PHY_CILA_CONTROL0			0x934
#define TEGRA_CSI_CSI_CIL_A_INTERRUPT_MASK		0x938
#define TEGRA_CSI_CSI_CIL_A_STATUS			0x93c
#define TEGRA_CSI_CSI_CILA_STATUS			0x940
#define TEGRA_CSI_CIL_A_ESCAPE_MODE_COMMAND		0x944
#define TEGRA_CSI_CIL_A_ESCAPE_MODE_DATA		0x948
#define TEGRA_CSI_CSICIL_SW_SENSOR_A_RESET		0x94c

#define TEGRA_CSI_CILB_PAD_CONFIG0			0x960
#define TEGRA_CSI_CILB_PAD_CONFIG1			0x964
#define TEGRA_CSI_PHY_CILB_CONTROL0			0x968
#define TEGRA_CSI_CSI_CIL_B_INTERRUPT_MASK		0x96c
#define TEGRA_CSI_CSI_CIL_B_STATUS			0x970
#define TEGRA_CSI_CSI_CILB_STATUS			0x974
#define TEGRA_CSI_CIL_B_ESCAPE_MODE_COMMAND		0x978
#define TEGRA_CSI_CIL_B_ESCAPE_MODE_DATA		0x97c
#define TEGRA_CSI_CSICIL_SW_SENSOR_B_RESET		0x980

#define TEGRA_CSI_CILC_PAD_CONFIG0			0x994
#define TEGRA_CSI_CILC_PAD_CONFIG1			0x998
#define TEGRA_CSI_PHY_CILC_CONTROL0			0x99c
#define TEGRA_CSI_CSI_CIL_C_INTERRUPT_MASK		0x9a0
#define TEGRA_CSI_CSI_CIL_C_STATUS			0x9a4
#define TEGRA_CSI_CSI_CILC_STATUS			0x9a8
#define TEGRA_CSI_CIL_C_ESCAPE_MODE_COMMAND		0x9ac
#define TEGRA_CSI_CIL_C_ESCAPE_MODE_DATA		0x9b0
#define TEGRA_CSI_CSICIL_SW_SENSOR_C_RESET		0x9b4

#define TEGRA_CSI_CILD_PAD_CONFIG0			0x9c8
#define TEGRA_CSI_CILD_PAD_CONFIG1			0x9cc
#define TEGRA_CSI_PHY_CILD_CONTROL0			0x9d0
#define TEGRA_CSI_CSI_CIL_D_INTERRUPT_MASK		0x9d4
#define TEGRA_CSI_CSI_CIL_D_STATUS			0x9d8
#define TEGRA_CSI_CSI_CILD_STATUS			0x9dc
#define TEGRA_CSI_CIL_D_ESCAPE_MODE_COMMAND		0x9ec
#define TEGRA_CSI_CIL_D_ESCAPE_MODE_DATA		0x9f0
#define TEGRA_CSI_CSICIL_SW_SENSOR_D_RESET		0x9f4

#define TEGRA_CSI_CILE_PAD_CONFIG0			0xa08
#define TEGRA_CSI_CILE_PAD_CONFIG1			0xa0c
#define TEGRA_CSI_PHY_CILE_CONTROL0			0xa10
#define TEGRA_CSI_CSI_CIL_E_INTERRUPT_MASK		0xa14
#define TEGRA_CSI_CSI_CIL_E_STATUS			0xa18
#define TEGRA_CSI_CIL_E_ESCAPE_MODE_COMMAND		0xa1c
#define TEGRA_CSI_CIL_E_ESCAPE_MODE_DATA		0xa20
#define TEGRA_CSI_CSICIL_SW_SENSOR_E_RESET		0xa24

#define TEGRA_CSI_PATTERN_GENERATOR_CTRL_A		0xa68
#define TEGRA_CSI_PG_BLANK_A				0xa6c
#define TEGRA_CSI_PG_PHASE_A				0xa70
#define TEGRA_CSI_PG_RED_FREQ_A				0xa74
#define TEGRA_CSI_PG_RED_FREQ_RATE_A			0xa78
#define TEGRA_CSI_PG_GREEN_FREQ_A			0xa7c
#define TEGRA_CSI_PG_GREEN_FREQ_RATE_A			0xa80
#define TEGRA_CSI_PG_BLUE_FREQ_A			0xa84
#define TEGRA_CSI_PG_BLUE_FREQ_RATE_A			0xa88

#define TEGRA_CSI_PATTERN_GENERATOR_CTRL_B		0xa9c
#define TEGRA_CSI_PG_BLANK_B				0xaa0
#define TEGRA_CSI_PG_PHASE_B				0xaa4
#define TEGRA_CSI_PG_RED_FREQ_B				0xaa8
#define TEGRA_CSI_PG_RED_FREQ_RATE_B			0xaac
#define TEGRA_CSI_PG_GREEN_FREQ_B			0xab0
#define TEGRA_CSI_PG_GREEN_FREQ_RATE_B			0xab4
#define TEGRA_CSI_PG_BLUE_FREQ_B			0xab8
#define TEGRA_CSI_PG_BLUE_FREQ_RATE_B			0xabc

#define TEGRA_CSI_DPCM_CTRL_A				0xad0
#define TEGRA_CSI_DPCM_CTRL_B				0xad4
#define TEGRA_CSI_STALL_COUNTER				0xae8
#define TEGRA_CSI_CSI_READONLY_STATUS			0xaec
#define TEGRA_CSI_CSI_SW_STATUS_RESET			0xaf0
#define TEGRA_CSI_CLKEN_OVERRIDE			0xaf4
#define TEGRA_CSI_DEBUG_CONTROL				0xaf8
#define TEGRA_CSI_DEBUG_COUNTER_0			0xafc
#define TEGRA_CSI_DEBUG_COUNTER_1			0xb00
#define TEGRA_CSI_DEBUG_COUNTER_2			0xb04

/* These go into the TEGRA_VI_CSI_n_IMAGE_DEF registers bits 23:16 */
#define TEGRA_IMAGE_FORMAT_T_L8				16
#define TEGRA_IMAGE_FORMAT_T_R16_I			32
#define TEGRA_IMAGE_FORMAT_T_B5G6R5			33
#define TEGRA_IMAGE_FORMAT_T_R5G6B5			34
#define TEGRA_IMAGE_FORMAT_T_A1B5G5R5			35
#define TEGRA_IMAGE_FORMAT_T_A1R5G5B5			36
#define TEGRA_IMAGE_FORMAT_T_B5G5R5A1			37
#define TEGRA_IMAGE_FORMAT_T_R5G5B5A1			38
#define TEGRA_IMAGE_FORMAT_T_A4B4G4R4			39
#define TEGRA_IMAGE_FORMAT_T_A4R4G4B4			40
#define TEGRA_IMAGE_FORMAT_T_B4G4R4A4			41
#define TEGRA_IMAGE_FORMAT_T_R4G4B4A4			42
#define TEGRA_IMAGE_FORMAT_T_A8B8G8R8			64
#define TEGRA_IMAGE_FORMAT_T_A8R8G8B8			65
#define TEGRA_IMAGE_FORMAT_T_B8G8R8A8			66
#define TEGRA_IMAGE_FORMAT_T_R8G8B8A8			67
#define TEGRA_IMAGE_FORMAT_T_A2B10G10R10		68
#define TEGRA_IMAGE_FORMAT_T_A2R10G10B10		69
#define TEGRA_IMAGE_FORMAT_T_B10G10R10A2		70
#define TEGRA_IMAGE_FORMAT_T_R10G10B10A2		71
#define TEGRA_IMAGE_FORMAT_T_A8Y8U8V8			193
#define TEGRA_IMAGE_FORMAT_T_V8U8Y8A8			194
#define TEGRA_IMAGE_FORMAT_T_A2Y10U10V10		197
#define TEGRA_IMAGE_FORMAT_T_V10U10Y10A2		198
#define TEGRA_IMAGE_FORMAT_T_Y8_U8__Y8_V8		200
#define TEGRA_IMAGE_FORMAT_T_Y8_V8__Y8_U8		201
#define TEGRA_IMAGE_FORMAT_T_U8_Y8__V8_Y8		202
#define TEGRA_IMAGE_FORMAT_T_T_V8_Y8__U8_Y8		203
#define TEGRA_IMAGE_FORMAT_T_T_Y8__U8__V8_N444		224
#define TEGRA_IMAGE_FORMAT_T_Y8__U8V8_N444		225
#define TEGRA_IMAGE_FORMAT_T_Y8__V8U8_N444		226
#define TEGRA_IMAGE_FORMAT_T_Y8__U8__V8_N422		227
#define TEGRA_IMAGE_FORMAT_T_Y8__U8V8_N422		228
#define TEGRA_IMAGE_FORMAT_T_Y8__V8U8_N422		229
#define TEGRA_IMAGE_FORMAT_T_Y8__U8__V8_N420		230
#define TEGRA_IMAGE_FORMAT_T_Y8__U8V8_N420		231
#define TEGRA_IMAGE_FORMAT_T_Y8__V8U8_N420		232
#define TEGRA_IMAGE_FORMAT_T_X2Lc10Lb10La10		233
#define TEGRA_IMAGE_FORMAT_T_A2R6R6R6R6R6		234

/* These go into the TEGRA_VI_CSI_n_CSI_IMAGE_DT registers bits 7:0 */
#define TEGRA_IMAGE_DT_YUV420_8				24
#define TEGRA_IMAGE_DT_YUV420_10			25
#define TEGRA_IMAGE_DT_YUV420CSPS_8			28
#define TEGRA_IMAGE_DT_YUV420CSPS_10			29
#define TEGRA_IMAGE_DT_YUV422_8				30
#define TEGRA_IMAGE_DT_YUV422_10			31
#define TEGRA_IMAGE_DT_RGB444				32
#define TEGRA_IMAGE_DT_RGB555				33
#define TEGRA_IMAGE_DT_RGB565				34
#define TEGRA_IMAGE_DT_RGB666				35
#define TEGRA_IMAGE_DT_RGB888				36
#define TEGRA_IMAGE_DT_RAW6				40
#define TEGRA_IMAGE_DT_RAW7				41
#define TEGRA_IMAGE_DT_RAW8				42
#define TEGRA_IMAGE_DT_RAW10				43
#define TEGRA_IMAGE_DT_RAW12				44
#define TEGRA_IMAGE_DT_RAW14				45

/* Clock settings for camera */
static struct tegra_camera_clk vi2_clks[] = {
	{
		.name = "csi",
		.freq = 408000000,
	},
	{
		.name = "csus",
		.freq = 0,
	},
	{
		.name = "sclk",
		.freq = 80000000,
	},
	{
		.name = "cilab",
		.freq = 102000000,
	},
	{
		.name = "cilcd",
		.freq = 102000000,
	},
	{
		.name = "cile",
		.freq = 102000000,
	},
	{
		.name = "vi_sensor",
		.freq = 24000000,
	},
	{
		.name = "vi_sensor2",
		.freq = 24000000,
	},
	{
		.name = "cam_mclk",
		.freq = 24000000,
	},
	{
		.name = "cam_mclk2",
		.freq = 24000000,
	},
	{
		.name = "vim2_clk",
		.freq = 24000000,
	},
};

static int vi2_host1x_init(struct host1x_client *client)
{
	return 0;
}

static int vi2_host1x_exit(struct host1x_client *client)
{
	return 0;
}

static const struct host1x_client_ops vi2_client_ops = {
	.init = vi2_host1x_init,
	.exit = vi2_host1x_exit,
};

static int vi2_ops_init(struct tegra_camera *cam)
{
	struct platform_device *pdev = cam->pdev;
	struct tegra_camera_clk *clks;
	int i;
	int err;

	/* Init clocks */
	cam->num_clks = ARRAY_SIZE(vi2_clks);
	cam->clks = vi2_clks;

	for (i = 0; i < cam->num_clks; i++) {
		clks = &cam->clks[i];

		clks->clk = devm_clk_get(&pdev->dev, clks->name);
		if (IS_ERR_OR_NULL(clks->clk)) {
			dev_err(&pdev->dev, "Failed to get clock %s.\n",
				clks->name);
			return PTR_ERR(clks->clk);
		}
	}

	/* Init host1x client */
	INIT_LIST_HEAD(&cam->client.list);
	cam->client.ops = &vi2_client_ops;
	cam->client.dev = &cam->pdev->dev;
	err = host1x_client_register(&cam->client);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register host1x client: %d\n",
			err);
		goto exit_free_clks;
	}

	/* Init syncpts */
	cam->syncpt_id_csi_a = host1x_syncpt_request(&cam->pdev->dev,
		HOST1X_SYNCPT_HAS_BASE | HOST1X_SYNCPT_CLIENT_MANAGED);

	cam->syncpt_id_csi_b = host1x_syncpt_request(&cam->pdev->dev,
		HOST1X_SYNCPT_HAS_BASE | HOST1X_SYNCPT_CLIENT_MANAGED);

	/* Init Regulator */
	cam->reg = devm_regulator_get(&pdev->dev, "power");
	if (IS_ERR_OR_NULL(cam->reg)) {
		dev_err(&pdev->dev, "couldn't get power regulator, err %ld\n",
			PTR_ERR(cam->reg));
		cam->reg = NULL;
		err = PTR_ERR(cam->reg);
		goto exit_free_clks;
	}

	cam->clk = devm_clk_get(&pdev->dev, "vi");
	if (IS_ERR(cam->clk)) {
		dev_err(&pdev->dev, "failed to get vi clock\n");
		err = PTR_ERR(cam->clk);
		goto exit_free_clks;
	}

	err = clk_set_rate(cam->clk, 408000000);
	if (err) {
		dev_err(&pdev->dev, "failed to set vi clock rate to 408MHz\n");
		goto exit_free_clks;
	}

	cam->rst = devm_reset_control_get(&pdev->dev, "vi");
	if (IS_ERR(cam->rst)) {
		dev_err(&pdev->dev, "failed to get reset\n");
		err = PTR_ERR(cam->rst);
		goto exit_free_clks;
	}

	err = tegra_powergate_sequence_power_up(TEGRA_POWERGATE_VENC, cam->clk,
						cam->rst);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to power partition: %d\n", err);
		goto exit_free_clks;
	}

	return 0;

exit_free_clks:
	for (i = 0; i < cam->num_clks; i++) {
		clks = &cam->clks[i];
		if (clks->clk)
		clk_put(clks->clk);
	}

	return err;
}

static void vi2_ops_deinit(struct tegra_camera *cam)
{
	struct tegra_camera_clk *clks;
	int i;

	/* Free syncpts */
	host1x_syncpt_free(cam->syncpt_id_csi_a);
	host1x_syncpt_free(cam->syncpt_id_csi_b);

	/* Deinit host1x client */
	host1x_client_unregister(&cam->client);

	/* Free clocks */
	for (i = 0; i < cam->num_clks; i++) {
		clks = &cam->clks[i];
		if (clks->clk)
		clk_put(clks->clk);
	}
}

static int vi2_clock_start(struct tegra_camera *cam)
{
	struct tegra_camera_clk *clks;
	int i;

	for (i = 0; i < cam->num_clks; i++) {
		clks = &cam->clks[i];
		if (clks->clk) {
			clk_prepare_enable(clks->clk);
			if (clks->freq > 0)
				clk_set_rate(clks->clk, clks->freq);
		}
	}

	return 0;
}

static void vi2_clock_stop(struct tegra_camera *cam)
{
	struct tegra_camera_clk *clks;
	int i;

	for (i = cam->num_clks - 1; i >= 0; i--) {
		clks = &cam->clks[i];
		if (clks->clk)
			clk_disable_unprepare(clks->clk);
	}
}

static int vi2_activate(struct tegra_camera *cam)
{
	int ret;

	/* Enable external power */
	if (cam->reg) {
		ret = regulator_enable(cam->reg);
		if (ret)
			dev_err(&cam->pdev->dev, "enabling regulator failed\n");
	}

	/* T12_CG_2ND_LEVEL_EN */
	TC_VI_REG_WT(cam, TEGRA_VI_CFG_CG_CTRL, 1);
	udelay(10);

	/* Clean up status */
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_A_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_B_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_C_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_D_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_E_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILA_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILB_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILC_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILD_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_PIXEL_PARSER_A_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_PIXEL_PARSER_B_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_ERROR_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_ERROR_STATUS, 0xFFFFFFFF);

	cam->sof = 1;

	return 0;
}

static void vi2_deactivate(struct tegra_camera *cam)
{
	/* Disable external power */
	if (cam->reg)
		regulator_disable(cam->reg);

	cam->sof = 0;
}

static bool vi2_port_is_valid(struct tegra_camera_port *cam_port)
{
	/* Make sure port is in the valid range. */
	if ((cam_port->port < TEGRA_CAMERA_PORT_VIP) ||
	    (cam_port->port >= TEGRA_CAMERA_PORT_NUM))
		return 0;

	/* For CSI, make sure the number of lanes is valid (1, 2, or 4). */
	if ((cam_port->port == TEGRA_CAMERA_PORT_CSI_A) ||
	    (cam_port->port == TEGRA_CAMERA_PORT_CSI_B)) {
		if ((cam_port->num_lanes != 1) &&
		    (cam_port->num_lanes != 2) &&
		    (cam_port->num_lanes != 4))
			return 0;
	}

	/*
	 * Lanes 0 and 1 form CILA, and can be used for 1 or 2 lane support.
	 * Lanes 2 and 3 form CILB, and can be used for 1 or 2 lane support.
	 * Lanes 4 and 5 form CILC, and can be used for 1 or 2 lane support.
	 * Lanes 6 and 7 form CILD, and can be used for 1 or 2 lane support..
	 * Lane 8 forms CILE, and .can be used for 1 lane support.
	 *
	 * CILA and CILB can team together for 4-lane support.  Likewise,
	 * CILC and CILD can team together for 4-lane support.
	 *
	 * Each CIL has a clock.  CILA's clock is 0, CILB's clock is 1, etc.
	 * all the way to CILE, which is clock 4.
	 *
	 * When teaming CILA and CILB, either clock 0 or 1 can be used.
	 * Likewise, when teaming CILC and CILD, either clock 2 or 3 can be
	 * used.  CILE must use clock 4.
	 */
	if (cam_port->cila_used && cam_port->cilb_used) {
		if ((cam_port->clk_lane != 0) && (cam_port->clk_lane != 1))
			return false;
	} else if (cam_port->cilc_used && cam_port->cild_used) {
		if ((cam_port->clk_lane != 2) && (cam_port->clk_lane != 3))
			return false;
	} else if (cam_port->cila_used) {
		if (cam_port->clk_lane != 0)
			return false;
	} else if (cam_port->cilb_used) {
		if (cam_port->clk_lane != 1)
			return false;
	} else if (cam_port->cilc_used) {
		if (cam_port->clk_lane != 2)
			return false;
	} else if (cam_port->cild_used) {
		if (cam_port->clk_lane != 3)
			return false;
	} else if (cam_port->cile_used) {
		if (cam_port->clk_lane != 4)
			return false;
	}

	return true;
}

static void vi2_capture_clean(struct tegra_camera *cam)
{
	/* Clean up status */
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_A_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_B_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_C_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_D_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_E_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILA_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILB_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILC_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILD_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_PIXEL_PARSER_A_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_PIXEL_PARSER_B_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_ERROR_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_ERROR_STATUS, 0xFFFFFFFF);
}

static void vi2_setup_phys(struct tegra_camera *cam,
			   struct tegra_camera_port *cam_port)
{
	if (cam_port->cila_used)
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILA_CONTROL0, 0x9);
	if (cam_port->cilb_used)
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILB_CONTROL0, 0x9);
	if (cam_port->cilc_used)
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILC_CONTROL0, 0x9);
	if (cam_port->cild_used)
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILD_CONTROL0, 0x9);
	if (cam_port->cile_used)
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILE_CONTROL0, 0x9);

	TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND,
		     (cam_port->cila_used ? 0x00000001 : 0x00000002) |
		     (cam_port->cilb_used ? 0x00000100 : 0x00000200) |
		     (cam_port->cilc_used ? 0x00010000 : 0x00020000) |
		     (cam_port->cild_used ? 0x01000000 : 0x02000000) |
		     (cam_port->cile_used ? 0x10000000 : 0x20000000));

	TC_VI_REG_WT(cam, TEGRA_CSI_CILA_PAD_CONFIG0, 0x00000000);
	TC_VI_REG_WT(cam, TEGRA_CSI_CILB_PAD_CONFIG0, 0x00000000);
	TC_VI_REG_WT(cam, TEGRA_CSI_CILC_PAD_CONFIG0, 0x00000000);
	TC_VI_REG_WT(cam, TEGRA_CSI_CILD_PAD_CONFIG0, 0x00000000);
	TC_VI_REG_WT(cam, TEGRA_CSI_CILE_PAD_CONFIG0, 0x00000000);
	if (cam_port->cila_used && cam_port->cilb_used) {
		if (cam_port->clk_lane == 0)
			TC_VI_REG_WT(cam, TEGRA_CSI_CILA_PAD_CONFIG0, 0x10000);
		else
			TC_VI_REG_WT(cam, TEGRA_CSI_CILA_PAD_CONFIG0, 0x20000);
	} else if (cam_port->cilc_used && cam_port->cild_used) {
		if (cam_port->clk_lane == 2)
			TC_VI_REG_WT(cam, TEGRA_CSI_CILC_PAD_CONFIG0, 0x10000);
		else
			TC_VI_REG_WT(cam, TEGRA_CSI_CILC_PAD_CONFIG0, 0x20000);
	}


}

static int vi2_capture_setup_csi_0(struct tegra_camera *cam,
				   struct tegra_camera_port *cam_port,
				   struct soc_camera_device *icd)
{
	int format;
	int data_type;
	int word_count;

#ifdef DEBUG
	TC_VI_REG_WT(cam, TEGRA_CSI_DEBUG_CONTROL,
		     0x3 | (0x1 << 5) | (0x40 << 8));
#endif

	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND, 0xf007);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_PIXEL_PARSER_A_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_A_CONTROL0, 0x2A0301f0);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND, 0xf007);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_A_CONTROL1, 0x11);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_A_GAP, 0x140000);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_A_EXPECTED_FRAME, 0x0);

	TC_VI_REG_WT(cam, TEGRA_CSI_INPUT_STREAM_A_CONTROL,
			0x3f0000 | (cam_port->num_lanes - 1));

	if ((icd->current_fmt->code == V4L2_MBUS_FMT_UYVY8_2X8) ||
	    (icd->current_fmt->code == V4L2_MBUS_FMT_VYUY8_2X8) ||
	    (icd->current_fmt->code == V4L2_MBUS_FMT_YUYV8_2X8) ||
	    (icd->current_fmt->code == V4L2_MBUS_FMT_YVYU8_2X8)) {
		/* TBD */
	} else if ((icd->current_fmt->code == V4L2_MBUS_FMT_SBGGR8_1X8) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SGBRG8_1X8) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SGRBG8_1X8) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SRGGB8_1X8)) {
		format = TEGRA_IMAGE_FORMAT_T_L8;
		data_type = TEGRA_IMAGE_DT_RAW8;
		word_count = icd->user_width;
	} else if ((icd->current_fmt->code == V4L2_MBUS_FMT_SBGGR10_1X10) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SGBRG10_1X10) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SGRBG10_1X10) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SRGGB10_1X10)) {
		format = TEGRA_IMAGE_FORMAT_T_R16_I;
		data_type = TEGRA_IMAGE_DT_RAW10;
		word_count = icd->user_width * 10 / 8;
	}

	/*
	 * word_count is the number of bytes per line, and it has to be aligned
	 * to a 16-byte boundary.  Pad it out if this is not the case.
	 */
	if (word_count % 16)
		word_count = word_count + 16 - (word_count % 16);

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_IMAGE_DEF,
		     (1 << 24) | (format << 16) | 0x1);

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_CSI_IMAGE_DT, data_type);

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_CSI_IMAGE_SIZE_WC, word_count);

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_CSI_IMAGE_SIZE,
			(icd->user_height << 16) | icd->user_width);

	return 0;
}

static int vi2_capture_setup_csi_1(struct tegra_camera *cam,
				   struct tegra_camera_port *cam_port,
				   struct soc_camera_device *icd)
{
	int format;
	int data_type;
	int word_count;

#ifdef DEBUG
	TC_VI_REG_WT(cam, TEGRA_CSI_DEBUG_CONTROL,
		     0x5 | (0x1 << 5) | (0x50 << 8));
#endif

	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND, 0xf007);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_PIXEL_PARSER_B_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_B_CONTROL0, 0x2A0301f1);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND, 0xf007);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_B_CONTROL1, 0x11);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_B_GAP, 0x140000);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_B_EXPECTED_FRAME, 0x0);

	TC_VI_REG_WT(cam, TEGRA_CSI_INPUT_STREAM_B_CONTROL,
			0x3f0000 | (cam_port->num_lanes - 1));

	if ((icd->current_fmt->code == V4L2_MBUS_FMT_UYVY8_2X8) ||
	    (icd->current_fmt->code == V4L2_MBUS_FMT_VYUY8_2X8) ||
	    (icd->current_fmt->code == V4L2_MBUS_FMT_YUYV8_2X8) ||
	    (icd->current_fmt->code == V4L2_MBUS_FMT_YVYU8_2X8)) {
		/* TBD */
	} else if ((icd->current_fmt->code == V4L2_MBUS_FMT_SBGGR8_1X8) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SGBRG8_1X8) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SGRBG8_1X8) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SRGGB8_1X8)) {
		format = TEGRA_IMAGE_FORMAT_T_L8;
		data_type = TEGRA_IMAGE_DT_RAW8;
		word_count = icd->user_width;
	} else if ((icd->current_fmt->code == V4L2_MBUS_FMT_SBGGR10_1X10) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SGBRG10_1X10) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SGRBG10_1X10) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SRGGB10_1X10)) {
		format = TEGRA_IMAGE_FORMAT_T_R16_I;
		data_type = TEGRA_IMAGE_DT_RAW10;
		word_count = icd->user_width * 10 / 8;
	}

	/*
	 * word_count is the number of bytes per line, and it has to be aligned
	 * to a 16-byte boundary.  Pad it out if this is not the case.
	 */
	if (word_count % 16)
		word_count = word_count + 16 - (word_count % 16);

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_IMAGE_DEF,
		     (1 << 24) | (format << 16) | 0x1);

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_CSI_IMAGE_DT, data_type);

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_CSI_IMAGE_SIZE_WC, word_count);

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_CSI_IMAGE_SIZE,
		     (icd->user_height << 16) | icd->user_width);

	return 0;
}

#define MIPI_CAL_BASE		0x700e3000
#define MIPI_BIAS_PAD_CFG0	0x58
#define E_VCLAMP_REF		0
#define MIPI_BIAS_PAD_CFG1	0x5c
#define MIPI_BIAS_PAD_CFG2	0x60
#define PDVREG			1

static int vi2_mipi_cal(struct tegra_camera *cam)
{
	void __iomem *mipi_cal;
	u32 val;

	/* Map registers */
	mipi_cal = ioremap(MIPI_CAL_BASE, 0x100);
	if (!mipi_cal)
		return -ENOMEM;

	val = readl(mipi_cal + MIPI_BIAS_PAD_CFG0);
	val |= 1 << E_VCLAMP_REF;
	writel(val, mipi_cal + MIPI_BIAS_PAD_CFG0);

	val = readl(mipi_cal + MIPI_BIAS_PAD_CFG2);
	val &= ~(1 << PDVREG);
	writel(val, mipi_cal + MIPI_BIAS_PAD_CFG2);

	return 0;
}

static int vi2_capture_setup(struct tegra_camera *cam,
			     struct tegra_camera_port *cam_port,
			     struct tegra_camera_buffer *buf)
{
	struct soc_camera_device *icd = buf->icd;

	/* Skip VI2/CSI2 setup for second and later frame capture */
	if (!cam->sof)
		return 0;

	vi2_mipi_cal(cam);

	/* Common programming set for any config */
	TC_VI_REG_WT(cam, TEGRA_CSI_CLKEN_OVERRIDE, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_A_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_B_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_C_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_D_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_E_INTERRUPT_MASK, 0x0);

	vi2_setup_phys(cam, cam_port);

	/* Setup registers for CSI-A and CSI-B inputs */
	if (cam_port->port == TEGRA_CAMERA_PORT_CSI_A)
		return vi2_capture_setup_csi_0(cam, cam_port, icd);
	else if (cam_port->port == TEGRA_CAMERA_PORT_CSI_B)
		return vi2_capture_setup_csi_1(cam, cam_port, icd);
	else
		return -ENODEV;
}

static int vi2_capture_buffer_setup(struct tegra_camera *cam,
				    struct tegra_camera_buffer *buf)
{
	struct soc_camera_device *icd = buf->icd;
	struct tegra_camera_port *cam_port = icd->host_priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
			icd->current_fmt->host_fmt);

	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		/* FIXME: Setup YUV buffer */

	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_RGB32:
		if (cam_port->port == TEGRA_CAMERA_PORT_CSI_A) {
			switch (buf->output_channel) {
			case 0:
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_0_SURFACE0_OFFSET_MSB,
					     0x0);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_0_SURFACE0_OFFSET_LSB,
					     buf->buffer_addr);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_0_SURFACE0_STRIDE,
					     bytes_per_line);
				break;
			case 1:
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_0_SURFACE1_OFFSET_MSB,
					     0x0);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_0_SURFACE1_OFFSET_LSB,
					     buf->buffer_addr);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_0_SURFACE1_STRIDE,
					     bytes_per_line);
				break;
			case 2:
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_0_SURFACE2_OFFSET_MSB,
					     0x0);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_0_SURFACE2_OFFSET_LSB,
					     buf->buffer_addr);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_0_SURFACE2_STRIDE,
					     bytes_per_line);
				break;
			}
		} else if (cam_port->port == TEGRA_CAMERA_PORT_CSI_B) {
			switch (buf->output_channel) {
			case 0:
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_1_SURFACE0_OFFSET_MSB,
					     0x0);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_1_SURFACE0_OFFSET_LSB,
					     buf->buffer_addr);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_1_SURFACE0_STRIDE,
					     bytes_per_line);
				break;
			case 1:
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_1_SURFACE1_OFFSET_MSB,
					     0x0);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_1_SURFACE1_OFFSET_LSB,
					     buf->buffer_addr);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_1_SURFACE1_STRIDE,
					     bytes_per_line);
				break;
			case 2:
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_1_SURFACE2_OFFSET_MSB,
					     0x0);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_1_SURFACE2_OFFSET_LSB,
					     buf->buffer_addr);
				TC_VI_REG_WT(cam,
					     TEGRA_VI_CSI_1_SURFACE2_STRIDE,
					     bytes_per_line);
				break;
			}
		}
		break;

	default:
		dev_err(&cam->pdev->dev, "Wrong host format %c%c%c%c\n",
			((u8 *)&icd->current_fmt->host_fmt->fourcc)[0],
			((u8 *)&icd->current_fmt->host_fmt->fourcc)[1],
			((u8 *)&icd->current_fmt->host_fmt->fourcc)[2],
			((u8 *)&icd->current_fmt->host_fmt->fourcc)[3]);
		return -EINVAL;
	}

	return 0;
}

static void vi2_capture_error_status(struct tegra_camera *cam)
{
	u32 val;

#ifdef DEBUG
	val = TC_VI_REG_RD(cam, TEGRA_CSI_DEBUG_COUNTER_0);
	pr_err("TEGRA_CSI_DEBUG_COUNTER_0 0x%08x\n", val);
#endif
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CIL_A_STATUS);
	pr_err("TEGRA_CSI_CSI_CIL_A_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CILA_STATUS);
	pr_err("TEGRA_CSI_CSI_CILA_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CIL_B_STATUS);
	pr_err("TEGRA_CSI_CSI_CIL_B_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CILB_STATUS);
	pr_err("TEGRA_CSI_CSI_CILB_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CIL_C_STATUS);
	pr_err("TEGRA_CSI_CSI_CIL_C_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CILC_STATUS);
	pr_err("TEGRA_CSI_CSI_CILC_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CIL_D_STATUS);
	pr_err("TEGRA_CSI_CSI_CIL_D_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CILD_STATUS);
	pr_err("TEGRA_CSI_CSI_CILD_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CIL_E_STATUS);
	pr_err("TEGRA_CSI_CSI_CIL_E_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_PIXEL_PARSER_A_STATUS);
	pr_err("TEGRA_CSI_CSI_PIXEL_PARSER_A_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_PIXEL_PARSER_B_STATUS);
	pr_err("TEGRA_CSI_CSI_PIXEL_PARSER_B_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_VI_CSI_0_ERROR_STATUS);
	pr_err("TEGRA_VI_CSI_0_ERROR_STATUS 0x%08x\n", val);
	val = TC_VI_REG_RD(cam, TEGRA_VI_CSI_1_ERROR_STATUS);
	pr_err("TEGRA_VI_CSI_1_ERROR_STATUS 0x%08x\n", val);

	vi2_capture_clean(cam);
}

static int vi2_capture_start(struct tegra_camera *cam,
			     struct tegra_camera_port *cam_port,
			     struct tegra_camera_buffer *buf)
{
	int err, value;

	err = vi2_capture_buffer_setup(cam, buf);
	if (err < 0)
		return err;

	/* Only wait on CSI frame end syncpt if we're using CSI. */
	if (cam_port->port == TEGRA_CAMERA_PORT_CSI_A) {
		cam->syncpt_csi_a =
			host1x_syncpt_incr_max(cam->syncpt_id_csi_a, 1);
		TC_VI_REG_WT(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT,
			(6 << 8) | host1x_syncpt_id(cam->syncpt_id_csi_a));
		TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND,
			0x0000f005);
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SINGLE_SHOT, 0x1);
		err = host1x_syncpt_wait(cam->syncpt_id_csi_a,
			cam->syncpt_csi_a,
			TEGRA_SYNCPT_CSI_WAIT_TIMEOUT,
			&value);
	} else if (cam_port->port == TEGRA_CAMERA_PORT_CSI_B) {
		cam->syncpt_csi_b =
			host1x_syncpt_incr_max(cam->syncpt_id_csi_b, 1);
		TC_VI_REG_WT(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT,
			(7 << 8) | host1x_syncpt_id(cam->syncpt_id_csi_b));
		TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND,
			0x0000f005);
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SINGLE_SHOT, 0x1);
		err = host1x_syncpt_wait(cam->syncpt_id_csi_b,
			cam->syncpt_csi_b,
			TEGRA_SYNCPT_CSI_WAIT_TIMEOUT,
			&value);
	}

	/* Mark SOF flag to Zero after we captured the FIRST frame */
	if (cam->sof)
		cam->sof = 0;

	/* Capture syncpt timeout err, then dump error status */
	if (err) {
		if (cam_port->port == TEGRA_CAMERA_PORT_CSI_A)
			dev_err(&cam->pdev->dev,
				"CSI_A syncpt timeout, syncpt = %d, err = %d\n",
				host1x_syncpt_id(cam->syncpt_id_csi_a), err);
		else if (cam_port->port == TEGRA_CAMERA_PORT_CSI_B)
			dev_err(&cam->pdev->dev,
				"CSI_B syncpt timeout, syncpt = %d, err = %d\n",
				host1x_syncpt_id(cam->syncpt_id_csi_b), err);
		vi2_capture_error_status(cam);
	}

	return err;
}

static int vi2_capture_stop(struct tegra_camera *cam,
			    struct tegra_camera_port *cam_port)
{
	if (cam_port->port == TEGRA_CAMERA_PORT_CSI_A)
		TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND,
			     0x0000f002);
	else if (cam_port->port == TEGRA_CAMERA_PORT_CSI_B)
		TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND,
			     0x0000f002);

	return 0;
}

static int vi2_capture_frame(struct tegra_camera *cam,
			     struct tegra_camera_buffer *buf)
{
	struct vb2_buffer *vb = &buf->vb;
	struct soc_camera_device *icd = buf->icd;
	struct tegra_camera_port *cam_port = icd->host_priv;
	int retry = TEGRA_SYNCPT_RETRY_COUNT;
	int err;

	/* Setup capture registers */
	vi2_capture_setup(cam, cam_port, buf);

	while (retry) {
		err = vi2_capture_start(cam, cam_port, buf);
		/* Capturing succeed, stop capturing */
		vi2_capture_stop(cam, cam_port);
		if (err) {
			retry--;
			continue;
		}
		break;
	}

	/* Reset hardware for too many errors */
	if (!retry) {
		vi2_deactivate(cam);
		mdelay(5);
		vi2_activate(cam);
	}

	spin_lock_irq(&cam->videobuf_queue_lock);

	do_gettimeofday(&vb->v4l2_buf.timestamp);
	vb->v4l2_buf.field = cam->field;
	if (cam_port->port == TEGRA_CAMERA_PORT_CSI_A)
		vb->v4l2_buf.sequence = cam->sequence_a++;
	else if (cam_port->port == TEGRA_CAMERA_PORT_CSI_B)
		vb->v4l2_buf.sequence = cam->sequence_b++;

	vb2_buffer_done(vb, err < 0 ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);

	cam->num_frames++;

	spin_unlock_irq(&cam->videobuf_queue_lock);

	return err;
}

static struct tegra_camera_ops vi2_ops = {
	.init		= vi2_ops_init,
	.deinit		= vi2_ops_deinit,
	.clock_start	= vi2_clock_start,
	.clock_stop	= vi2_clock_stop,
	.activate	= vi2_activate,
	.deactivate	= vi2_deactivate,
	.port_is_valid	= vi2_port_is_valid,
	.capture_frame	= vi2_capture_frame,
};

static int __init vi2_init(void)
{
	tegra_camera_register_ops(&vi2_ops);
	return platform_driver_register(&tegra_camera_driver);
}

static void __exit vi2_exit(void)
{
	tegra_camera_register_ops(NULL);
        platform_driver_unregister(&tegra_camera_driver);
}

module_init(vi2_init);
module_exit(vi2_exit);

MODULE_DESCRIPTION("TEGRA SoC Camera Host VI2 driver");
MODULE_AUTHOR("Bryan Wu <pengw@nvidia.com>");
MODULE_LICENSE("GPL v2");
