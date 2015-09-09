/*
 * Copyright (c) 2013-2015, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/clk.h>
#include <linux/host1x.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/iommu.h>
#include <linux/regulator/consumer.h>
#include <soc/tegra/pmc.h>

#include "drm.h"
#include "vi.h"

#define VI_NUM_SYNCPTS 6

struct vi_config {
	u32 class_id;
	int powergate_id;
};

struct vi {
	struct tegra_drm_client client;

	void __iomem *regs;

	struct host1x_channel *channel;
	struct device *dev;
	struct clk *clk;
	struct reset_control *rst;
	struct regulator *reg;

	struct iommu_domain *domain;

	/* Platform configuration */
	struct vi_config *config;
};

static inline struct vi *to_vi(struct tegra_drm_client *client)
{
	return container_of(client, struct vi, client);
}

static inline void vi_writel(struct vi *vi, u32 v, u32 r)
{
	writel(v, vi->regs + r);
}

static int vi_power_off(struct device *dev)
{
	struct vi *vi = dev_get_drvdata(dev);
	int err;

	err = reset_control_assert(vi->rst);
	if (err)
		return err;

	err = regulator_disable(vi->reg);
	if (err) {
		reset_control_deassert(vi->rst);
		dev_err(dev, "disable csi regulator failed");
		return err;
	}

	clk_disable_unprepare(vi->clk);

	return tegra_power_partition_power_off(vi->config->powergate_id);
}

static int vi_power_on(struct device *dev)
{
	struct vi *vi = dev_get_drvdata(dev);
	int err;

	err = tegra_powergate_sequence_power_up(vi->config->powergate_id,
						vi->clk, vi->rst);
	if (err)
		return err;

	err = regulator_enable(vi->reg);
	if (err) {
		tegra_power_partition_power_off(TEGRA_POWERGATE_VENC);
		dev_err(dev, "enable csi regulator failed.\n");
		return err;
	}

	vi_writel(vi, T12_CG_2ND_LEVEL_EN, T12_VI_CFG_CG_CTRL);

	return 0;
}

static void vi_reset(struct device *dev)
{
	struct vi *vi = dev_get_drvdata(dev);

	vi_power_off(dev);
	vi_power_on(dev);

	/* Reset sensor A */
	vi_writel(vi, 0x00000000, T12_VI_CSI_0_CSI_IMAGE_DT);

	vi_writel(vi, T12_CSI_CSICIL_SW_SENSOR_A_RESET_SENSOR_A_RESET,
		  T12_CSI_CSICIL_SW_SENSOR_A_RESET);

	vi_writel(vi, T12_CSI_CSI_SW_SENSOR_A_RESET_SENSOR_A_RESET,
		  T12_CSI_CSI_SW_SENSOR_A_RESET);

	vi_writel(vi, T12_VI_CSI_0_SW_RESET_SHADOW_RESET |
		      T12_VI_CSI_0_SW_RESET_SENSORCTL_RESET |
		      T12_VI_CSI_0_SW_RESET_PF_RESET |
		      T12_VI_CSI_0_SW_RESET_MCINTF_RESET |
		      T12_VI_CSI_0_SW_RESET_ISPINTF_RESET,
		  T12_VI_CSI_0_SW_RESET);

	/* Reset sensor B */
	vi_writel(vi, 0x00000000, T12_VI_CSI_1_CSI_IMAGE_DT);
	vi_writel(vi, T12_CSI_CSICIL_SW_SENSOR_B_RESET_SENSOR_B_RESET,
		  T12_CSI_CSICIL_SW_SENSOR_B_RESET);

	vi_writel(vi, T12_CSI_CSI_SW_SENSOR_B_RESET_SENSOR_B_RESET,
		  T12_CSI_CSI_SW_SENSOR_B_RESET);

	vi_writel(vi, T12_VI_CSI_1_SW_RESET_SHADOW_RESET |
		      T12_VI_CSI_1_SW_RESET_SENSORCTL_RESET |
		      T12_VI_CSI_1_SW_RESET_PF_RESET |
		      T12_VI_CSI_1_SW_RESET_MCINTF_RESET |
		      T12_VI_CSI_1_SW_RESET_ISPINTF_RESET,
		  T12_VI_CSI_1_SW_RESET);

	udelay(10);

	vi_writel(vi, 0x00000000, T12_CSI_CSI_SW_SENSOR_A_RESET);
	vi_writel(vi, 0x00000000, T12_CSI_CSICIL_SW_SENSOR_A_RESET);
	vi_writel(vi, 0x00000000, T12_CSI_CSI_SW_SENSOR_B_RESET);
	vi_writel(vi, 0x00000000, T12_CSI_CSICIL_SW_SENSOR_B_RESET);
	vi_writel(vi, 0x00000000, T12_VI_CSI_0_SW_RESET);
	vi_writel(vi, 0x00000000, T12_VI_CSI_1_SW_RESET);
}

static int vi_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct tegra_drm *tegra = dev->dev_private;
	unsigned long flags = HOST1X_SYNCPT_HAS_BASE;
	struct vi *vi = to_vi(drm);
	unsigned int i;
	int err;

	if (tegra->domain) {
		err = iommu_attach_device(tegra->domain, client->dev);
		if (err < 0) {
			dev_err(client->dev, "failed to attach to domain: %d\n",
				err);
			return err;
		}

		vi->domain = tegra->domain;
	}

	vi->channel = host1x_channel_request(client->dev);
	if (!vi->channel) {
		err = -ENOMEM;
		goto error_iommu_detach_device;
	}

	for (i = 0; i < VI_NUM_SYNCPTS; i++) {
		client->syncpts[i] = host1x_syncpt_request(client->dev, flags);
		if (!client->syncpts[i]) {
			err = -ENOMEM;
			goto error_host1x_syncpt_free;
		}
	}

	err = tegra_drm_register_client(dev->dev_private, drm);
	if (err)
		goto error_host1x_syncpt_free;

	return 0;

error_host1x_syncpt_free:
	for (i = 0; i < VI_NUM_SYNCPTS; i++)
		if (client->syncpts[i]) {
			host1x_syncpt_free(client->syncpts[i]);
			client->syncpts[i] = NULL;
		}
	host1x_channel_free(vi->channel);
error_iommu_detach_device:
	if (vi->domain) {
		iommu_detach_device(vi->domain, vi->dev);
		vi->domain = NULL;
	}
	return err;
}

static int vi_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct vi *vi = to_vi(drm);
	unsigned int i;
	int err;

	err = tegra_drm_unregister_client(dev->dev_private, drm);

	for (i = 0; i < VI_NUM_SYNCPTS; i++)
		if (client->syncpts[i]) {
			host1x_syncpt_free(client->syncpts[i]);
			client->syncpts[i] = NULL;
		}

	host1x_channel_free(vi->channel);

	if (vi->domain) {
		iommu_detach_device(vi->domain, vi->dev);
		vi->domain = NULL;
	}

	return 0;
}

static const struct host1x_client_ops vi_client_ops = {
	.init = vi_init,
	.exit = vi_exit,
};

static int vi_open_channel(struct tegra_drm_client *client,
			   struct tegra_drm_context *context)
{
	struct vi *vi = to_vi(client);

	pm_runtime_get_sync(vi->dev);

	context->channel = host1x_channel_get(vi->channel);
	if (!context->channel)
		return -ENOMEM;

	return 0;
}

static void vi_close_channel(struct tegra_drm_context *context)
{
	struct vi *vi = to_vi(context->client);

	if (!context->channel)
		return;

	host1x_channel_put(context->channel);
	context->channel = NULL;

	pm_runtime_mark_last_busy(vi->dev);
	pm_runtime_put_autosuspend(vi->dev);
}

static int vi_is_addr_reg(struct device *dev, u32 class, u32 offset)
{
	return 0;
}

static const struct tegra_drm_client_ops vi_ops = {
	.open_channel = vi_open_channel,
	.close_channel = vi_close_channel,
	.is_addr_reg = vi_is_addr_reg,
	.submit = tegra_drm_submit,
	.reset = vi_reset,
};

static const struct vi_config vi_t210_config = {
	.class_id = HOST1X_CLASS_VI,
	.powergate_id = TEGRA_POWERGATE_VENC,
};

static const struct of_device_id vi_match[] = {
	{ .compatible = "nvidia,tegra210-vi", .data = &vi_t210_config },
	{ },
};

static int vi_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct vi_config *vi_config;
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct resource *regs;
	struct vi *vi;
	int err;

	if (!dev->of_node) {
		dev_err(&pdev->dev, "no dt node\n");
		return -EINVAL;
	}

	match = of_match_node(vi_match, pdev->dev.of_node);
	if (match)
		vi_config = (struct vi_config *)match->data;
	else
		return -ENXIO;

	vi = devm_kzalloc(dev, sizeof(*vi), GFP_KERNEL);
	if (!vi)
		return -ENOMEM;

	syncpts = devm_kzalloc(dev, VI_NUM_SYNCPTS * sizeof(*syncpts),
			       GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vi->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(vi->regs))
		return PTR_ERR(vi->regs);

	vi->rst = devm_reset_control_get(&pdev->dev, "vi");
	if (IS_ERR(vi->rst)) {
		dev_err(&pdev->dev, "cannot get reset\n");
		return PTR_ERR(vi->rst);
	}

	vi->clk = devm_clk_get(dev, "vi");
	if (IS_ERR(vi->clk))
		return PTR_ERR(vi->clk);

	vi->reg = devm_regulator_get(dev, "avdd-dsi-csi");
	if (IS_ERR(vi->reg)) {
		err = PTR_ERR(vi->reg);
		return err;
	}

	INIT_LIST_HEAD(&vi->client.base.list);
	vi->client.base.ops = &vi_client_ops;
	vi->client.base.dev = dev;
	vi->client.base.class = vi_config->class_id;
	vi->client.base.syncpts = syncpts;
	vi->client.base.num_syncpts = VI_NUM_SYNCPTS;
	vi->dev = dev;
	vi->config = vi_config;

	INIT_LIST_HEAD(&vi->client.list);
	vi->client.ops = &vi_ops;

	platform_set_drvdata(pdev, vi);

	err = vi_power_on(&pdev->dev);
	if (err) {
		dev_err(dev, "cannot power on device\n");
		return err;
	}

	err = host1x_client_register(&vi->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto error_vi_power_off;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 5000);

	dev_info(&pdev->dev, "initialized");

	return 0;

error_vi_power_off:
	vi_power_off(&pdev->dev);
	return err;
}

static int vi_remove(struct platform_device *pdev)
{
	struct vi *vi = platform_get_drvdata(pdev);
	int err;

	err = host1x_client_unregister(&vi->client.base);
	if (err < 0)
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);

	vi_power_off(&pdev->dev);

	return err;
}

static int __maybe_unused vi_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return vi_power_off(dev);
}

static int __maybe_unused vi_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return vi_power_on(dev);
}

static const struct dev_pm_ops vi_pm_ops = {
	SET_RUNTIME_PM_OPS(vi_runtime_suspend, vi_runtime_resume, NULL)
};

struct platform_driver tegra_vi_driver = {
	.driver = {
		.name = "tegra-vi",
		.of_match_table = vi_match,
		.pm = &vi_pm_ops,
	},
	.probe = vi_probe,
	.remove = vi_remove,
};
