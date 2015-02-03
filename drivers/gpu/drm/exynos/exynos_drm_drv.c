/*
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic.h>

#include <drm/exynos_drm.h>

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
#include <linux/kds.h>
#endif
#ifdef CONFIG_DRM_DMA_SYNC
#include <drm/drm_sync_helper.h>
#endif

#include "exynos_drm_debugfs.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fbdev.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_gem.h"
#include "exynos_drm_vidi.h"
#include "exynos_drm_dmabuf.h"
#include "exynos_drm_g2d.h"
#include "exynos_drm_ipp.h"
#include "exynos_drm_iommu.h"
#include "exynos_drm_fimd.h"
#include "exynos_mixer.h"

#include <linux/i2c.h>
#include <linux/pm_runtime.h>

#define DRIVER_NAME	"exynos"
#define DRIVER_DESC	"Samsung SoC DRM"
#define DRIVER_DATE	"20110530"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

/* platform device pointer for eynos drm device. */
static struct platform_device *exynos_drm_pdev;

static void exynos_drm_setup_encoder_clones(struct drm_device *dev)
{
	struct drm_encoder *e;
	unsigned int clone_mask = 0;
	int cnt = 0;

	DRM_DEBUG_KMS("\n");

	list_for_each_entry(e, &dev->mode_config.encoder_list, head)
			clone_mask |= (1 << (cnt++));

	list_for_each_entry(e, &dev->mode_config.encoder_list, head)
		e->possible_clones = clone_mask;
}

static int exynos_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct exynos_drm_private *private;
	int ret;

	DRM_DEBUG_DRIVER("flags: 0x%lx\n", flags);

	private = kzalloc(sizeof(struct exynos_drm_private), GFP_KERNEL);
	if (!private) {
		DRM_ERROR("failed to allocate private\n");
		return -ENOMEM;
	}

	dev_set_drvdata(dev->dev, dev);
	dev->dev_private = (void *)private;

#ifdef CONFIG_DRM_DMA_SYNC
	private->cpu_fence_context = fence_context_alloc(1);
	atomic_set(&private->cpu_fence_seqno, 0);
#endif

	/*
	 * create mapping to manage iommu table and set a pointer to iommu
	 * mapping structure to iommu_mapping of private data.
	 * also this iommu_mapping can be used to check if iommu is supported
	 * or not.
	 */
	ret = drm_create_iommu_mapping(dev);
	if (ret < 0) {
		DRM_ERROR("failed to create iommu mapping.\n");
		goto err_freepriv;
	}

	drm_mode_config_init(dev);

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(dev);

	exynos_drm_mode_config_init(dev);
	/*
	 * probe sub drivers such as display controller and hdmi driver,
	 * that were registered at probe() of platform driver
	 * to the sub driver and create encoder and connector for them.
	 */
	ret = exynos_drm_device_register(dev);
	if (ret)
		goto err_mode_config_cleanup;

	if (private->dp_encoder && private->fimd_crtc)
		private->dp_encoder->possible_crtcs =
			1 << exynos_drm_pipe_from_crtc(private->fimd_crtc);

	if (private->hdmi_encoder && private->mixer_crtc)
		private->hdmi_encoder->possible_crtcs =
			1 << exynos_drm_pipe_from_crtc(private->mixer_crtc);

	if (private->vidi_encoder && private->vidi_crtc)
		private->vidi_encoder->possible_crtcs =
			1 << exynos_drm_pipe_from_crtc(private->vidi_crtc);

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = 1, we can use the vblank feature.
	 *
	 * Note that we don't use drm irq handler, since we have multiple
	 * interrupt sources and the drm framework supports just one.
	 */
	dev->irq_enabled = 1;

	ret = drm_vblank_init(dev, MAX_CRTC);
	if (ret)
		goto err_drm_device;

	/* setup possible_clones. */
	exynos_drm_setup_encoder_clones(dev);

	/*
	 * create and configure fb helper and also exynos specific
	 * fbdev object.
	 */
	ret = exynos_drm_fbdev_init(dev);
	if (ret) {
		DRM_ERROR("failed to initialize drm fbdev\n");
		goto err_vblank;
	}

	return 0;

err_vblank:
	drm_vblank_cleanup(dev);
err_drm_device:
	exynos_drm_device_unregister(dev);
err_mode_config_cleanup:
	drm_mode_config_cleanup(dev);
	drm_release_iommu_mapping(dev);
err_freepriv:
	kfree(private);

	return ret;
}

static int exynos_drm_unload(struct drm_device *dev)
{
	struct exynos_drm_private *private = dev->dev_private;

	DRM_DEBUG_DRIVER("\n");

	exynos_drm_fbdev_fini(dev);
	exynos_drm_device_unregister(dev);
	drm_vblank_cleanup(dev);
	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);

	drm_release_iommu_mapping(dev);
	kfree(private);

	dev->dev_private = NULL;

	return 0;
}

static int exynos_drm_suspend(struct drm_device *dev, pm_message_t state)
{
	struct drm_connector *connector;

	DRM_DEBUG_DRIVER("pm_event:%d\n", state.event);

	mutex_lock(&dev->mode_config.mutex);
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		int old_dpms = connector->dpms;

		if (connector->funcs->dpms)
			connector->funcs->dpms(connector, DRM_MODE_DPMS_OFF);

		/* Set the old mode back to the connector for resume */
		connector->dpms = old_dpms;
	}
	mutex_unlock(&dev->mode_config.mutex);

	return 0;
}

static int exynos_drm_resume(struct drm_device *dev)
{
	struct drm_connector *connector;
	enum drm_connector_status status;
	bool changed = false;

	DRM_DEBUG_DRIVER("\n");

	mutex_lock(&dev->mode_config.mutex);
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		int desired_mode = connector->dpms;

		connector->dpms = DRM_MODE_DPMS_OFF;

		/*
		 * If the connector has been disconnected during suspend,
		 * disconnect it from the encoder and leave it off. We'll notify
		 * userspace at the end.
		 */
		if (desired_mode == DRM_MODE_DPMS_ON) {
			status = connector->funcs->detect(connector, true);
			if (status == connector_status_disconnected) {
				connector->encoder = NULL;
				connector->status = status;
				changed = true;
				continue;
			}
		}

		if (connector->funcs->dpms)
			connector->funcs->dpms(connector, desired_mode);
	}

	mutex_unlock(&dev->mode_config.mutex);

	if (changed)
		drm_kms_helper_hotplug_event(dev);

	return 0;
}

static int exynos_drm_open(struct drm_device *dev, struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv;

	DRM_DEBUG_DRIVER("\n");

	file_priv = kzalloc(sizeof(*file_priv), GFP_KERNEL);
	if (!file_priv)
		return -ENOMEM;
	INIT_LIST_HEAD(&file_priv->gem_cpu_acquire_list);

	file->driver_priv = file_priv;

	return exynos_drm_subdrv_open(dev, file);
}

static void exynos_drm_preclose(struct drm_device *dev,
				struct drm_file *file)
{
	struct drm_exynos_file_private *file_private = file->driver_priv;
	struct exynos_drm_gem_obj_node *cur, *d;

	DRM_DEBUG_DRIVER("\n");

	mutex_lock(&dev->struct_mutex);
	/* release kds resource sets for outstanding GEM object acquires */
	list_for_each_entry_safe(cur, d,
			&file_private->gem_cpu_acquire_list, list) {
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
		BUG_ON(cur->exynos_gem_obj->resource_set == NULL);
		kds_resource_set_release(&cur->exynos_gem_obj->resource_set);
#endif
#ifdef CONFIG_DRM_DMA_SYNC
		BUG_ON(!cur->exynos_gem_obj->acquire_fence);
		drm_fence_signal_and_put(&cur->exynos_gem_obj->acquire_fence);
#endif
		drm_gem_object_unreference(&cur->exynos_gem_obj->base);
		kfree(cur);
	}
	mutex_unlock(&dev->struct_mutex);
	INIT_LIST_HEAD(&file_private->gem_cpu_acquire_list);

	exynos_drm_subdrv_close(dev, file);
}

static void exynos_drm_postclose(struct drm_device *dev, struct drm_file *file)
{
	DRM_DEBUG_DRIVER("\n");

	if (!file->driver_priv)
		return;

	kfree(file->driver_priv);
	file->driver_priv = NULL;
}

static void exynos_drm_lastclose(struct drm_device *dev)
{
	DRM_DEBUG_DRIVER("\n");

	exynos_drm_fbdev_restore_mode(dev);
}

int exynos_drm_pipe_from_crtc(struct drm_crtc *crtc)
{
	struct drm_crtc *c;
	int i = 0;

	list_for_each_entry(c, &crtc->dev->mode_config.crtc_list, head) {
		if (crtc == c)
			return i;
		i++;
	}
	DRM_ERROR("Could not find pipe from crtc %d\n", DRM_BASE_ID(crtc));
	return -EINVAL;
}

static struct drm_crtc *exynos_drm_crtc_from_pipe(struct drm_device *dev,
				int pipe)
{
	struct drm_crtc *crtc;
	int i = 0;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (i == pipe)
			return crtc;
		i++;
	}
	return NULL;
}

static u32 exynos_drm_get_vblank_counter(struct drm_device *dev, int pipe)
{
	struct drm_crtc *crtc;
	u32 cur_vblank;
	struct timeval last_timestamp;

	crtc = exynos_drm_crtc_from_pipe(dev, pipe);
	if (!crtc) {
		DRM_ERROR("Could not find crtc for pipe %d\n", pipe);
		return -ENODEV;
	}

	cur_vblank = drm_vblank_count_and_time(dev, pipe, &last_timestamp);
	if (crtc->enabled && !dev->vblank_enabled[pipe] && crtc->framedur_ns) {
		struct timeval cur_timestamp = drm_get_timestamp();
		u64 num = timeval_to_ns(&cur_timestamp) -
				timeval_to_ns(&last_timestamp);
		do_div(num, crtc->framedur_ns);
		cur_vblank += num;
	}

	return cur_vblank;
}

static int exynos_drm_enable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_crtc *crtc;

	crtc = exynos_drm_crtc_from_pipe(dev, pipe);
	if (!crtc) {
		DRM_ERROR("Could not find crtc for pipe %d\n", pipe);
		return -ENODEV;
	}

	if (pipe == fimd_get_crtc_id(dev))
		return fimd_enable_vblank(crtc);
	else if (pipe == mixer_get_crtc_id(dev))
		return mixer_enable_vblank(crtc);
	else if (pipe == vidi_get_crtc_id(dev))
		return vidi_enable_vblank(crtc);

	DRM_ERROR("Could not find enable_vblank for crtc %d\n", pipe);
	return -ENODEV;
}

static void exynos_drm_disable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_crtc *crtc;

	crtc = exynos_drm_crtc_from_pipe(dev, pipe);
	if (!crtc) {
		DRM_ERROR("Could not find crtc for pipe %d\n", pipe);
		return;
	}

	if (pipe == fimd_get_crtc_id(dev))
		return fimd_disable_vblank(crtc);
	else if (pipe == mixer_get_crtc_id(dev))
		return mixer_disable_vblank(crtc);
	else if (pipe == vidi_get_crtc_id(dev))
		return vidi_disable_vblank(crtc);

	DRM_ERROR("Could not find disable_vblank for crtc %d\n", pipe);
}

static const struct vm_operations_struct exynos_drm_gem_vm_ops = {
	.fault = exynos_drm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static struct drm_ioctl_desc exynos_ioctls[] = {
	DRM_IOCTL_DEF_DRV(EXYNOS_GEM_CREATE, exynos_drm_gem_create_ioctl,
			DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_GEM_MAP_OFFSET,
			exynos_drm_gem_map_offset_ioctl, DRM_UNLOCKED |
			DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_GEM_MMAP,
			exynos_drm_gem_mmap_ioctl, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_GEM_GET,
			exynos_drm_gem_get_ioctl, DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(EXYNOS_VIDI_CONNECTION,
			vidi_connection_ioctl, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_G2D_GET_VER,
			exynos_g2d_get_ver_ioctl, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_G2D_SET_CMDLIST,
			exynos_g2d_set_cmdlist_ioctl, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_G2D_EXEC,
			exynos_g2d_exec_ioctl, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_IPP_GET_PROPERTY,
			exynos_drm_ipp_get_property, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_IPP_SET_PROPERTY,
			exynos_drm_ipp_set_property, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_IPP_QUEUE_BUF,
			exynos_drm_ipp_queue_buf, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_IPP_CMD_CTRL,
			exynos_drm_ipp_cmd_ctrl, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_GEM_CPU_ACQUIRE,
			exynos_drm_gem_cpu_acquire_ioctl,
			DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(EXYNOS_GEM_CPU_RELEASE,
			exynos_drm_gem_cpu_release_ioctl,
			DRM_UNLOCKED | DRM_AUTH),
};

static const struct file_operations exynos_drm_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.mmap		= exynos_drm_gem_mmap,
	.poll		= drm_poll,
	.read		= drm_read,
	.unlocked_ioctl	= drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.release	= drm_release,
};

static struct drm_driver exynos_drm_driver = {
	.driver_features	= DRIVER_HAVE_IRQ | DRIVER_MODESET |
					DRIVER_GEM | DRIVER_PRIME,
	.load			= exynos_drm_load,
	.unload			= exynos_drm_unload,
	.suspend		= exynos_drm_suspend,
	.resume			= exynos_drm_resume,
	.open			= exynos_drm_open,
	.preclose		= exynos_drm_preclose,
	.lastclose		= exynos_drm_lastclose,
	.postclose		= exynos_drm_postclose,
	.get_vblank_counter	= exynos_drm_get_vblank_counter,
	.enable_vblank		= exynos_drm_enable_vblank,
	.disable_vblank		= exynos_drm_disable_vblank,
#if defined(CONFIG_DEBUG_FS)
	.debugfs_init		= exynos_drm_debugfs_init,
	.debugfs_cleanup	= exynos_drm_debugfs_cleanup,
#endif
	.gem_free_object	= exynos_drm_gem_free_object,
	.gem_vm_ops		= &exynos_drm_gem_vm_ops,
	.dumb_create		= exynos_drm_gem_dumb_create,
	.dumb_map_offset	= exynos_drm_gem_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.atomic_begin     = drm_atomic_begin,
	.atomic_set_event = drm_atomic_set_event,
	.atomic_check     = drm_atomic_check,
	.atomic_commit    = drm_atomic_commit,
	.atomic_end       = drm_atomic_end,
	.atomic_funcs     = &drm_atomic_funcs,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_export	= exynos_dmabuf_prime_export,
	.gem_prime_import	= exynos_dmabuf_prime_import,
	.ioctls			= exynos_ioctls,
	.fops			= &exynos_drm_driver_fops,
	.name	= DRIVER_NAME,
	.desc	= DRIVER_DESC,
	.date	= DRIVER_DATE,
	.major	= DRIVER_MAJOR,
	.minor	= DRIVER_MINOR,
};

static int exynos_drm_platform_probe(struct platform_device *pdev)
{
	DRM_DEBUG_DRIVER("[PDEV:%s]\n", pdev->name);

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	exynos_drm_driver.num_ioctls = DRM_ARRAY_SIZE(exynos_ioctls);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	return drm_platform_init(&exynos_drm_driver, pdev);
}

static int exynos_drm_platform_remove(struct platform_device *pdev)
{
	DRM_DEBUG_DRIVER("[PDEV:%s]\n", pdev->name);

	drm_platform_exit(&exynos_drm_driver, pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int exynos_drm_sys_suspend(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	pm_message_t message;

	if (pm_runtime_suspended(dev))
		return 0;

	message.event = PM_EVENT_SUSPEND;
	return exynos_drm_suspend(drm_dev, message);
}

static int exynos_drm_sys_resume(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	if (pm_runtime_suspended(dev))
		return 0;

	return exynos_drm_resume(drm_dev);
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int exynos_drm_runtime_suspend(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	pm_message_t message;

	if (pm_runtime_suspended(dev))
		return 0;

	message.event = PM_EVENT_SUSPEND;
	return exynos_drm_suspend(drm_dev, message);
}

static int exynos_drm_runtime_resume(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	if (!pm_runtime_suspended(dev))
		return 0;

	return exynos_drm_resume(drm_dev);
}
#endif

static const struct dev_pm_ops exynos_drm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(exynos_drm_sys_suspend, exynos_drm_sys_resume)
	SET_RUNTIME_PM_OPS(exynos_drm_runtime_suspend,
			exynos_drm_runtime_resume, NULL)
};

static struct platform_driver exynos_drm_platform_driver = {
	.probe		= exynos_drm_platform_probe,
	.remove		= exynos_drm_platform_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "exynos-drm",
		.pm	= &exynos_drm_pm_ops,
	},
};

static int __init exynos_drm_init(void)
{
	int ret;

	DRM_DEBUG_DRIVER("\n");

#ifdef CONFIG_DRM_EXYNOS_DP
	ret = platform_driver_register(&dp_driver);
	if (ret < 0)
		goto out_dp;
#endif

#ifdef CONFIG_DRM_EXYNOS_FIMD
	ret = platform_driver_register(&fimd_driver);
	if (ret < 0)
		goto out_fimd;
#endif

#ifdef CONFIG_DRM_EXYNOS_HDMI
	ret = platform_driver_register(&hdmi_driver);
	if (ret < 0)
		goto out_hdmi;
	ret = platform_driver_register(&mixer_driver);
	if (ret < 0)
		goto out_mixer;
#endif

#ifdef CONFIG_DRM_EXYNOS_VIDI
	ret = platform_driver_register(&vidi_driver);
	if (ret < 0)
		goto out_vidi;
#endif

#ifdef CONFIG_DRM_EXYNOS_G2D
	ret = platform_driver_register(&g2d_driver);
	if (ret < 0)
		goto out_g2d;
#endif

#ifdef CONFIG_DRM_EXYNOS_FIMC
	ret = platform_driver_register(&fimc_driver);
	if (ret < 0)
		goto out_fimc;
#endif

#ifdef CONFIG_DRM_EXYNOS_ROTATOR
	ret = platform_driver_register(&rotator_driver);
	if (ret < 0)
		goto out_rotator;
#endif

#ifdef CONFIG_DRM_EXYNOS_GSC
	ret = platform_driver_register(&gsc_driver);
	if (ret < 0)
		goto out_gsc;
#endif

#ifdef CONFIG_DRM_EXYNOS_IPP
	ret = platform_driver_register(&ipp_driver);
	if (ret < 0)
		goto out_ipp;
#endif

	ret = platform_driver_register(&exynos_drm_platform_driver);
	if (ret < 0)
		goto out_drm;

	exynos_drm_pdev = platform_device_register_simple("exynos-drm", -1,
				NULL, 0);
	if (IS_ERR_OR_NULL(exynos_drm_pdev)) {
		ret = PTR_ERR(exynos_drm_pdev);
		goto out;
	}

	return 0;

out:
	platform_driver_unregister(&exynos_drm_platform_driver);

out_drm:
#ifdef CONFIG_DRM_EXYNOS_IPP
	platform_driver_unregister(&ipp_driver);
out_ipp:
#endif

#ifdef CONFIG_DRM_EXYNOS_GSC
	platform_driver_unregister(&gsc_driver);
out_gsc:
#endif

#ifdef CONFIG_DRM_EXYNOS_ROTATOR
	platform_driver_unregister(&rotator_driver);
out_rotator:
#endif

#ifdef CONFIG_DRM_EXYNOS_FIMC
	platform_driver_unregister(&fimc_driver);
out_fimc:
#endif

#ifdef CONFIG_DRM_EXYNOS_G2D
	platform_driver_unregister(&g2d_driver);
out_g2d:
#endif

#ifdef CONFIG_DRM_EXYNOS_VIDI
	platform_driver_unregister(&vidi_driver);
out_vidi:
#endif

#ifdef CONFIG_DRM_EXYNOS_HDMI
	platform_driver_unregister(&mixer_driver);
out_mixer:
	platform_driver_unregister(&hdmi_driver);
out_hdmi:
#endif

#ifdef CONFIG_DRM_EXYNOS_FIMD
	platform_driver_unregister(&fimd_driver);
out_fimd:
#endif

#ifdef CONFIG_DRM_EXYNOS_DP
	platform_driver_unregister(&dp_driver);
out_dp:
#endif

	return ret;
}

static void __exit exynos_drm_exit(void)
{
	DRM_DEBUG_DRIVER("\n");

	platform_device_unregister(exynos_drm_pdev);

	platform_driver_unregister(&exynos_drm_platform_driver);

#ifdef CONFIG_DRM_EXYNOS_IPP
	platform_driver_unregister(&ipp_driver);
#endif

#ifdef CONFIG_DRM_EXYNOS_GSC
	platform_driver_unregister(&gsc_driver);
#endif

#ifdef CONFIG_DRM_EXYNOS_ROTATOR
	platform_driver_unregister(&rotator_driver);
#endif

#ifdef CONFIG_DRM_EXYNOS_FIMC
	platform_driver_unregister(&fimc_driver);
#endif

#ifdef CONFIG_DRM_EXYNOS_G2D
	platform_driver_unregister(&g2d_driver);
#endif

#ifdef CONFIG_DRM_EXYNOS_HDMI
	platform_driver_unregister(&mixer_driver);
	platform_driver_unregister(&hdmi_driver);
#endif

#ifdef CONFIG_DRM_EXYNOS_VIDI
	platform_driver_unregister(&vidi_driver);
#endif

#ifdef CONFIG_DRM_EXYNOS_FIMD
	platform_driver_unregister(&fimd_driver);
#endif
#ifdef CONFIG_DRM_EXYNOS_DP
	platform_driver_unregister(&dp_driver);
#endif
}

module_init(exynos_drm_init);
module_exit(exynos_drm_exit);

MODULE_AUTHOR("Inki Dae <inki.dae@samsung.com>");
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_AUTHOR("Seung-Woo Kim <sw0312.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC DRM Driver");
MODULE_LICENSE("GPL");
