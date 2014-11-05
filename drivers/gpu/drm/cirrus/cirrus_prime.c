/*
 * Copyright © 2014 The Chromium OS Authors
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License version 2. See the file COPYING in the main
 * directory of this archive for more details.
 *
 */

#include <ttm/ttm_page_alloc.h>
#include <drm/drmP.h>
#include <linux/dma-buf.h>
#include "cirrus_drv.h"

#define CIRRUS_FD_PERMS 0600

static
struct sg_table *cirrus_gem_prime_get_sg_table(struct drm_gem_object *obj)
{
	struct cirrus_bo *cirrusbo = gem_to_cirrus_bo(obj);
	unsigned long npages = cirrusbo->bo.num_pages;

	return drm_prime_pages_to_sg(cirrusbo->bo.ttm->pages, npages);
}

static int cirrus_gem_prime_pin(struct drm_gem_object *obj)
{
	struct cirrus_bo *cirrusbo = gem_to_cirrus_bo(obj);
	int ret = 0;

	ret = cirrus_bo_reserve(cirrusbo, false);
	if (unlikely(ret != 0))
		goto out;

	ret = cirrus_bo_pin(cirrusbo, TTM_PL_FLAG_SYSTEM, NULL);
	if (ret)
		goto unreserve_out;

	ttm_pool_populate(cirrusbo->bo.ttm);

unreserve_out:
	cirrus_bo_unreserve(cirrusbo);
out:
	return ret;
}

static void cirrus_gem_prime_unpin(struct drm_gem_object *obj)
{
	struct cirrus_bo *cirrusbo = gem_to_cirrus_bo(obj);
	int ret = 0;

	ret = cirrus_bo_reserve(cirrusbo, false);
	if (unlikely(ret != 0))
		return;

	cirrus_bo_unpin(cirrusbo);
	cirrus_bo_unreserve(cirrusbo);
}

static
struct sg_table *cirrus_gem_map_dma_buf(struct dma_buf_attachment *attach,
					enum dma_data_direction dir)
{
	struct drm_gem_object *obj = attach->dmabuf->priv;
	struct sg_table *sg;
	int ret;

	ret = cirrus_gem_prime_pin(obj);

	if (ret)
		return ERR_PTR(ret);

	sg = cirrus_gem_prime_get_sg_table(obj);
	if (!sg) {
		cirrus_gem_prime_unpin(obj);
		return NULL;
	}

	return sg;
}

static void cirrus_gem_unmap_dma_buf(struct dma_buf_attachment *attach,
				     struct sg_table *sg,
				     enum dma_data_direction data_direction)
{
	sg_free_table(sg);
	kfree(sg);
	cirrus_gem_prime_unpin(attach->dmabuf->priv);
}


static void cirrus_dmabuf_release(struct dma_buf *dmabuf)
{
	struct drm_gem_object *obj = dmabuf->priv;

	if (obj->export_dma_buf == dmabuf) {
		obj->export_dma_buf = NULL;
		drm_gem_object_unreference_unlocked(obj);
	}
}

static void *cirrus_kmap_atomic_dma_buf(struct dma_buf *dma_buf,
					unsigned long page_num)
{
	return NULL;
}

static void *cirrus_kmap_dma_buf(struct dma_buf *dma_buf,
				 unsigned long page_num)
{
	return NULL;
}

static int cirrus_mmap_dma_buf(struct dma_buf *dma_buf,
			       struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct dma_buf_ops cirrus_dmabuf_ops = {
	.map_dma_buf	= cirrus_gem_map_dma_buf,
	.unmap_dma_buf	= cirrus_gem_unmap_dma_buf,
	.release	= cirrus_dmabuf_release,
	.kmap_atomic	= cirrus_kmap_atomic_dma_buf,
	.kmap		= cirrus_kmap_dma_buf,
	.mmap		= cirrus_mmap_dma_buf,
};


struct dma_buf *cirrus_gem_prime_export(struct drm_device *dev,
					struct drm_gem_object *obj,
					int flags)
{
	return dma_buf_export(obj, &cirrus_dmabuf_ops,
			      obj->size,
			      CIRRUS_FD_PERMS);
}


struct drm_gem_object *cirrus_gem_prime_import(struct drm_device *dev,
					       struct dma_buf *dma_buf)
{
	return ERR_PTR(-EINVAL);
}
