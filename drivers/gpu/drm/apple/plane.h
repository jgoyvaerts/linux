// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright (C) The Asahi Linux Contributors
 */

#ifndef __APPLE_PLANE_H__
#define __APPLE_PLANE_H__

#include <drm/drm_plane.h>

#include <linux/types.h>

#include "iomfb_plane.h"

/* Largest ARGB cursor that fits the cursor plane's padding buffers. */
#define APPLE_CURSOR_SIZE 128

struct apple_plane {
	struct drm_plane base;
	u32 iomfb_surf;
	/* Cursor plane: padding buffers for edge-clipped cursors, used in turn. */
	struct drm_gem_dma_object *pad[2];
	unsigned int pad_next;
};

#define to_apple_plane(x) container_of(x, struct apple_plane, base)

struct apple_plane_state {
	struct drm_plane_state base;
	struct dcp_surface surf;
	struct dcp_rect src_rect;
	struct dcp_rect dst_rect;
	u64 iova;
};

#define to_apple_plane_state(x) container_of(x, struct apple_plane_state, base)

struct drm_gem_dma_object *apple_drm_gem_create_scanout(struct drm_device *drm,
							 size_t size);

struct drm_plane *apple_plane_init(struct drm_device *dev,
				   unsigned long possible_crtcs,
				   u32 iomfb_surf,
				   bool supports_l10r,
				   enum drm_plane_type type);

#endif /* __APPLE_PLANE_H__ */
