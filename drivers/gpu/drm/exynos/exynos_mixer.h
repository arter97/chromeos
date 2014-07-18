/*
 * Copyright (C) 2013 Google, Inc.
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

#ifndef _EXYNOS_MIXER_H_
#define _EXYNOS_MIXER_H_

/* This function returns the horizontal offset for a mode */
unsigned mixer_get_horizontal_offset(unsigned width, unsigned height);

int mixer_mode_valid(struct drm_display_mode *mode);

#endif
