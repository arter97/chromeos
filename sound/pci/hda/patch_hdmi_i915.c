/*
 *  hda_hdmi_i915.c - routines for Haswell HDA controller power well support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the Free
 *  Software Foundation; either version 2 of the License, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 *  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <sound/core.h>
#include <drm/i915_sync_audio.h>
#include "patch_hdmi_i915.h"

static int (*sync_audio_rate)(int port, int rate);

/* There is a fixed mapping between audio pin node and display port
 * on current Intel platforms:
 * Pin Widget 5 - PORT B (port = 1 in i915 driver)
 * Pin Widget 6 - PORT C (port = 2 in i915 driver)
 * Pin Widget 7 - PORT D (port = 3 in i915 driver)
 */
static int intel_pin2port(hda_nid_t pin_nid)
{
	return pin_nid - 4;
}

int patch_hdmi_i915_sync_audio_rate(hda_nid_t pin_nid, int rate)
{
	if (!sync_audio_rate)
		return -ENODEV;

	snd_printdd("HDA sync audio rate %d\n", rate);

	return sync_audio_rate(intel_pin2port(pin_nid), rate);
}

int patch_hdmi_i915_init(void)
{
	int err = 0;

	sync_audio_rate = symbol_request(i915_sync_audio_rate);
	if (!sync_audio_rate)
		pr_warn("hda-i915: sync_audio_rate symbol request failed\n");
	else
		pr_debug("hda-i915: sync_audio_rate symbol request success\n");

	return err;
}

int patch_hdmi_i915_exit(void)
{
	if (sync_audio_rate) {
		symbol_put(i915_sync_audio_rate);
		sync_audio_rate = NULL;
	}

	return 0;
}
