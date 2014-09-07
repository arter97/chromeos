/*
 * This header provides constants for binding nvidia,tegra*-gpio.
 *
 * The first cell in Tegra's GPIO specifier is the GPIO ID. The macros below
 * provide names for this.
 *
 * The second cell contains standard flag values specified in gpio.h.
 */

#ifndef _DT_BINDINGS_MEDIA_TEGRA_CAMERA_H
#define _DT_BINDINGS_MEDIA_TEGRA_CAMERA_H

#define TEGRA_CAMERA_PORT_VIP	0
#define TEGRA_CAMERA_PORT_CSI_A	1
#define TEGRA_CAMERA_PORT_CSI_B	2
#define TEGRA_CAMERA_PORT_NUM	3

#define TEGRA_CAMERA_EP_SENSOR	0
#define TEGRA_CAMERA_EP_VCM	1
#define TEGRA_CAMERA_EP_FLASH	2

#endif
