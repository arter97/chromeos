/*
 * Copyright (C) 2015 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Note about the original authors:
 *
 * This driver is based on the original driver for AP3223 that was distributed
 * by Dyna Image. That driver was using input framework. The driver from Dyna
 * image is not available online anywhere, so there is no reference for it here.
 * However, that driver is also GPLv2.
 * The following is part of the header found in that file
 * (The GPL notice from the original header is removed)
 *
 * >> This file is part of the AP3223, AP3212C and AP3216C sensor driver.
 * >> AP3426 is combined proximity and ambient light sensor.
 * >> AP3216C is combined proximity, ambient light sensor and IRLED.
 * >>
 * >> Contact: John Huang <john.huang@dyna-image.com>
 * >>	       Templeton Tsai <templeton.tsai@dyna-image.com>
 *
 * Another author initials mentioned in that file was just YC (and no name).
 *
 * Not sure for what kernel version the driver from dyna image was written for.
 * Vic Lee <Vic_Lee@asus.com> made modifications to it to run on 3.14.
 *
 * Datasheet:
 * http://www.dyna-image.com/english/product/optical-sensor-detail.php?cpid=2&dpid=8#doc
 */

#ifndef __AP3223_H__
#define __AP3223_H__

/* ap3223 control registers */

#define AP3223_REG_SYS_CTRL		0x00
#define AP3223_REG_SYS_CTRL_SHIFT	(0)
#define AP3223_REG_SYS_CTRL_MASK	0x07

/* System Mode (AP3223_REG_SYS_CTRL) */

#define	AP3223_SYS_DEV_DOWN		0x00
#define	AP3223_SYS_ALS_ENABLE		0x01
#define	AP3223_SYS_PS_ENABLE		0x02
#define	AP3223_SYS_ALS_PS_ENABLE	0x03
#define	AP3223_SYS_DEV_RESET		0x04

#define AP3223_REG_SYS_INTSTATUS	0x01
#define AP3223_REG_SYS_INT_SHIFT	(0)
#define AP3223_REG_SYS_INT_ALS_SHIFT	(0)
#define AP3223_REG_SYS_INT_PS_SHIFT	(1)
#define AP3223_REG_SYS_INT_OBJ_SHIFT	(4)
#define AP3223_REG_SYS_INT_IR_OV_SHIFT	(5)

/* INT FLAG BIT MASK */

#define AP3223_REG_SYS_INT_MASK		0x03
#define AP3223_REG_SYS_INT_AMASK	0x01
#define AP3223_REG_SYS_INT_PMASK	0x02
#define AP3223_REG_SYS_INT_OBJ_MASK	0x10
#define AP3223_REG_SYS_INT_IR_OV_MASK	0x20

#define AP3223_REG_SYS_INTCTRL		0x02
#define AP3223_SYS_INT_AIEN_MASK	0x08
#define AP3223_SYS_INT_PIEN_MASK	0x80
#define AP3223_SYS_INT_CLR_MNR_MASK	0x01

#define AP3223_SYS_INT_AIEN_SHIFT	(3)
#define AP3223_SYS_INT_PIEN_SHIFT	(7)

/* ALS Interrupt */

#define AP3223_SYS_INT_AINT_EN		(1 << AP3223_SYS_INT_AIEN_SHIFT)

/* PS Interrupt */

#define AP3223_SYS_INT_PINT_EN		(1 << AP3223_SYS_INT_PIEN_SHIFT)

/* INT Clear Manner */

#define	AP3223_SYS_INT_CLEAR_AUTO	0x00
#define	AP3223_SYS_INT_CLEAR_MANUAL	0x01

#define AP3223_REG_SYS_WAITTIME		0x06

#define AP3223_WAITTIME_SLOT(n)		(n)

/* ap3223 data registers */

#define AP3223_REG_IR_DATA_LOW		0x0A
#define AP3223_REG_IR_DATA_LOW_SHIFT	(0)
#define AP3223_REG_IR_DATA_LOW_MASK	0xFF

#define AP3223_REG_IR_DATA_HIGH		0x0B
#define AP3223_REG_IR_DATA_HIGH_SHIFT	(0)
#define AP3223_REG_IR_DATA_HIGH_MASK	0x03

#define AP3223_REG_ALS_DATA_LOW		0x0C

#define AP3223_REG_ALS_DATA_HIGH	0x0D

#define AP3223_REG_PS_DATA_LOW		0x0E
#define AP3223_REG_PS_DATA_LOW_SHIFT	(0)
#define	AL3223_REG_PS_DATA_LOW_MASK	0xFF

#define AP3223_REG_PS_DATA_HIGH		0x0F
#define AP3223_REG_PS_DATA_HIGH_SHIFT	(0)
#define	AL3223_REG_PS_DATA_HIGH_MASK	0x03

#define AP3223_REG_ALS_GAIN		0x10

/* ALS Gain */

#define AP3223_ALS_RANGE_0		0x00	/* Full range 65535 lux */
#define AP3223_ALS_RANGE_1		0x01	/* Full range 16383 lux */
#define AP3223_ALS_RANGE_2		0x02	/* Full range 4095 lux */
#define AP3223_ALS_RANGE_3		0x03	/* Full range 1023 lux */
#define AP3223_ALS_RANGE_MASK		0x30
#define AP3223_ALS_RANGE_SHIFT		(4)
#define AP3223_ALS_PERSIST_MASK		0x0F

#define AP3223_REG_ALS_PERSIST		0x14
#define AP3223_REG_ALS_PERSIST_SHIFT	(0)
#define AP3223_REG_ALS_PERSIST_MASK	0x3F

#define AP3223_REG_ALS_THDL_L		0x1A
#define AP3223_REG_ALS_THDL_L_SHIFT	(0)
#define AP3223_REG_ALS_THDL_L_MASK	0xFF

#define AP3223_REG_ALS_THDL_H		0x1B
#define AP3223_REG_ALS_THDL_H_SHIFT	(0)
#define AP3223_REG_ALS_THDL_H_MASK	0xFF

#define AP3223_REG_ALS_THDH_L		0x1C
#define AP3223_REG_ALS_THDH_L_SHIFT	(0)
#define AP3223_REG_ALS_THDH_L_MASK	0xFF

#define AP3223_REG_ALS_THDH_H		0x1D
#define AP3223_REG_ALS_THDH_H_SHIFT	(0)
#define AP3223_REG_ALS_THDH_H_MASK	0xFF

/* ap3223 PS Gain registers */

#define AP3223_REG_PS_GAIN		0x20
#define AP3223_REG_PS_GAIN_SHIFT	(2)
#define AP3223_REG_PS_GAIN_MASK		0x0C

/* PS Gain */

#define AP3223_PS_GAIN_1		0x00	/* PS resolution * 1 */
#define AP3223_PS_GAIN_2		0x01	/* PS resolution * 2 */
#define AP3223_PS_GAIN_4		0x02	/* PS resolution * 4 */
#define AP3223_PS_GAIN_8		0x03	/* PS resolution * 8 */

#define AP3223_REG_PS_LEDD		0x21	/* PS LED DRIVER */
#define AP3223_REG_PS_LEDD_SHIFT	(0)
#define AP3223_REG_PS_LEDD_MASK		0x03

/* PS LED Driver current percentage */

#define AP3223_PS_LED_DRVR_CUR_P_16_7	0x00	/* 16.7% */
#define AP3223_PS_LED_DRVR_CUR_P_33_3	0x01	/* 33.3% */
#define AP3223_PS_LED_DRVR_CUR_P_66_7	0x02	/* 66.7% */
#define AP3223_PS_LED_DRVR_CUR_P_100	0x03	/* 100% (default) */

#define AP3223_REG_PS_IFORM		0x22	/* PS INT Mode */

#define AP3223_PS_INT_ALGO_ZONE_MODE	0x00
#define AP3223_PS_INT_ALSO_HYST_MODE	0x01

#define AP3223_REG_PS_MEAN		0x23
#define AP3223_REG_PS_MEAN_SHIFT	(0)
#define AP3223_REG_PS_MEAN_MASK		0x03

/* PS MEAN */

#define AP3223_PS_MEAN_0		0x00	/* 5ms @2T */
#define AP3223_PS_MEAN_1		0x01	/* 9.6ms @2T */
#define AP3223_PS_MEAN_2		0x02	/* 14.1ms @2T */
#define AP3223_PS_MEAN_3		0x03	/* 18.7ms @2T */

#define AP3223_REG_PS_SMARTINT		0x24	/* PS Smart INT for low power */
#define AP3223_PS_SMARTINT_DISABLE	0x00
#define AP3223_PS_SMARTINT_ENABLE	0x01

#define AP3223_REG_PS_INTEGR_TIME	0x25

#define AP3223_PS_INTEGR_TIME_SEL(t)	(t)

#define AP3223_REG_PS_PERSIST		0x26
#define AP3223_REG_PS_PERSIST_SHIFT	(0)
#define AP3223_REG_PS_PERSIST_MASK	0x3F

#define AP3223_PS_PERSIST_CONV_TIME(t)	(t)

#define AP3223_REG_PS_CAL_L		0x28
#define AP3223_REG_PS_CAL_L_SHIFT	(0)
#define AP3223_REG_PS_CAL_L_MASK	0xFF

#define AP3223_REG_PS_CAL_H		0x29
#define AP3223_REG_PS_CAL_H_SHIFT	(0)
#define AP3223_REG_PS_CAL_H_MASK	0x01

#define AP3223_REG_PS_THDL_L		0x2A
#define AP3223_REG_PS_THDL_L_SHIFT	(0)
#define AP3223_REG_PS_THDL_L_MASK	0xFF

#define AP3223_REG_PS_THDL_H		0x2B
#define AP3223_REG_PS_THDL_H_SHIFT	(0)
#define AP3223_REG_PS_THDL_H_MASK	0x03

#define AP3223_REG_PS_THDH_L		0x2C
#define AP3223_REG_PS_THDH_L_SHIFT	(0)
#define AP3223_REG_PS_THDH_L_MASK	0xFF

#define AP3223_REG_PS_THDH_H		0x2D
#define AP3223_REG_PS_THDH_H_SHIFT	(0)
#define AP3223_REG_PS_THDH_H_MASK	0x03

/* PS Engineering Registers */

#define AP3223_REG_PS_DC_1		0x30	/* Only in Engineering chip,
						couldn't find in datasheet */
#define AP3223_REG_PS_DC_1_SHIFT	(0)
#define AP3223_REG_PS_DC_1_MASK		0xFF

#define AP3223_REG_PS_DC_2		0x32	/* Only in Engineering chip,
						couldn't find in datasheet */
#define AP3223_REG_PS_DC_2_SHIFT	(0)
#define AP3223_REG_PS_DC_2_MASK		0xFF

#endif /* __AP3223_H__ */
