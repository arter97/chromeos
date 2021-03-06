/*
 * Copyright (C) 2010,2011 Google, Inc.
 * Copyright (C) 2011-2012 NVIDIA CORPORATION. All Rights Reserved.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Erik Gilling <konkers@google.com>
 *	Doug Anderson <dianders@chromium.org>
 *	Stephen Warren <swarren@nvidia.com>
 *
 * Portions based on mach-omap2's debug-macro.S
 * Copyright (C) 1994-1999 Russell King
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
 */

#include <linux/serial_reg.h>

#define UART_SHIFT 2

/* Physical addresses */
#define TEGRA_CLK_RESET_BASE		0x60006000
#define TEGRA_APB_MISC_BASE		0x70000000
#define TEGRA_UARTA_BASE		0x70006000
#define TEGRA_UARTB_BASE		0x70006040
#define TEGRA_UARTC_BASE		0x70006200
#define TEGRA_UARTD_BASE		0x70006300
#define TEGRA_UARTE_BASE		0x70006400
#define TEGRA_PMC_BASE			0x7000e400

#define TEGRA_CLK_RST_DEVICES_L		(TEGRA_CLK_RESET_BASE + 0x04)
#define TEGRA_CLK_RST_DEVICES_H		(TEGRA_CLK_RESET_BASE + 0x08)
#define TEGRA_CLK_RST_DEVICES_U		(TEGRA_CLK_RESET_BASE + 0x0c)
#define TEGRA_CLK_OUT_ENB_L		(TEGRA_CLK_RESET_BASE + 0x10)
#define TEGRA_CLK_OUT_ENB_H		(TEGRA_CLK_RESET_BASE + 0x14)
#define TEGRA_CLK_OUT_ENB_U		(TEGRA_CLK_RESET_BASE + 0x18)
#define TEGRA_PMC_SCRATCH20		(TEGRA_PMC_BASE + 0xa0)
#define TEGRA_APB_MISC_GP_HIDREV	(TEGRA_APB_MISC_BASE + 0x804)

/*
 * Must be section-aligned since a section mapping is used early on.
 * Must not overlap with regions in mach-tegra/io.c:tegra_io_desc[].
 */
#define UART_VIRTUAL_BASE		0xfe800000

#define checkuart(rp, rv, lhu, bit, uart) \
		/* Load address of CLK_RST register */ \
		movw	rp, #TEGRA_CLK_RST_DEVICES_##lhu & 0xffff ; \
		movt	rp, #TEGRA_CLK_RST_DEVICES_##lhu >> 16 ; \
		/* Load value from CLK_RST register */ \
		ldr	rp, [rp, #0] ; \
		/* Test UART's reset bit */ \
		tst	rp, #(1 << bit) ; \
		/* If set, can't use UART; jump to save no UART */ \
		bne	90f ; \
		/* Load address of CLK_OUT_ENB register */ \
		movw	rp, #TEGRA_CLK_OUT_ENB_##lhu & 0xffff ; \
		movt	rp, #TEGRA_CLK_OUT_ENB_##lhu >> 16 ; \
		/* Load value from CLK_OUT_ENB register */ \
		ldr	rp, [rp, #0] ; \
		/* Test UART's clock enable bit */ \
		tst	rp, #(1 << bit) ; \
		/* If clear, can't use UART; jump to save no UART */ \
		beq	90f ; \
		/* Passed all tests, load address of UART registers */ \
		movw	rp, #TEGRA_UART##uart##_BASE & 0xffff ; \
		movt	rp, #TEGRA_UART##uart##_BASE >> 16 ; \
		/* Jump to save UART address */ \
		b 91f

		.macro  addruart, rp, rv, tmp
		adr	\rp, 99f		@ actual addr of 99f
		ldr	\rv, [\rp]		@ linked addr is stored there
		sub	\rv, \rv, \rp		@ offset between the two
		ldr	\rp, [\rp, #4]		@ linked tegra_uart_config
		sub	\tmp, \rp, \rv		@ actual tegra_uart_config
		ldr	\rp, [\tmp]		@ Load tegra_uart_config
		cmp	\rp, #1			@ needs initialization?
		bne	100f			@ no; go load the addresses
		mov	\rv, #0			@ yes; record init is done
		str	\rv, [\tmp]

#ifdef CONFIG_TEGRA_DEBUG_UART_AUTO_ODMDATA
		/* Check ODMDATA */
10:		movw	\rp, #TEGRA_PMC_SCRATCH20 & 0xffff
		movt	\rp, #TEGRA_PMC_SCRATCH20 >> 16
		ldr	\rp, [\rp, #0]		@ Load PMC_SCRATCH20
		ubfx	\rv, \rp, #18, #2	@ 19:18 are console type
		cmp	\rv, #2			@ 2 and 3 mean DCC, UART
		beq	11f			@ some boards swap the meaning
		cmp	\rv, #3			@ so accept either
		bne	90f
11:		ubfx	\rv, \rp, #15, #3	@ 17:15 are UART ID
		cmp	\rv, #0			@ UART 0?
		beq	20f
		cmp	\rv, #1			@ UART 1?
		beq	21f
		cmp	\rv, #2			@ UART 2?
		beq	22f
		cmp	\rv, #3			@ UART 3?
		beq	23f
		cmp	\rv, #4			@ UART 4?
		beq	24f
		b	90f			@ invalid
#endif

#if defined(CONFIG_TEGRA_DEBUG_UARTA) || \
    defined(CONFIG_TEGRA_DEBUG_UART_AUTO_ODMDATA)
		/* Check UART A validity */
20:		checkuart(\rp, \rv, L, 6, A)
#endif

#if defined(CONFIG_TEGRA_DEBUG_UARTB) || \
    defined(CONFIG_TEGRA_DEBUG_UART_AUTO_ODMDATA)
		/* Check UART B validity */
21:		checkuart(\rp, \rv, L, 7, B)
#endif

#if defined(CONFIG_TEGRA_DEBUG_UARTC) || \
    defined(CONFIG_TEGRA_DEBUG_UART_AUTO_ODMDATA)
		/* Check UART C validity */
22:		checkuart(\rp, \rv, H, 23, C)
#endif

#if defined(CONFIG_TEGRA_DEBUG_UARTD) || \
    defined(CONFIG_TEGRA_DEBUG_UART_AUTO_ODMDATA)
		/* Check UART D validity */
23:		checkuart(\rp, \rv, U, 1, D)
#endif

#if defined(CONFIG_TEGRA_DEBUG_UARTE) || \
    defined(CONFIG_TEGRA_DEBUG_UART_AUTO_ODMDATA)
		/* Check UART E validity */
24:
		checkuart(\rp, \rv, U, 2, E)
#endif

		/* No valid UART found */
90:		mov	\rp, #0
		/* fall through */

		/* Record whichever UART we chose */
91:		str	\rp, [\tmp, #4]		@ Store in tegra_uart_phys
		cmp	\rp, #0			@ Valid UART address?
		bne	92f			@ Yes, go process it
		str	\rp, [\tmp, #8]		@ Store 0 in tegra_uart_virt
		b	100f			@ Done
92:		and	\rv, \rp, #0xffffff	@ offset within 1MB section
		add	\rv, \rv, #UART_VIRTUAL_BASE
		str	\rv, [\tmp, #8]		@ Store in tegra_uart_virt
		movw	\rv, #TEGRA_APB_MISC_GP_HIDREV & 0xffff
		movt	\rv, #TEGRA_APB_MISC_GP_HIDREV >> 16
		ldr	\rv, [\rv, #0]		@ Load HIDREV
		ubfx	\rv, \rv, #8, #8	@ 15:8 are SoC version
		cmp	\rv, #0x20		@ Tegra20?
		moveq	\rv, #0x75		@ Tegra20 divisor
		movne	\rv, #0xdd		@ Tegra30 divisor
		str	\rv, [\tmp, #12]	@ Save divisor to scratch
		/* uart[UART_LCR] = UART_LCR_WLEN8 | UART_LCR_DLAB; */
		mov	\rv, #UART_LCR_WLEN8 | UART_LCR_DLAB
		str	\rv, [\rp, #UART_LCR << UART_SHIFT]
		/* uart[UART_DLL] = div & 0xff; */
		ldr	\rv, [\tmp, #12]
		and	\rv, \rv, #0xff
		str	\rv, [\rp, #UART_DLL << UART_SHIFT]
		/* uart[UART_DLM] = div >> 8; */
		ldr	\rv, [\tmp, #12]
		lsr	\rv, \rv, #8
		str	\rv, [\rp, #UART_DLM << UART_SHIFT]
		/* uart[UART_LCR] = UART_LCR_WLEN8; */
		mov	\rv, #UART_LCR_WLEN8
		str	\rv, [\rp, #UART_LCR << UART_SHIFT]
		b	100f

		.align
99:		.word	.
		.word	tegra_uart_config
		.ltorg

		/* Load previously selected UART address */
100:		ldr	\rp, [\tmp, #4]		@ Load tegra_uart_phys
		ldr	\rv, [\tmp, #8]		@ Load tegra_uart_virt
		.endm

/*
 * Code below is swiped from <asm/hardware/debug-8250.S>, but add an extra
 * check to make sure that the UART address is actually valid.
 */

		.macro	senduart, rd, rx
		cmp	\rx, #0
		strneb	\rd, [\rx, #UART_TX << UART_SHIFT]
1001:
		.endm

		.macro	busyuart, rd, rx
		cmp	\rx, #0
		beq	1002f
1001:		ldrb	\rd, [\rx, #UART_LSR << UART_SHIFT]
		and	\rd, \rd, #UART_LSR_TEMT | UART_LSR_THRE
		teq	\rd, #UART_LSR_TEMT | UART_LSR_THRE
		bne	1001b
1002:
		.endm

		.macro	waituart, rd, rx
#ifdef FLOW_CONTROL
		cmp	\rx, #0
		beq	1002f
1001:		ldrb	\rd, [\rx, #UART_MSR << UART_SHIFT]
		tst	\rd, #UART_MSR_CTS
		beq	1001b
1002:
#endif
		.endm
