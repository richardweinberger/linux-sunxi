/*
 * Allwinner DEBE (Display Engine BackEnd) header
 *
 * Copyright (C) 2014 Free Electrons
 *
 * Author: Boris Brezillon <boris.brezillon@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __SUNXI_TCON_H__
#define __SUNXI_TCON_H__

#include <linux/kernel.h>

#define SUNXI_TCON_GCTL_REG			0x0
#define SUNXI_TCON_GINT0_REG			0x4
#define SUNXI_TCON_GINT1_REG			0x8
#define SUNXI_TCON_FRM_CTL_REG			0x10
#define SUNXI_TCON0_CTL_REG			0x40
#define SUNXI_TCON0_DCLK_REG			0x44
#define SUNXI_TCON0_BASIC0_REG			0x48
#define SUNXI_TCON0_BASIC1_REG			0x4c
#define SUNXI_TCON0_BASIC2_REG			0x50
#define SUNXI_TCON0_BASIC3_REG			0x54
#define SUNXI_TCON0_HV_IF_REG			0x58
#define SUNXI_TCON0_CPU_IF_REG			0x60
#define SUNXI_TCON0_CPU_WR_REG			0x64
#define SUNXI_TCON0_CPU_RD0_REG			0x68
#define SUNXI_TCON0_CPU_RDA_REG			0x6c
#define SUNXI_TCON0_TTL0_REG			0x70
#define SUNXI_TCON0_TTL1_REG			0x74
#define SUNXI_TCON0_TTL2_REG			0x78
#define SUNXI_TCON0_TTL3_REG			0x7c
#define SUNXI_TCON0_TTL4_REG			0x80
#define SUNXI_TCON0_LVDS_IF_REG			0x84
#define SUNXI_TCON0_IO_POL_REG			0x88
#define SUNXI_TCON0_IO_TRI_REG			0x8c
#define SUNXI_TCON1_CTL_REG			0x90
#define SUNXI_TCON1_BASIC0_REG			0x94
#define SUNXI_TCON1_BASIC1_REG			0x98
#define SUNXI_TCON1_BASIC2_REG			0x9c
#define SUNXI_TCON1_BASIC3_REG			0xa0
#define SUNXI_TCON1_BASIC4_REG			0xa4
#define SUNXI_TCON1_BASIC5_REG			0xa8
#define SUNXI_TCON1_IO_POL_REG			0xf0
#define SUNXI_TCON1_IO_TRI_REG			0xf4
#define SUNXI_TCON_CEU_CTL_REG			0x100
#define SUNXI_TCON_CEU_MUL_RR_REG		0x110
#define SUNXI_TCON_CEU_MUL_RG_REG		0x114
#define SUNXI_TCON_CEU_MUL_RB_REG		0x118
#define SUNXI_TCON_CEU_ADD_RC_REG		0x11c
#define SUNXI_TCON_CEU_MUL_GR_REG		0x120
#define SUNXI_TCON_CEU_MUL_GG_REG		0x124
#define SUNXI_TCON_CEU_MUL_GB_REG		0x128
#define SUNXI_TCON_CEU_ADD_GC_REG		0x12c
#define SUNXI_TCON_CEU_MUL_BR_REG		0x130
#define SUNXI_TCON_CEU_MUL_BG_REG		0x134
#define SUNXI_TCON_CEU_MUL_BB_REG		0x138
#define SUNXI_TCON_CEU_ADD_BC_REG		0x13c
#define SUNXI_TCON_CEU_RANGE_R_REG		0x140
#define SUNXI_TCON_CEU_RANGE_G_REG		0x144
#define SUNXI_TCON_CEU_RANGE_B_REG		0x148
#define SUNXI_TCON1_FILL_CTL_REG		0x300
#define SUNXI_TCON1_FILL_BEG0_REG		0x304
#define SUNXI_TCON1_FILL_END0_REG		0x308
#define SUNXI_TCON1_FILL_DATA0_REG		0x30c
#define SUNXI_TCON1_FILL_BEG1_REG		0x310
#define SUNXI_TCON1_FILL_END1_REG		0x314
#define SUNXI_TCON1_FILL_DATA1_REG		0x318
#define SUNXI_TCON1_FILL_BEG2_REG		0x31c
#define SUNXI_TCON1_FILL_END2_REG		0x320
#define SUNXI_TCON1_FILL_DATA2_REG		0x324
#define SUNXI_TCON1_GAMMA_TABLE_REG		0x400

struct sunxi_tcon {
	void __iomem *regs;
};

#endif /* __SUNXI_TCON_H__ */
