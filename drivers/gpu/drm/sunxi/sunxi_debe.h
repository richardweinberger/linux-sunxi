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

#ifndef __SUNXI_DEBE_H__
#define __SUNXI_DEBE_H__

#include <linux/kernel.h>

#define SUNXI_DEBE_MODCTL_REG			0x800
#define SUNXI_DEBE_MODCTL_LINE_SEL		BIT(29)
#define SUNXI_DEBE_MODCTL_ITLMOD_EN		BIT(28)
#define SUNXI_DEBE_MODCTL_OUT_SEL		GENMASK(22, 20)
#define SUNXI_DEBE_MODCTL_OUT_LCD		(0 << 20)
#define SUNXI_DEBE_MODCTL_OUT_FE0		(6 << 20)
#define SUNXI_DEBE_MODCTL_OUT_FE1		(7 << 20)
#define SUNXI_DEBE_MODCTL_HWC_EN		BIT(16)
#define SUNXI_DEBE_MODCTL_LAY_EN(l)		BIT(8 + l)
#define SUNXI_DEBE_MODCTL_OCSC_EN		BIT(5)
#define SUNXI_DEBE_MODCTL_DFLK_EN		BIT(4)
#define SUNXI_DEBE_MODCTL_DLP_START_CTL		BIT(2)
#define SUNXI_DEBE_MODCTL_START_CTL		BIT(1)
#define SUNXI_DEBE_MODCTL_DEBE_EN		BIT(0)

#define SUNXI_DEBE_BACKCOLOR_REG		0x804
#define SUNXI_DEBE_BACKCOLOR(r, g, b)		(((r) << 16) | ((g) << 8) | (b))

#define SUNXI_DEBE_DISSIZE_REG			0x808
#define SUNXI_DEBE_DISSIZE(w, h)		(((((h) - 1) & 0xffff) << 16) | \
						 (((w) - 1) & 0xffff))

#define SUNXI_DEBE_LAYSIZE_REG(l)		(0x810 + (0x4 * (l)))
#define SUNXI_DEBE_LAYSIZE(w, h)		(((((h) - 1) & 0x1fff) << 16) | \
						 (((w) - 1) & 0x1fff))

#define SUNXI_DEBE_LAYCOOR_REG(l)		(0x820 + (0x4 * (l)))
#define SUNXI_DEBE_LAYCOOR(x, y)		((((u32)(y) & 0xffff) << 16) | \
						 ((u32)(x) & 0xffff))

#define SUNXI_DEBE_LAYLINEWIDTH_REG(l)		(0x840 + (0x4 * (l)))

#define SUNXI_DEBE_LAYFB_L32ADD_REG(l)		(0x850 + (0x4 * (l)))

#define SUNXI_DEBE_LAYFB_H4ADD_REG		0x860
#define SUNXI_DEBE_LAYFB_H4ADD_MSK(l)		GENMASK(3 + ((l) * 8), 0)
#define SUNXI_DEBE_LAYFB_H4ADD(l, val)		((val) << ((l) * 8))

#define SUNXI_DEBE_REGBUFFCTL_REG		0x870
#define SUNXI_DEBE_REGBUFFCTL_AUTOLOAD_DIS	BIT(1)
#define SUNXI_DEBE_REGBUFFCTL_LOADCTL		BIT(0)

#define SUNXI_DEBE_CKMAX_REG			0x880
#define SUNXI_DEBE_CKMIN_REG			0x884
#define SUNXI_DEBE_CKCFG_REG			0x888
#define SUNXI_DEBE_ATTCTL_REG0(l)		(0x890 + (0x4 * (l)))

#define SUNXI_DEBE_ATTCTL_REG1(l)		(0x8a0 + (0x4 * (l)))
#define SUNXI_DEBE_ATTCTL_REG1_LAY_HSCAFCT	GENMASK(15, 14)
#define SUNXI_DEBE_ATTCTL_REG1_LAY_WSCAFCT	GENMASK(13, 12)
#define SUNXI_DEBE_ATTCTL_REG1_LAY_FBFMT	GENMASK(11, 8)
#define SUNXI_DEBE_LAY_FBFMT_1BPP		(0 << 8)
#define SUNXI_DEBE_LAY_FBFMT_2BPP		(1 << 8)
#define SUNXI_DEBE_LAY_FBFMT_4BPP		(2 << 8)
#define SUNXI_DEBE_LAY_FBFMT_8BPP		(3 << 8)
#define SUNXI_DEBE_LAY_FBFMT_RGB655		(4 << 8)
#define SUNXI_DEBE_LAY_FBFMT_RGB565		(5 << 8)
#define SUNXI_DEBE_LAY_FBFMT_RGB556		(6 << 8)
#define SUNXI_DEBE_LAY_FBFMT_ARGB1555		(7 << 8)
#define SUNXI_DEBE_LAY_FBFMT_RGBA5551		(8 << 8)
#define SUNXI_DEBE_LAY_FBFMT_XRGB8888		(9 << 8)
#define SUNXI_DEBE_LAY_FBFMT_ARGB8888		(10 << 8)
#define SUNXI_DEBE_LAY_FBFMT_RGB888		(11 << 8)
#define SUNXI_DEBE_LAY_FBFMT_ARGB4444		(12 << 8)
#define SUNXI_DEBE_LAY_FBFMT_RGBA4444		(13 << 8)

#define SUNXI_DEBE_DLCDPCTL_REG			0x8b0
#define SUNXI_DEBE_DLCDPFRMBUF_ADDRCTL_REG	0x8b4
#define SUNXI_DEBE_DLCDPCOOR_REG0		0x8b8
#define SUNXI_DEBE_DLCDPCOOR_REG1		0x8bc

#define SUNXI_DEBE_INT_EN_REG			0x8c0
#define SUNXI_DEBE_INT_FLAG_REG			0x8c4
#define SUNXI_DEBE_REG_LOAD_FINISHED		BIT(1)

#define SUNXI_DEBE_HWCCTL_REG			0x8d8
#define SUNXI_DEBE_HWCFBCTL_REG			0x8e0
#define SUNXI_DEBE_WBCTL_REG			0x8f0
#define SUNXI_DEBE_WBADD_REG			0x8f4
#define SUNXI_DEBE_WBLINEWIDTH_REG		0x8f8
#define SUNXI_DEBE_SPREN_REG			0x900
#define SUNXI_DEBE_SPRFMTCTL_REG		0x908
#define SUNXI_DEBE_SPRALPHACTL_REG		0x90c
#define SUNXI_DEBE_IYUVCTL_REG			0x920
#define SUNXI_DEBE_IYUVADD_REG(c)		(0x930 + (0x4 * (c)))
#define SUNXI_DEBE_IYUVLINEWITDTH_REG(c)	(0x940 + (0x4 * (c)))
#define SUNXI_DEBE_YGCOEF_REG(c)		(0x950 + (0x4 * (c)))
#define SUNXI_DEBE_YGCONS_REG			0x95c
#define SUNXI_DEBE_URCOEF_REG(c)		(0x960 + (0x4 * (c)))
#define SUNXI_DEBE_URCONS_REG			0x96c
#define SUNXI_DEBE_VBCOEF_REG(c)		(0x970 + (0x4 * (c)))
#define SUNXI_DEBE_VBCONS_REG			0x97c
#define SUNXI_DEBE_KSCTL_REG			0x980
#define SUNXI_DEBE_KSBKCOLOR_REG		0x984
#define SUNXI_DEBE_KSFSTLINEWIDTH_REG		0x988
#define SUNXI_DEBE_KSVSCAFCT_REG		0x98c
#define SUNXI_DEBE_KSHSCACOEF_REG(x)		(0x9a0 + (0x4 * (x)))
#define SUNXI_DEBE_OCCTL_REG			0x9c0
#define SUNXI_DEBE_OCRCOEF_REG(x)		(0x9d0 + (0x4 * (x)))
#define SUNXI_DEBE_OCRCONS_REG			0x9dc
#define SUNXI_DEBE_OCGCOEF_REG(x)		(0x9e0 + (0x4 * (x)))
#define SUNXI_DEBE_OCGCONS_REG			0x9ec
#define SUNXI_DEBE_OCBCOEF_REG(x)		(0x9f0 + (0x4 * (x)))
#define SUNXI_DEBE_OCBCONS_REG			0x9fc
#define SUNXI_DEBE_SPRCOORCTL_REG(s)		(0xa00 + (0x4 * (s)))
#define SUNXI_DEBE_SPRATTCTL_REG(s)		(0xb00 + (0x4 * (s)))
#define SUNXI_DEBE_SPRADD_REG(s)		(0xc00 + (0x4 * (s)))
#define SUNXI_DEBE_SPRLINEWIDTH_REG(s)		(0xd00 + (0x4 * (s)))

#define SUNXI_DEBE_SPRPALTAB_OFF		0x4000
#define SUNXI_DEBE_GAMMATAB_OFF			0x4400
#define SUNXI_DEBE_HWCPATTERN_OFF		0x4800
#define SUNXI_DEBE_HWCCOLORTAB_OFF		0x4c00
#define SUNXI_DEBE_PIPE_OFF(p)			(0x5000 + (0x400 * (p)))

struct sunxi_debe {
	void __iomem *regs;
};

#endif /* __SUNXI_DEBE_H__ */
