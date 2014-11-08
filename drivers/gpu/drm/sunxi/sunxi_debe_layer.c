#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "sunxi_debe.h"

struct sunxi_debe_layer {
	struct drm_plane plane;
	struct sunxi_debe *debe;
	int id;
};

static inline struct sunxi_debe_layer *to_debe_layer(struct drm_plane *plane)
{
	return container_of(plane, struct sunxi_debe_layer, plane);
}

static int sunxi_debe_layer_config_rgb(struct sunxi_debe_layer *layer,
				       struct drm_framebuffer *fb,
				       uint32_t src_x, uint32_t src_y)
{
	void __iomem *regs = layer->debe->regs;
	struct drm_gem_cma_object *gem;
	dma_addr_t paddr;
	u32 val;
	int bpp;

	val = readl(regs + SUNXI_DEBE_ATTCTL_REG1(layer->id)) &
	      ~SUNXI_DEBE_ATTCTL_REG1_LAY_FBFMT;

	switch (fb->pixel_format) {
	case DRM_FORMAT_RGB565:
		val |= SUNXI_DEBE_LAY_FBFMT_RGB565;
		break;

	case DRM_FORMAT_ARGB1555:
		val |= SUNXI_DEBE_LAY_FBFMT_ARGB1555;
		break;

	case DRM_FORMAT_RGBA5551:
		val |= SUNXI_DEBE_LAY_FBFMT_RGBA5551;
		break;

	case DRM_FORMAT_XRGB8888:
		val |= SUNXI_DEBE_LAY_FBFMT_XRGB8888;
		break;

	case DRM_FORMAT_ARGB8888:
		val |= SUNXI_DEBE_LAY_FBFMT_ARGB8888;
		break;

	case DRM_FORMAT_RGB888:
		val |= SUNXI_DEBE_LAY_FBFMT_RGB888;
		break;

	case DRM_FORMAT_ARGB4444:
		val |= SUNXI_DEBE_LAY_FBFMT_ARGB4444;
		break;

	case DRM_FORMAT_RGBA4444:
		val |= SUNXI_DEBE_LAY_FBFMT_RGBA4444;
		break;

	default:
		return -EINVAL;
	}

	writel(val, regs + SUNXI_DEBE_ATTCTL_REG1(layer->id));

	writel(fb->pitches[0] * 8,
	       regs + SUNXI_DEBE_LAYLINEWIDTH_REG(layer->id));

	gem = drm_fb_cma_get_gem_obj(fb, 0);
	bpp = drm_format_plane_cpp(fb->pixel_format, 0);
	paddr = gem->paddr + fb->offsets[0];
	paddr += (src_x * bpp) + (src_y * fb->pitches[0]);
	writel(paddr, regs + SUNXI_DEBE_LAYFB_L32ADD_REG(layer->id));

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	val = readl(regs + SUNXI_DEBE_LAYFB_H4ADD_REG);

	val &= SUNXI_DEBE_LAYFB_H4ADD_MSK(layer->id);
	val |= SUNXI_DEBE_LAYFB_H4ADD(layer->id, paddr >> 32);
	writel(val, regs + SUNXI_DEBE_LAYFB_H4ADD_REG);
#endif

	return 0;
}

static int sunxi_debe_layer_update_plane(struct drm_plane *plane,
					 struct drm_crtc *crtc,
					 struct drm_framebuffer *fb,
					 int crtc_x, int crtc_y,
					 unsigned int crtc_w,
					 unsigned int crtc_h,
					 uint32_t src_x, uint32_t src_y,
					 uint32_t src_w, uint32_t src_h)
{
	struct sunxi_debe_layer *layer = to_debe_layer(plane);
	void __iomem *regs = layer->debe->regs;
	int ret;

	writel(SUNXI_DEBE_LAYSIZE(crtc_w, crtc_h),
	       regs + SUNXI_DEBE_LAYSIZE_REG(layer->id));

	writel(SUNXI_DEBE_LAYCOOR((s16)crtc_x, (s16)crtc_y),
	       regs + SUNXI_DEBE_LAYCOOR_REG(layer->id));

	switch (fb->pixel_format) {
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_RGBA5551:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_RGB888:
	case DRM_FORMAT_ARGB4444:
	case DRM_FORMAT_RGBA4444:
		ret = sunxi_debe_layer_config_rgb(layer, fb, src_x, src_y);
		break;

	/*
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_YVYU:
	case DRM_FORMAT_UYVY:
	case DRM_FORMAT_VYUY:
	case DRM_FORMAT_AYUV:
	case DRM_FORMAT_YUV411:
	case DRM_FORMAT_YUV422:
	case DRM_FORMAT_YUV444:
	*/
	default:
		ret = -EINVAL;
		break;
	}

	if (ret)
		return ret;

	writel(SUNXI_DEBE_MODCTL_LAY_EN(layer->id) |
	       readl(regs + SUNXI_DEBE_REGBUFFCTL_REG),
	       regs + SUNXI_DEBE_REGBUFFCTL_REG);

	/* Apply new config */
	writel(SUNXI_DEBE_REGBUFFCTL_AUTOLOAD_DIS |
	       SUNXI_DEBE_REGBUFFCTL_LOADCTL,
	       regs + SUNXI_DEBE_REGBUFFCTL_REG);

	return 0;
}

static int sunxi_debe_layer_disable_plane(struct drm_plane *plane)
{
	struct sunxi_debe_layer *layer = to_debe_layer(plane);
	void __iomem *regs = layer->debe->regs;

	writel(readl(regs + SUNXI_DEBE_REGBUFFCTL_REG) &
	       ~SUNXI_DEBE_MODCTL_LAY_EN(layer->id),
	       regs + SUNXI_DEBE_REGBUFFCTL_REG);

	/* Apply new config */
	writel(SUNXI_DEBE_REGBUFFCTL_AUTOLOAD_DIS |
	       SUNXI_DEBE_REGBUFFCTL_LOADCTL,
	       regs + SUNXI_DEBE_REGBUFFCTL_REG);

	return 0;
}

static const struct drm_plane_funcs sunxi_debe_layer_funcs = {
	.update_plane = sunxi_debe_layer_update_plane,
	.disable_plane = sunxi_debe_layer_disable_plane,
	/*
	.set_property = rcar_du_plane_set_property,
	*/
	.destroy = drm_plane_cleanup,
};

static const uint32_t sunxi_debe_layer_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_RGBA5551,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_RGBA4444,
	/*
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YVYU,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY,
	DRM_FORMAT_AYUV,
	DRM_FORMAT_YUV411,
	DRM_FORMAT_YUV422,
	DRM_FORMAT_YUV444,
	*/
};

