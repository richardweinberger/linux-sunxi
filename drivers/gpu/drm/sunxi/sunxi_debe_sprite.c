#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "sunxi_debe.h"

struct sunxi_debe_sprite {
	struct drm_plane plane;
	void __iomem *regs;
};

static inline struct sunxi_debe_sprite *to_debe_sprite(struct drm_plane *plane)
{
	return container_of(plane, struct sunxi_debe_sprite, plane);
}

struct sunxi_debe_layer {
	struct drm_plane plane;
	struct sunxi_debe *debe;
};

static inline struct sunxi_debe_sprite *to_debe_sprite(struct drm_plane *plane)
{
	return container_of(plane, struct sunxi_debe_sprite, plane);
}


static int sunxi_debe_plane_update(struct drm_plane *plane,
				   struct drm_crtc *crtc,
				   struct drm_framebuffer *fb,
				   int crtc_x, int crtc_y,
				   unsigned int crtc_w, unsigned int crtc_h,
				   uint32_t src_x, uint32_t src_y,
				   uint32_t src_w, uint32_t src_h)
{
	struct sunxi_debe_layer *layer =
}

static const struct drm_plane_funcs sunxi_debe_plane_funcs = {
	.update_plane = sunxi_debe_plane_update,
//	.disable_plane = sunxi_debe_plane_disable,
//	.set_property = rcar_du_plane_set_property,
	.destroy = drm_plane_cleanup,
};
