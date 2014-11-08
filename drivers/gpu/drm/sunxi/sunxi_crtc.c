#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "sunxi_debe.h"
#include "sunxi_tcon.h"

struct sunxi_crtc {
	struct drm_crtc crtc;
	struct sunxi_debe *debe;
	struct sunxi_tcon *tcon;
};

static inline struct sunxi_crtc *to_sunxi_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct sunxi_crtc, crtc);
}

static int sunxi_crtc_mode_set(struct drm_crtc *c,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adj_mode,
			       int x, int y,
			       struct drm_framebuffer *old_fb)
{
	struct sunxi_crtc *crtc = to_sunxi_crtc(c);
	struct drm_plane *plane = c->primary;
	struct drm_framebuffer *fb = plane->fb;

	writel(SUNXI_DEBE_DISSIZE(mode->hdisplay, mode->vdisplay),
	       crtc->debe->regs + SUNXI_DEBE_DISSIZE_REG);

	plane->fb = old_fb;
	return plane->funcs->update_plane(plane, c, fb,
					  0, 0,
					  adj_mode->hdisplay,
					  adj_mode->vdisplay,
					  x << 16, y << 16,
					  adj_mode->hdisplay << 16,
					  adj_mode->vdisplay << 16);
}

static void sunxi_crtc_disable(struct drm_crtc *c)
{
	struct sunxi_crtc *crtc = to_sunxi_crtc(c);

	writel(readl(crtc->tcon->regs + SUNXI_DEBE_MODCTL_REG) &
	       ~SUNXI_DEBE_MODCTL_DEBE_EN,
	       crtc->debe->regs + SUNXI_DEBE_MODCTL_REG);

	writel(readl(crtc->debe->regs + SUNXI_DEBE_MODCTL_REG) &
	       ~SUNXI_DEBE_MODCTL_DEBE_EN,
	       crtc->debe->regs + SUNXI_DEBE_MODCTL_REG);

	/* Apply new config */
	writel(SUNXI_DEBE_REGBUFFCTL_AUTOLOAD_DIS |
	       SUNXI_DEBE_REGBUFFCTL_LOADCTL,
	       crtc->debe->regs + SUNXI_DEBE_REGBUFFCTL_REG);
}

static const struct drm_crtc_helper_funcs sunxi_crtc_helper_funcs = {
//	.dpms = rcar_du_crtc_dpms,
//	.mode_fixup = rcar_du_crtc_mode_fixup,
//	.prepare = rcar_du_crtc_mode_prepare,
//	.commit = rcar_du_crtc_mode_commit,
	.mode_set = sunxi_crtc_mode_set,
//	.mode_set_base = rcar_du_crtc_mode_set_base,
	.disable = sunxi_crtc_disable,
};

static int sunxi_crtc_page_flip(struct drm_crtc *crtc,
				     struct drm_framebuffer *fb,
				     struct drm_pending_vblank_event *event,
				     uint32_t flags)
{
	return 0;
}

static const struct drm_crtc_funcs sunxi_debe_crtc_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_crtc_helper_set_config,
	.page_flip = sunxi_crtc_page_flip,
};
