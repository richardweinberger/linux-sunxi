#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#define SUNXI_DEBE_MAX			2

struct sunxi_de {
	struct regmap *debe[SUNXI_DEBE_MAX];
	struct regmap *tcon;
};

static int sunxi_de_load(struct drm_device *ddev, unsigned long flags)
{
	return 0;
}

static int sunxi_de_unload(struct drm_device *ddev)
{
	return 0;
}

static void sunxi_de_preclose(struct drm_device *ddev,
			      struct drm_file *file_priv)
{

}

static void sunxi_de_lastclose(struct drm_device *ddev)
{

}

static int sunxi_de_enable_vblank(struct drm_device *dev, int crtc)
{
	return 0;
}

static void sunxi_de_disable_vblank(struct drm_device *dev, int crtc)
{

}

static const struct file_operations rcar_du_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.release	= drm_release,
	.unlocked_ioctl	= drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= drm_compat_ioctl,
#endif
	.poll		= drm_poll,
	.read		= drm_read,
	.llseek		= no_llseek,
	.mmap		= drm_gem_cma_mmap,
};

static struct drm_driver sunxi_de_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME,
	.load			= sunxi_de_load,
	.unload			= sunxi_de_unload,
	.preclose		= sunxi_de_preclose,
	.lastclose		= sunxi_de_lastclose,
	.set_busid		= drm_platform_set_busid,
	.get_vblank_counter	= drm_vblank_count,
	.enable_vblank		= sunxi_de_enable_vblank,
	.disable_vblank		= sunxi_de_disable_vblank,
	.gem_free_object	= drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,
	.dumb_create		= drm_gem_cma_dumb_create,
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.fops			= &rcar_du_fops,
	.name			= "rcar-du",
	.desc			= "Renesas R-Car Display Unit",
	.date			= "20130110",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id sunxi_de_of_table[] = {
	{ .compatible = "allwinner,sun7i-a20-display-engine" },
	{ }
};
MODULE_DEVICE_TABLE(of, rcar_du_of_table);

static int sunxi_de_probe(struct platform_device *pdev)
{
        struct drm_device *ddev;
        int ret;

        ddev = drm_dev_alloc(&sunxi_de_driver, &pdev->dev);
        if (!ddev)
                return -ENOMEM;

        ret = drm_dev_set_unique(ddev, dev_name(ddev->dev));
        if (ret) {
                drm_dev_unref(ddev);
                return ret;
        }

        ret = drm_dev_register(ddev, 0);
        if (ret) {
                drm_dev_unref(ddev);
                return ret;
        }

        return 0;
}

static int sunxi_de_remove(struct platform_device *pdev)
{
        struct drm_device *ddev = platform_get_drvdata(pdev);

        drm_dev_unregister(ddev);
        drm_dev_unref(ddev);

	return 0;
}

static struct platform_driver rcar_du_platform_driver = {
	.probe		= sunxi_de_probe,
	.remove		= sunxi_de_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "sunxi-display-engine",
		.of_match_table = sunxi_de_of_table,
	},
};

module_platform_driver(rcar_du_platform_driver);

MODULE_AUTHOR("Boris Brezillon <boris.brezillon@free-electrons.com>");
MODULE_DESCRIPTION("Allwinner Display Engine DRM Driver");
MODULE_LICENSE("GPL");
