/*
 * Copyright (C) 2011 Franz Brauße.
 *
 * Partly based on lxfb
 * 	(which is Copyright (C) 2007 Advanced Micro Devices, Inc.),
 * xf86-video-geode
 * 	(which is (C) ),
 * with code borrowed from i915 and radeon drivers.
 */

#include <linux/cs5535.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/vga_switcheroo.h>

#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_file.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_modes.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "lx.h"

static void lx_crtc_load_lut(struct drm_crtc *crtc);
static void lx_crtc_fb_gamma_set(struct drm_crtc *crtc, u16 red, u16 green,
				 u16 blue, int regno);
static void lx_crtc_fb_gamma_get(struct drm_crtc *crtc, u16 *red, u16 *green,
				 u16 *blue, int regno);

/* This table is based on xf86-video-geode/src/cim/cim_modes.c. I was told that
 * one originated from AMD's WinCE driver.
 * It is nearly identical to the one in drivers/video/geode/lxfb_ops.c with the
 * exceptions being the values for 106.5 and 341.349 MHz missing there */
static struct {
	/* resulting dot freq = 48 MHz * (N + 1) / ((M + 1) * (P + 1))
	 * minimum pll freq (not post-divided by 4) is 15 MHz */
	u32 pllval;   /* [16]   : divide output by 4 (unused in this table) */
	              /* [14:12]: input clock divisor (M) */
	              /* [11:4] : dot clock PLL divisor (N) */
	              /* [3:0]  : post scaler divisor (P) */
	u32 freq_khz; /* resulting dot-freq in kHz when PLL stabilizes */
} const lx_pll_freq[] = {   /* M+1 N+1 P+1 */
	/* gap till 15000 */
	{ 0x31AC,  24923 }, /*   4, 27, 13 */
	{ 0x215D,  25175 }, /*   3, 22, 14 */
	{ 0x1087,  27000 }, /*   2,  9,  8 */
	{ 0x216C,  28322 }, /*   3, 23, 13 */
	{ 0x218D,  28560 }, /*   3, 25, 14 */
	{ 0x10C9,  31200 }, /*   2, 13, 10 */
	{ 0x3147,  31500 }, /*   4, 21,  8 */
	{ 0x10A7,  33320 }, /*   2, 11,  8 */
	{ 0x2159,  35112 }, /*   3, 22, 10 */
	{ 0x4249,  35500 }, /*   5, 37, 10 */
	{ 0x0057,  36000 }, /*   1,  6,  8 */
	{ 0x219A,  37889 }, /*   3, 26, 11 */
	{ 0x2158,  39168 }, /*   3, 22,  9 */
	{ 0x0045,  40000 }, /*   1,  5,  6 */
	/* gap */
	{ 0x0089,  43163 }, /*   1,  9, 10 */
	{ 0x10E7,  44900 }, /*   2, 15,  8 */
	{ 0x2136,  45720 }, /*   3, 20,  7 */
	{ 0x3207,  49500 }, /*   4, 33,  8 */
	{ 0x2187,  50000 }, /*   3, 25,  8 */
	/* gap */
	{ 0x4286,  56250 }, /*   5, 41,  7 */
	{ 0x10E5,  60650 }, /*   2, 15,  6 */
	{ 0x4214,  65000 }, /*   5, 34,  5 */
	{ 0x1105,  68179 }, /*   2, 17,  6 */
	/* gap */
	{ 0x31E4,  74250 }, /*   4, 31,  5 */
	{ 0x3183,  75000 }, /*   4, 25,  4 */
	{ 0x4284,  78750 }, /*   5, 41,  5 */
	{ 0x1104,  81600 }, /*   2, 17,  5 */
	/* gap */
	{ 0x6363,  94500 }, /*   7, 55,  4 */
	{ 0x5303,  97520 }, /*   6, 49,  4 */
	{ 0x2183, 100187 }, /*   3, 25,  4 */
	{ 0x2122, 101420 }, /*   3, 19,  3 */
	/* gap */
	{ 0x1081, 108000 }, /*   2,  9,  2 */
	{ 0x6201, 113310 }, /*   7, 33,  2 */
	{ 0x0041, 119650 }, /*   1,  5,  2 */
	/* gap */
	{ 0x41A1, 129600 }, /*   5, 27,  2 */
	{ 0x2182, 133500 }, /*   3, 25,  3 */
	{ 0x41B1, 135000 }, /*   5, 28,  2 */
	{ 0x0051, 144000 }, /*   1,  6,  2 */
	{ 0x41E1, 148500 }, /*   5, 31,  2 */
	{ 0x62D1, 157500 }, /*   7, 46,  2 */
	{ 0x31A1, 162000 }, /*   4, 27,  2 */
	{ 0x0061, 169203 }, /*   1,  7,  2 */
	{ 0x4231, 172800 }, /*   5, 36,  2 */
	{ 0x2151, 175500 }, /*   3, 22,  2 */
	/* gap */
	{ 0x52E1, 189000 }, /*   6, 47,  2 */
	{ 0x0071, 192000 }, /*   1,  8,  2 */
	{ 0x3201, 198000 }, /*   4, 33,  2 */
	{ 0x4291, 202500 }, /*   5, 42,  2 */
	{ 0x1101, 204750 }, /*   2, 17,  2 */
	{ 0x7481, 218250 }, /*   8, 73,  2 */
	{ 0x4170, 229500 }, /*   5, 24,  1 */
	{ 0x6210, 234000 }, /*   7, 34,  1 */
	/* gap */
	{ 0x3140, 251182 }, /*   4, 21,  1 */
	{ 0x6250, 261000 }, /*   7, 38,  1 */
	/* gap */
	{ 0x41C0, 278400 }, /*   5, 29,  1 */
	{ 0x5220, 280640 }, /*   6, 35,  1 */
	{ 0x0050, 288000 }, /*   1,  6,  1 */
	{ 0x41E0, 297000 }, /*   5, 31,  1 */
	/* gap */
	{ 0x2130, 320207 }, /*   3, 20,  1 */
	/* gap */
	{ 0x6310, 341349 }, /*   7, 50,  1 */
};

/* Loops through the above table searching the entry with the minimum deviation
 * from the given value. Also takes the minimum PLL freq into account.
 * If that value is minorized by the given argument and cannot be compensated,
 * -EINVAL is returned. Otherwise returns a positive value to be fed into the
 * upper 32 bits of the DOTPLL configuration MSR.
 */
static s32 lx_find_matching_pll(u32 *freq_khz, u32 freq_tolerance) {
	unsigned i, min_idx = 0;
	int f, min = INT_MAX;
	u32 freq = *freq_khz;
	int divided = (freq < LX_DOTPLL_MINKHZ) ? MSR_GLCP_DOTPLL_HI_DIV4 : 0;

	if (divided) {
		/* PLL freq would be too low, so need to insert the post-divider
		 * by 4 */
		freq *= 4;
	}

	if (freq < LX_DOTPLL_MINKHZ) {
		/* post-divider already in place, still too low.
		 * Sorry, can't help here ... only way out: increase the
		 * resolution and/or frame rate */
		return -MODE_CLOCK_LOW;
	}

	/* all values not in the PLL table are untested and therefore
	 * unsupported for the time being ... */
	if (freq < lx_pll_freq[0].freq_khz - freq_tolerance)
		return -MODE_CLOCK_LOW;
	if (freq > lx_pll_freq[ARRAY_SIZE(lx_pll_freq)-1].freq_khz + freq_tolerance)
		return -MODE_CLOCK_HIGH;

	/* find closest matching entry in the PLL table */
	for (i=0; i<ARRAY_SIZE(lx_pll_freq); i++) {
		f = lx_pll_freq[i].freq_khz - freq;
		if (f < 0)
			f = -f;
		if (f < min) {
			min = f;
			min_idx = i;
		}
	}

	/* closest match is still too far off */
	if (min > freq_tolerance) {
		DRV_INFO("unable to retrieve matching PLL setting for "
			 "requested frequency %u: table gaps too large\n",
			 *freq_khz);
		return -MODE_CLOCK_RANGE;
	}

	freq = lx_pll_freq[min_idx].freq_khz;
	if (divided)
		freq = (freq + freq / 2) / 4; /* round towards nearest */
	*freq_khz = freq;
	return divided | lx_pll_freq[min_idx].pllval;
}

static int lx_graphic_mode_valid(const struct drm_display_mode *mode,
				 u32 *fixed_freq, u32 *pllval)
{
	/* TODO: check crtc_* instead */

	u32 freq = mode->clock;
	s32 ret;

	if (mode->hdisplay < 64 || (mode->htotal - 1) >> 12)
		return MODE_H_ILLEGAL;
	if (mode->hsync_start - mode->hdisplay < 8)
		return MODE_HBLANK_NARROW;
	if (mode->hsync_end - mode->hsync_start < 8)
		return MODE_HSYNC_NARROW;
	/* no requirement for hdisplay / htotal / hsync_(start|stop) being a
	 * multiple of 8 anymore on this chip */

	if (mode->flags & DRM_MODE_FLAG_INTERLACE) {
		/* #total lines in interlaced mode must be odd */
		if (!(mode->vtotal & 1))
			return MODE_BAD_VVALUE;
	}
	/* no requirement for vdisplay / vtotal / vsync_(start|stop) being a
	 * multiple of 8 anymore on this chip */

	ret = lx_find_matching_pll(&freq, LX_MODE_FREQ_TOL);
	if (ret < 0)
		return -ret;
	if (fixed_freq)
		*fixed_freq = freq;
	if (pllval)
		*pllval = ret;

	return MODE_OK;
}

/* --------------------------------------------------------------------------
 * Buffer objects
 * -------------------------------------------------------------------------- */

static struct drm_mm_node * lx_mm_alloc_at(struct lx_priv *priv, unsigned size,
					   unsigned alignment, unsigned from,
					   int best_match)
{
	struct drm_mm_node *node;
	int ret;

	/* TODO: do any of these calls need mutex protection? */
	/* TODO: super inefficient: store *node in whereever it's needed */
	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		goto err_alloc;

	ret = drm_mm_insert_node_in_range(&priv->mman.mm, node, size, alignment,
	                                  0, from, priv->vmem_size,
	                                  best_match ? DRM_MM_INSERT_BEST : 0);
	if (ret < 0)
		goto err_insert;

	return node;

err_insert:
	kfree(node);
err_alloc:
	return NULL;
}

#define lx_mm_alloc(priv, size, alignment, best_match) \
	lx_mm_alloc_at((priv), (size), (alignment), 0, (best_match))

static int lx_bo_addmap(struct lx_priv * priv, struct lx_bo * bo)
{
	resource_size_t off;
	int ret;

	if (bo->map)
		return 0;

	off = bo->node->start + priv->vmem_addr;
	ret = drm_legacy_addmap(priv->ddev, off, bo->node->size,
	                        _DRM_FRAME_BUFFER,
	                        _DRM_WRITE_COMBINING | _DRM_DRIVER, &bo->map);

	return ret;
}

static int lx_bo_rmmap(struct lx_priv *priv, struct lx_bo *bo)
{
	if (!bo->map)
		return 0;

	if (bo->map->handle) {
		drm_legacy_ioremapfree(bo->map, priv->ddev);
		bo->map->handle = NULL;
	}

	drm_legacy_rmmap(priv->ddev, bo->map);

	bo->map = NULL;
	return 0;
}

static struct lx_bo * lx_bo_create(struct drm_file *file_priv,
				   struct drm_mm_node *node)
{
	struct lx_bo *bo;
	int ret = 0;

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return NULL;

	if (file_priv) {
		idr_preload(GFP_KERNEL);
		spin_lock(&file_priv->table_lock);
		ret = idr_alloc(&file_priv->object_idr, bo, 1, 0, GFP_NOWAIT);
		spin_unlock(&file_priv->table_lock);
		idr_preload_end();
	}

	if (ret < 0) {
		if (ret == -ENOSPC)
			DRV_ERR("too many BOs");
		goto err;
	}

	bo->id = ret;
	bo->node = node;
	return bo;

err:
	kfree(bo);
	return NULL;
}

static void lx_bo_destroy(struct lx_priv *priv, struct drm_file *file_priv,
			  struct lx_bo *bo)
{
	int ret;

	if (file_priv) {
		spin_lock(&file_priv->table_lock);
		idr_remove(&file_priv->object_idr, bo->id);
		spin_unlock(&file_priv->table_lock);
	}

	ret = lx_bo_rmmap(priv, bo);
	if (ret)
		DRM_ERROR("removing local map for object %d, size: %llu @ %llx; %d\n",
			  bo->id, bo->node->size,
			  bo->node->start + priv->vmem_addr, ret);

	if (bo->free)
		bo->free(priv, bo);

	if (bo->node)
		drm_mm_remove_node(bo->node);

	kfree(bo);
}

/* --------------------------------------------------------------------------
 * FB
 * -------------------------------------------------------------------------- */

static int lx_user_fb_create_handle(struct drm_framebuffer *fb,
				    struct drm_file *file_priv,
				    unsigned int *handle)
{
	struct lx_fb *lfb = to_lx_fb(fb);

	if (!lfb->bo) {
		DRM_ERROR("non-managed fb-object\n");
		return -ENOENT;
	}

	DRM_DEBUG_DRIVER("handle: %d\n", lfb->bo->id);
	*handle = lfb->bo->id;

	return 0;
}

static void lx_user_fb_destroy(struct drm_framebuffer *fb)
{
	struct lx_fb *lfb = to_lx_fb(fb);

	drm_framebuffer_cleanup(fb);
	kfree(lfb);
}

static struct drm_framebuffer_funcs lx_fb_funcs = {
	.create_handle = lx_user_fb_create_handle,
	.destroy = lx_user_fb_destroy,
#if 0
	/**
	 * Optinal callback for the dirty fb ioctl.
	 *
	 * Userspace can notify the driver via this callback
	 * that a area of the framebuffer has changed and should
	 * be flushed to the display hardware.
	 *
	 * See documentation in drm_mode.h for the struct
	 * drm_mode_fb_dirty_cmd for more information as all
	 * the semantics and arguments have a one to one mapping
	 * on this function.
	 */
	int (*dirty)(struct drm_framebuffer *framebuffer,
		     struct drm_file *file_priv, unsigned flags,
		     unsigned color, struct drm_clip_rect *clips,
		     unsigned num_clips);
#endif
};

static int lx_fb_init(struct lx_priv *priv, struct lx_fb *lfb,
		      const struct drm_mode_fb_cmd2 *mode_cmd,
		      struct lx_bo *bo)
{
	int ret;

	lfb->bo = bo;

	ret = drm_framebuffer_init(priv->ddev, &lfb->base, &lx_fb_funcs);
	if (ret) {
		DRM_DEBUG_DRIVER("drm_framebuffer_init failed: %d\n", ret);
		return ret;
	}

	drm_helper_mode_fill_fb_struct(priv->ddev, &lfb->base, mode_cmd);

	return 0;
}

static struct fb_ops lx_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = drm_fb_helper_check_var,
	.fb_set_par = drm_fb_helper_set_par,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_pan_display = drm_fb_helper_pan_display,
	.fb_blank = drm_fb_helper_blank,
	.fb_setcmap = drm_fb_helper_setcmap,
	.fb_debug_enter = drm_fb_helper_debug_enter,
	.fb_debug_leave = drm_fb_helper_debug_leave,
};

/* pitch needs to be aligned to qword boundary */
#define lx_gfx_pitch_for(width, bpp)	ALIGN((width) * ALIGN((bpp), 8) / 8, 8)

#ifndef CONFIG_DRM_FBDEV_EMULATION
# error require CONFIG_DRM_FBDEV_EMULATION=y
#endif

/**
 * @sizes  init params for new fb object
 * @return negative: error, zero: no modes retrieved -> don't use fb,
 *         positive: ok, new fb created */
static int lx_fb_helper_probe(struct drm_fb_helper *helper,
			      struct drm_fb_helper_surface_size *sizes)
{
	struct lx_fb *lfb = helper_to_lx_fb(helper);
	struct lx_priv *priv = lfb->priv;
	struct drm_device *dev = priv->ddev;
	struct drm_framebuffer *fb = &lfb->base;
	struct fb_info *info;
	struct drm_mode_fb_cmd mode_cmd;
	struct drm_mm_node *node;
	struct lx_bo *bo;
	unsigned size;
	unsigned at;
	int ret;

	DRM_DEBUG_DRIVER("helper: %p, sizes: %p, priv: %p, fb: %p\n",
			 helper, sizes, priv, helper->fb);
	if (helper->fb)
		return 0;

	mode_cmd.width  = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.bpp    = sizes->surface_bpp;
	mode_cmd.depth  = sizes->surface_depth;
	mode_cmd.pitch  = lx_gfx_pitch_for(mode_cmd.width, mode_cmd.bpp);

	/* allocate backing store */
	size = mode_cmd.pitch * mode_cmd.height;

	/* Allocate a chunk from the far end of memory, so the compression
	 * feature is still available for the more serious users (like X).
	 * The assumption here is that this code is the first to ever allocate
	 * something from vmem (which is true so far since this is called at
	 * init time). */
	if (priv->vmem_size < size) {
		DRM_ERROR("vmem too small to create fb, need: %u KB, got %u KB; "
			  "reduce resolution or color depth or increase the "
			  "size of available Video-RAM (usually in the BIOS)\n",
			  size >> 10, (unsigned)(priv->vmem_size >> 10));
		return -ENOMEM;
	}
	at = priv->vmem_size - size;
	node = lx_mm_alloc_at(priv, size, 8, at, 1);
	if (!node) {
		DRM_ERROR("allocating initial fbcon fb at %u, sz: %u\n",
			  at, size);
		return -ENOMEM;
	}
	bo = lx_bo_create(NULL, node);
	if (!bo) {
		DRM_ERROR("creating bo for initial fbcon fb\n");
		drm_mm_remove_node(node);
		return -ENOMEM;
	}
	ret = lx_bo_addmap(priv, bo);
	if (ret) {
		DRM_ERROR("adding map for initial fbcon fb's bo: %d\n", ret);
		goto err_bo_destroy;
	}
	drm_legacy_ioremap_wc(bo->map, priv->ddev);

	mutex_lock(&dev->struct_mutex);

	info = framebuffer_alloc(0, &priv->pdev->dev);
	if (!info) {
		DRM_ERROR("allocating fb info\n");
		ret = -ENOMEM;
		goto err_unlock0;
	}

	info->par = helper;

	/* TODO: use drm_driver_legacy_fb_format() instead to allow for quirks
	 *       to be set? */
	/* modelled mainly after logic in drm_mode_addfb() */
	struct drm_mode_fb_cmd2 mode_cmd2 = {
		.fb_id        = mode_cmd.fb_id,
		.width        = mode_cmd.width,
		.height       = mode_cmd.height,
		.pixel_format = drm_mode_legacy_fb_format(mode_cmd.bpp, mode_cmd.depth),
		.flags        = 0,
		.pitches[0]   = mode_cmd.pitch,
		.handles[0]   = mode_cmd.handle,
	};
	if (mode_cmd2.pixel_format == DRM_FORMAT_INVALID) {
		DRM_DEBUG_DRIVER("bad {bpp:%d, depth:%d}\n", mode_cmd.bpp, mode_cmd.depth);
		return -EINVAL;
	}

	ret = lx_fb_init(priv, lfb, &mode_cmd2, bo);
	if (ret) {
		DRM_ERROR("Failed to initialize drm_framebuffer: %d\n", ret);
		goto err_unlock1;
	}

	/* setup helper */
	helper->fb = fb;
	helper->fbdev = info;

	info->flags = FBINFO_DEFAULT /*| FBINFO_VIRTFB*/;
	info->fbops = &lx_fb_ops;

	info->fix.smem_start = bo->map->offset;
	info->fix.smem_len = size;
	info->screen_base = bo->map->handle;
	info->screen_size = size;

	drm_fb_helper_fill_info(info, helper, sizes);

	info->apertures = alloc_apertures(1);
	if (info->apertures) {
		info->apertures->ranges[0].base = info->fix.smem_start;
		info->apertures->ranges[0].size = size;
	}

	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret) {
		DRV_INFO("error allocating cmap: %d, ignoring\n", ret);
		info->cmap.len = 0;
	}

	mutex_unlock(&dev->struct_mutex);

	/* TODO: init info->pixmap */

	DRM_INFO("fb mappable at 0x%lx\n",  info->fix.smem_start);
	DRM_INFO("vram aper   at 0x%lx\n", (unsigned long)priv->vmem_addr);
	DRM_INFO("size %u (0x%x)\n", size, size);
	DRM_INFO("fb depth is %d\n", fb->format->depth);
	DRM_INFO("   pitch is %d\n", fb->pitches[0]);
	DRM_INFO("  height is %d\n", fb->height);
	DRM_INFO("   width is %d\n", fb->width);

	vga_switcheroo_client_fb_set(priv->pdev, info);

	return 1;

err_unlock1:
	framebuffer_release(info);
err_unlock0:
	mutex_unlock(&dev->struct_mutex);
err_bo_destroy:
	lx_bo_destroy(priv, NULL, bo);
	return ret;
}

static struct drm_fb_helper_funcs lx_fb_helper_funcs = {
#if 0
	.gamma_get = lx_crtc_fb_gamma_get,
#endif
	.fb_probe  = lx_fb_helper_probe,
};

static int lx_fbdev_init(struct drm_device *dev)
{
	struct lx_priv *priv = dev->dev_private;
	struct lx_fb *lfb;
	int bpp = 8;
	int ret;

	DRM_DEBUG_DRIVER("\n");

	lfb = kzalloc(sizeof(*lfb), GFP_KERNEL);
	if (!lfb)
		return -ENOMEM;

	lfb->priv = priv;

	priv->fb = lfb;

	drm_fb_helper_prepare(dev, &lfb->helper, &lx_fb_helper_funcs);
	ret = drm_fb_helper_init(dev, &lfb->helper);
	if (ret) {
		DRM_DEBUG_DRIVER("error init'ing fb_helper: %d\n", ret);
		return ret;
	}

	drm_fb_helper_initial_config(&lfb->helper, bpp);

	return 0;
}

static void lx_fbdev_fini(struct drm_device *dev)
{
	struct lx_priv *priv = dev->dev_private;
	struct lx_fb *lfb = priv->fb;
	struct fb_info *info = lfb->helper.fbdev;

	if (info) {
		unregister_framebuffer(info);
		if (info->cmap.len)
			fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
	}

	drm_fb_helper_fini(&lfb->helper);
	drm_framebuffer_cleanup(&lfb->base);

	if (lfb->bo)
		lx_bo_destroy(priv, NULL, lfb->bo);

	kfree(lfb);
	priv->fb = NULL;
}

static struct drm_framebuffer *
lx_user_fb_create(struct drm_device *dev,
		  struct drm_file *file_priv,
		  const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct lx_bo *bo = idr_find(&file_priv->object_idr, mode_cmd->handles[0]);
	struct lx_fb *lfb;
	int ret;

	if (!bo) {
		DRM_ERROR("object with handle %d doesn't exist\n",
			  mode_cmd->handles[0]);
		return ERR_PTR(-ENOENT);
	}

	lfb = kzalloc(sizeof(*lfb), GFP_KERNEL);
	if (!lfb)
		return ERR_PTR(-ENOMEM);

	lfb->priv = dev->dev_private;
	ret = lx_fb_init(dev->dev_private, lfb, mode_cmd, bo);
	if (ret) {
		kfree(lfb);
		return ERR_PTR(ret);
	}

	return &lfb->base;
}


static void lx_mode_output_poll_changed(struct drm_device *dev) {
	struct lx_priv *priv = dev->dev_private;
	DRM_DEBUG_DRIVER("\n");
	drm_fb_helper_hotplug_event(&priv->fb->helper);
}

static struct drm_mode_config_funcs lx_mode_funcs = {
	.fb_create = lx_user_fb_create,
	.output_poll_changed = lx_mode_output_poll_changed,
};

/* --------------------------------------------------------------------------
 * Connector
 * -------------------------------------------------------------------------- */

static struct drm_encoder * lx_connector_best_encoder(
					struct drm_connector *connector);

static struct drm_encoder * lx_connector_attached_encoder(
		struct drm_connector *connector)
{
	return connector->encoder;
}

static bool lx_connector_get_edid(struct drm_connector *connector,
				  struct i2c_adapter *ddc, bool verbose)
{
	struct lx_connector *lx_conn = to_lx_connector(connector);
	struct edid *edid = NULL;

	kfree(lx_conn->edid);
	
	if (!ddc)
		goto out;
	if (!lx_ddc_probe(ddc))
		goto out;

	edid = drm_get_edid(connector, ddc);

	if (!edid) {
		if (verbose)
			DRV_INFO("failed to retrieve EDID for %s\n",
				 connector->name);
	} else if (!drm_edid_is_valid(edid)) {
		if (verbose)
			DRV_INFO("EDID retrieved for %s is invalid\n",
				 connector->name);
		kfree(edid);
		edid = NULL;
	}

out:
	/* In any case update the edid property to reflect the current status */
	drm_connector_update_edid_property(connector, edid);
	lx_conn->edid = edid;

	return edid != NULL;
}

static enum drm_connector_status lx_connector_detect(
		struct drm_connector *connector, bool force)
{
	struct lx_priv *priv = connector->dev->dev_private;
	struct lx_connector *lx_conn = to_lx_connector(connector);
	struct drm_encoder *encoder;
	const struct drm_encoder_helper_funcs *encoder_funcs;
	const struct drm_connector_helper_funcs *connector_funcs;
	enum drm_connector_status status = connector_status_unknown;

	switch (lx_conn->id) {
	case LX_CONNECTOR_VGA:
		if (lx_connector_get_edid(connector, priv->ddc, false))
			status = connector_status_connected;
		break;
	case LX_CONNECTOR_LVDS:
	case LX_CONNECTOR_VOP:
		/* TODO: detect connector status */
		break;
	}

	if (force && status == connector_status_unknown) {
		connector_funcs = connector->helper_private;
		encoder = connector_funcs->best_encoder(connector);
		if (encoder) {
			encoder_funcs = encoder->helper_private;
			status = encoder_funcs->detect(encoder, connector);
		}
	}

	return status;
}

static int lx_connector_mode_valid(struct drm_connector *connector,
				   struct drm_display_mode *mode)
{
	struct lx_connector *lx_conn = to_lx_connector(connector);

	if (drm_mode_vrefresh(mode) > LX_MODE_MAX_VFREQ)
		return MODE_CLOCK_HIGH;

	if (lx_conn->max_hz && mode->clock - LX_MODE_FREQ_TOL > lx_conn->max_hz)
		return MODE_CLOCK_HIGH;

	if (lx_conn->max_width && mode->hdisplay > lx_conn->max_width)
		return MODE_BAD_HVALUE;

	if (lx_conn->max_height && mode->vdisplay > lx_conn->max_height)
		return MODE_BAD_VVALUE;

	return lx_graphic_mode_valid(mode, NULL, NULL);
}

static void lx_connector_destroy(struct drm_connector *connector)
{
	struct lx_connector *lx_conn = to_lx_connector(connector);

	DRM_DEBUG_DRIVER("\n");

	/* TODO: switch off outputs? */

	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);

	if (lx_conn->edid)
		kfree(lx_conn->edid);

	/* TODO: release & cleanup output device(s) / registers */
}

/** Called through drm_connector_helper_funcs by
 * drm_helper_probe_single_connector_modes.
 *
 * Probes the connector for modes or adds some default modes provided by
 * EDID and returns the number of modes successfully added.
 */
static int lx_connector_vga_get_modes(struct drm_connector *connector)
{
	struct lx_priv *priv = connector->dev->dev_private;
	struct lx_connector *lx_conn = to_lx_connector(connector);
	bool have_edid = lx_connector_get_edid(connector, priv->ddc, true);
	int num_modes;

	if (have_edid) {
		num_modes = drm_add_edid_modes(connector, lx_conn->edid);
	} else {
		num_modes = drm_add_modes_noedid(connector, 1920, 1440);
	}
	DRV_INFO("added %d %s modes for connector %s\n", num_modes,
		 have_edid ? "EDID" : "static",
		 connector->name);

	return num_modes;
}

static void lx_connector_vga_reset(struct drm_connector *connector) {
	struct lx_priv *priv = connector->dev->dev_private;
	u32 dcfg;

	DRM_DEBUG_DRIVER("\n");

	dcfg = read_vp(priv, VP_DCFG);
	/* reset display control logic */
	write_vp(priv, VP_DCFG, dcfg & ~VP_DCFG_CRT_EN);

	dcfg |= VP_DCFG_DAC_BL_EN;         /* normal DAC blanking */
	dcfg |= VP_DCFG_VSYNC_EN | VP_DCFG_HSYNC_EN;
	dcfg |= VP_DCFG_CRT_EN;            /* re-enable display control logic */
	write_vp(priv, VP_DCFG, dcfg);
}

static const struct drm_connector_helper_funcs lx_connector_vga_helper_funcs = {
	.get_modes	= lx_connector_vga_get_modes,
	.mode_valid	= lx_connector_mode_valid,
	.best_encoder	= lx_connector_best_encoder,
};

static const struct drm_connector_funcs lx_connector_vga_funcs = {
	.dpms		= drm_helper_connector_dpms,
	.detect		= lx_connector_detect,
	.fill_modes	= drm_helper_probe_single_connector_modes,
	.destroy	= lx_connector_destroy,
	.reset		= lx_connector_vga_reset,
#if 0
	void (*save)(struct drm_connector *connector);
	void (*restore)(struct drm_connector *connector);

	int (*set_property)(struct drm_connector *connector,
			    struct drm_property *property,
			    uint64_t val);
	void (*force)(struct drm_connector *connector);
#endif
};

static struct {
	int type;
	int encoder;
	const struct drm_connector_funcs *funcs;
	const struct drm_connector_helper_funcs *hfuncs;
} const lx_connector_types[LX_NUM_CONNECTORS] = {
	[LX_CONNECTOR_VGA] = {
		DRM_MODE_CONNECTOR_VGA,
		LX_ENCODER_DAC,
		&lx_connector_vga_funcs,
		&lx_connector_vga_helper_funcs
	},
	[LX_CONNECTOR_LVDS] = {
		DRM_MODE_CONNECTOR_LVDS,
		LX_ENCODER_PANEL,
	},
};

static struct drm_encoder * lx_connector_best_encoder(
					struct drm_connector *connector)
{
	struct lx_priv *priv = connector->dev->dev_private;
	struct lx_connector *lx_conn = to_lx_connector(connector);
	enum lx_encoders encoder_id = lx_connector_types[lx_conn->id].encoder;
	return &priv->encoders[encoder_id].base;
}

static struct drm_connector * lx_connector_init(struct drm_device *dev,
						enum lx_connectors connector_id)
{
	struct lx_priv *priv = dev->dev_private;
	struct lx_connector *lx_conn = &priv->connectors[connector_id];
	struct drm_connector *connector = &lx_conn->base;

	if (lx_conn->id < LX_NUM_CONNECTORS)
		return connector;
	lx_conn->id = connector_id;

	drm_connector_init(dev, connector, lx_connector_types[connector_id].funcs,
			   lx_connector_types[connector_id].type);
	drm_connector_helper_add(connector,
				 lx_connector_types[connector_id].hfuncs);

#if 0
/* TODO: */
	drm_connector_attach_property(
			connector,
			dev->mode_config.scaling_mode_property,
			DRM_MODE_SCALE_NONE);
#endif

	switch (connector_id) {
	case LX_CONNECTOR_VGA:
		lx_conn->max_width = 1920;
		lx_conn->max_height = 1440;
		lx_conn->max_hz = 340000;
		connector->polled = DRM_CONNECTOR_POLL_CONNECT;
		/* TODO: shall remember whether last ddc query was successful */
		if (0 && priv->ddc)
			connector->polled |= DRM_CONNECTOR_POLL_DISCONNECT;
		break;
	case LX_CONNECTOR_LVDS:
		lx_conn->max_width = 1600;
		lx_conn->max_height = 1200;
		lx_conn->max_hz = 162000;
		break;
	case LX_CONNECTOR_VOP:
		lx_conn->max_width = 1920;
		lx_conn->max_height = 1080;
		lx_conn->max_hz = 75000;
		connector->interlace_allowed = 1;
		connector->doublescan_allowed = 1;
		break;
	}

	/* TODO: really setup outputs? */
#if 0 /* TODO: This API did go away since v3.0. Where to? */
	connector->initial_x = 0;
	connector->initial_y = 0;
#endif

	/* TODO: handle return value */
	drm_connector_register(connector);

	return connector;
}

/* --------------------------------------------------------------------------
 * Encoder
 * -------------------------------------------------------------------------- */

/* DAC test value; monitor connected leads to about 0.3V on the RGB cables */
#define LX_LOAD_DETECT_DAC_VAL 0x70

static enum drm_connector_status lx_encoder_dac_detect(
		struct drm_encoder *encoder, struct drm_connector *connector)
{
	struct lx_priv *priv = encoder->dev->dev_private;
	
	/* DAC load detect:
	 *   VTM[6] = 0
	 *   MSR_DIAG_VB[19:16] = 0b0101
	 *   run test via MSR_DIAG_VB[27:20] ~= 0x80 and GLCP_DAC[13:11]
	 *   MSR_DIAG_VB[19:16] = 0b1000 */
	u32 diag_lo, diag_hi, v, dac_lo, dac_hi;
	u32 saved_vtm = read_vp(priv, VP_VTM);

	rdmsr(MSR_VP_DIAG, diag_lo, diag_hi);

	/* DAC outputs below (0.7V / 2) iff monitor connected */
	v = diag_lo & ~(0xfff << 16);
	v |= 5 << 16;                          /* enter DAC test mode */
	v |= LX_LOAD_DETECT_DAC_VAL << 20;

	write_vp(priv, VP_VTM, saved_vtm & ~(1U << 6));
	wrmsr(MSR_VP_DIAG, v, diag_hi);

	udelay(10);
	/* read DAC compare values */
	rdmsr(MSR_GLCP_DAC, dac_lo, dac_hi);

	/* restore register values */
	wrmsr(MSR_VP_DIAG, diag_lo, diag_hi);
	write_vp(priv, VP_VTM, saved_vtm);

	DRM_DEBUG_DRIVER("DAC MSR: %x\n", dac_lo);
	dac_lo >>= 11;
	dac_lo &= 7;
	if (dac_lo == 7) {
		/* all 3 DACs report an output voltage above 0.35V, so there is
		 * no monitor connected that would pull down this value */
		return connector_status_disconnected;
	} else if (dac_lo == 0) {
		/* all 3 DACs report an output voltage below 0.35V */
		return connector_status_connected;
	} else {
		/* appearently there is some deviation between the DACs, so
		 * ignore the results. Need to raise LX_LOAD_DETECT_DAC_VAL */
		DRV_INFO("DAC load detection unreliable, please report this "
			 "incident to <dev@karlchenofhell.org>\n");
		return connector_status_unknown;
	}
}

static const char *dpms_mode_names[] = { "on", "standby", "suspend", "off" };

/* controls just the CRT signals */
static void lx_encoder_dac_dpms(struct drm_encoder *encoder, int mode) {
	struct lx_priv *priv = encoder->dev->dev_private;
	struct lx_encoder *lx_enc = to_lx_encoder(encoder);
	u32 dcfg, misc;

	DRM_DEBUG_DRIVER("%s\n", dpms_mode_names[mode]);

	dcfg  = read_vp(priv, VP_DCFG);
	misc  = read_vp(priv, VP_MISC);
	dcfg &= ~(VP_DCFG_DAC_BL_EN | VP_DCFG_VSYNC_EN | VP_DCFG_HSYNC_EN);
	misc |= VP_MISC_APWRDN | VP_MISC_DACPWRDN;

	switch (mode) {
	case DRM_MODE_DPMS_ON: /* DACs unblanked & [hv]sync enabled */
		dcfg |= VP_DCFG_DAC_BL_EN | VP_DCFG_VSYNC_EN | VP_DCFG_HSYNC_EN;
		misc &= ~(VP_MISC_APWRDN | VP_MISC_DACPWRDN);
		lx_enc->enabled = true;
		break;
	case DRM_MODE_DPMS_STANDBY: /* DACs blanked & hsync disabled */
		dcfg |= VP_DCFG_VSYNC_EN;
		lx_enc->enabled = false;
		break;
	case DRM_MODE_DPMS_SUSPEND: /* DACs blanked & vsync disabled */
		dcfg |= VP_DCFG_HSYNC_EN;
		lx_enc->enabled = false;
		break;
	case DRM_MODE_DPMS_OFF: /* DACs blanked & [hv]sync disabled */
		lx_enc->enabled = false;
		break;
	}

	write_vp(priv, VP_DCFG, dcfg);
	write_vp(priv, VP_MISC, misc);
}

static void lx_encoder_prepare(struct drm_encoder *encoder)
{
	const struct drm_encoder_helper_funcs *encoder_funcs;

	encoder_funcs = encoder->helper_private;
	encoder_funcs->dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void lx_encoder_commit(struct drm_encoder *encoder)
{
	const struct drm_encoder_helper_funcs *encoder_funcs;

	encoder_funcs = encoder->helper_private;
	encoder_funcs->dpms(encoder, DRM_MODE_DPMS_ON);
}

static bool lx_encoder_dac_mode_fixup(struct drm_encoder *encoder,
				      struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode)
{
	/* TODO: adjust for scaling (using DRM_MODE_SCALE_*) & set flags as
	 * necessary */
	return true;
}

static unsigned lx_crtc_get_enabled_encoders(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_encoder *encoder;
	unsigned idx = 0, mask = 0;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->crtc == crtc && to_lx_encoder(encoder)->enabled)
			mask |= 1 << idx;
		idx++;
	}

	return mask;
}

static unsigned lx_crtc_get_enabled_connectors(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	unsigned idx = 0, mask = 0;

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		encoder = connector->encoder;
		if (encoder) {
			if (encoder->crtc == crtc &&
			    to_lx_encoder(encoder)->enabled)
				mask |= 1 << idx;
		}
		idx++;
	}

	return mask;
}

static void lx_encoder_dac_mode_set(struct drm_encoder *encoder,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted_mode)
{
	struct lx_encoder *lx_enc = to_lx_encoder(encoder);
	struct lx_priv *priv = encoder->dev->dev_private;
	u32 lo, hi;
	int cloned;

	DRM_DEBUG_DRIVER("enabled: %d\n", lx_enc->enabled);
	return;

	if (!lx_enc->enabled)
		return;

	/* select output format */

	rdmsr(LX_MSR(LX_VP, LX_GLD_MSR_CONFIG), lo, hi);

	lo &= ~MSR_VP_GLD_MSR_CONFIG_FMT_MASK;
	lo &= ~MSR_VP_GLD_MSR_CONFIG_FPC;

	switch (lx_enc->id) {
	case LX_ENCODER_DAC:
	case LX_ENCODER_PANEL:
		cloned  = priv->encoders[LX_ENCODER_DAC].enabled;
		cloned &= priv->encoders[LX_ENCODER_PANEL].enabled;
		if (cloned) {
			lo |= MSR_VP_GLD_MSR_CONFIG_FMT_FP;
			lo |= MSR_VP_GLD_MSR_CONFIG_FPC;
		} else if (lx_enc->id == LX_ENCODER_DAC) {
			lo |= MSR_VP_GLD_MSR_CONFIG_FMT_CRT;
		} else {
			lo |= MSR_VP_GLD_MSR_CONFIG_FMT_FP;
		}
		break;
	}

	wrmsr(LX_MSR(LX_VP, LX_GLD_MSR_CONFIG), lo, hi);

	
}

/* encoder is just the DAC or FPDC, nothing else, especially no transformation
 * or gamma adjustment */
static const struct drm_encoder_helper_funcs lx_encoder_dac_helper_funcs = {
	.detect = lx_encoder_dac_detect,
	.dpms = lx_encoder_dac_dpms,
	.prepare = lx_encoder_prepare,
	.commit = lx_encoder_commit,
#if 0 /* TODO: This API went away since v3.0. Where to? */
	.mode_fixup = lx_encoder_dac_mode_fixup,
#endif
	.mode_set = lx_encoder_dac_mode_set,
#if 0
	void (*save)(struct drm_encoder *encoder);
	void (*restore)(struct drm_encoder *encoder);

	struct drm_crtc *(*get_crtc)(struct drm_encoder *encoder);
	/* disable encoder when not in use - more explicit than dpms off */
	void (*disable)(struct drm_encoder *encoder);
#endif
};

static const struct drm_encoder_funcs lx_encoder_funcs = {
	// void (*reset)(struct drm_encoder *encoder);
	.destroy = drm_encoder_cleanup
};

static struct {
	int type;
	const struct drm_encoder_helper_funcs *hfuncs;
} const lx_encoder_types[LX_NUM_ENCODERS] = {
	[LX_ENCODER_DAC]    = { DRM_MODE_ENCODER_DAC, &lx_encoder_dac_helper_funcs },
	[LX_ENCODER_PANEL]  = { DRM_MODE_ENCODER_LVDS, },
};

static struct drm_encoder * lx_encoder_init(struct drm_device *dev,
					    enum lx_encoders encoder_id)
{
	struct lx_priv *priv = dev->dev_private;
	struct lx_encoder *lx_enc = &priv->encoders[encoder_id];
	struct drm_encoder *encoder = &lx_enc->base;

	if (lx_enc->id < LX_NUM_ENCODERS)
		return encoder;
	lx_enc->id = encoder_id;

	drm_encoder_init(dev, encoder, &lx_encoder_funcs,
			 lx_encoder_types[encoder_id].type,
			 "lx:enc:%s",
			 encoder_id == LX_ENCODER_DAC ? "dac" : "panel");

	drm_encoder_helper_add(encoder, lx_encoder_types[encoder_id].hfuncs);

	return encoder;
}

/* --------------------------------------------------------------------------
 * CRTC
 * -------------------------------------------------------------------------- */

/** Sets the color ramps on behalf of fbcon */
static void lx_crtc_fb_gamma_set(struct drm_crtc *crtc, u16 red, u16 green,
				 u16 blue, int regno)
{
	struct lx_priv *priv = crtc->dev->dev_private;
	struct lx_fb *lfb = priv->fb;
	struct lx_crtc *lx_crtc = to_lx_crtc(crtc);
	struct lx_rgb *lut_entry = lx_crtc->lut + regno;

	if (!lfb) {
		DRM_ERROR("lx_crtc_fb_gamma_set w/o fb\n");
		return;
	}

	DRM_DEBUG_DRIVER("%02x: %04x %04x %04x\n", regno, red, green, blue);

	lut_entry->r = red >> 8;
	lut_entry->g = green >> 8;
	lut_entry->b = blue >> 8;
}

/** Gets the color ramps on behalf of fbcon */
static void lx_crtc_fb_gamma_get(struct drm_crtc *crtc, u16 *red, u16 *green,
				 u16 *blue, int regno)
{
	struct lx_crtc *lx_crtc = to_lx_crtc(crtc);
	struct lx_rgb *lut_entry = lx_crtc->lut + regno;

	*red	= lut_entry->r << 8;
	*green	= lut_entry->g << 8;
	*blue   = lut_entry->b << 8;
}

static int lx_crtc_gamma_set(struct drm_crtc *crtc, u16 *r, u16 *g, u16 *b,
			     uint32_t size, struct drm_modeset_acquire_ctx *ctx)
{
	struct lx_crtc *lx_crtc = to_lx_crtc(crtc);
	struct lx_rgb *lut_entry;
	uint32_t start = 0;
	uint32_t i, end;

	end = (start + size > LX_LUT_SIZE) ? LX_LUT_SIZE : (start + size);

	for (i = start, lut_entry = lx_crtc->lut; i < end; i++, lut_entry++) {
		DRM_DEBUG_DRIVER("%02x: %04hx %04hx %04hx\n", i, r[i], g[i], b[i]);
		lut_entry->r = r[i] >> 8;
		lut_entry->g = g[i] >> 8;
		lut_entry->b = b[i] >> 8;
	}

	lx_crtc_load_lut(crtc);

	return 0;
}

static int lx_crtc_cursor_set(struct drm_crtc *crtc, struct drm_file *file_priv,
			      uint32_t handle, uint32_t width, uint32_t height)
{
	struct lx_priv *priv = file_priv->driver_priv;
	struct lx_crtc *lx_crtc = to_lx_crtc(crtc);
	struct lx_bo *bo;
	unsigned hw_w, hw_h, hw_p; /* width, height, pitch needed by the hw */
	u32 gcfg;

	if (lx_crtc->id != LX_CRTC_GRAPHIC) {
		/* That's technically not correct, the VP is also able to
		 * provide a cursor image overlayed onto the screen via color
		 * keying; though only 2 colors are supported and it's a
		 * quite some work to get it going, so ... */
		DRM_ERROR("cursor is only supported on the graphics crtc\n");
		/* Maybe later via driver-private IOCTLs
		 * (TODO: does X support color-keyed cursors overlayed on
		 *  video?) */
		return -EINVAL;
	}

	if (!handle) {
		/* disable cursor */
		gcfg = read_dc(priv, DC_GENERAL_CFG);
		gcfg &= ~DC_GENERAL_CFG_CURE; /* disable hardware cursor */

		lx_crtc->cursor_enabled = false;

		dc_unlock(priv);
		write_dc(priv, DC_GENERAL_CFG, gcfg);
		dc_lock(priv);
		return 0;
	}

	bo = idr_find(&file_priv->object_idr, handle);
	if (!bo) {
		DRM_ERROR("handle %d does not exist\n", handle);
		return -ENOENT;
	}
	if (bo->width != width || bo->height != height) {
		DRM_ERROR("bo-dimensions (%ux%u) are inconsistent with "
			  "requested cursor size (%ux%u)\n",
			  bo->width, bo->height, width, height);
		return -EINVAL;
	}

	gcfg = read_dc(priv, DC_GENERAL_CFG);
	gcfg |= DC_GENERAL_CFG_CURE; /* enable hardware cursor */

	switch (bo->bpp) {
	case 2: /* monochrome, 16px AND mask, 16px XOR mask */
		hw_w = 64;
		gcfg &= ~DC_GENERAL_CFG_CLR_CUR;
		break;
	case 32: /* color, ARGB */
		hw_w = 48;
		gcfg |= DC_GENERAL_CFG_CLR_CUR;
		break;
	default:
		DRM_ERROR("cursor bpp unsupported: %u\n", bo->bpp);
		return -EINVAL;
	}
	hw_h = 64;
	hw_p = hw_w * bo->bpp / 8;
	if (width != hw_w || height != hw_h || bo->pitch != hw_p) {
		DRM_ERROR("%u bpp cursor dimension must be %ux%u with a pitch "
			  "of %u bytes (got %ux%u, pitch: %u)\n",
			  bo->bpp, hw_w, hw_h, hw_p,
			  width, height, bo->pitch);
		return -EINVAL;
	}

	/* bo->node->start must always be 32 byte aligned */
	BUG_ON(bo->node->start & 0x1f);

	lx_crtc->cursor_offset = bo->node->start;
	lx_crtc->cursor_enabled = true;
	lx_crtc->cursor_color = !!(gcfg & DC_GENERAL_CFG_CLR_CUR);

	dc_unlock(priv);
	write_dc(priv, DC_CURS_ST_OFFSET, lx_crtc->cursor_offset);
	write_dc(priv, DC_GENERAL_CFG, gcfg);
	dc_lock(priv);

	return 0;
}

static int lx_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	struct lx_priv *priv = crtc->dev->dev_private;
	struct lx_crtc *lx_crtc = to_lx_crtc(crtc);
	unsigned height = 64;
	unsigned width = lx_crtc->cursor_color ? 48 : 64;
	unsigned pitch = width * (lx_crtc->cursor_color ? 32 : 2) / 8;

	if (lx_crtc->id != LX_CRTC_GRAPHIC) {
		DRM_ERROR("cursor is only supported on the graphics crtc\n");
		return -EINVAL;
	}
	/* a disabled cursor means we don't know whether the values we set here
	 * are actually valid for a newly set cursor as the width / strides as
	 * well as the offset to the cursor image of course may differ from the
	 * last known cursor image (2bpp vs. 32bpp) */
	if (!lx_crtc->cursor_enabled)
		DRM_DEBUG_DRIVER("WARNING: moving disabled cursor\n");

	dc_unlock(priv);
	{
		union {
			struct {
				u32 coord  : 11;
				u32 offset : 6;
				u32 pad    : 15;
			} c;
			u32 v;
		} rx = { {
			.offset = x < 0 ? min_t(u32, -x, width - 1) : 0,
			.coord  = x < 0 ? 0 : x,
		} },
		  ry = { {
			.offset = y < 0 ? min_t(u32, -y, height - 1) : 0,
			.coord  = y < 0 ? 0 : y,
		} };

		/* the cursor offset register must always point to the first
		 * line to be drawn on the screen */
		u32 off = lx_crtc->cursor_offset + pitch * ry.c.offset;

		write_dc(priv, DC_CURS_ST_OFFSET, off);

		write_dc(priv, DC_CURSOR_X, rx.v);
		write_dc(priv, DC_CURSOR_Y, ry.v);
	}
	dc_lock(priv);

	return 0;
}

static void lx_crtc_reset(struct drm_crtc *crtc)
{
	struct lx_priv *priv = crtc->dev->dev_private;
	u32 genlk, filt_ctl, lo, hi;

	DRM_DEBUG_DRIVER("\n");
	return;

	/* Clear the various buffers */

	dc_unlock(priv);

	write_dc(priv, DC_FB_ST_OFFSET, 0);
	write_dc(priv, DC_CB_ST_OFFSET, 0);
	write_dc(priv, DC_CURS_ST_OFFSET, 0);

	genlk = read_dc(priv, DC_GENLK_CTL);
	genlk &= ~(DC_GENLK_CTL_ALPHA_FLICK_EN |
		   DC_GENLK_CTL_FLICK_EN |
		   DC_GENLK_CTL_FLICK_SEL_MASK);

	filt_ctl = read_dc(priv, DC_IRQ_FILT_CTL);
	filt_ctl &= ~(DC_IRQ_FILT_CTL_INTERLACE_ADDR |
		      DC_IRQ_FILT_CTL_ALPHA_FILT_ENA |
		      DC_IRQ_FILT_CTL_FILT_ENA |
		      DC_IRQ_FILT_CTL_INTL_EN);

	/* Default scaling params */

	write_dc(priv, DC_GFX_SCALE, (0x4000 << 16) | 0x4000);
	write_dc(priv, DC_IRQ_FILT_CTL, filt_ctl);
	write_dc(priv, DC_GENLK_CTL, genlk);

	/* Dirty / Valid RAM */
	
	write_dc(priv, DC_DV_TOP, 0);
	/* disable RAM snooping */
#if 0 /* don't overwrite offset as set by the BIOS */
	write_dc(priv, DC_DV_CTL, DC_DV_CTL_DV_MASK | DC_DV_CTL_CLEAR_DV_RAM);
#endif

	dc_lock(priv);

	/* set default watermark values */

	rdmsr(MSR_DC_SPARE_MSR, lo, hi);

	lo |= MSR_DC_SPARE_MSR_DIS_VIFO_WM;

	wrmsr(MSR_DC_SPARE_MSR, lo, hi);
}

/* called by drm framework for the page-flip ioctl */
static int lx_crtc_page_flip(struct drm_crtc *crtc, struct drm_framebuffer *fb,
			     struct drm_pending_vblank_event *event,
			     uint32_t _flags, struct drm_modeset_acquire_ctx *ctx)
{
	struct lx_priv *priv = crtc->dev->dev_private;
	struct lx_crtc *lx_crtc = to_lx_crtc(crtc);
	struct lx_fb *lfb = to_lx_fb(fb);
	struct lx_fb *lfb2 = to_lx_fb(crtc->primary->fb);
	unsigned long flags;
	unsigned offset;
	int ret;

	DRM_DEBUG_DRIVER("pipe: %d, old fb: %p, bo @ %lx, size: %lu; new fb: %p, bo @ %lx, size: %lu\n",
			 event->pipe, crtc->primary->fb,
			 crtc->primary->fb && lfb2->bo ? (unsigned long)(lfb2->bo->node->start + priv->vmem_addr) : 0UL,
			 crtc->primary->fb && lfb2->bo ? (unsigned long)(lfb2->bo->node->size) : 0UL,
			 fb,
			 lfb->bo ? (unsigned long)(lfb->bo->node->start + priv->vmem_addr) : 0UL,
			 lfb->bo ? (unsigned long)(lfb->bo->node->size) : 0UL);

	offset = (lfb->bo) ? lfb->bo->node->start : 0;
	offset += crtc->y * fb->pitches[0] + crtc->x * ALIGN(drm_format_info_bpp(fb->format, 0), 8) / 8;

	/* Abuse the event_lock for protecting the event */
	spin_lock_irqsave(&priv->ddev->event_lock, flags);

	/* Check whether a flip has already been scheduled by user-space.
	 * If so, leave. */
	if (lx_crtc->flip_scheduled) {
		/* Another flip is still pending */
		spin_unlock_irqrestore(&priv->ddev->event_lock, flags);
		return -EBUSY;
	}
	lx_crtc->event = event;
	lx_crtc->flip_scheduled = true;

	dc_unlock(priv);
	/* The value programmed takes effect at the next frame scan */
	write_dc(priv, DC_FB_ST_OFFSET, offset);
	dc_lock(priv);
	spin_unlock_irqrestore(&priv->ddev->event_lock, flags);

	/* TODO: call drm_mode_set_config_internal() instead? */
	crtc->primary->fb = fb;

	ret = drm_crtc_vblank_get(crtc);
	if (ret) {
		DRM_ERROR("failed to get vblank event before page flip: %d\n", ret);
		lx_crtc->event = NULL;
	}

	return ret;
}

/* called from our IRQ handler to deliver a pending page-flip event */
static void lx_crtc_do_page_flip(struct drm_crtc *crtc)
{
	struct lx_crtc *lx_crtc = to_lx_crtc(crtc);
	struct lx_priv *priv = crtc->dev->dev_private;
	struct drm_pending_vblank_event *e;
	unsigned long flags;

	spin_lock_irqsave(&priv->ddev->event_lock, flags);

	if (!lx_crtc->flip_scheduled) {
		spin_unlock_irqrestore(&priv->ddev->event_lock, flags);
		return;
	}

	e = lx_crtc->event;
	lx_crtc->event = NULL;
	lx_crtc->flip_scheduled = false;
	if (e) {
		struct timespec64 ts = ktime_to_timespec64(priv->last_vblank);
		/* wakeup userspace */
		e->event.vbl.sequence = drm_crtc_vblank_count(crtc);
		e->event.vbl.tv_sec = ts.tv_sec;
		e->event.vbl.tv_usec = ts.tv_nsec / 1000;
		list_add_tail(&e->base.link, &e->base.file_priv->event_list);
		wake_up_interruptible(&e->base.file_priv->event_wait);
	}

	spin_unlock_irqrestore(&priv->ddev->event_lock, flags);

	drm_crtc_vblank_put(crtc);
}


/**
 * Gets the current scanline value from the scanout logic.
 *
 * @return negative when the value is currently transitioning due to being
 *         unbuffered, a valid vpos otherwise
 */
static int lx_get_scanout_vpos(struct lx_priv *priv)
{
	u32 v = read_dc(priv, DC_LINE_CNT);
	u32 w = read_dc(priv, DC_LINE_CNT);
	return (v == w) ? v & DC_LINE_CNT_DOT_LINE_CNT_MASK : -EAGAIN;
}

/**
 * @get_vblank_timestamp:
 *
 * Called by drm_get_last_vbltimestamp(). Should return a precise
 * timestamp when the most recent vblank interval ended or will end.
 *
 * Specifically, the timestamp in @vblank_time should correspond as
 * closely as possible to the time when the first video scanline of
 * the video frame after the end of vblank will start scanning out,
 * the time immediately after end of the vblank interval. If the
 * @crtc is currently inside vblank, this will be a time in the future.
 * If the @crtc is currently scanning out a frame, this will be the
 * past start time of the current scanout. This is meant to adhere
 * to the OpenML OML_sync_control extension specification.
 *
 * Parameters:
 *
 * crtc:
 *     CRTC for which timestamp should be returned.
 * max_error:
 *     Maximum allowable timestamp error in nanoseconds.
 *     Implementation should strive to provide timestamp
 *     with an error of at most max_error nanoseconds.
 *     Returns true upper bound on error for timestamp.
 * vblank_time:
 *     Target location for returned vblank timestamp.
 * in_vblank_irq:
 *     True when called from drm_crtc_handle_vblank().  Some drivers
 *     need to apply some workarounds for gpu-specific vblank irq quirks
 *     if flag is set.
 *
 * Returns:
 *
 * True on success, false on failure, which means the core should
 * fallback to a simple timestamp taken in drm_crtc_handle_vblank().
 */
static bool lx_crtc_get_vblank_timestamp(struct drm_crtc *crtc,
					 int *max_error, ktime_t *vblank_time,
					 bool in_vblank_irq)
{
	struct drm_device *dev = crtc->dev;
	struct lx_crtc *lcrtc = to_lx_crtc(crtc);
	struct lx_priv *priv = dev->dev_private;

	if (lcrtc->id != LX_CRTC_GRAPHIC)
		return false;

	if (in_vblank_irq) {
		/* the correct timestamp has been saved by our irq-handler */
		*vblank_time = priv->last_vblank;
		return true;
	}

	return false;
}

/**
 * enable_vblank - enable vblank interrupt events
 * @dev: DRM device
 * @crtc: which irq to enable
 *
 * Enable vblank interrupts for @crtc.  If the device doesn't have
 * a hardware vblank counter, this routine should be a no-op, since
 * interrupts will have to stay on to keep the count accurate.
 *
 * This is deprecated and should not be used by new drivers.
 * Use drm_crtc_funcs.enable_vblank instead.
 *
 * @return  zero on success, appropriate errno if the given @crtc's vblank
 *          interrupt cannot be enabled.
 */
static int lx_driver_enable_vblank(struct drm_device *dev, unsigned int pipe)
{
	return 0;
}

/**
 * disable_vblank - disable vblank interrupt events
 * @dev: DRM device
 * @crtc: which irq to enable
 *
 * This is deprecated and should not be used by new drivers.
 * Use drm_crtc_funcs.disable_vblank instead.
 *
 * Disable vblank interrupts for @crtc.  If the device doesn't have
 * a hardware vblank counter, this routine should be a no-op, since
 * interrupts will have to stay on to keep the count accurate.
 */
static void lx_driver_disable_vblank(struct drm_device *dev, unsigned int pipe)
{
}

static const struct drm_crtc_funcs lx_crtc_funcs = {
	.set_config		= drm_crtc_helper_set_config,
	.destroy		= drm_crtc_cleanup,
	/*
	 * Flip to the given framebuffer.  This implements the page
	 * flip ioctl described in drm_mode.h, specifically, the
	 * implementation must return immediately and block all
	 * rendering to the current fb until the flip has completed.
	 * If userspace set the event flag in the ioctl, the event
	 * argument will point to an event to send back when the flip
	 * completes, otherwise it will be NULL.
	 */
	.page_flip		= lx_crtc_page_flip,
	.get_vblank_timestamp	= lx_crtc_get_vblank_timestamp,
#if 0 /* This API went away since v3.0. Where to? */
	.gamma_set = lx_crtc_fb_gamma_set,
#else
	.gamma_set = lx_crtc_gamma_set,
#endif	/* TODO: these need testing: */
	.cursor_set = lx_crtc_cursor_set,
	.cursor_move = lx_crtc_cursor_move,
#if 0
	/* Save CRTC state */
	void (*save)(struct drm_crtc *crtc); /* suspend? */
	/* Restore CRTC state */
	void (*restore)(struct drm_crtc *crtc); /* resume? */
	/* Reset CRTC state */
	void (*reset)(struct drm_crtc *crtc);
#endif
};

static void lx_crtc_dpms(struct drm_crtc *crtc, int mode) {
	struct lx_priv *priv = crtc->dev->dev_private;
	u32 gcfg = read_dc(priv, DC_GENERAL_CFG);
	u32 dcfg = read_dc(priv, DC_DISPLAY_CFG);

	DRM_DEBUG_DRIVER("%s\n", dpms_mode_names[mode]);
	/* TODO: enable PD bit at GLPC_DOTPLL[14] to pwr down the dotpll */

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		gcfg |= DC_GENERAL_CFG_DFLE;
		dcfg |= DC_DISPLAY_CFG_GDEN | DC_DISPLAY_CFG_VDEN;
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		gcfg &= ~DC_GENERAL_CFG_DFLE;
		dcfg &= ~(DC_DISPLAY_CFG_GDEN | DC_DISPLAY_CFG_VDEN);
		break;
	}

	dc_unlock(priv);
	write_dc(priv, DC_GENERAL_CFG, gcfg);
	write_dc(priv, DC_DISPLAY_CFG, dcfg);
	dc_lock(priv);
}

static bool lx_crtc_mode_fixup(struct drm_crtc *crtc,
			       const struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	int ret;
	u32 pllval;

	drm_mode_set_crtcinfo(adjusted_mode, 0);

	ret = lx_graphic_mode_valid(adjusted_mode, &adjusted_mode->clock,
				    &pllval);
	DRM_DEBUG_DRIVER("ret: %d, clock: %d, pllval: %04x", ret,
			 adjusted_mode->clock, pllval);

	return (ret == MODE_OK);
}

static u32 lx_set_dotpll(u32 pllval) {
	u32 pll_lo, pll_hi;
	int i;
	u32 real_freq = 48000 * (1 + ((pllval >> 4) & 0xff)) /
			((1 + ((pllval >> 12) & 0x7)) * (1 + (pllval & 0xf))) /
			((pllval & MSR_GLCP_DOTPLL_HI_DIV4) ? 4 : 1);

	DRM_DEBUG_DRIVER("setting dot-pll to %04x (%u kHz)\n", pllval, real_freq);

	rdmsr(MSR_GLCP_DOTPLL, pll_lo, pll_hi);
/*
	if ((pll_lo & MSR_GLCP_DOTPLL_LO_LOCK) && pll_hi == pllval)
		return 0;
*/
	/* the DOTPLL register settings need to be changed with DOTRESET active */
	pll_lo |= MSR_GLCP_DOTPLL_LO_DOTRESET;
	pll_lo &= ~(MSR_GLCP_DOTPLL_LO_BYPASS | MSR_GLCP_DOTPLL_LO_HALFPIX);
	pll_hi = pllval;

	wrmsr(MSR_GLCP_DOTPLL, pll_lo, pll_hi);

	/* wait until PLL locks or a timeout occurs */
	udelay(100); /* max. value for dot-PLL to lock according to data book */
	for (i=0; i<1000; i++) {
		rdmsr(MSR_GLCP_DOTPLL, pll_lo, pll_hi);
		if (pll_lo & MSR_GLCP_DOTPLL_LO_LOCK)
			break;
	}

	DRM_DEBUG_DRIVER("PLL locked: %d, %08x_%08x\n",
		!!(pll_lo & MSR_GLCP_DOTPLL_LO_LOCK), pll_hi, pll_lo);

	/* clear DOTRESET */
	pll_lo &= ~MSR_GLCP_DOTPLL_LO_DOTRESET;
	wrmsr(MSR_GLCP_DOTPLL, pll_lo, pll_hi);

	return (pll_lo & MSR_GLCP_DOTPLL_LO_LOCK) ? real_freq : 0;
}

static bool lx_compression_enabled(struct lx_priv *priv) {
	return false && priv->pan_x == 0 && priv->pan_y == 0;
}

static bool lx_video_overlay_enabled(struct drm_crtc *crtc) {
	return false;
}

static int lx_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
				 struct drm_framebuffer *old_fb)
{
	struct lx_priv *priv = crtc->dev->dev_private;
	struct drm_framebuffer *fb = crtc->primary->fb;
	struct lx_fb *lfb;
	struct lx_crtc *lx_crtc = to_lx_crtc(crtc);
	int offset, atomic = 0;
	u32 gcfg, dcfg, dvctl;
	u32 gfx_pitch, fb_width, fb_height, line_sz;
	unsigned bpp;

	DRM_DEBUG_DRIVER("x: %d, y: %d, new fb: %p, old fb: %p\n",
			 x, y, fb, old_fb);

	priv->pan_x = x;
	priv->pan_y = y;

	/* see kernel git commit ffbc559b0699891c6deb9fd2b4750671eab94999:
	 *     drm/nv50/crtc: Bail out if FB is not bound to crtc
	 * shutdown case: e.g. monitor unplugged */
	if (!atomic && !fb) {
		DRM_DEBUG_DRIVER("no fb bound to crtc\n");
		return 0;
	}

	lfb = to_lx_fb(fb);
	BUG_ON(!lfb->bo);

	dc_unlock(priv);

	/* setup hw to use the new fb crtc->fb with scanout starting at (FB-)
	 * coords (x,y) using crtc->fb->pitch. */
	/* release old fb (if it is different from the new) and possibly clear
	 * the object if it is no longer in use */

	gcfg = DC_GENERAL_CFG_DFLE; /* enable display FIFO */
	gcfg |= DISP_PRIO_HIGH_END << DC_GENERAL_CFG_DFHPEL_SHIFT;
	gcfg |= DISP_PRIO_HIGH_START << DC_GENERAL_CFG_DFHPSL_SHIFT;

	gcfg &= ~DC_GENERAL_CFG_FDTY;
	gcfg &= ~DC_GENERAL_CFG_STFM;
	switch (fb->pitches[0]) {
	case 1024:
	case 2048:
	case 4096:
		// TODO: break;
	default:
		gcfg |= DC_GENERAL_CFG_FDTY;
		break;
	}

	if (lx_compression_enabled(priv)) {
		gcfg |= DC_GENERAL_CFG_DECE;
		gcfg |= DC_GENERAL_CFG_CMPE;
	}

	if (lx_video_overlay_enabled(crtc))
		gcfg |= DC_GENERAL_CFG_VIDE;

	/* hardware cursor settings */
	if (!lx_crtc->cursor_enabled)
		gcfg &= ~DC_GENERAL_CFG_CURE;
	else
		gcfg |= DC_GENERAL_CFG_CURE;
	if (!lx_crtc->cursor_color)
		gcfg &= ~DC_GENERAL_CFG_CLR_CUR;
	else
		gcfg |= DC_GENERAL_CFG_CLR_CUR;

	write_dc(priv, DC_GENERAL_CFG, gcfg);

	dcfg = 0;
	dcfg |= DC_DISPLAY_CFG_DCEN; /* center display on flat panels */

	if (lx_video_overlay_enabled(crtc)) {
		dcfg |= VID_OVL_PRIO_HIGH_END << DC_DISPLAY_CFG_VFHPEL_SHIFT;
		dcfg |= VID_OVL_PRIO_HIGH_START << DC_DISPLAY_CFG_VFHPSL_SHIFT;
	}

	bpp = drm_format_info_bpp(fb->format, 0);
	switch (bpp) {
	case 8:
		dcfg |= DC_DISPLAY_CFG_DISP_MODE_8BPP;
		break;
	case 16:
		dcfg |= DC_DISPLAY_CFG_DISP_MODE_16BPP;
		if (fb->format->depth == 15) {
			dcfg |= DC_DISPLAY_CFG_16BPP_0555;
		} else {
			dcfg |= DC_DISPLAY_CFG_16BPP_0565;
		}
		break;
	case 24:
		dcfg |= DC_DISPLAY_CFG_DISP_MODE_24BPP;
		break;
	case 32:
		dcfg |= DC_DISPLAY_CFG_DISP_MODE_32BPP;
		break;
	}

	if (lx_video_overlay_enabled(crtc)) /* enable video data pass-through */
		dcfg |= DC_DISPLAY_CFG_VDEN;

	dcfg &= ~DC_DISPLAY_CFG_PALB; /* no palette bypass in > 8bpp modes */
	dcfg |= DC_DISPLAY_CFG_GDEN; /* enable graphics data pass-through */
	dcfg |= DC_DISPLAY_CFG_TGEN; /* enable timing generator */
	dcfg |= DC_DISPLAY_CFG_TRUP; /* update timings immediatly */
	dcfg |= DC_DISPLAY_CFG_VISL; /* TODO: ?? */

	write_dc(priv, DC_DISPLAY_CFG, dcfg);

	write_dc(priv, DC_ARB_CFG, 0);

	/* bits [2:0] are added to the pixel panning offset after next vsync */
	/* If this value is != 0, framebuffer compression is unavailable */
	offset = lfb->bo->node->start;
	offset += y * fb->pitches[0] + x * ALIGN(bpp, 8) / 8;
	write_dc(priv, DC_FB_ST_OFFSET, offset);

	/* TODO: write compressed buffer offset */
	/* TODO: write video (Y|U|V) buffers' offsets */
	/* TODO: possibly enable & write DV_TOP_ADDR */

	write_dc(priv, DC_DV_TOP, 0); /* TODO: DV-RAM disabled */

	/* #qwords to transfer per scanline */
	line_sz = ALIGN(fb->width * ALIGN(bpp, 8) / 8, 8) / 8;
	BUG_ON(line_sz >= 1024);
	write_dc(priv, DC_LINE_SIZE, /* in qwords */
		 0 << 20 /* TODO: video, align 4 */ |
		 0 << 10 /* TODO: compressed buffer, val+1, val <= 65 */ |
		 line_sz << 0); /* frame buffer */

	/* pitch in qwords */
	gfx_pitch = fb->pitches[0] / 8;
	write_dc(priv, DC_GFX_PITCH, 0 << 16 /* TODO */ | gfx_pitch << 0);

	/* TODO: DC_VID_YUV_PITCH */
	/* TODO: scaling, see DC_FB_ACTIVE */

	fb_width = (fb->width - 1) | 7;
	fb_height = fb->height - 1;
	write_dc(priv, DC_FB_ACTIVE, fb_width << 16 | fb_height << 0);

	DRM_DEBUG_DRIVER("line_sz: %u qwords, gfx_pitch: %u qwords, fb_width: "
			 "px 0 to %u, fb_height: line 0 to %u, fb->width: %u, "
			 "fb->height: %u, fb->pitch: %u, fb->bpp: %u\n",
			 line_sz, gfx_pitch, fb_width, fb_height, fb->width,
			 fb->height, fb->pitches[0], bpp);

	/* TODO: video downscaling */

	/* DV-RAM Control */

	/* initializing this value is strictly speaking unnecessary due to the
	 * BIOS that should already have initialized it, but it serves as a
	 * reminder that:
	 * a) DV snoops the memory controller, not the DC where it's located,
	 *    so the real physical memory addresses are needed, not the linear
	 *    ones we use everywhere else
	 * b) another address may be entered here, to snoop on any other memory
	 *    range */
	/* TODO: module param to disable dv-ram */
	if (~priv->vmem_phys) {
		/* got the real physical address, so let's write it and don't
		 * trust the BIOS */
		dvctl = ALIGN(priv->vmem_phys, 4096);
	} else {
		/* no phys addess, either trust the BIOS or have no DV-RAM */
		dvctl = read_dc(priv, DC_DV_CTL) & DC_DV_CTL_DV_OFF_MASK;
	}

	if (line_sz < 128)
		dvctl |= DC_DV_CTL_DV_LINE_SIZE_1K;
	else if (line_sz < 256)
		dvctl |= DC_DV_CTL_DV_LINE_SIZE_2K;
	else if (line_sz < 512)
		dvctl |= DC_DV_CTL_DV_LINE_SIZE_4K;
	else /* line_sz < 1024 */
		dvctl |= DC_DV_CTL_DV_LINE_SIZE_8K;

	if (fb->height <= 512)
		dvctl |= DC_DV_CTL_DV_RANGE_512;
	else if (fb->height <= 1024)
		dvctl |= DC_DV_CTL_DV_RANGE_1024;
	else if (fb->height <= 1536)
		dvctl |= DC_DV_CTL_DV_RANGE_1536;
	else if (fb->height <= 2048)
		dvctl |= DC_DV_CTL_DV_RANGE_2048;

	if (fb->width < 8192 && fb->height <= 2048 && offset == 0 &&
	    lx_compression_enabled(priv)) {
		/* mark DV-RAM as dirty & invalid */
		dvctl |= DC_DV_CTL_CLEAR_DV_RAM;
	} else {
		/* disable DV-RAM */
		dvctl |= DC_DV_CTL_DV_MASK;
	}

	write_dc(priv, DC_DV_CTL, dvctl);

	/* TODO: DC_GFX_SCALE */

	dc_lock(priv);

	DRM_DEBUG_DRIVER("\n");
	return 0;
}

/* @crtc: CRTC to program
 * @mode: mode to use
 * @x: width of mode
 * @y: height of mode */
static int lx_crtc_mode_set(struct drm_crtc *crtc,
			    struct drm_display_mode *mode,
			    struct drm_display_mode *m, /* adjusted mode */
			    int x, int y, struct drm_framebuffer *old_fb)
{
	struct lx_priv *priv = crtc->dev->dev_private;
	u32 lo, hi, dcfg, vcfg, real_freq, freq_khz;
	s32 pllval;
	unsigned encoders = lx_crtc_get_enabled_encoders(crtc);

	DRM_DEBUG_DRIVER("\n");
	drm_mode_debug_printmodeline(m);

	/* dot-PLL */
	freq_khz = m->clock;
	pllval = lx_find_matching_pll(&freq_khz, LX_MODE_FREQ_TOL);
	if (pllval < 0)
		return MODE_CLOCK_RANGE;
	real_freq = lx_set_dotpll(pllval & LX_MODE_PFLAG_DOTPLL_MASK);
	if (!real_freq)
		return MODE_CLOCK_RANGE;

	m->clock = real_freq;

	dc_unlock(priv);

	dcfg = read_dc(priv, DC_DISPLAY_CFG);
	/* timing changes are ignored until this bit is set */
	dcfg &= ~DC_DISPLAY_CFG_TRUP;
	write_dc(priv, DC_DISPLAY_CFG, dcfg);

	DRM_DEBUG_DRIVER("timings: htotal: %u, hactive: %u, "
			 "hblank: %u to %u, hsync: %u to %u, "
			 "vtotal: %u, vactive: %u, vblank: %u to %u, "
			 "vsync: %u to %u\n",
			 m->crtc_htotal, m->crtc_hdisplay,
			 m->crtc_hblank_start, m->crtc_hblank_end,
			 m->crtc_hsync_start, m->crtc_hsync_end,
			 m->crtc_vtotal, m->crtc_vdisplay,
			 m->crtc_vblank_start, m->crtc_vblank_end,
			 m->crtc_vsync_start, m->crtc_vsync_end);

	/* timings */
	write_dc(priv, DC_H_ACTIVE_TIMING,
		(m->crtc_htotal     - 1) << 16 | (m->crtc_hdisplay - 1));
	write_dc(priv, DC_H_BLANK_TIMING,
		(m->crtc_hblank_end - 1) << 16 | (m->crtc_hblank_start - 1));
	write_dc(priv, DC_H_SYNC_TIMING,
		(m->crtc_hsync_end  - 1) << 16 | (m->crtc_hsync_start - 1));
	write_dc(priv, DC_V_ACTIVE_TIMING, /* XXX: see datasheet for flat panel */
		(m->crtc_vtotal     - 1) << 16 | (m->crtc_vdisplay - 1));
	write_dc(priv, DC_V_BLANK_TIMING,
		(m->crtc_vblank_end - 1) << 16 | (m->crtc_vblank_start - 1));
	write_dc(priv, DC_V_SYNC_TIMING,
		(m->crtc_vsync_end  - 1) << 16 | (m->crtc_vsync_start - 1));

	/* update working timing regs on next active edge of vsync */
	dcfg |= DC_DISPLAY_CFG_TRUP;
	write_dc(priv, DC_DISPLAY_CFG, dcfg);

	/* TODO: take private flags from mode set scaling values to
	 * DC_GFX_SCALE, DC_IRQ_FILT_CTL and DC_FILT_COEFF(1|2) */

	/* vscale / hscale, format: fixed 2.14 */
	write_dc(priv, DC_GFX_SCALE, 0x4000 << 16 | 0x4000);

	/* trigger line count IRQ when vblank starts (if IRQ is enabled) */
	/* TODO: graphics/flicker/alpha filter, interlacing */
	// write_dc(priv, DC_IRQ_FILT_CTL, 0*m->crtc_vblank_start << 16);

	/* disable VBI */
	write_dc(priv, DC_VBI_EVEN_CTL, 0);
	write_dc(priv, DC_VBI_ODD_CTL, 0);
	write_dc(priv, DC_VBI_LN_ODD, 0);
	write_dc(priv, DC_VBI_LN_EVEN, 0);
	write_dc(priv, DC_VBI_PITCH, 0);

	/* TODO: disable color keying */
	write_dc(priv, DC_CLR_KEY, 0);

	/* disable GenLock & (TODO:) Flicker Filter */
	write_dc(priv, DC_GENLK_CTL, 0);

	/* nothing to do for the VGA emulation block */

	dc_lock(priv);

	/* set output format */
	rdmsr(LX_MSR(LX_VP, LX_GLD_MSR_CONFIG), lo, hi);

	lo &= ~MSR_VP_GLD_MSR_CONFIG_FPC;
	lo &= ~MSR_VP_GLD_MSR_CONFIG_FMT_MASK;

	if (encoders == 1 << LX_ENCODER_DAC)
		lo |= MSR_VP_GLD_MSR_CONFIG_FMT_CRT;
	else if (encoders & (1 << LX_ENCODER_PANEL))
		lo |= MSR_VP_GLD_MSR_CONFIG_FMT_FP;
#if 0
	else if (encoders & (1 << LX_ENCODER_VOP))
		lo |= MSR_VP_GLD_MSR_CONFIG_FMT_VOP;
	else if (encoders == 1 << LX_ENCODER_BYPASS) {
		lo |= MSR_VP_GLD_MSR_CONFIG_FMT_DRGB;
		/* TODO: set format byte order [7:6] based on drm-property */
		/* TODO: set interchange UV [14] based on drm-property */
	}
#endif

	if (encoders & (1 << LX_ENCODER_PANEL /*| 1 << LX_ENCODER_VOP*/))
		lo |= MSR_VP_GLD_MSR_CONFIG_FPC;

	wrmsr(LX_MSR(LX_VP, LX_GLD_MSR_CONFIG), lo, hi);

	/* disable video acceleration hw */
	vcfg = read_vp(priv, VP_VCFG);
	vcfg &= ~VP_VCFG_VID_EN;
	write_vp(priv, VP_VCFG, vcfg);

	/* VP display configuration */
	vcfg = read_vp(priv, VP_DCFG);
	vcfg |= VP_DCFG_GV_GAM; /* select VP's gamma RAM for the video stream */

	if (m->flags & DRM_MODE_FLAG_NHSYNC) {  /* polarity of active edge */
		vcfg |= VP_DCFG_CRT_HSYNC_POL;  /* low */
	} else {
		vcfg &= ~VP_DCFG_CRT_HSYNC_POL; /* high */
	}
	if (m->flags & DRM_MODE_FLAG_NVSYNC) {
		vcfg |= VP_DCFG_CRT_VSYNC_POL;
	} else {
		vcfg &= ~VP_DCFG_CRT_VSYNC_POL;
	}

	vcfg |= VP_DCFG_DAC_BL_EN;
	vcfg |= VP_DCFG_CRT_EN;

	write_vp(priv, VP_DCFG, vcfg);

	return lx_crtc_mode_set_base(crtc, x, y, old_fb);
}

static void lx_crtc_prepare(struct drm_crtc *crtc)
{
	DRM_DEBUG_DRIVER("\n");
	lx_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);

	
}

static void lx_crtc_commit(struct drm_crtc *crtc)
{
	DRM_DEBUG_DRIVER("\n");
	lx_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
}

static void lx_crtc_load_lut(struct drm_crtc *crtc)
{
	struct lx_crtc *lx_crtc = to_lx_crtc(crtc);
	struct lx_priv *priv = crtc->dev->dev_private;
	unsigned i;

	DRM_DEBUG_DRIVER("enabled: %d\n", crtc->enabled);

	if (!crtc->enabled)
		return;

	/* copy values to DC's Palette / Gamma space */
	write_dc(priv, DC_PAL_ADDRESS, 0);

	/* TODO: regard mode_config.depth */

	for (i = 0; i < LX_LUT_SIZE; i++) {
		/* accesses to DC_PAL_DATA (palette data @DC_PAL_ADDRESS)
		 * increase DC_PAL_ADDRESS */
		write_dc(priv, DC_PAL_DATA,
			 lx_crtc->lut[i].r << 16 |
			 lx_crtc->lut[i].g <<  8 |
			 lx_crtc->lut[i].b <<  0);
	}
}

static const struct drm_crtc_helper_funcs lx_crtc_helper_funcs = {
	.dpms			= lx_crtc_dpms, /* if (ON) update_scanout; commit */
	.mode_fixup		= lx_crtc_mode_fixup, /* return 1 */
	.mode_set		= lx_crtc_mode_set, /* update (omap_)crtc->info w/ adjusted_mode && update_scanout */
	.prepare		= lx_crtc_prepare,
	.commit			= lx_crtc_commit,
	.mode_set_base		= lx_crtc_mode_set_base,
#if 0 /* This API went away since v3.0. Where to? */
	.load_lut	= lx_crtc_load_lut,
#endif
#if 0
	.disable, /* more explicit than dpms */
	int (*mode_set_base_atomic)(struct drm_crtc *crtc,
				    struct drm_framebuffer *fb, int x, int y,
				    enum mode_set_atomic);
#endif
};

static void lx_crtc_init(struct drm_device *dev, enum lx_crtcs crtc_id)
{
	struct lx_priv *priv = dev->dev_private;
	struct lx_crtc *lx_crtc = &priv->crtcs[crtc_id];
	struct drm_crtc *crtc = &lx_crtc->base;
	unsigned i;

	/* maybe we've been initialized already ... */
	if (lx_crtc->id < LX_NUM_CRTCS)
		return;
	lx_crtc->id = crtc_id;

	crtc->enabled = 1;
	drm_crtc_init(dev, crtc, &lx_crtc_funcs);

	if (drm_mode_crtc_set_gamma_size(crtc, 256)) {
		struct lx_rgb *en;
		/* init our backing store for gamma values */
		for (i = 0, en = lx_crtc->lut; i < 256; i++, en++) {
			en->r = i;
			en->g = i;
			en->b = i;
		}
		en = lx_crtc->lut + LX_LUT_CURSOR_COL0;
		en->r = en->g = en->b = 0x00;
		en = lx_crtc->lut + LX_LUT_CURSOR_COL1;
		en->r = en->g = en->b = 0xff;
		en = lx_crtc->lut + LX_LUT_ICON_COL0;
		en->r = en->g = en->b = 0x00;
		en = lx_crtc->lut + LX_LUT_ICON_COL1;
		en->r = en->g = en->b = 0xff;
		en = lx_crtc->lut + LX_LUT_ICON_BORDER;
		en->r = en->g = en->b = 0x00;
	} else {
		DRV_INFO("unable to create backing store for gamma values\n");
	}

	drm_crtc_helper_add(crtc, &lx_crtc_helper_funcs);
}

/* ========================================================================== */

static int lx_connector_encoder_init(struct drm_device *dev,
				     enum lx_connectors connector_id)
{
	enum lx_encoders encoder_id = lx_connector_types[connector_id].encoder;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	int ret;

	connector = lx_connector_init(dev, connector_id);

	/* the encoder may already be initialized... */
	encoder = lx_encoder_init(dev, encoder_id);

	encoder->possible_crtcs = 1 << LX_CRTC_GRAPHIC | 1 << LX_CRTC_VIDEO;
	if (encoder_id != LX_ENCODER_DAC)
		encoder->possible_clones = 1 << LX_ENCODER_DAC;

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret) {
		DRM_ERROR("failed to attach encoder %s to %s\n",
			  encoder->name, connector->name);
		goto out;
	}

	return 0;

out:
	encoder->funcs->destroy(encoder);
	connector->funcs->destroy(connector);
	return ret;
}

static void lx_modeset_init(struct drm_device *dev) {
	struct lx_priv *priv = dev->dev_private;
	struct drm_connector *conn_vga = conn_vga;
	int ret, i;

	/* init GPIO DDC i2c_adapter */
	ret = lx_ddc_init(dev);
	if (ret) {
		DRM_INFO("lx: unable to initialize built-in GPIOs for DDC: %d\n",
			 ret);
	}

	drm_mode_config_init(dev);
	dev->mode_config.min_width = 64;
	dev->mode_config.min_height = 8;
	dev->mode_config.max_width = 1920;
	dev->mode_config.max_height = 1440;
	dev->mode_config.funcs = &lx_mode_funcs;
	dev->mode_config.fb_base = priv->vmem_addr;

	/* TODO: create & attach drm_device specific drm props */

	/* init the ids to an invalid number, so singleton initialization may
	 * proceed */
	for (i=0; i<LX_NUM_CONNECTORS; i++)
		priv->connectors[i].id = LX_NUM_CONNECTORS;
	for (i=0; i<LX_NUM_ENCODERS; i++)
		priv->encoders[i].id = LX_NUM_ENCODERS;
	for (i=0; i<LX_NUM_CRTCS; i++)
		priv->crtcs[i].id = LX_NUM_CRTCS;

	lx_crtc_init(dev, LX_CRTC_GRAPHIC);

	/* Init connectors and their corresponding 'best' encoders */
	ret = lx_connector_encoder_init(dev, LX_CONNECTOR_VGA);/*
	lx_connector_init(dev, LX_CONNECTOR_LVDS);
	lx_connector_init(dev, LX_CONNECTOR_VOP);*/

	/* Additional, 'non-best' encoders:
	lx_encoder_init(dev, LX_ENCODER_TVDAC);
	drm_mode_connector_attach_encoder(&priv->connectors[LX_CONNECTOR_VGA],
					  &priv->encoders[LX_ENCODER_TVDAC]);
	*/

	/* TODO: create & attach connector/encoder specific drm props */

	ret = lx_fbdev_init(dev);

	drm_kms_helper_poll_init(dev);
}

static void lx_modeset_cleanup(struct drm_device *dev) {
	lx_fbdev_fini(dev);
	drm_mode_config_cleanup(dev);
	drm_kms_helper_poll_fini(dev);
	/* delete DDC i2c_adapter (if available) */
	lx_ddc_cleanup(dev);
}


/* The following memory regions should be usable (see DC_GLIU0_MEM_OFFSET, all
 * within a single 256 MB sized, 16 MB aligned memory region):
 * uncompressed frame buffer:
 *   - max. size: 2048x2048, 32 bpp (w/ and w/o scaling filter as it only
 *     supports scaling when width <= 1024 and also only supports max. 2:1
 *     downscaling)
 *   -> 16 MB
 *   - alignment: none (via DC_FB_ST_OFFSET)
 * compressed display buffer (optional):
 *   - max. 64+4 qwords per line written to memory = 544 bytes per line
 *   - max. framebuffer height: 2048
 *   - max. vertical downscaling: up to, but excluding 2:1 (w/o filter)
 *     (TODO: does scaling really come into play here?)
 *   -> max. 4095 lines of framebuffer data need to be compressed
 *   -> max. 4095 * 544 = 2227680 bytes of compressed buffer size needed
 *                      = 2176 KB - 544 byte
 *   -> so take 2176 KB for the compressed framebuffer
 * cursor buffer (optional) (union):
 *   -  2bpp: 64x64-2  ->  1 KB
 *   - 32bpp: 64x48-32 -> 12 KB
 * video buffer(s) (union):
 *   - YUV-4:2:0:
 *     - 1920 + 960 + 960
 *   - YUV-4:2:2:
 *     - 1440
 * 
 * others (addressing w/o GLIU0_MEM_OFFSET):
 * command buffer:
 *   - max. size: 16 MB
 *   - alignment: 1 MB (via GP's GLD_MSR_CONFIG)
 */

#if 0
static int lx_alloc_defaults(struct lx_priv *priv, unsigned cmd_buf_size) {
	struct drm_mm_node *fb;
	struct drm_mm_node *cfb;
	struct drm_mm_node *cursor;
	struct drm_mm_node *cmd_buf;

	/* framebuffer: 16 MB max. */
	fb = lx_mm_alloc(priv, 16 << 20, 4096, 0);
	if (!fb)
		return -ENOMEM;

	/* compressed framebuffer, 2176 KB */
	cfb = lx_mm_alloc(priv, 4096 * (64 + 4) * 8, 32, 0);
	if (!cfb) {
		DRV_INFO("unable to find 2176 KB of free vmem, disabling "
			 "framebuffer compression\n");
	}

	/* cursor, 12 KB */
	cursor = lx_mm_alloc(priv, 12 << 10, 32, 0);

	/* GP command buffer */
	cmd_buf = lx_mm_alloc(priv, cmd_buf_size, 1 << 20, 0);

	return 0;
}

enum lx_video_format {
	LX_VIDEO_OFF,
	LX_VIDEO_420,
	LX_VIDEO_422
};

static int lx_video_reinit(struct lx_priv *priv, enum lx_video_format vfmt) {
	struct drm_mm_node *y, *u, *v;

	if (priv->video_fmt == vfmt)
		return;

	drm_mm_put_block(priv->buf_vid_y);
	drm_mm_put_block(priv->buf_vid_u);
	drm_mm_put_block(priv->buf_vid_v);

	switch (vfmt) {
	case LX_VIDEO_OFF:
		y = NULL;
		u = NULL;
		v = NULL;
		break;
	case LX_VIDEO_420:
		y = lx_mm_alloc(priv, 1920, 32, 0);
		if (!y)
			goto err;
		u = lx_mm_alloc(priv, 960, 8, 0);
		if (!u)
			goto err;
		v = lx_mm_alloc(priv, 960, 8, 0);
		if (!v)
			goto err;
		break;
	case LX_VIDEO_422:
		y = lx_mm_alloc(priv, 1440, 32, 0);
		if (!y)
			goto err;
		u = NULL;
		v = NULL;
		break;
	}

	dc_unlock(priv);

	write_dc(priv, DC_VID_Y_ST_OFFSET, y->start);
	if (u)
		write_dc(priv, DC_VID_U_ST_OFFSET, u->start);
	if (v)
		write_dc(priv, DC_VID_V_ST_OFFSET, v->start);

	dc_lock(priv);

	priv->video_fmt = vfmt;

	priv->buf_vid_y = y;
	priv->buf_vid_u = u;
	priv->buf_vid_v = v;

	return 0;

err:
	if (v)
		drm_mm_put_block(v);
	if (u)
		drm_mm_put_block(u);
	if (y)
		drm_mm_put_block(y);
	return -ENOMEM;
}
#endif

/**
 * Determines the size of the stolen memory region in system memory and -
 * optionally - it's physical location (on the LX).
 */
/* originally from lxfb_ops.c, modified */
static bool lx_determine_vmem(resource_size_t *vmem_size,
			      resource_size_t *vmem_phys)
{
	unsigned int val;
#if 0
	if (cs5535_has_vsa2()) {
		/* The frame buffer size is reported by a VSM in VSA II */
		/* Virtual Register Class    = 0x02                     */
		/* VG_MEM_SIZE (1MB units)   = 0x00                     */

		outw(VSA_VR_UNLOCK, VSA_VRC_INDEX);
		outw(VSA_VR_MEM_SIZE, VSA_VRC_INDEX);

		val = (unsigned int)(inw(VSA_VRC_DATA)) & 0xFE;

		*vmem_size = (val << 20);
	} else
#endif
	{
		unsigned msr;
		uint32_t n_p2d_ro, hi, lo;
		uint32_t pdid, pmin, pmax, poffset;

		/* find number of P2D RO registers programmed */
		rdmsr(MSR_GLIU0_PHY_CAP, lo, hi);
		n_p2d_ro = (lo & 0x00fc0000) >> 18;

		/* iterate through all valid P2D RO registers that remap an
		 * arbitrary (page-grained) range of memory accesses by adding
		 * (in 32 bit sense, so w/ overflow) poffset to it */
		for (msr = 0; msr < n_p2d_ro; msr++) {
			rdmsr(MSR_GLIU0_P2D_RO_0 + msr, lo, hi);
			pdid = hi >> 29;
			/* got a range remapping to the memory controller, this
			 * just has to be our framebuffer */
			if (pdid == LX_GLIU0_DID_GLMC)
				break;
		}
		if (msr == n_p2d_ro)
			return false;

		poffset = (hi & 0x0fffff00) >> 8;
		pmax = ((hi & 0xff) << 12) | ((lo & 0xfff00000) >> 20);
		pmin = (lo & 0x000fffff);

		val = (pmax - pmin) + 1;

		/* The number of pages is (PMAX - PMIN)+1 */
		/* The page size is 4k */
		*vmem_size = (val << 12);
		/* address all memory accesses between pmin and pmax (incl) get
		 * remapped to */
		*vmem_phys = ((pmin + poffset) << 12) & 0xffffffff;
	}

	return true;
}

static int lx_map_video_memory(struct drm_device *dev)
{
	static const char *region_names[] = {
		"lx-vram", /* shared video memory set aside by the bios */
		"lx-gp", /* graphics processor MSRs */
		"lx-dc", /* display controller MSRs */
		"lx-vp", /* video processor MSRs */
		/* the unused BAR 4 points to MSRs for the video input port */
	};
	char tmp[24];
	struct lx_priv *priv = dev->dev_private;
	struct pci_dev *pdev = priv->pdev;
	unsigned i;
	int ret;

	for (i=0; i<ARRAY_SIZE(region_names); i++) {
		ret = pci_request_region(pdev, i, region_names[i]);
		if (ret) {
			DRM_ERROR("failed requesting PCI region %d: %d\n", i, ret);
			goto failed_req;
		}
	}

	priv->vmem_addr = pci_resource_start(pdev, 0); /* I/O linear address */
	/* this space really lies in the RAM, the mapping is done via
	 * MSR_GLIU_P2D_RO0 */
	/* pci_resource_len(pdev, 0) is wrong. only half this amount seems to
	 * reside in physical RAM */
	priv->vmem_phys = ~(resource_size_t)0; /* uninitialized, not aligned */
	if (!lx_determine_vmem(&priv->vmem_size, &priv->vmem_phys)) {
		DRM_ERROR("failed to determining size (and physical location) "
			  "of video memory\n");
		/* TODO: provide vmem module param
		 * TODO: continue startup, just w/o DV-RAM and mandatory vmem arg */
		ret = -ENOMEM; /* really _no_ memory :) */
		goto failed_mm;
	}

	/* todo: mark this region as write-combined (uncacheable)
	 * (see LX Processor Data Book, Table 5-17, p. 170) */

	/* stolen memory, add priv->vmem_phys to all blocks from this mm to get
	 * the linear address
	 * (mapped by the GLIU0 to the end of the physical RAM) */
	drm_mm_init(&priv->mman.mm, 0, priv->vmem_size);

	ret = drm_legacy_addmap(dev,
		   pci_resource_start(pdev, 1), pci_resource_len(pdev, 1),
		   _DRM_REGISTERS, _DRM_READ_ONLY | _DRM_DRIVER, &priv->gp);
	if (ret)
		goto failed_map_gp;

	ret = drm_legacy_addmap(dev,
		   pci_resource_start(pdev, 2), pci_resource_len(pdev, 2),
		   _DRM_REGISTERS, _DRM_READ_ONLY | _DRM_DRIVER, &priv->dc);
	if (ret)
		goto failed_map_dc;

	ret = drm_legacy_addmap(dev,
		   pci_resource_start(pdev, 3), pci_resource_len(pdev, 3),
		   _DRM_REGISTERS, _DRM_READ_ONLY | _DRM_DRIVER, &priv->vp);
	if (ret)
		goto failed_map_vp;

	/* need to unlock the MSRs for WR */
	dc_unlock(priv);
	/* base address for graphics memory region, alignment: 16MB */
	write_dc(priv, DC_GLIU0_MEM_OFFSET, priv->vmem_addr & 0xFF000000);
	dc_lock(priv);

	if (~priv->vmem_phys)
		snprintf(tmp, sizeof(tmp), "0x%lx",
			 (unsigned long)priv->vmem_phys);
	tmp[sizeof(tmp)-1] = '\0';

	DRV_INFO("%lu KB of video memory at linear address 0x%lx, physical "
	         "address: %s\n",
		 (unsigned long)priv->vmem_size / 1024,
		 (unsigned long)priv->vmem_addr,
		 ~priv->vmem_phys ? tmp : "unknown");

	return 0;

failed_map_vp:
	drm_legacy_rmmap(dev, priv->dc);
failed_map_dc:
	drm_legacy_rmmap(dev, priv->gp);
failed_map_gp:
	drm_mm_takedown(&priv->mman.mm);
failed_mm:
	i = ARRAY_SIZE(region_names);
failed_req:
	while (i--)
		pci_release_region(pdev, i);
	return ret;
}

static void lx_unmap_video_memory(struct drm_device *dev)
{
	struct lx_priv *priv = dev->dev_private;
	struct pci_dev *pdev = priv->pdev;

	drm_legacy_rmmap(dev, priv->vp);
	drm_legacy_rmmap(dev, priv->dc);
	drm_legacy_rmmap(dev, priv->gp);

	drm_mm_takedown(&priv->mman.mm);

	pci_release_region(pdev, 3);
	pci_release_region(pdev, 2);
	pci_release_region(pdev, 1);
	pci_release_region(pdev, 0);
}

#if 0 /* v3.0 API; seems to have been replaced by DRIVER_USE_AGP */
static int lx_driver_device_is_agp(struct drm_device *dev)
{
	return 0;
}
#endif

#ifndef CONFIG_DRM_LEGACY
# error require CONFIG_DRM_LEGACY=y
#endif

/** @flags: set in pci id list, not needed here */
int lx_driver_load(struct drm_device *dev, unsigned long flags)
{
	struct lx_priv *priv;
	u32 lo, hi;
	int ret, i;

	/* TODO: setup performance counters 0 and 1 */

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ddev = dev;
	priv->pdev = to_pci_dev(dev->dev);

	dev->dev_private = priv;

	ret = lx_map_video_memory(dev);
	if (ret) {
		DRV_ERR("failed mapping video memory or controller registers\n");
		goto failed_map_video_memory;
	}

	lx_modeset_init(dev);

	drm_vblank_init(dev, 1 /* TODO: LX_NUM_CRTCS */);
	// v3.0 API: drm_irq_install(dev);

	rdmsr(0xa0002001, lo, hi);
	DRM_DEBUG_DRIVER("GP GLD config: %08x\n", lo);
	rdmsr(0x80002001, lo, hi);
	DRM_DEBUG_DRIVER("DC GLD config: %08x\n", lo);
	rdmsr(0x48002001, lo, hi);
	DRM_DEBUG_DRIVER("VP GLD config: %08x\n", lo);
	rdmsr(0x54002001, lo, hi);
	DRM_DEBUG_DRIVER("VI GLD config: %08x\n", lo);
	rdmsr(0x4c000014, lo, hi);
	DRM_DEBUG_DRIVER("CP sys_rstpll: %08x_%08x\n", hi, lo);

	for (i=0x10000020; i<0x1000002c; i++) {
		rdmsr(i, lo, hi);
		DRM_DEBUG_DRIVER("GLIU0 P2D 0x%x: pdid1: %d, biz: %d, "
			"offset: %05x, pmax: %05x, pmin: %05x\n",
			i, hi >> 29, (hi & (1 << 28)) != 0, (hi >> 8) & 0xfffff,
			((hi << 12) | (lo >> 20)) & 0xfffff, lo & 0xfffff);
	}
	rdmsr(i, lo, hi);
	DRM_DEBUG_DRIVER("GLIU0 P2D SC: pdid1: %d, biz: %d, wen: %04x, "
		"ren: %04x, pscbase: %04x\n",
		hi >> 29, (hi & (1 << 28)) != 0, hi & 0xffff, lo >> 16,
		(lo & 0x3fff) << 2);
	for (i=0x100000e0; i<0x100000e3; i++) {
		rdmsr(i, lo, hi);
		DRM_DEBUG_DRIVER("GLIU0 IOD 0x%x: idid: %d, biz: %d, "
			"ibase: %05x, imask: %05x\n", i, hi >> 29,
			!!(hi & (1 << 28)), ((hi << 12) | (lo >> 20)) & 0xfffff,
			lo & 0xfffff);
	}

	lo = read_dc(priv, DC_GLIU0_MEM_OFFSET);
	hi = read_dc(priv, DC_DV_CTL);
	DRM_DEBUG_DRIVER("DC GLIU0 mem offset: @ %03x MB, "
		"DV RAM address: %03x, offset: %05x\n",
		lo >> 20, lo & 0x7ff, hi >> 12);

	DRM_DEBUG_DRIVER("FP PM: %08x\n", read_fp(priv, FP_PM));

	return 0;

failed_map_video_memory:
	kfree(priv);
	return ret;
}

static void lx_driver_unload(struct drm_device *dev)
{
	struct lx_priv *priv = dev->dev_private;
	int ret;

	if (drm_core_check_feature(dev, DRIVER_HAVE_IRQ) && dev->irq_enabled) {
		ret = drm_legacy_irq_uninstall(dev);
		if (ret)
			DRV_INFO("failed uninstalling IRQs: %d\n", ret);
	}

	// v3.0 API: drm_vblank_cleanup(dev);

	lx_modeset_cleanup(dev);
	lx_unmap_video_memory(dev);

	kfree(priv);
}

static irqreturn_t lx_driver_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct lx_priv *priv = dev->dev_private;
	irqreturn_t ret = IRQ_NONE;
	u32 dc_irq, gp_irq = 0;

	/* this should be protected by disabled IRQs */
	{
		dc_irq = read_dc(priv, DC_IRQ);
#if 0
		gp_irq = read_gp(priv, GP_INT_CNTRL);
#endif

		dc_irq &= ~(dc_irq << 16); /* clear all masked interrupts */
		gp_irq &= ~(gp_irq << 16); /* clear all masked interrupts */

		if (dc_irq & LX_IRQ_STATUS_MASK) {
			/* some of DC's IRQ status bits seem to be enabled */
			dc_unlock(priv);
			write_dc(priv, DC_IRQ, dc_irq); /* clear DC IRQ status */
			dc_lock(priv);
		}

		if (gp_irq & LX_IRQ_STATUS_MASK) {
			/* some of GP's IRQ status bits seem to be enabled */
			write_gp(priv, GP_INT_CNTRL, gp_irq); /* clear GP IRQ status */
		}
	}

	if (dc_irq & DC_IRQ_STATUS) {
		/* programmed line counter value (0) has been reached by the DC,
		 * so the beginning of the scanout buffer */

		/* need to get the timestamp first as the do_page_flip code may
		 * already depend on it to send back to userspace */
		priv->last_vblank = ktime_get();

		/* process possibly pending page flip */
		lx_crtc_do_page_flip(&priv->crtcs[LX_CRTC_GRAPHIC].base);

		drm_handle_vblank(dev, LX_CRTC_GRAPHIC);
		ret = IRQ_HANDLED;
	}
	if (dc_irq & DC_IRQ_VIP_VSYNC_IRQ_STATUS) {
		ret = IRQ_HANDLED;
	}

	if (gp_irq & GP_INT_IDLE_STATUS) {
		ret = IRQ_HANDLED;
	}
	if (gp_irq & GP_INT_CMD_BUF_EMPTY_STATUS) {
		ret = IRQ_HANDLED;
	}

	return ret;
}

static void lx_irq_enable_vblank(struct drm_device *dev)
{
	struct lx_priv *priv = dev->dev_private;
	u32 dc_irq_filt_ctl;
	u32 dc_irq;

	dc_unlock(priv);

	/* set line counter */
	dc_irq_filt_ctl = read_dc(priv, DC_IRQ_FILT_CTL);
	dc_irq_filt_ctl &= ~0x07ff0000; /* clear bits [26:16] */
	/* this is really a scanout line trigger (the "real" vblank interrupt
	 * the Geode provides is an SMM one, so unaccessible to us), but this
	 * should fit our needs */
	dc_irq_filt_ctl |= 0 << 16; /* line 0 of scanout buffer */
	write_dc(priv, DC_IRQ_FILT_CTL, dc_irq_filt_ctl);

	/* enable line count interrupt */
	dc_irq = read_dc(priv, DC_IRQ);
	dc_irq &= ~LX_IRQ_STATUS_MASK;  /* don't clear any interrupt status bits
	                                 * by writing back what was read */
	dc_irq &= ~DC_IRQ_MASK;         /* unmask line cnt interrupt */
	write_dc(priv, DC_IRQ, dc_irq);

	dc_lock(priv);
}

static void lx_irq_disable_vblank(struct drm_device *dev)
{
	struct lx_priv *priv = dev->dev_private;
	u32 dc_irq;

	dc_unlock(priv);

	dc_irq = read_dc(priv, DC_IRQ);
	dc_irq &= ~LX_IRQ_STATUS_MASK; /* don't clear any interrupt status bits */
	dc_irq |= DC_IRQ_MASK; /* mask line cnt interrupt */
	write_dc(priv, DC_IRQ, dc_irq);

	dc_lock(priv);
}

static void lx_driver_irq_preinstall(struct drm_device *dev)
{
	struct lx_priv *priv = dev->dev_private;

	/* disable all interrupts */

	dc_unlock(priv);

	/* DC: line cnt & vip vsync loss interrupts */
	write_dc(priv, DC_IRQ, DC_IRQ_MASK | DC_IRQ_VIP_VSYNC_IRQ_MASK);
	/* GP: idle & cmd buffer empty */
	write_gp(priv, GP_INT_CNTRL, GP_INT_IDLE_MASK |
				     GP_INT_CMD_BUF_EMPTY_MASK);

	dc_lock(priv);
}

static int lx_driver_irq_postinstall(struct drm_device *dev)
{
	lx_irq_enable_vblank(dev);

	return 0;
}

static void lx_driver_irq_uninstall(struct drm_device *dev)
{
	lx_irq_disable_vblank(dev);
	lx_driver_irq_preinstall(dev);
}

/* --------------------------------------------------------------------------
 * Stubs
 * -------------------------------------------------------------------------- */

static int lx_driver_open(struct drm_device *dev, struct drm_file *file)
{
	DRM_DEBUG_DRIVER("\n");

	file->driver_priv = dev->dev_private;
	idr_init(&file->object_idr);
	spin_lock_init(&file->table_lock);

	return 0;
}

/**
 * lastclose - clean up after all DRM clients have exited
 * @dev: DRM device
 *
 * Take care of cleaning up after all DRM clients have exited.  In the
 * mode setting case, we want to restore the kernel's initial mode (just
 * in case the last client left us in a bad state).
 *
 * Additionally, in the non-mode setting case, we'll tear down the AGP
 * and DMA structures, since the kernel won't be using them, and clean
 * up any GEM state.
 */
static void lx_driver_lastclose(struct drm_device *dev)
{
	struct lx_priv *priv = dev->dev_private;

	DRM_DEBUG_DRIVER("\n");
	drm_fb_helper_restore_fbdev_mode_unlocked(&priv->fb->helper);
	vga_switcheroo_process_delayed_switch();
}

static void lx_driver_preclose(struct drm_device *dev, struct drm_file *file)
{
	DRM_DEBUG_DRIVER("\n");
}

static int lx_idr_free(int id, void * p, void * data)
{
	struct drm_file * file_priv = data;
	struct lx_priv * priv = file_priv->driver_priv;
	struct lx_bo * bo = p;

	DRV_INFO(
	        "removing left-over chunk with handle %d, size: %llu at %llx\n",
	        id, bo->node->size, bo->node->start);
	lx_bo_destroy(priv, file_priv, bo);

	return 0;
}

static void lx_driver_postclose(struct drm_device *dev, struct drm_file *file)
{
	DRM_DEBUG_DRIVER("\n");

	idr_for_each(&file->object_idr, lx_idr_free, file);
	idr_destroy(&file->object_idr);

	file->driver_priv = NULL;
}

static struct drm_ioctl_desc lx_ioctls[] = {
};

static int lx_driver_dumb_create(struct drm_file *file_priv,
				 struct drm_device *dev,
				 struct drm_mode_create_dumb *args)
{
	struct lx_priv *priv = dev->dev_private;
	struct drm_mm_node *node;
	struct lx_bo *bo;
	unsigned pitch;

	/* TODO?: args->bpp = ALIGN(args->bpp, 8); */
	pitch = ALIGN(args->width * args->bpp, 8 * 8) / 8;
	if (pitch > args->pitch)
		args->pitch = pitch;
	args->size = ALIGN(args->pitch * args->height, PAGE_SIZE);

	DRM_DEBUG_DRIVER("%ux%u-%u, flags: %x -> size: %llu\n",
			 args->width, args->height, args->bpp, args->flags,
			 args->size);

	node = lx_mm_alloc(priv, args->size, 32, 1);
	if (!node)
		return -ENOMEM;

	bo = lx_bo_create(file_priv, node);
	if (!bo) {
		drm_mm_remove_node(node);
		return -ENOMEM;
	}

	bo->width = args->width;
	bo->height = args->height;
	bo->bpp = args->bpp;
	bo->pitch = args->pitch;

	args->handle = bo->id;

	return 0;
}

static int lx_driver_dumb_map_offset(struct drm_file *file_priv,
				     struct drm_device *dev, uint32_t handle,
				     uint64_t *offset)
{
	struct lx_priv *priv = dev->dev_private;
	struct lx_bo *bo = idr_find(&file_priv->object_idr, handle);
	int ret;

	DRM_DEBUG_DRIVER("handle: %u, bo: %p\n", handle, bo);

	if (!bo)
		return -ENOENT;

	ret = lx_bo_addmap(priv, bo);
	if (ret) {
		DRM_ERROR("unable to add local map for object %d, size: %llu\n",
			  handle, bo->node->size);
		return ret;
	}

	*offset = bo->map->offset;

	return 0;
}

static int lx_driver_dumb_destroy(struct drm_file *file_priv,
				  struct drm_device *dev, uint32_t handle)
{
	struct lx_priv *priv = dev->dev_private;
	struct lx_bo *bo = idr_find(&file_priv->object_idr, handle);

	DRM_DEBUG_DRIVER("handle: %u, bo: %p\n", handle, bo);

	if (!bo) {
		DRM_ERROR("object %d doesn't exist\n", handle);
		return -ENOENT;
	}

	lx_bo_destroy(priv, file_priv, bo);

	return 0;
}

static const struct file_operations lx_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.release	= drm_release,
	.unlocked_ioctl	= drm_ioctl,
	.mmap		= drm_legacy_mmap,
/*	.fasync		= drm_fasync, // drm_fasync got removed since v3.0 */
	.poll		= drm_poll,
	.read		= drm_read,
	.llseek		= noop_llseek,
};

static const struct drm_driver lx_driver = {
	.driver_features	= DRIVER_HAVE_IRQ |
/*				  DRIVER_IRQ_SHARED | // v3.0 API */
				  DRIVER_MODESET,
	.dev_priv_size		= sizeof(struct lx_priv),
	.load			= lx_driver_load,
	.unload			= lx_driver_unload,
/*	.device_is_agp		= lx_driver_device_is_agp, // v3.0 API */
	.irq_preinstall		= lx_driver_irq_preinstall,
	.irq_postinstall	= lx_driver_irq_postinstall,
	.irq_uninstall		= lx_driver_irq_uninstall,
	.irq_handler		= lx_driver_irq_handler,
	.enable_vblank		= lx_driver_enable_vblank,
	.disable_vblank		= lx_driver_disable_vblank,
/*	.get_vblank_counter	= drm_vblank_count, // legacy API, drm_vblank_count() went away since v3.0 */

	/* These need rework ... */
	.open			= lx_driver_open,
	.lastclose		= lx_driver_lastclose,
	.preclose		= lx_driver_preclose,
	.postclose		= lx_driver_postclose,
	.dumb_create		= lx_driver_dumb_create,
	.dumb_map_offset	= lx_driver_dumb_map_offset,
	.dumb_destroy		= lx_driver_dumb_destroy,

	/* These are stubs ... */
	.ioctls			= lx_ioctls,
	.num_ioctls		= ARRAY_SIZE(lx_ioctls),
#if 0
	.reclaim_buffers	= drm_core_reclaim_buffers,
	.get_scanout_position   = lx_driver_get_scanout_position,
	// .dma_quiescent		= lx_driver_dma_quiescent,
	// .dma_ioctl		= lx_driver_dma_ioctl,
#endif
	.fops			= &lx_driver_fops,

	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
	.patchlevel		= DRIVER_PATCHLEVEL,
};

static const struct pci_device_id pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_LX_VIDEO) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, pci_ids);

static struct pci_driver lx_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = pci_ids,
};

#ifndef CONFIG_PCI
# error require CONFIG_PCI=y
#endif

static int __init lx_init(void) {
	return drm_legacy_pci_init(&lx_driver, &lx_pci_driver);
}

static void __exit lx_exit(void) {
	drm_legacy_pci_exit(&lx_driver, &lx_pci_driver);
}

module_init(lx_init);
module_exit(lx_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(__stringify(DRIVER_MAJOR) "."
	       __stringify(DRIVER_MINOR) "."
	       __stringify(DRIVER_PATCHLEVEL));
