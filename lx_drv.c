/*
 * Copyright (C) 2011 Franz Brauße.
 *
 * Partly based on lxfb
 * 	(which is Copyright (C) 2007 Advanced Micro Devices, Inc.),
 * xf86-video-geode
 * 	(which is (C) ),
 * with code borrowed from i915 and radeon drivers.
 */

#include <linux/vga_switcheroo.h>
#include <linux/cs5535.h>
#include <drm.h>

#include "drmP.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"

#include "lx.h"

static struct pci_device_id pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_LX_VIDEO) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, pci_ids);

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
	u32 pllval;   /* [15]   : divide output by 4 (unused in this table) */
	              /* [14:12]: input clock divisor (M) */
	              /* [11:4] : dot clock PLL divisor (N) */
	              /* [3:0]  : post scaler divisor (P) */
	u32 freq_khz; /* resulting dot-freq in kHz when PLL stabilizes */
} const lx_pll_freq[] = {   /* M+1 N+1 P+1 */
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
	{ 0x0089,  43163 }, /*   1,  9, 10 */
	{ 0x10E7,  44900 }, /*   2, 15,  8 */
	{ 0x2136,  45720 }, /*   3, 20,  7 */
	{ 0x3207,  49500 }, /*   4, 33,  8 */
	{ 0x2187,  50000 }, /*   3, 25,  8 */
	{ 0x4286,  56250 }, /*   5, 41,  7 */
	{ 0x10E5,  60650 }, /*   2, 15,  6 */
	{ 0x4214,  65000 }, /*   5, 34,  5 */
	{ 0x1105,  68179 }, /*   2, 17,  6 */
	{ 0x31E4,  74250 }, /*   4, 31,  5 */
	{ 0x3183,  75000 }, /*   4, 25,  4 */
	{ 0x4284,  78750 }, /*   5, 41,  5 */
	{ 0x1104,  81600 }, /*   2, 17,  5 */
	{ 0x6363,  94500 }, /*   7, 55,  4 */
	{ 0x5303,  97520 }, /*   6, 49,  4 */
	{ 0x2183, 100187 }, /*   3, 25,  4 */
	{ 0x2122, 101420 }, /*   3, 19,  3 */
	{ 0x41B1, 106500 }, /*   5, 28,  2 */
	{ 0x1081, 108000 }, /*   2,  9,  2 */
	{ 0x6201, 113310 }, /*   7, 33,  2 */
	{ 0x0041, 119650 }, /*   1,  5,  2 */
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
	{ 0x52E1, 189000 }, /*   6, 47,  2 */
	{ 0x0071, 192000 }, /*   1,  8,  2 */
	{ 0x3201, 198000 }, /*   4, 33,  2 */
	{ 0x4291, 202500 }, /*   5, 42,  2 */
	{ 0x1101, 204750 }, /*   2, 17,  2 */
	{ 0x7481, 218250 }, /*   8, 73,  2 */
	{ 0x4170, 229500 }, /*   5, 24,  1 */
	{ 0x6210, 234000 }, /*   7, 34,  1 */
	{ 0x3140, 251182 }, /*   4, 21,  1 */
	{ 0x6250, 261000 }, /*   7, 38,  1 */
	{ 0x41C0, 278400 }, /*   5, 29,  1 */
	{ 0x5220, 280640 }, /*   6, 35,  1 */
	{ 0x0050, 288000 }, /*   1,  6,  1 */
	{ 0x41E0, 297000 }, /*   5, 31,  1 */
	{ 0x2130, 320207 }, /*   3, 20,  1 */
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


/* --------------------------------------------------------------------------
 * FB
 * -------------------------------------------------------------------------- */

static int lx_user_fb_create_handle(struct drm_framebuffer *fb,
				    struct drm_file *file_priv,
				    unsigned int *handle)
{
	DRM_DEBUG_DRIVER("\n");
	return 0;
}

static void lx_user_fb_destroy(struct drm_framebuffer *fb) {
	drm_framebuffer_cleanup(fb);
}

static struct drm_framebuffer_funcs lx_fb_funcs = {
	.create_handle = lx_user_fb_create_handle,
	.destroy = lx_user_fb_destroy,
};

static int lx_fb_init(struct lx_priv *priv, struct lx_fb *lfb,
		       struct drm_mode_fb_cmd *mode_cmd,
		       struct ttm_buffer_object *bo)
{
	int ret;

	lfb->bo = bo;

	ret = drm_framebuffer_init(priv->ddev, &lfb->base, &lx_fb_funcs);
	if (ret) {
		DRM_DEBUG_DRIVER("drm_framebuffer_init failed: %d\n", ret);
		return ret;
	}

	drm_helper_mode_fill_fb_struct(&lfb->base, mode_cmd);

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
	unsigned long size;
	int ret;

	unsigned page_align = 1;
	bool kernel = true;
	struct ttm_placement placement;

	DRM_DEBUG_DRIVER("helper: %p, sizes: %p, priv: %p, fb: %p\n",
			 helper, sizes, priv, helper->fb);
	if (helper->fb)
		return 0;

	mode_cmd.width  = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.bpp    = sizes->surface_bpp;
	mode_cmd.depth  = sizes->surface_depth;
	mode_cmd.pitch  = mode_cmd.width * ((mode_cmd.bpp + 7) / 8);

	/* allocate backing store */
	size = mode_cmd.pitch * mode_cmd.height;
#if 0
	size = ALIGN(size, PAGE_SIZE);
	/* TODO: init placement */
	/* lock vram-mutex */
	ret = ttm_bo_create(&priv->mman.bdev, size, ttm_bo_type_device,
			    &placement, page_align, 0, !kernel, NULL, &lxfb->bo);
	/* unlock vram-mutex */
	if (ret) {
		DRM_ERROR("error allocating bo: %d\n", ret);
		/* TODO: retry */
		framebuffer_release(info);
		return ret;
	}
#endif

	mutex_lock(&dev->struct_mutex);

	info = framebuffer_alloc(0, &priv->pdev->dev);
	if (!info)
		return -ENOMEM;

	info->par = helper;

	ret = lx_fb_init(priv, lfb, &mode_cmd, lfb->bo);
	if (ret) {
		DRM_ERROR("Failed to initialize drm_framebuffer: %d\n", ret);
		framebuffer_release(info);
		return ret;
	}

	/* setup helper */
	helper->fb = fb;
	helper->fbdev = info;

	strcpy(info->fix.id, "lxdrmfb");

	drm_fb_helper_fill_fix(info, fb->pitch, fb->depth);

	info->flags = FBINFO_DEFAULT /*| FBINFO_VIRTFB*/;
	info->fbops = &lx_fb_ops;

	info->fix.smem_start = priv->vmem_phys;
	info->fix.smem_len = size;
	info->screen_base = ioremap(priv->vmem_phys, size);
	info->screen_size = size;

	memset_io(info->screen_base, 0, size);

	drm_fb_helper_fill_var(info, helper, sizes->fb_width, sizes->fb_height);

	info->apertures = alloc_apertures(1);
	DRM_DEBUG_DRIVER("fb aper: %p\n", info->apertures);
	if (info->apertures) {
		info->apertures->ranges[0].base = priv->vmem_phys;
		info->apertures->ranges[0].size = priv->vmem_size;
	}

	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret) {
		DRM_ERROR("error allocating cmap: %d\n", ret);
		info->cmap.len = 0;
	}

	mutex_unlock(&dev->struct_mutex);

	/* TODO: init info->pixmap */
	/* TODO: fb_alloc_cmap(&info->cmap, 256, 0) */

	DRM_DEBUG_DRIVER("dev->mode_config.mutex locked: %d\n",
			 mutex_is_locked(&priv->ddev->mode_config.mutex));

	DRM_INFO("fb mappable at 0x%lx\n",  info->fix.smem_start);
	DRM_INFO("vram aper   at 0x%lx\n", (unsigned long)priv->vmem_phys);
	DRM_INFO("size %lu\n", size);
	DRM_INFO("fb depth is %d\n", fb->depth);
	DRM_INFO("   pitch is %d\n", fb->pitch);
	DRM_INFO("  height is %d\n", fb->height);
	DRM_INFO("   width is %d\n", fb->width);

	vga_switcheroo_client_fb_set(priv->pdev, info);

	/* TODO: retrieve enough VMEM space for FB from GEM/TTM, init fb_info
	 * and return 1 */
	return 1;
}

static struct drm_fb_helper_funcs lx_fb_helper_funcs = {
	.gamma_get = lx_crtc_fb_gamma_get,
	.gamma_set = lx_crtc_fb_gamma_set,
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
	lfb->helper.funcs = &lx_fb_helper_funcs;

	priv->fb = lfb;

	ret = drm_fb_helper_init(dev, &lfb->helper, 1, LX_NUM_CONNECTORS);
	if (ret) {
		DRM_DEBUG_DRIVER("error init'ing fb_helper: %d\n", ret);
		return ret;
	}

	drm_fb_helper_single_add_all_connectors(&lfb->helper);
	drm_fb_helper_initial_config(&lfb->helper, bpp);

	return 0;
}

static void lx_fbdev_fini(struct drm_device *dev) {
	struct lx_priv *priv = dev->dev_private;
	struct lx_fb *lfb = priv->fb;
	struct fb_info *info = lfb->helper.fbdev;

	if (info) {
		unregister_framebuffer(info);
		if (info->cmap.len)
			fb_dealloc_cmap(&info->cmap);
		iounmap(info->screen_base);
		framebuffer_release(info);
	}

	drm_fb_helper_fini(&lfb->helper);
	drm_framebuffer_cleanup(&lfb->base); /* TODO: don't? */

	kfree(lfb);
	priv->fb = NULL;
}

static struct drm_framebuffer * lx_user_fb_create(struct drm_device *dev,
						  struct drm_file *file_priv,
						  struct drm_mode_fb_cmd *mode_cmd)
{
	struct lx_fb *lfb;
	int ret;

	/* see radeon_display.c:
	 * TODO: lookup ttm object via mode_cmd->handle */

	lfb = kzalloc(sizeof(*lfb), GFP_KERNEL);
	if (!lfb)
		return ERR_PTR(-ENOMEM);

	lfb->priv = dev->dev_private;
	ret = lx_fb_init(dev->dev_private, lfb, mode_cmd, NULL);
	if (ret)
		return ERR_PTR(ret);

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

static const int connector_best_encoders[LX_NUM_CONNECTORS] = {
	[LX_CONNECTOR_VGA] = LX_ENCODER_DAC,
	[LX_CONNECTOR_LVDS] = LX_ENCODER_LVDS, /* PANEL */
	[LX_CONNECTOR_VOP] = LX_ENCODER_BYPASS, /* VOP */
	[LX_CONNECTOR_COMPANION] = LX_ENCODER_BYPASS, /* DRGB */
};

static struct drm_encoder * lx_connector_best_encoder(
					struct drm_connector *connector)
{
	struct lx_priv *priv = connector->dev->dev_private;
	struct lx_connector *lx_conn = to_lx_connector(connector);
	enum lx_encoders encoder_id = connector_best_encoders[lx_conn->id];
	return &priv->encoders[encoder_id].base;
}

/*
static struct drm_encoder * lx_connector_attached_encoder(
		struct drm_connector *connector)
{
	int i;

	for (i=0; i<DRM_CONNECTOR_MAX_ENCODER; i++) {
		struct drm_mode_object *obj;

		if (connector->encoder_ids[i] == 0)
			break;

		obj = drm_mode_object_find(connector->dev,
					   connector->encoder_ids[i],
					   DRM_MODE_OBJECT_ENCODER);

		if (obj)
			return obj_to_encoder(obj);
	}

	return NULL;
}*/

static enum drm_connector_status lx_connector_vga_detect(
		struct drm_connector *connector, bool force)
{
	struct lx_priv *priv = connector->dev->dev_private;
	struct drm_encoder *encoder;
	struct drm_encoder_helper_funcs *encoder_funcs;

	if (priv->ddc && lx_ddc_probe(priv->ddc))
		return connector_status_connected;

	if (force) {
		encoder = lx_connector_best_encoder(connector);
		if (encoder) {
			encoder_funcs = encoder->helper_private;
			return encoder_funcs->detect(encoder, connector);
		}
	}

	return connector->status;
}

/** Called through drm_connector_helper_funcs by
 * drm_helper_probe_single_connector_modes.
 *
 * Probes the connector for modes or adds some default modes provided by
 * DRM EDID and returns the number of modes successfully added.
 */
static int lx_connector_vga_get_modes(struct drm_connector *connector)
{
	struct lx_priv *priv = connector->dev->dev_private;
	struct edid *edid = NULL;
	int num_modes;

	if (priv->ddc) {
		edid = drm_get_edid(connector, priv->ddc);
		if (!edid) {
			DRM_INFO("lx: failed to retrieve EDID via builtin DDC\n");
		} else if (!drm_edid_is_valid(edid)) {
			DRM_INFO("lx: EDID retrieved via builtin DDC is invalid\n");
			kfree(connector->display_info.raw_edid);
			connector->display_info.raw_edid = NULL;
			edid = NULL;
		}
	}

	if (edid) {
		num_modes = drm_add_edid_modes(connector, edid);
	} else {
		num_modes = drm_add_modes_noedid(connector, 1920, 1440);
	}
	DRM_INFO("lx: added %d %s modes for connector %s\n", num_modes,
		 edid ? "EDID" : "static", drm_get_connector_name(connector));

	/* In any case update the edid property to reflect the current status */
	drm_mode_connector_update_edid_property(connector, edid);

	return num_modes;
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

static int lx_connector_mode_valid(struct drm_connector *connector,
				   struct drm_display_mode *mode)
{
	struct lx_connector *lx_conn = to_lx_connector(connector);
	int ret = lx_graphic_mode_valid(mode, NULL, NULL);
	if (ret != MODE_OK)
		return ret;

	if (mode->vrefresh > 100)
		return MODE_CLOCK_HIGH;

	switch (lx_conn->id) {
	case LX_CONNECTOR_VGA:
		if (mode->hdisplay > 1920)
			return MODE_BAD_HVALUE;
		if (mode->vdisplay > 1440)
			return MODE_BAD_VVALUE;
		if (mode->hdisplay == 1920 && mode->vdisplay == 1440 &&
		    mode->vrefresh > 85)
			return MODE_CLOCK_HIGH;
		if (mode->clock > 340000 + LX_MODE_FREQ_TOL)
			return MODE_CLOCK_HIGH;
		break;
	case LX_CONNECTOR_LVDS:
		if (mode->hdisplay > 1600)
			return MODE_BAD_HVALUE;
		if (mode->vdisplay > 1200)
			return MODE_BAD_VVALUE;
		if (mode->clock > 170000 + LX_MODE_FREQ_TOL)
			return MODE_CLOCK_HIGH;
		break;
	}

	return MODE_OK;
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

static void lx_connector_vga_destroy(struct drm_connector *connector)
{
	DRM_DEBUG_DRIVER("\n");

	/* TODO: switch off outputs? */

	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);

	/* TODO: release & cleanup output device(s) / registers */
}

static const struct drm_connector_helper_funcs lx_connector_vga_helper_funcs = {
	.get_modes	= lx_connector_vga_get_modes,
	.mode_valid	= lx_connector_mode_valid,
	.best_encoder	= lx_connector_best_encoder,
};

static const struct drm_connector_funcs lx_connector_vga_funcs = {
	.dpms		= drm_helper_connector_dpms,
	.detect		= lx_connector_vga_detect,
	.fill_modes	= drm_helper_probe_single_connector_modes,
	.destroy	= lx_connector_vga_destroy,
	.reset		= lx_connector_vga_reset,
#if 0
	void (*save)(struct drm_connector *connector);
	void (*restore)(struct drm_connector *connector);

	int (*set_property)(struct drm_connector *connector, struct drm_property *property,
			     uint64_t val);
	void (*force)(struct drm_connector *connector);
#endif
};

static struct {
	int type;
	const struct drm_connector_funcs *funcs;
	const struct drm_connector_helper_funcs *hfuncs;
} const lx_connector_types[LX_NUM_CONNECTORS] = {
	[LX_CONNECTOR_VGA]       = { DRM_MODE_CONNECTOR_VGA, &lx_connector_vga_funcs, &lx_connector_vga_helper_funcs },
	[LX_CONNECTOR_LVDS]      = { DRM_MODE_CONNECTOR_LVDS, },
	[LX_CONNECTOR_COMPANION] = { DRM_MODE_CONNECTOR_Unknown, },
	[LX_CONNECTOR_VOP]       = { DRM_MODE_CONNECTOR_Unknown, },
};

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

	/* TODO: really setup outputs */
	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;
	connector->initial_x = 0;
	connector->initial_y = 0;

	if (connector_id == LX_CONNECTOR_VGA) {
		connector->polled = DRM_CONNECTOR_POLL_CONNECT;
		if (priv->ddc)
			connector->polled |= DRM_CONNECTOR_POLL_DISCONNECT;
	}

	drm_sysfs_connector_add(connector);

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
	struct drm_encoder_helper_funcs *encoder_funcs;
	encoder_funcs = encoder->helper_private;
	encoder_funcs->dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void lx_encoder_commit(struct drm_encoder *encoder) {
	struct drm_encoder_helper_funcs *encoder_funcs;
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

static unsigned lx_get_enabled_encoders(struct drm_crtc *crtc) {
	struct drm_encoder *enc;
	unsigned idx = 0, mask = 0;

	list_for_each_entry(enc, &crtc->dev->mode_config.encoder_list, head) {
		if (enc->crtc == crtc && to_lx_encoder(enc)->enabled)
			mask |= BIT_MASK(idx);
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
	case LX_ENCODER_LVDS:
		cloned  = priv->encoders[LX_ENCODER_DAC].enabled;
		cloned &= priv->encoders[LX_ENCODER_LVDS].enabled;
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
	.mode_fixup = lx_encoder_dac_mode_fixup,
	.mode_set = lx_encoder_dac_mode_set,
#if 0
	void (*save)(struct drm_encoder *encoder);
	void (*restore)(struct drm_encoder *encoder);

	void (*mode_set)(struct drm_encoder *encoder,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode);
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
	[LX_ENCODER_LVDS]   = { DRM_MODE_ENCODER_LVDS, },
	[LX_ENCODER_TVDAC]  = { DRM_MODE_ENCODER_TVDAC, },
	[LX_ENCODER_BYPASS] = { DRM_MODE_ENCODER_NONE, },
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
			 lx_encoder_types[encoder_id].type);

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

static void lx_crtc_gamma_set(struct drm_crtc *crtc, u16 *r, u16 *g, u16 *b,
			      uint32_t start, uint32_t size)
{
	struct lx_crtc *lx_crtc = to_lx_crtc(crtc);
	struct lx_rgb *lut_entry;
	uint32_t i, end;

	end = (start + size > LX_LUT_SIZE) ? LX_LUT_SIZE : (start + size);

	for (i = start, lut_entry = lx_crtc->lut; i < end; i++, lut_entry++) {
		lut_entry->r = r[i] >> 8;
		lut_entry->g = g[i] >> 8;
		lut_entry->b = b[i] >> 8;
	}

	lx_crtc_load_lut(crtc);
}

static int lx_crtc_cursor_set(struct drm_crtc *crtc, struct drm_file *file_priv,
			      uint32_t handle, uint32_t width, uint32_t height)
{
	/* TODO: fetch (32 byte aligned) address to this object in VMEM, write
	 *       to DC_CURS_ST_OFFSET (@ 0x018) */
	return -ENXIO;
}

static int lx_crtc_cursor_move(struct drm_crtc *crtc, int x, int y) {
	struct lx_priv *priv = crtc->dev->dev_private;
	union {
		struct {
			u32 pad    : 16;
			u32 offset : 6;
			u32 coord  : 10;
		} c;
		u32 v;
	} rx = { { .offset = x < 0 ? -x : 0, .coord = x < 0 ? 0 : x, } },
	  ry = { { .offset = y < 0 ? -y : 0, .coord = y < 0 ? 0 : y, } };

	write_dc(priv, DC_CURSOR_X, rx.v);
	write_dc(priv, DC_CURSOR_Y, ry.v);

	return 0;
}

static void lx_crtc_reset(struct drm_crtc *crtc)
{
	struct lx_priv *priv = crtc->dev->dev_private;
	u32 genlk, filt_ctl, lo, hi;

	DRM_DEBUG_DRIVER("\n");
	return;

	/* Clear the various buffers */

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
	/* set default watermark values */

	rdmsr(MSR_DC_SPARE_MSR, lo, hi);

	lo |= MSR_DC_SPARE_MSR_DIS_VIFO_WM;

	wrmsr(MSR_DC_SPARE_MSR, lo, hi);
}

static const struct drm_crtc_funcs lx_crtc_funcs = {
	.gamma_set = lx_crtc_gamma_set,
	.set_config = drm_crtc_helper_set_config,
	.destroy = drm_crtc_cleanup,
#if 0
	.cursor_set = lx_crtc_cursor_set,
	.cursor_move = lx_crtc_cursor_move,
#endif
	// .page_flip = NULL,
	/* crtc->fb = fb;
	 * my_crtc->event = event;
	 * omap_gem_op_async(omap_framebuffer_bo(fb), OMAP_GEM_READ,
	 * 		     page_flip_cb, crtc);
	 *
	 * page_flip_cb:
	 * event = my_crtc->event;
	 * my_crtc->event = NULL;
	 * update_scanout;
	 * commit;
	 * // after flip: wakeup userspace:
	 * if (!event) return;
	 * spin_lock_irqsave(&dev->event_lock, flags);
	 * event->event.sequence = drm_vblank_count_and_time(dev, some_id, &now);
	 * event->tv_sec = now.tv_sec;
	 * event->tv_usec = now.tv_usec;
	 * list_add_tail(&event->base.link, &event->base.file_priv->event_list);
	 * wake_up_interruptible(&event->base.file_priv->event_wait);
	 * spin_unlock_irqrestore(&dev->event_lock, flags); */
#if 0
	/* Save CRTC state */
	void (*save)(struct drm_crtc *crtc); /* suspend? */
	/* Restore CRTC state */
	void (*restore)(struct drm_crtc *crtc); /* resume? */
	/* Reset CRTC state */
	void (*reset)(struct drm_crtc *crtc);

	/*
	 * Flip to the given framebuffer.  This implements the page
	 * flip ioctl described in drm_mode.h, specifically, the
	 * implementation must return immediately and block all
	 * rendering to the current fb until the flip has completed.
	 * If userspace set the event flag in the ioctl, the event
	 * argument will point to an event to send back when the flip
	 * completes, otherwise it will be NULL.
	 */
	int (*page_flip)(struct drm_crtc *crtc,
			 struct drm_framebuffer *fb,
			 struct drm_pending_vblank_event *event);
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

	write_dc(priv, DC_UNLOCK, DC_UNLOCK_UNLOCK);
	write_dc(priv, DC_GENERAL_CFG, gcfg);
	write_dc(priv, DC_DISPLAY_CFG, dcfg);
	write_dc(priv, DC_UNLOCK, DC_UNLOCK_LOCK);
}

static bool lx_crtc_mode_fixup(struct drm_crtc *crtc,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	int ret;

	drm_mode_set_crtcinfo(adjusted_mode, 0);

	ret = lx_graphic_mode_valid(adjusted_mode,
				    &adjusted_mode->clock,
				    &adjusted_mode->private_flags);
	DRM_DEBUG_DRIVER("ret: %d, clock: %d, pllval: %04x", ret,
			 adjusted_mode->clock, adjusted_mode->private_flags);

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

enum lx_cursor_status {
	CURSOR_DISABLED,
	CURSOR_2BPP,
	CURSOR_32BPP,
};

/* in 256 bytes */
#define DISP_PRIO_HIGH_END		0xb /* FIFO high prio end threshold */
#define DISP_PRIO_HIGH_START		0x6 /* FIFO high prio start threshold */
/* in 64 bytes */
#define VID_OVL_PRIO_HIGH_END		0xb
#define VID_OVL_PRIO_HIGH_START		0x6

static enum lx_cursor_status lx_cursor_status(struct drm_crtc *crtc) {
	return CURSOR_DISABLED;
}

static int lx_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
				 struct drm_framebuffer *old_fb)
{
	struct lx_priv *priv = crtc->dev->dev_private;
	struct drm_framebuffer *fb = crtc->fb;
	int offset, atomic = 0;
	u32 gcfg, dcfg, dvctl;
	u32 gfx_pitch, fb_width, fb_height, line_sz;

	DRM_DEBUG_DRIVER("x: %d, y: %d, old fb: %p\n", x, y, old_fb);

	priv->pan_x = x;
	priv->pan_y = y;

	/* see kernel git commit ffbc559b0699891c6deb9fd2b4750671eab94999:
	 *     drm/nv50/crtc: Bail out if FB is not bound to crtc
	 * shutdown case: e.g. monitor unplugged */
	if (!atomic && !fb) {
		DRM_DEBUG_DRIVER("no fb bound to crtc\n");
		return 0;
	}

	write_dc(priv, DC_UNLOCK, DC_UNLOCK_UNLOCK);

	/* setup hw to use the new fb crtc->fb with scanout starting at (FB-)
	 * coords (x,y) using crtc->fb->pitch. */
	/* release old fb (if it is different from the new) and possibly clear
	 * the object if it is no longer in use */

	gcfg = 0*DC_GENERAL_CFG_DFLE; /* enable display FIFO */
	gcfg |= DISP_PRIO_HIGH_END << DC_GENERAL_CFG_DFHPEL_SHIFT;
	gcfg |= DISP_PRIO_HIGH_START << DC_GENERAL_CFG_DFHPSL_SHIFT;

	gcfg &= ~DC_GENERAL_CFG_FDTY;
	gcfg &= ~DC_GENERAL_CFG_STFM;
	switch (fb->pitch) {
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

	switch (lx_cursor_status(crtc)) {
	case CURSOR_DISABLED:
		gcfg &= ~DC_GENERAL_CFG_CURE;
		break;
	case CURSOR_2BPP:
		gcfg |= DC_GENERAL_CFG_CURE;
		gcfg &= ~DC_GENERAL_CFG_CLR_CUR;
		break;
	case CURSOR_32BPP:
		gcfg |= DC_GENERAL_CFG_CURE;
		gcfg |= DC_GENERAL_CFG_CLR_CUR;
		break;
	}

	write_dc(priv, DC_GENERAL_CFG, gcfg);

	dcfg = 0;
	dcfg &= ~DC_DISPLAY_CFG_PALB; /* never bypass gamma / palette RAM */
	// dcfg |= DC_DISPLAY_CFG_DCEN; /* center display on flat panels */

	if (lx_video_overlay_enabled(crtc)) {
		dcfg |= VID_OVL_PRIO_HIGH_END << DC_DISPLAY_CFG_VFHPEL_SHIFT;
		dcfg |= VID_OVL_PRIO_HIGH_START << DC_DISPLAY_CFG_VFHPSL_SHIFT;
	}

	switch (fb->bits_per_pixel) {
	case 8:
		dcfg |= DC_DISPLAY_CFG_DISP_MODE_8BPP;
		break;
	case 16:
		dcfg |= DC_DISPLAY_CFG_DISP_MODE_16BPP;
		if (fb->depth == 15) {
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

	dcfg |= DC_DISPLAY_CFG_PALB; /* palette bypass in > 8bpp modes */
	dcfg |= DC_DISPLAY_CFG_GDEN; /* enable graphics data pass-through */
	dcfg |= DC_DISPLAY_CFG_TGEN; /* enable timing generator */
	dcfg |= DC_DISPLAY_CFG_TRUP; /* update timings immediatly */
	dcfg |= DC_DISPLAY_CFG_VISL; /* TODO: ?? */

	write_dc(priv, DC_DISPLAY_CFG, dcfg);

	write_dc(priv, DC_ARB_CFG, 0);

	/* bits [2:0] are added to the pixel panning offset after next vsync */
	/* The framebuffer space always starts at offset 0 from GLIU0_MEM_OFFSET
	 * because that way the compression may be enabled, which is unavailable
	 * otherwise. */
	offset = y * fb->pitch + x * ((fb->bits_per_pixel + 7) / 8);
	write_dc(priv, DC_FB_ST_OFFSET, offset << 3);

	/* TODO: write compressed buffer offset */
	/* TODO: write cursor buffer offset */
	/* TODO: write video (Y|U|V) buffers' offsets */
	/* TODO: possibly enable & write DV_TOP_ADDR */

	write_dc(priv, DC_DV_TOP, 0); /* TODO: DV-RAM disabled */

	line_sz = (fb->width * ((fb->bits_per_pixel + 7) / 8) + 7) / 8;
	write_dc(priv, DC_LINE_SIZE, /* in qwords */
		 0 << 20 /* TODO: video, align 4 */ |
		 0 << 10 /* TODO: compressed buffer, val+1, val <= 65 */ |
		 line_sz << 0); /* frame buffer */

	gfx_pitch = fb->pitch / 8;
	write_dc(priv, DC_GFX_PITCH, 0 << 16 /* TODO */ | gfx_pitch << 0);

	/* TODO: DC_VID_YUV_PITCH */
	/* TODO: scaling, see DC_FB_ACTIVE */

	fb_width = (fb->width - 1) | 7;
	fb_height = fb->height - 1;
	write_dc(priv, DC_FB_ACTIVE, fb_width << 16 | fb_height << 0);

	DRM_DEBUG_DRIVER("line_sz: %u qwords, gfx_pitch: %u qwords, fb_width: "
			 "%u qwords, fb_height: %u lines, fb->width: %u, "
			 "fb->height: %u, fb->pitch: %u, fb->bpp: %u\n",
			 line_sz, gfx_pitch, fb_width, fb_height, fb->width,
			 fb->height, fb->pitch, fb->bits_per_pixel);

	/* TODO: video downscaling */

	/* DV-RAM Control */
	/* Linear memory address is always aligned to 4096 */
	dvctl = priv->vmem_phys;

	if (fb->width < 1024) {
		dvctl |= DC_DV_CTL_DV_LINE_SIZE_1K;
	} else if (fb->width < 2048) {
		dvctl |= DC_DV_CTL_DV_LINE_SIZE_2K;
	} else if (fb->width < 4096) {
		dvctl |= DC_DV_CTL_DV_LINE_SIZE_4K;
	} else if (fb->width < 8192) {
		dvctl |= DC_DV_CTL_DV_LINE_SIZE_8K;
	}

	if (fb->height <= 512) {
		dvctl |= DC_DV_CTL_DV_RANGE_512;
	} else if (fb->height <= 1024) {
		dvctl |= DC_DV_CTL_DV_RANGE_1024;
	} else if (fb->height <= 1536) {
		dvctl |= DC_DV_CTL_DV_RANGE_1536;
	} else if (fb->height <= 2048) {
		dvctl |= DC_DV_CTL_DV_RANGE_2048;
	}

	if (fb->width < 8192 && fb->height <= 2048 &&
	    lx_compression_enabled(priv)) {
		/* mark DV-RAM as dirty & invalid */
		dvctl |= DC_DV_CTL_CLEAR_DV_RAM;
	} else {
		/* disable DV-RAM */
		dvctl |= DC_DV_CTL_DV_MASK;
	}

#if 0 /* don't overwrite offset as set by the BIOS */
	write_dc(priv, DC_DV_CTL, dvctl);
#else
	/* disable RAM snooping */
	write_dc(priv, DC_DV_CTL, read_dc(priv, DC_DV_CTL) | DC_DV_CTL_DV_MASK);
#endif

	/* TODO: DC_GFX_SCALE */

	write_dc(priv, DC_UNLOCK, DC_UNLOCK_LOCK);

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
	u32 lo, hi, dcfg, vcfg, real_freq;
	unsigned encoders = lx_get_enabled_encoders(crtc);

	DRM_DEBUG_DRIVER("\n");
	drm_mode_debug_printmodeline(m);

	/* dot-PLL */
	real_freq = lx_set_dotpll(m->private_flags & LX_MODE_PFLAG_DOTPLL_MASK);
	if (!real_freq)
		return MODE_CLOCK_RANGE;

	m->clock = real_freq;

	write_dc(priv, DC_UNLOCK, DC_UNLOCK_UNLOCK);

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
	write_dc(priv, DC_IRQ_FILT_CTL, m->crtc_vblank_start << 16);

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

	write_dc(priv, DC_UNLOCK, DC_UNLOCK_LOCK);

	/* set output format */
	rdmsr(LX_MSR(LX_VP, LX_GLD_MSR_CONFIG), lo, hi);

	lo &= ~MSR_VP_GLD_MSR_CONFIG_FPC;
	lo &= ~MSR_VP_GLD_MSR_CONFIG_FMT_MASK;

	if (encoders == BIT_MASK(LX_ENCODER_DAC))
		lo |= MSR_VP_GLD_MSR_CONFIG_FMT_CRT;
	else if (encoders & BIT_MASK(LX_ENCODER_LVDS))
		lo |= MSR_VP_GLD_MSR_CONFIG_FMT_FP;
	else if (encoders & BIT_MASK(LX_ENCODER_TVDAC))
		lo |= MSR_VP_GLD_MSR_CONFIG_FMT_VOP;
	else if (encoders == BIT_MASK(LX_ENCODER_BYPASS)) {
		lo |= MSR_VP_GLD_MSR_CONFIG_FMT_DRGB;
		/* TODO: set format byte order [7:6] based on drm-property */
		/* TODO: set interchange UV [14] based on drm-property */
	}

	if (encoders & (BIT_MASK(LX_ENCODER_LVDS) | BIT_MASK(LX_ENCODER_TVDAC)))
		lo |= MSR_VP_GLD_MSR_CONFIG_FPC;

	wrmsr(LX_MSR(LX_VP, LX_GLD_MSR_CONFIG), lo, hi);

	/* disable video acceleration hw */
	vcfg = read_vp(priv, VP_VCFG);
	vcfg &= ~VP_VCFG_VID_EN;
	write_vp(priv, VP_VCFG, vcfg);

	/* VP display configuration */
	vcfg = read_vp(priv, VP_DCFG);
	vcfg &= ~VP_DCFG_GV_GAM; /* select gamma RAM for graphics */

	if ((m->flags & DRM_MODE_FLAG_NHSYNC)) {
		vcfg &= ~VP_DCFG_CRT_HSYNC_POL;
	} else {
		vcfg |= VP_DCFG_CRT_HSYNC_POL;
	}
	if ((m->flags & DRM_MODE_FLAG_NVSYNC)) {
		vcfg &= ~VP_DCFG_CRT_VSYNC_POL;
	} else {
		vcfg |= VP_DCFG_CRT_VSYNC_POL;
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

	/* TODO: disable BYP_BOTH in VP's MISC MSR */
	/* TODO: enable GV_GAM in VP's DCFG MSR */

	/* TODO: Geode GX rev 2.1 Spec Update, 1.38:
	 *       if (GV_GAM) {
	 *           GV_GAM = 0,
	 *           wait for next vsync
	 *       }
	 *       change gamma RAM */

	/* copy values to VP's PDR space */
	write_vp(priv, VP_PAR, 0);

	/* todo: regard mode_config.depth */

	for (i = 0; i < LX_LUT_SIZE; i++) {
		/* writes to VP_PDR (palette data @VP_PAR) increase VP_PAR */
		write_vp(priv, VP_PDR,
			 lx_crtc->lut[i].r << 16 |
			 lx_crtc->lut[i].g <<  8 |
			 lx_crtc->lut[i].b <<  0);
	}
}

static const struct drm_crtc_helper_funcs lx_crtc_helper_funcs = {
	.dpms = lx_crtc_dpms, /* if (ON) update_scanout; commit */
	.mode_fixup = lx_crtc_mode_fixup, /* return 1 */
	.mode_set = lx_crtc_mode_set, /* update (omap_)crtc->info w/ adjusted_mode && update_scanout */
	.prepare = lx_crtc_prepare, /* .dpms(DRM_MODE_DPMS_OFF) */
	.commit = lx_crtc_commit, /* .dpms(DRM_MODE_DPMS_ON) */
	.mode_set_base = lx_crtc_mode_set_base, /* update_scanout; commit */
	.load_lut = lx_crtc_load_lut,
#if 0
	.disable = NULL, /* more explicit than dpms */
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
	}

	drm_crtc_helper_add(crtc, &lx_crtc_helper_funcs);
}

/* ========================================================================== */

static void lx_connector_encoder_init(struct drm_device *dev,
				      enum lx_connectors connector_id)
{
	enum lx_encoders encoder_id = connector_best_encoders[connector_id];
	struct drm_connector *connector;
	struct drm_encoder *encoder;

	connector = lx_connector_init(dev, connector_id);
	/* the encoder may already be initialized... */
	encoder = lx_encoder_init(dev, encoder_id);

	encoder->possible_crtcs = BIT_MASK(LX_CRTC_GRAPHIC);

	drm_mode_connector_attach_encoder(connector, encoder);
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
	dev->mode_config.fb_base = priv->vmem_phys;

	/* TODO: create & attach drm_device specific drm props */

	/* init the ids to an invalid number, so one-time initialization may
	 * proceed */
	for (i=0; i<LX_NUM_CONNECTORS; i++)
		priv->connectors[i].id = LX_NUM_CONNECTORS;
	for (i=0; i<LX_NUM_ENCODERS; i++)
		priv->encoders[i].id = LX_NUM_ENCODERS;
	for (i=0; i<LX_NUM_CRTCS; i++)
		priv->crtcs[i].id = LX_NUM_CRTCS;

	lx_crtc_init(dev, LX_CRTC_GRAPHIC);

	/* Init connectors and their corresponding 'best' encoders */
	lx_connector_encoder_init(dev, LX_CONNECTOR_VGA);/*
	lx_connector_init(dev, LX_CONNECTOR_LVDS);
	lx_connector_init(dev, LX_CONNECTOR_COMPANION);
	lx_connector_init(dev, LX_CONNECTOR_VOP);*/

	/* Additional, 'non-best' encoders:
	lx_encoder_init(dev, LX_ENCODER_TVDAC);
	drm_mode_connector_attach_encoder(&priv->connectors[LX_CONNECTOR_VGA],
					  &priv->encoders[LX_ENCODER_TVDAC]);
	*/

	/* TODO: create & attach connector/encoder specific drm props */

	lx_fbdev_init(dev);

	drm_kms_helper_poll_init(dev);
}

static void lx_modeset_cleanup(struct drm_device *dev) {
	drm_kms_helper_poll_disable(dev);
	drm_kms_helper_poll_fini(dev);
	lx_fbdev_fini(dev);
	drm_mode_config_cleanup(dev);
	/* delete DDC i2c_adapter (if available) */
	lx_ddc_cleanup(dev);
}


/* The following memory regions should be usable (see DC_GLIU0_MEM_OFFSET, all
 * within a single 16 MB sized, 1 MB aligned memory region):
 * uncompressed frame buffer:
 *   - size dictated by resolution / depth (max. 44.2 MB for 1920x1440x32
 *     using 1/2 scaling filter, 11 MB w/o filter)
 *   - alignment: none (via DC_FB_ST_OFFSET)
 * compressed display buffer (optional)
 * cursor buffer
 * color cursor buffer
 * video buffer(s)
 * 
 * others (addressing w/o GLIU0_MEM_OFFSET):
 * command buffer:
 *   - max. size: 16 MB
 *   - alignment: 1 MB (via GP's GLD_MSR_CONFIG)
 */

/* TODO: what's this needed for? */
#define DRM_FILE_PAGE_OFFSET (0x100000000ULL >> PAGE_SHIFT)

static struct ttm_backend * lx_bo_create_ttm_backend_entry(
		struct ttm_bo_device *bdev)
{
	return NULL;
}

static int lx_bo_invalidate_caches(struct ttm_bo_device *bdev, uint32_t flags)
{
	return 0;
}

static int lx_bo_init_mem_type(struct ttm_bo_device *bdev, uint32_t type,
			       struct ttm_mem_type_manager *man)
{
	struct lx_mman *mman = container_of(bdev, struct lx_mman, bdev);
	struct lx_priv *priv = container_of(mman, struct lx_priv, mman);

	switch (type) {
	case TTM_PL_SYSTEM:
		/* System memory */
		man->flags = TTM_MEMTYPE_FLAG_MAPPABLE;
		man->available_caching = TTM_PL_MASK_CACHING;
		man->default_caching = TTM_PL_FLAG_CACHED;
		break;
	case TTM_PL_VRAM:
		man->flags = TTM_MEMTYPE_FLAG_FIXED |
			     TTM_MEMTYPE_FLAG_MAPPABLE;
		man->available_caching = TTM_PL_FLAG_UNCACHED |
					 TTM_PL_FLAG_WC;
		man->default_caching = TTM_PL_FLAG_WC;
		man->gpu_offset = priv->vmem_phys;
		man->func = &ttm_bo_manager_func;
		break;
	default:
		DRM_ERROR("Unsupported memory type %u\n", (unsigned)type);
		return -EINVAL;
	}

	return 0;
}

static void
lx_bo_evict_flags(struct ttm_buffer_object *bo, struct ttm_placement *pl)
{
}

static int lx_bo_move(struct ttm_buffer_object *bo, bool evict, bool intr,
		      bool no_wait_reserve, bool no_wait_gpu,
		      struct ttm_mem_reg *new_mem)
{
	return 0;
}

static int lx_bo_verify_access(struct ttm_buffer_object *bo, struct file *filp)
{
	return 0;
}

static int lx_ttm_io_mem_reserve(struct ttm_bo_device *bdev,
				 struct ttm_mem_reg *mem)
{
	return 0;
}

static void lx_ttm_io_mem_free(struct ttm_bo_device *bdev,
			       struct ttm_mem_reg *mem)
{
}

static int lx_ttm_fault_reserve_notify(struct ttm_buffer_object *bo)
{
	return 0;
}

static struct ttm_bo_driver lx_bo_driver = {
	.create_ttm_backend_entry = lx_bo_create_ttm_backend_entry,
	.invalidate_caches = lx_bo_invalidate_caches,
	.init_mem_type = lx_bo_init_mem_type,
	.evict_flags = lx_bo_evict_flags,
	.move = lx_bo_move,
	.verify_access = lx_bo_verify_access,
	.fault_reserve_notify = lx_ttm_fault_reserve_notify,
	.io_mem_reserve = lx_ttm_io_mem_reserve,
	.io_mem_free = lx_ttm_io_mem_free,
};

static int generic_ttm_mem_global_init(struct drm_global_reference *ref)
{
	return ttm_mem_global_init(ref->object);
}

static void generic_ttm_mem_global_release(struct drm_global_reference *ref)
{
	ttm_mem_global_release(ref->object);
}

static void generic_ttm_global_release(struct lx_mman *ttm)
{
	if (ttm->mem_global_ref.release == NULL)
		return;

	drm_global_item_unref(&ttm->bo_global_ref.ref);
	drm_global_item_unref(&ttm->mem_global_ref);
	ttm->mem_global_ref.release = NULL;
}

static int generic_ttm_global_init(struct lx_mman *ttm) {
	struct drm_global_reference *global_ref;
	int r;

	global_ref = &ttm->mem_global_ref;
	global_ref->global_type = DRM_GLOBAL_TTM_MEM;
	global_ref->size = sizeof(struct ttm_mem_global);
	global_ref->init = generic_ttm_mem_global_init;
	global_ref->release = generic_ttm_mem_global_release;
	r = drm_global_item_ref(global_ref);
	if (r != 0) {
		DRM_ERROR("Failed setting up TTM memory accounting "
			  "subsystem.\n");
		return r;
	}

	ttm->bo_global_ref.mem_glob = ttm->mem_global_ref.object;
	global_ref = &ttm->bo_global_ref.ref;
	global_ref->global_type = DRM_GLOBAL_TTM_BO;
	global_ref->size = sizeof(struct ttm_bo_global);
	global_ref->init = ttm_bo_global_init;
	global_ref->release = ttm_bo_global_release;
	r = drm_global_item_ref(global_ref);
	if (r != 0) {
		DRM_ERROR("Failed setting up TTM BO subsystem.\n");
		drm_global_item_unref(&ttm->mem_global_ref);
		return r;
	}

	return 0;
}

/* copied from lxfb_ops.c */
static unsigned int lx_vmem_size(void)
{
	unsigned int val;

	if (!cs5535_has_vsa2()) {
		uint32_t hi, lo;

		/* The number of pages is (PMAX - PMIN)+1 */
		rdmsr(MSR_GLIU_P2D_RO0, lo, hi);

		/* PMAX */
		val = ((hi & 0xff) << 12) | ((lo & 0xfff00000) >> 20);
		/* PMIN */
		val -= (lo & 0x000fffff);
		val += 1;

		/* The page size is 4k */
		return (val << 12);
	}

	/* The frame buffer size is reported by a VSM in VSA II */
	/* Virtual Register Class    = 0x02                     */
	/* VG_MEM_SIZE (1MB units)   = 0x00                     */

	outw(VSA_VR_UNLOCK, VSA_VRC_INDEX);
	outw(VSA_VR_MEM_SIZE, VSA_VRC_INDEX);

	val = (unsigned int)(inw(VSA_VRC_DATA)) & 0xFE;
	return (val << 20);
}

static int lx_map_video_memory(struct pci_dev *pdev, struct lx_priv *priv) {
	static const char *region_names[] = {
		"lx-vram", /* shared video memory set aside by the bios */
		"lx-gp", /* graphics processor MSRs */
		"lx-dc", /* display controller MSRs */
		"lx-vp", /* video processor MSRs */
		/* the unused BAR 4 points to MSRs for the video input port */
	};
	unsigned i;
	int ret;
/*
	ret = pci_enable_device(pdev);
	if (ret)
		return ret;
*/
	for (i=0; i<ARRAY_SIZE(region_names); i++) {
		ret = pci_request_region(pdev, i, region_names[i]);
		if (ret) {
			DRM_ERROR("failed requesting PCI region %d: %d\n", i, ret);
			goto failed_req;
		}
	}

	priv->vmem_phys = pci_resource_start(pdev, 0); /* physical address */
	/* rather I/O address, since this space really lies in the RAM, the
	 * mapping is done via MSR_GLIU_P2D_RO0 */
	/* pci_resource_len(pdev, 0) is wrong. only half this value seems to
	 * reside in physical RAM */
	priv->vmem_size = lx_vmem_size();

	ret = -ENOMEM;

	/* todo: mark this region as write-combined (uncacheable)
	 * (see LX Processor Data Book, Table 5-17, p. 170) */
#if 0
	ret = generic_ttm_global_init(&priv->mman);

	ret = ttm_bo_device_init(&priv->mman.bdev,
				 priv->mman.bo_global_ref.ref.object,
				 &lx_bo_driver, DRM_FILE_PAGE_OFFSET,
				 false); /* TODO: really need dma32? */

	DRM_DEBUG_DRIVER("ttm_bo_init_mm with size %u MB\n", priv->vmem_size >> 20);

	ret = ttm_bo_init_mm(&priv->mman.bdev, TTM_PL_VRAM,
			     priv->vmem_size >> PAGE_SHIFT);
#endif
	/* XXX: rather use pci_iomap? */
	priv->gp_regs = pci_ioremap_bar(pdev, 1);
	if (priv->gp_regs == NULL)
		goto failed_map_gp;

	priv->dc_regs = pci_ioremap_bar(pdev, 2);
	if (priv->dc_regs == NULL)
		goto failed_map_dc;

	priv->vp_regs = pci_ioremap_bar(pdev, 3);
	if (priv->vp_regs == NULL)
		goto failed_map_vp;

	/* need to unlock the MSRs for WR with a magic value */
	write_dc(priv, DC_UNLOCK, DC_UNLOCK_UNLOCK);
	/* base address for graphics memory region, alignment: 16MB */
	write_dc(priv, DC_GLIU0_MEM_OFFSET, priv->vmem_phys & 0xFF000000);
	write_dc(priv, DC_UNLOCK, DC_UNLOCK_LOCK);

	DRV_INFO("%d KB of video memory at linear address 0x%x\n",
		 priv->vmem_size / 1024, priv->vmem_phys);

	return 0;

failed_map_vp:
	pci_iounmap(pdev, priv->dc_regs);
failed_map_dc:
	pci_iounmap(pdev, priv->gp_regs);
failed_map_gp:
	i = ARRAY_SIZE(region_names);
failed_req:
	while (i--)
		pci_release_region(pdev, i);
	pci_disable_device(pdev);
	return ret;
}

static void lx_unmap_video_memory(struct pci_dev *pdev, struct lx_priv *priv) {
	pci_iounmap(pdev, priv->vp_regs);
	pci_iounmap(pdev, priv->dc_regs);
	pci_iounmap(pdev, priv->gp_regs);
	// iounmap(priv->vmem_virt);
#if 0
	ttm_bo_clean_mm(&priv->mman.bdev, TTM_PL_VRAM);
	ttm_bo_device_release(&priv->mman.bdev);
	generic_ttm_global_release(&priv->mman);
#endif
	pci_release_region(pdev, 3);
	pci_release_region(pdev, 2);
	pci_release_region(pdev, 1);
	pci_release_region(pdev, 0);
	// pci_disable_device(pdev);
}

static int lx_driver_device_is_agp(struct drm_device *dev) {
	return 0;
}

/** @flags: set in pci id list, not needed here */
int lx_driver_load(struct drm_device *dev, unsigned long flags) {
	struct lx_priv *priv;
	u32 lo, hi;
	int ret, i;

	/* TODO: setup performance counters 0 and 1 */

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ddev = dev;
	priv->pdev = dev->pdev;

	dev->dev_private = priv;

	ret = lx_map_video_memory(dev->pdev, priv);
	if (ret) {
		DRV_ERR("failed mapping video memory or controller registers\n");
		goto failed_map_video_memory;
	}

	lx_modeset_init(dev);

	// drm_irq_install(dev);
	// drm_vblank_init(dev, LX_NUM_CRTCS);

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

	return 0;

failed_map_video_memory:
	kfree(priv);
	return ret;
}

int lx_driver_unload(struct drm_device *dev) {
	struct lx_priv *priv = dev->dev_private;
	// drm_vblank_cleanup(dev);
	// drm_irq_uninstall(dev);
	lx_modeset_cleanup(dev);
	lx_unmap_video_memory(dev->pdev, priv);
	kfree(priv);
	return 0;
}

/** Gets the current scanline value from the scanout logic.
 * @return negative when the value is currently transitioning due to being
 *         unbuffered, a valid vpos otherwise */
int lx_get_scanout_vpos(struct lx_priv *priv) {
	int v = read_dc(priv, DC_LINE_CNT);
	int w = read_dc(priv, DC_LINE_CNT);
	return (v == w) ? v : -1;
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
 * RETURNS
 * Zero on success, appropriate errno if the given @crtc's vblank
 * interrupt cannot be enabled.
 */
static int lx_driver_enable_vblank(struct drm_device *dev, int crtc) {
	struct lx_priv *priv = dev->dev_private;
	struct drm_display_mode *mode = &priv->crtcs[crtc].base.hwmode;
	u32 dc_irq_filt_ctl;
	u32 dc_irq;

	/* set line counter */
	dc_irq_filt_ctl = read_dc(priv, DC_IRQ_FILT_CTL);
	dc_irq_filt_ctl &= ~(((1 << 11) - 1) << 16); /* clear bits [26:16] */
	/* set IRQ trigger value there */
	dc_irq_filt_ctl |= mode->crtc_vblank_start << 16;
	write_dc(priv, DC_IRQ_FILT_CTL, dc_irq_filt_ctl);

	/* enable line count interrupt */
	dc_irq = read_dc(priv, DC_IRQ);
	dc_irq &= ~LX_IRQ_STATUS_MASK; /* don't clear any interrupt status bits */
	dc_irq &= ~DC_IRQ_MASK; /* unmask line cnt interrupt */
	write_dc(priv, DC_IRQ, dc_irq);

	DRM_DEBUG_DRIVER("crtc: %d\n", crtc);
	return 0;
}

/**
 * disable_vblank - disable vblank interrupt events
 * @dev: DRM device
 * @crtc: which irq to enable
 *
 * Disable vblank interrupts for @crtc.  If the device doesn't have
 * a hardware vblank counter, this routine should be a no-op, since
 * interrupts will have to stay on to keep the count accurate.
 */
static void lx_driver_disable_vblank(struct drm_device *dev, int crtc) {
	struct lx_priv *priv = dev->dev_private;
	u32 dc_irq;

	if (crtc < 0 || crtc >= LX_NUM_CRTCS)
		return;

	dc_irq = read_dc(priv, DC_IRQ);
	dc_irq &= ~LX_IRQ_STATUS_MASK; /* don't clear any interrupt status bits */
	dc_irq |= DC_IRQ_MASK; /* mask line cnt interrupt */
	write_dc(priv, DC_IRQ, dc_irq);

	DRM_DEBUG_DRIVER("crtc: %d\n", crtc);
}

/* --------------------------------------------------------------------------
 * Stubs
 * -------------------------------------------------------------------------- */

static irqreturn_t lx_driver_irq_handler(DRM_IRQ_ARGS) {
	struct drm_device *dev = arg;
	struct lx_priv *priv = dev->dev_private;
	irqreturn_t ret = IRQ_NONE;
	u32 dc_irq, gp_irq;

	 {
		dc_irq = read_dc(priv, DC_IRQ);

		gp_irq = read_gp(priv, GP_INT_CNTRL);
		// write_gp(priv, GP_INT_CNTRL, gp_irq); /* clear GP IRQ status */

		DRM_DEBUG_DRIVER("IRQ: dc: %08x, gp: %08x\n", dc_irq, gp_irq);
#if 0
		if (!(dc_irq & LX_IRQ_STATUS_MASK) &&
		    !(gp_irq & LX_IRQ_STATUS_MASK))
			return ret;
#endif
		if (dc_irq & DC_IRQ_STATUS) {
			DRM_DEBUG_DRIVER("IRQ: vline\n");
			drm_handle_vblank(dev, LX_CRTC_GRAPHIC);
			ret = IRQ_HANDLED;

			/* TODO: process any page flips, etc. */
		}

		if (gp_irq & GP_INT_IDLE_STATUS) {
		}
		if (gp_irq & GP_INT_CMD_BUF_EMPTY_STATUS) {
		}

		/* clear DC IRQ status */
		write_dc(priv, DC_IRQ, DC_IRQ_STATUS | ~LX_IRQ_STATUS_MASK);
	}
	return ret;
}

static void lx_driver_irq_preinstall(struct drm_device *dev) {
	struct lx_priv *priv = dev->dev_private;

	DRM_DEBUG_DRIVER("\n");
	/* TODO: disable all interrupts */

	/* DC: line cnt & vip vsync loss interrupts */
	write_dc(priv, DC_IRQ, ~LX_IRQ_STATUS_MASK);
	/* GP: idle & cmd buffer empty */
	write_gp(priv, GP_INT_CNTRL, ~LX_IRQ_STATUS_MASK);
}

static int lx_driver_irq_postinstall(struct drm_device *dev) {
	DRM_DEBUG_DRIVER("\n");
	/* TODO: enable interrupts */

	return 0;
}

static void lx_driver_irq_uninstall(struct drm_device *dev) {
	DRM_DEBUG_DRIVER("\n");
	lx_driver_irq_preinstall(dev);
}

static int lx_driver_open(struct drm_device *dev, struct drm_file *file) {
	DRM_DEBUG_DRIVER("\n");
	file->driver_priv = NULL;
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
	DRM_DEBUG_DRIVER("\n");
	vga_switcheroo_process_delayed_switch();
}

static void lx_driver_preclose(struct drm_device *dev, struct drm_file *file)
{
	DRM_DEBUG_DRIVER("\n");
}

static void lx_driver_postclose(struct drm_device *dev, struct drm_file *file)
{
	DRM_DEBUG_DRIVER("\n");
}

static struct drm_ioctl_desc lx_ioctls[] = {
};

static struct drm_driver driver = {
	.driver_features	= DRIVER_HAVE_IRQ | DRIVER_IRQ_SHARED |
				  DRIVER_MODESET,
	.dev_priv_size		= sizeof(struct lx_priv),
	.load			= lx_driver_load,
	.unload			= lx_driver_unload,
	.device_is_agp		= lx_driver_device_is_agp,

	/* These need testing ... */
	.irq_preinstall		= lx_driver_irq_preinstall,
	.irq_postinstall	= lx_driver_irq_postinstall,
	.irq_uninstall		= lx_driver_irq_uninstall,
	.irq_handler		= lx_driver_irq_handler,
	.enable_vblank		= lx_driver_enable_vblank,
	.disable_vblank		= lx_driver_disable_vblank,
	.get_vblank_counter	= drm_vblank_count,

	/* These are stubs ... */
	.open			= lx_driver_open,
	.lastclose		= lx_driver_lastclose,
	.preclose		= lx_driver_preclose,
	.postclose		= lx_driver_postclose,
	.ioctls			= lx_ioctls,
	.num_ioctls		= ARRAY_SIZE(lx_ioctls),
#if 0
	.reclaim_buffers	= drm_core_reclaim_buffers,
	.get_scanout_position   = lx_driver_get_scanout_position,
	// .dma_quiescent		= lx_driver_dma_quiescent,
	// .dma_ioctl		= lx_driver_dma_ioctl,
#endif
	.fops = {
		.owner		= THIS_MODULE,
		.open		= drm_open,
		.release	= drm_release,
		.unlocked_ioctl	= drm_ioctl,
		.mmap		= drm_mmap,
		.fasync		= drm_fasync,
		.poll		= drm_poll,
		.read		= drm_read,
		.llseek		= noop_llseek,
	},
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
	.patchlevel		= DRIVER_PATCHLEVEL,
};

static int __devinit lx_pci_probe(struct pci_dev *pdev,
				  const struct pci_device_id *ent)
{
	DRM_DEBUG_DRIVER("\n");
	return drm_get_pci_dev(pdev, ent, &driver);
}

static void __devexit lx_pci_remove(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	DRM_DEBUG_DRIVER("\n");
	drm_put_dev(dev);
}


static struct pci_driver lx_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = pci_ids,
	.probe = lx_pci_probe,
	.remove = lx_pci_remove,
};

static int __init lx_init(void) {
	// driver.num_ioctls = lx_max_ioctl;
	return drm_pci_init(&driver, &lx_pci_driver);
}

static void __exit lx_exit(void) {
	drm_pci_exit(&driver, &lx_pci_driver);
}

module_init(lx_init);
module_exit(lx_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(__stringify(DRIVER_MAJOR) "."
	       __stringify(DRIVER_MINOR) "."
	       __stringify(DRIVER_PATCHLEVEL));
