
#ifndef _LX_H_
#define _LX_H_

#include "drm_fb_helper.h"

#define DRIVER_AUTHOR		"Franz Brauße"

#define DRIVER_NAME		"lx"
#define DRIVER_DESC		"Geode LX KMS"
#define DRIVER_DATE		"20110906"

#define DRIVER_MAJOR		0
#define DRIVER_MINOR		0
#define DRIVER_PATCHLEVEL	1

#define DRV_INFO(fmt, ...)					\
	DRM_INFO(DRIVER_NAME ": " fmt, ##__VA_ARGS__)
#define DRV_ERR							\
	DRM_ERROR

#include <ttm/ttm_bo_api.h>
#include <ttm/ttm_bo_driver.h>
#include <ttm/ttm_placement.h>
#include <ttm/ttm_module.h>
#include <ttm/ttm_page_alloc.h>

/* TODO:
 * - connector / output {DPMS property,subpixel?}
 * - argb-cursor really needs to be 64x64 sized
 * - DRM_MODE_PROP_{IMMUTABLE,RANGE,ENUM}
 * - don't return handle 0 for anything (maybe use for fbcon's fb)
 * - set mode_config.{max_cursor_{width,height},preferred_depth,prefer_shadow}
 */


/* VGA <- DAC 24 bit (video/graphics input)
 * LVDS <- LVDS 24 bit (video/graphics input) several bit depths
 * COMPANION <- other (bypass) 24 bit + 3 bit (delay) (graphics input)
 * TV <- TV (video/graphics input) several bit depths?
 * VOP <- (video/graphics input) */

#define LX_NUM_CONNECTORS	3
enum lx_connectors {
	LX_CONNECTOR_VGA,
	LX_CONNECTOR_LVDS,
	LX_CONNECTOR_VOP,
};

#define LX_NUM_ENCODERS		2
enum lx_encoders {
	LX_ENCODER_DAC, /* RGB [CRT]*/
	LX_ENCODER_PANEL, /* digital RGB [Panel] */
	// LX_ENCODER_VOP, /* YUV [VOP] */
	// LX_ENCODER_BYPASS, /* digital RGB w/o mixing/overlay/alpha-blending or gamma correction -> to companion chip [bypass] */
};

/* encoders: 3-bit value: abc
 * c:
 * ~
 * 0: primary analog CRT
 * 1: primary digital TFT (+ optionally cloned CRT)
 * 
 * ab:
 * ~~
 * 00: normal operation: either CRT only or TFT + optionally cloned CRT
 * 01: legacy mode: optionally cloned CRT + either RGB_565 or RGB 888 (via companion chip, if connected)
 * 10: debug mode: debug signals on DRGB_565 and/or RGB_323 on the low bits (+ optionally cloned CRT?)
 * 11: alternate configuration: VOP / "DRGB"
 *
 * this suggests the following
 *   connectors: VGA, LVDS
 *   encoders: DAC, {PANEL, VOP, DRGB}
 */

/* video: scaling + mixer -> output also to VOP
 * graphics: mixer -> output also to VOP
 * graphics: -> output not to VOP */

#define LX_NUM_CRTCS		2
enum lx_crtcs {
	LX_CRTC_GRAPHIC,
	LX_CRTC_VIDEO,
};


struct lx_bo {
	struct drm_mm_node *node;
	drm_local_map_t *map; /* for drm_mmap */
	unsigned width;
	unsigned height;
	unsigned bpp;
	unsigned pitch;
	int id;
};

#define to_lx_connector(connector)	\
		container_of(connector, struct lx_connector, base)
struct lx_connector {
	struct drm_connector base;

	enum lx_connectors id;

	unsigned max_width, max_height, max_hz;
	struct edid *edid;
};

#define to_lx_encoder(encoder)		\
		container_of(encoder, struct lx_encoder, base)
struct lx_encoder {
	struct drm_encoder base;

	enum lx_encoders id;

	unsigned enabled : 1;
};

/* 256 x 24 bit palette / gamma CLUT + 5 x 24 bit overlay colors */
/* The DC's palette RAM is used for graphics and VP's palette RAM is unused for
 * now */
#define LX_LUT_PAL_GAMMA_START	0x000
#define LX_LUT_PAL_GAMMA_STOP	0x100
/* cursor and icon overlay */
#define LX_LUT_CURSOR_COL0	0x100 /* AND mask 0, XOR mask 0 */
#define LX_LUT_CURSOR_COL1	0x101 /* AND mask 0, XOR mask 1 */
#define LX_LUT_ICON_COL0	0x102 /* AND mask 0, XOR mask 0 */
#define LX_LUT_ICON_COL1	0x103 /* AND mask 0, XOR mask 1 */
#define LX_LUT_ICON_BORDER	0x104 /* AND mask 1, XOR mask 1 */
#define LX_LUT_SIZE		0x105

#define to_lx_crtc(crtc)	container_of(crtc, struct lx_crtc, base)
struct lx_crtc {
	struct drm_crtc base;

	enum lx_crtcs id;

	/* non-null when there is a flip pending */
	struct drm_pending_vblank_event *event;

	/* max. 24bpp supported by the hw, no need to store higher precision */
	struct lx_rgb {
		u8 b;
		u8 g;
		u8 r;
	} lut[LX_LUT_SIZE];

	struct lx_bo *cursor_bo;

	unsigned flip_pending : 1;
};

struct lx_priv;

#define to_lx_fb(fb)		container_of(fb, struct lx_fb, base)
#define helper_to_lx_fb(helper)	container_of(helper, struct lx_fb, helper)
struct lx_fb {
	struct drm_framebuffer base;
	struct drm_fb_helper helper;
	struct lx_priv *priv;
	struct lx_bo *bo;
};

struct lx_priv {
	struct drm_device *ddev;
	struct pci_dev *pdev;

	resource_size_t vmem_phys; /* linear address for framebuffer & off screen memory */
	resource_size_t vmem_size;
	// void __iomem *vmem_virt; /* virtual address for framebuffer & off screen memory */
	// drm_local_map_t *vmem;
	drm_local_map_t *gp; /* graphics processor regs */
	drm_local_map_t *dc; /* display controller regs */
	drm_local_map_t *vp; /* video processor regs */

	struct i2c_adapter *ddc;

	struct lx_connector connectors[LX_NUM_CONNECTORS];
	struct lx_encoder   encoders[LX_NUM_ENCODERS];
	struct lx_crtc      crtcs[LX_NUM_CRTCS];

	/* used by irq-handler and get_vblank_timestamp:
	 * temporary to keep the vblank timestamps as accurate as possible */
	struct timeval      last_vblank;

	/* singleton fbdev */
	struct lx_fb        *fb;
	unsigned pan_x, pan_y;

	struct lx_mman {
		struct drm_global_reference mem_global_ref;
		struct ttm_bo_global_ref bo_global_ref;
		struct ttm_bo_device bdev;

		struct drm_mm mm;
	} mman;
};

/* part of lx_i2c.c */
extern bool lx_ddc_probe(struct i2c_adapter *ddc);
extern int  lx_ddc_init(struct drm_device *dev);
extern void lx_ddc_cleanup(struct drm_device *dev);


#define LX_DOTPLL_MINKHZ		15000
#define LX_MODE_MAX_VFREQ		100    /* in Hz */
#define LX_MODE_FREQ_TOL		4000   /* in kHz */

/* used for drm_display_mode's private_flags */
#define LX_MODE_PFLAG_DOTPLL_MASK	0x17fff


/* The IRQ registers of the LX are 32 bits wide, the upper half of which are
 * status bits (1: interrupt requested; writing 1 clears status bit, 0 does
 * nothing) and the lower half are the corresponding mask-bits (1: IRQ disabled,
 * though the status flag still may be enabled but no IRQ will be triggered). */
#define LX_IRQ_STATUS_SHIFT		16
#define LX_IRQ_STATUS_MASK		(~0U << LX_IRQ_STATUS_SHIFT)

/* These form the upper six bytes of the MSR address for the respective
 * component for outgoing r/w requests from the CPU core */
#define LX_MC				0x20 /* memory controller */
#define LX_GLCP				0x4c /* GeodeLink control processor */
#define LX_VP				0x48 /* video processor */
#define LX_GP				0xa0 /* graphics processor */
#define LX_MSR(gldev, adr)		((gldev) << 24 | (adr & 0xffff))

/* addresses for all GeodeLink devices */
#define LX_GLD_MSR_CAP			0x2000
#define LX_GLD_MSR_CONFIG		0x2001
#define LX_GLD_MSR_SMI			0x2002
#define LX_GLD_MSR_ERROR		0x2003
#define LX_GLD_MSR_PM			0x2004

/* device specific MSRs */
#define MSR_VP_DIAG			LX_MSR(LX_VP  , 0x2010)
#define MSR_GLCP_DAC			LX_MSR(LX_GLCP, 0x0023)
#define MSR_DC_SPARE_MSR		0x80000011

#define MSR_GLCP_DOTPLL_HI_DIV4		(1 << 16)

#define MSR_GLCP_DOTPLL_LO_LOCK		(1 << 25)	/* r/o */
#define MSR_GLCP_DOTPLL_LO_HALFPIX	(1 << 24)
#define MSR_GLCP_DOTPLL_LO_BYPASS	(1 << 15)
#define MSR_GLCP_DOTPLL_LO_DOTRESET	(1 << 0)

#define MSR_VP_GLD_MSR_CONFIG_FPC	(1 << 15)
#define MSR_VP_GLD_MSR_CONFIG_FMT_MASK	(7 << 3)
#define MSR_VP_GLD_MSR_CONFIG_FMT_CRT	(0 << 3)
#define MSR_VP_GLD_MSR_CONFIG_FMT_FP	(1 << 3)
#define MSR_VP_GLD_MSR_CONFIG_FMT_CRT_DBG	(4 << 3)
#define MSR_VP_GLD_MSR_CONFIG_FMT_VOP	(6 << 3)
#define MSR_VP_GLD_MSR_CONFIG_FMT_DRGB	(7 << 3)

#define MSR_DC_SPARE_MSR_DIS_CFIFO_HGO	(1 << 11)	/* undocumented */
#define MSR_DC_SPARE_MSR_VFIFO_ARB_SEL	(1 << 10)	/* undocumented */
#define MSR_DC_SPARE_MSR_WM_LPEN_OVRD	(1 << 9)	/* undocumented */
#define MSR_DC_SPARE_MSR_LOAD_WM_LPEN_M	(1 << 8)	/* undocumented */
#define MSR_DC_SPARE_MSR_DIS_INIT_V_PRI	(1 << 7)	/* undocumented */
#define MSR_DC_SPARE_MSR_DIS_VIFO_WM	(1 << 6)
#define MSR_DC_SPARE_MSR_DIS_CWD_CHECK	(1 << 5)	/* undocumented */
#define MSR_DC_SPARE_MSR_PIX8_PAN_FIX	(1 << 4)	/* undocumented */
#define MSR_DC_SPARE_MSR_FIRST_REQ_MASK	(1 << 1)	/* undocumented */

/* Graphics Processor registers (table 6-29 from the data book) */
enum gp_registers {
	GP_DST_OFFSET = 0,
	GP_SRC_OFFSET,
	GP_STRIDE,
	GP_WID_HEIGHT,

	GP_SRC_COLOR_FG,
	GP_SRC_COLOR_BG,
	GP_PAT_COLOR_0,
	GP_PAT_COLOR_1,

	GP_PAT_COLOR_2,
	GP_PAT_COLOR_3,
	GP_PAT_COLOR_4,
	GP_PAT_COLOR_5,

	GP_PAT_DATA_0,
	GP_PAT_DATA_1,
	GP_RASTER_MODE,
	GP_VECTOR_MODE,

	GP_BLT_MODE,
	GP_BLT_STATUS,
	GP_HST_SRC,
	GP_BASE_OFFSET,

	GP_CMD_TOP,
	GP_CMD_BOT,
	GP_CMD_READ,
	GP_CMD_WRITE,

	GP_CH3_OFFSET,
	GP_CH3_MODE_STR,
	GP_CH3_WIDHI,
	GP_CH3_HSRC,

	GP_LUT_INDEX,
	GP_LUT_DATA,
	GP_INT_CNTRL, /* 0x78 */
};

#define GP_BLT_MODE_CP                  (1 << 11) /* interrupt when BLT done */
#define GP_BLT_MODE_TH                  (1 << 10) /* begin BLT at next VBLANK */
#define GP_BLT_MODE_X_DEC               (1 << 9) /* negative increment for x */
#define GP_BLT_MODE_X_INC               (0 << 9) /* positive increment for x */
#define GP_BLT_MODE_Y_DEC               (1 << 8) /* negative increment for y */
#define GP_BLT_MODE_Y_INC               (0 << 8) /* positive increment for y */
#define GP_BLT_MODE_SM_MONO_PACK        (2 << 6) /* src is byte-packed mono */
#define GP_BLT_MODE_SM_MONO_UNPACK      (1 << 6) /* src is unpacked mono */
#define GP_BLT_MODE_SM_COL              (0 << 6) /* src is color bitmap */
#define GP_BLT_MODE_DR                  (1 << 2) /* dst data from fb required */
#define GP_BLT_MODE_SR_HOST             (2 << 0) /* src data from GP_HST_SRC */
#define GP_BLT_MODE_SR_FB               (1 << 0) /* src data from framebuffer */
#define GP_BLT_MODE_SR_NONE             (0 << 0) /* no src data required */

#define GP_BLT_STATUS_UF                (1 << 7) /* channel 3 underflow */
#define GP_BLT_STATUS_RP                (1 << 6) /* GP has read pending */
#define GP_BLT_STATUS_EH                (1 << 5) /* host data expected */
#define GP_BLT_STATUS_CE                (1 << 4) /* cmd buf empty */
#define GP_BLT_STATUS_SHE               (1 << 3) /* src host FIFO half empty */
#define GP_BLT_STATUS_PP                (1 << 2) /* primitive pending */
#define GP_BLT_STATUS_IN                (1 << 1) /* GP interrupt signal */
#define GP_BLT_STATUS_PB                (1 << 0) /* primitive busy */

#define GP_INT_IDLE_STATUS		(1 << 17)
#define GP_INT_CMD_BUF_EMPTY_STATUS	(1 << 16)
#define GP_INT_IDLE_MASK		(1 << 1)
#define GP_INT_CMD_BUF_EMPTY_MASK	(1 << 0)

/* Display Controller registers (table 6-47 from the data book) */
enum dc_registers {
	DC_UNLOCK = 0,
	DC_GENERAL_CFG,
	DC_DISPLAY_CFG,
	DC_ARB_CFG,

	DC_FB_ST_OFFSET,
	DC_CB_ST_OFFSET,
	DC_CURS_ST_OFFSET,
	DC_RSVD_0, /* DC_ICON_ST_OFFSET? */

	DC_VID_Y_ST_OFFSET,
	DC_VID_U_ST_OFFSET,
	DC_VID_V_ST_OFFSET,
	DC_DV_TOP,

	DC_LINE_SIZE,
	DC_GFX_PITCH,
	DC_VID_YUV_PITCH,
	DC_RSVD_1,

	DC_H_ACTIVE_TIMING,
	DC_H_BLANK_TIMING,
	DC_H_SYNC_TIMING,
	DC_RSVD_2,

	DC_V_ACTIVE_TIMING,
	DC_V_BLANK_TIMING,
	DC_V_SYNC_TIMING,
	DC_FB_ACTIVE,

	DC_CURSOR_X,
	DC_CURSOR_Y,
	DC_RSVD_3, /* icon x/y? */
	DC_LINE_CNT,

	DC_PAL_ADDRESS,
	DC_PAL_DATA,
	DC_DFIFO_DIAG,
	DC_CFIFO_DIAG,

	DC_VID_DS_DELTA,
	DC_GLIU0_MEM_OFFSET,
	DC_DV_CTL,
	DC_DV_ACCESS,

	DC_GFX_SCALE,
	DC_IRQ_FILT_CTL,
	DC_FILT_COEFF1,
	DC_FILT_COEFF2,

	DC_VBI_EVEN_CTL,
	DC_VBI_ODD_CTL,
	DC_VBI_HOR,
	DC_VBI_LN_ODD,

	DC_VBI_LN_EVEN,
	DC_VBI_PITCH,
	DC_CLR_KEY,
	DC_CLR_KEY_MASK,

	DC_CLR_KEY_X,
	DC_CLR_KEY_Y,
	DC_IRQ,
	DC_RSVD_4,

	DC_RSVD_5,
	DC_GENLK_CTL,
	DC_VID_EVEN_Y_ST_OFFSET,
	DC_VID_EVEN_U_ST_OFFSET,

	DC_VID_EVEN_V_ST_OFFSET,
	DC_V_ACTIVE_EVEN_TIMING,
	DC_V_BLANK_EVEN_TIMING,
	DC_V_SYNC_EVEN_TIMING,	/* 0xec */
};

#define DC_UNLOCK_LOCK			0x00000000
#define DC_UNLOCK_UNLOCK		0x00004758	/* magic value */

#define DC_GENERAL_CFG_FDTY		(1 << 17)
#define DC_GENERAL_CFG_STFM		(1 << 16)
#define DC_GENERAL_CFG_DFHPEL_SHIFT	(12)
#define DC_GENERAL_CFG_DFHPSL_SHIFT	(8)
#define DC_GENERAL_CFG_VGAE		(1 << 7)
#define DC_GENERAL_CFG_DECE		(1 << 6)
#define DC_GENERAL_CFG_CMPE		(1 << 5)
#define DC_GENERAL_CFG_VIDE		(1 << 3)
#define DC_GENERAL_CFG_CLR_CUR		(1 << 2)
#define DC_GENERAL_CFG_CURE		(1 << 1)
#define DC_GENERAL_CFG_DFLE		(1 << 0)

#define DC_DISPLAY_CFG_VISL		(1 << 27)
#define DC_DISPLAY_CFG_PALB		(1 << 25)
#define DC_DISPLAY_CFG_DCEN		(1 << 24)
#define DC_DISPLAY_CFG_VFHPEL_SHIFT	(16)
#define DC_DISPLAY_CFG_VFHPSL_SHIFT	(12)
#define DC_DISPLAY_CFG_16BPP_4444	(2 << 10)
#define DC_DISPLAY_CFG_16BPP_0555	(1 << 10)
#define DC_DISPLAY_CFG_16BPP_0565	(0 << 10)
#define DC_DISPLAY_CFG_DISP_MODE_32BPP	(3 << 8)
#define DC_DISPLAY_CFG_DISP_MODE_24BPP	(2 << 8)
#define DC_DISPLAY_CFG_DISP_MODE_16BPP	(1 << 8)
#define DC_DISPLAY_CFG_DISP_MODE_8BPP	(0 << 8)
#define DC_DISPLAY_CFG_TRUP		(1 << 6)
#define DC_DISPLAY_CFG_VIEN		(1 << 5) /* SMI on vsync/vblank (VISL) */
#define DC_DISPLAY_CFG_VDEN		(1 << 4)
#define DC_DISPLAY_CFG_GDEN		(1 << 3)
#define DC_DISPLAY_CFG_TGEN		(1 << 0)

#define DC_DV_TOP_DV_TOP_EN		(1 << 0)

#define DC_DV_CTL_DV_LINE_SIZE_MASK	(3 << 10)
#define DC_DV_CTL_DV_LINE_SIZE_1K	(0 << 10) /* in bytes */
#define DC_DV_CTL_DV_LINE_SIZE_2K	(1 << 10)
#define DC_DV_CTL_DV_LINE_SIZE_4K	(2 << 10)
#define DC_DV_CTL_DV_LINE_SIZE_8K	(3 << 10)
#define DC_DV_CTL_DV_RANGE_MASK		(3 << 8)
#define DC_DV_CTL_DV_RANGE_2048		(0 << 8) /* in lines */
#define DC_DV_CTL_DV_RANGE_512		(1 << 8)
#define DC_DV_CTL_DV_RANGE_1024		(2 << 8)
#define DC_DV_CTL_DV_RANGE_1536		(3 << 8)
#define DC_DV_CTL_DV_MASK		(1 << 1)
#define DC_DV_CTL_CLEAR_DV_RAM		(1 << 0)

#define DC_IRQ_FILT_CTL_INTERLACE_ADDR	(1 << 28)
#define DC_IRQ_FILT_CTL_ALPHA_FILT_ENA	(1 << 14)
#define DC_IRQ_FILT_CTL_FILT_ENA	(1 << 12)
#define DC_IRQ_FILT_CTL_INTL_EN		(1 << 11)
#define DC_IRQ_FILT_CTL_H_FILT_SEL	(1 << 10)
#define DC_IRQ_FILT_CTL_LINEBUF_REG_EN	(1 << 9) /* rsvd, see bits [30:29] */

#define DC_CLR_KEY_CLR_KEY_EN		(1 << 24)

#if 0 /* the data book says otherwise */
# define DC_IRQ_VIP_VSYNC_IRQ_STATUS	(1 << 21)	/* undocumented? */
# define DC_IRQ_STATUS			(1 << 20)	/* undocumented? */
#else
# define DC_IRQ_VIP_VSYNC_IRQ_STATUS	(1 << 17)
# define DC_IRQ_STATUS			(1 << 16)
#endif
#define DC_IRQ_VIP_VSYNC_IRQ_MASK	(1 << 1)
#define DC_IRQ_MASK			(1 << 0)

#define DC_GENLK_CTL_FLICK_SEL_MASK	(0x0F << 28)
#define DC_GENLK_CTL_ALPHA_FLICK_EN	(1 << 25)
#define DC_GENLK_CTL_FLICK_EN		(1 << 24)
#define DC_GENLK_CTL_GENLK_EN		(1 << 18)


/*
 * Video Processor registers (table 6-71).
 * There is space for 64 bit values, but we never use more than the
 * lower 32 bits.  The actual register save/restore code only bothers
 * to restore those 32 bits.
 */
enum vp_registers {
	VP_VCFG = 0,
	VP_DCFG,

	VP_VX,
	VP_VY,

	VP_SCL,
	VP_VCK,

	VP_VCM,
	VP_PAR,

	VP_PDR,
	VP_SLR,

	VP_MISC,
	VP_CCS,

	VP_VYS,
	VP_VXS,

	VP_RSVD_0,
	VP_VDC,

	VP_RSVD_1,
	VP_CRC,

	VP_CRC32,
	VP_VDE,

	VP_CCK,
	VP_CCM,

	VP_CC1,
	VP_CC2,

	VP_A1X,
	VP_A1Y,

	VP_A1C,
	VP_A1T,

	VP_A2X,
	VP_A2Y,

	VP_A2C,
	VP_A2T,

	VP_A3X,
	VP_A3Y,

	VP_A3C,
	VP_A3T,

	VP_VRR,
	VP_AWT,

	VP_VTM,
	VP_VYE,

	VP_A1YE,
	VP_A2YE,

	VP_A3YE,	/* 0x150 */

	VP_VCR = 0x1000, /* 0x1000 - 0x1fff */
};

#define VP_VCFG_VID_EN			(1 << 0)

#define VP_DCFG_GV_GAM			(1 << 21)
#define VP_DCFG_PWR_SEQ_DELAY		((1 << 17) | (1 << 18) | (1 << 19))
#define VP_DCFG_PWR_SEQ_DELAY_DEFAULT	(1 << 19)	/* undocumented */
#define VP_DCFG_CRT_SYNC_SKW_MASK	(7 << 14)
#define VP_DCFG_CRT_SYNC_SKW_DEFAULT	(4 << 14)
#define VP_DCFG_CRT_VSYNC_POL		(1 << 9)
#define VP_DCFG_CRT_HSYNC_POL		(1 << 8)
#define VP_DCFG_DAC_BL_EN		(1 << 3)
#define VP_DCFG_VSYNC_EN		(1 << 2)
#define VP_DCFG_HSYNC_EN		(1 << 1)
#define VP_DCFG_CRT_EN			(1 << 0)

#define VP_MISC_APWRDN			(1 << 11)
#define VP_MISC_DACPWRDN		(1 << 10)
#define VP_MISC_BYP_BOTH		(1 << 0)


/*
 * Flat Panel registers (table 6-71).
 * Also 64 bit registers; see above note about 32-bit handling.
 */

/* we're actually in the VP register space, starting at address 0x400 */
#define VP_FP_START	0x400

enum fp_registers {
	FP_PT1 = 0,
	FP_PT2,

	FP_PM,
	FP_DFC,

	FP_RSVD_0,
	FP_RSVD_1,

	FP_RSVD_2,
	FP_RSVD_3,

	FP_RSVD_4,
	FP_DCA,

	FP_DMD,
	FP_CRC, /* 0x458 */

	FP_RSVD_5,
	FP_CRC32, /* 0x468 */
};

#define FP_PT2_HSP			(1 << 22)
#define FP_PT2_VSP			(1 << 23)
#define FP_PT2_SCRC			(1 << 27)	/* shfclk free */

#define FP_PM_P				(1 << 24)	/* panel power ctl */
#define FP_PM_PANEL_PWR_UP		(1 << 3)	/* r/o */
#define FP_PM_PANEL_PWR_DOWN		(1 << 2)	/* r/o */
#define FP_PM_PANEL_OFF			(1 << 1)	/* r/o */
#define FP_PM_PANEL_ON			(1 << 0)	/* r/o */

#define FP_DFC_BC			((1 << 4) | (1 << 5) | (1 << 6))


/* also in VP register space, starting at address 0x800 */
#define VP_VOP_START	0x800

enum vop_registers {
	VOP_CONFIG = 0,
	VOP_SIG
};


/* register access functions */

static inline uint32_t read_gp(struct lx_priv *priv, int reg)
{
	return DRM_READ32(priv->gp, 4 * reg);
}

static inline void write_gp(struct lx_priv *priv, int reg, uint32_t val)
{
	DRM_WRITE32(priv->gp, 4 * reg, val);
}

static inline uint32_t read_dc(struct lx_priv *priv, int reg)
{
	return DRM_READ32(priv->dc, 4 * reg);
}

static inline void write_dc(struct lx_priv *priv, int reg, uint32_t val)
{
	DRM_WRITE32(priv->dc, 4 * reg, val);
}

static inline uint32_t read_vp(struct lx_priv *priv, int reg)
{
	return DRM_READ32(priv->vp, 8 * reg);
}

static inline void write_vp(struct lx_priv *priv, int reg, uint32_t val)
{
	DRM_WRITE32(priv->vp, 8 * reg, val);
}

static inline uint32_t read_fp(struct lx_priv *priv, int reg)
{
	return DRM_READ32(priv->vp, 8 * reg + VP_FP_START);
}

static inline void write_fp(struct lx_priv *priv, int reg, uint32_t val)
{
	DRM_WRITE32(priv->vp, 8 * reg + VP_FP_START, val);
}

static inline uint32_t read_vop(struct lx_priv *priv, int reg)
{
	return DRM_READ32(priv->vp, 8 * reg + VP_VOP_START);
}

static inline void write_vop(struct lx_priv *priv, int reg, uint32_t val)
{
	DRM_WRITE32(priv->vp, 8 * reg + VP_VOP_START, val);
}

#endif
