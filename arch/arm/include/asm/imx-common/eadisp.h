/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __EADISP_H_
#define __EADISP_H_

#include <linux/fb.h>
#include <ipu_pixfmt.h>

struct display_info_t {
	int	bus;	/* (bus >> 8) is gpio to enable bus if <>0 */
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev, int enable);

#define FB_RGB		0
#define FB_LVDS0	1
#define FB_LVDS1	2
#define FB_HDMI		3
#define FB_COUNT	4
	int	fbtype;

#define FBF_MODESTR		1
#define FBF_JEIDA		2
#define FBF_SPLITMODE	4
	int	fbflags;

#define MISC_HSYNC_ACTIVE_HIGH     1
#define MISC_VSYNC_ACTIVE_HIGH     2
#define MISC_DE_ACTIVE_HIGH        4
#define MISC_PIXELCLK_ACTIVE_HIGH  8
	int 	misc;

	struct	fb_videomode mode;
};

void board_enable_hdmi(const struct display_info_t *di, int enable);
void board_enable_lcd(const struct display_info_t *di, int enable);
void board_enable_lvds0(const struct display_info_t *di, int enable);
void board_enable_lvds1(const struct display_info_t *di, int enable);

void eadisp_enable_fb(struct display_info_t const *di, int enable);
int eadisp_detect_i2c(struct display_info_t const *di);
void eadisp_setup_display(const struct display_info_t *displays, int cnt);

/* LVDS settings */
#define EADISP_HANNSTAR10(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x4,\
	.pixfmt	= IPU_PIX_FMT_RGB666,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.mode	= {\
		.name           = "hannstar",\
		.refresh        = 60,\
		.xres           = 1024,\
		.yres           = 768,\
		.pixclock       = 1000000000000ULL/((1024+220+40+60)*(768+21+7+10)*60),\
		.left_margin    = 220,\
		.right_margin   = 40,\
		.upper_margin   = 21,\
		.lower_margin   = 7,\
		.hsync_len      = 60,\
		.vsync_len      = 10,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}
#define EADISP_LP101WH4(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x4,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.misc   = MISC_DE_ACTIVE_HIGH,\
	.mode	= {\
		.name           = "lp101wh4",\
		.refresh        = 60,\
		.xres           = 1366,\
		.yres           = 768,\
		.pixclock       = 1000000000000ULL/((1366+80+48+32)*(768+14+3+5)*60),\
		.left_margin    = 80,\
		.right_margin   = 48,\
		.upper_margin   = 14,\
		.lower_margin   = 3,\
		.hsync_len      = 32,\
		.vsync_len      = 5,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define EADISP_LP101WH4_X11(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x4,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.misc   = MISC_DE_ACTIVE_HIGH,\
	.mode	= {\
		.name           = "lp101wh4_X11",\
		.refresh        = 60,\
		.xres           = 1360,\
		.yres           = 768,\
		.pixclock       = 1000000000000ULL/((1360+80+48+32)*(768+14+3+5)*60),\
		.left_margin    = 80,\
		.right_margin   = 48,\
		.upper_margin   = 14,\
		.lower_margin   = 3,\
		.hsync_len      = 32,\
		.vsync_len      = 5,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* RGB settings */
#define EADISP_INNOLUX_AT070TN(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.misc   = MISC_DE_ACTIVE_HIGH,\
	.mode	= {\
		.name           = "Innolux-AT070TN",\
		.refresh        = 49,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+89+164+10)*(480+75+75+10)*49),\
		.left_margin    = 89,\
		.right_margin   = 164,\
		.upper_margin   = 75,\
		.lower_margin   = 75,\
		.hsync_len      = 10,\
		.vsync_len      = 10,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define EADISP_NHD_43480272EF(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.misc   = MISC_DE_ACTIVE_HIGH,\
	.mode	= {\
		.name           = "nhd-4.3-480272ef",\
		.refresh        = 60,\
		.xres           = 480,\
		.yres           = 272,\
		.pixclock       = 1000000000000ULL/((480+2+2+41)*(272+2+2+10)*60),\
		.left_margin    = 2,\
		.right_margin   = 2,\
		.upper_margin   = 2,\
		.lower_margin   = 2,\
		.hsync_len      = 41,\
		.vsync_len      = 10,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define EADISP_NHD_50800480TF(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.misc   = MISC_DE_ACTIVE_HIGH,\
	.mode	= {\
		.name           = "nhd-5.0-800480tf",\
		.refresh        = 62,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+40+40+48)*(480+29+13+3)*60),\
		.left_margin    = 40,\
		.right_margin   = 40,\
		.upper_margin   = 29,\
		.lower_margin   = 13,\
		.hsync_len      = 48,\
		.vsync_len      = 3,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define EADISP_NHD_70800480EF(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.misc   = MISC_DE_ACTIVE_HIGH,\
	.mode	= {\
		.name           = "nhd-7.0-800480ef",\
		.refresh        = 62,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+40+40+48)*(480+29+13+3)*60),\
		.left_margin    = 40,\
		.right_margin   = 40,\
		.upper_margin   = 29,\
		.lower_margin   = 13,\
		.hsync_len      = 48,\
		.vsync_len      = 3,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define EADISP_UMSH_8864(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.misc   = MISC_DE_ACTIVE_HIGH,\
	.mode	= {\
		.name           = "umsh-8864",\
		.refresh        = 55,\
		.xres           = 480,\
		.yres           = 272,\
		.pixclock       = 1000000000000ULL/((480+20+20+3)*(272+20+20+3)*55),\
		.left_margin    = 20,\
		.right_margin   = 20,\
		.upper_margin   = 20,\
		.lower_margin   = 20,\
		.hsync_len      = 3,\
		.vsync_len      = 3,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define EADISP_UMSH_8596_30T(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.misc   = MISC_DE_ACTIVE_HIGH + MISC_PIXELCLK_ACTIVE_HIGH,\
	.mode	= {\
		.name           = "umsh-8596-30t",\
		.refresh        = 60,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+128+120+8)*(480+20+20+5)*60),\
		.left_margin    = 128,\
		.right_margin   = 120,\
		.upper_margin   = 20,\
		.lower_margin   = 20,\
		.hsync_len      = 8,\
		.vsync_len      = 5,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define EADISP_UMSH_8596_33T(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.misc   = MISC_DE_ACTIVE_HIGH + MISC_PIXELCLK_ACTIVE_HIGH,\
	.mode	= {\
		.name           = "umsh-8596-33t",\
		.refresh        = 48,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+200+200+1)*(480+45+45+1)*48),\
		.left_margin    = 200,\
		.right_margin   = 200,\
		.upper_margin   = 45,\
		.lower_margin   = 45,\
		.hsync_len      = 1,\
		.vsync_len      = 1,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define EADISP_ROGIN_RX050A(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = 0,\
	.misc   = MISC_DE_ACTIVE_HIGH,\
	.mode	= {\
		.name           = "rogin-rx050a",\
		.refresh        = 48,\
		.xres           = 800,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((800+200+200+1)*(480+45+45+1)*48),\
		.left_margin    = 200,\
		.right_margin   = 200,\
		.upper_margin   = 45,\
		.lower_margin   = 45,\
		.hsync_len      = 1,\
		.vsync_len      = 1,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

/* hdmi settings */
#define EADISP_HDMI_1280_720M_60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x50,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "1280x720M@60",\
		.refresh        = 60,\
		.xres           = 1280,\
		.yres           = 720,\
		.pixclock       = 1000000000000ULL/((1280+216+72+80)*(720+22+3+5)*60),\
		.left_margin    = 220,\
		.right_margin   = 110,\
		.upper_margin   = 20,\
		.lower_margin   = 5,\
		.hsync_len      = 40,\
		.vsync_len      = 5,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}


#define EADISP_HDMI_1920_1080M_60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x50,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "1920x1080M@60",\
		.refresh        = 60,\
		.xres           = 1920,\
		.yres           = 1080,\
		.pixclock       = 1000000000000ULL/((1920+148+88+44)*(1080+36+4+5)*60),\
		.left_margin    = 148,\
		.right_margin   = 88,\
		.upper_margin   = 36,\
		.lower_margin   = 4,\
		.hsync_len      = 44,\
		.vsync_len      = 5,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define EADISP_HDMI_640_480M_60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x50,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "640x480M@60",\
		.refresh        = 60,\
		.xres           = 640,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((640+48+16+96)*(480+33+10+2)*60),\
		.left_margin    = 48,\
		.right_margin   = 16,\
		.upper_margin   = 33,\
		.lower_margin   = 10,\
		.hsync_len      = 96,\
		.vsync_len      = 2,\
		.sync           = FB_SYNC_EXT,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}

#define EADISP_HDMI_720_480M_60(_mode, _detect, _bus) \
{\
	.bus	= _bus,\
	.addr	= 0x50,\
	.pixfmt	= IPU_PIX_FMT_RGB24,\
	.detect	= _detect ? eadisp_detect_i2c : NULL,\
	.enable	= eadisp_enable_fb,\
	.fbtype = FB_##_mode,\
	.fbflags = FBF_MODESTR,\
	.mode	= {\
		.name           = "720x480M@60",\
		.refresh        = 60,\
		.xres           = 720,\
		.yres           = 480,\
		.pixclock       = 1000000000000ULL/((720+60+16+62)*(480+30+9+6)*60),\
		.left_margin    = 60,\
		.right_margin   = 16,\
		.upper_margin   = 30,\
		.lower_margin   = 9,\
		.hsync_len      = 62,\
		.vsync_len      = 6,\
		.sync           = 0,\
		.vmode          = FB_VMODE_NONINTERLACED\
	}\
}


#endif /* __EADISP_H_ */
