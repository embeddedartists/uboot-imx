/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/errno.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/sys_proto.h>
#if !defined(CONFIG_MX7D)
  #include <asm/arch/mxc_hdmi.h>
#endif
#include <asm/gpio.h>
#include <asm/imx-common/eadisp.h>
#include <asm/io.h>
#include <div64.h>
#include <i2c.h>
#include <malloc.h>
#include <video_fb.h>
#include <stdlib.h>
#if defined(CONFIG_MX6SX) || defined(CONFIG_MX6UL) || defined(CONFIG_MX7D)
  #include <linux/fb.h>
  #include <mxsfb.h>
#endif

/*
 * The board file should call eadisp_setup_display to register a list of
 * supported configurations for each supported display type.
 *
 * The eadisp command:
 * - Supports enabling/disabling of each supported display type
 * - Supports selecting one display to use in u-boot (otherwise the first)
 *   in the list is used
 * - Supports adding new display configurations (complete including pixclock,
 *   resolution, and timing information)
 * - Supports removal of added configurations
 *
 * The eadisp command does this by setting environment variables (XX stand for
 * lcd,hdmi,lvds0,lvds1):
 * - fb_XX               hold the selected configuration
 * - eadisp_XX_enabled   set to yes if the XX-type display is enabled
 * - eadisp_prefer       set to XX if XX is the preferred display type
 * - cmd_XX              holds fdt commands that will be executed by a bootscript
 */
static const char *const fbnames[] = {
[FB_RGB] = "fb_rgb",
[FB_LVDS0] = "fb_lvds0",
[FB_LVDS1] = "fb_lvds1",
[FB_HDMI] = "fb_hdmi"
};

static const char *const timings_names[] = {
[FB_RGB] = "t_rgb",
[FB_LVDS0] = "t_lvds0",
[FB_LVDS1] = "t_lvds1",
[FB_HDMI] = "t_hdmi"
};

static const char *const ch_names[] = {
[FB_RGB] = "",
[FB_LVDS0] = "ldb/lvds-channel@0",
[FB_LVDS1] = "ldb/lvds-channel@1",
[FB_HDMI] = ""
};

static const char *const bl_names[] = {
[FB_RGB] = "bl_rgb",
[FB_LVDS0] = "bl_lvds",
[FB_LVDS1] = "bl_lvds",
[FB_HDMI] = ""
};

static const char *const cmd_fbnames[] = {
[FB_RGB] = "cmd_rgb",
[FB_LVDS0] = "cmd_lvds0",
[FB_LVDS1] = "cmd_lvds1",
[FB_HDMI] = "cmd_hdmi"
};

static const char *const short_names[] = {
[FB_RGB] = "rgb",
[FB_LVDS0] = "lvds0",
[FB_LVDS1] = "lvds1",
[FB_HDMI] = "hdmi"
};

#if defined(CONFIG_MX6SX)
  #define DISPTYPES  "(rgb|lvds0)"
  #define NUM_FBS    2  /* Only have 1x RGB and 1x LVDS on SoloX */
#elif defined(CONFIG_MX6Q)
  #define DISPTYPES  "(hdmi|rgb|lvds0|lvds1)"
  #define NUM_FBS    4
#elif defined(CONFIG_MX6DL)
  #define DISPTYPES  "(hdmi|rgb|lvds0|lvds1)"
  #define NUM_FBS    4
#elif defined(CONFIG_MX6UL)
  #define DISPTYPES  "rgb"
  #define NUM_FBS    1  /* Only have 1x RGB on UltraLite */
#elif defined(CONFIG_MX7D)
  #define DISPTYPES  "rgb"
  #define NUM_FBS    1  /* Only have 1x RGB on 7 Dual */
#else
  #error Unsupported hardware
#endif

static const char *const timings_properties[] = {
"clock-frequency",
"hactive",
"vactive",
"hback-porch",
"hfront-porch",
"vback-porch",
"vfront-porch",
"hsync-len",
"vsync-len",
};

static const int timings_offsets[] = {
	offsetof(struct fb_videomode, pixclock),
	offsetof(struct fb_videomode, xres),
	offsetof(struct fb_videomode, yres),
	offsetof(struct fb_videomode, left_margin),
	offsetof(struct fb_videomode, right_margin),
	offsetof(struct fb_videomode, upper_margin),
	offsetof(struct fb_videomode, lower_margin),
	offsetof(struct fb_videomode, hsync_len),
	offsetof(struct fb_videomode, vsync_len),
};

#define MAX_OWN_DISPLAYS  5
static struct display_info_t g_di_own[MAX_OWN_DISPLAYS];
static char g_di_own_mode_str[MAX_OWN_DISPLAYS][80];
static int found_own_displays = 0;
static int loaded_own_configs = 0;

static const struct display_info_t *g_displays;
static int g_display_cnt;
static const struct display_info_t *g_di_active;
static char g_mode_str[4][80];
static struct display_info_t g_di_temp[FB_COUNT];
static const char rgb24[] = "RGB24";
static const char rgb666[] = "RGB666";
static const char yuyv16[] = "YUYV16";

static void __board_enable_hdmi(const struct display_info_t *di, int enable)
{
}

static void __board_enable_rgb(const struct display_info_t *di, int enable)
{
}

static void __board_enable_lvds0(const struct display_info_t *di, int enable)
{
}

static void __board_enable_lvds1(const struct display_info_t *di, int enable)
{
}

void board_enable_hdmi(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_hdmi")));
void board_enable_rgb(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_rgb")));
void board_enable_lvds0(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_lvds0")));
void board_enable_lvds1(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_lvds1")));

void eadisp_enable_fb(struct display_info_t const *di, int enable)
{
	switch (di->fbtype) {
	case FB_HDMI:
		board_enable_hdmi(di, enable);
		break;
	case FB_RGB:
		board_enable_rgb(di, enable);
		break;
	case FB_LVDS0:
		board_enable_lvds0(di, enable);
		break;
	case FB_LVDS1:
		board_enable_lvds1(di, enable);
		break;
	}
}

static const struct display_info_t *find_panel(const struct display_info_t *di, int cnt, unsigned fb, const char *name)
{
	int i;

	for (i = 0; i < cnt; i++, di++) {
		if ((fb == di->fbtype) && !strcmp(name, di->mode.name))
			return di;
	}
	return NULL;
}

static const struct display_info_t * parse_mode(
		const struct display_info_t *gdi, int cnt, const char *p,
		unsigned fb)
{
	char c;
	char *endp;
	unsigned value;
	int i;
	struct display_info_t *di;
	char *mode_str = g_mode_str[fb];

	i = 0;
	while (i < 80 - 1) {
		c = *p;
		if (c)
			p++;
		if (!c || (c == ':')) {
			break;
		}
		mode_str[i++] = c;
	}
	mode_str[i] = 0;
	c = *p;
	if (!c) {
		// only have a mode_str, no other configuration
		if (gdi == NULL) {
			// parsing own configurations - no point in looking for mode_str
			return NULL;
		}
		return find_panel(gdi, cnt, fb, mode_str);
	}
	di = &g_di_temp[fb];
	memset(di, 0, sizeof(*di));

	di->fbtype = fb;
	di->mode.name = mode_str;
	di->enable = eadisp_enable_fb;

	if (c == 'm') {
		di->fbflags |= FBF_MODESTR;
		p++;
		c = *p;
	}
	if (c == 'j') {
		di->fbflags |= FBF_JEIDA;
		p++;
		c = *p;
	}
	if (c == 's') {
		di->fbflags |= FBF_SPLITMODE;
		p++;
		c = *p;
	}
	value = simple_strtoul(p, &endp, 10);
	if (endp <= p) {
		printf("expecting 18|24\n");
		return NULL;
	}
	if ((value != 18) && (value != 24)) {
		printf("expecting 18|24, found %d\n", value);
		return NULL;
	}
	p = endp;
	di->pixfmt = (value == 24) ? IPU_PIX_FMT_RGB24 : IPU_PIX_FMT_RGB666;
	c = *p;
	if (*p != ':') {
		printf("expected ':', %s\n", p);
		return NULL;
	}
	p++;

	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		u32 *dest = (u32 *)((char *)&di->mode + timings_offsets[i]);
		u32 val;

		val = simple_strtoul(p, &endp, 10);
		if (endp <= p) {
			printf("expecting integer:%s\n", p);
			return NULL;
		}
		if (i == 0) {
			u64 lval = 1000000000000ULL;

			do_div(lval, val);
			val = (u32)lval;
		}
		*dest = val;
		p = endp;
		if (*p == ',')
			p++;
		if (*p == ' ')
			p++;
	}

	di->misc = 0;
	for (i = 0; i < 4; i++)
	{
		u32 val;
		val = simple_strtoul(p, &endp, 10);
                if (endp <= p) {
                        printf("expecting integer:%s\n", p);
                        return NULL;
                }
                if (!(val==0 || val==1)) {
                        printf("expecting 0 or 1:%s\n", p);
                        return NULL;
                }
		if (val == 1) {
			switch (i) {
			case 0: di->misc |= MISC_HSYNC_ACTIVE_HIGH; break;
			case 1: di->misc |= MISC_VSYNC_ACTIVE_HIGH; break;
			case 2: di->misc |= MISC_DE_ACTIVE_HIGH; break;
			case 3: di->misc |= MISC_PIXELCLK_ACTIVE_HIGH; break;
			}
		}
		p = endp;
		if (*p == ',')
			p++;
		if (*p == ' ')
			p++;
	}

	if (*p) {
		printf("extra parameters found:%s\n", p);
		return NULL;
	}
	return di;
}

#if defined(CONFIG_MX6Q)
static int is_fb_enabled(int fb)
{
	char buf[40];
	if (getenv(fbnames[FB_LVDS0])) {
		sprintf(buf, "eadisp_%s_enabled", short_names[fb]);
		if (!strcmp("yes", getenv(buf))) {
			return 1;
		}
	}
	return 0;
}
#endif

static void setup_cmd_fb(unsigned fb, const struct display_info_t *di, char *buf, int size)
{
	const char *mode_str = NULL;
	int i;
	int sz;
	const char *buf_start = buf;
	const struct fb_videomode *mode;
	const char * fmt;

	if (!di) {
		sz = snprintf(buf, size, "fdt set %s status disabled", fbnames[fb]);
		buf += sz;
		size -= sz;
		if (fb == FB_RGB) {
			snprintf(buf, size, ";fdt set %s status disabled", bl_names[fb]);
#if defined(CONFIG_MX6SX)
		} else if (fb == FB_LVDS0) {
			snprintf(buf, size, ";fdt set %s status disabled", bl_names[fb]);
#elif defined(CONFIG_MX6Q)
		} else if (fb == FB_LVDS1) {
			// Enough to do this check for FB_LVDS1 as it will test FB_LVDS0 as well
			if (!is_fb_enabled(FB_LVDS0)) {
				// neither LVDS interface is enabled so no need for backlight
				snprintf(buf, size, ";fdt set %s status disabled", bl_names[fb]);
			}
#endif
		}
		setenv(cmd_fbnames[fb], buf_start);
		//printf("Disabling %s\n", short_names[fb]);
		return;
	} else {
		if (di->fbflags & FBF_MODESTR) {
			mode_str = di->mode.name;
		}
	}

	sz = snprintf(buf, size, "fdt set %s status okay;", fbnames[fb]);
	buf += sz;
	size -= sz;

	if (di->pixfmt == IPU_PIX_FMT_RGB24)
		fmt = rgb24;
	else if (di->pixfmt == IPU_PIX_FMT_YUYV)
		fmt = yuyv16;
	else
		fmt = rgb666;

	if (di && ((fb == FB_RGB) || (fb == FB_LVDS0) || (fb == FB_LVDS1))) {
#if defined(CONFIG_MX6SX) || defined(CONFIG_MX6UL) || defined(CONFIG_MX7D)
		sz = snprintf(buf, size, "fdt set %s bus-width <%u>;", short_names[fb],
				(di->pixfmt == IPU_PIX_FMT_RGB24) ? 24 : 18);

#else
		sz = snprintf(buf, size, "fdt set %s interface_pix_fmt %s;",
				fbnames[fb], fmt);
#endif
		buf += sz;
		size -= sz;
	}

	if (di && (fb == FB_RGB)) {
		sz = snprintf(buf, size, "fdt set rgb default_ifmt %s;", fmt);
		buf += sz;
		size -= sz;
	}

	if (di && ((fb == FB_LVDS0) || (fb == FB_LVDS1))) {

		sz = snprintf(buf, size, "fdt set %s fsl,data-width <%u>;",
				ch_names[fb],
				(di->pixfmt == IPU_PIX_FMT_RGB24) ? 24 : 18);
		buf += sz;
		size -= sz;

		sz = snprintf(buf, size, "fdt set %s fsl,data-mapping %s;",
				ch_names[fb],
				(di->fbflags & FBF_JEIDA) ? "jeida" : "spwg");
		buf += sz;
		size -= sz;

		if (di->fbflags & FBF_SPLITMODE) {
			sz = snprintf(buf, size, "fdt set ldb split-mode 1;");
			buf += sz;
			size -= sz;
		}
	}

	if (mode_str) {
		snprintf(buf, size, "fdt set %s mode_str %s;", fbnames[fb], mode_str);
		setenv(cmd_fbnames[fb], buf_start);
		return;
#ifdef CONFIG_MX6Q
	} else if (fb == FB_RGB) {
		sz = snprintf(buf, size, "fdt set %s mode_str eadisp_special;", fbnames[fb]);
		buf += sz;
		size -= sz;
#endif
	}

	mode = &di->mode;
	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		u32 *p = (u32 *)((char *)mode + timings_offsets[i]);
		u32 val;

		if (i == 0) {
			u64 lval = 1000000000000ULL;

			do_div(lval, mode->pixclock);
			val = (u32)lval;
		} else {
			val = *p;
		}
		sz = snprintf(buf, size, "fdt set %s %s <%u>;", timings_names[fb], timings_properties[i], val);
		buf += sz;
		size -= sz;
	}

	sz = snprintf(buf, size, "fdt set %s hsync-active <%u>;", timings_names[fb], (di->misc & MISC_HSYNC_ACTIVE_HIGH)?1:0);
	buf += sz;
	size -= sz;
	sz = snprintf(buf, size, "fdt set %s vsync-active <%u>;", timings_names[fb], (di->misc & MISC_VSYNC_ACTIVE_HIGH)?1:0);
	buf += sz;
	size -= sz;
	sz = snprintf(buf, size, "fdt set %s de-active <%u>;", timings_names[fb], (di->misc & MISC_DE_ACTIVE_HIGH)?1:0);
	buf += sz;
	size -= sz;
	sz = snprintf(buf, size, "fdt set %s pixelclk-active <%u>;", timings_names[fb], (di->misc & MISC_PIXELCLK_ACTIVE_HIGH)?1:0);
	buf += sz;
	size -= sz;

	setenv(cmd_fbnames[fb], buf_start);
}

static const struct display_info_t *select_display(
		const struct display_info_t *gdi, int cnt)
{
	const char* tmp = getenv("eadisp_prefer");
	const char* enabled;
	const struct display_info_t* di = NULL;
	const struct display_info_t* first = NULL;
	char *buf = malloc(4096);
	int preferred_idx = -1;
	int i;
	uint32_t enmask = 0;

	/* See if a preferred display is selected */
	if (tmp) {
		for (i = 0; i < FB_COUNT; i++) {
			if (!strcmp(tmp, short_names[i])) {
				preferred_idx = i;
				break;
			}
		}
	}

	/* Enable/disable displays */
	for (i = 0; i < NUM_FBS; i++) {
		sprintf(buf, "eadisp_%s_enabled", short_names[i]);
		enabled = getenv(buf);
		tmp = getenv(fbnames[i]);
		if (tmp) {
			di = parse_mode(g_displays, g_display_cnt, tmp, i);
			if (di) {
				if (enabled && !strcmp(enabled, "yes")) {
					setup_cmd_fb(i, di, buf, 4096);
					enmask |= (1<<i);
					if (!first || i==preferred_idx) {
						first = di;
					}
					continue;
				}
			}
		}
		// disable
		setup_cmd_fb(i, NULL, buf, 4096);
	}

#if defined(CONFIG_MX6Q)
	// DTS has LVDS1 as primary, change that unless LVDS1 is preferred
	if (enmask & (1<<FB_LVDS0)) {
		if (preferred_idx!=FB_LVDS1 || !(enmask & (1<<FB_LVDS1))) {
			// LVDS0 is preferred over the default LVDS1
			tmp = getenv(cmd_fbnames[FB_LVDS0]);
			snprintf(buf, 4096, "fdt rm %s primary;fdt set %s primary;%s",
					 ch_names[FB_LVDS1], ch_names[FB_LVDS0], tmp);
			setenv(cmd_fbnames[FB_LVDS0], buf);
		}
	}
#endif

	free(buf);
	return first;
}

static void str_mode(char *p, int size, const struct display_info_t *di)
{
	int count;
	int i;

	if (!di) {
		count = snprintf(p, size, "off");
		if (size > count) {
			p += count;
			size -= count;
		}
		*p = 0;
		return;
	}
	count = snprintf(p, size, "%s:", di->mode.name);
	if (size > count) {
		p += count;
		size -= count;
	}
	if (di->fbflags & FBF_MODESTR) {
		*p++ = 'm';
		size--;
	}
	if (di->fbflags & FBF_JEIDA) {
		*p++ = 'j';
		size--;
	}
	if (di->fbflags & FBF_SPLITMODE) {
		*p++ = 's';
		size--;
	}
	count = snprintf(p, size, "%d:", (di->pixfmt == IPU_PIX_FMT_RGB24) ? 24 : 18);
	if (size > count) {
		p += count;
		size -= count;
	}

	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		u32 *src = (u32 *)((char *)&di->mode + timings_offsets[i]);
		u32 val;

		if (i == 0) {
			u64 lval = 1000000000000ULL;

			do_div(lval, di->mode.pixclock);
			val = (u32)lval;
		} else {
			val = *src;
			if (size > 1) {
				*p++ = ',';
				size--;
			}
		}
		count = snprintf(p, size, "%d", val);
		if (size > count) {
			p += count;
			size -= count;
		}
	}

	count = snprintf(p, size, ",%d,%d,%d,%d",
		((di->misc & MISC_HSYNC_ACTIVE_HIGH) ? 1 : 0),
		((di->misc & MISC_VSYNC_ACTIVE_HIGH) ? 1 : 0),
		((di->misc & MISC_DE_ACTIVE_HIGH) ? 1 : 0),
		((di->misc & MISC_PIXELCLK_ACTIVE_HIGH) ? 1 : 0));
	if (size > count) {
		p += count;
		size -= count;
	}

	*p = 0;
}

static void add_own_config(const struct display_info_t* di)
{
	if (di && found_own_displays < MAX_OWN_DISPLAYS) {
		memcpy(&g_di_own[found_own_displays], di, sizeof(struct display_info_t));
		memcpy(g_di_own_mode_str[found_own_displays], di->mode.name, strlen(di->mode.name)+1);
		g_di_own[found_own_displays].mode.name = g_di_own_mode_str[found_own_displays];
		found_own_displays++;
	}
}

static void load_own_configs(void)
{
	int i, j;
	char key[40];
	char val[40];
	const char* existing;
	const struct display_info_t* di = NULL;
	if (loaded_own_configs) {
		return;
	}
	found_own_displays = 0;
	for (i = 0; i < MAX_OWN_DISPLAYS; i++) {
		sprintf(key, "eadisp_own_%d", i);
		existing = getenv(key);
		if (existing) {
			for (j = 0; j < NUM_FBS; j++) {
				sprintf(val, "%s:", short_names[j]);
				if (!strncmp(val, existing, strlen(val))) {
					di = parse_mode(g_displays, g_display_cnt, existing+strlen(val), j);
					add_own_config(di);
					break;
				}
			}
		}
	}
	loaded_own_configs = 1;
}

static void store_own_configs(void)
{
	int i;
	int saved = 0;
	char key[40];
	char *buf = malloc(4096);
	for (i = 0; i < found_own_displays; i++) {
		if (g_di_own[i].fbtype != FB_COUNT) {
			sprintf(buf, "%s:", short_names[g_di_own[i].fbtype]);
			str_mode(buf+strlen(buf), 256, &g_di_own[i]);
			sprintf(key, "eadisp_own_%d", saved);
			saved++;
			setenv(key, buf);
		}
	}
	for (i = saved; i < MAX_OWN_DISPLAYS; i++) {
		sprintf(key, "eadisp_own_%d", i);
		setenv(key, NULL);
	}
	free(buf);
}

static void print_current_config(void)
{
	int i;
	char buf[50];
	const char* msg;
	int preferred = -1;

	msg = getenv("eadisp_prefer");
	if (msg) {
		for (i = 0; i < NUM_FBS; i++) {
			if (!strcmp(msg, short_names[i])) {
				preferred = i;
				break;
			}
		}
	}
	printf("\nCurrent Selection:\n");
	printf("\t\tenabled\tprefer\tconfiguration\n");
	for (i = 0; i < NUM_FBS; i++) {
		int enabled = 0;
		sprintf(buf, "eadisp_%s_enabled", short_names[i]);
		msg = getenv(buf);
		if (msg && !strcmp(msg, "yes")) {
			enabled = 1;
		}
		msg = getenv(fbnames[i]);
		printf("\t%s:\t  %s\t  %s\t%s\n", short_names[i], enabled?"yes":"no ", preferred==i?"yes":"no ", msg?msg:"Not Configured");
	}
}

static void print_modes(const struct display_info_t *di, int cnt)
{
	int i;
	char buf[256];

	printf("\nAvailable display configurations:\n");
	for (i = 0; i < cnt; i++) {
		str_mode(buf, sizeof(buf), di+i);
		printf("   %2d) %5s  %s\n", i, short_names[(di+i)->fbtype], buf);
	}
	for (i = 0; i < found_own_displays; i++) {
		str_mode(buf, sizeof(buf), &g_di_own[i]);
		printf("   %2d) %5s  %s\n", i+cnt, short_names[g_di_own[i].fbtype], buf);
	}

	print_current_config();
}

void board_video_enable(void)
{
	const struct display_info_t *di = g_di_active;
	if (di && di->enable)
		di->enable(di, 1);
}

static int init_display(const struct display_info_t *di)
{
#if defined(CONFIG_MX6SX) || defined(CONFIG_MX6UL) || defined(CONFIG_MX7D)
    #if defined(CONFIG_MX7D)
	uint32_t lcdif_base_addr =  ELCDIF1_IPS_BASE_ADDR;
    #else
	uint32_t lcdif_base_addr = (di->fbtype==FB_RGB) ? LCDIF1_BASE_ADDR : LCDIF2_BASE_ADDR;
    #endif
	int bpp;
	switch(di->pixfmt) {
	case IPU_PIX_FMT_RGB666:
	case IPU_PIX_FMT_BGR666:
	case IPU_PIX_FMT_LVDS666:
		bpp = 18;
		break;
	case IPU_PIX_FMT_RGB565:
		bpp = 16;
		break;
	case IPU_PIX_FMT_LVDS888:
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
		bpp = 24;
		break;
	default:
		printf("LCD %s cannot be configured, invalid pixfmt 0x%08x\n",  di->mode.name, di->pixfmt);
		return -EINVAL;
	}
	int ret = mxs_lcd_panel_setup(di->mode, bpp, lcdif_base_addr);
#else
	int ret = ipuv3_fb_init(&di->mode, 0, di->pixfmt);
#endif
	if (ret) {
		printf("LCD %s cannot be configured: %d\n", di->mode.name, ret);
		return -EINVAL;
	}
	printf("Display: %s:%s (%ux%u)\n", short_names[di->fbtype],
			di->mode.name, di->mode.xres, di->mode.yres);
	g_di_active = di;
	board_video_enable();
	return 0;
}

static void set_defaults(void)
{
	char buf[256];
	int i, j;
	for (i = 0; i < NUM_FBS; i++) {
		if (!getenv(fbnames[i])) {
			for (j = 0; j < g_display_cnt; j++) {
				if (g_displays[j].fbtype == i) {
					str_mode(buf, 256, &g_displays[j]);
					setenv(fbnames[i], buf);
					break;
				}
			}
		}
	}
}

static int do_eadisp(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int i;
	int fb = -1;
	int ret = CMD_RET_SUCCESS;
	const char *cmd;
	const char *fbname;
	const struct display_info_t* di = g_displays;
	int cnt = g_display_cnt;
	char* buf;
	load_own_configs();
	if (argc < 2) {
		print_modes(di, cnt);
		return CMD_RET_SUCCESS;
	}
	if (argc < 3) {
		return CMD_RET_USAGE;
	}

	cmd = argv[1];

	if (!strcmp("rm", cmd)) {
		if (argc == 3) {
			int idx = simple_strtoul(argv[2], NULL, 10);
			if (idx >= 0 && idx < cnt) {
				printf("cannot remove builtin configs\n");
				return CMD_RET_FAILURE;
			}
			idx = idx - cnt;
			if (idx >= 0 && idx < found_own_displays) {
				printf("removing %d - %s\n", idx+cnt, g_di_own[idx].mode.name);
				g_di_own[idx].fbtype = FB_COUNT;
				store_own_configs();
				loaded_own_configs = 0;
				load_own_configs();
				return CMD_RET_SUCCESS;
			}
		}
		return CMD_RET_USAGE;
	}


	fbname = argv[2];
	for (i = 0; i < NUM_FBS; i++) {
		if (!strcmp(short_names[i], fbname)) {
			fb = i;
			break;
		}
	}
	if (fb < 0) {
		return CMD_RET_USAGE;
	}

	buf = malloc(4096);
	if (!buf) {
		return -ENOMEM;
	}
	do {
		if (!strcmp("prefer", cmd)) {
			setenv("eadisp_prefer", fbname);
			break;
		} else if (!strcmp("enable", cmd)) {
			sprintf(buf, "eadisp_%s_enabled", fbname);
			setenv(buf, "yes");
			break;
		} else if (!strcmp("disable", cmd)) {
			sprintf(buf, "eadisp_%s_enabled", fbname);
			setenv(buf, "no");
			break;
		} else if (!strcmp("conf", cmd)) {
			if (argc == 4) {
				int idx = simple_strtoul(argv[3], NULL, 10);
				if (idx >= 0 && idx < cnt) {
					di = &g_displays[idx];
				} else if (idx < (cnt+found_own_displays)) {
					di = &g_di_own[idx-cnt];
				} else {
					printf("invalid index (%d) for %s\n", idx, fbname);
					ret = CMD_RET_FAILURE;
					break;
				}
				if (di->fbtype == fb) {
					printf("selecting %s=%s\n", fbname, di->mode.name);
					str_mode(buf, 256, di);
					setenv(fbnames[fb], buf);
					break;
				} else {
					printf("invalid index (%d) for %s (wrong type)\n", idx, fbname);
					ret = CMD_RET_FAILURE;
					break;
				}
			}
		} else if (!strcmp("add", cmd)) {
			if (argc == 4) {
				const char *p = argv[3];
				di = parse_mode(di, cnt, p, fb);
				if (!di) {
					printf("invalid format string\n");
					ret = CMD_RET_FAILURE;
					break;
				}
				if (found_own_displays < MAX_OWN_DISPLAYS) {
					add_own_config(di);
					store_own_configs();
					loaded_own_configs = 0;
					load_own_configs();
					break;
				} else {
					printf("too many own configurations already - remove one and try again\n");
					ret = CMD_RET_FAILURE;
					break;
				}
			}
		}
		ret = CMD_RET_USAGE;

	} while(0);
	if (ret == CMD_RET_SUCCESS) {
		// Update all variables
		select_display(g_displays, g_display_cnt);
		print_current_config();
	}
	free(buf);
	return ret;
}

U_BOOT_CMD(eadisp, 4, 0, do_eadisp,
		   "Configure Display Support",
		   " - Show current configuration for all displays\n"
		   "eadisp prefer "DISPTYPES" - Set preferred display\n"
		   "eadisp enable "DISPTYPES" - Enable selected display\n"
		   "eadisp disable "DISPTYPES" - Disable selected display\n"
		   "eadisp conf "DISPTYPES" num - Select configuration for display\n"
		   "eadisp add "DISPTYPES" [\"mode_str[:[m][j][s][18|24]:pixclkfreq,xres,yres,hback-porch,hfront-porch,vback-porch,vfront-porch,hsync,vsync,hsact,vsact,deact,clkact]\"]\n"
		   "eadisp rm num - Remove added configuration\n");

int board_video_skip(void)
{
	set_defaults();
	const struct display_info_t *di = select_display(g_displays, g_display_cnt);

	if (!di) {
		return -EINVAL;
	}
	int ret = init_display(di);
	return ret;
}

void eadisp_setup_display(const struct display_info_t *displays, int cnt)
{
	g_displays = displays;
	g_display_cnt = cnt;
}
