/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#if defined(CONFIG_CMD_EADISP)
#include <asm/mach-imx/eadisp.h>
#include <asm/mach-imx/eatouch.h>
#endif
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#ifdef CONFIG_SYS_I2C
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#endif
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>

#include <usb.h>
#include <usb/ehci-ci.h>

#ifdef CONFIG_IMX_RDC
#include <asm/mach-imx/rdc-sema.h>
#include <asm/arch/imx-rdc.h>
#endif

#ifdef CONFIG_VIDEO_MXS
#include <linux/fb.h>
#include <mxsfb.h>
#endif

#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#endif /*CONFIG_FSL_FASTBOOT*/

#include <dm.h>
#include <fdt_support.h>

#include "../common/ea_eeprom.h"
#include "../common/ea_common.h"
#include "../common/ea_gpio_expander.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_120ohm   | PAD_CTL_SRE_FAST)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define WDOG_PAD_CTRL (PAD_CTL_PUE | PAD_CTL_PKE | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_GPIO1_IO04__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_GPIO1_IO05__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const peri_3v3_pads[] = {
	MX6_PAD_QSPI1B_DATA2__GPIO4_IO_26 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_b_pad = {
	MX6_PAD_GPIO1_IO13__GPIO1_IO_13 | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

#ifdef CONFIG_FEC_MXC
static iomux_v3_cfg_t const phy_control_pads[] = {
	/* Phy 25M Clock */
	MX6_PAD_ENET2_RX_CLK__ENET2_REF_CLK_25M | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),

	/* ENET PHY Power */
	MX6_PAD_ENET1_COL__GPIO2_IO_0 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* AR8031 PHY Reset. */
	MX6_PAD_ENET2_CRS__GPIO2_IO_7 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_QSPI

#define QSPI_PAD_CTRL1	\
		(PAD_CTL_SRE_FAST | PAD_CTL_SPEED_MED | \
		 PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_47K_UP | PAD_CTL_DSE_60ohm)

static iomux_v3_cfg_t const quadspi_pads[] = {
	MX6_PAD_NAND_WP_B__QSPI2_A_DATA_0		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_READY_B__QSPI2_A_DATA_1	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CE0_B__QSPI2_A_DATA_2	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CE1_B__QSPI2_A_DATA_3	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_ALE__QSPI2_A_SS0_B		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CLE__QSPI2_A_SCLK		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA07__QSPI2_A_DQS		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA01__QSPI2_B_DATA_0	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA00__QSPI2_B_DATA_1	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_WE_B__QSPI2_B_DATA_2		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_RE_B__QSPI2_B_DATA_3		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA03__QSPI2_B_SS0_B	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA02__QSPI2_B_SCLK		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA05__QSPI2_B_DQS		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
};

int board_qspi_init(void)
{
	/*
	 * Note (to be investigated):
	 *  With the Device model this iomux call shouldn't be needed since
	 *  pads are configured in dts. Without this call Linux doesn't boot
	 *  properly. There will be an'Internal error', and Linux won't
	 *  boot to the login prompt
	 *
	 *  Internal error: Oops: 5 [#1] PREEMPT SMP AR
	 *  ...
	 *  PC is at sysfs_kf_seq_show+0x68/0xec
	 *  ...
	 *  Process udevadm (pid: 182, stack limit = 0xa8b42210
	 *  ...
	 *  [FAILED] Failed to start udev Coldplug all Devices.
	 *  ...
	 *  A start job is running for dev-ttymxc0.device (1min / 1min 30s)
	 *  [TIME ] Timed out waiting for device dev-ttymxc0.device.
	 */


	/* Set the iomux */
	imx_iomux_v3_setup_multiple_pads(quadspi_pads, ARRAY_SIZE(quadspi_pads));

	/* Set the clock */
	enable_qspi_clk(1);

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lvds_ctrl_pads[] = {
	/* CABC enable */
	MX6_PAD_ENET1_CRS__GPIO2_IO_1 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Use GPIO for Brightness adjustment, duty cycle = period */
	MX6_PAD_USB_H_STROBE__GPIO7_IO_11 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Backlight power enable */
	MX6_PAD_GPIO1_IO09__GPIO1_IO_9 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_LCD1_CLK__LCDIF1_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_ENABLE__LCDIF1_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_HSYNC__LCDIF1_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_VSYNC__LCDIF1_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA00__LCDIF1_DATA_0 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA01__LCDIF1_DATA_1 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA02__LCDIF1_DATA_2 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA03__LCDIF1_DATA_3 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA04__LCDIF1_DATA_4 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA05__LCDIF1_DATA_5 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA06__LCDIF1_DATA_6 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA07__LCDIF1_DATA_7 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA08__LCDIF1_DATA_8 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA09__LCDIF1_DATA_9 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA10__LCDIF1_DATA_10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA11__LCDIF1_DATA_11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA12__LCDIF1_DATA_12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA13__LCDIF1_DATA_13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA14__LCDIF1_DATA_14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA15__LCDIF1_DATA_15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA16__LCDIF1_DATA_16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA17__LCDIF1_DATA_17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA18__LCDIF1_DATA_18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA19__LCDIF1_DATA_19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA20__LCDIF1_DATA_20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA21__LCDIF1_DATA_21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA22__LCDIF1_DATA_22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA23__LCDIF1_DATA_23 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_RESET__GPIO3_IO_27 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Use GPIO for Brightness adjustment, duty cycle = period */
	MX6_PAD_USB_H_DATA__GPIO7_IO_10 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* CABC enable */
	MX6_PAD_ENET1_CRS__GPIO2_IO_1 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Backlight power enable */
	MX6_PAD_GPIO1_IO09__GPIO1_IO_9 | MUX_PAD_CTRL(NO_PAD_CTRL),
};


#ifdef CONFIG_CMD_EADISP
void board_enable_lvds0(const struct display_info_t *di, int enable)
{
	static u32 base_addr = LCDIF2_BASE_ADDR;
	if (enable) {
		enable_lcdif_clock(base_addr, 1);
		enable_lvds_bridge(base_addr);
		if (di->pixfmt == IPU_PIX_FMT_RGB24) {
			/* enable_lvds() defaults to 18-bit data width */
			struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
			u32 reg = readl(&iomux->gpr[6]);
			reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
			writel(reg, &iomux->gpr[6]);
		}

		imx_iomux_v3_setup_multiple_pads(lvds_ctrl_pads,
						ARRAY_SIZE(lvds_ctrl_pads));
		mdelay(100); /* let panel sync up before enabling backlight */

		/* Enable CABC */
		gpio_request(IMX_GPIO_NR(2, 1), "CABC enable");
		gpio_direction_output(IMX_GPIO_NR(2, 1) , 1);

		/* Set Brightness to high */
		gpio_request(IMX_GPIO_NR(7, 11), "lvds backlight");
		gpio_direction_output(IMX_GPIO_NR(7, 11) , 1);

		/* Backlight power enable */
		gpio_request(IMX_GPIO_NR(1, 9), "backlight pwr");
		gpio_direction_output(IMX_GPIO_NR(1, 9) , 1);
	}
}

void board_enable_rgb(const struct display_info_t *di, int enable)
{
	static u32 base_addr = MX6SX_LCDIF1_BASE_ADDR;
	if (enable) {
		enable_lcdif_clock(base_addr, 1);
		imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));
		mdelay(100); /* let panel sync up before enabling backlight */

		/* Power up the LCD */
		gpio_request(IMX_GPIO_NR(2, 1), "lcd pwr");
		gpio_direction_output(IMX_GPIO_NR(2, 1) , 1);

		/* Set Brightness to high */
		gpio_request(IMX_GPIO_NR(7, 10), "lcd brightness");
		gpio_direction_output(IMX_GPIO_NR(7, 10) , 1);

		/* Backlight power enable */
		gpio_request(IMX_GPIO_NR(1, 9), "backlight pwr");
		gpio_direction_output(IMX_GPIO_NR(1, 9) , 1);
	}
}

static const struct display_info_t displays[] = {
	/* LVDS */
	EADISP_HANNSTAR10(LVDS0, 0, 0),
	EADISP_LP101WH4_X11(LVDS0, 0, 0),
	EADISP_LP101WH4(LVDS0, 0, 0),
	EADISP_NHD_1024600AF(LVDS0, 0, 0),

	/* RGB */
	EADISP_INNOLUX_AT070TN(RGB, 0, 0),
	EADISP_NHD_43480272EF(RGB, 0, 0),
	EADISP_NHD_50800480TF(RGB, 0, 0),
	EADISP_NHD_70800480EF(RGB, 0, 0),
	EADISP_UMSH_8864(RGB, 0, 0),
	EADISP_UMSH_8596_30T(RGB, 0, 0),
	EADISP_UMSH_8596_33T(RGB, 0, 0),
	EADISP_ROGIN_RX050A(RGB, 0, 0),
};

#else /* CONFIG_CMD_EADISP */
struct lcd_panel_info_t {
	unsigned int lcdif_base_addr;
	int depth;
	void	(*enable)(struct lcd_panel_info_t const *dev);
	struct fb_videomode mode;
};

void do_enable_lvds(struct lcd_panel_info_t const *dev)
{
	enable_lcdif_clock(dev->lcdif_base_addr, 1);
	enable_lvds(dev->lcdif_base_addr);

	imx_iomux_v3_setup_multiple_pads(lvds_ctrl_pads,
							ARRAY_SIZE(lvds_ctrl_pads));

	/* Enable CABC */
	gpio_request(IMX_GPIO_NR(2, 1), "CABC enable");
	gpio_direction_output(IMX_GPIO_NR(2, 1) , 1);

	/* Set Brightness to high */
	gpio_request(IMX_GPIO_NR(7, 11), "lvds backlight");
	gpio_direction_output(IMX_GPIO_NR(7, 11) , 1);

	/* Backlight power enable */
	gpio_request(IMX_GPIO_NR(1, 9), "backlight pwr");
	gpio_direction_output(IMX_GPIO_NR(1, 9) , 1);
}

void do_enable_parallel_lcd(struct lcd_panel_info_t const *dev)
{
	enable_lcdif_clock(dev->lcdif_base_addr);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	/* Power up the LCD */
	gpio_request(IMX_GPIO_NR(2, 1), "LCD pwr");
	gpio_direction_output(IMX_GPIO_NR(2, 1) , 1);

	/* Set Brightness to high */
	gpio_request(IMX_GPIO_NR(7, 10), "lcd brightness");
	gpio_direction_output(IMX_GPIO_NR(7, 10) , 1);

	/* Backlight power enable */
	gpio_request(IMX_GPIO_NR(1, 9), "backlight pwr");
	gpio_direction_output(IMX_GPIO_NR(1, 9) , 1);
}

static struct lcd_panel_info_t const displays[] = {{
	.lcdif_base_addr = LCDIF2_BASE_ADDR,
	.depth = 18,
	.enable	= do_enable_lvds,
	.mode	= {
		.name			= "Hannstar-XGA",
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.lcdif_base_addr = LCDIF1_BASE_ADDR,
	.depth = 24,
	.enable	= do_enable_parallel_lcd,
	.mode	= {
		.name			= "Innolux-AT070TN",
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 29850,
		.left_margin    = 89,
		.right_margin   = 164,
		.upper_margin   = 75,
		.lower_margin   = 75,
		.hsync_len      = 10,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = env_get("panel");
	if (!panel) {
		panel = displays[0].mode.name;
		printf("No panel detected: default to %s\n", panel);
		i = 0;
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = mxs_lcd_panel_setup(displays[i].mode, displays[i].depth,
				    displays[i].lcdif_base_addr);
		if (!ret) {
			if (displays[i].enable)
				displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_CMD_EADISP */
#endif /* CONFIG_VIDEO_MXS */

#ifdef CONFIG_FEC_MXC
static int setup_fec(int fec_id)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	int reg, ret;

	if (0 == fec_id)
		/* Use 125M anatop loopback REF_CLK1 for ENET1, clear gpr1[13], gpr1[17]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC1_MASK, 0);
	else
		/* Use 125M anatop loopback REF_CLK1 for ENET2, clear gpr1[14], gpr1[18]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC2_MASK, 0);

        ret = enable_fec_anatop_clock(fec_id, ENET_125MHZ);
        if (ret)
                return ret;

	imx_iomux_v3_setup_multiple_pads(phy_control_pads,
		ARRAY_SIZE(phy_control_pads));

	/* Enable the ENET power, active low */
	gpio_request(IMX_GPIO_NR(2, 0), "enet pwr");
	gpio_direction_output(IMX_GPIO_NR(2, 0) , 0);

	/* Reset AR8031 PHY */
	gpio_request(IMX_GPIO_NR(2, 7), "ar8081 phy");
	gpio_direction_output(IMX_GPIO_NR(2, 7) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(2, 7), 1);

        reg = readl(&anatop->pll_enet);
        reg |= BM_ANADIG_PLL_ENET_REF_25M_ENABLE;
        writel(reg, &anatop->pll_enet);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	/* Enable 1.8V(SEL_1P5_1P8_POS_REG) on
	   Phy control debug reg 0 */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	/* rgmii tx clock delay enable */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#endif

int power_init_board(void)
{
	struct udevice *dev;
	unsigned int reg, dev_id, rev_id;
	int ret;
	unsigned char offset, i, switch_num;

	ret = pmic_get("pfuze100", &dev);
	if (ret)
		return ret;

	dev_id = pmic_reg_read(dev, PFUZE100_DEVICEID);
	rev_id = pmic_reg_read(dev, PFUZE100_REVID);
	printf("PMIC: PFUZE100! DEV_ID=0x%x REV_ID=0x%x\n", dev_id, rev_id);

	/* Set SW1AB stanby volage to 0.975V */
	reg = pmic_reg_read(dev, PFUZE100_SW1ABSTBY);
	reg &= ~SW1x_STBY_MASK;
	reg |= SW1x_0_975V;
	pmic_reg_write(dev, PFUZE100_SW1ABSTBY, reg);

	/* Set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
	reg = pmic_reg_read(dev, PFUZE100_SW1ABCONF);
	reg &= ~SW1xCONF_DVSSPEED_MASK;
	reg |= SW1xCONF_DVSSPEED_4US;
	pmic_reg_write(dev, PFUZE100_SW1ABCONF, reg);

	/* Set SW1C standby voltage to 0.975V */
	reg = pmic_reg_read(dev, PFUZE100_SW1CSTBY);
	reg &= ~SW1x_STBY_MASK;
	reg |= SW1x_0_975V;
	pmic_reg_write(dev, PFUZE100_SW1CSTBY, reg);

	/* Set SW1C/VDDSOC step ramp up time from 16us to 4us/25mV */
	reg = pmic_reg_read(dev, PFUZE100_SW1CCONF);
	reg &= ~SW1xCONF_DVSSPEED_MASK;
	reg |= SW1xCONF_DVSSPEED_4US;
	pmic_reg_write(dev, PFUZE100_SW1CCONF, reg);

	if ((dev_id & 0xf) == 0) {
		switch_num = 6;
		offset = PFUZE100_SW1CMODE;
	} else if ((dev_id &0xf) == 1) {
		switch_num = 4;
		offset = PFUZE100_SW2MODE;
	} else {
		printf("Not supported, id=%d\n", (dev_id&0xf));
		return -EINVAL;
	}

	ret = pmic_reg_write(dev, PFUZE100_SW1ABMODE, APS_PFM);
	if (ret < 0) {
		printf("Set SW1AB mode error!\n");
		return ret;
	}

	for (i = 0; i < switch_num - 1; i++) {
		ret = pmic_reg_write(dev, offset + i * SWITCH_SIZE, APS_PFM);
		if (ret < 0) {
			printf("Set switch 0x%x mode error!\n",
			       offset + i * SWITCH_SIZE);
			return ret;
		}
	}

	/* Enable power of VGEN5 3V3, needed for SD3 */
	reg = pmic_reg_read(dev, PFUZE100_VGEN5VOL);
	reg &= ~LDO_VOL_MASK;
	reg |= (LDOB_3_30V | (1 << LDO_EN));
	pmic_reg_write(dev, PFUZE100_VGEN5VOL, reg);

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	struct udevice *dev;
	int ret;
	int is_400M;
	u32 vddarm;

	ret = pmic_get("pfuze100", &dev);
	if (ret == -ENODEV) {
		printf("No PMIC found!\n");
		return;
	}

	/* switch to ldo_bypass mode , boot on 800Mhz */
	if (ldo_bypass) {
		prep_anatop_bypass();

		/* decrease VDDARM for 400Mhz DQ:1.1V, DL:1.275V */
		pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, PFUZE100_SW1ABC_SETP(12750));

		/* increase VDDSOC to 1.3V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, PFUZE100_SW1ABC_SETP(13000));

		is_400M = set_anatop_bypass(2);
		if (is_400M)
			vddarm = PFUZE100_SW1ABC_SETP(10750);
		else
			vddarm = PFUZE100_SW1ABC_SETP(11750);

		pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, vddarm);

		/* decrease VDDSOC to 1.175V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, PFUZE100_SW1ABC_SETP(11750));

		finish_anatop_bypass();
		printf("switch to ldo_bypass mode!\n");
	}
}
#endif

int mmc_map_to_kernel_blk(int dev_no)
{
	/* eMMC device is available at mmcblk2 in Linux */
	return 2;
}

int board_mmc_get_env_dev(int devno)
{
	int no = devno;
	bool is_v2 = true;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	if (ea_conf->magic == EA_CONFIG_MAGIC) {
		is_v2 = ea_conf->is_carrier_v2;
	}

	/*
	 * This function is used to get the MMC device used for
	 * the u-boot environment. The devno argument specifies the
	 * boot device as defined in BOOT_CFG2 (see SD/eSD Boot Fusemap in
	 * NXP's User's Manual).
	 *
	 * For the SoloX COM board eMMC is on USDHC3 which gives devno=2.
	 *
	 * When using the Device Module (dts) USDHC devices are added
	 * in the order they are defined in the dts file. Depending on
	 * which carrier board being used either USDHC2 or USDHC4 is
	 * enabled in board_fix_fdt (SD card is on either USDHC2 or USDHC4).
	 * This however effects which device number being assigned to eMMC.
	 * When USDHC2 is enabled it will eMMC will be on mmc1 (since USDHC2
	 * is on mmc0). When USDHC4 is enabled eMMC will be on mmc0 (and
	 * USDHC4 on mmc1).
	 */

	if (is_v2) {
		no = 0;
	}
	else {
		no = 1;
	}

	return no;
}

#ifdef CONFIG_IMX_RDC
static rdc_peri_cfg_t const shared_resources[] = {
	(RDC_PER_UART1 | RDC_DOMAIN(0)),
	(RDC_PER_GPT | RDC_DOMAIN(0)),
};
#endif

int board_early_init_f(void)
{
#ifdef CONFIG_IMX_RDC
	imx_rdc_setup_peripherals(shared_resources, ARRAY_SIZE(shared_resources));
#endif

#ifdef CONFIG_SYS_AUXCORE_FASTUP
	arch_auxiliary_core_up(0, CONFIG_SYS_AUXCORE_BOOTDATA);
#endif

	return 0;
}

int board_init_common(void)
{
	/* Enable PERI_3V3, which is used by SD2, ENET, LVDS, BT */
	imx_iomux_v3_setup_multiple_pads(peri_3v3_pads, ARRAY_SIZE(peri_3v3_pads));
	gpio_request(IMX_GPIO_NR(4, 26), "peri 3.3  pwr");
	gpio_direction_output(IMX_GPIO_NR(4, 26) , 1);

#ifndef CONFIG_EA_NO_UART_FLUSH
	/*
	 * Empty UART RX FIFO 5ms after PERI_PWR_ENABLE goes high.
	 *
	 * Needed to avoid u-boot to stop booting (boot delay) due to
	 * data being available in the uart buffer. This problem is
	 * available on COM Carrier board rev A to rev D. The problem
	 * doesn't exist on rev E (also known as COM Carrier Board V2)
	 */
	udelay(5000);
	while (tstc()) {
		(void)getc();
	}
#endif

	setup_iomux_uart();

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	board_init_common();

	/*
	 * Because kernel set WDOG_B mux before pad with the commone pinctrl
	 * framwork now and wdog reset will be triggered once set WDOG_B mux
	 * with default pad setting, we set pad setting here to workaround this.
	 * Since imx_iomux_v3_setup_pad also set mux before pad setting, we set
	 * as GPIO mux firstly here to workaround it.
	 */
	imx_iomux_v3_setup_pad(wdog_b_pad);

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif

#ifdef CONFIG_FEC_MXC
        setup_fec(CONFIG_FEC_ENET_DEV);
#endif

#if defined(CONFIG_CMD_EADISP) && !defined(CONFIG_SPL_BUILD)
	eadisp_setup_display(displays, ARRAY_SIZE(displays));
#endif
#ifdef CONFIG_EA_IMX_PTP
	ea_configure_tfp410();
#endif

	ea_print_board();

	return 0;
}

int board_late_init(void)
{

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

#ifdef CONFIG_CMD_EADISP
	eatouch_init();
#endif

#ifdef CONFIG_FEC_MXC
	/*
	 * Loading ethernet addresses must be done in late_init
	 * since they update the environment (env_set). The
	 * environment isn't loaded and ready at board_init.
	 */
	if (ea_load_ethaddr()) {
		printf("Failed to load MAC addresses\n");
	}
#endif

#ifdef CONFIG_SYS_I2C_MXC
	ea_gpio_exp_configure(1);
#endif
	return 0;
}

int board_fix_fdt(void* rw_fdt_blob)
{
	int ret = 0;

	bool is_v2 = true;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	if (ea_conf->magic == EA_CONFIG_MAGIC) {
		is_v2 = ea_conf->is_carrier_v2;
	}

	/*
	 * On Carrier board V2 the usdhc4 interfaces is connected to
	 * MMC/SD card connector. On older versions usdhc2 us used.
	 */
	if (is_v2) {
		ret = fdt_status_okay_by_alias(rw_fdt_blob, "usdhc4");
	}
	else {
		ret = fdt_status_okay_by_alias(rw_fdt_blob, "usdhc2");
	}

	if (ret) {
		printf("%s: failed to enable usdhc node, ret=%d\n", __func__, ret);
	}

	return ret;
}
