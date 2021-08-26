/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#if defined(CONFIG_CMD_EADISP)
#include <asm/mach-imx/eadisp.h>
#include <asm/mach-imx/eatouch.h>
#endif
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>

#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#ifdef CONFIG_SYS_I2C_MXC
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#endif
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <usb.h>
#ifdef CONFIG_FSL_FASTBOOT
#include <fb_fsl.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

#include <dm.h>
#include <fdt_support.h>

#include "../common/ea_eeprom.h"
#include "../common/ea_common.h"
#include "../common/ea_gpio_expander.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)


iomux_v3_cfg_t const peri_pwr_pads[] = {
	(MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* enable USB Host 5V. Doesn't seem to get this to work in Linux/DTS */
	(MX6_PAD_GPIO_0__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_CSI0_DAT12__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT13__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] = {
	/* AR8031 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* enet pwr en */
	MX6_PAD_GPIO_18__GPIO7_IO13		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* enet pwr en */
	gpio_direction_output(IMX_GPIO_NR(7, 13) , 0);
	udelay(1000);

	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
}

iomux_v3_cfg_t const rgb_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,	/* DISP0_CLK */
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,		/* DISP0_HSYNC */
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,		/* DISP0_VSYNC */
	MX6_PAD_DI0_PIN4__IPU1_DI0_PIN04,
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00,
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01,
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02,
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03,
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04,
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05,
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06,
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07,
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08,
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09,
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10,
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11,
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12,
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13,
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14,
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15,
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16,
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17,
	MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18,
	MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19,
	MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20,
	MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21,
	MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22,
	MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23,
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	int is_400M;
	unsigned char vddarm;
	struct udevice *dev;
	int ret;

	ret = pmic_get("pfuze100", &dev);
	if (ret == -ENODEV) {
		printf("No PMIC found!\n");
		return;
	}

	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0; /* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		if (is_mx6dqp()) {
			/* increase VDDARM to 1.425V */
			pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x29);
		} else {
			/* increase VDDARM to 1.425V */
			pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, 0x2d);
		}
		/* increase VDDSOC to 1.425V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, 0x2d);
	}
	/* switch to ldo_bypass mode , boot on 800Mhz */
	if (ldo_bypass) {
		prep_anatop_bypass();
		if (is_mx6dqp()) {
			/* decrease VDDARM for 400Mhz DQP:1.1V*/
			pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x1c);
		} else {
			/* decrease VDDARM for 400Mhz DQ:1.1V, DL:1.275V */
			if (is_mx6dl())
				pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, 0x27);
			else
				pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, 0x20);
		}
		/* increase VDDSOC to 1.3V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, 0x28);

		/*
		 * MX6Q/DQP:
		 * VDDARM:1.15V@800M; VDDSOC:1.175V@800M
		 * VDDARM:0.975V@400M; VDDSOC:1.175V@400M
		 * MX6DL:
		 * VDDARM:1.175V@800M; VDDSOC:1.175V@800M
		 * VDDARM:1.15V@400M; VDDSOC:1.175V@400M
		 */
		is_400M = set_anatop_bypass(2);
		if (is_mx6dqp()) {
			if (is_400M)
				pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x17);
			else
				pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x1e);
		}

		if (is_400M) {
			if (is_mx6dl())
				vddarm = 0x22;
			else
				vddarm = 0x1b;
		} else {
			if (is_mx6dl())
				vddarm = 0x23;
			else
				vddarm = 0x22;
		}
		pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, vddarm);

		/* decrease VDDSOC to 1.175V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, 0x23);

		finish_anatop_bypass();
		printf("switch to ldo_bypass mode!\n");
	}
}
#endif

#ifdef CONFIG_FSL_ESDHC

int mmc_map_to_kernel_blk(int dev_no)
{
	bool is_v2 = false;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	if (ea_conf->magic == EA_CONFIG_MAGIC) {
		is_v2 = ea_conf->is_carrier_v2;
	}

	if (is_v2) {
		return dev_no + 2;
	} else {
		if (dev_no == 1) {
			return 3;
		} else {
			return 1;
		}
	}
}

int board_mmc_get_env_dev(int devno)
{
	int no = devno;
	bool is_v2 = false;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	if (ea_conf->magic == EA_CONFIG_MAGIC) {
		is_v2 = ea_conf->is_carrier_v2;
	}

	if (is_v2) {
		/* need to subtract 2 to map to the mmc device id
		 * see the comments in board_mmc_init function
		 */
		no -= 2;
	} else {
		/* need to map to the mmc device id
		 * see the comments in board_mmc_init function
		 */
		if (no == 3) {
			no = 1;
		} else {
			no = 0;
		}
	}

	return no;
}


#endif

static int ar8031_phy_fixup(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	ar8031_phy_fixup(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)

static iomux_v3_cfg_t const display_ctrl_pads[] = {
	/* Display enable for LVDS0 and Parallel RGB */
	MX6_PAD_EIM_BCLK__GPIO6_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define DISPENABLE_GP_LVDS0_RGB  IMX_GPIO_NR(6, 31)

	/* Display enable for LVDS1 */
	MX6_PAD_EIM_D20__GPIO3_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define DISPENABLE_GP_LVDS1  IMX_GPIO_NR(3, 20)


	/* LVDS0 and LVDS1 Brightness (gpio instead of PWM1, duty cycle = period) */
	MX6_PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define BRIGHTNESS_GP_LVDSx  IMX_GPIO_NR(1, 21)

	/* Parallel RGB Brightness (gpio instead of PWM2, duty cycle = period) */
	MX6_PAD_SD1_DAT2__GPIO1_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define BRIGHTNESS_GP_RGB  IMX_GPIO_NR(1, 19)


	/* Backlight Power Enable for LVDS0 and Parallel RGB */
	MX6_PAD_EIM_WAIT__GPIO5_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define BACKLIGHT_GP_LVDS0_RGB  IMX_GPIO_NR(5, 0)

	/* Backlight Power Enable for LVDS1 */
	MX6_PAD_EIM_LBA__GPIO2_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define BACKLIGHT_GP_LVDS1  IMX_GPIO_NR(2, 27)
};

#ifdef CONFIG_CMD_EADISP

void board_enable_rgb(const struct display_info_t *di, int enable)
{
	if (enable) {
		/* Setup pads for RGB */
		imx_iomux_v3_setup_multiple_pads(rgb_pads, ARRAY_SIZE(rgb_pads));

		/* Default interface is LVDS1 so pinning must be changed */
		gpio_request(DISPENABLE_GP_LVDS0_RGB, "lcd pwr");
		gpio_direction_output(DISPENABLE_GP_LVDS0_RGB, 1);

		gpio_request(BRIGHTNESS_GP_LVDSx, "brightness lvds");
		gpio_direction_output(BRIGHTNESS_GP_LVDSx, 0);

		gpio_request(BRIGHTNESS_GP_RGB, "brightness rgb");
		gpio_direction_output(BRIGHTNESS_GP_RGB, 1);

		gpio_request(BACKLIGHT_GP_LVDS0_RGB, "backlight");
		gpio_direction_output(BACKLIGHT_GP_LVDS0_RGB, 1);
	}
}
void board_enable_lvds0(const struct display_info_t *di, int enable)
{
	if (enable) {
		/* Default is lvds1 so change that to lvds0 */
		struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

		int reg = readl(&iomux->gpr[2]);

		reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
			 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);
		reg |= (IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0 |
			IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED);

		writel(reg, &iomux->gpr[2]);

		reg = readl(&iomux->gpr[3]);
		reg &= ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK |
			 IOMUXC_GPR3_LVDS1_MUX_CTL_MASK |
			 IOMUXC_GPR3_HDMI_MUX_CTL_MASK);
		reg |= (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
			   << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);

		writel(reg, &iomux->gpr[3]);

		/* Now change pinning as well */
		gpio_request(DISPENABLE_GP_LVDS0_RGB, "lcd pwr");
		gpio_direction_output(DISPENABLE_GP_LVDS0_RGB, 1);


		gpio_request(BRIGHTNESS_GP_LVDSx, "brightness lvds");
		gpio_direction_output(BRIGHTNESS_GP_LVDSx, 1);


		gpio_request(BRIGHTNESS_GP_RGB, "brightness rgb");
		gpio_direction_output(BRIGHTNESS_GP_RGB, 0);

		gpio_request(BACKLIGHT_GP_LVDS0_RGB, "backlight");
		gpio_direction_output(BACKLIGHT_GP_LVDS0_RGB, 1);
	}
}

void board_enable_lvds1(const struct display_info_t *di, int enable)
{
	if (enable) {
		/* Default interface so only pinning is needed */
		gpio_request(DISPENABLE_GP_LVDS1, "lcd pwr");
		gpio_direction_output(DISPENABLE_GP_LVDS1, 1);

		gpio_request(BRIGHTNESS_GP_LVDSx, "brightness lvds");
		gpio_direction_output(BRIGHTNESS_GP_LVDSx, 1);

		gpio_request(BACKLIGHT_GP_LVDS1, "backlight lvds1");
		gpio_direction_output(BACKLIGHT_GP_LVDS1, 1);
	}
}

void board_enable_hdmi(const struct display_info_t *di, int enable)
{
	if (enable) {
		/* Default is lvds1 so disable it */
		struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

		int reg = readl(&iomux->gpr[2]);

		reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
			 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

		writel(reg, &iomux->gpr[2]);

		/* Now enable HDMI */

		/* NOTE:
		 * We have seen problems with hdmi when it has been enabled both
		 * in u-boot and in the Linux kernel. The kernel could seem to freeze
		 * due to a massive amount of overflow interrupts.
		 * https://community.nxp.com/thread/342916
		 */

		/*imx_enable_hdmi_phy();*/
	}
}

static const struct display_info_t displays[] = {
	/* LVDS */
	EADISP_HANNSTAR10(LVDS0, 0, 0),
	EADISP_NHD_1024600AF(LVDS0, 0, 0),
	EADISP_HANNSTAR10(LVDS1, 0, 0),

	/* RGB */
	EADISP_INNOLUX_AT070TN(RGB, 0, 0),
	EADISP_NHD_43480272EF(RGB, 0, 0),
	EADISP_NHD_50800480TF(RGB, 0, 0),
	EADISP_NHD_70800480EF(RGB, 0, 0),
	EADISP_UMSH_8864(RGB, 0, 0),
	EADISP_UMSH_8596_30T(RGB, 0, 0),
	EADISP_UMSH_8596_33T(RGB, 0, 0),
	EADISP_ROGIN_RX050A(RGB, 0, 0),

	/* HDMI */
	EADISP_HDMI_1280_720M_60(HDMI, 0, 0),
	EADISP_HDMI_1920_1080M_60(HDMI, 0, 0),
	EADISP_HDMI_640_480M_60(HDMI, 0, 0),
	EADISP_HDMI_720_480M_60(HDMI, 0, 0),
};

#else  /* CONFIG_CMD_EADISP */
struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_lvds0(struct display_info_t const *dev)
{
	/* Default is lvds1 so change that to lvds0 */
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);
	reg |= (IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0 |
		IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED);

	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg &= ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK |
		 IOMUXC_GPR3_LVDS1_MUX_CTL_MASK |
		 IOMUXC_GPR3_HDMI_MUX_CTL_MASK);
	reg |= (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);

	writel(reg, &iomux->gpr[3]);

	/* Now change pinning as well */
	gpio_request(DISPENABLE_GP_LVDS0_RGB, "lcd pwr");
	gpio_direction_output(DISPENABLE_GP_LVDS0_RGB, 1);

	gpio_request(BRIGHTNESS_GP_LVDSx, "brightness lvds");
	gpio_direction_output(BRIGHTNESS_GP_LVDSx, 1);

	gpio_request(BRIGHTNESS_GP_RGB, "brightness rgb");
	gpio_direction_output(BRIGHTNESS_GP_RGB, 0);

	gpio_request(BACKLIGHT_GP_LVDS0_RGB, "backlight");
	gpio_direction_output(BACKLIGHT_GP_LVDS0_RGB, 1);
}

static void do_enable_lvds1(struct display_info_t const *dev)
{
	/* Default interface so only pinning is needed */
	gpio_request(DISPENABLE_GP_LVDS1, "lcd pwr");
	gpio_direction_output(DISPENABLE_GP_LVDS1, 1);

	gpio_request(BRIGHTNESS_GP_LVDSx, "brightness lvds");
	gpio_direction_output(BRIGHTNESS_GP_LVDSx, 1);

	gpio_request(BACKLIGHT_GP_LVDS1, "backlight lvds1");
	gpio_direction_output(BACKLIGHT_GP_LVDS1, 1);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	/* Default is lvds1 so disable it */
	disable_lvds(dev);

	/* NOTE:
	 * We have seen problems with hdmi when it has been enabled both
	 * in u-boot and in the Linux kernel. The kernel could seem to freeze
	 * due to a massive amount of overflow interrupts.
	 * https://community.nxp.com/thread/342916
	 */

	/* imx_enable_hdmi_phy();*/
}

static void do_enable_parallel_rgb(struct display_info_t const *dev)
{
	/* Setup pads for RGB */
	imx_iomux_v3_setup_multiple_pads(rgb_pads, ARRAY_SIZE(rgb_pads));

	/* Default interface is LVDS1 so pinning must be changed */
	gpio_request(DISPENABLE_GP_LVDS0_RGB, "lcd pwr");
	gpio_direction_output(DISPENABLE_GP_LVDS0_RGB, 1);

	gpio_request(BRIGHTNESS_GP_LVDSx, "brightness lvds");
	gpio_direction_output(BRIGHTNESS_GP_LVDSx, 0);

	gpio_request(BRIGHTNESS_GP_RGB, "brightness rgb");
	gpio_direction_output(BRIGHTNESS_GP_RGB, 1);

	gpio_request(BACKLIGHT_GP_LVDS0_RGB, "backlight");
	gpio_direction_output(BACKLIGHT_GP_LVDS0_RGB, 1);
}

static struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= do_enable_lvds1,
	.mode	= {
		.name           = "Hannstar-XGA-LVDS1",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= do_enable_lvds0,
	.mode	= {
		.name           = "Hannstar-XGA-LVDS0",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = 39721,
		.left_margin    = 48,
		.right_margin   = 16,
		.upper_margin   = 33,
		.lower_margin   = 10,
		.hsync_len      = 96,
		.vsync_len      = 2,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= do_enable_parallel_rgb,
	.mode	= {
		.name           = "Innolux-AT070TN",
		.refresh        = 60,
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
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays+i;
			if (dev->detect && dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			panel = displays[0].mode.name;
			printf("No panel detected: default to %s\n", panel);
			i = 0;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
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

#endif  /* CONFIG_CMD_EADISP */

int ipu_displays_init(void)
{
        return board_video_skip();
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

	/* DisplayEnable, BacklightEnable, Brightness controls for RGB, LVDS0 and LVDS1 */
	imx_iomux_v3_setup_multiple_pads(display_ctrl_pads,
					ARRAY_SIZE(display_ctrl_pads));
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();
	return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
	return 0;
}

int board_init_common(void)
{
	/*
	 * The functionality below was previously in board_early_init_f,
	 * but when moving to the Device Model gpio is not available
	 * before reallocation and hence PERI PWR wasn't enabled.
	 */

	/* configure and enable pwr */
	imx_iomux_v3_setup_multiple_pads(peri_pwr_pads,
			ARRAY_SIZE(peri_pwr_pads));
	gpio_request(IMX_GPIO_NR(7, 12), "peri pwr");
	gpio_direction_output(IMX_GPIO_NR(7, 12), 1);

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
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	board_init_common();

	/* enable USB 5V. Doesn't seem to work to do this in Linux/DTS?? */
	gpio_request(IMX_GPIO_NR(1, 0), "USB5V");
	gpio_direction_output(IMX_GPIO_NR(1, 0), 1);

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

#ifdef CONFIG_CMD_EADISP
	eadisp_setup_display(displays, ARRAY_SIZE(displays));
#endif
#ifdef CONFIG_EA_IMX_PTP
	ea_configure_tfp410();
#endif

	ea_print_board();

	return 0;
}

int power_init_board(void)
{
	struct udevice *dev;
	unsigned int reg, dev_id, rev_id;
	unsigned char offset, i, switch_num;
	int ret;

	ret = pmic_get("pfuze100@8", &dev);
	if (ret) {
		printf("%s: failed to get pmic\n", __func__);
		return ret;
	}

	dev_id = pmic_reg_read(dev, PFUZE100_DEVICEID);
	rev_id = pmic_reg_read(dev, PFUZE100_REVID);
	printf("PMIC: PFUZE100! DEV_ID=0x%x REV_ID=0x%x\n", dev_id, rev_id);

	/*increase VGEN5 from 2.8 to 3V*/
	reg = pmic_reg_read(dev, PFUZE100_VGEN5VOL);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_3_00V;
	pmic_reg_write(dev, PFUZE100_VGEN5VOL, reg);


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

	/* mode init  --> */

	if ((dev_id & 0xf) == 0) {
		switch_num = 6;
		offset = PFUZE100_SW1CMODE;
	} else if ((dev_id & 0xf) == 1) {
		switch_num = 4;
		offset = PFUZE100_SW2MODE;
	} else {
		printf("Not supported, id=%d\n", (dev_id & 0xf));
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

	/* mode init  <-- */


	/* set SW1AB staby volatage 0.975V*/
	reg = pmic_reg_read(dev, PFUZE100_SW1ABSTBY);
	reg &= ~0x3f;
	reg |= 0x1b;
	pmic_reg_write(dev, PFUZE100_SW1ABSTBY, reg);

	/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
	reg = pmic_reg_read(dev, PFUZE100_SW1ABCONF);
	reg &= ~0xc0;
	reg |= 0x40;
	pmic_reg_write(dev, PFUZE100_SW1ABCONF, reg);

	/* set SW1C staby volatage 0.975V*/
	reg = pmic_reg_read(dev, PFUZE100_SW1CSTBY);
	reg &= ~0x3f;
	reg |= 0x1b;
	pmic_reg_write(dev, PFUZE100_SW1CSTBY, reg);

	/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
	reg = pmic_reg_read(dev, PFUZE100_SW1CCONF);
	reg &= ~0xc0;
	reg |= 0x40;
	pmic_reg_write(dev, PFUZE100_SW1CCONF, reg);


	/*set SW3AB to 1.35V*/
	reg = pmic_reg_read(dev, PFUZE100_SW3AVOL);
	if (reg & 0x40)
		reg = 0x4B;
	else
		reg = 0x26;
	pmic_reg_write(dev, PFUZE100_SW3AVOL, reg);
	pmic_reg_write(dev, PFUZE100_SW3BVOL, reg);
	pmic_reg_write(dev, PFUZE100_SW3ASTBY, reg);
	pmic_reg_write(dev, PFUZE100_SW3BSTBY, reg);

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

	ea_gpio_exp_configure(2);

	return 0;
}

#ifdef CONFIG_USB_EHCI_MX6

int board_ehci_hcd_init(int port)
{
	switch (port) {
	case 0:
		/*
		 * Set daisy chain for otg_pin_id on 6q.
		 *  For 6dl, this bit is reserved.
		 */
		imx_iomux_set_gpr_register(1, 13, 1, 0);
		break;
	case 1:
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}
	return 0;
}

#endif


int board_fix_fdt(void* rw_fdt_blob)
{
	int ret = 0;

	bool is_v2 = false;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	if (ea_conf->magic == EA_CONFIG_MAGIC) {
		is_v2 = ea_conf->is_carrier_v2;
	}

	/*
	 * On Carrier board V2 the usdhc3 interfaces is connected to
	 * MMC/SD card connector. On older versions usdhc2 us used.
	 */
	if (is_v2) {
		ret = fdt_status_okay_by_alias(rw_fdt_blob, "usdhc3");
	}
	else {
		ret = fdt_status_okay_by_alias(rw_fdt_blob, "usdhc2");
	}

	if (ret) {
		printf("%s: failed to enable usdhc node, ret=%d\n", __func__, ret);
	}

	return ret;
}
