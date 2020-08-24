/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6ul_pins.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/mxc_i2c.h>
#if defined(CONFIG_CMD_EADISP)
#include <asm/mach-imx/eadisp.h>
#include <asm/mach-imx/eatouch.h>
#endif
#include <asm/io.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <linux/sizes.h>
#include <linux/fb.h>
#include <miiphy.h>
#include <mmc.h>
#include <mxsfb.h>
#include <netdev.h>
#include <usb.h>
#include <usb/ehci-ci.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>

#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
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

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define PERI_PWR_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_DSE_40ohm)

iomux_v3_cfg_t const peri_pwr_pads[] = {
	(MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(PERI_PWR_PAD_CTRL)),
};

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#ifdef CONFIG_FEC_MXC
/*
 * pin conflicts for fec1 and fec2, GPIO1_IO06 and GPIO1_IO07 can only
 * be used for ENET1 or ENET2, cannot be used for both.
 */

static iomux_v3_cfg_t const enet_pwr_pads[] = {
	/* enet pwr en */
	MX6_PAD_SNVS_TAMPER3__GPIO5_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL),
};


#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_ESDHC

int mmc_map_to_kernel_blk(int dev_no)
{
	/* eMMC device is available at mmcblk1 in Linux */
	return 1;
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
	 * For the iMX6 Ultralite COM board eMMC is on USDHC2 which gives devno=1.
	 *
	 * When using the Device Module (dts) USDHC devices are added
	 * in the order they are defined in the dts file. Depending on
	 * which carrier board being used USDHC1 might also be enabled
	 * (in board_fix_fdt)
	 * This however effects which device number being assigned to eMMC.
	 * When USDHC1 is enabled eMMC will be on mmc1 (since USDHC1
	 * is on mmc0). When USDHC1 isn't enabled eMMC will be on mmc0.
	 */

	if (is_v2) {
		no = 0;
	}
	else {
		no = 1;
	}

	return no;

}

#endif

#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_LCD_CLK__LCDIF_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_ENABLE__LCDIF_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_HSYNC__LCDIF_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_VSYNC__LCDIF_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA00__LCDIF_DATA00 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA01__LCDIF_DATA01 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA02__LCDIF_DATA02 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA03__LCDIF_DATA03 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA04__LCDIF_DATA04 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA05__LCDIF_DATA05 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA06__LCDIF_DATA06 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA07__LCDIF_DATA07 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA08__LCDIF_DATA08 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA09__LCDIF_DATA09 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA10__LCDIF_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA11__LCDIF_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA12__LCDIF_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA13__LCDIF_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA14__LCDIF_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA15__LCDIF_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA16__LCDIF_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA17__LCDIF_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA18__LCDIF_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA19__LCDIF_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA20__LCDIF_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA21__LCDIF_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA22__LCDIF_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA23__LCDIF_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	/*
	 * Use GPIO for Brightness adjustment, duty cycle = period.
	 */
	MX6_PAD_GPIO1_IO08__GPIO1_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Display power enable */
	MX6_PAD_GPIO1_IO02__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* Backlight enable */
	MX6_PAD_GPIO1_IO01__GPIO1_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#ifdef CONFIG_CMD_EADISP
void board_enable_rgb(const struct display_info_t *di, int enable)
{
	if (enable) {
		enable_lcdif_clock(LCDIF1_BASE_ADDR, 1);

		imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

		mdelay(100); /* let panel sync up before enabling backlight */

		/* Power up the LCD */
		gpio_request(IMX_GPIO_NR(1, 2), "lcd pwr");
		gpio_direction_output(IMX_GPIO_NR(1, 2) , 1);

		/* Set Brightness to high */
		gpio_request(IMX_GPIO_NR(1, 8), "lcd brightness");
		gpio_direction_output(IMX_GPIO_NR(1, 8) , 1);

		/* Backlight power enable */
		gpio_request(IMX_GPIO_NR(1, 1), "nacklight pwr");
		gpio_direction_output(IMX_GPIO_NR(1, 1) , 1);
	}
}

static const struct display_info_t displays[] = {
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
	void (*enable)(struct lcd_panel_info_t const *dev);
	struct fb_videomode mode;
};

void do_enable_parallel_lcd(struct lcd_panel_info_t const *dev)
{
	enable_lcdif_clock(dev->lcdif_base_addr, 1);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	/* Power up the LCD */
	gpio_request(IMX_GPIO_NR(1, 2), "lcd pwr");
	gpio_direction_output(IMX_GPIO_NR(1, 2) , 1);

	/* Set Brightness to high */
	gpio_request(IMX_GPIO_NR(1, 8), "lcdbrightness");
	gpio_direction_output(IMX_GPIO_NR(1, 8) , 1);

	/* Backlight power enable */
	gpio_request(IMX_GPIO_NR(1, 1), "backlight pwr");
	gpio_direction_output(IMX_GPIO_NR(1, 1) , 1);
}

static struct lcd_panel_info_t const displays[] = {{
	.lcdif_base_addr = MX6UL_LCDIF1_BASE_ADDR,
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
#endif

#ifdef CONFIG_FEC_MXC

static int setup_fec(int fec_id)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;
	if (0 == fec_id) {
                if (check_module_fused(MX6_MODULE_ENET1))
                        return -1;

		/*
		 * Use 50M anatop loopback REF_CLK1 for ENET1,
		 * clear gpr1[13], set gpr1[17]
		 */
		clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
				IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
	} else {
                if (check_module_fused(MX6_MODULE_ENET2))
                        return -1;

		/*
		 * Use 50M anatop loopback REF_CLK2 for ENET2,
		 * clear gpr1[14], set gpr1[18]
		 */
		clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
				IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);
	}

	ret = enable_fec_anatop_clock(fec_id, ENET_50MHZ);
	if (ret)
		return ret;

	enable_enet_clk(1);

	imx_iomux_v3_setup_multiple_pads(enet_pwr_pads, ARRAY_SIZE(enet_pwr_pads));

	/* enet pwr en */
	gpio_request(IMX_GPIO_NR(5, 3), "enet pwr");
	gpio_direction_output(IMX_GPIO_NR(5, 3) , 0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8190);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x16, 0x202);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#endif

int power_init_board(void)
{
	struct udevice *dev;
	int ret, dev_id, rev_id;

	ret = pmic_get("pfuze3000", &dev);
	if (ret == -ENODEV)
		return 0;
	if (ret != 0)
		return ret;

	dev_id = pmic_reg_read(dev, PFUZE3000_DEVICEID);
	rev_id = pmic_reg_read(dev, PFUZE3000_REVID);
	printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", dev_id, rev_id);

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	u32 vddarm;
	struct udevice *dev;
	int ret;

	ret = pmic_get("pfuze3000", &dev);
	if (ret == -ENODEV) {
		printf("No PMIC found!\n");
		return;
	}

	/* switch to ldo_bypass mode */
	if (ldo_bypass) {
		prep_anatop_bypass();
		/* decrease VDDARM to 1.275V */
		value = pmic_reg_read(dev, PFUZE3000_SW1BVOLT);
		value &= ~0x1f;
		value |= PFUZE3000_SW1AB_SETP(12750);
		pmic_reg_write(dev, PFUZE3000_SW1BVOLT, value);

		set_anatop_bypass(1);
		vddarm = PFUZE3000_SW1AB_SETP(11750);

		value = pmic_reg_read(dev, PFUZE3000_SW1BVOLT);
		value &= ~0x1f;
		value |= vddarm;
		pmic_reg_write(dev, PFUZE3000_SW1BVOLT, value);

		finish_anatop_bypass();

		printf("switch to ldo_bypass mode!\n");
	}
}
#endif

int board_early_init_f(void)
{
	return 0;
}

int board_init_common(void)
{
	/* configure and enable pwr on carrier board*/
	imx_iomux_v3_setup_multiple_pads(peri_pwr_pads,
			ARRAY_SIZE(peri_pwr_pads));
	gpio_request(IMX_GPIO_NR(5, 2), "peri 3.3  pwr");
	gpio_direction_output(IMX_GPIO_NR(5, 2), 1);

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

#ifdef	CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif


#ifdef CONFIG_CMD_EADISP
	eadisp_setup_display(displays, ARRAY_SIZE(displays));
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
	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

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
	 * On Carrier board V2 there isn't any SD card interface available
	 * since this interface is connected to M.2 connector.
	 * On V1 boards USDHC1 is used.
	 */
	if (!is_v2) {
		ret = fdt_status_okay_by_alias(rw_fdt_blob, "usdhc1");
	}

	if (ret) {
		printf("%s: failed to enable usdhc node, ret=%d\n", __func__, ret);
	}

	return ret;
}
