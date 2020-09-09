/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
 
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
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
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#ifdef CONFIG_SYS_I2C_MXC
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#endif
#include <asm/arch/crm_regs.h>
#include <usb.h>
#include <usb/ehci-ci.h>
#ifdef CONFIG_VIDEO_MXS
#include <linux/fb.h>
#include <mxsfb.h>
#endif

#include <dm.h>
#include <fdt_support.h>

#include "../common/ea_eeprom.h"
#include "../common/ea_common.h"
#include "../common/ea_gpio_expander.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_DSE_3P3V_49OHM | \
	PAD_CTL_PUS_PU100KOHM | PAD_CTL_HYS)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_PU100KOHM | \
	PAD_CTL_DSE_3P3V_49OHM)


int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	if (gd->ram_size == 0)
		gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

static iomux_v3_cfg_t const wdog_pads[] = {
	MX7D_PAD_GPIO1_IO00__WDOG1_WDOG_B | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const uart1_pads[] = {
	MX7D_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX7D_PAD_LCD_CLK__LCD_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_ENABLE__LCD_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_HSYNC__LCD_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_VSYNC__LCD_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA00__LCD_DATA0 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA01__LCD_DATA1 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA02__LCD_DATA2 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA03__LCD_DATA3 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA04__LCD_DATA4 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA05__LCD_DATA5 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA06__LCD_DATA6 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA07__LCD_DATA7 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA08__LCD_DATA8 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA09__LCD_DATA9 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA10__LCD_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA11__LCD_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA12__LCD_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA13__LCD_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA14__LCD_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA15__LCD_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA16__LCD_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA17__LCD_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA18__LCD_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA19__LCD_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA20__LCD_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA21__LCD_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA22__LCD_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA23__LCD_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	/*
	 * Use GPIO for Brightness adjustment, duty cycle = period.
	 */
	MX7D_PAD_GPIO1_IO01__GPIO1_IO1 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Display power enable */
	MX7D_PAD_EPDC_GDOE__GPIO2_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Backlight enable */
	MX7D_PAD_EPDC_GDCLK__GPIO2_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#ifdef CONFIG_CMD_EADISP

struct lcd_panel_info_t {
	unsigned int lcdif_base_addr;
	int depth;
	void	(*enable)(struct lcd_panel_info_t const *dev);
	struct fb_videomode mode;
};

void board_enable_rgb(const struct display_info_t *di, int enable)
{
	if (enable) {
		imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

		mdelay(100); /* let panel sync up before enabling backlight */

		/* Power up the LCD */
		gpio_request(IMX_GPIO_NR(2, 25), "lcd pwr");
		gpio_direction_output(IMX_GPIO_NR(2, 25) , 1);

		/* Set Brightness to high */
		gpio_request(IMX_GPIO_NR(1, 1), "lcd brightness");
		gpio_direction_output(IMX_GPIO_NR(1, 1) , 1);

		/* Backlight power enable */
		gpio_request(IMX_GPIO_NR(2, 24), "backlight pwr");
		gpio_direction_output(IMX_GPIO_NR(2, 24) , 1);
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

void do_enable_parallel_lcd(struct lcd_panel_info_t const *dev)
{
	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	mdelay(100); /* let panel sync up before enabling backlight */

	/* Power up the LCD */
	gpio_direction_output(IMX_GPIO_NR(2, 25) , 1);

	/* Set Brightness to high */
	gpio_direction_output(IMX_GPIO_NR(1, 1) , 1);

	/* Backlight power enable */
	gpio_direction_output(IMX_GPIO_NR(2, 24) , 1);
}

static struct lcd_panel_info_t const displays[] = {{
	.lcdif_base_addr = ELCDIF1_IPS_BASE_ADDR,
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

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_QSPI
int board_qspi_init(void)
{
	/* Set the clock */
	set_clk_qspi();

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC

int board_mmc_get_env_dev(int devno)
{
        return devno - 1;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no + 1;
}

#endif

#ifdef CONFIG_FEC_MXC

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;
	int ret;
	unsigned int gpio;

	/* Use 125M anatop REF_CLK1 for ENET1, clear gpr1[13], gpr1[17]*/
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
		(IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK |
		 IOMUXC_GPR_GPR1_GPR_ENET1_CLK_DIR_MASK), 0);

	ret = gpio_lookup_name("GPIO7_15", NULL, NULL, &gpio);
	if (ret) {
		printf("%s: GPIO7_17 not found\n", __func__);
		return -ENODEV;
	}

	ret = gpio_request(gpio, "enet_phy_rst");
	if (ret && ret != -EBUSY) {
		printf("gpio: requesting pin %u failed\n", gpio);
		return ret;
	}

	/* enet reset (active high) before pwr en */
	gpio_direction_output(gpio , 1);
	udelay(10000);
	gpio_direction_output(gpio , 0);

	ret = set_clk_enet(ENET_125MHZ);
	if (ret)
		return ret;

	return 0;
}


int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */

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

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
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
#ifdef CONFIG_DM_PMIC

#ifdef CONFIG_TARGET_MX7DEA_COM
int power_init_board(void)
{
	struct udevice *dev;
	int ret, dev_id, rev_id;
	u32 sw3mode;

	ret = pmic_get("pfuze3000@8", &dev);
	if (ret == -ENODEV)
		return 0;
	if (ret != 0)
		return ret;

	dev_id = pmic_reg_read(dev, PFUZE3000_DEVICEID);
	rev_id = pmic_reg_read(dev, PFUZE3000_REVID);
	printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", dev_id, rev_id);


	/* change sw3 mode to avoid DDR power off */
	sw3mode = pmic_reg_read(dev, PFUZE3000_SW3MODE);
	ret = pmic_reg_write(dev, PFUZE3000_SW3MODE, sw3mode | 0x20);
	if (ret < 0)
		printf("PMIC: PFUZE3000 change sw3 mode failed\n");

	return 0;
}

#else /* TARGET_MX7DEA_UCOM */

#define I2C_PMIC	0
#define POWER_PMIC_BD71815_I2C_ADDR (0x4B)
#define BD71815_REG_LDO_MODE1 (0x10)
#define BD71815_REG_LDO3_VOLT (0x16)
int power_init_board(void)
{
	struct udevice *bus;
	struct udevice *i2c_dev;
	int ret;
	uint8_t value;

	ret = uclass_get_device_by_seq(UCLASS_I2C, I2C_PMIC, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return -EINVAL;
	}

        ret = dm_i2c_probe(bus, POWER_PMIC_BD71815_I2C_ADDR, 0, &i2c_dev);
        if (ret) {
                printf("%s: Can't find device id=0x%x\n",
                        __func__, POWER_PMIC_BD71815_I2C_ADDR);
                return -ENODEV;
        }

	/*
	 * Fix for LDO3 when DCIN is not supplied.
	 */

        ret = dm_i2c_read(i2c_dev, BD71815_REG_LDO_MODE1, &value, 1);
        if (ret) {
                printf("%s dm_i2c_read failed, err %d\n", __func__, ret);
                return -EIO;
        }

	/* Bit 2: LDO3_REG_MODE_0. When set to 1 LDO3 is controlled via register */
	value |= (1<<2);
	ret = dm_i2c_write(i2c_dev, BD71815_REG_LDO_MODE1, &value, 1);
	if (ret) {
		printf("%s failed to write to %x, err %d\n", __func__, BD71815_REG_LDO_MODE1, ret);
		return -EIO;
	}

	/* 3.15V */
	value = 0x2f;
	ret = dm_i2c_write(i2c_dev, BD71815_REG_LDO3_VOLT, &value, 1);
	if (ret) {
		printf("%s failed to write to %x, err %d\n", __func__, BD71815_REG_LDO3_VOLT, ret);
		return -EIO;
	}

	return 0;
}

#endif
#endif /* CONFIG_DM_PMI */

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

	ea_gpio_exp_configure(1);

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	return 0;
}

int board_fix_fdt(void* rw_fdt_blob)
{
	int ret = 0;
	bool is_v2 = false;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	if (ea_conf->magic == EA_CONFIG_MAGIC) {
		is_v2 = ea_conf->is_carrier_v2;
	}

	/*
	 * On Carrier board V2 the usdhc2 interfaces is connected to
	 * MMC/SD card connector. On older versions usdhc1 us used.
	 */
	if (is_v2) {
		ret = fdt_status_okay_by_alias(rw_fdt_blob, "usdhc2");
	}
	else {
		ret = fdt_status_okay_by_alias(rw_fdt_blob, "usdhc1");
	}

	if (ret) {
		printf("%s: failed to enable usdhc node, ret=%d\n", __func__, ret);
	}

	return ret;
}


