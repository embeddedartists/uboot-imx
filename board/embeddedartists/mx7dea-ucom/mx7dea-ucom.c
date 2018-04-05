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
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#if defined(CONFIG_CMD_EADISP)
#include <asm/imx-common/eadisp.h>
#include <asm/imx-common/eatouch.h>
#endif
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <power/pmic.h>
#ifdef CONFIG_SYS_I2C_MXC
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#endif
#if defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#include <asm/arch/crm_regs.h>
#include <usb.h>
#include <usb/ehci-fsl.h>
#ifdef CONFIG_VIDEO_MXS
#include <linux/fb.h>
#include <mxsfb.h>
#endif

#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

#include "../common/mx6ea_eeprom.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_DSE_3P3V_49OHM | \
	PAD_CTL_PUS_PU100KOHM | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)
#define ENET_PAD_CTRL_MII  (PAD_CTL_DSE_3P3V_32OHM)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)

#define I2C_PAD_CTRL    (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU100KOHM)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_PU100KOHM | \
	PAD_CTL_DSE_3P3V_49OHM)

#define QSPI_PAD_CTRL	\
	(PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

#define SPI_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)

#define BUTTON_PAD_CTRL    (PAD_CTL_PUS_PU5KOHM | PAD_CTL_DSE_3P3V_98OHM)

#define NAND_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)

#define NAND_PAD_READY0_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUS_PU5KOHM)

#define EPDC_PAD_CTRL	0x0

#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C1_SCL__GPIO4_IO8 | PC,
		.gp = IMX_GPIO_NR(4, 8),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C1_SDA__GPIO4_IO9 | PC,
		.gp = IMX_GPIO_NR(4, 9),
	},
};

/* I2C3 */
struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C3_SCL__I2C3_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C3_SCL__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C3_SDA__I2C3_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C3_SDA__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13),
	},
};
#endif



int dram_init(void)
{
	ea_eeprom_config_t config;

	// default size from configuration file
	gd->ram_size = PHYS_SDRAM_SIZE;

        // getting actual size from eeprom configuration

        if (ea_eeprom_get_config(&config) == 0) {
                /* ddr_size is given in MB */
                gd->ram_size = (config.ddr_size << 20);
        }

	return 0;
}

static iomux_v3_cfg_t const wdog_pads[] = {
	MX7D_PAD_GPIO1_IO00__WDOG1_WDOG_B | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const uart1_pads[] = {
	MX7D_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX7D_PAD_SD1_CLK__SD1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_CMD__SD1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA0__SD1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA1__SD1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA2__SD1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA3__SD1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	MX7D_PAD_SD1_CD_B__GPIO5_IO0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_RESET_B__GPIO5_IO2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_emmc_pads[] = {
	MX7D_PAD_SD3_CLK__SD3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_CMD__SD3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_STROBE__SD3_STROBE	 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	MX7D_PAD_SD3_RESET_B__GPIO6_IO11 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
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
		gpio_direction_output(IMX_GPIO_NR(2, 25) , 1);

		/* Set Brightness to high */
		gpio_direction_output(IMX_GPIO_NR(1, 1) , 1);

		/* Backlight power enable */
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
	char const *panel = getenv("panel");
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

#ifdef CONFIG_EA_IMX_PTP
#define TFP410_ADDR 0x3A
/*
 * Configure the TFP410 chip on the iMX PTP board.
 */
static void configure_tfp410(void)
{
       uint8_t buff[15] = {0};
       printf("Override for mfgtool\n");
       i2c_set_bus_num(0);
       i2c_init(CONFIG_SYS_I2C_SPEED, TFP410_ADDR);
       if (i2c_probe(TFP410_ADDR)) {
               printf("Failed to probe TFP410\n");
               return;
       }
       if (i2c_read(TFP410_ADDR, 0, 1, buff, 5)) {
               printf("Failed to read reg 0x00 from TFP410\n");
               return;
       }
       printf("TFP410: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", buff[0], buff[1], buff[2], buff[3], buff[4]);
       buff[0] = 0x35;
       buff[1] = 0x3d;
       buff[2] = 0x80;
       if (i2c_write(TFP410_ADDR, 0x08, 1, buff, 3)) {
               printf("Failed to write to reg 0x08-0x0a on TFP410\n");
               return;
       }
       buff[0] = 0x16;
       buff[1] = 0x40;
       buff[2] = 0x50;
       buff[3] = 0x00;
       buff[4] = 0x80;
       buff[5] = 0x02;
       buff[6] = 0xe0;
       buff[7] = 0x01;
       //if (i2c_write(TFP410_ADDR, 0x32, 1, buff, 8)) {
       //        printf("Failed to write to reg 0x32-0x39 on TFP410\n");
       //        return;
       //}
       printf("TFP410: DE generator disabled\n");
       printf("Successfully initialized TFP410!\n");
}
#endif

#ifdef CONFIG_FEC_MXC
static iomux_v3_cfg_t const fec1_pads[] = {
	MX7D_PAD_ENET1_RGMII_RX_CTL__ENET1_RGMII_RX_CTL | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD0__ENET1_RGMII_RD0 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD1__ENET1_RGMII_RD1 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD2__ENET1_RGMII_RD2 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD3__ENET1_RGMII_RD3 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RXC__ENET1_RGMII_RXC | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TX_CTL__ENET1_RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TD0__ENET1_RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TD1__ENET1_RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TD2__ENET1_RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TD3__ENET1_RGMII_TD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TXC__ENET1_RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_GPIO1_IO10__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL_MII),
	MX7D_PAD_GPIO1_IO11__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL_MII),
	MX7D_PAD_ENET1_COL__GPIO7_IO15 | MUX_PAD_CTRL(ENET_PAD_CTRL), /*
	MX7D_PAD_ENET1_RX_CLK__GPIO7_IO13 |MUX_PAD_CTRL(ENET_PAD_CTRL), */
};

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_pads, ARRAY_SIZE(fec1_pads));
}
#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_QSPI
static iomux_v3_cfg_t const quadspi_pads[] = {
	MX7D_PAD_EPDC_DATA00__QSPI_A_DATA0 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA01__QSPI_A_DATA1 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA02__QSPI_A_DATA2 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA03__QSPI_A_DATA3 | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA05__QSPI_A_SCLK  | MUX_PAD_CTRL(QSPI_PAD_CTRL),
	MX7D_PAD_EPDC_DATA06__QSPI_A_SS0_B | MUX_PAD_CTRL(QSPI_PAD_CTRL),
};

int board_qspi_init(void)
{
	/* Set the iomux */
	imx_iomux_v3_setup_multiple_pads(quadspi_pads, ARRAY_SIZE(quadspi_pads));

	/* Set the clock */
	set_clk_qspi();

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC

#define USDHC1_CD_GPIO	IMX_GPIO_NR(5, 0)
#define USDHC1_PWR_GPIO	IMX_GPIO_NR(5, 2)
#define USDHC3_PWR_GPIO IMX_GPIO_NR(6, 11)

static struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC1_BASE_ADDR, 0, 4},
	{USDHC3_BASE_ADDR},
};

int board_mmc_get_env_dev(int devno)
{
        if (devno == 2)
                devno--;

        return devno;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	if (1 == dev_no)
		dev_no++;

	return dev_no;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = 1; /* Assume uSDHC3 emmc is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int i, ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc2                    USDHC3 (eMMC)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
#if defined(CONFIG_SPL_BUILD)
                // The SPL framework expects there to be only one MMC device
                // and we always loads u-boot from eMMC which is mapped to mmc1
                if (i != 1) continue;
#endif

		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			gpio_request(USDHC1_CD_GPIO, "usdhc1_cd");
			gpio_direction_input(USDHC1_CD_GPIO);
			gpio_request(USDHC1_PWR_GPIO, "usdhc1_pwr");
			gpio_direction_output(USDHC1_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC1_PWR_GPIO, 1);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_emmc_pads, ARRAY_SIZE(usdhc3_emmc_pads));
			gpio_request(USDHC3_PWR_GPIO, "usdhc3_pwr");
			gpio_direction_output(USDHC3_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC3_PWR_GPIO, 1);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return 0;
			}

			ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
			if (ret)
				return ret;
	}

	return 0;
}

int check_mmc_autodetect(void)
{
	char *autodetect_str = getenv("mmcautodetect");

	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

	return 0;
}

void board_late_mmc_env_init(void)
{
        char cmd[32];
        char mmcblk[32];
        u32 dev_no = mmc_get_env_dev();

        if (!check_mmc_autodetect())
                return;

        setenv_ulong("mmcdev", dev_no);

        /* Set mmcblk env */
        sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
                mmc_map_to_kernel_blk(dev_no));
        setenv("mmcroot", mmcblk);

        sprintf(cmd, "mmc dev %d", dev_no);
        run_command(cmd, 0);
}

#endif

#ifdef CONFIG_FEC_MXC
int board_eth_init(bd_t *bis)
{
	int ret;
	ea_eeprom_config_t config;

	setup_iomux_fec();

	ret = fecmxc_initialize_multi(bis, 0,
		CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
	if (ret)
		printf("FEC1 MXC: %s:failed\n", __func__);

        /* stored MAC addresses to env variables */
        if (ea_eeprom_get_config(&config) == 0) {

                if (is_valid_ethaddr(config.mac1) && !getenv("ethaddr")) {
                        eth_setenv_enetaddr("ethaddr", config.mac1);
                }

                if (is_valid_ethaddr(config.mac2) && !getenv("eth1addr")) {
                        eth_setenv_enetaddr("eth1addr", config.mac2);
                }

                if (is_valid_ethaddr(config.mac3) && !getenv("eth2addr")) {
                        eth_setenv_enetaddr("eth2addr", config.mac3);
                }

                if (is_valid_ethaddr(config.mac4) && !getenv("eth3addr")) {
                        eth_setenv_enetaddr("eth3addr", config.mac4);
                }

        }

	return ret;
}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;
	int ret;

	/* Use 125M anatop REF_CLK1 for ENET1, clear gpr1[13], gpr1[17]*/
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
		(IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK |
		 IOMUXC_GPR_GPR1_GPR_ENET1_CLK_DIR_MASK), 0);


	/* enet reset (active high) before pwr en */
	gpio_direction_output(IMX_GPIO_NR(7, 15) , 1);
	udelay(10000);
	gpio_direction_output(IMX_GPIO_NR(7, 15) , 0);

	ret = set_clk_enet(ENET_125MHz);
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

void board_get_hwaddr(int dev_id, unsigned char *mac)
{
        ea_eeprom_config_t config;

        if (ea_eeprom_get_config(&config) == 0) {
                if (dev_id == 0) {
                        memcpy(mac, config.mac1, 6);
                }
                else {
                        memcpy(mac, config.mac2, 6);
                }
        }

}


#endif

#ifdef CONFIG_USB_EHCI_MX7
iomux_v3_cfg_t const usb_otg1_pads[] = {
	MX7D_PAD_GPIO1_IO05__USB_OTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const usb_otg2_pads[] = {
	MX7D_PAD_GPIO1_IO07__USB_OTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg1_pads,
                                                 ARRAY_SIZE(usb_otg1_pads));

	imx_iomux_v3_setup_multiple_pads(usb_otg2_pads,
                                                 ARRAY_SIZE(usb_otg2_pads));
}

int board_usb_phy_mode(int port)
{
        if (port == 0)
                return usb_phy_mode(port);
        else
                return USB_INIT_HOST;
}


#endif


int board_early_init_f(void)
{
	setup_iomux_uart();

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);
#endif

#ifdef CONFIG_USB_EHCI_MX7
        setup_usb();
#endif


	ea_eeprom_init();

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;


#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#ifdef CONFIG_SYS_USE_NAND
	setup_gpmi_nand();
#endif

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif


#ifdef CONFIG_CMD_EADISP
	eadisp_setup_display(displays, ARRAY_SIZE(displays));
#endif
#ifdef CONFIG_EA_IMX_PTP
	configure_tfp410();
#endif
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1", MAKE_CFGVAL(0x10, 0x10, 0x00, 0x00)},
	{"emmc", MAKE_CFGVAL(0x10, 0x2a, 0x00, 0x00)},
	/* TODO: Nand */
	{"qspi", MAKE_CFGVAL(0x00, 0x40, 0x00, 0x00)},
	{NULL,   0},
};
#endif

#ifdef CONFIG_POWER
#define I2C_PMIC	0
#define POWER_PMIC_BD71815_I2C_ADDR (0x4B)
#define BD71815_REG_LDO_MODE1 (0x10)
#define BD71815_REG_LDO3_VOLT (0x16)
int power_init_board(void)
{
	uint8_t value;

	/*
	 * Fix for LDO3 when DCIN is not supplied.
	 */

	i2c_set_bus_num(I2C_PMIC);

	if (i2c_probe(POWER_PMIC_BD71815_I2C_ADDR)) {
		printf("failed to probe PMIC (%x)\n", POWER_PMIC_BD71815_I2C_ADDR);
		return -1;
	}

	if (i2c_read(POWER_PMIC_BD71815_I2C_ADDR, BD71815_REG_LDO_MODE1, 1, &value, 1)) {
		printf("power_init_board: failed to read register 0x10\n");
		return -1;
	}

	/* Bit 2: LDO3_REG_MODE_0. When set to 1 LDO3 is controlled via register */
	value |= (1<<2);
	if (i2c_write(POWER_PMIC_BD71815_I2C_ADDR, BD71815_REG_LDO_MODE1, 1, &value, 1)) {
		printf("power_init_board: failed to write to reg 0x10\n");
	}

	/* 3.15V */
	value = 0x2f;
	if (i2c_write(POWER_PMIC_BD71815_I2C_ADDR, BD71815_REG_LDO3_VOLT, 1, &value, 1)) {
		printf("power_init_board: failed to write to reg 0x16\n");
	}

	return 0;
}
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

#ifdef CONFIG_CMD_EADISP
	eatouch_init();
#endif

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	return 0;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

int checkboard(void)
{
	ea_eeprom_config_t config;

	puts("Board: Embedded Artists ");
        if (ea_eeprom_get_config(&config) == 0) {

                printf("%s\n", config.name);
                printf("       %05d, %s, WO%d\n",
                        config.board_part_nr,
                        config.board_rev,
                        config.batch);

        }
        else {
                puts(" [Unknown board due to invalid configuration data]\n");
        }

	return 0;
}


#ifdef CONFIG_FSL_FASTBOOT

void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc0");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc1");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices\n");
		break;
	}
}

#ifdef CONFIG_ANDROID_RECOVERY

/* Use S3 button for recovery key */
#define GPIO_VOL_DN_KEY IMX_GPIO_NR(5, 10)
iomux_v3_cfg_t const recovery_key_pads[] = {
	(MX7D_PAD_SD2_WP__GPIO5_IO10 | MUX_PAD_CTRL(BUTTON_PAD_CTRL)),
};

int check_recovery_cmd_file(void)
{
	int button_pressed = 0;
	int recovery_mode = 0;

	recovery_mode = recovery_check_and_clean_flag();

	/* Check Recovery Combo Button press or not. */
	imx_iomux_v3_setup_multiple_pads(recovery_key_pads,
		ARRAY_SIZE(recovery_key_pads));

	gpio_direction_input(GPIO_VOL_DN_KEY);

	if (gpio_get_value(GPIO_VOL_DN_KEY) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
	}

	return recovery_mode || button_pressed;
}

void board_recovery_setup(void)
{
	int bootdev = get_boot_device();

	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "boota mmc0 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "boota mmc1 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}

	printf("setup env for recovery..\n");
	setenv("bootcmd", "run bootcmd_android_recovery");
}
#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FSL_FASTBOOT*/
