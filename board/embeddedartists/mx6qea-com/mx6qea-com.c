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
#include <usb.h>
#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

#include "../common/ea_eeprom.h"
#include "../common/ea_common.h"
#include "../common/ea_gpio_expander.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC3_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_48ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

iomux_v3_cfg_t const peri_pwr_pads[] = {
	(MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* enable USB Host 5V. Doesn't seem to get this to work in Linux/DTS */
	(MX6_PAD_GPIO_0__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 PMIC, EEPROM */

struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | PC,
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | PC,
		.gp = IMX_GPIO_NR(5, 26)
	}
};
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_5__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_5__GPIO1_IO05 | PC,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO_6__GPIO1_IO06 | PC,
		.gp = IMX_GPIO_NR(1, 6)
	}
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

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_CSI0_DAT12__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT13__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
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

/* USDHC2: v1:uSD Card, v2:wifi */
iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/* CD pin */
	MX6_PAD_NANDF_WP_B__GPIO6_IO09	| MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/* SD PWR Reset */
	MX6_PAD_SD3_RST__GPIO7_IO08     | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/* USDHC3: v1:N/A, v2:uSD Card */
iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC3_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC3_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC3_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC3_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC3_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC3_PAD_CTRL),

	/* CD pin */
	MX6_PAD_NANDF_WP_B__GPIO6_IO09	| MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/* SD PWR Reset */
	MX6_PAD_SD3_RST__GPIO7_IO08     | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/* USDHC4 / eMMC */
iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const pcie_pads[] = {
	MX6_PAD_GPIO_1__GPIO1_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* RESET */
};

static void setup_pcie(void)
{
	if (!ea_is_carrier_v2(2)) {
		imx_iomux_v3_setup_multiple_pads(pcie_pads, ARRAY_SIZE(pcie_pads));
	}
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
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

#ifdef CONFIG_SYS_I2C_MXC

/* set all switches APS in normal and PFM mode in standby */
static int setup_pmic_mode(int chip)
{
	unsigned char offset, i, switch_num, value;

	if (!chip) {
		/* pfuze100 */
		switch_num = 6;
		offset = 0x31;
	} else {
		/* pfuze200 */
		switch_num = 4;
		offset = 0x38;
	}

	value = 0xc;
	if (i2c_write(0x8, 0x23, 1, &value, 1)) {
		printf("Set SW1AB mode error!\n");
		return -1;
	}

	for (i = 0; i < switch_num - 1; i++) {
		if (i2c_write(0x8, offset + i * 7, 1, &value, 1)) {
			printf("Set switch%x mode error!\n", offset);
			return -1;
		}
	}

	return 0;
}

static int setup_pmic_voltages(void)
{
	unsigned char value, rev_id = 0 ;

	i2c_set_bus_num(0);
	if (!i2c_probe(0x8)) {

		if (i2c_read(0x8, 0, 1, &value, 1)) {
			printf("Read device ID error!\n");
			return -1;
		}
		if (i2c_read(0x8, 3, 1, &rev_id, 1)) {
			printf("Read Rev ID error!\n");
			return -1;
		}
		printf("Found PFUZE%s deviceid=%x,revid=%x\n",
			((value & 0xf) == 0) ? "100" : "200", value, rev_id);

		if (setup_pmic_mode(value & 0xf)) {
			printf("setup pmic mode error!\n");
			return -1;
		}

		/*increase VGEN5 from 2.8 to 3V*/
		if (i2c_read(0x8, 0x70, 1, &value, 1)) {
			printf("Read VGEN5 error!\n");
			return -1;
		}
		value &= ~0xf;
		value |= 0xc;
		if (i2c_write(0x8, 0x70, 1, &value, 1)) {
			printf("Set VGEN5 error!\n");
			return -1;
		}
		/* set SW1AB staby volatage 0.975V*/
		if (i2c_read(0x8, 0x21, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0x3f;
		value |= 0x1b;
		if (i2c_write(0x8, 0x21, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}
		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		if (i2c_read(0x8, 0x24, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0xc0;
		value |= 0x40;
		if (i2c_write(0x8, 0x24, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}

		/* set SW1C staby volatage 0.975V*/
		if (i2c_read(0x8, 0x2f, 1, &value, 1)) {
			printf("Read SW1CSTBY error!\n");
			return -1;
		}
		value &= ~0x3f;
		value |= 0x1b;
		if (i2c_write(0x8, 0x2f, 1, &value, 1)) {
			printf("Set SW1CSTBY error!\n");
			return -1;
		}

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		if (i2c_read(0x8, 0x32, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0xc0;
		value |= 0x40;
		if (i2c_write(0x8, 0x32, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}

		/*set SW3AB to 1.35V*/
		if (i2c_read(0x8, 0x3C, 1, &value, 1)) {
			printf("Read SW3AB error!\n");
			return -1;
		}
		if (value & 0x40)
			value = 0x4B;
		else
			value = 0x26;
		if (i2c_write(0x8, 0x3C, 1, &value, 1)) {
			printf("Set SW3AB error (w:0x3c)!\n");
			return -1;
		}
		if (i2c_write(0x8, 0x43, 1, &value, 1)) {
			printf("Set SW3AB error (w:0x43)!\n");
			return -1;
		}
		if (i2c_write(0x8, 0x3D, 1, &value, 1)) {
			printf("Set SW3AB error (w:0x3d)!\n");
			return -1;
		}
		if (i2c_write(0x8, 0x44, 1, &value, 1)) {
			printf("Set SW3AB error (w:0x44)!\n");
			return -1;
		}
	}


	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned char value;
	int is_400M;
	unsigned char vddarm;

	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0;	/* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		/* increase VDDARM to 1.425V */
		if (i2c_read(0x8, 0x20, 1, &value, 1)) {
			printf("Read SW1AB error!\n");
			return;
		}
		value &= ~0x3f;
		value |= 0x2d;
		if (i2c_write(0x8, 0x20, 1, &value, 1)) {
			printf("Set SW1AB error!\n");
			return;
		}
		/* increase VDDSOC to 1.425V */
		if (i2c_read(0x8, 0x2e, 1, &value, 1)) {
			printf("Read SW1C error!\n");
			return;
		}
		value &= ~0x3f;
		value |= 0x2d;
		if (i2c_write(0x8, 0x2e, 1, &value, 1)) {
			printf("Set SW1C error!\n");
			return;
		}
	}
	/* switch to ldo_bypass mode , boot on 800Mhz */
	if (ldo_bypass) {
		prep_anatop_bypass();

		/* decrease VDDARM for 400Mhz DQ:1.1V, DL:1.275V */
		if (i2c_read(0x8, 0x20, 1, &value, 1)) {
			printf("Read SW1AB error!\n");
			return;
		}
		value &= ~0x3f;
#if defined(CONFIG_MX6DL)
		value |= 0x27;
#else
		value |= 0x20;
#endif
		if (i2c_write(0x8, 0x20, 1, &value, 1)) {
			printf("Set SW1AB error!\n");
			return;
		}
		/* increase VDDSOC to 1.3V */
		if (i2c_read(0x8, 0x2e, 1, &value, 1)) {
			printf("Read SW1C error!\n");
			return;
		}
		value &= ~0x3f;
		value |= 0x28;
		if (i2c_write(0x8, 0x2e, 1, &value, 1)) {
			printf("Set SW1C error!\n");
			return;
		}

		/*
		 * MX6Q:
		 * VDDARM:1.15V@800M; VDDSOC:1.175V@800M
		 * VDDARM:0.975V@400M; VDDSOC:1.175V@400M
		 * MX6DL:
		 * VDDARM:1.175V@800M; VDDSOC:1.175V@800M
		 * VDDARM:1.075V@400M; VDDSOC:1.175V@400M
		 */
		is_400M = set_anatop_bypass(2);
		if (is_400M)
#if defined(CONFIG_MX6DL)
			vddarm = 0x1f;
#else
			vddarm = 0x1b;
#endif
		else
#if defined(CONFIG_MX6DL)
			vddarm = 0x23;
#else
			vddarm = 0x22;
#endif
		if (i2c_read(0x8, 0x20, 1, &value, 1)) {
			printf("Read SW1AB error!\n");
			return;
		}
		value &= ~0x3f;
		value |= vddarm;
		if (i2c_write(0x8, 0x20, 1, &value, 1)) {
			printf("Set SW1AB error!\n");
			return;
		}

		/* decrease VDDSOC to 1.175V */
		if (i2c_read(0x8, 0x2e, 1, &value, 1)) {
			printf("Read SW1C error!\n");
			return;
		}
		value &= ~0x3f;
		value |= 0x23;
		if (i2c_write(0x8, 0x2e, 1, &value, 1)) {
			printf("Set SW1C error!\n");
			return;
		}

		finish_anatop_bypass();
		printf("switch to ldo_bypass mode!\n");
	}
}
#endif
#endif

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC3_BASE_ADDR}, /* changed in board_mmc_init() if rev v1 */
	{USDHC4_BASE_ADDR},
};

int mmc_map_to_kernel_blk(int dev_no)
{
	if (ea_is_carrier_v2(2)) {
		return dev_no + 2;
	} else {
		if (dev_no == 1) {
			return 3;
		} else {
			return 1;
		}
	}
}

#define USDHC_CD_GPIO	IMX_GPIO_NR(6, 9)
#define USDHC_PWR_GPIO	IMX_GPIO_NR(7, 8)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		if (ea_is_carrier_v2(2)) {
			ret = 1; /* uSDHC2 always assumed to be present. M.2 connector */
		} else {
			ret = !gpio_get_value(USDHC_CD_GPIO);
		}
		break;
	case USDHC3_BASE_ADDR:
		if (ea_is_carrier_v2(2)) {
			ret = !gpio_get_value(USDHC_CD_GPIO);
		}
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    v1: USHC2 v2:USDHC3 (uSD Card)
	 * mmc1                    USDHC4 (eMMC)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {

#if defined(CONFIG_SPL_BUILD)

		// The SPL framework expects there to be only one MMC device
		// and we always loads u-boot from eMMC which is mapped to mmc1
		if (i != 1) continue;
#endif

		switch (i) {
		case 0:
			if (ea_is_carrier_v2(2)) {
				imx_iomux_v3_setup_multiple_pads(
					usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
				usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			} else {
				imx_iomux_v3_setup_multiple_pads(
					usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
				usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
				usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			}
			gpio_direction_input(USDHC_CD_GPIO);
			gpio_direction_output(USDHC_PWR_GPIO, 1);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

	return status;
}

int board_mmc_get_env_dev(int devno)
{
	int no = devno;

	if (ea_is_carrier_v2(2)) {
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

int mx6_rgmii_rework(struct phy_device *phydev)
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
	mx6_rgmii_rework(phydev);

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
		gpio_direction_output(DISPENABLE_GP_LVDS0_RGB, 1);
		gpio_direction_output(BRIGHTNESS_GP_LVDSx, 0);
		gpio_direction_output(BRIGHTNESS_GP_RGB, 1);
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
		gpio_direction_output(DISPENABLE_GP_LVDS0_RGB, 1);
		gpio_direction_output(BRIGHTNESS_GP_LVDSx, 1);
		gpio_direction_output(BRIGHTNESS_GP_RGB, 0);
		gpio_direction_output(BACKLIGHT_GP_LVDS0_RGB, 1);
	}
}

void board_enable_lvds1(const struct display_info_t *di, int enable)
{
	if (enable) {
		/* Default interface so only pinning is needed */
		gpio_direction_output(DISPENABLE_GP_LVDS1, 1);
		gpio_direction_output(BRIGHTNESS_GP_LVDSx, 1);
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
	gpio_direction_output(DISPENABLE_GP_LVDS0_RGB, 1);
	gpio_direction_output(BRIGHTNESS_GP_LVDSx, 1);
	gpio_direction_output(BRIGHTNESS_GP_RGB, 0);
	gpio_direction_output(BACKLIGHT_GP_LVDS0_RGB, 1);
}

static void do_enable_lvds1(struct display_info_t const *dev)
{
	/* Default interface so only pinning is needed */
	gpio_direction_output(DISPENABLE_GP_LVDS1, 1);
	gpio_direction_output(BRIGHTNESS_GP_LVDSx, 1);
	gpio_direction_output(BACKLIGHT_GP_LVDS1, 1);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	/* Default is lvds1 so disable it */
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

static void do_enable_parallel_rgb(struct display_info_t const *dev)
{
	/* Setup pads for RGB */
	imx_iomux_v3_setup_multiple_pads(rgb_pads, ARRAY_SIZE(rgb_pads));

	/* Default interface is LVDS1 so pinning must be changed */
	gpio_direction_output(DISPENABLE_GP_LVDS0_RGB, 1);
	gpio_direction_output(BRIGHTNESS_GP_LVDSx, 0);
	gpio_direction_output(BRIGHTNESS_GP_RGB, 1);
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
	//	printf("Failed to write to reg 0x32-0x39 on TFP410\n");
	//	return;
	//}
	printf("TFP410: DE generator disabled\n");
	printf("Successfully initialized TFP410!\n");
}
#endif


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
	setup_pcie();

	if (ea_load_ethaddr()) {
		printf("Failed to load MAC addresses\n");
	}

	return cpu_eth_init(bis);
}

int board_early_init_f(void)
{
	/* configure and enable pwr */
	imx_iomux_v3_setup_multiple_pads(peri_pwr_pads,
			ARRAY_SIZE(peri_pwr_pads));
	gpio_direction_output(IMX_GPIO_NR(7, 12), 1);

	/* enable USB 5V. Doesn't seem to work to do this in Linux/DTS?? */
	gpio_direction_output(IMX_GPIO_NR(1, 0), 1);

	/*
	 * Must initialize timer early since delay functions are used.
	 * Without timer_init a delay function will hang.
	 */
	timer_init();

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

// Configuration parameters are stored in I2C mapped eeprom and
// must be initialized here since the configuration is accessed
// early in the boot sequence
#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
#endif

	ea_eeprom_init();

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
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
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
	int ret = 0;

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_SYS_I2C_MXC
	ret = setup_pmic_voltages();
	if (ret)
		return -1;
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

#ifdef CONFIG_CMD_EADISP
	eatouch_init();
#endif

#ifdef CONFIG_SYS_I2C_MXC
	ea_gpio_exp_configure(2);
#endif
	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(1, 5)
iomux_v3_cfg_t const recovery_key_pads[] = {
	(MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
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
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!env_get("bootcmd_android_recovery"))
			env_set("bootcmd_android_recovery",
				"boota sata recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
		if (!env_get("bootcmd_android_recovery"))
			env_set("bootcmd_android_recovery",
				"boota mmc0 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!env_get("bootcmd_android_recovery"))
			env_set("bootcmd_android_recovery",
				"boota mmc1 recovery");
		break;
	case MMC4_BOOT:
		if (!env_get("bootcmd_android_recovery"))
			env_set("bootcmd_android_recovery",
				"boota mmc2 recovery");
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

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET 0x800
#define UCTRL_PWR_POL (1<<9)

iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_EIM_D22__USB_OTG_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const usb_hc1_pads[] = {
	MX6_PAD_GPIO_0__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int board_ehci_hcd_init(int port)
{
	u32 * usbnc_usb_ctrl;

	switch (port) {
	case 0:
		imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
			ARRAY_SIZE(usb_otg_pads));

		/*set daisy chain for otg_pin_id on 6q. for 6dl, this bit is reserved*/
		imx_iomux_set_gpr_register(1, 13, 1, 0);
		break;
	case 1:
		imx_iomux_v3_setup_multiple_pads(usb_hc1_pads,
			ARRAY_SIZE(usb_hc1_pads));
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
		port * 4);

	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		if (on)
			gpio_direction_output(IMX_GPIO_NR(1, 0), 1);
		else
			gpio_direction_output(IMX_GPIO_NR(1, 0), 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}
	return 0;
}
#endif
