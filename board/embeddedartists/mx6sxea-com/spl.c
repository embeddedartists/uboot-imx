/*
 * Copyright 2019 Embedded Artists
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <common.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/gpio.h>
#include <asm/arch/crm_regs.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/sys_proto.h>
#include <asm/arch/mx6-pins.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch-mx6/mx6-ddr.h>

#include "../common/ea_common.h"
#include "../common/ea_eeprom.h"
#include "../common/ea_gpio_expander.h"

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* i2c1 for eeprom */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO1_IO00__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO00__GPIO1_IO_0 | PC,
		.gp = IMX_GPIO_NR(1, 0),
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO01__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO01__GPIO1_IO_1 | PC,
		.gp = IMX_GPIO_NR(1, 1),
	},
};

/* i2c2 for gpio expander */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO1_IO02__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO02__GPIO1_IO_2 | PC,
		.gp = IMX_GPIO_NR(1, 2),
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO03__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO03__GPIO1_IO_3 | PC,
		.gp = IMX_GPIO_NR(1, 3),
	},
};

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

/* USDHC3: eMMC */
static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__USDHC3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__USDHC3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA0__USDHC3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA1__USDHC3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA2__USDHC3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA3__USDHC3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA4__USDHC3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA5__USDHC3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA6__USDHC3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA7__USDHC3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

/*
 * DDR initialization
 *
 * Default values for DDR initialization. Actual values will
 * be read from eeprom (if available).
 */

static ea_ddr_cfg_pair_t ddr_init_mx6sx[] = {
	{0x020e0618, 0x000c0000},
	{0x020e05fc, 0x00000000},

	{0x020e032c, 0x00000030},

	{0x020e0300, 0x00000020},
	{0x020e02fc, 0x00000020},
	{0x020e05f4, 0x00000020},

	{0x020e0340, 0x00000020},

	{0x020e0320, 0x00000000},
	{0x020e0310, 0x00000020},
	{0x020e0314, 0x00000020},
	{0x020e0614, 0x00000020},

	{0x020e05f8, 0x00020000},
	{0x020e0330, 0x00000028},
	{0x020e0334, 0x00000028},
	{0x020e0338, 0x00000028},
	{0x020e033c, 0x00000028},

	{0x020e0608, 0x00020000},
	{0x020e060c, 0x00000028},
	{0x020e0610, 0x00000028},
	{0x020e061c, 0x00000028},
	{0x020e0620, 0x00000028},
	{0x020e02ec, 0x00000028},
	{0x020e02f0, 0x00000028},
	{0x020e02f4, 0x00000028},
	{0x020e02f8, 0x00000028},

	{0x021b0800, 0xa1390003},

	{0x021b080c, 0x002b0025},
	{0x021b0810, 0x00290024},

	{0x021b083c, 0x4153014B},
	{0x021b0840, 0x013E0132},

	{0x021b0848, 0x43434549},
	{0x021b0850, 0x36363A35},

	{0x021b081c, 0x33333333},
	{0x021b0820, 0x33333333},
	{0x021b0824, 0x33333333},
	{0x021b0828, 0x33333333},

	{0x021b08b8, 0x00000800},

	{0x021b0004, 0x0002002d},
	{0x021b0008, 0x00333030},
	{0x021b000c, 0x676b52f3},
	{0x021b0010, 0xb66d8b63},
	{0x021b0014, 0x01ff00db},
	{0x021b0018, 0x00011740},
	{0x021b001c, 0x00008000},
	{0x021b002c, 0x000026d2},
	{0x021b0030, 0x006b1023},
	{0x021b0040, 0x0000005f},
	{0x021b0000, 0x84190000},

	{0x021b001c, 0x04008032},
	{0x021b001c, 0x00008033},
	{0x021b001c, 0x00048031},
	{0x021b001c, 0x05208030},
	{0x021b001c, 0x04008040},

	{0x021b0020, 0x00000800},
	{0x021b0818, 0x00011117},
	{0x021b001c, 0x00000000},
};


static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

/*
 * Load DDR configuration from eeprom and map
 * the values to structs later used to initialize
 * the memory (mx7_dram_cfg)
 */
static int load_ddr_cfg(ea_ddr_cfg_t* cfg)
{
#define EA_DDR_CFG_BUF_SZ (8)

	int ret = 0;
	int i, j;
	int num_read = 0;
	ea_ddr_cfg_pair_t cfg_buf[EA_DDR_CFG_BUF_SZ];
	ea_ddr_cfg_pair_t* c;

	for (i = 0; i < cfg->num_pairs; i+= EA_DDR_CFG_BUF_SZ) {

		ret = ea_eeprom_ddr_cfg_read(cfg, &cfg_buf[0],
			EA_DDR_CFG_BUF_SZ, &num_read);

		if (ret) break;

		if (num_read > 0) {

			for(j = 0; j < num_read; j++) {
				c = &cfg_buf[j];
				writel(c->val, c->reg);
			}
		}
	}

	return ret;
}

static void dram_table_init(ea_ddr_cfg_pair_t *cfg, int size)
{
	int i;

	for(i = 0; i < size; i++)
	{
		writel(cfg[i].val, (unsigned long)cfg[i].reg);
	}
}

static void spl_dram_init(uint32_t *size)
{
	ea_ddr_cfg_t cfg;
	int ret;

	/* set default value, will be replaced if  eeprom cfg is valid */
	*size = (PHYS_SDRAM_SIZE >> 20);

	ret = ea_eeprom_ddr_cfg_init(&cfg);

	/* If eeprom is valid read ddr config; otherwise use default */
	if (!ret) {
		*size = cfg.ddr_size_mb;

		ret = load_ddr_cfg(&cfg);
		if (ret) {
			printf("Failed to load DDR CFG from eeprom (%d)\n", ret);
			return;
		}

	} else {
		dram_table_init(ddr_init_mx6sx, ARRAY_SIZE(ddr_init_mx6sx));
	}

}

void spl_board_init(void)
{
}

static struct fsl_esdhc_cfg usdhc_cfg = {
	USDHC3_BASE_ADDR
};

int board_mmc_getcd(struct mmc *mmc)
{
	/* eMMC always available */
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;

        /*
         * SPL only needs access to eMMC (esdhc3)
         */

	imx_iomux_v3_setup_multiple_pads(
		usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
	usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);

	status |= fsl_esdhc_initialize(bis, &usdhc_cfg);

	return status;
}

int board_init_common(void);

void board_init_f(ulong dummy)
{
	uint32_t size;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();

	/* needed to be able to useelay function */
	timer_init();

	/* call to setup uart pad muxing and peri pwr*/
	board_init_common();

	preloader_console_init();

	/* setup I2C1 for eeprom and I2C3 for gpio expander */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	/* DDR initialization */
	spl_dram_init(&size);

	/*
	 * Setting configuration data that will be shared with u-boot.
	 *
	 * SPL is able to use i2c to detect gpio expander or read data
	 * from eeprom. U-boot is not able to do this before relocation
	 * when the device model (CONFIG_DM) is enabled.
	 */
	ea_conf->magic = EA_CONFIG_MAGIC;
	ea_conf->is_carrier_v2 = ea_is_carrier_v2(1);
	ea_conf->ddr_size = size;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	board_init_r(NULL, 0);
}
