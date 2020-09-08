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
		.i2c_mode =  MX6_PAD_UART4_TX_DATA__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_UART4_TX_DATA__GPIO1_IO28 | PC,
		.gp = IMX_GPIO_NR(1, 28),
	},
	.sda = {
		.i2c_mode = MX6_PAD_UART4_RX_DATA__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_UART4_RX_DATA__GPIO1_IO29 | PC,
		.gp = IMX_GPIO_NR(1, 29),
	},
};

/* i2c2 for gpio expander */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode =  MX6_PAD_UART5_TX_DATA__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_UART5_TX_DATA__GPIO1_IO30 | PC,
		.gp = IMX_GPIO_NR(1, 30),
	},
	.sda = {
		.i2c_mode = MX6_PAD_UART5_RX_DATA__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_UART5_RX_DATA__GPIO1_IO31 | PC,
		.gp = IMX_GPIO_NR(1, 31),
	},
};

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

/* USDHC2 / eMMC */
static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA04__USDHC2_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA05__USDHC2_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA06__USDHC2_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA07__USDHC2_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/*
	 * RST_B
	 */
	MX6_PAD_NAND_ALE__GPIO4_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/*
 * DDR initialization
 *
 * Default values for DDR initialization. Actual values will
 * be read from eeprom (if available).
 */

static ea_ddr_cfg_pair_t ddr_init_mx6ul[] = {
	{0x020c4068, 0xffffffff},
	{0x020c406c, 0xffffffff},
	{0x020c4070, 0xffffffff},
	{0x020c4074, 0xffffffff},
	{0x020c4078, 0xffffffff},
	{0x020c407c, 0xffffffff},
	{0x020c4080, 0xffffffff},
	{0x020c4084, 0xffffffff},
	{0x020E04B4, 0x000C0000},
	{0x020E04AC, 0x00000000},
	{0x020E027C, 0x00000008},
	{0x020E0250, 0x00000030},
	{0x020E024C, 0x00000030},
	{0x020E0490, 0x00000030},
	{0x020E0288, 0x00000030},
	{0x020E0270, 0x00000000},
	{0x020E0260, 0x00000030},
	{0x020E0264, 0x00000030},
	{0x020E04A0, 0x00000030},
	{0x020E0494, 0x00020000},
	{0x020E0280, 0x00000038},
	{0x020E0284, 0x00000030},
	{0x020E04B0, 0x00020000},
	{0x020E0498, 0x00000030},
	{0x020E04A4, 0x00000030},
	{0x020E0244, 0x00000030},
	{0x020E0248, 0x00000030},
	{0x021B001C, 0x00008000},
	{0x021B0800, 0xA1390003},
	{0x021B080C, 0x00090000},
	{0x021B083C, 0x41540154},
	{0x021B0848, 0x40404442},
	{0x021B0850, 0x40405450},
	{0x021B081C, 0x33333333},
	{0x021B0820, 0x33333333},
	{0x021B082C, 0xf3333333},
	{0x021B0830, 0xf3333333},
	{0x021B08C0, 0x00922012},
	{0x021B0858, 0x00000F00},
	{0x021B08b8, 0x00000800},
	{0x021B0004, 0x0002002D},
	{0x021B0008, 0x1B333000},

	{0x021B000C, 0x676B54F3},
	{0x021B0010, 0xB68E0A83},
	{0x021B0014, 0x01FF00DB},
	{0x021B0018, 0x00211740},
	{0x021B001C, 0x00008000},
	{0x021B002C, 0x000026D2},
	{0x021B0030, 0x006B1023},
	{0x021B0040, 0x0000004F},
	{0x021B0000, 0x84180000},
// ------> NOK
/*	{0x021B001C, 0x02008032},
	{0x021B001C, 0x00008033},
	{0x021B001C, 0x00048031},
	{0x021B001C, 0x15208030},
	{0x021B001C, 0x04008040},*/
// <----- NOK
	{0x021B0020, 0x00000800},
	{0x021B0818, 0x00000227},
	{0x021B0004, 0x0002552D},
	{0x021B0404, 0x00011006},
	{0x021B001C, 0x00000000},
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
		dram_table_init(ddr_init_mx6ul, ARRAY_SIZE(ddr_init_mx6ul));
	}

}

void spl_board_init(void)
{
}

static struct fsl_esdhc_cfg usdhc_cfg = {
	USDHC2_BASE_ADDR, 0, 8
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
		usdhc2_pads, ARRAY_SIZE(usdhc2_pads));

	/* reset */
	gpio_direction_output(IMX_GPIO_NR(4, 10), 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(4, 10), 1);

	usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);


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

	/* call to setup uart pad muxing and peri pwr */
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
