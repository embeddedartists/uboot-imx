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
#include <fsl_esdhc.h>
#include <mmc.h>
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch-mx6/mx6-ddr.h>

#include "../common/ea_common.h"
#include "../common/ea_eeprom.h"
#include "../common/ea_gpio_expander.h"

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* i2c1 for eeprom */
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

/* i2c3 for gpio expander */
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


#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

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


/*
 * DDR initialization
 *
 * Default values for DDR initialization. Actual values will
 * be read from eeprom (if available).
 */

static ea_ddr_cfg_pair_t ddr_init_mx6q[] = {
	{0x020e0798, 0x000C0000},
	{0x020e0758, 0x00000000},
	{0x020e0588, 0x00000030},
	{0x020e0594, 0x00000030},
	{0x020e056c, 0x00000030},
	{0x020e0578, 0x00000030},
	{0x020e074c, 0x00000030},
	{0x020e057c, 0x00000030},
	{0x020e058c, 0x00000000},
	{0x020e059c, 0x00000030},
	{0x020e05a0, 0x00000030},
	{0x020e078c, 0x00000030},
	{0x020e0750, 0x00020000},
	{0x020e05a8, 0x00000028},
	{0x020e05b0, 0x00000028},
	{0x020e0524, 0x00000028},
	{0x020e051c, 0x00000028},
	{0x020e0518, 0x00000028},
	{0x020e050c, 0x00000028},
	{0x020e05b8, 0x00000028},
	{0x020e05c0, 0x00000028},
	{0x020e0774, 0x00020000},
	{0x020e0784, 0x00000028},
	{0x020e0788, 0x00000028},
	{0x020e0794, 0x00000028},
	{0x020e079c, 0x00000028},
	{0x020e07a0, 0x00000028},
	{0x020e07a4, 0x00000028},
	{0x020e07a8, 0x00000028},
	{0x020e0748, 0x00000028},
	{0x020e05ac, 0x00000028},
	{0x020e05b4, 0x00000028},
	{0x020e0528, 0x00000028},
	{0x020e0520, 0x00000028},
	{0x020e0514, 0x00000028},
	{0x020e0510, 0x00000028},
	{0x020e05bc, 0x00000028},
	{0x020e05c4, 0x00000028},
	{0x021b0800, 0xa1390003},
	{0x021b080c, 0x001F0018},
	{0x021b0810, 0x0023001F},
	{0x021b480c, 0x000F001F},
	{0x021b4810, 0x0006001D},
	{0x021b083c, 0x43340344},
	{0x021b0840, 0x03300328},
	{0x021b483c, 0x432C0340},
	{0x021b4840, 0x03300274},
	{0x021b0848, 0x3A303232},
	{0x021b4848, 0x36362E3E},
	{0x021b0850, 0x32343E3C},
	{0x021b4850, 0x4032463E},
	{0x021b081c, 0x33333333},
	{0x021b0820, 0x33333333},
	{0x021b0824, 0x33333333},
	{0x021b0828, 0x33333333},
	{0x021b481c, 0x33333333},
	{0x021b4820, 0x33333333},
	{0x021b4824, 0x33333333},
	{0x021b4828, 0x33333333},
	{0x021b08b8, 0x00000800},
	{0x021b48b8, 0x00000800},
	{0x021b0004, 0x00020036},
	{0x021b0008, 0x09444040},
	{0x021b000c, 0x8A8F7955},
	{0x021b0010, 0xFF328F64},
	{0x021b0014, 0x01FF00DB},
	{0x021b0018, 0x00001740},
	{0x021b001c, 0x00008000},
	{0x021b002c, 0x000026d2},
	{0x021b0030, 0x008F1023},
	{0x021b0040, 0x00000047},
	{0x021b0000, 0x841A0000},
// ---------------------------------->
/*	{0x021b001c, 0x04088032},
	{0x021b001c, 0x00008033},
	{0x021b001c, 0x00048031},
	{0x021b001c, 0x09408030},
	{0x021b001c, 0x04008040},*/
// <----------------------------------
	{0x021b0020, 0x00005800},
	{0x021b0818, 0x00011117},
	{0x021b4818, 0x00011117},
	{0x021b0004, 0x00025576},
	{0x021b0404, 0x00011006},
	{0x021b001c, 0x00000000},
	{0x020c4068, 0x00C03F3F},
	{0x020c406c, 0x0030FC03},
	{0x020c4070, 0x0FFFC000},
	{0x020c4074, 0x3FF00000},
	{0x020c4078, 0x00FFF300},
	{0x020c407c, 0x0F0000F3},
	{0x020c4080, 0x000003FF},
	{0x020e0010, 0xF00000CF},
	{0x020e0018, 0x007F007F},
	{0x020e001c, 0x007F007F},
	{0x020c4060, 0x000000fb},
};

static ea_ddr_cfg_pair_t ddr_init_mx6dl[] = {
        {0x020e0774, 0x000C0000},
        {0x020e0754, 0x00000000},
        {0x020e04ac, 0x00000030},
        {0x020e04b0, 0x00000030},
        {0x020e0464, 0x00000030},
        {0x020e0490, 0x00000030},
        {0x020e074c, 0x00000030},
        {0x020e0494, 0x00000030},
        {0x020e04a0, 0x00000000},
        {0x020e04b4, 0x00000030},
        {0x020e04b8, 0x00000030},
        {0x020e076c, 0x00000030},
        {0x020e0750, 0x00020000},
        {0x020e04bc, 0x00000030},
        {0x020e04c0, 0x00000030},
        {0x020e04c4, 0x00000030},
        {0x020e04c8, 0x00000030},
        {0x020e04cc, 0x00000030},
        {0x020e04d0, 0x00000030},
        {0x020e04d4, 0x00000030},
        {0x020e04d8, 0x00000030},
        {0x020e0760, 0x00020000},
        {0x020e0764, 0x00000030},
        {0x020e0770, 0x00000030},
        {0x020e0778, 0x00000030},
        {0x020e077c, 0x00000030},
        {0x020e0780, 0x00000030},
        {0x020e0784, 0x00000030},
        {0x020e078c, 0x00000030},
        {0x020e0748, 0x00000030},
        {0x020e0470, 0x00000030},
        {0x020e0474, 0x00000030},
        {0x020e0478, 0x00000030},
        {0x020e047c, 0x00000030},
        {0x020e0480, 0x00000030},
        {0x020e0484, 0x00000030},
        {0x020e0488, 0x00000030},
        {0x020e048c, 0x00000030},
        {0x021b0800, 0xa1390003},
        {0x021b080c, 0x004E004E},
        {0x021b0810, 0x0042004A},
        {0x021b480c, 0x002D0030},
        {0x021b4810, 0x002C0045},
        {0x021b083c, 0x42460244},
        {0x021b0840, 0x02300230},
        {0x021b483c, 0x422C0238},
        {0x021b4840, 0x021E0222},
        {0x021b0848, 0x44464848},
        {0x021b4848, 0x42484842},
        {0x021b0850, 0x342E2A32},
        {0x021b4850, 0x362E322A},
        {0x021b081c, 0x33333333},
        {0x021b0820, 0x33333333},
        {0x021b0824, 0x33333333},
        {0x021b0828, 0x33333333},
        {0x021b481c, 0x33333333},
        {0x021b4820, 0x33333333},
        {0x021b4824, 0x33333333},
        {0x021b4828, 0x33333333},
        {0x021b08b8, 0x00000800},
        {0x021b48b8, 0x00000800},
        {0x021b0004, 0x0002002D},
        {0x021b0008, 0x00333030},
        {0x021b000c, 0x3F435313},
        {0x021b0010, 0xB66E8B63},
        {0x021b0014, 0x01FF00DB},
        {0x021b0018, 0x00011740},
        {0x021b001c, 0x00008000},
        {0x021b002c, 0x000026d2},
        {0x021b0030, 0x00431023},
        {0x021b0040, 0x00000027},
        {0x021b0000, 0x831A0000},
//------------------->
        /*{0x021b001c, 0x04008032},
        {0x021b001c, 0x00008033},
        {0x021b001c, 0x00048031},
        {0x021b001c, 0x05208030},*/
// <-----------------
        {0x021b0020, 0x00005800},
        {0x021b0818, 0x00011117},
        {0x021b4818, 0x00011117},
        {0x021b0004, 0x0002556D},
        {0x021b0404, 0x00011006},
        {0x021b001c, 0x00000000},
        {0x020c4068, 0x00C03F3F},
        {0x020c406c, 0x0030FC03},
        {0x020c4070, 0x0FFFC000},
        {0x020c4074, 0x3FF00000},
        {0x020c4078, 0x00FFF300},
        {0x020c407c, 0x0F0000C3},
        {0x020c4080, 0x000003FF},
        {0x020e0010, 0xF00000CF},
        {0x020e0018, 0x007F007F},
        {0x020e001c, 0x007F007F},

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
		if (is_mx6dq())
			dram_table_init(ddr_init_mx6q, ARRAY_SIZE(ddr_init_mx6q));
		else if (is_mx6dl())
			dram_table_init(ddr_init_mx6dl, ARRAY_SIZE(ddr_init_mx6dl));
	}

}

void spl_board_init(void)
{
}

static struct fsl_esdhc_cfg usdhc_cfg = {
	USDHC4_BASE_ADDR
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
		usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
	usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

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

	/* call to setup uart pad muxing and enable peri pwr */
	board_init_common();

	preloader_console_init();

	/* setup I2C1 for eeprom and I2C3 for gpio expander */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

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
