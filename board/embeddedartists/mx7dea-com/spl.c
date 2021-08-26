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
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/mx7d_pins.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch-mx7/mx7-ddr.h>


#include "../common/ea_common.h"
#include "../common/ea_eeprom.h"
#include "../common/ea_gpio_expander.h"

#define I2C_PAD_CTRL    (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU100KOHM)

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* i2c1 for eeprom */
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

/* i2c2 for gpio expander */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C2_SCL__I2C2_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C2_SCL__GPIO4_IO10 | PC,
		.gp = IMX_GPIO_NR(4, 10),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C2_SDA__I2C2_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C2_SDA__GPIO4_IO11 | PC,
		.gp = IMX_GPIO_NR(4, 11),
	},
};

#define USDHC_PAD_CTRL (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

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



/*
 * DDR initialization
 *
 * Default values for DDR initialization. Actual values will
 * be read from eeprom (if available).
 */

#define DDRC_ADDRMAP6 (DDRC_IPS_BASE_ADDR+0x0218)
#define DDRPHY_ZQ_CON0 (DDRPHY_IPS_BASE_ADDR+0x00c0)

#ifdef CONFIG_TARGET_MX7DEA_COM

static struct ddrc ea_spl_ddrc = {
        .mstr           = 0x01040001,
	.rfshtmg	= 0x0040005e,
	.init0		= 0x00020001,
	.init1		= 0x00010000,
	.init3		= 0x09300004,
	.init4		= 0x04080000,
	.init5		= 0x00090004,
	.rankctl	= 0x0000033f,
	.dramtmg0	= 0x0908120a,
	.dramtmg1	= 0x0002020e,
	.dramtmg2	= 0x03040407,
	.dramtmg3	= 0x00002006,
	.dramtmg4	= 0x04020204,
	.dramtmg5	= 0x03030202,
	.dramtmg8	= 0x03030803,
	.zqctl0		= 0x00800020,
	.dfitmg0	= 0x02098204,
	.dfitmg1	= 0x00030303,
	.dfiupd0	= 0x80400003,
	.dfiupd1	= 0x00100020,
	.dfiupd2	= 0x80100004,
	.addrmap0	= 0x00000016,
	.addrmap1	= 0x00171717,
	.addrmap2	= 0x0,
	.addrmap3	= 0x0,
	.addrmap4	= 0x00000F0F,
	.addrmap5	= 0x04040404,
	.addrmap6	= 0x0F040404,
	.odtcfg		= 0x06000601,
	.odtmap		= 0x00001323,
};

static struct ddrc_mp ea_spl_ddrc_mp = {
	.pctrl_0	= 0x00000001
};

static struct ddr_phy ea_spl_ddr_phy = {
	.phy_con0	= 0x17420f40,
	.phy_con1	= 0x10210100,
	.phy_con4	= 0x00060807,
	.offset_lp_con0	= 0x0000000f,
	.offset_rd_con0	= 0x08080808,
	.offset_wr_con0	= 0x08080808,
	.cmd_sdll_con0	= 0x00000010,
	.drvds_con0	= 0x00000d6e,
	.mdll_con0	= 0x1010007e,
	.zq_con0	= 0x0e407304,
};

static struct mx7_calibration ea_spl_calib = {
	.num_val 	= 5,
	.values 	= {
		0x0e407304,
		0x0e447304,
		0x0e447306,
		0x0e447304,
		0x0e407304,
	},
};

#else /* CONFIG_TARGET_MX7DEA_UCOM */

static struct ddrc ea_spl_ddrc = {
        .mstr           = 0x01040008,
        .rfshtmg        = 0x00200038,
        .init0          = 0x00350001,
        .init1          = 0x0,
        .init3          = 0x00C3000A,
        .init4          = 0x00010000,
        .init5          = 0x00110006,
        .rankctl        = 0x0000033f,
        .dramtmg0       = 0x0A0E110B,
        .dramtmg1       = 0x00020211,
        .dramtmg2       = 0x03060708,
        .dramtmg3       = 0x00A0500C,
        .dramtmg4       = 0x05020307,
        .dramtmg5       = 0x02020404,
        .dramtmg8       = 0x00000202,
        .zqctl0         = 0x00600018,
        .dfitmg0        = 0x02098205,
        .dfitmg1        = 0x00060303,
        .dfiupd0        = 0x80400003,
        .dfiupd1        = 0x00100020,
        .dfiupd2        = 0x80100004,
        .addrmap0       = 0x00000016,
        .addrmap1       = 0x00171717,
        .addrmap2       = 0x0,
        .addrmap3       = 0x0,
        .addrmap4       = 0x00000F0F,
        .addrmap5       = 0x04040404,
        .addrmap6       = 0x0F040404,
        .odtcfg         = 0x06000601,
        .odtmap         = 0x00000000,
};

static struct ddrc_mp ea_spl_ddrc_mp = {
        .pctrl_0        = 0x00000001
};

static struct ddr_phy ea_spl_ddr_phy = {
        .phy_con0       = 0x17421e40,
        .phy_con1       = 0x10210100,
        .phy_con4       = 0x0007080C,
        .offset_lp_con0 = 0x0000000f,
        .offset_rd_con0 = 0x0A0A0A0A,
        .offset_wr_con0 = 0x06060606,
        .cmd_sdll_con0  = 0x00000008,
        .drvds_con0     = 0x00000b24,
        .mdll_con0      = 0x1010007e,
        .zq_con0        = 0x0C487304,
};

static struct mx7_calibration ea_spl_calib = {
        .num_val        = 4,
        .values         = {
                0x0C487304,
                0x0C4C7304,
                0x0C4C7306,
                0x0C4C7304,
        },
};


#endif

static void set_ddrc_val(ea_ddr_cfg_t* cfg, u32 reg, u32 val)
{
	u32* v = (u32*)&ea_spl_ddrc;
	int idx = (reg - DDRC_IPS_BASE_ADDR) / sizeof(u32);

	v[idx] = val;

	/*
	 * Some boards with 1G SDRAM had the wrong configuration value
	 * stored in eeprom for addrmap6. Correcting below.
	 */
	if (reg == DDRC_ADDRMAP6 && val != 0x0F040404 &&
		(cfg->ddr_size_mb << 20) == SZ_1G) {
		v[idx] = 0x0F040404;
	}
};

static void set_ddrc_mp_val(u32 addr, u32 val)
{
	u32* v = (u32*)&ea_spl_ddrc_mp;
	int idx = (addr - DDRC_MP_BASE_ADDR) / sizeof(u32);

	v[idx] = val;
}

static void set_ddr_phy_val(u32 addr, u32 val)
{
	u32* v = (u32*)&ea_spl_ddr_phy;
	int idx = (addr - DDRPHY_IPS_BASE_ADDR) / sizeof(u32);

	v[idx] = val;

	/*
	 * These values are used for calibration
	 */
	if (addr == DDRPHY_ZQ_CON0 && ea_spl_calib.num_val < MX7_CAL_VAL_MAX) {
		ea_spl_calib.values[ea_spl_calib.num_val] = val;
		ea_spl_calib.num_val++;
	}
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

				if (c->reg >= DDRC_IPS_BASE_ADDR &&
					c->reg < (DDRC_IPS_BASE_ADDR+sizeof(struct ddrc))) {
					set_ddrc_val(cfg, c->reg, c->val);
				}
				else if (c->reg >= DDRC_MP_BASE_ADDR &&
					c->reg < (DDRC_MP_BASE_ADDR+sizeof(struct ddrc_mp))) {
					set_ddrc_mp_val(c->reg, c->val);
				}
				else if (c->reg >= DDRPHY_IPS_BASE_ADDR &&
					c->reg < (DDRC_MP_BASE_ADDR+sizeof(struct ddrc_mp))) {
					set_ddr_phy_val(c->reg, c->val);
				}

			}

		}


	}

	return ret;
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

		/* reset num calibration values -> will be read from eeprom */
		ea_spl_calib.num_val = 0;

		ret = load_ddr_cfg(&cfg);
		if (ret) {
			printf("Failed to load DDR CFG from eeprom (%d)\n", ret);
			return;
		}
        }

	mx7_dram_cfg(&ea_spl_ddrc,
                     &ea_spl_ddrc_mp,
                     &ea_spl_ddr_phy,
                     &ea_spl_calib);
}

void spl_board_init(void)
{
}

#define USDHC3_PWR_GPIO IMX_GPIO_NR(6, 11)

static struct fsl_esdhc_cfg usdhc_cfg = {
	USDHC3_BASE_ADDR, 0, 4
};

int board_mmc_getcd(struct mmc *mmc)
{
	/* eMMC always available */
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	int ret;

	/*
	 * SPL only needs access to eMMC (esdhc3)
	 */

	imx_iomux_v3_setup_multiple_pads(
		usdhc3_emmc_pads, ARRAY_SIZE(usdhc3_emmc_pads));
	gpio_request(USDHC3_PWR_GPIO, "usdhc3_pwr");
	gpio_direction_output(USDHC3_PWR_GPIO, 0);
	udelay(500);
	gpio_direction_output(USDHC3_PWR_GPIO, 1);
	usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);

	ret = fsl_esdhc_initialize(bis, &usdhc_cfg);
	if (ret)
		return ret;

	return 0;
}

void board_init_f(ulong dummy)
{
	uint32_t size;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	arch_cpu_init();

	/* needed to be able to useelay function */
	timer_init();

	/* call to setup uart pad muxing */
	board_early_init_f();

	preloader_console_init();

	/* setup I2C1 for eeprom and I2C2 for gpio expander */
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
