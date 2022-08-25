/*
 * Copyright 2018-2019 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <cpu_func.h>
#include <hang.h>
#include <spl.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx8mn_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/arch/ddr.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <gzip.h>
#include <init.h>
#include <timer.h>

#include "../common/ea_common.h"
#include "../common/ea_eeprom.h"
#include "../common/ea_gpio_expander.h"

DECLARE_GLOBAL_DATA_PTR;


#define I2C_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = IMX8MN_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = IMX8MN_PAD_I2C1_SCL__GPIO5_IO14 | PC,
		.gp = IMX_GPIO_NR(5, 14),
	},
	.sda = {
		.i2c_mode = IMX8MN_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = IMX8MN_PAD_I2C1_SDA__GPIO5_IO15 | PC,
		.gp = IMX_GPIO_NR(5, 15),
	},
};
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = IMX8MN_PAD_I2C2_SCL__I2C2_SCL | PC,
		.gpio_mode = IMX8MN_PAD_I2C2_SCL__GPIO5_IO16 | PC,
		.gp = IMX_GPIO_NR(5, 16),
	},
	.sda = {
		.i2c_mode = IMX8MN_PAD_I2C2_SDA__I2C2_SDA | PC,
		.gpio_mode = IMX8MN_PAD_I2C2_SDA__GPIO5_IO17 | PC,
		.gp = IMX_GPIO_NR(5, 17),
	},
};

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE |PAD_CTL_PE | \
			 PAD_CTL_FSEL2)
#define USDHC_GPIO_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_DSE1)
#define USDHC_CD_PAD_CTRL (PAD_CTL_PE |PAD_CTL_PUE |PAD_CTL_HYS | PAD_CTL_DSE4)


static iomux_v3_cfg_t const usdhc3_pads[] = {
	IMX8MN_PAD_NAND_WE_B__USDHC3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MN_PAD_NAND_WP_B__USDHC3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MN_PAD_NAND_DATA04__USDHC3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MN_PAD_NAND_DATA05__USDHC3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MN_PAD_NAND_DATA06__USDHC3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MN_PAD_NAND_DATA07__USDHC3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MN_PAD_NAND_RE_B__USDHC3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MN_PAD_NAND_CE2_B__USDHC3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MN_PAD_NAND_CE3_B__USDHC3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MN_PAD_NAND_CLE__USDHC3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

#include "ddr4_timing.c"

enum ea_ddr_field {
	EA_DDR_DDRC   = 1,
	EA_DDR_DDRPHY,
	EA_DDR_DDRPHY_TRAINED,
	EA_DDR_PHY_PIE,
	EA_DDR_FSP_INFO,
	EA_DDR_FSP0,
	EA_DDR_FSP1,
	EA_DDR_FSP2,
	EA_DDR_FSP3,
};

struct dram_fsp_msg ea_ddr_dram_fsp_msg[4] = {{1}};

#define EA_DBUF_SZ (16384)
#define EA_GZBUF_SZ (6144)
static unsigned char ea_dbuf[EA_DBUF_SZ] = {1};
static unsigned char ea_gzbuf[EA_GZBUF_SZ] = {1};

static void spl_ddr_map_array(enum ea_ddr_field idx, struct dram_cfg_param* a, int sz)
{
	switch(idx) {
	case EA_DDR_DDRC:
		dram_timing.ddrc_cfg = a;
		dram_timing.ddrc_cfg_num = sz;

		break;
	case EA_DDR_DDRPHY:
		dram_timing.ddrphy_cfg = a;
		dram_timing.ddrphy_cfg_num = sz;

		break;
	case EA_DDR_DDRPHY_TRAINED:
		dram_timing.ddrphy_trained_csr = a;
		dram_timing.ddrphy_trained_csr_num = sz;

		break;
	case EA_DDR_PHY_PIE:
		dram_timing.ddrphy_pie = a;
		dram_timing.ddrphy_pie_num = sz;

		break;
	case EA_DDR_FSP_INFO:
		/*
		 * [0].reg = size of the fsp table
		 * [1].reg = fsp_table[0]
		 * [1].val = fsp_table[1]
		 * [2].reg = fsp_table[2]
		 * [2].val = fsp_table[3]
		 */
		dram_timing.fsp_msg_num = a[0].reg;
		dram_timing.fsp_msg = ea_ddr_dram_fsp_msg;
		dram_timing.fsp_table[0] = a[1].reg;
		dram_timing.fsp_table[1] = a[1].val;
		dram_timing.fsp_table[2] = a[2].reg;
		dram_timing.fsp_table[3] = a[2].val;
		break;
	case EA_DDR_FSP0:
		/*
		 * First pair conatins drate and fw_type
		 */
		ea_ddr_dram_fsp_msg[0].drate   = a[0].reg;
		ea_ddr_dram_fsp_msg[0].fw_type = a[0].val;
		ea_ddr_dram_fsp_msg[0].fsp_cfg = &a[1];

		/* sz also contains the drate and fw_type pair -> remove one */
		ea_ddr_dram_fsp_msg[0].fsp_cfg_num = sz-1;

		break;
	case EA_DDR_FSP1:
		ea_ddr_dram_fsp_msg[1].drate   = a[0].reg;
		ea_ddr_dram_fsp_msg[1].fw_type = a[0].val;
		ea_ddr_dram_fsp_msg[1].fsp_cfg = &a[1];
		ea_ddr_dram_fsp_msg[1].fsp_cfg_num = sz-1;

		break;
	case EA_DDR_FSP2:
		ea_ddr_dram_fsp_msg[2].drate   = a[0].reg;
		ea_ddr_dram_fsp_msg[2].fw_type = a[0].val;
		ea_ddr_dram_fsp_msg[2].fsp_cfg = &a[1];
		ea_ddr_dram_fsp_msg[2].fsp_cfg_num = sz-1;

		break;
	case EA_DDR_FSP3:
		ea_ddr_dram_fsp_msg[3].drate   = a[0].reg;
		ea_ddr_dram_fsp_msg[3].fw_type = a[0].val;
		ea_ddr_dram_fsp_msg[3].fsp_cfg = &a[1];
		ea_ddr_dram_fsp_msg[3].fsp_cfg_num = sz-1;

		break;
	default:
		printf("Invalid ddr field index (%d). Invalid data in eeprom?\n", idx);
		break;
	}
}

static int spl_ddr_unpack_data(ea_eeprom_config_t* cfg)
{
	int ret;
	int offset;
	int nread=0;
	unsigned long len;
	struct dram_cfg_param* p;

	/* data_size is in this case the size of the gzipped data */
	len = cfg->data_size;

	ret = ea_eeprom_read_all_data(ea_gzbuf, EA_GZBUF_SZ, &nread);
	if (ret) {
		printf("Failed to read ddr data from eeprom %d\n", ret);
		return ret;
	}

	ret = gunzip(ea_dbuf, EA_DBUF_SZ, ea_gzbuf, &len);
	if (ret) {
		printf("Failed to unpack ddr data %d\n", ret);
		return ret;
	}

	p = (struct dram_cfg_param*)&ea_dbuf[0];

	offset = 0;
	while(offset*sizeof(struct dram_cfg_param) < len) {
		spl_ddr_map_array(p[offset].reg, &p[offset+1], p[offset].val);
		offset += (p[offset].val+1);
	}

	return ret;
}

static void spl_dram_init(uint32_t *size)
{
	ea_eeprom_config_t cfg;
	int ret;

        /* set default value, will be replaced if  eeprom cfg is valid */
        *size = (PHYS_SDRAM_SIZE >> 20);

#ifdef CONFIG_EA_IMX_PTP
	/* Skip reading eeprom */
	(void)cfg;
	(void)ret;
#else
        ret = ea_eeprom_get_config(&cfg);

        /* If eeprom is valid read ddr config; otherwise use default */
        if (!ret) {
                *size = cfg.ddr_size;

                /*
                 * timing values might exist in eeprom as gzipped data
                 */
                if (cfg.data_type == EA_EEPROM_DATA_TYPE_GZIP) {
                        printf("EA: Using gzipped ddr data from eeprom\n");
                        ret = spl_ddr_unpack_data(&cfg);
                }
        }
#endif

	ddr_init(&dram_timing);
}

int spl_board_boot_device(enum boot_device boot_dev_spl)
{
#ifdef CONFIG_SPL_BOOTROM_SUPPORT
	return BOOT_DEVICE_BOOTROM;
#else
	switch (boot_dev_spl) {
	case SD1_BOOT:
	case MMC1_BOOT:
	case SD2_BOOT:
	case MMC2_BOOT:
		return BOOT_DEVICE_MMC1;
	case SD3_BOOT:
	case MMC3_BOOT:
		return BOOT_DEVICE_MMC2;
	case QSPI_BOOT:
		return BOOT_DEVICE_NOR;
	case NAND_BOOT:
		return BOOT_DEVICE_NAND;
	case USB_BOOT:
		return BOOT_DEVICE_BOARD;
	default:
		return BOOT_DEVICE_NONE;
	}
#endif
}

void spl_board_init(void)
{
	puts("Normal Boot\n");
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR, 0, 4},
	{USDHC3_BASE_ADDR, 0, 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	/* we always boot from eMMC, which is always connected */
	return 1;
}

int board_mmc_init(struct bd_info *bis)
{
	/* Dummy code - Needed as SPL wants to boot from MMC1 so there
	   must be a MMC0. */
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
	//imx_iomux_v3_setup_multiple_pads(
	//	usdhc2_pads, ARRAY_SIZE(usdhc2_pads));

	fsl_esdhc_initialize(bis, &usdhc_cfg[0]);

	init_clk_usdhc(2);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	imx_iomux_v3_setup_multiple_pads(
		usdhc3_pads, ARRAY_SIZE(usdhc3_pads));

	return fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
}

#ifdef CONFIG_POWER
#define I2C_PMIC	0
int power_init_board(void)
{
	struct pmic *p;
	int ret;

	ret = power_bd71837_init(I2C_PMIC);
	if (ret)
		printf("power init failed");

	p = pmic_get("BD71837");
	pmic_probe(p);


	/* decrease RESET key long push time from the default 10s to 10ms */
	pmic_reg_write(p, BD718XX_PWRONCONFIG1, 0x0);

	/* unlock the PMIC regs */
	pmic_reg_write(p, BD718XX_REGLOCK, 0x1);

	/* increase VDD_ARM to typical value 0.85v for 1.2Ghz */
	pmic_reg_write(p, BD718XX_BUCK2_VOLT_RUN, 0xf);

	/* increase VDD_SOC/VDD_DRAM to typical value 0.85v for nominal mode */
	pmic_reg_write(p, BD718XX_BUCK1_VOLT_RUN, 0xf);

	/* increase VDD_DRAM to 0.975v for 3Ghz DDR */
	pmic_reg_write(p, BD718XX_1ST_NODVS_BUCK_VOLT, 0x83);

#ifdef CONFIG_IMX8M_DDR4
	/* increase NVCC_DRAM_1V2 to 1.2v for DDR4 */
	pmic_reg_write(p, BD718XX_4TH_NODVS_BUCK_VOLT, 0x28);
#endif

	/* lock the PMIC regs */
	pmic_reg_write(p, BD718XX_REGLOCK, 0x11);

	return 0;
}
#endif


#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	int ret;
	uint32_t size;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	board_early_init_f();

	timer_init();

	preloader_console_init();

	ret = spl_init();
	if (ret) {
		debug("spl_init() failed: %d\n", ret);
		hang();
	}

	enable_tzc380();

	/* Adjust pmic voltage to 1.0V for 800M */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	power_init_board();

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
	ea_conf->is_carrier_v2 = ea_is_carrier_v2(0);
	ea_conf->ddr_size = size;

	board_init_r(NULL, 0);
}

#ifdef CONFIG_SPL_MMC_SUPPORT

#define UBOOT_RAW_SECTOR_OFFSET 0x40
unsigned long spl_mmc_get_uboot_raw_sector(struct mmc *mmc)
{
	u32 boot_dev = spl_boot_device();
	switch (boot_dev) {
		case BOOT_DEVICE_MMC1:
			return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;
		case BOOT_DEVICE_MMC2:
			return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR - UBOOT_RAW_SECTOR_OFFSET;
	}
	return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;
}
#endif
