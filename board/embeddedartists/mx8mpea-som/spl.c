// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2018-2019, 2021 NXP
 *
 */

#include <hang.h>
#include <init.h>
#include <log.h>
#include <spl.h>
#include <asm/global_data.h>
#include <asm/arch/imx8mp_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/boot_mode.h>
#include <power/pmic.h>

#include <power/pca9450.h>
#include <asm/arch/clock.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/ddr.h>
#include <asm/sections.h>

#include <gzip.h>
#include "../common/ea_common.h"
#include "../common/ea_eeprom.h"

DECLARE_GLOBAL_DATA_PTR;

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

void spl_dram_init(void)
{
	ea_eeprom_config_t cfg;
	int ret;

	ret = ea_eeprom_get_config(&cfg);
	if (!ret) {

		printf("EA: Using ddr timing data from eeprom\n");

		/*
		 * timing values might exist in eeprom as gzipped data
		 */
		if (cfg.data_type == EA_EEPROM_DATA_TYPE_GZIP) {
			printf("EA: Using gzipped ddr data from eeprom\n");
			ret = spl_ddr_unpack_data(&cfg);
		}

	}

	ddr_init(&dram_timing);
}

void spl_board_init(void)
{
	arch_misc_init();

	/*
	 * Set GIC clock to 500Mhz for OD VDD_SOC. Kernel driver does
	 * not allow to change it. Should set the clock after PMIC
	 * setting done. Default is 400Mhz (system_pll1_800m with div = 2)
	 * set by ROM for ND VDD_SOC
	 */
#if defined(CONFIG_IMX8M_LPDDR4) && !defined(CONFIG_IMX8M_VDD_SOC_850MV)
	clock_enable(CCGR_GIC, 0);
	clock_set_target_val(GIC_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(5));
	clock_enable(CCGR_GIC, 1);

	puts("Normal Boot\n");
#endif
}

#if CONFIG_IS_ENABLED(DM_PMIC_PCA9450)
int power_init_board(void)
{
	struct udevice *dev;
	int ret;

	ret = pmic_get("pmic@25", &dev);
	if (ret == -ENODEV) {
		puts("No pmic@25\n");
		return 0;
	}
	if (ret < 0)
		return ret;

	/* BUCKxOUT_DVS0/1 control BUCK123 output */
	pmic_reg_write(dev, PCA9450_BUCK123_DVS, 0x29);

#ifdef CONFIG_IMX8M_LPDDR4
	/*
	 * Increase VDD_SOC to typical value 0.95V before first
	 * DRAM access, set DVS1 to 0.85V for suspend.
	 * Enable DVS control through PMIC_STBY_REQ and
	 * set B1_ENMODE=1 (ON by PMIC_ON_REQ=H)
	 */
	if (CONFIG_IS_ENABLED(IMX8M_VDD_SOC_850MV))
		pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS0, 0x14);
	else
		pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS0, 0x1C);

	pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS1, 0x14);
	pmic_reg_write(dev, PCA9450_BUCK1CTRL, 0x59);

	/*
	 * Kernel uses OD/OD freq for SOC.
	 * To avoid timing risk from SOC to ARM,increase VDD_ARM to OD
	 * voltage 0.95V.
	 */

	pmic_reg_write(dev, PCA9450_BUCK2OUT_DVS0, 0x1C);
#elif defined(CONFIG_IMX8M_DDR4)
	/* DDR4 runs at 3200MTS, uses default ND 0.85v for VDD_SOC and VDD_ARM */
	pmic_reg_write(dev, PCA9450_BUCK1CTRL, 0x59);

	/* Set NVCC_DRAM to 1.2v for DDR4 */
	pmic_reg_write(dev, PCA9450_BUCK6OUT, 0x18);
#endif

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
	struct udevice *dev;
	int ret;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	board_early_init_f();

	timer_init();

	ret = spl_early_init();
	if (ret) {
		debug("spl_early_init() failed: %d\n", ret);
		hang();
	}

	ret = uclass_get_device_by_name(UCLASS_CLK,
					"clock-controller@30380000",
					&dev);
	if (ret < 0) {
		printf("Failed to find clock node. Check device tree\n");
		hang();
	}

	preloader_console_init();

	enable_tzc380();

	power_init_board();

	/* DDR initialization */
	spl_dram_init();

	board_init_r(NULL, 0);
}
