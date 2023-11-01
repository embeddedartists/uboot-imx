// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#include <common.h>
#include <command.h>
#include <cpu_func.h>
#include <hang.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <spl.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch/imx93_pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch-mx7ulp/gpio.h>
#include <asm/mach-imx/syscounter.h>
#include <asm/mach-imx/ele_api.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <linux/delay.h>
#include <asm/arch/clock.h>
#include <asm/arch/ccm_regs.h>
#include <asm/arch/ddr.h>
#include <power/pmic.h>
#include <power/pca9450.h>
#include <asm/arch/trdc.h>

#include <gzip.h>
#include "../common/ea_common.h"
#include "../common/ea_eeprom.h"

DECLARE_GLOBAL_DATA_PTR;

#define I2C_PAD_CTRL	(PAD_CTL_DSE(6) | PAD_CTL_HYS | PAD_CTL_PUE)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX93_PAD_I2C1_SCL__LPI2C1_SCL | PC,
		.gpio_mode = MX93_PAD_I2C1_SCL__GPIO1_IO00 | PC,
		.gp = IMX_GPIO_NR(1, 0),
	},
	.sda = {
		.i2c_mode = MX93_PAD_I2C1_SDA__LPI2C1_SDA | PC,
		.gpio_mode = MX93_PAD_I2C1_SDA__GPIO1_IO01 | PC,
		.gp = IMX_GPIO_NR(1, 1),
	},
};


int spl_board_boot_device(enum boot_device boot_dev_spl)
{
#ifdef CONFIG_SPL_BOOTROM_SUPPORT
	return BOOT_DEVICE_BOOTROM;
#else
	switch (boot_dev_spl) {
	case SD1_BOOT:
	case MMC1_BOOT:
		return BOOT_DEVICE_MMC1;
	case SD2_BOOT:
	case MMC2_BOOT:
		return BOOT_DEVICE_MMC2;
	default:
		return BOOT_DEVICE_NONE;
	}
#endif
}

void spl_board_init(void)
{
	int ret;

	puts("Normal Boot\n");

	ret = ahab_start_rng();
	if (ret)
		printf("Fail to start RNG: %d\n", ret);
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

void spl_dram_init(uint32_t *size)
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

#if CONFIG_IS_ENABLED(DM_PMIC_PCA9450)
int power_init_board(void)
{
	struct udevice *dev;
	int ret;

	ret = pmic_get("pmic@25", &dev);
	if (ret == -ENODEV) {
		puts("No pca9450@25\n");
		return 0;
	}
	if (ret != 0)
		return ret;

	/* BUCKxOUT_DVS0/1 control BUCK123 output */
	pmic_reg_write(dev, PCA9450_BUCK123_DVS, 0x29);

	/* enable DVS control through PMIC_STBY_REQ */
	pmic_reg_write(dev, PCA9450_BUCK1CTRL, 0x59);

	if (IS_ENABLED(CONFIG_IMX9_LOW_DRIVE_MODE)){
		/* 0.8v for Low drive mode
		 */
		pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS0, 0x10);
		pmic_reg_write(dev, PCA9450_BUCK3OUT_DVS0, 0x10);
	} else {
		/* 0.9v for Over drive mode
		 */
		pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS0, 0x18);
		pmic_reg_write(dev, PCA9450_BUCK3OUT_DVS0, 0x18);
	}

	/* set standby voltage to 0.65v */
	pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS1, 0x4);

	/* I2C_LT_EN*/
	pmic_reg_write(dev, 0xa, 0x3);

	/* set WDOG_B_CFG to cold reset */
	pmic_reg_write(dev, PCA9450_RESET_CTRL, 0xA1);
	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	int ret;
	uint32_t size;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	timer_init();

	arch_cpu_init();

	board_early_init_f();

	spl_early_init();

	preloader_console_init();

	ret = arch_cpu_init();
	if (ret) {
		printf("Fail to init Sentinel API\n");
	} else {
		printf("SOC: 0x%x\n", gd->arch.soc_rev);
		printf("LC: 0x%x\n", gd->arch.lifecycle);
	}

	power_init_board();

	if (!IS_ENABLED(CONFIG_IMX9_LOW_DRIVE_MODE))
		set_arm_core_max_clk();

	/* Init power of mix */
	soc_power_init();

	/* Setup TRDC for DDR access */
	trdc_init();

	/* DDR initialization */
	spl_dram_init(&size);

	/* Put M33 into CPUWAIT for following kick */
	ret = m33_prepare();
	if (!ret)
		printf("M33 prepare ok\n");

	board_init_r(NULL, 0);
}
