// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2018-2019, 2021 NXP
 *
 */

#include <command.h>
#include <cpu_func.h>
#include <hang.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <spl.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx8mn_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/arch/ddr.h>
#include <asm/sections.h>

#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <power/pmic.h>
#include <power/pca9450.h>
#include <power/bd71837.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <linux/delay.h>

#include "../common/ea_common.h"
#include "../common/ea_eeprom.h"
#include "../common/ea_gpio_expander.h"

DECLARE_GLOBAL_DATA_PTR;

#include "ddr4_timing.c"

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

void spl_dram_init(void)
{
	ddr_init(&dram_timing);
}

#if CONFIG_IS_ENABLED(DM_PMIC_BD71837)
int power_init_board(void)
{
	struct udevice *dev;
	int ret;

	ret = pmic_get("bd71837@4b", &dev);
	if (ret == -ENODEV) {
		puts("No bd71837@4b\n");
		return 0;
	}
	if (ret != 0)
		return ret;

	/* decrease RESET key long push time from the default 10s to 10ms */
	pmic_reg_write(dev, BD718XX_PWRONCONFIG1, 0x0);

	/* unlock the PMIC regs */
	pmic_reg_write(dev, BD718XX_REGLOCK, 0x1);

	/* Set VDD_ARM to typical value 0.85v for 1.2Ghz */
	pmic_reg_write(dev, BD718XX_BUCK2_VOLT_RUN, 0xf);

#ifdef CONFIG_IMX8MN_LOW_DRIVE_MODE
	/* Set VDD_SOC/VDD_DRAM to typical value 0.8v for low drive mode */
	pmic_reg_write(dev, BD718XX_BUCK1_VOLT_RUN, 0xa);
#else
	/* Set VDD_SOC/VDD_DRAM to typical value 0.85v for nominal mode */
	pmic_reg_write(dev, BD718XX_BUCK1_VOLT_RUN, 0xf);
#endif /* CONFIG_IMX8MN_LOW_DRIVE_MODE */

	/* increase VDD_DRAM to 0.975v for 3Ghz DDR */
	pmic_reg_write(dev, BD718XX_1ST_NODVS_BUCK_VOLT, 0x83);

#ifdef CONFIG_IMX8M_DDR4
	/* increase NVCC_DRAM_1V2 to 1.2v for DDR4 */
	pmic_reg_write(dev, BD718XX_4TH_NODVS_BUCK_VOLT, 0x28);
#endif

	/* lock the PMIC regs */
	pmic_reg_write(dev, BD718XX_REGLOCK, 0x11);

	return 0;
}
#endif

void spl_board_init(void)
{
	arch_misc_init();

	puts("Normal Boot\n");
}

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

#ifdef CONFIG_SPL_MMC
#define UBOOT_RAW_SECTOR_OFFSET 0x40
unsigned long spl_mmc_get_uboot_raw_sector(struct mmc *mmc, unsigned long raw_sect)
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
