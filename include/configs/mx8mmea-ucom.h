/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __IMX8MM_EVK_H
#define __IMX8MM_EVK_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#if defined(CONFIG_SPL_BUILD)
#undef CONFIG_DM_I2C
#endif


#define CFG_SYS_UBOOT_BASE	\
	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD
/* malloc f used before GD_FLG_FULL_MALLOC_INIT set */
#define CFG_MALLOC_F_ADDR		0x912000
#endif

#define CFG_FEC_MXC_PHYADDR          0

/* Link Definitions */

#define CFG_SYS_INIT_RAM_ADDR        0x40000000
#define CFG_SYS_INIT_RAM_SIZE        0x200000

#define CFG_SYS_SDRAM_BASE              0x40000000
#define PHYS_SDRAM                      0x40000000
#define PHYS_SDRAM_SIZE					0x40000000 /* 1GB DDR */

#define CFG_MXC_UART_BASE		UART2_BASE_ADDR

#define CFG_SYS_FSL_USDHC_NUM		3
#define CFG_SYS_FSL_ESDHC_ADDR		0

#define EA_SHARED_CONFIG_MEM (CONFIG_SPL_CUSTOM_SYS_MALLOC_ADDR + CONFIG_SPL_SYS_MALLOC_SIZE)

#endif
