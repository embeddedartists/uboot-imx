/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX8MN_EVK_H
#define __MX8MN_EVK_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#include "imx_env.h"

#if defined(CONFIG_SPL_BUILD)
#undef CONFIG_DM_I2C
#endif

#define CFG_SYS_UBOOT_BASE \
	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD
/* malloc f used before GD_FLG_FULL_MALLOC_INIT set */
#define CFG_MALLOC_F_ADDR		0x00940000
#endif

#define CFG_FEC_MXC_PHYADDR          0

#ifdef CONFIG_DISTRO_DEFAULTS
#define BOOT_TARGET_DEVICES(func) \
	func(USB, usb, 0) \
	func(MMC, mmc, 1) \
	func(MMC, mmc, 2)

#include <config_distro_bootcmd.h>
#else
#define BOOTENV
#endif


#define JAILHOUSE_ENV \
	"jh_clk= \0 " \
	"jh_mmcboot=mw 0x303d0518 0xff; setenv fdt_file fsl-imx8mn-ddr4-evk-root.dtb;" \
		"setenv jh_clk clk_ignore_unused; " \
			   "if run loadimage; then " \
				   "run mmcboot; " \
			   "else run jh_netboot; fi; \0" \
	"jh_netboot=mw 0x303d0518 0xff; setenv fdt_file fsl-imx8mn-ddr4-evk-root.dtb; setenv jh_clk clk_ignore_unused; run netboot; \0 "

#define CFG_MFG_ENV_SETTINGS \
	CFG_MFG_ENV_SETTINGS_DEFAULT \
	"initrd_addr=0x43800000\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"emmc_dev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0"\
	"sd_dev=0\0" \

#define M_CORE_ENV \
	"cm_addr=0x7e0000\0" \
	"cm_image=cm_TCM_hello_world.bin\0" \
	"cm_loadimage=fatload mmc ${mmcdev} ${loadaddr} ${cm_image}\0" \
	"cm_boot=run cm_loadimage; cp.b ${loadaddr} ${cm_addr} ${filesize}; dcache flush; bootaux ${cm_addr}\0"

/* Initial environment variables */
#define CFG_EXTRA_ENV_SETTINGS		\
	CFG_MFG_ENV_SETTINGS \
	JAILHOUSE_ENV \
	M_CORE_ENV \
	BOOTENV \
	"scriptaddr=0x43500000\0" \
	"kernel_addr_r=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
	"bsp_script=boot.scr\0" \
	"image=Image\0" \
	"splashimage=0x50000000\0" \
	"console=ttymxc1,115200\0" \
	"fdt_addr_r=0x43000000\0"			\
	"fdt_addr=0x43000000\0"			\
	"fdt_high=0xffffffffffffffff\0"		\
	"boot_fdt=try\0" \
	"fdt_file=\0" \
	"bootm_size=0x10000000\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=1\0" \
	"mmcroot=" CFG_MMCROOT " rootwait rw\0" \
	"mmcautodetect=yes\0" \
	"mmcargs=setenv bootargs ${jh_clk} console=${console} root=${mmcroot} " \
		"${args_from_script}\0" \
	"loadbootscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bsp_script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr_r} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"booti ${loadaddr} - ${fdt_addr_r}; " \
			"else " \
				"echo WARN: Cannot load the DT; " \
			"fi; " \
		"else " \
			"echo wait for boot; " \
		"fi;\0" \
	"netargs=setenv bootargs ${jh_clk} console=${console} " \
		"root=/dev/nfs " \
		"${args_from_script}\0" \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs;  " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${loadaddr} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr_r} ${fdt_file}; then " \
				"booti ${loadaddr} - ${fdt_addr_r}; " \
			"else " \
				"echo WARN: Cannot load the DT; " \
			"fi; " \
		"else " \
			"booti; " \
		"fi;\0" \
	   "bsp_bootcmd=echo Running BSP bootcmd ...; " \
		"mmc dev ${mmcdev}; if mmc rescan; then " \
		   "if run loadbootscript; then " \
			   "run bootscript; " \
		   "else " \
			   "if run loadimage; then " \
				   "run mmcboot; " \
			   "else run netboot; " \
			   "fi; " \
		   "fi; " \
	   "fi;"

/* Link Definitions */

#define CFG_SYS_INIT_RAM_ADDR        0x40000000
#define CFG_SYS_INIT_RAM_SIZE        0x80000

#define CFG_MMCROOT			"/dev/mmcblk1p2"  /* USDHC2 */

#define CFG_SYS_SDRAM_BASE              0x40000000
#define PHYS_SDRAM                      0x40000000
#define PHYS_SDRAM_SIZE			0x40000000 /* 1GB DDR */

#define CFG_MXC_UART_BASE		UART2_BASE_ADDR

#define CFG_SYS_FSL_USDHC_NUM		3
#define CFG_SYS_FSL_ESDHC_ADDR		0

#define EA_SHARED_CONFIG_MEM (CONFIG_CUSTOM_SYS_SPL_MALLOC_ADDR + CONFIG_SYS_SPL_MALLOC_SIZE)

#endif
