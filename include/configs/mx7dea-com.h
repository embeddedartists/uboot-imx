/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Embedded Artists i.MX7 Dual COM Board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#ifndef __MX7DEA_COM_H
#define __MX7DEA_COM_H

#include "mx7_common.h"

#ifdef CONFIG_SPL
#include "imx7_spl.h"
#endif

#define CONFIG_DBG_MONITOR

/* uncomment for PLUGIN mode support */
/* #define CONFIG_USE_PLUGIN */

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

#ifdef CONFIG_SECURE_BOOT
#ifndef CONFIG_CSF_SIZE
#define CONFIG_CSF_SIZE 0x2000
#endif
#endif

#define CONFIG_MXC_UART_BASE            UART1_IPS_BASE_ADDR

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN           (32 * SZ_1M)

#ifdef CONFIG_IMX_BOOTAUX
/* Set to QSPI1 A flash at default */
#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_AUXCORE_BOOTDATA 0x60100000 /* Set to QSPI1 A flash, offset 1M */
#else
#define CONFIG_SYS_AUXCORE_BOOTDATA 0x7F8000 /* Set to TCML address */
#endif


#ifdef CONFIG_FSL_QSPI
#define UPDATE_M4_ENV \
        "m4image=m4_qspi.bin\0" \
        "loadm4image=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${m4image}\0" \
        "update_m4_from_sd=" \
                "if sf probe 0:0; then " \
                        "if run loadm4image; then " \
                                "setexpr fw_sz ${filesize} + 0xffff; " \
                                "setexpr fw_sz ${fw_sz} / 0x10000; "    \
                                "setexpr fw_sz ${fw_sz} * 0x10000; "    \
                                "sf erase 0x100000 ${fw_sz}; " \
                                "sf write ${loadaddr} 0x100000 ${filesize}; " \
                        "fi; " \
                "fi\0" \
        "m4boot=sf probe 0:0; bootaux "__stringify(CONFIG_SYS_AUXCORE_BOOTDATA)"\0"
#else
#define UPDATE_M4_ENV \
        "m4image=m4_qspi.bin\0" \
        "loadm4image=fatload mmc ${mmcdev}:${mmcpart} "__stringify(CONFIG_SYS_AUXCORE_BOOTDATA)" ${m4image}\0" \
        "m4boot=run loadm4image; bootaux "__stringify(CONFIG_SYS_AUXCORE_BOOTDATA)"\0"
#endif

#else
#define UPDATE_M4_ENV ""
#endif

#define CONFIG_MFG_ENV_SETTINGS_DEFAULT \
        "mfgtool_args=setenv bootargs console=${console},${baudrate} " \
                "rdinit=/linuxrc " \
                "clk_ignore_unused "\
                "\0" \
        "kboot=bootz\0"\
        "bootcmd_mfg=run mfgtool_args;" \
        "if iminfo ${initrd_addr}; then " \
            "if test ${tee} = yes; then " \
                "bootm ${tee_addr} ${initrd_addr} ${fdt_addr}; " \
            "else " \
                "bootz ${loadaddr} ${initrd_addr} ${fdt_addr}; " \
            "fi; " \
        "else " \
            "echo \"Run fastboot ...\"; fastboot 0; "  \
        "fi;\0"\

#define CONFIG_MFG_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS_DEFAULT \
	"initrd_addr=0x83800000\0" \
	"initrd_high=0xffffffff\0" \
	"emmc_dev=1\0"\

#ifdef EA_IMX_PTP
#define EA_IMX_PTP_ENV_SETTINGS "eadisp_rgb_enabled=yes\0"
#else
#define EA_IMX_PTP_ENV_SETTINGS ""
#endif

#define CONFIG_DFU_ENV_SETTINGS \
	"dfu_alt_info=image raw 0 0x800000;"\
		"u-boot raw 0 0x4000;"\
		"bootimg part 0 1;"\
		"rootfs part 0 2\0"\

#ifdef CONFIG_TARGET_MX7DEA_COM
#define FDT_FILE "imx7dea-com-kit_v2.dtb"
#else
#define FDT_FILE "imx7dea-ucom-kit_v2.dtb"
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	EA_IMX_PTP_ENV_SETTINGS \
	UPDATE_M4_ENV \
	CONFIG_MFG_ENV_SETTINGS \
	CONFIG_DFU_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"console=ttymxc0\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_file=" FDT_FILE "\0" \
	"fdt_addr=0x83000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"panel=Hannstar-XGA\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"mmcautodetect=yes\0" \
  "fmac_txrx_opt=brcmfmac.sdio_wq_highpri=1\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot} ${fmac_txrx_opt} ${args_from_script}\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
	"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp ${args_from_script}\0" \
		"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev};" \
	   "mmc dev ${mmcdev}; if mmc rescan; then " \
		   "if run loadbootscript; then " \
			   "run bootscript; " \
		   "else " \
			   "if run loadimage; then " \
				   "run mmcboot; " \
			   "else run netboot; " \
			   "fi; " \
		   "fi; " \
	   "else run netboot; fi"

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x10000)

#define CONFIG_SYS_LOAD_ADDR            CONFIG_LOADADDR
#define CONFIG_SYS_HZ                   1000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			SZ_1G

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_FSL_QSPI_AHB
#define CONFIG_SF_DEFAULT_BUS           1
#define CONFIG_SF_DEFAULT_CS            0
#define CONFIG_SF_DEFAULT_SPEED         40000000
#define CONFIG_SF_DEFAULT_MODE          SPI_MODE_0
#define FSL_QSPI_FLASH_NUM              1
#define FSL_QSPI_FLASH_SIZE             SZ_32M
#define QSPI0_BASE_ADDR                 QSPI1_IPS_BASE_ADDR
#define QSPI0_AMBA_BASE                 QSPI0_ARB_BASE_ADDR
#endif



/* MMC Configuration */
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_MMC_ENV_DEV		1  /* USDHC3/eMMC */
#define CONFIG_SYS_MMC_ENV_PART		1  /* 0=user area, 1=1st MMC boot part., 2=2nd MMC boot part. */
#define CONFIG_MMCROOT			"/dev/mmcblk2p2"  /* USDHC3/eMMC */

#define CONFIG_SYS_FSL_ESDHC_ADDR	0

/* I2C Configs */
#ifndef CONFIG_DM_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#endif
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		  100000

#define CONFIG_SUPPORT_EMMC_BOOT        /* eMMC specific */

/* Network */

#if CONFIG_DM_ETH

#define CONFIG_FEC_MXC
#define CONFIG_MII

#define IMX_FEC_BASE			ENET_IPS_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x1

#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_ETHPRIME                 "eth0"

#define CONFIG_PHY_BROADCOM

#endif

/* USB Configs */
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)

#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
/*#define CONFIG_IMX_VIDEO_SKIP*/
#endif

#define CONFIG_ENV_SIZE			SZ_8K
#define CONFIG_ENV_OFFSET		(SZ_2M - CONFIG_ENV_SIZE)
/*#define CONFIG_ENV_OFFSET		(8 * SZ_64K)*/

#if defined(CONFIG_ANDROID_SUPPORT)
#include "mx7dsabresdandroid.h"
#elif defined(CONFIG_ANDROID_THINGS_SUPPORT)
#include "mx7dsabresd_androidthings.h"
#else
#define CONFIG_USBD_HS
#endif

#define EA_SHARED_CONFIG_MEM (CONFIG_SYS_SPL_MALLOC_START + CONFIG_SYS_SPL_MALLOC_SIZE)


#endif				/* __CONFIG_H */
