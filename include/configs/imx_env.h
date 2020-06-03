/* Copyright 2018 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef __IMX_COMMON_CONFIG_H
#define __IMX_COMMON_CONFIG_H

#ifdef CONFIG_ARM64
    #define MFG_BOOT_CMD "booti "
#else
    #define MFG_BOOT_CMD "bootz "
#endif

#ifdef CONFIG_EA_IMX_PTP
    #define EXTRA_MFGTOOL_ARGS	"drm_kms_helper.edid_firmware=edid/800x480.bin "
#else
    #define EXTRA_MFGTOOL_ARGS  " "
#endif

#define CONFIG_MFG_ENV_SETTINGS_DEFAULT \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"rdinit=/linuxrc " \
		"clk_ignore_unused "\
                EXTRA_MFGTOOL_ARGS \
		"\0" \
	"kboot="MFG_BOOT_CMD"\0"\
	"bootcmd_mfg=run mfgtool_args;" \
        "if iminfo ${initrd_addr}; then " \
            "if test ${tee} = yes; then " \
                "bootm ${tee_addr} ${initrd_addr} ${fdt_addr}; " \
            "else " \
                MFG_BOOT_CMD "${loadaddr} ${initrd_addr} ${fdt_addr}; " \
            "fi; " \
        "else " \
            "echo \"Run fastboot ...\"; fastboot 0; "  \
        "fi;\0" \

#endif

#define CONFIG_FAT_WRITE
