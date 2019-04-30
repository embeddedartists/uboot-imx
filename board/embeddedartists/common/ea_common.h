/*
 * Copyright (C) 2019 Embedded Artists AB
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __EA_COMMON_H
#define __EA_COMMON_H

#define EA_CONFIG_MAGIC 0xEA534852

/*
 * Confguration data setup by SPL and given to u-boot
 * via shared memory (EA_SHARED_CONFIG_MEM)
 *
 */
typedef struct {
	uint32_t magic;
	uint32_t ddr_size;	/* size in MB */
	bool is_carrier_v2;	/* true if a V2 carrier is detected */
} ea_config_t;

/*
 * Print information about the COM board
 */
int ea_print_board(void);

/*
 * Load MAC addresses from eeprom
 */
int ea_load_ethaddr(void);

/*
 * Configure TFP410
 */
int ea_configure_tfp410(void);

#endif

