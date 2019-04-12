/*
 * Copyright (C) 2015 Embedded Artists AB
 *
 * Configuration parameters stored in EEPROM for the Embedded Artists 
 * i.MX 6 COM Board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6SXEA_EEPROM_H
#define __MX6SXEA_EEPROM_H


#define EA_EEPROM_I2C_BUS   0
#define EA_EEPROM_I2C_SLAVE 0x55

#define EA_EEPROM_MAGIC 0xEA434F4D

typedef struct {
	uint32_t magic;
	uint8_t  config_version;
	uint32_t board_part_nr;
	uint8_t  board_rev[4];
	uint32_t batch;
	uint8_t  name[32];
	uint8_t  mac1[6];
	uint8_t  mac2[6];
	uint8_t  mac3[6];
	uint8_t  mac4[6];
	uint32_t ddr_size;
	uint32_t num_reg_value_pairs;
	uint8_t  reserved[3];
} __attribute__((__packed__)) ea_eeprom_config_t;

typedef struct {
	u32 num_pairs;
	u32 next;
	u32 ddr_size_mb;
} ea_ddr_cfg_t;

typedef struct {
	u32 reg;
	u32 val;
} ea_ddr_cfg_pair_t;

int ea_eeprom_init(void);
int ea_eeprom_get_config(ea_eeprom_config_t* config);
int ea_eeprom_dram_init(void);
int ea_eeprom_ddr_cfg_init(ea_ddr_cfg_t *cfg);
int ea_eeprom_ddr_cfg_read(ea_ddr_cfg_t *cfg, ea_ddr_cfg_pair_t* pairs,
        int num, int *num_read);


#endif

