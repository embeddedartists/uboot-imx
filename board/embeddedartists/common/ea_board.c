/*
 * Copyright (C) 2019 Embedded Artists AB
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <env.h>
#include "ea_eeprom.h"

static char ea_board_part[9];
static char ea_board_batch[9];

int ea_print_board(void)
{
	ea_eeprom_config_t config;

	printf("Board: Embedded Artists ");
	if (ea_eeprom_get_config(&config) == 0) {

		printf("%s\n", config.name);
		printf("       %05d, %s, WO%d\n",
			config.board_part_nr,
			config.board_rev,
			config.batch);
	}
	else {
		printf(" [Unknown board due to invalid configuration data]\n");
	}

	return 0;
}

int ea_board_info_to_env(void)
{
	ea_eeprom_config_t config;

	if (ea_eeprom_get_config(&config) == 0) {

		sprintf(ea_board_part, "%d", config.board_part_nr);
		env_set("board_part", ea_board_part);
		sprintf(ea_board_batch, "%d", config.batch);
		env_set("board_batch", ea_board_batch);
		env_set("board_rev", (const char*)config.board_rev);
		env_save();
	}

	return 0;
}

