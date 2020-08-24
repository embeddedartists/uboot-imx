/*
 * Copyright (C) 2019 Embedded Artists AB
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include "ea_eeprom.h"

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

