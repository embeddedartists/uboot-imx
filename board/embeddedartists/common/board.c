/*
 * Copyright (C) 2019 Embedded Artists AB
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include "ea_eeprom.h"

int checkboard(void)
{
	ea_eeprom_config_t config;

	puts ("Board: Embedded Artists ");
	if (ea_eeprom_get_config(&config) == 0) {

		printf("%s\n", config.name);
		printf("       %05d, %s, WO%d\n",
			config.board_part_nr,
			config.board_rev,
			config.batch);
	}
	else {
		puts(" [Unknown board due to invalid configuration data]\n");
	}

	return 0;
}
