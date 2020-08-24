/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include "ea_common.h"
#include "ea_eeprom.h"

int ea_load_ethaddr(void)
{
	ea_eeprom_config_t config;

	/* stored MAC addresses to env variables */
	if (ea_eeprom_get_config(&config) == 0) {

		if (is_valid_ethaddr(config.mac1) && !env_get("ethaddr")) {
			eth_env_set_enetaddr("ethaddr", config.mac1);
		}

		if (is_valid_ethaddr(config.mac2) && !env_get("eth1addr")) {
			eth_env_set_enetaddr("eth1addr", config.mac2);
		}

		if (is_valid_ethaddr(config.mac3) && !env_get("eth2addr")) {
			eth_env_set_enetaddr("eth2addr", config.mac3);
		}

		if (is_valid_ethaddr(config.mac4) && !env_get("eth3addr")) {
			eth_env_set_enetaddr("eth3addr", config.mac4);
		}

		return 0;
	}

	return -1;
}
