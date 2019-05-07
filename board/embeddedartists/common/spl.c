/*
 * Copyright (C) 2015 Embedded Artists AB
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>

#include <spl.h>


#if defined(CONFIG_SPL_BUILD)

#include "ea_eeprom.h"

#if !(defined(CONFIG_TARGET_MX7DEA_COM) || defined(CONFIG_TARGET_MX7DEA_UCOM) || defined(CONFIG_TARGET_MX6QEA_COM) || defined(CONFIG_TARGET_MX6DLEA_COM) || defined(CONFIG_TARGET_MX8MQEA_COM) || defined(CONFIG_TARGET_MX6SXEA_COM))
void spl_board_init(void)
{
	// must be called to get correct clock for MMC/SD
	get_clocks();

	/* setup GP timer */
	timer_init();

	// called to setup I2C mux
	board_early_init_f();

	preloader_console_init();

	// initialize dram based on data from eeprom
	if (ea_eeprom_dram_init() != 0) {
		hang();
	}

}
#endif



#endif
