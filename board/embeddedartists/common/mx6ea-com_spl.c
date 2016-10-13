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

#include "mx6ea_eeprom.h"


void spl_board_init(void)
{

	// must be called to get correct clock for MMC/SD
	get_clocks();

	// called to setup I2C mux
	board_early_init_f();

	preloader_console_init();

	// initialize dram based on data from eeprom
	if (ea_eeprom_dram_init() != 0) {
		hang();
	}

}



#endif
