/*
 * Copyright (C) 2019 Embedded Artists AB
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __EA_GPIO_EXPANDER_H
#define __EA_GPIO_EXPANDER_H

int ea_gpio_exp_configure(int i2c_bus);
bool ea_is_carrier_v2(int i2c_bus);

#endif

