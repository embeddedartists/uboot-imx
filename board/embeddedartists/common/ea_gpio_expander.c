/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <i2c.h>
#include <dm/uclass.h>
#include "ea_gpio_expander.h"

#define PCA6416_ADDR_V2 0x20
#define PCA6416_ADDR_V3 0x21

static int gpio_exp_configure(int i2c_bus, int i2c_addr, unsigned char* data)
{
#if !defined(CONFIG_DM_I2C)

	i2c_set_bus_num(i2c_bus);
	if (!i2c_probe(i2c_addr)) {
		if (i2c_write(i2c_addr, 0x02, 1, &data[0], 1)) {
			printf("Failed to configure PCA6416 GPIO Expander!\n");
			return -EIO;
		}
		if (i2c_write(i2c_addr, 0x03, 1, &data[1], 1)) {
			printf("Failed to configure PCA6416 GPIO Expander!\n");
			return -EIO;
		}
		if (i2c_write(i2c_addr, 0x06, 1, &data[2], 1)) {
			printf("Failed to configure PCA6416 GPIO Expander!\n");
			return -EIO;
		}
		if (i2c_write(i2c_addr, 0x07, 1, &data[3], 1)) {
			printf("Failed to configure PCA6416 GPIO Expander!\n");
			return -EIO;
		}
	}
#else

	struct udevice *bus;
	struct udevice *i2c_dev;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return -ENODEV;
	}

	ret = dm_i2c_probe(bus, i2c_addr, 0, &i2c_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x\n",
			__func__, i2c_addr);
		return -ENODEV;
	}


	if (dm_i2c_write(i2c_dev, 0x02, &data[0], 1)) {
		printf("Failed to configure PCA6416 GPIO Expander!\n");
		return -EIO;
	}
	if (dm_i2c_write(i2c_dev, 0x03, &data[1], 1)) {
		printf("Failed to configure PCA6416 GPIO Expander!\n");
		return -EIO;
	}
	if (dm_i2c_write(i2c_dev, 0x06, &data[2], 1)) {
		printf("Failed to configure PCA6416 GPIO Expander!\n");
		return -EIO;
	}
	if (dm_i2c_write(i2c_dev, 0x07, &data[3], 1)) {
		printf("Failed to configure PCA6416 GPIO Expander!\n");
		return -EIO;
	}


#endif
	else {
		return -ENODEV;
	}

	return 0;
}

int ea_get_carrier_board_version(int i2c_bus)
{
#if !defined(CONFIG_DM_I2C)
	i2c_set_bus_num(i2c_bus);
	i2c_init(CONFIG_SYS_I2C_SPEED, PCA6416_ADDR_V2);
	if (!i2c_probe(PCA6416_ADDR_V2)) {
		// Found PCA6416 => it is a rev v2 board
		return 2;
	}
	if (!i2c_probe(PCA6416_ADDR_V3)) {
		// Found PCA6416 => it is a rev v3 board
		return 3;
	}

#else
	struct udevice *bus;
	struct udevice *i2c_dev;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return -1;
	}

	ret = dm_i2c_probe(bus, PCA6416_ADDR_V2, 0, &i2c_dev);
	if (!ret) {
		// Found PCA6416 => it is a rev v2 board
		return 2;
	}
	ret = dm_i2c_probe(bus, PCA6416_ADDR_V3, 0, &i2c_dev);
	if (!ret) {
		// Found PCA6416 => it is a rev v3 board
		return 3;
	}
#endif
	return 1;
}


bool ea_is_carrier_v2(int i2c_bus)
{
	return ea_get_carrier_board_version(i2c_bus)==2;
}

bool ea_is_carrier_v3(int i2c_bus)
{
	return ea_get_carrier_board_version(i2c_bus)==3;
}

int ea_gpio_exp_configure(int i2c_bus)
{
	// All pins as outputs, LOW
	unsigned char settings[4] = {0,0,0,0};

	int v = ea_get_carrier_board_version(i2c_bus);
	if (v == 2) {
		return gpio_exp_configure(i2c_bus, PCA6416_ADDR_V2, settings);
	} else if (v == 3) {
		// Still all outputs (ignoring button) but LEDs are
		// set to HIGH to turn them off.
		settings[0] = (1<<4)|(1<<5)|(1<<6);
		settings[1] = (1<<6)|(1<<7);
		return gpio_exp_configure(i2c_bus, PCA6416_ADDR_V3, settings);
	}
	return 0;
}
