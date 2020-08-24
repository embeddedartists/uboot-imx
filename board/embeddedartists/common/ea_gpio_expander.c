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

#define PCA6416_ADDR 0x20

int ea_gpio_exp_configure(int i2c_bus)
{
	unsigned char val = 0x00;

#if !defined(CONFIG_DM_I2C)

	i2c_set_bus_num(i2c_bus);
	if (!i2c_probe(PCA6416_ADDR)) {
		if (i2c_write(PCA6416_ADDR, 0x02, 1, &val, 1)) {
			printf("Failed to configure PCA6416 GPIO Expander!\n");
			return -EIO;
		}
		if (i2c_write(PCA6416_ADDR, 0x03, 1, &val, 1)) {
			printf("Failed to configure PCA6416 GPIO Expander!\n");
			return -EIO;
		}
		if (i2c_write(PCA6416_ADDR, 0x06, 1, &val, 1)) {
			printf("Failed to configure PCA6416 GPIO Expander!\n");
			return -EIO;
		}
		if (i2c_write(PCA6416_ADDR, 0x07, 1, &val, 1)) {
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

	ret = dm_i2c_probe(bus, PCA6416_ADDR, 0, &i2c_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x\n",
			__func__, PCA6416_ADDR);
		return -ENODEV;
	}


	if (dm_i2c_write(i2c_dev, 0x02, &val, 1)) {
		printf("Failed to configure PCA6416 GPIO Expander!\n");
		return -EIO;
	}
	if (dm_i2c_write(i2c_dev, 0x03, &val, 1)) {
		printf("Failed to configure PCA6416 GPIO Expander!\n");
		return -EIO;
	}
	if (dm_i2c_write(i2c_dev, 0x06, &val, 1)) {
		printf("Failed to configure PCA6416 GPIO Expander!\n");
		return -EIO;
	}
	if (dm_i2c_write(i2c_dev, 0x07, &val, 1)) {
		printf("Failed to configure PCA6416 GPIO Expander!\n");
		return -EIO;
	}


#endif
	else {
		//printf("COM Carrier Board pre rev PE9!\n");
		return -ENODEV;
	}

	return 0;
}

bool ea_is_carrier_v2(int i2c_bus)
{
#if !defined(CONFIG_DM_I2C)
	i2c_set_bus_num(i2c_bus);
	i2c_init(CONFIG_SYS_I2C_SPEED, PCA6416_ADDR);
	if (!i2c_probe(PCA6416_ADDR)) {
		// Found PCA6416 => it is a rev v2 board
		return true;
	}

#else
	struct udevice *bus;
	struct udevice *i2c_dev;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return false;
	}

	ret = dm_i2c_probe(bus, PCA6416_ADDR, 0, &i2c_dev);
	if (!ret) {
		// Found PCA6416 => it is a rev v2 board
		return true;
	}

#endif
	return false;
}
