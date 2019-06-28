/*
 * Copyright (C) 2015 Embedded Artists AB
 *
 * Configuration parameters stored in EEPROM for the Embedded Artists
 * i.MX COM Board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <dm.h>

#include "ea_eeprom.h"

#ifdef CONFIG_DM_I2C
static int ea_dm_i2c_init(struct udevice **i2c_dev)
{
	struct udevice *bus;
        int ret;

        ret = uclass_get_device_by_seq(UCLASS_I2C, EA_EEPROM_I2C_BUS, &bus);
        if (ret) {
                printf("%s: Can't find bus\n", __func__);
                return -EINVAL;
        }

        ret = dm_i2c_probe(bus, EA_EEPROM_I2C_SLAVE, 0, i2c_dev);
        if (ret) {
                printf("%s: Can't find device id=0x%x\n",
                        __func__, EA_EEPROM_I2C_SLAVE);
                return -ENODEV;
        }

	return i2c_set_chip_offset_len(*i2c_dev, 2);

}
#endif


int ea_eeprom_init(void)
{
#if !defined(CONFIG_DM_I2C)
	i2c_set_bus_num(EA_EEPROM_I2C_BUS);
	i2c_init(CONFIG_SYS_I2C_SPEED, EA_EEPROM_I2C_SLAVE);
#endif

	return 0;
}

int ea_eeprom_get_config(ea_eeprom_config_t* config)
{
#if !defined(CONFIG_DM_I2C)

	i2c_set_bus_num(EA_EEPROM_I2C_BUS);

	if (i2c_probe(EA_EEPROM_I2C_SLAVE)) {
		return -ENODEV;
	}

	if (i2c_read(EA_EEPROM_I2C_SLAVE,
		0,
		2,
		(uint8_t *)config,
		sizeof(ea_eeprom_config_t)))
	{
		return -EIO;
	}
#else
	struct udevice *i2c_dev = NULL;
	int ret;

	ret = ea_dm_i2c_init(&i2c_dev);
	if (ret) {
		return ret;
	}

        ret = dm_i2c_read(i2c_dev, 0, (uint8_t *)config, sizeof(ea_eeprom_config_t));
        if (ret) {
                printf("%s dm_i2c_read failed, err %d\n", __func__, ret);
                return -EIO;
        }

#endif

	if (config->magic != EA_EEPROM_MAGIC) {
		printf("EA config: invalid magic number\n");
		return -EINVAL;
	}

	if (config->version > EA_EEPROM_CFG_VERSION) {
		printf("EA config: Unsupported config version (%d != %d)\n",
			config->version, EA_EEPROM_CFG_VERSION);
		return -EINVAL;
	}

	return 0;
}

int ea_eeprom_ddr_cfg_init(ea_ddr_cfg_t *cfg)
{
	ea_eeprom_config_t config;
	int ret = 0;

	ea_eeprom_init();
	ret = ea_eeprom_get_config(&config);
	if (!ret) {
		cfg->num_pairs = config.data_size;
		cfg->next = 0;
		cfg->ddr_size_mb = config.ddr_size;
	}

	return ret;
}

int ea_eeprom_ddr_cfg_read(ea_ddr_cfg_t *cfg, ea_ddr_cfg_pair_t* pairs,
	int num, int *num_read)
{
	int to_read;

#ifdef CONFIG_DM_I2C
	int ret;
	struct udevice *i2c_dev = NULL;

        ret = ea_dm_i2c_init(&i2c_dev);
        if (ret) {
                return ret;
        }

#endif

	*num_read = 0;

	/* max to read */
	to_read = cfg->num_pairs - cfg->next;

	/* no more to read */
	if (to_read <= 0) return 0;

	/* fewer requested */
	if (num < to_read) to_read = num;


#if !defined(CONFIG_DM_I2C)
	ea_eeprom_init();
	if (i2c_read(EA_EEPROM_I2C_SLAVE,
		sizeof(ea_eeprom_config_t)+cfg->next*sizeof(ea_ddr_cfg_pair_t),
		2,
		(uint8_t *)pairs,
		to_read*sizeof(ea_ddr_cfg_pair_t)))
	{
		return -EIO;
	}
#else
	ret = dm_i2c_read(i2c_dev,
		sizeof(ea_eeprom_config_t)+cfg->next*sizeof(ea_ddr_cfg_pair_t),
		(uint8_t *)pairs,
		to_read*sizeof(ea_ddr_cfg_pair_t));
	if (ret) {
		printf("%s dm_i2c_read failed, err %d\n", __func__, ret);
		return -EIO;
	}
#endif

	*num_read = to_read;
	cfg->next += to_read;

	return 0;
}

int ea_eeprom_read_all_data(uint8_t* buf, int buf_sz, int *read)
{
	int to_read;
        ea_eeprom_config_t config;
        int ret = 0;

        ea_eeprom_init();
        ret = ea_eeprom_get_config(&config);
        if (ret) return ret;


#ifdef CONFIG_DM_I2C
	struct udevice *i2c_dev = NULL;

        ret = ea_dm_i2c_init(&i2c_dev);
        if (ret) return ret;

#endif

	*read = 0;

	to_read = config.data_size;
	if (buf_sz < to_read) return -EINVAL;


#if !defined(CONFIG_DM_I2C)

	if (i2c_read(EA_EEPROM_I2C_SLAVE,
		sizeof(ea_eeprom_config_t),
		2,
		buf,
		to_read))
	{
		return -EIO;
	}
#else
	ret = dm_i2c_read(i2c_dev,
		sizeof(ea_eeprom_config_t),
		buf,
		to_read);
	if (ret) {
		printf("%s dm_i2c_read failed, err %d\n", __func__, ret);
		return -EIO;
	}
#endif

	*read = to_read;

	return 0;
}
