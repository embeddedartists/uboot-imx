/*
 * Copyright (C) 2019 Embedded Artists AB
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <i2c.h>
#include <dm.h>

#define TFP410_ADDR 0x3A

/*
 * Configure the TFP410 chip on the iMX PTP board.
 */
int ea_configure_tfp410(void)
{
	int ret;
	struct udevice *bus;
	struct udevice *i2c_dev;
	uint8_t buff[15] = {0};

	printf("Override for mfgtool\n");

	ret = uclass_get_device_by_seq(UCLASS_I2C, 0, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return -EINVAL;
	}

	ret = dm_i2c_probe(bus, TFP410_ADDR, 0, &i2c_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x\n",
			__func__, TFP410_ADDR);
		return -ENODEV;
	}

	ret = dm_i2c_read(i2c_dev, 0, buff, 5);
	if (ret) {
		printf("%s dm_i2c_read failed, err %d\n", __func__, ret);
		return -EIO;
	}

	printf("TFP410: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		buff[0], buff[1], buff[2], buff[3], buff[4]);

	buff[0] = 0x35;
	buff[1] = 0x3d;
	buff[2] = 0x80;

	ret = dm_i2c_write(i2c_dev, 0x08, buff, 3);
	if (ret) {
		printf("%s failed to write to 0x08-0x0a, err %d\n", __func__, ret);
		return -EIO;
	}

	buff[0] = 0x16;
	buff[1] = 0x40;
	buff[2] = 0x50;
	buff[3] = 0x00;
	buff[4] = 0x80;
	buff[5] = 0x02;
	buff[6] = 0xe0;
	buff[7] = 0x01;
//        ret = dm_i2c_write(i2c_dev, 0x32, buff, 8);
//        if (ret) {
//                printf("%s failed to write to 0x32-0x39, err %d\n", __func__, ret);
//                return -EIO;
//        }

	printf("TFP410: DE generator disabled\n");
	printf("Successfully initialized TFP410!\n");

	return 0;
}

