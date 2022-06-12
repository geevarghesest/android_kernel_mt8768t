/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#ifndef __EEPROM_I2C_CUSTOM_DRIVER_H
#define __EEPROM_I2C_CUSTOM_DRIVER_H
#include <linux/i2c.h>

/************************************************************
 * I2C read function (Custom)
 * Customer's driver can put on here
 * Below is an example
 ************************************************************/
unsigned int Custom_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

unsigned int GC5035_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

unsigned int S5K5E9_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

unsigned int OV8856_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

unsigned int S5K4H7_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

#endif				/* __CAM_CAL_LIST_H */
