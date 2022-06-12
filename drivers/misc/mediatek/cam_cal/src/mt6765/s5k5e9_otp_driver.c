// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cam_cal_list.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

extern u8 s5k5e9_2a_data[25];
extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
	u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static struct i2c_client *g_pstI2CclientG;

static u16 read_cmos_sensor(u32 addr)
{
	u16 get_byte = 0;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, g_pstI2CclientG->addr << 1);

	return get_byte;
}

static void s5k5e9_read_sensor_id(u32 addr, u8 *data, u16 i2c_id, u32 size)
{
	u32 *sensor_id = (u32 *)data;

	if (size == 2) {
		*sensor_id = ((read_cmos_sensor(addr) << 8) | read_cmos_sensor(addr+1));
	}
	pr_debug("G-Dg otp sensor id = 0x%04X\n", *sensor_id);
}

static int s5k5e9_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size)
{
	int ret = 0;

	if (addr == 0x0000) { // check sensor id
		s5k5e9_read_sensor_id(addr, data, i2c_id, size);
	} else if (addr == 0x0a04) {
		pr_debug("G-Dg s5k5e9 2a data\n");
		memcpy(data, &s5k5e9_2a_data[0], sizeof(s5k5e9_2a_data));
	}

	return ret;
}

unsigned int S5K5E9_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	g_pstI2CclientG = client;
	if (s5k5e9_read_region(addr, data, g_pstI2CclientG->addr, size) == 0)
		return size;
	else
		return 0;
}
