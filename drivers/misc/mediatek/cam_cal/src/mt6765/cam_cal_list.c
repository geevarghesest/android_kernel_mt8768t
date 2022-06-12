// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/* P502 OTP */
	{GC5035_SENSOR_ID, 0x6e, GC5035_read_region},
	{S5K5E9_SENSOR_ID, 0x20, S5K5E9_read_region},
	{OV8856_SENSOR_ID, 0x6c, OV8856_read_region},
	{S5K4H7_SENSOR_ID, 0x20, S5K4H7_read_region},

	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


