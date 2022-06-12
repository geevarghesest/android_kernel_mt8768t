// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "kd_imgsensor.h"
#include "imgsensor_sensor_list.h"

/* Add Sensor Init function here
 * Note:
 * 1. Add by the resolution from ""large to small"", due to large sensor
 *    will be possible to be main sensor.
 *    This can avoid I2C error during searching sensor.
 * 2. This should be the same as
 *     mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp
 */
struct IMGSENSOR_INIT_FUNC_LIST kdSensorList[MAX_NUM_OF_SUPPORT_SENSOR] = {
/* P502 */
#if defined(GC5035_MIPI_RAW)
	{GC5035_SENSOR_ID,
	SENSOR_DRVNAME_GC5035_MIPI_RAW,
	GC5035_MIPI_RAW_SensorInit},
#endif
#if defined(S5K5E9_MIPI_RAW)
	{S5K5E9_SENSOR_ID,
	SENSOR_DRVNAME_S5K5E9_MIPI_RAW,
	S5K5E9_MIPI_RAW_SensorInit},
#endif
#if defined(OV8856_MIPI_RAW)
	{OV8856_SENSOR_ID,
	SENSOR_DRVNAME_OV8856_MIPI_RAW,
	OV8856_MIPI_RAW_SensorInit},
#endif
#if defined(S5K4H7_MIPI_RAW)
	{S5K4H7_SENSOR_ID,
	SENSOR_DRVNAME_S5K4H7_MIPI_RAW,
	S5K4H7_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1_MIPI_RAW)
	{GC02M1_SENSOR_ID,
	SENSOR_DRVNAME_GC02M1_MIPI_RAW,
	GC02M1_MIPI_RAW_SensorInit},
#endif
#if defined(OV02B_MIPI_RAW)
	{OV02B_SENSOR_ID,
	SENSOR_DRVNAME_OV02B_MIPI_RAW,
	OV02B_MIPI_RAW_SensorInit},
#endif

	/*  ADD sensor driver before this line */
	{0, {0}, NULL}, /* end of list */
};
/* e_add new sensor driver here */

