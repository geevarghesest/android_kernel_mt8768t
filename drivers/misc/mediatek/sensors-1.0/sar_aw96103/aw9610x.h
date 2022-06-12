#ifndef _AW9610X_H_
#define _AW9610X_H_

#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/leds.h>

#define AW9610X_CHIP_ID		0xa961
#define AWINIC_CFG_UPDATE_DELAY	1
#define AW_SAR_SUCCESS		0
#define AW_SAR_CAHNNEL_MAX	6
//#define ABS_DIS    1
/**********************************************
* cfg load situation
**********************************************/
enum aw9610x_cfg_situ {
	AW_CFG_UNLOAD = 0,
	AW_CFG_LOADED = 1,
};

/**********************************************
* cali mode
**********************************************/
enum aw9610x_cali_mode {
	AW_CALI_NORM_MODE = 0,
	AW_CALI_NODE_MODE = 1,
};

/**********************************************
*spereg cali flag
**********************************************/
enum aw9610x_cali_flag {
	AW_NO_CALI = 0,
	AW_CALI = 1,
};

/**********************************************
*spereg addr offset
**********************************************/
enum aw9610x_spereg_addr_offset {
	AW_CL1SPE_CALI_OS = 20,
	AW_CL1SPE_DEAL_OS = 60,
	AW_CL2SPE_CALI_OS = 4,
	AW_CL2SPE_DEAL_OS = 4,
};

/**********************************************
*the flag of i2c read/write
**********************************************/
enum aw9610x_i2c_flags {
	AW9610X_I2C_WR = 0,
	AW9610X_I2C_RD = 1,
};

/*********************************************************
* aw9610x error flag:
* @AW_MALLOC_FAILED: malloc space failed.
* @AW_CHIPID_FAILED: the chipid is error.
* @AW_IRQIO_FAILED: irq gpio invalid.
* @AW_IRQ_REQUEST_FAILED: irq requst failed.
* @AW_CFG_LOAD_TIME_FAILED : cfg situation not confirmed.
**********************************************************/
enum aw9610x_err_flags {
	AW_MALLOC_FAILED = 200,
	AW_CHIPID_FAILED = 201,
	AW_IRQIO_FAILED = 202,
	AW_IRQ_REQUEST_FAILED = 203,
	AW_INPUT_ALLOCATE_FILED = 204,
	AW_INPUT_REGISTER_FAILED = 205,
	AW_CFG_LOAD_TIME_FAILED = 206,
};

enum aw9610x_ichannel {
	aw9610x_SS0 = 0,
	aw9610x_SS1 = 1,
	aw9610x_SS2 = 2,
	aw9610x_SS3 = 3,
	aw9610x_SS4 = 4,
	aw9610x_SS5 = 5,
};

struct aw_i2c_package {
	uint8_t addr_bytes;
	uint8_t data_bytes;
	uint8_t reg_num;
	uint8_t init_addr[4];
	uint8_t *p_reg_data;
};

struct aw9610x {
	uint8_t cali_flag;
	uint8_t node;
	const char *chip_name;

	int32_t irq_gpio;
	uint32_t irq_status;
	uint32_t hostirqen;
	uint32_t first_irq_flag;
	uint32_t spedata[8];
	uint32_t nvspe_data[8];
	bool pwprox_dete;
	bool firmware_flag;

	struct delayed_work cfg_work;
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;
	struct delayed_work dworker; /* work struct for worker function */
	struct aw_bin *aw_bin;
	struct aw_i2c_package aw_i2c_package;
};

struct aw9610x_cfg {
	int len;
	unsigned int data[];
};

uint32_t attr_buf[] = {
	8, 10,
	9, 100,
	10, 1000,
};

/******************************************************
* Register Detail
******************************************************/
uint32_t aw9610x_reg_default[] = {
	0x0000, 0x00003f3f,
	0x0004, 0x00000064,
	0x0008, 0x0017c11e,
	0x000c, 0x05000000,
	0x0010, 0x00093ffd,
	0x0014, 0x19240009,
	0x0018, 0xd81c0207,
	0x001c, 0xff000000,
	0x0020, 0x00241900,
	0x0024, 0x00093ff7,
	0x0028, 0x58020009,
	0x002c, 0xd81c0207,
	0x0030, 0xff000000,
	0x0034, 0x00025800,
	0x0038, 0x00093fdf,
	0x003c, 0x7d3b0009,
	0x0040, 0xd81c0207,
	0x0044, 0xff000000,
	0x0048, 0x003b7d00,
	0x004c, 0x00093f7f,
	0x0050, 0xe9310009,
	0x0054, 0xd81c0207,
	0x0058, 0xff000000,
	0x005c, 0x0031e900,
	0x0060, 0x00093dff,
	0x0064, 0x1a0c0009,
	0x0068, 0xd81c0207,
	0x006c, 0xff000000,
	0x0070, 0x000c1a00,
	0x0074, 0x80093fff,
	0x0078, 0x043d0009,
	0x007c, 0xd81c0207,
	0x0080, 0xff000000,
	0x0084, 0x003d0400,
	0x0408, 0x00060000,
	0x040c, 0x00000000,
	0x00a0, 0xe6400000,
	0x00a4, 0x00000000,
	0x00a8, 0x010408d2,
	0x00ac, 0x00000000,
	0x00b0, 0x00000000,
	0x00b8, 0x00005fff,
	0x00bc, 0x00000000,
	0x00c0, 0x00000000,
	0x00c4, 0x00000000,
	0x00c8, 0x00000000,
	0x00cc, 0x00000000,
	0x00d0, 0x00000000,
	0x00d4, 0x00000000,
	0x00d8, 0x00000000,
	0x00dc, 0xe6447800,
	0x00e0, 0x78000000,
	0x00e4, 0x010408d2,
	0x00e8, 0x00000000,
	0x00ec, 0x00000000,
	0x00f4, 0x00005fff,
	0x00f8, 0x00000000,
	0x00fc, 0x00000000,
	0x0100, 0x00000000,
	0x0104, 0x00000000,
	0x0108, 0x00000000,
	0x010c, 0x02000000,
	0x0110, 0x00000000,
	0x0114, 0x00000000,
	0x0118, 0xe6447800,
	0x011c, 0x78000000,
	0x0120, 0x010408d2,
	0x0124, 0x00000000,
	0x0128, 0x00000000,
	0x0130, 0x00005fff,
	0x0134, 0x00000000,
	0x0138, 0x00000000,
	0x013c, 0x00000000,
	0x0140, 0x00000000,
	0x0144, 0x00000000,
	0x0148, 0x02000000,
	0x014c, 0x00000000,
	0x0150, 0x00000000,
	0x0154, 0xe6447800,
	0x0158, 0x78000000,
	0x015c, 0x010408d2,
	0x0160, 0x00000000,
	0x0164, 0x00000000,
	0x016c, 0x00005fff,
	0x0170, 0x00000000,
	0x0174, 0x00000000,
	0x0178, 0x00000000,
	0x017c, 0x00000000,
	0x0180, 0x00000000,
	0x0184, 0x02000000,
	0x0188, 0x00000000,
	0x018c, 0x00000000,
	0x0190, 0xe6447800,
	0x0194, 0x78000000,
	0x0198, 0x010408d2,
	0x019c, 0x00000000,
	0x01a0, 0x00000000,
	0x01a8, 0x00005fff,
	0x01ac, 0x00000000,
	0x01b0, 0x00000000,
	0x01b4, 0x00000000,
	0x01b8, 0x00000000,
	0x01bc, 0x00000000,
	0x01c0, 0x02000000,
	0x01c4, 0x00000000,
	0x01c8, 0x00000000,
	0x01cc, 0xe6407800,
	0x01d0, 0x78000000,
	0x01d4, 0x010408d2,
	0x01d8, 0x00000000,
	0x01dc, 0x00000000,
	0x01e4, 0x00005fff,
	0x01e8, 0x00000000,
	0x01ec, 0x00000000,
	0x01f0, 0x00000000,
	0x01f4, 0x00000000,
	0x01f8, 0x00000000,
	0x01fc, 0x02000000,
	0x0200, 0x00000000,
	0x0204, 0x00000000,
	0x0208, 0x00000008,
	0x020c, 0x0000000d,
	0x41fc, 0x00000000,
	0x4400, 0x00000000,
	0x4410, 0x00000000,
	0x4420, 0x00000000,
	0x4430, 0x00000000,
	0x4440, 0x00000000,
	0x4450, 0x00000000,
	0x4460, 0x00000000,
	0x4470, 0x00000000,
	0xf080, 0x00003018,
	0xf084, 0x00000fff,
	0xf800, 0x00000000,
	0xf804, 0x00002e00,
	0xf8d0, 0x00000001,
	0xf8d4, 0x00000000,
	0xff00, 0x00000301,
	0xff0c, 0x01000000,
	0xffe0, 0x00000000,
	0xfff4, 0x00004011,
	0x0090, 0x00000000,
	0x0094, 0x00000000,
	0x0098, 0x00000000,
	0x009c, 0x3f3f3f3f,
};
#endif
