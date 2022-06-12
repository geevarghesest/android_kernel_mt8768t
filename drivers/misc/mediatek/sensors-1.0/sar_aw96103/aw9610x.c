/*
* aw9610x.c
*
* Version: v0.1.3
*
* Copyright (c) 2020 AWINIC Technology CO., LTD
*
* Author: Alex <zhangpengbiao@awinic.com>
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/string.h>
//#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include "aw_bin_parse.h"
#include "aw9610x.h"
#include "aw9610x_reg.h"

#define AW9610X_I2C_NAME "aw9610x_sar"
#define AW9610X_DRIVER_VERSION "v0.1.3"

#define AW_READ_CHIPID_RETRIES 5
#define AW_I2C_RETRIES 5
#define AW9610X_SCAN_DEFAULT_TIME 10000
#define CALI_FILE_MAX_SIZE 128
#define AWINIC_CALI_FILE "/mnt/aw_cali.bin"
static char *aw9610x_cfg_name = "aw9610x.bin";
static int cal_flag = 0;//add for calibration
static int onoff_value = 0;//add for onoff
struct aw9610x *aw9610x;
int saraw_flag=0;
extern unsigned int board_id_get_Network(void);
static uint8_t last_ch[6] = { 0 };
/******************************************************
*
* aw9610x i2c write/read
*
******************************************************/

static int32_t
i2c_write(struct aw9610x *aw9610x, uint16_t reg_addr16, uint32_t reg_data32)
{	int32_t ret =  -ENOMEM;
	struct i2c_client *i2c = aw9610x->i2c;
	struct i2c_msg msg;
	uint8_t w_buf[6];

	/*reg_addr*/
	w_buf[0] = (u8)(reg_addr16>>8);
	w_buf[1] = (u8)(reg_addr16);
	/*data*/
	w_buf[2] = (u8)(reg_data32 >> 24);
	w_buf[3] = (u8)(reg_data32 >> 16);
	w_buf[4] = (u8)(reg_data32 >> 8);
	w_buf[5] = (u8)(reg_data32);

	msg.addr = i2c->addr;
	msg.flags = AW9610X_I2C_WR;
	msg.len = 6;
	/*2 bytes regaddr + 4 bytes data*/
	msg.buf = (unsigned char *)w_buf;

	ret = i2c_transfer(i2c->adapter, &msg, 1);
	if (ret < 0)
		pr_info("%s: i2c write reg 0x%x error %d\n", __func__,
							reg_addr16, ret);

	return ret;
}

static int32_t
i2c_read(struct aw9610x *aw9610x, uint16_t reg_addr16, uint32_t *reg_data32)
{
	int32_t ret =  -ENOMEM;
	struct i2c_client *i2c = aw9610x->i2c;
	struct i2c_msg msg[2];
	uint8_t w_buf[2];
	uint8_t buf[4];

	w_buf[0] = (unsigned char)(reg_addr16 >> 8);
	w_buf[1] = (unsigned char)(reg_addr16);
	msg[0].addr = i2c->addr;
	msg[0].flags = AW9610X_I2C_WR;
	msg[0].len = 2;
	msg[0].buf = (unsigned char *)w_buf;

	msg[1].addr = i2c->addr;
	msg[1].flags = AW9610X_I2C_RD;
	msg[1].len = 4;
	msg[1].buf = (unsigned char *)buf;

	ret = i2c_transfer(i2c->adapter, msg, 2);
	if (ret < 0)
		pr_info("%s: i2c read reg 0x%x error %d\n", __func__,
							reg_addr16, ret);

	reg_data32[0] = ((u32)buf[3]) | ((u32)buf[2]<<8) |
			((u32)buf[1]<<16) | ((u32)buf[0]<<24);

	return ret;
}

static int32_t aw9610x_i2c_write(struct aw9610x *aw9610x,
				uint16_t reg_addr16, uint32_t reg_data32)
{
	int32_t ret = -1;
	uint8_t cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_write(aw9610x, reg_addr16, reg_data32);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n",
							__func__, cnt, ret);
		} else {
			break;
		}
		cnt++;
	}

	return ret;
}

static int32_t aw9610x_i2c_read(struct aw9610x *aw9610x,
				uint16_t reg_addr16, uint32_t *reg_data32)
{
	int32_t ret = -1;
	uint8_t cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_read(aw9610x, reg_addr16, reg_data32);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
	}
	return ret;
}

static int32_t aw9610x_i2c_write_bits(struct aw9610x *aw9610x,
				 uint16_t reg_addr16, uint32_t mask,
				 uint32_t reg_data32)
{
	uint32_t reg_val;

	aw9610x_i2c_read(aw9610x, reg_addr16, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data32;
	aw9610x_i2c_write(aw9610x, reg_addr16, reg_val);

	return 0;
}

/******************************************************************************
*
* aw9610x i2c sequential write/read --- one first addr with multiple data.
*
******************************************************************************/
static int32_t i2c_write_seq(struct aw9610x *aw9610x)
{
	int32_t ret =  -ENOMEM;
	struct i2c_client *i2c = aw9610x->i2c;
	struct i2c_msg msg;
	uint8_t w_buf[228];
	uint8_t addr_bytes = aw9610x->aw_i2c_package.addr_bytes;
	uint8_t msg_cnt = 0;
	uint8_t data_bytes = aw9610x->aw_i2c_package.data_bytes;
	uint8_t reg_num = aw9610x->aw_i2c_package.reg_num;
	uint8_t *p_reg_data = aw9610x->aw_i2c_package.p_reg_data;
	uint8_t msg_idx = 0;

	for (msg_idx = 0; msg_idx < addr_bytes; msg_idx++) {
		w_buf[msg_idx] = aw9610x->aw_i2c_package.init_addr[msg_idx];
		pr_info("%s: w_buf_addr[%d] = 0x%02x\n",
					__func__, msg_idx, w_buf[msg_idx]);
	}
	msg_cnt = addr_bytes;
	for (msg_idx = 0; msg_idx < data_bytes * reg_num; msg_idx++) {
		w_buf[msg_cnt] = *p_reg_data++;
		pr_info("%s: w_buf_addr[%d] = 0x%02x\n",
					__func__, msg_cnt, w_buf[msg_cnt]);
		msg_cnt++;
	}
	pr_info("%s: %d\n", __func__, msg_cnt);
	p_reg_data = aw9610x->aw_i2c_package.p_reg_data;
	msg.addr = i2c->addr;
	msg.flags = AW9610X_I2C_WR;
	msg.len = msg_cnt;
	msg.buf = (uint8_t *)w_buf;
	ret = i2c_transfer(i2c->adapter, &msg, 1);

	if (ret < 0)
		pr_info("%s: i2c write seq error %d\n", __func__, ret);

	return ret;
}

static int32_t i2c_read_seq(struct aw9610x *aw9610x, uint8_t *reg_data)
{
	int32_t ret =  -ENOMEM;
	struct i2c_client *i2c = aw9610x->i2c;
	struct i2c_msg msg[2];
	uint8_t w_buf[4];
	uint8_t buf[228];
	uint8_t data_bytes = aw9610x->aw_i2c_package.data_bytes;
	uint8_t reg_num = aw9610x->aw_i2c_package.reg_num;
	uint8_t addr_bytes = aw9610x->aw_i2c_package.addr_bytes;
	uint8_t msg_idx = 0;
	uint8_t msg_cnt = 0;

	/*
	* step 1 : according to addr_bytes assemble first_addr.
	* step 2 : initialize msg[0] including first_addr transfer to client.
	* step 3 : wait for client return reg_data.
	*/
	for (msg_idx = 0; msg_idx < addr_bytes; msg_idx++) {
		w_buf[msg_idx] = aw9610x->aw_i2c_package.init_addr[msg_idx];
		pr_info("%s: w_buf_addr[%d] = 0x%02x\n",
					__func__, msg_idx, w_buf[msg_idx]);
	}
	msg[0].addr = i2c->addr;
	msg[0].flags = AW9610X_I2C_WR;
	msg[0].len = msg_idx;
	msg[0].buf = (uint8_t *)w_buf;

	/*
	* recieve client to msg[1].buf.
	*/
	msg_cnt = data_bytes * reg_num;
	msg[1].addr = i2c->addr;
	msg[1].flags = AW9610X_I2C_RD;
	msg[1].len = msg_cnt;
	msg[1].buf = (uint8_t *)buf;

	ret = i2c_transfer(i2c->adapter, msg, 2);
	for (msg_idx = 0; msg_idx < msg_cnt; msg_idx++) {
		reg_data[msg_idx] = buf[msg_idx];
		pr_info("%s: buf = 0x%02x\n", __func__, buf[msg_idx]);
	}

	if (ret < 0)
		pr_info("%s: i2c write error %d\n", __func__, ret);

	return ret;
}

static void
aw9610x_addrblock_load(struct device *dev, const char *buf)
{
	uint32_t addrbuf[4] = { 0 };
	uint8_t temp_buf[2] = { 0 };
	uint32_t i = 0;
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	uint8_t addr_bytes = aw9610x->aw_i2c_package.addr_bytes;
	uint8_t reg_num = aw9610x->aw_i2c_package.reg_num;

	for (i = 0; i < addr_bytes; i++) {
		if (reg_num < attr_buf[1]) {
			temp_buf[0] = buf[attr_buf[0] + i * 5];
			temp_buf[1] = buf[attr_buf[0] + i * 5 + 1];
		} else if (reg_num >= attr_buf[1] && reg_num < attr_buf[3]) {
			temp_buf[0] = buf[attr_buf[2] + i * 5];
			temp_buf[1] = buf[attr_buf[2] + i * 5 + 1];
		} else if (reg_num >= attr_buf[3] && reg_num < attr_buf[5]) {
			temp_buf[0] = buf[attr_buf[4] + i * 5];
			temp_buf[1] = buf[attr_buf[4] + i * 5 + 1];
		}
		if (sscanf(temp_buf, "%02x", &addrbuf[i]) == 1)
			aw9610x->aw_i2c_package.init_addr[i] =
							(uint8_t)addrbuf[i];
	}
}

/******************************************************
 *
 *the document of storage_spedata
 *
 ******************************************************/
static int32_t aw9610x_filedata_deal(struct aw9610x *aw9610x)
{
	struct file *fp = NULL;
	mm_segment_t fs;
	int8_t *buf;
	int8_t temp_buf[8] = { 0 };
	uint8_t i = 0;
	uint8_t j = 0;
	int32_t ret;
	uint32_t nv_flag = 0;

	pr_info("%s: enter, cali_node = %d\n", __func__, aw9610x->node);

	fp = filp_open(AWINIC_CALI_FILE, O_RDWR | O_CREAT, 0644);

	if (IS_ERR(fp)) {
		pr_err("%s: open failed!\n", __func__);
		return -EINVAL;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	buf = (char *)kzalloc(CALI_FILE_MAX_SIZE, GFP_KERNEL);
	if (!buf) {
		pr_err("%s: malloc failed!\n", __func__);
		filp_close(fp, NULL);
		set_fs(fs);
		return -EINVAL;
	}
	ret = vfs_read(fp, buf, CALI_FILE_MAX_SIZE, &(fp->f_pos));
	if (ret < 0) {
		pr_info("%s: read failed\n", __func__);
		return ret;
	} else if (ret == 0) {
		aw9610x->nvspe_data[i] = 0;
	} else {
		if (aw9610x->node == AW_CALI_NORM_MODE) {
			return 0;
		} else {
			for (i = 0; i < 8; i++) {
				for (j = 0; j < 8; j++)
					temp_buf[j] = buf[8 * i + j];

				if (sscanf(temp_buf, "%08x",
						&aw9610x->nvspe_data[i]) == 1)
					pr_info("%s: nv_spe_data[%d] = 0x%08x\n",
						__func__, i, aw9610x->nvspe_data[i]);
			}
		}
	}

	set_fs(fs);

	filp_close(fp, NULL);
	kfree(buf);
	/* nvspe_datas come from nv*/

	for (i = 0; i < 8; i++) {
		nv_flag |= aw9610x->nvspe_data[i];
		if (nv_flag != 0)
			break;
	}

	if (aw9610x->node == AW_CALI_NORM_MODE) {
		if (nv_flag == 0) {
			aw9610x->cali_flag = AW_CALI;
			pr_info("%s: the chip need to cali! nv_flag = 0x%08x\n",
							__func__, nv_flag);
		} else {
			aw9610x->cali_flag = AW_NO_CALI;
			pr_info("%s: chip not need to cali! nv_flag = 0x%08x\n",
							__func__, nv_flag);
		}
	}

	return 0;
}

static int32_t aw9610x_store_spedata_to_file(struct aw9610x *aw9610x, char *buf)
{
	struct file *fp = NULL;
	loff_t pos = 0;
	mm_segment_t fs;

	pr_info("%s: buf = %s\n", __func__, buf);

	fp = filp_open(AWINIC_CALI_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("%s: open failed!\n", __func__);
		return -EINVAL;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, buf, strlen(buf), &pos);

	set_fs(fs);

	pr_info("%s: write successfully!\n", __func__);

	filp_close(fp, NULL);
	return 0;
}

/******************************************************
 *
 *configuration of special reg
 *
 ******************************************************/
static void aw9610x_get_calidata(struct aw9610x *aw9610x)
{
	uint8_t i = 0;
	uint32_t buf_size = 0;
	int32_t ret;
	uint8_t temp_buf[9] = { 0 };
	uint8_t buf[CALI_FILE_MAX_SIZE] = { 0 };

	pr_info("%s enter\n", __func__);

	/*class 1 special reg*/
	for (i = 0; i < 6; i++) {
		aw9610x_i2c_read(aw9610x,
				REG_AFECFG1_CH0 + i * AW_CL1SPE_CALI_OS,
				&aw9610x->spedata[i]);
		pr_info("%s: specialdata[%d]=0x%08x\n",
				__func__, i, aw9610x->spedata[i]);
	}
	/*class 2 special reg*/
	for (; i < 8; i++) {
		aw9610x_i2c_read(aw9610x,
				REG_REFACFG + (i - 6) * AW_CL2SPE_CALI_OS,
				&aw9610x->spedata[i]);
		pr_info("%s: channel number = 0x%08x\n", __func__,
						aw9610x->spedata[i]);
	}
	/* spedatas come from register*/

	/* write spedatas to nv */
	for (i = 0; i < 8; i++) {
		snprintf(temp_buf, sizeof(temp_buf), "%08x",
							aw9610x->spedata[i]);
		memcpy(buf + buf_size, temp_buf, strlen(temp_buf));
		buf_size = strlen(buf);
	}
	ret = aw9610x_store_spedata_to_file(aw9610x, buf);
	if (ret < 0) {
		pr_info("%s: store spedata failed\n", __func__);
		return;
	}

	pr_info("%s: successfully write_spereg_to_file\n", __func__);
}

static void aw9610x_class1_reg(struct aw9610x *aw9610x)
{
	int32_t i = 0;
	uint32_t reg_val;

	pr_info("%s enter\n", __func__);

	for (i = 0; i < 6; i++) {
		reg_val = (aw9610x->nvspe_data[i] >> 16) & 0x0000ffff;
		aw9610x_i2c_write_bits(aw9610x,
				REG_INITPROX0_CH0 + i * AW_CL1SPE_DEAL_OS,
				~(0xffff), reg_val);
	}
}

static void aw9610x_class2_reg(struct aw9610x *aw9610x)
{
	int32_t i = 0;
	uint32_t reg_val = 0;
	uint32_t ret = 0;

	pr_info("%s enter\n", __func__);

	for (i = 6; i < 8; i++) {
		ret = aw9610x->nvspe_data[i] & 0x07;
		switch (ret) {
		case 0x00:
			aw9610x_i2c_read(aw9610x, REG_VALID_CH0,
							&reg_val);
			break;
		case 0x01:
			aw9610x_i2c_read(aw9610x, REG_VALID_CH1,
							&reg_val);
			break;
		case 0x02:
			aw9610x_i2c_read(aw9610x, REG_VALID_CH2,
							&reg_val);
			break;
		case 0x03:
			aw9610x_i2c_read(aw9610x, REG_VALID_CH3,
							&reg_val);
			break;
		case 0x04:
			aw9610x_i2c_read(aw9610x, REG_VALID_CH4,
							&reg_val);
			break;
		case 0x05:
			aw9610x_i2c_read(aw9610x, REG_VALID_CH5,
							&reg_val);
			break;
		default:
			return;
		}

		reg_val = ((reg_val >> 6) & 0x03fffff0) |
					(aw9610x->nvspe_data[i] & 0xfc00000f);
		aw9610x_i2c_write(aw9610x,
			REG_REFACFG + (i - 6) * AW_CL2SPE_DEAL_OS,
			reg_val);
	}
}

static void aw9610x_spereg_deal(struct aw9610x *aw9610x)
{
	pr_info("%s enter!\n", __func__);

	aw9610x_class1_reg(aw9610x);
	aw9610x_class2_reg(aw9610x);
}

static void aw9610x_datablock_load(struct device *dev, const char *buf)
{
	uint32_t i = 0;
	uint8_t reg_data[220] = { 0 };
	uint32_t databuf[220] = { 0 };
	uint8_t temp_buf[2] = { 0 };
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	uint8_t addr_bytes = aw9610x->aw_i2c_package.addr_bytes;
	uint8_t data_bytes = aw9610x->aw_i2c_package.data_bytes;
	uint8_t reg_num = aw9610x->aw_i2c_package.reg_num;

	for (i = 0; i < data_bytes * reg_num; i++) {
		if (reg_num < attr_buf[1]) {
			temp_buf[0] = buf[attr_buf[0] + (addr_bytes + i) * 5];
			temp_buf[1] =
				    buf[attr_buf[0] + (addr_bytes + i) * 5 + 1];
		} else if (reg_num >= attr_buf[1] && reg_num < attr_buf[3]) {
			temp_buf[0] = buf[attr_buf[2] + (addr_bytes + i) * 5];
			temp_buf[1] =
				    buf[attr_buf[2] + (addr_bytes + i) * 5 + 1];
		} else if (reg_num >= attr_buf[3] && reg_num < attr_buf[5]) {
			temp_buf[0] = buf[attr_buf[4] + (addr_bytes + i) * 5];
			temp_buf[1] =
				    buf[attr_buf[4] + (addr_bytes + i) * 5 + 1];
		}
		sscanf(temp_buf, "%02x", &databuf[i]);
		reg_data[i] = (uint8_t)databuf[i];
	}
	aw9610x->aw_i2c_package.p_reg_data = reg_data;
	i2c_write_seq(aw9610x);
}

static void aw9610x_channel_scan_start(struct aw9610x *aw9610x)
{
	uint32_t reg_data;
	int32_t ret;
	uint32_t temp_time = AW9610X_SCAN_DEFAULT_TIME;

	pr_info("%s: enter\n", __func__);

	if (aw9610x->pwprox_dete == true) {
		ret = aw9610x_filedata_deal(aw9610x);
		if ((aw9610x->cali_flag == AW_NO_CALI) && ret >= 0)
			aw9610x_spereg_deal(aw9610x);
	} else {
		aw9610x->cali_flag = AW_NO_CALI;
	}

	aw9610x_i2c_write(aw9610x, REG_HOSTIRQEN, 0);
	aw9610x_i2c_write(aw9610x, REG_CMD, 0x0001);
	while ((temp_time)--) {
		aw9610x_i2c_read(aw9610x, REG_HOSTIRQSRC, &reg_data);
		reg_data = (reg_data >> 4) & 0x01;
		if (reg_data == 1) {
			pr_info("%s: time = %d\n", __func__, temp_time);
			if ((aw9610x->cali_flag == AW_CALI) && ret >= 0)
				aw9610x_get_calidata(aw9610x);
			break;
			msleep(1);
		}
	}
	aw9610x_i2c_write(aw9610x, REG_HOSTIRQEN, aw9610x->hostirqen);
#ifdef CONFIG_KERNEL_CUSTOM_FACTORY
	aw9610x_i2c_write(aw9610x,0xF008,0x02);
	msleep(200);
	pr_info("aw9610x factory done \n");
#else
	pr_info("aw9610x done \n");
#endif
}

static void aw9610x_bin_valid_loaded(struct aw9610x *aw9610x,
					struct aw_bin *aw_bin_data_s)
{
	uint32_t i;
	uint16_t reg_addr;
	uint32_t reg_data;
	uint32_t start_addr = aw_bin_data_s->header_info[0].valid_data_addr;

	for (i = 0; i < aw_bin_data_s->header_info[0].valid_data_len;
						i += 6, start_addr += 6) {
		reg_addr = (aw_bin_data_s->info.data[start_addr]) |
				aw_bin_data_s->info.data[start_addr + 1] << 8;
		reg_data = aw_bin_data_s->info.data[start_addr + 2] |
			(aw_bin_data_s->info.data[start_addr + 3] << 8) |
			(aw_bin_data_s->info.data[start_addr + 4] << 16) |
			(aw_bin_data_s->info.data[start_addr + 5] << 24);
		aw9610x_i2c_write(aw9610x, reg_addr, reg_data);
		if (reg_addr == REG_HOSTIRQEN)
			aw9610x->hostirqen = reg_data;
		pr_info("%s :reg_addr = 0x%04x, reg_data = 0x%08x\n", __func__,
							reg_addr, reg_data);
	}
	pr_info("%s bin writen completely: \n", __func__);
	aw9610x_channel_scan_start(aw9610x);
}

/***************************************************************************
* para loaded
****************************************************************************/
static int32_t aw9610x_para_loaded(struct aw9610x *aw9610x)
{
	int32_t i = 0;
	int32_t len =
		sizeof(aw9610x_reg_default)/sizeof(aw9610x_reg_default[0]);

	pr_info("%s: start to download para!\n", __func__);
	for (i = 0; i < len; i = i + 2) {
		aw9610x_i2c_write(aw9610x,
				(uint16_t)aw9610x_reg_default[i],
				aw9610x_reg_default[i+1]);
		if (aw9610x_reg_default[i] == REG_HOSTIRQEN)
			aw9610x->hostirqen = aw9610x_reg_default[i+1];
		pr_info("%s: reg_addr = 0x%04x, reg_data = 0x%08x\n",
						__func__,
						aw9610x_reg_default[i],
						aw9610x_reg_default[i+1]);
	}
	pr_info("%s para writen completely: \n", __func__);

	aw9610x_channel_scan_start(aw9610x);

	return 0;
}

static void aw9610x_cfg_all_loaded(const struct firmware *cont, void *context)
{
	int32_t ret;
	struct aw_bin *aw_bin;
	struct aw9610x *aw9610x = context;

	pr_info("%s enter\n", __func__);

	if (!cont) {
		pr_info("%s: %s download failed\n", __func__, aw9610x_cfg_name);
		release_firmware(cont);
		return;
	} else {
		pr_info("%s download successfully\n", aw9610x_cfg_name);
	}

	aw_bin = kzalloc(cont->size + sizeof(struct aw_bin), GFP_KERNEL);
	if (!aw_bin) {
		kfree(aw_bin);
		release_firmware(cont);
		pr_err("%s: failed to allcating memory!\n", __func__);
		return;
	}
	aw_bin->info.len = cont->size;
	memcpy(aw_bin->info.data, cont->data, cont->size);
	ret = aw_parsing_bin_file(aw_bin);
	if (ret < 0) {
		pr_info("%s:aw9610x parse bin fail! ret = %d\n", __func__, ret);
		kfree(aw_bin);
		release_firmware(cont);
		return;
	}

	ret = strcmp(aw9610x->chip_name, aw_bin->header_info[0].chip_type);
	if (ret != 0) {
		pr_info("%s:chip name(%s) incompatible with bin chip(%s)\n",
					__func__, aw9610x->chip_name,
					aw_bin->header_info[0].chip_type);
		kfree(aw_bin);
		release_firmware(cont);
		return;
	}
	aw9610x_bin_valid_loaded(aw9610x, aw_bin);
	kfree(aw_bin);
	release_firmware(cont);
}

static int32_t aw9610x_cfg_update(struct aw9610x *aw9610x)
{
	pr_info("%s: enter\n", __func__);

	if (aw9610x->firmware_flag == true)
		return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
							aw9610x_cfg_name,
							aw9610x->dev,
							GFP_KERNEL,
							aw9610x,
							aw9610x_cfg_all_loaded);
	else
		aw9610x_para_loaded(aw9610x);

	return AW_SAR_SUCCESS;
}

static void aw9610x_cfg_work_routine(struct work_struct *work)
{
	struct aw9610x
		*aw9610x = container_of(work, struct aw9610x, cfg_work.work);

	pr_info("%s: enter\n", __func__);

	aw9610x_cfg_update(aw9610x);
}

static int32_t aw9610x_sar_cfg_init(struct aw9610x *aw9610x, int32_t flag)
{
	uint32_t cfg_timer_val = 0;

	pr_info("%s: enter\n", __func__);

	if (flag == AW_CFG_LOADED) {
		cfg_timer_val = 20;
		aw9610x->node = AW_CALI_NODE_MODE;
	} else if (flag == AW_CFG_UNLOAD) {
		cfg_timer_val = 5000;
		aw9610x->node = AW_CALI_NORM_MODE;
	} else {
		return -AW_CFG_LOAD_TIME_FAILED;
	}

	pr_info("%s: cali_node = %d\n", __func__, aw9610x->node);

	INIT_DELAYED_WORK(&aw9610x->cfg_work, aw9610x_cfg_work_routine);
	schedule_delayed_work(&aw9610x->cfg_work,
					      msecs_to_jiffies(cfg_timer_val));

	return AW_SAR_SUCCESS;
}

/*****************************************************
 *
 * first irq clear
 *
 *****************************************************/
static int32_t aw9610x_init_irq_handle(struct aw9610x *aw9610x)
{
	uint8_t cnt = 20;
	uint32_t reg_data;

	pr_info("%s enter\n", __func__);
	while (cnt--) {
		aw9610x_i2c_read(aw9610x, REG_HOSTIRQSRC, &reg_data);
		aw9610x->first_irq_flag = reg_data & 0x01;
		if (aw9610x->first_irq_flag == 1) {
			pr_info("%s: cnt = %d\n", __func__, cnt);
			return AW_SAR_SUCCESS;
		}
		msleep(1);
	}
	pr_err("%s: hardware has trouble!\n", __func__);

	return -AW_IRQIO_FAILED;
}

/*****************************************************
 *
 * software reset
 *
 *****************************************************/
static void aw9610x_sw_reset(struct aw9610x *aw9610x)
{
	pr_info("%s: enter\n", __func__);

	aw9610x_i2c_write(aw9610x, REG_HOSTCTRL2, 0);
}

/******************************************************
 *
 * sys group attribute
 *
 ******************************************************/
static ssize_t aw9610x_set_reg_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw9610x_i2c_write(aw9610x,
				(uint16_t)databuf[0],
				(uint32_t)databuf[1]);

	return count;
}

static ssize_t aw9610x_get_reg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint32_t i = 0;
	uint32_t reg_val = 0;
	uint32_t reg_num = 0;

	reg_num = ARRAY_SIZE(aw9610x_reg_access);
	for (i = 0; i < reg_num; i++)
		if (aw9610x_reg_access[i].rw & REG_RD_ACCESS) {
			aw9610x_i2c_read(aw9610x,
					aw9610x_reg_access[i].reg,
					&reg_val);
			len += snprintf(buf + len, PAGE_SIZE - len,
						"reg:0x%04x=0x%08x\n",
						aw9610x_reg_access[i].reg,
						reg_val);
		}

	return len;
}

static ssize_t aw9610x_valid_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	uint32_t reg_val = 0;

	for (i = 0; i < AW_SAR_CAHNNEL_MAX; i++) {
		aw9610x_i2c_read(aw9610x, REG_VALID_CH0 + i * 4, &reg_val);
		reg_val /= 1024;
		len += snprintf(buf+len, PAGE_SIZE-len,
					"VALID_CH%d = %d\n", i, reg_val);
	}

	return len;
}

static ssize_t aw9610x_baseline_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	uint32_t reg_val = 0;

	for (i = 0; i < AW_SAR_CAHNNEL_MAX; i++) {
		aw9610x_i2c_read(aw9610x, REG_BASELINE_CH0 + i * 4, &reg_val);
		reg_val /= 1024;
		len += snprintf(buf+len, PAGE_SIZE-len,
					"BASELINE_CH%d = %d\n", i, reg_val);
	}

	return len;
}

static ssize_t aw9610x_diff_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	uint32_t reg_val = 0;

	for (i = 0; i < AW_SAR_CAHNNEL_MAX; i++) {
		aw9610x_i2c_read(aw9610x, REG_DIFF_CH0 + i * 4, &reg_val);
		reg_val /= 1024;
		len += snprintf(buf+len, PAGE_SIZE-len,
					"DIFF_CH%d = %d\n", i, reg_val);
	}

	return len;
}

static ssize_t aw9610x_raw_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	uint32_t reg_val = 0;

	for (i = 0; i < AW_SAR_CAHNNEL_MAX; i++) {
		aw9610x_i2c_read(aw9610x, REG_RAW_CH0 + i * 4, &reg_val);
		reg_val /= 1024;
		len += snprintf(buf+len, PAGE_SIZE-len,
					"RAW_DATA_CH%d = %d\n", i, reg_val);
	}

	return len;
}
static ssize_t sar_show_proxstatus_cs1(struct device *dev,
								struct device_attribute *attr, char *buf)
{   
	uint8_t status;
	uint32_t reg_value = 0;
	int state =0;
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	pr_info("Reading SAR state\n");
	
	aw9610x_i2c_read(aw9610x,0x0090,&reg_value);
	pr_info("%s REG_STAT0 = 0x%08x\n", __func__, reg_value);
	status = (uint8_t)((reg_value&0x3f000000) >> 24);
	if (((status>>1) & 0x01)==1)
	{
		state=1;//near
		pr_info("[CH1]Reading SAR state is:%d\n",state);
	}
	else
	{
		state=2;//realse
	}
	return  sprintf(buf, "%d\n", state);
}
static ssize_t sar_show_proxstatus_cs2(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	uint8_t status;
	uint32_t reg_value = 0;
	int state =0;
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	pr_info("Reading SAR state\n");
	
	aw9610x_i2c_read(aw9610x,0x0090,&reg_value);
	pr_info("%s REG_STAT0 = 0x%08x\n", __func__, reg_value);
	status = (uint8_t)((reg_value&0x3f000000) >> 24);
	if (((status>>2) & 0x01)==1)
	{
		state=1;//near
		pr_info("[CH2]Reading SAR state is:%d\n",state);
	}
	else
	{
		state=2;//realse
	}
	return  sprintf(buf, "%d\n", state);
}


static ssize_t sar_info_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{       
	uint32_t reg_value = 0;
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	pr_info("Reading SAR ID aw FOR ATA TEST\n");
	pr_info("aw9610 cfg name is %s\n", aw9610x_cfg_name);
	aw9610x_i2c_read(aw9610x,0X0088,&reg_value);
	return sprintf(buf, "0x%x\n", reg_value);
}

static ssize_t aw9610x_awrw_get(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	uint8_t reg_data[228] = { 0 };
	uint8_t i = 0;
	ssize_t len = 0;
	uint8_t reg_num = aw9610x->aw_i2c_package.reg_num;
	uint8_t data_bytes = aw9610x->aw_i2c_package.data_bytes;

	for (i = 0; i < reg_num * data_bytes - 1; i++) {
		i2c_read_seq(aw9610x, reg_data);
		len += snprintf(buf + len, PAGE_SIZE - len,
					"0x%02x,", reg_data[i]);
	}
	if (!i)
		i2c_read_seq(aw9610x, reg_data);
	len += snprintf(buf + len, PAGE_SIZE - len,
					"0x%02x\n", reg_data[i]);

	return len;
}

static ssize_t aw9610x_cali_set(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	uint32_t databuf[1] = { 0 };

	if (sscanf(buf, "%d", &databuf[0]) == 1) {
		if ((databuf[0] == 1) && (aw9610x->pwprox_dete == true)) {
			aw9610x_sw_reset(aw9610x);
			aw9610x->cali_flag = AW_CALI;
		} else {
			pr_info("%s:aw_unsupport the pw_prox_dete=%d\n",
						__func__, aw9610x->pwprox_dete);
			return count;
		}
		aw9610x_sar_cfg_init(aw9610x, AW_CFG_LOADED);
	}

	return count;
}

static ssize_t aw9610x_awrw_set(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	uint32_t datatype[3] = { 0 };

	if (sscanf(buf, "%d %d %d", &datatype[0], &datatype[1],
							&datatype[2]) == 3) {
		aw9610x->aw_i2c_package.addr_bytes = (uint8_t)datatype[0];
		aw9610x->aw_i2c_package.data_bytes = (uint8_t)datatype[1];
		aw9610x->aw_i2c_package.reg_num = (uint8_t)datatype[2];

		aw9610x_addrblock_load(dev, buf);
		if (count > 7 + 5 * aw9610x->aw_i2c_package.addr_bytes)
			aw9610x_datablock_load(dev, buf);
	}

	return count;
}

static ssize_t aw9610x_set_update(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	ssize_t ret;
	uint32_t state;
	int32_t cfg_timer_val = 10;
	struct aw9610x *aw9610x = dev_get_drvdata(dev);
	cal_flag=0;
	ret = kstrtouint(buf, 10, &state);
	if (ret) {
		pr_err("%s: fail to change str to int\n", __func__);
		cal_flag=2;
		return ret;
	}
	if (state){
		schedule_delayed_work(&aw9610x->cfg_work,
				      msecs_to_jiffies(cfg_timer_val));
		msleep(500);
		cal_flag=1;
		pr_info("awsar calibration ok");
	}
	return count;
}
int awsar_calibration(struct aw9610x *aw9610x)
{
#ifdef CONFIG_KERNEL_CUSTOM_FACTORY
	pr_info("awsar calibration in ear do nothing end \n");
	return 0;
#else
	uint32_t data_en = 0;
	pr_info("awsar calibration start");
	aw9610x_i2c_read(aw9610x, REG_SCANCTRL0, &data_en);
	msleep(10);
	aw9610x_i2c_write_bits(aw9610x, REG_SCANCTRL0, ~(0x3f << 8),
							(data_en & 0x3f) << 8);
	msleep(500);
	input_report_key(aw9610x->input, KEY_SAR_RELEASECS1, 1);
	input_sync(aw9610x->input);
	input_report_key(aw9610x->input, KEY_SAR_RELEASECS1, 0);
	input_sync(aw9610x->input);
	
	input_report_key(aw9610x->input, KEY_SAR_RELEASECS2, 1);
	input_sync(aw9610x->input);
	input_report_key(aw9610x->input, KEY_SAR_RELEASECS2, 0);
	input_sync(aw9610x->input);
	pr_info("aw96103 calibration end \n");
	return 0;
#endif
}
EXPORT_SYMBOL(awsar_calibration);

static ssize_t sar_docalibration_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 8, "%d\n", cal_flag);

}

static ssize_t onoff_store(struct device *dev,
			struct device_attribute *attr,const char *buf, size_t count)
{
	struct aw9610x *aw9610x = dev_get_drvdata(dev);

    if (!strncmp(buf, "0", 1))
    {
	  aw9610x_i2c_write(aw9610x,0xF008,0x02);
	  msleep(200);
	  onoff_value = 0 ;
	  pr_info("[AW96103]:onoff sar sensor is off \n");
    }
	if (!strncmp(buf, "1", 1))
    {
	 aw9610x_i2c_write(aw9610x,0xF008,0x01);
	 msleep(200);
	 onoff_value = 1 ;
	 pr_info("[AW96103]:onoff on \n");
    }



	return count;
}

static ssize_t onoff_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 8, "%d\n", onoff_value);

}

static ssize_t aw9610x_aot_cali_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	ssize_t ret;
	uint32_t state;
	uint32_t data_en = 0;
	struct aw9610x *aw9610x = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 10, &state);
	if (ret) {
		dev_err(aw9610x->dev, "%s: fail to set aot cali\n", __func__);
		cal_flag=2;
		return ret;
	}
	aw9610x_i2c_read(aw9610x, REG_SCANCTRL0, &data_en);

	if (state != 0){
		aw9610x_i2c_write_bits(aw9610x, REG_SCANCTRL0, ~(0x3f << 8),
							(data_en & 0x3f) << 8);
		msleep(500);
		cal_flag=1;
		pr_info("awsar calibration ok");
	}
	else{
		cal_flag=2;
		dev_err(aw9610x->dev, "%s: fail to set aot cali\n", __func__);
	}

	return count;
}

static DEVICE_ATTR(reg, 0664, aw9610x_get_reg_show, aw9610x_set_reg_store);
static DEVICE_ATTR(valid, 0664, aw9610x_valid_show, NULL);
static DEVICE_ATTR(baseline, 0664, aw9610x_baseline_show, NULL);
static DEVICE_ATTR(diff, 0664, aw9610x_diff_show, NULL);
static DEVICE_ATTR(raw_data, 0664, aw9610x_raw_data_show, NULL);
static DEVICE_ATTR(cali, 0664, NULL, aw9610x_cali_set);
static DEVICE_ATTR(awrw, 0664, aw9610x_awrw_get, aw9610x_awrw_set);
static DEVICE_ATTR(update, 0644, NULL, aw9610x_set_update);
static DEVICE_ATTR(sar_info,0444,sar_info_show,NULL);
static DEVICE_ATTR(sar_proxstatus_cs1,0444,sar_show_proxstatus_cs1,NULL);
static DEVICE_ATTR(sar_proxstatus_cs2,0444,sar_show_proxstatus_cs2,NULL);
static DEVICE_ATTR(sar_calibration, 0664, sar_docalibration_show,aw9610x_aot_cali_set);
static DEVICE_ATTR(onoff_sensor,0664,onoff_show,onoff_store);
static struct attribute *aw9610x_sar_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_valid.attr,
	&dev_attr_baseline.attr,
	&dev_attr_diff.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_awrw.attr,
	&dev_attr_cali.attr,
	&dev_attr_update.attr,
	&dev_attr_sar_info.attr,
	&dev_attr_sar_proxstatus_cs1.attr,
	&dev_attr_sar_proxstatus_cs2.attr,
	&dev_attr_sar_calibration.attr,
	&dev_attr_onoff_sensor.attr,
	NULL
};

static struct attribute_group aw9610x_sar_attribute_group = {
	.attrs = aw9610x_sar_attributes
};

static int32_t aw9610x_input_sys_init(struct aw9610x *aw9610x)
{
	int32_t ret = 0;
	struct input_dev *input_dev;

	pr_info("%s: enter\n", __func__);
	input_dev = input_allocate_device();
	if (!input_dev)
		return -AW_INPUT_ALLOCATE_FILED;

	input_dev->name = AW9610X_I2C_NAME;
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(KEY_F1, input_dev->keybit);
	__set_bit(KEY_SARCS1, input_dev->keybit);
	__set_bit(KEY_SARCS2, input_dev->keybit);
	__set_bit(KEY_SAR_RELEASECS1, input_dev->keybit);
	__set_bit(KEY_SAR_RELEASECS2, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 100, 0, 0);

	ret = input_register_device(input_dev);
	if (ret) {
		pr_err("%s: failed to register input device: %s\n", __func__,
						dev_name(&aw9610x->i2c->dev));
		input_free_device(input_dev);
		return -AW_INPUT_REGISTER_FAILED;
	}
	aw9610x->input = input_dev;

	return AW_SAR_SUCCESS;
}

/*****************************************************
*
* irq init
*
*****************************************************/
static void aw9610x_interrupt_clear(struct aw9610x *aw9610x)
{
	uint32_t irq_flag = 0;
	uint32_t state0;
	uint8_t curr_ch[6] = { 0 };
	uint8_t status;
	uint8_t i = 0;

	pr_info("%s enter\n", __func__);

	aw9610x_i2c_read(aw9610x, REG_HOSTIRQSRC, &aw9610x->irq_status);
	pr_info("%s REG_HOSTIRQSRC = 0x%08x\n", __func__, aw9610x->irq_status);
	irq_flag = (aw9610x->irq_status >> 4) & 0x01;
	if (irq_flag == 1) {
		aw9610x_i2c_write_bits(aw9610x, REG_HOSTIRQEN,
					AW9610X_BIT_REG_HOSTIRQEN_MASK, 0);
	}
	
	aw9610x_i2c_read(aw9610x, REG_STAT0, &state0);
	pr_info("%s REG_STAT0 = 0x%08x\n", __func__, state0);
	status = (uint8_t)((state0&0x3f000000) >> 24);
	pr_info("%s status = 0x%02x\n", __func__, status);
#ifdef ABS_DIS
	if(status){
		for (i = 0; i<6; i++){
			if((status >> i) & 0x1)
			input_report_abs(aw9610x->input, ABS_DISTANCE, i+1);
		}
	}else{
		input_report_abs(aw9610x->input, ABS_DISTANCE, 0);
	}
#else
	
	
	for (i = 0; i < 6; i++)
	{
		curr_ch[i] = (((uint8_t)(state0 >> (24+i)) & 0x01) << 0) | //prox0
					 (((uint8_t)(state0 >> (16+i)) & 0x01) << 1) | //prox1
					 (((uint8_t)(state0 >> (8+i)) & 0x01) << 2) | //prox2
					 (((uint8_t)(state0 >> (0+i)) & 0x01) << 3);  //prox3
		pr_info("%s: CH[%d] = 0x%02x  ",__func__, i, curr_ch[i]); 
	}
	if(last_ch[0] != curr_ch[0]){
		if(curr_ch[0]){
			pr_info("%s CH[0] approach status = %d\n",__func__, curr_ch[0]);
			input_report_key(aw9610x->input, KEY_F1, 1);
		}else{
			pr_info("%s CH[0] far status = %d\n",__func__, curr_ch[0]);
			input_report_key(aw9610x->input, KEY_F1, 0);
		}	
	}
	if(last_ch[1] != curr_ch[1]){
		if (curr_ch[1]){
			pr_info("%s CH[1] approach status = %d\n",__func__, curr_ch[1]);
			input_report_key(aw9610x->input, KEY_SARCS1, 1);
			input_sync(aw9610x->input);
			input_report_key(aw9610x->input, KEY_SARCS1, 0);
			input_sync(aw9610x->input);
		}else{
			pr_info("%s CH[1] far status = %d\n",__func__, curr_ch[1]);
			input_report_key(aw9610x->input, KEY_SAR_RELEASECS1, 1);
			input_sync(aw9610x->input);
			input_report_key(aw9610x->input, KEY_SAR_RELEASECS1, 0);
			input_sync(aw9610x->input);
		}
	}
	if(last_ch[2] != curr_ch[2]){
		if (curr_ch[2]){
			pr_info("%s CH[2] approach status = %d\n",__func__, curr_ch[2]);
			input_report_key(aw9610x->input, KEY_SARCS2, 1);
			input_sync(aw9610x->input);
			input_report_key(aw9610x->input, KEY_SARCS2, 0);
			input_sync(aw9610x->input);
		}else{
			pr_info("%s CH[2] far status = %d\n",__func__, curr_ch[2]);
			input_report_key(aw9610x->input, KEY_SAR_RELEASECS2, 1);
			input_sync(aw9610x->input);
			input_report_key(aw9610x->input, KEY_SAR_RELEASECS2, 0);
			input_sync(aw9610x->input);
		}
	}
	
	for(i = 0; i < 6; i++)
	{
		last_ch[i] = curr_ch[i];
		pr_info("%s:last CH[%d] = 0x%02x  ",__func__, i, last_ch[i]);
	}
	
	input_sync(aw9610x->input);	
#endif
	/*the end of report*/
	//input_sync(aw9610x->input);
}

static irqreturn_t aw9610x_irq(int32_t irq, void *data)
{
	struct aw9610x *aw9610x = data;

	pr_info("%s enter\n", __func__);

	aw9610x_interrupt_clear(aw9610x);
	pr_info("%s exit\n", __func__);

	return IRQ_HANDLED;
}

static int32_t aw9610x_interrupt_init(struct aw9610x *aw9610x)
{
	int32_t irq_flags = 0;
	int32_t ret = 0;

	pr_info("%s enter\n", __func__);

	if (gpio_is_valid(aw9610x->irq_gpio)) {
		ret = devm_gpio_request_one(&aw9610x->i2c->dev,
						aw9610x->irq_gpio,
						GPIOF_DIR_IN,
						"aw9610x_irq_gpio");
		if (ret) {
			pr_err("%s: request irq gpio failed, ret = %d\n",
							__func__, ret);
			ret = -AW_IRQIO_FAILED;
		} else {
			/* register irq handler */
			irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
			ret = devm_request_threaded_irq(&aw9610x->i2c->dev,
						gpio_to_irq(aw9610x->irq_gpio),
						NULL, aw9610x_irq, irq_flags,
						"aw9610x_irq", aw9610x);
			if (ret != 0) {
				pr_err("%s: failed to request IRQ %d: %d\n",
								__func__,
						gpio_to_irq(aw9610x->irq_gpio),
								ret);
				ret = -AW_IRQ_REQUEST_FAILED;
			} else {
				pr_info("%s: IRQ request successfully!\n",
								__func__);
				ret = AW_SAR_SUCCESS;
			}
		}
	} else {
		pr_err("%s: irq gpio invalid!\n", __func__);
		return -AW_IRQIO_FAILED;
	}
	return ret;
}

/*****************************************************
 *
 * parse dts
 *
 *****************************************************/
static void aw9610x_parse_dt(struct device *dev, struct aw9610x *aw9610x,
			   struct device_node *np)
{
	int32_t val = 0;

	aw9610x->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw9610x->irq_gpio < 0) {
		aw9610x->irq_gpio = -1;
		pr_err("%s: no irq gpio provided.\n", __func__);
		return;
	} else {
		pr_info("%s: irq gpio provided ok.\n", __func__);
	}

	val = of_property_read_string(np, "chip_name", &aw9610x->chip_name);
	if (val != 0) {
		aw9610x->chip_name = NULL;
		pr_info("%s: failed to find chip name\n", __func__);
	} else {
		pr_info("%s: the chip name is %s detected\n",
						__func__, aw9610x->chip_name);
	}

	aw9610x->firmware_flag =
			of_property_read_bool(np, "aw9610x,using-firmware");
	pr_info("%s firmware_flag = <%d>\n", __func__, aw9610x->firmware_flag);

	aw9610x->pwprox_dete =
		of_property_read_bool(np, "aw9610x,using-pwon-prox-dete");
	pr_info("%s pwprox_dete = <%d>\n", __func__, aw9610x->pwprox_dete);
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int32_t aw9610x_read_chipid(struct aw9610x *aw9610x)
{
	int32_t ret = -1;
	uint8_t cnt = 0;
	uint32_t reg_val = 0;
	uint32_t reg_eedata0 = 0;
	uint32_t reg_eedata1 = 0;
	uint32_t trim = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw9610x_i2c_read(aw9610x, REG_CHIP_ID, &reg_val);
		if (ret < 0) {
			pr_err("%s: read CHIP ID failed: %d\n", __func__, ret);
		} else {
			reg_val = reg_val >> 16;
			break;
		}

		cnt++;
		usleep_range(2000, 3000);
	}

	aw9610x_i2c_read(aw9610x, REG_EEDA0, &reg_eedata0);
	aw9610x_i2c_read(aw9610x, REG_EEDA1, &reg_eedata1);
	trim = reg_eedata0 + reg_eedata1;

	if (((reg_val == AW9610X_CHIP_ID) && trim) != 0) {
		pr_info("%s aw9610x detected\n", __func__);
		return AW_SAR_SUCCESS;
	} else {
		pr_info("%s unsupported device,the chipid is (0x%04x)\n",
							__func__, reg_val);
		if (trim == 0)
			pr_info("%s: No trim ! trim1 = 0x%08x, trim2 = 0x%08x\n",
						__func__, reg_eedata0, reg_eedata1);
	}

	return -AW_CHIPID_FAILED;
}

static void aw9610x_i2c_set(struct i2c_client *i2c, struct aw9610x *aw9610x)
{
	pr_info("%s: enter\n", __func__);
	aw9610x->dev = &i2c->dev;
	aw9610x->i2c = i2c;
	i2c_set_clientdata(i2c, aw9610x);
}

static int32_t
aw9610x_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	//struct aw9610x *aw9610x;
	struct device_node *np = i2c->dev.of_node;
	int32_t ret = 0;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw9610x = devm_kzalloc(&i2c->dev, sizeof(struct aw9610x), GFP_KERNEL);
	if (aw9610x == NULL) {
		pr_err("%s:failed to malloc memory for aw9610x!\n", __func__);
		ret = -AW_MALLOC_FAILED;
		goto err_malloc;
	}

	aw9610x_i2c_set(i2c, aw9610x);

	/* aw9610x chip id */
	ret = aw9610x_read_chipid(aw9610x);
	if (ret != AW_SAR_SUCCESS) {
		pr_err("%s: read chipid failed, ret=%d\n", __func__, ret);
		saraw_flag=3;
		goto err_chipid;
	}
	if(0 == board_id_get_Network()){
	aw9610x_cfg_name = "aw9610x_lte.bin";
	pr_info("aw9610 cfg name is %s\n", aw9610x_cfg_name);
	}
	if(1 == board_id_get_Network()){	
	aw9610x_cfg_name = "wifi_aw9610x_wifi.bin";
	pr_info("aw9610 cfg name is %s\n", aw9610x_cfg_name);	
	}
	aw9610x_sw_reset(aw9610x);

	ret = aw9610x_init_irq_handle(aw9610x);
	if (ret != AW_SAR_SUCCESS) {
		pr_err("%s: hardware has trouble!, ret=%d\n", __func__, ret);
		goto err_first_irq;
	}

	aw9610x_parse_dt(&i2c->dev, aw9610x, np);

	ret = aw9610x_interrupt_init(aw9610x);
	if (ret == -AW_IRQ_REQUEST_FAILED) {
		pr_err("%s: request irq failed!, ret=%d\n", __func__, ret);
		goto err_requst_irq;
	}

	/* input device */
	ret = aw9610x_input_sys_init(aw9610x);
	if (ret == -AW_INPUT_ALLOCATE_FILED) {
		pr_err("%s:allocate input failed, ret = %d\n", __func__, ret);
		goto exit_input_dev_alloc_failed;
	} else if (ret == -AW_INPUT_REGISTER_FAILED) {
		pr_err("%s:register input failed, ret = %d\n", __func__, ret);
		goto exit_input_register_device_failed;
	}

	/* attribute */
	ret = sysfs_create_group(&i2c->dev.kobj, &aw9610x_sar_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev, "%s error creating sysfs attr files\n",
			 __func__);
		goto err_sysfs;
	}

	ret = aw9610x_sar_cfg_init(aw9610x, AW_CFG_UNLOAD);
	if (ret < 0) {
		pr_info("%s: cfg situation not confirmed!\n", __func__);
		goto err_cfg;
	}
	saraw_flag=2;
	return AW_SAR_SUCCESS;

err_cfg:
err_sysfs:
	sysfs_remove_group(&i2c->dev.kobj, &aw9610x_sar_attribute_group);
exit_input_register_device_failed:
exit_input_dev_alloc_failed:
	devm_kfree(&i2c->dev, aw9610x);
err_requst_irq:
	devm_gpio_free(&i2c->dev, aw9610x->irq_gpio);
err_first_irq:
err_chipid:
err_malloc:
	return ret;
}

static int32_t aw9610x_i2c_remove(struct i2c_client *i2c)
{
	struct aw9610x *aw9610x = i2c_get_clientdata(i2c);

	sysfs_remove_group(&i2c->dev.kobj, &aw9610x_sar_attribute_group);
	devm_kfree(&i2c->dev, aw9610x);

	return 0;
}

static const struct of_device_id aw9610x_dt_match[] = {
	{ .compatible = "awinic,aw9160x_sar" },
	{ },
};

static const struct i2c_device_id aw9610x_i2c_id[] = {
	{ AW9610X_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw9610x_i2c_id);

static struct i2c_driver aw9610x_i2c_driver = {
	.driver = {
		.name = AW9610X_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw9610x_dt_match),
	},
	.probe = aw9610x_i2c_probe,
	.remove = aw9610x_i2c_remove,
	.id_table = aw9610x_i2c_id,
};

static int32_t __init aw9610x_i2c_init(void)
{
	int32_t ret = 0;

	pr_info("aw9610x driver version %s\n", AW9610X_DRIVER_VERSION);

	ret = i2c_add_driver(&aw9610x_i2c_driver);
	if (ret) {
		pr_err("fail to add aw9610x device into i2c\n");
		return ret;
	}

	return 0;
}

late_initcall(aw9610x_i2c_init);
static void __exit aw9610x_i2c_exit(void)
{
	i2c_del_driver(&aw9610x_i2c_driver);
}
module_exit(aw9610x_i2c_exit);
MODULE_DESCRIPTION("AW9610X SAR Driver");

MODULE_LICENSE("GPL v2");
