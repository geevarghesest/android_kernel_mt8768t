/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include "lcm_define.h"
#include "disp_dts_gpio.h"
#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>

#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>

#ifndef CONFIG_FPGA_EARLY_PORTING
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#endif

#endif
#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/*****************************************************************************
 * Define
 *****************************************************************************/
#ifndef CONFIG_FPGA_EARLY_PORTING
#define I2C_I2C_LCD_BIAS_CHANNEL 5
#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL        /* for I2C channel 0 */
#define I2C_ID_NAME "sm5109"
#define TPS_ADDR 0x3E

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info sm5109_board_info __initdata = {
	I2C_BOARD_INFO(I2C_ID_NAME,
			TPS_ADDR) };

static struct i2c_client *sm5109_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int sm5109_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int sm5109_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct sm5109_dev {
	struct i2c_client *client;

};

static const struct of_device_id _lcm_i2c_of_match[] = {
	{ .compatible = "mediatek,I2C_LCD_BIAS", },
	{},
};

static const struct i2c_device_id sm5109_id[] = {
	{I2C_ID_NAME, 0},
	{}
};
static struct i2c_driver sm5109_iic_driver = {
	.id_table = sm5109_id,
	.probe = sm5109_probe,
	.remove = sm5109_remove,
	/* .detect               = mt6605_detect, */
	.driver = {
		.owner = THIS_MODULE,
		.name = "sm5109",
		.of_match_table = _lcm_i2c_of_match,
	},
};

static int sm5109_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	pr_info("[LCM]%s\n",__func__);
	sm5109_i2c_client = client;
	return 0;
}

static int sm5109_remove(struct i2c_client *client)
{
	pr_info("[LCM]%s\n",__func__);
	//sm5109_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

int sm5109_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = sm5109_i2c_client;
	char write_data[2] = { 0 };
	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_notice("[LCM]sm5109 write data fail !!\n");
	return ret;
}

static int __init sm5109_iic_init(void)
{
	pr_info("[LCM]%s\n",__func__);
	i2c_register_board_info(TPS_I2C_BUSNUM, &sm5109_board_info, 1);
	pr_info("[LCM]%s\n",__func__);
	i2c_add_driver(&sm5109_iic_driver);
	pr_info("[LCM]%s\n",__func__);
	return 0;
}

static void __exit sm5109_iic_exit(void)
{
	pr_info("[LCM]%s\n",__func__);
	i2c_del_driver(&sm5109_iic_driver);
}


module_init(sm5109_iic_init);
module_exit(sm5109_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK sm5109 I2C Driver");
MODULE_LICENSE("GPL");
#endif
