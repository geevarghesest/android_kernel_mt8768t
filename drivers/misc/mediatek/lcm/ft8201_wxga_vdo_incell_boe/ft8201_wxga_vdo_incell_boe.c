// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
#include "lcm_define.h"

#ifndef BUILD_LK
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
#endif

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#include <linux/gpio.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif
#endif

#ifndef MACH_FPGA
#include <lcm_pmic.h>
#endif

#include "../../../../input/touchscreen/lct_tp_gesture.h"

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID (0x98)

static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;
extern int sm5109_write_bytes(unsigned char addr, unsigned char value);
extern int SGM37604A_write_bytes(unsigned char addr, unsigned char value);
#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V4(para_tbl, size, force_update) \
	lcm_util.dsi_set_cmdq_V4(para_tbl, size, force_update)
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define GPIO_OUT_LOW 0
#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0

extern unsigned int GPIO_LCD_BL_EN_PIN;
extern unsigned int GPIO_LCD_BIAS_ENP;
extern unsigned int GPIO_LCD_BIAS_ENN;
extern unsigned int GPIO_TP_RST;

/* ----------------------------------------------------------------- */
/* Local Constants */
/* ----------------------------------------------------------------- */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE      0
#define FRAME_WIDTH           (800)
#define FRAME_HEIGHT          (1280)
#define LCM_DENSITY           (240)

#define LCM_PHYSICAL_WIDTH    (107640)
#define LCM_PHYSICAL_HEIGHT	  (172224)


#define REGFLAG_DELAY		  0xFFFC
#define REGFLAG_UDELAY	      0xFFFB
#define REGFLAG_END_OF_TABLE  0xFFFD
#define REGFLAG_RESET_LOW     0xFFFE
#define REGFLAG_RESET_HIGH    0xFFFF
#define	DSI_DCS_SHORT_PACKET_ID_1	0x15

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

static unsigned int BIAS_FLAG = 0;


/* ----------------------------------------------------------------- */
/*  Local Variables */
/* ----------------------------------------------------------------- */

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, output);
#else
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
#endif
}

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};


static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {}},
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {}},
	{0x04, 0x01,{0x5a}},
	{0x05, 0x01,{0x5a}},
	{REGFLAG_DELAY, 150, {}},
};

static struct LCM_setting_table init_setting_vdo[] = {
	{0x11,0x01,{0x00}},
	{REGFLAG_DELAY,120,{}},
	{0x50,0x02,{0x5a,0x23}},
	{0x90,0x02,{0x00,0x00}},
	{0x94,0x02,{0x2c,0x00}},
	{0x50,0x02,{0x5a,0x19}},
	{0xa0,0x03,{0x00,0x00,0x44}},
	{0x50,0x02,{0x5A,0x0A}},
	{0x80,0x10,{0xB7,0x6F,0x03,0x00,0x0E,0x1A,0x28,0x39,0x49,0x4D,0x7C,0x45,0x79,0xC3,0x94,0xA6}},
	{0x90,0x10,{0x75,0x6D,0x5A,0x46,0x31,0x23,0x0B,0x00,0x0E,0x1A,0x28,0x39,0x49,0x4D,0x7C,0x45}},
	{0xA0,0x0C,{0x79,0xC3,0x94,0xA6,0x75,0x6D,0x5A,0x46,0x31,0x23,0x0B,0x00}},
	{0x50,0x01,{0x00}},
	{0x29,0x01,{0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table_V4 BL_Level[] = {
	{0x39, 0x50, 2, {0x5A,0x23}, 0 },
	{0x15, 0x90, 1, {0xFF}, 0 },
	{0x15, 0x50, 1, {0x00}, 0 },
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count,
				table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->density = LCM_DENSITY;

	params->dsi.mode = BURST_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	params->dsi.switch_mode_enable = 1;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 8;
	params->dsi.vertical_backporch = 32;
	params->dsi.vertical_frontporch = 184;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 14;
	params->dsi.horizontal_backporch = 50;
	params->dsi.horizontal_frontporch = 25;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_range = 4;
	params->dsi.ssc_disable = 1;

#ifndef CONFIG_FPGA_EARLY_PORTING
	params->dsi.PLL_CLOCK = 250;
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
	params->dsi.clk_lp_per_line_enable = 0;

	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;


#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 0;
	params->corner_pattern_width = 720;
	params->corner_pattern_height = 32;
#endif

	/* params->use_gpioID = 1; */
	/* params->gpioID_value = 0; */
}


static void lcm_init(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	if(BIAS_FLAG == 0){
		pr_notice("lsy doubletouch to wake up is open,no need power on again\n");
		lcm_set_gpio_output(GPIO_TP_RST, GPIO_OUT_ZERO);
		MDELAY(1);
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, GPIO_OUT_ZERO);
		MDELAY(1);
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, GPIO_OUT_ZERO);
		MDELAY(10);
		SET_RESET_PIN(0);
		MDELAY(1);
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, GPIO_OUT_ONE);
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, GPIO_OUT_ONE);
		sm5109_write_bytes(0x00,20);
		sm5109_write_bytes(0x01,20);
		MDELAY(10);
		lcm_set_gpio_output(GPIO_TP_RST, GPIO_OUT_ONE);
		MDELAY(1);
		SET_RESET_PIN(1);
		MDELAY(8);
		lcm_set_gpio_output(GPIO_TP_RST, GPIO_OUT_ZERO);
		MDELAY(5);
		SET_RESET_PIN(0);
		MDELAY(5);
		lcm_set_gpio_output(GPIO_TP_RST, GPIO_OUT_ONE);
		MDELAY(1);
		SET_RESET_PIN(1);
		MDELAY(50);
	}
	else{
		pr_notice("lsy doubletouch to wake up is closed,need power on\n");
		lcm_set_gpio_output(GPIO_TP_RST, GPIO_OUT_ZERO);
		MDELAY(5);
		SET_RESET_PIN(0);
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, GPIO_OUT_ONE);
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, GPIO_OUT_ONE);
		sm5109_write_bytes(0x00,20);
		sm5109_write_bytes(0x01,20);
		MDELAY(10);
		lcm_set_gpio_output(GPIO_TP_RST, GPIO_OUT_ONE);
		MDELAY(5);
		SET_RESET_PIN(1);
		MDELAY(2);
		lcm_set_gpio_output(GPIO_TP_RST, GPIO_OUT_ZERO);
		MDELAY(5);
		SET_RESET_PIN(0);
		MDELAY(5);
		lcm_set_gpio_output(GPIO_TP_RST, GPIO_OUT_ONE);
		MDELAY(1);
		SET_RESET_PIN(1);
		MDELAY(50);

		BIAS_FLAG = 0;
	}

	push_table(NULL,
		init_setting_vdo,
		sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table),
		1);

	MDELAY(1);
	lcm_set_gpio_output(GPIO_LCD_BL_EN_PIN, GPIO_OUT_ONE);

	pr_notice("[Kernel/LCM] %s exit\n", __func__);
}

static void lcm_suspend(void)
{
	pr_notice("[Kernel/LCM] %s enter_lsy 111\n", __func__);

	push_table(NULL,
		lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
		1);

	MDELAY(1);
	lcm_set_gpio_output(GPIO_LCD_BL_EN_PIN, GPIO_OUT_ZERO);

	if(get_lct_tp_gesture_status() == 0){
		pr_notice("lsy doubletouch to wake up is closed,need power off\n");
		MDELAY(100);
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, GPIO_OUT_ZERO);
		MDELAY(1);
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, GPIO_OUT_ZERO);
		BIAS_FLAG = 1;
	}

	pr_notice("[Kernel/LCM] %s exit\n", __func__);

}

static void lcm_resume(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	lcm_init();

	pr_notice("[Kernel/LCM] %s exit\n", __func__);
}


#if (LCM_DSI_CMD_MODE)

/* partial update restrictions:
 * 1. roi width must be 1080 (full lcm width)
 * 2. vertical start (y) must be multiple of 16
 * 3. vertical height (h) must be multiple of 16
 */
static void lcm_validate_roi(int *x, int *y, int *width, int *height)
{
	unsigned int y1 = *y;
	unsigned int y2 = *height + y1 - 1;
	unsigned int x1, w, h;

	x1 = 0;
	w = FRAME_WIDTH;

	y1 = round_down(y1, 16);
	h = y2 - y1 + 1;

	/* in some cases, roi maybe empty. Then we need to use minimu roi */
	if (h < 16)
		h = 16;

	h = round_up(h, 16);

	/* check height again */
	if (y1 >= FRAME_HEIGHT || y1 + h > FRAME_HEIGHT) {
		/* assign full screen roi */
		pr_notice("%s calc error, assign full roi: y=%d, h=%d\n",
			__func__, *y, *height);
		y1 = 0;
		h = FRAME_HEIGHT;
	}

	/*LCM_LOGD("lcm_validate_roi (%d,%d,%d,%d) to (%d,%d,%d,%d)\n",*/
	/*	*x, *y, *width, *height, x1, y1, w, h);*/

	*x = x1;
	*width = w;
	*y = y1;
	*height = h;
}
#endif

static unsigned int lcm_ata_check(unsigned char *buffer)
{
	return 1;
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{

	if(level != 0 && level < 4)
	{
		level = 4;
		SGM37604A_write_bytes(0x19,0xA0);		//Limiting maximum current by I2C
	}
	else
		SGM37604A_write_bytes(0x19,0xCF);		//Limiting maximum current by I2C

	pr_notice("%s,ft8201 backlight: level = %d\n", __func__, level);

	BL_Level[1].para_list[0] = level;
	dsi_set_cmdq_V4(BL_Level,
				sizeof(BL_Level)
				/ sizeof(struct LCM_setting_table_V4), 1);

}


struct LCM_DRIVER ft8201_wxga_vdo_incell_boe_lcm_drv = {
	.name = "ft8201_wxga_vdo_incell_boe",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.ata_check = lcm_ata_check,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
#if (LCM_DSI_CMD_MODE)
	.validate_roi = lcm_validate_roi,
#endif
};

