// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author Wy Chuang<wy.chuang@mediatek.com>
 */

#include <linux/device.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mfd/mt6397/core.h>/* PMIC MFD core header */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <mtk_musb.h>
#include <tcpm.h>
#include "mtk_battery.h"

/* ============================================================ */
/* pmic control start*/
/* ============================================================ */
#define PMIC_RG_BC11_VREF_VTH_ADDR                         0xb98
#define PMIC_RG_BC11_VREF_VTH_MASK                         0x3
#define PMIC_RG_BC11_VREF_VTH_SHIFT                        0

#define PMIC_RG_BC11_CMP_EN_ADDR                           0xb98
#define PMIC_RG_BC11_CMP_EN_MASK                           0x3
#define PMIC_RG_BC11_CMP_EN_SHIFT                          2

#define PMIC_RG_BC11_IPD_EN_ADDR                           0xb98
#define PMIC_RG_BC11_IPD_EN_MASK                           0x3
#define PMIC_RG_BC11_IPD_EN_SHIFT                          4

#define PMIC_RG_BC11_IPU_EN_ADDR                           0xb98
#define PMIC_RG_BC11_IPU_EN_MASK                           0x3
#define PMIC_RG_BC11_IPU_EN_SHIFT                          6

#define PMIC_RG_BC11_BIAS_EN_ADDR                          0xb98
#define PMIC_RG_BC11_BIAS_EN_MASK                          0x1
#define PMIC_RG_BC11_BIAS_EN_SHIFT                         8

#define PMIC_RG_BC11_BB_CTRL_ADDR                          0xb98
#define PMIC_RG_BC11_BB_CTRL_MASK                          0x1
#define PMIC_RG_BC11_BB_CTRL_SHIFT                         9

#define PMIC_RG_BC11_RST_ADDR                              0xb98
#define PMIC_RG_BC11_RST_MASK                              0x1
#define PMIC_RG_BC11_RST_SHIFT                             10

#define PMIC_RG_BC11_VSRC_EN_ADDR                          0xb98
#define PMIC_RG_BC11_VSRC_EN_MASK                          0x3
#define PMIC_RG_BC11_VSRC_EN_SHIFT                         11

#define PMIC_RG_BC11_DCD_EN_ADDR                           0xb98
#define PMIC_RG_BC11_DCD_EN_MASK                           0x1
#define PMIC_RG_BC11_DCD_EN_SHIFT                          13

#define PMIC_RGS_BC11_CMP_OUT_ADDR                         0xb98
#define PMIC_RGS_BC11_CMP_OUT_MASK                         0x1
#define PMIC_RGS_BC11_CMP_OUT_SHIFT                        14

#define PMIC_RGS_CHRDET_ADDR                               0xa88
#define PMIC_RGS_CHRDET_MASK                               0x1
#define PMIC_RGS_CHRDET_SHIFT                              4

#define R_CHARGER_1	330
#define R_CHARGER_2	39

int typec_cc_orientation = 0;

static int pogo_real_type = 0;
bool g_batt_protect_mode = false;
extern void bq25601_set_batfet_disable(unsigned int val);
extern int pogo_charger_is_pogo_charging(void);
extern int pogo_charger_is_usb_charging(void);
extern unsigned int board_id_get_ProjectName(void);
extern int pogo_charger_is_otg_exist(void);
extern int is_only_pogo_contact(void);
extern int is_usb_pogo_not_contact(void);

struct mtk_charger_type {
	struct mt6397_chip *chip;
	struct regmap *regmap;
	struct platform_device *pdev;
	struct mutex ops_lock;
	struct tcpc_device *tcpc_dev;
	struct notifier_block pd_nb;

	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	struct power_supply *psy;
	struct power_supply_desc ac_desc;
	struct power_supply_config ac_cfg;
	struct power_supply *ac_psy;
	struct power_supply_desc usb_desc;
	struct power_supply_config usb_cfg;
	struct power_supply *usb_psy;

	struct iio_channel *chan_vbus;
	struct work_struct chr_work;

	enum power_supply_usb_type type;

	int first_connect;
	int bc12_active;
};

static enum power_supply_property chr_type_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property mt_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mt_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
	POWER_SUPPLY_PROP_BATTERY_PROTECT_MODE,
	POWER_SUPPLY_PROP_REAL_TYPE,
};

void bc11_set_register_value(struct regmap *map,
	unsigned int addr,
	unsigned int mask,
	unsigned int shift,
	unsigned int val)
{
	regmap_update_bits(map,
		addr,
		mask << shift,
		val << shift);
}

unsigned int bc11_get_register_value(struct regmap *map,
	unsigned int addr,
	unsigned int mask,
	unsigned int shift)
{
	unsigned int value = 0;

	regmap_read(map, addr, &value);
	value =
		(value &
		(mask << shift))
		>> shift;
	return value;
}

static void hw_bc11_init(struct mtk_charger_type *info)
{
#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
	int timeout = 200;
#endif
	msleep(200);
	if (info->first_connect == true) {
#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
		/* add make sure USB Ready */
		if (is_usb_rdy() == false) {
			pr_info("CDP, block\n");
			while (is_usb_rdy() == false && timeout > 0) {
				msleep(100);
				timeout--;
			}
			if (timeout == 0)
				pr_info("CDP, timeout\n");
			else
				pr_info("CDP, free\n");
		} else
			pr_info("CDP, PASS\n");
#endif
		info->first_connect = false;
	}

	/* RG_bc11_BIAS_EN=1 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_BIAS_EN_ADDR,
		PMIC_RG_BC11_BIAS_EN_MASK,
		PMIC_RG_BC11_BIAS_EN_SHIFT,
		1);
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPU_EN_ADDR,
		PMIC_RG_BC11_IPU_EN_MASK,
		PMIC_RG_BC11_IPU_EN_SHIFT,
		0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0);
	/* bc11_RST=1 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_RST_ADDR,
		PMIC_RG_BC11_RST_MASK,
		PMIC_RG_BC11_RST_SHIFT,
		1);
	/* bc11_BB_CTRL=1 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_BB_CTRL_ADDR,
		PMIC_RG_BC11_BB_CTRL_MASK,
		PMIC_RG_BC11_BB_CTRL_SHIFT,
		1);
	/* add pull down to prevent PMIC leakage */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x1);
	msleep(50);

#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
	Charger_Detect_Init();
#endif
}

static unsigned int hw_bc11_DCD(struct mtk_charger_type *info)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_IPU_EN[1.0] = 10 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPU_EN_ADDR,
		PMIC_RG_BC11_IPU_EN_MASK,
		PMIC_RG_BC11_IPU_EN_SHIFT,
		0x2);
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x1);
	/* RG_bc11_VREF_VTH = [1:0]=01 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x1);
	/* RG_bc11_CMP_EN[1.0] = 10 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x2);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(info->regmap,
		PMIC_RGS_BC11_CMP_OUT_ADDR,
		PMIC_RGS_BC11_CMP_OUT_MASK,
		PMIC_RGS_BC11_CMP_OUT_SHIFT);

	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPU_EN_ADDR,
		PMIC_RG_BC11_IPU_EN_MASK,
		PMIC_RG_BC11_IPU_EN_SHIFT,
		0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepA2(struct mtk_charger_type *info)
{
	unsigned int wChargerAvail = 0;

	/* RG_bc11_VSRC_EN[1.0] = 10 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0x2);
	/* RG_bc11_IPD_EN[1:0] = 01 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x1);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(info->regmap,
					PMIC_RGS_BC11_CMP_OUT_ADDR,
					PMIC_RGS_BC11_CMP_OUT_MASK,
					PMIC_RGS_BC11_CMP_OUT_SHIFT);

	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepB2(struct mtk_charger_type *info)
{
	unsigned int wChargerAvail = 0;

	/*enable the voltage source to DM*/
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0x1);
	/* enable the pull-down current to DP */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x2);
	/* VREF threshold voltage for comparator  =0.325V */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x0);
	/* enable the comparator to DP */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x2);
	msleep(80);
	wChargerAvail = bc11_get_register_value(info->regmap,
		PMIC_RGS_BC11_CMP_OUT_ADDR,
		PMIC_RGS_BC11_CMP_OUT_MASK,
		PMIC_RGS_BC11_CMP_OUT_SHIFT);
	/*reset to default value*/
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0x0);
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x0);
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x0);
	if (wChargerAvail == 1) {
		bc11_set_register_value(info->regmap,
			PMIC_RG_BC11_VSRC_EN_ADDR,
			PMIC_RG_BC11_VSRC_EN_MASK,
			PMIC_RG_BC11_VSRC_EN_SHIFT,
			0x2);
		pr_info("charger type: DCP, keep DM voltage source in stepB2\n");
	}
	return wChargerAvail;

}

static void hw_bc11_done(struct mtk_charger_type *info)
{
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0x0);
	/* RG_bc11_VREF_VTH = [1:0]=0 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPU_EN_ADDR,
		PMIC_RG_BC11_IPU_EN_MASK,
		PMIC_RG_BC11_IPU_EN_SHIFT,
		0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x0);
	/* RG_bc11_BIAS_EN=0 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_BIAS_EN_ADDR,
		PMIC_RG_BC11_BIAS_EN_MASK,
		PMIC_RG_BC11_BIAS_EN_SHIFT,
		0x0);

#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
	Charger_Detect_Release();
#endif
}

static void dump_charger_name(int type)
{
	switch (type) {
	case POWER_SUPPLY_TYPE_UNKNOWN:
		pr_info("charger type: %d, CHARGER_UNKNOWN\n", type);
		break;
	case POWER_SUPPLY_TYPE_USB:
		pr_info("charger type: %d, Standard USB Host\n", type);
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		pr_info("charger type: %d, Charging USB Host\n", type);
		break;
#ifdef FIXME
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		pr_info("charger type: %d, Non-standard Charger\n", type);
		break;
#endif
	case POWER_SUPPLY_TYPE_USB_DCP:
		pr_info("charger type: %d, Standard Charger\n", type);
		break;
	case POWER_SUPPLY_USB_TYPE_POGO:
		pr_info("charger type: %d, Pogo Charger\n", type);
		break;
	default:
		pr_info("charger type: %d, Not Defined!!!\n", type);
		break;
	}
}

static int get_charger_type(struct mtk_charger_type *info)
{
	enum power_supply_usb_type type;
	if (0 == pogo_charger_is_pogo_charging())//not pogo charger, do normal usb dectection.
	{
		hw_bc11_init(info);
		if (hw_bc11_DCD(info)) {
			info->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			type = POWER_SUPPLY_USB_TYPE_DCP;
		} else {
			if (hw_bc11_stepA2(info)) {
				if (hw_bc11_stepB2(info)) {
					info->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
					type = POWER_SUPPLY_USB_TYPE_DCP;
				} else {
					info->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
					type = POWER_SUPPLY_USB_TYPE_CDP;
				}
			} else {
				info->psy_desc.type = POWER_SUPPLY_TYPE_USB;
				type = POWER_SUPPLY_USB_TYPE_SDP;
			}
		}

		if (type != POWER_SUPPLY_USB_TYPE_DCP)
			hw_bc11_done(info);
		else
			pr_info("charger type: skip bc11 release for BC12 DCP SPEC\n");
	}
	else//pogo charger,set type to POGO
	{
		type = POWER_SUPPLY_USB_TYPE_POGO;
		pr_info("charger type: pogo\n");
	}

	dump_charger_name(info->psy_desc.type);

	return type;
}

static int get_vbus_voltage(struct mtk_charger_type *info,
	int *val)
{
	int ret;

	if (!IS_ERR(info->chan_vbus)) {
		ret = iio_read_channel_processed(info->chan_vbus, val);
		if (ret < 0)
			pr_notice("[%s]read fail,ret=%d\n", __func__, ret);
	} else {
		pr_notice("[%s]chan error %d\n", __func__, info->chan_vbus);
		ret = -ENOTSUPP;
	}

	*val = (((R_CHARGER_1 +
			R_CHARGER_2) * 100 * *val) /
			R_CHARGER_2) / 100;

	return ret;
}

enum charge_detect_work_state {
	CHARGER_DETECT_UNUSED = 0,
	CHARGER_DETECT_USING,
	CHARGER_DETECT_DO_AGAIN,
};

int g_do_charger_detect = CHARGER_DETECT_UNUSED;
void do_charger_detect(struct mtk_charger_type *info, bool en)
{
	union power_supply_propval prop, prop2, prop3;
	int ret = 0;

#ifndef CONFIG_TCPC_CLASS
	if (!mt_usb_is_device()) {
		pr_info("charger type: UNKNOWN, Now is usb host mode. Skip detection\n");
		return;
	}
#endif

	prop.intval = en;
	if (en) {
		ret = power_supply_set_property(info->psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
		ret = power_supply_get_property(info->psy,
				POWER_SUPPLY_PROP_TYPE, &prop2);
		ret = power_supply_get_property(info->psy,
				POWER_SUPPLY_PROP_USB_TYPE, &prop3);
	} else {
		prop2.intval = POWER_SUPPLY_TYPE_UNKNOWN;
		prop3.intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		info->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		info->type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	}

	pr_notice("%s type:%d usb_type:%d\n", __func__, prop2.intval, prop3.intval);

	switch (prop3.intval) {
	case POWER_SUPPLY_USB_TYPE_UNKNOWN:
		info->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case POWER_SUPPLY_USB_TYPE_SDP:
		info->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		info->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
		info->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case POWER_SUPPLY_USB_TYPE_POGO:
		info->psy_desc.type = POWER_SUPPLY_TYPE_POGO;
		break;
	}

	power_supply_changed(info->psy);
}

static void do_charger_detection_work(struct work_struct *data)
{
	struct mtk_charger_type *info = (struct mtk_charger_type *)container_of(
				     data, struct mtk_charger_type, chr_work);
	unsigned int chrdet = 0;
	int timeout = 200;

	if (g_do_charger_detect == CHARGER_DETECT_UNUSED)
	{
		g_do_charger_detect = CHARGER_DETECT_USING;
	}
	pr_info("%s: g_do_charger_detect s:%d\n", __func__, g_do_charger_detect);

	/* add make sure USB Ready begin*/
	 if (is_usb_rdy() == false) {
		 pr_info("CDP, block\n");
		 while (is_usb_rdy() == false && timeout > 0) {
			 msleep(100);
			 timeout--;
		 }
		 if (timeout == 0)
			 pr_info("CDP, timeout\n");
		 else
			 pr_info("CDP, free\n");
	 } else
		 pr_info("CDP, PASS\n");
	/* add make sure USB Ready end*/

	chrdet = bc11_get_register_value(info->regmap,
		PMIC_RGS_CHRDET_ADDR,
		PMIC_RGS_CHRDET_MASK,
		PMIC_RGS_CHRDET_SHIFT);

	if(pogo_charger_is_otg_exist())//standalone poweron with OTG diaplay charging
	chrdet = 0;

	pr_notice("%s: chrdet:%d\n", __func__, chrdet);
	//if (chrdet)
		do_charger_detect(info, chrdet);

	pr_info("%s: g_do_charger_detect e:%d\n", __func__, g_do_charger_detect);
	if (g_do_charger_detect == CHARGER_DETECT_USING)
	{
		g_do_charger_detect = CHARGER_DETECT_UNUSED;
	}
	else if (g_do_charger_detect == CHARGER_DETECT_DO_AGAIN)
	{
		if (is_usb_rdy() == false) {
			pr_info("CDP, block\n");
			while (is_usb_rdy() == false && timeout > 0) {
				msleep(100);
				timeout--;
			}
			if (timeout == 0)
				pr_info("CDP, timeout\n");
			else
				pr_info("CDP, free\n");
		} else
			pr_info("CDP, PASS\n");

		chrdet = bc11_get_register_value(info->regmap,
			PMIC_RGS_CHRDET_ADDR,
			PMIC_RGS_CHRDET_MASK,
			PMIC_RGS_CHRDET_SHIFT);

		if(pogo_charger_is_otg_exist())
			chrdet = 0;

		pr_notice("%s: chrdet:%d\n", __func__, chrdet);
		do_charger_detect(info, chrdet);

		g_do_charger_detect = CHARGER_DETECT_UNUSED;
	}
}

extern int g_soc_state;
extern int g_usb_plug_event;
extern int g_pogo_charge;
irqreturn_t chrdet_int_handler(int irq, void *data)
{
	struct mtk_charger_type *info = data;
	unsigned int chrdet = 0;
	bool power_off_chg = is_kernel_power_off_charging();

	if(0 == board_id_get_ProjectName()){//standalone
	if(1 == pogo_charger_is_otg_exist())
	return IRQ_HANDLED;
	}

	chrdet = bc11_get_register_value(info->regmap,
		PMIC_RGS_CHRDET_ADDR,
		PMIC_RGS_CHRDET_MASK,
		PMIC_RGS_CHRDET_SHIFT);

	if (power_off_chg == false)
	{
		pr_notice("%s: g_do_charger_detect s:%d\n", __func__, g_do_charger_detect);
		if (g_do_charger_detect == CHARGER_DETECT_USING)
		{
			g_do_charger_detect = CHARGER_DETECT_DO_AGAIN;
			return IRQ_HANDLED;
		}
		do_charger_detect(info, chrdet);
	}
	else
	{
		if (g_usb_plug_event == 1)
		{
			g_usb_plug_event = 0;
			if (is_only_pogo_contact())
			{
				do_charger_detect(info, true);
				g_pogo_charge = 1;
				pr_notice("%s:  pogo in\n", __func__);
			}
		}
		else
		{
			if (is_only_pogo_contact())
			{
				do_charger_detect(info, true);
				g_pogo_charge = 1;
				pr_notice("%s: pogo in\n", __func__);
			}
			else if (is_usb_pogo_not_contact())
			{
				if (g_pogo_charge == 1)
				{
					do_charger_detect(info, false);
					g_pogo_charge = 0;
				}
				pr_notice("%s: pogo out\n", __func__);
			}
		}
	}
	pr_notice("%s: chrdet:%d, power_off_chg:%d\n",__func__, chrdet, power_off_chg);

	g_soc_state = 0;

	return IRQ_HANDLED;
}


static int psy_chr_type_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_charger_type *info;
	int vbus = 0;

	pr_notice("%s: prop:%d\n", __func__, psp);
	info = (struct mtk_charger_type *)power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (info->type == POWER_SUPPLY_USB_TYPE_UNKNOWN)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		 val->intval = info->psy_desc.type;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = info->type;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		get_vbus_voltage(info, &vbus);
		val->intval = vbus;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int psy_chr_type_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	struct mtk_charger_type *info;

	pr_notice("%s: prop:%d %d\n", __func__, psp, val->intval);

	info = (struct mtk_charger_type *)power_supply_get_drvdata(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		info->type = get_charger_type(info);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mt_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_charger_type *info;

	info = (struct mtk_charger_type *)power_supply_get_drvdata(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		/* Force to 1 in all charger type */
		if (info->type != POWER_SUPPLY_USB_TYPE_UNKNOWN)
			val->intval = 1;
		/* Reset to 0 if charger type is USB */
		if ((info->type == POWER_SUPPLY_USB_TYPE_SDP) ||
			(info->type == POWER_SUPPLY_USB_TYPE_CDP))
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mt_usb_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_charger_type *info;

	info = (struct mtk_charger_type *)power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if ((info->type == POWER_SUPPLY_USB_TYPE_SDP) ||
			(info->type == POWER_SUPPLY_USB_TYPE_CDP))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = 500000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = 5000000;
		break;
	case POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION:
		val->intval = typec_cc_orientation;
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_BATTERY_PROTECT_MODE:
		val->intval = g_batt_protect_mode;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = pogo_real_type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mt_usb_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	struct mtk_charger_type *info;

	pr_notice("%s: prop:%d %d\n", __func__, psp, val->intval);

	info = (struct mtk_charger_type *)power_supply_get_drvdata(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		bq25601_set_batfet_disable(val->intval);//1 = Turn off BATFET
		break;
	case POWER_SUPPLY_PROP_BATTERY_PROTECT_MODE:
		g_batt_protect_mode = val->intval;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		pogo_real_type = val->intval;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int psy_charger_type_property_is_writeable(struct power_supply *psy,
					       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
	case POWER_SUPPLY_PROP_BATTERY_PROTECT_MODE:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_usb_type mt6357_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
};

static char *mt6357_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static int pd_tcp_notifier_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct mtk_charger_type *chg_data =
		(struct mtk_charger_type *)container_of(nb,
		struct mtk_charger_type, pd_nb);
	bool power_off_chg = is_kernel_power_off_charging();

	if (power_off_chg == true)
	{
		switch (event)
		{
			case TCP_NOTIFY_TYPEC_STATE:
				if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
				  (noti->typec_state.new_state == TYPEC_ATTACHED_SNK ||
				  noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC ||
				  noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC))
				{
					pr_info("%s USB Plug in, pol = %d\n", __func__,
					      noti->typec_state.polarity);
					//handle_typec_attach(chg_data, true);
					//do_charger_detection_work(&chg_data->chr_work);
					schedule_work(&chg_data->chr_work);
				}
				else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SNK ||
				  noti->typec_state.old_state == TYPEC_ATTACHED_CUSTOM_SRC ||
				  noti->typec_state.old_state == TYPEC_ATTACHED_NORP_SRC)
				  && noti->typec_state.new_state == TYPEC_UNATTACHED)
				{
					pr_info("%s USB Plug out\n", __func__);
					schedule_work(&chg_data->chr_work);
				}
				break;

			default:
				break;
		}
	}

	pr_notice("%s: notifier_call done\n", __func__);
	return NOTIFY_OK;
}

static int mt6357_charger_type_probe(struct platform_device *pdev)
{
	struct mtk_charger_type *info;
	struct iio_channel *chan_vbus;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret = 0;

	pr_notice("%s: starts\n", __func__);

	chan_vbus = devm_iio_channel_get(
		&pdev->dev, "pmic_vbus");
	if (IS_ERR(chan_vbus)) {
		pr_notice("mt6357 charger type requests probe deferral ret:%d\n",
			chan_vbus);
		return -EPROBE_DEFER;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info),
		GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->chip = (struct mt6397_chip *)dev_get_drvdata(
		pdev->dev.parent);
	info->regmap = info->chip->regmap;

	dev_set_drvdata(&pdev->dev, info);
	info->pdev = pdev;
	mutex_init(&info->ops_lock);

	info->psy_desc.name = "mtk_charger_type";
	info->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_desc.properties = chr_type_properties;
	info->psy_desc.num_properties = ARRAY_SIZE(chr_type_properties);
	info->psy_desc.get_property = psy_chr_type_get_property;
	info->psy_desc.set_property = psy_chr_type_set_property;
	info->psy_desc.property_is_writeable =
			psy_charger_type_property_is_writeable;
	info->psy_desc.usb_types = mt6357_charger_usb_types,
	info->psy_desc.num_usb_types = ARRAY_SIZE(mt6357_charger_usb_types),
	info->psy_cfg.drv_data = info;

	info->psy_cfg.of_node = np;
	info->psy_cfg.supplied_to = mt6357_charger_supplied_to;
	info->psy_cfg.num_supplicants = ARRAY_SIZE(mt6357_charger_supplied_to);

	info->ac_desc.name = "ac";
	info->ac_desc.type = POWER_SUPPLY_TYPE_MAINS;
	info->ac_desc.properties = mt_ac_properties;
	info->ac_desc.num_properties = ARRAY_SIZE(mt_ac_properties);
	info->ac_desc.get_property = mt_ac_get_property;
	info->ac_cfg.drv_data = info;

	info->usb_desc.name = "usb";
	info->usb_desc.type = POWER_SUPPLY_TYPE_USB;
	info->usb_desc.properties = mt_usb_properties;
	info->usb_desc.num_properties = ARRAY_SIZE(mt_usb_properties);
	info->usb_desc.get_property = mt_usb_get_property;
	info->usb_desc.set_property = mt_usb_set_property;
	info->usb_cfg.drv_data = info;

	info->psy = power_supply_register(&pdev->dev, &info->psy_desc,
			&info->psy_cfg);

	if (IS_ERR(info->psy)) {
		pr_notice("%s Failed to register power supply: %ld\n",
			__func__, PTR_ERR(info->psy));
		return PTR_ERR(info->psy);
	}
	pr_notice("%s register psy success\n", __func__);

	info->chan_vbus = devm_iio_channel_get(
		&pdev->dev, "pmic_vbus");
	if (IS_ERR(info->chan_vbus)) {
		pr_notice("chan_vbus auxadc get fail, ret=%d\n",
			PTR_ERR(info->chan_vbus));
	}

	if (of_property_read_u32(np, "bc12_active", &info->bc12_active) < 0)
		pr_notice("%s: no bc12_active\n", __func__);

	pr_notice("%s: bc12_active:%d\n", __func__, info->bc12_active);

	if (info->bc12_active) {
		info->ac_psy = power_supply_register(&pdev->dev,
				&info->ac_desc, &info->ac_cfg);

		if (IS_ERR(info->ac_psy)) {
			pr_notice("%s Failed to register power supply: %ld\n",
				__func__, PTR_ERR(info->ac_psy));
			return PTR_ERR(info->ac_psy);
		}

		info->usb_psy = power_supply_register(&pdev->dev,
				&info->usb_desc, &info->usb_cfg);

		if (IS_ERR(info->usb_psy)) {
			pr_notice("%s Failed to register power supply: %ld\n",
				__func__, PTR_ERR(info->usb_psy));
			return PTR_ERR(info->usb_psy);
		}

		info->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
		if (!info->tcpc_dev) {
			pr_notice("%s get tcpc device type_c_port0 fail\n", __func__);
			ret = -ENODEV;
			return ret;
		}

		info->pd_nb.notifier_call = pd_tcp_notifier_call;
		ret = register_tcp_dev_notifier(info->tcpc_dev, &info->pd_nb,
						TCP_NOTIFY_TYPE_ALL);
		if (ret < 0) {
			pr_notice("%s: register tcpc notifer fail\n", __func__);
			ret = -EINVAL;
			return ret;
		}

		INIT_WORK(&info->chr_work, do_charger_detection_work);
		schedule_work(&info->chr_work);

		ret = devm_request_threaded_irq(&pdev->dev,
			platform_get_irq_byname(pdev, "chrdet"), NULL,
			chrdet_int_handler, IRQF_TRIGGER_HIGH, "chrdet", info);
		if (ret < 0)
			pr_notice("%s request chrdet irq fail\n", __func__);
	}

	info->first_connect = true;

	pr_notice("%s: done\n", __func__);

	return 0;
}

static const struct of_device_id mt6357_charger_type_of_match[] = {
	{.compatible = "mediatek,mt6357-charger-type",},
	{},
};

static int mt6357_charger_type_remove(struct platform_device *pdev)
{
	struct mtk_charger_type *info = platform_get_drvdata(pdev);

	if (info)
		devm_kfree(&pdev->dev, info);
	return 0;
}

MODULE_DEVICE_TABLE(of, mt6357_charger_type_of_match);

static struct platform_driver mt6357_charger_type_driver = {
	.probe = mt6357_charger_type_probe,
	.remove = mt6357_charger_type_remove,
	//.shutdown = mt6357_charger_type_shutdown,
	.driver = {
		.name = "mt6357-charger-type-detection",
		.of_match_table = mt6357_charger_type_of_match,
		},
};

static int __init mt6357_charger_type_init(void)
{
	return platform_driver_register(&mt6357_charger_type_driver);
}
module_init(mt6357_charger_type_init);

static void __exit mt6357_charger_type_exit(void)
{
	platform_driver_unregister(&mt6357_charger_type_driver);
}
module_exit(mt6357_charger_type_exit);

MODULE_AUTHOR("wy.chuang <wy.chuang@mediatek.com>");
MODULE_DESCRIPTION("MTK Charger Type Detection Driver");
MODULE_LICENSE("GPL");

