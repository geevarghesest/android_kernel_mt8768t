/* Copyright © 2020, The Lenovo. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/extcon-provider.h>
//#include <linux/usb/tcpm.h>
#include <tcpm.h>
//#include <linux/power/pogo_charger.h>
#include <linux/delay.h>

extern unsigned int ProjectName;
int g_usb_plug_event = 0;
int g_pogo_charge = 0;

enum {
	POGO_CHG_SWITCH_NONE,
	POGO_CHG_SWITCH_USB,
	POGO_CHG_SWITCH_POGO,
};

static const unsigned int pogo_charger_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};


struct pogo_charger {
	struct device *dev;

	/* usb ovp protect */
	int gpio_usb_ovp_int;
	int gpio_usb_ovp_en;
	int gpio_usb_ovp_sel;

	/* charger switch */
	int gpio_usb_en;
	int gpio_usb_int;
	int gpio_pogo_en;
	int gpio_pogo_int;
	int gpio_pogo_ocp_int;
	int gpio_otg_en;
	int gpio_dcdc_en;
	int gpio_ocp_en;

	struct pinctrl *pctrl;
	struct pinctrl_state *ps_default;

	int usb_in;
	int pogo_in;
	int otg_in;

	struct regulator *vbus_reg;
	struct extcon_dev	*extcon;
	struct tcpc_device *tcpc_dev;
	struct notifier_block		nb;

	struct power_supply *batt_psy;
	struct power_supply *usb_psy;
};

static struct pogo_charger *the_pogochg;
ATOMIC_NOTIFIER_HEAD(pogo_chg_notifier);

static int pogo_charger_update_real_type(struct pogo_charger *pogochg);
extern unsigned int board_id_get_ProjectName(void);

/*
 * value: 1 disable vbus out to in
 *        0 enable vbus out to in
 */
static int pogo_charger_otg_en_set(struct pogo_charger *pogochg, int enable)
{
	pr_info("%s: set otg_en to %s\n", __func__, enable?"enable":"disable");
	if(1 == board_id_get_ProjectName()){//smart with pogo
	if (enable) {
		gpio_direction_output(pogochg->gpio_dcdc_en, 1);
		msleep(20);
		gpio_direction_output(pogochg->gpio_ocp_en, 1);
	} else {
		gpio_direction_output(pogochg->gpio_dcdc_en, 0);
		msleep(20);
		gpio_direction_output(pogochg->gpio_ocp_en, 0);
	}
	}
	return 0;
}

int is_only_pogo_contact()
{
	struct pogo_charger *pogochg = the_pogochg;
	int pogo_contact = 0;

	if (!pogochg)
	{
		pr_err("%s: pogochg not initilized!\n", __func__);
		return 0;
	}

	if (!gpio_get_value(pogochg->gpio_pogo_int) &&
			gpio_get_value(pogochg->gpio_usb_int))
	{
		pogo_contact = 1;
	}

	pr_err("%s: pogo_contact:%d\n", __func__, pogo_contact);

	return pogo_contact;
}

int is_usb_contact()
{
	struct pogo_charger *pogochg = the_pogochg;
	int usb_contact = 0;

	if (!pogochg)
	{
		pr_err("%s: pogochg not initilized!\n", __func__);
		return 0;
	}

	if (!gpio_get_value(pogochg->gpio_usb_int))
	{
		usb_contact = 1;
	}

	pr_err("%s: usb_contact:%d\n", __func__, usb_contact);

	return usb_contact;
}

int is_usb_pogo_not_contact()
{
	struct pogo_charger *pogochg = the_pogochg;
	int not_contact = 0;

	if (!pogochg)
	{
		pr_err("%s: pogochg not initilized!\n", __func__);
		return 0;
	}

	if (gpio_get_value(pogochg->gpio_pogo_int) &&
			gpio_get_value(pogochg->gpio_usb_int))
	{
		not_contact = 1;
	}

	pr_err("%s: not_contact:%d\n", __func__, not_contact);

	return not_contact;
}


int pogo_charger_is_pogo_charging(void)
{
	struct pogo_charger *pogochg = the_pogochg;
	int pogo_enabled = 0;

	if (!pogochg) {
		pr_err("pogochg not initilized!\n");
		return 0;
	}

	/* pogo en gpio, 0 is enable, 1 is disable */
	pogo_enabled = !gpio_get_value(pogochg->gpio_pogo_en);
	if (pogochg->pogo_in && pogo_enabled) {
		pr_debug("%s: pogo charging was enabled\n", __func__);
		return 1;
	}

	return 0;
}

int pogo_charger_is_usb_charging(void)
{
	struct pogo_charger *pogochg = the_pogochg;
	int usb_enabled = 0;

	if (!pogochg) {
		pr_err("pogochg not initilized!\n");
		return 0;
	}

	/* usb en gpio, 0 is enable, 1 is disable */
	usb_enabled = !gpio_get_value(pogochg->gpio_usb_en);
	if (pogochg->usb_in && usb_enabled) {
		pr_debug("%s: usb charging was enabled\n", __func__);
		return 1;
	}

	return 0;
}

int pogo_charger_is_otg_exist(void)
{
	struct pogo_charger *pogochg = the_pogochg;

	if (!pogochg) {
		pr_err("pogochg not initilized!\n");
		return 0;
	}

	/* pogo en gpio, 0 is enable, 1 is disable */
	if (pogochg->otg_in) {
		pr_debug("%s: otg exist\n", __func__);
		return 1;
	}

	return 0;
}

static int pogo_charger_switch_update(struct pogo_charger *pogochg)
{
	pr_info("%s: otg_in %d, usb_in %d, pogo_in %d\n",
		__func__, pogochg->otg_in, pogochg->usb_in, pogochg->pogo_in);
	if (pogochg->otg_in) {
		gpio_direction_output(pogochg->gpio_pogo_en, 0);
		gpio_direction_output(pogochg->gpio_usb_en, 1);
		pogo_charger_otg_en_set(pogochg, true);
	} else {
		if (pogochg->usb_in == 1 && pogochg->pogo_in == 0) {
			// only usb insert
			gpio_direction_output(pogochg->gpio_pogo_en, 1);
			msleep(50);
			gpio_direction_output(pogochg->gpio_usb_en, 0);
		} else if (pogochg->usb_in == 0 && pogochg->pogo_in == 1) {
			// only pogo insert
			gpio_direction_output(pogochg->gpio_usb_en, 0);//pogo in, usb open to discharge
			msleep(50);
			gpio_direction_output(pogochg->gpio_pogo_en, 0);
		} else if (pogochg->usb_in == 0 && pogochg->pogo_in == 0) {
			// no insert
			gpio_direction_output(pogochg->gpio_usb_en, 1);
			msleep(50);
			gpio_direction_output(pogochg->gpio_pogo_en, 1);
		} else {
			gpio_direction_output(pogochg->gpio_pogo_en, 1);
			msleep(50);
			gpio_direction_output(pogochg->gpio_usb_en, 0);
		}
		pogo_charger_otg_en_set(pogochg, false);
	}

	if (pogochg->usb_in || pogochg->pogo_in) {
		pr_debug("%s: charger exist, stay awake\n", __func__);
		pm_stay_awake(pogochg->dev);
	} else {
		pr_debug("%s: charger remove, relax\n", __func__);
		pm_relax(pogochg->dev);
	}
#if 0
	/* Handle no tcpc */
	if (!pogochg->tcpc_dev) {
		if (pogochg->otg_in) {
			extcon_set_state_sync(pogochg->extcon,
					EXTCON_USB_HOST, true);
		} else if (pogochg->usb_in) {
			extcon_set_state_sync(pogochg->extcon,
					EXTCON_USB, true);
		} else {
			extcon_set_state_sync(pogochg->extcon,
					EXTCON_USB, false);
			extcon_set_state_sync(pogochg->extcon,
					EXTCON_USB_HOST, false);
		}
	}
#endif
	/* update usb psy real type */
	pogo_charger_update_real_type(pogochg);

	return 0;
}

static int pogo_charger_notify_batt(struct pogo_charger *pogochg,
			enum power_supply_property psp, int value)
{
	union power_supply_propval val = {0, };

	if (!pogochg->batt_psy) {
		pogochg->batt_psy = power_supply_get_by_name("battery");
		if (!pogochg->batt_psy) {
			pr_err("%s: failed to get battery psy\n", __func__);
			return -1;
		}
	}

	val.intval = value;
	return power_supply_set_property(pogochg->batt_psy, psp, &val);
}

static int pogo_charger_update_real_type(struct pogo_charger *pogochg)
{
	union power_supply_propval val = {0, };

	if (!pogochg->usb_psy) {
		pogochg->usb_psy = power_supply_get_by_name("usb");
		if (!pogochg->usb_psy) {
			pr_err("%s: failed to get usb psy\n", __func__);
			return -1;
		}
	}

	/* force usb psy update real type */
	
	if (1 == pogochg->pogo_in)
	val.intval = 23;
	else //if (1 == pogochg->usb_in)
	val.intval = 4;
	//else
	//val.intval = 0;
	
	power_supply_set_property(pogochg->usb_psy,
			POWER_SUPPLY_PROP_REAL_TYPE, &val);
	power_supply_changed(pogochg->usb_psy);

	return 0;
}

static int pogo_charger_update_pd_active(struct pogo_charger *pogochg, int active)
{
	union power_supply_propval val;

	if (!pogochg->usb_psy) {
		pogochg->usb_psy = power_supply_get_by_name("usb");
		if (!pogochg->usb_psy) {
			pr_err("%s: failed to get usb psy\n", __func__);
			return -1;
		}
	}

	val.intval = active;
	power_supply_set_property(pogochg->usb_psy,
			POWER_SUPPLY_PROP_PD_ACTIVE, &val);

	return 0;
}

static int pogo_charger_power_role_swap_handle(struct pogo_charger *pogochg,
			int power_role)
{
	pr_info("%s: power role swap\n", __func__);
	return 0;
}

static int pogo_charger_source_vbus_handle(struct pogo_charger *pogochg,
			struct tcp_ny_vbus_state *vbus_state)
{
	int rc = 0;
	bool otg_pwr_en = 0;
	int otg_pwr_enabled = 0;

	otg_pwr_en = (vbus_state->mv) ? true : false;
	otg_pwr_enabled = regulator_is_enabled(pogochg->vbus_reg);
	pr_info("%s: otg_pwr_en %d, otg_pwr_enabled %d, pogochg->otg_in %d\n", __func__, otg_pwr_en, otg_pwr_enabled, pogochg->otg_in);

	/* implement enable otg power or not */
	if (otg_pwr_en && pogochg->otg_in == 0) {
		/* Disable pogo */
		pogochg->otg_in = true;
		pogo_charger_switch_update(pogochg);
	}
	else if (otg_pwr_en == 0 && pogochg->otg_in == 1) {
		/* Disable pogo */
		pogochg->otg_in = false;
		pogo_charger_switch_update(pogochg);
	}
#if 0
	if (otg_pwr_en && !otg_pwr_enabled) {
		rc = regulator_enable(pogochg->vbus_reg);
	} else if (!otg_pwr_en && otg_pwr_enabled) {
		rc = regulator_disable(pogochg->vbus_reg);
	} else {
		pr_err("%s: wrong vbus state, set %d, is_enabled %d\n",
			 __func__, otg_pwr_en, otg_pwr_enabled);
	}

	if (rc) {
		pr_err("%s: failed to control source vbus", __func__);
		return rc;
	}
#endif
	return rc;
}

static int pogo_charger_sink_vbus_handle(struct pogo_charger *pogochg,
						struct tcp_ny_vbus_state *vbus_state)
{
	int rc = 0;
			
	pr_info("%s: mv %d, ma %d\n", __func__, vbus_state->mv, vbus_state->ma);
	// TODO handle the voltage limit
	return rc;
}

// TODO make every event to seperate handler to make it more clear
static int pogo_charger_tcp_notifer_call(struct notifier_block *nb,
			unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct pogo_charger *pogochg = container_of(nb, struct pogo_charger, nb);
	static struct tcp_ny_vbus_state vbus_state;

	switch (event) {
	/* OTG related EVENT */
	case TCP_NOTIFY_SOURCE_VBUS:
		pr_info("%s source vbus = %dmv\n",
					__func__, noti->vbus_state.mv);
		if (1 == board_id_get_ProjectName()) {
			pogo_charger_source_vbus_handle(pogochg, &noti->vbus_state);
		}
		break;

	case TCP_NOTIFY_DR_SWAP:
		pr_info("%s TCP_NOTIFY_DR_SWAP, new role = %d\n",
				__func__, noti->swap_state.new_role);
		break;

	/* CHG related event */
	case TCP_NOTIFY_SINK_VBUS:
		pr_info("%s sink vbus %dmv, %dma\n", __func__,
				noti->vbus_state.mv, noti->vbus_state.ma);
		/* implement sink vbus event */
		pogo_charger_sink_vbus_handle(pogochg, &noti->vbus_state);
		vbus_state = noti->vbus_state;
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		pr_info("%s old_stat = %d, new_stat = %d\n", __func__,
				noti->typec_state.old_state,
				noti->typec_state.new_state);
		if (noti->typec_state.old_state
					== TYPEC_UNATTACHED &&
			(noti->typec_state.new_state
					== TYPEC_ATTACHED_SNK ||
			 noti->typec_state.new_state
					== TYPEC_ATTACHED_CUSTOM_SRC ||
			 noti->typec_state.new_state
					== TYPEC_ATTACHED_NORP_SRC)) {
			pr_info("%s USB Plug in\n", __func__);
		} else if ((noti->typec_state.old_state
					== TYPEC_ATTACHED_SRC ||
			noti->typec_state.old_state
					== TYPEC_ATTACHED_SNK ||
			noti->typec_state.old_state ==
					TYPEC_ATTACHED_CUSTOM_SRC ||
			noti->typec_state.old_state
					== TYPEC_ATTACHED_NORP_SRC) &&
			noti->typec_state.new_state
					== TYPEC_UNATTACHED) {
			pr_info("%s OTG/USB Plug out\n", __func__);
			/* implement OTG/USB plug out */
			//extcon_set_state_sync(lechg->extcon,
			//		EXTCON_USB, false);
			//extcon_set_state_sync(lechg->extcon,
			//		EXTCON_USB_HOST, false);

			pogo_charger_update_pd_active(pogochg, false);

			pogochg->otg_in = false;
			pogo_charger_notify_batt(pogochg,
				POWER_SUPPLY_PROP_USB_OTG, pogochg->otg_in);
			pogo_charger_switch_update(pogochg);
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_SRC &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SNK) {
			pr_info("%s source_to_sink\n", __func__);
			/* implement source_to_sink event */
		} else if (noti->typec_state.new_state == TYPEC_ATTACHED_SRC &&
			noti->typec_state.old_state == TYPEC_ATTACHED_SNK) {
			pr_info("%s sink_to_source\n", __func__);
			/* implement sink_to_source event */
		} else if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			pr_info("%s OTG Plug in\n", __func__);
			/* implement OTG plug in event */
			//extcon_set_state_sync(pogochg->extcon,
			//		EXTCON_USB_HOST, true);

			pogochg->otg_in = true;
			pogo_charger_notify_batt(pogochg,
				POWER_SUPPLY_PROP_USB_OTG, pogochg->otg_in);
			pogo_charger_switch_update(pogochg);
		}
		break;
	case TCP_NOTIFY_PR_SWAP:
		pr_info("%s TCP_NOTIFY_PR_SWAP, new role = %d\n",
				__func__, noti->swap_state.new_role);
		pogo_charger_power_role_swap_handle(pogochg,
				noti->swap_state.new_role);
		break;
	case TCP_NOTIFY_PD_STATE:
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			pogo_charger_update_pd_active(pogochg, false);
			pr_info("%s PD_CONNECT_NONE\n", __func__);
			/* implement PD_CONNECT_NONE event */
			break;
		case PD_CONNECT_PE_READY_SNK:
			pr_info("%s PD_CONNECT_PE_READY_SNK\n", __func__);
			pogo_charger_update_pd_active(pogochg, true);
			/* implement PD_CONNECT_PE_READY_SNK event */
			break;
		case PD_CONNECT_PE_READY_SNK_PD30:
			pr_info("%s PD_CONNECT_PE_READY_SNK_PD30\n", __func__);
			pogo_charger_update_pd_active(pogochg, true);
			/* implement PD_CONNECT_PE_READY_SNK_PD30 event */
			break;
		case PD_CONNECT_PE_READY_SNK_APDO:
			pogo_charger_update_pd_active(pogochg, true);
			pr_info("%s PD_CONNECT_PE_READY_SNK_APDO\n", __func__);
			/* implement PD_CONNECT_PE_READY_SNK_APDO event */
			break;
		case PD_CONNECT_TYPEC_ONLY_SNK:
			pogo_charger_update_pd_active(pogochg, true);
			pr_info("%s PD_CONNECT_TYPEC_ONLY_SNK\n", __func__);
			/* implement PD_CONNECT_TYPEC_ONLY_SNK event */
			break;
		}
		break;
	}
	return NOTIFY_OK;
}

irqreturn_t usb_charger_switch_handler(int irq, void *data)
{
	struct pogo_charger *pogochg = data;
	//int state = 0;
	int usb_int = gpio_get_value(pogochg->gpio_usb_int);
	int pogo_int = gpio_get_value(pogochg->gpio_pogo_int);

	pr_info("%s: usb_int %d, pogo_int %d\n", __func__, usb_int, pogo_int);

	// TODO handle the shake
	pogochg->usb_in = !usb_int;
	pogochg->pogo_in = !pogo_int;
	pogo_charger_switch_update(pogochg);

	/* bit[0:1] 00: usb_int = 0, pogo_int = 0
	 * bit[0:1] 10: usb_int = 1, pogo_int = 0
	 * bit[0:1] 01: usb_int = 0, pogo_int = 1
	 * bit[0:1] 11: usb_int = 1, pogo_int = 1
	 */
	//state = (!!usb_int) << 1 | (!!pogo_int);
	//atomic_notifier_call_chain(&pogo_chg_notifier, state, NULL);

	g_usb_plug_event = 1;

	return IRQ_HANDLED;
}

irqreturn_t pogo_charger_switch_handler(int irq, void *data)
{
	struct pogo_charger *pogochg = data;
	//int state = 0;
	int usb_int = gpio_get_value(pogochg->gpio_usb_int);
	int pogo_int = gpio_get_value(pogochg->gpio_pogo_int);

	pr_info("%s: usb_int %d, pogo_int %d\n", __func__, usb_int, pogo_int);

	// TODO handle the shake
	pogochg->usb_in = !usb_int;
	pogochg->pogo_in = !pogo_int;
	pogo_charger_switch_update(pogochg);

	/* bit[0:1] 00: usb_int = 0, pogo_int = 0
	 * bit[0:1] 10: usb_int = 1, pogo_int = 0
	 * bit[0:1] 01: usb_int = 0, pogo_int = 1
	 * bit[0:1] 11: usb_int = 1, pogo_int = 1
	 */
	//state = (!!usb_int) << 1 | (!!pogo_int);
	//atomic_notifier_call_chain(&pogo_chg_notifier, state, NULL);

	return IRQ_HANDLED;
}

irqreturn_t pogo_charger_otg_ocp_handler(int irq, void *data)
{
	struct pogo_charger *pogochg = data;
	//int state = 0;
	int pogo_ocp_int = gpio_get_value(pogochg->gpio_pogo_ocp_int);

	pr_info("%s: pogo_ocp_int %d\n", __func__, pogo_ocp_int);
	if (1==pogo_ocp_int){//no ocp
		gpio_direction_output(pogochg->gpio_ocp_en, 1); // en ocp_en
	}else{
		gpio_direction_output(pogochg->gpio_ocp_en, 0); // dis ocp_en
	}
	
	return IRQ_HANDLED;
}

static int pogo_charger_switcher_init(struct pogo_charger *pogochg)
{
	int rc = 0;
#if 0
	if (!pogochg->pctrl) {
		pogochg->pctrl = devm_pinctrl_get(pogochg->dev);
		if (IS_ERR_OR_NULL(pogochg->pctrl)) {
			pr_err("pinctrl get failed\n");
			return PTR_ERR(pogochg->pctrl);
		}
	}

	pogochg->ps_default = pinctrl_lookup_state(pogochg->pctrl,
				"pogo_sw_default");
	if (IS_ERR_OR_NULL(pogochg->ps_default)) {
		pr_err("Failed to get pinctrl state\n");
		devm_pinctrl_put(pogochg->pctrl);
		return -EINVAL;
	}
	pinctrl_select_state(pogochg->pctrl, pogochg->ps_default);
#endif
	pogochg->gpio_usb_en =
		of_get_named_gpio(pogochg->dev->of_node, "pogo,usb-en", 0);
	pogochg->gpio_usb_int =
		of_get_named_gpio(pogochg->dev->of_node, "pogo,usb-int", 0);
	pogochg->gpio_pogo_en =
		of_get_named_gpio(pogochg->dev->of_node, "pogo,pogo-en", 0);
	pogochg->gpio_pogo_int =
		of_get_named_gpio(pogochg->dev->of_node, "pogo,pogo-int", 0);
	pogochg->gpio_pogo_ocp_int =
		of_get_named_gpio(pogochg->dev->of_node, "pogo,pogo-ocp-int", 0);
	pogochg->gpio_otg_en =
		of_get_named_gpio(pogochg->dev->of_node, "pogo,otg-en", 0);
	pogochg->gpio_dcdc_en =
		of_get_named_gpio(pogochg->dev->of_node, "pogo,dcdc-en", 0);
	pogochg->gpio_ocp_en =
		of_get_named_gpio(pogochg->dev->of_node, "pogo,ocp-en", 0);

	if (pogochg->gpio_usb_en < 0 ||
		pogochg->gpio_usb_int < 0 ||
		pogochg->gpio_pogo_en < 0 ||
		pogochg->gpio_pogo_int < 0 ||
		pogochg->gpio_pogo_ocp_int < 0) {
		pr_err("%s: Not support charger switcher\n", __func__);
		return -ENODEV;
	}
	pr_info("%s: pogochg->gpio_usb_en %d, pogochg->gpio_usb_int %d\n", __func__, pogochg->gpio_usb_en, pogochg->gpio_usb_int);
	pr_info("%s: pogochg->gpio_pogo_en %d, pogochg->gpio_pogo_int %d\n", __func__, pogochg->gpio_pogo_en, pogochg->gpio_pogo_int);
	pr_info("%s: pogochg->gpio_otg_en %d, pogochg->gpio_pogo_ocp_int %d\n", __func__, pogochg->gpio_otg_en, pogochg->gpio_pogo_ocp_int);
	pr_info("%s: pogochg->gpio_dcdc_en %d, pogochg->gpio_ocp_en %d\n", __func__, pogochg->gpio_dcdc_en, pogochg->gpio_ocp_en);

	gpio_request(pogochg->gpio_usb_en, "usb_en");
	gpio_request(pogochg->gpio_usb_int, "usb_int");
	gpio_request(pogochg->gpio_pogo_en, "pogo_en");
	gpio_request(pogochg->gpio_pogo_int, "pogo_int");
	gpio_request(pogochg->gpio_pogo_ocp_int, "pogo_ocp_int");
	gpio_request(pogochg->gpio_otg_en, "otg_en");
	gpio_request(pogochg->gpio_dcdc_en, "dcdc_en");
	gpio_request(pogochg->gpio_ocp_en, "ocp_en");

	/* init usb and pogo */
	gpio_direction_output(pogochg->gpio_usb_en, 0); // enable usb
	gpio_direction_output(pogochg->gpio_pogo_en, 0); // disable pogo

	gpio_direction_output(pogochg->gpio_otg_en, 0); //dis otg
	gpio_direction_output(pogochg->gpio_dcdc_en, 0); // dis dcdc
	gpio_direction_output(pogochg->gpio_ocp_en, 0); // dis ocp

	/* request irq */
	rc = devm_request_threaded_irq(pogochg->dev,
			gpio_to_irq(pogochg->gpio_usb_int), NULL,
			usb_charger_switch_handler,
			IRQF_ONESHOT | IRQF_TRIGGER_FALLING |
			IRQF_TRIGGER_RISING,
			"usb int", pogochg);
	if (rc < 0) {
		pr_err("%s: Couldn't request usb_int irq %d\n", __func__,
			gpio_to_irq(pogochg->gpio_usb_int));
		return rc;
	}

	rc = devm_request_threaded_irq(pogochg->dev,
			gpio_to_irq(pogochg->gpio_pogo_int), NULL,
			pogo_charger_switch_handler,
			IRQF_ONESHOT | IRQF_TRIGGER_FALLING |
			IRQF_TRIGGER_RISING,
			"pogo int", pogochg);
	if (rc < 0) {
		pr_err("%s: Couldn't request pogo_int irq %d\n", __func__,
			gpio_to_irq(pogochg->gpio_pogo_int));
		return rc;
	}

	rc = devm_request_threaded_irq(pogochg->dev,
			gpio_to_irq(pogochg->gpio_pogo_ocp_int), NULL,
			pogo_charger_otg_ocp_handler,
			IRQF_ONESHOT | IRQF_TRIGGER_FALLING |
			IRQF_TRIGGER_RISING,
			"pogo ocp int", pogochg);
	if (rc < 0) {
		pr_err("%s: Couldn't request pogo_ocp_int irq %d\n", __func__,
			gpio_to_irq(pogochg->gpio_pogo_ocp_int));
		return rc;
	}

	/* update init state */
	//pogo_charger_switch_handler(gpio_to_irq(pogochg->gpio_usb_int), pogochg);

	enable_irq_wake(gpio_to_irq(pogochg->gpio_usb_int));
	enable_irq_wake(gpio_to_irq(pogochg->gpio_pogo_int));
	enable_irq_wake(gpio_to_irq(pogochg->gpio_pogo_ocp_int));

	return 0;
}

static void pogo_charger_disable_interrupts(struct pogo_charger *pogochg)
{
	if (pogochg->gpio_usb_ovp_int > 0) {
		disable_irq(gpio_to_irq(pogochg->gpio_usb_ovp_int));
	}

	if (pogochg->gpio_usb_int > 0) {
		disable_irq(gpio_to_irq(pogochg->gpio_usb_int));
	}

	if (pogochg->gpio_pogo_int > 0) {
		disable_irq(gpio_to_irq(pogochg->gpio_pogo_int));
	}
}

static int pogo_charger_deinit(struct pogo_charger *pogochg)
{
	pr_info("%s: deinit\n", __func__);
	return 0;
}

static int pogo_charger_probe(struct platform_device *pdev)
{
	struct pogo_charger *pogochg;
	int rc = 0;

	pr_info("%s: start\n", __func__);

	pogochg = devm_kzalloc(&pdev->dev, sizeof(*pogochg), GFP_KERNEL);
	if (!pogochg)
		return -ENOMEM;

	pogochg->dev = &pdev->dev;

	platform_set_drvdata(pdev, pogochg);
#if 1
	/* init vbus regulator */
	pogochg->vbus_reg = devm_regulator_get_optional(pogochg->dev,
				"usb-otg-vbus");
	if (IS_ERR(pogochg->vbus_reg)) {
		/* regulators may not be ready, so retry again later */
		pr_info("%s: get vbus regulator failed, retry later\n",
				 __func__);
		pogochg->vbus_reg = NULL;
		rc = -EPROBE_DEFER;
		goto cleanup;
	}
	pr_info("%s: get vbus regulator success\n", __func__);
#endif
	pogochg->batt_psy = power_supply_get_by_name("battery");
	pogochg->usb_psy = power_supply_get_by_name("usb");
#if 0
	rc = pogo_charger_ovp_protect_init(pogochg);
	if (rc) {
		pr_err("Failed in ovp_protect init rc=%d\n", rc);
		goto cleanup;
	}
#endif
#if 1
	pogochg->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!pogochg->tcpc_dev) {
		pr_err("%s: Failed to get tcpc_dev\n", __func__);
	} else {
		pogochg->nb.notifier_call = pogo_charger_tcp_notifer_call;
//		rc = register_tcp_dev_notifier(pogochg->tcpc_dev, &pogochg->nb,
//			TCP_NOTIFY_TYPE_VBUS | TCP_NOTIFY_TYPE_USB);
		rc = register_tcp_dev_notifier(pogochg->tcpc_dev,
				&pogochg->nb, TCP_NOTIFY_TYPE_ALL);
		if (rc < 0) {
			pr_err("%s: Failed to register_tcp_dev_notifier\n",
					__func__);
			rc = -ENOMEM;
			goto cleanup;
		}
		pr_info("%s: register_tcp_dev_notifier done\n", __func__);
	}
#endif
#if 0
	/* extcon registration */
	pogochg->extcon = devm_extcon_dev_allocate(pogochg->dev,
			pogo_charger_extcon_cable);
	if (IS_ERR(pogochg->extcon)) {
		rc = PTR_ERR(pogochg->extcon);
		pr_err("failed to allocate extcon device rc=%d\n", rc);
		goto cleanup;
	}

	rc = devm_extcon_dev_register(pogochg->dev, pogochg->extcon);
	if (rc < 0) {
		dev_err(pogochg->dev, "failed to register extcon device rc=%d\n",
				rc);
		goto cleanup;
	}
#endif

	//device_init_wakeup(pogochg->dev, true);

	rc = pogo_charger_switcher_init(pogochg);
	if (rc < 0) {
		pr_err("Failed in switcher init\n");
		goto cleanup;
	}

	the_pogochg = pogochg;
	g_pogo_charge = is_only_pogo_contact();
	g_usb_plug_event = is_usb_contact();

	pr_info("%s: end\n", __func__);
	return 0;

cleanup:
	if (pogochg->tcpc_dev) {
		pr_info("%s: unregister_tcp_dev_notifier\n", __func__);
		unregister_tcp_dev_notifier(pogochg->tcpc_dev,
					&pogochg->nb, TCP_NOTIFY_TYPE_ALL);
	}
	pogo_charger_deinit(pogochg);
	platform_set_drvdata(pdev, NULL);
	return rc;
}

static int pogo_charger_remove(struct platform_device *pdev)
{
	struct pogo_charger *pogochg = platform_get_drvdata(pdev);

	pogo_charger_deinit(pogochg);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void pogo_charger_shutdown(struct platform_device *pdev)
{
	struct pogo_charger *pogochg = platform_get_drvdata(pdev);

	/* disable all interrupts */
	pogo_charger_disable_interrupts(pogochg);
}

static const struct of_device_id match_table[] = {
	{ .compatible = "lc,pogo_charger", },
	{ },
};

static struct platform_driver pogo_charger_driver = {
	.driver = {
		.name = "lc,pogo_charger",
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
	.probe = pogo_charger_probe,
	.remove = pogo_charger_remove,
	.shutdown = pogo_charger_shutdown,
};

module_platform_driver(pogo_charger_driver);

MODULE_DESCRIPTION("LongCheer         	  pogo Charger Driver");
MODULE_LICENSE("GPL v2");

