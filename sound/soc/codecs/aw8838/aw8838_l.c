/*
 * aw8838_l.c   aw8838_l codec module
 *
 * Version: v1.0.1
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/syscalls.h>
#include <sound/tlv.h>
#include "aw8838_l.h"
#include "aw8838_reg_l.h"

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define aw8838_l_I2C_NAME "aw8838_l_smartpa"
//#define aw8838_l_I2C_NAME "i2c_smartpa"

#define aw8838_l_VERSION "v1.0.1"

#define aw8838_l_RATES SNDRV_PCM_RATE_8000_48000
#define aw8838_l_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
                                    SNDRV_PCM_FMTBIT_S24_LE | \
                                    SNDRV_PCM_FMTBIT_S32_LE)

#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 5
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 5
#define AW_1000_US			(1000)
#define AW_VOLUME_STEP_DB		(12*2)
#define AW_FADE_OUT_TARGET_VOL_50db  	(132)
#define AW_FADE_OUT_TARGET_VOL_0db 		(0)


static int aw8838_l_spk_control = 0;
static int aw8838_l_rcv_control = 0;
static unsigned int g_fade_in_time = AW_1000_US / 10;
static unsigned int g_fade_out_time = AW_1000_US >> 1;
static unsigned int g_fade_step = AW_VOLUME_STEP_DB;
static unsigned int g_spin = 0; //0-L 1-R


#define aw8838_l_MAX_FIRMWARE_LOAD_CNT 20
static char *aw8838_l_cfg_name = "aw8838_l_cfg.bin";

#ifdef AW_KERNEL_VER_OVER_4_19_1
 static struct aw_componet_codec_ops aw_componet_codec_ops = {
	 .kcontrol_codec = snd_soc_kcontrol_component,
	 .codec_get_drvdata = snd_soc_component_get_drvdata,
	 .add_codec_controls = snd_soc_add_component_controls,
	 .unregister_codec = snd_soc_unregister_component,
	 .register_codec = snd_soc_register_component,
 };
#else 
 static struct aw_componet_codec_ops aw_componet_codec_ops = {
	 .kcontrol_codec = snd_soc_kcontrol_codec,
	 .codec_get_drvdata = snd_soc_codec_get_drvdata,
	 .add_codec_controls = snd_soc_add_codec_controls,
	 .unregister_codec = snd_soc_unregister_codec,
	 .register_codec = snd_soc_register_codec,
 };
#endif

static aw_snd_soc_codec_t *aw_get_codec(struct snd_soc_dai *dai)
{
#ifdef AW_KERNEL_VER_OVER_4_19_1
	return dai->component;
#else
	return dai->codec;
#endif
}
 /******************************************************
 *
 * aw8838_l i2c write/read
 *
 ******************************************************/
static int i2c_write(struct aw8838_l *aw8838_l,
        unsigned char addr, unsigned int reg_data)
{
    int ret = -1;
    u8 wbuf[512] = {0};

    struct i2c_msg msgs[] = {
        {
            .addr   = aw8838_l->i2c->addr,
            .flags  = 0,
            .len    = 3,
            .buf    = wbuf,
        },
    };

    wbuf[0] = addr;
    wbuf[1] = (unsigned char)((reg_data & 0xff00)>>8);
    wbuf[2] = (unsigned char)(reg_data & 0x00ff);

    ret = i2c_transfer(aw8838_l->i2c->adapter, msgs, 1);
    if (ret < 0) {
        pr_err("%s: i2c write error: %d\n", __func__, ret);
    }

    return ret;
}

static int i2c_read(struct aw8838_l *aw8838_l,
        unsigned char addr, unsigned int *reg_data)
{
    int ret = -1;
    unsigned char rbuf[512] = {0};
    unsigned int get_data = 0;

    struct i2c_msg msgs[] = {
        {
            .addr   = aw8838_l->i2c->addr,
            .flags  = 0,
            .len    = 1,
            .buf    = &addr,
        },
        {
            .addr   = aw8838_l->i2c->addr,
            .flags  = I2C_M_RD,
            .len    = 2,
            .buf    = rbuf,
        },
    };

    ret = i2c_transfer(aw8838_l->i2c->adapter, msgs, 2);
    if (ret < 0) {
        pr_err("%s: i2c read error: %d\n", __func__, ret);
        return ret;
    }

    get_data = (unsigned int)(rbuf[0] & 0x00ff);
    get_data <<= 8;
    get_data |= (unsigned int)rbuf[1];

    *reg_data = get_data;

    return ret;
}

static int aw8838_l_i2c_write(struct aw8838_l *aw8838_l,
        unsigned char reg_addr, unsigned int reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
    ret = i2c_write(aw8838_l, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
        cnt ++;
    }

    return ret;
}

static int aw8838_l_i2c_read(struct aw8838_l *aw8838_l,
        unsigned char reg_addr, unsigned int *reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
    ret = i2c_read(aw8838_l, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
        cnt ++;
    }

    return ret;
}

static int aw8838_l_i2c_write_bits(struct aw8838_l *aw8838_l,
        unsigned char reg_addr, unsigned int mask, unsigned int reg_data)
{
    unsigned int reg_val;

    aw8838_l_i2c_read(aw8838_l, reg_addr, &reg_val);
    reg_val &= mask;
    reg_val |= reg_data;
    aw8838_l_i2c_write(aw8838_l, reg_addr, reg_val);

    return 0;
}

/******************************************************
 *
 * aw8838_l control
 *
 ******************************************************/
static void aw8838_l_run_mute(struct aw8838_l *aw8838_l, bool mute)
{
    pr_debug("%s enter\n", __func__);

    if(mute) {
        aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_PWMCTRL,
                aw8838_l_BIT_PWMCTRL_HMUTE_MASK, aw8838_l_BIT_PWMCTRL_HMUTE_ENABLE);
    } else {
        aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_PWMCTRL,
                aw8838_l_BIT_PWMCTRL_HMUTE_MASK, aw8838_l_BIT_PWMCTRL_HMUTE_DISABLE);
    }
}

static void aw8838_l_run_pwd(struct aw8838_l *aw8838_l, bool pwd)
{
    pr_debug("%s enter\n", __func__);

	if(pwd) {
		aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_SYSCTRL,
                aw8838_l_BIT_SYSCTRL_CP_MASK, aw8838_l_BIT_SYSCTRL_CP_PDN);
        aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_SYSCTRL,
                aw8838_l_BIT_SYSCTRL_PW_MASK, aw8838_l_BIT_SYSCTRL_PW_PDN);
    } else {
		aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_SYSCTRL,
                aw8838_l_BIT_SYSCTRL_PW_MASK, aw8838_l_BIT_SYSCTRL_PW_ACTIVE);
    	aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_SYSCTRL,
                aw8838_l_BIT_SYSCTRL_CP_MASK, aw8838_l_BIT_SYSCTRL_CP_ACTIVE);
    }
}

static void aw8838_l_spk_rcv_mode(struct aw8838_l *aw8838_l)
{
    pr_info("%s spk_rcv=%d\n", __func__, aw8838_l->spk_rcv_mode);

    if(aw8838_l->spk_rcv_mode == aw8838_l_SPEAKER_MODE) {
        aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_SYSCTRL,
                aw8838_l_BIT_SYSCTRL_MODE_MASK, aw8838_l_BIT_SYSCTRL_SPK_MODE);
    } else if(aw8838_l->spk_rcv_mode == aw8838_l_RECEIVER_MODE){
        aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_SYSCTRL,
                aw8838_l_BIT_SYSCTRL_MODE_MASK, aw8838_l_BIT_SYSCTRL_RCV_MODE);
    } else {
    }
}


static void aw8838_l_start(struct aw8838_l *aw8838_l)
{
    unsigned int reg_val = 0;
    unsigned int iis_check_max = 5;
    unsigned int i = 0;

    pr_info("%s enter\n", __func__);

    aw8838_l_run_pwd(aw8838_l, false);
    msleep(2);
    for(i=0; i<iis_check_max; i++) {
        aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_SYSST, &reg_val);
        if(reg_val & aw8838_l_BIT_SYSST_PLLS) {
            aw8838_l_run_mute(aw8838_l, false);
            pr_debug("%s iis signal check pass!\n", __func__);
            return;
        }
        msleep(2);
    }
    aw8838_l_run_pwd(aw8838_l, true);
    pr_err("%s: iis signal check error\n", __func__);
}
static void aw8838_l_stop(struct aw8838_l *aw8838_l)
{
    pr_debug("%s enter\n", __func__);

    aw8838_l_run_mute(aw8838_l, true);
	aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_CPCTRL,
                aw8838_l_BIT_CPCTRL_HMUTE_MASK, aw8838_l_BIT_CPCTRL_HMUTE_DISABLE);
    aw8838_l_run_pwd(aw8838_l, true);
}

static void aw8838_l_container_update(struct aw8838_l *aw8838_l,
        struct aw8838_l_container *aw8838_l_cont)
{
    int i = 0;
    int reg_addr = 0;
    int reg_val = 0;

    pr_debug("%s enter\n", __func__);

    for(i=0; i<aw8838_l_cont->len; i+=4) {
        reg_addr = (aw8838_l_cont->data[i+1]<<8) + aw8838_l_cont->data[i+0];
        reg_val = (aw8838_l_cont->data[i+3]<<8) + aw8838_l_cont->data[i+2];
        pr_info("%s: reg=0x%04x, val = 0x%04x\n", __func__, reg_addr, reg_val);
        aw8838_l_i2c_write(aw8838_l, (unsigned char)reg_addr, (unsigned int)reg_val);
    }

    pr_debug("%s exit\n", __func__);
}

static void aw8838_l_cfg_loaded(const struct firmware *cont, void *context)
{
    struct aw8838_l *aw8838_l = context;
    struct aw8838_l_container *aw8838_l_cfg;
    unsigned int i = 0;

    if (!cont) {
        pr_err("%s: failed to read %s\n", __func__, aw8838_l_cfg_name);
        release_firmware(cont);
        return;
    }

    pr_info("%s: loaded %s - size: %zu\n", __func__, aw8838_l_cfg_name,
            cont ? cont->size : 0);

    for(i=0; i<cont->size; i++) {
        pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
    }

    aw8838_l_cfg = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
    if (!aw8838_l_cfg) {
        release_firmware(cont);
        pr_err("%s: error allocating memory\n", __func__);
        return;
    }
    aw8838_l_cfg->len = cont->size;
    memcpy(aw8838_l_cfg->data, cont->data, cont->size);
    release_firmware(cont);

    aw8838_l_container_update(aw8838_l, aw8838_l_cfg);

    kfree(aw8838_l_cfg);

    aw8838_l->init = 1;
    pr_info("%s: cfg update complete\n", __func__);

    aw8838_l_spk_rcv_mode(aw8838_l);
    aw8838_l_start(aw8838_l);
}

static int aw8838_l_load_cfg(struct aw8838_l *aw8838_l)
{
    pr_info("%s enter\n", __func__);

    return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
            aw8838_l_cfg_name, aw8838_l->dev, GFP_KERNEL,
            aw8838_l, aw8838_l_cfg_loaded);
}

static void aw8838_l_cold_start(struct aw8838_l *aw8838_l)
{
    int ret = -1;

    pr_info("%s enter\n", __func__);

    ret = aw8838_l_load_cfg(aw8838_l);
    if(ret) {
        pr_err("%s: cfg loading requested failed: %d\n", __func__, ret);
    }
}

static void aw8838_l_speaker_cfg(struct aw8838_l *aw8838_l, bool flag)
{
    pr_info("%s, flag = %d\n", __func__, flag);

    if(flag == true) {
        if(aw8838_l->init == 0) {
            pr_info("%s, init = %d\n", __func__, aw8838_l->init);
            aw8838_l_cold_start(aw8838_l);
        } else {
            aw8838_l_spk_rcv_mode(aw8838_l);
            aw8838_l_start(aw8838_l);
        }
    } else {
        aw8838_l_stop(aw8838_l);
    }
}
static void aw8838_l_receiver_cfg(struct aw8838_l *aw8838_l, bool flag)
{
    pr_info("%s, flag = %d\n", __func__, flag);

    if(flag == true) {
        if(aw8838_l->init == 0) {
            pr_info("%s, init = %d\n", __func__, aw8838_l->init);
            aw8838_l_cold_start(aw8838_l);
        } else {
            aw8838_l_spk_rcv_mode(aw8838_l);
            aw8838_l_start(aw8838_l);
        }
    } else {
        aw8838_l_stop(aw8838_l);
    }
}

/******************************************************
 *
 * kcontrol
 *
 ******************************************************/
 static const char *const spk_function[] = { "Off", "On" };
 static const char *const rcv_function[] = { "Off", "On" };
 static const char *const aw8838_l_spin[] = { "L2R", "R2L" };
 static const DECLARE_TLV_DB_SCALE(digital_gain,0,50,0);

 struct soc_mixer_control aw8838_l_mixer ={
    .reg    = aw8838_l_REG_HAGCCFG8,
    .shift  = aw8838_l_VOL_REG_SHIFT,
    .max    = aw8838_l_VOLUME_MAX,
    .min    = aw8838_l_VOLUME_MIN,
 };

int aw8838_l_get_volume(struct aw8838_l *aw8838_l)
{
		unsigned int reg_value = 0;
		unsigned int  value = 0;

		
	    aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_HAGCCFG8, &reg_value);
        value = (reg_value >> 8)&(aw8838_l_BIT_HAGCCFG8_VOL_MASK);
		return value;
}

int aw8838_l_set_volume(struct aw8838_l *aw8838_l, uint32_t value)
{
    unsigned int reg_value = 0;
//cal real value
    value = (value&aw8838_l_BIT_HAGCCFG8_VOL_MASK)<<8;
    aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_HAGCCFG8, &reg_value);
    value = value | (reg_value&0x00ff);

    //write value
    aw8838_l_i2c_write(aw8838_l, aw8838_l_REG_HAGCCFG8, value);


	return 0;
}


static void aw8838_l_fade_in(struct aw8838_l *aw8838_l)
{
	int i = 0;

	if (0)
		return;

	if (g_fade_step == 0 || g_fade_in_time == 0) {
		aw8838_l_set_volume(aw8838_l, AW_FADE_OUT_TARGET_VOL_0db); //xxdb->0db
		return;
	}

	/*volume up*/
	for (i = AW_FADE_OUT_TARGET_VOL_50db; i >= AW_FADE_OUT_TARGET_VOL_0db; i -= g_fade_step) {
		if (i < AW_FADE_OUT_TARGET_VOL_0db)
			i = AW_FADE_OUT_TARGET_VOL_0db;

		aw8838_l_set_volume(aw8838_l, i);
		usleep_range(g_fade_in_time, g_fade_in_time + 10);
	}

	if (i != AW_FADE_OUT_TARGET_VOL_0db)
		aw8838_l_set_volume(aw8838_l, AW_FADE_OUT_TARGET_VOL_0db);

}

static void aw8838_l_fade_out(struct aw8838_l *aw8838_l)
{
	int i = 0;
	//unsigned start_volume = 0;

	if (0)
		return;

	if (g_fade_step == 0 || g_fade_out_time == 0) {
		 pr_info("%s, 0db to -50db\n", __func__);
		aw8838_l_set_volume(aw8838_l, AW_FADE_OUT_TARGET_VOL_50db);
		return;
	}

	i = aw8838_l_get_volume(aw8838_l);
	pr_info("%s, g_volume = %d \n", __func__,i);
	for (i; i <= AW_FADE_OUT_TARGET_VOL_50db; i += g_fade_step) {
		if (i > (int32_t)AW_FADE_OUT_TARGET_VOL_50db)
			i = (int32_t)AW_FADE_OUT_TARGET_VOL_50db;
		pr_info("%s, fade_out_volume = %d \n", __func__,i);
		aw8838_l_set_volume(aw8838_l, i);
		usleep_range(g_fade_out_time, g_fade_out_time + 10);
	}
	if (i != AW_FADE_OUT_TARGET_VOL_50db) {
		aw8838_l_set_volume(aw8838_l, AW_FADE_OUT_TARGET_VOL_50db);
		usleep_range(g_fade_out_time, g_fade_out_time + 10);
	}
}

void aw8838_l_L2R(struct aw8838_l *aw8838_l)
{
	pr_info("%s enter\n", __func__);
	aw8838_l_fade_out(aw8838_l);
	aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_SYSCTRL,
                aw8838_l_BIT_SYSCTRL_WSINV_MASK, aw8838_l_BIT_SYSCTRL_WS_NO_INVERT);
	aw8838_l_fade_in(aw8838_l);
	pr_info("%s out\n", __func__);
}

void aw8838_l_R2L(struct aw8838_l *aw8838_l)
{
	pr_info("%s enter\n", __func__);
	aw8838_l_fade_out(aw8838_l);
	aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_SYSCTRL,
                aw8838_l_BIT_SYSCTRL_WSINV_MASK, aw8838_l_BIT_SYSCTRL_WS_INVERT);
	aw8838_l_fade_in(aw8838_l);
	pr_info("%s out\n", __func__);
}


static int aw8838_l_volume_info(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_info *uinfo)
{
    struct soc_mixer_control *mc = (struct soc_mixer_control*) kcontrol->private_value;

    //set kcontrol info
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = mc->max - mc->min;
    return 0;
}

static int aw8838_l_volume_get(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
    aw_snd_soc_codec_t *codec = 
	aw_componet_codec_ops.kcontrol_codec(kcontrol);
    struct aw8838_l *aw8838_l = aw_componet_codec_ops.codec_get_drvdata(codec);
    unsigned int reg_val = 0;
    unsigned int value = 0;
    struct soc_mixer_control *mc = (struct soc_mixer_control*) kcontrol->private_value;

    aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_HAGCCFG8, &reg_val);
    ucontrol->value.integer.value[0] = (value >> mc->shift)\
            &(aw8838_l_BIT_HAGCCFG8_VOL_MASK);
    return 0;
}

static int aw8838_l_volume_put(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
    struct soc_mixer_control *mc = (struct soc_mixer_control*) kcontrol->private_value;
    aw_snd_soc_codec_t *codec = aw_componet_codec_ops.kcontrol_codec(kcontrol);
    struct aw8838_l *aw8838_l = aw_componet_codec_ops.codec_get_drvdata(codec);
    unsigned int value = 0;
    unsigned int reg_value = 0;

    //value is right
    value = ucontrol->value.integer.value[0];
    if(value > (mc->max-mc->min)|| value <0){
      pr_err("%s:value over range \n",__func__);
      return -1;
    }

    //smartpa have clk
    aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_SYSST, &reg_value);
    if(!(reg_value&aw8838_l_BIT_SYSST_PLLS)){
      pr_err("%s: NO I2S CLK ,cat not write reg \n",__func__);
      return 0;
    }
    //cal real value
    value = value << mc->shift&aw8838_l_BIT_HAGCCFG8_VOL_MASK;
    aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_HAGCCFG8, &reg_value);
    value = value | (reg_value&0x00ff);

    //write value
    aw8838_l_i2c_write(aw8838_l, aw8838_l_REG_HAGCCFG8, value);

    return 0;
}

static struct snd_kcontrol_new aw8838_l_volume = {
    .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
    .name  = "aw8838_l_rx_volume",
    .access= SNDRV_CTL_ELEM_ACCESS_TLV_READ|SNDRV_CTL_ELEM_ACCESS_READWRITE,
    .tlv.p  = (digital_gain),
    .info = aw8838_l_volume_info,
    .get =  aw8838_l_volume_get,
    .put =  aw8838_l_volume_put,
    .private_value = (unsigned long)&aw8838_l_mixer,
};

static int aw8838_l_spk_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    pr_debug("%s: aw8838_l_spk_control=%d\n", __func__, aw8838_l_spk_control);
    ucontrol->value.integer.value[0] = aw8838_l_spk_control;
    return 0;
}

static int aw8838_l_spk_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    aw_snd_soc_codec_t *codec = aw_componet_codec_ops.kcontrol_codec(kcontrol);
    struct aw8838_l *aw8838_l = aw_componet_codec_ops.codec_get_drvdata(codec);

    pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n ",
            __func__, ucontrol->value.integer.value[0]);
    if(ucontrol->value.integer.value[0] == aw8838_l_spk_control)
        return 1;

    aw8838_l_spk_control = ucontrol->value.integer.value[0];

    aw8838_l->spk_rcv_mode = aw8838_l_SPEAKER_MODE;

    if(ucontrol->value.integer.value[0]) {
        aw8838_l_speaker_cfg(aw8838_l, true);
    } else {
        aw8838_l_speaker_cfg(aw8838_l, false);
    }

    return 0;
}

static int aw8838_l_rcv_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    pr_debug("%s: aw8838_l_rcv_control=%d\n", __func__, aw8838_l_rcv_control);
    ucontrol->value.integer.value[0] = aw8838_l_rcv_control;
    return 0;
}
static int aw8838_l_rcv_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    aw_snd_soc_codec_t *codec = aw_componet_codec_ops.kcontrol_codec(kcontrol);
    struct aw8838_l *aw8838_l = aw_componet_codec_ops.codec_get_drvdata(codec);
    pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n ",
            __func__, ucontrol->value.integer.value[0]);
    if(ucontrol->value.integer.value[0] == aw8838_l_rcv_control)
        return 1;

    aw8838_l_rcv_control = ucontrol->value.integer.value[0];

    aw8838_l->spk_rcv_mode = aw8838_l_RECEIVER_MODE;

    if(ucontrol->value.integer.value[0]) {
        aw8838_l_receiver_cfg(aw8838_l, true);
    } else {
        aw8838_l_receiver_cfg(aw8838_l, false);
    }
    return 0;
}
static int aw8838_l_spin_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    pr_info("%s: aw8838_l_rcv_control=%d\n", __func__, g_spin);
    ucontrol->value.integer.value[0] = g_spin;
    return 0;
}

static int aw8838_l_spin_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    aw_snd_soc_codec_t *codec = aw_componet_codec_ops.kcontrol_codec(kcontrol);
    struct aw8838_l *aw8838_l = aw_componet_codec_ops.codec_get_drvdata(codec);
    pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n ",
            __func__, ucontrol->value.integer.value[0]);
    if(ucontrol->value.integer.value[0] == g_spin)
        return 1;

    g_spin = ucontrol->value.integer.value[0];

    if(ucontrol->value.integer.value[0]) {
        aw8838_l_R2L(aw8838_l);
    } else {
        aw8838_l_L2R(aw8838_l);
    }
    return 0;
}

static const struct soc_enum aw8838_l_snd_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_function), spk_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rcv_function), rcv_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(aw8838_l_spin), aw8838_l_spin),
};

static struct snd_kcontrol_new aw8838_l_controls[] = {
    SOC_ENUM_EXT("aw8838_l_speaker_switch", aw8838_l_snd_enum[0],
            aw8838_l_spk_get, aw8838_l_spk_set),
    SOC_ENUM_EXT("aw8838_l_receiver_switch", aw8838_l_snd_enum[1],
            aw8838_l_rcv_get, aw8838_l_rcv_set),
    SOC_ENUM_EXT("aw8838_l_spin", aw8838_l_snd_enum[2],
            aw8838_l_spin_get, aw8838_l_spin_set),
};

static void aw8838_l_add_codec_controls(struct aw8838_l *aw8838_l)
{
    pr_info("%s enter\n", __func__);

    aw_componet_codec_ops.add_codec_controls(aw8838_l->codec, aw8838_l_controls,
            ARRAY_SIZE(aw8838_l_controls));

    aw_componet_codec_ops.add_codec_controls(aw8838_l->codec, &aw8838_l_volume,1);
}


/******************************************************
 *
 * Digital Audio Interface
 *
 ******************************************************/
static int aw8838_l_startup(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{
    aw_snd_soc_codec_t *codec = aw_get_codec(dai);
    struct aw8838_l *aw8838_l = 
		aw_componet_codec_ops.codec_get_drvdata(codec);

    pr_info("%s: enter\n", __func__);
    aw8838_l_run_pwd(aw8838_l, false);

    return 0;
}

static int aw8838_l_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    //struct aw8838_l *aw8838_l = aw_componet_codec_ops.codec_get_drvdata(dai->codec);
    aw_snd_soc_codec_t *codec = aw_get_codec(dai);

    pr_info("%s: fmt=0x%x\n", __func__, fmt);

    /* Supported mode: regular I2S, slave, or PDM */
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
    case SND_SOC_DAIFMT_I2S:
        if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
            dev_err(codec->dev, "%s: invalid codec master mode\n", __func__);
            return -EINVAL;
        }
        break;
    default:
        dev_err(codec->dev, "%s: unsupported DAI format %d\n", __func__,
                fmt & SND_SOC_DAIFMT_FORMAT_MASK);
        return -EINVAL;
    }
    return 0;
}

static int aw8838_l_set_dai_sysclk(struct snd_soc_dai *codec_dai,
        int clk_id, unsigned int freq, int dir)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(codec_dai);
    struct aw8838_l *aw8838_l = 
		aw_componet_codec_ops.codec_get_drvdata(codec);

    pr_info("%s: freq=%d\n", __func__, freq);

    aw8838_l->sysclk = freq;
    return 0;
}

static int aw8838_l_hw_params(struct snd_pcm_substream *substream,
    struct snd_pcm_hw_params *params,
    struct snd_soc_dai *dai)
{
    aw_snd_soc_codec_t *codec = aw_get_codec(dai);
    struct aw8838_l *aw8838_l = 
		aw_componet_codec_ops.codec_get_drvdata(codec);
    unsigned int rate = 0;
    int reg_value = 0;
    int width = 0;
    /* Supported */

    //get rate param
    aw8838_l->rate=rate = params_rate(params);
    pr_debug("%s: requested rate: %d, sample size: %d\n", __func__, rate,
            snd_pcm_format_width(params_format(params)));
    //match rate
    switch(rate)
    {
        case 8000:
            reg_value = aw8838_l_BIT_I2SCTRL_SR_8K;
            break;
        case 16000:
            reg_value = aw8838_l_BIT_I2SCTRL_SR_16K;
            break;
        case 32000:
            reg_value = aw8838_l_BIT_I2SCTRL_SR_32K;
            break;
        case 44100:
            reg_value = aw8838_l_BIT_I2SCTRL_SR_44P1K;
            break;
        case 48000:
            reg_value = aw8838_l_BIT_I2SCTRL_SR_48K;
            break;
        default:
            reg_value = aw8838_l_BIT_I2SCTRL_SR_48K;
            pr_err("%s: rate can not support\n", __func__);
            break;
    }
    //set chip rate
    if(-1 != reg_value){
        aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_I2SCTRL,
                aw8838_l_BIT_I2SCTRL_SR_MASK, reg_value);
    }

    //get bit width
    width = params_width(params);
    pr_debug("%s: width = %d \n",__func__,width);
    switch(width)
    {
        case 16:
            reg_value = aw8838_l_BIT_I2SCTRL_FMS_16BIT;
            break;
        case 20:
            reg_value = aw8838_l_BIT_I2SCTRL_FMS_20BIT;
            break;
        case 24:
            reg_value = aw8838_l_BIT_I2SCTRL_FMS_24BIT;
            break;
        case 32:
            reg_value = aw8838_l_BIT_I2SCTRL_FMS_32BIT;
            break;
        default:
            reg_value = aw8838_l_BIT_I2SCTRL_FMS_16BIT;
            pr_err("%s: width can not support\n", __func__);
            break;
    }
    //set width
    if(-1 != reg_value){
        aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_I2SCTRL,
                aw8838_l_BIT_I2SCTRL_FMS_MASK, reg_value);
    }

    return 0;
}

static int aw8838_l_mute(struct snd_soc_dai *dai, int mute, int stream)
{
    aw_snd_soc_codec_t *codec = aw_get_codec(dai);
    struct aw8838_l *aw8838_l = 
		aw_componet_codec_ops.codec_get_drvdata(codec);

    pr_info("%s: mute state=%d\n", __func__, mute);

    if (!(aw8838_l->flags & aw8838_l_FLAG_START_ON_MUTE))
        return 0;

    if (mute) {
        if (stream == SNDRV_PCM_STREAM_PLAYBACK)
            aw8838_l->pstream = 0;
        else
            aw8838_l->cstream = 0;
        if (aw8838_l->pstream != 0 || aw8838_l->cstream != 0)
            return 0;

        aw8838_l_stop(aw8838_l);
    } else {
        if (stream == SNDRV_PCM_STREAM_PLAYBACK)
            aw8838_l->pstream = 1;
        else
            aw8838_l->cstream = 1;
		aw8838_l_i2c_write_bits(aw8838_l, aw8838_l_REG_CPCTRL,
                aw8838_l_BIT_CPCTRL_HMUTE_MASK, aw8838_l_BIT_CPCTRL_HMUTE_ENABLE);
        aw8838_l_start(aw8838_l);
    }

    return 0;
}

static void aw8838_l_shutdown(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{
    aw_snd_soc_codec_t *codec = aw_get_codec(dai);
    struct aw8838_l *aw8838_l = 
		aw_componet_codec_ops.codec_get_drvdata(codec);

    aw8838_l->rate = 0;
    aw8838_l_run_pwd(aw8838_l, true);
}

static const struct snd_soc_dai_ops aw8838_l_dai_ops = {
    .startup = aw8838_l_startup,
    .set_fmt = aw8838_l_set_fmt,
    .set_sysclk = aw8838_l_set_dai_sysclk,
    .hw_params = aw8838_l_hw_params,
    .mute_stream = aw8838_l_mute,
    .shutdown = aw8838_l_shutdown,
};

static struct snd_soc_dai_driver aw8838_l_dai[] = {
    {
        .name = "aw8838_l-aif",
        .id = 1,
        .playback = {
            .stream_name = "Speaker_Playback",
            .channels_min = 1,
            .channels_max = 2,
            .rates = aw8838_l_RATES,
            .formats = aw8838_l_FORMATS,
        },
        .capture = {
            .stream_name = "Speaker_Capture",
            .channels_min = 1,
            .channels_max = 2,
            .rates = aw8838_l_RATES,
            .formats = aw8838_l_FORMATS,
         },
        .ops = &aw8838_l_dai_ops,
        .symmetric_rates = 1,
        .symmetric_channels = 1,
        .symmetric_samplebits = 1,
    },
};

/*****************************************************
 *
 * codec driver
 *
 *****************************************************/
static int aw8838_l_probe(aw_snd_soc_codec_t *codec)
{
    struct aw8838_l *aw8838_l = aw_componet_codec_ops.codec_get_drvdata(codec);
    int ret = 0;

    pr_info("%s enter\n", __func__);

    aw8838_l->codec = codec;

    aw8838_l_add_codec_controls(aw8838_l);

    if (codec->dev->of_node)
        dev_set_name(codec->dev, "%s", "aw8838_l_smartpa");

    pr_info("%s exit\n", __func__);

    return ret;
}

static void aw8838_l_remove(aw_snd_soc_codec_t *codec)
{
    //struct aw8838_l *aw8838_l = aw_componet_codec_ops.codec_get_drvdata(codec);
    pr_info("%s enter\n", __func__);

}


/*static unsigned int aw8838_l_codec_read(aw_snd_soc_codec_t *codec,unsigned int reg)
{
    struct aw8838_l *aw8838_l=aw_componet_codec_ops.codec_get_drvdata(codec);
    unsigned int value =0;
    int ret;
    pr_debug("%s:enter \n",__func__);

    if(aw8838_l_reg_access[reg]&REG_RD_ACCESS){
        ret=aw8838_l_i2c_read(aw8838_l,reg,&value);
    if(ret<0){
        pr_debug("%s: read register failed \n",__func__);
        return ret;
    }
    }else{
        pr_debug("%s:Register 0x%x NO read access\n",__func__,reg);
        return -1;
    }
    return value;
}

static int aw8838_l_codec_write(aw_snd_soc_codec_t *codec,unsigned int reg,unsigned int value)
{
    int ret ;
    struct aw8838_l *aw8838_l=aw_componet_codec_ops.codec_get_drvdata(codec);
    pr_debug("%s:enter ,reg is 0x%x value is 0x%x\n",__func__,reg,value);

    if(aw8838_l_reg_access[reg]&REG_WR_ACCESS){
        ret=aw8838_l_i2c_write(aw8838_l,reg,value);
        return ret;
    }else{
        pr_debug("%s: Register 0x%x NO write access \n",__func__,reg);
    }

    return -1;
}*/


static struct snd_soc_component_driver soc_codec_dev_aw8838_l = {
    .probe = aw8838_l_probe,
    .remove = aw8838_l_remove,
    //.read = aw8838_l_codec_read,
    //.write = aw8838_l_codec_write,
    //.reg_cache_size = aw8838_l_REG_MAX,
    //.reg_word_size = 2,
};


/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw8838_l_interrupt_setup(struct aw8838_l *aw8838_l)
{
    unsigned int reg_val;

    pr_info("%s enter\n", __func__);

    aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_SYSINTM, &reg_val);
    reg_val &= (~aw8838_l_BIT_SYSINTM_UVLOM);
    reg_val &= (~aw8838_l_BIT_SYSINTM_PLLM);
    reg_val &= (~aw8838_l_BIT_SYSINTM_OTHM);
    reg_val &= (~aw8838_l_BIT_SYSINTM_OCDM);
    aw8838_l_i2c_write(aw8838_l, aw8838_l_REG_SYSINTM, reg_val);
}

static void aw8838_l_interrupt_clear(struct aw8838_l *aw8838_l)
{
    unsigned int reg_val = 0;

    pr_info("%s enter\n", __func__);

    aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_SYSST, &reg_val);
    pr_info("%s: reg SYSST=0x%x\n", __func__, reg_val);

    aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_SYSINT, &reg_val);
    pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

    aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_SYSINTM, &reg_val);
    pr_info("%s: reg SYSINTM=0x%x\n", __func__, reg_val);
}

static irqreturn_t aw8838_l_irq(int irq, void *data)
{
    struct aw8838_l *aw8838_l = data;

    pr_info("%s enter\n", __func__);

    aw8838_l_interrupt_clear(aw8838_l);

    pr_info("%s exit\n", __func__);

    return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw8838_l_parse_dt(struct device *dev, struct aw8838_l *aw8838_l,
        struct device_node *np)
{
    aw8838_l->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (aw8838_l->reset_gpio < 0) {
        dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
        return -1;
    } else {
        dev_info(dev, "%s: reset gpio provided ok\n", __func__);
    }
    aw8838_l->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
    if (aw8838_l->irq_gpio < 0) {
        dev_info(dev, "%s: no irq gpio provided.\n", __func__);
    } else {
        dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
    }

    return 0;
}

int aw8838_l_hw_reset(struct aw8838_l *aw8838_l)
{
    pr_info("%s enter\n", __func__);

    if (aw8838_l && gpio_is_valid(aw8838_l->reset_gpio)) {
        gpio_set_value_cansleep(aw8838_l->reset_gpio, 0);
        msleep(1);
        gpio_set_value_cansleep(aw8838_l->reset_gpio, 1);
        msleep(1);
    } else {
        dev_err(aw8838_l->dev, "%s:  failed\n", __func__);
    }
    return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
int aw8838_l_read_chipid(struct aw8838_l *aw8838_l)
{
    int ret = -1;
    unsigned int cnt = 0;
    unsigned int reg = 0;

    while(cnt < AW_READ_CHIPID_RETRIES) {
        ret = aw8838_l_i2c_read(aw8838_l, aw8838_l_REG_ID, &reg);
        if (ret < 0) {
            dev_err(aw8838_l->dev, "%s: failed to read register aw8838_l_REG_ID: %d\n", __func__, ret);
            return -EIO;
        }
        switch (reg) {
        case 0x1730:
            pr_info("%s aw8838_l detected\n", __func__);
            aw8838_l->flags |= aw8838_l_FLAG_SKIP_INTERRUPTS;
            aw8838_l->flags |= aw8838_l_FLAG_START_ON_MUTE;
            aw8838_l->chipid = aw8838_l_ID;
            pr_info("%s aw8838_l->flags=0x%x\n", __func__, aw8838_l->flags);
            return 0;
        default:
            pr_info("%s unsupported device revision (0x%x)\n", __func__, reg );
            break;
        }
        cnt ++;

        msleep(AW_READ_CHIPID_RETRY_DELAY);
    }

    return -EINVAL;
}

/******************************************************
 *
 * sys bin attribute
 *
 *****************************************************/
static ssize_t aw8838_l_reg_write(struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t off, size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct aw8838_l *aw8838_l = dev_get_drvdata(dev);

    if (count != 1) {
        pr_info("invalid register address");
        return -EINVAL;
    }

    aw8838_l->reg = buf[0];

    return 1;
}

static ssize_t aw8838_l_rw_write(struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t off, size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct aw8838_l *aw8838_l = dev_get_drvdata(dev);
    u8 *data;
    int ret;
    int retries = AW_I2C_RETRIES;

    data = kmalloc(count+1, GFP_KERNEL);
    if (data == NULL) {
        pr_err("can not allocate memory\n");
        return  -ENOMEM;
    }

    data[0] = aw8838_l->reg;
    memcpy(&data[1], buf, count);

    retry:
    ret = i2c_master_send(aw8838_l->i2c, data, count+1);
    if (ret < 0) {
        pr_warn("i2c error, retries left: %d\n", retries);
        if (retries) {
              retries--;
              msleep(AW_I2C_RETRY_DELAY);
              goto retry;
        }
    }

    kfree(data);
    return ret;
}

static ssize_t aw8838_l_rw_read(struct file *filp, struct kobject *kobj,
        struct bin_attribute *bin_attr,
        char *buf, loff_t off, size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct aw8838_l *aw8838_l = dev_get_drvdata(dev);
    struct i2c_msg msgs[] = {
        {
            .addr = aw8838_l->i2c->addr,
            .flags = 0,
            .len = 1,
            .buf = &aw8838_l->reg,
        },
        {
            .addr = aw8838_l->i2c->addr,
            .flags = I2C_M_RD,
            .len = count,
            .buf = buf,
        },
    };
    int ret;
    int retries = AW_I2C_RETRIES;
    retry:
    ret = i2c_transfer(aw8838_l->i2c->adapter, msgs, ARRAY_SIZE(msgs));
    if (ret < 0) {
        pr_warn("i2c error, retries left: %d\n", retries);
        if (retries) {
            retries--;
            msleep(AW_I2C_RETRY_DELAY);
            goto retry;
        }
        return ret;
    }
    /* ret contains the number of i2c messages send */
    return 1 + ((ret > 1) ? count : 0);
}

static struct bin_attribute dev_attr_rw = {
    .attr = {
        .name = "rw",
        .mode = S_IRUSR | S_IWUSR,
    },
    .size = 0,
    .read = aw8838_l_rw_read,
    .write = aw8838_l_rw_write,
};

static struct bin_attribute dev_attr_regaddr = {
    .attr = {
        .name = "regaddr",
        .mode = S_IWUSR,
    },
    .size = 0,
    .read = NULL,
    .write = aw8838_l_reg_write,
};

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw8838_l_reg_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct aw8838_l *aw8838_l = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw8838_l_i2c_write(aw8838_l, databuf[0], databuf[1]);
    }

    return count;
}

static ssize_t aw8838_l_reg_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct aw8838_l *aw8838_l = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned char i = 0;
    unsigned int reg_val = 0;
    for(i = 0; i < aw8838_l_REG_MAX; i ++) {
    if(!(aw8838_l_reg_access[i]&REG_RD_ACCESS))
       continue;
        aw8838_l_i2c_read(aw8838_l, i, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%04x \n", i, reg_val);
    }
    return len;
}

static ssize_t aw8838_l_spk_rcv_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct aw8838_l *aw8838_l = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0};

    if(1 == sscanf(buf, "%d", &databuf[0])) {
        aw8838_l->spk_rcv_mode = databuf[0];
    }

    return count;
}

static ssize_t aw8838_l_spk_rcv_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct aw8838_l *aw8838_l = dev_get_drvdata(dev);
    ssize_t len = 0;
    if(aw8838_l->spk_rcv_mode == aw8838_l_SPEAKER_MODE) {
        len += snprintf(buf+len, PAGE_SIZE-len, "aw8838_l spk_rcv: %d, speaker mode\n", aw8838_l->spk_rcv_mode);
    } else if (aw8838_l->spk_rcv_mode == aw8838_l_RECEIVER_MODE) {
        len += snprintf(buf+len, PAGE_SIZE-len, "aw8838_l spk_rcv: %d, receiver mode\n", aw8838_l->spk_rcv_mode);
    } else {
        len += snprintf(buf+len, PAGE_SIZE-len, "aw8838_l spk_rcv: %d, unknown mode\n", aw8838_l->spk_rcv_mode);
    }

    return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw8838_l_reg_show, aw8838_l_reg_store);
static DEVICE_ATTR(spk_rcv, S_IWUSR | S_IRUGO, aw8838_l_spk_rcv_show, aw8838_l_spk_rcv_store);

static struct attribute *aw8838_l_attributes[] = {
    &dev_attr_reg.attr,
    &dev_attr_spk_rcv.attr,
    NULL
};

static struct attribute_group aw8838_l_attribute_group = {
    .attrs = aw8838_l_attributes
};


/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw8838_l_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    struct snd_soc_dai_driver *dai;
    struct aw8838_l *aw8838_l;
    struct device_node *np = i2c->dev.of_node;
    int irq_flags = 0;
    int ret = -1;

    pr_info("%s enter\n", __func__);

    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
        dev_err(&i2c->dev, "check_functionality failed\n");
        return -EIO;
    }

    aw8838_l = devm_kzalloc(&i2c->dev, sizeof(struct aw8838_l), GFP_KERNEL);
    if (aw8838_l == NULL)
        return -ENOMEM;

    aw8838_l->dev = &i2c->dev;
    aw8838_l->i2c = i2c;

    i2c_set_clientdata(i2c, aw8838_l);
    mutex_init(&aw8838_l->cfg_lock);

    /* aw8838_l rst & int */
    if (np) {
        ret = aw8838_l_parse_dt(&i2c->dev, aw8838_l, np);
        if (ret) {
              dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
              goto err;
        }
    } else {
        aw8838_l->reset_gpio = -1;
        aw8838_l->irq_gpio = -1;
    }

    if (gpio_is_valid(aw8838_l->reset_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw8838_l->reset_gpio,
              GPIOF_OUT_INIT_LOW, "aw8838_l_rst");
        if (ret){
              dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
              goto err;
        }
    }

    if (gpio_is_valid(aw8838_l->irq_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw8838_l->irq_gpio,
              GPIOF_DIR_IN, "aw8838_l_int");
        if (ret){
              dev_err(&i2c->dev, "%s: int request failed\n", __func__);
              goto err;
        }
    }

    /* hardware reset */
    aw8838_l_hw_reset(aw8838_l);

    /* aw8838_l chip id */
    ret = aw8838_l_read_chipid(aw8838_l);
    if (ret < 0) {
        dev_err(&i2c->dev, "%s: aw8838_l_read_chipid failed ret=%d\n", __func__, ret);
        return -EIO;
    }

    /* aw8838_l device name */
    if (i2c->dev.of_node) {
        dev_set_name(&i2c->dev, "%s", "aw8838_l_smartpa");
    } else {
        dev_err(&i2c->dev, "%s failed to set device name: %d\n", __func__, ret);
    }

    /* register codec */
    dai = devm_kzalloc(&i2c->dev, sizeof(aw8838_l_dai), GFP_KERNEL);
    if (!dai)
        return -ENOMEM;

    memcpy(dai, aw8838_l_dai, sizeof(aw8838_l_dai));
    pr_info("%s dai->name(%s)\n", __func__, dai->name);

    ret = aw_componet_codec_ops.register_codec(&i2c->dev, &soc_codec_dev_aw8838_l,
            dai, ARRAY_SIZE(aw8838_l_dai));
    if (ret < 0) {
        dev_err(&i2c->dev, "%s failed to register aw8838_l: %d\n", __func__, ret);
        goto err_off;
    }

    /* aw8838_l irq */
    if (gpio_is_valid(aw8838_l->irq_gpio) &&
        !(aw8838_l->flags & aw8838_l_FLAG_SKIP_INTERRUPTS)) {
        aw8838_l_interrupt_setup(aw8838_l);
        /* register irq handler */
        irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
        ret = devm_request_threaded_irq(&i2c->dev,
                          gpio_to_irq(aw8838_l->irq_gpio),
                          NULL, aw8838_l_irq, irq_flags,
                          "aw8838_l", aw8838_l);
        if (ret != 0) {
              dev_err(&i2c->dev, "failed to request IRQ %d: %d\n",
                          gpio_to_irq(aw8838_l->irq_gpio), ret);
              goto err_off;
        }
    } else {
        dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
        /* disable feature support if gpio was invalid */
        aw8838_l->flags |= aw8838_l_FLAG_SKIP_INTERRUPTS;
    }

    /* Register the sysfs files for climax backdoor access */
    ret = device_create_bin_file(&i2c->dev, &dev_attr_rw);
    if (ret)
        dev_info(&i2c->dev, "%s error creating sysfs files: rw\n", __func__);
    ret = device_create_bin_file(&i2c->dev, &dev_attr_regaddr);
    if (ret)
        dev_info(&i2c->dev, "%s error creating sysfs files: regaddr\n", __func__);

    dev_set_drvdata(&i2c->dev, aw8838_l);
    ret = sysfs_create_group(&i2c->dev.kobj, &aw8838_l_attribute_group);
    if (ret < 0) {
        dev_info(&i2c->dev, "%s error creating sysfs attr files\n", __func__);
    }


    pr_info("%s probe completed successfully!\n", __func__);

    return 0;

err_off:

err:
      return ret;
}

static int aw8838_l_i2c_remove(struct i2c_client *i2c)
{
    struct aw8838_l *aw8838_l = i2c_get_clientdata(i2c);

    pr_info("%s enter\n", __func__);

    device_remove_bin_file(&i2c->dev, &dev_attr_regaddr);
    device_remove_bin_file(&i2c->dev, &dev_attr_rw);

    aw_componet_codec_ops.unregister_codec(&i2c->dev);

    if (gpio_is_valid(aw8838_l->irq_gpio))
        devm_gpio_free(&i2c->dev, aw8838_l->irq_gpio);
    if (gpio_is_valid(aw8838_l->reset_gpio)) {
        devm_gpio_free(&i2c->dev, aw8838_l->reset_gpio);
	}
	return 0;
}

static const struct i2c_device_id aw8838_l_i2c_id[] = {
    { aw8838_l_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, aw8838_l_i2c_id);

static struct of_device_id aw8838_l_dt_match[] = {
    { .compatible = "awinic,aw8838_l_smartpa" },
    //{ .compatible = "mediatek,i2c_smartpa" },
    { },
};

static struct i2c_driver aw8838_l_i2c_driver = {
    .driver = {
        .name = aw8838_l_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(aw8838_l_dt_match),
    },
    .probe = aw8838_l_i2c_probe,
    .remove = aw8838_l_i2c_remove,
    .id_table = aw8838_l_i2c_id,
};


static int __init aw8838_l_i2c_init(void)
{
    int ret = 0;

    pr_info("aw8838_l driver version %s\n", aw8838_l_VERSION);

    ret = i2c_add_driver(&aw8838_l_i2c_driver);
    if(ret){
        pr_err("fail to add aw8838_l device into i2c\n");
        return ret;
    }

    return 0;
}
module_init(aw8838_l_i2c_init);


static void __exit aw8838_l_i2c_exit(void)
{
    i2c_del_driver(&aw8838_l_i2c_driver);
}
module_exit(aw8838_l_i2c_exit);


MODULE_DESCRIPTION("ASoC aw8838_l Smart PA Driver");
MODULE_LICENSE("GPL v2");
