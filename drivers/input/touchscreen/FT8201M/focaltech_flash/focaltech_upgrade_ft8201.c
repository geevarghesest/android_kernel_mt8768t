/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_upgrade_ft8201.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-12-29
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "../focaltech_flash.h"

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
u8 pb_file_ft8201[] = {
#include "../include/pramboot/FT8201_Pramboot_V1.6_20180426_le.i"
};

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_MAX_LEN_APP_FT8201    (94 * 1024)

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
/* calculate lcd init code checksum */
static u16 cal_lcdinitcode_checksum(u8 *ptr , int length)
{
    /* CRC16 */
    u16 cfcs = 0;
    int i, j;

    if (length % 2) {
        return 0xFFFF;
    }

    for ( i = 0; i < length; i += 2 ) {
        cfcs ^= ((ptr[i] << 8) + ptr[i + 1]);
        for (j = 0; j < 16; j ++) {
            if (cfcs & 1) {
                cfcs = (u16)((cfcs >> 1) ^ ((1 << 15) + (1 << 10) + (1 << 3)));
            } else {
                cfcs >>= 1;
            }
        }
    }
    return cfcs;
}

/*
 * check_initial_code_valid - check initial code valid or not
 */
static int check_initial_code_valid(u8 *buf)
{
    u16 initcode_checksum = 0;
    u16 buf_checksum = 0;
    u16 hlic_len = 0;

    hlic_len = (u16)(((u16)buf[2]) << 8) + buf[3];
    if ((hlic_len >= FTS_MAX_LEN_SECTOR) || (hlic_len <= FTS_MIN_LEN)) {
        FTS_ERROR("host lcd init code len(%x) is too large", hlic_len);
        return -EINVAL;
    }

    initcode_checksum = cal_lcdinitcode_checksum(buf + 2, hlic_len - 2);
    buf_checksum = ((u16)((u16)buf[0] << 8) + buf[1]);
    FTS_INFO("lcd init code calc checksum:0x%04x,0x%04x", initcode_checksum, buf_checksum);
    if (initcode_checksum != buf_checksum) {
        FTS_ERROR("Initial Code checksum fail");
        return -EINVAL;
    }

    return 0;
}

static bool fts_ft8201_check_ide(u8 *buf, u32 len)
{
    u32 off = 0;

    FTS_INFO("Host FW file IDE version check");
    if (NULL == buf) {
        FTS_ERROR("buf is null fail");
        return false;
    }

    if (len < FTS_MAX_LEN_FILE) {
        FTS_INFO("buf len(%x) abnormal, no IDE", len);
        return false;
    }

    off = upgrade_func_ft8201.paramcfgoff;
    if ((buf[off] == 'I') && (buf[off + 1] == 'D') && (buf[off + 2] == 'E'))
        return true;

    return false;
}

/* fts_ft8201_write_ecc - write and check ecc
 * return 0 if success
 */
static int fts_ft8201_write_ecc(u32 saddr, u8 *buf, u32 len)
{
    int ecc_in_host = 0;
    int ecc_in_tp = 0;

    ecc_in_host = fts_flash_write_buf(saddr, buf, len, 1);
    if (ecc_in_host < 0 ) {
        FTS_ERROR("write buffer to flash fail");
        return ecc_in_host;
    }

    /* ecc */
    ecc_in_tp = fts_fwupg_ecc_cal(saddr, len);
    if (ecc_in_tp < 0 ) {
        FTS_ERROR("ecc read fail");
        return ecc_in_tp;
    }

    FTS_INFO("ecc in tp:%x, host:%x", ecc_in_tp, ecc_in_host);
    if (ecc_in_tp != ecc_in_host) {
        FTS_ERROR("ecc check fail");
        return -EIO;
    }

    return 0;
}

/************************************************************************
 * Name: fts_ft8201_param_flash
 * Brief: param upgrade(erase/write/ecc check)
 * Input: buf - all.bin
 *        len - len of all.bin
 * Output:
 * Return: return 0 if success, otherwise return error code
 ***********************************************************************/
static int fts_ft8201_param_flash(u8 *buf, u32 len)
{
    int ret = 0;
    u8 cmd[2] = { 0 };
    u32 delay = 0;
    u32 start_addr = 0;
    u32 paramcfg_len = 0;
    u8 *tmpbuf = NULL;

    /* erase gesture & parameter sector */
    cmd[0] = FTS_CMD_FLASH_MODE;
    cmd[1] = FLASH_MODE_PARAM_VALUE;
    ret = fts_write(cmd, 2);
    if (ret < 0) {
        FTS_ERROR("upgrade mode(09) cmd write fail");
        goto PARAM_FLASH_ERR;
    }

    delay = FTS_ERASE_SECTOR_DELAY * 2;
    ret = fts_fwupg_erase(delay);
    if (ret < 0) {
        FTS_ERROR("erase cmd write fail");
        goto PARAM_FLASH_ERR;
    }

    /* write flash */
    start_addr = upgrade_func_ft8201.paramcfgoff;
    paramcfg_len = FTS_MAX_LEN_SECTOR;
    tmpbuf = buf + start_addr;
    ret = fts_ft8201_write_ecc(start_addr, tmpbuf, paramcfg_len);
    if (ret < 0 ) {
        FTS_ERROR("parameter configure area write fail");
        goto PARAM_FLASH_ERR;
    }

    start_addr = upgrade_func_ft8201.paramcfg2off;
    paramcfg_len = FTS_MAX_LEN_SECTOR;
    tmpbuf = buf + start_addr;
    ret = fts_ft8201_write_ecc(start_addr, tmpbuf, paramcfg_len);
    if (ret < 0 ) {
        FTS_ERROR("parameter2 configure area write fail");
        goto PARAM_FLASH_ERR;
    }

    return 0;

PARAM_FLASH_ERR:
    return ret;
}

/*
 * fts_get_hlic_ver - read host lcd init code version
 *
 * return 0 if host lcd init code is valid, otherwise return error code
 */
static int fts_ft8201_get_hlic_ver(u8 *initcode)
{
    u8 *hlic_buf = initcode;
    u16 hlic_len = 0;
    u8 hlic_ver[2] = { 0 };

    hlic_len = (u16)(((u16)hlic_buf[2]) << 8) + hlic_buf[3];
    FTS_INFO("host lcd init code len:%x", hlic_len);
    if ((hlic_len >= FTS_MAX_LEN_SECTOR) || (hlic_len <= FTS_MIN_LEN)) {
        FTS_ERROR("host lcd init code len(%x) is too large", hlic_len);
        return -EINVAL;
    }

    hlic_ver[0] = hlic_buf[hlic_len];
    hlic_ver[1] = hlic_buf[hlic_len + 1];

    FTS_INFO("host lcd init code ver:%x %x", hlic_ver[0], hlic_ver[1]);
    if (0xFF != (hlic_ver[0] + hlic_ver[1])) {
        FTS_ERROR("host lcd init code version check fail");
        return -EINVAL;
    }

    return hlic_ver[0];
}

/************************************************************************
 * Name: fts_ft8201_upgrade
 * Brief:
 * Input: buf - all.bin
 *        len - len of all.bin
 * Output:
 * Return: return 0 if success, otherwise return error code
 ***********************************************************************/
static int fts_ft8201_upgrade(u8 *buf, u32 len)
{
    int ret = 0;
    u8 *tmpbuf = NULL;
    u8 cmd[2] = { 0 };
    u32 delay = 0;
    u32 start_addr = 0;
    u32 app_1_len = 0;
    u32 app_2_len = 0;
    u32 app_len = 0;
    u32 off = 0;

    FTS_INFO("app upgrade...");
    if (NULL == buf) {
        FTS_ERROR("fw file buffer is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_FILE)) {
        FTS_ERROR("fw file buffer len(%x) fail", len);
        return -EINVAL;
    }

    off = upgrade_func_ft8201.appoff + FTS_APPINFO_OFF + FTS_APPINFO_APPLEN_OFF;
    app_1_len = (((u32)buf[off] << 8) + buf[off + 1]);
    off = upgrade_func_ft8201.appoff + FTS_APPINFO_OFF + FTS_APPINFO_APPLEN2_OFF;
    app_2_len = (((u32)buf[off] << 8) + buf[off + 1]);
    app_len = (app_2_len << 16) + app_1_len;
    if ((app_len < FTS_MIN_LEN) || (app_len > FTS_MAX_LEN_APP_FT8201)) {
        FTS_ERROR("app len(%x) fail", app_len);
        return -EINVAL;
    }

    /* enter into upgrade environment */
    ret = fts_fwupg_enter_into_boot();
    if (ret < 0) {
        FTS_ERROR("enter into pramboot/bootloader fail,ret=%d", ret);
        goto APP_UPG_ERR;
    }

    /* erase gesture & parameter sector */
    cmd[0] = FTS_CMD_FLASH_MODE;
    cmd[1] = FLASH_MODE_UPGRADE_VALUE;
    ret = fts_write(cmd, 2);
    if (ret < 0) {
        FTS_ERROR("upgrade mode(09) cmd write fail");
        goto APP_UPG_ERR;
    }

    delay = FTS_ERASE_SECTOR_DELAY * (app_len / FTS_MAX_LEN_SECTOR);
    ret = fts_fwupg_erase(delay);
    if (ret < 0) {
        FTS_ERROR("erase cmd write fail");
        goto APP_UPG_ERR;
    }

    /* write flash */
    start_addr = upgrade_func_ft8201.appoff;
    tmpbuf = buf + start_addr;
    ret = fts_ft8201_write_ecc(start_addr, tmpbuf, app_len);
    if (ret < 0 ) {
        FTS_ERROR("app buffer write fail");
        goto APP_UPG_ERR;
    }

    if (fts_ft8201_check_ide(buf, len)) {
        FTS_INFO("erase and write param configure area");
        ret = fts_ft8201_param_flash(buf, len);
        if (ret < 0 ) {
            FTS_ERROR("param upgrade(erase/write/ecc) fail");
            goto APP_UPG_ERR;
        }
    }

    FTS_INFO("upgrade success, reset to normal boot");
    ret = fts_fwupg_reset_in_boot();
    if (ret < 0) {
        FTS_ERROR("reset to normal boot fail");
    }
    msleep(400);
    return 0;

APP_UPG_ERR:
    return ret;
}

/************************************************************************
 * Name: fts_ft8201_lic_upgrade
 * Brief:
 * Input: buf - all.bin
 *        len - len of all.bin
 * Output:
 * Return: return 0 if success, otherwise return error code
 ***********************************************************************/
static int fts_ft8201_lic_upgrade(u8 *buf, u32 len)
{
    int ret = 0;
    u8 *tmpbuf = NULL;
    u8 cmd[2] = { 0 };
    u32 delay = 0;
    u32 start_addr = 0;
    u32 lic_len = 0;

    FTS_INFO("LCD initial code upgrade...");
    if (NULL == buf) {
        FTS_ERROR("fw file buffer is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_FILE)) {
        FTS_ERROR("fw file buffer len(%x) fail", len);
        return -EINVAL;
    }

    ret = check_initial_code_valid(buf);
    if (ret < 0) {
        FTS_ERROR("initial code invalid, not upgrade lcd init code");
        return -EINVAL;
    }

    lic_len = FTS_MAX_LEN_SECTOR;
    /* remalloc memory for initcode, need change content of initcode afterwise */
    tmpbuf = kzalloc(lic_len, GFP_KERNEL);
    if (NULL == tmpbuf) {
        FTS_INFO("initial code buf malloc fail");
        return -EINVAL;
    }
    start_addr = upgrade_func_ft8201.licoff;
    memcpy(tmpbuf, buf + start_addr, lic_len);

    /* enter into upgrade environment */
    ret = fts_fwupg_enter_into_boot();
    if (ret < 0) {
        FTS_ERROR("enter into pramboot/bootloader fail,ret=%d", ret);
        goto LIC_UPG_ERR;
    }

    /* erase gesture & parameter sector */
    cmd[0] = FTS_CMD_FLASH_MODE;
    cmd[1] = FLASH_MODE_LIC_VALUE;
    ret = fts_write(cmd, 2);
    if (ret < 0) {
        FTS_ERROR("upgrade mode(09) cmd write fail");
        goto LIC_UPG_ERR;
    }

    delay = FTS_ERASE_SECTOR_DELAY * 1;
    ret = fts_fwupg_erase(delay);
    if (ret < 0) {
        FTS_ERROR("erase cmd write fail");
        goto LIC_UPG_ERR;
    }

    ret = fts_ft8201_write_ecc(start_addr, tmpbuf, lic_len);
    if (ret < 0 ) {
        FTS_ERROR("flash ecc write fail");
        goto LIC_UPG_ERR;
    }

    FTS_INFO("upgrade success, reset to normal boot");
    ret = fts_fwupg_reset_in_boot();
    if (ret < 0) {
        FTS_ERROR("reset to normal boot fail");
    }

    if (tmpbuf) {
        kfree(tmpbuf);
        tmpbuf = NULL;
    }

    msleep(400);
    return 0;

LIC_UPG_ERR:
    if (tmpbuf) {
        kfree(tmpbuf);
        tmpbuf = NULL;
    }
    return ret;
}

/************************************************************************
 * Name: fts_ft8201_param_upgrade
 * Brief:
 * Input: buf - all.bin
 *        len - len of all.bin
 * Output:
 * Return: return 0 if success, otherwise return error code
 ***********************************************************************/
static int fts_ft8201_param_upgrade(u8 *buf, u32 len)
{
    int ret = 0;

    FTS_INFO("parameter configure upgrade...");
    if (NULL == buf) {
        FTS_ERROR("fw file buffer is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_FILE)) {
        FTS_ERROR("fw file buffer len(%x) fail", len);
        return -EINVAL;
    }

    /* enter into upgrade environment */
    ret = fts_fwupg_enter_into_boot();
    if (ret < 0) {
        FTS_ERROR("enter into pramboot/bootloader fail,ret=%d", ret);
        goto PARAM_UPG_ERR;
    }

    ret = fts_ft8201_param_flash(buf, len);
    if (ret < 0 ) {
        FTS_ERROR("param upgrade(erase/write/ecc) fail");
        goto PARAM_UPG_ERR;
    }

    FTS_INFO("upgrade success, reset to normal boot");
    ret = fts_fwupg_reset_in_boot();
    if (ret < 0) {
        FTS_ERROR("reset to normal boot fail");
    }

    msleep(400);
    return 0;

PARAM_UPG_ERR:
    return ret;
}

/************************************************************************
 * Name: fts_ft8201_force_upgrade
 * Brief:
 * Input: buf - all.bin
 *        len - constant:128 * 1024
 * Output:
 * Return: return 0 if success, otherwise return error code
 ***********************************************************************/
static int fts_ft8201_force_upgrade(u8 *buf, u32 len)
{
    int ret = 0;
    u8 *tmpbuf = NULL;
    u8 cmd[2] = { 0 };
    u32 delay = 0;
    u32 start_addr = 0;
    u32 tmplen = 0;

    FTS_INFO("fw force upgrade...");
    if (NULL == buf) {
        FTS_ERROR("fw file buffer is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_FILE)) {
        FTS_ERROR("fw file buffer len(%x) fail", len);
        return -EINVAL;
    }

    /* enter into upgrade environment */
    ret = fts_fwupg_enter_into_boot();
    if (ret < 0) {
        FTS_ERROR("enter into pramboot/bootloader fail,ret=%d", ret);
        goto FORCE_UPG_ERR;
    }

    /* erase 0k~116k flash */
    cmd[0] = FTS_CMD_FLASH_MODE;
    cmd[1] = FLASH_MODE_WRITE_FLASH_VALUE;
    ret = fts_write(cmd, 2);
    if (ret < 0) {
        FTS_ERROR("upgrade mode(09) cmd write fail");
        goto FORCE_UPG_ERR;
    }

    if (len > (116 * 1024)) {
        tmplen = 116 * 1024;
    } else {
        tmplen = len;
    }
    delay = FTS_ERASE_SECTOR_DELAY * (tmplen / FTS_MAX_LEN_SECTOR);
    ret = fts_fwupg_erase(delay);
    if (ret < 0) {
        FTS_ERROR("erase cmd write fail");
        goto FORCE_UPG_ERR;
    }

    /* write flash */
    start_addr = 0;
    tmpbuf = buf + start_addr;
    ret = fts_ft8201_write_ecc(start_addr, tmpbuf, tmplen);
    if (ret < 0 ) {
        FTS_ERROR("app buffer write fail");
        goto FORCE_UPG_ERR;
    }

    if (fts_ft8201_check_ide(buf, len)) {
        FTS_INFO("erase and write param configure area");
        ret = fts_ft8201_param_flash(buf, len);
        if (ret < 0 ) {
            FTS_ERROR("param upgrade(erase/write/ecc) fail");
            goto FORCE_UPG_ERR;
        }
    }

    FTS_INFO("upgrade success, reset to normal boot");
FORCE_UPG_ERR:
    ret = fts_fwupg_reset_in_boot();
    if (ret < 0) {
        FTS_ERROR("reset to normal boot fail");
    }

    msleep(400);
    return ret;
}

struct upgrade_func upgrade_func_ft8201 = {
    .ctype = {0x10, 0x1A},
    .fwveroff = 0x510E,
    .fwcfgoff = 0x0F80,
    .appoff = 0x5000,
    .licoff = 0x0000,
    .paramcfgoff = 0x1F000,
    .paramcfgveroff = 0x1F004,
    .paramcfg2off = 0x4000,
    .pramboot_supported = true,
    .pramboot = pb_file_ft8201,
    .pb_length = sizeof(pb_file_ft8201),
    .hid_supported = false,
    .upgrade = fts_ft8201_upgrade,
    .get_hlic_ver = fts_ft8201_get_hlic_ver,
    .lic_upgrade = fts_ft8201_lic_upgrade,
    .param_upgrade = fts_ft8201_param_upgrade,
    .force_upgrade = fts_ft8201_force_upgrade,
};
