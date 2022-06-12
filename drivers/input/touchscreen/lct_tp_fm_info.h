/****************************************************************************************
 *
 * @File Name   : lct_tp_info.h
 * @Author      : wanghan
 * @E-mail      : <wanghan@longcheer.com>
 * @Create Time : 2018-08-17 17:34:43
 * @Description : Display touchpad information.
 *
 ****************************************************************************************/

#ifndef LCT_TP_FM_INFO_H
#define LCT_TP_FM_INFO_H

#define SUPPORT_READ_TP_VERSION // *#87# can read tp version

extern int init_tp_fm_info(char* version_info_str);
extern void update_tp_fm_info(char *version_info_str);

#endif
