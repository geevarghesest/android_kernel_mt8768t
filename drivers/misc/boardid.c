/*
 *  driver interface to finger print sensor  for 
 *	Copyright (c) 2015  ChipSailing Technology.
 *	All rights reserved.
***********************************************************/
  
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/string.h>

unsigned int ProjectName = 2;
unsigned int market = 2;
unsigned int ClockVersion = 2;
unsigned int Network = 2;

unsigned int board_id_get_ProjectName(void)
{
	return ProjectName;
}

unsigned int board_id_get_market(void)
{
	return market;
}

unsigned int board_id_get_ClockVersion(void)
{
	return ClockVersion;
}

unsigned int board_id_get_Network(void)
{
	return Network;
}

EXPORT_SYMBOL(board_id_get_ProjectName);
EXPORT_SYMBOL(board_id_get_market);
EXPORT_SYMBOL(board_id_get_ClockVersion);
EXPORT_SYMBOL(board_id_get_Network);

int __init setup_get_ProjectName(char *str)
{
	pr_info("pr_info,aguement str:%s\n",str);
	if(!strncmp(str,"Akita",5))
		ProjectName = 0;
	else if(!strncmp(str,"Anna2",5))
		ProjectName = 1;
	pr_info("pr_info,board_id-ProjectName:%d\n",ProjectName);
	return 1;
}

int __init setup_get_market(char *str)
{
	pr_info("pr_info,aguement str:%s\n",str);
	if(!strncmp(str,"ROW",3))
		market = 0;
	else if(!strncmp(str,"PRC",3))
		market = 1;
	pr_info("pr_info,board_id-market:%d\n",market);
	return 1;
}

int __init setup_get_ClockVersion(char *str)
{
	pr_info("pr_info,aguement str:%s\n",str);
	if(!strncmp(str,"NEW",3))
		ClockVersion = 0;
	else if(!strncmp(str,"OLD",3))
		ClockVersion = 1;
	pr_info("pr_info,board_id-ClockVersion:%d\n",ClockVersion);
	return 2;
}

int __init setup_get_Network(char *str)
{
	pr_info("pr_info,aguement str:%s\n",str);
	if(!strncmp(str,"LTE",3))
		Network = 0;
	else if(!strncmp(str,"WIF",3))
		Network = 1;
	pr_info("pr_info,board_id-Network:%d\n", Network);
	return 2;
}
__setup("androidboot.ProjectName=", setup_get_ProjectName);
__setup("androidboot.Market=", setup_get_market);
__setup("androidboot.ClockVersion=", setup_get_ClockVersion);
__setup("androidboot.Network=", setup_get_Network);
