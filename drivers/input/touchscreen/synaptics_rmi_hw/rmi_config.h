/*
 * Copyright (c) 2012 Huawei Device Company
 *
 * This file include Touch fw and config for solving touch performance.
 * 
 * fw be named as followed:
 * PhoneName_ICName_ModuleName_FW
 * for example: C8820_S3200_TPK_FW
 *
 * config be named as followed:
 * PhoneName_ICName_ModuleName_Config
 * for example: C8820_S3200_TPK_Config
 *
 */
#ifndef _RMI_CONFIG_H
#define _RMI_CONFIG_H

/*function declare*/

/*get touch IC
 *  val    ic
 * 2202 : S2202
 * 3200 : S3200
 * other: S3000/S2100/S2000
 */
u16 get_touch_ic(void);

/* get touch module id
 *	{0,"none"},
 *	{1,"BYD"},
 *	{2,"CMI"},
 *	{3,"Truly"},
 *	{4,"TPK"},
 *	{5,"LensOne"},
 *	{6,"Ofilm"},
 *	{7,"EEly"},
 *	{8,"Success"},
 *	{9,"Alps"},
 */
u8 get_fn34_module_id(void);
u8 get_fn01_module_id(void);

/*get fn34 config id*/
u32 get_fn34_config_ver(void);

/*get fn01 config id for 2100 3000 2000*/
u8 get_fn01_config_ver(void);

/*return config array for tp performance*/
unsigned char * get_config_array(void);


/* synaptics module name and id table*/
#define BYD 1
#define CMI 2
#define TRULY 3
#define TPK 4
#define LENSONE 5
#define OFILM 6
#define EELY 7
#define SUCCESS 8
#define ALPS 9

/*synaptics IC macro*/
#define S3200 3200
#define S2202 2202

#endif

