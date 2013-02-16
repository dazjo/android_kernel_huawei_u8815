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
#include <asm/mach-types.h>
#include <linux/rmi.h> 
#include "rmi_config.h"
#include "linux/hardware_self_adapt.h"

/*return C8820 config array for tp performance*/
static unsigned char * get_c8820_config_array(void);

/*return C8655 config array for tp performance*/
static unsigned char * get_c8655_config_array(void);

/*return M660 config array for tp performance*/
static unsigned char * get_m660_config_array(void);

/*return module name*/
static char * get_touch_module_name(u8 module_id);

static char touch_info[50] = {0};

unsigned char *C8820_S3200_TPK_Config  = NULL;

unsigned char *C8820_S2202_TPK_Config  = NULL;

unsigned char *M660_S3200_EELY_Config  = NULL;

unsigned char *M660_S2202_EELY_Config  = NULL;

unsigned char *M660_S3200_OFilm_Config = NULL; 

unsigned char *M660_S2202_OFilm_Config = NULL;

unsigned char *C8655_S3200_EELY_Config = NULL;

unsigned char *C8655_S2202_EELY_Config = NULL;

unsigned char *C8655_S3200_OFilm_Config = NULL;

unsigned char *C8655_S2202_OFilm_Config = NULL;

/*prototype*/

/*return config array for tp performance*/
unsigned char * get_config_array()
{
	if (machine_is_msm7x27a_C8820() 
		|| machine_is_msm7x27a_C8825D() )		
	{
		return get_c8820_config_array();
	}
	else if (machine_is_msm7x27a_C8655_NAND())
	{
		return get_c8655_config_array();
	}
	else if (machine_is_msm7x27a_M660())
	{
		return get_m660_config_array();
	}

	return NULL;	
}

/*return C8820 config array for tp performance*/
static unsigned char * get_c8820_config_array()
{
	u16 touch_ic = 0;
	u8 module_id = 0;
	
	touch_ic = get_touch_ic();
	module_id = get_fn34_module_id();
	if (touch_ic == S2202)
	{
		switch (module_id)
		{
			case TPK:
				return C8820_S2202_TPK_Config;
			default :
				return NULL;
		}
	}
	else if (touch_ic == S3200)
	{
		switch (module_id)
		{
			case TPK:
				return C8820_S3200_TPK_Config;
			default :
				return NULL;
		}
	}
		
	return NULL;
}

/*return C8655 config array for tp performance*/
static unsigned char * get_c8655_config_array()
{
	u16 touch_ic = 0;
	u8 module_id = 0;
	
	touch_ic = get_touch_ic();
	module_id = get_fn34_module_id();
	if (touch_ic == S2202)
	{
		switch (module_id)
		{
			case EELY:
				return C8655_S2202_EELY_Config;
			case OFILM:
				return C8655_S2202_OFilm_Config;
			default :
				return NULL;
		}
	}
	else if (touch_ic == S3200)
	{
		switch (module_id)
		{
			case EELY:
				return C8655_S3200_EELY_Config;
			case OFILM:
				return C8655_S3200_OFilm_Config;
			default :
				return NULL;
		}
	}
	
	return NULL;
}

/*return M660 config array for tp performance*/
static unsigned char * get_m660_config_array()
{
	u16 touch_ic = 0;
	u8 module_id = 0;
	
	touch_ic = get_touch_ic();
	module_id = get_fn34_module_id();
	if (touch_ic == S2202)
	{
		switch (module_id)
		{
			case EELY:
				return M660_S3200_EELY_Config;
			case OFILM:
				return M660_S3200_OFilm_Config;
			default :
				return NULL;
		}
	}
	else if (touch_ic == S3200)
	{
		switch (module_id)
		{
			case EELY:
				return M660_S3200_EELY_Config;
			case OFILM:
				return M660_S3200_OFilm_Config;
			default :
				return NULL;
		}
	}
	
	return NULL;
}

/* named Rule
 * 2202 3200 : syanptics-IC-Module.ver
 * for example: syanptics-3200-tpk.2
 *
 * 2000 2100 3000 :syanptics-Module.ver
 * for example: syanptics-tpk.2
 */
char * get_synaptics_touch_info(void)
{
	u16 touch_ic = 0;
	u8 module_id = 0;
	u32 config_id = 0;
	char * module_name = NULL;
	
	touch_ic = get_touch_ic();
	if ((touch_ic == S2202) || (touch_ic == S3200))
	{
		module_id = get_fn34_module_id();
	}
	else
	{
		module_id = get_fn01_module_id();
	}
	
	module_name = get_touch_module_name(module_id);
	if (module_name == NULL)
		return NULL;
	
	if (touch_ic == S2202)
	{
		config_id = get_fn34_config_ver();
		sprintf(touch_info,"synaptics-2202-%s.%d",module_name,config_id);		
	}
	else if (touch_ic == S3200)
	{
		config_id = get_fn34_config_ver();
		sprintf(touch_info,"synaptics-3200-%s.%d",module_name,config_id);	
	}
	else
	{
		config_id = get_fn01_config_ver();
		sprintf(touch_info,"synaptics-%s.%d",module_name,config_id);	
	}

	return touch_info;
}

static char * get_touch_module_name(u8 module_id)
{
	switch(module_id)
	{
		case BYD:
			return "BYD";
		case CMI:
			return "CMI";
		case TRULY:
			return "TRULY";
		case TPK:
			return "TPK";
		case LENSONE:
			return "LENSONE";
		case OFILM:
			return "OFILM";
		case EELY:
			return "EELY";
		case SUCCESS:
			return "SUCCESS";
		case ALPS:
			return "ALPS";
		default:
			return NULL;
	}

	return NULL;
}
