/* drivers\video\msm\c8600_backlight.c
 * backlight driver for 7x25 platform
 *
 * Copyright (C) 2009 HUAWEI Technology Co., ltd.
 * 
 * Date: 2009/12/29
 * By Jia Lin
 * 
 */

#include <mach/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include "lcdc_huawei_config.h"
#include <mach/rpc_pmapp.h>

/*CABC CTL MACRO , RANGE 0 to 255*/
#define PWM_LEVEL 255
#define ADD_VALUE			4
#define PWM_LEVEL_ADJUST	226
#define BL_MIN_LEVEL 	    30

/*LPG CTL MACRO  range 0 to 100*/
#define PWM_LEVEL_ADJUST_LPG	100
#define BL_MIN_LEVEL_LPG 	    10

/* use the mmp pin like three-leds */
void lcd_set_backlight_pwm(int level)
{
    static uint8 last_level = 0;
	/* keep duty 10% < level < 100% */
	if (level)    
   	{   
		level = ((level * PWM_LEVEL_ADJUST_LPG) / PWM_LEVEL ); 
		if (level < BL_MIN_LEVEL_LPG)        
		{    
			level = BL_MIN_LEVEL_LPG;      
		}  
	}

    if (last_level == level)
    {
        return ;
    }
    last_level = level;

    pmapp_disp_backlight_set_brightness(last_level);
}

void cabc_backlight_set(struct msm_fb_data_type * mfd)
{	     
	struct msm_fb_panel_data *pdata = NULL;   
	uint32 bl_level = mfd->bl_level;	
		/* keep duty 10% < level < 100% */
	if (bl_level)    
   	{   
	/****delete one line codes for backlight*****/
		if (bl_level < BL_MIN_LEVEL)        
		{    
			bl_level = BL_MIN_LEVEL;      
		}  
	}
	/* backlight ctrl by LCD-self, like as CABC */  
	apply_lock_source(mfd);
	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;  
	if ((pdata) && (pdata->set_cabc_brightness))   
   	{       
		pdata->set_cabc_brightness(mfd,bl_level);    
	}  
	release_lock_source(mfd);
}

void pwm_set_backlight(struct msm_fb_data_type *mfd)
{
	lcd_panel_type lcd_panel_wvga = LCD_NONE;
	
	lcd_panel_wvga = get_lcd_panel_type();
	if ((MIPI_RSP61408_CHIMEI_WVGA == lcd_panel_wvga ) 
		|| (MIPI_RSP61408_BYD_WVGA == lcd_panel_wvga )
		|| (MIPI_RSP61408_TRULY_WVGA == lcd_panel_wvga )
		|| (MIPI_HX8369A_TIANMA_WVGA == lcd_panel_wvga ))
	{
		/* keep duty is 75% of the quondam duty */
		mfd->bl_level = mfd->bl_level * 75 / 100;
	}
	
	if (get_hw_lcd_ctrl_bl_type() == CTRL_BL_BY_MSM)
	{
		lcd_set_backlight_pwm(mfd->bl_level);
 	}   
	else    
 	{
		cabc_backlight_set(mfd);  
 	}
	return;
}

/* 
 * when setting backlight, apply lock source 
 */
void apply_lock_source(struct msm_fb_data_type * mfd)
{
	if (get_hw_lcd_interface_type() == LCD_IS_MIPI)
	{
		down(&mfd->dma->mutex);
		down(&mfd->sem);
	}
	else
	{
		down(&mfd->sem);
	}
}

/*
 * when setting backlight, release lock source 
 */
void release_lock_source(struct msm_fb_data_type * mfd)
{
	if (get_hw_lcd_interface_type() == LCD_IS_MIPI)
	{
		up(&mfd->sem);
		up(&mfd->dma->mutex);
	}
	else
	{
		up(&mfd->sem);
	}
}

