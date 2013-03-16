/* drivers\video\msm\mddi_rsp61408.c
 * hx8369a LCD driver for 7x30 platform
 *
 * Copyright (C) 2010 HUAWEI Technology Co., ltd.
 * 
 * Date: 2011/12/10
 * By qitongliang
 * 
 */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <linux/mfd/pmic8058.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/hardware_self_adapt.h>
#include <linux/pwm.h>
#include <mach/pmic.h>
#include "hw_backlight.h"
#include "hw_lcd_common.h"
#include "lcd_hw_debug.h"
struct sequence* hx8369a_wvga_init_table = NULL;
static lcd_panel_type lcd_panel_wvga = LCD_NONE;
#define PM_GPIO_24 24
#define PM_GPIO_HIGH_VALUE 1 


static struct sequence hx8369a_wvga_write_cabc_brightness_table[]= 
{
	{0x00051,TYPE_COMMAND,0},
	{0x000FF,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0},
};

static const struct sequence hx8369a_wvga_standby_exit_table[]=
{ 
	{0x00011,TYPE_COMMAND,0}, 
	{0x00000,TYPE_PARAMETER,0},
	{0x00029,TYPE_COMMAND,120}, 
	{0x00000,TYPE_PARAMETER,0}, 
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,20}, //the end flag,it don't sent to driver IC
};

static const struct sequence hx8369a_wvga_standby_enter_table[]= 
{
	{0x00028,TYPE_COMMAND,0}, 
	{0x00000,TYPE_PARAMETER,0},
	{0x0010,TYPE_COMMAND,20},
	{0x0000,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,120}, //the end flag,it don't sent to driver IC
};

/* gamma 2.2 */
static const struct sequence hx8369a_wvga_dynamic_gamma22_table[] = 
{
	{0xE0,TYPE_COMMAND,0},
	{0x02,TYPE_PARAMETER,0},
	{0x11,TYPE_PARAMETER,0},
	{0x1A,TYPE_PARAMETER,0},
	{0x3B,TYPE_PARAMETER,0},
	{0x3F,TYPE_PARAMETER,0}, 
	{0x3f,TYPE_PARAMETER,0},
	{0x2B,TYPE_PARAMETER,0},
	{0x49,TYPE_PARAMETER,0},
	{0x0A,TYPE_PARAMETER,0}, 
	{0x0F,TYPE_PARAMETER,0}, 
	{0x0E,TYPE_PARAMETER,0}, 
	{0x13,TYPE_PARAMETER,0}, 
	{0x14,TYPE_PARAMETER,0}, 
	{0x13,TYPE_PARAMETER,0}, 
	{0x14,TYPE_PARAMETER,0},
	{0x10,TYPE_PARAMETER,0},
	{0x14,TYPE_PARAMETER,0},
	{0x02,TYPE_PARAMETER,0},
	{0x11,TYPE_PARAMETER,0},
	{0x1A,TYPE_PARAMETER,0}, 
	{0x3B,TYPE_PARAMETER,0}, 
	{0x3F,TYPE_PARAMETER,0}, 
	{0x3f,TYPE_PARAMETER,0},
	{0x2B,TYPE_PARAMETER,0},
	{0x49,TYPE_PARAMETER,0},
	{0x0A,TYPE_PARAMETER,0},
	{0x0F,TYPE_PARAMETER,0},
	{0x0E,TYPE_PARAMETER,0},
	{0x13,TYPE_PARAMETER,0},
	{0x14,TYPE_PARAMETER,0},
	{0x13,TYPE_PARAMETER,0},
	{0x14,TYPE_PARAMETER,0}, 
	{0x10,TYPE_PARAMETER,0}, 
	{0x14,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};
/* gamma1.9 */
static const struct sequence hx8369a_wvga_dynamic_gamma19_table[] = {};
/* gamma2.5 */
static const struct sequence hx8369a_wvga_dynamic_gamma25_table[] = 
{
	{0xE0,TYPE_COMMAND,0},
	{0x0A,TYPE_PARAMETER,0},
	{0x18,TYPE_PARAMETER,0},
	{0x1E,TYPE_PARAMETER,0},
	{0x39,TYPE_PARAMETER,0},
	{0x3F,TYPE_PARAMETER,0}, 
	{0x3f,TYPE_PARAMETER,0},
	{0x2F,TYPE_PARAMETER,0},
	{0x4D,TYPE_PARAMETER,0},
	{0x08,TYPE_PARAMETER,0}, 
	{0x0D,TYPE_PARAMETER,0}, 
	{0x0F,TYPE_PARAMETER,0}, 
	{0x13,TYPE_PARAMETER,0}, 
	{0x16,TYPE_PARAMETER,0}, 
	{0x13,TYPE_PARAMETER,0}, 
	{0x14,TYPE_PARAMETER,0},
	{0x14,TYPE_PARAMETER,0},
	{0x14,TYPE_PARAMETER,0},
	{0x0A,TYPE_PARAMETER,0},
	{0x18,TYPE_PARAMETER,0},
	{0x1E,TYPE_PARAMETER,0}, 
	{0x39,TYPE_PARAMETER,0}, 
	{0x3F,TYPE_PARAMETER,0}, 
	{0x3f,TYPE_PARAMETER,0},
	{0x2F,TYPE_PARAMETER,0},
	{0x4D,TYPE_PARAMETER,0},
	{0x08,TYPE_PARAMETER,0},
	{0x0D,TYPE_PARAMETER,0},
	{0x0F,TYPE_PARAMETER,0},
	{0x13,TYPE_PARAMETER,0},
	{0x16,TYPE_PARAMETER,0},
	{0x13,TYPE_PARAMETER,0},
	{0x14,TYPE_PARAMETER,0}, 
	{0x14,TYPE_PARAMETER,0}, 
	{0x14,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};

/* add the function  to set different gama by different mode */
int hx8369a_set_dynamic_gamma(enum danymic_gamma_mode  gamma_mode)
{
    int ret = 0;
	
    if (LOW_LIGHT == gamma_mode)
    {
        printk(KERN_ERR "the dynamic_gamma_setting is wrong\n");
    }

    switch(gamma_mode)
    {
        case GAMMA25:
            ret = process_mddi_table((struct sequence*)&hx8369a_wvga_dynamic_gamma25_table,
                        ARRAY_SIZE(hx8369a_wvga_dynamic_gamma25_table), lcd_panel_wvga);
            break ;
        case GAMMA22:
			 ret = process_mddi_table((struct sequence*)&hx8369a_wvga_dynamic_gamma22_table,
                        ARRAY_SIZE(hx8369a_wvga_dynamic_gamma22_table), lcd_panel_wvga);
            break;
        case HIGH_LIGHT:
            ret = process_mddi_table((struct sequence*)&hx8369a_wvga_dynamic_gamma19_table,
                        ARRAY_SIZE(hx8369a_wvga_dynamic_gamma19_table), lcd_panel_wvga);
            break;
        default:
            ret= -1;
            break;
    }
	LCD_DEBUG("%s: change gamma mode to %d\n",__func__,gamma_mode);
    return ret;
}

static int hx8369a_lcd_on(struct platform_device *pdev)
{
	boolean para_debug_flag = FALSE;
    uint32 para_num = 0;
	int ret = 0;

 /* open debug file and read the para */
    para_debug_flag = lcd_debug_malloc_get_para( "hx8369a_wvga_init_table", 
    	(void**)&hx8369a_wvga_init_table,&para_num);       
	
	/* If exist the init file ,then init lcd with it for debug */
    if( (TRUE == para_debug_flag)&&(NULL != hx8369a_wvga_init_table))
    {
		
		ret = process_mddi_table(hx8369a_wvga_init_table, para_num, lcd_panel_wvga);
    }
    else
    {  
		/* Exit Standby Mode */
        ret = process_mddi_table((struct sequence*)&hx8369a_wvga_standby_exit_table, 
					ARRAY_SIZE(hx8369a_wvga_standby_exit_table), lcd_panel_wvga);	
    }
	
	
	/* Must malloc before,then you can call free */
	if((TRUE == para_debug_flag)&&(NULL != hx8369a_wvga_init_table))
	{
		lcd_debug_free_para((void *)hx8369a_wvga_init_table);
	}

	LCD_DEBUG("%s: hx8369a_lcd exit sleep mode,ret=%d\n",__func__,ret);
	
	return ret;
}

static int hx8369a_lcd_off(struct platform_device *pdev)
{
	int ret = 0;

	ret = process_mddi_table((struct sequence*)&hx8369a_wvga_standby_enter_table, 
    	      		ARRAY_SIZE(hx8369a_wvga_standby_enter_table), lcd_panel_wvga);
	LCD_DEBUG("%s: hx8369a_lcd enter sleep mode,ret=%d\n",__func__,ret);
	return ret;
}

static void hx8369a_set_cabc_brightness(struct msm_fb_data_type *mfd,uint32 bl_level)
{
	int ret = 0;	 
	hx8369a_wvga_write_cabc_brightness_table[1].reg = bl_level;
	ret = process_mddi_table((struct sequence*)&hx8369a_wvga_write_cabc_brightness_table,
                    ARRAY_SIZE(hx8369a_wvga_write_cabc_brightness_table), lcd_panel_wvga);
}

static int __devinit hx8369a_probe(struct platform_device *pdev)
{
	msm_fb_add_device(pdev);
 	return 0;
}

static struct platform_driver this_driver = {
	.probe  = hx8369a_probe,
	.driver = {
		.name   = "mddi_hx8369a_wvga",
	},
};

static struct msm_fb_panel_data hx8369a_panel_data = {
	.on = hx8369a_lcd_on,
	.off = hx8369a_lcd_off,
	.set_backlight = pwm_set_backlight,
	.set_cabc_brightness = hx8369a_set_cabc_brightness,
    .set_dynamic_gamma = hx8369a_set_dynamic_gamma,
};

static struct platform_device this_device = {
	.name   = "mddi_hx8369a_wvga",
	.id	= 0,
	.dev	= {
		.platform_data = &hx8369a_panel_data,
	}
};
static int __init hx8369a_init(void)
{
	int ret = 0;
	struct msm_panel_info *pinfo = NULL;
	bpp_type bpp = MDDI_OUT_16BPP;
	hw_lcd_interface_type mddi_port_type = get_hw_lcd_interface_type();

	lcd_panel_wvga=get_lcd_panel_type();
	if(MDDI_HX8369A_TIANMA_WVGA != lcd_panel_wvga)
	{
		return 0;
	}

	LCD_DEBUG("%s:start init %s\n",__func__,this_device.name);
	
	/* Select which bpp accroding MDDI port type */
	if(LCD_IS_MDDI_TYPE1 == mddi_port_type)
	{
		bpp = MDDI_OUT_16BPP;
	}
	else if(LCD_IS_MDDI_TYPE2 == mddi_port_type)
	{
		bpp = MDDI_OUT_24BPP;
	}
	else
	{
		bpp = MDDI_OUT_16BPP;
	}
	
	ret = platform_driver_register(&this_driver);
	if (!ret) 
	{
		pinfo = &hx8369a_panel_data.panel_info;
		pinfo->xres = 480;
		pinfo->yres = 800;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
		pinfo->bpp = (uint32)bpp;
		pinfo->fb_num = 2;
		pinfo->clk_rate = 192000000;
		pinfo->clk_min = 192000000;
		pinfo->clk_max = 192000000;
		pinfo->lcd.vsync_enable = TRUE;
        pinfo->lcd.refx100 = 6000;
		pinfo->lcd.v_back_porch = 0;
		pinfo->lcd.v_front_porch = 0;
		pinfo->lcd.v_pulse_width = 22;
		pinfo->lcd.hw_vsync_mode = TRUE;
		pinfo->lcd.vsync_notifier_period = 0;
		pinfo->bl_max = 255;

		ret = platform_device_register(&this_device);
		if (ret)
		{
			platform_driver_unregister(&this_driver);
			LCD_DEBUG("%s: Failed on platform_device_register(): rc=%d \n",__func__, ret);
		}
	}

	return ret;
}
module_init(hx8369a_init);

