/* drivers\video\msm\mddi_rsp61408.c
 * rsp61408 LCD driver for 7x27 platform
 *
 * Copyright (C) 2010 HUAWEI Technology Co., ltd.
 * 
 * Date: 2011/09/01
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
#define PM_GPIO_24 24
#define PM_GPIO_HIGH_VALUE 1 
struct sequence* rsp61408_wvga_init_table = NULL;
static lcd_panel_type lcd_panel_wvga = LCD_NONE;

static struct sequence rsp61408_wvga_write_cabc_brightness_table[]= 
{
	/* solve losing control of the backlight */
	{0x000B9,TYPE_COMMAND,0},//B9H
	{0x00001,TYPE_PARAMETER,0},
	{0x00000,TYPE_PARAMETER,0},
    {0x00002,TYPE_PARAMETER,0},
    {0x00018,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};

static const struct sequence rsp61408_wvga_standby_exit_table[]= 
{
	{0x0011,TYPE_COMMAND,0},
	{0x0000,TYPE_PARAMETER,0},
	{0x0003A,TYPE_COMMAND,150},
	{0x00077,TYPE_PARAMETER,0},//11h
	{0x00035,TYPE_COMMAND,0},
	{0x00000,TYPE_PARAMETER,0},
	{0x00029,TYPE_COMMAND,0}, //29h
	{0x00000,TYPE_PARAMETER,0},	
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};
static const struct sequence reverse_rsp61408_wvga_exit_table[]= 
{
	{0x0011,TYPE_COMMAND,0},
	{0x0000,TYPE_PARAMETER,0},
	{0x0003A,TYPE_COMMAND,150},
	{0x00077,TYPE_PARAMETER,0},//11h
	{0x00035,TYPE_COMMAND,0},
	{0x00000,TYPE_PARAMETER,0},

	{0x00044,TYPE_COMMAND,0}, //44h
	{0x00001,TYPE_PARAMETER,0},
	{0x00090,TYPE_PARAMETER,0},
	{0x000B0,TYPE_COMMAND,0}, //B0h
	{0x00004,TYPE_PARAMETER,0},
	{0x000C1,TYPE_COMMAND,0}, //C1h
	{0x00063,TYPE_PARAMETER,0}, 
	{0x00031,TYPE_PARAMETER,0}, 
	{0x00000,TYPE_PARAMETER,0},
	{0x000D6,TYPE_COMMAND,0}, //D6h
	{0x00028,TYPE_PARAMETER,0}, //
       
	{0x00029,TYPE_COMMAND,0}, //29h
	{0x00000,TYPE_PARAMETER,0},	
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};
static const struct sequence rsp61408_wvga_standby_enter_table[]= 
{
	{0x00034,TYPE_COMMAND,0},
	{0x00000,TYPE_PARAMETER,0},
	{0x00028,TYPE_COMMAND,0}, //29h
	{0x00000,TYPE_PARAMETER,0},
	{0x0010,TYPE_COMMAND,20},
	{0x0000,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,120}, //the end flag,it don't sent to driver IC
};

/* gamma 2.2 */
static const struct sequence rsp61408_wvga_dynamic_gamma22_table[] = 
{          

	{0x000C8,TYPE_COMMAND,0},//C8  gamma 2.2
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38 37
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	
	{0x000C9,TYPE_COMMAND,0},//C9
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38 37
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	
	{0x000CA,TYPE_COMMAND,0},//CA
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38 37
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};
static const struct sequence rsp61408_wvga_dynamic_gamma19_table[] = {};
static const struct sequence rsp61408_wvga_dynamic_gamma25_table[] = 
{
	{0x000C8,TYPE_COMMAND,0},//C8  gamma 2.5
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	
	{0x000C9,TYPE_COMMAND,0},//C9
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	
	{0x000CA,TYPE_COMMAND,0},//CA
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};
int rsp61408_set_dynamic_gamma(enum danymic_gamma_mode  gamma_mode)
{
    int ret = 0;
	
    if (LOW_LIGHT == gamma_mode)
    {
        printk(KERN_ERR "the dynamic_gamma_setting is wrong\n");
    }

    switch(gamma_mode)
    {
        case GAMMA25:
            ret = process_mddi_table((struct sequence*)&rsp61408_wvga_dynamic_gamma25_table,
                        ARRAY_SIZE(rsp61408_wvga_dynamic_gamma25_table), lcd_panel_wvga);
            break ;
        case GAMMA22:
			 ret = process_mddi_table((struct sequence*)&rsp61408_wvga_dynamic_gamma22_table,
                        ARRAY_SIZE(rsp61408_wvga_dynamic_gamma22_table), lcd_panel_wvga);
            break;
        case HIGH_LIGHT:
            ret = process_mddi_table((struct sequence*)&rsp61408_wvga_dynamic_gamma19_table,
                        ARRAY_SIZE(rsp61408_wvga_dynamic_gamma19_table), lcd_panel_wvga);
            break;
        default:
            ret= -1;
            break;
    }
	LCD_DEBUG("%s: change gamma mode to %d\n",__func__,gamma_mode);
    return ret;
}
static int rsp61408_lcd_on(struct platform_device *pdev)
{
	boolean para_debug_flag = FALSE;
    uint32 para_num = 0;
	int ret = 0;
	switch(lcd_panel_wvga)
	{
		case MDDI_RSP61408_BYD_WVGA:
		case MDDI_RSP61408_CHIMEI_WVGA:
			para_debug_flag = lcd_debug_malloc_get_para( "rsp61408_wvga_init_table", 
	    		(void**)&rsp61408_wvga_init_table,&para_num);
			break;
		default:
			break;
	}
    if( (TRUE == para_debug_flag)&&(NULL != rsp61408_wvga_init_table))
    {
		ret = process_mddi_table(rsp61408_wvga_init_table, para_num, lcd_panel_wvga);
    }
    else
    {
		/*reset the direction register to resolve the problem of revert. */
		/* Exit Standby Mode */
		if(machine_is_msm8255_u8730())
		{
			ret = process_mddi_table((struct sequence*)&reverse_rsp61408_wvga_exit_table, 
					ARRAY_SIZE(reverse_rsp61408_wvga_exit_table), lcd_panel_wvga);
		}
		else
		{
		ret = process_mddi_table((struct sequence*)&rsp61408_wvga_standby_exit_table, 
			ARRAY_SIZE(rsp61408_wvga_standby_exit_table), lcd_panel_wvga);
		}
    }
       
	if((TRUE == para_debug_flag)&&(NULL != rsp61408_wvga_init_table))
	{
		lcd_debug_free_para((void *)rsp61408_wvga_init_table);
	}
	
    LCD_DEBUG("%s: rsp61408_lcd exit sleep mode ,on_ret=%d\n",__func__,ret);
	
	return ret;
}

static int rsp61408_lcd_off(struct platform_device *pdev)
{
	int ret = 0;
	ret = process_mddi_table((struct sequence*)&rsp61408_wvga_standby_enter_table, 
    	      		ARRAY_SIZE(rsp61408_wvga_standby_enter_table), lcd_panel_wvga);
    LCD_DEBUG("%s: rsp61408_lcd enter sleep mode ,off_ret=%d\n",__func__,ret);
	return ret;
}
void mddi_rsp61408_set_cabc_backlight(struct msm_fb_data_type *mfd,uint32 bl_level)
{
 
	rsp61408_wvga_write_cabc_brightness_table[2].reg = bl_level;
	process_mddi_table((struct sequence*)&rsp61408_wvga_write_cabc_brightness_table,
                    ARRAY_SIZE(rsp61408_wvga_write_cabc_brightness_table), lcd_panel_wvga);
}

static int __devinit rsp61408_probe(struct platform_device *pdev)
{
	msm_fb_add_device(pdev);
 	return 0;
}

static struct platform_driver this_driver = {
	.probe  = rsp61408_probe,
	.driver = {
		.name   = "mddi_rsp61408_wvga",
	},
};

static struct msm_fb_panel_data rsp61408_panel_data = {
	.on = rsp61408_lcd_on,
	.off = rsp61408_lcd_off,
	.set_backlight = pwm_set_backlight,
	.set_cabc_brightness = mddi_rsp61408_set_cabc_backlight,
};

static struct platform_device this_device = {
	.name   = "mddi_rsp61408_wvga",
	.id	= 0,
	.dev	= {
		.platform_data = &rsp61408_panel_data,
	}
};
static int __init rsp61408_init(void)
{
	int ret = 0;
	struct msm_panel_info *pinfo = NULL;
	bpp_type bpp = MDDI_OUT_24BPP;		
	hw_lcd_interface_type mddi_port_type = get_hw_lcd_interface_type();

	lcd_panel_wvga=get_lcd_panel_type();
	if((MDDI_RSP61408_CHIMEI_WVGA != lcd_panel_wvga) &&
            (MDDI_RSP61408_BYD_WVGA != lcd_panel_wvga)&&
                (MDDI_RSP61408_TRULY_WVGA != lcd_panel_wvga))
	{
		return 0;
	}

	LCD_DEBUG("%s:start init %s\n",__func__,this_device.name);
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
		pinfo = &rsp61408_panel_data.panel_info;
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
		}
	}

	return ret;
}
module_init(rsp61408_init);
