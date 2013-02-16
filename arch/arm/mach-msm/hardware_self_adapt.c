/*
 * Copyright (C) 2010 Huawei, Inc.
 * Copyright (c) 2008-2010, Huawei. All rights reserved.
 */
 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/errno.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <asm/mach-types.h>
#include "linux/hardware_self_adapt.h"
#ifdef CONFIG_HUAWEI_SYNAPTICS_RMI_TOUCH
#include "linux/rmi.h"
#endif

static unsigned int lcd_id = 0;
static unsigned int sub_board_id = 0;
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
static unsigned int charge_flag = 0;
#endif

static unsigned int lcd_y_res = 480;

/* framebuffer size self adapter */
#ifdef CONFIG_FRAMEBUF_SELF_ADAPT
static __u32	frame_buffer_size = 0;
static __u32	frame_buffer_start = 0;	/* physical start address */
#endif

/* cust size self adapter */
static __u32	cust_buffer_size = 0;
static __u32	cust_buffer_start = 0;	/* physical start address */

static unsigned int recovery_boot_mode = 0;
 
#define ATAG_LCD_ID 0x4d534D73
int __init parse_tag_lcd_id(const struct tag *tags)
{
	 struct tag *t = (struct tag *)tags;
 
	 lcd_id = t->u.revision.rev;
	 printk(KERN_DEBUG "%s: lcd_id = 0x%x\n", __func__, lcd_id);
	 
	 return lcd_id;
}
 __tagtable(ATAG_LCD_ID, parse_tag_lcd_id);
 
#define ATAG_SUB_BOARD_ID 0x4d534D76
int __init parse_tag_sub_board_id(const struct tag *tags)
{
	 struct tag *t = (struct tag *)tags;

	 sub_board_id = t->u.revision.rev;
	 printk(KERN_DEBUG "%s: sub_board_id = 0x%x\n", __func__, sub_board_id);
	 
	 return sub_board_id;
}
 __tagtable(ATAG_SUB_BOARD_ID, parse_tag_sub_board_id);

#ifdef CONFIG_USB_AUTO_INSTALL
#define ATAG_BOOT_MODE_ID   0x4d534d77
 int __init parse_tag_boot_mode_id(const struct tag *tags)
 {
	 struct tag *t = (struct tag *)tags;
 
	 recovery_boot_mode = t->u.revision.rev;
	 printk(KERN_DEBUG "%s: usb_mode_id = 0x%x\n", __func__, recovery_boot_mode);
	 return recovery_boot_mode;
 }
 __tagtable(ATAG_BOOT_MODE_ID, parse_tag_boot_mode_id);
#endif  /* CONFIG_USB_AUTO_INSTALL */
 
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
#define ATAG_CHARGE_FLAG  0x4d534D78
int __init parse_tag_charge_flag(const struct tag *tags)
{
    struct tag *t = (struct tag *)tags;

    charge_flag = t->u.revision.rev;
    printk(KERN_DEBUG "%s: charge_flag = 0x%x\n", __func__, charge_flag);

    return charge_flag;  
}
__tagtable(ATAG_CHARGE_FLAG, parse_tag_charge_flag);
#endif
 
/*get framebuffer address and size from atag, passed by bootloader*/
#ifdef CONFIG_FRAMEBUF_SELF_ADAPT
#define ATAG_FRAME_BUFFER_ID 0x4d534D79
int __init parse_tag_frame_buffer(const struct tag *tags)
{
	frame_buffer_size = tags->u.mem.size;
	frame_buffer_start = tags->u.mem.start;
	
    printk(KERN_DEBUG "%s: fb addr= 0x%x, size=0x%0x\n", __func__, frame_buffer_start, frame_buffer_size);
    return 0;
}
__tagtable(ATAG_FRAME_BUFFER_ID, parse_tag_frame_buffer);

#define ATAG_LCD_Y_RES_FLAG 0x4d534D7A
int __init parse_tag_lcd_y_res_flag(const struct tag *tags)
{
    struct tag *t = (struct tag *)tags;

    lcd_y_res= t->u.revision.rev;
    printk(KERN_DEBUG "%s: lcd_y_res = %d\n", __func__, lcd_y_res);

    return lcd_y_res;  
}
__tagtable(ATAG_LCD_Y_RES_FLAG, parse_tag_lcd_y_res_flag);

/*used in board-msm7x27a.c*/
void get_frame_buffer_mem_region(__u32 *start_addr, __u32 *size)
{
	*start_addr = frame_buffer_start;
	*size = frame_buffer_size;
}
#endif


/*get cust address and size from atag, passed by bootloader*/
#define ATAG_CUST_BUFFER_ID 0x4d534D7B
int __init parse_tag_cust_buffer(const struct tag * tags)
{
	cust_buffer_size = tags->u.mem.size;
	cust_buffer_start = tags->u.mem.start;
	
    printk(KERN_DEBUG "%s: cust addr= 0x%x, size=0x%0x\n", __func__, cust_buffer_start, cust_buffer_size);
    return 0;
}
__tagtable(ATAG_CUST_BUFFER_ID, parse_tag_cust_buffer);

/*used in board-msm7x27a.c*/
void get_cust_buffer_mem_region(__u32 *start_addr, __u32 *size)
{
	*start_addr = cust_buffer_start;
	*size = cust_buffer_size;
}

char *get_wifi_device_name(void)
{                                                                                                        
  hw_wifi_device_model wifi_device_model = WIFI_UNKNOW;  
  char *wifi_device_id = NULL;                       
                                                             
  wifi_device_model = get_hw_wifi_device_model();        
  printk("wifi_device_id = %d\n",wifi_device_model);    
  if(WIFI_BROADCOM_4330 == wifi_device_model)                 
  {                                                  
	wifi_device_id = "1.2";
  }                                                  
  else if(WIFI_QUALCOMM_6005 == wifi_device_model)            
  { 
    wifi_device_id = "2.1"; 
  }                                                  
  else                                               
  {                                                  
    wifi_device_id = "UNKNOWN WIFI DEVICE";          
  }                                                  
  return wifi_device_id;                             
} 

void get_audio_property(char *audio_property)
{
  unsigned int property = AUDIO_PROPERTY_INVALID;
  audio_property_type mic_type = MIC_NONE;
  audio_property_type fir_enable = FIR_DISABLE;
  audio_property_type fm_type = FM_BROADCOM;
  
  mic_type = get_audio_mic_type();
  fir_enable = get_audio_fir_enabled();
  fm_type =  get_audio_fm_type();

  property = fir_enable | mic_type | fm_type;

  sprintf(audio_property, "%8x", property);
}

unsigned int get_hw_lcd_id(void)
{
	return lcd_id;
}

hw_ver_sub_type get_hw_sub_board_id(void)
{
	return (hw_ver_sub_type)(sub_board_id&HW_VER_SUB_MASK);
}

#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
unsigned int get_charge_flag(void)
{
    return charge_flag;
}
#endif

lcd_type atag_get_lcd_y_res(void)
{
   return (lcd_type)lcd_y_res;
}

/* the function interface to check boot mode in kernel */
unsigned char bootimage_is_recovery(void)
{
  return recovery_boot_mode;
}

/*
 *brief: get lcd control backlight type
 */
hw_lcd_ctrl_bl_type get_hw_lcd_ctrl_bl_type(void)
{
    hw_lcd_ctrl_bl_type ctrl_bl_type = CTRL_BL_BY_UNKNOW;
	/*control backlight by MSM pwm*/
	/* U8661 uses PM pwm. */
	/* C8820VC uses PM pwm. */
	if (machine_is_msm7x27a_umts() || machine_is_msm7x27a_cdma()
		|| machine_is_msm7x27a_U8815() || machine_is_msm7x27a_U8655_EMMC()
		|| machine_is_msm7x27a_U8185() || machine_is_msm7x27a_U8655()
		|| machine_is_msm7x27a_M660()  || machine_is_msm7x27a_U8661()
		|| (machine_is_msm7x27a_C8820() && (HW_VER_SUB_VC <= get_hw_sub_board_id()))
		)
	{
		ctrl_bl_type = CTRL_BL_BY_MSM;
	}
	/*control backlight by LCD output pwm*/
	else if(machine_is_msm7x27a_C8655_NAND()
	        || (machine_is_msm7x27a_C8820() && (HW_VER_SUB_VA == get_hw_sub_board_id()))
		    || machine_is_msm7x27a_C8825D())
	{
		ctrl_bl_type = CTRL_BL_BY_LCD;
	}
	else
	{
		ctrl_bl_type = CTRL_BL_BY_LCD;
	}

    return ctrl_bl_type;
}

/*
 *brief: get lcd panel resolution
 */
lcd_type get_hw_lcd_resolution_type(void)
{
    lcd_type lcd_resolution = LCD_IS_HVGA;

	if ( machine_is_msm7x27a_U8815() || machine_is_msm7x27a_C8820()
		|| machine_is_msm7x27a_C8825D() )
	{
		lcd_resolution = LCD_IS_WVGA;
	}
	/* U8661 uses HVGA */
	else if ( machine_is_msm7x27a_M660() || machine_is_msm7x27a_U8655()	
		|| machine_is_msm7x27a_U8655_EMMC()|| machine_is_msm7x27a_C8655_NAND()
		|| machine_is_msm7x27a_U8661())
	{
		lcd_resolution = LCD_IS_HVGA;
	}
	else if (machine_is_msm7x27a_U8185())
	{
		lcd_resolution = LCD_IS_QVGA;
	}
	else
	{
		lcd_resolution = LCD_IS_HVGA;
	}
   
    return lcd_resolution;
}

lcd_panel_type get_lcd_panel_type(void)
{
	 lcd_panel_type hw_lcd_panel = LCD_NONE;
	 unsigned int lcd_id = LCD_NONE;

	 lcd_id = get_hw_lcd_id();

	 if( machine_is_msm7x27a_umts()||machine_is_msm7x27a_cdma())
	 {
		 switch (lcd_id)
		 {
			case 0:
				hw_lcd_panel = MIPI_NT35560_TOSHIBA_FWVGA;
				break;
			case 1:
				hw_lcd_panel = LCD_HX8357B_TIANMA_HVGA;
				break;

			default: 
				hw_lcd_panel = LCD_HX8357B_TIANMA_HVGA;
				break;
		 }
	 }
    else if( machine_is_msm7x27a_U8815() ||
             machine_is_msm7x27a_C8820() ||
             machine_is_msm7x27a_C8825D() )
	 {
		 switch (lcd_id)
		 {
			case 0:
				hw_lcd_panel = MIPI_RSP61408_CHIMEI_WVGA;
				break;
			case 1:
				hw_lcd_panel = MIPI_HX8369A_TIANMA_WVGA;
				break;			
			case 2:
				hw_lcd_panel = MIPI_RSP61408_BYD_WVGA;
				break;
			case 3:
				hw_lcd_panel = MIPI_RSP61408_TRULY_WVGA;
				break;

			default: 
				/*no mipi LCD lead to block, so default lcd RGB */
				hw_lcd_panel = LCD_HX8357B_TIANMA_HVGA;
				break;
		 }
	 }
	 /* add U8655_EMMC, use the u8655 configuration */
	 else if( machine_is_msm7x27a_U8655() || machine_is_msm7x27a_U8655_EMMC() || machine_is_msm7x27a_C8655_NAND()||
	          machine_is_msm7x27a_U8661())
	 {
		 switch (lcd_id)
		 {
			/*this IC stand id number 0 and 3,temp add*/
			case 0:
				hw_lcd_panel = MIPI_HX8357C_CHIMEI_HVGA;
				break;
			case 1:
			    hw_lcd_panel = MIPI_HX8357C_TIANMA_IPS_HVGA;
				break;
			case 2:
				hw_lcd_panel = MIPI_HX8357C_CHIMEI_IPS_HVGA;
				break;
			case 3:
				hw_lcd_panel = MIPI_HX8357C_TIANMA_HVGA;
				break;

			default: 
				/*no mipi LCD lead to block, so default lcd RGB */
				hw_lcd_panel = LCD_HX8357B_TIANMA_HVGA;
				break;
		 }
	 }
	 else if( machine_is_msm7x27a_U8185())
	 {
	 	switch(lcd_id)
		{
			case 0:
				hw_lcd_panel = LCD_HX8347D_TRULY_QVGA;
				break;
			case 2:
				hw_lcd_panel = LCD_HX8347G_TIANMA_QVGA;
				break;
			case 3:
				hw_lcd_panel = LCD_HX8347D_CHIMEI_QVGA;
				break;
			default:
				hw_lcd_panel = LCD_HX8347G_TIANMA_QVGA;
				break;
	 	}
	 }
	 else if(machine_is_msm7x27a_M660())
 	 {
 	 	switch(lcd_id)
 	 	{
 	 		case 0:
				hw_lcd_panel = LCD_HX8357C_TIANMA_HVGA;
				break;
			case 1:
				hw_lcd_panel = LCD_HX8357B_TIANMA_HVGA;
				break;		
			default:
				hw_lcd_panel = LCD_HX8357C_TIANMA_HVGA;
				break;
 	 	}
 	 }	 
	 else 
	 {
		hw_lcd_panel = LCD_HX8357B_TIANMA_HVGA;
	 }
	 return hw_lcd_panel;
}

/*===========================================================================


FUNCTION     get_lcd_align_type

DESCRIPTION
  This function probe which LCD align type should be used

DEPENDENCIES
  
RETURN VALUE
  None

SIDE EFFECTS
  None
===========================================================================*/
lcd_align_type get_lcd_align_type (void)
{
    lcd_panel_type  hw_lcd_panel = LCD_NONE;
    lcd_align_type  lcd_align    = LCD_PANEL_ALIGN_LSB;
     
	hw_lcd_panel = get_lcd_panel_type();
	if ((hw_lcd_panel == LCD_ILI9481DS_TIANMA_HVGA) ||(hw_lcd_panel == LCD_ILI9481D_INNOLUX_HVGA))
	{
		lcd_align = LCD_PANEL_ALIGN_MSB;
	}
	else
	{
		lcd_align = LCD_PANEL_ALIGN_LSB;
	}

    return lcd_align;
}
char *get_lcd_panel_name(void)
{
	 lcd_panel_type hw_lcd_panel = LCD_NONE;
	 char *pname = NULL;
	 
	 hw_lcd_panel = get_lcd_panel_type();
	 
	 switch (hw_lcd_panel)
	 {
		 case LCD_S6D74A0_SAMSUNG_HVGA:
			 pname = "SAMSUNG S6D74A0";
			 break;
			 
		 case LCD_ILI9325_INNOLUX_QVGA:
			 pname = "INNOLUX ILI9325";
			 break;

		 case LCD_ILI9331B_TIANMA_QVGA:
			 pname = "TIANMA ILI9331B";
			 break;

		 case LCD_ILI9325_BYD_QVGA:
			 pname = "BYD ILI9325";
			 break;

		 case LCD_ILI9325_WINTEK_QVGA:
			 pname = "WINTEK ILI9325";
			 break;

		 case LCD_SPFD5408B_KGM_QVGA:
			 pname = "KGM SPFD5408B";
			 break;

		 case LCD_HX8368A_TRULY_QVGA:
			 pname = "TRULY HX8368A";
			 break;

		 case LCD_HX8368A_SEIKO_QVGA:
			 pname = "SEIKO HX8368A";
			 break;

		 case LCD_HX8347D_TRULY_QVGA:
			 pname = "TRULY HX8347D";
			 break;

		 case LCD_HX8347D_INNOLUX_QVGA:
			 pname = "INNOLUX HX8347D";
			 break;
		 case LCD_HX8347D_CHIMEI_QVGA:
			 pname = "CHIMEI HX8347D";
		 	 break;
		 case LCD_HX8347G_TIANMA_QVGA:
			 pname = "TIANMA HX8347G";
		 	 break;
		 case LCD_ILI9325C_WINTEK_QVGA:
			 pname = "WINTEK ILI9325C";
			 break;

		 case LCD_HX8357A_TRULY_HVGA:
			 pname = "TRULY HX8357A";
			 break;

		 case LCD_HX8357A_WINTEK_HVGA:
			 pname = "WIMTEK HX8357A";
			 break;
             
         case LCD_HX8357B_TIANMA_HVGA:
             pname = "TIANMA HX8357B";
             break;  
 		case LCD_HX8357C_TIANMA_HVGA:
			 pname = "TIANMA HX8357C";
			 break;

		 case LCD_R61529_TRULY_HVGA:
			 pname = "TRULY R61529";
			 break;                        

        case LCD_S6D05A0_INNOLUX_HVGA:
            pname = "INNOLUX S6D05A0";
			 break;

		case LCD_MDDI_NT35582_BYD_WVGA:
			pname = "BYD NT35582";
			break;

		case LCD_MDDI_NT35582_TRULY_WVGA:
			pname = "TRULY NT35582";
			break;

		case LCD_MDDI_NT35510_ALPHA_SI_WVGA:
			pname =  "TRULY NT35510";
			break;

		case LCD_ILI9481DS_TIANMA_HVGA:
			pname = "TIANMA ILI9481";
			break;

		case LCD_ILI9481D_INNOLUX_HVGA:
			pname = "INNOLUX ILI9481";
			break;

        case LCD_NT35410_CHIMEI_HVGA:
            pname = "CHIMEI NT35410";
            break;
        case MIPI_RSP61408_CHIMEI_WVGA:
            pname = "CHIMEI RSP61408";
            break;
        case MIPI_RSP61408_BYD_WVGA:
            pname = "BYD RSP61408";
            break;
        case MIPI_HX8357C_CHIMEI_HVGA:
            pname = "CHIMEI HX8357C";
            break;
        case MIPI_HX8357C_TIANMA_HVGA:
            pname = "TIANMA HX8357C";
            break;
        case MIPI_HX8369A_TIANMA_WVGA:
            pname = "TIANMA HX8369A";
            break;
        case MIPI_HX8357C_CHIMEI_IPS_HVGA:
            pname = "CHIMEI IPS HX8357C";
            break;
		case MIPI_HX8357C_TIANMA_IPS_HVGA:
		    pname = "TIANMA IPS HX8357C";
		    break;
		case MIPI_RSP61408_TRULY_WVGA:
			pname = "TRULY RSP61408";
			break;

		 default:
			 pname = "UNKNOWN LCD";
			 break;
	 }

     return pname;
}

int board_surport_fingers(bool * is_surport_fingers)
{
	 int result = 0;

	 if (is_surport_fingers == NULL)
	 {
		  return -ENOMEM;
	 }
	 
	 *is_surport_fingers = false;

	 return result;
}

int board_use_tssc_touch(bool * use_touch_key)
{
	 int result = 0;

	 *use_touch_key = false;
	 return result;
}

int board_support_ofn(bool * ofn_support)
{
	 int ret = 0;

	 if(NULL == ofn_support)
	 {
		 return -EPERM;
	 }

	 *ofn_support = false;
	 return ret;
}

static bool camera_i2c_state = false;
bool camera_is_supported(void)
{
	 return camera_i2c_state;
}

void set_camera_support(bool status)
{
	 camera_i2c_state = status;
}

bool board_support_flash(void)
{
	 /*only U8815 has light flash for now*/
	 if(machine_is_msm7x27a_U8815())
	 {
		 return true;
	 }
	 return false;
}
static bool st303_gs_state = false;
bool st303_gs_is_supported(void)
{
	 return st303_gs_state;
}

void set_st303_gs_support(bool status)
{
	 st303_gs_state = status;
}

/*
*  return: 0 ----not support RGB LED driver
* 		 1 ----support RGB LED driver
*/
bool rgb_led_is_supported(void)
{
	bool ret = false;

	return ret;
}

bool qwerty_is_supported(void)
{
	bool ret = false;
	ret=( machine_is_msm7x27a_umts()||machine_is_msm7x27a_cdma() || machine_is_msm7x27a_M660());
	return ret;
}
/* get sensors list by product */
static int get_sensors_list(void)
{
	int sensors_list = G_SENSOR + L_SENSOR + P_SENSOR + M_SENSOR;
	/*version A and version B has compass, since version C don't have compass*/
	/*add M866*/
	if( machine_is_msm7x27a_U8661() || ( machine_is_msm7x27a_C8820() && (HW_VER_SUB_VC <= get_hw_sub_board_id()))
		  || ( machine_is_msm7x27a_C8655_NAND() && (HW_VER_SUB_VE > get_hw_sub_board_id())) )
	{
	    sensors_list = G_SENSOR + L_SENSOR + P_SENSOR;
	}
	else if (machine_is_msm7x27a_U8655() ||
	    machine_is_msm7x27a_U8655_EMMC()||
	    machine_is_msm7x27a_C8655_NAND() ||
	    machine_is_msm7x27a_M660()||
	    machine_is_msm7x27a_U8815()||
	    machine_is_msm7x27a_C8820()||
	    machine_is_msm7x27a_C8825D())
	{
	    sensors_list = G_SENSOR + L_SENSOR + P_SENSOR + M_SENSOR;
	}
	/* move this part to up */
	else if(machine_is_msm7x27a_U8185())
	{
	    sensors_list = G_SENSOR;
	}
	else
	{
	    sensors_list = G_SENSOR + L_SENSOR + P_SENSOR + M_SENSOR;
	}
	return sensors_list;
}

char *get_sensors_list_name(void)
{
	int sensors_list = G_SENSOR + L_SENSOR + P_SENSOR + M_SENSOR;
	char *list_name=NULL;

	sensors_list = get_sensors_list();

	switch(sensors_list)
	{
		case G_SENSOR + L_SENSOR + P_SENSOR + M_SENSOR + GY_SENSOR:
			 list_name = "G_L_P_M_GY_SENSORS";
			 break;
			 
		case G_SENSOR + L_SENSOR + P_SENSOR + M_SENSOR:
			 list_name = "G_L_P_M_SENSORS";
			 break;
			 
		case G_SENSOR + L_SENSOR + P_SENSOR:
			 list_name = "G_L_P_SENSORS";
			 break;
			 
		case G_SENSOR:
			 list_name = "G_SENSORS";
			 break;
			 
		case G_SENSOR + M_SENSOR + GY_SENSOR:
			 list_name = "G_M_GY_SENSORS";
			 break;
			 
		case G_SENSOR + M_SENSOR:
			 list_name = "G_M_SENSORS";
			 break;
			 
		case NONE_SENSOR:
			 list_name = "NONE_SENSORS";
			 break;
			 
		default:
			 list_name = "G_L_P_M_SENSORS";
			 break;
	}

	return list_name;
	
}
/*return the string by compass position*/
char *get_compass_gs_position_name(void)
{
	compass_gs_position_type compass_gs_position=COMPASS_TOP_GS_TOP;
	char *position_name=NULL;

	compass_gs_position = get_compass_gs_position();

	switch(compass_gs_position)
	{
		case COMPASS_TOP_GS_TOP:
			 position_name = "COMPASS_TOP_GS_TOP";
			 break;
			 
		case COMPASS_TOP_GS_BOTTOM:
			 position_name = "COMPASS_TOP_GS_BOTTOM";
			 break;

		case COMPASS_BOTTOM_GS_TOP:
			 position_name = "COMPASS_BOTTOM_GS_TOP";
			 break;

		case COMPASS_BOTTOM_GS_BOTTOM:
			 position_name = "COMPASS_BOTTOM_GS_BOTTOM";
			 break;
			 
		case COMPASS_NONE_GS_BOTTOM:
			 position_name = "COMPASS_NONE_GS_BOTTOM";
			 break;
		/*add gs position of COMPASS_NONE_GS_TOP*/			 
		case COMPASS_NONE_GS_TOP:
			 position_name = "COMPASS_NONE_GS_TOP";
			 break;

		default:
			 position_name = "COMPASS_TOP_GS_TOP";
			 break;
	}

	return position_name;
	
}
/*  FUNCTION  get_hw_lcd_interface_type
 *  DEPENDENCIES 
 *      get lcd interface type
 *      affect nfc.
 *  RETURN VALUE
 *      lcd interface type:LCD_IS_MIPI or LCD_IS_RGB
 */
hw_lcd_interface_type get_hw_lcd_interface_type(void)
{
  if(machine_is_msm7x27a_U8185()||machine_is_msm7x27a_M660())
  {
     return LCD_IS_RGB;
  }
  else
  {
     return LCD_IS_MIPI;
  }
}

/*  FUNCTION  get_hw_wifi_device_type
 *  DEPENDENCIES 
 *      get wifi device type.
 *      affect wifi and camer.
 *  RETURN VALUE
 *      wifi device type:WIFI_QUALCOMM or WIFI_BROADCOM
 */
hw_wifi_device_type get_hw_wifi_device_type(void)
{
  if(machine_is_msm7x27a_U8185()|| machine_is_msm7x27a_U8661())
  {
      return WIFI_QUALCOMM;
  }
  else
  {
      return WIFI_BROADCOM;
  }
}
/*  FUNCTION  get_hw_wifi_device_model
 *  DEPENDENCIES 
 *      get wifi device model.
 *      affect app_info.
 *  RETURN VALUE
 *      wifi device model:WIFI_QUALCOMM_6005 or WIFI_BROADCOM_4329 ro WIFI_BROADCOM_4330
 */
hw_wifi_device_model get_hw_wifi_device_model(void)
{
  if(machine_is_msm7x27a_U8185()|| machine_is_msm7x27a_U8661())
  {
      return WIFI_QUALCOMM_6005;
  }
  else
  {
      return WIFI_BROADCOM_4330;
  }
}
/*  FUNCTION  get_hw_ds_type
 *  DEPENDENCIES 
 *      get single sim card or double sim card,
 *      affect led.
 *  RETURN VALUE
 *      single sim card:sim card type HW_NODS 
 *      double sim card:sim card type HW_DS
 */
hw_ds_type get_hw_ds_type(void)
{
    hw_ds_type ret = HW_NONES;
    if( machine_is_msm7x27a_C8820() || machine_is_msm7x27a_C8825D() || machine_is_msm7x27a_U8661())
    {
	    ret = HW_DS;
    }
    else
    {
      ret = HW_NODS;
    }
  return ret;
}
/*  FUNCTION  get_hw_sd_trigger_type
 *  DEPENDENCIES 
 *      get sd interrupt trigger type
 *      affect sd detect.
 *  RETURN VALUE
 *      raise edge trigger : return RAISE_TRIGGER
 *      fall edge trigger : return FALL_TRIGGER
 */
hw_sd_trigger_type get_hw_sd_trigger_type(void)
{
  if(machine_is_msm7x27a_U8185())
  {
      return RAISE_TRIGGER;
  }
  else
  {
      return FALL_TRIGGER;
  }
}

/*  FUNCTION  get_hw_sd_trigger_type
 *  DESCRIPTION 
 *      get the bt wakeup gpio type
 *
 *  RETURN VALUE
 *       the gpio number
 */
hw_bt_wakeup_gpio_type get_hw_bt_wakeup_gpio_type(void)
{
    hw_bt_wakeup_gpio_type bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_NONES;
    hw_ver_sub_type ver_sub_type = HW_VER_SUB_MAX;
    ver_sub_type = get_hw_sub_board_id();
	
    if (machine_is_msm7x27a_U8815())
    {
        bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_83;
    }
    else if (machine_is_msm7x27a_U8655())
    {
        bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_83;
    }
    else if (machine_is_msm7x27a_U8655_EMMC())
    {
        bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_83;
    }
    else if (machine_is_msm7x27a_C8655_NAND())
    {
        if (ver_sub_type > HW_VER_SUB_VB)
		{
            bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_27;
		}
		else
		{
            bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_83;
		}
    }
    else if (machine_is_msm7x27a_M660())
    {
        if (ver_sub_type > HW_VER_SUB_VA)
		{
            bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_27;
		}
		else
		{
            bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_83;
		}
    }
    else if (machine_is_msm7x27a_C8820())
    {
        if (ver_sub_type > HW_VER_SUB_VB)
		{
            bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_27;
		}
		else
		{
            bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_83;
		}
    }
    else if (machine_is_msm7x27a_C8825D())
    {
        bt_wakeup_gpio_type = HW_BT_WAKEUP_GPIO_IS_83;
    }
    	
    printk(KERN_INFO "the bt_wakeup_gpio_type is %d\n", bt_wakeup_gpio_type);
    return bt_wakeup_gpio_type;
}

/* add U8661 for SINGLE_MIC */
/* add C8820VC for SINGLE_MIC */
audio_property_type get_audio_mic_type(void)
{
  if(machine_is_msm7x27a_U8185() || machine_is_msm7x27a_U8661()
     || (machine_is_msm7x27a_C8820() && (HW_VER_SUB_VC <= get_hw_sub_board_id()))
	 )
  {
      return SINGLE_MIC;
  }
  else
  {
      return DUAL_MIC;
  }  
}

/* if you want to enable fir function, please return FIR_ENABLE for adapted project */
audio_property_type get_audio_fir_enabled(void)
{
    return FIR_DISABLE;
}
audio_property_type get_audio_fm_type(void)
{
   if (machine_is_msm7x27a_U8185()
	|| machine_is_msm7x27a_U8661())
   {
       return FM_QUALCOMM;
   }
   else
   {
       return FM_BROADCOM;
   }
}

hw_camera_type get_hw_camera_mirror_type(void)
{
    hw_camera_type ret = HW_CAMERA_NONES;
    if( machine_is_msm7x27a_C8820() || machine_is_msm7x27a_C8825D() || machine_is_msm7x27a_U8661())
    {
	    ret = HW_MIRROR_AND_FLIP;
    }
    else
    {
      ret = HW_NOT_MIRROR_OR_FLIP;
    }
  return ret;
}

/* get touch info */
char * get_touch_info(void)
{
	char * touch_info = NULL;
	touch_info = get_synaptics_touch_info();
	if (touch_info != NULL)
		return touch_info;

	touch_info = get_melfas_touch_info();
	if (touch_info != NULL)
		return touch_info;

	return NULL;
	
}
