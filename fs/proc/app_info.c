/*
 *  linux/fs/proc/app_info.c
 *
 *
 * Changes:
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/fs.h>
#include <linux/tty.h>
#include <linux/string.h>
#include <linux/mman.h>
#include <linux/quicklist.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/genhd.h>
#include <linux/smp.h>
#include <linux/signal.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/times.h>
#include <linux/profile.h>
#include <linux/utsname.h>
#include <linux/blkdev.h>
#include <linux/hugetlb.h>
#include <linux/jiffies.h>
#include <linux/sysrq.h>
#include <linux/vmalloc.h>
#include <linux/crash_dump.h>
#include <linux/pid_namespace.h>
#include <linux/bootmem.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/tlb.h>
#include <asm/div64.h>
#include "internal.h"
#include <asm/setup.h>
#include <asm/mach-types.h>
#include "linux/hardware_self_adapt.h"

#define PROC_MANUFACTURER_STR_LEN 30
#define MAX_VERSION_CHAR 40
#define LCD_NAME_LEN 20
/* MSM7X27A the length of BOARD_ID and HW_VERSION increase */
#define BOARD_ID_LEN 30
#define HW_VERSION   30
#define HW_VERSION_SUB_VER  10
#define SUB_VER_LEN  10
#define AUDIO_PROPERTY_LEN 20
static char appsboot_version[MAX_VERSION_CHAR + 1];
static char str_flash_nand_id[PROC_MANUFACTURER_STR_LEN] = {0};
static u32 camera_id;
static u32 ts_id;
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
static u32 charge_flag;
extern unsigned int get_charge_flag(void);
#endif
/*del 1 line*/

#ifdef CONFIG_HUAWEI_KERNEL
extern void get_flash_id(char *,int);
#endif

typedef struct
{
   int  mach_type; 
   char s_board_id[BOARD_ID_LEN];
   char hw_version_id[HW_VERSION];
}s_board_hw_version_type;

/* this is s_board_id and hw_version_id list,
 * when you want to add s_board_id and hw_version_if for new product,
 * add you s_board_id and hw_version_id this list.
 */
const s_board_hw_version_type s_board_hw_version_table[] =
{  /* machine_arch_type        s_board_id      hw_version_id */
   {MACH_TYPE_MSM7X27A_U8815, "MSM7227A_U8815", "HD1U861M"},
   {MACH_TYPE_MSM7X27A_U8655, "MSM7225A_U8655", "HD1U8655M"},
   {MACH_TYPE_MSM7X27A_U8655_EMMC, "MSM7225A_U8665", "HD2U8655M"},
   {MACH_TYPE_MSM7X27A_C8655_NAND, "MSM7625A_C8655", "HC1C8655M"},
   {MACH_TYPE_MSM7X27A_U8185, "MSM7225A_U8185", "HD1U8185M"},
   {MACH_TYPE_MSM7X27A_UMTS, "MSM7X27A_UMTS", "BOM_MSM7X27A_UMTS"},
   {MACH_TYPE_MSM7X27A_CDMA, "MSM7X27A_CDMA", "BOM_MSM7X27A_CDMA"},
   {MACH_TYPE_MSM7X27A_M660, "MSM7627A_M660", "HC1M660M"},
   /*add comma for project menu*/
   {MACH_TYPE_MSM7X27A_C8820, "MSM7627A_C8820","HC1C8820M"},
   {MACH_TYPE_MSM7X27A_C8825D, "MSM7627A_C8825D","HC1C8820M"},
   {MACH_TYPE_MSM7X27A_U8661, "MSM7225A_U8661","HD1U8661M"},
};

void set_s_board_hw_version(char *s_board_id,char *hw_version_id)
{  
    int temp_num = 0;
    int table_num = 0;

    if ((NULL == s_board_id) || (NULL == hw_version_id))
    {
         printk("app_info : s_board_id or hw_version_type is null!\n");    
         return ;
    }

    table_num = sizeof(s_board_hw_version_table)/sizeof(s_board_hw_version_type);
    for(temp_num = 0;temp_num < table_num;temp_num++)
    {
         if(s_board_hw_version_table[temp_num].mach_type == machine_arch_type )
         {
             memcpy(s_board_id,s_board_hw_version_table[temp_num].s_board_id, BOARD_ID_LEN-1);
             memcpy(hw_version_id,s_board_hw_version_table[temp_num].hw_version_id, HW_VERSION-1);
             break;
         }
    }

    if(table_num == temp_num)
    {
        memcpy(s_board_id,"ERROR", (BOARD_ID_LEN-1));
        memcpy(hw_version_id,"ERROR", HW_VERSION-1);
    }
}

/*===========================================================================


FUNCTION     set_s_board_hw_version_special

DESCRIPTION
  This function deal with special hw_version_id s_board_id and so on
DEPENDENCIES
  
RETURN VALUE
  None

SIDE EFFECTS
  None
===========================================================================*/
static void set_s_board_hw_version_special(char*hw_version_id,char*hw_version_sub_ver,
                                char*s_board_id,char*sub_ver)
{
	/* change sub version to right version*/
    if((HW_VER_SUB_VB <= get_hw_sub_board_id()) &&
            (MACH_TYPE_MSM7X27A_U8815 == machine_arch_type))
    {
        memcpy(hw_version_id,"HD1U8815M", BOARD_ID_LEN-1);
        sprintf(hw_version_sub_ver, ".Ver%c", 'A'+(char)get_hw_sub_board_id());
        strcat(hw_version_id, hw_version_sub_ver);
        hw_version_id[HW_VERSION-1] = '\0';

    }    
     
    /* change U8185 hw_version to VerB */
    if((HW_VER_SUB_VE > get_hw_sub_board_id()) &&
            (MACH_TYPE_MSM7X27A_U8185 == machine_arch_type))
    {
        memcpy(hw_version_id,"HD1U8185M", BOARD_ID_LEN-1);
        sprintf(hw_version_sub_ver, ".Ver%c", 'A'+(char)get_hw_sub_board_id() + 1);
        strcat(hw_version_id, hw_version_sub_ver);
        hw_version_id[HW_VERSION-1] = '\0';
    }

    /* add U8186 hw_version */
    /* and the U8186 sub ver keep same as U8185 sub ver. */
    if((HW_VER_SUB_VE <= get_hw_sub_board_id()) &&
            (MACH_TYPE_MSM7X27A_U8185 == machine_arch_type))
    {
        memcpy(hw_version_id,"HD1U8186M", BOARD_ID_LEN-1);
        sprintf(hw_version_sub_ver, ".Ver%c", 'A'+(char)get_hw_sub_board_id() - 3);
        strcat(hw_version_id, hw_version_sub_ver);
        hw_version_id[HW_VERSION-1] = '\0';
    }

    /* change sub version to right version */
    if((HW_VER_SUB_VE <= get_hw_sub_board_id()) &&
           (MACH_TYPE_MSM7X27A_U8655_EMMC == machine_arch_type))
    {
       memcpy(hw_version_id,"HD2U8655M", BOARD_ID_LEN-1);
       sprintf(hw_version_sub_ver, ".Ver%c", 'A'+(char)get_hw_sub_board_id() -3);
       strcat(hw_version_id, hw_version_sub_ver);
       hw_version_id[HW_VERSION-1] = '\0';
    }
    /* add M866 HW_version */
    if((HW_VER_SUB_VE <= get_hw_sub_board_id()) &&
           (MACH_TYPE_MSM7X27A_C8655_NAND == machine_arch_type))
    {
       memcpy(hw_version_id,"HC1M866M", BOARD_ID_LEN-1);
       sprintf(hw_version_sub_ver, ".Ver%c", 'A'+(char)get_hw_sub_board_id() -4);
       strcat(hw_version_id, hw_version_sub_ver);
       hw_version_id[HW_VERSION-1] = '\0';
    }
}


/* same as in proc_misc.c */
static int
proc_calc_metrics(char *page, char **start, off_t off, int count, int *eof, int len)
{
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

#define ATAG_BOOT_READ_FLASH_ID 0x4d534D72
static int __init parse_tag_boot_flash_id(const struct tag *tag)
{
    char *tag_flash_id=(char*)&tag->u;
    memset(str_flash_nand_id, 0, PROC_MANUFACTURER_STR_LEN);
    memcpy(str_flash_nand_id, tag_flash_id, PROC_MANUFACTURER_STR_LEN);
    
    printk("########proc_misc.c: tag_boot_flash_id= %s\n", tag_flash_id);

    return 0;
}
__tagtable(ATAG_BOOT_READ_FLASH_ID, parse_tag_boot_flash_id);

/*parse atag passed by appsboot, ligang 00133091, 2009-4-13, start*/
#define ATAG_BOOT_VERSION 0x4d534D71 /* ATAG BOOT VERSION */
static int __init parse_tag_boot_version(const struct tag *tag)
{
    char *tag_boot_ver=(char*)&tag->u;
    memset(appsboot_version, 0, MAX_VERSION_CHAR + 1);
    memcpy(appsboot_version, tag_boot_ver, MAX_VERSION_CHAR);
     
    //printk("nand_partitions.c: appsboot_version= %s\n\n", appsboot_version);

    return 0;
}
__tagtable(ATAG_BOOT_VERSION, parse_tag_boot_version);


#define ATAG_CAMERA_ID 0x4d534D74
static int __init parse_tag_camera_id(const struct tag *tag)
{
    char *tag_boot_ver=(char*)&tag->u;
	
    memcpy((void*)&camera_id, tag_boot_ver, sizeof(u32));
     
    return 0;
}
__tagtable(ATAG_CAMERA_ID, parse_tag_camera_id);


#define ATAG_TS_ID 0x4d534D75
static int __init parse_tag_ts_id(const struct tag *tag)
{
    char *tag_boot_ver=(char*)&tag->u;
	
    memcpy((void*)&ts_id, tag_boot_ver, sizeof(u32));
     
    return 0;
}

__tagtable(ATAG_TS_ID, parse_tag_ts_id);


/*write the position of compass into the file of "/proc/app_info" */
static int app_version_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	char *compass_gs_name = NULL;
	char *sensors_list_name = NULL;
    /*declare s_board_id¡¢sub_ver¡¢hw_version_id and hw_version*/
	char s_board_id[BOARD_ID_LEN] = {0};
	char sub_ver[SUB_VER_LEN] = {0};
	char hw_version_id[HW_VERSION] = {0};
	char hw_version_sub_ver[HW_VERSION_SUB_VER] = {0};	
	char *lcd_name = NULL;
	char * touch_info = NULL;
	char *wifi_device_name = NULL;
	char audio_property[AUDIO_PROPERTY_LEN] = {0};
		charge_flag = get_charge_flag();
    set_s_board_hw_version(s_board_id,hw_version_id);
    /*read sub_board_id from smem,depend on sub_board_id,evaluate sub_ver and hw_version_sub_ver*/	
	sprintf(sub_ver, ".Ver%c", 'A'+(char)get_hw_sub_board_id());
	sprintf(hw_version_sub_ver, ".Ver%c", 'A'+(char)get_hw_sub_board_id());
	strcat(s_board_id, sub_ver);
	s_board_id[BOARD_ID_LEN-1] = '\0';
	strcat(hw_version_id, hw_version_sub_ver);
	hw_version_id[HW_VERSION-1] = '\0';

    set_s_board_hw_version_special(hw_version_id,hw_version_sub_ver,s_board_id,sub_ver);


	compass_gs_name=get_compass_gs_position_name();
	sensors_list_name = get_sensors_list_name();

	lcd_name = get_lcd_panel_name();
	if( NULL == lcd_name)
	{
	    lcd_name = "UNKNOWN LCD";
	}
	wifi_device_name = get_wifi_device_name();
	get_audio_property(audio_property);
	touch_info = get_touch_info();
	if (touch_info == NULL)
	{
		touch_info = "Unknow touch";
	}
#ifdef CONFIG_HUAWEI_KERNEL	
    memset(str_flash_nand_id,0,PROC_MANUFACTURER_STR_LEN);
    get_flash_id(str_flash_nand_id,PROC_MANUFACTURER_STR_LEN);
#endif

    /* write the power down charge flag to the file /proc/app_info,
     * so we can read the flag in recovery mode to decide we enter  
     * the recovery mode or power down charge movie
     */
	len = snprintf(page, PAGE_SIZE,
	"board_id:\n%s\n"
	"hw_version:\n%s\n"
	"charge_flag:\n%d\n"
	"compass_gs_position:\n%s\n"
	"sensors_list:\n%s\n"
	"FLASH_ID:\n%s\n"
	"lcd_id:\n%s\n"
	"wifi_chip:\n%s\n"
	"audio_property:\n%s\n"
	"touch_info:\n%s\n",s_board_id, 
	hw_version_id,charge_flag,compass_gs_name,
	sensors_list_name,str_flash_nand_id,lcd_name,
	wifi_device_name,audio_property,touch_info);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

void __init proc_app_info_init(void)
{
	static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
	} *p, simple_ones[] = {
		
        {"app_info", app_version_read_proc},
		{NULL,}
	};
	for (p = simple_ones; p->name; p++)
		create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);

}


