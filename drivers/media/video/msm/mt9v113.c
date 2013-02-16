
/*
 * Copyright (c) 2008-2009 QUALCOMM USA, INC.
 *
 * All source code in this file is licensed under the following license
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <linux/slab.h>
#include "mt9v113.h"
#include "linux/hardware_self_adapt.h"

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
 #include <linux/hw_dev_dec.h>
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9V113
 #undef CDBG
 #define CDBG(fmt, args...) printk(KERN_INFO "mt9v113.c: " fmt, ## args)
#endif


/*=============================================================
    SENSOR REGISTER DEFINES
==============================================================*/
#define MT9V113_REG_MODEL_ID 0x0000
#define MT9V113_CHIP_ID 0x2280
#define MT9V113_DEFAULT_CLOCK_RATE 12000000



 
enum mt9v113_test_mode_t
{
    TEST_OFF,
    TEST_1,
    TEST_2,
    TEST_3
};

enum mt9v113_resolution_t
{
    QTR_SIZE,
    FULL_SIZE,
    INVALID_SIZE
};



static bool CSI_CONFIG;

enum mt9v113_reg_update_t
{
    /* Sensor egisters that need to be updated during initialization */
    REG_INIT,

    /* Sensor egisters that needs periodic I2C writes */
    UPDATE_PERIODIC,

    /* All the sensor Registers will be updated */
    UPDATE_ALL,

    /* Not valid update */
    UPDATE_INVALID
};

enum mt9v113_setting_t
{
    RES_PREVIEW,
    RES_CAPTURE
};






struct mt9v113_work_t
{
    struct work_struct work;
};

struct mt9v113_ctrl_t
{
    const struct  msm_camera_sensor_info *sensordata;

    int      sensormode;
    uint32_t fps_divider; /* init to 1 * 0x00000400 */
    uint32_t pict_fps_divider; /* init to 1 * 0x00000400 */

    uint16_t curr_lens_pos;
    uint16_t init_curr_lens_pos;
    uint16_t my_reg_gain;
    uint32_t my_reg_line_count;

    enum mt9v113_resolution_t prev_res;
    enum mt9v113_resolution_t pict_res;
    enum mt9v113_resolution_t curr_res;
    enum mt9v113_test_mode_t  set_test;

    unsigned short imgaddr;
};

typedef enum
{
    E_REGISTER_END,
    E_REGISTER_CMD_8BIT,
    E_REGISTER_CMD_16BIT,
    E_REGISTER_WAIT,
    E_REGISTER_MAX,
} e_cmd_type;

struct mt9v113_i2c_reg_conf
{
    e_cmd_type     type;
    unsigned short reg;
    unsigned short value;
};

static struct mt9v113_i2c_reg_conf mt9v113_init_reg_config[] =
{	
	{E_REGISTER_CMD_16BIT, 0x0018, 0x4028},
	{E_REGISTER_WAIT     , 0x0000, 100   },//delay 100
	{E_REGISTER_CMD_16BIT, 0x001A, 0x0011},
	{E_REGISTER_CMD_16BIT, 0x001A, 0x0010},
	{E_REGISTER_CMD_16BIT, 0x0018, 0x4028},
	{E_REGISTER_WAIT     , 0x0000, 100   },//delay 100
	{E_REGISTER_CMD_16BIT, 0x098C, 0x02F0},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0000},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x02F2},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0210},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x02F4},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x001A},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2145},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x02F4},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA134},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0001},
	{E_REGISTER_CMD_16BIT, 0x31E0, 0x0001},
	{E_REGISTER_CMD_16BIT, 0x001A, 0x0010},
	{E_REGISTER_CMD_16BIT, 0x3400, 0x7A28},
	{E_REGISTER_CMD_16BIT, 0x321C, 0x8003},
	{E_REGISTER_CMD_16BIT, 0x001E, 0x0777},
	{E_REGISTER_CMD_16BIT, 0x0016, 0x42DF},
	{E_REGISTER_CMD_16BIT, 0x0014, 0xB04B},
	{E_REGISTER_CMD_16BIT, 0x0014, 0xB049},
	{E_REGISTER_CMD_16BIT, 0x0010, 0x0631},
	{E_REGISTER_CMD_16BIT, 0x0012, 0x0000},
	{E_REGISTER_CMD_16BIT, 0x0014, 0x244B},
	{E_REGISTER_CMD_16BIT, 0x0014, 0x304B},
	{E_REGISTER_WAIT     , 0x0000, 100   },//delay 100
	{E_REGISTER_CMD_16BIT, 0x0014, 0xB04A},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xAB1F},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x00C7},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xAB31},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x001E},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x274F},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0004},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2741},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0004},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xAB20},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0054},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xAB21},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0046},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xAB22},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0002},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xAB24},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0005},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2B28},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x170C},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2B2A},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x3E80},
	{E_REGISTER_CMD_16BIT, 0x3210, 0x09B0},
	{E_REGISTER_CMD_16BIT, 0x364E, 0x0A90},
	{E_REGISTER_CMD_16BIT, 0x3650, 0x8F8C},
	{E_REGISTER_CMD_16BIT, 0x3652, 0x8F6F},
	{E_REGISTER_CMD_16BIT, 0x3654, 0x4509},
	{E_REGISTER_CMD_16BIT, 0x3656, 0x450B},
	{E_REGISTER_CMD_16BIT, 0x3658, 0x0A90},
	{E_REGISTER_CMD_16BIT, 0x365A, 0x8F8C},
	{E_REGISTER_CMD_16BIT, 0x365C, 0x8F6F},
	{E_REGISTER_CMD_16BIT, 0x365E, 0x4509},
	{E_REGISTER_CMD_16BIT, 0x3660, 0x450B},
	{E_REGISTER_CMD_16BIT, 0x3662, 0x0A90},
	{E_REGISTER_CMD_16BIT, 0x3664, 0x8F8C},
	{E_REGISTER_CMD_16BIT, 0x3666, 0x8F6F},
	{E_REGISTER_CMD_16BIT, 0x3668, 0x4509},
	{E_REGISTER_CMD_16BIT, 0x366A, 0x450B},
	{E_REGISTER_CMD_16BIT, 0x366C, 0x0A90},
	{E_REGISTER_CMD_16BIT, 0x366E, 0x8F8C},
	{E_REGISTER_CMD_16BIT, 0x3670, 0x8F6F},
	{E_REGISTER_CMD_16BIT, 0x3672, 0x4509},
	{E_REGISTER_CMD_16BIT, 0x3674, 0x450B},
	{E_REGISTER_CMD_16BIT, 0x3676, 0x354C},
	{E_REGISTER_CMD_16BIT, 0x3678, 0xF9C6},
	{E_REGISTER_CMD_16BIT, 0x367A, 0xF989},
	{E_REGISTER_CMD_16BIT, 0x367C, 0x50E3},
	{E_REGISTER_CMD_16BIT, 0x367E, 0x5105},
	{E_REGISTER_CMD_16BIT, 0x3680, 0x354C},
	{E_REGISTER_CMD_16BIT, 0x3682, 0xF9C6},
	{E_REGISTER_CMD_16BIT, 0x3684, 0xF989},
	{E_REGISTER_CMD_16BIT, 0x3686, 0x5103},
	{E_REGISTER_CMD_16BIT, 0x3688, 0x5105},
	{E_REGISTER_CMD_16BIT, 0x368A, 0x354C},
	{E_REGISTER_CMD_16BIT, 0x368C, 0xF9C6},
	{E_REGISTER_CMD_16BIT, 0x368E, 0xF9A9},
	{E_REGISTER_CMD_16BIT, 0x3690, 0x5103},
	{E_REGISTER_CMD_16BIT, 0x3692, 0x5105},
	{E_REGISTER_CMD_16BIT, 0x3694, 0x354C},
	{E_REGISTER_CMD_16BIT, 0x3696, 0xF9C6},
	{E_REGISTER_CMD_16BIT, 0x3698, 0xF989},
	{E_REGISTER_CMD_16BIT, 0x369A, 0x5103},
	{E_REGISTER_CMD_16BIT, 0x369C, 0x5105},
	{E_REGISTER_CMD_16BIT, 0x369E, 0x8F6F},
	{E_REGISTER_CMD_16BIT, 0x36A0, 0x45C9},
	{E_REGISTER_CMD_16BIT, 0x36A2, 0x45AC},
	{E_REGISTER_CMD_16BIT, 0x36A4, 0xA306},
	{E_REGISTER_CMD_16BIT, 0x36A6, 0xA308},
	{E_REGISTER_CMD_16BIT, 0x36A8, 0x8F6F},
	{E_REGISTER_CMD_16BIT, 0x36AA, 0x45C9},
	{E_REGISTER_CMD_16BIT, 0x36AC, 0x45AC},
	{E_REGISTER_CMD_16BIT, 0x36AE, 0xA306},
	{E_REGISTER_CMD_16BIT, 0x36B0, 0xA308},
	{E_REGISTER_CMD_16BIT, 0x36B2, 0x8FCF},
	{E_REGISTER_CMD_16BIT, 0x36B4, 0x4629},
	{E_REGISTER_CMD_16BIT, 0x36B6, 0x460C},
	{E_REGISTER_CMD_16BIT, 0x36B8, 0xA386},
	{E_REGISTER_CMD_16BIT, 0x36BA, 0xA388},
	{E_REGISTER_CMD_16BIT, 0x36BC, 0x8FCF},
	{E_REGISTER_CMD_16BIT, 0x36BE, 0x4629},
	{E_REGISTER_CMD_16BIT, 0x36C0, 0x460C},
	{E_REGISTER_CMD_16BIT, 0x36C2, 0xA386},
	{E_REGISTER_CMD_16BIT, 0x36C4, 0xA388},
	{E_REGISTER_CMD_16BIT, 0x36C6, 0xF84E},
	{E_REGISTER_CMD_16BIT, 0x36C8, 0x29A9},
	{E_REGISTER_CMD_16BIT, 0x36CA, 0x296C},
	{E_REGISTER_CMD_16BIT, 0x36CC, 0xADE6},
	{E_REGISTER_CMD_16BIT, 0x36CE, 0xAE28},
	{E_REGISTER_CMD_16BIT, 0x36D0, 0xF84E},
	{E_REGISTER_CMD_16BIT, 0x36D2, 0x29A9},
	{E_REGISTER_CMD_16BIT, 0x36D4, 0x296C},
	{E_REGISTER_CMD_16BIT, 0x36D6, 0xAE26},
	{E_REGISTER_CMD_16BIT, 0x36D8, 0xAE28},
	{E_REGISTER_CMD_16BIT, 0x36DA, 0xF58E},
	{E_REGISTER_CMD_16BIT, 0x36DC, 0x27A9},
	{E_REGISTER_CMD_16BIT, 0x36DE, 0x278C},
	{E_REGISTER_CMD_16BIT, 0x36E0, 0xABE6},
	{E_REGISTER_CMD_16BIT, 0x36E2, 0xAC28},
	{E_REGISTER_CMD_16BIT, 0x36E4, 0xF56E},
	{E_REGISTER_CMD_16BIT, 0x36E6, 0x27A9},
	{E_REGISTER_CMD_16BIT, 0x36E8, 0x278C},
	{E_REGISTER_CMD_16BIT, 0x36EA, 0xAC26},
	{E_REGISTER_CMD_16BIT, 0x36EC, 0xAC28},
	{E_REGISTER_CMD_16BIT, 0x36EE, 0x5110},
	{E_REGISTER_CMD_16BIT, 0x36F0, 0x8ECB},
	{E_REGISTER_CMD_16BIT, 0x36F2, 0x8EAE},
	{E_REGISTER_CMD_16BIT, 0x36F4, 0x1268},
	{E_REGISTER_CMD_16BIT, 0x36F6, 0x12AA},
	{E_REGISTER_CMD_16BIT, 0x36F8, 0x5110},
	{E_REGISTER_CMD_16BIT, 0x36FA, 0x8ECB},
	{E_REGISTER_CMD_16BIT, 0x36FC, 0x8EAE},
	{E_REGISTER_CMD_16BIT, 0x36FE, 0x12A8},
	{E_REGISTER_CMD_16BIT, 0x3700, 0x12AA},
	{E_REGISTER_CMD_16BIT, 0x3702, 0x5110},
	{E_REGISTER_CMD_16BIT, 0x3704, 0x8ECB},
	{E_REGISTER_CMD_16BIT, 0x3706, 0x8EAE},
	{E_REGISTER_CMD_16BIT, 0x3708, 0x1268},
	{E_REGISTER_CMD_16BIT, 0x370A, 0x12AA},
	{E_REGISTER_CMD_16BIT, 0x370C, 0x5110},
	{E_REGISTER_CMD_16BIT, 0x370E, 0x8ECB},
	{E_REGISTER_CMD_16BIT, 0x3710, 0x8EAE},
	{E_REGISTER_CMD_16BIT, 0x3712, 0x12A8},
	{E_REGISTER_CMD_16BIT, 0x3714, 0x12AA},
	{E_REGISTER_CMD_16BIT, 0x3644, 0x0180},
	{E_REGISTER_CMD_16BIT, 0x3642, 0x00A4},
	{E_REGISTER_CMD_16BIT, 0x3210, 0x09B8},
	{E_REGISTER_CMD_16BIT, 0x3210, 0x09B8},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2306},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0315},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2308},
	{E_REGISTER_CMD_16BIT, 0x0990, 0xFDDC},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x230A},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x003A},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x230C},
	{E_REGISTER_CMD_16BIT, 0x0990, 0xFF58},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x230E},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x02B7},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2310},
	{E_REGISTER_CMD_16BIT, 0x0990, 0xFF31},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2312},
	{E_REGISTER_CMD_16BIT, 0x0990, 0xFF4C},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2314},
	{E_REGISTER_CMD_16BIT, 0x0990, 0xFE4C},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2316},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x039E},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2318},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x001C},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x231A},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0039},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x231C},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x007F},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x231E},
	{E_REGISTER_CMD_16BIT, 0x0990, 0xFF77},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2320},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x000A},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2322},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0020},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2324},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x001B},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2326},
	{E_REGISTER_CMD_16BIT, 0x0990, 0xFFC6},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2328},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0086},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x232A},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x00B5},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x232C},
	{E_REGISTER_CMD_16BIT, 0x0990, 0xFEC3},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x232E},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0001},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2330},
	{E_REGISTER_CMD_16BIT, 0x0990, 0xFFEF},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA348},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0008},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA349},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0002},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA34A},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0090},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA34B},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x00FF},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA34C},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0075},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA34D},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x00EF},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA351},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0000},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA352},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x007F},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA354},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0043},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA355},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0001},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA35D},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0078},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA35E},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0086},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA35F},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x007E},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA360},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0082},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2361},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0040},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA363},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x00D2},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA364},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x00F6},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA302},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0000},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA303},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x00EF},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xAB20},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0024},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA103},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0006},
	{E_REGISTER_WAIT     , 0x0000, 100   },//delay 100
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA103},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0005},
	{E_REGISTER_WAIT     ,  0x0000, 100  },//delay 100
	{E_REGISTER_CMD_16BIT, 0x098C, 0x222D},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0088},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA408},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0020},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA409},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0023},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA40A},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0027},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA40B},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x002A},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2411},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0088},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2413},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x00A4},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2415},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0088},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2417},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x00A4},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA404},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0010},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA40D},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0002},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA40E},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0003},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA103},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0006},
	{E_REGISTER_WAIT     , 0x0000, 100},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA103},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0005},
	{E_REGISTER_WAIT     , 0x0000, 100},
	
    //	[VGA]
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2739},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0000},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x273B},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x027F},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x273D},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0000},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x273F},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x01DF},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2703},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0280},
	{E_REGISTER_CMD_16BIT, 0x098C, 0x2705},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x01E0},
	{E_REGISTER_CMD_16BIT, 0x098C, 0xA103},
	{E_REGISTER_CMD_16BIT, 0x0990, 0x0005},
	{E_REGISTER_CMD_16BIT, 0x3400, 0x7A28},
	

};



/* avert with the effect can not open the front camera*/
static struct mt9v113_i2c_reg_conf mt9v113_effect_off_reg_config[] =
{
    {E_REGISTER_CMD_16BIT, 0x098E, 0x8400},	// LOGICAL_ADDRESS_ACCESS [SEQ_CMD]
   // {E_REGISTER_CMD_8BIT,  0x8400,   0x01},// SEQ_CMD
    {E_REGISTER_CMD_8BIT,  0xA010,   0x00},// CAM_SELECT_FX
    {E_REGISTER_CMD_8BIT,  0x8400,   0x02},// SEQ_CMD
};

static struct mt9v113_i2c_reg_conf mt9v113_effect_mono_reg_config[] =
{
    {E_REGISTER_CMD_16BIT, 0x098E, 0x8400},	// LOGICAL_ADDRESS_ACCESS [SEQ_CMD]
   // {E_REGISTER_CMD_8BIT,  0x8400,   0x01},// SEQ_CMD
    {E_REGISTER_CMD_8BIT,  0xA010,   0x01},// CAM_SELECT_FX
    {E_REGISTER_CMD_8BIT,  0x8400,   0x02},// SEQ_CMD
};

static struct mt9v113_i2c_reg_conf mt9v113_effect_negative_reg_config[] =
{
    {E_REGISTER_CMD_16BIT, 0x098E, 0x8400},	// LOGICAL_ADDRESS_ACCESS [SEQ_CMD]
   // {E_REGISTER_CMD_8BIT,  0x8400,   0x01},// SEQ_CMD
    {E_REGISTER_CMD_8BIT,  0xA010,   0x03},// CAM_SELECT_FX
    {E_REGISTER_CMD_8BIT,  0x8400,   0x02},// SEQ_CMD
};

static struct mt9v113_i2c_reg_conf mt9v113_effect_solarize_reg_config[] =
{
    {E_REGISTER_CMD_16BIT, 0x098E, 0x8400}, 	// LOGICAL_ADDRESS_ACCESS [SEQ_CMD]
    //{E_REGISTER_CMD_8BIT,  0x8400,   0x01}, 	// SEQ_CMD
    {E_REGISTER_CMD_8BIT,  0xA010,   0x04}, 	// CAM_SELECT_FX
    {E_REGISTER_CMD_8BIT,  0x8400,   0x02}, 	// SEQ_CMD
};

static struct mt9v113_i2c_reg_conf mt9v113_effect_sepia_reg_config[] =
{
    {E_REGISTER_CMD_16BIT, 0x098E, 0x8400}, 	// LOGICAL_ADDRESS_ACCESS [SEQ_CMD]
   // {E_REGISTER_CMD_8BIT,  0x8400,   0x01},// SEQ_CMD
    {E_REGISTER_CMD_8BIT,  0xA010,   0x02},// CAM_SELECT_FX
    {E_REGISTER_CMD_8BIT,  0xA012,   0x1E},// CAM_SEPIA_CR
    {E_REGISTER_CMD_8BIT,  0xA013,   0xD8},// CAM_SEPIA_CB
    {E_REGISTER_CMD_8BIT,  0x8400,   0x02},// SEQ_CMD
};

static struct mt9v113_i2c_reg_conf mt9v113_effect_aqua_reg_config[] =
{
 
    {E_REGISTER_CMD_16BIT, 0x098E, 0x8400},   // LOGICAL_ADDRESS_ACCESS [SEQ_CMD]
    // {E_REGISTER_CMD_8BIT,  0x8400,   0x01},   // SEQ_CMD
    {E_REGISTER_CMD_8BIT,  0xA010,   0x02},   // CAM_SELECT_FX
    {E_REGISTER_CMD_8BIT,  0xA012,   0xCA},   // CAM_SEPIA_CR
    {E_REGISTER_CMD_8BIT,  0xA013,   0x28},   // CAM_SEPIA_CB
    {E_REGISTER_CMD_8BIT,  0x8400,   0x02},   // SEQ_CMD


};

/*add the wb setting*/
static struct mt9v113_i2c_reg_conf mt9v113_wb_auto_reg_config[] =
{
    {E_REGISTER_CMD_8BIT,  0x9401, 0x0D}, // AWB_MODE

    
    
};  

static struct mt9v113_i2c_reg_conf mt9v113_wb_incandescent_reg_config[] =
{
    {E_REGISTER_CMD_8BIT,  0x9401, 0x0C}, // AWB_MODE
    {E_REGISTER_CMD_8BIT,  0x9436, 0x5E},     // AWB_R_RATIO_PRE_AWB
    {E_REGISTER_CMD_8BIT,  0x9437, 0x33},     // AWB_B_RATIO_PRE_AWB

};

static struct mt9v113_i2c_reg_conf mt9v113_wb_fluorescent_reg_config[] =
{
    {E_REGISTER_CMD_8BIT,  0x9401, 0x0C}, // AWB_MODE
    {E_REGISTER_CMD_8BIT,  0x9436, 0x4D},     // AWB_R_RATIO_PRE_AWB
    {E_REGISTER_CMD_8BIT,  0x9437, 0x3C},     // AWB_B_RATIO_PRE_AWB

};

static struct mt9v113_i2c_reg_conf mt9v113_wb_daylight_reg_config[] =
{

    {E_REGISTER_CMD_8BIT,  0x9401, 0x0C}, // AWB_MODE
    {E_REGISTER_CMD_8BIT,  0x9436, 0x42},     // AWB_R_RATIO_PRE_AWB
    {E_REGISTER_CMD_8BIT,  0x9437, 0x56}, // AWB_B_RATIO_PRE_AWB

};

static struct mt9v113_i2c_reg_conf mt9v113_wb_cloudy_reg_config[] =
{

    {E_REGISTER_CMD_8BIT,  0x9401, 0x0C}, // AWB_MODE
    {E_REGISTER_CMD_8BIT,  0x9436, 0x4B},     // AWB_R_RATIO_PRE_AWB
    {E_REGISTER_CMD_8BIT,  0x9437, 0x49}, // AWB_B_RATIO_PRE_AWB

};


static struct  mt9v113_work_t *mt9v113sensorw = NULL;
static struct  i2c_client *mt9v113_client  = NULL;
static struct mt9v113_ctrl_t *mt9v113_ctrl = NULL;

#define MODEL_SUNNY 0
#define MODEL_BYD 1
unsigned short model_id = 0;

static DECLARE_WAIT_QUEUE_HEAD(mt9v113_wait_queue);
DEFINE_MUTEX(mt9v113_sem);

static int mt9v113_i2c_rxdata
(   unsigned short saddr,
    unsigned char *rxdata,
    int            length)
{
    struct i2c_msg msgs[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = 2,
            .buf = rxdata,
        },
        {
            .addr  = saddr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxdata,
        },
    };

    if (i2c_transfer(mt9v113_client->adapter, msgs, 2) < 0)
    {
        CDBG("mt9v113_i2c_rxdata failed!\n");
        return -EIO;
    }

    return 0;
}

static int32_t mt9v113_i2c_read_word
(   unsigned short  saddr,
    unsigned short  raddr,
    unsigned short *rdata)
{
    int32_t rc = 0;
    unsigned char buf[4];

    if (!rdata)
    {
        return -EIO;
    }

    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0xFF00) >> 8;
    buf[1] = (raddr & 0x00FF);

    rc = mt9v113_i2c_rxdata(saddr, buf, 2);
    if (rc < 0)
    {
        return rc;
    }

    *rdata = buf[0] << 8 | buf[1];

    if (rc < 0)
    {
        CDBG("mt9v113_i2c_read_word failed!\n");
    }

    return rc;
}

static int32_t mt9v113_i2c_txdata
(   unsigned short saddr,
    unsigned char *txdata,
    int            length)
{
    struct i2c_msg msg[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = length,
            .buf = txdata,
        },
    };

    if (i2c_transfer(mt9v113_client->adapter, msg, 1) < 0)
    {
        CDBG("mt9v113_i2c_txdata faild\n");
        return -EIO;
    }

    return 0;
}

static int32_t mt9v113_i2c_write_word
(   unsigned short saddr,
    unsigned short waddr,
    unsigned short wdata)
{
    int32_t rc = -EFAULT;
    unsigned char buf[4];
    int32_t i = 0;

    memset(buf, 0, sizeof(buf));
    buf[0] = (waddr & 0xFF00) >> 8;
    buf[1] = (waddr & 0x00FF);
    buf[2] = (wdata & 0xFF00) >> 8;
    buf[3] = (wdata & 0x00FF);

    /*write 3 times, if error, return -EIO*/
    for (i = 0; i < 3; i++)
    {
        rc = mt9v113_i2c_txdata(saddr , buf, 4);
        if (0 <= rc)
        {
            return 0;
        }
    }

    if (3 == i)
    {
        CDBG("mt9v113_i2c_write_word failed, addr = 0x%x, val = 0x%x!\n", waddr, wdata);
        return -EIO;
    }

    return 0;
}

static int mt9v113_i2c_write_byte
(   unsigned short saddr,
    unsigned short waddr,
    unsigned char  cdata)
{
    int32_t rc = -EFAULT;
    unsigned char buf[3];
    int32_t i = 0;

    memset(buf, 0, sizeof(buf));
    buf[0] = (waddr & 0xFF00) >> 8;
    buf[1] = (waddr & 0x00FF);
    buf[2] = (cdata & 0x00FF);

    /*write 3 times, if error, return -EIO*/
    for (i = 0; i < 3; i++)
    {
        rc = mt9v113_i2c_txdata(saddr, buf, 3);
        if (0 <= rc)
        {
            return 0;
        }
    }

    if (3 == i)
    {
        CDBG("mt9v113_i2c_write_byte failed, addr = 0x%x, val = 0x%x!\n", waddr, cdata);
        return -EIO;
    }

    return 0;
}

int32_t mt9v113_i2c_write_table(struct mt9v113_i2c_reg_conf *reg_conf_tbl, int num_of_items_in_table)
{
    int i;
    int32_t rc = -EFAULT;

    CDBG("mt9v113_i2c_write_table.\n");

    for (i = 0; i < num_of_items_in_table; i++)
    {
        if (E_REGISTER_CMD_8BIT == reg_conf_tbl->type)
        {
            rc = mt9v113_i2c_write_byte(mt9v113_client->addr,
                                        reg_conf_tbl->reg, reg_conf_tbl->value);
            if (rc < 0)
            {
                CDBG("mt9v113_i2c_write_table,reg[%d] write failed.\n", i);
                break;
            }
        }
        else if (E_REGISTER_CMD_16BIT == reg_conf_tbl->type)
        {
            rc = mt9v113_i2c_write_word(mt9v113_client->addr,
                                        reg_conf_tbl->reg, reg_conf_tbl->value);
            if (rc < 0)
            {
                CDBG("mt9v113_i2c_write_table,reg[%d] write failed.\n", i);
                break;
            }
        }
        else if (E_REGISTER_WAIT == reg_conf_tbl->type)
        {
            mdelay(reg_conf_tbl->value);
        }
        else
        {
            CDBG("mt9v113_i2c_write_table,reg[%d].type is wrong.\n", i);
        }

        reg_conf_tbl++;
    }

	 CDBG("mt9v113_i2c_write_table ....OK\n");

    return rc;
}

int32_t mt9v113_set_default_focus(uint8_t af_step)
{
    CDBG("mt9v113_set_default_focus,af_step:%d\n", af_step);
    return 0;
}


/*add the wb setting*/

int32_t mt9v113_set_fps(struct fps_cfg *fps)
{
    /* input is new fps in Q8 format */
    int32_t rc = 0;

    CDBG("mt9v113_set_fps\n");
    return rc;
}

int32_t mt9v113_write_exp_gain(uint16_t gain, uint32_t line)
{
    CDBG("mt9v113_write_exp_gain\n");
    return 0;
}

int32_t mt9v113_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
    int32_t rc = 0;

    CDBG("mt9v113_set_pict_exp_gain\n");


    /* camera_timed_wait(snapshot_wait*exposure_ratio); */
    return rc;
}

int32_t mt9v113_setting(enum mt9v113_reg_update_t rupdate,
                        enum mt9v113_setting_t    rt)
{
    struct msm_camera_csi_params mt9v113_csi_params;
    int32_t rc = 0;

    switch (rupdate)
    {
    case UPDATE_PERIODIC:
		/* mt9v113 preview and snapshot do not need write register*/
        if (rt == RES_PREVIEW)
        {
          
            if (!CSI_CONFIG)
            {
                CDBG("mt9v113: init CSI  config!\n");
                mt9v113_csi_params.data_format = CSI_8BIT;
                mt9v113_csi_params.lane_cnt = 1;
                mt9v113_csi_params.lane_assign = 0xe4;
                mt9v113_csi_params.dpcm_scheme = 0;
                mt9v113_csi_params.settle_cnt = 0x18;
                rc = msm_camio_csi_config(&mt9v113_csi_params);
                CSI_CONFIG = 1;
            }
        }
        else
        {
        }

        break;

    case REG_INIT:

        CSI_CONFIG = 0;
		
        rc = mt9v113_i2c_write_table(mt9v113_init_reg_config,
                                     sizeof(mt9v113_init_reg_config) / sizeof(mt9v113_init_reg_config[0]));
        if (rc < 0)
        {
            CDBG("mt9v113_setting,reg init failed.\n");
        }
	    
        return rc;
        break;

    default:
        rc = -EFAULT;
        break;
    } /* end switch (rupdate) */

    return rc;
}

int32_t mt9v113_video_config(int mode, int res)
{
    int32_t rc;

    switch (res)
    {
    case QTR_SIZE:
        rc = mt9v113_setting(UPDATE_PERIODIC, RES_PREVIEW);
        if (rc < 0)
        {
            CDBG("mt9v113_video_config:sensor configuration fail!\n");
            return rc;
        }

        CDBG("sensor configuration done!\n");
        break;

    case FULL_SIZE:
        rc = mt9v113_setting(UPDATE_PERIODIC, RES_CAPTURE);
        if (rc < 0)
        {
            CDBG("mt9v113_video_config:sensor configuration fail!\n");
            return rc;
        }

        break;

    default:
        return 0;
    } /* end switch */

    mt9v113_ctrl->prev_res   = res;
    mt9v113_ctrl->curr_res   = res;
    mt9v113_ctrl->sensormode = mode;

    return rc;
}

int32_t mt9v113_snapshot_config(int mode)
{
    int32_t rc = 0;
	
	CDBG("mt9v113_snapshot_config in\n");
    rc = mt9v113_setting(UPDATE_PERIODIC, RES_CAPTURE); 
    mdelay(50);
 
    if (rc < 0)
    {
        return rc;
    }

    mt9v113_ctrl->curr_res = mt9v113_ctrl->pict_res;

    mt9v113_ctrl->sensormode = mode;

    return rc;
}

int32_t mt9v113_power_down(void)
{
    int32_t rc = 0;

    mdelay(5);
    return rc;
}

int32_t mt9v113_move_focus(int direction, int32_t num_steps)
{
 
    return 0;
}

static int mt9v113_sensor_init_done(const struct msm_camera_sensor_info *data)
{
   
    gpio_direction_output(data->sensor_reset, 0);
    gpio_free(data->sensor_reset);

    gpio_direction_output(data->sensor_pwd, 1);
    gpio_free(data->sensor_pwd);

    /*disable the power*/
    if (data->vreg_disable_func)
    {
        data->vreg_disable_func(0);
    }

    return 0;
}

static int mt9v113_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
    int rc;
    unsigned short chipid;

    CDBG("sensor_pwd is =%d", data->sensor_pwd);
    /* pull down power down */
    rc = gpio_request(data->sensor_pwd, "mt9v113");
    if (!rc || (rc == -EBUSY))
    {
        gpio_direction_output(data->sensor_pwd, 1);
    }
    else
    {
        goto init_probe_fail;
    }

    /* Set the sensor reset */
    rc = gpio_request(data->sensor_reset, "mt9v113");
    if (!rc)
    {
        rc = gpio_direction_output(data->sensor_reset, 0);
    }
    else
    {
        goto init_probe_fail;
    }

    /*enable the power*/
    mdelay(10);
    if (data->vreg_enable_func)
    {
       data->vreg_enable_func(1);
    }  
    mdelay(20);

  
    rc = gpio_direction_output(data->sensor_pwd, 0);
    if (rc < 0)
    {
        goto init_probe_fail;
    }

    mdelay(50);

    /*hardware reset*/
    rc = gpio_direction_output(data->sensor_reset, 1);
    if (rc < 0)
    {
        goto init_probe_fail;
    }

    mdelay(100);
  
    
    /* Read sensor Model ID: */
    rc = mt9v113_i2c_read_word(mt9v113_client->addr,
                               MT9V113_REG_MODEL_ID, &chipid);
    if (rc < 0)
    {
        CDBG("mt9v113_i2c_read_word Model_ID failed!! rc=%d", rc);
        goto init_probe_fail;
    }

    CDBG("mt9v113 model_id = 0x%x\n", chipid);

    /* Compare sensor ID to MT9V113_CHIP_ID: */
    if (chipid != MT9V113_CHIP_ID)
    {
        CDBG("MT9V113_CHIP_ID  error!!");
        rc = -ENODEV;
        goto init_probe_fail;
    }
    CDBG("sensor name is %s.", data->sensor_name);

    goto init_probe_done;

init_probe_fail:
    mt9v113_sensor_init_done(data);
init_probe_done:
    return rc;
}

int mt9v113_sensor_open_init(const struct msm_camera_sensor_info *data)
{
    int32_t rc;
    mt9v113_ctrl = kzalloc(sizeof(struct mt9v113_ctrl_t), GFP_KERNEL);
    if (!mt9v113_ctrl)
    {
        CDBG("mt9v113_sensor_open_init failed!\n");
        rc = -ENOMEM;
        goto init_done;
    }

    CDBG("mt9v113_sensor_open_init!\n");
    mt9v113_ctrl->fps_divider = 1 * 0x00000400;
    mt9v113_ctrl->pict_fps_divider = 1 * 0x00000400;
    mt9v113_ctrl->set_test = TEST_OFF;
    mt9v113_ctrl->prev_res = QTR_SIZE;
    mt9v113_ctrl->pict_res = FULL_SIZE;

    if (data)
    {
        mt9v113_ctrl->sensordata = data;
    }

    /* enable mclk first */
    msm_camio_clk_rate_set(MT9V113_DEFAULT_CLOCK_RATE);
    mdelay(20);

    rc = mt9v113_probe_init_sensor(data);
    if (rc < 0)
    {
        CDBG("mt9v113 init failed!!!!!\n");
        goto init_fail;
    }
    else
    {
        rc = mt9v113_setting(REG_INIT, RES_PREVIEW);
        CDBG("mt9v113 init succeed!!!!! rc = %d \n", rc);
        goto init_done;
    }

init_fail:
    kfree(mt9v113_ctrl);
init_done:
    return rc;
}

int mt9v113_init_client(struct i2c_client *client)
{
    /* Initialize the MSM_CAMI2C Chip */
    init_waitqueue_head(&mt9v113_wait_queue);
    return 0;
}

int32_t mt9v113_set_sensor_mode(int mode, int res)
{
    int32_t rc = 0;

    switch (mode)
    {
    case SENSOR_PREVIEW_MODE:
        CDBG("SENSOR_PREVIEW_MODE,res=%d\n", res);
        rc = mt9v113_video_config(mode, res);
        break;

    case SENSOR_SNAPSHOT_MODE:
    case SENSOR_RAW_SNAPSHOT_MODE:
        CDBG("SENSOR_SNAPSHOT_MODE\n");
        rc = mt9v113_snapshot_config(mode);
        break;

    default:
        rc = -EINVAL;
        break;
    }

    return rc;
}

/*add the effect setting*/
int32_t mt9v113_set_effect(int32_t effect)
{
	struct mt9v113_i2c_reg_conf  *reg_conf_tbl = NULL;
    int num_of_items_in_table = 0;
	long rc = 0;

	switch (effect) {
	case CAMERA_EFFECT_OFF:
        reg_conf_tbl = mt9v113_effect_off_reg_config;
        num_of_items_in_table = sizeof(mt9v113_effect_off_reg_config) / sizeof(mt9v113_effect_off_reg_config[0]);
        break;

	case CAMERA_EFFECT_MONO:
        reg_conf_tbl = mt9v113_effect_mono_reg_config;
        num_of_items_in_table = sizeof(mt9v113_effect_mono_reg_config) / sizeof(mt9v113_effect_mono_reg_config[0]);
		break;

	case CAMERA_EFFECT_NEGATIVE:
        reg_conf_tbl = mt9v113_effect_negative_reg_config;
        num_of_items_in_table = sizeof(mt9v113_effect_negative_reg_config) / sizeof(mt9v113_effect_negative_reg_config[0]);
		break;

	case CAMERA_EFFECT_SOLARIZE:
        reg_conf_tbl = mt9v113_effect_solarize_reg_config;
        num_of_items_in_table = sizeof(mt9v113_effect_solarize_reg_config) / sizeof(mt9v113_effect_solarize_reg_config[0]);
		break;

	case CAMERA_EFFECT_SEPIA:
        reg_conf_tbl = mt9v113_effect_sepia_reg_config;
        num_of_items_in_table = sizeof(mt9v113_effect_sepia_reg_config) / sizeof(mt9v113_effect_sepia_reg_config[0]);
		break;
        
	case CAMERA_EFFECT_AQUA:
        reg_conf_tbl = mt9v113_effect_aqua_reg_config;
        num_of_items_in_table = sizeof(mt9v113_effect_aqua_reg_config) / sizeof(mt9v113_effect_aqua_reg_config[0]);
		break;
	default: 
		return 0;
	}

		rc = mt9v113_i2c_write_table(reg_conf_tbl, num_of_items_in_table);
    return rc;
}

int32_t mt9v113_set_wb(int32_t wb)
{
	struct mt9v113_i2c_reg_conf *reg_conf_tbl = NULL;
    int num_of_items_in_table = 0;
	long rc = 0;
    CDBG("mt9v113_set_wb: wb = %d\n",wb);
	switch (wb) {
	case CAMERA_WB_AUTO:
        reg_conf_tbl = mt9v113_wb_auto_reg_config;
        num_of_items_in_table = sizeof(mt9v113_wb_auto_reg_config) / sizeof(mt9v113_wb_auto_reg_config[0]);
        break;

	case CAMERA_WB_INCANDESCENT:
        reg_conf_tbl = mt9v113_wb_incandescent_reg_config;
        num_of_items_in_table = sizeof(mt9v113_wb_incandescent_reg_config) / sizeof(mt9v113_wb_incandescent_reg_config[0]);
		break;

	case CAMERA_WB_CUSTOM:       
	case CAMERA_WB_FLUORESCENT:
        reg_conf_tbl = mt9v113_wb_fluorescent_reg_config;
        num_of_items_in_table = sizeof(mt9v113_wb_fluorescent_reg_config) / sizeof(mt9v113_wb_fluorescent_reg_config[0]);
		break;

	case CAMERA_WB_DAYLIGHT:
        reg_conf_tbl = mt9v113_wb_daylight_reg_config;
        num_of_items_in_table = sizeof(mt9v113_wb_daylight_reg_config) / sizeof(mt9v113_wb_daylight_reg_config[0]);
		break;
        
	case CAMERA_WB_CLOUDY_DAYLIGHT:
        reg_conf_tbl = mt9v113_wb_cloudy_reg_config;
        num_of_items_in_table = sizeof(mt9v113_wb_cloudy_reg_config) / sizeof(mt9v113_wb_cloudy_reg_config[0]);
		break;
              
	default: 
		return 0;
	}

    rc = mt9v113_i2c_write_table(reg_conf_tbl, num_of_items_in_table);
    return rc;

}



int mt9v113_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cdata;
    long rc = 0;
    if (copy_from_user(&cdata,
                       (void *)argp,
                       sizeof(struct sensor_cfg_data)))
    {
        return -EFAULT;
    }

    mutex_lock(&mt9v113_sem);

    CDBG("mt9v113_sensor_config: cfgtype = %d\n", cdata.cfgtype);
    switch (cdata.cfgtype)
    {
    case CFG_GET_PICT_FPS:
        break;

    case CFG_GET_PREV_L_PF:
        break;

    case CFG_GET_PREV_P_PL:
        break;

    case CFG_GET_PICT_L_PF:
        break;

    case CFG_GET_PICT_P_PL:
        break;

    case CFG_GET_PICT_MAX_EXP_LC:
        break;

    case CFG_SET_FPS:
    case CFG_SET_PICT_FPS:
        rc = mt9v113_set_fps(&(cdata.cfg.fps));
        break;

    case CFG_SET_EXP_GAIN:
        rc =
            mt9v113_write_exp_gain(
            cdata.cfg.exp_gain.gain,
            cdata.cfg.exp_gain.line);
        break;

    case CFG_SET_PICT_EXP_GAIN:
        rc =
            mt9v113_set_pict_exp_gain(
            cdata.cfg.exp_gain.gain,
            cdata.cfg.exp_gain.line);
        break;

    case CFG_SET_MODE:
        rc = mt9v113_set_sensor_mode(cdata.mode,
                                     cdata.rs);
        break;

    case CFG_PWR_DOWN:
        rc = mt9v113_power_down();
        break;

    case CFG_MOVE_FOCUS:
        rc =
            mt9v113_move_focus(
            cdata.cfg.focus.dir,
            cdata.cfg.focus.steps);
        break;

    case CFG_SET_DEFAULT_FOCUS:
        rc =
            mt9v113_set_default_focus(
            cdata.cfg.focus.steps);
        break;

    case CFG_SET_EFFECT:
		if(0)
        rc = mt9v113_set_effect(
            cdata.cfg.effect);
        break;

    case CFG_SET_WB:
        rc = mt9v113_set_wb(
            cdata.cfg.effect);
        break;
    default:
        rc = -EFAULT;
        break;
    }

   mutex_unlock(&mt9v113_sem);
    
    return rc;
}

int mt9v113_sensor_release(void)
{
    int rc = -EBADF;

   mutex_lock(&mt9v113_sem);

    mt9v113_power_down();

    mt9v113_sensor_init_done(mt9v113_ctrl->sensordata);

    msleep(150);
    kfree(mt9v113_ctrl);

    mutex_unlock(&mt9v113_sem);
 
    return rc;
}

static int mt9v113_i2c_probe(struct i2c_client *         client,
                             const struct i2c_device_id *id)
{
    int rc = 0;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        rc = -ENOTSUPP;
        goto probe_failure;
    }

    mt9v113sensorw =
        kzalloc(sizeof(struct mt9v113_work_t), GFP_KERNEL);

    if (!mt9v113sensorw)
    {
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, mt9v113sensorw);
    mt9v113_init_client(client);
    mt9v113_client = client;

    mdelay(50);

    CDBG("i2c probe ok\n");
    return 0;

probe_failure:
    kfree(mt9v113sensorw);
    mt9v113sensorw = NULL;
    pr_err("i2c probe failure %d\n", rc);
    return rc;
}

static const struct i2c_device_id mt9v113_i2c_id[] =
{
    { "mt9v113", 0},
    { }
};

static struct i2c_driver mt9v113_i2c_driver =
{
    .id_table = mt9v113_i2c_id,
    .probe    = mt9v113_i2c_probe,
    .remove   = __exit_p(mt9v113_i2c_remove),
    .driver   = {
        .name = "mt9v113",
    },
};

static int mt9v113_sensor_probe(const struct msm_camera_sensor_info *info,
                                struct msm_sensor_ctrl *s)
{
    /* We expect this driver to match with the i2c device registered
     * in the board file immediately. */
    int rc = i2c_add_driver(&mt9v113_i2c_driver);

    if ((rc < 0) || (mt9v113_client == NULL))
    {
        rc = -ENOTSUPP;
		
		CDBG("i2c_add_driver error \n");
        goto probe_done;
    }

    /* enable mclk first */
    msm_camio_clk_rate_set(MT9V113_DEFAULT_CLOCK_RATE);
    mdelay(20);

    rc = mt9v113_probe_init_sensor(info);
    if (rc < 0)
    {
        CDBG("mt9v113 probe failed!!!!\n");
        i2c_del_driver(&mt9v113_i2c_driver);
        goto probe_done;
    }
    else
    {
        rc = mt9v113_i2c_write_table(mt9v113_init_reg_config,
                                     sizeof(mt9v113_init_reg_config) / sizeof(mt9v113_init_reg_config[0]));
        mdelay(50);
        /*
        * after sensor is initialized ,we distinguish the model by CAM_ID gpio:
        * sunny model's GPIO[1] connects DGND, byd model's GPIO[1] connects to DOVDD
        * bit[9] of register 0x1070 is the value of GPIO[1] signal, so we read
        * the value of 0x1070 and move right 9 bits to get the GPIO[1] value
        */
        rc = mt9v113_i2c_write_word(mt9v113_client->addr,0x098C, 0x1070);
        rc = mt9v113_i2c_read_word(mt9v113_client->addr,0x0990, &model_id);  
        model_id = (model_id & 0x0200) >> 9;
        CDBG("cam_id gpio[1]= %d\n", model_id);

        if(MODEL_SUNNY == model_id)
        {
            strncpy((char *)info->sensor_name, "23060075FF-MT-S", strlen("23060075FF-MT-S"));
        }
        else
        {
            strncpy((char *)info->sensor_name, "23060075FF-MT-B", strlen("23060075FF-MT-B"));
        }
        CDBG("mt9v113 probe succeed!!!!\n");
    }

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    CDBG("CONFIG_HUAWEI_HW_DEV_DCT\n");
    set_hw_dev_flag(DEV_I2C_CAMERA_SLAVE);
#endif

    s->s_init = mt9v113_sensor_open_init;
    s->s_release = mt9v113_sensor_release;
    s->s_config = mt9v113_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;

    /*set the s_mount_angle value of sensor*/
    s->s_mount_angle = info->sensor_platform_info->mount_angle;
	
    mt9v113_sensor_init_done(info);
	
    /* For go to sleep mode, follow the datasheet */
    msleep(150);
    set_camera_support(true);
probe_done:
	
    return rc;
}

static int __mt9v113_probe(struct platform_device *pdev)
{
  
    return msm_camera_drv_start(pdev, mt9v113_sensor_probe);
}

static struct platform_driver msm_camera_driver =
{
    .probe     = __mt9v113_probe,
    .driver    = {
        .name  = "msm_camera_mt9v113",
        .owner = THIS_MODULE,
    },
};

static int __init mt9v113_init(void)
	
{  
    return platform_driver_register(&msm_camera_driver);
}

module_init(mt9v113_init);

