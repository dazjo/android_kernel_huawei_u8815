/* drivers\i2c\chips\tpa2028d1.c
 *
 * Copyright (C) 2009 HUAWEI Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/audio_amplifier.h>
#include <linux/delay.h>
#include <asm/mach-types.h>

#include "linux/hardware_self_adapt.h"

//#define TPA_DEBUG  0
#ifdef TPA_DEBUG
#define TPA_DEBUG_TPA(fmt, args...) printk(KERN_INFO fmt, ##args)
#else
#define TPA_DEBUG_TPA(fmt, args...)
#endif
#define TPA2028D1_I2C_NAME "tpa2028d1"
#define CLASSD_EN 116
static struct i2c_client *g_client;

static int tpa2028d1_i2c_write(char *txData, int length)
{

	struct i2c_msg msg[] = {
		{
		 .addr = g_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	if (i2c_transfer(g_client->adapter, msg, 1) < 0) 
    {
		TPA_DEBUG_TPA("tpa2028d1_i2c_write: transfer error\n");
		return -EIO;
	} 
    else
    {
        return 0;
    }
}
static int tpa2028d1_i2c_read(char * reg, char *rxData)
{
    
	struct i2c_msg msgs[] = {
		{
		 .addr = g_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = reg,
		 },
		{
		 .addr = g_client->addr,
		 .flags = I2C_M_RD,
		 .len = 1,
		 .buf = rxData,
		 },
	};

	if (i2c_transfer(g_client->adapter, msgs, 2) < 0) 
    {
		TPA_DEBUG_TPA("tpa2028d1_i2c_read: transfer error\n");
		return -EIO;
	} 
    else
	{
        TPA_DEBUG_TPA("reg(0x%x)'s value:0x%x\n",*reg, *rxData);
		return 0;
    }
}
void tpa2028d1_amplifier_on(void)
{
    u8 r_data, w_data[2];
    char reg = 0x01;
    TPA_DEBUG_TPA("tpa2028d1_amplifier_on\n");
     /*enable amplifier*/
    tpa2028d1_i2c_read(&reg, &r_data);

    r_data &= ~(1 << 5);        //disable SWS bit
    r_data |= (1 << 6);         //enable EN bit

    w_data[0] = 0x01;           //reg address:0x01
    w_data[1] = r_data;
    tpa2028d1_i2c_write(w_data, 2);
}

void tpa2028d1_amplifier_off(void)
{
    u8 r_data, w_data[2];
    char reg = 0x01;
    TPA_DEBUG_TPA("tpa2028d1_amplifier_off\n");
     /*disable amplifier*/
    tpa2028d1_i2c_read(&reg, &r_data);

    r_data &= ~(1 << 6);        //disable EN bit
    r_data |= (1 << 5);         //enable SWS bit

    w_data[0] = 0x01;           //reg address:0x01
    w_data[1] = r_data;
    tpa2028d1_i2c_write(w_data, 2);
}

/* initialization command */
static char init_data[7][2] = 
{
    {0x01, 0x83},
    {0x02, 0x05},
    {0x03, 0x02},
    {0x04, 0x02},
    {0x05, 0x00},
    {0x06, 0x5a},
    {0x07, 0x92}      
};

static int tpa2028d1_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
       
	char i = 8;
    int ret = 0;
	int gpio_config = 0;
    u8 * init_datap = &(init_data[0][0]);

    if (machine_is_msm7x27a_U8815()
	  &&HW_VER_SUB_VA == get_hw_sub_board_id())
    {
        TPA_DEBUG_TPA("tpa2028d1_probe\n");

        g_client = client;
        gpio_config = GPIO_CFG(CLASSD_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
        ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
        if (ret) {
            printk(KERN_ERR "tpa2028d1_probe: gpio_tlmm_config failed\n");
            ret = -EIO;
            return ret;
        }
        ret = gpio_request(CLASSD_EN, "tpa2028d1");
        if (ret) {
            printk(KERN_ERR "tpa2028d1_probe: gpio_request failed\n");
            goto probe_faild;
        }

        gpio_direction_output(CLASSD_EN,1);  /* enable spkr poweramp */

        mdelay(100);
        /*init tpa2028d1 reg*/
        for (i = 0; i < ARRAY_SIZE(init_data); i++)
        {
            ret = tpa2028d1_i2c_write(init_datap, 2);
            if(ret) {
                printk( KERN_ERR "tpa2028d1_i2c_write ERROR\n");
                goto probe_faild;
            }
            init_datap += 2;
        }

        tpa2028d1_amplifier_on();
        goto probe_success;
    }
    else
    {
        printk( KERN_ERR "tpa2028d1_probe ERROR\n");
		/* if the board is not U8815 Ver.A , return error directly */
		ret = -ENODEV;
        return ret;
    }
probe_success:
	TPA_DEBUG_TPA("tpa2028d1_probe completed\n");
	return 0;
	
probe_faild:
	gpio_free(CLASSD_EN);
    return ret;
}

static int tpa2028d1_remove(struct i2c_client *client)
{
    struct amplifier_platform_data *pdata = client->dev.platform_data;
    if(pdata)
    {
        pdata->amplifier_on = NULL;
        pdata->amplifier_off = NULL;
    }
	gpio_free(CLASSD_EN);
	return 0;
}


static const struct i2c_device_id tpa2028d1_id[] = {
	{TPA2028D1_I2C_NAME, 0},
	{ }
};

static struct i2c_driver tpa2028d1_driver = {
	.probe		= tpa2028d1_probe,
	.remove		= tpa2028d1_remove,
	.id_table	= tpa2028d1_id,
	.driver = {
	    .name	= TPA2028D1_I2C_NAME,
	},
};

static int __devinit tpa2028d1_init(void)
{
    TPA_DEBUG_TPA("add tpa2028d1 driver\n");
	return i2c_add_driver(&tpa2028d1_driver);
}

static void __exit tpa2028d1_exit(void)
{
	i2c_del_driver(&tpa2028d1_driver);
}

module_init(tpa2028d1_init);
module_exit(tpa2028d1_exit);

MODULE_DESCRIPTION("tpa2028d1 Driver");
MODULE_LICENSE("GPL");
