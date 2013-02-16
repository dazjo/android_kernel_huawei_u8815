/* drivers/input/misc/aps-12d.c
 *
 * Copyright (C) 2010 HUAWEI, Inc.
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
 
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include "aps-12d.h"
#include <linux/hardware_self_adapt.h>
#include <linux/wakelock.h>
#include <linux/light.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

//#define PROXIMITY_DEBUG
#undef PROXIMITY_DEBUG

#ifdef PROXIMITY_DEBUG
#define PROXIMITY_DEBUG(fmt, args...) printk(KERN_INFO fmt, ##args)
#else
#define PROXIMITY_DEBUG(fmt, args...)
#endif

#ifndef abs
#define abs(a)  ((0 < (a)) ? (a) : -(a))
#endif
static int aps_debug_mask;
module_param_named(aps_debug, aps_debug_mask, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define APS_DBG(x...) do {\
	if (aps_debug_mask) \
		printk(KERN_DEBUG x);\
	} while (0)

static struct workqueue_struct *aps_wq;
/* to compare the new_lsb value and the old_lsb value */
static u8 old_lsb = 1;
static u8 old_msb = 1;
struct aps_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct mutex  mlock;
	struct hrtimer timer;
	struct work_struct  work;	
	int (*power)(int on);
};

static struct aps_data  *this_aps_data;
/*set a flag to choose intersil or everlight,default intersil*/
static EVE_INTER_F intersil_flag = EVERLIGHT;

#ifdef CONFIG_MELFAS_UPDATE_TS_FIRMWARE
	struct aps_data  *TS_updateFW_aps_data;
	struct workqueue_struct *TS_updateFW_aps_wq;
#endif

extern struct input_dev *sensor_dev;

static int aps_12d_delay = APS_12D_TIMRER;     /*1s*/
static int aps_12d_timer_count = 0;
static char light_device_id[] = "EVERLIGHT-12D";
static int aps_first_read = 1;
static int light_device_minor = 0;

static int proximity_device_minor = 0;
static struct wake_lock proximity_wake_lock;

static atomic_t l_flag;
static atomic_t p_flag;
/*get value of proximity and light*/
static int proximity_data_value = 0;
static int light_data_value = 0;

#define LSENSOR_MAX_LEVEL 7
/* add U8861 proximity and lsensor */
static uint16_t lsensor_adc_U8661_table[LSENSOR_MAX_LEVEL] = 
{
	22, 40, 65, 110, 256, 640, 1024
};
static uint16_t lsensor_adc_U8661_Inter_table[LSENSOR_MAX_LEVEL] = 
{
	30, 50, 80, 120, 256, 640, 1024
};

#define 	TOTAL_RANGE_NUM 	2	/* aps-12d has 4 types of range,but we use two range */
#define 	MAX_ADC_OUTPUT  	4096	/* adc max value */
#define		RANGE_FIX		500	/* adc */
#define 	ADJUST_GATE		5	/* 1/ADJUST_GATE */

static unsigned int range_index = 0;
static unsigned int adjust_time = 0;
static int last_event = -1;

/*adjust the proximity limited value*/
static unsigned int low_threshold_value_U8661[TOTAL_RANGE_NUM]  = {300, 15};
static unsigned int high_threshold_value_U8661[TOTAL_RANGE_NUM] = {580, 18};
static unsigned int low_threshold_value_U8661_I[TOTAL_RANGE_NUM]  = {90, 50};
static unsigned int high_threshold_value_U8661_I[TOTAL_RANGE_NUM] = {200, 55};

/*changge the err threshold value for U8510*/
static unsigned int err_threshold_value[TOTAL_RANGE_NUM] = {4096,0};

static unsigned int range_reg_value[TOTAL_RANGE_NUM] = { APS_12D_RANGE_SEL_ALS_1000, \
						     APS_12D_RANGE_SEL_ALS_64000 };
static unsigned int up_range_value[TOTAL_RANGE_NUM] = {0};
static unsigned int down_range_value[TOTAL_RANGE_NUM] = {0};

/* modify iotcl interface */
static long
aps_12d_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg);
	   
static inline int aps_i2c_reg_read(struct aps_data *aps , int reg)
{
	int val = 0;

	mutex_lock(&aps->mlock);

	/* First write reg, then read reg data, resolve i2c error */
	val = i2c_smbus_write_byte(aps->client, reg);
	if (val < 0)
		printk(KERN_ERR "%s: failed to write reg[%d], err=%d\n", __FUNCTION__, reg, val);

	val = i2c_smbus_read_byte(aps->client);
	if (val < 0)
		printk(KERN_ERR "%s: failed to read reg[%d], err=%d\n", __FUNCTION__, reg, val);

	mutex_unlock(&aps->mlock);

	return val;
}
static inline int aps_i2c_reg_write(struct aps_data *aps, int reg, uint8_t val)
{
	int ret;

	mutex_lock(&aps->mlock);
	ret = i2c_smbus_write_byte_data(aps->client, reg, val);
	if(ret < 0) {
		printk(KERN_ERR "%s: failed to write %d to reg[%d], err=%d\n", __FUNCTION__, val, reg, ret);
	}
	mutex_unlock(&aps->mlock);

	return ret;
}

static int aps_12d_open(struct inode *inode, struct file *file)
{	
	PROXIMITY_DEBUG("aps_12d_open enter, timer_count=%d\n", aps_12d_timer_count);

	if( light_device_minor == iminor(inode) ){
		aps_first_read = 1;
		PROXIMITY_DEBUG("%s:light sensor open", __func__);
	}

	if( proximity_device_minor == iminor(inode) ){
		printk("%s:proximity_device_minor == iminor(inode)", __func__);
		wake_lock( &proximity_wake_lock);
		
		/* 0 is close, 1 is far */
		input_report_abs(this_aps_data->input_dev, ABS_DISTANCE, 1);			
		input_sync(this_aps_data->input_dev);
		PROXIMITY_DEBUG("%s:proximity = %d", __func__, 1);
	}
	if( 0 == aps_12d_timer_count )
		hrtimer_start(&this_aps_data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	aps_12d_timer_count++;
	
	return nonseekable_open(inode, file);
}

static int aps_12d_release(struct inode *inode, struct file *file)
{
	PROXIMITY_DEBUG("aps_12d_release enter, timer_count=%d\n ", aps_12d_timer_count);

	aps_12d_timer_count--;
	
	if( 0 == aps_12d_timer_count ) {
		hrtimer_cancel(&this_aps_data->timer);
		aps_12d_delay = APS_12D_TIMRER;
	}
	if( proximity_device_minor == iminor(inode) ){
		printk("%s: proximity_device_minor == iminor(inode)", __func__);
		wake_unlock( &proximity_wake_lock);
	}
	   
	return 0;
}

/* modify iotcl interface */
static long
aps_12d_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag;

	switch (cmd) 
	{
		case ECS_IOCTL_APP_SET_LFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
				break;
		case ECS_IOCTL_APP_SET_PFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
				break;
		case ECS_IOCTL_APP_SET_DELAY:
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
				break;
		
			default:
				break;
	}

	switch (cmd) 
	{
		case ECS_IOCTL_APP_SET_LFLAG:
			atomic_set(&l_flag, flag);
			break;

		case ECS_IOCTL_APP_GET_LFLAG:  /*get open acceleration sensor flag*/
			flag = atomic_read(&l_flag);
			break;

		case ECS_IOCTL_APP_SET_PFLAG:
			atomic_set(&p_flag, flag);
			if( flag )
			{
				/*
				 * this means the proximity sensor is open.
				 * so init the range_index to zero 
				 */
				range_index = 0;
			}
			break;

		case ECS_IOCTL_APP_GET_PFLAG:  /*get open acceleration sensor flag*/
			flag = atomic_read(&p_flag);
			break;

		case ECS_IOCTL_APP_SET_DELAY:
			if(flag)
				aps_12d_delay = flag;
			else
				aps_12d_delay = 20;   /*200ms*/
			break;

		case ECS_IOCTL_APP_GET_DELAY:
			flag = aps_12d_delay;
			break;
		/*get value of proximity and light*/
		case ECS_IOCTL_APP_GET_PDATA_VALVE:
			flag = proximity_data_value;
       		break;
        
		case ECS_IOCTL_APP_GET_LDATA_VALVE:
       		flag = light_data_value;
       		break;
		default:
			break;
	}

	switch (cmd) 
	{
		case ECS_IOCTL_APP_GET_LFLAG:
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
			break;

		case ECS_IOCTL_APP_GET_PFLAG:
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
			break;

		case ECS_IOCTL_APP_GET_DELAY:
			if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
			
			break;
		case ECS_IOCTL_APP_GET_APSID:
        {
            if (copy_to_user(argp, light_device_id, strlen(light_device_id)+1))
				return -EFAULT;
			break;
        }
		/*get value of proximity and light*/
		case ECS_IOCTL_APP_GET_PDATA_VALVE:
       		if (copy_to_user(argp, &flag, sizeof(flag)))
           		return -EFAULT;
       		 break;
        
		case ECS_IOCTL_APP_GET_LDATA_VALVE:
      		if (copy_to_user(argp, &flag, sizeof(flag)))
          		return -EFAULT;
       		 break;
		default:
			break;
	}
	return 0;
	
}

/* modify iotcl interface */
static struct file_operations aps_12d_fops = {
	.owner = THIS_MODULE,
	.open = aps_12d_open,
	.release = aps_12d_release,
	.unlocked_ioctl = aps_12d_ioctl,
};

static struct miscdevice light_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "light",
	.fops = &aps_12d_fops,
};

static struct miscdevice proximity_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "proximity",
	.fops = &aps_12d_fops,
};

static void aps_12d_work_func(struct work_struct *work)
{
	int flag = -1;
	// delete flag_old
	int ret;
	int reg_val_lsb;
	int reg_val_msb;
	int	sesc = aps_12d_delay/1000;
	int nsesc = (aps_12d_delay%1000)*1000000;
	uint16_t high_threshold = 0;
    uint16_t low_threshold = 0;
	int ir_count = 0;
	int ps_count = 0;
	uint16_t als_count = 0;
	uint8_t als_level = 0;
	/* del als_level_old */
	uint8_t i;
	struct aps_data *aps = container_of(work, struct aps_data, work);

	PROXIMITY_DEBUG("aps_12d_work_func enter\n ");
	/* add printf infomation at the key point */
	if(INTERSIL == intersil_flag)
	{
		APS_DBG("intersil!\n");
	}
	else
	{
		APS_DBG("everlight!\n");
	}
	if (atomic_read(&p_flag)) {
		/* Command 1 register: IR once */
		adjust_time = 0;
	re_adjust:
		/* init the range to the num last time we set */
		if(( range_index >=0 ) && ( range_index < TOTAL_RANGE_NUM ))
		{
            /* deal with intersil and everlight with diferent valure */
            if(EVERLIGHT == intersil_flag)
            {
                aps_i2c_reg_write(aps, APS_12D_REG_CMD2, \
                	(uint8_t)(APS_12D_IRDR_SEL_50MA << 6 | \
                		APS_12D_FREQ_SEL_DC << 4 | \
                		APS_12D_RES_SEL_12 << 2 | \
                		range_reg_value[range_index]));
				high_threshold = high_threshold_value_U8661[range_index];
				low_threshold = low_threshold_value_U8661[range_index];
            }
			else 
			{
			    aps_i2c_reg_write(aps, APS_12D_REG_CMD2, \
			        	(uint8_t)(APS_12D_IRDR_SEL_INTERSIL_50MA << 4 | \
			                		APS_FREQ_INTERSIL_DC << 6 | \
			                		APS_ADC_12 << 2 | \
			                		APS_INTERSIL_SCHEME_OFF| \
			                		range_reg_value[range_index]));
		        high_threshold = high_threshold_value_U8661_I[range_index];
			  	low_threshold = low_threshold_value_U8661_I[range_index];	
			}
		}
		else
		{
			PROXIMITY_DEBUG("BUG: range_index error!!!!\n");
			range_index = 0;
		}
	er_adjust:
        /* deal with intersil and everlight with diferent mode */
        if(EVERLIGHT == intersil_flag)
		{
			ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD1, APS_12D_IR_ONCE);
		}
        else
        {
            ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD1, APS_12D_IR_CONTINUOUS);
		}
	    msleep(45);
	    reg_val_lsb = aps_i2c_reg_read(aps, APS_12D_DATA_LSB);
	    reg_val_msb = aps_i2c_reg_read(aps, APS_12D_DATA_MSB);
	    ir_count = ((uint16_t)reg_val_msb << 8) + (uint16_t)reg_val_lsb;
		
	    PROXIMITY_DEBUG("IR once lsb=%d; msb=%d; ir_count=%d \n", reg_val_lsb, reg_val_msb, ir_count);
	    if (ir_count > 0xFFF){
		    PROXIMITY_DEBUG("get wrong ir value, ir_count=%d \n", ir_count);
		    ir_count = 0xFFF;
	    }
	    if (ir_count < 0){
		    PROXIMITY_DEBUG("get wrong ir value, ir_count=%d \n", ir_count);
		    ir_count = 0;
	    }
		/*
		 * auto adjust the range
		 * stratety:
		 * if current adc value >= up_range_value[i]
		 *     switch to upper range
		 * if current adc value < down_range_value[i]
		 *     switch to lower range
		 */
		if(ir_count > up_range_value[range_index])
		{
			if(adjust_time < TOTAL_RANGE_NUM-1)
			{
				if(range_index < TOTAL_RANGE_NUM-1)
				{
					range_index++;
					adjust_time++;
					goto re_adjust;
				}
				else
				{
					PROXIMITY_DEBUG("infrared ray TOO HIGH?\n");
				}
			}
			else
			{
				PROXIMITY_DEBUG("proximity readjust exceed max retry times.\n");
			}
		}
		else if((ir_count < down_range_value[range_index]))
		{
			if(adjust_time < TOTAL_RANGE_NUM-1)
			{
				if(range_index >= TOTAL_RANGE_NUM-1)
				{
					range_index--;
					adjust_time++;
					goto re_adjust;
				}
				else
				{
					PROXIMITY_DEBUG("BUG: no exist lux value!!\n");
				}
			}
			else
			{
				PROXIMITY_DEBUG("proximity readjust exceed max retry times.\n");
			}
		}
		
        /*all of the intersil's ONCE module has a big risk */
        if(EVERLIGHT == intersil_flag)
		{
			ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD1, APS_12D_PROXIMITY_ONCE);
		}
        else
        {
            ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD1, APS_12D_PROXIMITY_CONTINUOUS);
		}
	    msleep(45);
	    reg_val_lsb = aps_i2c_reg_read(aps, APS_12D_DATA_LSB);
	    reg_val_msb = aps_i2c_reg_read(aps, APS_12D_DATA_MSB);
	    ps_count = ((uint16_t)reg_val_msb << 8) + (uint16_t)reg_val_lsb;
	    PROXIMITY_DEBUG("PS once lsb=%d; msb=%d; ps_count=%d \n", reg_val_lsb, reg_val_msb, ps_count);
	    if (ps_count > 0xFFF){
		    PROXIMITY_DEBUG("get wrong ps value, ps_count=%d \n", ps_count);
		    ps_count = 0xFFF;
	    }
	    if (ps_count < 0){
		    PROXIMITY_DEBUG("get wrong ps value, ps_count=%d \n", ps_count);
		    ps_count = 0;
	    }
		/*get value of proximity and light*/
	     proximity_data_value = ps_count;
		if (range_index == 1){     
		     light_data_value = ir_count*RANG_VALUE;
		}		     
		else {		     
		     light_data_value = ir_count;
		}		     
		if ((ps_count - ir_count) > err_threshold_value[range_index] || (ps_count - ir_count) < 0 )
			flag = -1;
	    else if( (ps_count - ir_count) > high_threshold )
	     flag = 0;
	    else if( (ps_count - ir_count) < low_threshold )
	    	flag = 1;
	    else{
		    PROXIMITY_DEBUG("the value is in the threshold, do not report. \n");
	    }
        APS_DBG("the ps -ir is %d,the ps is %d,the ir is %d,the range_index is %d!\n",ps_count - ir_count,ps_count,ir_count,range_index);
        /* add printf infomation at the key point */
        APS_DBG("approch flag is %d ,0 is close,1 is far\n",flag);
		APS_DBG("lsb = 0x%x,msb = 0x%x\n",old_lsb,old_msb);
		/* skip invalid event */
		if(-1 != flag)
		{
			if(1 == flag)
			{
				/* report far event immediately */
				/* 0 is close, 1 is far */
				input_report_abs(aps->input_dev, ABS_DISTANCE, flag);
				input_sync(aps->input_dev);
			}
			else if(last_event != flag)
			{
				PROXIMITY_DEBUG("NOTE: skip unstable data: %s !!!\n", flag ? "far" : "close");
				last_event = flag;
				goto er_adjust;
			}
			else
			{
				PROXIMITY_DEBUG("report distance flag=%d \n", flag);
				/* 0 is close, 1 is far */
				input_report_abs(aps->input_dev, ABS_DISTANCE, flag);
				input_sync(aps->input_dev);
			}
		}

		/* save the last event */
		last_event = flag;

		/*
		 * reset the sensor range to 1000.
		 */
		if(0 != range_index)
		{
		    /*all of the intersil's ONCE module has a big risk */
            if(EVERLIGHT == intersil_flag)
            {
        		aps_i2c_reg_write(aps, APS_12D_REG_CMD2, \
        				(uint8_t)(APS_12D_IRDR_SEL_50MA << 6 | \
        					APS_12D_FREQ_SEL_DC << 4 | \
        					APS_12D_RES_SEL_12 << 2 | \
        					APS_12D_RANGE_SEL_ALS_1000));
            }
            else 
            {
                aps_i2c_reg_write(aps, APS_12D_REG_CMD2, \
                    	(uint8_t)(APS_12D_IRDR_SEL_INTERSIL_50MA << 4 | \
                    		APS_FREQ_INTERSIL_DC << 6 | \
                    		APS_ADC_12 << 2 | \
                    		APS_INTERSIL_SCHEME_OFF | \
                    		APS_12D_RANGE_SEL_ALS_1000));
            }
		}
	}

	if (atomic_read(&l_flag)) 
		{
		/* deal with intersil and everlight with diferent mode */
        if(EVERLIGHT == intersil_flag)
        {
		    ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD1, APS_12D_ALS_ONCE);
        }
        else
        {
            ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD1, APS_12D_ALS_CONTINUOUS);
        }
		msleep(45);
		reg_val_lsb = aps_i2c_reg_read(aps, APS_12D_DATA_LSB);
		reg_val_msb = aps_i2c_reg_read(aps, APS_12D_DATA_MSB);
		als_count = ((uint16_t)reg_val_msb << 8) + (uint16_t)reg_val_lsb;
        /* add printf infomation at the key point */
        APS_DBG("ALS once lsb=%d; msb=%d; als_count=%d \n", reg_val_lsb, reg_val_msb, als_count);

		if (als_count > 0xFFF){
			PROXIMITY_DEBUG("get wrong als value, als_count=%d \n", als_count);
			als_count = 0xFFF;
		}

		als_level = LSENSOR_MAX_LEVEL - 1;
		/* deal with intersil and everlight lsensor */
		for (i = 0; i < ARRAY_SIZE(lsensor_adc_U8661_table); i++)
		{
		    if(EVERLIGHT == intersil_flag)
            {
                if (als_count <= lsensor_adc_U8661_table[i])
                {
    				als_level = i;
    				break;
    			}
            }
            else
            {
                if (als_count <= lsensor_adc_U8661_Inter_table[i])
                {
    				als_level = i;
    				break;
    			}
            }
		}
		/* if it's intersil,it shoule be a power_down,because Power-up and Power Supply Considerations is not correct */
		APS_DBG("report adc level=%d \n", als_level);
		if(INTERSIL == intersil_flag)
		{
			ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD1, APS_12D_POWER_DOWN);
			msleep(10);				
		}
		if(aps_first_read)
		{
			/* report a invalid key first */
			aps_first_read = 0;
		/*zhangtao 20101230 make the changge to config the sensor begin */
		/*when the aps first read we changge the report value the value give the brghtness min */
		/*when the aps first read we don't changge the leds brightness */
			input_report_abs(aps->input_dev, ABS_LIGHT, -1);
		/*zhangtao 20101230 make the changge to config the sensor end */
			input_sync(aps->input_dev);
		}
		else
		{
			PROXIMITY_DEBUG("report lux value=%d \n", lsensor_lux_table[als_level]);
		/*zhangtao 20101230 make the changge to config the sensor begin */
		/*in the android 2.3 it will make the value table in the hal so we report only value*/
			input_report_abs(aps->input_dev, ABS_LIGHT, als_level);
		/*zhangtao 20101230 make the changge to config the sensor end */
			input_sync(aps->input_dev);
		}
		
	}
	
	if (atomic_read(&p_flag) || atomic_read(&l_flag))
		hrtimer_start(&aps->timer, ktime_set(sesc, nsesc), HRTIMER_MODE_REL);
	
}

static enum hrtimer_restart aps_timer_func(struct hrtimer *timer)
{
	struct aps_data *aps = container_of(timer, struct aps_data, timer);		
	queue_work(aps_wq, &aps->work);
	return HRTIMER_NORESTART;
}

static int aps_12d_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{	
        int i;
    /* define and initialization the value */
    int value_lsb = 0;
    int value_msb = 0;   
	int ret;
	struct aps_data *aps;

	printk(KERN_INFO "aps_12d_probe enter\n ");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "aps_12d_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	
	aps = kzalloc(sizeof(*aps), GFP_KERNEL);
	if (aps == NULL) {
		ret = -ENOMEM;
		goto err_check_functionality_failed;
	}

	mutex_init(&aps->mlock);

	INIT_WORK(&aps->work, aps_12d_work_func);
	aps->client = client;
	i2c_set_clientdata(client, aps);

	printk(KERN_INFO "aps_12d_probe send command 2\n ");
	/* Command 2 register: 25mA,DC,12bit,Range1 */
    
	/*power_down to avoid the iic read error */
    aps_i2c_reg_write(aps, APS_12D_REG_CMD1, APS_12D_POWER_DOWN);
	
	/*init the flag,because everlight's 0x06,0x07's register's value low 4 bit is 0*/
    value_lsb = aps_i2c_reg_read(aps, APS_INT_HT_LSB);
    value_msb = aps_i2c_reg_read(aps, APS_INT_HT_MSB);
    old_lsb = value_lsb;
    old_msb = value_msb;
	
	/* debug--- stare at the value of lsb & msb */
	APS_DBG("value_lsb=%d,value_msb=%d\n",value_lsb,value_msb);
	/* judge the device type */
    if((0xFF == value_lsb) || (0xFF == value_msb))
    {
        intersil_flag = INTERSIL;
    }
	
	/* write reg value for the two device */
    if(EVERLIGHT == intersil_flag)
    {
	    ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD2, \
	                         (uint8_t)(APS_12D_IRDR_SEL_50MA << 6 | \
	                                   APS_12D_FREQ_SEL_DC << 4 | \
	                                   APS_12D_RES_SEL_12 << 2 | \
	                                   APS_12D_RANGE_SEL_ALS_1000));
    }
    else 
    {
		/*because Power-up and Power Supply Considerations is not good enough for intersil's datasheet,so avoid it via software*/
		ret = aps_i2c_reg_write(aps, APS_TEST, APS_12D_POWER_DOWN);
		if (ret < 0) 
		{
			PROXIMITY_DEBUG("APS_TEST error!\n");
		}
		msleep(10);
		ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD1, APS_12D_POWER_DOWN);
		if (ret < 0) 
		{
			PROXIMITY_DEBUG("APS_12D_POWER_DOWN error!\n");
		}
		msleep(10);
        ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD2, \
    	                         (uint8_t)(APS_12D_IRDR_SEL_INTERSIL_50MA << 4 | \
    	                                   APS_FREQ_INTERSIL_DC << 6 | \
    	                                   APS_ADC_12 << 2 | \
    	                                   APS_INTERSIL_SCHEME_OFF| \
    	                                   APS_12D_RANGE_SEL_ALS_1000));
    }
        
        err_threshold_value[1] = 50;
	if (ret < 0) {
		goto err_detect_failed;
	}
	range_index = 0;

    #ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_APS);
    #endif

	for(i = 0; i < TOTAL_RANGE_NUM; i++)
	{
		/* NOTE: do NOT use the last one */
		/* get the down_range_value */
		if(EVERLIGHT == intersil_flag)
		{
			up_range_value[i] = MAX_ADC_OUTPUT - high_threshold_value_U8661[i] - RANGE_FIX + 500; 
		}
		else
		{
			up_range_value[i] = MAX_ADC_OUTPUT - high_threshold_value_U8661_I[i] - RANGE_FIX + 500; 
		}
	}

	down_range_value[0] = 0;
	for(i = 1; i < TOTAL_RANGE_NUM; i++)
	{
		/* NOTE: do not use the first one */
        
		/* get the down_range_value */
		if(EVERLIGHT == intersil_flag)
		{
			down_range_value[i] = (MAX_ADC_OUTPUT - high_threshold_value_U8661[i-1] - (MAX_ADC_OUTPUT / ADJUST_GATE)) / 4 - 650;
		}
		else
		{
			down_range_value[i] = (MAX_ADC_OUTPUT - high_threshold_value_U8661_I[i-1] - (MAX_ADC_OUTPUT / ADJUST_GATE)) / 4 - 650;
		}
	}

	/*zhangtao 20101230 make the changge to config the sensor begin */
	/*we don't use the input device sensors again */
	aps->input_dev = input_allocate_device();
	if (aps->input_dev == NULL) {
		ret = -ENOMEM;
		PROXIMITY_DEBUG(KERN_ERR "aps_12d_probe: Failed to allocate input device\n");
		goto err_detect_failed;
	}
	aps->input_dev->name = "sensors_aps";
	
	aps->input_dev->id.bustype = BUS_I2C;
	
	input_set_drvdata(aps->input_dev, aps);
	
	ret = input_register_device(aps->input_dev);
	if (ret) {
		printk(KERN_ERR "aps_probe: Unable to register %s input device\n", aps->input_dev->name);
		goto err_input_register_device_failed;
	}
	/*zhangtao 20101230 make the changge to config the sensor begin */
	
	set_bit(EV_ABS, aps->input_dev->evbit);
	input_set_abs_params(aps->input_dev, ABS_LIGHT, 0, 10240, 0, 0);
	input_set_abs_params(aps->input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = misc_register(&light_device);
	if (ret) {
		printk(KERN_ERR "aps_12d_probe: light_device register failed\n");
		goto err_light_misc_device_register_failed;
	}

	ret = misc_register(&proximity_device);
	if (ret) {
		printk(KERN_ERR "aps_12d_probe: proximity_device register failed\n");
		goto err_proximity_misc_device_register_failed;
	}

	if( light_device.minor != MISC_DYNAMIC_MINOR ){
		light_device_minor = light_device.minor;
	}
	
	if( proximity_device.minor != MISC_DYNAMIC_MINOR ){
		proximity_device_minor = proximity_device.minor ;
	}

	wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proximity");

	hrtimer_init(&aps->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aps->timer.function = aps_timer_func;
	
	aps_wq = create_singlethread_workqueue("aps_wq");

	if (!aps_wq) {
		ret = -ENOMEM;
		goto err_create_workqueue_failed;
	}
	
	this_aps_data =aps;

	printk(KERN_INFO "aps_12d_probe: Start Proximity Sensor APS-12D\n");

#ifdef CONFIG_MELFAS_UPDATE_TS_FIRMWARE
	TS_updateFW_aps_data = this_aps_data;
	TS_updateFW_aps_wq = aps_wq;
#endif
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_APS);
#endif

	return 0;
	
err_create_workqueue_failed:
	misc_deregister(&proximity_device);
err_proximity_misc_device_register_failed:
	misc_deregister(&light_device);
err_light_misc_device_register_failed:
err_input_register_device_failed:
	input_free_device(aps->input_dev);
err_detect_failed:
	kfree(aps);
err_check_functionality_failed:
	return ret;
  
}

static int aps_12d_remove(struct i2c_client *client)
{
	struct aps_data *aps = i2c_get_clientdata(client);

	PROXIMITY_DEBUG("aps_12d_remove enter\n ");

	hrtimer_cancel(&aps->timer);

	misc_deregister(&light_device);
	misc_deregister(&proximity_device);

	input_unregister_device(aps->input_dev);

	kfree(aps);
	return 0;
}

static int aps_12d_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct aps_data *aps = i2c_get_clientdata(client);

	PROXIMITY_DEBUG("aps_12d_suspend enter\n ");

	hrtimer_cancel(&aps->timer);
	ret = cancel_work_sync(&aps->work);

	ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD1, APS_12D_POWER_DOWN);

	if (aps->power) {
		ret = aps->power(0);
		if (ret < 0)
			printk(KERN_ERR "aps_12d_suspend power off failed\n");
	}

	return 0;
}

static int aps_12d_resume(struct i2c_client *client)
{
	int ret;
	struct aps_data *aps = i2c_get_clientdata(client);

	PROXIMITY_DEBUG("aps_12d_resume enter\n ");
        
    /* write reg value for the two device */
    if(EVERLIGHT == intersil_flag)
    {
		ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD2, \
	                         (uint8_t)(APS_12D_IRDR_SEL_50MA << 6 | \
	                                   APS_12D_FREQ_SEL_DC << 4 | \
	                                   APS_12D_RES_SEL_12 << 2 | \
	                                   APS_12D_RANGE_SEL_ALS_1000));
    }
    else 
    {
        ret = aps_i2c_reg_write(aps, APS_12D_REG_CMD2, \
    	                         (uint8_t)(APS_12D_IRDR_SEL_INTERSIL_50MA << 4 | \
    	                                   APS_FREQ_INTERSIL_DC << 6 | \
    	                                   APS_ADC_12 << 2 | \
    	                                   APS_INTERSIL_SCHEME_OFF| \
    	                                   APS_12D_RANGE_SEL_ALS_1000));
    }

	hrtimer_start(&aps->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return 0;
}

static const struct i2c_device_id aps_id[] = {
	{ "aps-12d", 0 },
	{ }
};

static struct i2c_driver aps_driver = {
	.probe		= aps_12d_probe,
	.remove		= aps_12d_remove,
	.suspend	= aps_12d_suspend,
	.resume		= aps_12d_resume,
	.id_table	= aps_id,
	.driver = {
		.name	="aps-12d",
	},
};

static int __devinit aps_12d_init(void)
{
	return i2c_add_driver(&aps_driver);
}

static void __exit aps_12d_exit(void)
{
	i2c_del_driver(&aps_driver);
	if (aps_wq)
		destroy_workqueue(aps_wq);
}

device_initcall_sync(aps_12d_init);
module_exit(aps_12d_exit);

MODULE_DESCRIPTION("Proximity Driver");
MODULE_LICENSE("GPL");
