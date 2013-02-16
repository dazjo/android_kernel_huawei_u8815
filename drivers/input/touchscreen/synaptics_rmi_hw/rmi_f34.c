/*
 * Copyright (c) 2011 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include "rmi_driver.h"
#include "rmi_config.h"

/* define fn $34 commands */
#define WRITE_FW_BLOCK            0x2
#define ERASE_ALL                 0x3
#define READ_CONFIG_BLOCK         0x5
#define WRITE_CONFIG_BLOCK        0x6
#define ERASE_CONFIG              0x7
#define ENABLE_FLASH_PROG         0xf

#define STATUS_IN_PROGRESS        0xff
#define STATUS_IDLE		  0x80

#define PDT_START_SCAN_LOCATION	0x00e9
#define PDT_END_SCAN_LOCATION	0x0005

#define BLK_SZ_OFF	3
#define IMG_BLK_CNT_OFF	5
#define CFG_BLK_CNT_OFF	7

#define BLK_NUM_OFF 2

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 32)
#define KERNEL_VERSION_ABOVE_2_6_32 1
#endif

/* data specific to fn $34 that needs to be kept around */
struct rmi_fn_34_data {
	unsigned char status;
	unsigned char cmd;
	unsigned short bootloaderid;
	unsigned short blocksize;
	unsigned short imageblockcount;
	unsigned short configblockcount;
	unsigned short blocknum;
	bool inflashprogmode;
	unsigned int configid; 
	struct mutex attn_mutex;
};

static int register_update_firmware_node(struct rmi_function_container *fc);

static ssize_t update_firmware_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf);
static ssize_t update_firmware_store(struct kobject *kobj, 
					struct kobj_attribute *attr, const char *buf, size_t count);
static int i2c_update_firmware(struct rmi_function_container *fc);

/*cmd = WRITE_CONFIG_BLOCK download config
 *cmd = WRITE_FW_BLOCK download fw
 */
static int synaptics_download(struct rmi_function_container *fc,
					const unsigned char *pgm_data,u8 cmd);
static int rmi4_read_PDT(struct rmi_function_container *fc);
static int rmi4_enable_program(struct rmi_function_container *fc);
static int rmi4_wait_attn(struct rmi_function_container *fc,int udelayed);
static int rmi4_program_firmware(struct rmi_function_container *fc,
					const unsigned char *pgm_data);
static int rmi4_check_firmware(struct rmi_function_container *fc,
					const unsigned char *pgm_data);
static int rmi4_program_configuration(struct rmi_function_container *fc,
					const unsigned char *pgm_data,u8 cmd );
static int rmi4_write_image(struct rmi_function_container *fc,
					unsigned char type_cmd,const unsigned char *pgm_data);
static int rmi4_disable_program(struct rmi_function_container *fc);

/*entry of auto update config*/
static int auto_update_config(struct rmi_function_container *fc);
/*
 * return 1: latest performance ,don't need update
 */
static int config_array_is_latest(struct rmi_function_container *fc,
					unsigned char * pconfigarray);

static struct kobj_attribute update_firmware_attribute = 
{
	.attr = {
			.name = "update_firmware",
			.mode = 0664
		   },
	.show = update_firmware_show,
	.store = update_firmware_store
};

static struct rmi_function_container *g_fc = NULL;

static void get_module_id(struct rmi_function_container *fc);

static ssize_t rmi_fn_34_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf);


static ssize_t rmi_fn_34_status_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count);

static ssize_t rmi_fn_34_cmd_show(struct device *dev,
				  struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_34_cmd_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

#ifdef KERNEL_VERSION_ABOVE_2_6_32
static ssize_t rmi_fn_34_data_read(struct file *data_file, struct kobject *kobj,
				   struct bin_attribute *attributes,
				   char *buf, loff_t pos, size_t count);

static ssize_t rmi_fn_34_data_write(struct file *data_file,
				    struct kobject *kobj,
				    struct bin_attribute *attributes, char *buf,
				    loff_t pos, size_t count);
#else
static ssize_t rmi_fn_34_data_read(struct kobject *kobj,
				   struct bin_attribute *attributes,
				   char *buf, loff_t pos, size_t count);

static ssize_t rmi_fn_34_data_write(struct kobject *kobj,
				    struct bin_attribute *attributes, char *buf,
				    loff_t pos, size_t count);
#endif

static ssize_t rmi_fn_34_bootloaderid_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf);

static ssize_t rmi_fn_34_bootloaderid_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count);

static ssize_t rmi_fn_34_blocksize_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t rmi_fn_34_imageblockcount_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf);

static ssize_t rmi_fn_34_configblockcount_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf);

static ssize_t rmi_fn_34_blocknum_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

static ssize_t rmi_fn_34_blocknum_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

static ssize_t rmi_fn_34_rescanPDT_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);


static ssize_t rmi_fn_34_moduleID_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t rmi_fn_34_configID_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf);

static int rmi_f34_alloc_memory(struct rmi_function_container *fc);

static void rmi_f34_free_memory(struct rmi_function_container *fc);

static int rmi_f34_initialize(struct rmi_function_container *fc);

static int rmi_f34_config(struct rmi_function_container *fc);

static int rmi_f34_reset(struct rmi_function_container *fc);

static int rmi_f34_create_sysfs(struct rmi_function_container *fc);

static char productid[RMI_PRODUCT_ID_LENGTH + 1] = {0};
static u8 touch_module_id = 0;
u32 touch_configid = 0;

static struct device_attribute attrs[] = {
	__ATTR(status, RMI_RW_ATTR,
	       rmi_fn_34_status_show, rmi_fn_34_status_store),

	/* Also, sysfs will need to have a file set up to distinguish
	 * between commands - like Config write/read, Image write/verify. */
	__ATTR(cmd, RMI_RW_ATTR,
	       rmi_fn_34_cmd_show, rmi_fn_34_cmd_store),
	__ATTR(bootloaderid, RMI_RW_ATTR,
	       rmi_fn_34_bootloaderid_show, rmi_fn_34_bootloaderid_store),
	__ATTR(blocksize, RMI_RO_ATTR,
	       rmi_fn_34_blocksize_show, rmi_store_error),
	__ATTR(imageblockcount, RMI_RO_ATTR,
	       rmi_fn_34_imageblockcount_show, rmi_store_error),
	__ATTR(configblockcount, RMI_RO_ATTR,
	       rmi_fn_34_configblockcount_show, rmi_store_error),
	__ATTR(blocknum, RMI_RW_ATTR,
	       rmi_fn_34_blocknum_show, rmi_fn_34_blocknum_store),
	__ATTR(rescanPDT, RMI_WO_ATTR,
	       rmi_show_error, rmi_fn_34_rescanPDT_store),
	__ATTR(moduleID, RMI_RW_ATTR,
	       rmi_fn_34_moduleID_show, rmi_store_error),
	__ATTR(configID, RMI_RO_ATTR,
	       rmi_fn_34_configID_show, rmi_store_error),
};

struct bin_attribute dev_attr_data = {
	.attr = {
		 .name = "data",
		 /*change mode of data */
		 .mode = 0664},
	.size = 0,
	.read = rmi_fn_34_data_read,
	.write = rmi_fn_34_data_write,
};


static int rmi_f34_init(struct rmi_function_container *fc)
{
	int retval;

	dev_info(&fc->dev, "Intializing f34 values.");

	/* init instance data, fill in values and create any sysfs files */
	retval = rmi_f34_alloc_memory(fc);
	if (retval < 0)
		goto exit_free_data;

	retval = rmi_f34_initialize(fc);
	if (retval < 0)
		goto exit_free_data;

	retval = rmi_f34_create_sysfs(fc);
	if (retval < 0)
		goto exit_free_data;

	/*auto update config for touch performance*/
	retval = auto_update_config(fc);
	if (retval < 0)
		dev_warn(&fc->dev, "update config fail\n");

	/*register update firmware node*/
	register_update_firmware_node(fc);
	return 0;

exit_free_data:
	rmi_f34_free_memory(fc);

	return retval;
}

static int rmi_f34_alloc_memory(struct rmi_function_container *fc)
{
	struct rmi_fn_34_data *f34;

	f34 = kzalloc(sizeof(struct rmi_fn_34_data), GFP_KERNEL);
	if (!f34) {
		dev_err(&fc->dev, "Failed to allocate rmi_fn_34_data.\n");
		return -ENOMEM;
	}
	fc->data = f34;

	return 0;
}

static void rmi_f34_free_memory(struct rmi_function_container *fc)
{
	kfree(fc->data);
	fc->data = NULL;
}

static int rmi_f34_initialize(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_device_platform_data *pdata;
	int retval = 0;
	struct rmi_fn_34_data *f34 = fc->data;
	u16 query_base_addr;
	u16 control_base_addr;
	unsigned char buf[2];
	unsigned char configid[5];

	pdata = to_rmi_platform_data(rmi_dev);
	dev_dbg(&fc->dev, "Initializing F34 values for %s.\n",
		pdata->sensor_name);

	mutex_init(&f34->attn_mutex);

	/* get the Bootloader ID and Block Size. */
	query_base_addr = fc->fd.query_base_addr;
	control_base_addr = fc->fd.control_base_addr;

	retval = rmi_read_block(fc->rmi_dev, query_base_addr, buf,
			ARRAY_SIZE(buf));

	if (retval < 0) {
		dev_err(&fc->dev, "Could not read bootloaderid from 0x%04x.\n",
			query_base_addr);
		return retval;
	}

	batohs(&f34->bootloaderid, buf);

	retval = rmi_read_block(fc->rmi_dev, query_base_addr + BLK_SZ_OFF, buf,
			ARRAY_SIZE(buf));

	if (retval < 0) {
		dev_err(&fc->dev, "Could not read block size from 0x%04x, "
			"error=%d.\n", query_base_addr + BLK_SZ_OFF, retval);
		return retval;
	}
	batohs(&f34->blocksize, buf);

	/* Get firmware image block count and store it in the instance data */
	retval = rmi_read_block(fc->rmi_dev, query_base_addr + IMG_BLK_CNT_OFF,
			buf, ARRAY_SIZE(buf));

	if (retval < 0) {
		dev_err(&fc->dev, "Couldn't read image block count from 0x%x, "
			"error=%d.\n", query_base_addr + IMG_BLK_CNT_OFF,
			retval);
		return retval;
	}
	batohs(&f34->imageblockcount, buf);

	/* Get config block count and store it in the instance data */
	retval = rmi_read_block(fc->rmi_dev, query_base_addr + 7, buf,
			ARRAY_SIZE(buf));

	if (retval < 0) {
		dev_err(&fc->dev, "Couldn't read config block count from 0x%x, "
			"error=%d.\n", query_base_addr + CFG_BLK_CNT_OFF,
			retval);
		return retval;
	}
	batohs(&f34->configblockcount, buf);

	/*get config id*/
	retval = rmi_read_block(fc->rmi_dev, control_base_addr,
			(unsigned char *)configid, 4);
	if (retval < 0) {
		dev_err(&fc->dev, "%s : Could not read data from 0x%04x\n",
		       __func__, control_base_addr);
	}
	configid[4] = '\0';
	retval = kstrtou32(configid, 10, &f34->configid);
	if (retval < 0) 
		dev_err(&fc->dev, "%s : transfer fail\n",
		       __func__);
	touch_configid = f34->configid;
	/*get touch module and set touch_module_id*/
	get_module_id(fc);
	return 0;
}

static int rmi_f34_create_sysfs(struct rmi_function_container *fc)
{
	int attr_count = 0;
	int rc;

	dev_dbg(&fc->dev, "Creating sysfs files.");

	/* We need a sysfs file for the image/config block to write or read.
	 * Set up sysfs bin file for binary data block. Since the image is
	 * already in our format there is no need to convert the data for
	 * endianess. */
	rc = sysfs_create_bin_file(&fc->dev.kobj,
				&dev_attr_data);
	if (rc < 0) {
		dev_err(&fc->dev, "Failed to create sysfs file for F34 data "
		     "(error = %d).\n", rc);
		return -ENODEV;
	}

	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&fc->dev.kobj, &attrs[attr_count].attr) < 0) {
			dev_err(&fc->dev, "Failed to create sysfs file for %s.",
			     attrs[attr_count].attr.name);
			rc = -ENODEV;
			goto err_remove_sysfs;
		}
	}

	return 0;

err_remove_sysfs:
	sysfs_remove_bin_file(&fc->dev.kobj, &dev_attr_data);

	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&fc->dev.kobj,
				  &attrs[attr_count].attr);
	return rc;
}

static int auto_update_config(struct rmi_function_container *fc)
{
	unsigned char * pconfig_array = NULL;
	int irq = 0;
	struct rmi_device_platform_data *pdata;
	int retval;
	struct rmi_function_container *f01_fc; 
	struct rmi_driver_data *driver_data;
		
	/*get config array*/
	pconfig_array = get_config_array();
	if (pconfig_array == NULL)
	{
		pr_warn("%s: can't get config array\n",__func__);
		return 0;
	}

	/*determine latest performance*/
	if (config_array_is_latest(fc,pconfig_array))
	{
		pr_info("%s: the touch performance is latest, don't need update!!\n",__func__);
		return 0;
	}

	/*auto update config to touch IC*/
			
	pdata = to_rmi_platform_data(fc->rmi_dev);
		
	irq = gpio_to_irq(pdata->attn_gpio);
	printk("irq:%d\n",irq);
		
	disable_irq(irq);
	synaptics_download(fc,pconfig_array,WRITE_CONFIG_BLOCK);
	enable_irq(irq);

	/*f01 reset*/
	driver_data = rmi_get_driverdata(fc->rmi_dev);
	f01_fc = driver_data->f01_container;
	retval = rmi_write(fc->rmi_dev, f01_fc->fd.command_base_addr,1);
	if (retval < 0) {
		pr_info("%s: f01 reset fail!!\n",__func__);
		return retval;
	}
	return 0;
}


/*register update firmware node*/
static int register_update_firmware_node(struct rmi_function_container *fc)
{
	int ret;
	struct kobject *kobject_ts;
	kobject_ts = kobject_create_and_add("touch_screen", NULL);
	if (!kobject_ts)
	{
		printk("create kobjetct error!\n");
		return -1;
	}
	g_fc = fc;
	ret = sysfs_create_file(kobject_ts, &update_firmware_attribute.attr);
	if (ret) {
		kobject_put(kobject_ts);
		printk("create file error\n");
		return -1;
	}
	return 0;	
}

static ssize_t update_firmware_show(struct kobject *kobj, 
										struct kobj_attribute *attr,
										char *buf)
{
	return 1;
}

static ssize_t update_firmware_store(struct kobject *kobj,
										struct kobj_attribute *attr, 
										const char *buf, size_t count)
{
	struct rmi_device *rmi_dev;
	struct rmi_device_platform_data *pdata;
	int irq = 0;
	char ret = -1;

	printk("#################update_firmware_store######################\n");

	if ( (buf[0] == '2')&&(buf[1] == '\0') )
	{
		rmi_dev = g_fc->rmi_dev;
		pdata = to_rmi_platform_data(rmi_dev);
		
		irq = gpio_to_irq(pdata->attn_gpio);
		printk("irq:%d\n",irq);
		
		disable_irq(irq);
		
		/*update firmware*/
		ret = i2c_update_firmware(g_fc);
		
		enable_irq(irq);
 
		if( 0 != ret )
		{
			printk("Update firmware failed!\n");
			ret = -1;
		} 
		else 
		{
			printk("Update firmware success!\n");
			arm_pm_restart(0,&ret);
			ret = 1;
		}
	}

	return ret;
}

static int i2c_update_firmware(struct rmi_function_container *fc) 
{
	char *buf;
	struct file	*filp;
	struct inode *inode = NULL;
	mm_segment_t oldfs;
	uint16_t	length;
	int ret = 0;
	const char filename[]="/sdcard/update/synaptics.img";

	/* open file */
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(filename, O_RDONLY, S_IRUSR);
	if (IS_ERR(filp))
	{
            printk("%s: file %s filp_open error\n", __FUNCTION__,filename);
            set_fs(oldfs);
            return -1;
	}

	if (!filp->f_op)
	{
            printk("%s: File Operation Method Error\n", __FUNCTION__);
            filp_close(filp, NULL);
            set_fs(oldfs);
            return -1;
	}

	inode = filp->f_path.dentry->d_inode;
	if (!inode) 
	{
		printk("%s: Get inode from filp failed\n", __FUNCTION__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		
		return -1;
	}

	/* file's size */
	length = i_size_read(inode->i_mapping->host);
	if (!( length > 0 && length < 62*1024 ))
	{
		printk("file size error\n");
		filp_close(filp, NULL);
		set_fs(oldfs);
		
		return -1;
	}

	/* allocation buff size */
	buf = vmalloc(length+(length%2));
	if (!buf) 
	{
		printk("alloctation memory failed\n");
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* read data */
	if (filp->f_op->read(filp, buf, length, &filp->f_pos) != length)
	{
		printk("%s: file read error\n", __FUNCTION__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		vfree(buf);
		
		return -1;
	}

	ret = synaptics_download(fc,buf,WRITE_FW_BLOCK);

 	filp_close(filp, NULL);
	set_fs(oldfs);
	vfree(buf);
	
	return ret;
}
static int synaptics_download(struct rmi_function_container *fc,
	 								const unsigned char *pgm_data,u8 cmd)
{
	int ret;

	ret = rmi4_read_PDT(fc);
	if(ret != 0)
	{
		printk("RMI page func check error\n");
		return -1;
	}

	ret = rmi4_enable_program(fc);
	if( ret != 0)
	{
		printk("%s:%d:RMI enable program error,return...\n",__FUNCTION__,__LINE__);
		goto error;
	}

    if (cmd == WRITE_FW_BLOCK)
    {
		ret = rmi4_check_firmware(fc,pgm_data);
		if( ret != 0)
		{
			printk("%s:%d:RMI check firmware error,return...\n",__FUNCTION__,__LINE__);
			goto error;
		}
    

		ret = rmi4_program_firmware(fc, pgm_data + 0x100);
		if( ret != 0)
		{
			printk("%s:%d:RMI program firmware error,return...",__FUNCTION__,__LINE__);
			goto error;
		}

		rmi4_program_configuration(fc, pgm_data +  0x100,cmd);
	}
	else 
	{
		rmi4_program_configuration(fc, pgm_data ,cmd);
	}

	
	return rmi4_disable_program(fc);

error:
	rmi4_disable_program(fc);
	printk("%s:%d:error,return ....",__FUNCTION__,__LINE__);
	return -1;

}

static int rmi4_read_PDT(struct rmi_function_container *fc)
{
	struct rmi_fn_34_data *instance_data;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *driver_data;
	struct pdt_entry pdt_entry;
	bool fn01found = false;
	bool fn34found = false;
	int irq_count = 0;
	int retval = 0;
	int i;

	/* Rescan of the PDT is needed since issuing the Flash Enable cmd
	 * the device registers for Fn$01 and Fn$34 moving around because
	 * of the change from Bootloader mode to Flash Programming mode
	 * may change to a different PDT with only Fn$01 and Fn$34 that
	 * could have addresses for query, control, data, command registers
	 * that differ from the PDT scan done at device initialization. 
	 */
	instance_data = fc->data;
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);

	/* rescan the PDT - filling in Fn01 and Fn34 addresses -
	 * this is only temporary - the device will need to be reset
	 * to return the PDT to the normal values. 
	 */

	/* mini-parse the PDT - we only have to get Fn$01 and Fn$34 and
	 * since we are Flash Programming mode we only have page 0. 
	 */
	for (i = PDT_START_SCAN_LOCATION; i >= PDT_END_SCAN_LOCATION;
			i -= sizeof(pdt_entry)) 
	{
		retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					       sizeof(pdt_entry));
		if (retval != sizeof(pdt_entry)) 
		{
			pr_info( "%s: err frm rmi_read_block pdt "
					"entry data from PDT, "
					"error = %d.", __func__, retval);
			return retval;
		}

		if ((pdt_entry.function_number == 0x00) 
			||(pdt_entry.function_number == 0xff))
			break;

		pr_debug( "%s: Found F%.2X\n",
				__func__, pdt_entry.function_number);

		/* f01 found - just fill in the new addresses in
		 * the existing fc. 
		 */
		if (pdt_entry.function_number == 0x01) 
		{
			struct rmi_function_container *f01_fc =
					driver_data->f01_container;
			fn01found = true;
			f01_fc->fd.query_base_addr =
					pdt_entry.query_base_addr;
			f01_fc->fd.command_base_addr =
				  pdt_entry.command_base_addr;
			f01_fc->fd.control_base_addr =
				  pdt_entry.control_base_addr;
			f01_fc->fd.data_base_addr =
				  pdt_entry.data_base_addr;
			f01_fc->fd.function_number =
				  pdt_entry.function_number;
			f01_fc->fd.interrupt_source_count =
				  pdt_entry.interrupt_source_count;
			f01_fc->num_of_irqs =
				  pdt_entry.interrupt_source_count;
			f01_fc->irq_pos = irq_count;

			irq_count += f01_fc->num_of_irqs;

			if (fn34found)
				break;
		}

		/* f34 found - just fill in the new addresses in
		 * the existing fc. 
		 */
		if (pdt_entry.function_number == 0x34) 
		{
			fn34found = true;
			fc->fd.query_base_addr =
				  pdt_entry.query_base_addr;
			fc->fd.command_base_addr =
				  pdt_entry.command_base_addr;
			fc->fd.control_base_addr =
				  pdt_entry.control_base_addr;
			fc->fd.data_base_addr =
				  pdt_entry.data_base_addr;
			fc->fd.function_number =
				  pdt_entry.function_number;
			fc->fd.interrupt_source_count =
				  pdt_entry.interrupt_source_count;
			fc->num_of_irqs =
				  pdt_entry.interrupt_source_count;
			fc->irq_pos = irq_count;

			irq_count += fc->num_of_irqs;

			if (fn01found)
				break;
		}

	}

	if (!fn01found || !fn34found) 
	{
		pr_info("%s: failed to find fn$01 or fn$34 trying "
				"to do rescan PDT.\n"
				, __func__);
		return -EINVAL;
	}

	return 0;
}
static int rmi4_enable_program(struct rmi_function_container *fc)
{
	unsigned char data[2];
	struct rmi_fn_34_data *instance_data;
	u16 data_base_addr;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *driver_data;
	int ret = -1;
	unsigned char status;
	printk("RMI4 enable program...\n");

	instance_data = fc->data;
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	
	/* Write the Bootloader ID key data back to the first two Block
	 * Data registers (F34_Flash_Data2.0 and F34_Flash_Data2.1). */

	ret = rmi_read_block(fc->rmi_dev, 
			fc->fd.query_base_addr, data,
			ARRAY_SIZE(data));

	if (ret < 0) {
		dev_err(&fc->dev, "Could not read bootloaderid from 0x%04x.\n",
			fc->fd.query_base_addr);
		return ret;
	}

	data_base_addr = fc->fd.data_base_addr;

	ret = rmi_write_block(fc->rmi_dev,
				data_base_addr + BLK_NUM_OFF,
				data,
				ARRAY_SIZE(data));

	if (ret < 0) 
	{
		pr_err( "%s : Could not write bootloader id to 0x%x\n",
		       __func__, data_base_addr + BLK_NUM_OFF);
		return ret;
	}

	/* Issue the Enable flash command to the device. */
	ret = rmi_read(fc->rmi_dev,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF,&status);
	status |= ENABLE_FLASH_PROG;
	ret = rmi_write(fc->rmi_dev,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF, status);
	if (ret < 0) 
	{
		pr_err("%s: Could not write command 0x%02x "
				"to 0x%04x\n", __func__, instance_data->cmd,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF);
		return ret;
	}		
    //printk("f34_status addr:0x%x\n",data_base_addr + instance_data->blocksize +BLK_NUM_OFF);
	ret = rmi4_wait_attn(fc,12);
	if (ret < 0) 
	{
		pr_err("%s:%d,error",__func__,__LINE__);
		return ret;
	}

	//Rescan the Page Description Table
	rmi4_read_PDT(fc);


	/*check status*/
	ret = rmi_read(fc->rmi_dev,
			  	data_base_addr + instance_data->blocksize +
			  	BLK_NUM_OFF, &status);

	if ((ret < 0) || (status != 0x80)) 
	{
		dev_err(&fc->dev, "Could not read status from 0x%x\n",
		       	data_base_addr + instance_data->blocksize + BLK_NUM_OFF);
		/* failure */
		return -1;
	}
	return ret;
}
static int rmi4_wait_attn(struct rmi_function_container *fc,int udelayed)
{
	struct rmi_fn_34_data *instance_data = fc->data;
	u16 data_base_addr = fc->fd.data_base_addr;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *driver_data;
	u8 status;
	int retval;
	u8 temp;
	int loop_count=0;
	struct rmi_function_container *f01_fc;

	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	f01_fc = driver_data->f01_container;

	rmi_read(fc->rmi_dev,
			  	f01_fc->fd.data_base_addr , &status);
	//printk("f01database:0x%x,status:0x%x",f01_fc->fd.data_base_addr,status);
	do
	{
		mdelay(udelayed);
		/* Read the Fn $34 status from F34_Flash_Data3 to see the previous
		 * commands status. F34_Flash_Data3 will be the address after the
		 * 2 block number registers plus blocksize Data registers.
		 *  inform user space - through a sysfs param. */
		retval = rmi_read(fc->rmi_dev,
			  	data_base_addr + instance_data->blocksize +
			  	BLK_NUM_OFF, &status);

		if (retval < 0) 
		{
			dev_err(&fc->dev, "Could not read status from 0x%x\n",
		       		data_base_addr + instance_data->blocksize + BLK_NUM_OFF);
			status = 0xff;	/* failure */
		}

		/* Clear the attention assertion by reading the interrupt status register */
		rmi_read(fc->rmi_dev,
				f01_fc->fd.data_base_addr + 1, 
				&temp);		
	}while(loop_count++ < 0x10 && (status != 0x80));

	/*printk("f34:0x%x,status:0x%x",data_base_addr + instance_data->blocksize +
			  	BLK_NUM_OFF,status);*/
	if(loop_count >= 0x10)
	{
		pr_err("rmi4 wait attn timeout:retval=0x%x\n",retval);
		return -1;
	}
	return 0;
}
static int rmi4_program_firmware(struct rmi_function_container *fc,
					const unsigned char *pgm_data)
{
	unsigned char data[2];
	struct rmi_fn_34_data *instance_data;
	u16 data_base_addr;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *driver_data;
	int ret = -1;
	u8 status = 0;
	
	pr_info("RMI4 program UI firmware...\n");
	
	instance_data = fc->data;
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	

	/* Write the Bootloader ID key data back to the first two Block
	 * Data registers (F34_Flash_Data2.0 and F34_Flash_Data2.1). */
	data_base_addr = fc->fd.data_base_addr;

	ret = rmi_read_block(fc->rmi_dev, 
			fc->fd.query_base_addr, data,
			ARRAY_SIZE(data));

	if (ret < 0) {
		dev_err(&fc->dev, "Could not read bootloaderid from 0x%04x.\n",
			fc->fd.query_base_addr);
		return ret;
	}
	
	ret = rmi_write_block(fc->rmi_dev,
				data_base_addr + BLK_NUM_OFF,
				data,
				ARRAY_SIZE(data));
	if (ret < 0) 
	{
		pr_err( "%s : Could not write bootloader id to 0x%x\n",
		       __func__, data_base_addr + BLK_NUM_OFF);
		return ret;
	}
	
	rmi_read(fc->rmi_dev,
			  	data_base_addr + instance_data->blocksize +
			  	BLK_NUM_OFF, &status);
	//printk("status:%d\n",status);
	/* Issue the Erase all command to the device. */
	ret = rmi_write(fc->rmi_dev,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF, ERASE_ALL);

	if (ret < 0) 
	{
		pr_err("%s: Could not write command 0x%02x "
				"to 0x%04x\n", __func__, instance_data->cmd,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF);
		return ret;
	}	

	rmi4_wait_attn(fc,300);

	/*check status*/
	ret = rmi_read(fc->rmi_dev,
			  	data_base_addr + instance_data->blocksize +
			  	BLK_NUM_OFF, &status);

	if ((ret < 0) || (status != 0x80)) 
	{
		dev_err(&fc->dev, "Could not read status from 0x%x\n",
		       	data_base_addr + instance_data->blocksize + BLK_NUM_OFF);
		/* failure */
		return -1;
	}

	/*write firmware*/
	if( rmi4_write_image(fc,WRITE_FW_BLOCK,pgm_data) <0 )
	{
		pr_err("write UI firmware error!\n");
		return -1;
	}

	/*check status*/
	ret = rmi_read(fc->rmi_dev,
			  	data_base_addr + instance_data->blocksize +
			  	BLK_NUM_OFF, &status);

	if ((ret < 0) || (status != 0x80)) 
	{
		dev_err(&fc->dev, "Could not read status from 0x%x\n",
		       	data_base_addr + instance_data->blocksize + BLK_NUM_OFF);
		/* failure */
		return -1;
	}
	return 0;
}

static int rmi4_check_firmware(struct rmi_function_container *fc,
							const unsigned char *pgm_data)
{
	unsigned short bootloader_id;
	const unsigned char *SynaFirmware;
	struct rmi_fn_34_data *instance_data;
	unsigned char m_firmwareImgVersion;

	instance_data = fc->data;
  	SynaFirmware = pgm_data;
	bootloader_id = (unsigned int)SynaFirmware[4] + (unsigned int)SynaFirmware[5]*0x100;
	m_firmwareImgVersion = SynaFirmware[7];

	if ((m_firmwareImgVersion == 0) && (instance_data->bootloaderid != bootloader_id))
	{
		return -1;
	}
	return 0;
}

static int rmi4_program_configuration(struct rmi_function_container *fc,
							const unsigned char *pgm_data,u8 cmd )
{
	struct rmi_fn_34_data *instance_data;
	int ret;
	unsigned short block_size;
	unsigned short ui_blocks;
	u16 data_base_addr;
	u8 status = 0;
	unsigned short bootloader_id = 0 ;
	unsigned char data[2];

	printk("\nRMI4 program Config firmware...\n");
	instance_data = fc->data;
	data_base_addr = fc->fd.data_base_addr;
	if (cmd == WRITE_FW_BLOCK)
	{
		block_size = instance_data->blocksize;//read Block Size 0
		ui_blocks = instance_data->imageblockcount;
		

		if(rmi4_write_image(fc, WRITE_CONFIG_BLOCK,pgm_data+ui_blocks*block_size ) < 0)
		{
			pr_err("write configure image error\n");
			return -1;
		}
	}
	else
	{
		bootloader_id = instance_data->bootloaderid;

		/* Write the Bootloader ID key data back to the first two Block
	 	 * Data registers (F34_Flash_Data2.0 and F34_Flash_Data2.1). */
		hstoba(data, bootloader_id);
		ret = rmi_write_block(fc->rmi_dev,
				data_base_addr + BLK_NUM_OFF,
				data,
				ARRAY_SIZE(data));
		if (ret < 0) 
		{
			pr_err( "%s : Could not write bootloader id to 0x%x\n",
		       __func__, data_base_addr + BLK_NUM_OFF);
			return ret;
		}

		/* Issue the Erase config command to the device. */
		ret = rmi_write(fc->rmi_dev,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF, ERASE_CONFIG);
		if (ret < 0) 
		{
			pr_err("%s: Could not write command 0x%02x "
				"to 0x%04x\n", __func__, instance_data->cmd,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF);
			return ret;
		}	

		rmi4_wait_attn(fc,300);

		/*check status*/
		ret = rmi_read(fc->rmi_dev,
			  	data_base_addr + instance_data->blocksize +
			  	BLK_NUM_OFF, &status);
		if ((ret < 0) || (status != 0x80)) 
		{
			dev_err(&fc->dev, "Could not read status from 0x%x\n",
		       	data_base_addr + instance_data->blocksize + BLK_NUM_OFF);
			/* failure */
			return -1;
		}
	
		if(rmi4_write_image(fc, WRITE_CONFIG_BLOCK,pgm_data) < 0)
		{
			pr_err("write configure image error\n");
			return -1;
		}
	}
	
	/*check status*/
	ret = rmi_read(fc->rmi_dev,
			  	data_base_addr + instance_data->blocksize +
			  	BLK_NUM_OFF, &status);

	if ((ret < 0) || (status != 0x80)) 
	{
		dev_err(&fc->dev, "Could not read status from 0x%x\n",
		       	data_base_addr + instance_data->blocksize + BLK_NUM_OFF);
		/* failure */
		return -1;
	}
	
	return 0;
}

static int rmi4_write_image(struct rmi_function_container *fc,
							unsigned char type_cmd,const unsigned char *pgm_data)
{
	struct rmi_fn_34_data *instance_data;
	struct rmi_driver_data *driver_data;
	struct rmi_device *rmi_dev;
	unsigned short block_size;
	unsigned short img_blocks;
	unsigned short block_index;
	const unsigned char * p_data;
	int i;
	unsigned char data[2];
	u16 data_base_addr;
	int ret = -1;

	instance_data = fc->data;
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);

	data_base_addr = fc->fd.data_base_addr;
	block_size = instance_data->blocksize;
	
	switch (type_cmd)
	{
		case WRITE_FW_BLOCK:
			/* UI Firmware block count */
			img_blocks = instance_data->imageblockcount;
			break;
		case WRITE_CONFIG_BLOCK:
			/* read Configuration Block Count 0 */
			img_blocks = instance_data->configblockcount;	
			break;
		default:
			pr_err("image type error\n");
			return -1;
	}

	p_data = pgm_data;
	//printk("img_blocks:%d,block_size:%d\n",img_blocks,block_size);
	for(block_index = 0; block_index < img_blocks; ++block_index)
	{
		printk("#");
		// Write Block Number
		hstoba(data, block_index);
		ret = rmi_write_block(fc->rmi_dev,
				data_base_addr,
				data,ARRAY_SIZE(data));
		if (ret < 0) 
		{
			dev_err(&fc->dev, "Could not write to 0x%x\n",
		       		data_base_addr);
			/* failure */
			return -1;
		}
		
		for(i=0;i<block_size;i++)
		{
			ret = rmi_write(fc->rmi_dev,
					data_base_addr + BLK_NUM_OFF + i,
					*(p_data+i));
			if (ret < 0) 
			{
				dev_err(&fc->dev, "Could not write to 0x%x\n",
		       			data_base_addr+ BLK_NUM_OFF + i);
				/* failure */
				return -1;
			}
			
			udelay(15);
		}
		
		p_data += block_size;	
		
		// Issue Write Firmware or configuration Block command
		ret = rmi_write(fc->rmi_dev,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF, type_cmd);

		if (ret < 0) 
		{
			dev_err(&fc->dev, "Could not write to 0x%x\n",
		       		data_base_addr+ BLK_NUM_OFF + i);
			/* failure */
			return -1;
		}
		
		// Wait ATTN. Read Flash Command register and check error
		if(rmi4_wait_attn(fc,5) != 0)
		{
			return -1;
		}
	}

	return 0;
}
static int rmi4_disable_program(struct rmi_function_container *fc)
{
	struct rmi_fn_34_data *instance_data;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *driver_data;
	unsigned char cdata; 
	unsigned int loop_count=0;
	struct rmi_function_container *f01_fc; 
  
	printk("RMI4 disable program...\n");

	instance_data = fc->data;
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	f01_fc = driver_data->f01_container;
	
	// Issue a reset command
	rmi_write(fc->rmi_dev,
				f01_fc->fd.command_base_addr, 0x01);

	// Wait for ATTN to be asserted to see if device is in idle state
	rmi4_wait_attn(fc,20);

	// Read F01 Status flash prog, ensure the 6th bit is '0'
	do
	{
		rmi_read(fc->rmi_dev,
			  	f01_fc->fd.data_base_addr
			  	, &cdata);
		udelay(2);
	} while(((cdata & 0x40) != 0) && (loop_count++ < 10));

	//Rescan the Page Description Table
	return rmi4_read_PDT(fc);
}

u8 get_fn34_module_id()
{
	return touch_module_id;
}

u32 get_fn34_config_ver()
{
	return touch_configid;
}


static void get_module_id(struct rmi_function_container *fc)
{
	struct rmi_driver_data *driver_data;
	struct rmi_function_container *f01_fc; 
	struct rmi_device *rmi_dev;
	int retval = 0;
	
	rmi4_enable_program(fc);
					
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);	
	f01_fc = driver_data->f01_container;
	
	retval = rmi_read_block(fc->rmi_dev, f01_fc->fd.query_base_addr + RMI_PRODUCT_ID_LENGTH + 1,
			(unsigned char *)productid, RMI_PRODUCT_ID_LENGTH);

	if (retval < 0) {
		pr_err("%s : Could not read data from 0x%04x\n",
		       __func__, f01_fc->fd.query_base_addr + RMI_PRODUCT_ID_LENGTH + 1);
	}
	productid[RMI_PRODUCT_ID_LENGTH] = '\0';

	retval = kstrtou8(&productid[RMI_PRODUCT_ID_LENGTH-1], 10, &touch_module_id);
	pr_info("touch_module_id:%d\n",touch_module_id);
	if (retval)
		pr_err("%s : transfer error\n",__func__);
	
	rmi4_disable_program(fc);
}

static int config_array_is_latest(struct rmi_function_container *fc,
					unsigned char * pconfigarray)
{
    unsigned int configid = 0;
	struct  rmi_fn_34_data  *instance_data = fc->data;
	int retval = 0;
	unsigned char pdata[5];

	memcpy(pdata,pconfigarray,4);
	pdata[4] = '\0';

	retval = kstrtou32(pdata, 10, &configid);
	printk("configid:%d,  %d\n",configid,instance_data->configid);
	if (configid <= instance_data->configid)
		return 1; //latest performance, don't need update

	return 0;
}

static int rmi_f34_config(struct rmi_function_container *fc)
{
	/* for this function we should do nothing here */
	return 0;
}


static int rmi_f34_reset(struct rmi_function_container *fc)
{
	struct  rmi_fn_34_data  *instance_data = fc->data;

	instance_data->status = ECONNRESET;

	return 0;
}

static void rmi_f34_remove(struct rmi_function_container *fc)
{
	int attr_count;

	sysfs_remove_bin_file(&fc->dev.kobj,
						  &dev_attr_data);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
		sysfs_remove_file(&fc->dev.kobj,
				  &attrs[attr_count].attr);

	rmi_f34_free_memory(fc);
}

static int f34_read_status(struct rmi_function_container *fc)
{
	struct rmi_fn_34_data *instance_data = fc->data;
	u16 data_base_addr = fc->fd.data_base_addr;
	u8 status;
	int retval;

	if (instance_data->status == ECONNRESET)
		return instance_data->status;

	/* Read the Fn $34 status from F34_Flash_Data3 to see the previous
	 * commands status. F34_Flash_Data3 will be the address after the
	 * 2 block number registers plus blocksize Data registers.
	 *  inform user space - through a sysfs param. */
	retval = rmi_read(fc->rmi_dev,
			  data_base_addr + instance_data->blocksize +
			  BLK_NUM_OFF, &status);

	if (retval < 0) {
		dev_err(&fc->dev, "Could not read status from 0x%x\n",
		       data_base_addr + instance_data->blocksize + BLK_NUM_OFF);
		status = 0xff;	/* failure */
	}

	/* set a sysfs value that the user mode can read - only
	 * upper 4 bits are the status. successful is $80, anything
	 * else is failure */
	instance_data->status = status & 0xf0;

	/* put mode into Flash Prog Mode when we successfully do
	 * an Enable Flash Prog cmd. */
	if ((instance_data->status == STATUS_IDLE) &&
		(instance_data->cmd == ENABLE_FLASH_PROG))
		instance_data->inflashprogmode = true;

	return retval;
}

int rmi_f34_attention(struct rmi_function_container *fc, u8 *irq_bits)
{
	int retval;
	struct rmi_fn_34_data *data = fc->data;

	mutex_lock(&data->attn_mutex);
	retval = f34_read_status(fc);
	mutex_unlock(&data->attn_mutex);
	return retval;
}

static struct rmi_function_handler function_handler = {
	.func = 0x34,
	.init = rmi_f34_init,
	.config = rmi_f34_config,
	.reset = rmi_f34_reset,
	.attention = rmi_f34_attention,
	.remove = rmi_f34_remove
};

static ssize_t rmi_fn_34_bootloaderid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	unsigned short bootloaderid;
	unsigned char data[2];
	int retval;

	fc = to_rmi_function_container(dev);
	retval = rmi_read_block(fc->rmi_dev, fc->fd.query_base_addr, data,
			ARRAY_SIZE(data));

	if (retval < 0) {
		dev_err(&fc->dev, "Could not read bootloaderid from 0x%04x.\n",
			fc->fd.query_base_addr);
		return retval;
	}

	batohs(&bootloaderid, data);
	
	return snprintf(buf, PAGE_SIZE, "%u\n", bootloaderid);
}

static ssize_t rmi_fn_34_bootloaderid_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	int error;
	unsigned long val;
	unsigned char data[2];
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	u16 data_base_addr;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);

	if (error)
		return error;

	instance_data->bootloaderid = val;

	/* Write the Bootloader ID key data back to the first two Block
	 * Data registers (F34_Flash_Data2.0 and F34_Flash_Data2.1). */
	hstoba(data, (unsigned short)val);
	data_base_addr = fc->fd.data_base_addr;

	error = rmi_write_block(fc->rmi_dev,
				data_base_addr + BLK_NUM_OFF,
				data,
				ARRAY_SIZE(data));

	if (error < 0) {
		dev_err(dev, "%s : Could not write bootloader id to 0x%x\n",
		       __func__, data_base_addr + BLK_NUM_OFF);
		return error;
	}

	return count;
}

static ssize_t rmi_fn_34_blocksize_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->blocksize);
}

static ssize_t rmi_fn_34_imageblockcount_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->imageblockcount);
}

static ssize_t rmi_fn_34_configblockcount_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->configblockcount);
}

static ssize_t rmi_fn_34_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	int retval;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	mutex_lock(&instance_data->attn_mutex);
	retval = f34_read_status(fc);
	mutex_unlock(&instance_data->attn_mutex);

	if (retval < 0)
		return retval;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->status);
}


static ssize_t rmi_fn_34_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	instance_data->status = 0;

	return 0;
}


static ssize_t rmi_fn_34_cmd_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	u8 cmd;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	/* Issue the command to the device. */
	rmi_read(fc->rmi_dev,
			fc->fd.data_base_addr + instance_data->blocksize +
			BLK_NUM_OFF, &cmd);

	return snprintf(buf, PAGE_SIZE, "%u\n", cmd);
}

static ssize_t rmi_fn_34_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	unsigned long val;
	u16 data_base_addr;
	int error;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	data_base_addr = fc->fd.data_base_addr;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	/* make sure we are in Flash Prog mode for all cmds except the
	 * Enable Flash Programming cmd - otherwise we are in error */
	if ((val != ENABLE_FLASH_PROG) && !instance_data->inflashprogmode) {
		dev_err(dev, "%s: CANNOT SEND CMD %d TO SENSOR - "
			"NOT IN FLASH PROG MODE\n"
			, __func__, data_base_addr);
		return -EINVAL;
	}

	instance_data->cmd = val;

	/* Validate command value and (if necessary) write it to the command
	 * register.
	 */
	switch (instance_data->cmd) {
	case ENABLE_FLASH_PROG:
	case ERASE_ALL:
	case ERASE_CONFIG:
	case WRITE_FW_BLOCK:
	case READ_CONFIG_BLOCK:
	case WRITE_CONFIG_BLOCK:
		/* Reset the status to indicate we are in progress on a cmd. */
		/* The status will change when the ATTN interrupt happens
		   and the status of the cmd that was issued is read from
		   the F34_Flash_Data3 register - result should be 0x80 for
		   success - any other value indicates an error */

		/* Issue the command to the device. */
		error = rmi_write(fc->rmi_dev,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF, instance_data->cmd);

		if (error < 0) {
			dev_err(dev, "%s: Could not write command 0x%02x "
				"to 0x%04x\n", __func__, instance_data->cmd,
				data_base_addr + instance_data->blocksize +
				BLK_NUM_OFF);
			return error;
		}

		if (instance_data->cmd == ENABLE_FLASH_PROG)
			instance_data->inflashprogmode = true;

		/* set status to indicate we are in progress */
		instance_data->status = STATUS_IN_PROGRESS;
		break;
	default:
		dev_dbg(dev, "%s: RMI4 function $34 - "
				"unknown command 0x%02lx.\n", __func__, val);
		count = -EINVAL;
		break;
	}

	return count;
}

static ssize_t rmi_fn_34_blocknum_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	unsigned short blocknum;
	unsigned char data[2];
	int retval;

	fc = to_rmi_function_container(dev);
	retval = rmi_read_block(fc->rmi_dev, fc->fd.data_base_addr, data,
			ARRAY_SIZE(data));

	if (retval < 0) {
		dev_err(&fc->dev, "Could not read bootloaderid from 0x%04x.\n",
			fc->fd.data_base_addr);
		return retval;
	}

	batohs(&blocknum, data);

	return snprintf(buf, PAGE_SIZE, "%u\n", blocknum);
}

static ssize_t rmi_fn_34_blocknum_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	int error;
	unsigned long val;
	unsigned char data[2];
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	u16 data_base_addr;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	data_base_addr = fc->fd.data_base_addr;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);

	if (error)
		return error;

	instance_data->blocknum = val;

	/* Write the Block Number data back to the first two Block
	 * Data registers (F34_Flash_Data_0 and F34_Flash_Data_1). */
	hstoba(data, (unsigned short)val);

	error = rmi_write_block(fc->rmi_dev,
				data_base_addr,
				data,
				ARRAY_SIZE(data));

	if (error < 0) {
		dev_err(dev, "%s : Could not write block number %u to 0x%x\n",
		       __func__, instance_data->blocknum, data_base_addr);
		return error;
	}

	return count;
}

static ssize_t rmi_fn_34_rescanPDT_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *driver_data;
	struct pdt_entry pdt_entry;
	bool fn01found = false;
	bool fn34found = false;
	unsigned int rescan;
	int irq_count = 0;
	int retval = 0;
	int i;

	/* Rescan of the PDT is needed since issuing the Flash Enable cmd
	 * the device registers for Fn$01 and Fn$34 moving around because
	 * of the change from Bootloader mode to Flash Programming mode
	 * may change to a different PDT with only Fn$01 and Fn$34 that
	 * could have addresses for query, control, data, command registers
	 * that differ from the PDT scan done at device initialization. */

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);

	/* Make sure we are only in Flash Programming mode  - DON'T
	 * ALLOW THIS IN UI MODE. */
	if (instance_data->cmd != ENABLE_FLASH_PROG) {
		dev_err(dev, "%s: NOT IN FLASH PROG MODE - CAN'T RESCAN PDT.\n"
				, __func__);
		return -EINVAL;
	}

	/* The only good value to write to this is 1, we allow 0, but with
	 * no effect (this is consistent with the way the command bit works. */
	if (sscanf(buf, "%u", &rescan) != 1)
		return -EINVAL;
	if (rescan < 0 || rescan > 1)
		return -EINVAL;

	/* 0 has no effect, so we skip it entirely. */
	if (rescan) {
		/* rescan the PDT - filling in Fn01 and Fn34 addresses -
		 * this is only temporary - the device will need to be reset
		 * to return the PDT to the normal values. */

		/* mini-parse the PDT - we only have to get Fn$01 and Fn$34 and
		   since we are Flash Programming mode we only have page 0. */
		for (i = PDT_START_SCAN_LOCATION; i >= PDT_END_SCAN_LOCATION;
			i -= sizeof(pdt_entry)) {
			retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					       sizeof(pdt_entry));
			if (retval != sizeof(pdt_entry)) {
				dev_err(dev, "%s: err frm rmi_read_block pdt "
					"entry data from PDT, "
					"error = %d.", __func__, retval);
				return retval;
			}

			if ((pdt_entry.function_number == 0x00) ||
				(pdt_entry.function_number == 0xff))
				break;

			dev_dbg(dev, "%s: Found F%.2X\n",
				__func__, pdt_entry.function_number);

			/* f01 found - just fill in the new addresses in
			 * the existing fc. */
			if (pdt_entry.function_number == 0x01) {
				struct rmi_function_container *f01_fc =
					driver_data->f01_container;
				fn01found = true;
				f01_fc->fd.query_base_addr =
					pdt_entry.query_base_addr;
				f01_fc->fd.command_base_addr =
				  pdt_entry.command_base_addr;
				f01_fc->fd.control_base_addr =
				  pdt_entry.control_base_addr;
				f01_fc->fd.data_base_addr =
				  pdt_entry.data_base_addr;
				f01_fc->fd.function_number =
				  pdt_entry.function_number;
				f01_fc->fd.interrupt_source_count =
				  pdt_entry.interrupt_source_count;
				f01_fc->num_of_irqs =
				  pdt_entry.interrupt_source_count;
				f01_fc->irq_pos = irq_count;

				irq_count += f01_fc->num_of_irqs;

				if (fn34found)
					break;
			}

			/* f34 found - just fill in the new addresses in
			 * the existing fc. */
			if (pdt_entry.function_number == 0x34) {
				fn34found = true;
				fc->fd.query_base_addr =
				  pdt_entry.query_base_addr;
				fc->fd.command_base_addr =
				  pdt_entry.command_base_addr;
				fc->fd.control_base_addr =
				  pdt_entry.control_base_addr;
				fc->fd.data_base_addr =
				  pdt_entry.data_base_addr;
				fc->fd.function_number =
				  pdt_entry.function_number;
				fc->fd.interrupt_source_count =
				  pdt_entry.interrupt_source_count;
				fc->num_of_irqs =
				  pdt_entry.interrupt_source_count;
				fc->irq_pos = irq_count;

				irq_count += fc->num_of_irqs;

				if (fn01found)
					break;
			}

		}

		if (!fn01found || !fn34found) {
			dev_err(dev, "%s: failed to find fn$01 or fn$34 trying "
				"to do rescan PDT.\n"
				, __func__);
			return -EINVAL;
		}
	}

	return count;
}

static ssize_t rmi_fn_34_moduleID_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", productid);
}

static ssize_t rmi_fn_34_configID_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n", instance_data->configid);
}

#ifdef KERNEL_VERSION_ABOVE_2_6_32
static ssize_t rmi_fn_34_data_read(struct file *data_file,
				struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
#else
static ssize_t rmi_fn_34_data_read(struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
#endif
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	u16 data_base_addr;
	int error;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	data_base_addr = fc->fd.data_base_addr;

	if (count != instance_data->blocksize) {
		dev_err(dev,
			"%s : Incorrect F34 block size %d. "
			"Expected size %d.\n",
			__func__, count, instance_data->blocksize);
		return -EINVAL;
	}

	/* Read the data from flash into buf.  The app layer will be blocked
	 * at reading from the sysfs file.  When we return the count (or
	 * error if we fail) the app will resume. */
	error = rmi_read_block(fc->rmi_dev, data_base_addr + BLK_NUM_OFF,
			(unsigned char *)buf, count);

	if (error < 0) {
		dev_err(dev, "%s : Could not read data from 0x%04x\n",
		       __func__, data_base_addr + BLK_NUM_OFF);
		return error;
	}

	return count;
}

#ifdef KERNEL_VERSION_ABOVE_2_6_32
static ssize_t rmi_fn_34_data_write(struct file *data_file,
				struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
#else
static ssize_t rmi_fn_34_data_write(struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
#endif
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rmi_function_container *fc;
	struct rmi_fn_34_data *instance_data;
	u16 data_base_addr;
	int error;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	data_base_addr = fc->fd.data_base_addr;

	/* Write the data from buf to flash. The app layer will be
	 * blocked at writing to the sysfs file.  When we return the
	 * count (or error if we fail) the app will resume. */

	if (count != instance_data->blocksize) {
		dev_err(dev,
			"%s : Incorrect F34 block size %d. "
			"Expected size %d.\n",
			__func__, count, instance_data->blocksize);
		return -EINVAL;
	}

	/* Write the data block - only if the count is non-zero  */
	if (count) {
		error = rmi_write_block(fc->rmi_dev,
				data_base_addr + BLK_NUM_OFF,
				(unsigned char *)buf,
				count);

		if (error < 0) {
			dev_err(dev, "%s : Could not write block data "
				"to 0x%x\n", __func__,
				data_base_addr + BLK_NUM_OFF);
			return error;
		}
	}

	return count;
}

static int __init rmi_f34_module_init(void)
{
	int error;

	error = rmi_register_function_driver(&function_handler);
	if (error < 0) {
		pr_err("%s : register failed !\n", __func__);
		return error;
	}

	return 0;
}

static void rmi_f34_module_exit(void)
{
	rmi_unregister_function_driver(&function_handler);
}

module_init(rmi_f34_module_init);
module_exit(rmi_f34_module_exit);

MODULE_AUTHOR("William Manson <wmanson@synaptics.com");
MODULE_DESCRIPTION("RMI F34 module");
MODULE_LICENSE("GPL");

