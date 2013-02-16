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
#include <linux/rmi.h>
#include <linux/input.h>
#include <linux/slab.h>

#define QUERY_BASE_INDEX 1
#define MAX_LEN 256

struct f1a_0d_query {
	union {
		struct {
			u8 max_button_count:3;
		};
		u8 f1a_0d_query0;
	};
	union {
		struct {
			u8 has_general_control:1;
			u8 has_interrupt_enable:1;
			u8 has_multibutton_select:1;
			u8 has_tx_rx_map:1;
			u8 has_perbutton_threshold:1;
			u8 has_release_threshold:1;
			u8 has_strongestbtn_hysteresis:1;
			u8 has_filter_strength:1;
		};
		u8 f1a_0d_query1;
	};
};

struct f1a_0d_control_0 {
	union {
		struct {
			u8 multibutton_report:2;
			u8 filter_mode:2;
		};
		u8 f1a_0d_control0;
	};
};

struct f1a_0d_control_3_4 {
	u8 transmitterbutton;
	u8 receiverbutton;
};


struct f1a_0d_control {
	struct f1a_0d_control_0 general_control;
	u8 *button_int_enable;
	u8 *multi_button;
	struct f1a_0d_control_3_4 *electrode_map;
	u8 *button_threshold;
	u8 button_release_threshold;
	u8 strongest_button_hysteresis;
	u8 filter_strength;
};
/* data specific to fn $1a that needs to be kept around */
struct f1a_data {
	struct f1a_0d_control button_control;
	struct f1a_0d_query button_query;
	bool *button_down;
	unsigned char button_count;
	unsigned char button_register_count;
	unsigned char *button_data_buffer;
	unsigned char *button_map;
	char input_name[MAX_LEN];
	char input_phys[MAX_LEN];
	struct input_dev *input;
	int general_control_address;
	int button_int_enable_address;
	int multi_button_address;
	int electrode_map_address;
	int button_threshold_address;
	int button_release_threshold_address;
	int strongest_button_hysteresis_address;
	int filter_strength_address;
};

static ssize_t rmi_f1a_button_count_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);
static ssize_t rmi_f1a_button_map_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t rmi_f1a_button_map_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t rmi_f1a_has_general_control_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t rmi_f1a_has_interrupt_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t rmi_f1a_has_multibutton_select_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t rmi_f1a_has_tx_rx_map_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t rmi_f1a_has_perbutton_threshold_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t rmi_f1a_has_release_threshold_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t rmi_f1a_has_strongestbtn_hysteresis_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t rmi_f1a_has_filter_strength_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t rmi_f1a_multibutton_report_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t rmi_f1a_multibutton_report_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t rmi_f1a_filter_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);
static ssize_t rmi_f1a_filter_mode_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count);
static ssize_t rmi_f1a_button_int_enable_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf);
static ssize_t rmi_f1a_button_int_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count);
static ssize_t rmi_f1a_multibutton_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf);
static ssize_t rmi_f1a_multibutton_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count);
static ssize_t rmi_f1a_transmitter_button_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf);
static ssize_t rmi_f1a_transmitter_button_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count);
static ssize_t rmi_f1a_receiver_button_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf);
static ssize_t rmi_f1a_receiver_button_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count);
static ssize_t rmi_f1a_threshold_button_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf);
static ssize_t rmi_f1a_threshold_button_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count);
static ssize_t rmi_f1a_button_release_threshold_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf);
static ssize_t rmi_f1a_button_release_threshold_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count);
static ssize_t rmi_f1a_strongest_button_hysteresis_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf);
static ssize_t rmi_f1a_strongest_button_hysteresis_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count);
static ssize_t rmi_f1a_filter_strength_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf);
static ssize_t rmi_f1a_filter_strength_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count);

static int rmi_f1a_alloc_memory(struct rmi_function_container *fc);

static void rmi_f1a_free_memory(struct rmi_function_container *fc);

static int rmi_f1a_initialize(struct rmi_function_container *fc);

static int rmi_f1a_register_device(struct rmi_function_container *fc);

static int rmi_f1a_create_sysfs(struct rmi_function_container *fc);

static int rmi_f1a_config(struct rmi_function_container *fc);

static int rmi_f1a_reset(struct rmi_function_container *fc);

static struct device_attribute attrs[] = {
	__ATTR(button_count, RMI_RO_ATTR,
		rmi_f1a_button_count_show, rmi_store_error),
	__ATTR(button_map, RMI_RW_ATTR,
		rmi_f1a_button_map_show, rmi_f1a_button_map_store),
	__ATTR(has_general_control, RMI_RO_ATTR,
		rmi_f1a_has_general_control_show, rmi_store_error),
	__ATTR(has_interrupt_enable, RMI_RO_ATTR,
		rmi_f1a_has_interrupt_enable_show, rmi_store_error),
	__ATTR(has_multibutton_select, RMI_RO_ATTR,
		rmi_f1a_has_multibutton_select_show, rmi_store_error),
	__ATTR(has_tx_rx_map, RMI_RO_ATTR,
		rmi_f1a_has_tx_rx_map_show, rmi_store_error),
	__ATTR(has_perbutton_threshold, RMI_RO_ATTR,
		rmi_f1a_has_perbutton_threshold_show, rmi_store_error),
	__ATTR(has_release_threshold, RMI_RO_ATTR,
		rmi_f1a_has_release_threshold_show, rmi_store_error),
	__ATTR(has_strongestbtn_hysteresis, RMI_RO_ATTR,
		rmi_f1a_has_strongestbtn_hysteresis_show, rmi_store_error),
	__ATTR(has_filter_strength, RMI_RO_ATTR,
		rmi_f1a_has_filter_strength_show, rmi_store_error),
	__ATTR(multibutton_report, RMI_RW_ATTR,
		rmi_f1a_multibutton_report_show,
		rmi_f1a_multibutton_report_store),
	__ATTR(filter_mode, RMI_RW_ATTR,
		rmi_f1a_filter_mode_show, rmi_f1a_filter_mode_store),
	__ATTR(button_int_enable, RMI_RW_ATTR,
		rmi_f1a_button_int_enable_show,
		rmi_f1a_button_int_enable_store),
	__ATTR(multibutton, RMI_RW_ATTR,
		rmi_f1a_multibutton_show, rmi_f1a_multibutton_store),
	__ATTR(transmitter_button, RMI_RW_ATTR,
		rmi_f1a_transmitter_button_show,
		rmi_f1a_transmitter_button_store),
	__ATTR(receiver_button, RMI_RW_ATTR,
		rmi_f1a_receiver_button_show,
		rmi_f1a_receiver_button_store),
	__ATTR(threshold_button, RMI_RW_ATTR,
		rmi_f1a_threshold_button_show,
		rmi_f1a_threshold_button_store),
	__ATTR(button_release_threshold, RMI_RW_ATTR,
		rmi_f1a_button_release_threshold_show,
		rmi_f1a_button_release_threshold_store),
	__ATTR(strongest_button_hysteresis, RMI_RW_ATTR,
		rmi_f1a_strongest_button_hysteresis_show,
		rmi_f1a_strongest_button_hysteresis_store),
	__ATTR(filter_strength, RMI_RW_ATTR,
		rmi_f1a_filter_strength_show,
		rmi_f1a_filter_strength_store)
};

int rmi_f1a_read_control_parameters(struct rmi_device *rmi_dev,
	struct f1a_data *f1a)
{
	int error = 0;
	struct f1a_0d_control *button_control = &f1a->button_control;
	int ctrl_base_addr = f1a->general_control_address;

	error = rmi_read_block(rmi_dev, ctrl_base_addr,
			(u8 *)&button_control->general_control,
			sizeof(struct f1a_0d_control_0));
	if (error < 0) {
		dev_err(&rmi_dev->dev,
			"Failed to read f1a_0d_control_0, code:"
			" %d.\n", error);
		return error;
	}
	ctrl_base_addr = ctrl_base_addr +
			sizeof(struct f1a_0d_control_0);

	if (button_control->button_int_enable) {
		f1a->button_int_enable_address = ctrl_base_addr;
		error = rmi_read_block(rmi_dev, ctrl_base_addr,
			button_control->button_int_enable,
			(sizeof(u8)*f1a->button_register_count));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read f1a_0d_control_1,"
				" code: %d.\n", error);
			return error;
		}
		ctrl_base_addr = ctrl_base_addr +
			(sizeof(u8)*f1a->button_register_count);
	}

	if (button_control->multi_button) {
		f1a->multi_button_address = ctrl_base_addr;
		error = rmi_read_block(rmi_dev, ctrl_base_addr,
				button_control->multi_button,
				sizeof(u8)*f1a->button_register_count);
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read f1a_0d_control_2,"
				" code: %d.\n", error);
			return error;
		}
		ctrl_base_addr = ctrl_base_addr +
			(sizeof(u8)*f1a->button_register_count);
	}

	if (button_control->electrode_map) {
		f1a->electrode_map_address = ctrl_base_addr;
		error = rmi_read_block(rmi_dev, ctrl_base_addr,
			(u8 *)(button_control->electrode_map),
			sizeof(struct f1a_0d_control_3_4)*f1a->button_count);
		if (error < 0) {
			dev_err(&rmi_dev->dev,
			"Failed to read f1a_0d_control_3_4,"
			" code: %d.\n", error);
			return error;
		}
		ctrl_base_addr = ctrl_base_addr +
			(sizeof(struct f1a_0d_control_3_4)*f1a->button_count);
	}
	if (button_control->button_threshold) {
		f1a->button_threshold_address = ctrl_base_addr;
		error = rmi_read_block(rmi_dev, ctrl_base_addr,
			(u8 *)(button_control->button_threshold),
			sizeof(u8)*f1a->button_count);
		if (error < 0) {
			dev_err(&rmi_dev->dev,
			"Failed to read f1a_0d_control_5,"
			" code: %d.\n", error);
			return error;
		}
		ctrl_base_addr = ctrl_base_addr +
			(sizeof(u8)*f1a->button_count);
	}

	f1a->button_release_threshold_address = ctrl_base_addr;
	error = rmi_read_block(rmi_dev, ctrl_base_addr,
			(u8 *)&button_control->button_release_threshold,
			sizeof(u8));
	if (error < 0) {
		dev_err(&rmi_dev->dev,
			"Failed to read f1a_0d_control_6,"
			" code: %d.\n", error);
		return error;
	}
	ctrl_base_addr = ctrl_base_addr + sizeof(u8);

	f1a->strongest_button_hysteresis_address = ctrl_base_addr;
	error = rmi_read_block(rmi_dev, ctrl_base_addr,
			(u8 *)&button_control->
				strongest_button_hysteresis,
			sizeof(u8));
	if (error < 0) {
		dev_err(&rmi_dev->dev,
			"Failed to read f1a_0d_control_7,"
			" code: %d.\n", error);
		return error;
	}
	ctrl_base_addr = ctrl_base_addr + sizeof(u8);

	f1a->filter_strength_address = ctrl_base_addr;
	error = rmi_read_block(rmi_dev, ctrl_base_addr,
			(u8 *)&button_control->filter_strength,
			sizeof(u8));
	if (error < 0) {
		dev_err(&rmi_dev->dev,
			"Failed to read f1a_0d_control_8,"
			" code: %d.\n", error);
		return error;
	}
	return 0;
}


static int rmi_f1a_init(struct rmi_function_container *fc)
{
	int rc;

	rc = rmi_f1a_alloc_memory(fc);
	if (rc < 0)
		goto err_free_data;

	rc = rmi_f1a_initialize(fc);
	if (rc < 0)
		goto err_free_data;

	rc = rmi_f1a_register_device(fc);
	if (rc < 0)
		goto err_free_data;

	rc = rmi_f1a_create_sysfs(fc);
	if (rc < 0)
		goto err_free_data;

	return 0;

err_free_data:
	rmi_f1a_free_memory(fc);

	return rc;
}


static int rmi_f1a_alloc_memory(struct rmi_function_container *fc)
{
	struct f1a_data *f1a;
	int rc;

	f1a = kzalloc(sizeof(struct f1a_data), GFP_KERNEL);
	if (!f1a) {
		dev_err(&fc->dev, "Failed to allocate function data.\n");
		return -ENOMEM;
	}
	fc->data = f1a;

	rc = rmi_read_block(fc->rmi_dev, fc->fd.query_base_addr,
			(u8 *)&f1a->button_query,
			sizeof(struct f1a_0d_query));
	if (rc < 0) {
		dev_err(&fc->dev, "Failed to read query register.\n");
		return rc;
	}

	f1a->button_count = f1a->button_query.max_button_count+1;

	f1a->button_down = kcalloc(f1a->button_count,
			sizeof(bool), GFP_KERNEL);
	if (!f1a->button_down) {
		dev_err(&fc->dev, "Failed to allocate button state buffer.\n");
		return -ENOMEM;
	}

	f1a->button_register_count = (f1a->button_count + 7) / 8;
	f1a->button_data_buffer =
	    kcalloc(f1a->button_register_count,
		    sizeof(unsigned char), GFP_KERNEL);
	if (!f1a->button_data_buffer) {
		dev_err(&fc->dev, "Failed to allocate button data buffer.\n");
		return -ENOMEM;
	}

	f1a->button_map = kcalloc(f1a->button_count,
				sizeof(unsigned char), GFP_KERNEL);
	if (!f1a->button_map) {
		dev_err(&fc->dev, "Failed to allocate button map.\n");
		return -ENOMEM;
	}

	f1a->button_control.button_int_enable =
		kzalloc(f1a->button_register_count*sizeof(u8), GFP_KERNEL);
	if (!f1a->button_control.button_int_enable) {
		dev_err(&fc->dev, "Failed to allocate interrupt button.\n");
		return -ENOMEM;
	}

	f1a->button_control.multi_button =
		kzalloc(f1a->button_register_count*sizeof(u8), GFP_KERNEL);
	if (!f1a->button_control.multi_button) {
		dev_err(&fc->dev, "Failed to allocate"
			" multi button group select.\n");
		return -ENOMEM;
	}

	f1a->button_control.electrode_map =
		kzalloc(f1a->button_count *
			sizeof(struct f1a_0d_control_3_4), GFP_KERNEL);
	if (!f1a->button_control.electrode_map) {
		dev_err(&fc->dev, "Failed to allocate"
			" f1a_0d_control_3_4.\n");
		return -ENOMEM;
	}

	f1a->button_control.button_threshold =
		kzalloc(f1a->button_count*sizeof(u8), GFP_KERNEL);
	if (!f1a->button_control.button_threshold) {
		dev_err(&fc->dev, "Failed to allocate button threshold.\n");
		return -ENOMEM;
	}
	return 0;
}



static void rmi_f1a_free_memory(struct rmi_function_container *fc)
{
	struct f1a_data *f1a = fc->data;

	if (f1a) {
		kfree(f1a->button_down);
		kfree(f1a->button_data_buffer);
		kfree(f1a->button_map);
		kfree(f1a->button_control.button_int_enable);
		kfree(f1a->button_control.multi_button);
		kfree(f1a->button_control.electrode_map);
		kfree(f1a->button_control.button_threshold);
		kfree(f1a);
		fc->data = NULL;
	}
}


static int rmi_f1a_initialize(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_device_platform_data *pdata;
	struct f1a_data *f1a = fc->data;
	int i;
	int rc;

	dev_info(&fc->dev, "Intializing F1a values.");

	/* initial all default values for f1a data here */
	pdata = to_rmi_platform_data(rmi_dev);
	if (pdata) {
		if (!pdata->f1a_button_map)
			dev_warn(&fc->dev, "%s - button_map is NULL", __func__);
		else if (pdata->f1a_button_map->nbuttons !=
				f1a->button_count)
			dev_warn(&fc->dev,
				"Platformdata button map size (%d) != number "
				"of buttons on device (%d) - ignored.\n",
				pdata->f1a_button_map->nbuttons,
				f1a->button_count);
		else if (!pdata->f1a_button_map->map)
			dev_warn(&fc->dev,
				 "Platformdata button map is missing!\n");
		else
			for (i = 0; i < pdata->f1a_button_map->nbuttons; i++)
				f1a->button_map[i] =
					pdata->f1a_button_map->map[i];
	}

	f1a->general_control_address = fc->fd.control_base_addr;
	rc = rmi_f1a_read_control_parameters(rmi_dev, f1a);
	if (rc < 0) {
		dev_err(&fc->dev,
			"Failed to initialize F1a control params.\n");
		return rc;
	}

	return 0;
}



static int rmi_f1a_register_device(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct input_dev *input_dev;
	struct f1a_data *f1a = fc->data;
	int i;
	int rc;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&fc->dev, "Failed to allocate input device.\n");
		return -ENOMEM;
	}

	f1a->input = input_dev;
	snprintf(f1a->input_name, MAX_LEN, "%sfn%02x", dev_name(&rmi_dev->dev),
		fc->fd.function_number);
	input_dev->name = f1a->input_name;
	snprintf(f1a->input_phys, MAX_LEN, "%s/input0", input_dev->name);
	input_dev->phys = f1a->input_phys;
	input_dev->dev.parent = &rmi_dev->dev;
	input_set_drvdata(input_dev, f1a);

	/* Set up any input events. */
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	/* set bits for each button... */
	for (i = 0; i < f1a->button_count; i++)
		set_bit(f1a->button_map[i], input_dev->keybit);
	rc = input_register_device(input_dev);
	if (rc < 0) {
		dev_err(&fc->dev, "Failed to register input device.\n");
		goto error_free_device;
	}

	return 0;

error_free_device:
	input_free_device(input_dev);

	return rc;
}


static int rmi_f1a_create_sysfs(struct rmi_function_container *fc)
{
	int attr_count = 0;
	int rc;

	dev_dbg(&fc->dev, "Creating sysfs files.\n");
	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&fc->dev.kobj, &attrs[attr_count].attr) < 0) {
			dev_err(&fc->dev,
				"Failed to create sysfs file for %s.",
				attrs[attr_count].attr.name);
			rc = -ENODEV;
			goto err_remove_sysfs;
		}
	}

	return 0;

err_remove_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&fc->dev.kobj,
						  &attrs[attr_count].attr);
	return rc;

}



static int rmi_f1a_config(struct rmi_function_container *fc)
{
	struct f1a_data *data;
	int retval;

	data = fc->data;

	retval = rmi_write_block(fc->rmi_dev, data->general_control_address,
		(u8 *)&data->button_control.general_control,
			sizeof(struct f1a_0d_control_0));
	if (retval < 0) {
		dev_err(&fc->dev, "%s : Could not write general_control to 0x%x\n",
				__func__, fc->fd.control_base_addr);
		return retval;
	}

	retval = rmi_write_block(fc->rmi_dev, data->button_int_enable_address,
		(u8 *)data->button_control.button_int_enable,
		sizeof(u8)*data->button_register_count);
	if (retval < 0) {
		dev_err(&fc->dev, "%s : Could not write interrupt_enable_store"
			" to 0x%x\n", __func__,
				data->button_int_enable_address);
		return retval;
	}

	retval = rmi_write_block(fc->rmi_dev, data->multi_button_address,
		(u8 *)data->button_control.multi_button,
		sizeof(u8)*data->button_register_count);
	if (retval < 0) {
		dev_err(&fc->dev,
				"%s : Could not write multi_button_store to"
				" 0x%x\n", __func__,
				data->multi_button_address);
		return -EINVAL;
	}

	retval = rmi_write_block(fc->rmi_dev,  data->electrode_map_address,
		(u8 *)data->button_control.electrode_map,
			sizeof(struct f1a_0d_control_3_4)*data->button_count);
	if (retval < 0) {
		dev_err(&fc->dev, "%s : Could not electrode_map_store to 0x%x\n",
				__func__, data->electrode_map_address);
		return -EINVAL;
	}

	retval = rmi_write_block(fc->rmi_dev,
		data->button_threshold_address,
		(u8 *)&data->button_control.button_threshold,
		sizeof(u8));
	if (retval < 0) {
		dev_err(&fc->dev, "%s : Could not button_threshold_store to"
			" 0x%x\n", __func__,
			data->button_threshold_address);
		return retval;
	}

	retval = rmi_write_block(fc->rmi_dev,
		data->button_release_threshold_address,
		(u8 *)&data->button_control.button_release_threshold,
		sizeof(u8));
	if (retval < 0) {
		dev_err(&fc->dev, "%s : Could not write "
			"button_release_threshold store to 0x%x\n",
			__func__, data->button_release_threshold_address);
		return -EINVAL;
	}

	retval = rmi_write_block(fc->rmi_dev,
		data->strongest_button_hysteresis_address,
		(u8 *)&data->button_control.strongest_button_hysteresis,
		sizeof(u8));
	if (retval < 0) {
		dev_err(&fc->dev, "%s : Could not write "
			"strongest_button_hysteresis store to 0x%x\n",
			__func__, data->strongest_button_hysteresis_address);
		return -EINVAL;
	}

	retval = rmi_write_block(fc->rmi_dev,
		data->filter_strength_address,
		(u8 *)&data->button_control.filter_strength,
		sizeof(u8));
	if (retval < 0) {
		dev_err(&fc->dev, "%s : Could not write filter_strength "
			"store to 0x%x\n", __func__,
			data->filter_strength_address);
		return -EINVAL;
	}
	return 0;
}


static int rmi_f1a_reset(struct rmi_function_container *fc)
{
	/* we do nnothing here */
	return 0;
}


static void rmi_f1a_remove(struct rmi_function_container *fc)
{
	struct f1a_data *f1a = fc->data;
	int attr_count = 0;

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
		sysfs_remove_file(&fc->dev.kobj,
				  &attrs[attr_count].attr);

	input_unregister_device(f1a->input);

	rmi_f1a_free_memory(fc);
}

int rmi_f1a_attention(struct rmi_function_container *fc, u8 *irq_bits)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f1a_data *f1a = fc->data;
	u8 data_base_addr = fc->fd.data_base_addr;
	int error;
	int button;

	/* Read the button data. */
	error = rmi_read_block(rmi_dev, data_base_addr, f1a->button_data_buffer,
			f1a->button_register_count);
	if (error < 0) {
		dev_err(&fc->dev, "%s: Failed to read button data registers.\n",
			__func__);
		return error;
	}

	/* Generate events for buttons that change state. */
	for (button = 0; button < f1a->button_count; button++) {
		int button_reg;
		int button_shift;
		bool button_status;

		/* determine which data byte the button status is in */
		button_reg = button / 8;
		/* bit shift to get button's status */
		button_shift = button % 8;
		button_status =
		    ((f1a->button_data_buffer[button_reg] >> button_shift)
			& 0x01) != 0;

		/* if the button state changed from the last time report it
		 * and store the new state */
		if (button_status != f1a->button_down[button]) {
			dev_dbg(&fc->dev, "%s: Button %d (code %d) -> %d.\n",
				__func__, button, f1a->button_map[button],
				 button_status);
			/* Generate an event here. */
			input_report_key(f1a->input, f1a->button_map[button],
					 button_status);
			f1a->button_down[button] = button_status;
		}
	}

	input_sync(f1a->input); /* sync after groups of events */
	return 0;
}

static struct rmi_function_handler function_handler = {
	.func = 0x1a,
	.init = rmi_f1a_init,
	.config = rmi_f1a_config,
	.reset = rmi_f1a_reset,
	.attention = rmi_f1a_attention,
	.remove = rmi_f1a_remove
};

static int __init rmi_f1a_module_init(void)
{
	int error;

	error = rmi_register_function_driver(&function_handler);
	if (error < 0) {
		pr_err("%s: register failed!\n", __func__);
		return error;
	}

	return 0;
}

static void rmi_f1a_module_exit(void)
{
	rmi_unregister_function_driver(&function_handler);
}

static ssize_t rmi_f1a_filter_mode_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_control.general_control.filter_mode);

}

static ssize_t rmi_f1a_filter_mode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	unsigned int new_value;
	int result;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	if (sscanf(buf, "%u", &new_value) != 1) {
		dev_err(dev,
		"%s: Error - filter_mode_store has an "
		"invalid len.\n",
		__func__);
		return -EINVAL;
	}

	if (new_value < 0 || new_value > 4) {
		dev_err(dev, "%s: Error - filter_mode_store has an "
		"invalid value %d.\n",
		__func__, new_value);
		return -EINVAL;
	}
	data->button_control.general_control.filter_mode = new_value;
	result = rmi_write_block(fc->rmi_dev, data->general_control_address,
		(u8 *)&(data->button_control.general_control),
			sizeof(struct f1a_0d_control_0));
	if (result < 0) {
		dev_err(dev, "%s : Could not write filter_mode_store to 0x%x\n",
				__func__, fc->fd.control_base_addr);
		return result;
	}

	return count;
}

static ssize_t rmi_f1a_multibutton_report_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
		data->button_control.general_control.multibutton_report);

}

static ssize_t rmi_f1a_multibutton_report_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	unsigned int new_value;
	int result;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	if (sscanf(buf, "%u", &new_value) != 1) {
		dev_err(dev,
		"%s: Error - multibutton_report_store has an "
		"invalid len.\n",
		__func__);
		return -EINVAL;
	}

	if (new_value < 0 || new_value > 4) {
		dev_err(dev, "%s: Error - multibutton_report_store has an "
		"invalid value %d.\n",
		__func__, new_value);
		return -EINVAL;
	}
	data->button_control.general_control.multibutton_report = new_value;
	result = rmi_write_block(fc->rmi_dev, data->general_control_address,
		(u8 *)&(data->button_control.general_control),
			sizeof(struct f1a_0d_control_0));
	if (result < 0) {
		dev_err(dev, "%s : Could not write multibutton_report_store to 0x%x\n",
				__func__, fc->fd.control_base_addr);
		return result;
	}

	return count;

}

static ssize_t rmi_f1a_button_int_enable_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	int i, len, total_len = 0;
	char *current_buf = buf;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	/* loop through each button map value and copy its
	 * string representation into buf */
	for (i = 0; i < data->button_count; i++) {
		int button_reg;
		int button_shift;
		int interrupt_button;

		button_reg = i / 8;
		button_shift = i % 8;
		interrupt_button =
		    ((data->button_control.button_int_enable[button_reg] >>
				button_shift) & 0x01);

		/* get next button mapping value and write it to buf */
		len = snprintf(current_buf, PAGE_SIZE - total_len,
			"%u ", interrupt_button);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(dev, "%s: Failed to build interrupt button"
				" buffer, code = %d.\n", __func__, len);
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	}
	len = snprintf(current_buf, PAGE_SIZE - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(dev, "%s: Failed to append carriage return.\n",
			 __func__);
	return total_len;

}

static ssize_t rmi_f1a_button_int_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	int i;
	int button_count = 0;
	int retval = count;
	int button_reg = 0;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	for (i = 0; i < data->button_count && *buf != 0;
	     i++, buf += 2) {
		int button_shift;
		int button;
		int result;

		button_reg = i / 8;
		button_shift = i % 8;
		/* get next button mapping value and store and bump up to
		 * point to next item in buf */
		result = sscanf(buf, "%u", &button);

		if ((result != 1) || (button != 0 && button != 1)) {
			dev_err(dev,
				"%s: Error - interrupt enable button for"
				" button %d is not a valid value 0x%x.\n",
				__func__, i, button);
			return -EINVAL;
		}
		if (button == 0) {
			data->button_control.button_int_enable[button_reg]
				&= (1 << button_shift) ^ 0xFF;
		} else
			data->button_control.button_int_enable[button_reg]
				|= (1 << button_shift);

		button_count++;
	}

	/* Make sure the button count matches */
	if (button_count != data->button_count) {
		dev_err(dev,
			"%s: Error - interrupt enable button count of %d"
			" doesn't match device button count of %d.\n",
			 __func__, button_count, data->button_count);
		return -EINVAL;
	}
	/* write back to the control register */
	retval = rmi_write_block(fc->rmi_dev,
			data->button_int_enable_address,
			(u8 *)data->button_control.button_int_enable,
			sizeof(u8)*(data->button_register_count));
	if (retval < 0) {
		dev_err(dev, "%s : Could not write interrupt_enable_store"
			" to 0x%x\n", __func__,
			data->button_int_enable_address);
		return retval;
	}

	return count;
}

static ssize_t rmi_f1a_multibutton_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	int i, len, total_len = 0;
	char *current_buf = buf;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	/* loop through each button map value and copy its
	 * string representation into buf */
	for (i = 0; i < data->button_count; i++) {
		int button_reg;
		int button_shift;
		int multibutton;

		button_reg = i / 8;
		button_shift = i % 8;
		multibutton = ((data->button_control.
			multi_button[button_reg] >> button_shift) & 0x01);

		/* get next button mapping value and write it to buf */
		len = snprintf(current_buf, PAGE_SIZE - total_len,
			"%u ", multibutton);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(dev, "%s: Failed to build multibutton buffer"
				", code = %d.\n", __func__, len);
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	}
	len = snprintf(current_buf, PAGE_SIZE - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(dev, "%s: Failed to append carriage return.\n",
			 __func__);

	return total_len;

}

static ssize_t rmi_f1a_multibutton_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	int i;
	int button_count = 0;
	int retval = count;
	int button_reg = 0;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	for (i = 0; i < data->button_count && *buf != 0;
	     i++, buf += 2) {
		int button_shift;
		int button;
		int result;

		button_reg = i / 8;
		button_shift = i % 8;
		/* get next button mapping value and store and bump up to
		 * point to next item in buf */
		result = sscanf(buf, "%u", &button);

		if ((result != 1) || (button != 0 && button != 1)) {
			dev_err(dev,
				"%s: Error - multibutton for button %d"
				" is not a valid value 0x%x.\n",
				__func__, i, button);
			return -EINVAL;
		}

		if (button == 0) {
			data->button_control.multi_button[button_reg] &=
				(1 << button_shift) ^ 0xFF;
		} else
			data->button_control.multi_button[button_reg] |=
				(1 << button_shift);

		button_count++;
	}

	/* Make sure the button count matches */
	if (button_count != data->button_count) {
		dev_err(dev,
		    "%s: Error - multibutton count of %d doesn't match"
		     " device button count of %d.\n", __func__, button_count,
		     data->button_count);
		return -EINVAL;
	}
	/* write back to the control register */
	retval = rmi_write_block(fc->rmi_dev, data->multi_button_address,
		(u8 *)data->button_control.multi_button,
			sizeof(u8)*data->button_register_count);
	if (retval < 0) {
		dev_err(dev, "%s : Could not write multibutton_store to"
			" 0x%x\n", __func__, data->multi_button_address);
		return -EINVAL;
	}
	return count;
}

static ssize_t rmi_f1a_transmitter_button_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	int i, len, total_len = 0;
	char *current_buf = buf;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	for (i = 0; i < data->button_count; i++) {
		len = snprintf(current_buf, PAGE_SIZE - total_len, "%u ",
			data->button_control.electrode_map[i].
			transmitterbutton);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(dev, "%s: Failed to build transmitter buffer, "
				"code = %d.\n", __func__, len);
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	}
	len = snprintf(current_buf, PAGE_SIZE - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(dev, "%s: Failed to append carriage return.\n",
			 __func__);
	return total_len;


}

static ssize_t rmi_f1a_transmitter_button_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	int transmitter_button;
	int i;
	int result;
	int retval = count;
	int button_count = 0;
	fc = to_rmi_function_container(dev);
	data = fc->data;

	for (i = 0; i < data->button_count && *buf != 0; i++, buf += 2) {
		/* get next button mapping value and store and bump up to
		 * point to next item in buf */
		result = sscanf(buf, "%u", &transmitter_button);

		/* Make sure the key is a valid key */
		if ((result != 1) ||
			(transmitter_button < 0 || transmitter_button > 127)) {
			dev_err(dev,
				"%s: Error - transmitter_button for button %d "
				"is not a valid value 0x%x.\n",
				__func__, i, transmitter_button);
			return -EINVAL;
		}
		data->button_control.electrode_map[i].transmitterbutton =
			transmitter_button;
		button_count++;
	}

	if (button_count != data->button_count) {
		dev_err(dev,
		    "%s: Error - button map count of %d doesn't match device "
		     "button count of %d.\n", __func__, button_count,
		     data->button_count);
		return -EINVAL;
	}
	/* write back to the control register */
	retval = rmi_write_block(fc->rmi_dev, data->electrode_map_address,
		(u8 *)data->button_control.electrode_map,
			sizeof(struct f1a_0d_control_3_4)*button_count);
	if (retval < 0) {
		dev_err(dev, "%s : Could not transmitter_btn_store to 0x%x\n",
				__func__,  data->electrode_map_address);
		return -EINVAL;
	}
	return count;
}

static ssize_t rmi_f1a_receiver_button_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	int i, len, total_len = 0;
	char *current_buf = buf;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	for (i = 0; i < data->button_count; i++) {
		len = snprintf(current_buf, PAGE_SIZE - total_len, "%u ",
			data->button_control.electrode_map[i].
			receiverbutton);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(dev, "%s: Failed to build receiver buffer, "
				"code = %d.\n", __func__, len);
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	}
	len = snprintf(current_buf, PAGE_SIZE - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(dev, "%s: Failed to append carriage return.\n",
			 __func__);
	return total_len;


}

static ssize_t rmi_f1a_receiver_button_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	int receiver_button;
	int i;
	int result;
	int retval = count;
	int button_count = 0;
	fc = to_rmi_function_container(dev);
	data = fc->data;

	for (i = 0; i < data->button_count && *buf != 0; i++, buf += 2) {
		/* get next button mapping value and store and bump up to
		 * point to next item in buf */
		result = sscanf(buf, "%u", &receiver_button);

		/* Make sure the key is a valid key */
		if ((result != 1) ||
			(receiver_button < 0 || receiver_button > 127)) {
			dev_err(dev,
				"%s: Error - receiver_button for button %d "
				"is not a valid value 0x%x.\n",
				__func__, i, receiver_button);
			return -EINVAL;
		}

		data->button_control.electrode_map[i].receiverbutton =
			receiver_button;
		button_count++;
	}

	if (button_count != data->button_count) {
		dev_err(dev,
		    "%s: Error - button map count of %d doesn't match device "
		     "button count of %d.\n", __func__, button_count,
		     data->button_count);
		return -EINVAL;
	}

	/* write back to the control register */
	retval = rmi_write_block(fc->rmi_dev, data->electrode_map_address,
		(u8 *)data->button_control.electrode_map,
			sizeof(struct f1a_0d_control_3_4)*button_count);
	if (retval < 0) {
		dev_err(dev, "%s : Could not receiver_btn_store to 0x%x\n",
				__func__,  data->electrode_map_address);
		return -EINVAL;
	}
	return count;
}

static ssize_t rmi_f1a_threshold_button_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	int i, len, total_len = 0;
	char *current_buf = buf;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	for (i = 0; i < data->button_count; i++) {
		len = snprintf(current_buf, PAGE_SIZE - total_len, "%u ",
			data->button_control.button_threshold[i]);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(dev, "%s: Failed to build threshold_button, "
				"code = %d.\n", __func__, len);
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	}
	len = snprintf(current_buf, PAGE_SIZE - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(dev, "%s: Failed to append carriage return.\n",
			 __func__);
	return total_len;


}

static ssize_t rmi_f1a_threshold_button_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	int threshold_button;
	int i;
	int result;
	int retval = count;
	int button_count = 0;
	fc = to_rmi_function_container(dev);
	data = fc->data;

	for (i = 0; i < data->button_count && *buf != 0; i++, buf += 2) {
		/* get next button mapping value and store and bump up to
		 * point to next item in buf */
		result = sscanf(buf, "%u", &threshold_button);

		/* Make sure the key is a valid key */
		if ((result != 1) ||
			(threshold_button < 0 || threshold_button > 127)) {
			dev_err(dev,
				"%s: Error - threshold_button for button %d "
				"is not a valid value 0x%x.\n",
				__func__, i, threshold_button);
			return -EINVAL;
		}

		data->button_control.button_threshold[i] = threshold_button;
		button_count++;
	}

	if (button_count != data->button_count) {
		dev_err(dev,
		    "%s: Error - button map count of %d doesn't match device "
		     "button count of %d.\n", __func__, button_count,
		     data->button_count);
		return -EINVAL;
	}

	/* write back to the control register */
	retval = rmi_write_block(fc->rmi_dev, data->button_threshold_address,
		(u8 *)data->button_control.button_threshold,
			sizeof(u8)*button_count);
	if (retval < 0) {
		dev_err(dev, "%s : Could not threshold_button_store to 0x%x\n",
				__func__,  data->button_threshold_address);
		return -EINVAL;
	}
	return count;
}


static ssize_t rmi_f1a_button_release_threshold_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n", data->button_control.
		button_release_threshold);

}
static ssize_t rmi_f1a_button_release_threshold_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	unsigned int new_value;
	int len;

	fc = to_rmi_function_container(dev);

	data = fc->data;

	len = sscanf(buf, "%u", &new_value);

	data->button_control.button_release_threshold = new_value;
	/* write back to the control register */
	len = rmi_write_block(fc->rmi_dev,
		data->button_release_threshold_address,
		(u8 *)&(data->button_control.button_release_threshold),
			sizeof(u8));
	if (len < 0) {
		dev_err(dev, "%s : Could not button_release_threshold_store to"
			" 0x%x\n", __func__,
			data->button_release_threshold_address);
		return len;
	}

	return len;
}

static ssize_t rmi_f1a_strongest_button_hysteresis_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n", data->button_control.
		strongest_button_hysteresis);

}
static ssize_t rmi_f1a_strongest_button_hysteresis_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	unsigned int new_value;
	int len;

	fc = to_rmi_function_container(dev);

	data = fc->data;

	len = sscanf(buf, "%u", &new_value);

	data->button_control.strongest_button_hysteresis = new_value;
	/* write back to the control register */
	len = rmi_write_block(fc->rmi_dev,
		data->strongest_button_hysteresis_address,
		(u8 *)&(data->button_control.strongest_button_hysteresis),
			sizeof(u8));
	if (len < 0) {
		dev_err(dev, "%s : Could not strongest_button_hysteresis_store"
			" to 0x%x\n", __func__,
			data->strongest_button_hysteresis_address);
		return len;
	}

	return len;
}

static ssize_t rmi_f1a_filter_strength_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n", data->button_control.
		filter_strength);

}

static ssize_t rmi_f1a_filter_strength_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	unsigned int new_value;
	int len;

	fc = to_rmi_function_container(dev);

	data = fc->data;

	len = sscanf(buf, "%u", &new_value);
	if (new_value < 0 || new_value > 31)
		return -EINVAL;

	data->button_control.filter_strength = new_value;
	/* write back to the control register */
	len = rmi_write_block(fc->rmi_dev,
		data->filter_strength_address,
		(u8 *)&(data->button_control.filter_strength),
			sizeof(u8));
	if (len < 0) {
		dev_err(dev, "%s : Could not filter_strength_store to"
			" 0x%x\n", __func__,
			data->filter_strength_address);
		return len;
	}

	return len;
}


static ssize_t rmi_f1a_button_count_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_query.max_button_count);
}

static ssize_t rmi_f1a_has_general_control_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_query.has_general_control);
}

static ssize_t rmi_f1a_has_interrupt_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_query.has_interrupt_enable);
}

static ssize_t rmi_f1a_has_multibutton_select_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_query.has_multibutton_select);
}

static ssize_t rmi_f1a_has_tx_rx_map_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_query.has_tx_rx_map);
}

static ssize_t rmi_f1a_has_perbutton_threshold_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_query.has_perbutton_threshold);
}

static ssize_t rmi_f1a_has_release_threshold_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_query.has_release_threshold);
}

static ssize_t rmi_f1a_has_strongestbtn_hysteresis_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_query.has_strongestbtn_hysteresis);
}

static ssize_t rmi_f1a_has_filter_strength_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_query.has_filter_strength);
}

static ssize_t rmi_f1a_button_map_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{

	struct rmi_function_container *fc;
	struct f1a_data *data;
	int i, len, total_len = 0;
	char *current_buf = buf;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	/* loop through each button map value and copy its
	 * string representation into buf */
	for (i = 0; i < data->button_count; i++) {
		/* get next button mapping value and write it to buf */
		len = snprintf(current_buf, PAGE_SIZE - total_len,
			"%u ", data->button_map[i]);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(dev, "%s: Failed to build button map buffer, "
				"code = %d.\n", __func__, len);
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	}
	len = snprintf(current_buf, PAGE_SIZE - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(dev, "%s: Failed to append carriage return.\n",
			 __func__);
	return total_len;
}

static ssize_t rmi_f1a_button_map_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct rmi_function_container *fc;
	struct f1a_data *data;
	unsigned int button;
	int i;
	int retval = count;
	int button_count = 0;
	unsigned char temp_button_map[KEY_MAX];

	fc = to_rmi_function_container(dev);
	data = fc->data;

	/* Do validation on the button map data passed in.  Store button
	 * mappings into a temp buffer and then verify button count and
	 * data prior to clearing out old button mappings and storing the
	 * new ones. */
	for (i = 0; i < data->button_count && *buf != 0;
	     i++) {
		/* get next button mapping value and store and bump up to
		 * point to next item in buf */
		sscanf(buf, "%u", &button);

		/* Make sure the key is a valid key */
		if (button > KEY_MAX) {
			dev_err(dev,
				"%s: Error - button map for button %d is not a"
				" valid value 0x%x.\n", __func__, i, button);
			retval = -EINVAL;
			goto err_ret;
		}

		temp_button_map[i] = button;
		button_count++;

		/* bump up buf to point to next item to read */
		while (*buf != 0) {
			buf++;
			if (*(buf - 1) == ' ')
				break;
		}
	}

	/* Make sure the button count matches */
	if (button_count != data->button_count) {
		dev_err(dev,
		    "%s: Error - button map count of %d doesn't match device "
		     "button count of %d.\n", __func__, button_count,
		     data->button_count);
		retval = -EINVAL;
		goto err_ret;
	}

	/* Clear the key bits for the old button map. */
	for (i = 0; i < button_count; i++)
		clear_bit(data->button_map[i], data->input->keybit);

	/* Switch to the new map. */
	memcpy(data->button_map, temp_button_map,
	       data->button_count);

	/* Loop through the key map and set the key bit for the new mapping. */
	for (i = 0; i < button_count; i++)
		set_bit(data->button_map[i], data->input->keybit);

err_ret:
	return retval;
}

module_init(rmi_f1a_module_init);
module_exit(rmi_f1a_module_exit);

MODULE_AUTHOR("Vivian Ly <vly@synaptics.com>");
MODULE_DESCRIPTION("RMI F1a module");
MODULE_LICENSE("GPL");

