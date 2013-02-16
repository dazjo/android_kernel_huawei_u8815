/*
 * include/linux/synaptics_i2c_rmi.h
 *
 * Copyright (C) 2008 Google, Inc.
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

#ifndef _SYNPTICS_I2C_RMI_1564_H
#define _SYNPTICS_I2C_RMI_1564_H

struct rmi_function_info
{
	/* This is the number of data points supported - for example, for
	  * function $11 (2D sensor) the number of data points is equal to
	  * the number of fingers - for function $19 (buttons) it is equal to
	  * the number of buttons
	  */
	uint8_t points_supported;

	/* This is the interrupt register and mask - needed for enabling the
	  * interrupts and for checking what source had caused the attention
	  * line interrupt
	  */
	uint8_t interrupt_offset;
	uint8_t interrupt_mask;
	
	uint8_t data_offset;
	uint8_t data_length;
};

enum f11_finger_status
{
	f11_finger_none = 0,
	f11_finger_accurate = 1,
	f11_finger_inaccurate = 2
};

struct f11_finger_data
{
	enum f11_finger_status status;
	bool active;
	uint16_t x;
	uint16_t y;
	uint16_t z;
	uint32_t speed;	
};

struct synaptics_rmi4
{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct hrtimer timer;
	struct work_struct work;
	struct early_suspend early_suspend;
	struct i2c_msg data_i2c_msg[2];
	struct rmi_function_info f01;
	struct rmi_function_info f11;
	struct f11_finger_data *f11_fingers;
	struct rmi_function_info f19;
	struct rmi_function_info f30;
#ifdef CONFIG_HUAWEI_TOUCHSCREEN_EXTRA_KEY
	struct input_dev *key_input;
#endif
	
	uint8_t use_irq;
	uint8_t data_reg;
	uint8_t data_length;
	uint8_t *data;
	bool has_f11;
	uint8_t f11_has_gestures;
	uint8_t f11_has_relative;
	int f11_max_x;
	int f11_max_y;
	uint8_t *f11_egr;
	bool has_egr_pinch;
	bool has_egr_press;
	bool has_egr_flick;
	bool has_egr_early_tap;
	bool has_egr_double_tap;
	bool has_egr_tap_and_hold;
	bool has_egr_single_tap;
	bool has_egr_palm_detect;
	bool f11_has_sensitivity_adjust;
	bool is_support_multi_touch;
	bool has_f19;
	bool has_f30;
	bool enable;
};

#endif /* _SYNPTICS_I2C_RMI_1564_H */
