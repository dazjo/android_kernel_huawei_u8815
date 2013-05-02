/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio_event.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/usbdiag.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_serial_hs.h>
#include <linux/usb/android.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/mmc.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/mfd/marimba.h>
#include <mach/vreg.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_battery.h>
#include <linux/smsc911x.h>
#include <linux/atmel_maxtouch.h>
#include "devices.h"
#include "timer.h"
#include "board-msm7x27a-regulator.h"
#include "devices-msm7x2xa.h"
#include "pm.h"
#include <mach/rpc_server_handset.h>
#include "msm-keypad-devices.h"
#include <mach/socinfo.h>
#include "pm-boot.h"
#include "board-msm7627a.h"
#ifdef CONFIG_HUAWEI_KERNEL
#include <linux/hardware_self_adapt.h>
#include <linux/touch_platform_config.h>
static char buf_virtualkey[500];
static ssize_t  buf_vkey_size=0;
#endif

#ifdef CONFIG_HUAWEI_KERNEL
#include <asm-arm/huawei/smem_vendor_huawei.h>
#include <asm-arm/huawei/usb_switch_huawei.h>
#endif

#ifdef CONFIG_HUAWEI_NFC_PN544
#include <linux/nfc/pn544.h>
#endif

#define PMEM_KERNEL_EBI1_SIZE	0x3A000
#define MSM_PMEM_AUDIO_SIZE	0x5B000

#if defined(CONFIG_GPIO_SX150X)
enum {
	SX150X_CORE,
};

static struct sx150x_platform_data sx150x_data[] __initdata = {
	[SX150X_CORE]	= {
		.gpio_base		= GPIO_CORE_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0x02,
		.io_open_drain_ena	= 0xfef8,
		.irq_summary		= -1,
	},
};
#endif


static struct platform_device msm_wlan_ar6000_pm_device = {
	.name           = "wlan_ar6000_pm_dev",
	.id             = -1,
};

#ifdef CONFIG_HUAWEI_KERNEL
static struct platform_device rgb_leds_device = {
    .name   = "rgb-leds",
    .id     = 0,
};

static struct platform_device keyboard_backlight_device = {
    .name       = "button-backlight",
    .id     = 1,
}; 

static struct platform_device msm_device_pmic_leds = {
    .name   = "pmic-leds",
    .id = -1,
};
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ADI_ADXL346
static int gsensor_support_dummyaddr_adi346(void)
{
    int ret = -1;	/*default value means actual address*/

    ret = (int)GS_ADI346;

    return ret;
}
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_KXTIK1004
static int gsensor_support_dummyaddr_kxtik(void)
{
    int ret = -1;	/*default value means actual address*/
    ret = (int)GS_KXTIK1004;
    return ret;
}
#endif
static int gs_init_flag = 0;   /*gsensor is not initialized*/
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_MMA8452
static struct gs_platform_data gs_mma8452_platform_data = {
    .adapt_fn = NULL,
    .slave_addr = (0x38 >> 1),  /*i2c slave address*/
    .dev_id = 0x2A,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
};
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ST_LIS3XH
static struct gs_platform_data gs_st_lis3xh_platform_data = {
    .adapt_fn = NULL,
    .slave_addr = (0x30 >> 1),  /*i2c slave address*/
    .dev_id = 0x00,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
};
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ADI_ADXL346
static struct gs_platform_data gs_adi346_platform_data = {
    .adapt_fn = gsensor_support_dummyaddr_adi346,
    .slave_addr = (0xA6 >> 1),  /*i2c slave address*/
    .dev_id = 0x00,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
};
#endif 
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_KXTIK1004
static struct gs_platform_data gs_kxtik_platform_data = {
    .adapt_fn = gsensor_support_dummyaddr_kxtik,
    .slave_addr = (0x1E >> 1),  /*i2c slave address*/
    .dev_id = 0x05,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
};
#endif 
#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
static struct i2c_board_info core_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
	},
};

static void __init register_i2c_devices(void)
{
	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf())
		sx150x_data[SX150X_CORE].io_open_drain_ena = 0xe0f0;

	core_exp_i2c_info[0].platform_data =
			&sx150x_data[SX150X_CORE];

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				core_exp_i2c_info,
				ARRAY_SIZE(core_exp_i2c_info));
}
#endif

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(60, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc;

	if (adap_id < 0 || adap_id > 1)
		return;

	/* Each adapter gets 2 lines from the table */
	if (config_type)
		rc = msm_gpios_request_enable(&qup_i2c_gpios_hw[adap_id*2], 2);
	else
		rc = msm_gpios_request_enable(&qup_i2c_gpios_io[adap_id*2], 2);
	if (rc < 0)
		pr_err("QUP GPIO request/enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data msm_gsbi0_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi1_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

#ifdef CONFIG_ARCH_MSM7X27A
#define MSM_PMEM_MDP_SIZE       0x1A00000
#define MSM7x25A_MSM_PMEM_MDP_SIZE       0x1500000

#define MSM_PMEM_ADSP_SIZE      0xC00000
#define MSM7x25A_MSM_PMEM_ADSP_SIZE      0xB91000


#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_SIZE		0x261000
#define MSM7x25A_MSM_FB_SIZE	0xE1000
#else
#define MSM_FB_SIZE		0x196000
#define MSM7x25A_MSM_FB_SIZE	0x96000

#endif

#endif

static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc = 0;
	unsigned gpio;

	gpio = GPIO_HOST_VBUS_EN;

	rc = gpio_request(gpio, "i2c_host_vbus_en");
	if (rc < 0) {
		pr_err("failed to request %d GPIO\n", gpio);
		return;
	}
	gpio_direction_output(gpio, !!on);
	gpio_set_value_cansleep(gpio, !!on);
	gpio_free(gpio);
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
};

static void __init msm7x2x_init_host(void)
{
	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct regulator *reg_hsusb;
static int msm_hsusb_ldo_init(int init)
{
	int rc = 0;

	if (init) {
		reg_hsusb = regulator_get(NULL, "usb");
		if (IS_ERR(reg_hsusb)) {
			rc = PTR_ERR(reg_hsusb);
			pr_err("%s: could not get regulator: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_set_voltage(reg_hsusb, 3300000, 3300000);
		if (rc) {
			pr_err("%s: could not set voltage: %d\n",
					__func__, rc);
			goto reg_free;
		}

		return 0;
	}
	/* else fall through */
reg_free:
	regulator_put(reg_hsusb);
out:
	reg_hsusb = NULL;
	return rc;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (IS_ERR_OR_NULL(reg_hsusb))
		return reg_hsusb ? PTR_ERR(reg_hsusb) : -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	return enable ?
		regulator_enable(reg_hsusb) :
		regulator_disable(reg_hsusb);
}

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret = 0;

	if (init)
		ret = msm_pm_app_rpc_init(callback);
	else
		msm_pm_app_rpc_deinit(callback);

	return ret;
}
#endif

static struct msm_otg_platform_data msm_otg_pdata = {
#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init	 = msm_hsusb_pmic_notif_init,
#else
	.vbus_power		 = msm_hsusb_vbus_power,
#endif
	.rpc_connect		 = hsusb_rpc_connect,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.chg_init		 = hsusb_chg_init,
	.chg_connected		 = hsusb_chg_connected,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
};
#endif

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x90000300,
		.end   = 0x900003ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(4),
		.end   = MSM_GPIO_TO_INT(4),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

#ifdef CONFIG_HUAWEI_KERNEL
#if (defined(CONFIG_SERIAL_MSM_HS) && defined(CONFIG_HUAWEI_BT_WCN2243))
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup	= 1,
	.rx_to_inject		= 0xFD,
};
#endif
#else
#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup	= 1,
	.rx_to_inject		= 0xFD,
};
#endif
#endif
static struct msm_pm_platform_data msm7x27a_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 16000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 12000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 1,
					.latency = 2000,
					.residency = 0,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 2,
					.residency = 0,
	},
};

u32 msm7627a_power_collapse_latency(enum msm_pm_sleep_mode mode)
{
	switch (mode) {
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		return msm7x27a_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency;
		
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN:
		return msm7x27a_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency;
		
	case MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT:
		return msm7x27a_pm_data[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
		
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		return msm7x27a_pm_data[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
		
	default:
		return 0;
	}
}

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS,
	.p_addr = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}

early_param("fb_size", fb_size_setup);

#ifdef CONFIG_HUAWEI_KERNEL

#define GPIO_OUT_39   39
#define GPIO_OUT_38    38
#define GPIO_IN_36    36
#define GPIO_OUT_36    36
#define GPIO_OUT_31    31
#endif
#ifndef CONFIG_HUAWEI_KERNEL
static struct regulator_bulk_data regs_lcdc[] = {
	{ .supply = "gp2",   .min_uV = 2850000, .max_uV = 2850000 },
	{ .supply = "msme1", .min_uV = 1800000, .max_uV = 1800000 },
};

static uint32_t lcdc_gpio_initialized;

static void lcdc_toshiba_gpio_init(void)
{
	int rc = 0;
	if (!lcdc_gpio_initialized) {
		if (gpio_request(GPIO_SPI_CLK, "spi_clk")) {
			pr_err("failed to request gpio spi_clk\n");
			return;
		}
		if (gpio_request(GPIO_SPI_CS0_N, "spi_cs")) {
			pr_err("failed to request gpio spi_cs0_N\n");
			goto fail_gpio6;
		}
		if (gpio_request(GPIO_SPI_MOSI, "spi_mosi")) {
			pr_err("failed to request gpio spi_mosi\n");
			goto fail_gpio5;
		}
		if (gpio_request(GPIO_SPI_MISO, "spi_miso")) {
			pr_err("failed to request gpio spi_miso\n");
			goto fail_gpio4;
		}
		if (gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr")) {
			pr_err("failed to request gpio_disp_pwr\n");
			goto fail_gpio3;
		}
		if (gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en")) {
			pr_err("failed to request gpio_bkl_en\n");
			goto fail_gpio2;
		}
		pmapp_disp_backlight_init();

		rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_lcdc), regs_lcdc);
		if (rc) {
			pr_err("%s: could not get regulators: %d\n",
					__func__, rc);
			goto fail_gpio1;
		}

		rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_lcdc),
				regs_lcdc);
		if (rc) {
			pr_err("%s: could not set voltages: %d\n",
					__func__, rc);
			goto fail_vreg;
		}
		lcdc_gpio_initialized = 1;
	}
	return;
fail_vreg:
	regulator_bulk_free(ARRAY_SIZE(regs_lcdc), regs_lcdc);
fail_gpio1:
	gpio_free(GPIO_BACKLIGHT_EN);
fail_gpio2:
	gpio_free(GPIO_DISPLAY_PWR_EN);
fail_gpio3:
	gpio_free(GPIO_SPI_MISO);
fail_gpio4:
	gpio_free(GPIO_SPI_MOSI);
fail_gpio5:
	gpio_free(GPIO_SPI_CS0_N);
fail_gpio6:
	gpio_free(GPIO_SPI_CLK);
	lcdc_gpio_initialized = 0;
}

static uint32_t lcdc_gpio_table[] = {
	GPIO_SPI_CLK,
	GPIO_SPI_CS0_N,
	GPIO_SPI_MOSI,
	GPIO_DISPLAY_PWR_EN,
	GPIO_BACKLIGHT_EN,
	GPIO_SPI_MISO,
};

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n;

	if (lcdc_gpio_initialized) {
		/* All are IO Expander GPIOs */
		for (n = 0; n < (len - 1); n++)
			gpio_direction_output(table[n], 1);
	}
}

static void lcdc_toshiba_config_gpios(int enable)
{
	config_lcdc_gpio_table(lcdc_gpio_table,
		ARRAY_SIZE(lcdc_gpio_table), enable);
}

static int msm_fb_lcdc_power_save(int on)
{
	int rc = 0;
	/* Doing the init of the LCDC GPIOs very late as they are from
		an I2C-controlled IO Expander */
	lcdc_toshiba_gpio_init();

	if (lcdc_gpio_initialized) {
		gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);

		rc = on ? regulator_bulk_enable(
				ARRAY_SIZE(regs_lcdc), regs_lcdc) :
			  regulator_bulk_disable(
				ARRAY_SIZE(regs_lcdc), regs_lcdc);

		if (rc)
			pr_err("%s: could not %sable regulators: %d\n",
					__func__, on ? "en" : "dis", rc);
	}

	return rc;
}


static int lcdc_toshiba_set_bl(int level)
{
	int ret;

	ret = pmapp_disp_backlight_set_brightness(level);
	if (ret)
		pr_err("%s: can't set lcd backlight!\n", __func__);

	return ret;
}


static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = NULL,
	.lcdc_power_save   = msm_fb_lcdc_power_save,
};

static int lcd_panel_spi_gpio_num[] = {
		GPIO_SPI_MOSI,  /* spi_sdi */
		GPIO_SPI_MISO,  /* spi_sdoi */
		GPIO_SPI_CLK,   /* spi_clk */
		GPIO_SPI_CS0_N, /* spi_cs  */
};

static struct msm_panel_common_pdata lcdc_toshiba_panel_data = {
	.panel_config_gpio = lcdc_toshiba_config_gpios,
	.pmic_backlight = lcdc_toshiba_set_bl,
	.gpio_num	  = lcd_panel_spi_gpio_num,
};

static struct platform_device lcdc_toshiba_panel_device = {
	.name   = "lcdc_toshiba_fwvga_pt",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_toshiba_panel_data,
	}
};
#else
static uint32_t lcdc_gpio_initialized = 0;
static void lcdc_hw_gpio_init(void)
{
	if (!lcdc_gpio_initialized) {
		if (gpio_request(GPIO_OUT_38, "spi_clk")) {
			pr_err("failed to request gpio spi_clk\n");
			goto fail_gpio4;
		}
		if (gpio_request(GPIO_OUT_39, "spi_cs")) {
			pr_err("failed to request gpio spi_cs0_N\n");
			goto fail_gpio3;
		}
		if (gpio_request(GPIO_OUT_36, "spi_sdoi")) {
			pr_err("failed to request gpio spi_sdoi\n");
			goto fail_gpio2;
		}
		if (gpio_request(GPIO_OUT_31, "lcd_reset")) {
			pr_err("failed to request gpio lcd_reset\n");
			goto fail_gpio1;
		}
		if (get_hw_lcd_ctrl_bl_type() == CTRL_BL_BY_MSM)
		{
			pmapp_disp_backlight_init();
		}		
	
		lcdc_gpio_initialized = 1;
	}
	return;


fail_gpio1:
	gpio_free(GPIO_OUT_36);
fail_gpio2:
	gpio_free(GPIO_OUT_39);
fail_gpio3:
	gpio_free(GPIO_OUT_38);
fail_gpio4:
	lcdc_gpio_initialized = 0;
}

static uint32_t lcdc_gpio_table[] = {
	GPIO_CFG(GPIO_OUT_38, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_39, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_IN_36, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),	
	GPIO_CFG(GPIO_OUT_36, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n],
			enable ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}
static void lcdc_hw_config_gpios(int enable)
{
	config_lcdc_gpio_table(lcdc_gpio_table,
		ARRAY_SIZE(lcdc_gpio_table), enable);
}
static int msm_fb_lcdc_power_save(int on)
{
	int rc = 0;
	lcdc_hw_gpio_init();

	return rc;


}
static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = NULL,
	.lcdc_power_save   = msm_fb_lcdc_power_save,
};
static int lcd_panel_spi_gpio_num[] = {
	GPIO_OUT_38,/* spi_clk */
	GPIO_OUT_39,/* spi_cs  */
	GPIO_IN_36,/* spi_sdi */
	GPIO_OUT_36,/* spi_sdoi */
	GPIO_OUT_31/* LCD reset */
   

};
static struct msm_panel_common_pdata lcdc_hw_panel_data = {
	.panel_config_gpio = lcdc_hw_config_gpios,
	.gpio_num	  = lcd_panel_spi_gpio_num,
};
static struct platform_device lcdc_hx8357b_panel_device = 
{
    .name   = "lcdc_hx8357b_hvga",
    .id     = 0,
    .dev    = {
        .platform_data = &lcdc_hw_panel_data,
    }
};

static struct platform_device lcdc_hx8357c_panel_device = 
{
    .name   = "lcdc_hx8357c_hvga",
    .id     = 0,
    .dev    = {
        .platform_data = &lcdc_hw_panel_data,
    }
};
static struct platform_device lcdc_truly_r61529_panel_device = 
{
    .name   = "lcdc_truly_r61529_hvga",
    .id     = 0,
    .dev    = {
        .platform_data = &lcdc_hw_panel_data,
    }
};
static struct platform_device lcdc_nt35410_panel_device = 
{
    .name   = "lcdc_nt35410_hvga",
    .id     = 0,
    .dev    = {
        .platform_data = &lcdc_hw_panel_data,
    }
};
static struct platform_device lcdc_hx8347d_panel_device = {
	.name   = "lcdc_hx8347d_qvga",
	.id     = 0,	
	.dev    = {
		.platform_data = &lcdc_hw_panel_data,	
	}
};
static struct platform_device lcdc_hx8347g_panel_device = {
	.name   = "lcdc_hx8347g_qvga",
	.id 	= 0,
	.dev	= {
		.platform_data = &lcdc_hw_panel_data,
	}
};
#endif
static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

#define PANEL_NAME_MAX_LEN	30
#define LCDC_TOSHIBA_FWVGA_PANEL_NAME	"lcdc_toshiba_fwvga_pt"
#define MIPI_CMD_RENESAS_FWVGA_PANEL_NAME	"mipi_cmd_renesas_fwvga"

static int msm_fb_detect_panel(const char *name)
{
	int ret = -ENODEV;

	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf()) {
		if (!strncmp(name, "lcdc_toshiba_fwvga_pt", 21) ||
				!strncmp(name, "mipi_cmd_renesas_fwvga", 22))
			ret = 0;
	} else if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa()) {
		if (!strncmp(name, "mipi_cmd_renesas_fwvga", 22))
			ret = 0;
	}

#if !defined(CONFIG_FB_MSM_LCDC_AUTO_DETECT) && \
	!defined(CONFIG_FB_MSM_MIPI_PANEL_AUTO_DETECT) && \
	!defined(CONFIG_FB_MSM_LCDC_MIPI_PANEL_AUTO_DETECT)
		if (machine_is_msm7x27a_surf() ||
			machine_is_msm7625a_surf()) {
			if (!strncmp(name, LCDC_TOSHIBA_FWVGA_PANEL_NAME,
				strnlen(LCDC_TOSHIBA_FWVGA_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
				return 0;
		}
#endif
	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

#ifdef CONFIG_FB_MSM_MIPI_DSI
static int mipi_renesas_set_bl(int level)
{
	int ret;

	ret = pmapp_disp_backlight_set_brightness(level);

	if (ret)
		pr_err("%s: can't set lcd backlight!\n", __func__);

	return ret;
}

static struct msm_panel_common_pdata mipi_renesas_pdata = {
	.pmic_backlight = mipi_renesas_set_bl,
};


static struct platform_device mipi_dsi_renesas_panel_device = {
	.name = "mipi_renesas",
	.id = 0,
	.dev    = {
		.platform_data = &mipi_renesas_pdata,
	}
};
#endif

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
	SND(FM_DIGITAL_STEREO_HEADSET, 26),
	SND(FM_DIGITAL_SPEAKER_PHONE, 27),
	SND(FM_DIGITAL_BT_A2DP_HEADSET, 28),
	SND(STEREO_HEADSET_AND_SPEAKER, 31),
	SND(CURRENT, 0x7FFFFFFE),
	SND(FM_RADIO_STEREO_HEADSET, 29),
	SND(FM_RADIO_SPEAKER_PHONE, 30),	
	SND(FM_ANALOG_STEREO_HEADSET, 35),
	SND(FM_ANALOG_STEREO_HEADSET_CODEC, 36),
	SND(HANDSET_2NDMIC, 37),
	SND(HANDSET_HAC, 38),
	SND(IN_S_SADC_OUT_HANDSET_HAC, 39),
	SND(SPEAKER_HEADSETMIC, 40),
	SND(HANDSET_HEADSETMIC, 41),
	SND(HEADSET_MAINMIC, 42),
	SND(HEADSET_SECMIC, 43),
	SND(HEADSET_HEADSETMIC, 44),
	 
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	0, 0, 0, 0,

	/* Concurrency 7 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design     = 3200,
	.voltage_max_design     = 4200,
	.voltage_fail_safe      = 3340,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage	 = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage = msm_psy_batt_data.voltage_max_design;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}

static struct platform_device msm_batt_device = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_16BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start	= 0x90000000,
		.end	= 0x90007fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(48),
		.end	= MSM_GPIO_TO_INT(48),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data	= &smsc911x_config,
	},
};
#ifndef CONFIG_HUAWEI_CAMERA

static struct msm_gpio smsc911x_gpios[] = {
	{ GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "eth_fifo_sel" },
};

#define ETH_FIFO_SEL_GPIO	49
static void msm7x27a_cfg_smsc911x(void)
{
	int res;

	res = msm_gpios_request_enable(smsc911x_gpios,
				 ARRAY_SIZE(smsc911x_gpios));
	if (res) {
		pr_err("%s: unable to enable gpios for SMSC911x\n", __func__);
		return;
	}

	/* ETH_FIFO_SEL */
	res = gpio_direction_output(ETH_FIFO_SEL_GPIO, 0);
	if (res) {
		pr_err("%s: unable to get direction for gpio %d\n", __func__,
							 ETH_FIFO_SEL_GPIO);
		msm_gpios_disable_free(smsc911x_gpios,
						 ARRAY_SIZE(smsc911x_gpios));
		return;
	}
	gpio_set_value(ETH_FIFO_SEL_GPIO, 0);
}
#endif
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_9900
int aps9900_gpio_config_interrupt(void)
{
    int gpio_config = 0;
    int ret = 0;
  
    gpio_config = GPIO_CFG(MSM_7X27A_APS9900_INT, 0, GPIO_CFG_INPUT,GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
    ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
    return ret; 
}

static struct aps9900_hw_platform_data aps9900_hw_data = {
    .aps9900_gpio_config_interrupt = aps9900_gpio_config_interrupt,
};
#endif
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
static struct platform_device huawei_device_detect = {
	.name = "hw-dev-detect",
	.id   =-1,
};
#endif

#if defined(CONFIG_SERIAL_MSM_HSL_CONSOLE) \
		&& defined(CONFIG_MSM_SHARED_GPIO_FOR_UART2DM)
static struct msm_gpio uart2dm_gpios[] = {
	{GPIO_CFG(19, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rfr_n" },
	{GPIO_CFG(20, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_cts_n" },
	{GPIO_CFG(21, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rx"    },
	{GPIO_CFG(108, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_tx"    },
};

static void msm7x27a_cfg_uart2dm_serial(void)
{
	int ret;
	ret = msm_gpios_request_enable(uart2dm_gpios,
					ARRAY_SIZE(uart2dm_gpios));
	if (ret)
		pr_err("%s: unable to enable gpios for uart2dm\n", __func__);
}
#else
static void msm7x27a_cfg_uart2dm_serial(void) { }
#endif

static struct platform_device *rumi_sim_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&smc91x_device,
	&msm_device_nand,
	&msm_device_uart_dm1,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
};

static struct platform_device *surf_ffa_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_uart_dm1,
	&msm_device_uart_dm2,
	&msm_device_nand,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&android_usb_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_snd,
	&msm_device_adspdec,
	&msm_fb_device,
#ifndef CONFIG_HUAWEI_KERNEL
	&lcdc_toshiba_panel_device,
#else
	&lcdc_hx8357b_panel_device,
	&lcdc_hx8357c_panel_device,
	&lcdc_truly_r61529_panel_device,
    &lcdc_nt35410_panel_device,
	&lcdc_hx8347d_panel_device,
	&lcdc_hx8347g_panel_device,
#endif
	&msm_batt_device,
	&smsc911x_device,
#ifdef CONFIG_FB_MSM_MIPI_DSI
	&mipi_dsi_renesas_panel_device,
#endif

	&msm_kgsl_3d0,
#if defined(CONFIG_BT) && defined(CONFIG_HUAWEI_BT_WCN2243)
	&msm_bt_power_device,
#endif

#if defined(CONFIG_BT) && defined(CONFIG_HUAWEI_BT_BCM43XX)
    &msm_bt_power_device,
    &msm_bluesleep_device,	
#endif

#ifndef CONFIG_HUAWEI_KERNEL
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#ifdef CONFIG_HUAWEI_KERNEL
	&keyboard_backlight_device,
	&rgb_leds_device,
	&msm_device_pmic_leds,
#endif
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	&huawei_device_detect,
#endif
};

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static void __init msm_msm7x2x_allocate_memory_regions(void)
{
#ifdef CONFIG_FRAMEBUF_SELF_ADAPT
	__u32 framebuf_addr = 0; 
	__u32 framebuf_size = 0;

	get_frame_buffer_mem_region(&framebuf_addr, &framebuf_size);
	msm_fb_resources[0].start = framebuf_addr;
	msm_fb_resources[0].end = framebuf_addr + framebuf_size -1;
	
	pr_info("allocating 0x%x bytes at 0x%x for framebuffer\n", framebuf_size, framebuf_addr);
#else
	void *addr;
	unsigned long size;

	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa())
		fb_size = MSM7x25A_MSM_FB_SIZE;
	else
		fb_size = MSM_FB_SIZE;

	size = fb_size;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
#endif /* CONFIG_FRAMEBUF_SELF_ADAPT */
}

static struct memtype_reserve msm7x27a_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{

	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		pmem_mdp_size = MSM7x25A_MSM_PMEM_MDP_SIZE;
		pmem_adsp_size = MSM7x25A_MSM_PMEM_ADSP_SIZE;
	} else {
		pmem_mdp_size = MSM_PMEM_MDP_SIZE;
		pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
	}

#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_mdp_size;
	android_pmem_audio_pdata.size = pmem_audio_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x27a_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
}

static void __init msm7x27a_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x27a_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27a_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27a_reserve_table,
	.calculate_reserve_sizes = msm7x27a_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27a_paddr_to_memtype,
};

static void __init msm7x27a_reserve(void)
{
	reserve_info = &msm7x27a_reserve_info;
	msm_reserve();
}

#ifdef CONFIG_HUAWEI_KERNEL
struct vreg *vreg_s3 = NULL;

int i2c_power(void)
{
#if 0
	int rc = 0;
	
	vreg_s3 = vreg_get(NULL, "s3");
	if(IS_ERR(vreg_s3))
	{
		pr_err("%s:s3 power init get failed\n", __func__);
		goto err_power_fail;
	}

	rc = vreg_set_level(vreg_s3, 1800);
	if(rc)
	{
		pr_err("%s:s3 power init faild\n",__func__);
		goto err_power_fail;
	}

	rc = vreg_enable(vreg_s3);
	if (rc) 
	{
		pr_err("%s:s3 power init failed \n", __func__);
	}
	
err_power_fail:
	return rc;
#endif
     return 0;
}
#endif
static void __init msm_device_i2c_init(void)
{
	#ifdef CONFIG_HUAWEI_KERNEL
	i2c_power();
	#endif
	msm_gsbi0_qup_i2c_device.dev.platform_data = &msm_gsbi0_qup_i2c_pdata;
	msm_gsbi1_qup_i2c_device.dev.platform_data = &msm_gsbi1_qup_i2c_pdata;
}

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
	.mdp_rev = MDP_REV_303,
};


#ifdef CONFIG_FB_MSM
#ifndef CONFIG_HUAWEI_KERNEL
#define GPIO_LCDC_BRDG_PD	128
#define GPIO_LCDC_BRDG_RESET_N	129

#define LCDC_RESET_PHYS		0x90008014

static	void __iomem *lcdc_reset_ptr;

static unsigned mipi_dsi_gpio[] = {
	GPIO_CFG(GPIO_LCDC_BRDG_RESET_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA),       /* LCDC_BRDG_RESET_N */
	GPIO_CFG(GPIO_LCDC_BRDG_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA),       /* LCDC_BRDG_RESET_N */
};
#endif
enum {
	DSI_SINGLE_LANE = 1,
	DSI_TWO_LANES,
};

static int msm_fb_get_lane_config(void)
{
	int rc = DSI_TWO_LANES;

	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		rc = DSI_SINGLE_LANE;
		pr_info("DSI Single Lane\n");
	} else {
		pr_info("DSI Two Lanes\n");
	}
	return rc;
}
#ifndef CONFIG_HUAWEI_KERNEL
static int msm_fb_dsi_client_reset(void)
{
	int rc = 0;

	rc = gpio_request(GPIO_LCDC_BRDG_RESET_N, "lcdc_brdg_reset_n");
	if (rc < 0) {
		pr_err("failed to request lcd brdg reset_n\n");
		return rc;
	}

	rc = gpio_request(GPIO_LCDC_BRDG_PD, "lcdc_brdg_pd");
	if (rc < 0) {
		pr_err("failed to request lcd brdg pd\n");
		return rc;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[0], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge reset enable\n");
		goto gpio_error;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[1], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge pd enable\n");
		goto gpio_error2;
	}

	rc = gpio_direction_output(GPIO_LCDC_BRDG_RESET_N, 1);
	rc |= gpio_direction_output(GPIO_LCDC_BRDG_PD, 1);
	gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 0);

	if (!rc) {
		if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf()) {
			lcdc_reset_ptr = ioremap_nocache(LCDC_RESET_PHYS,
				sizeof(uint32_t));

			if (!lcdc_reset_ptr)
				return 0;
		}
		return rc;
	} else {
		goto gpio_error;
	}

gpio_error2:
	pr_err("Failed GPIO bridge pd\n");
	gpio_free(GPIO_LCDC_BRDG_PD);

gpio_error:
	pr_err("Failed GPIO bridge reset\n");
	gpio_free(GPIO_LCDC_BRDG_RESET_N);
	return rc;
}

static struct regulator_bulk_data regs_dsi[] = {
	{ .supply = "gp2",   .min_uV = 2850000, .max_uV = 2850000 },
	{ .supply = "msme1", .min_uV = 1800000, .max_uV = 1800000 },
};

static int dsi_gpio_initialized;

static int mipi_dsi_panel_power(int on)
{
	int rc = 0;
	uint32_t lcdc_reset_cfg;

	/* I2C-controlled GPIO Expander -init of the GPIOs very late */
	if (unlikely(!dsi_gpio_initialized)) {
		pmapp_disp_backlight_init();

		rc = gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr");
		if (rc < 0) {
			pr_err("failed to request gpio_disp_pwr\n");
			return rc;
		}

		if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf()) {
			rc = gpio_direction_output(GPIO_DISPLAY_PWR_EN, 1);
			if (rc < 0) {
				pr_err("failed to enable display pwr\n");
				goto fail_gpio1;
			}

			rc = gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en");
			if (rc < 0) {
				pr_err("failed to request gpio_bkl_en\n");
				goto fail_gpio1;
			}

			rc = gpio_direction_output(GPIO_BACKLIGHT_EN, 1);
			if (rc < 0) {
				pr_err("failed to enable backlight\n");
				goto fail_gpio2;
			}
		}

		rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_dsi), regs_dsi);
		if (rc) {
			pr_err("%s: could not get regulators: %d\n",
					__func__, rc);
			goto fail_gpio2;
		}

		rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_dsi), regs_dsi);
		if (rc) {
			pr_err("%s: could not set voltages: %d\n",
					__func__, rc);
			goto fail_vreg;
		}
		if (pmapp_disp_backlight_set_brightness(100))
			pr_err("backlight set brightness failed\n");

		dsi_gpio_initialized = 1;
	}

	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf()) {
		gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);
	} else if (machine_is_msm7x27a_ffa() ||
			 machine_is_msm7625a_ffa()) {
		if (on) {
			/* This line drives an active low pin on FFA */
			rc = gpio_direction_output(GPIO_DISPLAY_PWR_EN, !on);
			if (rc < 0)
				pr_err("failed to set direction for "
					"display pwr\n");
		} else {
			gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, !on);
			rc = gpio_direction_input(GPIO_DISPLAY_PWR_EN);
			if (rc < 0)
				pr_err("failed to set direction for "
					"display pwr\n");
		}
	}

	if (on) {
		gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 0);

		if (machine_is_msm7x27a_surf() ||
				 machine_is_msm7625a_surf()) {
			lcdc_reset_cfg = readl_relaxed(lcdc_reset_ptr);
			rmb();
			lcdc_reset_cfg &= ~1;

			writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
			msleep(20);
			wmb();
			lcdc_reset_cfg |= 1;
			writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
		} else {
			gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N, 0);
			msleep(20);
			gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N, 1);
		}
	} else {
		gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 1);
	}

	rc = on ? regulator_bulk_enable(ARRAY_SIZE(regs_dsi), regs_dsi) :
		  regulator_bulk_disable(ARRAY_SIZE(regs_dsi), regs_dsi);

	if (rc)
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, on ? "en" : "dis", rc);

	return rc;

fail_vreg:
	regulator_bulk_free(ARRAY_SIZE(regs_dsi), regs_dsi);
fail_gpio2:
	gpio_free(GPIO_BACKLIGHT_EN);
fail_gpio1:
	gpio_free(GPIO_DISPLAY_PWR_EN);
	dsi_gpio_initialized = 0;
	return rc;
}
#else
static int msm_fb_dsi_client_reset(void)
{
	return 0;
}

static int dsi_gpio_initialized;

static int mipi_dsi_panel_power(int on)
{
	if (!dsi_gpio_initialized) {	
		if (get_hw_lcd_ctrl_bl_type() == CTRL_BL_BY_MSM)
		{
			pmapp_disp_backlight_init();
		}
		
		dsi_gpio_initialized = 1;
	}
	
	return 0;
}
#endif
#endif

#define MDP_303_VSYNC_GPIO 97

#ifdef CONFIG_FB_MSM_MDP303
static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.vsync_gpio = MDP_303_VSYNC_GPIO,
	.dsi_power_save   = mipi_dsi_panel_power,
	.dsi_client_reset = msm_fb_dsi_client_reset,
	.get_lane_config = msm_fb_get_lane_config,
};
#endif

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("lcdc", &lcdc_pdata);
#ifdef CONFIG_FB_MSM_MDP303
	msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
#endif
}

#define MSM_EBI2_PHYS			0xa0d00000
#define MSM_EBI2_XMEM_CS2_CFG1		0xa0d10030

static void __init msm7x27a_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void __iomem *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_PHYS, sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_rumi3() || machine_is_msm7x27a_surf() ||
			machine_is_msm7625a_surf())
		ebi2_cfg |= (1 << 4); /* CS2 */

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);

	/* Enable A/D MUX[bit 31] from EBI2_XMEM_CS2_CFG1 */
	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_XMEM_CS2_CFG1,
							 sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf())
		ebi2_cfg |= (1 << 31);

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);
}

#ifdef CONFIG_HUAWEI_KERNEL
#define IC_PM_ON   1
#define IC_PM_OFF  0

#define MSM_7X27A_TOUCH_INT_PIN 82
#define MSM_7x27A_TOUCH_RESET_PIN 96 //13
atomic_t touch_detected_yet = ATOMIC_INIT(0);
struct vreg *vreg_l12 = NULL;

int power_switch(int pm)
{
#if 0
	int rc = 0;
	
	if (IC_PM_ON == pm)
	{
		
		vreg_l12 = vreg_get(NULL,"gp2");
		if (IS_ERR(vreg_l12)) 
		{
			pr_err("%s:l12 power init get failed\n", __func__);
			goto err_power_fail;
		}
		
		rc = vreg_set_level(vreg_l12, 2850);
		if(rc)
		{
			pr_err("%s:l12 power init faild\n",__func__);
			goto err_power_fail;
		}

		
		rc = vreg_enable(vreg_l12);
		if (rc) 
		{
			pr_err("%s:l12 power init failed \n", __func__);
		}

		mdelay(50);     

	}
	else if(IC_PM_OFF == pm)
	{
		if(NULL != vreg_l12)
		{
			rc = vreg_disable(vreg_l12);
			if (rc)
			{
				pr_err("%s:l12 power disable failed \n", __func__);
			}
		}
	}
	else 
	{
		rc = -EPERM;
		pr_err("%s:l12 power switch not support yet!\n", __func__);	
	}
err_power_fail:
	return rc;
#endif
	return 0;
}

int touch_gpio_config_interrupt(void)
{
	gpio_request(MSM_7X27A_TOUCH_INT_PIN, "TOUCH_INT");
	
	return gpio_tlmm_config(GPIO_CFG(MSM_7X27A_TOUCH_INT_PIN, 
						0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, 
						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
}

int touch_reset(void)
{
	int ret = 0;

	gpio_request(MSM_7x27A_TOUCH_RESET_PIN, "TOUCH_RESET");
	
	ret = gpio_tlmm_config(GPIO_CFG(MSM_7x27A_TOUCH_RESET_PIN, 
						0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, 
						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	
	ret = gpio_direction_output(MSM_7x27A_TOUCH_RESET_PIN, 1);
	mdelay(5);
	ret = gpio_direction_output(MSM_7x27A_TOUCH_RESET_PIN, 0);
	mdelay(10);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
	ret = gpio_direction_output(MSM_7x27A_TOUCH_RESET_PIN, 1);
	mdelay(50);//must more than 10ms.

	return ret;
}

int get_touch_reset_pin(void)
{
	int ret = MSM_7x27A_TOUCH_RESET_PIN;
	
	return ret;
}

void set_touch_probe_flag(int detected)
{
	if(detected >= 0)
	{
		atomic_set(&touch_detected_yet, 1);
	}
	else
	{
		atomic_set(&touch_detected_yet, 0);
	}
	
	return;
}

int read_touch_probe_flag(void)
{	
	return atomic_read(&touch_detected_yet);
}

static int get_phone_version(struct tp_resolution_conversion *tp_resolution_type)
{	
	if (machine_is_msm7x27a_U8815() ||
	    machine_is_msm7x27a_C8820() ||
		machine_is_msm7x27a_C8825D() )
	{
		tp_resolution_type->lcd_x = LCD_X_WVGA;
		tp_resolution_type->lcd_y = LCD_Y_WVGA;   
		tp_resolution_type->lcd_all = LCD_ALL_WVGA_4INCHTP;
	}
	else if (machine_is_msm7x27a_U8655() ||machine_is_msm7x27a_U8655_EMMC() || machine_is_msm7x27a_C8655_NAND()||
	         machine_is_msm7x27a_U8661())
	{
		tp_resolution_type->lcd_x = LCD_X_HVGA;
		tp_resolution_type->lcd_y = LCD_Y_HVGA;   
		tp_resolution_type->lcd_all = LCD_ALL_HVGA_35INCHTP;
	}
	else if (machine_is_msm7x27a_M660())
	{
		tp_resolution_type->lcd_x = LCD_X_HVGA;
		tp_resolution_type->lcd_y = LCD_Y_HVGA;   
		tp_resolution_type->lcd_all = LCD_ALL_HVGA_32INCHTP;
	}
    else if (machine_is_msm7x27a_U8185())
    {
        tp_resolution_type->lcd_x = LCD_X_QVGA;
		tp_resolution_type->lcd_y = LCD_Y_QVGA;   
		tp_resolution_type->lcd_all = LCD_ALL_QVGA;
    }
    else if (machine_is_msm7x27a_C8668D())
    {
        tp_resolution_type->lcd_x = LCD_X_HVGA;
        tp_resolution_type->lcd_y = LCD_Y_HVGA;   
        tp_resolution_type->lcd_all = LCD_ALL_HVGA_35INCHTP;    	
    }
	else
	{
		tp_resolution_type->lcd_x = LCD_X_FWVGA;
		tp_resolution_type->lcd_y = LCD_Y_FWVGA;   
		tp_resolution_type->lcd_all = LCD_ALL_FWVGA;
	}
	return 1;
}

static struct touch_hw_platform_data touch_hw_data = 
{
	.touch_power = power_switch,
	.touch_gpio_config_interrupt = touch_gpio_config_interrupt,
	.set_touch_probe_flag = set_touch_probe_flag,
	.read_touch_probe_flag = read_touch_probe_flag,
	.touch_reset = touch_reset,
	.get_touch_reset_pin = get_touch_reset_pin,
	.get_phone_version = get_phone_version,
};

#ifdef CONFIG_HUAWEI_NFC_PN544
static int pn544_ven_reset(void)
{
	int ret=0;
	int gpio_config=0;
	
	gpio_config = GPIO_CFG(GPIO_NFC_VEN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
	ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	ret = gpio_request(GPIO_NFC_VEN, "gpio 113 for NFC pn544");
	ret = gpio_direction_output(GPIO_NFC_VEN,0);
	gpio_set_value(GPIO_NFC_VEN, 1);
	mdelay(5);
	gpio_set_value(GPIO_NFC_VEN, 0);

	mdelay(10);
	gpio_set_value(GPIO_NFC_VEN, 1);
	mdelay(5);
	return 0;
}

static int pn544_interrupt_gpio_config(void)
{
	int ret=0;
	int gpio_config=0;
	gpio_config = GPIO_CFG(GPIO_NFC_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
	ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	ret = gpio_request(GPIO_NFC_INT, "gpio 114 for NFC pn544");
	ret = gpio_direction_input(GPIO_NFC_INT);
	return 0;
}

static int pn544_fw_download_pull_down(void)
{
	gpio_set_value(GPIO_NFC_LOAD, 1);
	mdelay(5);
	gpio_set_value(GPIO_NFC_LOAD, 0);
	mdelay(5);
	return 0;	
}

static int pn544_fw_download_pull_high(void)
{
	gpio_set_value(GPIO_NFC_LOAD, 0);
	mdelay(5);
	gpio_set_value(GPIO_NFC_LOAD, 1);
	mdelay(5);
	return 0;
}

static int pn544_clock_output_ctrl(int vote)
{
       const char * id = "nfcp";
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
		vote?PMAPP_CLOCK_VOTE_ON:PMAPP_CLOCK_VOTE_OFF);
	return 0;
}

static int pn544_clock_output_mode_ctrl(int mode)
{
       const char * id = "nfcp";
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1, 
		mode?PMAPP_CLOCK_VOTE_PIN_CTRL:PMAPP_CLOCK_VOTE_OFF);
	return 0;
}

static struct pn544_nfc_platform_data pn544_hw_data = 
{
	.pn544_ven_reset = pn544_ven_reset,
	.pn544_interrupt_gpio_config = pn544_interrupt_gpio_config,
	.pn544_fw_download_pull_down = pn544_fw_download_pull_down,
	.pn544_fw_download_pull_high = pn544_fw_download_pull_high,
	.pn544_clock_output_ctrl = pn544_clock_output_ctrl,
	.pn544_clock_output_mode_ctrl = pn544_clock_output_mode_ctrl,
};

#endif
static struct i2c_board_info msm7x27a_i2c_board_info[] __initdata = 
{
#ifdef CONFIG_HUAWEI_MELFAS_TOUCHSCREEN
	{
		I2C_BOARD_INFO("melfas-ts", 0x23),
		.platform_data = &touch_hw_data,
		.irq = MSM_GPIO_TO_INT(MSM_7X27A_TOUCH_INT_PIN),
        .flags = true,
	},
#endif
#ifdef CONFIG_HUAWEI_FEATURE_RMI_TOUCH
	{
		I2C_BOARD_INFO("Synaptics_rmi", 0x70),
		.platform_data = &touch_hw_data,
		.irq = MSM_GPIO_TO_INT(MSM_7X27A_TOUCH_INT_PIN),
        .flags = true,
	},
	{
		I2C_BOARD_INFO("Synaptics_rmi", 0x24),
		.platform_data = &touch_hw_data,
		.irq = MSM_GPIO_TO_INT(MSM_7X27A_TOUCH_INT_PIN),
        .flags = true,
	},
#endif
#ifdef CONFIG_QWERTY_KEYPAD_ADP5587
	{
		I2C_BOARD_INFO("adp5587", 0x34),
		.irq = MSM_GPIO_TO_INT(40)
	},
#endif 
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_MMA8452
    {
        I2C_BOARD_INFO("gs_mma8452", 0x38 >> 1),
        .platform_data = &gs_mma8452_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
#endif	
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_AK8975
    {
        I2C_BOARD_INFO("akm8975", 0x0D),//7 bit addr, no write bit
        .irq = MSM_GPIO_TO_INT(18)
    },
#endif 
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ST_LIS3XH
    {
        I2C_BOARD_INFO("gs_st_lis3xh", 0x30 >> 1),
	 .platform_data = &gs_st_lis3xh_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ADI_ADXL346
    {
        I2C_BOARD_INFO("gs_adi346", 0xA8>>1),  /* actual address 0xA6, fake address 0xA8*/
	 .platform_data = &gs_adi346_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
#endif 
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_KXTIK1004
	{ 
	    I2C_BOARD_INFO("gs_kxtik", 0x1E >> 1),  /* actual address 0x0F*/
	.platform_data = &gs_kxtik_platform_data,
	    .irq = MSM_GPIO_TO_INT(19)     //MEMS_INT1
	},
#endif
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_12D
	{   
		I2C_BOARD_INFO("aps-12d", 0x88 >> 1),  
	},
#endif

#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_9900
	{   
		I2C_BOARD_INFO("aps-9900", 0x39),
	    .irq = MSM_GPIO_TO_INT(MSM_7X27A_APS9900_INT),
        .platform_data = &aps9900_hw_data,
	},
#endif
#ifdef CONFIG_HUAWEI_FEATURE_TPS61310
	{
		I2C_BOARD_INFO("tps61310" , 0x33),
	},
#endif
#ifdef CONFIG_HUAWEI_NFC_PN544
	{
		I2C_BOARD_INFO(PN544_DRIVER_NAME, PN544_I2C_ADDR),
		.irq = MSM_GPIO_TO_INT(GPIO_NFC_INT),
		.platform_data = &pn544_hw_data,
	},
#endif
};
#else
#define ATMEL_TS_I2C_NAME "maXTouch"

static struct regulator_bulk_data regs_atmel[] = {
	{ .supply = "ldo2",  .min_uV = 2850000, .max_uV = 2850000 },
	{ .supply = "smps3", .min_uV = 1800000, .max_uV = 1800000 },
};

#define ATMEL_TS_GPIO_IRQ 82

static int atmel_ts_power_on(bool on)
{
	int rc = on ?
		regulator_bulk_enable(ARRAY_SIZE(regs_atmel), regs_atmel) :
		regulator_bulk_disable(ARRAY_SIZE(regs_atmel), regs_atmel);

	if (rc)
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, on ? "en" : "dis", rc);
	else
		msleep(50);

	return rc;
}

static int atmel_ts_platform_init(struct i2c_client *client)
{
	int rc;
	struct device *dev = &client->dev;

	rc = regulator_bulk_get(dev, ARRAY_SIZE(regs_atmel), regs_atmel);
	if (rc) {
		dev_err(dev, "%s: could not get regulators: %d\n",
				__func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_atmel), regs_atmel);
	if (rc) {
		dev_err(dev, "%s: could not set voltages: %d\n",
				__func__, rc);
		goto reg_free;
	}

	rc = gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (rc) {
		dev_err(dev, "%s: gpio_tlmm_config for %d failed\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto reg_free;
	}

	/* configure touchscreen interrupt gpio */
	rc = gpio_request(ATMEL_TS_GPIO_IRQ, "atmel_maxtouch_gpio");
	if (rc) {
		dev_err(dev, "%s: unable to request gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto ts_gpio_tlmm_unconfig;
	}

	rc = gpio_direction_input(ATMEL_TS_GPIO_IRQ);
	if (rc < 0) {
		dev_err(dev, "%s: unable to set the direction of gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto free_ts_gpio;
	}
	return 0;

free_ts_gpio:
	gpio_free(ATMEL_TS_GPIO_IRQ);
ts_gpio_tlmm_unconfig:
	gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_DISABLE);
reg_free:
	regulator_bulk_free(ARRAY_SIZE(regs_atmel), regs_atmel);
out:
	return rc;
}

static int atmel_ts_platform_exit(struct i2c_client *client)
{
	gpio_free(ATMEL_TS_GPIO_IRQ);
	gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	regulator_bulk_free(ARRAY_SIZE(regs_atmel), regs_atmel);
	return 0;
}

static u8 atmel_ts_read_chg(void)
{
	return gpio_get_value(ATMEL_TS_GPIO_IRQ);
}

static u8 atmel_ts_valid_interrupt(void)
{
	return !atmel_ts_read_chg();
}

#define ATMEL_X_OFFSET 13
#define ATMEL_Y_OFFSET 0

static struct maxtouch_platform_data atmel_ts_pdata = {
	.numtouch = 4,
	.init_platform_hw = atmel_ts_platform_init,
	.exit_platform_hw = atmel_ts_platform_exit,
	.power_on = atmel_ts_power_on,
	.display_res_x = 480,
	.display_res_y = 864,
	.min_x = ATMEL_X_OFFSET,
	.max_x = (505 - ATMEL_X_OFFSET),
	.min_y = ATMEL_Y_OFFSET,
	.max_y = (863 - ATMEL_Y_OFFSET),
	.valid_interrupt = atmel_ts_valid_interrupt,
	.read_chg = atmel_ts_read_chg,
};

static struct i2c_board_info atmel_ts_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO(ATMEL_TS_I2C_NAME, 0x4a),
		.platform_data = &atmel_ts_pdata,
		.irq = MSM_GPIO_TO_INT(ATMEL_TS_GPIO_IRQ),
	},
};
#endif
#ifndef CONFIG_HUAWEI_GPIO_KEYPAD
#define KP_INDEX(row, col) ((row)*ARRAY_SIZE(kp_col_gpios) + (col))

static unsigned int kp_row_gpios[] = {31, 32, 33, 34, 35};
static unsigned int kp_col_gpios[] = {36, 37, 38, 39, 40};

static const unsigned short keymap[ARRAY_SIZE(kp_col_gpios) *
					  ARRAY_SIZE(kp_row_gpios)] = {
	[KP_INDEX(0, 0)] = KEY_7,
	[KP_INDEX(0, 1)] = KEY_DOWN,
	[KP_INDEX(0, 2)] = KEY_UP,
	[KP_INDEX(0, 3)] = KEY_RIGHT,
	[KP_INDEX(0, 4)] = KEY_ENTER,

	[KP_INDEX(1, 0)] = KEY_LEFT,
	[KP_INDEX(1, 1)] = KEY_SEND,
	[KP_INDEX(1, 2)] = KEY_1,
	[KP_INDEX(1, 3)] = KEY_4,
	[KP_INDEX(1, 4)] = KEY_CLEAR,

	[KP_INDEX(2, 0)] = KEY_6,
	[KP_INDEX(2, 1)] = KEY_5,
	[KP_INDEX(2, 2)] = KEY_8,
	[KP_INDEX(2, 3)] = KEY_3,
	[KP_INDEX(2, 4)] = KEY_NUMERIC_STAR,

	[KP_INDEX(3, 0)] = KEY_9,
	[KP_INDEX(3, 1)] = KEY_NUMERIC_POUND,
	[KP_INDEX(3, 2)] = KEY_0,
	[KP_INDEX(3, 3)] = KEY_2,
	[KP_INDEX(3, 4)] = KEY_SLEEP,

	[KP_INDEX(4, 0)] = KEY_BACK,
	[KP_INDEX(4, 1)] = KEY_HOME,
	[KP_INDEX(4, 2)] = KEY_MENU,
	[KP_INDEX(4, 3)] = KEY_VOLUMEUP,
	[KP_INDEX(4, 4)] = KEY_VOLUMEDOWN,
};

/* SURF keypad platform device information */
static struct gpio_event_matrix_info kp_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keymap,
	.output_gpios	= kp_row_gpios,
	.input_gpios	= kp_col_gpios,
	.noutputs	= ARRAY_SIZE(kp_row_gpios),
	.ninputs	= ARRAY_SIZE(kp_col_gpios),
	.settle_time.tv_nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv_nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS,
};

static struct gpio_event_info *kp_info[] = {
	&kp_matrix_info.info
};

static struct gpio_event_platform_data kp_pdata = {
	.name		= "7x27a_kp",
	.info		= kp_info,
	.info_count	= ARRAY_SIZE(kp_info)
};

static struct platform_device kp_pdev = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &kp_pdata,
	},
};
#endif
static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_pdev = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct platform_device msm_proccomm_regulator_dev = {
	.name   = PROCCOMM_REGULATOR_DEV_NAME,
	.id     = -1,
	.dev    = {
		.platform_data = &msm7x27a_proccomm_regulator_data
	}
};

static void __init msm7627a_rumi3_init(void)
{
	msm7x27a_init_ebi2();
	platform_add_devices(rumi_sim_devices,
			ARRAY_SIZE(rumi_sim_devices));
}

#define LED_GPIO_PDM		96
#define UART1DM_RX_GPIO		45

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static int __init msm7x27a_init_ar6000pm(void)
{
	msm_wlan_ar6000_pm_device.dev.platform_data = &ar600x_wlan_power;
	return platform_device_register(&msm_wlan_ar6000_pm_device);
}
#else
static int __init msm7x27a_init_ar6000pm(void) { return 0; }
#endif

static void __init msm7x27a_init_regulators(void)
{
	int rc = platform_device_register(&msm_proccomm_regulator_dev);
	if (rc)
		pr_err("%s: could not register regulator device: %d\n",
				__func__, rc);
}

static ssize_t synaptics_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
        memcpy( buf, buf_virtualkey, buf_vkey_size );
		return buf_vkey_size; 
}

static struct kobj_attribute synaptics_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics",
		.mode = S_IRUGO,
	},
	.show = &synaptics_virtual_keys_show,
};
static struct kobj_attribute melfas_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.melfas-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &synaptics_virtual_keys_show,
};
static struct attribute *synaptics_properties_attrs[] = {
	&synaptics_virtual_keys_attr.attr,
	&melfas_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group synaptics_properties_attr_group = {
	.attrs = synaptics_properties_attrs,
};

static void __init virtualkeys_init(void)
{
    struct kobject *properties_kobj;
    int ret;
    if(machine_is_msm7x27a_U8815())
    {
    	buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":57:850:100:80"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":240:850:100:80"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":423:850:100:80"
        		   "\n"); 
    }
    else if (machine_is_msm7x27a_C8820()
        || machine_is_msm7x27a_C8825D())
    {
    	buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":57:850:100:80"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":240:850:100:80"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":423:850:100:80"
        		   "\n"); 
    }
	else if (machine_is_msm7x27a_U8655())
    {
    	buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":30:510:60:50"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":165:510:100:50"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":290:510:60:50"
        		   "\n"); 
    }
    else if (machine_is_msm7x27a_U8655_EMMC())
    {
    	if (HW_VER_SUB_VE <= get_hw_sub_board_id())
    	{
    	    buf_vkey_size = sprintf(buf_virtualkey,
        		          __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":30:510:60:50"
        		          ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":115:510:80:50"
        		          ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":205:510:80:50"
        		          ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":290:510:60:50"
        		          "\n"); 
    	}
    	else
    	{
            buf_vkey_size = sprintf(buf_virtualkey,
        		          __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":30:510:60:50"
        		          ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":165:510:100:50"
        		          ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":290:510:60:50"
        		          "\n"); 
    	}
    }
	else if (machine_is_msm7x27a_M660())
    {
    	buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":30:505:60:40"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":165:505:100:40"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":290:505:60:40"
        		   "\n"); 
    }
	else if ( machine_is_msm7x27a_C8655_NAND())
    {
    	buf_vkey_size = sprintf(buf_virtualkey,
        	       __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":30:520:60:50"
        	       ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":165:520:100:50"
        	       ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":290:520:60:50"
        		   "\n"); 
    }
    else if (machine_is_msm7x27a_U8185())
    {
        buf_vkey_size = sprintf(buf_virtualkey,
                    __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":30:345:70:40"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":120:345:60:40"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":210:345:70:40"
        		   "\n");
    }
    else if (machine_is_msm7x27a_U8661())
    {
        buf_vkey_size = sprintf(buf_virtualkey,
        		   __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":35:520:70:60"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":118:520:70:60"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":205:520:70:60"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":285:520:70:60"
        		   "\n");            	
    }
    else if (machine_is_msm7x27a_C8668D())
    {
    	buf_vkey_size = sprintf(buf_virtualkey,
        	       __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":30:520:60:50"
        	       ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":160:520:100:50"
        	       ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":290:520:60:50"
        		   "\n"); 
    }
	else 
    {
    	buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":57:840:100:70"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":240:840:100:70"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":423:840:100:70"
        		   "\n"); 
    }
    
    properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
					 &synaptics_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");
}

static void __init msm7x2x_init(void)
{
	msm7x2x_misc_init();

	/* Initialize regulators first so that other devices can use them */
	msm7x27a_init_regulators();

	/* Common functions for SURF/FFA/RUMI3 */
	msm_device_i2c_init();
	msm7x27a_init_ebi2();
	msm7x27a_cfg_uart2dm_serial();
#ifdef CONFIG_HUAWEI_KERNEL
#if (defined(CONFIG_SERIAL_MSM_HS) && defined(CONFIG_HUAWEI_BT_WCN2243))
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(UART1DM_RX_GPIO);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
#else
#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(UART1DM_RX_GPIO);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
#endif

#ifdef CONFIG_HUAWEI_KERNEL
    import_kernel_cmdline();
#endif


#ifdef CONFIG_USB_MSM_OTG_72K
	msm_otg_pdata.swfi_latency =
		msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#endif
	msm_device_gadget_peripheral.dev.platform_data =
		&msm_gadget_pdata;
		#ifndef CONFIG_HUAWEI_CAMERA
		msm7x27a_cfg_smsc911x();
		#endif
		#if (defined(CONFIG_HUAWEI_BT_BCM43XX) && defined(CONFIG_HUAWEI_KERNEL))
                bt_wake_msm_config();
        #endif
	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);
	platform_add_devices(surf_ffa_devices,
			ARRAY_SIZE(surf_ffa_devices));
	/* Ensure ar6000pm device is registered before MMC/SDC */
	msm7x27a_init_ar6000pm();
#ifdef CONFIG_MMC_MSM
	msm7627a_init_mmc();
#endif
	msm_fb_add_devices();
#ifdef CONFIG_USB_EHCI_MSM_72K
	msm7x2x_init_host();
#endif

	msm_pm_set_platform_data(msm7x27a_pm_data,
				ARRAY_SIZE(msm7x27a_pm_data));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
	register_i2c_devices();
#endif
#if (defined(CONFIG_HUAWEI_BT_WCN2243) || (!defined(CONFIG_HUAWEI_KERNEL)))
	msm7627a_bt_power_init();
#endif
#if (defined(CONFIG_HUAWEI_BT_BCM43XX) && defined(CONFIG_HUAWEI_KERNEL))
	bt_bcm4330_power_init();
#endif
#ifndef CONFIG_HUAWEI_KERNEL
	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		atmel_ts_pdata.min_x = 0;
		atmel_ts_pdata.max_x = 480;
		atmel_ts_pdata.min_y = 0;
		atmel_ts_pdata.max_y = 320;
	}

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
		atmel_ts_i2c_info,
		ARRAY_SIZE(atmel_ts_i2c_info));

#else
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID, 
		msm7x27a_i2c_board_info,
		ARRAY_SIZE(msm7x27a_i2c_board_info));
#endif
#if defined(CONFIG_MSM_CAMERA)
	msm7627a_camera_init();
#endif
#if defined(CONFIG_HUAWEI_GPIO_KEYPAD)
    if(machine_is_msm7x27a_U8185())
    {
        platform_device_register(&keypad_device_u8185);
    }
	else 
    {
        platform_device_register(&keypad_device_default);
    }
#else
	platform_device_register(&kp_pdev);
#endif
	platform_device_register(&hs_pdev);

	/* configure it as a pdm function*/
	if (gpio_tlmm_config(GPIO_CFG(LED_GPIO_PDM, 3,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, LED_GPIO_PDM);
	else
		platform_device_register(&led_pdev);

#ifdef CONFIG_MSM_RPC_VIBRATOR
#ifndef CONFIG_HUAWEI_KERNEL
	if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa())
#endif
		msm_init_pmic_vibrator();
#endif
	/*7x25a kgsl initializations*/
	msm7x25a_kgsl_3d0_init();
	#ifdef CONFIG_HUAWEI_FEATURE_OEMINFO
    rmt_oeminfo_add_device();
	#endif

	virtualkeys_init();

#ifdef CONFIG_HUAWEI_KERNEL
    hw_extern_sdcard_add_device();
#endif
}

#ifdef CONFIG_HUAWEI_KERNEL
static struct resource hw_extern_sdcard_resources[] = {
    {
        .flags  = IORESOURCE_MEM,
    },
};

static struct platform_device hw_extern_sdcard_device = {
    .name           = "hw_extern_sdcard",
    .id             = -1,
    .num_resources  = ARRAY_SIZE(hw_extern_sdcard_resources),
    .resource       = hw_extern_sdcard_resources,
};

static struct resource hw_extern_sdcardMounted_resources[] = {
    {
        .flags  = IORESOURCE_MEM,
    },
};

static struct platform_device hw_extern_sdcardMounted_device = {
    .name           = "hw_extern_sdcardMounted",
    .id             = -1,
    .num_resources  = ARRAY_SIZE(hw_extern_sdcardMounted_resources),
    .resource       = hw_extern_sdcardMounted_resources,
};

int __init hw_extern_sdcard_add_device(void)
{
    platform_device_register(&hw_extern_sdcard_device);
    platform_device_register(&hw_extern_sdcardMounted_device);
    return 0;
}
#endif


static void __init msm7x2x_init_early(void)
{
	msm_msm7x2x_allocate_memory_regions();
}

MACHINE_START(MSM7X27A_RUMI3, "QCT MSM7x27a RUMI3")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7627a_rumi3_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_SURF, "QCT MSM7x27a SURF")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_FFA, "QCT MSM7x27a FFA")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7625A_SURF, "QCT MSM7625a SURF")
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = msm_common_io_init,
	.reserve        = msm7x27a_reserve,
	.init_irq       = msm_init_irq,
	.init_machine   = msm7x2x_init,
	.timer          = &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7625A_FFA, "QCT MSM7625a FFA")
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = msm_common_io_init,
	.reserve        = msm7x27a_reserve,
	.init_irq       = msm_init_irq,
	.init_machine   = msm7x2x_init,
	.timer          = &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_UMTS, "MSM7x27a UMTS SURF")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_CDMA, "MSM7x27a CDMA SURF")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_U8815, "MSM7x27a U8815 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_U8655, "MSM7x27a U8655 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_U8655_EMMC, "MSM7x27a U8655Pro BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_C8655_NAND, "MSM7x27a C8655_NAND BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_U8185, "MSM7x27a U8185 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END

MACHINE_START(MSM7X27A_M660, "MSM7x27a M660 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END

MACHINE_START(MSM7X27A_C8820, "MSM7x27a C8820 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END

MACHINE_START(MSM7X27A_C8825D, "MSM7x27a C8825D BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_U8661, "MSM7x27a U8661 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_C8668D, "MSM7x27a C8668D BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
