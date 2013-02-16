/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
#include <mach/usb_gadget_fserial.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_serial_hs.h>
#include <linux/usb/android_composite.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#ifdef CONFIG_HUAWEI_KERNEL
#include <asm-arm/huawei/smem_vendor_huawei.h>
#include "smd_private.h"
#endif

#ifdef CONFIG_HUAWEI_WIFI_SDCC
#include <linux/wifi_tiwlan.h>
#include <linux/skbuff.h>
#endif
#include <asm/mach/mmc.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/mfd/marimba.h>
#include <mach/vreg.h>
#include <linux/power_supply.h>
#include <mach/rpc_pmapp.h>

#include <mach/msm_battery.h>
#include <linux/smsc911x.h>
#include <linux/atmel_maxtouch.h>
#include "devices.h"
#include "timer.h"
#include "devices-msm7x2xa.h"
#include "pm.h"
#include <mach/rpc_server_handset.h>
//add keypad driver
#include "msm-keypad-devices.h"
#include <mach/socinfo.h>

/* include header file */
#ifdef CONFIG_HUAWEI_SYNAPTICS_RMI_TOUCH
#ifdef CONFIG_RMI4_I2C
#include <linux/interrupt.h>
#include <linux/rmi.h>
#endif
#endif
#ifdef CONFIG_HUAWEI_KERNEL
#include <linux/hardware_self_adapt.h>
/*added for virtualkeys*/
static char buf_virtualkey[500];
static ssize_t  buf_vkey_size=0;
#endif

#ifdef CONFIG_HUAWEI_CAMERA
#define S5K5CA_AND_MT9T113_IS_NOT_ON 0 
#define S5K5CA_OR_MT9T113_IS_ON 1
/*we use the variable to sign if s5k5ca or mt9t113 is on*/
static int s5k5ca_or_mt9t113_is_on = S5K5CA_AND_MT9T113_IS_NOT_ON;
#endif
char  back_camera_name[128];
char  front_camera_name[128];
#ifdef CONFIG_USB_AUTO_INSTALL
#include "../../../drivers/usb/gadget/usb_switch_huawei.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"

#define USB_SERIAL_LEN 20
/* keep the parameters transmitted from SMEM */
smem_huawei_vender usb_para_data;
/* keep usb parameters transfered from modem */
app_usb_para usb_para_info;
/* all the pid used by mobile */
usb_pid_stru usb_pid_array[]={
    {PID_ONLY_CDROM,     PID_NORMAL,     PID_UDISK, PID_AUTH,     PID_GOOGLE, PID_WLAN}, /* for COMMON products */
    {PID_ONLY_CDROM_TMO, PID_NORMAL_TMO, PID_UDISK, PID_AUTH_TMO, PID_GOOGLE, PID_WLAN}, /* for TMO products */
};
/* pointer to the member of usb_pid_array[], according to the current product */
usb_pid_stru *curr_usb_pid_ptr = &usb_pid_array[0];
#endif  

#include <linux/audio_amplifier.h>
#ifdef CONFIG_HUAWEI_NFC_PN544
#include <linux/nfc/pn544.h>
#endif

#define PMEM_KERNEL_EBI1_SIZE	0x3A000
#define MSM_PMEM_AUDIO_SIZE	0x5B000
#define BAHAMA_SLAVE_ID_FM_ADDR         0x2A
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B
#define BAHAMA_SLAVE_ID_FM_REG 0x02
#define FM_GPIO	83

enum {
	GPIO_EXPANDER_IRQ_BASE	= NR_MSM_IRQS + NR_GPIO_IRQS,
	GPIO_EXPANDER_GPIO_BASE	= NR_MSM_GPIOS,
	/* SURF expander */
	GPIO_CORE_EXPANDER_BASE	= GPIO_EXPANDER_GPIO_BASE,
	GPIO_BT_SYS_REST_EN	= GPIO_CORE_EXPANDER_BASE,
	GPIO_WLAN_EXT_POR_N,
	GPIO_DISPLAY_PWR_EN,
	GPIO_BACKLIGHT_EN,
	GPIO_PRESSURE_XCLR,
	GPIO_VREG_S3_EXP,
	GPIO_UBM2M_PWRDWN,
	GPIO_ETM_MODE_CS_N,
	GPIO_HOST_VBUS_EN,
	GPIO_SPI_MOSI,
	GPIO_SPI_MISO,
	GPIO_SPI_CLK,
	GPIO_SPI_CS0_N,
	GPIO_CORE_EXPANDER_IO13,
	GPIO_CORE_EXPANDER_IO14,
	GPIO_CORE_EXPANDER_IO15,
	/* Camera expander */
	GPIO_CAM_EXPANDER_BASE	= GPIO_CORE_EXPANDER_BASE + 16,
	GPIO_CAM_GP_STROBE_READY	= GPIO_CAM_EXPANDER_BASE,
	GPIO_CAM_GP_AFBUSY,
	GPIO_CAM_GP_CAM_PWDN,
	GPIO_CAM_GP_CAM1MP_XCLR,
	GPIO_CAM_GP_CAMIF_RESET_N,
	GPIO_CAM_GP_STROBE_CE,
	GPIO_CAM_GP_LED_EN1,
	GPIO_CAM_GP_LED_EN2,
};

#ifdef HUAWEI_BT_WCN2243
#define GPIO_BT_SYS_REST 5
#endif


/* delet one line */

#ifdef CONFIG_HUAWEI_KERNEL
#ifdef HUAWEI_BT_BCM4330
/* BCM BT GPIOs config*/
#define GPIO_BT_UART_RTS   43 
#define GPIO_BT_UART_CTS   44
#define GPIO_BT_RX         45
#define GPIO_BT_TX         46

/*wake signals*/
#define GPIO_BT_WAKE_BT    107
#define GPIO_BT_WAKE_MSM   83

/*control signals*/
#define GPIO_BT_SHUTDOWN_N 5
#define GPIO_BT_RESET_N    14

/*pcm signals*/
#define GPIO_BT_PCM_OUT   68 
#define GPIO_BT_PCM_IN   69
#define GPIO_BT_PCM_SYNC         70
#define GPIO_BT_PCM_CLK         71

/*gpio function*/
#define GPIO_BT_FUN_0        0
#define GPIO_BT_FUN_1        1 
#define GPIO_BT_FUN_2        2 
#define GPIO_BT_ON           1
#define GPIO_BT_OFF          0
#endif
#endif

#if defined(CONFIG_GPIO_SX150X)
enum {
	SX150X_CORE,
	SX150X_CAM,
};

static struct sx150x_platform_data sx150x_data[] __initdata = {
	[SX150X_CORE]	= {
		.gpio_base		= GPIO_CORE_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0,
		.io_open_drain_ena	= 0xfef8,
		.irq_summary		= -1,
	},
	[SX150X_CAM]	= {
		.gpio_base		= GPIO_CAM_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0,
		.io_open_drain_ena	= 0x23,
		.irq_summary		= -1,
	},
};
#endif

compass_gs_position_type  get_compass_gs_position(void)
{
	compass_gs_position_type compass_gs_position=COMPASS_TOP_GS_TOP;
	/* modify compass and gs position by board id */
    //move C8820\25D define from TOP to BOTTOM
	if (machine_is_msm7x27a_surf() || machine_is_msm7x27a_ffa() ||
		machine_is_msm7x27a_umts() || machine_is_msm7x27a_cdma()||
		machine_is_msm7x27a_U8815()
		) 
	{
		compass_gs_position=COMPASS_TOP_GS_TOP;
	}
	/*version A and version B has compass, since version C don't have compass*/
	else if(machine_is_msm7x27a_C8820() && (HW_VER_SUB_VC <= get_hw_sub_board_id()))
	{
		compass_gs_position=COMPASS_NONE_GS_BOTTOM;
	}
	/* add U8655_EMMC, use the u8655 configuration */
    else if (machine_is_msm7x27a_U8655() ||
             machine_is_msm7x27a_U8655_EMMC()||
             machine_is_msm7x27a_C8655_NAND() ||
             machine_is_msm7x27a_M660()||
             machine_is_msm7x27a_U8661()||
             machine_is_msm7x27a_C8820()||
             machine_is_msm7x27a_C8825D())
	{
		compass_gs_position=COMPASS_BOTTOM_GS_BOTTOM;
	}
    else if (machine_is_msm7x27a_U8185() )	
	{
		compass_gs_position=COMPASS_NONE_GS_TOP;
	}
	else	
	{
		compass_gs_position=COMPASS_TOP_GS_TOP;
	}
	return compass_gs_position;
}
	/* FM Platform power and shutdown routines */
#define FPGA_MSM_CNTRL_REG2 0x90008010
/* Recover QC original code if QC BT chip (WCN2243) is used. */
#if (defined(HUAWEI_BT_WCN2243) || (!defined(CONFIG_HUAWEI_KERNEL)))
static void config_pcm_i2s_mode(int mode)
{
	void __iomem *cfg_ptr;
	u8 reg2;

	cfg_ptr = ioremap_nocache(FPGA_MSM_CNTRL_REG2, sizeof(char));

	if (!cfg_ptr)
		return;
	if (mode) {
		/*enable the pcm mode in FPGA*/
		reg2 = readb_relaxed(cfg_ptr);
		if (reg2 == 0) {
			reg2 = 1;
			writeb_relaxed(reg2, cfg_ptr);
		}
	} else {
		/*enable i2s mode in FPGA*/
		reg2 = readb_relaxed(cfg_ptr);
		if (reg2 == 1) {
			reg2 = 0;
			writeb_relaxed(reg2, cfg_ptr);
		}
	}
	iounmap(cfg_ptr);
}

static unsigned fm_i2s_config_power_on[] = {
	/*FM_I2S_SD*/
	GPIO_CFG(68, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*FM_I2S_WS*/
	GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*FM_I2S_SCK*/
	GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static unsigned fm_i2s_config_power_off[] = {
	/*FM_I2S_SD*/
	GPIO_CFG(68, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*FM_I2S_WS*/
	GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*FM_I2S_SCK*/
	GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static unsigned bt_config_power_on[] = {
	/*RFR*/
	GPIO_CFG(43, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*CTS*/
	GPIO_CFG(44, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*RX*/
	GPIO_CFG(45, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*TX*/
	GPIO_CFG(46, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static unsigned bt_config_pcm_on[] = {
	/*PCM_DOUT*/
	GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_DIN*/
	GPIO_CFG(69, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_SYNC*/
	GPIO_CFG(70, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_CLK*/
	GPIO_CFG(71, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static unsigned bt_config_power_off[] = {
	/*RFR*/
	GPIO_CFG(43, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*CTS*/
	GPIO_CFG(44, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*RX*/
	GPIO_CFG(45, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*TX*/
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};
static unsigned bt_config_pcm_off[] = {
	/*PCM_DOUT*/
	GPIO_CFG(68, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_DIN*/
	GPIO_CFG(69, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_SYNC*/
	GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_CLK*/
	GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

/* Add for WCN2243 BT_SYS_RST_N */
#ifdef CONFIG_HUAWEI_KERNEL
static unsigned bt_config_sys_rest[] = {
	GPIO_CFG(5, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};
#endif

static int config_i2s(int mode)
{
	int pin, rc = 0;

	if (mode == FM_I2S_ON) {
#ifndef CONFIG_HUAWEI_KERNEL
		if (machine_is_msm7x27a_surf())
#else
/* delete the judgement of board_id, this is public platform code.
 * delete 3 row.
 */
#endif
			config_pcm_i2s_mode(0);
		pr_err("%s mode = FM_I2S_ON", __func__);
		for (pin = 0; pin < ARRAY_SIZE(fm_i2s_config_power_on);
			pin++) {
				rc = gpio_tlmm_config(
					fm_i2s_config_power_on[pin],
					GPIO_CFG_ENABLE
					);
				if (rc < 0)
					return rc;
			}
	} else if (mode == FM_I2S_OFF) {
		pr_err("%s mode = FM_I2S_OFF", __func__);
		for (pin = 0; pin < ARRAY_SIZE(fm_i2s_config_power_off);
			pin++) {
				rc = gpio_tlmm_config(
					fm_i2s_config_power_off[pin],
					GPIO_CFG_ENABLE
					);
				if (rc < 0)
					return rc;
			}
	}
	return rc;
}
static int config_pcm(int mode)
{
	int pin, rc = 0;

	if (mode == BT_PCM_ON) {
#ifndef CONFIG_HUAWEI_KERNEL
		if (machine_is_msm7x27a_surf())
#else
/* delete the judgement of board_id, this is public platform code.
 * delete 3 row.
 */
#endif
			config_pcm_i2s_mode(1);
		pr_err("%s mode =BT_PCM_ON", __func__);
		for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_on);
			pin++) {
				rc = gpio_tlmm_config(bt_config_pcm_on[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0)
					return rc;
			}
	} else if (mode == BT_PCM_OFF) {
		pr_err("%s mode =BT_PCM_OFF", __func__);
		for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_off);
			pin++) {
				rc = gpio_tlmm_config(bt_config_pcm_off[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0)
					return rc;
			}

	}

	return rc;
}

static int msm_bahama_setup_pcm_i2s(int mode)
{
	int fm_state = 0, bt_state = 0;
	int rc = 0;
	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	fm_state = marimba_get_fm_status(&config);
	bt_state = marimba_get_bt_status(&config);

	switch (mode) {
	case BT_PCM_ON:
	case BT_PCM_OFF:
		if (!fm_state)
			rc = config_pcm(mode);
		break;
	case FM_I2S_ON:
		rc = config_i2s(mode);
		break;
	case FM_I2S_OFF:
		if (bt_state)
			rc = config_pcm(BT_PCM_ON);
		else
			rc = config_i2s(mode);
		break;
	default:
		rc = -EIO;
		pr_err("%s:Unsupported mode", __func__);
	}
	return rc;
}

static int bt_set_gpio(int on)
{
	int rc = 0;
	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};
	
#ifdef CONFIG_HUAWEI_KERNEL
	if (on) {
		rc = gpio_direction_output(GPIO_BT_SYS_REST, 1);
		msleep(100);
	} else {
		if (!marimba_get_fm_status(&config) &&
				!marimba_get_bt_status(&config)) {
			gpio_set_value_cansleep(GPIO_BT_SYS_REST, 0);
			rc = gpio_direction_input(GPIO_BT_SYS_REST);
			msleep(100);
		}
	}
#else
    if (on) {
		rc = gpio_direction_output(GPIO_BT_SYS_REST_EN, 1);
		msleep(100);
	} else {
		if (!marimba_get_fm_status(&config) &&
				!marimba_get_bt_status(&config)) {
			gpio_set_value_cansleep(GPIO_BT_SYS_REST_EN, 0);
			rc = gpio_direction_input(GPIO_BT_SYS_REST_EN);
			msleep(100);
		}
	}
#endif	
	if (rc)
		pr_err("%s: BT sys_reset_en GPIO : Error", __func__);

	return rc;
}
static struct vreg *fm_regulator;
static int fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc = 0;
	const char *id = "FMPW";
	uint32_t irqcfg;
	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};
	u8 value;

	/* Voting for 1.8V Regulator */
	fm_regulator = vreg_get(NULL , "msme1");
	if (IS_ERR(fm_regulator)) {
		pr_err("%s: vreg get failed with : (%ld)\n",
			__func__, PTR_ERR(fm_regulator));
		return -EINVAL;
	}

	/* Set the voltage level to 1.8V */
	rc = vreg_set_level(fm_regulator, 1800);
	if (rc < 0) {
		pr_err("%s: set regulator level failed with :(%d)\n",
			__func__, rc);
		goto fm_vreg_fail;
	}

	/* Enabling the 1.8V regulator */
	rc = vreg_enable(fm_regulator);
	if (rc) {
		pr_err("%s: enable regulator failed with :(%d)\n",
			__func__, rc);
		goto fm_vreg_fail;
	}

	/* Voting for 19.2MHz clock */
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
			PMAPP_CLOCK_VOTE_ON);
	if (rc < 0) {
		pr_err("%s: clock vote failed with :(%d)\n",
			 __func__, rc);
		goto fm_clock_vote_fail;
	}

	rc = bt_set_gpio(1);
	if (rc) {
		pr_err("%s: bt_set_gpio = %d", __func__, rc);
		goto fm_gpio_config_fail;
	}
	/*re-write FM Slave Id, after reset*/
	value = BAHAMA_SLAVE_ID_FM_ADDR;
	rc = marimba_write_bit_mask(&config,
			BAHAMA_SLAVE_ID_FM_REG, &value, 1, 0xFF);
	if (rc < 0) {
		pr_err("%s: FM Slave ID rewrite Failed = %d", __func__, rc);
		goto fm_gpio_config_fail;
	}
	/* Configuring the FM GPIO */
	irqcfg = GPIO_CFG(FM_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA);

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			 __func__, irqcfg, rc);
		goto fm_gpio_config_fail;
	}

	return 0;

fm_gpio_config_fail:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
		PMAPP_CLOCK_VOTE_OFF);
	bt_set_gpio(0);
fm_clock_vote_fail:
	vreg_disable(fm_regulator);

fm_vreg_fail:
	vreg_put(fm_regulator);

	return rc;
};

static void fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	const char *id = "FMPW";

	/* Releasing the GPIO line used by FM */
	uint32_t irqcfg = GPIO_CFG(FM_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
		GPIO_CFG_2MA);

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			 __func__, irqcfg, rc);

	/* Releasing the 1.8V Regulator */
	if (fm_regulator != NULL) {
		rc = vreg_disable(fm_regulator);
		if (rc)
			pr_err("%s: disable regulator failed:(%d)\n",
				__func__, rc);
		fm_regulator = NULL;
	}

	/* Voting off the clock */
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
		PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0)
		pr_err("%s: voting off failed with :(%d)\n",
			__func__, rc);
	rc = bt_set_gpio(0);
	if (rc)
		pr_err("%s: bt_set_gpio = %d", __func__, rc);
}

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.fm_setup = fm_radio_setup,
	.fm_shutdown = fm_radio_shutdown,
	.irq = MSM_GPIO_TO_INT(FM_GPIO),
	.vreg_s2 = NULL,
	.vreg_xo_out = NULL,
	/* Configuring the FM SoC as I2S Master */
	.is_fm_soc_i2s_master = true,
	.config_i2s_gpio = msm_bahama_setup_pcm_i2s,
};
#endif
static struct platform_device msm_wlan_ar6000_pm_device = {
	.name           = "wlan_ar6000_pm_dev",
	.id             = -1,
};

/* allocate atheros scatter buffer while system up */
#define AR6K_SCATTER_REQS	4
#define AR6K_SCATTER_SIZE	(18*1024)

static char *ath_scatter_buf[AR6K_SCATTER_REQS];

static int __init msm7x27a_init_ath_buf(void)
{
    int i = 0;

    pr_err("%s:  Enter. \n", __func__);

    for (i = 0; i < AR6K_SCATTER_REQS; i ++)
    {
        ath_scatter_buf[i] = kmalloc(AR6K_SCATTER_SIZE, GFP_KERNEL);

        if (ath_scatter_buf[i] == NULL)
        {
            pr_err("%s:  No buffer. \n", __func__);
            return -ENOMEM;
        }

    }

    return 0;
}

/* ath_scatter_buf_get: called by ar6005 driver, for get pointer of memory allocated while system up.  */
char *ath_scatter_buf_get(int size, int index)
{
    if (size > AR6K_SCATTER_SIZE)
    {
        pr_err("%s: scatter request size=%d, larger than AR6K_SCATTER_SIZE=%d \n", __func__, size, AR6K_SCATTER_SIZE);
        return NULL;
    }

    if (index >= AR6K_SCATTER_REQS)
    {
        pr_err("%s: scatter request index=%d, larger than AR6K_SCATTER_REQS=%d \n", __func__, index, AR6K_SCATTER_REQS);
        return NULL;
        }

    if (ath_scatter_buf[index] == NULL)
    {
        pr_err("%s: ath_scatter_buf[index] == NULL, Error Error!. \n", __func__);
        return NULL;
    }

    pr_err("%s: scatter request index=%d, buff addr=0x%p. \n", __func__, index, ath_scatter_buf[index]);

    return ath_scatter_buf[index];
}

EXPORT_SYMBOL(ath_scatter_buf_get);

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)

/* Recover QC code if QC BT chip (WCN2243) is used. */
#if (defined(HUAWEI_BT_WCN2243) || (!defined(CONFIG_HUAWEI_KERNEL)))

static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};
struct bahama_config_register {
		u8 reg;
		u8 value;
		u8 mask;
};
struct bt_vreg_info {
	const char *name;
	unsigned int pmapp_id;
	unsigned int level;
	unsigned int is_pin_controlled;
	struct vreg *vregs;
};
static struct bt_vreg_info bt_vregs[] = {
	{"msme1", 2, 1800, 0, NULL},
	{"bt", 21, 2900, 1, NULL}
};

static int bahama_bt(int on)
{

	int rc = 0;
	int i;

	struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};

	struct bahama_variant_register {
		const size_t size;
		const struct bahama_config_register *set;
	};

	const struct bahama_config_register *p;

	u8 version;

	const struct bahama_config_register v10_bt_on[] = {
		{ 0xE9, 0x00, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xE4, 0x00, 0xFF },
		{ 0xE5, 0x00, 0x0F },
#ifdef CONFIG_WLAN
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
		{ 0x01, 0x0C, 0x1F },
		{ 0x01, 0x08, 0x1F },
	};

	const struct bahama_config_register v20_bt_on_fm_off[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xF0, 0x00, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0x8E, 0x15, 0xFF },
		{ 0x8F, 0x15, 0xFF },
		{ 0x90, 0x15, 0xFF },

		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v20_bt_on_fm_on[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0x7F },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF },
	};

	const struct bahama_config_register v10_bt_off[] = {
		{ 0xE9, 0x00, 0xFF },
	};

	const struct bahama_config_register v20_bt_off_fm_off[] = {
		{ 0xF4, 0x84, 0xFF },
		{ 0xF0, 0x04, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};

	const struct bahama_config_register v20_bt_off_fm_on[] = {
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};
	const struct bahama_variant_register bt_bahama[2][3] = {
	{
		{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
		{ ARRAY_SIZE(v20_bt_off_fm_off), v20_bt_off_fm_off },
		{ ARRAY_SIZE(v20_bt_off_fm_on), v20_bt_off_fm_on }
	},
	{
		{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
		{ ARRAY_SIZE(v20_bt_on_fm_off), v20_bt_on_fm_off },
		{ ARRAY_SIZE(v20_bt_on_fm_on), v20_bt_on_fm_on }
	}
	};

	u8 offset = 0; /* index into bahama configs */
	on = on ? 1 : 0;
	version = marimba_read_bahama_ver(&config);
	if ((int)version < 0 || version == BAHAMA_VER_UNSUPPORTED) {
		dev_err(&msm_bt_power_device.dev, "%s: Bahama \
				version read Error, version = %d \n",
				__func__, version);
		return -EIO;
	}

	if (version == BAHAMA_VER_2_0) {
		if (marimba_get_fm_status(&config))
			offset = 0x01;
	}

	p = bt_bahama[on][version + offset].set;

	dev_info(&msm_bt_power_device.dev,
		"%s: found version %d\n", __func__, version);

	for (i = 0; i < bt_bahama[on][version + offset].size; i++) {
		u8 value = (p+i)->value;
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			dev_err(&msm_bt_power_device.dev,
				"%s: reg %x write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		dev_dbg(&msm_bt_power_device.dev,
			"%s: reg 0x%02x write value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
		value = 0;
		rc = marimba_read_bit_mask(&config,
				(p+i)->reg, &value,
				sizeof((p+i)->value), (p+i)->mask);
		if (rc < 0)
			dev_err(&msm_bt_power_device.dev, "%s marimba_read_bit_mask- error",
					__func__);
		dev_dbg(&msm_bt_power_device.dev,
			"%s: reg 0x%02x read value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	/* Update BT Status */
	if (on)
		marimba_set_bt_status(&config, true);
	else
		marimba_set_bt_status(&config, false);
	return rc;
}
static int bluetooth_switch_regulators(int on)
{
	int i, rc = 0;
	const char *id = "BTPW";

	for (i = 0; i < ARRAY_SIZE(bt_vregs); i++) {
		if (!bt_vregs[i].vregs) {
			pr_err("%s: vreg_get %s failed(%d)\n",
			__func__, bt_vregs[i].name, rc);
			goto vreg_fail;
		}
		rc = on ? vreg_set_level(bt_vregs[i].vregs,
				bt_vregs[i].level) : 0;

		if (rc < 0) {
			pr_err("%s: vreg set level failed (%d)\n",
					__func__, rc);
			goto vreg_set_level_fail;
		}
		rc = on ? vreg_enable(bt_vregs[i].vregs) : 0;

		if (rc < 0) {
			pr_err("%s: vreg %s %s failed(%d)\n",
					__func__, bt_vregs[i].name,
					on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
		if (bt_vregs[i].is_pin_controlled == 1) {
#ifdef CONFIG_HUAWEI_KERNEL			
			rc = pmapp_vreg_pincntrl_vote(id,
					bt_vregs[i].pmapp_id,
					PMAPP_CLOCK_ID_D1,
					on ? PMAPP_CLOCK_VOTE_ON :
					PMAPP_CLOCK_VOTE_OFF);
#else
	        rc = pmapp_vreg_lpm_pincntrl_vote(id,
					bt_vregs[i].pmapp_id,
					PMAPP_CLOCK_ID_D1,
					on ? PMAPP_CLOCK_VOTE_ON :
					PMAPP_CLOCK_VOTE_OFF);
#endif					
			if (rc < 0) {
				pr_err("%s: vreg %s pin ctrl failed(%d)\n",
						__func__, bt_vregs[i].name,
						rc);
				goto pincntrl_fail;
			}
		}
		rc = on ? 0 : vreg_disable(bt_vregs[i].vregs);
		if (rc < 0) {
			pr_err("%s: vreg %s %s failed(%d)\n",
					__func__, bt_vregs[i].name,
					on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
	}

	return rc;
pincntrl_fail:
	if (on)
		vreg_disable(bt_vregs[i].vregs);
vreg_fail:
	while (i) {
		if (on)
			vreg_disable(bt_vregs[--i].vregs);
		}
vreg_set_level_fail:
	vreg_put(bt_vregs[0].vregs);
	vreg_put(bt_vregs[1].vregs);
	return rc;
}

static unsigned int msm_bahama_setup_power(void)
{
	int rc = 0;
	struct vreg *vreg_s3 = NULL;

	vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(vreg_s3)) {
		pr_err("%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_s3));
		return PTR_ERR(vreg_s3);
	}
	rc = vreg_set_level(vreg_s3, 1800);
	if (rc < 0) {
		pr_err("%s: vreg set level failed (%d)\n",
				__func__, rc);
		goto vreg_fail;
	}
	rc = vreg_enable(vreg_s3);
	if (rc < 0) {
		pr_err("%s: vreg enable failed (%d)\n",
		       __func__, rc);
		goto vreg_fail;
	}

/* configure GPIO_BT_SYS_REST as output */
#ifdef CONFIG_HUAWEI_KERNEL	
    rc = gpio_tlmm_config(bt_config_sys_rest[0],
					GPIO_CFG_ENABLE);
	if (rc < 0) 
	{
		pr_err("%s: gpio_tlmm_config %d\n", __func__, rc);
		goto vreg_fail;
	}	
	/*setup Bahama_sys_reset_n*/
	rc = gpio_request(GPIO_BT_SYS_REST, "bahama sys_rst_n");    
	if (rc < 0) {
		pr_err("%s: gpio_request %d = %d\n", __func__,
			GPIO_BT_SYS_REST, rc);
		goto vreg_fail;
	}
	rc = bt_set_gpio(1);
	if (rc < 0) {
		pr_err("%s: bt_set_gpio %d = %d\n", __func__,
			GPIO_BT_SYS_REST, rc);
		goto gpio_fail;
	}
	return rc;

gpio_fail:
	gpio_free(GPIO_BT_SYS_REST);
#else
	/*setup Bahama_sys_reset_n*/
	rc = gpio_request(GPIO_BT_SYS_REST_EN, "bahama sys_rst_n");
	if (rc < 0) {
		pr_err("%s: gpio_request %d = %d\n", __func__,
			GPIO_BT_SYS_REST_EN, rc);
		goto vreg_fail;
	}
	rc = bt_set_gpio(1);
	if (rc < 0) {
		pr_err("%s: bt_set_gpio %d = %d\n", __func__,
			GPIO_BT_SYS_REST_EN, rc);
		goto gpio_fail;
	}
	return rc;

gpio_fail:
	gpio_free(GPIO_BT_SYS_REST_EN);

#endif	
vreg_fail:
	vreg_put(vreg_s3);
	return rc;
}

static unsigned int msm_bahama_shutdown_power(int value)
{
	int rc = 0;
	struct vreg *vreg_s3 = NULL;

	vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(vreg_s3)) {
		pr_err("%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_s3));
		return PTR_ERR(vreg_s3);
	}
	rc = vreg_disable(vreg_s3);
	if (rc) {
		pr_err("%s: vreg disable failed (%d)\n",
		       __func__, rc);
		vreg_put(vreg_s3);
		return rc;
	}
	if (value == BAHAMA_ID) {
		rc = bt_set_gpio(0);
		if (rc) {
			pr_err("%s: bt_set_gpio = %d\n",
					__func__, rc);
		}
	}
	return rc;
}

static unsigned int msm_bahama_core_config(int type)
{
	int rc = 0;

	if (type == BAHAMA_ID) {
		int i;
		struct marimba config = { .mod_id =  SLAVE_ID_BAHAMA};
		const struct bahama_config_register v20_init[] = {
			/* reg, value, mask */
			{ 0xF4, 0x84, 0xFF }, /* AREG */
			{ 0xF0, 0x04, 0xFF } /* DREG */
		};
		if (marimba_read_bahama_ver(&config) == BAHAMA_VER_2_0) {
			for (i = 0; i < ARRAY_SIZE(v20_init); i++) {
				u8 value = v20_init[i].value;
				rc = marimba_write_bit_mask(&config,
					v20_init[i].reg,
					&value,
					sizeof(v20_init[i].value),
					v20_init[i].mask);
				if (rc < 0) {
					pr_err("%s: reg %d write failed: %d\n",
						__func__, v20_init[i].reg, rc);
					return rc;
				}
				pr_debug("%s: reg 0x%02x value 0x%02x"
					" mask 0x%02x\n",
					__func__, v20_init[i].reg,
					v20_init[i].value, v20_init[i].mask);
			}
		}
	}
	rc = bt_set_gpio(0);
	if (rc) {
		pr_err("%s: bt_set_gpio = %d\n",
		       __func__, rc);
	}
	pr_debug("core type: %d\n", type);
	return rc;
}

static int bluetooth_power(int on)
{
	int pin, rc = 0;
	const char *id = "BTPW";
	int cid = 0;

	cid = adie_get_detected_connectivity_type();
	if (cid != BAHAMA_ID) {
		pr_err("%s: unexpected adie connectivity type: %d\n",
					__func__, cid);
		return -ENODEV;
	}
	if (on) {
#ifdef CONFIG_HUAWEI_KERNEL		
        rc = gpio_tlmm_config(bt_config_sys_rest[0],
					GPIO_CFG_ENABLE);
		if (rc < 0) 
	    {
		    pr_err("%s: gpio_tlmm_config %d\n", __func__, rc);
		    goto exit;
	    }
#endif		
		/*setup power for BT SOC*/
		rc = bt_set_gpio(on);
		if (rc) {
			pr_err("%s: bt_set_gpio = %d\n",
					__func__, rc);
			goto exit;
		}
		rc = bluetooth_switch_regulators(on);
		if (rc < 0) {
			pr_err("%s: bluetooth_switch_regulators rc = %d",
					__func__, rc);
			goto exit;
		}
		/*setup BT GPIO lines*/
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on);
			pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],
					GPIO_CFG_ENABLE);
			if (rc < 0) {
				pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
						__func__,
						bt_config_power_on[pin],
						rc);
				goto fail_power;
			}
		}
		/*Setup BT clocks*/
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
			PMAPP_CLOCK_VOTE_ON);
		if (rc < 0) {
			pr_err("Failed to vote for TCXO_D1 ON\n");
			goto fail_clock;
		}
		msleep(20);

		/*I2C config for Bahama*/
		rc = bahama_bt(1);
		if (rc < 0) {
			pr_err("%s: bahama_bt rc = %d", __func__, rc);
			goto fail_i2c;
		}
		msleep(20);

		/*setup BT PCM lines*/
		rc = msm_bahama_setup_pcm_i2s(BT_PCM_ON);
		if (rc < 0) {
			pr_err("%s: msm_bahama_setup_pcm_i2s , rc =%d\n",
				__func__, rc);
				goto fail_power;
			}
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
				  PMAPP_CLOCK_VOTE_PIN_CTRL);
		if (rc < 0)
			pr_err("%s:Pin Control Failed, rc = %d",
					__func__, rc);

	} else {
		rc = bahama_bt(0);
		if (rc < 0)
			pr_err("%s: bahama_bt rc = %d", __func__, rc);

		rc = bt_set_gpio(on);
		if (rc) {
			pr_err("%s: bt_set_gpio = %d\n",
					__func__, rc);
		}
fail_i2c:
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
				  PMAPP_CLOCK_VOTE_OFF);
		if (rc < 0)
			pr_err("%s: Failed to vote Off D1\n", __func__);
fail_clock:
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off);
			pin++) {
				rc = gpio_tlmm_config(bt_config_power_off[pin],
					GPIO_CFG_ENABLE);
				if (rc < 0) {
					pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_off[pin], rc);
				}
			}
		rc = msm_bahama_setup_pcm_i2s(BT_PCM_OFF);
		if (rc < 0) {
			pr_err("%s: msm_bahama_setup_pcm_i2s, rc =%d\n",
					__func__, rc);
				}
fail_power:
		rc = bluetooth_switch_regulators(0);
		if (rc < 0) {
			pr_err("%s: switch_regulators : rc = %d",\
					__func__, rc);
			goto exit;
		}
	}
	return rc;
exit:
	pr_err("%s: failed with rc = %d", __func__, rc);
	return rc;
}

static int __init bt_power_init(void)
{
	int i, rc = 0;
	for (i = 0; i < ARRAY_SIZE(bt_vregs); i++) {
			bt_vregs[i].vregs = vreg_get(NULL,
					bt_vregs[i].name);
			if (IS_ERR(bt_vregs[i].vregs)) {
				pr_err("%s: vreg get %s failed (%ld)\n",
				       __func__, bt_vregs[i].name,
				       PTR_ERR(bt_vregs[i].vregs));
				rc = PTR_ERR(bt_vregs[i].vregs);
				goto vreg_get_fail;
			}
		}

	msm_bt_power_device.dev.platform_data = &bluetooth_power;

	return rc;

vreg_get_fail:
	while (i)
		vreg_put(bt_vregs[--i].vregs);
	return rc;
}
#endif
#ifdef CONFIG_HUAWEI_FEATURE_TPA2028D1_AMPLIFIER
static struct amplifier_platform_data audio_amplifier_data = {
    .amplifier_on = NULL,
    .amplifier_off = NULL, 
};
#endif
/* the following definition only used for WCN2243 */
#if (defined(HUAWEI_BT_WCN2243) || (!defined(CONFIG_HUAWEI_KERNEL)))
static struct marimba_platform_data marimba_pdata = {
	.slave_id[SLAVE_ID_BAHAMA_FM]        = BAHAMA_SLAVE_ID_FM_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST]  = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
/* Recover QC code if QC BT chip (WCN2243) is used. */
/* #if (defined(HUAWEI_BT_WCN2243) || (!defined(CONFIG_HUAWEI_KERNEL))) */
	.bahama_setup                        = msm_bahama_setup_power,
	.bahama_shutdown                     = msm_bahama_shutdown_power,
	.bahama_core_config                  = msm_bahama_core_config,
	.fm				     = &marimba_fm_pdata,
/* #endif */
};
#endif
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ST_LSM303DLH
static int gsensor_support_dummyaddr(void)
{
    int ret = -1;	/*default value means actual address*/
    ret = (int)GS_ST303DLH;
    return ret;
}
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
/* add leds,button-backlight,pmic-leds device */
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
			
static int gs_init_flag = 0;   /*gsensor is not initialized*/
#ifdef CONFIG_HUAWEI_FEATURE_GYROSCOPE_L3G4200DH
static struct gyro_platform_data gy_l3g4200d_platform_data = {
    .gyro_power = NULL,
    .axis_map_x = 0,     /*x map read data[axis_map_x] from i2c*/
    .axis_map_y = 1,
    .axis_map_z = 2,	
    .negate_x = 0,       /*negative x,y or z*/
    .negate_y =0,
    .negate_z = 0,
    .slave_addr = 0x68,  	/*i2c slave address*/
    .dev_id = 0x0F,             /*WHO AM I*/
};
#endif

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_MMA8452
static struct gs_platform_data gs_mma8452_platform_data = {
    .adapt_fn = NULL,
    .slave_addr = (0x38 >> 1),  /*i2c slave address*/
    .dev_id = 0x2A,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
};
#endif

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ST_LSM303DLH
static struct gs_platform_data st303_gs_platform_data = {
    .adapt_fn = gsensor_support_dummyaddr,
    .slave_addr = (0x32 >> 1),  /*i2c slave address*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=NULL,
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
/*add kxtik's platform_data*/
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_KXTIK1004
static struct gs_platform_data gs_kxtik_platform_data = {
    .adapt_fn = gsensor_support_dummyaddr_kxtik,
    .slave_addr = (0x1E >> 1),  /*i2c slave address*/
    .dev_id = 0x05,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
};
#endif 
/* Code for BCM4330 */
#if defined(CONFIG_HUAWEI_KERNEL) && defined(HUAWEI_BT_BCM4330)
static struct platform_device msm_bt_power_device = {
    .name = "bt_power",
    .id     = -1
};

enum {
    BT_WAKE,
    BT_RFR,
    BT_CTS,
    BT_RX,
    BT_TX,
    BT_PCM_DOUT,
    BT_PCM_DIN,
    BT_PCM_SYNC,
    BT_PCM_CLK,
    BT_HOST_WAKE,
};
/* config all msm bt gpio here!*/

static struct msm_gpio bt_config_bcm4330_power_on[] = {
    { GPIO_CFG(GPIO_BT_UART_RTS, GPIO_BT_FUN_2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
        "UART1DM_RFR" },
    { GPIO_CFG(GPIO_BT_UART_CTS, GPIO_BT_FUN_2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
        "UART1DM_CTS" },
    { GPIO_CFG(GPIO_BT_RX, GPIO_BT_FUN_2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
        "UART1DM_Rx" },
    { GPIO_CFG(GPIO_BT_TX, GPIO_BT_FUN_2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
        "UART1DM_Tx" },
    /*following 2 are the wakeup between 4330 and MSM*/
    { GPIO_CFG(GPIO_BT_WAKE_BT, GPIO_BT_FUN_0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,  GPIO_CFG_2MA ),
        "MSM_WAKE_BT"  },
    { GPIO_CFG(GPIO_BT_WAKE_MSM, GPIO_BT_FUN_0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL ,  GPIO_CFG_2MA ),	
        "BT_WAKE_MSM"  },
    /*following 4 are the PCM between 4330 and MSM*/
    { GPIO_CFG(GPIO_BT_PCM_OUT, GPIO_BT_FUN_1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	
        "PCM_DOUT" },
    { GPIO_CFG(GPIO_BT_PCM_IN, GPIO_BT_FUN_1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	
        "PCM_DIN" },
    { GPIO_CFG(GPIO_BT_PCM_SYNC, GPIO_BT_FUN_1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	
        "PCM_SYNC" },
    { GPIO_CFG(GPIO_BT_PCM_CLK, GPIO_BT_FUN_1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	
        "PCM_CLK " }
};

static struct msm_gpio bt_config_power_control[] = {  
    /*following 2 are bt on/off control*/
    { GPIO_CFG(GPIO_BT_SHUTDOWN_N, GPIO_BT_FUN_0, GPIO_CFG_OUTPUT , GPIO_CFG_NO_PULL ,  GPIO_CFG_2MA ), 
        "BT_REG_ON"  },
    { GPIO_CFG(GPIO_BT_RESET_N, GPIO_BT_FUN_0, GPIO_CFG_OUTPUT , GPIO_CFG_NO_PULL ,  GPIO_CFG_2MA ), 
        "BT_PWR_ON"  }
};

static struct msm_gpio bt_config_bcm4330_power_off[] = {
    { GPIO_CFG(GPIO_BT_UART_RTS, GPIO_BT_FUN_0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
        "UART1DM_RFR" },
    { GPIO_CFG(GPIO_BT_UART_CTS, GPIO_BT_FUN_0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
        "UART1DM_CTS" },
    { GPIO_CFG(GPIO_BT_RX, GPIO_BT_FUN_0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
        "UART1DM_Rx" },
    { GPIO_CFG(GPIO_BT_TX, GPIO_BT_FUN_0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
        "UART1DM_Tx" },
    /*following 2 are the wakeup between 4330 and MSM*/
    { GPIO_CFG(GPIO_BT_WAKE_BT, GPIO_BT_FUN_0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN ,  GPIO_CFG_2MA ),
        "MSM_WAKE_BT"  },
    { GPIO_CFG(GPIO_BT_WAKE_MSM, GPIO_BT_FUN_0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN ,  GPIO_CFG_2MA ),	
        "BT_WAKE_MSM"  },
    /*following 4 are the PCM between 4330 and MSM*/
    { GPIO_CFG(GPIO_BT_PCM_OUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 
        "PCM_DOUT" },
    { GPIO_CFG(GPIO_BT_PCM_IN, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
        "PCM_DIN" },
    { GPIO_CFG(GPIO_BT_PCM_SYNC, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
        "PCM_SYNC" },
    { GPIO_CFG(GPIO_BT_PCM_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
     	"PCM_CLK" }
	};



/* configure all bt power here! */
static const char *vregs_bt_bcm4330_name[] = {
    "s3"
};

static struct vreg *vregs_bt_bcm4330[ARRAY_SIZE(vregs_bt_bcm4330_name)];

/* put power on for bt*/
static int bluetooth_bcm4330_power_regulators(int on)
{
    int i = 0;
    int rc = 0;

    for (i = 0; i < ARRAY_SIZE(vregs_bt_bcm4330_name); i++) {
        rc = on ? vreg_enable(vregs_bt_bcm4330[i]) :
            vreg_disable(vregs_bt_bcm4330[i]);
        if (rc < 0) {
        printk(KERN_ERR "%s: vreg %s %s failed (%d)\n",
            __func__, vregs_bt_bcm4330_name[i],
    			       on ? "enable" : "disable", rc);
        return -EIO;
        }
    }

    /*gpio power for bcm4330*/
    if(on)
    {

        rc = gpio_direction_output(GPIO_BT_SHUTDOWN_N, GPIO_BT_ON);  /*bt_reg_on off on :5 -->1*/
        if (rc) 
        {
            printk(KERN_ERR "%s:  bt power1 on fail (%d)\n",
                   __func__, rc);
            return -EIO;
        }
          
        mdelay(1);
        rc = gpio_direction_output(GPIO_BT_RESET_N, GPIO_BT_ON);  /*bt_pwr_on  on:14 -->1*/
        if (rc) 
        {
            printk(KERN_ERR "%s:  bt power2 off fail (%d)\n",
                   __func__, rc);
            return -EIO;
        }


    }
    else
    {
        rc = gpio_direction_output(GPIO_BT_RESET_N, GPIO_BT_OFF);  /*bt_pwr_on off:14 -->0*/
        if (rc) 
        {
            printk(KERN_ERR "%s:  bt power2 off fail (%d)\n",
                   __func__, rc);
            return -EIO;
        }
        mdelay(1);

        rc = gpio_direction_output(GPIO_BT_SHUTDOWN_N, GPIO_BT_OFF);  /*bt_reg_on :5 -->0*/
        if (rc) 
        {
            printk(KERN_ERR "%s:  bt power1 off fail (%d)\n",
                   __func__, rc);
            return -EIO;
        }
        mdelay(1);

    }		
    return 0;
}
	

static int bluetooth_bcm4330_power(int on)
{
    int rc = 0;

    if (on)
    {
        /* msm: config the msm  bt gpios*/
        rc = msm_gpios_enable(bt_config_bcm4330_power_on,
            ARRAY_SIZE(bt_config_bcm4330_power_on));
        if (rc < 0)
        {
            printk(KERN_ERR "%s: bcm4330_config_gpio on failed (%d)\n",
                __func__, rc);
            return rc;
        }

        rc = bluetooth_bcm4330_power_regulators(on);
        if (rc < 0) 
        {
            printk(KERN_ERR "%s: bcm4330_power_regulators on failed (%d)\n",
                __func__, rc);
            return rc;
        }
    }
    else
    {
        /* msm: config the msm  bt gpios*/
        rc = msm_gpios_enable(bt_config_bcm4330_power_off,
            ARRAY_SIZE(bt_config_bcm4330_power_off));
        if (rc < 0)
        {
            printk(KERN_ERR "%s: bcm4330_config_gpio on failed (%d)\n",
                __func__, rc);
            return rc;
        }

        /* check for initial rfkill block (power off) */
        if (platform_get_drvdata(&msm_bt_power_device) == NULL)
        {
            printk(KERN_DEBUG "bluetooth rfkill block error : \n");
            goto out;
        }
      
        rc = bluetooth_bcm4330_power_regulators(on);
        if (rc < 0) 
        {
            printk(KERN_ERR "%s: bcm4330_power_regulators off failed (%d)\n",
                __func__, rc);
            return rc;
        }
       


    }	
out:
    printk(KERN_DEBUG "Bluetooth BCM4330 power switch: %d\n", on);

    return 0;
}


	
static void __init bt_bcm4330_power_init(void)
{
    /*here will check the power, */
    int i = 0;
    int rc = -1;

    printk(KERN_ERR "bt_bcm4330_power_init pre\n");
		
    for (i = 0; i < ARRAY_SIZE(vregs_bt_bcm4330_name); i++)
    {
        vregs_bt_bcm4330[i] = vreg_get(NULL, vregs_bt_bcm4330_name[i]);
        if (IS_ERR(vregs_bt_bcm4330[i])) 
        {
            printk(KERN_ERR "%s: vreg get %s failed (%ld)\n",
                __func__, vregs_bt_bcm4330_name[i],
                PTR_ERR(vregs_bt_bcm4330[i]));
        return;
        }
    }
   
    /* handle bt power control: becareful */
    rc = msm_gpios_request_enable(bt_config_power_control,
                            ARRAY_SIZE(bt_config_power_control));
    if (rc < 0) {
            printk(KERN_ERR
                    "%s: bt power control request_enable failed (%d)\n",
                            __func__, rc);
            return;
    }
    
    rc = gpio_direction_output(GPIO_BT_RESET_N, GPIO_BT_OFF);  /*bt_pwr_on off:14 -->0*/
    if (rc) 
    {
        printk(KERN_ERR "%s:  bt power2 off fail (%d)\n",
               __func__, rc);
        return ;
    }
    mdelay(1);

    rc = gpio_direction_output(GPIO_BT_SHUTDOWN_N, GPIO_BT_OFF);  /*bt_reg_on :5 -->0*/
    if (rc) 
    {
        printk(KERN_ERR "%s:  bt power1 off fail (%d)\n",
               __func__, rc);
        return ;
    }
    mdelay(1);


    printk(KERN_ERR "bt_bcm4330_power_init after\n");
    /*config platform_data*/
    msm_bt_power_device.dev.platform_data = &bluetooth_bcm4330_power;
	
}

static struct resource bluesleep_resources[] = {
    {
    .name	= "gpio_host_wake",
    .start	= GPIO_BT_WAKE_MSM,
    .end	= GPIO_BT_WAKE_MSM,
    .flags	= IORESOURCE_IO,
    },
    {
    .name	= "gpio_ext_wake",
    .start	= GPIO_BT_WAKE_BT,
    .end	= GPIO_BT_WAKE_BT,
    .flags	= IORESOURCE_IO,
    },
    {
    .name	= "host_wake",
    .start	= MSM_GPIO_TO_INT(GPIO_BT_WAKE_MSM),
    .end	= MSM_GPIO_TO_INT(GPIO_BT_WAKE_MSM),
    .flags	= IORESOURCE_IRQ,
    },
};

static struct platform_device msm_bluesleep_device = {
    .name = "bluesleep",
    .id		= -1,
    .num_resources	= ARRAY_SIZE(bluesleep_resources),
    .resource	= bluesleep_resources,
};
#else
#define bt_bcm4330_power_init(x) do {} while (0)
#endif
#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
static struct i2c_board_info core_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
	},
};
static struct i2c_board_info cam_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data	= &sx150x_data[SX150X_CAM],
	},
};
#endif
#ifndef CONFIG_HUAWEI_KERNEL
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static struct i2c_board_info bahama_devices[] = {
{
	I2C_BOARD_INFO("marimba", 0xc),
	.platform_data = &marimba_pdata,
},
};
#endif
#else
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE) && defined(HUAWEI_BT_WCN2243)
static struct i2c_board_info bahama_devices[] = {
{
	/* I2C address changed from 0x1 to 0xc */
	I2C_BOARD_INFO("marimba", 0xc), 
	.platform_data = &marimba_pdata,
},
};
#endif
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
static void __init register_i2c_devices(void)
{

	i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID,
				cam_exp_i2c_info,
				ARRAY_SIZE(cam_exp_i2c_info));

#ifndef CONFIG_HUAWEI_KERNEL
	if (machine_is_msm7x27a_surf())
#else
/* delete the judgement of board_id, this is public platform code.
 * delete 3 row.
 */
#endif
		sx150x_data[SX150X_CORE].io_open_drain_ena = 0xe0f0;

	core_exp_i2c_info[0].platform_data =
			&sx150x_data[SX150X_CORE];

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				core_exp_i2c_info,
				ARRAY_SIZE(core_exp_i2c_info));
				
#ifndef CONFIG_HUAWEI_KERNEL
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				bahama_devices,
				ARRAY_SIZE(bahama_devices));
#endif
#else
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE) && defined(HUAWEI_BT_WCN2243)
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				bahama_devices,
				ARRAY_SIZE(bahama_devices));
#endif
#endif
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
	.clk			= "gsbi_qup_clk",
	.pclk			= "gsbi_qup_pclk",
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi1_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.clk			= "gsbi_qup_clk",
	.pclk			= "gsbi_qup_pclk",
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

#ifdef CONFIG_ARCH_MSM7X27A
#ifdef CONFIG_HUAWEI_KERNEL
/* set the mdp pmem same as 7x27 */
//#define MSM_PMEM_MDP_WVGA_SIZE  0x1780000
#define MSM_PMEM_MDP_HVGA_SIZE  0x0C10000
#define MSM_PMEM_MDP_QVGA_SIZE  0x0910000
/* reduce ADSP PMEM from 16M to 11M */
#define MSM_PMEM_ADSP_SIZE_FOR_256M     0xB91000 
#define MEM_SIZE_256M_DESC_IN_CMDLINE   "mem.size=256"
#endif
#define MSM_PMEM_MDP_SIZE       0x1DD1000
#define MSM7x25A_MSM_PMEM_MDP_SIZE	0x1000000

#define MSM_PMEM_ADSP_SIZE      0x1000000
#define MSM7x25A_MSM_PMEM_ADSP_SIZE      0xB91000

/*because of supporting fwvga (resolution 480*854, bpp 24)
 *framebuffer size >= 480*854*24*3 bit
 */
#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_SIZE		0x500000
#else
#define MSM_FB_SIZE		0x300000
#endif

#endif

#ifndef CONFIG_USB_AUTO_INSTALL
static char *usb_functions_default[] = {
	"diag",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_default_adb[] = {
	"diag",
	"adb",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
	"adb",
#ifdef CONFIG_USB_F_SERIAL
	"modem",
	"nmea",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif
	"usb_mass_storage",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id     = 0x9026,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions      = usb_functions_default,
	},
	{
		.product_id	= 0x9025,
		.num_functions	= ARRAY_SIZE(usb_functions_default_adb),
		.functions	= usb_functions_default_adb,
	},
	{
		.product_id	= 0xf00e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x9024,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Qualcomm Incorporated",
	.product	= "Mass storage",
	.release	= 0x0100,

};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x05C6,
	.product_id	= 0x9026,
	.version	= 0x0100,
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	.vendorID	= 0x05C6,
	.vendorDescr	= "Qualcomm Incorporated",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	    = -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#else /* huawei usb config */
static char *usb_functions_hw_normal_adb[] = {
	"modem",
	"nmea",
	"usb_mass_storage",
	"adb",
	"diag",
};

static char *usb_functions_hw_normal[] = {
	"modem",
	"nmea",
	"usb_mass_storage",
};

static char *usb_functions_hw_ms[] = {
	"usb_mass_storage",
};

static char *usb_functions_google_ms[] = {
	"usb_mass_storage",
};

static char *usb_functions_google_ms_adb[] = {
	"usb_mass_storage",
	"adb",	
};

static char *usb_functions_rndis[] = {
	"rndis", 
};

static char *usb_functions_rndis_adb[] = {
	"rndis", 
	"adb",	
};

static char *usb_functions_all[] = {	
#ifdef CONFIG_USB_ANDROID_RNDIS
    "rndis",
#endif
#ifdef CONFIG_USB_F_SERIAL
    "modem",
    "nmea",
#endif
    "usb_mass_storage",
    "adb",
#ifdef CONFIG_USB_ANDROID_DIAG
    "diag",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
    "rmnet",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
    "acm",
#endif
};

/* nluns : 1 presents one cdrom and no udisks to support
 * nluns : 2 presents two udisks to support internal and external sdcard
 * nluns : 3 presents one cdrom and two udisks to support internal and external sdcard
 * cdrom_index : -1 presents no cdrom to support
 * cdrom_index : 0 presents one cdrom to support
 */
static struct android_usb_product_hw usb_products[] = {
    {
        .adb_product_id = PID_UDISK,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .adb_functions  = usb_functions_hw_ms,
        .product_id = PID_UDISK,
        .num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .functions  = usb_functions_hw_ms,
        .nluns = 2,
        .cdrom_index=-1,
    },
    {
        .adb_product_id = PID_ONLY_CDROM,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .adb_functions  = usb_functions_hw_ms,
        .product_id = PID_ONLY_CDROM,
        .num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .functions  = usb_functions_hw_ms,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_AUTH,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .adb_functions  = usb_functions_hw_normal_adb,
        .product_id = PID_AUTH,
        .num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .functions  = usb_functions_hw_normal_adb,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_NORMAL,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .adb_functions  = usb_functions_hw_normal_adb,
        .product_id = PID_NORMAL,
        .num_functions = ARRAY_SIZE(usb_functions_hw_normal),
        .functions  = usb_functions_hw_normal,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_ONLY_CDROM_TMO,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .adb_functions  = usb_functions_hw_ms,
        .product_id = PID_ONLY_CDROM_TMO,
        .num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .functions  = usb_functions_hw_ms,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_AUTH_TMO,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .adb_functions  = usb_functions_hw_normal_adb,
        .product_id = PID_AUTH_TMO,
        .num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .functions  = usb_functions_hw_normal_adb,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_NORMAL_TMO,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .adb_functions  = usb_functions_hw_normal_adb,
        .product_id = PID_NORMAL_TMO,
        .num_functions = ARRAY_SIZE(usb_functions_hw_normal),
        .functions  = usb_functions_hw_normal,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_GOOGLE,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_google_ms_adb),
        .adb_functions  = usb_functions_google_ms_adb,
        .product_id = PID_GOOGLE_MS,
        .num_functions = ARRAY_SIZE(usb_functions_google_ms),
        .functions  = usb_functions_google_ms,
        .nluns = 3,
        .cdrom_index=1,
    },
    {
        .adb_product_id = PID_WLAN_ADB,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
        .adb_functions  = usb_functions_rndis_adb,
        .product_id = PID_WLAN,
        .num_functions  = ARRAY_SIZE(usb_functions_rndis),
        .functions  = usb_functions_rndis,
        .nluns = 1,
        .cdrom_index=-1,
    },
};

/* keep usb parameters transfered from modem */
static char product_name[MAX_NAME_LEN];
static char vendor_name[MAX_NAME_LEN];
static char manufacturer_name[MAX_NAME_LEN];
#define MAX_LENS 3

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= MAX_LENS,
	.vendor 	= vendor_name,
	.product	= product_name,
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id     = -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= HUAWEI_VID,
	.product_id	= PID_NORMAL,
	.version	= 0x0100,
	.product_name 	= product_name,
	.manufacturer_name = manufacturer_name,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	    = -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

/* ethaddr is filled by board_serialno_setup */
static struct usb_ether_platform_data rndis_pdata = {
	.vendorID	    = HUAWEI_VID,
	.vendorDescr	= manufacturer_name,
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	    = -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

/* provide a method to map pid_index to usb_pid, 
 * pid_index is kept in NV(4526). 
 * At power up, pid_index is read in modem and transfer to app in share memory.
 * pid_index can be modified through write file fixusb(msm_hsusb_store_fixusb).
 */
u16 pid_index_to_pid(u32 pid_index)
{
	u16 usb_pid = 0xFFFF;
		
	switch(pid_index)
	{
		case CDROM_INDEX:
		case SLATE_TEST_INDEX:
			usb_pid = curr_usb_pid_ptr->cdrom_pid;
			break;
            
		case NORM_INDEX:
			usb_pid = curr_usb_pid_ptr->norm_pid;
			break;
            
		case AUTH_INDEX:
			usb_pid = curr_usb_pid_ptr->auth_pid;
			break;
            
		case GOOGLE_INDEX:
			usb_pid = curr_usb_pid_ptr->google_pid;
			break;
				
		case GOOGLE_WLAN_INDEX:
			usb_pid = curr_usb_pid_ptr->wlan_pid;
			break;
            
		/* set the USB pid to multiport when the index is 0
		 * This is happened when the NV is not set or set to 0
		 */
		case ORI_INDEX:
		default:
			usb_pid = curr_usb_pid_ptr->norm_pid;
			break;
	}

	USB_PR("%s, pid_index=%d, usb_pid=0x%x\n", __func__, pid_index, usb_pid);
	return usb_pid;
}

void set_usb_device_name(void)
{
	memset(manufacturer_name, 0, MAX_NAME_LEN);
	memset(product_name, 0, MAX_NAME_LEN);
	memset(vendor_name, 0, MAX_NAME_LEN);
	
	strcpy(manufacturer_name, "Huawei Incorporated");
	strcpy(product_name, "Android Adapter");
}

/*	
 * Set usb serial number according to pid.
 */
void set_usb_pid_sn(u32 pid_index)
{
	switch(pid_index)
	{
		case GOOGLE_WLAN_INDEX:
			USB_PR("set pid=0x%x, sn=NULL\n", GOOGLE_WLAN_INDEX);
			android_set_product_id(PID_WLAN);
			set_usb_sn(NULL);
			break;
            
		case GOOGLE_INDEX:
			USB_PR("set pid=0x%x, sn=%s\n", PID_GOOGLE_MS, usb_para_data.usb_para.usb_serial);
			android_set_product_id(PID_GOOGLE_MS);
			set_usb_sn(usb_para_data.usb_para.usb_serial);
			break;
						
		case NORM_INDEX:
			USB_PR("set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, USB_SN_STRING);
			android_set_product_id(curr_usb_pid_ptr->norm_pid);
			set_usb_sn(USB_SN_STRING);
			break;
            
		case SLATE_TEST_INDEX:
		case CDROM_INDEX:
			USB_PR("set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->cdrom_pid, "");
			android_set_product_id(curr_usb_pid_ptr->cdrom_pid);
			set_usb_sn(NULL);
			break;
						
		case ORI_INDEX:
			USB_PR("set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, "");
			android_set_product_id(curr_usb_pid_ptr->norm_pid);
			set_usb_sn(NULL);
			break;
						
		case AUTH_INDEX:
			USB_PR("set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->auth_pid, "");
			android_set_product_id(curr_usb_pid_ptr->auth_pid);
			set_usb_sn(NULL);
			break;
						
		default:
			USB_PR("set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, "");
			android_set_product_id(curr_usb_pid_ptr->norm_pid);
			set_usb_sn(NULL);
			break;
	}
}

/*	
 * Get usb parameter from share memory and set usb serial number accordingly.
 */
static void proc_usb_para(void)
{
	smem_huawei_vender *usb_para_ptr;
	char *vender_name="t-mobile";

	USB_PR("%s\n", __func__);

	/* initialize */
	usb_para_info.usb_pid_index = 0;
	usb_para_info.usb_pid = PID_NORMAL;

	set_usb_device_name();
		
	/* now the smem_id_vendor0 smem id is a new struct */
	usb_para_ptr = (smem_huawei_vender*)smem_alloc(SMEM_ID_VENDOR0, sizeof(smem_huawei_vender));
	if (!usb_para_ptr)
	{
		USB_PR("%s: Can't find usb parameter\n", __func__);
		return;
	}

	USB_PR("vendor:%s,country:%s\n", usb_para_ptr->vender_para.vender_name,
            usb_para_ptr->vender_para.country_name);

	memcpy(&usb_para_data, usb_para_ptr, sizeof(smem_huawei_vender));
		
	/* decide usb pid array according to the vender name */
	if(!memcmp(usb_para_ptr->vender_para.vender_name, vender_name, strlen(vender_name)))
	{
		curr_usb_pid_ptr = &usb_pid_array[1];
		USB_PR("USB setting is TMO\n");
	}
	else
	{
		curr_usb_pid_ptr = &usb_pid_array[0];
		USB_PR("USB setting is NORMAL\n");
	}

	USB_PR("smem usb_serial=%s, usb_pid_index=%d\n", usb_para_ptr->usb_para.usb_serial,
            usb_para_ptr->usb_para.usb_pid_index);

	/* when manufacture, we need to use the diag. so if the usb_serial is null
	 * and the nv value is google index, we set the ports to normal.
	 */
	if (0 == usb_para_data.usb_para.usb_serial[0] 
		&& GOOGLE_INDEX == usb_para_ptr->usb_para.usb_pid_index)
	{
		USB_PR("%s usb serial number is null in google mode. so switch to original mode\n", __func__);
		usb_para_ptr->usb_para.usb_pid_index = ORI_INDEX;
	}

	usb_para_info.usb_pid_index = usb_para_ptr->usb_para.usb_pid_index;
		
	usb_para_info.usb_pid = pid_index_to_pid(usb_para_ptr->usb_para.usb_pid_index);

	/* add new pid config for google */
	set_usb_pid_sn(usb_para_info.usb_pid_index);

    /* the vendor emobile in japan doesn't need the cdrom */
    if ((0 == memcmp(usb_para_data.vender_para.vender_name, VENDOR_EMOBILE, strlen(VENDOR_EMOBILE)))
        && (0 == memcmp(usb_para_data.vender_para.country_name, COUNTRY_JAPAN, strlen(COUNTRY_JAPAN))))
    {
        u16 index = 0;
        /* find the google mode from the products array */
        for (; index < android_usb_pdata.num_products; index++)
        {
            if (PID_GOOGLE == android_usb_pdata.products[index].adb_product_id)
            {
                /* set the nluns to 2 from 3, del the cdrom */
                android_usb_pdata.products[index].nluns = 2; /* 2 is two udisk */
                android_usb_pdata.products[index].cdrom_index = -1; /* -1 is not support cdrom */
                USB_PR("%s %s%s doesn't need the cdrom. nluns=%d, cdrom_index=%d\n",
                        __func__, usb_para_data.vender_para.country_name, usb_para_data.vender_para.vender_name,
                        android_usb_pdata.products[index].nluns, android_usb_pdata.products[index].cdrom_index);
            }  
        }  
    }

    /* M866 doesn't need usb tehter fucntion */
    if ((0 == memcmp(usb_para_data.vender_para.vender_name, VENDOR_TRACFONE, strlen(VENDOR_TRACFONE)))
       && (0 == memcmp(usb_para_data.vender_para.country_name, COUNTRY_US, strlen(COUNTRY_US)))
       && machine_is_msm7x27a_C8655_NAND())
       
    {
        u16 index = 0;
        /* find the tether mode from the products array */
        for (; index < android_usb_pdata.num_products; index++)
        {
            if (PID_WLAN == android_usb_pdata.products[index].product_id)
            {
                /* use ineffective pid to take the place of tether pid*/
                android_usb_pdata.products[index].adb_product_id = PID_NONE;
                android_usb_pdata.products[index].product_id = PID_NONE;
                USB_PR("%s %s %s H866C doesn't need tether. Tether pid is replaced by 0x%x\n",
                        __func__, usb_para_data.vender_para.country_name, usb_para_data.vender_para.vender_name,
                        android_usb_pdata.products[index].product_id);
            }  
        }  
    }
	USB_PR("curr_usb_pid_ptr: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", 
			curr_usb_pid_ptr->cdrom_pid, 
			curr_usb_pid_ptr->norm_pid, 
			curr_usb_pid_ptr->udisk_pid,
			curr_usb_pid_ptr->auth_pid,
			curr_usb_pid_ptr->google_pid);
	USB_PR("usb_para_info: usb_pid_index=%d, usb_pid = 0x%x\n", 
			usb_para_info.usb_pid_index, 
			usb_para_info.usb_pid);
}
#endif	/* CONFIG_USB_AUTO_INSTALL */

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	/* usb serial number is set to bt address */
#ifndef CONFIG_HUAWEI_KERNEL
	android_usb_pdata.serial_number = serialno;
#endif
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

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

static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
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
	.core_clk		 = 1,
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

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

static unsigned long vreg_sts, gpio_sts;
static struct vreg *vreg_mmc;
static struct vreg *vreg_emmc;

struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

/**
 * Due to insufficient drive strengths for SDC GPIO lines some old versioned
 * SD/MMC cards may cause data CRC errors. Hence, set optimal values
 * for SDC slots based on timing closure and marginality. SDC1 slot
 * require higher value since it should handle bad signal quality due
 * to size of T-flash adapters.
 */
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_14MA),
								"sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_0"},
};

static struct msm_gpio sdc2_sleep_cfg_data[] = {
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_0"},
};
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_0"},
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	{GPIO_CFG(19, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_7"},
	{GPIO_CFG(20, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_6"},
	{GPIO_CFG(21, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_5"},
	{GPIO_CFG(108, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_4"},
#endif
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(19, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_3"},
	{GPIO_CFG(20, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_2"},
	{GPIO_CFG(21, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_1"},
	{GPIO_CFG(107, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_cmd"},
	{GPIO_CFG(108, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_0"},
	{GPIO_CFG(109, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_clk"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = sdc2_sleep_cfg_data,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
	},
};

static int msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			pr_err("%s: Failed to turn on GPIOs for slot %d\n",
					__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			rc = msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
			return rc;
		}
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
	return rc;
}

static int msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;

	curr = &sdcc_vreg_data[dev_id - 1];

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc)
			pr_err("%s: vreg_set_level() = %d\n", __func__, rc);

		rc = vreg_enable(curr->vreg_data);
		if (rc)
			pr_err("%s: vreg_enable() = %d\n", __func__, rc);
	} else {
		clear_bit(dev_id, &vreg_sts);
		rc = vreg_disable(curr->vreg_data);
		if (rc)
			pr_err("%s: vreg_disable() = %d\n", __func__, rc);
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	rc = msm_sdcc_setup_gpio(pdev->id, !!vdd);
	if (rc)
		goto out;

	rc = msm_sdcc_setup_vreg(pdev->id, !!vdd);
out:
	return rc;
}

#define GPIO_SDC1_HW_DET 85

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT) \
	&& defined(CONFIG_MMC_MSM_CARD_HW_DETECTION)
static unsigned int msm7x2xa_sdcc_slot_status(struct device *dev)
{
	int status;

	status = gpio_tlmm_config(GPIO_CFG(GPIO_SDC1_HW_DET, 2, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (status)
		pr_err("%s:Failed to configure tlmm for GPIO %d\n", __func__,
				GPIO_SDC1_HW_DET);

	status = gpio_request(GPIO_SDC1_HW_DET, "SD_HW_Detect");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n", __func__,
				GPIO_SDC1_HW_DET);
	} else {
		status = gpio_direction_input(GPIO_SDC1_HW_DET);
		if (!status) {
#ifdef CONFIG_HUAWEI_KERNEL
			if(machine_is_msm7x27a_U8185())
			{
				//u8185 is different from other products.
				status = gpio_get_value(GPIO_SDC1_HW_DET);
			}
			else
			{
				status = !gpio_get_value(GPIO_SDC1_HW_DET);
			}
#else
			status = gpio_get_value(GPIO_SDC1_HW_DET);
#endif
		}
		gpio_free(GPIO_SDC1_HW_DET);
	}
	return status;
}
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data sdc1_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x2xa_sdcc_slot_status,
	.status_irq  = MSM_GPIO_TO_INT(GPIO_SDC1_HW_DET),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data sdc2_plat_data = {
	/*
	 * SDC2 supports only 1.8V, claim for 2.85V range is just
	 * for allowing buggy cards who advertise 2.8V even though
	 * they can operate at 1.8V supply.
	 */
	.ocr_mask	= MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	/*.sdiowakeup_irq = MSM_GPIO_TO_INT(66),*/
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data sdc3_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
static struct mmc_platform_data sdc4_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
};
#endif
#endif
/* The following config was used for WCN2243 only. 0xFD means wake.*/
#ifdef CONFIG_HUAWEI_KERNEL
#if defined(CONFIG_SERIAL_MSM_HS) && defined(HUAWEI_BT_WCN2243)
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
static const char * const msm_fb_lcdc_vreg[] = {
		"gp2",
		"msme1",
};

static const int msm_fb_lcdc_vreg_mV[] = {
	2850,
	1800,
};

struct vreg *lcdc_vreg[ARRAY_SIZE(msm_fb_lcdc_vreg)];

static uint32_t lcdc_gpio_initialized;

static void lcdc_toshiba_gpio_init(void)
{
	int i, rc = 0;
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

		for (i = 0; i < ARRAY_SIZE(msm_fb_lcdc_vreg); i++) {
			lcdc_vreg[i] = vreg_get(0, msm_fb_lcdc_vreg[i]);

			rc = vreg_set_level(lcdc_vreg[i],
						msm_fb_lcdc_vreg_mV[i]);

			if (rc < 0) {
				pr_err("%s: set regulator level failed "
					"with :(%d)\n", __func__, rc);
				goto fail_gpio1;
			}
		}
		lcdc_gpio_initialized = 1;
	}
	return;

fail_gpio1:
	for (; i > 0; i--)
			vreg_put(lcdc_vreg[i - 1]);

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
	int i, rc = 0;
	/* Doing the init of the LCDC GPIOs very late as they are from
		an I2C-controlled IO Expander */
	lcdc_toshiba_gpio_init();

	if (lcdc_gpio_initialized) {
		gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);

		for (i = 0; i < ARRAY_SIZE(msm_fb_lcdc_vreg); i++) {
			if (on) {
				rc = vreg_enable(lcdc_vreg[i]);

				if (rc) {
					printk(KERN_ERR "vreg_enable: %s vreg"
						"operation failed\n",
						msm_fb_lcdc_vreg[i]);
						goto lcdc_vreg_fail;
				}
			} else {
				rc = vreg_disable(lcdc_vreg[i]);

				if (rc) {
					printk(KERN_ERR "vreg_disable: %s vreg "
						"operation failed\n",
						msm_fb_lcdc_vreg[i]);
					goto lcdc_vreg_fail;
				}
			}
		}
	}

	return rc;

lcdc_vreg_fail:
	if (on) {
		for (; i > 0; i--)
			vreg_disable(lcdc_vreg[i - 1]);
	} else {
		for (; i > 0; i--)
			vreg_enable(lcdc_vreg[i - 1]);
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

		/*if ctrl bl by msm , init pmapp*/
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
	/* Doing the init of the LCDC GPIOs very late as they are from
		an I2C-controlled IO Expander */
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
/*add three new LCD device*/
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
/* add the hx8347d_chimei device */
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

/* resume the code of qualcomm, because huawei don't use the function.*/
static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;


	if (machine_is_msm7x27a_surf()) {
    	if (!strncmp(name, "lcdc_toshiba_fwvga_pt", 21))
    		ret = 0;
	} else {
		ret = -ENODEV;
	}

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

static struct resource huawei_share_memory_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
	}
};

static struct platform_device huawei_share_memory_device = {
	.name   	    = "hw_share_mem",
	.id     	    = 0,
	.num_resources  = ARRAY_SIZE(huawei_share_memory_resources),
	.resource       = huawei_share_memory_resources,
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

#ifdef CONFIG_HUAWEI_WIFI_SDCC
#define TAG_BCM			"BCM_4330"

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM			16

/* for wifi awake */
#define WLAN_WAKES_MSM        		48	
/* for wifi power supply */
#define WLAN_REG 					6

#define WLAN_GPIO_FUNC_0         	0
#define WLAN_GPIO_FUNC_1         	1
#define WLAN_STAT_ON             	1
#define WLAN_STAT_OFF            	0
	
static unsigned wlan_wakes_msm[] = {
	GPIO_CFG( WLAN_WAKES_MSM, WLAN_GPIO_FUNC_0 , GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA ) 
};

static unsigned wifi_config_init[] = {
	GPIO_CFG( WLAN_REG, WLAN_GPIO_FUNC_0 , GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL , GPIO_CFG_2MA ) 
};

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

/*wlan static memory alloc*/
static void *bcm_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

/*wlan power control*/
static int bcm_wifi_set_power(int enable)
{
	int ret = 0;

   	if (enable)
	{
			/* turn on wifi_vreg */
            ret = gpio_direction_output(WLAN_REG, WLAN_STAT_ON);
            if (ret < 0) {
            	printk(KERN_ERR "%s: turn on wlan_reg failed (%d)\n" , __func__, ret);
            	return -EIO;
            }
            mdelay(150);
            printk(KERN_ERR "%s: wifi power successed to pull up\n" , __func__ );
		
	}
    else { 
        	/* turn off wifi_vreg */
            ret = gpio_direction_output(WLAN_REG, WLAN_STAT_OFF);
            if (ret < 0) {
            	printk(KERN_ERR "%s: turn off wlan_reg failed (%d)\n" , __func__, ret);
            	return -EIO;
            }
            mdelay(1);
            printk(KERN_ERR "%s: wifi power successed to pull down\n",__func__ );
	}

	return ret;
}

int __init bcm_wifi_init_gpio_mem(void)
{
	int i = 0;
	int rc = 0;

	/* config gpio WLAN_WAKES_MSM */
	rc = gpio_tlmm_config(wlan_wakes_msm[0], GPIO_CFG_ENABLE);	
	if( rc ) 
		printk(KERN_ERR "%s: %s gpio_tlmm_config(wlan_wakes_msm) failed rc = %d\n", __func__ , TAG_BCM , rc);
	else 
		printk(KERN_ERR "%s: %s gpio_tlmm_config(wlan_wakes_msm) successfully\n", __func__ , TAG_BCM);
	/* request gpio WLAN_WAKES_MSM */
	rc = gpio_request( WLAN_WAKES_MSM , "WLAN_WAKES_MSM" );
	if( rc ) 
		printk(KERN_ERR "%s: %s Failed to gpio_request(WLAN_WAKES_MSM) rc = %d\n" , __func__ , TAG_BCM , rc );	
	else 
		printk(KERN_ERR "%s: %s Success to gpio_request(WLAN_WAKES_MSM)\n" , __func__ , TAG_BCM );	

	/* config gpio WLAN_REG */
	rc = gpio_tlmm_config(wifi_config_init[0], GPIO_CFG_ENABLE);
	if( rc )
		printk(KERN_ERR "%s: %s gpio_tlmm_config(wifi_config_init) failed rc = %d\n", __func__ , TAG_BCM , rc);
	else
		printk(KERN_ERR "%s: %s gpio_tlmm_config(wifi_config_init) successfully\n", __func__ , TAG_BCM);
	/* request gpio WLAN_REG */
	rc = gpio_request( WLAN_REG , "WLAN_REG" );
	if( rc )
		printk(KERN_ERR "%s: %s Failed to gpio_request(WLAN_REG) rc = %d\n" , __func__ , TAG_BCM , rc);
	else
		printk(KERN_ERR "%s: %s Success to gpio_request(WLAN_REG)\n" , __func__ , TAG_BCM );
	
    mdelay(5);

    /* turn off wifi_vreg */
    rc = gpio_direction_output(WLAN_REG, 0);
    if (rc < 0) {
		printk(KERN_ERR "%s: %s turn off wlan_reg failed (%d)\n" , __func__, TAG_BCM,  rc);
		return -EIO;
    }
    else {
		printk(KERN_ERR "%s: %s turn off wlan_reg successfully (%d)\n" , __func__, TAG_BCM,  rc);
    }

    mdelay(5);
       
	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096); 	/* malloc skb 4k buffer */
		else
			wlan_static_skb[i] = dev_alloc_skb(32768); 	/* malloc skb 32k buffer */
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	
	printk("%s: %s bcm_wifi_init_gpio_mem successfully\n" , __func__ , TAG_BCM );
	return 0;
}

static struct wifi_platform_data bcm_wifi_control = {
	.mem_prealloc	= bcm_wifi_mem_prealloc,
	.set_power	=bcm_wifi_set_power,
};

static struct platform_device bcm_wifi_device = {
        .name           = "bcm4330_wlan",	/*bcm4330 wlan device*/
        .id             = 1,
        .num_resources  = 0,
        .resource       = NULL,
        .dev            = {
                .platform_data = &bcm_wifi_control,
        },
};
#endif


/* for wifi awake */
#define ATH_WLAN_WAKES_MSM        		48	
/* for wifi power supply */
#define ATH_WLAN_REG 					6

#define TAG_ATH			"ATH_6005"

static unsigned ath_wlan_wakes_msm[] = {
	GPIO_CFG( ATH_WLAN_WAKES_MSM, WLAN_GPIO_FUNC_0 , GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA ) 
};

static unsigned ath_wifi_config_init[] = {
	GPIO_CFG( ATH_WLAN_REG, WLAN_GPIO_FUNC_0 , GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL , GPIO_CFG_2MA ) 
};

int __init ath_wifi_init_gpio_mem(void)
{
	int rc = 0;

	/* config gpio WLAN_WAKES_MSM */
	rc = gpio_tlmm_config(ath_wlan_wakes_msm[0], GPIO_CFG_ENABLE);	
	if( rc ) 
		printk(KERN_ERR "%s: %s gpio_tlmm_config(ath_wlan_wakes_msm) failed rc = %d\n", __func__ , TAG_ATH , rc);
	else 
		printk(KERN_ERR "%s: %s gpio_tlmm_config(ath_wlan_wakes_msm) successfully\n", __func__ , TAG_ATH);
	/* request gpio WLAN_WAKES_MSM */
	rc = gpio_request( ATH_WLAN_WAKES_MSM , "ATH_WLAN_WAKES_MSM" );
	if( rc ) 
		printk(KERN_ERR "%s: %s Failed to gpio_request(ATH_WLAN_WAKES_MSM) rc = %d\n" , __func__ , TAG_ATH , rc );	
	else 
		printk(KERN_ERR "%s: %s Success to gpio_request(ATH_WLAN_WAKES_MSM)\n" , __func__ , TAG_ATH );	

	/* config gpio WLAN_REG */
	rc = gpio_tlmm_config(ath_wifi_config_init[0], GPIO_CFG_ENABLE);
	if( rc )
		printk(KERN_ERR "%s: %s gpio_tlmm_config(ath_wifi_config_init) failed rc = %d\n", __func__ , TAG_ATH , rc);
	else
		printk(KERN_ERR "%s: %s gpio_tlmm_config(ath_wifi_config_init) successfully\n", __func__ , TAG_ATH);
	/* request gpio WLAN_REG */
	rc = gpio_request( ATH_WLAN_REG , "ATH_WLAN_REG" );
	if( rc )
		printk(KERN_ERR "%s: %s Failed to gpio_request(ATH_WLAN_REG) rc = %d\n" , __func__ , TAG_ATH , rc);
	else
		printk(KERN_ERR "%s: %s Success to gpio_request(ATH_WLAN_REG)\n" , __func__ , TAG_ATH);
	
    mdelay(5);

    /* turn off wifi_vreg */
    rc = gpio_direction_output(ATH_WLAN_REG, 0);
    if (rc < 0) {
		printk(KERN_ERR "%s: %s turn off ath_wlan_reg failed (%d)\n" , __func__, TAG_ATH,  rc);
		return -EIO;
    }
    else {
		printk(KERN_ERR "%s: %s turn off ath_wlan_reg successfully (%d)\n" , __func__, TAG_ATH,  rc);
    }

    mdelay(5);
    
    printk("%s: %s ath_wifi_init_gpio_mem successfully\n" , __func__ , TAG_ATH );
    return 0;

}    


static void __init msm7x27a_init_mmc(void)
{
    struct vreg *vreg_s3 = NULL;
 /*S3 is always on for emmc,L10 is not used,so turn off L10 in the modem,and don't configure the L10 in the AP,
set L10 = PM_VREG_INVALID_ID*/
#ifdef CONFIG_HUAWEI_KERNEL       
        vreg_emmc = vreg_get(NULL, "gp1");
#else
        vreg_emmc = vreg_get(NULL, "emmc");

#endif

	if (IS_ERR(vreg_emmc)) {
		pr_err("%s: vreg get failed (%ld)\n",
				__func__, PTR_ERR(vreg_emmc));
		return;
	}

	vreg_mmc = vreg_get(NULL, "mmc");
	if (IS_ERR(vreg_mmc)) {
		pr_err("%s: vreg get failed (%ld)\n",
				__func__, PTR_ERR(vreg_mmc));
		return;
	}
	vreg_s3 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_s3)) 
    {
		pr_err("%s: vreg get failed (%ld)\n",
				__func__, PTR_ERR(vreg_s3));
		return;
	}
	/* eMMC slot */
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
    /* in nand flash, don't use the sdc3 (emmc sdcc) */
#ifndef CONFIG_HUAWEI_KERNEL
    sdcc_vreg_data[2].vreg_data = vreg_emmc;
    sdcc_vreg_data[2].level = 3000;
    msm_add_sdcc(3, &sdc3_plat_data);
#else
    if (machine_is_msm7x27a_U8655() 
        || machine_is_msm7x27a_C8655_NAND()
        || machine_is_msm7x27a_U8185()
		|| machine_is_msm7x27a_U8661())
    {
        pr_info("nand product, ignore the emmc sdcc\n");
    }
    else
    {
        sdcc_vreg_data[2].vreg_data = vreg_emmc;
        sdcc_vreg_data[2].level = 3000;
        msm_add_sdcc(3, &sdc3_plat_data);
    }
#endif
#endif
	/* Micro-SD slot */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	sdcc_vreg_data[0].vreg_data = vreg_mmc;
	sdcc_vreg_data[0].level = 2850;
	msm_add_sdcc(1, &sdc1_plat_data);
#endif
	/* SDIO WLAN slot */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
    /* wifi use VREG_S3 (1.8V), but not VREG_L13_SD */
	sdcc_vreg_data[1].vreg_data = vreg_s3;
	sdcc_vreg_data[1].level = 1800;
	msm_add_sdcc(2, &sdc2_plat_data);
    if(WIFI_QUALCOMM == get_hw_wifi_device_type())
    {
        ath_wifi_init_gpio_mem();
    }
    else
    {
        #ifdef CONFIG_HUAWEI_WIFI_SDCC
        bcm_wifi_init_gpio_mem();
        platform_device_register(&bcm_wifi_device);
        #endif
    }
#endif
	/* Not Used */
#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
	sdcc_vreg_data[3].vreg_data = vreg_mmc;
	sdcc_vreg_data[3].level = 2850;
	msm_add_sdcc(4, &sdc4_plat_data);
#endif
}
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
	SND(CURRENT, 0x7FFFFFFE),
	/* add new device for FM AUX_PGA path */
	SND(FM_RADIO_STEREO_HEADSET, 29),
	SND(FM_RADIO_SPEAKER_PHONE, 30),	
	SND(HEADSET_AND_SPEAKER, 31),
	SND(FM_ANALOG_STEREO_HEADSET, 35),
	SND(FM_ANALOG_STEREO_HEADSET_CODEC, 36),
	/* add new device for 2nd mic MMI test*/
	SND(HANDSET_2NDMIC, 37),
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
	.voltage_min_design     = 2800,
	.voltage_max_design     = 4300,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage	 = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage = msm_psy_batt_data.voltage_max_design;

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
/* gpio 49 is for camera reset*/
#ifndef CONFIG_HUAWEI_CAMERA

static struct msm_gpio smsc911x_gpios[] = {
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "smsc911x_irq"  },
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
#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(8, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RESET For mt9v113 */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
	GPIO_CFG(32, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /*PWD for M660 camera*/
	GPIO_CFG(37, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /*PWD for U8185 camera*/
	GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RESET */

	/* Camera I2C config like normal gpio with value 0 when camera turn off */
	GPIO_CFG(60, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), 
	GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), 
	GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /*PWD*/
	GPIO_CFG(120, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /*PWD for mt9v113 */
};

/* add the camera reset gpio 49*/
static uint32_t camera_on_gpio_table[] = {
#ifdef CONFIG_HUAWEI_CAMERA
	GPIO_CFG(8,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RESET For mt9v113 */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* MCLK: increase the drive capability */
	GPIO_CFG(32, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /*PWD for M660 camera*/
	GPIO_CFG(37, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /*PWD for U8185 camera*/
	GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RESET */

    GPIO_CFG(7, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* VCM_PWD */
	/* Camera I2C config to I2C when camera start */
	GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), 
	GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), 
	GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* PWD */
	GPIO_CFG(120, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* PWD for mt9v113 */
#endif
};

#ifdef CONFIG_MSM_CAMERA_FLASH
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	._fsrc.current_driver_src.led1 = GPIO_CAM_GP_LED_EN1,
	._fsrc.current_driver_src.led2 = GPIO_CAM_GP_LED_EN2,
};
#endif

/* camera uesed verg_bt 2.85v */
#ifdef CONFIG_HUAWEI_CAMERA
/* Use vreg L5 to switch camera vreg 1.8V */
static struct vreg *vreg_L17 = NULL;
static struct vreg *vreg_L15 = NULL;
static struct vreg *vreg_L5 = NULL;

static void msm_camera_vreg_config(int vreg_en)
{
	int rc;
	
	if (vreg_L5 == NULL) {
		vreg_L5 = vreg_get(NULL, "wlan2");
		if (IS_ERR(vreg_L5)) {
			pr_err("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "bt", PTR_ERR(vreg_L5));
			return;
		}

		rc = vreg_set_level(vreg_L5, 1300);
		if (rc) {
			pr_err("%s: GP2 set_level failed (%d)\n",
				__func__, rc);
		}
	}
	/*L15 is the 2.85v supplier for U8185 camera */
	/*adjust supplier by wifi_device_type */
	if (WIFI_QUALCOMM == get_hw_wifi_device_type())
	{
        /* For U8661 UIM2 uses L15 to supply power. */
		if (HW_DS != get_hw_ds_type())
		{
			if (NULL == vreg_L15) 		
			{	
				vreg_L15 = vreg_get(NULL, "usim2");
				if (IS_ERR(vreg_L15)) 
				{
					pr_err("%s: vreg_get(%s) failed (%ld)\n",
						__func__, "bt", PTR_ERR(vreg_L15));
					 return;
				}
	
				rc = vreg_set_level(vreg_L15, 2850);
				if (rc) 
				{
					pr_err("%s: GP2 set_level failed (%d)\n",
						    __func__, rc);
				}
			}
		}
	}
	else
	{
		if (vreg_L17 == NULL) {	
		vreg_L17 = vreg_get(NULL, "bt");
		if (IS_ERR(vreg_L17)) {
			pr_err("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "bt", PTR_ERR(vreg_L17));
			return;
		}

		rc = vreg_set_level(vreg_L17, 2850);
		if (rc) {
			pr_err("%s: GP2 set_level failed (%d)\n",
				__func__, rc);
		}
		}
	}

	if (vreg_en) {
		rc = vreg_enable(vreg_L5);
		if (rc) {
			pr_err("%s: L5 enable failed (%d)\n",
				__func__, rc);
		}
		/*add a delay between pull up AVDD and IOVDD*/
		udelay(140);
		
		if (WIFI_QUALCOMM == get_hw_wifi_device_type())
		{
            /* For U8661 UIM2 uses L15 to supply power. */
		    if (HW_DS != get_hw_ds_type())
		    {
			    rc = vreg_enable(vreg_L15);
			    if (rc) 
				{
				    pr_err("%s: L15 enable failed (%d)\n",
					       __func__, rc);
			    }
			}
		}
		else
		{
			rc = vreg_enable(vreg_L17);
			if (rc) {
				pr_err("%s: L17 enable failed (%d)\n",
					__func__, rc);
			}
		}
	} else {
		rc = vreg_disable(vreg_L5);
		if (rc) {
			pr_err("%s: L5 disable failed (%d)\n",
				__func__, rc);
		}	
		udelay(400);
		/*adjust supplier by wifi_device_type */
		if (WIFI_QUALCOMM == get_hw_wifi_device_type())
		{
            /* For U8661 UIM2 uses L15 to supply power. */
		    if (HW_DS != get_hw_ds_type())
		    {
			   rc = vreg_disable(vreg_L15);
			   if (rc) 
			   {
			        pr_err("%s: L15 disable failed (%d)\n",
					       __func__, rc);
			   }
			}
		}
		else
		{
			rc = vreg_disable(vreg_L17);
			if (rc) {
				pr_err("%s: L17 disable failed (%d)\n",
					__func__, rc);
			}
		}
	}
}
#endif

static int config_gpio_table(uint32_t *table, int len)
{
	int rc = 0, i = 0;

	for (i = 0; i < len; i++) {
		rc = gpio_tlmm_config(table[i], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s not able to get gpio\n", __func__);
			for (i--; i >= 0; i--)
				gpio_tlmm_config(camera_off_gpio_table[i],
							GPIO_CFG_ENABLE);
			break;
		}
	}
	return rc;
}

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1_data;
static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data;
#ifdef CONFIG_HUAWEI_CAMERA
/*we transfer the func after s5k5ca is probe succeed*/
static void camera_s5k5ca_or_mt9t113_is_on(int s5k5ca_or_mt9t113_probe_success)
{
	s5k5ca_or_mt9t113_is_on = s5k5ca_or_mt9t113_probe_success ;
}
static int config_camera_on_gpios_rear(void)
{
	int rc = 0;
/*delete some lines for power enable*/
	
	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_rear(void)
{
/*delete some lines for power disable*/

	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

static int config_camera_on_gpios_front(void)
{
	int rc = 0;

	/*delete one line for power enable*/

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
			"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_front(void)
{
	/*delete one line for power disable*/

	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}
#endif
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_9900
int aps9900_gpio_config_interrupt(void)
{
    int gpio_config = 0;
    int ret = 0;
    
    gpio_config = GPIO_CFG(MSM_7X30_APS9900_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
    ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
    return ret; 
}

static struct aps9900_hw_platform_data aps9900_hw_data = {
    .aps9900_gpio_config_interrupt = aps9900_gpio_config_interrupt,
};
#endif

struct msm_camera_device_platform_data msm_camera_device_data_rear = {
	.camera_gpio_on  = config_camera_on_gpios_rear,
	.camera_gpio_off = config_camera_off_gpios_rear,
	.ioext.csiphy = 0xA1000000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
#ifdef CONFIG_HUAWEI_CAMERA 	
	.get_board_support_flash = board_support_flash,
#endif
};

struct msm_camera_device_platform_data msm_camera_device_data_front = {
	.camera_gpio_on  = config_camera_on_gpios_front,
	.camera_gpio_off = config_camera_off_gpios_front,
	.ioext.csiphy = 0xA0F00000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_0,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1
static struct msm_camera_sensor_platform_info s5k4e1_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_s5k4e1 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1_data = {
	.sensor_name    = (char*)back_camera_name,
	.sensor_reset_enable = 1,
	.sensor_reset   = 49,
	.sensor_pwd             = 119,
	.vcm_pwd                = 7,
	.vcm_enable             = 1,
	.pdata                  = &msm_camera_device_data_rear,
	.flash_data             = &flash_s5k4e1,
	.sensor_platform_info   = &s5k4e1_sensor_7627a_info,
	.csi_if                 = 1,
	.vreg_enable_func = msm_camera_vreg_config,
	.vreg_disable_func = msm_camera_vreg_config,
	/*back camera is not slave_sensor, below as the same*/
	.slave_sensor           = 0
};

static struct platform_device msm_camera_sensor_s5k4e1 = {
	.name   = "msm_camera_s5k4e1",
	.dev    = {
		.platform_data = &msm_camera_sensor_s5k4e1_data,
	},
};
#endif

#ifdef CONFIG_IMX072
static struct msm_camera_sensor_platform_info imx072_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_imx072 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_imx072_data = {
	.sensor_name    = "imx072",
	.sensor_reset_enable = 1,
	.sensor_reset   = GPIO_CAM_GP_CAMIF_RESET_N, /* TODO 106,*/
	.sensor_pwd             = 85,
	.vcm_pwd                = GPIO_CAM_GP_CAM_PWDN,
	.vcm_enable             = 1,
	.pdata                  = &msm_camera_device_data_rear,
	.flash_data             = &flash_imx072,
	.sensor_platform_info = &imx072_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_imx072 = {
	.name   = "msm_camera_imx072",
	.dev    = {
		.platform_data = &msm_camera_sensor_imx072_data,
	},
};
#endif

#ifdef CONFIG_WEBCAM_OV9726
static struct msm_camera_sensor_platform_info ov9726_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_ov9726 = {
	.flash_type             = MSM_CAMERA_FLASH_NONE,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data = {
	.sensor_name    = "ov9726",
	.sensor_reset_enable = 0,
	.sensor_reset   = GPIO_CAM_GP_CAM1MP_XCLR,
	.sensor_pwd             = 85,
	.vcm_pwd                = 1,
	.vcm_enable             = 0,
	.pdata                  = &msm_camera_device_data_front,
	.flash_data             = &flash_ov9726,
	.sensor_platform_info   = &ov9726_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov9726 = {
	.name   = "msm_camera_ov9726",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov9726_data,
	},
};
#endif

#ifdef CONFIG_MT9E013
static struct msm_camera_sensor_platform_info mt9e013_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9e013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

#ifdef CONFIG_HUAWEI_SENSOR_MT9E013
static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data = {
	.sensor_name    = (char*)back_camera_name,
	.sensor_reset   = 49,
	.sensor_reset_enable = 1,
	.sensor_pwd     = 119,
	.vcm_pwd        = 7,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data_rear,
	.flash_data     = &flash_mt9e013,
	.sensor_platform_info   = &mt9e013_sensor_7627a_info,
	.csi_if         = 1,
	.vreg_enable_func = msm_camera_vreg_config,
	.vreg_disable_func = msm_camera_vreg_config,
	.slave_sensor   = 0
};
#endif

static struct platform_device msm_camera_sensor_mt9e013 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data,
	},
};
#endif
#ifdef CONFIG_HUAWEI_SENSOR_MT9P017
static struct msm_camera_sensor_platform_info mt9p017_sensor_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9p017 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p017_data = {
	.sensor_name    =(char*)back_camera_name,
	.sensor_reset   = 49,
	.sensor_reset_enable = 1,
	.sensor_pwd     = 119,
	.vcm_pwd        = 7,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data_rear,
	.flash_data     = &flash_mt9p017,
	.sensor_platform_info   = &mt9p017_sensor_info,
	.csi_if         = 1,
	.vreg_enable_func = msm_camera_vreg_config,
	.vreg_disable_func = msm_camera_vreg_config,
	.slave_sensor   = 0
};

static struct platform_device msm_camera_sensor_mt9p017 = {
	.name      = "msm_camera_mt9p017",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p017_data,
	},
};
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_S5K5CA
static struct msm_camera_sensor_platform_info s5k5ca_sensor_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_s5k5ca = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k5ca_data = {
	.sensor_name    = (char*)back_camera_name,
	.sensor_reset   = 49,
	.sensor_reset_enable = 1,
	.sensor_pwd     = 119,
	.vcm_pwd        = 7,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data_rear,
	.flash_data     = &flash_s5k5ca,
	.sensor_platform_info   = &s5k5ca_sensor_info,
	.s5k5ca_or_mt9t113_on      = camera_s5k5ca_or_mt9t113_is_on,
	.csi_if         = 1,
	.vreg_enable_func = msm_camera_vreg_config,
	.vreg_disable_func = msm_camera_vreg_config,
	.slave_sensor   = 0
};

static struct platform_device msm_camera_sensor_s5k5ca = {
	.name      = "msm_camera_s5k5ca",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k5ca_data,
	},
};
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9T113
static struct msm_camera_sensor_platform_info mt9t113_sensor_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9t113 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9t113_data = {
	.sensor_name    =(char*)back_camera_name,
	.sensor_reset   = 49,
	.sensor_reset_enable = 1,
	.sensor_pwd     = 119,
	.vcm_pwd        = 7,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data_rear,
	.flash_data     = &flash_mt9t113,
	.sensor_platform_info   = &mt9t113_sensor_info,
	.s5k5ca_or_mt9t113_on      = camera_s5k5ca_or_mt9t113_is_on,
	.csi_if         = 1,
	.vreg_enable_func = msm_camera_vreg_config,
	.vreg_disable_func = msm_camera_vreg_config,
	.slave_sensor   = 0
};

static struct platform_device msm_camera_sensor_mt9t113 = {
	.name      = "msm_camera_mt9t113",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t113_data,
	},
};
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9D113
static struct msm_camera_sensor_platform_info mt9d113_sensor_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9d113 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9d113_data = {
	.sensor_name    = (char*)back_camera_name,
	.sensor_reset   = 49,
	.sensor_reset_enable = 1,
	.sensor_pwd     = 119,
	.vcm_pwd        = 7,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data_rear,
	.flash_data     = &flash_mt9d113,
	.sensor_platform_info   = &mt9d113_sensor_info,
	.csi_if         = 1,
	.vreg_enable_func = msm_camera_vreg_config,
	.vreg_disable_func = msm_camera_vreg_config,
	.slave_sensor   = 0
};

static struct platform_device msm_camera_sensor_mt9d113 = {
	.name      = "msm_camera_mt9d113",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9d113_data,
	},
};
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9V113
static struct msm_camera_sensor_platform_info mt9v113_sensor_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9v113 = {
	.flash_type =MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9v113_data = {
	.sensor_name    =(char*)front_camera_name,
	.sensor_reset   = 8,
	.sensor_reset_enable = 1,
	.sensor_pwd     = 120,
	.vcm_pwd        = 7,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data_front,
	.flash_data     = &flash_mt9v113,
	.sensor_platform_info   = &mt9v113_sensor_info,
	.csi_if         = 1,
	.vreg_enable_func = msm_camera_vreg_config,
	.vreg_disable_func = msm_camera_vreg_config,
	.slave_sensor   = 1
};

static struct platform_device msm_camera_sensor_mt9v113 = {
	.name      = "msm_camera_mt9v113",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9v113_data,
	},
};
#endif
/* driver for hw device detect */
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
static struct platform_device huawei_device_detect = {
	.name = "hw-dev-detect",
	.id   =-1,
};
#endif

static struct i2c_board_info i2c_camera_devices[] = {
	#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1
	{
		I2C_BOARD_INFO("s5k4e1", 0x6E >> 1),
	},
	{
		I2C_BOARD_INFO("s5k4e1_af", 0x18 >> 1),
	},
	#endif
	#ifdef CONFIG_WEBCAM_OV9726
	{
		I2C_BOARD_INFO("ov9726", 0x10),
	},
	#endif
	#ifdef CONFIG_IMX072
	{
		I2C_BOARD_INFO("imx072", 0x34),
	},
	#endif
	#ifdef CONFIG_MT9E013
	{
		I2C_BOARD_INFO("mt9e013", 0x6C >> 2),
	},
	#endif
	{
		I2C_BOARD_INFO("sc628a", 0x37),
	},
	#ifdef CONFIG_HUAWEI_SENSOR_MT9P017
	{
		I2C_BOARD_INFO("mt9p017", 0x6C),
	},
	#endif
	#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_S5K5CA
	{
		I2C_BOARD_INFO("s5k5ca", 0x2D),
	},
	#endif
	#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9T113
	{
		/* mt9t113 real i2c address is 0x3C*/
		I2C_BOARD_INFO("mt9t113", 0x3C),
	},
	#endif
	#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9D113
	{
		I2C_BOARD_INFO("mt9d113", 0x3C >> 1),
	},
	#endif
	#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9V113
	{
		I2C_BOARD_INFO("mt9v113", 0x3D),
	},
	#endif
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
    #ifndef CONFIG_HUAWEI_KERNEL
    /*Del uart1 register*/
	&msm_device_uart1,
    #endif
	&msm_device_nand,
	&msm_device_uart_dm1,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
};

static struct platform_device *surf_ffa_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
    #ifndef CONFIG_HUAWEI_KERNEL
    /*Del uart1 register*/
	&msm_device_uart1,
    #endif
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
	&usb_mass_storage_device,
	&rndis_device,
	&usb_diag_device,
	&usb_gadget_fserial_device,
	&android_pmem_audio_device,
	&msm_device_snd,
	&msm_device_adspdec,
	&msm_fb_device,
	&huawei_share_memory_device,
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
#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1
	&msm_camera_sensor_s5k4e1,
#endif
#ifdef CONFIG_IMX072
	&msm_camera_sensor_imx072,
#endif
#ifdef CONFIG_WEBCAM_OV9726
	&msm_camera_sensor_ov9726,
#endif
#ifdef CONFIG_MT9E013
	&msm_camera_sensor_mt9e013,
#endif
#ifdef CONFIG_HUAWEI_SENSOR_MT9P017
	&msm_camera_sensor_mt9p017,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_S5K5CA
	&msm_camera_sensor_s5k5ca,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9T113
	&msm_camera_sensor_mt9t113,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9D113
	&msm_camera_sensor_mt9d113,
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9V113
    &msm_camera_sensor_mt9v113,
#endif
#ifdef CONFIG_FB_MSM_MIPI_DSI
	&mipi_dsi_renesas_panel_device,
#endif
	&msm_kgsl_3d0,
/* for WCN2243 */
#if defined(CONFIG_BT) && defined(HUAWEI_BT_WCN2243)
	&msm_bt_power_device,
#endif

/* for BCM_4330 */
#if defined(CONFIG_BT) && defined(HUAWEI_BT_BCM4330)
    &msm_bt_power_device,
    &msm_bluesleep_device,	
#endif

/* default  */
#ifndef CONFIG_HUAWEI_KERNEL
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
	/* Registration device */
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
	/* delet one line */
    __u32 cust_addr = 0; 
	__u32 cust_size = 0;
	
#ifdef CONFIG_FRAMEBUF_SELF_ADAPT
	__u32 framebuf_addr = 0; 
	__u32 framebuf_size = 0;

	/*get framebuffer address and size*/
	get_frame_buffer_mem_region(&framebuf_addr, &framebuf_size);
	msm_fb_resources[0].start = framebuf_addr;
	msm_fb_resources[0].end = framebuf_addr + framebuf_size -1;
	
	pr_info("allocating %x bytes at %x for framebuffer\n", framebuf_size, framebuf_addr);
#else
    void *addr;
	/* delet 1 line */

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
#endif /* CONFIG_FRAMEBUF_SELF_ADAPT */

#ifdef CONFIG_HUAWEI_SHARE_MEMORY_DEVICE
	/*get framebuffer address and size*/
	get_cust_buffer_mem_region(&cust_addr, &cust_size);
	/* delet one line */
	if (cust_size) 
	{
		huawei_share_memory_resources[0].start = cust_addr;
		huawei_share_memory_resources[0].end = huawei_share_memory_resources[0].start +  cust_size - 1;
	}
#endif /* CONFIG_HUAWEI_SHARE_MEMORY_DEVICE */
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

#ifdef CONFIG_HUAWEI_KERNEL
/* set the mdp pmem by yres of lcd */
static unsigned __init set_msm_pmem_mdp_size(void)
{
    lcd_type lcd_y_res = LCD_IS_WVGA;

    /* get lcd_y_res from ATAG_LCD_Y_RES_FLAG */
    lcd_y_res =  atag_get_lcd_y_res();
    switch(lcd_y_res)
    {
        case LCD_IS_QVGA:
            pmem_mdp_size = MSM_PMEM_MDP_QVGA_SIZE;
            break;
        case LCD_IS_HVGA:
            pmem_mdp_size = MSM_PMEM_MDP_HVGA_SIZE;
            break;
        case LCD_IS_WVGA:
            pmem_mdp_size = MSM_PMEM_MDP_SIZE;
            break;
        default:
            pmem_mdp_size = MSM_PMEM_MDP_SIZE;
            break;
    }
    
    printk("%s: get lcd_y_res = %d. pmem_mdp_size=0x%x \n", __func__, lcd_y_res, pmem_mdp_size);
    return pmem_mdp_size;
}
#endif

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	/* set the mdp pmem by yres of lcd */
#ifdef CONFIG_HUAWEI_KERNEL
    /* for increasing the free memory in 256M, reduce the ADSP pmem. */
    if (strstr(boot_command_line, MEM_SIZE_256M_DESC_IN_CMDLINE))
    {
        android_pmem_adsp_pdata.size = MSM_PMEM_ADSP_SIZE_FOR_256M;
        pr_info("%s memory size is 256M, adsp pmem size=%#x\n", __func__, (int)android_pmem_adsp_pdata.size);
    }
    else
    {
        android_pmem_adsp_pdata.size = pmem_adsp_size;
        pr_info("%s memory size >256M, adsp pmem size=%#x\n", __func__, (int)android_pmem_adsp_pdata.size);
    }    
    android_pmem_pdata.size = set_msm_pmem_mdp_size();
#else
    android_pmem_adsp_pdata.size = pmem_adsp_size;
    android_pmem_pdata.size = pmem_mdp_size;        
#endif
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

#define GPIO_LCDC_BRDG_PD	128
#define GPIO_LCDC_BRDG_RESET_N	129

#define LCDC_RESET_PHYS		0x90008014
#ifndef CONFIG_HUAWEI_KERNEL
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

	if (cpu_is_msm7x25a() || cpu_is_msm7x25aa()) {
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
#ifndef CONFIG_HUAWEI_KERNEL
		if (machine_is_msm7x27a_surf()) {
#else
/* delete the judgement of board_id, this is public platform code. */
		{
#endif
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

static const char * const msm_fb_dsi_vreg[] = {
	"gp2",
	"msme1",
};

static const int msm_fb_dsi_vreg_mV[] = {
	2850,
	1800,
};

static struct vreg *dsi_vreg[ARRAY_SIZE(msm_fb_dsi_vreg)];
static int dsi_gpio_initialized;

static int mipi_dsi_panel_power(int on)
{
	int i, rc = 0;
	uint32_t lcdc_reset_cfg;

	/* I2C-controlled GPIO Expander -init of the GPIOs very late */
	if (!dsi_gpio_initialized) {
		pmapp_disp_backlight_init();

		rc = gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr");
		if (rc < 0) {
			pr_err("failed to request gpio_disp_pwr\n");
			return rc;
		}

#ifndef CONFIG_HUAWEI_KERNEL
		if (machine_is_msm7x27a_surf()) {
#else
/* delete the judgement of board_id, this is public platform code. */
		{
#endif
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

		for (i = 0; i < ARRAY_SIZE(msm_fb_dsi_vreg); i++) {
			dsi_vreg[i] = vreg_get(0, msm_fb_dsi_vreg[i]);

			if (IS_ERR(dsi_vreg[i])) {
				pr_err("%s: vreg get failed with : (%ld)\n",
					__func__, PTR_ERR(dsi_vreg[i]));
				goto fail_gpio2;
			}

			rc = vreg_set_level(dsi_vreg[i],
				msm_fb_dsi_vreg_mV[i]);

			if (rc < 0) {
				pr_err("%s: set regulator level failed "
					"with :(%d)\n",	__func__, rc);
				goto vreg_fail1;
			}
		}
		dsi_gpio_initialized = 1;
	}

/* delete the judgement of board_id, this is public platform code.
 * coordinate the code for a unitary code. 
 * so delete the old DTS numbers.
 */
#ifndef CONFIG_HUAWEI_KERNEL
		if (machine_is_msm7x27a_surf()) {
			gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
			gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);
		} else if (machine_is_msm7x27a_ffa()) {
			if (on) {
				/* This line drives an active low pin on FFA */
				rc = gpio_direction_output(GPIO_DISPLAY_PWR_EN,
					!on);
				if (rc < 0)
					pr_err("failed to set direction for "
						"display pwr\n");
			} else {
				gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN,
					!on);
				rc = gpio_direction_input(GPIO_DISPLAY_PWR_EN);
				if (rc < 0)
					pr_err("failed to set direction for "
						"display pwr\n");
			}
		}
#else
			gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
			gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);
#endif

		if (on) {
			gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 0);

/* delete the judgement of board_id, this is public platform code.
 * coordinate the code for a unitary code. 
 * so delete the old DTS numbers.
 */
#ifndef CONFIG_HUAWEI_KERNEL
			if (machine_is_msm7x27a_surf()) {
				lcdc_reset_cfg = readl_relaxed(lcdc_reset_ptr);
				rmb();
				lcdc_reset_cfg &= ~1;

				writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
				msleep(20);
				wmb();
				lcdc_reset_cfg |= 1;
				writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
			} else {
				gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N,
					0);
				msleep(20);
				gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N,
					1);
			}
#else
			lcdc_reset_cfg = readl_relaxed(lcdc_reset_ptr);
			rmb();
			lcdc_reset_cfg &= ~1;

			writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
			msleep(20);
			wmb();
			lcdc_reset_cfg |= 1;
			writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
#endif


			if (pmapp_disp_backlight_set_brightness(100))
				pr_err("backlight set brightness failed\n");
		} else {
			gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 1);

			if (pmapp_disp_backlight_set_brightness(0))
				pr_err("backlight set brightness failed\n");
		}

		/*Configure vreg lines */
		for (i = 0; i < ARRAY_SIZE(msm_fb_dsi_vreg); i++) {
			if (on) {
				rc = vreg_enable(dsi_vreg[i]);

				if (rc) {
					printk(KERN_ERR "vreg_enable: %s vreg"
						"operation failed\n",
						msm_fb_dsi_vreg[i]);

					goto vreg_fail2;
				}
			} else {
				rc = vreg_disable(dsi_vreg[i]);

				if (rc) {
					printk(KERN_ERR "vreg_disable: %s vreg "
						"operation failed\n",
						msm_fb_dsi_vreg[i]);
					goto vreg_fail2;
				}
			}
		}

	return rc;

vreg_fail2:
	if (on) {
		for (; i > 0; i--)
			vreg_disable(dsi_vreg[i - 1]);
	} else {
		for (; i > 0; i--)
			vreg_enable(dsi_vreg[i - 1]);
	}

	return rc;

vreg_fail1:
	for (; i > 0; i--)
		vreg_put(dsi_vreg[i - 1]);

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
	msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
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
#ifndef CONFIG_HUAWEI_KERNEL
	if (machine_is_msm7x27a_rumi3() || machine_is_msm7x27a_surf())
#else
/* delete the judgement of board_id, this is public platform code.
 * delete 3 row.
 */
#endif
		ebi2_cfg |= (1 << 4); /* CS2 */

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);

	/* Enable A/D MUX[bit 31] from EBI2_XMEM_CS2_CFG1 */
	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_XMEM_CS2_CFG1,
							 sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
#ifndef CONFIG_HUAWEI_KERNEL
	if (machine_is_msm7x27a_surf())
#else
/* delete the judgement of board_id, this is public platform code.
 * delete 3 row.
 */
#endif
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

/*this function reset touch panel */
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

/*this function return reset gpio at 7x30 platform */
int get_touch_reset_pin(void)
{
	int ret = MSM_7x27A_TOUCH_RESET_PIN;
	
	return ret;
}

/*we use this to detect the probe is detected*/
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

/*this function get the tp  resolution*/
static int get_phone_version(struct tp_resolution_conversion *tp_resolution_type)
{	
	/*move touch_reset function to outside */
	if (machine_is_msm7x27a_U8815() ||
	    machine_is_msm7x27a_C8820() ||
		machine_is_msm7x27a_C8825D() )
	{
		tp_resolution_type->lcd_x = LCD_X_WVGA;
		tp_resolution_type->lcd_y = LCD_Y_WVGA;   
		tp_resolution_type->lcd_all = LCD_ALL_WVGA_4INCHTP;
	}
	/* add U8655_EMMC, use the u8655 configuration */
	//add C8655 T0, use the u8655 configuration
	else if (machine_is_msm7x27a_U8655() ||machine_is_msm7x27a_U8655_EMMC() || machine_is_msm7x27a_C8655_NAND())
	{
		tp_resolution_type->lcd_x = LCD_X_HVGA;
		tp_resolution_type->lcd_y = LCD_Y_HVGA;   
		tp_resolution_type->lcd_all = LCD_ALL_HVGA_35INCHTP;
	}
	/* Add M660 */
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
    else if (machine_is_msm7x27a_U8661())
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
/* this function is used to reset pn544 by controlling the ven pin */
static int pn544_ven_reset(void)
{
	int ret=0;
	int gpio_config=0;
	
	gpio_config = GPIO_CFG(GPIO_NFC_VEN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
	ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	ret = gpio_request(GPIO_NFC_VEN, "gpio 113 for NFC pn544");
	ret = gpio_direction_output(GPIO_NFC_VEN,0);
	/* pull up first, then pull down for 10 ms, and enable last */
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
	gpio_set_value(pn544_download_gpio, 1);
	mdelay(5);
	gpio_set_value(pn544_download_gpio, 0);
	mdelay(5);
	return 0;	
}

static int pn544_fw_download_pull_high(void)
{
	gpio_set_value(pn544_download_gpio, 0);
	mdelay(5);
	gpio_set_value(pn544_download_gpio, 1);
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

static int pn544_clock_output_mode_ctrl(void)
{
       const char * id = "nfcp";
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_D1,
		PMAPP_CLOCK_VOTE_PIN_CTRL);
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

#ifdef CONFIG_HUAWEI_SYNAPTICS_RMI_TOUCH
#if defined(CONFIG_RMI4_I2C)

struct syna_gpio_data {
	u16 gpio_number;
	char* gpio_name;
};

/* The board file currently knows about the following board numbers:
 *	2000
 *  2100
 *  3000
 *  3200
 *  2202
 * To enable multiple module testing, we build up a bitmask indicating which
 * boards are attached.
 */

#define SYNA_IC2000 0x00000001
#define SYNA_IC2100 0x00000002
#define SYNA_IC3000 0x00000004
#define SYNA_IC3200 0x00000008
#define SYNA_IC2202 0x00000010

#define SYNA_BOARDS (SYNA_IC2202 | SYNA_IC3200 | SYNA_IC3000 | SYNA_IC2100 | SYNA_IC2000) 

#define SYNA_BOARD_PRESENT(board_mask) (SYNA_BOARDS & board_mask)

static int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	int retval=0;
	struct syna_gpio_data *data = gpio_data;

	if (configure) {
		retval = gpio_request(data->gpio_number, "rmi4_attn");
		if (retval) {
			pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			return retval;
		}

		retval = gpio_tlmm_config(GPIO_CFG(data->gpio_number, 
						0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, 
						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (retval) {
			pr_err("%s: Failed to config attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			gpio_free(data->gpio_number);
			return retval;
		}
		retval = gpio_direction_input(data->gpio_number);
		if (retval) {
			pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			gpio_free(data->gpio_number);
		}
	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
		       __func__, data->gpio_number);
	}

	return retval;
}

/****************** START BOARD RELATED STUFF ***********************/

	/* This configures the sensor for landscape orientation,
	 * with the tail pointing toward the user's left, so
	 * the manufacturer logo is at the top of the sensor.
	 */

#define IC2000_ADDR	0x24
#define IC2100_ADDR	0x70

static struct syna_gpio_data touch_gpiodata = {
	.gpio_number = MSM_7X27A_TOUCH_INT_PIN,
	.gpio_name = "sdmmc2_clk.gpio_82",
};

static struct rmi_device_platform_data hw_touch_data = {
	.driver_name = "rmi-generic",
	.attn_gpio = MSM_7X27A_TOUCH_INT_PIN,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.gpio_data = &touch_gpiodata,
	.gpio_config = synaptics_touchpad_gpio_setup,
	.reset_delay_ms = 100,
	.hardware_reset = touch_reset,
	.get_phone_version = get_phone_version,
};

#endif
#endif


static struct i2c_board_info msm7x27a_i2c_board_info[] __initdata = 
{
#ifdef CONFIG_HUAWEI_MELFAS_TOUCHSCREEN
	{
		I2C_BOARD_INFO("melfas-ts", 0x23),
		.platform_data = &touch_hw_data,
		.irq = MSM_GPIO_TO_INT(MSM_7X27A_TOUCH_INT_PIN),
		/*support multi point*/
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
	/* synaptics IC s2000 for U8661
	 * I2C ADDR	: 0x24
	 */
	{
		I2C_BOARD_INFO("Synaptics_rmi", 0x24),
		.platform_data = &touch_hw_data,
		.irq = MSM_GPIO_TO_INT(MSM_7X27A_TOUCH_INT_PIN),
        .flags = true,
	},
#endif
#ifdef CONFIG_HUAWEI_SYNAPTICS_RMI_TOUCH
     {
         I2C_BOARD_INFO("rmi-i2c", IC2100_ADDR),
        .platform_data = &hw_touch_data,
        .flags = true,
     },
	/* synaptics IC s2000 for U8661
	 * I2C ADDR	: 0x24
	 */
	{
		I2C_BOARD_INFO("rmi-i2c", IC2000_ADDR),
		.platform_data = &hw_touch_data,
        .flags = true,
	},
#endif
#ifdef CONFIG_ACCELEROMETER_ST_L1S35DE
    {
        I2C_BOARD_INFO("gs_st", 0x70 >> 1),  // actual address 0x38, fake address (0x38 << 1)
        .platform_data = &gs_st_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },	
  
/* Add issue-fixed for c8600 t1 board: 
 *    issue: SDO pin of ST_L1S35DE is pulled up by hardware guy.
 *       fixed way: Change i2c addr from 0x38 to 0x3A */     
    {
        I2C_BOARD_INFO("gs_st", 0x3A >> 1),	  
       .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },	

#endif
#ifdef CONFIG_SENSORS_AKM8973
    {
        I2C_BOARD_INFO("akm8973", 0x3c >> 1),//7 bit addr, no write bit
        .irq = MSM_GPIO_TO_INT(107)
    },
#endif 
	#ifdef CONFIG_HUAWEI_FEATURE_GYROSCOPE_L3G4200DH
    {   
		I2C_BOARD_INFO("l3g4200d", 0x68),  
		.platform_data = &gy_l3g4200d_platform_data,
    },
	#endif
/*Register i2c information for flash tps61310*/
#ifdef CONFIG_HUAWEI_FEATURE_TPS61310
	{
		I2C_BOARD_INFO("tps61310" , 0x33),
	},
#endif

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ST_LSM303DLH
	{
		I2C_BOARD_INFO("st303_gs", 0x64 >> 1),         
		.platform_data = &st303_gs_platform_data,
		//.irq = MSM_GPIO_TO_INT() 
	},
	{
		I2C_BOARD_INFO("st303_compass", 0x3e >> 1),/* actual i2c address is 0x3c    */             
		//.irq = MSM_GPIO_TO_INT() 
	},
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_MMA8452
    {
        I2C_BOARD_INFO("gs_mma8452", 0x38 >> 1),
        .platform_data = &gs_mma8452_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
#endif	
/*fack address,because IIC is interrupt with bluetooth*/
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
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_9900
	{   
		I2C_BOARD_INFO("aps-9900", 0x39),
        .irq = MSM_GPIO_TO_INT(MSM_7X30_APS9900_INT),
        .platform_data = &aps9900_hw_data,
	},
#endif
#ifdef CONFIG_QWERTY_KEYPAD_ADP5587
	{
		I2C_BOARD_INFO("adp5587", 0x34),
		.irq = MSM_GPIO_TO_INT(40)
	},
#endif 
#ifdef CONFIG_HUAWEI_FEATURE_TPA2028D1_AMPLIFIER
    {   
		I2C_BOARD_INFO("tpa2028d1", 0x58),  
        .platform_data = &audio_amplifier_data,
    },
#endif
#ifdef CONFIG_HUAWEI_NFC_PN544
	{
		I2C_BOARD_INFO(PN544_DRIVER_NAME, PN544_I2C_ADDR),
		.irq = MSM_GPIO_TO_INT(GPIO_NFC_INT),
		.platform_data = &pn544_hw_data,
	},
#endif
/*add proximity fuction*/
#ifdef CONFIG_PROXIMITY_EVERLIGHT_APS_12D
	{   
		I2C_BOARD_INFO("aps-12d", 0x88 >> 1),  
	},
#endif
};
#else
#define ATMEL_TS_I2C_NAME "maXTouch"
static struct vreg *vreg_l12;
static struct vreg *vreg_s3;

#define ATMEL_TS_GPIO_IRQ 82

static int atmel_ts_power_on(bool on)
{
	int rc;

	rc = on ? vreg_enable(vreg_l12) : vreg_disable(vreg_l12);
	if (rc) {
		pr_err("%s: vreg %sable failed (%d)\n",
		       __func__, on ? "en" : "dis", rc);
		return rc;
	}

	rc = on ? vreg_enable(vreg_s3) : vreg_disable(vreg_s3);
	if (rc) {
		pr_err("%s: vreg %sable failed (%d) for S3\n",
		       __func__, on ? "en" : "dis", rc);
		!on ? vreg_enable(vreg_l12) : vreg_disable(vreg_l12);
		return rc;
	}
	/* vreg stabilization delay */
	msleep(50);
	return 0;
}

static int atmel_ts_platform_init(struct i2c_client *client)
{
	int rc;

	vreg_l12 = vreg_get(NULL, "gp2");
	if (IS_ERR(vreg_l12)) {
		pr_err("%s: vreg_get for L2 failed\n", __func__);
		return PTR_ERR(vreg_l12);
	}

	rc = vreg_set_level(vreg_l12, 2850);
	if (rc) {
		pr_err("%s: vreg set level failed (%d) for l2\n",
		       __func__, rc);
		goto vreg_put_l2;
	}

	vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(vreg_s3)) {
		pr_err("%s: vreg_get for S3 failed\n", __func__);
		rc = PTR_ERR(vreg_s3);
		goto vreg_put_l2;
	}

	rc = vreg_set_level(vreg_s3, 1800);
	if (rc) {
		pr_err("%s: vreg set level failed (%d) for S3\n",
		       __func__, rc);
		goto vreg_put_s3;
	}

	rc = gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto vreg_put_s3;
	}

	/* configure touchscreen interrupt gpio */
	rc = gpio_request(ATMEL_TS_GPIO_IRQ, "atmel_maxtouch_gpio");
	if (rc) {
		pr_err("%s: unable to request gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto ts_gpio_tlmm_unconfig;
	}

	rc = gpio_direction_input(ATMEL_TS_GPIO_IRQ);
	if (rc < 0) {
		pr_err("%s: unable to set the direction of gpio %d\n",
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
vreg_put_s3:
	vreg_put(vreg_s3);
vreg_put_l2:
	vreg_put(vreg_l12);
	return rc;
}

static int atmel_ts_platform_exit(struct i2c_client *client)
{
	gpio_free(ATMEL_TS_GPIO_IRQ);
	gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	vreg_disable(vreg_s3);
	vreg_put(vreg_s3);
	vreg_disable(vreg_l12);
	vreg_put(vreg_l12);
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

static struct mxt_platform_data atmel_ts_pdata = {
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
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
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

#define LED_GPIO_PDM		96
#define UART1DM_RX_GPIO		45

static int __init msm7x27a_init_ar6000pm(void)
{
	return platform_device_register(&msm_wlan_ar6000_pm_device);
}

#ifdef CONFIG_HUAWEI_KERNEL
static unsigned char network_cdma = 0;
static unsigned char runmode_factory = 0;
/* the function interface to check network in kernel */
unsigned char network_is_cdma(void)
{
  return network_cdma;
}
/* the function interface to check factory mode in kernel */
unsigned char runmode_is_factory(void)
{
  return runmode_factory;
}

/*  
 * Get factory parameter from share memory.
 */
static void proc_factory_para(void)
{
    smem_huawei_vender * factory_para_ptr;
    /* now the smem_id_vendor0 smem id is a new struct */
    factory_para_ptr = (smem_huawei_vender*)smem_alloc(SMEM_ID_VENDOR0, sizeof(smem_huawei_vender));
    if (!factory_para_ptr)
    {
    	printk("zdy: %s: Can't find factory parameter\n", __func__);
        return;
    }

    if (NETWORK_CDMA == factory_para_ptr->network_type)
    { 
      network_cdma = 1;
      printk("zdy: %s: network_cdma\n", __func__);
    }
    else
    {
      network_cdma = 0;
      printk("zdy: %s: network_umts\n", __func__);
    }
    
    if (MAGIC_NUMBER_FACTORY == factory_para_ptr->run_mode)
    {  
      runmode_factory = 1;
      printk("zdy: %s: runmode_factory\n", __func__);
    }
    else
    {
      runmode_factory = 0;
      printk("zdy: %s: runmode_normal\n", __func__);
    }
    
    if (bootimage_is_recovery())
    {
      printk("zdy: %s: bootmode_recovery\n", __func__);
    }
    else
    {  
      printk("zdy: %s: bootmode_system\n", __func__);
    }
} 
#endif
/* add virtual keys fucntion */

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
/* add melfas virtual key node */
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
#ifdef CONFIG_HUAWEI_CAMERA
static void camera_sensor_pwd_config(void)
{
	/*distinguish the camera pwd gpio by lcd_interface_type*/
	if(LCD_IS_RGB == get_hw_lcd_interface_type())
	{
		int gpio_pwd = 37;
		/*camera pwd gpio for M660 is 32*/
		if(machine_is_msm7x27a_M660())
		{
			gpio_pwd = 32;
		}
		pr_err("camera sensor pwd gpio is %d\n",gpio_pwd);
		
		/*config the camera pwd gpio*/
		msm_camera_sensor_mt9e013_data.sensor_pwd = gpio_pwd;
		msm_camera_sensor_mt9p017_data.sensor_pwd = gpio_pwd;
		msm_camera_sensor_s5k4e1_data.sensor_pwd = gpio_pwd;
		msm_camera_sensor_s5k5ca_data.sensor_pwd = gpio_pwd;
		msm_camera_sensor_mt9t113_data.sensor_pwd = gpio_pwd;
		msm_camera_sensor_mt9d113_data.sensor_pwd = gpio_pwd;
	}
}
#endif

#ifdef HUAWEI_BT_BCM4330
static void bt_wake_msm_config(void)
{
    /*distinguish the bt_wake_msm gpio by get_hw_bt_wakeup_gpio_type*/
    hw_bt_wakeup_gpio_type bt_wake_msm_gpio =  get_hw_bt_wakeup_gpio_type();
    if( bt_wake_msm_gpio == HW_BT_WAKEUP_GPIO_IS_27)
    {
        bluesleep_resources[0].start = bt_wake_msm_gpio;
        bluesleep_resources[0].end = bt_wake_msm_gpio;
        bluesleep_resources[2].start = MSM_GPIO_TO_INT(bt_wake_msm_gpio);
        bluesleep_resources[2].end = MSM_GPIO_TO_INT(bt_wake_msm_gpio);
        bt_config_bcm4330_power_on[5].gpio_cfg = GPIO_CFG(bt_wake_msm_gpio, GPIO_BT_FUN_0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL ,  GPIO_CFG_2MA );
        bt_config_bcm4330_power_off[5].gpio_cfg = GPIO_CFG(bt_wake_msm_gpio, GPIO_BT_FUN_0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN ,  GPIO_CFG_2MA );
    }
    printk(KERN_DEBUG "bt_wake_msm_gpio = %d\n", bt_wake_msm_gpio); 
}
#endif

static void __init msm7x2x_init(void)
{
	msm7x2x_misc_init();

	/* Common functions for SURF/FFA/RUMI3 */
	msm_device_i2c_init();
	msm7x27a_init_ebi2();
	msm7x27a_cfg_uart2dm_serial();
/* The following config was used for WCN2243 only. */
#ifdef CONFIG_HUAWEI_KERNEL
#if defined(CONFIG_SERIAL_MSM_HS) && defined(HUAWEI_BT_WCN2243)
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(UART1DM_RX_GPIO);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
#else
#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(UART1DM_RX_GPIO);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
#endif
#ifdef CONFIG_USB_AUTO_INSTALL
	proc_usb_para();
#endif  /* #ifdef CONFIG_USB_AUTO_INSTALL */

	if (machine_is_msm7x27a_rumi3()) {
		platform_add_devices(rumi_sim_devices,
				ARRAY_SIZE(rumi_sim_devices));
	}
#ifndef CONFIG_HUAWEI_KERNEL
	if (machine_is_msm7x27a_surf() || machine_is_msm7x27a_ffa()) {
#else
/* delete the judgement of board_id, this is public platform code. */
	{
#endif
 
#ifdef CONFIG_HUAWEI_KERNEL
	/* init the factory para in kernel */
    proc_factory_para();
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
#ifdef CONFIG_HUAWEI_CAMERA
		/*before camera probe, config the camera pwd gpio*/
		camera_sensor_pwd_config();
#endif
#ifdef HUAWEI_BT_BCM4330
                /*before bt probe, config the bt_wake_msm gpio*/
                bt_wake_msm_config();
#endif
		platform_add_devices(msm_footswitch_devices,
			     msm_num_footswitch_devices);
		platform_add_devices(surf_ffa_devices,
				ARRAY_SIZE(surf_ffa_devices));
		msm_fb_add_devices();
		/* Ensure ar6000pm device is registered before MMC/SDC */
		msm7x27a_init_ar6000pm();
        /* init buffer for atheros wlan while system up */
        msm7x27a_init_ath_buf();
#ifdef CONFIG_MMC_MSM
		msm7x27a_init_mmc();
#endif
#ifdef CONFIG_USB_EHCI_MSM_72K
		msm7x2x_init_host();
#endif
	}

	msm_pm_set_platform_data(msm7x27a_pm_data,
				ARRAY_SIZE(msm7x27a_pm_data));

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
	register_i2c_devices();
#endif
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
/* init bt power according to the chip used. */
#ifdef HUAWEI_BT_BCM4330
        bt_bcm4330_power_init();
#endif

#ifdef HUAWEI_BT_WCN2243
        bt_power_init();
#endif
/* default  */ 	
#ifndef CONFIG_HUAWEI_KERNEL
	   bt_power_init();
#endif
#endif
#ifndef CONFIG_HUAWEI_KERNEL
	if (cpu_is_msm7x25a() || cpu_is_msm7x25aa()) {
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

	i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID,
			i2c_camera_devices,
			ARRAY_SIZE(i2c_camera_devices));
//add keypad driver
#if defined(CONFIG_HUAWEI_GPIO_KEYPAD)
    /* because all production KEY_VOLUMEUP and KEY_VOLUMEDOWN sameness,
     * so use keypad_device_default ,
     * del 4 row,
     * for tending to promote code unity.
    */ 
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
#ifdef CONFIG_HUAWEI_KERNEL
/* initialize vibrator driver */
		msm_init_pmic_vibrator();
#endif
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
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_SURF, "QCT MSM7x27a SURF")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_FFA, "QCT MSM7x27a FFA")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_UMTS, "MSM7x27a UMTS SURF")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_CDMA, "MSM7x27a CDMA SURF")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_U8815, "MSM7x27a U8815 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_U8655, "MSM7x27a U8655 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_U8655_EMMC, "MSM7x27a U8655Pro BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_C8655_NAND, "MSM7x27a C8655_NAND BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_U8185, "MSM7x27a U8185 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END

MACHINE_START(MSM7X27A_M660, "MSM7x27a M660 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END

MACHINE_START(MSM7X27A_C8820, "MSM7x27a C8820 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END

MACHINE_START(MSM7X27A_C8825D, "MSM7x27a C8825D BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
MACHINE_START(MSM7X27A_U8661, "MSM7x27a U8661 BOARD")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
MACHINE_END
