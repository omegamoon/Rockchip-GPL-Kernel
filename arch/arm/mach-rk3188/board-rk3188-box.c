/*
 *
 * Copyright (C) 2012 ROCKCHIP, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/ion.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <mach/dvfs.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <linux/rk_fb.h>
#include <linux/regulator/machine.h>
#include <linux/rfkill-rk.h>
#include <linux/sensor-dev.h>
#include <linux/mfd/tps65910.h>
#include <linux/regulator/act8846.h>
#include <linux/regulator/rk29-pwm-regulator.h>

#define OMEGAMOON_CHANGED		1

#if defined(CONFIG_CT36X_TS)
#include <linux/ct36x.h>
#endif
#include <plat/efuse.h>

#if defined(CONFIG_MFD_RK610)
#include <linux/mfd/rk610_core.h>
#endif
//$_rbox_$_modify_$ zhengyang modified for box
#include <linux/display-sys.h>
#include <linux/rk_fb.h>
//$_rbox_$_modify_$ zhengyang modified end
#if defined(CONFIG_SPIM_RK29)
#include "../../../drivers/spi/rk29_spim.h"
#endif
#if defined(CONFIG_MT6229)
#include <linux/mt6229.h>
#endif
#if defined(CONFIG_GPS_RK)
#include "../../../drivers/misc/gps/rk_gps/rk_gps.h"
#endif

/*$_rbox_$_modify_$_huangzhibao begin*/
#ifdef CONFIG_RK_REMOTECTL
#include <mach/remotectl.h>
#endif
/*$_rbox_$_modify_$_huangzhibao endif*/
#if defined(CONFIG_MT6620)
#include <linux/gps.h>
#endif

#include "../mach-rk30/board-rk3168-ds1006h-camera.c"
#include <plat/key.h>

#ifdef CONFIG_RK29_VMAC
#include "../mach-rk30/board-rk31-vmac.c"
#endif

static struct rk29_keys_button key_button[] = {
	{
		.desc	= "play",
		.code	= KEY_POWER,
		.gpio	= RK30_PIN0_PA4, 
		.active_low = PRESS_LEV_LOW,
		.wakeup	= 1,
	},
	{
		.desc	= "esc",
		.code	= KEY_BACK,
		.adc_value	= 1,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
};
struct rk29_keys_platform_data rk29_keys_pdata = {
	.buttons	= key_button,
	.nbuttons	= ARRAY_SIZE(key_button),
	.chn	= 1,  //chn: 0-7, if do not use ADC,set 'chn' -1
};

#if defined(CONFIG_CT36X_TS)

#define TOUCH_MODEL		363
#define TOUCH_MAX_X		1280
#define TOUCH_MAX_y		800
#define TOUCH_RESET_PIN		RK30_PIN0_PB6
#define TOUCH_INT_PIN		RK30_PIN1_PB7

static struct ct36x_platform_data ct36x_info = {
	.model   = TOUCH_MODEL,
	.x_max   = TOUCH_MAX_X,
	.y_max   = TOUCH_MAX_y,

	.rst_io = {
		.gpio = TOUCH_RESET_PIN,
		.active_low = 1,
	},
	.irq_io = {
		.gpio = TOUCH_INT_PIN,
		.active_low = 1,
	},
	.orientation = {1, 0, 1, 0},
};
#endif
static struct spi_board_info board_spi_devices[] = {
};

/***********************************************************
*	rk30  backlight
************************************************************/
#ifdef CONFIG_BACKLIGHT_RK29_BL
#define PWM_ID            3
#define PWM_MODE	  PWM3
#define PWM_EFFECT_VALUE  1

#define LCD_DISP_ON_PIN

#ifdef  LCD_DISP_ON_PIN
#define BL_EN_PIN         RK30_PIN0_PA2
#define BL_EN_VALUE       GPIO_HIGH
#endif
static int rk29_backlight_io_init(void)
{
	int ret = 0;

	iomux_set(PWM_MODE);
#ifdef  LCD_DISP_ON_PIN
	ret = gpio_request(BL_EN_PIN, NULL);
	if (ret != 0) {
		gpio_free(BL_EN_PIN);
	}

	gpio_direction_output(BL_EN_PIN, 0);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif

	return ret;
}

static int rk29_backlight_io_deinit(void)
{
	int ret = 0, pwm_gpio;

#ifdef  LCD_DISP_ON_PIN
	gpio_free(BL_EN_PIN);
#endif

	pwm_gpio = iomux_mode_to_gpio(PWM_MODE);
	gpio_request(pwm_gpio, NULL);
	gpio_direction_output(pwm_gpio, GPIO_LOW);
	return ret;
}

static int rk29_backlight_pwm_suspend(void)
{
	int ret = 0, pwm_gpio;

	pwm_gpio = iomux_mode_to_gpio(PWM_MODE);
	if (gpio_request(pwm_gpio, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return -1;
	}
	gpio_direction_output(pwm_gpio, GPIO_LOW);
#ifdef  LCD_DISP_ON_PIN
	gpio_direction_output(BL_EN_PIN, 0);
	gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
#endif
	return ret;
}

static int rk29_backlight_pwm_resume(void)
{
	int pwm_gpio = iomux_mode_to_gpio(PWM_MODE);

	gpio_free(pwm_gpio);
	iomux_set(PWM_MODE);
#ifdef  LCD_DISP_ON_PIN
	msleep(30);
	gpio_direction_output(BL_EN_PIN, 1);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif
	return 0;
}

static struct rk29_bl_info rk29_bl_info = {
        .min_brightness = 65,
        .max_brightness = 150,
        .brightness_mode =BRIGHTNESS_MODE_CONIC,
	.pre_div = 40 * 1000,  // pwm output clk: 40k;
	.pwm_id = PWM_ID,
	.bl_ref = PWM_EFFECT_VALUE,
	.io_init = rk29_backlight_io_init,
	.io_deinit = rk29_backlight_io_deinit,
	.pwm_suspend = rk29_backlight_pwm_suspend,
	.pwm_resume = rk29_backlight_pwm_resume,
};

static struct platform_device rk29_device_backlight = {
	.name	= "rk29_backlight",
	.id 	= -1,
	.dev	= {
		.platform_data  = &rk29_bl_info,
	}
};

#endif

/*MMA8452 gsensor*/
#if defined (CONFIG_GS_MMA8452)
#define MMA8452_INT_PIN   RK30_PIN0_PB7

static int mma8452_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data mma8452_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = mma8452_init_platform_hw,
        .orientation = {-1, 0, 0, 0, 0, -1, 0, 1, 0},
};
#endif
#if defined (CONFIG_GS_LIS3DH)
#define LIS3DH_INT_PIN   RK30_PIN0_PB7

static int lis3dh_init_platform_hw(void)
{

        return 0;
}

static struct sensor_platform_data lis3dh_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = lis3dh_init_platform_hw,
	.orientation = {-1, 0, 0, 0, 0, -1, 0, 1, 0},
};
#endif

#if (defined(CONFIG_SENSORS_AK8963) || defined(CONFIG_SENSORS_AK8963_MODULE)) 
static struct akm8963_platform_data akm_platform_data_8963 = { 
                 .gpio_DRDY      = RK30_PIN3_PD7, 
                 .gpio_RST        = 0, 
                 .layout              = 3, 
                 .outbit           = 1, 
}; 
#endif

#if defined(CONFIG_LS_PHOTORESISTOR)
static struct sensor_platform_data light_photoresistor_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 0,
        .address = 2   ,
	.poll_delay_ms = 200,
};
#endif

#if defined (CONFIG_COMPASS_AK8975)
static struct sensor_platform_data akm8975_info =
{
	.type = SENSOR_TYPE_COMPASS,
	.irq_enable = 1,
	.poll_delay_ms = 30,
	.m_layout = 
	{
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	}
};

#endif

#if defined(CONFIG_MT6229)
static int mt6229_io_init(void)
{
      #if 0
	rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
      k30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	rk30_mux_api_set(GPIO2B7_LCDC1DATA15_SMCADDR19_HSADCDATA7_NAME, GPIO2B_GPIO2B7);
	rk30_mux_api_set(GPIO2C0_LCDCDATA16_GPSCLK_HSADCCLKOUT_NAME, GPIO2C_GPIO2C0);
	rk30_mux_api_set(GPIO2C1_LCDC1DATA17_SMCBLSN0_HSADCDATA6_NAME, GPIO2C_GPIO2C1);
	rk30_mux_api_set(GPIO2C1_LCDC1DATA17_SMCBLSN0_HSADCDATA6_NAME, GPIO2C_GPIO2C1);
      #endif
	 return 0;
}

static int mt6229_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mt6229_data rk29_mt6229_info = {
	.io_init = mt6229_io_init,
  	.io_deinit = mt6229_io_deinit,
	.modem_power_en = RK30_PIN0_PC6,
	.bp_power = RK30_PIN2_PD5,
	.modem_usb_en = RK30_PIN0_PC7,
	.modem_uart_en = RK30_PIN2_PD4,
	.bp_wakeup_ap = RK30_PIN0_PC5,
	.ap_ready = RK30_PIN0_PC4,

};
struct platform_device rk29_device_mt6229 = {	
        .name = "mt6229",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk29_mt6229_info,
	}    	
    };
#endif

#if defined(CONFIG_GYRO_L3G4200D)

#include <linux/l3g4200d.h>
#define L3G4200D_INT_PIN  RK30_PIN0_PB4

static int l3g4200d_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data l3g4200d_info = {
	.type = SENSOR_TYPE_GYROSCOPE,
	.irq_enable = 1,
	.poll_delay_ms = 0,
	.orientation = {1, 0, 0 , 0 , -1, 0, 0, 0, -1},
	.init_platform_hw = l3g4200d_init_platform_hw,
	.x_min = 40,//x_min,y_min,z_min = (0-100) according to hardware
	.y_min = 40,
	.z_min = 20,
};

#endif

#ifdef CONFIG_LS_CM3217
static struct sensor_platform_data cm3217_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 0,
	.poll_delay_ms = 500,
};

#endif

#ifdef CONFIG_FB_ROCKCHIP

#define LCD_CS_PIN         INVALID_GPIO
#define LCD_CS_VALUE       GPIO_HIGH

#define LCD_EN_PIN         RK30_PIN0_PB0
#define LCD_EN_VALUE       GPIO_LOW

static int rk_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
	int ret = 0;

	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_CS_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_CS_PIN);
			printk(KERN_ERR "request lcd cs pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_CS_PIN, LCD_CS_VALUE);
		}
	}

	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_EN_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_EN_PIN);
			printk(KERN_ERR "request lcd en pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_EN_PIN, LCD_EN_VALUE);
		}
	}
	return 0;
}
static int rk_fb_io_disable(void)
{
	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, !LCD_CS_VALUE);
	}
	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, !LCD_EN_VALUE);
	}
	return 0;
}
static int rk_fb_io_enable(void)
{
	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	}
	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
	}
	return 0;
}

#ifdef OMEGAMOON_CHANGED

#if defined(CONFIG_LCDC0_RK3188)
struct rk29fb_info lcdc0_screen_info = {
#if defined(CONFIG_RK_LCDC0_AS_PRIMARY)
	.prop      = PRMRY,		//primary display device
#else
	.prop	   = EXTEND,    //extend display device
#endif
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,
};
#endif

#if defined(CONFIG_LCDC1_RK3188)
struct rk29fb_info lcdc1_screen_info = {
#if defined(CONFIG_RK_LCDC1_AS_PRIMARY)
	.prop      = PRMRY,		//primary display device
#else
	.prop	   = EXTEND,    //extend display device
#endif
    .lcd_info  = NULL,
    .set_screen_info = set_lcd_info,
};
#endif

#else // Omegamoon >> Original code

#if defined(CONFIG_LCDC0_RK3188)
struct rk29fb_info lcdc0_screen_info = {
	.prop           = EXTEND,       //extend display device
       .lcd_info  = NULL,
       .set_screen_info = set_lcd_info,

};
#endif

#if defined(CONFIG_LCDC1_RK3188)
struct rk29fb_info lcdc1_screen_info = {
	.prop	   = PRMRY,		//primary display device
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,
	
};
#endif

#endif // OMEGAMOON_CHANGED

static struct resource resource_fb[] = {
	[0] = {
		.name  = "fb0 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "ipp buf",  //for rotate
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = "fb2 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device device_fb = {
	.name		= "rk-fb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_fb),
	.resource	= resource_fb,
};
#endif

#if defined(CONFIG_LCDC0_RK3188)
static struct resource resource_lcdc0[] = {
	[0] = {
		.name  = "lcdc0 reg",
		.start = RK30_LCDC0_PHYS,
		.end   = RK30_LCDC0_PHYS + RK30_LCDC0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	
	[1] = {
		.name  = "lcdc0 irq",
		.start = IRQ_LCDC0,
		.end   = IRQ_LCDC0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc0 = {
	.name		  = "rk30-lcdc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(resource_lcdc0),
	.resource	  = resource_lcdc0,
	.dev 		= {
		.platform_data = &lcdc0_screen_info,
	},
};
#endif
#if defined(CONFIG_LCDC1_RK3188) 
static struct resource resource_lcdc1[] = {
	[0] = {
		.name  = "lcdc1 reg",
		.start = RK30_LCDC1_PHYS,
		.end   = RK30_LCDC1_PHYS + RK30_LCDC1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "lcdc1 irq",
		.start = IRQ_LCDC1,
		.end   = IRQ_LCDC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc1 = {
	.name		  = "rk30-lcdc",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(resource_lcdc1),
	.resource	  = resource_lcdc1,
	.dev 		= {
		.platform_data = &lcdc1_screen_info,
	},
};
#endif

#if defined(CONFIG_MFD_RK610)
#define RK610_RST_PIN 			RK30_PIN3_PB2
static int rk610_power_on_init(void)
{
	int ret;
	if(RK610_RST_PIN != INVALID_GPIO)
	{
		ret = gpio_request(RK610_RST_PIN, "rk610 reset");
		if (ret)
		{
			printk(KERN_ERR "rk610_control_probe request gpio fail\n");
		}
		else 
		{
			gpio_direction_output(RK610_RST_PIN, GPIO_HIGH);
			msleep(100);
			gpio_direction_output(RK610_RST_PIN, GPIO_LOW);
			msleep(100);
	    		gpio_set_value(RK610_RST_PIN, GPIO_HIGH);
		}
	}

	return 0;
	
}


static struct rk610_ctl_platform_data rk610_ctl_pdata = {
	.rk610_power_on_init = rk610_power_on_init,
};
#endif

#ifdef CONFIG_SND_SOC_RK610
static int rk610_codec_io_init(void)
{
//if need iomux.
//Must not gpio_request
	return 0;
}

static struct rk610_codec_platform_data rk610_codec_pdata = {
	.spk_ctl_io = RK30_PIN2_PD7,
	.io_init = rk610_codec_io_init,
	.boot_depop = 1,
};
#endif

#ifdef CONFIG_RK_HDMI
#define RK_HDMI_RST_PIN 			RK30_PIN3_PB2
static int rk_hdmi_power_init(void)
{
	int ret;

	if(RK_HDMI_RST_PIN != INVALID_GPIO)
	{
		if (gpio_request(RK_HDMI_RST_PIN, NULL)) {
			printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
			return -1;
		}
		gpio_direction_output(RK_HDMI_RST_PIN, GPIO_LOW);
		gpio_set_value(RK_HDMI_RST_PIN, GPIO_LOW);
		msleep(100);
		gpio_set_value(RK_HDMI_RST_PIN, GPIO_HIGH);
		msleep(50);
	}
	return 0;
}
static struct rk_hdmi_platform_data rk_hdmi_pdata = {
	//.io_init = rk_hdmi_power_init,
};
#endif
#ifdef CONFIG_ION
#define ION_RESERVE_SIZE        (120 * SZ_1M)
static struct ion_platform_data rk30_ion_pdata = {
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_NOR_HEAP_ID,
			.name = "norheap",
			.size = ION_RESERVE_SIZE,
		}
	},
};

static struct platform_device device_ion = {
	.name = "ion-rockchip",
	.id = 0,
	.dev = {
		.platform_data = &rk30_ion_pdata,
	},
};
#endif

/**************************************************************************************************
 * SDMMC devices,  include the module of SD,MMC,and sdio.noted by xbw at 2012-03-05
**************************************************************************************************/
#ifdef CONFIG_SDMMC_RK29
#include "board-rk3188-ds1006h-sdmmc-config.c"
#include "../plat-rk/rk-sdmmc-ops.c"
#include "../plat-rk/rk-sdmmc-wifi.c"
#endif //endif ---#ifdef CONFIG_SDMMC_RK29

#ifdef CONFIG_SDMMC0_RK29
static int rk29_sdmmc0_cfg_gpio(void)
{
#ifdef CONFIG_SDMMC_RK29_OLD
	iomux_set(MMC0_CMD);
	iomux_set(MMC0_CLKOUT);
	iomux_set(MMC0_D0);
	iomux_set(MMC0_D1);
	iomux_set(MMC0_D2);
	iomux_set(MMC0_D3);

	iomux_set_gpio_mode(iomux_mode_to_gpio(MMC0_DETN));

	gpio_request(RK30_PIN3_PA7, "sdmmc-power");
	gpio_direction_output(RK30_PIN3_PA7, GPIO_LOW);

#else
	rk29_sdmmc_set_iomux(0, 0xFFFF);

    #if defined(CONFIG_SDMMC0_RK29_SDCARD_DET_FROM_GPIO)
        #if SDMMC_USE_NEW_IOMUX_API
        iomux_set_gpio_mode(iomux_gpio_to_mode(RK29SDK_SD_CARD_DETECT_N));
        #else
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO);
        #endif
    #else
        #if SDMMC_USE_NEW_IOMUX_API       
        iomux_set(MMC0_DETN);
        #else
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FMUX);
        #endif
    #endif	

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	gpio_request(SDMMC0_WRITE_PROTECT_PIN, "sdmmc-wp");
	gpio_direction_input(SDMMC0_WRITE_PROTECT_PIN);
#endif

#endif

	return 0;
}

#define CONFIG_SDMMC0_USE_DMA
struct rk29_sdmmc_platform_data default_sdmmc0_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36),
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc0_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif

	.dma_name = "sd_mmc",
#ifdef CONFIG_SDMMC0_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC) && defined(CONFIG_USE_SDMMC0_FOR_WIFI_DEVELOP_BOARD)
    .status = rk29sdk_wifi_mmc0_status,
    .register_status_notify = rk29sdk_wifi_mmc0_status_register,
#endif

#if defined(RK29SDK_SD_CARD_PWR_EN) || (INVALID_GPIO != RK29SDK_SD_CARD_PWR_EN)
    .power_en = RK29SDK_SD_CARD_PWR_EN,
    .power_en_level = RK29SDK_SD_CARD_PWR_EN_LEVEL,
#else
    .power_en = INVALID_GPIO,
    .power_en_level = GPIO_LOW,
#endif    
	.enable_sd_wakeup = 0,

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	.write_prt = SDMMC0_WRITE_PROTECT_PIN,
	.write_prt_enalbe_level = SDMMC0_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    .det_pin_info = {    
    #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N, //INVALID_GPIO,
        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
    #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
    #endif    
    }, 

};
#endif // CONFIG_SDMMC0_RK29

#ifdef CONFIG_SDMMC1_RK29
#define CONFIG_SDMMC1_USE_DMA
static int rk29_sdmmc1_cfg_gpio(void)
{
#if defined(CONFIG_SDMMC_RK29_OLD)
	iomux_set(MMC1_CMD);
	iomux_set(MMC1_CLKOUT);
	iomux_set(MMC1_D0);
	iomux_set(MMC1_D1);
	iomux_set(MMC1_D2);
	iomux_set(MMC1_D3);
#else

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	gpio_request(SDMMC1_WRITE_PROTECT_PIN, "sdio-wp");
	gpio_direction_input(SDMMC1_WRITE_PROTECT_PIN);
#endif

#endif

	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc1_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34),

#if !defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
		      MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#else
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#endif

	.io_init = rk29_sdmmc1_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif

	.dma_name = "sdio",
#ifdef CONFIG_SDMMC1_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_CONTROL_FUNC) || defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
    .status = rk29sdk_wifi_status,
    .register_status_notify = rk29sdk_wifi_status_register,
#endif

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	.write_prt = SDMMC1_WRITE_PROTECT_PIN,
	    .write_prt_enalbe_level = SDMMC1_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    #if defined(CONFIG_RK29_SDIO_IRQ_FROM_GPIO)
        .sdio_INT_gpio = RK29SDK_WIFI_SDIO_CARD_INT,
    #endif

    .det_pin_info = {    
#if defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
     #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N,
     #else
         .io             = INVALID_GPIO,
     #endif   

        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
 #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
#endif
    },
   
	.enable_sd_wakeup = 0,
};
#endif //endif--#ifdef CONFIG_SDMMC1_RK29

/**************************************************************************************************
 * the end of setting for SDMMC devices
**************************************************************************************************/

#ifdef CONFIG_BATTERY_RK30_ADC_FAC
static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
        .dc_det_pin      = RK30_PIN0_PB2,
        .batt_low_pin    = INVALID_GPIO, 
        .charge_set_pin  = INVALID_GPIO,
        .charge_ok_pin   = RK30_PIN0_PA6,
	 .usb_det_pin = INVALID_GPIO,
        .dc_det_level    = GPIO_LOW,
        .charge_ok_level = GPIO_HIGH,

	.reference_voltage = 1800, // the rK2928 is 3300;RK3066 and rk29 are 2500;rk3066B is 1800;
       .pull_up_res = 200,     //divider resistance ,  pull-up resistor
       .pull_down_res = 120, //divider resistance , pull-down resistor

	.is_reboot_charging = 1,
        .save_capacity   = 1 ,
        .low_voltage_protection = 3600,    
};

static struct platform_device rk30_device_adc_battery = {
        .name   = "rk30-battery",
        .id     = -1,
        .dev = {
                .platform_data = &rk30_adc_battery_platdata,
        },
};
#endif
/*$_rbox_$_modify_$_huangzhibao_begin$_20120508_$*/
#ifdef CONFIG_RK_REMOTECTL

void rk30_remotectl_iomux(void)
{
	;
}

struct RKxx_remotectl_platform_data rk30_remotectl_pdata = {
    .gpio	=   RK30_PIN0_PB2, 
    .wakeup	= 1,
    .rep    = 0,
    .set_iomux = rk30_remotectl_iomux,    
};

static struct platform_device rk30_device_remotectl = {
	.name		= "rkxx-remotectl",
	.id		= -1,
	.dev		= {
		.platform_data	= &rk30_remotectl_pdata,
	},
};
#endif
/*$_rbox_$_modify_$_huangzhibao_end$_20120508_$*/
#ifdef CONFIG_RK30_PWM_REGULATOR
static int pwm_voltage_map[] = {
	 800000,  825000,  850000,  875000,  900000,  925000,
	 950000,  975000, 1000000, 1025000, 1050000, 1075000, 
	1100000, 1125000, 1150000, 1175000, 1200000, 1225000, 
	1250000, 1275000, 1300000, 1325000, 1350000, 1375000
#ifdef OMEGAMOON_CHANGED
    // Omegamoon >> Set max voltage from 1375000 to 1425000
    , 1400000, 1425000 
#endif

};

static struct regulator_consumer_supply pwm_dcdc1_consumers[] = {
	{
		.supply = "vdd_cpu",
	}
};

struct regulator_init_data pwm_regulator_init_dcdc[1] =
{
	{
		.constraints = {
			.name = "PWM_DCDC1",
			.min_uV = 600000,
			.max_uV = 1800000,	//0.6-1.8V
			.apply_uV = true,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(pwm_dcdc1_consumers),
		.consumer_supplies = pwm_dcdc1_consumers,
	},
};

static struct pwm_platform_data pwm_regulator_info[1] = {
	{
		.pwm_id = 1,
		.pwm_gpio = RK30_PIN3_PD4,
		.pwm_iomux_pwm = PWM1,
		.pwm_iomux_gpio = GPIO3_D4,
		.pwm_voltage = 1100000,
		.suspend_voltage = 1000000,
		.min_uV = 800000,
#ifdef OMEGAMOON_CHANGED
		// Omegamoon >> Set max voltage from 1375000 to 1425000
		.max_uV	= 1425000,
#else
		.max_uV = 1375000,
#endif
		.coefficient = 575,	//57.5%
		.pwm_voltage_map = pwm_voltage_map,
		.init_data	= &pwm_regulator_init_dcdc[0],
	},
};

struct platform_device pwm_regulator_device[1] = {
	{
		.name = "pwm-voltage-regulator",
		.id = 0,
		.dev		= {
			.platform_data = &pwm_regulator_info[0],
		}
	},
};
#endif

#ifdef CONFIG_RFKILL_RK
// bluetooth rfkill device, its driver in net/rfkill/rfkill-rk.c
static struct rfkill_rk_platform_data rfkill_rk_platdata = {
    .type               = RFKILL_TYPE_BLUETOOTH,

    .poweron_gpio       = { // BT_REG_ON
#ifdef CONFIG_RFKILL_RK_POWERON_ENABLE	
	  // Omegamoon: Rockchip default is INVALID_GPIO
      .io             = CONFIG_RFKILL_RK_POWERON_GPIO,
#else
      .io             = INVALID_GPIO,
#endif	  
      .enable         = GPIO_HIGH,
	  .iomux		  = {
	      .name	 = "bt_poweron",
#ifdef CONFIG_RFKILL_RK_POWERON_ENABLE
	      .fgpio = CONFIG_RFKILL_RK_POWERON_IOMUX,
#endif
      },
    },

    .reset_gpio         = { // BT_RST
#ifdef CONFIG_RFKILL_RK_RESET_ENABLE	
	  // Omegamoon: Rockchip default is RK30_PIN3_PD1
      .io             = CONFIG_RFKILL_RK_RESET_GPIO,
#else
      .io             = INVALID_GPIO,
#endif
      .enable         = GPIO_LOW,
      .iomux		= {
	      .name	 = "bt_reset",
#ifdef CONFIG_RFKILL_RK_RESET_ENABLE	
          .fgpio = CONFIG_RFKILL_RK_RESET_IOMUX,
#endif
      },
    }, 

    .wake_gpio          = { // BT_WAKE, use to control bt's sleep and wakeup
#ifdef CONFIG_RFKILL_RK_WAKE_ENABLE	
		// Omegamoon: Rockchip default is RK30_PIN3_PC6
        .io           = CONFIG_RFKILL_RK_WAKE_GPIO,
#else
      .io             = INVALID_GPIO,
#endif
        .enable       = GPIO_HIGH,
        .iomux		= {
            .name	= "bt_wake",
#ifdef CONFIG_RFKILL_RK_WAKE_ENABLE	
            .fgpio	= CONFIG_RFKILL_RK_WAKE_IOMUX,
#endif
        },
    },

    .wake_host_irq      = { // BT_HOST_WAKE, for bt wakeup host when it is in deep sleep
        .gpio           = {
#ifdef CONFIG_RFKILL_RK_WAKEHOST_ENABLE	
			// Omegamoon: Rockchip default is RK30_PIN3_PC7
            .io         = CONFIG_RFKILL_RK_WAKEHOST_GPIO,
#else
            .io         = INVALID_GPIO,
#endif
            .enable     = GPIO_LOW,      // set GPIO_LOW for falling, set 0 for rising
            .iomux      = {
                .name   = NULL,
#ifdef CONFIG_RFKILL_RK_WAKEHOST_ENABLE	
                .fgpio	= CONFIG_RFKILL_RK_WAKEHOST_IOMUX,
#endif
            },
        },
    },

    .rts_gpio           = { // UART_RTS, enable or disable BT's data coming
#ifdef CONFIG_RFKILL_RK_RTS_ENABLE
		// Omegamoon: Rockchip default is RK30_PIN1_PA3
        .io             = CONFIG_RFKILL_RK_RTS_GPIO,
#else
		.io             = INVALID_GPIO,
#endif
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = "bt_rts",
#ifdef CONFIG_RFKILL_RK_RTS_ENABLE
			// Omegamoon: Rockchip default is GPIO1_A3
            .fgpio      = CONFIG_RFKILL_RK_RTS_IOMUX,
#endif
            .fmux       = UART0_RTSN,
        },
    },
};

static struct platform_device device_rfkill_rk = {
    .name   = "rfkill_rk",
    .id     = -1,
    .dev    = {
        .platform_data = &rfkill_rk_platdata,
    },
};
#endif

#if defined(CONFIG_GPS_RK)
int rk_gps_io_init(void)
{
	printk("%s \n", __FUNCTION__);
	
	rk30_mux_api_set(GPIO1B5_UART3RTSN_NAME, GPIO1B_GPIO1B5);//VCC_EN
	gpio_request(RK30_PIN1_PB5, NULL);
	gpio_direction_output(RK30_PIN1_PB5, GPIO_LOW);

	rk30_mux_api_set(GPIO1B4_UART3CTSN_GPSRFCLK_NAME, GPIO1B_GPSRFCLK);//GPS_CLK
	rk30_mux_api_set(GPIO1B2_UART3SIN_GPSMAG_NAME, GPIO1B_GPSMAG);//GPS_MAG
	rk30_mux_api_set(GPIO1B3_UART3SOUT_GPSSIG_NAME, GPIO1B_GPSSIG);//GPS_SIGN

	rk30_mux_api_set(GPIO1A6_UART1CTSN_SPI0CLK_NAME, GPIO1A_GPIO1A6);//SPI_CLK
	gpio_request(RK30_PIN1_PA6, NULL);
	gpio_direction_output(RK30_PIN1_PA6, GPIO_LOW);

	rk30_mux_api_set(GPIO1A5_UART1SOUT_SPI0TXD_NAME, GPIO1A_GPIO1A5);//SPI_MOSI
	gpio_request(RK30_PIN1_PA5, NULL);
	gpio_direction_output(RK30_PIN1_PA5, GPIO_LOW);	

	rk30_mux_api_set(GPIO1A7_UART1RTSN_SPI0CSN0_NAME, GPIO1A_GPIO1A7);//SPI_CS
	gpio_request(RK30_PIN1_PA7, NULL);
	gpio_direction_output(RK30_PIN1_PA7, GPIO_LOW);		
	return 0;
}
int rk_gps_power_up(void)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}

int rk_gps_power_down(void)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}

int rk_gps_reset_set(int level)
{
	return 0;
}
int rk_enable_hclk_gps(void)
{
	printk("%s \n", __FUNCTION__);
	clk_enable(clk_get(NULL, "hclk_gps"));
	return 0;
}
int rk_disable_hclk_gps(void)
{
	printk("%s \n", __FUNCTION__);
	clk_disable(clk_get(NULL, "hclk_gps"));
	return 0;
}
struct rk_gps_data rk_gps_info = {
	.io_init = rk_gps_io_init,
	.power_up = rk_gps_power_up,
	.power_down = rk_gps_power_down,
	.reset = rk_gps_reset_set,
	.enable_hclk_gps = rk_enable_hclk_gps,
	.disable_hclk_gps = rk_disable_hclk_gps,
	.GpsSign = RK30_PIN1_PB3,
	.GpsMag = RK30_PIN1_PB2,        //GPIO index
	.GpsClk = RK30_PIN1_PB4,        //GPIO index
	.GpsVCCEn = RK30_PIN1_PB5,     //GPIO index
	.GpsSpi_CSO = RK30_PIN1_PA4,    //GPIO index
	.GpsSpiClk = RK30_PIN1_PA5,     //GPIO index
	.GpsSpiMOSI = RK30_PIN1_PA7,	  //GPIO index
	.GpsIrq = IRQ_GPS,
	.GpsSpiEn = 0,
	.GpsAdcCh = 2,
	.u32GpsPhyAddr = RK30_GPS_PHYS,
	.u32GpsPhySize = RK30_GPS_SIZE,
};

struct platform_device rk_device_gps = {
	.name = "gps_hv5820b",
	.id = -1,
	.dev		= {
	.platform_data = &rk_gps_info,
		}
	};
#endif

#if defined(CONFIG_MT5931_MT6622)
static struct mt6622_platform_data mt6622_platdata = {
		    .power_gpio         = { // BT_REG_ON
		    	.io             = RK30_PIN3_PD5, // set io to INVALID_GPIO for disable it
			    .enable         = GPIO_HIGH,
			    .iomux          = {
				    .name       = NULL,
				},
		    },

		    .reset_gpio         = { // BT_RST
		        .io             = RK30_PIN0_PD7,
		        .enable         = GPIO_HIGH,
		        .iomux          = {
		            .name       = NULL,
		        },
		    },

		    .irq_gpio           = {
			    .io             = RK30_PIN3_PD2,
			    .enable         = GPIO_HIGH,
			    .iomux          = {
				    .name       = NULL,
				},
		    }
};

static struct platform_device device_mt6622 = {
		    .name   = "mt6622",
			.id     = -1,
			.dev    = {
			       .platform_data = &mt6622_platdata,
			},
};	
#endif

/*$_rbox_$_modify_$_zhengyang_begin$_20130407_$*/
/*
 * Codec for the ASoC Rockchip HDMI machine driver
 */
#ifdef CONFIG_SND_SOC_RK_HDMI_CODEC
static struct platform_device rockchip_hdmi_codec = {
	.name	= "rockchip-hdmi-codec",
	.id	= -1,
};
#endif
/*
 * Device for the ASoC Rockchip HDMI machine driver
 */
#ifdef CONFIG_SND_RK_SOC_HDMI
static struct platform_device rockchip_hdmi_audio = {
	.name	= "rockchip-hdmi-audio",
	.id	= -1,
};
#endif
/*$_rbox_$_modify_$_zhengyang_end$_20130407_$*/
//leolas RDA5876 BT chip support (QX1 and old MK802IV)
#if defined CONFIG_TCC_BT_DEV
static struct tcc_bt_platform_data tcc_bt_platdata = {

    .power_gpio   = { // ldoon
//SAW QX1 setting thanks to Leolas
#ifdef CONFIG_IMITO_QX1
        .io             = RK30_PIN3_PD1,//leolas:for QX1
#else
        .io             = RK30_PIN3_PC7,//leolas:not absolutely sure but I think this value must work for old MK802IV
#endif
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = NULL,
            },
        },

    .wake_host_gpio  = { // BT_HOST_WAKE, for bt wakeup host when it is in deep sleep
#ifdef CONFIG_IMITO_QX1
        .io             = RK30_PIN3_PC6, //leolas:for QX1
#else
        .io             = RK30_PIN3_PD0, //leolas:not absolutely sure but I think this value must work for old MK802IV
#endif
        .enable     = IRQF_TRIGGER_RISING,// set IRQF_TRIGGER_FALLING for falling, set IRQF_TRIGGER_RISING for rising
        .iomux      = {
            .name       = NULL,
        },
    },
};

static struct platform_device device_tcc_bt = {
    .name   = "tcc_bt_dev",
    .id     = -1,
    .dev    = {
        .platform_data = &tcc_bt_platdata,
        },
};
#endif
//leolas RDA5876 BT chip support end
static struct platform_device *devices[] __initdata = {

#ifdef CONFIG_ION
	&device_ion,
#endif
#ifdef CONFIG_WIFI_CONTROL_FUNC
	&rk29sdk_wifi_device,
#endif

#if defined(CONFIG_MT6620)
	    &mt3326_device_gps,
#endif

#ifdef CONFIG_BATTERY_RK30_ADC_FAC
 	&rk30_device_adc_battery,
#endif
#ifdef CONFIG_RFKILL_RK
	&device_rfkill_rk,
#endif
#ifdef CONFIG_GPS_RK
	&rk_device_gps,
#endif
#ifdef CONFIG_MT5931_MT6622
	&device_mt6622,
#endif
/*$_rbox_$_modify_$_huangzhibao_begin$_20120508_$*/
#ifdef CONFIG_RK_REMOTECTL	
    &rk30_device_remotectl,
#endif
/*$_rbox_$_modify_$_huangzhibao_end$_20120508_$*/
#if defined(CONFIG_MT6229)
	&rk29_device_mt6229,
#endif
/*$_rbox_$_modify_$_zhengyang_begin$_20130407_$*/
#ifdef CONFIG_SND_SOC_RK_HDMI_CODEC
	&rockchip_hdmi_codec,
#endif
#ifdef CONFIG_SND_RK_SOC_HDMI
	&rockchip_hdmi_audio,
#endif
/*$_rbox_$_modify_$_zhengyang_end$_20130407_$*/

//leolas RDA5876 BT chip support
#ifdef CONFIG_TCC_BT_DEV
        &device_tcc_bt,
#endif //leolas end RDA5876 BT chip support
};


static int rk_platform_add_display_devices(void)
{
	struct platform_device *fb = NULL;  //fb
	struct platform_device *lcdc0 = NULL; //lcdc0
	struct platform_device *lcdc1 = NULL; //lcdc1
	struct platform_device *bl = NULL; //backlight
#ifdef CONFIG_FB_ROCKCHIP
	fb = &device_fb;
#endif

#if defined(CONFIG_LCDC0_RK3188)
	lcdc0 = &device_lcdc0,
#endif

#if defined(CONFIG_LCDC1_RK3188)
	lcdc1 = &device_lcdc1,
#endif

#ifdef CONFIG_BACKLIGHT_RK29_BL
	bl = &rk29_device_backlight,
#endif
	__rk_platform_add_display_devices(fb,lcdc0,lcdc1,bl);

	return 0;
	
}
//$_rbox_$_modify_$ zhengyang modified for box
static struct rkdisplay_platform_data hdmi_data = {
	.property 		= DISPLAY_MAIN,
#if defined(CONFIG_RK_LCDC0_AS_PRIMARY)
	.video_source 	= DISPLAY_SOURCE_LCDC0,
#else
	.video_source 	= DISPLAY_SOURCE_LCDC1,
#endif
	.io_pwr_pin 	= INVALID_GPIO,
	.io_reset_pin 	= RK30_PIN3_PB2,
};

#if defined(CONFIG_RK1000_TVOUT) || defined(CONFIG_MFD_RK1000)
static struct rkdisplay_platform_data tv_data = {
	.property 		= DISPLAY_AUX,
	.video_source 	= DISPLAY_SOURCE_LCDC0,
	.io_pwr_pin 	= INVALID_GPIO,
	// Omegamoon: Rockchip default is RK30_PIN3_PD4
	.io_reset_pin 	= CONFIG_RK1000_RESET_GPIO, 
	.io_switch_pin	= INVALID_GPIO,
};
#endif
//$_rbox_$_modify_$ zhengyang modified end

// i2c
#ifdef CONFIG_I2C0_RK30
static struct i2c_board_info __initdata i2c0_info[] = {
#if defined (CONFIG_GS_MMA8452)
	{
		.type	        = "gs_mma8452",
		.addr	        = 0x1d,
		.flags	        = 0,
		.irq	        = MMA8452_INT_PIN,
		.platform_data = &mma8452_info,
	},
#endif
#if defined (CONFIG_LS_US5151)
        {    
                .type           = "us5151",
                .addr           = 0x10,
                .flags          = 0, 
        },   
#endif

#if defined (CONFIG_GS_LIS3DH)
	{
		.type	        = "gs_lis3dh",
		.addr	        = 0x19,   //0x19(SA0-->VCC), 0x18(SA0-->GND)
		.flags	        = 0,
		.irq	        = LIS3DH_INT_PIN,
		.platform_data = &lis3dh_info,
	},
#endif
#if defined (CONFIG_SENSORS_AK8963)
        {
                .type             = "akm8963",
                .addr           = 0x0d,
                .flags           = I2C_CLIENT_WAKE,
               .irq             = RK30_PIN3_PD7,
              .platform_data = &akm_platform_data_8963,
        },
#endif
#if defined (CONFIG_LS_PHOTORESISTOR)
	{
		.type           = "ls_photoresistor",
		.addr           = 0x5e,            
		.flags          = 0,
		.irq            = INVALID_GPIO,	
		.platform_data = &light_photoresistor_info,
	},
#endif

#if defined (CONFIG_COMPASS_AK8975)
	{
		.type          = "ak8975",
		.addr          = 0x0d,
		.flags         = 0,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8975_info,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8975_info,
	},
#endif
#if defined (CONFIG_GYRO_L3G4200D)
	{
		.type          = "l3g4200d_gryo",
		.addr          = 0x69,
		.flags         = 0,
		.irq           = L3G4200D_INT_PIN,
		.platform_data = &l3g4200d_info,
	},
#endif
//$_rbox_$_modify_$ zhengyang modified for box
//#if defined (CONFIG_SND_SOC_RK1000)
//	{
//		.type          = "rk1000_i2c_codec",
//		.addr          = 0x60,
//		.flags         = 0,
//	},
//	{
//		.type          = "rk1000_control",
//		.addr          = 0x40,
//		.flags         = 0,
//	},
//#endif
//$_rbox_$_modify_$ zhengyang modified end
#if defined (CONFIG_SND_SOC_RT5631)
        {
                .type                   = "rt5631",
                .addr                   = 0x1a,
                .flags                  = 0,
        },
#endif
};
#endif

int __sramdata g_pmic_type =  0;
#ifdef CONFIG_I2C1_RK30
#ifdef CONFIG_REGULATOR_ACT8846
#define PMU_POWER_SLEEP RK30_PIN0_PA1
#define PMU_VSEL RK30_PIN3_PD3
#define ACT8846_HOST_IRQ                RK30_PIN0_PB3

static struct pmu_info  act8846_dcdc_info[] = {
	{
		.name          = "act_dcdc1",   //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		.suspend_vol  =   1200000,
	},
	{
		.name          = "vdd_core",    //logic
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  1200000,
		#else
		.suspend_vol  =  900000,
		#endif
	},
	{
		.name          = "vdd_cpu",   //arm
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  1200000,
		#else
		.suspend_vol  =  900000,
		#endif
	},
	{
		.name          = "act_dcdc4",   //vccio 
//SAW special voltage for QX1, from Leolas
#ifdef CONFIG_IMITO_QX1
		.min_uv		= 3000000,
		.max_uv		= 3000000,
#else
		.min_uv         = 3300000,
		.max_uv         = 3300000,
#endif

		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  3000000,
		#else
		.suspend_vol  =  2800000,
		#endif
	},
	
};
static  struct pmu_info  act8846_ldo_info[] = {
	{
		.name          = "act_ldo1",   //vdd10
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "act_ldo2",    //vdd12
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "act_ldo3",   //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo4",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "act_ldo5",   //vcc_rmii
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "act_ldo6",   //vcc_jetta
//SAW volt set via kernel config, default 1800000 for all devices except QX1
#ifdef CONFIG_IMITO_QX1
		.min_uv		= 3300000,
		.max_uv		= 3300000,
#else
		.min_uv         = 1800000, 
		.max_uv         = 1800000, 
#endif
	},
	{
		.name          = "act_ldo7",   //vcc18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo8",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
 };

#include "../mach-rk30/board-pmu-act8846.c"
#endif

#ifdef CONFIG_MFD_WM831X_I2C
#define PMU_POWER_SLEEP 		RK30_PIN0_PA1 

static struct pmu_info  wm8326_dcdc_info[] = {
	{
		.name          = "vdd_core",   //logic
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  950000,
	},
	{
		.name          = "vdd_cpu",    //arm
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  950000,
	},
	{
		.name          = "dcdc3",   //ddr
		.min_uv          = 1150000,
		.max_uv         = 1150000,
		.suspend_vol  =  1150000,
	},
	#ifdef CONFIG_MACH_RK3066_SDK
	{
		.name          = "dcdc4",   //vcc_io
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3000000,
	},
	#else
	{
		.name          = "dcdc4",   //vcc_io
		.min_uv          = 3000000,
		.max_uv         = 3000000,
		.suspend_vol  =  2800000,
	},
	#endif
};

static struct pmu_info  wm8326_ldo_info[] = {
	{
		.name          = "ldo1",   //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
	{
		.name          = "ldo2",    //vccio_wl
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
	{
		.name          = "ldo3",   //
		.min_uv          = 1100000,
		.max_uv         = 1100000,
		.suspend_vol  =  1100000,
	},
	{
		.name          = "ldo4",   //vdd11
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  1000000,
	},
	{
		.name          = "ldo5",   //vcc25
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
	{
		.name          = "ldo6",   //vcc33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3300000,
	},
	{
		.name          = "ldo7",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
		.suspend_vol  =  2800000,
	},
	{
		.name          = "ldo8",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3300000,
	},
	{
		.name          = "ldo9",   //vcc_tp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3300000,
	},
	{
		.name          = "ldo10",   //flash_io
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
};

#include "../mach-rk30/board-pmu-wm8326.c"
#endif

#ifdef CONFIG_MFD_TPS65910
#ifdef CONFIG_ARCH_RK3188
#define TPS65910_HOST_IRQ        RK30_PIN0_PB3
#else
#define TPS65910_HOST_IRQ        RK30_PIN6_PA4
#endif

#define PMU_POWER_SLEEP RK30_PIN0_PA1

static struct pmu_info  tps65910_dcdc_info[] = {
	{
		.name          = "vdd_core",   //logic
		.min_uv          = 1100000,
		.max_uv         = 1100000,
	},
	{
		.name          = "vdd2",    //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "vio",   //vcc_io
		.min_uv          = 2500000,
		.max_uv         = 2500000,
	},
	
};
static  struct pmu_info  tps65910_ldo_info[] = {
	{
		.name          = "vpll",   //vdd10
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "vdig1",    //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "vdig2",   //vdd_jetta
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "vaux1",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "vaux2",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vaux33",   //vcc_tp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vmmc",   //vcc30
		.min_uv          = 3000000,
		.max_uv         = 3000000,
	},
	{
		.name          = "vdac",   //vcc18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
 };

#include "../mach-rk30/board-pmu-tps65910.c"
#endif

static struct i2c_board_info __initdata i2c1_info[] = {
#if defined (CONFIG_REGULATOR_ACT8846)
	{
		.type    		= "act8846",
		.addr           = 0x5a, 
		.flags			= 0,
		.irq            = ACT8846_HOST_IRQ,
		.platform_data=&act8846_data,
	},
#endif
#if defined (CONFIG_RTC_HYM8563)
	{
		.type                   = "rtc_hym8563",
		.addr           = 0x51,
		.flags                  = 0,
#ifdef CONFIG_IMITO_QX1
		.irq            = RK30_PIN1_PA4,//for imito QX1
#else
		.irq            = RK30_PIN0_PB5,//leolas:for the rest of devices
#endif
	},
#endif
#if defined (CONFIG_MFD_WM831X_I2C)
	{
		.type          = "wm8326",
		.addr          = 0x34,
		.flags         = 0,
		.irq           = RK30_PIN0_PB3,
		.platform_data = &wm831x_platdata,
	},
#endif
#if defined (CONFIG_MFD_TPS65910)
	{
        .type           = "tps65910",
        .addr           = TPS65910_I2C_ID0,
        .flags          = 0,
        .irq            = TPS65910_HOST_IRQ,
    	.platform_data = &tps65910_data,
	},
#endif
};
#endif

void __sramfunc board_pmu_suspend(void)
{      
        #if defined (CONFIG_REGULATOR_ACT8846)
       if(pmic_is_act8846())
       board_pmu_act8846_suspend(); 
       #endif
	#if defined (CONFIG_MFD_WM831X_I2C)
       if(pmic_is_wm8326())
       board_pmu_wm8326_suspend();
	#endif
	#if defined (CONFIG_MFD_TPS65910)
       if(pmic_is_tps65910())
       board_pmu_tps65910_suspend(); 
    #endif   
}

void __sramfunc board_pmu_resume(void)
{      
        #if defined (CONFIG_REGULATOR_ACT8846)
       if(pmic_is_act8846())
       board_pmu_act8846_resume(); 
       #endif
	#if defined (CONFIG_MFD_WM831X_I2C)
       if(pmic_is_wm8326())
       board_pmu_wm8326_resume();
	#endif
	#if defined (CONFIG_MFD_TPS65910)
       if(pmic_is_tps65910())
       board_pmu_tps65910_resume(); 
	#endif
}

 int __sramdata gpio3d6_iomux,gpio3d6_do,gpio3d6_dir,gpio3d6_en;

#define grf_readl(offset)	readl_relaxed(RK30_GRF_BASE + offset)
#define grf_writel(v, offset)	do { writel_relaxed(v, RK30_GRF_BASE + offset); dsb(); } while (0)
 
void __sramfunc rk30_pwm_logic_suspend_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR

//	int gpio0d7_iomux,gpio0d7_do,gpio0d7_dir,gpio0d7_en;
	sram_udelay(10000);
	gpio3d6_iomux = grf_readl(GRF_GPIO3D_IOMUX);
	gpio3d6_do = grf_readl(GRF_GPIO3H_DO);
	gpio3d6_dir = grf_readl(GRF_GPIO3H_DIR);
	gpio3d6_en = grf_readl(GRF_GPIO3H_EN);

	grf_writel((1<<28), GRF_GPIO3D_IOMUX);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_DIR);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_DO);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_EN);
#endif 
}
void __sramfunc rk30_pwm_logic_resume_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	grf_writel((1<<28)|gpio3d6_iomux, GRF_GPIO3D_IOMUX);
	grf_writel((1<<30)|gpio3d6_en, GRF_GPIO3H_EN);
	grf_writel((1<<30)|gpio3d6_dir, GRF_GPIO3H_DIR);
	grf_writel((1<<30)|gpio3d6_do, GRF_GPIO3H_DO);
	sram_udelay(10000);

#endif

}
extern void pwm_suspend_voltage(void);
extern void pwm_resume_voltage(void);
void  rk30_pwm_suspend_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_suspend_voltage();
#endif
}
void  rk30_pwm_resume_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_resume_voltage();
#endif
}


#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {
#if defined (CONFIG_CT36X_TS)
	{
		.type	       = CT36X_NAME,
		.addr          = 0x01,
		.flags         = 0,
		.platform_data = &ct36x_info,
	},
#endif
#if defined (CONFIG_LS_CM3217)
	{
		.type          = "lightsensor",
		.addr          = 0x10,
		.flags         = 0,
		.platform_data = &cm3217_info,
	},
#endif
//$_rbox_$_modify_$ zhengyang modified for box
#ifdef CONFIG_IT66121
{
		.type		= "it66121",
		.addr		= 0x4c,
		.flags		= 0,
		.irq		= RK30_PIN2_PD6,
		.platform_data 	= &hdmi_data,
},
#endif
//#if defined(CONFIG_HDMI_CAT66121)
//	{
//		.type		= "cat66121_hdmi",
//		.addr		= 0x4c,
//		.flags		= 0,
//		.irq		= RK30_PIN2_PD6,
//		.platform_data 	= &rk_hdmi_pdata,
//	},
//#endif
//$_rbox_$_modify_$ zhengyang modified end
};
#endif

#ifdef CONFIG_I2C3_RK30
static struct i2c_board_info __initdata i2c3_info[] = {
};
#endif

#ifdef CONFIG_I2C4_RK30
static struct i2c_board_info __initdata i2c4_info[] = {
#ifdef CONFIG_MFD_RK610
		{
			.type			= "rk610_ctl",
			.addr			= 0x40,
			.flags			= 0,
			.platform_data		= &rk610_ctl_pdata,
		},
#ifdef CONFIG_RK610_TVOUT
		{
			.type			= "rk610_tvout",
			.addr			= 0x42,
			.flags			= 0,
		},
#endif
#ifdef CONFIG_HDMI_RK610
		{
			.type			= "rk610_hdmi",
			.addr			= 0x46,
			.flags			= 0,
			.irq			= INVALID_GPIO,
		},
#endif
#ifdef CONFIG_SND_SOC_RK610
		{//RK610_CODEC addr  from 0x60 to 0x80 (0x60~0x80)
			.type			= "rk610_i2c_codec",
			.addr			= 0x60,
			.flags			= 0,
			.platform_data		= &rk610_codec_pdata,			
		},
#endif
#endif
//$_rbox_$_modify_$ zhengyang modified for box
#if defined (CONFIG_MFD_RK1000)
	{
		.type			= "rk1000_control",
		.addr			= 0x40,
		.flags			= 0,
		.platform_data = &tv_data,
	},
#ifdef CONFIG_RK1000_TVOUT
    {
		.type           = "rk1000_tvout",
		.addr           = 0x42,
		.flags          = 0,
		.platform_data = &tv_data,
    },
#endif
#ifdef CONFIG_SND_SOC_RK1000
    {
		.type           = "rk1000_i2c_codec",
		.addr           = 0x60,
		.flags          = 0,
    },
#endif
#endif
//$_rbox_$_modify_$ zhengyang modified end
};
#endif

#ifdef CONFIG_I2C_GPIO_RK30
#define I2C_SDA_PIN     INVALID_GPIO// RK30_PIN2_PD6   //set sda_pin here
#define I2C_SCL_PIN     INVALID_GPIO//RK30_PIN2_PD7   //set scl_pin here
static int rk30_i2c_io_init(void)
{
        //set iomux (gpio) here
        //rk30_mux_api_set(GPIO2D7_I2C1SCL_NAME, GPIO2D_GPIO2D7);
        //rk30_mux_api_set(GPIO2D6_I2C1SDA_NAME, GPIO2D_GPIO2D6);

        return 0;
}
struct i2c_gpio_platform_data default_i2c_gpio_data = {
       .sda_pin = I2C_SDA_PIN,
       .scl_pin = I2C_SCL_PIN,
       .udelay = 5, // clk = 500/udelay = 100Khz
       .timeout = 100,//msecs_to_jiffies(100),
       .bus_num    = 5,
       .io_init = rk30_i2c_io_init,
};
static struct i2c_board_info __initdata i2c_gpio_info[] = {
};
#endif

static void __init rk30_i2c_register_board_info(void)
{
#ifdef CONFIG_I2C0_RK30
	i2c_register_board_info(0, i2c0_info, ARRAY_SIZE(i2c0_info));
#endif
#ifdef CONFIG_I2C1_RK30
	i2c_register_board_info(1, i2c1_info, ARRAY_SIZE(i2c1_info));
#endif
#ifdef CONFIG_I2C2_RK30
	i2c_register_board_info(2, i2c2_info, ARRAY_SIZE(i2c2_info));
#endif
#ifdef CONFIG_I2C3_RK30
	i2c_register_board_info(3, i2c3_info, ARRAY_SIZE(i2c3_info));
#endif
#ifdef CONFIG_I2C4_RK30
	i2c_register_board_info(4, i2c4_info, ARRAY_SIZE(i2c4_info));
#endif
#ifdef CONFIG_I2C_GPIO_RK30
	i2c_register_board_info(5, i2c_gpio_info, ARRAY_SIZE(i2c_gpio_info));
#endif
}
//end of i2c

#define POWER_ON_PIN RK30_PIN0_PA0   //power_hold
static void rk30_pm_power_off(void)
{
	printk(KERN_ERR "rk30_pm_power_off start...\n");
#if defined(CONFIG_MFD_WM831X)
	wm831x_set_bits(Wm831x,WM831X_GPIO_LEVEL,0x0001,0x0000);  //set sys_pwr 0
	wm831x_device_shutdown(Wm831x);//wm8326 shutdown
#endif
#if defined(CONFIG_REGULATOR_ACT8846)
       if (pmic_is_act8846()) {
               printk("enter dcdet===========\n");
               if(gpio_get_value (RK30_PIN0_PB2) == GPIO_LOW)
               {
                       printk("enter restart===========\n");
                       arm_pm_restart(0, NULL);
               }
		/** code here may cause tablet cannot boot when shutdown without charger pluged in
		  * and then plug in charger. -- Cody Xie
               else
		{
			act8846_device_shutdown();
		}
		  */
       }
#endif
	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	while (1);
}

static void __init machine_rk30_board_init(void)
{
	avs_init();
	gpio_request(POWER_ON_PIN, "poweronpin");
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
#ifdef CONFIG_POWERON_LED_ENABLE
	int pwm_gpio;
	pwm_gpio = iomux_mode_to_gpio(PWM0);
	if (gpio_request(pwm_gpio, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return -1;
	}
	gpio_direction_output(pwm_gpio, GPIO_HIGH);
#endif
	pm_power_off = rk30_pm_power_off;
	
        gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);


	rk30_i2c_register_board_info();
	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	rk_platform_add_display_devices();
	board_usb_detect_init(RK30_PIN0_PA7);

#ifdef CONFIG_WIFI_CONTROL_FUNC
	rk29sdk_wifi_bt_gpio_control_init();
#endif

#if defined(CONFIG_MT6620)
	    clk_set_rate(clk_get_sys("rk_serial.1", "uart"), 48*1000000);
#endif

#if defined(CONFIG_MT5931_MT6622)
		clk_set_rate(clk_get_sys("rk_serial.0", "uart"), 24*1000000);
#endif		
}

static void __init rk30_reserve(void)
{
#ifdef CONFIG_ION
	rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ION_RESERVE_SIZE);
#endif

#ifdef CONFIG_FB_ROCKCHIP
	resource_fb[0].start = board_mem_reserve_add("fb0 buf", get_fb_size());
	resource_fb[0].end = resource_fb[0].start + get_fb_size()- 1;
#if 0
	resource_fb[1].start = board_mem_reserve_add("ipp buf", RK30_FB0_MEM_SIZE);
	resource_fb[1].end = resource_fb[1].start + RK30_FB0_MEM_SIZE - 1;
#endif

#if defined(CONFIG_FB_ROTATE) || !defined(CONFIG_THREE_FB_BUFFER)
	resource_fb[2].start = board_mem_reserve_add("fb2 buf",get_fb_size());
	resource_fb[2].end = resource_fb[2].start + get_fb_size() - 1;
#endif
#endif


#ifdef CONFIG_VIDEO_RK29
	rk30_camera_request_reserve_mem();
#endif
	
#ifdef CONFIG_GPS_RK
	//it must be more than 8MB
	rk_gps_info.u32MemoryPhyAddr = board_mem_reserve_add("gps", SZ_8M);
#endif
	board_mem_reserved();
}

/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs 
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 * @logic_volt	: logic voltage arm requests depend on frequency
 * comments	: min arm/logic voltage
 */
static struct cpufreq_frequency_table dvfs_arm_table[] = {
#ifdef CONFIG_RK_CPU_312
        {.frequency = 312 * 1000,       .index = CONFIG_RK_CPU_312_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_504
        {.frequency = 504 * 1000,       .index = CONFIG_RK_CPU_504_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_816
        {.frequency = 816 * 1000,       .index = CONFIG_RK_CPU_816_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1008
        {.frequency = 1008 * 1000,      .index = CONFIG_RK_CPU_1008_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1200
        {.frequency = 1200 * 1000,      .index = CONFIG_RK_CPU_1200_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1416
        {.frequency = 1416 * 1000,      .index = CONFIG_RK_CPU_1416_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1608
        {.frequency = 1608 * 1000,      .index = CONFIG_RK_CPU_1608_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1704
        {.frequency = 1704 * 1000,      .index = CONFIG_RK_CPU_1704_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1800
        {.frequency = 1800 * 1000,      .index = CONFIG_RK_CPU_1800_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1896
        {.frequency = 1896 * 1000,      .index = CONFIG_RK_CPU_1896_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1920
        {.frequency = 1920 * 1000,      .index = CONFIG_RK_CPU_1920_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_2016
        {.frequency = 2016 * 1000,      .index = CONFIG_RK_CPU_2016_VOLT * 1000},
#endif
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_gpu_table[] = {
#ifdef CONFIG_RK_GPU_133
	   {.frequency = 133 * 1000,       .index = CONFIG_RK_GPU_133_VOLT * 1000},
#endif
#ifdef CONFIG_RK_GPU_200
       {.frequency = 200 * 1000,       .index = CONFIG_RK_GPU_200_VOLT * 1000},  
#endif
#ifdef CONFIG_RK_GPU_266
       {.frequency = 266 * 1000,       .index = CONFIG_RK_GPU_266_VOLT * 1000},  
#endif
#ifdef CONFIG_RK_GPU_300
       {.frequency = 300 * 1000,       .index = CONFIG_RK_GPU_300_VOLT * 1000},  
#endif
#ifdef CONFIG_RK_GPU_400
       {.frequency = 400 * 1000,       .index = CONFIG_RK_GPU_400_VOLT * 1000},
#endif
#ifdef CONFIG_RK_GPU_600
       {.frequency = 600 * 1000,       .index = CONFIG_RK_GPU_600_VOLT * 1000},
#endif
#ifdef CONFIG_RK_GPU_798
       {.frequency = 798 * 1000,       .index = CONFIG_RK_GPU_798_VOLT * 1000},
#endif
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
	{.frequency = 400 * 1000 + DDR_FREQ_IDLE,       .index = 1000 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_SUSPEND,    .index = 1000 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_VIDEO,      .index = 1000 * 1000},
#ifdef CONFIG_RK_DDR_300
	{.frequency = 300 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_300_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_360
	{.frequency = 360 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_360_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_400
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_400_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_500
	{.frequency = 500 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_500_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_536
	{.frequency = 536 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_536_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_600
	{.frequency = 600 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_600_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_640
	{.frequency = 640 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_640_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_672
	{.frequency = 672 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_672_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_700
	{.frequency = 700 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_700_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_720
	{.frequency = 720 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_720_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_768
	{.frequency = 768 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_768_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_800
	{.frequency = 800 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_800_VOLT * 1000},
#endif
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table_t[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 950 * 1000},
	{.frequency = 460 * 1000 + DDR_FREQ_NORMAL,     .index = 1150 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

//#define DVFS_CPU_TABLE_SIZE	(ARRAY_SIZE(dvfs_cpu_logic_table))
//static struct cpufreq_frequency_table cpu_dvfs_table[DVFS_CPU_TABLE_SIZE];
//static struct cpufreq_frequency_table dep_cpu2core_table[DVFS_CPU_TABLE_SIZE];
int get_max_freq(struct cpufreq_frequency_table *table)
{
	int i,temp=0;
	
	for(i=0;table[i].frequency!= CPUFREQ_TABLE_END;i++)
	{
		if(temp<table[i].frequency)
			temp=table[i].frequency;
	}	
	printk("get_max_freq=%d\n",temp);
	return temp;
}

void __init board_clock_init(void)
{
	u32 flags=RK30_CLOCKS_DEFAULT_FLAGS;
#if !defined(CONFIG_ARCH_RK3188)
	if(get_max_freq(dvfs_gpu_table)<=(400*1000))
	{	
		flags=RK30_CLOCKS_DEFAULT_FLAGS|CLK_GPU_GPLL;
	}
	else
		flags=RK30_CLOCKS_DEFAULT_FLAGS|CLK_GPU_CPLL;
#endif	
	rk30_clock_data_init(periph_pll_default, codec_pll_default, flags);
	//dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);	
	dvfs_set_freq_volt_table(clk_get(NULL, "cpu"), dvfs_arm_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);
#if defined(CONFIG_ARCH_RK3188)
	if (rk_pll_flag() == 0)
		dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
	else
		dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table_t);
#else
	dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
#endif
}

MACHINE_START(RK30, "RK30board")
	.boot_params	= PLAT_PHYS_OFFSET + 0x800,
	.fixup		= rk30_fixup,
	.reserve	= &rk30_reserve,
	.map_io		= rk30_map_io,
	.init_irq	= rk30_init_irq,
	.timer		= &rk30_timer,
	.init_machine	= machine_rk30_board_init,
MACHINE_END
