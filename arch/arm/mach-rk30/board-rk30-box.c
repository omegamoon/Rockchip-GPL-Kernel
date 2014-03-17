/* based on arch/arm/mach-rk30/board-rk30-sdk.c
 *
 * Copyright (C) 2013 Omegamoon
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

#if defined(CONFIG_RK_HDMI)
	#include "../../../drivers/video/rockchip/hdmi/rk_hdmi.h"
#endif

#if defined(CONFIG_SPIM_RK29)
#include "../../../drivers/spi/rk29_spim.h"
#endif
#if defined(CONFIG_MT6229)
#include <linux/mt6229.h>
#endif
#if defined(CONFIG_GPS_RK)
#include "../../../drivers/misc/gps/rk_gps/rk_gps.h"
#endif

#ifdef CONFIG_RK_REMOTECTL
#include <mach/remotectl.h>
#endif
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
		.desc	= "vol-",
		.code	= KEY_VOLUMEDOWN,
		.gpio	= RK30_PIN4_PC5,
		.active_low = PRESS_LEV_LOW,
	},
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
#define PWM_ID            0
#define PWM_MUX_NAME      GPIO0A3_PWM0_NAME
#define PWM_MUX_MODE      GPIO0A_PWM0
#define PWM_MUX_MODE_GPIO GPIO0A_GPIO0A3
#define PWM_GPIO 	  RK30_PIN0_PA3
#define PWM_EFFECT_VALUE  1

#define LCD_DISP_ON_PIN

#ifdef  LCD_DISP_ON_PIN
//#define BL_EN_MUX_NAME    GPIOF34_UART3_SEL_NAME
//#define BL_EN_MUX_MODE    IOMUXB_GPIO1_B34

#define BL_EN_PIN         RK30_PIN6_PB3
#define BL_EN_VALUE       GPIO_HIGH
#endif
static int rk29_backlight_io_init(void)
{
	int ret = 0;
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE);
#ifdef  LCD_DISP_ON_PIN
	// rk30_mux_api_set(BL_EN_MUX_NAME, BL_EN_MUX_MODE);

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
	int ret = 0;
#ifdef  LCD_DISP_ON_PIN
	gpio_free(BL_EN_PIN);
#endif
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	return ret;
}

static int rk29_backlight_pwm_suspend(void)
{
	int ret = 0;
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	if (gpio_request(PWM_GPIO, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return -1;
	}
	gpio_direction_output(PWM_GPIO, GPIO_LOW);
#ifdef  LCD_DISP_ON_PIN
	gpio_direction_output(BL_EN_PIN, 0);
	gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
#endif
	return ret;
}

static int rk29_backlight_pwm_resume(void)
{
	gpio_free(PWM_GPIO);
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE);
#ifdef  LCD_DISP_ON_PIN
	msleep(30);
	gpio_direction_output(BL_EN_PIN, 1);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif
	return 0;
}

static struct rk29_bl_info rk29_bl_info = {
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
#define MMA8452_INT_PIN   RK30_PIN4_PC0

static int mma8452_init_platform_hw(void)
{
	rk30_mux_api_set(GPIO4C0_SMCDATA0_TRACEDATA0_NAME, GPIO4C_GPIO4C0);

	return 0;
}

static struct sensor_platform_data mma8452_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = mma8452_init_platform_hw,
        .orientation = {-1, 0, 0, 0, 0, 1, 0, -1, 0},
};
#endif
#if defined (CONFIG_GS_LIS3DH)
#define LIS3DH_INT_PIN   RK30_PIN4_PC0

static int lis3dh_init_platform_hw(void)
{
        rk30_mux_api_set(GPIO4C0_SMCDATA0_TRACEDATA0_NAME, GPIO4C_GPIO4C0);

        return 0;
}

static struct sensor_platform_data lis3dh_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = lis3dh_init_platform_hw,
	.orientation = {-1, 0, 0, 0, 0, 1, 0, -1, 0},
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
#define L3G4200D_INT_PIN  RK30_PIN4_PC3

static int l3g4200d_init_platform_hw(void)
{
	rk30_mux_api_set(GPIO4C3_SMCDATA3_TRACEDATA3_NAME, GPIO4C_GPIO4C3);
	
	return 0;
}

static struct sensor_platform_data l3g4200d_info = {
	.type = SENSOR_TYPE_GYROSCOPE,
	.irq_enable = 1,
	.poll_delay_ms = 30,
	.orientation = {0, 1, 0, -1, 0, 0, 0, 0, 1},
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

#define LCD_CS_MUX_NAME    GPIO4C7_SMCDATA7_TRACEDATA7_NAME
#define LCD_CS_PIN         INVALID_GPIO
#define LCD_CS_VALUE       GPIO_HIGH

#define LCD_EN_MUX_NAME    GPIO4C7_SMCDATA7_TRACEDATA7_NAME
#define LCD_EN_PIN         INVALID_GPIO
#define LCD_EN_VALUE       GPIO_LOW
#define LCD_EN_PIN         RK30_PIN6_PB4
#define LCD_EN_VALUE       GPIO_LOW

static int rk_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
	int ret = 0;
	if(LCD_CS_PIN != INVALID_GPIO) {
		rk30_mux_api_set(LCD_CS_MUX_NAME, GPIO4C_GPIO4C7);
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
	if(LCD_EN_PIN != INVALID_GPIO) {
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

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK30)
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

#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK30)
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

#if defined(CONFIG_LCDC0_RK30)
struct rk29fb_info lcdc0_screen_info = {
	.prop           = EXTEND,       //extend display device
       .lcd_info  = NULL,
       .set_screen_info = set_lcd_info,

};
#endif

#if defined(CONFIG_LCDC1_RK30)
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
#if defined(CONFIG_ARCH_RK30)
static struct resource resource_mali[] = {
	[0] = {
	.name  = "ump buf",
	.start = 0,
	.end   = 0,
	.flags = IORESOURCE_MEM,
	},

};

static struct platform_device device_mali= {
	.name		= "mali400_ump",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_mali),
	.resource	= resource_mali,
};
#endif

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK30)
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
#if defined(CONFIG_LCDC1_RK30) 
extern struct rk29fb_info lcdc1_screen_info;
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
	.io_init = rk_hdmi_power_init,
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
#include "board-rk30-sdk-sdmmc.c"
#endif

#ifdef CONFIG_SDMMC0_RK29
static int rk29_sdmmc0_cfg_gpio(void)
{
#ifdef CONFIG_SDMMC_RK29_OLD
	rk30_mux_api_set(GPIO3B1_SDMMC0CMD_NAME, GPIO3B_SDMMC0_CMD);
	rk30_mux_api_set(GPIO3B0_SDMMC0CLKOUT_NAME, GPIO3B_SDMMC0_CLKOUT);
	rk30_mux_api_set(GPIO3B2_SDMMC0DATA0_NAME, GPIO3B_SDMMC0_DATA0);
	rk30_mux_api_set(GPIO3B3_SDMMC0DATA1_NAME, GPIO3B_SDMMC0_DATA1);
	rk30_mux_api_set(GPIO3B4_SDMMC0DATA2_NAME, GPIO3B_SDMMC0_DATA2);
	rk30_mux_api_set(GPIO3B5_SDMMC0DATA3_NAME, GPIO3B_SDMMC0_DATA3);

	rk30_mux_api_set(GPIO3B6_SDMMC0DETECTN_NAME, GPIO3B_GPIO3B6);

	rk30_mux_api_set(GPIO3A7_SDMMC0PWREN_NAME, GPIO3A_GPIO3A7);
	gpio_request(RK30_PIN3_PA7, "sdmmc-power");
	gpio_direction_output(RK30_PIN3_PA7, GPIO_LOW);

#else
	    rk29_sdmmc_set_iomux(0, 0xFFFF);

    #if defined(CONFIG_SDMMC0_RK29_SDCARD_DET_FROM_GPIO)
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO);
    #else
	    rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FMUX);
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
	rk30_mux_api_set(GPIO3C0_SMMC1CMD_NAME, GPIO3C_SMMC1_CMD);
	rk30_mux_api_set(GPIO3C5_SDMMC1CLKOUT_NAME, GPIO3C_SDMMC1_CLKOUT);
	rk30_mux_api_set(GPIO3C1_SDMMC1DATA0_NAME, GPIO3C_SDMMC1_DATA0);
	rk30_mux_api_set(GPIO3C2_SDMMC1DATA1_NAME, GPIO3C_SDMMC1_DATA1);
	rk30_mux_api_set(GPIO3C3_SDMMC1DATA2_NAME, GPIO3C_SDMMC1_DATA2);
	rk30_mux_api_set(GPIO3C4_SDMMC1DATA3_NAME, GPIO3C_SDMMC1_DATA3);
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

#if defined(CONFIG_BATTERY_RK30_ADC)||defined(CONFIG_BATTERY_RK30_ADC_FAC)
static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
        .dc_det_pin      = RK30_PIN6_PA5,
        .batt_low_pin    = RK30_PIN6_PA0,
        .charge_set_pin  = INVALID_GPIO,
        .charge_ok_pin   = RK30_PIN6_PA6,
        .dc_det_level    = GPIO_LOW,
        .charge_ok_level = GPIO_HIGH,
};

static struct platform_device rk30_device_adc_battery = {
        .name   = "rk30-battery",
        .id     = -1,
        .dev = {
                .platform_data = &rk30_adc_battery_platdata,
        },
};
#endif

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
#ifdef CONFIG_RK_REMOTECTL

void rk30_remotectl_iomux(void)
{
	;
}

struct RKxx_remotectl_platform_data rk30_remotectl_pdata = {
    .gpio	=   RK30_PIN6_PA1, 
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

#ifdef CONFIG_RK30_PWM_REGULATOR
const static int pwm_voltage_map[] = {
        1000000, 1025000, 1050000, 1075000, 1100000, 1125000, 
		1150000, 1175000, 1200000, 1225000, 1250000, 1275000, 
		1300000, 1325000, 1350000, 1375000, 1400000
#ifdef OMEGAMOON_CHANGED
		// Omegamoon >> Set max voltage from 1400000 to 1425000
		, 1425000 
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
                .pwm_id = 3,
                .pwm_gpio = RK30_PIN0_PD7,
                .pwm_iomux_name = GPIO0D7_PWM3_NAME,
                .pwm_iomux_pwm = GPIO0D_PWM3,
                .pwm_iomux_gpio = GPIO0D_GPIO0D6,
                .pwm_voltage = 1100000,
                .suspend_voltage = 1050000,
                .min_uV = 1000000,
#ifdef OMEGAMOON_CHANGED
				// Omegamoon >> Set max voltage from 1400000 to 1425000
                .max_uV = 1425000,
#else
                .max_uV = 1400000,
#endif
                .coefficient = 455,     //45.5%
                .pwm_voltage_map = pwm_voltage_map,
                .init_data      = &pwm_regulator_init_dcdc[0],
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

#ifdef CONFIG_RK29_VMAC
#define PHY_PWR_EN_GPIO	RK30_PIN1_PD6
#include "board-rk30-sdk-vmac.c"
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
        .io             = RK30_PIN3_PC7, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = NULL,
        },
    },

    .reset_gpio         = { // BT_RST
        .io             = RK30_PIN3_PD1,
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = NULL,
        },
    },

    .irq_gpio           = {
        .io             = RK30_PIN6_PA7,
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

static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_ION
	&device_ion,
#endif
#if defined(CONFIG_WIFI_CONTROL_FUNC)||defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
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
#if defined(CONFIG_ARCH_RK30)
	&device_mali,
#endif
#ifdef CONFIG_MT5931_MT6622
	&device_mt6622,
#endif
#ifdef CONFIG_RK_REMOTECTL	
    &rk30_device_remotectl,
#endif
#ifdef CONFIG_SND_SOC_RK_HDMI_CODEC
	&rockchip_hdmi_codec,
#endif
#ifdef CONFIG_SND_RK_SOC_HDMI
	&rockchip_hdmi_audio,
#endif
};

#if defined(CONFIG_SII902XA)
static struct rkdisplay_platform_data hdmi_data = {
	#ifdef CONFIG_HDMI_RK30
	.property 		= DISPLAY_AUX,
	#else
	.property 		= DISPLAY_MAIN,
	#endif
	.video_source 	= DISPLAY_SOURCE_LCDC0,
	.io_pwr_pin 	= INVALID_GPIO,
	.io_reset_pin 	= RK30_PIN4_PD4,
};
#endif


static int rk_platform_add_display_devices(void)
{
	struct platform_device *fb = NULL;  //fb
	struct platform_device *lcdc0 = NULL; //lcdc0
	struct platform_device *lcdc1 = NULL; //lcdc1
	struct platform_device *bl = NULL; //backlight
#ifdef CONFIG_FB_ROCKCHIP
	fb = &device_fb;
#endif

#if defined(CONFIG_LCDC0_RK30)
	lcdc0 = &device_lcdc0,
#endif

#if defined(CONFIG_LCDC1_RK30)
	lcdc1 = &device_lcdc1,
#endif

#ifdef CONFIG_BACKLIGHT_RK29_BL
	bl = &rk29_device_backlight,
#endif
	__rk_platform_add_display_devices(fb,lcdc0,lcdc1,bl);

	return 0;
	
}

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
#ifndef OMEGAMOON_CHANGED
	// Omegamoon: .io_switch_pin was removed in recent driver
	.io_switch_pin	= INVALID_GPIO,
#endif
};
#endif


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
#if defined (CONFIG_GS_LIS3DH)
	{
		.type	        = "gs_lis3dh",
		.addr	        = 0x19,   //0x19(SA0-->VCC), 0x18(SA0-->GND)
		.flags	        = 0,
		.irq	        = LIS3DH_INT_PIN,
		.platform_data = &lis3dh_info,
	},
#endif
#if defined (CONFIG_GS_KXTIK)
	{
		.type	        = "gs_kxtik",
		.addr	        = 0x0F,
		.flags	        = 0,
		.irq	        = KXTIK_INT_PIN,
		.platform_data = &kxtik_info,
	},
#endif
#if defined (CONFIG_COMPASS_AK8975)
	{
		.type          = "ak8975",
		.addr          = 0x0d,
		.flags         = 0,
		.irq           = RK30_PIN4_PC1,
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
#if defined (CONFIG_LS_AL3006)
	{
		.type           = "light_al3006",
		.addr           = 0x1c,             //sel = 0; if sel =1, then addr = 0x1D
		.flags          = 0,
		.irq            = RK30_PIN6_PA2,	
		.platform_data = &light_al3006_info,
	},
#endif
#if defined (CONFIG_LS_STK3171)
	{
		.type           = "ls_stk3171",
		.addr           = 0x48,            
		.flags          = 0,
		.irq            = RK30_PIN6_PA2,	
		.platform_data = &light_stk3171_info,
	},
#endif


#if defined (CONFIG_PS_AL3006)
	{
		.type           = "proximity_al3006",
		.addr           = 0x1c,             //sel = 0; if sel =1, then addr = 0x1D
		.flags          = 0,
		.irq            = RK30_PIN6_PA2,	
		.platform_data = &proximity_al3006_info,
	},
#endif

#if defined (CONFIG_PS_STK3171)
	{
		.type           = "ps_stk3171",
		.addr           = 0x48,            
		.flags          = 0,
		.irq            = RK30_PIN6_PA2,	
		.platform_data = &proximity_stk3171_info,
	},
#endif
#if defined (CONFIG_SND_SOC_RK1000)
	{
		.type          = "rk1000_i2c_codec",
		.addr          = 0x60,
		.flags         = 0,
	},
	{
		.type          = "rk1000_control",
		.addr          = 0x40,
		.flags         = 0,
	},
#endif
#if defined (CONFIG_SND_SOC_RT5631)
        {
                .type                   = "rt5631",
                .addr                   = 0x1a,
                .flags                  = 0,
        },
#endif

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
#ifdef CONFIG_RK610_HDMI
		{
			.type			= "rk610_hdmi",
			.addr			= 0x46,
			.flags			= 0,
			.irq			= RK30_PIN4_PD3,
		},
#endif
#ifdef CONFIG_SND_SOC_RK610
		{//RK610_CODEC addr  from 0x60 to 0x80 (0x60~0x80)
			.type			= "rk610_i2c_codec",
			.addr			= 0x60,
			.flags			= 0,
		},
#endif
#endif

};
#endif
int __sramdata g_pmic_type =  0;
#ifdef CONFIG_I2C1_RK30
#ifdef CONFIG_MFD_WM831X_I2C
//Galland  #include "board-rk30-sdk-wm8326.c"
#define PMU_POWER_SLEEP RK30_PIN6_PB1	//Galland from board-rk30-sdk-wm8326.c
#include "board-pmu-wm8326.c" //Galland
#endif
#ifdef CONFIG_MFD_TPS65910
//Galland #define TPS65910_HOST_IRQ        RK30_PIN6_PA4
//Galland #include "board-rk30-sdk-tps65910.c"
//Galland struct from board-rk3168-ds1006h.c but with values from board-rk30-sdk-tps65910.c
#ifdef CONFIG_ARCH_RK3066B
#define TPS65910_HOST_IRQ        RK30_PIN0_PB3
#else
#define TPS65910_HOST_IRQ        RK30_PIN6_PA4
#endif

#define PMU_POWER_SLEEP RK30_PIN6_PB1 //Galland from board-rk30-sdk-tps65910.c (and PMIC_SLEEP in schematic)

static struct pmu_info  tps65910_dcdc_info[] = {
	{
		.name          = "vdd_cpu",   //logic
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "vdd2",    //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "vio",   //vcc_io
		.min_uv          = 3000000,
		.max_uv         = 3000000,
	},
	{
		.name          = "vaux1",   //vcc25_hdmi
		.min_uv          = 2500000,
		.max_uv         = 2500000,
	},
};
static  struct pmu_info  tps65910_ldo_info[] = {
	{
		.name          = "vpll",   //vcc25
		.min_uv          = 2500000,
		.max_uv         = 2500000,
	},
	{
		.name          = "vdig1",    //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "vdig2",   //vdd11
		.min_uv          = 1100000,
		.max_uv         = 1100000,
	},
	{
		.name          = "vmmc",   //vcc28_cif
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
		.name          = "vdac",   //vccio_wl
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
 };

#include "board-pmu-tps65910.c"
#endif

static struct i2c_board_info __initdata i2c1_info[] = {
#if defined (CONFIG_MFD_WM831X_I2C)
	{
		.type          = "wm8326",
		.addr          = 0x34,
		.flags         = 0,
		.irq           = RK30_PIN6_PA4,
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
	#if defined (CONFIG_MFD_WM831X_I2C)
       if(pmic_is_wm8326())
       board_pmu_wm8326_resume();
	#endif
	#if defined (CONFIG_MFD_TPS65910)
       if(pmic_is_tps65910())
       board_pmu_tps65910_resume(); 
	#endif
}

#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {
#if defined (CONFIG_TOUCHSCREEN_GT8XX)
	{
		.type          = "Goodix-TS",
		.addr          = 0x55,
		.flags         = 0,
		.irq           = RK30_PIN4_PC2,
		.platform_data = &goodix_info,
	},
#endif
#if defined (CONFIG_LS_CM3217)
	{
		.type          = "light_cm3217",
		.addr          = 0x10,
		.flags         = 0,
		.platform_data = &cm3217_info,
	},
#endif
#if defined(CONFIG_DP501)
	{
		.type = "dp501",
		.addr = 0x30,
		.flags = 0,
		.platform_data = &dp501_platform_data,
	},
#endif
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
};
#endif

#ifdef CONFIG_I2C3_RK30
static struct i2c_board_info __initdata i2c3_info[] = {
};
#endif

#ifdef CONFIG_I2C4_RK30
static struct i2c_board_info __initdata i2c4_info[] = {
};
#endif

#ifdef CONFIG_I2C_GPIO_RK30
#define I2C_SDA_PIN     INVALID_GPIO// RK30_PIN2_PD6   //set sda_pin here
#define I2C_SCL_PIN     INVALID_GPIO// RK30_PIN2_PD7   //set scl_pin here

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
	#ifdef CONFIG_CH7025_7026_TVOUT
	{
		.type			= "ch7025/7026",
		.addr			= 0x75,
		.flags			= 0,
		.platform_data = &tv_data,
	},
	#endif
	#if defined (CONFIG_SII902XA)
    {
		.type           = "sii902x",
        .addr           = 0x72 >> 1,
        .flags          = 0,
        .irq            = RK30_PIN4_PD3,
        .platform_data = &hdmi_data,
    },
	#endif
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

#define POWER_ON_PIN RK30_PIN6_PB0   //power_hold
static void rk30_pm_power_off(void)
{
	printk(KERN_ERR "rk30_pm_power_off start...\n");
	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	#if defined(CONFIG_MFD_WM831X)	
	if(pmic_is_wm8326())
	{
		wm831x_set_bits(Wm831x,WM831X_GPIO_LEVEL,0x0001,0x0000);  //set sys_pwr 0
		wm831x_device_shutdown(Wm831x);//wm8326 shutdown
	}
	#endif
	#if defined(CONFIG_MFD_TPS65910)
	if(pmic_is_tps65910())
	{
		tps65910_device_shutdown();//tps65910 shutdown
	}
	#endif

	while (1);
}

static void __init machine_rk30_board_init(void)
{
	avs_init();
	gpio_request(POWER_ON_PIN, "poweronpin");
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
	
	pm_power_off = rk30_pm_power_off;
	
	rk30_i2c_register_board_info();
	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	rk_platform_add_display_devices();
	board_usb_detect_init(RK30_PIN6_PA3);

#if defined(CONFIG_WIFI_CONTROL_FUNC)
	rk29sdk_wifi_bt_gpio_control_init();
#elif defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
    rk29sdk_wifi_combo_module_gpio_init();
#endif

#if defined(CONFIG_MT6620)
    clk_set_rate(clk_get_sys("rk_serial.0", "uart"), 48*1000000);
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
	board_mem_reserved();
}

/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs 
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 * @logic_volt	: logic voltage arm requests depend on frequency
 * comments	: min arm/logic voltage
 */
static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
#ifdef OMEGAMOON_CHANGED
	{.frequency = 252 * 1000,	.cpu_volt = 1025 * 1000,	.logic_volt = 1050 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1025 * 1000,	.logic_volt = 1100 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1050 * 1000,	.logic_volt = 1150 * 1000},//1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1100 * 1000,	.logic_volt = 1150 * 1000},//1.025V/1.050V
/*	{.frequency = 1200 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},//1.100V/1.050V */
	{.frequency = 1200 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1150 * 1000},//1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1225 * 1000,	.logic_volt = 1200 * 1000},//1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1300 * 1000,	.logic_volt = 1200 * 1000},//1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1350 * 1000,	.logic_volt = 1250 * 1000},//1.300V/1.150V
	{.frequency = 1608 * 1000,	.cpu_volt = 1375 * 1000,	.logic_volt = 1275 * 1000},//1.325V/1.175V
	{.frequency = 1704 * 1000,	.cpu_volt = 1400 * 1000,	.logic_volt = 1300 * 1000},//1.325V/1.175V
	{.frequency = 1800 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},//1.325V/1.175V
    // Omegamoon >>	Beware, 1425 volt seems to be the maximum!
#else
	{.frequency = 252 * 1000,	.cpu_volt = 1075 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1100 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},//1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},//1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},//1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1225 * 1000,	.logic_volt = 1200 * 1000},//1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1300 * 1000,	.logic_volt = 1200 * 1000},//1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1350 * 1000,	.logic_volt = 1250 * 1000},//1.300V/1.150V
	{.frequency = 1608 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1300 * 1000},//1.325V/1.175V
#endif
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_gpu_table[] = {	
#ifdef OMEGAMOON_CHANGED_timing
	{.frequency = 266 * 1000,	.index = 875 * 1000},
	{.frequency = 350 * 1000,	.index = 925 * 1000},
	{.frequency = 440 * 1000,	.index = 975 * 1000},
	{.frequency = 533 * 1000,	.index = 1000 * 1000},
	{.frequency = 600 * 1000,	.index = 1025 * 1000},
	{.frequency = 640 * 1000,	.index = 1050 * 1000},
	{.frequency = 666 * 1000,	.index = 1075 * 1000},
	{.frequency = 700 * 1000,	.index = 1100 * 1000},
	{.frequency = 740 * 1000,	.index = 1125 * 1000},
#else
	{.frequency = 266 * 1000,	.index = 1050 * 1000},
	{.frequency = 400 * 1000,	.index = 1275 * 1000},
#endif
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
    {.frequency = 200 * 1000 + DDR_FREQ_SUSPEND, .index = 1050 * 1000},
	{.frequency = 300 * 1000 + DDR_FREQ_VIDEO, .index = 1050 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL, .index = 1125 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

#define DVFS_CPU_TABLE_SIZE	(ARRAY_SIZE(dvfs_cpu_logic_table))
static struct cpufreq_frequency_table cpu_dvfs_table[DVFS_CPU_TABLE_SIZE];
static struct cpufreq_frequency_table dep_cpu2core_table[DVFS_CPU_TABLE_SIZE];

#ifdef OMEGAMOON_CHANGED
void rk30_clk_dump_regs(void);
#endif

void __init board_clock_init(void)
{
#ifdef OMEGAMOON_CHANGED
	printk("Omegamoon >> %s called\n", __func__);
#endif
	rk30_clock_data_init(periph_pll_default, codec_pll_default, RK30_CLOCKS_DEFAULT_FLAGS);
	dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
#ifdef OMEGAMOON_CHANGED
	dvfs_clk_enable_limit(clk_get(NULL, "gpu"), 133 * 1000000, 266 * 1000000);
	dvfs_clk_enable_limit(clk_get(NULL, "cpu"), 126 * 1000000, 1608 * 1000000);
#endif
#ifdef OMEGAMOON_CHANGED
	//printk("Omegamoon >> rk30_clk_dump_regs()...");
	//rk30_clk_dump_regs();
	printk("Omegamoon >> Current GPU frequency is %luMHz\n", clk_get_rate(clk_get(NULL, "gpu"))/1000000);
	printk("Omegamoon >> Current CPU frequency is %luMHz\n", clk_get_rate(clk_get(NULL, "cpu"))/1000000);
	printk("Omegamoon >> aclk_cpu = %lu MHz\n", clk_get_rate(clk_get(NULL, "aclk_cpu"))/1000000);
	printk("Omegamoon >> hclk_cpu = %lu MHz\n", clk_get_rate(clk_get(NULL, "hclk_cpu"))/1000000);
	printk("Omegamoon >> pclk_cpu = %lu MHz\n", clk_get_rate(clk_get(NULL, "pclk_cpu"))/1000000);
	printk("Omegamoon >> aclk_periph = %lu MHz\n", clk_get_rate(clk_get(NULL, "aclk_periph"))/1000000);
	printk("Omegamoon >> hclk_periph = %lu MHz\n", clk_get_rate(clk_get(NULL, "hclk_periph"))/1000000);
	printk("Omegamoon >> pclk_periph = %lu MHz\n", clk_get_rate(clk_get(NULL, "pclk_periph"))/1000000);
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
