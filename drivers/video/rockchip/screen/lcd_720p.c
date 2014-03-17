#ifndef __LCD_720P_RBOX__
#define __LCD_720P_RBOX__

/* $_FOR_ROCKCHIP_RBOX_$ */
//#include <linux/fb.h>
//#include <linux/delay.h>
//#include <mach/gpio.h>
//#include <mach/iomux.h>
//#include <mach/board.h>
//#include <linux/rk_fb.h>
//#include <linux/rk_screen.h>

#define SCREEN_TYPE	    SCREEN_RGB
#define OUT_TYPE		SCREEN_HDMI
#define OUT_FACE		OUT_P888 // Omegamoon >>> Originally: OUT_P888
#define LCDC_ACLK		312000000
#define LVDS_FORMAT     LVDS_8BIT_2

#define DCLK	    	74250000
//#define LCDC_ACLK     300000000

/* Timing */
#define H_PW			40
#define H_BP			220
#define H_VD			1280
#define H_FP			110

#define V_PW			5
#define V_BP			20
#define V_VD			720
#define V_FP			5

#define LCD_WIDTH       1280
#define LCD_HEIGHT      720

/* Other */
#define DCLK_POL		1
#define SWAP_RB			0
#define DEN_POL			0
#define VSYNC_POL		0
#define HSYNC_POL		0

#define SWAP_RB			0
#define SWAP_RG			0
#define SWAP_GB			0

#endif