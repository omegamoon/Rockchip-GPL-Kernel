/****************************************************************************
*
*   Copyright (c) 2008 Dave Hylands     <dhylands@gmail.com>
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*
*   Alternatively, this software may be distributed under the terms of BSD
*   license.
*
*   See README and COPYING for more details.
*
****************************************************************************
*
*   Interface file for the user-gpio library, which allows user-mode
*   access to gpio pins.
*
****************************************************************************/

#if !defined( USER_GPIO_H )
#define USER_GPIO_H

/* ---- Include Files ---------------------------------------------------- */

/* ---- Constants and Types ---------------------------------------------- */

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

#if defined( __cplusplus )
extern "C" {
#endif

int  gpio_init( void );
void gpio_term( void );

/*
 * This is the same API as gpiolib, which is used from kernel space. 
 */ 

int  gpio_request( unsigned gpio, const char *label );
void gpio_free( unsigned gpio );
int  gpio_direction_input( unsigned gpio );
int  gpio_direction_output( unsigned gpio, int initialValue );
int  gpio_get_value( unsigned gpio );
void gpio_set_value( unsigned gpio, int value );
int  gpio_is_requested( unsigned gpio, const char *label );

#if defined( __cplusplus )
}
#endif

#endif  /* USER_GPIO_H */

