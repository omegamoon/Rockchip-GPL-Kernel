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
*   Implements the user-gpio library, which allows user-mode
*   access to gpio pins.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include "user-gpio.h"
#include "user-gpio-drv.h"

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

/* ---- Private Variables ------------------------------------------------ */

static int  gFd = -1;

/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Functions  ------------------------------------------------------- */

/****************************************************************************
*
*   Initializes the GPIO library.
*
***************************************************************************/

int gpio_init( void )
{
    if ( gFd < 0 )
    {
        if (( gFd = open( "/dev/user-gpio", O_RDWR )) < 0 )
        {
            return -1;
        }
    }
    return 0;
}

/****************************************************************************
*
*   Terminates the GPIO library.
*
***************************************************************************/

void gpio_term( void )
{
    if ( gFd >= 0 )
    {
        close( gFd );
        gFd = -1;
    }
}

/****************************************************************************
*
*   Requests a GPIO - fails if the GPIO is already "owned" and returns label
*
*   Returns 0 on success, -1 on error.
*
***************************************************************************/

int  gpio_is_requested( unsigned gpio, const char *label )
{
    GPIO_Request_t  request;

    request.gpio = gpio;
    strncpy( request.label, label, sizeof( request.label ));
    request.label[ sizeof( request.label ) - 1 ] = '\0';

    if ( ioctl( gFd, GPIO_IOCTL_ISREQUEST, &request ) != 0 )
    {
        return -errno;
    }
    return 0;
}
/****************************************************************************
*
*   Requests a GPIO - fails if the GPIO is already "owned"
*
*   Returns 0 on success, -1 on error.
*
***************************************************************************/

int  gpio_request( unsigned gpio, const char *label )
{
    GPIO_Request_t  request;

    request.gpio = gpio;
    strncpy( request.label, label, sizeof( request.label ));
    request.label[ sizeof( request.label ) - 1 ] = '\0';

    if ( ioctl( gFd, GPIO_IOCTL_REQUEST, &request ) != 0 )
    {
        return -errno;
    }
    return 0;
}

/****************************************************************************
*
*   Releases a previously request GPIO
*
***************************************************************************/

void gpio_free( unsigned gpio )
{
    ioctl( gFd, GPIO_IOCTL_FREE, gpio );
}

/****************************************************************************
*
*   Configures a GPIO for input
*
***************************************************************************/

int  gpio_direction_input( unsigned gpio )
{
    if ( ioctl( gFd, GPIO_IOCTL_DIRECTION_INPUT, gpio ) != 0 )
    {
        return -errno;
    }
    return 0;
}

/****************************************************************************
*
*   Configures a GPIO for output and sets the initial value.
*
*   Returns 0 if the direction was set successfully, negative on error.
*
***************************************************************************/

int  gpio_direction_output( unsigned gpio, int initialValue )
{
    GPIO_Value_t    setDirOutput;

    setDirOutput.gpio = gpio;
    setDirOutput.value = initialValue;

    if ( ioctl( gFd, GPIO_IOCTL_DIRECTION_OUTPUT, &setDirOutput ) != 0 )
    {
        return -errno;
    }
    return 0;
}

/****************************************************************************
*
*   Retrieves the value of a GPIO pin.
*
*   Returns 0 if the pin is low, non-zero (not necessarily 1) if the pin is high.
*   Returns negative if an error occurs.
*
***************************************************************************/

int  gpio_get_value( unsigned gpio )
{
    GPIO_Value_t    getValue;

    getValue.gpio = gpio;

    if ( ioctl( gFd, GPIO_IOCTL_GET_VALUE, &getValue ) != 0 )
    {
        return -errno;
    }
    return getValue.value;
}

/****************************************************************************
*
*   Sets the value of the GPIO pin.
*
***************************************************************************/

void gpio_set_value( unsigned gpio, int value )
{
    GPIO_Value_t    setValue;

    setValue.gpio = gpio;
    setValue.value = value;

    ioctl( gFd, GPIO_IOCTL_SET_VALUE, &setValue );
}

