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
*   Interface file for the user-gpio driver, which allows user-mode
*   access to gpio pins.
*
****************************************************************************/

#if !defined( USER_GPIO_DRV_H )
#define USER_GPIO_DRV_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>

/* ---- Constants and Types ---------------------------------------------- */

#define GPIO_MAGIC  'G'

#define GPIO_CMD_REQUEST                0x80
#define GPIO_CMD_FREE                   0x81
#define GPIO_CMD_DIRECTION_INPUT        0x82
#define GPIO_CMD_DIRECTION_OUTPUT       0x83
#define GPIO_CMD_GET_VALUE              0x84
#define GPIO_CMD_SET_VALUE              0x85
#define GPIO_CMD_ISREQUESTED            0x86

typedef struct
{
    unsigned    gpio;
    char        label[ 32 ];

} GPIO_Request_t;

typedef struct
{
    unsigned    gpio;
    int         value;

} GPIO_Value_t;

#define GPIO_IOCTL_REQUEST              _IOW(   GPIO_MAGIC, GPIO_CMD_REQUEST, GPIO_Request_t )              /* arg is GPIO_Request_t * */
#define GPIO_IOCTL_FREE                 _IO(    GPIO_MAGIC, GPIO_CMD_FREE )                                 /* arg is int */
#define GPIO_IOCTL_DIRECTION_INPUT      _IO(    GPIO_MAGIC, GPIO_CMD_DIRECTION_INPUT )                      /* arg is int */
#define GPIO_IOCTL_DIRECTION_OUTPUT     _IOW(   GPIO_MAGIC, GPIO_CMD_DIRECTION_OUTPUT, GPIO_Value_t )       /* arg is GPIO_Value_t * */
#define GPIO_IOCTL_GET_VALUE            _IOWR(  GPIO_MAGIC, GPIO_CMD_GET_VALUE, GPIO_Value_t )              /* arg is GPIO_Value_t * */
#define GPIO_IOCTL_SET_VALUE            _IOW(   GPIO_MAGIC, GPIO_CMD_SET_VALUE, GPIO_Value_t )              /* arg is GPIO_Value_t * */
#define GPIO_IOCTL_ISREQUEST             _IOW(   GPIO_MAGIC, GPIO_CMD_ISREQUESTED, GPIO_Value_t )              /* arg is GPIO_Value_t * */

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* USER_GPIO_DRV_H */

