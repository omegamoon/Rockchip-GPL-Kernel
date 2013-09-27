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
*   User mode front-end for manipulating gpio pins.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "user-gpio.h"

/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

/* ---- Private Variables ------------------------------------------------ */

/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Functions  ------------------------------------------------------- */

/****************************************************************************
*
*  Usage - prints the programs usage.
*
***************************************************************************/

static void Usage( void )
{
    fprintf( stderr, "Usage: gpio command args\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "Command is one of the following:\n" );
    fprintf( stderr, "  request gpio label   Requests a gpio for use and assigns a label\n" );
    fprintf( stderr, "  free gpio            Releases a gpio\n" );
    fprintf( stderr, "  input gpio           Configures a GPIO for input\n" );
    fprintf( stderr, "  output gpio value    Configures a GPIO for output and sets the initial value\n" );
    fprintf( stderr, "  get gpio             Reports the current value of a GPIO pin\n" );
    fprintf( stderr, "  set gpio value       Sets the value of an output pin\n" );
    fprintf( stderr, "  dump gpio       Dumps gpio to kernel log\n" );
}

/****************************************************************************
*
*  main
*
***************************************************************************/

int main( int argc, char **argv )
{
    char               *cmdStr;
    unsigned int        gpio;

    if ( gpio_init() < 0 )
    {
        perror( "gpio_init failed" );
        exit( 1 );
    }

    if (( argc > 4 ) || ( argc < 3 ))
    {
        Usage();
        exit( 1 );
    }

    cmdStr = argv[ 1 ];
    gpio = (unsigned int)strtoul( argv[2], NULL, 0 );

    if ( argc == 4 )
    {
        char *argStr = argv[3];

        if ( strcmp( cmdStr, "request" ) == 0 )
        {
            if ( gpio_request( gpio, argStr ) < 0 )
            {
                perror( "Request failed" );
                exit( 1 );
            }
        }
        else
        if ( strcmp( cmdStr, "output" ) == 0 )
        {
            if( gpio_direction_output( gpio, atoi( argStr )) < 0 )
            {
                perror( "gpio_direction_output failed" );
                exit( 1 );
            }
        }
        else
        if ( strcmp( cmdStr, "set" ) == 0 )
        {
            gpio_set_value( gpio, atoi( argStr ));
        }
        else
        {
            Usage();
            exit( 1 );
        }
    }
    else
    if ( argc == 3 )
    {
        if ( strcmp( cmdStr, "free" ) == 0 )
        {
            gpio_free( gpio );
        }
        else
        if ( strcmp( cmdStr, "input" ) == 0 )
        {
            if ( gpio_direction_input( gpio ) < 0 )
            {
                perror( "gpio_direction_input failed" );
                exit( 1 );
            }
        }
        else
        if ( strcmp( cmdStr, "get" ) == 0 )
        {
            int rc = gpio_get_value( gpio );

            if ( rc < 0 )
            {
                perror( "gpio_get_value failed" );
                exit( 1 );
            }
            printf( "%d\n", rc != 0 );
        }
        else
        if ( strcmp( cmdStr, "dump" ) == 0 )
        {
            int rc = gpio_is_requested( 1,"dump" );

            if ( rc < 0 )
            {
                perror( "gpio_is_requested failed" );
                exit( 1 );
            }
            printf( "%d\n", rc != 0 );
        }
        else
        {
            Usage();
            exit( 1 );
        }
    }

    gpio_term();
    exit( 0 );
    return 0;
}

