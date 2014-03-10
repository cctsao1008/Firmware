/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 TMR Development Team. All rights reserved.
 *
 *   Author : CHIA-CHENG, TSAO
 *
 *   E-Mail : chiacheng.tsao@gmail.com
 *
 *   Date :07/09/2013
 *
 ****************************************************************************/

/**
 * @file tmrfc_led.c
 *
 * TMRFC LED backend.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>

#include <fcntl.h>

#include <systemlib/err.h>

#include "stm32.h"
#include "board_config.h"

#include <arch/board/board.h>

#include <drivers/drv_led.h>

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
 int led_init(void);
 void led_on(uint16_t led);
 void led_off(uint16_t led);
 void led_toggle(uint16_t led);
__END_DECLS

int fd = -1;

__EXPORT int led_init(void)
{
    /* Configure LED1-5 for output */
    if (fd > 0)
    {
        printf("[TMRFC_LED] led_init, already initialized \n");
        return 1;
    }

    fd = open(PCA953X_DEVICE_PATH, O_RDONLY);

    if(fd > 0)
    {
        printf("[TMRFC_LED] led_init, successfully \n");
        return 1;
    }

     printf("[TMRFC_LED] led_init, failed \n");

     return -1;
    
}

__EXPORT void led_on(uint16_t led)
{
    printf("[TMRFC_LED] led_on, led = 0x%X \n", led);

    if(fd > 0)
    {
        if (led == BOARD_LED1_BIT)
        {
            ioctl(fd, LED_ON, BOARD_LED1_BIT);
        }
    
        if (led == BOARD_LED2_BIT)
        {
            ioctl(fd, LED_ON, BOARD_LED2_BIT);
        }
    
        if (led == BOARD_LED3_BIT)
        {
            ioctl(fd, LED_ON, BOARD_LED3_BIT);
        }
    
        if (led == BOARD_LED4_BIT)
        {
            ioctl(fd, LED_ON, BOARD_LED4_BIT);
        }

        if (led == BOARD_LED5_BIT)
        {
            ioctl(fd, LED_ON, BOARD_LED5_BIT);
        }
    }
}

__EXPORT void led_off(uint16_t led)
{
    printf("[TMRFC_LED] led_off, led = 0x%X \n", led);

    if(fd > 0)
    {
        if (led == BOARD_LED1_BIT)
        {
            ioctl(fd, LED_OFF, BOARD_LED1_BIT);
        }
    
        if (led == BOARD_LED2_BIT)
        {
            ioctl(fd, LED_OFF, BOARD_LED2_BIT);
        }
    
        if (led == BOARD_LED3_BIT)
        {
            ioctl(fd, LED_OFF, BOARD_LED3_BIT);
        }
    
        if (led == BOARD_LED4_BIT)
        {
            ioctl(fd, LED_OFF, BOARD_LED4_BIT);
        }

        if (led == BOARD_LED5_BIT)
        {
            ioctl(fd, LED_OFF, BOARD_LED5_BIT);
        }
    }

}
    
__EXPORT void led_toggle(uint16_t led)
{
    printf("[TMRFC_LED] led_toggle, led = 0x%X \n", led);

    if(fd > 0)
    {
        if (led == BOARD_LED1_BIT)
        {
            ioctl(fd, LED_TOGGLE, BOARD_LED1_BIT);
        }
    
        if (led == BOARD_LED2_BIT)
        {
            ioctl(fd, LED_TOGGLE, BOARD_LED2_BIT);
        }
    
        if (led == BOARD_LED3_BIT)
        {
            ioctl(fd, LED_TOGGLE, BOARD_LED3_BIT);
        }
    
        if (led == BOARD_LED4_BIT)
        {
            ioctl(fd, LED_TOGGLE, BOARD_LED4_BIT);
        }

        if (led == BOARD_LED5_BIT)
        {
            ioctl(fd, LED_TOGGLE, BOARD_LED5_BIT);
        }
    }

}

