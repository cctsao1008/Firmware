/****************************************************************************
 *
 *   Copyright (C) 2012 TMR Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name TMR nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tmrfc_led.c
 *
 * TMRFC LED backend.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <fcntl.h>

#include <systemlib/err.h>

#include "stm32.h"
#include "board_config.h"

#include <arch/board/board.h>

#include <drivers/drv_led.h>

#define _LED_BASE       0x2800
#define LED_ON          _IOC(_LED_BASE, 0)
#define LED_OFF         _IOC(_LED_BASE, 1)
#define LED_TOGGLE      _IOC(_LED_BASE, 2)

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init();
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

int fd;

__EXPORT void led_init()
{
    /* Configure LED1-5 for output */
    fd = open(PCA953X_DEVICE_PATH, O_RDONLY);
}

__EXPORT void led_on(int led)
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

__EXPORT void led_off(int led)
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
    
__EXPORT void led_toggle(int led)
{
    if (led == BOARD_LED1_BIT)
    {
        ioctl(fd, LED_TOGGLE, BOARD_LED1_BIT);
    }
    
    if (led == BOARD_LED2)
    {
        ioctl(fd, LED_TOGGLE, BOARD_LED2_BIT);
    }
    
    if (led == BOARD_LED3)
    {
        ioctl(fd, LED_TOGGLE, BOARD_LED3_BIT);
    }
    
    if (led == BOARD_LED4)
    {
        ioctl(fd, LED_TOGGLE, BOARD_LED4_BIT);
    }

    if (led == BOARD_LED5)
    {
        ioctl(fd, LED_TOGGLE, BOARD_LED5_BIT);
    }
}
