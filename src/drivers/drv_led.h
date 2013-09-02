/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file drv_led.h
 *
 * LED driver API
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

#define LED_DEVICE_PATH		"/dev/led"

#if defined(CONFIG_ARCH_BOARD_TMRFC_V1)
#define PCA953X_DEVICE_PATH "/dev/led"
#endif

#define _LED_BASE		0x2800

/* LED colour codes */
#if defined(CONFIG_ARCH_BOARD_TMRFC_V1)
/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with stm32_setled()
 *  
 *  Note : All LEDs are controlled by PCA9533 and PCA9536 in TMR-FC V1.0 via I2C
 */
 
#define BOARD_LED1        0 /* PCA9533 LED0 : AMBER */
#define BOARD_LED2        1 /* PCA9533 LED1 : BLUE */
#define BOARD_LED3        2 /* PCA9533 LED2 : GREEN ( ON, OS in running state )*/
#define BOARD_LED4        3 /* PCA9533 LED3 : RED     */
#define BOARD_LED5        3 /* PCA9536 IO3   : RED ( Power ON ) */
#define BOARD_NLEDS       5

/* LED bits for use with stm32_setleds() */

#define BOARD_LED1_BIT    ((1 << BOARD_LED1) | 0x330)
#define BOARD_LED2_BIT    ((1 << BOARD_LED2) | 0x330)
#define BOARD_LED3_BIT    ((1 << BOARD_LED3) | 0x330)
#define BOARD_LED4_BIT    ((1 << BOARD_LED4) | 0x330)
#define BOARD_LED5_BIT    ((1 << BOARD_LED5) | 0x360)

#define BOARD_LED_BLUE    BOARD_LED1_BIT
#define BOARD_LED_RED     BOARD_LED3_BIT

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on board the
 * tmrfc-v1.  The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* LED1 */
#define LED_STACKCREATED  3  /* LED1 + LED2 */
#define LED_INIRQ         4  /* LED1 */
#define LED_SIGNAL        5  /* LED2 */
#define LED_ASSERTION     6  /* LED1 + LED2 */
#define LED_PANIC         7  /* LED1 + LED2 */

#define LED_AMBER		BOARD_LED4_BIT
#define LED_BLUE		BOARD_LED2_BIT
#define LED_GREEN       BOARD_LED3_BIT
#define LED_RED			BOARD_LED4_BIT
#define LED_RED_PWR  	BOARD_LED5_BIT

#define LED_ON			_IOC(_LED_BASE, 0)
#define LED_OFF			_IOC(_LED_BASE, 1)
#define LED_TOGGLE		_IOC(_LED_BASE, 2)
#else
#define LED_AMBER		1
#define LED_RED			1	/* some boards have red rather than amber */
#define LED_BLUE		0
#define LED_SAFETY		2

#define LED_ON			_IOC(_LED_BASE, 0)
#define LED_OFF			_IOC(_LED_BASE, 1)
#define LED_TOGGLE		_IOC(_LED_BASE, 2)
#endif

__BEGIN_DECLS

/*
 * Initialise the LED driver.
 */
__EXPORT void drv_led_start(void);

#if defined(CONFIG_ARCH_BOARD_TMRFC_V1)
/* All LEDs controlled by PCA9533 and PCA9536 in TMR-FC via I2C */
__EXPORT void drv_pca953x_start(void);
#endif

__END_DECLS
