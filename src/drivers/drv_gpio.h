/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file drv_gpio.h
 *
 * Generic GPIO ioctl interface.
 */

#ifndef _DRV_GPIO_H
#define _DRV_GPIO_H

#include <sys/ioctl.h>

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
/*
 * PX4FMU GPIO numbers.
 *
 * For shared pins, alternate function 1 selects the non-GPIO mode 
 * (USART2, CAN2, etc.)
 */
# define GPIO_EXT_1		(1<<0)		/**< high-power GPIO 1 */
# define GPIO_EXT_2		(1<<1)		/**< high-power GPIO 1 */
# define GPIO_MULTI_1		(1<<2)		/**< USART2 CTS */
# define GPIO_MULTI_2		(1<<3)		/**< USART2 RTS */
# define GPIO_MULTI_3		(1<<4)		/**< USART2 TX */
# define GPIO_MULTI_4		(1<<5)		/**< USART2 RX */
# define GPIO_CAN_TX		(1<<6)		/**< CAN2 TX */
# define GPIO_CAN_RX		(1<<7)		/**< CAN2 RX */

/**
 * Default GPIO device - other devices may also support this protocol if
 * they also export GPIO-like things.  This is always the GPIOs on the
 * main board.
 */
# define PX4FMU_DEVICE_PATH	"/dev/px4fmu"
# define PX4IO_DEVICE_PATH	"/dev/px4io"

#ifndef PX4IO_DEVICE_PATH
#  error No GPIO support for this board.
#endif
#elif defined(CONFIG_ARCH_BOARD_TMRFC_V1)
/*
 * TMRFC GPIO numbers.
 *
 * For shared pins, alternate function 1 selects the non-GPIO mode 
 * (USART2, CAN2, etc.)
 */
# define GPIO_EXT_1		(1<<0)		/**< high-power GPIO 1 */
# define GPIO_EXT_2		(1<<1)		/**< high-power GPIO 1 */
# define GPIO_MULTI_1		(1<<2)		/**< USART2 CTS */
# define GPIO_MULTI_2		(1<<3)		/**< USART2 RTS */
# define GPIO_MULTI_3		(1<<4)		/**< USART2 TX */
# define GPIO_MULTI_4		(1<<5)		/**< USART2 RX */
# define GPIO_CAN_TX		(1<<6)		/**< CAN2 TX */
# define GPIO_CAN_RX		(1<<7)		/**< CAN2 RX */

/**
 * Default GPIO device - other devices may also support this protocol if
 * they also export GPIO-like things.  This is always the GPIOs on the
 * main board.
 */
# define TMRFC_DEVICE_PATH	"/dev/tmrfc"
#endif

/*
 * IOCTL definitions.
 *
 * For all ioctls, the (arg) argument is a bitmask of GPIOs to be affected
 * by the operation, with the LSB being the lowest-numbered GPIO.
 *
 * Note that there may be board-specific relationships between GPIOs;
 * applications using GPIOs should be aware of this.
 */
#define _GPIOCBASE	0x2700
#define GPIOC(_x)	_IOC(_GPIOCBASE, _x)

/** reset all board GPIOs to their default state */
#define GPIO_RESET	GPIOC(0)

/** configure the board GPIOs in (arg) as outputs */
#define GPIO_SET_OUTPUT	GPIOC(1)

/** configure the board GPIOs in (arg) as inputs */
#define GPIO_SET_INPUT	GPIOC(2)

/** configure the board GPIOs in (arg) for the first alternate function (if supported) */
#define GPIO_SET_ALT_1	GPIOC(3)

/** configure the board GPIO (arg) for the second alternate function (if supported) */
#define GPIO_SET_ALT_2	GPIOC(4)

/** configure the board GPIO (arg) for the third alternate function (if supported) */
#define GPIO_SET_ALT_3	GPIOC(5)

/** configure the board GPIO (arg) for the fourth alternate function (if supported) */
#define GPIO_SET_ALT_4	GPIOC(6)

/** set the GPIOs in (arg) */
#define GPIO_SET	GPIOC(10)

/** clear the GPIOs in (arg) */
#define GPIO_CLEAR	GPIOC(11)

/** read all the GPIOs and return their values in *(uint32_t *)arg */
#define GPIO_GET	GPIOC(12)

#endif /* _DRV_GPIO_H */