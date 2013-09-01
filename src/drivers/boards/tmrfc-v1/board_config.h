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
 * @file board_config.h
 *
 * TMRFC internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <stm32.h>
#include <arch/board/board.h>
 
/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

//#ifdef CONFIG_STM32_SPI2
//#  error "SPI2 is not supported on this board"
//#endif

#if defined(CONFIG_STM32_CAN1)
#  warning "CAN1 is not supported on this board"
#endif

/* TMRFC GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN15)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)

/* External interrupts */
#define GPIO_EXTI_COMPASS   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN3)
// MPU6050 DRDY?

/* SPI chip selects */

#define GPIO_SPI_CS_GYRO    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN14)
#define GPIO_SPI_CS_ACCEL   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
#define GPIO_SPI_CS_MPU     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)
#define GPIO_SPI_CS_SDCARD  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

/*
 * I2C busses
 */
#define TMR_I2C_BUS_ONBOARD     2
#define TMR_I2C_BUS_EXPANSION   1

/*
 * Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define TMR_I2C_OBDEV_HMC5883   0x1e
#define TMR_I2C_OBDEV_MS5611    0x77
#define TMR_I2C_OBDEV_PCA9536   0x41
#define TMR_I2C_OBDEV_PCA9533   0x62

#define TMR_I2C_OBDEV_EEPROM    NOTDEFINED

/* User GPIOs
 *
 * GPIO0-1 are the buffered high-power GPIOs.
 * GPIO2-5 are the USART2 pins.
 * GPIO6-7 are the CAN2 pins.
 */
#define GPIO_GPIO0_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN4)
#define GPIO_GPIO1_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN5)
#define GPIO_GPIO2_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN0)
#define GPIO_GPIO3_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO4_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO5_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO6_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN13)
#define GPIO_GPIO7_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN2)
#define GPIO_GPIO0_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN4)
#define GPIO_GPIO1_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN5)
#define GPIO_GPIO2_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#define GPIO_GPIO3_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO4_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO5_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO6_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN13)
#define GPIO_GPIO7_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN12)
#define GPIO_GPIO_DIR       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)


/*
 * PWM
 *
 * Four PWM outputs can be configured on pins otherwise shared with
 * USART2; two can take the flow control pins if they are not being used.
 *
 * Pins:
 *
 * CTS - PA0 - TIM2CH1
 * RTS - PA1 - TIM2CH2
 * TX  - PA2 - TIM2CH3
 * RX  - PA3 - TIM2CH4
 *
 */
#define GPIO_TIM3_CH1OUT    GPIO_TIM3_CH1OUT_1 // PWM OUT CH1
#define GPIO_TIM3_CH2OUT    GPIO_TIM3_CH2OUT_1
#define GPIO_TIM3_CH3OUT    GPIO_TIM3_CH3OUT_1
#define GPIO_TIM3_CH4OUT    GPIO_TIM3_CH4OUT_1
#define GPIO_TIM4_CH3OUT    GPIO_TIM4_CH3OUT_1
#define GPIO_TIM4_CH4OUT    GPIO_TIM4_CH4OUT_1
#define GPIO_TIM8_CH2OUT    GPIO_TIM8_CH2N_2
#define GPIO_TIM8_CH3OUT    GPIO_TIM8_CH3N_2

#define GPIO_TIM5_CH1OUT    GPIO_TIM5_CH1OUT_1
#define GPIO_TIM5_CH4OUT    GPIO_TIM5_CH4OUT_1
#define GPIO_TIM5_CH3OUT    GPIO_TIM5_CH3OUT_1
#define GPIO_TIM2_CH1OUT    GPIO_TIM2_CH1OUT_2
#define GPIO_TIM8_CH1OUT    GPIO_TIM8_CH1N_1

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 */
#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN13)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER  (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN2)
#else
#  define GPIO_OTGFS_OVER  (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN2)
#endif

/* High-resolution timer
 */
#define HRT_TIMER		1	/* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */
#define HRT_PPM_CHANNEL		3	/* use capture/compare channel 3 */
#define GPIO_PPM_IN		(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN10)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the TMRFC board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);
extern void stm32_usbinitialize(void);

#endif /* __ASSEMBLY__ */

__END_DECLS
