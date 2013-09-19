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


/*
 * @file tmrfc_pwm_servo.c
 *
 * Configuration data for the stm32 pwm_servo driver.
 *
 * Note that these arrays must always be fully-sized.
 */

#include <stdint.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#include <drivers/stm32/drv_pwm_servo.h>
#include <drivers/drv_pwm_output.h>

#include "board_config.h"

#define TIM3_INDEX 0
#define TIM4_INDEX 1
#define TIM5_INDEX 2
#define TIM8_INDEX 3

__EXPORT const struct pwm_servo_timer pwm_timers[PWM_SERVO_MAX_TIMERS] = {
    {
        .base = STM32_TIM3_BASE,
        .clock_register = STM32_RCC_APB1ENR,
        .clock_bit = RCC_APB1ENR_TIM3EN,
        .clock_freq = STM32_APB1_TIM3_CLKIN
    },
    {
        .base = STM32_TIM4_BASE,
        .clock_register = STM32_RCC_APB1ENR,
        .clock_bit = RCC_APB1ENR_TIM4EN,
        .clock_freq = STM32_APB1_TIM4_CLKIN
    },
    {
        .base = STM32_TIM5_BASE,
        .clock_register = STM32_RCC_APB1ENR,
        .clock_bit = RCC_APB1ENR_TIM5EN,
        .clock_freq = STM32_APB1_TIM5_CLKIN
    },
    {
        .base = STM32_TIM8_BASE,
        .clock_register = STM32_RCC_APB2ENR,
        .clock_bit = RCC_APB2ENR_TIM8EN,
        .clock_freq = STM32_APB2_TIM8_CLKIN
    }
};

__EXPORT const struct pwm_servo_channel pwm_channels[PWM_SERVO_MAX_CHANNELS] = {
    {
        .gpio = GPIO_TIM3_CH1OUT,
        .timer_index = TIM3_INDEX,
        .timer_channel = 1,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM3_CH2OUT,
        .timer_index = TIM3_INDEX,
        .timer_channel = 2,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM3_CH3OUT,
        .timer_index = TIM3_INDEX,
        .timer_channel = 3,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM3_CH4OUT,
        .timer_index = TIM3_INDEX,
        .timer_channel = 4,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM4_CH3OUT,
        .timer_index = TIM4_INDEX,
        .timer_channel = 3,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM4_CH4OUT,
        .timer_index = TIM4_INDEX,
        .timer_channel = 4,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM8_CH2OUT,
        .timer_index = TIM8_INDEX,
        .timer_channel = 2,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM8_CH3OUT,
        .timer_index = TIM8_INDEX,
        .timer_channel = 3,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM5_CH1OUT,
        .timer_index = TIM5_INDEX,
        .timer_channel = 1,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM5_CH4OUT,
        .timer_index = TIM5_INDEX,
        .timer_channel = 4,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM5_CH3OUT,
        .timer_index = TIM5_INDEX,
        .timer_channel = 3,
        .default_value = 1000,
    },
    {
        .gpio = GPIO_TIM8_CH1OUT,
        .timer_index = TIM8_INDEX,
        .timer_channel = 1,
        .default_value = 1000,
    }
};
