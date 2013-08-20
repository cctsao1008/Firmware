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
 * @file px4io.h
 *
 * General defines and structures for the PX4IO module firmware.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <drivers/boards/px4io/px4io_internal.h>

#include "protocol.h"

/*
 * Constants and limits.
 */
#define MAX_CONTROL_CHANNELS	12
#define IO_SERVO_COUNT		8

/*
 * Debug logging
 */

#ifdef DEBUG
# include <debug.h>
# define debug(fmt, args...)	lowsyslog(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

/*
 * Registers.
 */
extern uint16_t			r_page_status[];	/* PX4IO_PAGE_STATUS */
extern uint16_t			r_page_actuators[];	/* PX4IO_PAGE_ACTUATORS */
extern uint16_t			r_page_servos[];	/* PX4IO_PAGE_SERVOS */
extern uint16_t			r_page_raw_rc_input[];	/* PX4IO_PAGE_RAW_RC_INPUT */
extern uint16_t			r_page_rc_input[];	/* PX4IO_PAGE_RC_INPUT */
extern uint16_t			r_page_adc[];		/* PX4IO_PAGE_RAW_ADC_INPUT */

extern volatile uint16_t	r_page_setup[];		/* PX4IO_PAGE_SETUP */
extern volatile uint16_t	r_page_controls[];	/* PX4IO_PAGE_CONTROLS */
extern uint16_t			r_page_rc_input_config[]; /* PX4IO_PAGE_RC_INPUT_CONFIG */
extern uint16_t			r_page_servo_failsafe[]; /* PX4IO_PAGE_FAILSAFE_PWM */

/*
 * Register aliases.
 *
 * Handy aliases for registers that are widely used.
 */
#define r_status_flags		r_page_status[PX4IO_P_STATUS_FLAGS]
#define r_status_alarms		r_page_status[PX4IO_P_STATUS_ALARMS]

#define r_raw_rc_count		r_page_raw_rc_input[PX4IO_P_RAW_RC_COUNT]
#define r_raw_rc_values		(&r_page_raw_rc_input[PX4IO_P_RAW_RC_BASE])
#define r_rc_valid		r_page_rc_input[PX4IO_P_RC_VALID]
#define r_rc_values		(&r_page_rc_input[PX4IO_P_RAW_RC_BASE])

#define r_setup_features	r_page_setup[PX4IO_P_SETUP_FEATURES]
#define r_setup_arming		r_page_setup[PX4IO_P_SETUP_ARMING]
#define r_setup_pwm_rates	r_page_setup[PX4IO_P_SETUP_PWM_RATES]
#define r_setup_pwm_defaultrate	r_page_setup[PX4IO_P_SETUP_PWM_DEFAULTRATE]
#define r_setup_pwm_altrate	r_page_setup[PX4IO_P_SETUP_PWM_ALTRATE]
#define r_setup_relays		r_page_setup[PX4IO_P_SETUP_RELAYS]

#define r_control_values	(&r_page_controls[0])

/*
 * System state structure.
 */
struct sys_state_s {

	volatile uint64_t	rc_channels_timestamp;

	/**
	 * Last FMU receive time, in microseconds since system boot
	 */
	volatile uint64_t	fmu_data_received_time;

};

extern struct sys_state_s system_state;

/*
 * GPIO handling.
 */
#define LED_BLUE(_s)		stm32_gpiowrite(GPIO_LED1, !(_s))
#define LED_AMBER(_s)		stm32_gpiowrite(GPIO_LED2, !(_s))
#define LED_SAFETY(_s)		stm32_gpiowrite(GPIO_LED3, !(_s))

#define POWER_SERVO(_s)		stm32_gpiowrite(GPIO_SERVO_PWR_EN, (_s))
#ifdef GPIO_ACC1_PWR_EN
    #define POWER_ACC1(_s)		stm32_gpiowrite(GPIO_ACC1_PWR_EN, (_s))
#endif
#ifdef GPIO_ACC2_PWR_EN
    #define POWER_ACC2(_s)		stm32_gpiowrite(GPIO_ACC2_PWR_EN, (_s))
#endif
#ifdef GPIO_RELAY1_EN
    #define POWER_RELAY1(_s)	stm32_gpiowrite(GPIO_RELAY1_EN, (_s))
#endif
#ifdef GPIO_RELAY2_EN
    #define POWER_RELAY2(_s)	stm32_gpiowrite(GPIO_RELAY2_EN, (_s))
#endif

#define OVERCURRENT_ACC		(!stm32_gpioread(GPIO_ACC_OC_DETECT))
#define OVERCURRENT_SERVO	(!stm32_gpioread(GPIO_SERVO_OC_DETECT))
#define BUTTON_SAFETY		stm32_gpioread(GPIO_BTN_SAFETY)

#define ADC_VBATT		4
#define ADC_IN5			5
#define ADC_CHANNEL_COUNT	2

/*
 * Mixer
 */
extern void	mixer_tick(void);
extern void	mixer_handle_text(const void *buffer, size_t length);

/**
 * Safety switch/LED.
 */
extern void	safety_init(void);

#ifdef CONFIG_STM32_I2C1
/**
 * FMU communications
 */
extern void	i2c_init(void);
#endif

/**
 * Register space
 */
extern void	registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);
extern int	registers_get(uint8_t page, uint8_t offset, uint16_t **values, unsigned *num_values);

/**
 * Sensors/misc inputs
 */
extern int	adc_init(void);
extern uint16_t	adc_measure(unsigned channel);

/**
 * R/C receiver handling.
 *
 * Input functions return true when they receive an update from the RC controller.
 */
extern void	controls_init(void);
extern void	controls_tick(void);
extern int	dsm_init(const char *device);
extern bool	dsm_input(uint16_t *values, uint16_t *num_values);
extern void	dsm_bind(uint16_t cmd, int pulses);
extern int	sbus_init(const char *device);
extern bool	sbus_input(uint16_t *values, uint16_t *num_values);

/** global debug level for isr_debug() */
extern volatile uint8_t debug_level;

/* send a debug message to the console */
extern void	isr_debug(uint8_t level, const char *fmt, ...);

#ifdef CONFIG_STM32_I2C1
void		i2c_dump(void);
void		i2c_reset(void);
#endif
