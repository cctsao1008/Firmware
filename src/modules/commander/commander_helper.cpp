/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
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
 * @file commander_helper.cpp
 * Commander helper functions implementations
 */

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_led.h>
#include <drivers/drv_rgbled.h>


#include "commander_helper.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

bool is_multirotor(const struct vehicle_status_s *current_status)
{
	return ((current_status->system_type == VEHICLE_TYPE_QUADROTOR) ||
	    (current_status->system_type == VEHICLE_TYPE_HEXAROTOR) ||
	    (current_status->system_type == VEHICLE_TYPE_OCTOROTOR) ||
	    (current_status->system_type == VEHICLE_TYPE_TRICOPTER));
}

bool is_rotary_wing(const struct vehicle_status_s *current_status)
{
	return is_multirotor(current_status) || (current_status->system_type == VEHICLE_TYPE_HELICOPTER)
	|| (current_status->system_type == VEHICLE_TYPE_COAXIAL);
}

static int buzzer;

int buzzer_init()
{
	buzzer = open("/dev/tone_alarm", O_WRONLY);

	if (buzzer < 0) {
		warnx("Buzzer: open fail\n");
		return ERROR;
	}

	return OK;
}

void buzzer_deinit()
{
	close(buzzer);
}

void tune_error()
{
	ioctl(buzzer, TONE_SET_ALARM, 2);
}

void tune_positive()
{
	ioctl(buzzer, TONE_SET_ALARM, 3);
}

void tune_neutral()
{
	ioctl(buzzer, TONE_SET_ALARM, 4);
}

void tune_negative()
{
	ioctl(buzzer, TONE_SET_ALARM, 5);
}

int tune_arm()
{
	return ioctl(buzzer, TONE_SET_ALARM, 12);
}

int tune_low_bat()
{
	return ioctl(buzzer, TONE_SET_ALARM, 13);
}

int tune_critical_bat()
{
	return ioctl(buzzer, TONE_SET_ALARM, 14);
}



void tune_stop()
{
	ioctl(buzzer, TONE_SET_ALARM, 0);
}

static int leds;
static int rgbleds;

int led_init()
{
	/* first open normal LEDs */
	leds = open(LED_DEVICE_PATH, 0);

	if (leds < 0) {
		warnx("LED: open fail\n");
		return ERROR;
	}

	/* the blue LED is only available on FMUv1 but not FMUv2 */
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1

	if (ioctl(leds, LED_ON, LED_BLUE)) {
		warnx("Blue LED: ioctl fail\n");
		return ERROR;
	}
#endif

	if (ioctl(leds, LED_ON, LED_AMBER)) {
		warnx("Amber LED: ioctl fail\n");
		return ERROR;
	}

	/* then try RGB LEDs, this can fail on FMUv1*/
	rgbleds = open(RGBLED_DEVICE_PATH, 0);
	if (rgbleds == -1) {
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
		errx(1, "Unable to open " RGBLED_DEVICE_PATH);
#else
		warnx("No RGB LED found");
#endif
	}

	return 0;
}

void led_deinit()
{
	close(leds);

	if (rgbleds != -1) {
		close(rgbleds);
	}
}

int led_toggle(int led)
{
	return ioctl(leds, LED_TOGGLE, led);
}

int led_on(int led)
{
	return ioctl(leds, LED_ON, led);
}

int led_off(int led)
{
	return ioctl(leds, LED_OFF, led);
}

void rgbled_set_color(rgbled_color_t color) {

	if (rgbleds != -1)
		ioctl(rgbleds, RGBLED_SET_COLOR, (unsigned long)color);
}

void rgbled_set_mode(rgbled_mode_t mode) {

	if (rgbleds != -1)
		ioctl(rgbleds, RGBLED_SET_MODE, (unsigned long)mode);
}

void rgbled_set_pattern(rgbled_pattern_t *pattern) {

	if (rgbleds != -1)
		ioctl(rgbleds, RGBLED_SET_PATTERN, (unsigned long)pattern);
}

float battery_remaining_estimate_voltage(float voltage)
{
	float ret = 0;
	static param_t bat_volt_empty;
	static param_t bat_volt_full;
	static param_t bat_n_cells;
	static bool initialized = false;
	static unsigned int counter = 0;
	static float ncells = 3;
	// XXX change cells to int (and param to INT32)

	if (!initialized) {
		bat_volt_empty = param_find("BAT_V_EMPTY");
		bat_volt_full = param_find("BAT_V_FULL");
		bat_n_cells = param_find("BAT_N_CELLS");
		initialized = true;
	}

	static float chemistry_voltage_empty = 3.2f;
	static float chemistry_voltage_full = 4.05f;

	if (counter % 100 == 0) {
		param_get(bat_volt_empty, &chemistry_voltage_empty);
		param_get(bat_volt_full, &chemistry_voltage_full);
		param_get(bat_n_cells, &ncells);
	}

	counter++;

	ret = (voltage - ncells * chemistry_voltage_empty) / (ncells * (chemistry_voltage_full - chemistry_voltage_empty));

	/* limit to sane values */
	ret = (ret < 0.0f) ? 0.0f : ret;
	ret = (ret > 1.0f) ? 1.0f : ret;
	return ret;
}
