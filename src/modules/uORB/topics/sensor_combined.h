/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file sensor_combined.h
 * Definition of the sensor_combined uORB topic.
 */

#ifndef SENSOR_COMBINED_H_
#define SENSOR_COMBINED_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

enum MAGNETOMETER_MODE {
	MAGNETOMETER_MODE_NORMAL = 0,
	MAGNETOMETER_MODE_POSITIVE_BIAS,
	MAGNETOMETER_MODE_NEGATIVE_BIAS
};

/**
 * @addtogroup topics
 * @{
 */

/**
 * Sensor readings in raw and SI-unit form.
 *
 * These values are read from the sensors. Raw values are in sensor-specific units,
 * the scaled values are in SI-units, as visible from the ending of the variable
 * or the comments. The use of the SI fields is in general advised, as these fields
 * are scaled and offset-compensated where possible and do not change with board
 * revisions and sensor updates.
 *
 */
struct sensor_combined_s {

	/*
	 * Actual data, this is specific to the type of data which is stored in this struct
	 * A line containing L0GME will be added by the Python logging code generator to the
	 * logged dataset.
	 */

	/* NOTE: Ordering of fields optimized to align to 32 bit / 4 bytes Change with consideration only   */

	uint64_t timestamp;			/**< Timestamp in microseconds since boot         */

	int16_t	gyro_raw[3];			/**< Raw sensor values of angular velocity        */
	uint16_t gyro_counter;			/**< Number of raw measurments taken              */
	float gyro_rad_s[3];			/**< Angular velocity in radian per seconds       */
	
	int16_t accelerometer_raw[3];		/**< Raw acceleration in NED body frame           */
	uint32_t accelerometer_counter;		/**< Number of raw acc measurements taken         */
	float accelerometer_m_s2[3];		/**< Acceleration in NED body frame, in m/s^2     */
	int accelerometer_mode;			/**< Accelerometer measurement mode */
	float accelerometer_range_m_s2;		/**< Accelerometer measurement range in m/s^2 */

	int16_t	magnetometer_raw[3];		/**< Raw magnetic field in NED body frame         */
	float magnetometer_ga[3];		/**< Magnetic field in NED body frame, in Gauss   */
	int magnetometer_mode;			/**< Magnetometer measurement mode */
	float magnetometer_range_ga;		/**< ± measurement range in Gauss */
	float magnetometer_cuttoff_freq_hz;	/**< Internal analog low pass frequency of sensor */
	uint32_t magnetometer_counter;		/**< Number of raw mag measurements taken         */
	
	float baro_pres_mbar;			/**< Barometric pressure, already temp. comp.     */
	float baro_alt_meter;			/**< Altitude, already temp. comp.                */
	float baro_temp_celcius;		/**< Temperature in degrees celsius               */
	float adc_voltage_v[4];			/**< ADC voltages of ADC Chan 10/11/12/13 or -1      */
	float mcu_temp_celcius;			/**< Internal temperature measurement of MCU */
	uint32_t baro_counter;			/**< Number of raw baro measurements taken        */

	float differential_pressure_pa;				/**< Airspeed sensor differential pressure                  */ 
	uint32_t differential_pressure_counter;		/**< Number of raw differential pressure measurements taken */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(sensor_combined);

#endif
