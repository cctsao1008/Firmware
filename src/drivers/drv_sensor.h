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
 * @file drv_sensor.h
 *
 * Common sensor API and ioctl definitions.
 */

#ifndef _DRV_SENSOR_H
#define _DRV_SENSOR_H

#include <stdint.h>
#include <sys/ioctl.h>

/*
 * ioctl() definitions
 *
 * Note that a driver may not implement all of these operations, but
 * if the operation is implemented it should conform to this API.
 */

#define _SENSORIOCBASE		(0x2000)
#define _SENSORIOC(_n)		(_IOC(_SENSORIOCBASE, _n))

/**
 * Set the driver polling rate to (arg) Hz, or one of the SENSOR_POLLRATE
 * constants
 */
#define SENSORIOCSPOLLRATE	_SENSORIOC(0)

/**
 * Return the driver's approximate polling rate in Hz, or one of the
 * SENSOR_POLLRATE values.
 */
#define SENSORIOCGPOLLRATE	_SENSORIOC(1)

#define SENSOR_POLLRATE_MANUAL		1000000	/**< poll when read */
#define SENSOR_POLLRATE_EXTERNAL	1000001	/**< poll when device signals ready */
#define SENSOR_POLLRATE_MAX		1000002	/**< poll at device maximum rate */
#define SENSOR_POLLRATE_DEFAULT		1000003	/**< poll at driver normal rate */

/**
 * Set the internal queue depth to (arg) entries, must be at least 1
 *
 * This sets the upper bound on the number of readings that can be
 * read from the driver.
 */
#define SENSORIOCSQUEUEDEPTH	_SENSORIOC(2)

/** return the internal queue depth */
#define SENSORIOCGQUEUEDEPTH	_SENSORIOC(3)

/**
 * Reset the sensor to its default configuration.
 */
#define SENSORIOCRESET		_SENSORIOC(4)

#endif /* _DRV_SENSOR_H */