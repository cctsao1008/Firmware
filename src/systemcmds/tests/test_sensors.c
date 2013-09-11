/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file test_sensors.c
 * Tests the onboard sensors.
 * 
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "tests.h"

#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int accel(int argc, char *argv[]);
static int gyro(int argc, char *argv[]);
static int mag(int argc, char *argv[]);
static int baro(int argc, char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct {
	const char	*name;
	const char	*path;
	int	(* test)(int argc, char *argv[]);
} sensors[] = {
	{"accel",	"/dev/accel",	accel},
	{"gyro",	"/dev/gyro",	gyro},
	{"mag",		"/dev/mag",	mag},
	{"baro",	"/dev/baro",	baro},
	{NULL, NULL, NULL}
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int
accel(int argc, char *argv[])
{
	printf("\tACCEL: test start\n");
	fflush(stdout);

	int		fd;
	struct accel_report buf;
	int		ret;

	fd = open("/dev/accel", O_RDONLY);

	if (fd < 0) {
		printf("\tACCEL: open fail, run <mpu6000 start> or <lsm303 start> or <bma180 start> first.\n");
		return ERROR;
	}

	/* wait at least 100ms, sensor should have data after no more than 20ms */
	usleep(100000);

	/* read data - expect samples */
	ret = read(fd, &buf, sizeof(buf));

	if (ret != sizeof(buf)) {
		printf("\tACCEL: read fail (%d)\n", ret);
		return ERROR;

	} else {
		printf("\tACCEL accel: x:%8.4f\ty:%8.4f\tz:%8.4f m/s^2\n", (double)buf.x, (double)buf.y, (double)buf.z);
	}

	// /* wait at least 10ms, sensor should have data after no more than 2ms */
	// usleep(100000);

	// ret = read(fd, buf, sizeof(buf));

	// if (ret != sizeof(buf)) {
	// 	printf("\tMPU-6000: read2 fail (%d)\n", ret);
	// 	return ERROR;

	// } else {
	// 	printf("\tMPU-6000 values: acc: x:%d\ty:%d\tz:%d\tgyro: r:%d\tp:%d\ty:%d\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
	// }

	/* XXX more tests here */

	/* Let user know everything is ok */
	printf("\tOK: ACCEL passed all tests successfully\n");

	return OK;
}

static int
gyro(int argc, char *argv[])
{
	printf("\tGYRO: test start\n");
	fflush(stdout);

	int		fd;
	struct gyro_report buf;
	int		ret;

	fd = open("/dev/gyro", O_RDONLY);

	if (fd < 0) {
		printf("\tGYRO: open fail, run <l3gd20 start> or <mpu6000 start> first.\n");
		return ERROR;
	}

	/* wait at least 5 ms, sensor should have data after that */
	usleep(5000);

	/* read data - expect samples */
	ret = read(fd, &buf, sizeof(buf));

	if (ret != sizeof(buf)) {
		printf("\tGYRO: read fail (%d)\n", ret);
		return ERROR;

	} else {
		printf("\tGYRO rates: x:%8.4f\ty:%8.4f\tz:%8.4f rad/s\n", (double)buf.x, (double)buf.y, (double)buf.z);
	}

	/* Let user know everything is ok */
	printf("\tOK: GYRO passed all tests successfully\n");

	return OK;
}

static int
mag(int argc, char *argv[])
{
	printf("\tMAG: test start\n");
	fflush(stdout);

	int		fd;
	struct mag_report buf;
	int		ret;

	fd = open("/dev/mag", O_RDONLY);

	if (fd < 0) {
		printf("\tMAG: open fail, run <hmc5883 start> or <lsm303 start> first.\n");
		return ERROR;
	}

	/* wait at least 5 ms, sensor should have data after that */
	usleep(5000);

	/* read data - expect samples */
	ret = read(fd, &buf, sizeof(buf));

	if (ret != sizeof(buf)) {
		printf("\tMAG: read fail (%d)\n", ret);
		return ERROR;

	} else {
		printf("\tMAG values: x:%8.4f\ty:%8.4f\tz:%8.4f\n", (double)buf.x, (double)buf.y, (double)buf.z);
	}

	/* Let user know everything is ok */
	printf("\tOK: MAG passed all tests successfully\n");

	return OK;
}

static int
baro(int argc, char *argv[])
{
	printf("\tBARO: test start\n");
	fflush(stdout);

	int		fd;
	struct baro_report buf;
	int		ret;

	fd = open("/dev/baro", O_RDONLY);

	if (fd < 0) {
		printf("\tBARO: open fail, run <ms5611 start> or <lps331 start> first.\n");
		return ERROR;
	}

	/* wait at least 5 ms, sensor should have data after that */
	usleep(5000);

	/* read data - expect samples */
	ret = read(fd, &buf, sizeof(buf));

	if (ret != sizeof(buf)) {
		printf("\tBARO: read fail (%d)\n", ret);
		return ERROR;

	} else {
		printf("\tBARO pressure: %8.4f mbar\talt: %8.4f m\ttemp: %8.4f deg C\n", (double)buf.pressure, (double)buf.altitude, (double)buf.temperature);
	}

	/* Let user know everything is ok */
	printf("\tOK: BARO passed all tests successfully\n");

	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_sensors
 ****************************************************************************/

int test_sensors(int argc, char *argv[])
{
	unsigned	i;

	printf("Running sensors tests:\n\n");
	fflush(stdout);

	int ret = OK;

	for (i = 0; sensors[i].name; i++) {
		printf("  sensor: %s\n", sensors[i].name);

		/* Flush and leave enough time for the flush to become effective */
		fflush(stdout);
		usleep(50000);
		/* Test the sensor - if the tests crash at this point, the right sensor name has been printed */

		ret += sensors[i].test(argc, argv);
	}

	return ret;
}
