/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 TMR Development Team. All rights reserved.
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
 * @file pca953x.cpp
 *
 * Driver for the onboard NXP PCA9533 4-bit I2C-bus LED dimmer  
 * and PCA9536 4-bit I2C-bus and SMBus I/O port connected via I2C.
 *
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

#include <nuttx/wqueue.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_led.h>
#include <drivers/drv_gpio.h>

class PCA953X : public device::I2C
{
public:
	PCA953X(int bus, int pca953x);
	virtual ~PCA953X();


	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	work_s			_work;

    /**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Reset the device
	 */
	int			reset();

    uint8_t pca953x_update(uint8_t pca_id);
    uint8_t pca9533_set_peroid(uint8_t psc, uint32_t msec);
    uint8_t pca9533_set_pwm(uint8_t pwm, uint32_t duty);
    uint8_t pca9533_set_led(uint8_t led, uint32_t mode);
    uint8_t pca9536_config_io(uint8_t io, uint8_t set);
};

/* for now, we only support one PCA953X */
namespace
{
	PCA953X *g_pca953x;
}


extern "C" __EXPORT int pca953x_main(int argc, char *argv[]);

PCA953X::PCA953X(int bus, int pca953x) :
	I2C("pca953x", PCA953X_DEVICE_PATH, bus, pca953x, 100000)
{
	memset(&_work, 0, sizeof(_work));
}

PCA953X::~PCA953X()
{
}

int
PCA953X::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	return OK;
}

int
PCA953X::probe()
{
	int ret = true;

	return ret;
}

int
PCA953X::info()
{
	int ret;

	if (ret == OK) {
		/* we don't care about power-save mode */
		#if 0
		log("state: %s", on ? "ON" : "OFF");
		log("red: %u, green: %u, blue: %u", (unsigned)r, (unsigned)g, (unsigned)b);
		#endif
		;
	} else {
		warnx("failed to read pca953x");
	}

	return ret;
}

int
PCA953X::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;
	switch (cmd) {

	default:
		break;
	}

	return ret;
}

void pca953x_usage();


void pca953x_usage() {
	errx(0, "missing command: try 'start', 'stop', 'test', 'info'");
}

int
pca953x_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int pca953xadr = 0;
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc-1, &argv[1], "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			pca953xadr = strtol(optarg, NULL, 0);
			break;
		case 'b':
			i2cdevice = strtol(optarg, NULL, 0);
			break;
		default:
			pca953x_usage();
		}
	}

	const char *verb = argv[1];

	int fd;
	int ret;

	if (!strcmp(verb, "start")) {
		if (g_pca953x != nullptr)
			errx(1, "already started");

		if (i2cdevice == -1) {
			// try the external bus first
			i2cdevice = TMR_I2C_BUS_EXPANSION;
			g_pca953x = new PCA953X(TMR_I2C_BUS_EXPANSION, pca953xadr);
			if (g_pca953x != nullptr && OK != g_pca953x->init()) {
				delete g_pca953x;
				g_pca953x = nullptr;
			}
			if (g_pca953x == nullptr) {
				// fall back to default bus
				i2cdevice = TMR_I2C_BUS_ONBOARD;
			}
		}
		if (g_pca953x == nullptr) {
			g_pca953x = new PCA953X(i2cdevice, pca953xadr);
			if (g_pca953x == nullptr)
				errx(1, "new failed");

			if (OK != g_pca953x->init()) {
				delete g_pca953x;
				g_pca953x = nullptr;
				errx(1, "init failed");
			}
		}

		exit(0);
	}

	/* need the driver past this point */
	if (g_pca953x == nullptr) {
	    warnx("not started");
	    pca953x_usage();
	    exit(0);
	}

	if (!strcmp(verb, "test")) {
		fd = open(PCA953X_DEVICE_PATH, 0);
		if (fd == -1) {
			errx(1, "Unable to open " PCA953X_DEVICE_PATH);
		}

        #if 0
		ret = ioctl(fd, PCA953X_SET_PATTERN, (unsigned long)&pattern);
		#endif

		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "info")) {
		g_pca953x->info();
		exit(0);
	}

    #if 0
	if (!strcmp(verb, "stop") || !strcmp(verb, "off")) {
		/* although technically it doesn't stop, this is the excepted syntax */
		fd = open(PCA953X_DEVICE_PATH, 0);
		if (fd == -1) {
			errx(1, "Unable to open " PCA953X_DEVICE_PATH);
		}
		ret = ioctl(fd, PCA953X_SET_MODE, (unsigned long)PCA953X_MODE_OFF);
		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "rgb")) {
		fd = open(PCA953X_DEVICE_PATH, 0);
		if (fd == -1) {
			errx(1, "Unable to open " PCA953X_DEVICE_PATH);
		}
		if (argc < 5) {
			errx(1, "Usage: pca953x rgb <red> <green> <blue>");
		}
		pca953x_rgbset_t v;
		v.red   = strtol(argv[1], NULL, 0);
		v.green = strtol(argv[2], NULL, 0);
		v.blue  = strtol(argv[3], NULL, 0);
		ret = ioctl(fd, PCA953X_SET_RGB, (unsigned long)&v);
		close(fd);
		exit(ret);
	}
	#endif

	pca953x_usage();
}
