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
 * @file hmc5883.cpp
 *
 * Driver for the HMC5883 magnetometer connected via I2C.
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <float.h>

/*
 * HMC5883 internal constants and data structures.
 */
 
#if defined(CONFIG_ARCH_BOARD_TMRFC_V1)
#define HMC5883L_ADDRESS		TMR_I2C_OBDEV_HMC5883
#else
#define HMC5883L_ADDRESS		PX4_I2C_OBDEV_HMC5883
#endif

/* Max measurement rate is 160Hz */
#define HMC5883_CONVERSION_INTERVAL	(1000000 / 160)	/* microseconds */

#define ADDR_CONF_A			0x00
#define ADDR_CONF_B			0x01
#define ADDR_MODE			0x02
#define ADDR_DATA_OUT_X_MSB		0x03
#define ADDR_DATA_OUT_X_LSB		0x04
#define ADDR_DATA_OUT_Z_MSB		0x05
#define ADDR_DATA_OUT_Z_LSB		0x06
#define ADDR_DATA_OUT_Y_MSB		0x07
#define ADDR_DATA_OUT_Y_LSB		0x08
#define ADDR_STATUS			0x09
#define ADDR_ID_A			0x0a
#define ADDR_ID_B			0x0b
#define ADDR_ID_C			0x0c

/* modes not changeable outside of driver */
#define HMC5883L_MODE_NORMAL		(0 << 0)  /* default */
#define HMC5883L_MODE_POSITIVE_BIAS	(1 << 0)  /* positive bias */
#define HMC5883L_MODE_NEGATIVE_BIAS	(1 << 1)  /* negative bias */

#define HMC5883L_AVERAGING_1		(0 << 5) /* conf a register */
#define HMC5883L_AVERAGING_2		(1 << 5)
#define HMC5883L_AVERAGING_4		(2 << 5)
#define HMC5883L_AVERAGING_8		(3 << 5)

#define MODE_REG_CONTINOUS_MODE		(0 << 0)
#define MODE_REG_SINGLE_MODE		(1 << 0) /* default */

#define STATUS_REG_DATA_OUT_LOCK	(1 << 1) /* page 16: set if data is only partially read, read device to reset */
#define STATUS_REG_DATA_READY		(1 << 0) /* page 16: set if all axes have valid measurements */

#define ID_A_WHO_AM_I			'H'
#define ID_B_WHO_AM_I			'4'
#define ID_C_WHO_AM_I			'3'


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class HMC5883 : public device::I2C
{
public:
	HMC5883(int bus);
	virtual ~HMC5883();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:
	work_s			_work;
	unsigned		_measure_ticks;

	unsigned		_num_reports;
	volatile unsigned	_next_report;
	volatile unsigned	_oldest_report;
	mag_report		*_reports;
	mag_scale		_scale;
	float 			_range_scale;
	float 			_range_ga;
	bool			_collect_phase;

	orb_advert_t		_mag_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/* status reporting */
	bool			_sensor_ok;		/**< sensor was found and reports ok */
	bool			_calibrated;		/**< the calibration is valid */

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return		True if the device is present.
	 */
	int			probe_address(uint8_t address);

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
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int			calibrate(struct file *filp, unsigned enable);

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test positive strap, -1 to enable
	 *        negative strap, 0 to set to normal mode
	 */
	int			set_excitement(unsigned enable);

	/**
	 * Set the sensor range.
	 *
	 * Sets the internal range to handle at least the argument in Gauss.
	 */
	int 			set_range(unsigned range);

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in		A signed 16-bit big-endian value.
	 * @return		The floating-point representation of the value.
	 */
	float			meas_to_float(uint8_t in[2]);

	/**
	 * Check the current calibration and update device status
	 *
	 * @return 0 if calibration is ok, 1 else
	 */
	 int 			check_calibration();

	 /**
	 * Check the current scale calibration
	 *
	 * @return 0 if scale calibration is ok, 1 else
	 */
	 int 			check_scale();

	 /**
	 * Check the current offset calibration
	 *
	 * @return 0 if offset calibration is ok, 1 else
	 */
	 int 			check_offset();

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hmc5883_main(int argc, char *argv[]);


HMC5883::HMC5883(int bus) :
	I2C("HMC5883", MAG_DEVICE_PATH, bus, HMC5883L_ADDRESS, 400000),
	_measure_ticks(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_range_scale(0), /* default range scale from counts to gauss */
	_range_ga(1.3f),
	_mag_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "hmc5883_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hmc5883_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "hmc5883_buffer_overflows")),
	_sensor_ok(false),
	_calibrated(false)
{
	// enable debug() calls
	_debug_enabled = false;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

HMC5883::~HMC5883()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
HMC5883::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct mag_report[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the mag topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_mag_topic = orb_advertise(ORB_ID(sensor_mag), &_reports[0]);

	if (_mag_topic < 0)
		debug("failed to create sensor_mag object");

	/* set range */
	set_range(_range_ga);

	ret = OK;
	/* sensor is ok, but not calibrated */
	_sensor_ok = true;
out:
	return ret;
}

int HMC5883::set_range(unsigned range)
{
	uint8_t range_bits;

	if (range < 1) {
		range_bits = 0x00;
		_range_scale = 1.0f / 1370.0f;
		_range_ga = 0.88f;

	} else if (range <= 1) {
		range_bits = 0x01;
		_range_scale = 1.0f / 1090.0f;
		_range_ga = 1.3f;

	} else if (range <= 2) {
		range_bits = 0x02;
		_range_scale = 1.0f / 820.0f;
		_range_ga = 1.9f;

	} else if (range <= 3) {
		range_bits = 0x03;
		_range_scale = 1.0f / 660.0f;
		_range_ga = 2.5f;

	} else if (range <= 4) {
		range_bits = 0x04;
		_range_scale = 1.0f / 440.0f;
		_range_ga = 4.0f;

	} else if (range <= 4.7f) {
		range_bits = 0x05;
		_range_scale = 1.0f / 390.0f;
		_range_ga = 4.7f;

	} else if (range <= 5.6f) {
		range_bits = 0x06;
		_range_scale = 1.0f / 330.0f;
		_range_ga = 5.6f;

	} else {
		range_bits = 0x07;
		_range_scale = 1.0f / 230.0f;
		_range_ga = 8.1f;
	}

	int ret;

	/*
	 * Send the command to set the range
	 */
	ret = write_reg(ADDR_CONF_B, (range_bits << 5));

	if (OK != ret)
		perf_count(_comms_errors);

	uint8_t range_bits_in;
	ret = read_reg(ADDR_CONF_B, range_bits_in);

	if (OK != ret)
		perf_count(_comms_errors);

	return !(range_bits_in == (range_bits << 5));
}

int
HMC5883::probe()
{
	uint8_t data[3] = {0, 0, 0};

	_retries = 10;

	if (read_reg(ADDR_ID_A, data[0]) ||
	    read_reg(ADDR_ID_B, data[1]) ||
	    read_reg(ADDR_ID_C, data[2]))
		debug("read_reg fail");

	_retries = 2;

	if ((data[0] != ID_A_WHO_AM_I) ||
	    (data[1] != ID_B_WHO_AM_I) ||
	    (data[2] != ID_C_WHO_AM_I)) {
		debug("ID byte mismatch (%02x,%02x,%02x)", data[0], data[1], data[2]);
		return -EIO;
	}

	return OK;
}

ssize_t
HMC5883::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_oldest_report != _next_report) {
				memcpy(buffer, _reports + _oldest_report, sizeof(*_reports));
				ret += sizeof(_reports[0]);
				INCREMENT(_oldest_report, _num_reports);
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_oldest_report = _next_report = 0;

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(HMC5883_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		memcpy(buffer, _reports, sizeof(*_reports));
		ret = sizeof(*_reports);

	} while (0);

	return ret;
}

int
HMC5883::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

				/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(HMC5883_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(HMC5883_CONVERSION_INTERVAL))
						return -EINVAL;

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* add one to account for the sentinel in the ring */
			arg++;

			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 2) || (arg > 100))
				return -EINVAL;

			/* allocate new buffer */
			struct mag_report *buf = new struct mag_report[arg];

			if (nullptr == buf)
				return -ENOMEM;

			/* reset the measurement state machine with the new buffer, free the old */
			stop();
			delete[] _reports;
			_num_reports = arg;
			_reports = buf;
			start();

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _num_reports - 1;

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	case MAGIOCSSAMPLERATE:
		/* not supported, always 1 sample per poll */
		return -EINVAL;

	case MAGIOCSRANGE:
		return set_range(arg);

	case MAGIOCSLOWPASS:
		/* not supported, no internal filtering */
		return -EINVAL;

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (mag_scale *)arg, sizeof(_scale));
		/* check calibration, but not actually return an error */
		(void)check_calibration();
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((mag_scale *)arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCCALIBRATE:
		return calibrate(filp, arg);

	case MAGIOCEXSTRAP:
		return set_excitement(arg);

	case MAGIOCSELFTEST:
		return check_calibration();

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

void
HMC5883::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&HMC5883::cycle_trampoline, this, 1);
}

void
HMC5883::stop()
{
	work_cancel(HPWORK, &_work);
}

void
HMC5883::cycle_trampoline(void *arg)
{
	HMC5883 *dev = (HMC5883 *)arg;

	dev->cycle();
}

void
HMC5883::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			log("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(HMC5883_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&HMC5883::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(HMC5883_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure())
		log("measure error");

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&HMC5883::cycle_trampoline,
		   this,
		   USEC2TICK(HMC5883_CONVERSION_INTERVAL));
}

int
HMC5883::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	ret = write_reg(ADDR_MODE, MODE_REG_SINGLE_MODE);

	if (OK != ret)
		perf_count(_comms_errors);

	return ret;
}

int
HMC5883::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		z[2];
		uint8_t		y[2];
	}	hmc_report;
#pragma pack(pop)
	struct {
		int16_t		x, y, z;
	} report;
	int	ret = -EIO;
	uint8_t	cmd;


	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	cmd = ADDR_DATA_OUT_X_MSB;
	ret = transfer(&cmd, 1, (uint8_t *)&hmc_report, sizeof(hmc_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		debug("data/status read error");
		goto out;
	}

	/* swap the data we just received */
	report.x = (((int16_t)hmc_report.x[0]) << 8) + hmc_report.x[1];
	report.y = (((int16_t)hmc_report.y[0]) << 8) + hmc_report.y[1];
	report.z = (((int16_t)hmc_report.z[0]) << 8) + hmc_report.z[1];

	/*
	 * If any of the values are -4096, there was an internal math error in the sensor.
	 * Generalise this to a simple range check that will also catch some bit errors.
	 */
	if ((abs(report.x) > 2048) ||
	    (abs(report.y) > 2048) ||
	    (abs(report.z) > 2048))
		goto out;

	/*
	 * RAW outputs
	 *
	 * to align the sensor axes with the board, x and y need to be flipped
	 * and y needs to be negated
	 */
	_reports[_next_report].x_raw = report.y;
	_reports[_next_report].y_raw = ((report.x == -32768) ? 32767 : -report.x);
	/* z remains z */
	_reports[_next_report].z_raw = report.z;

	/* scale values for output */

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */

#ifdef PX4_I2C_BUS_ONBOARD
	if (_bus == PX4_I2C_BUS_ONBOARD) {
		/* to align the sensor axes with the board, x and y need to be flipped */
		_reports[_next_report].x = ((report.y * _range_scale) - _scale.x_offset) * _scale.x_scale;
		/* flip axes and negate value for y */
		_reports[_next_report].y = ((((report.x == -32768) ? 32767 : -report.x) * _range_scale) - _scale.y_offset) * _scale.y_scale;
		/* z remains z */
		_reports[_next_report].z = ((report.z * _range_scale) - _scale.z_offset) * _scale.z_scale;
	} else {
#endif
		/* XXX axis assignment of external sensor is yet unknown */
		_reports[_next_report].x = ((report.y * _range_scale) - _scale.x_offset) * _scale.x_scale;
		/* flip axes and negate value for y */
		_reports[_next_report].y = ((((report.x == -32768) ? 32767 : -report.x) * _range_scale) - _scale.y_offset) * _scale.y_scale;
		/* z remains z */
		_reports[_next_report].z = ((report.z * _range_scale) - _scale.z_offset) * _scale.z_scale;
#ifdef PX4_I2C_BUS_ONBOARD
	}
#endif

	/* publish it */
	orb_publish(ORB_ID(sensor_mag), _mag_topic, &_reports[_next_report]);

	/* post a report to the ring - note, not locked */
	INCREMENT(_next_report, _num_reports);

	/* if we are running up against the oldest report, toss it */
	if (_next_report == _oldest_report) {
		perf_count(_buffer_overflows);
		INCREMENT(_oldest_report, _num_reports);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int HMC5883::calibrate(struct file *filp, unsigned enable)
{
	struct mag_report report;
	ssize_t sz;
	int ret = 1;

	// XXX do something smarter here
	int fd = (int)enable;

	struct mag_scale mscale_previous = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	struct mag_scale mscale_null = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	float avg_excited[3] = {0.0f, 0.0f, 0.0f};
	unsigned i;

	warnx("starting mag scale calibration");

	/* do a simple demand read */
	sz = read(filp, (char *)&report, sizeof(report));

	if (sz != sizeof(report)) {
		warn("immediate read failed");
		ret = 1;
		goto out;
	}

	warnx("current measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
	warnx("time:        %lld", report.timestamp);
	warnx("sampling 500 samples for scaling offset");

	/* set the queue depth to 10 */
	if (OK != ioctl(filp, SENSORIOCSQUEUEDEPTH, 10)) {
		warn("failed to set queue depth");
		ret = 1;
		goto out;
	}

	/* start the sensor polling at 50 Hz */
	if (OK != ioctl(filp, SENSORIOCSPOLLRATE, 50)) {
		warn("failed to set 2Hz poll rate");
		ret = 1;
		goto out;
	}

	/* Set to 2.5 Gauss */
	if (OK != ioctl(filp, MAGIOCSRANGE, 2)) {
		warnx("failed to set 2.5 Ga range");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCEXSTRAP, 1)) {
		warnx("failed to enable sensor calibration mode");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCGSCALE, (long unsigned int)&mscale_previous)) {
		warn("WARNING: failed to get scale / offsets for mag");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_null)) {
		warn("WARNING: failed to set null scale / offsets for mag");
		ret = 1;
		goto out;
	}

	/* read the sensor 10x and report each value */
	for (i = 0; i < 500; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("timed out waiting for sensor data");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("periodic read failed");
			goto out;

		} else {
			avg_excited[0] += report.x;
			avg_excited[1] += report.y;
			avg_excited[2] += report.z;
		}

		//warnx("periodic read %u", i);
		//warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
	}

	avg_excited[0] /= i;
	avg_excited[1] /= i;
	avg_excited[2] /= i;

	warnx("done. Performed %u reads", i);
	warnx("measurement avg: %.6f  %.6f  %.6f", (double)avg_excited[0], (double)avg_excited[1], (double)avg_excited[2]);

	float scaling[3];

	/* calculate axis scaling */
	scaling[0] = fabsf(1.16f / avg_excited[0]);
	/* second axis inverted */
	scaling[1] = fabsf(1.16f / -avg_excited[1]);
	scaling[2] = fabsf(1.08f / avg_excited[2]);

	warnx("axes scaling: %.6f  %.6f  %.6f", (double)scaling[0], (double)scaling[1], (double)scaling[2]);

	/* set back to normal mode */
	/* Set to 1.1 Gauss */
	if (OK != ::ioctl(fd, MAGIOCSRANGE, 1)) {
		warnx("failed to set 1.1 Ga range");
		goto out;
	}

	if (OK != ::ioctl(fd, MAGIOCEXSTRAP, 0)) {
		warnx("failed to disable sensor calibration mode");
		goto out;
	}

	/* set scaling in device */
	mscale_previous.x_scale = scaling[0];
	mscale_previous.y_scale = scaling[1];
	mscale_previous.z_scale = scaling[2];

	if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_previous)) {
		warn("WARNING: failed to set new scale / offsets for mag");
		goto out;
	}

	ret = OK;

out:

	if (ret == OK) {
		if (!check_scale()) {
			warnx("mag scale calibration successfully finished.");
		} else {
			warnx("mag scale calibration finished with invalid results.");
			ret = ERROR;
		}

	} else {
		warnx("mag scale calibration failed.");
	}

	return ret;
}

int HMC5883::check_scale()
{
	bool scale_valid;

	if ((-FLT_EPSILON + 1.0f < _scale.x_scale && _scale.x_scale < FLT_EPSILON + 1.0f) &&
		(-FLT_EPSILON + 1.0f < _scale.y_scale && _scale.y_scale < FLT_EPSILON + 1.0f) &&
		(-FLT_EPSILON + 1.0f < _scale.z_scale && _scale.z_scale < FLT_EPSILON + 1.0f)) {
		/* scale is one */
		scale_valid = false;
	} else {
		scale_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !scale_valid;
}

int HMC5883::check_offset()
{
	bool offset_valid;

	if ((-2.0f * FLT_EPSILON < _scale.x_offset && _scale.x_offset < 2.0f * FLT_EPSILON) &&
		(-2.0f * FLT_EPSILON < _scale.y_offset && _scale.y_offset < 2.0f * FLT_EPSILON) &&
		(-2.0f * FLT_EPSILON < _scale.z_offset && _scale.z_offset < 2.0f * FLT_EPSILON)) {
		/* offset is zero */
		offset_valid = false;
	} else {
		offset_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !offset_valid;
}

int HMC5883::check_calibration()
{
	bool offset_valid = (check_offset() == OK);
	bool scale_valid  = (check_scale() == OK);

	if (_calibrated != (offset_valid && scale_valid)) {
		warnx("mag cal status changed %s%s", (scale_valid) ? "" : "scale invalid ",
					  (offset_valid) ? "" : "offset invalid");
		_calibrated = (offset_valid && scale_valid);


		// XXX Change advertisement

		/* notify about state change */
		struct subsystem_info_s info = {
			true,
			true,
			_calibrated,
			SUBSYSTEM_TYPE_MAG};
		static orb_advert_t pub = -1;

		if (pub > 0) {
			orb_publish(ORB_ID(subsystem_info), pub, &info);
		} else {
			pub = orb_advertise(ORB_ID(subsystem_info), &info);
		}
	}

	/* return 0 if calibrated, 1 else */
	return (!_calibrated);
}

int HMC5883::set_excitement(unsigned enable)
{
	int ret;
	/* arm the excitement strap */
	uint8_t conf_reg;
	ret = read_reg(ADDR_CONF_A, conf_reg);

	if (OK != ret)
		perf_count(_comms_errors);

	if (((int)enable) < 0) {
		conf_reg |= 0x01;

	} else if (enable > 0) {
		conf_reg |= 0x02;

	} else {
		conf_reg &= ~0x03;
	}

	ret = write_reg(ADDR_CONF_A, conf_reg);

	if (OK != ret)
		perf_count(_comms_errors);

	uint8_t conf_reg_ret;
	read_reg(ADDR_CONF_A, conf_reg_ret);

	return !(conf_reg == conf_reg_ret);
}

int
HMC5883::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t cmd[] = { reg, val };

	return transfer(&cmd[0], 2, nullptr, 0);
}

int
HMC5883::read_reg(uint8_t reg, uint8_t &val)
{
	return transfer(&reg, 1, &val, 1);
}

float
HMC5883::meas_to_float(uint8_t in[2])
{
	union {
		uint8_t	b[2];
		int16_t	w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
HMC5883::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
}

/**
 * Local functions in support of the shell command.
 */
namespace hmc5883
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

HMC5883	*g_dev;

void	start();
void	test();
void	reset();
void	info();
int	calibrate();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		/* if already started, the still command succeeded */
		errx(0, "already started");

	/* create the driver, attempt expansion bus first */
	#if defined(CONFIG_ARCH_BOARD_TMRFC_V1)
	g_dev = new HMC5883(TMR_I2C_BUS_EXPANSION);
	#else
	g_dev = new HMC5883(PX4_I2C_BUS_EXPANSION);
	#endif
	if (g_dev != nullptr && OK != g_dev->init()) {
		delete g_dev;
		g_dev = nullptr;
	}

#if defined(CONFIG_ARCH_BOARD_TMRFC_V1)

    #ifdef TMR_I2C_BUS_ONBOARD
	/* if this failed, attempt onboard sensor */
	if (g_dev == nullptr) {
		g_dev = new HMC5883(TMR_I2C_BUS_ONBOARD);
		if (g_dev != nullptr && OK != g_dev->init()) {
			goto fail;
		}
	}
    #endif

#else

    #ifdef PX4_I2C_BUS_ONBOARD
	/* if this failed, attempt onboard sensor */
	if (g_dev == nullptr) {
		g_dev = new HMC5883(PX4_I2C_BUS_ONBOARD);
		if (g_dev != nullptr && OK != g_dev->init()) {
			goto fail;
		}
	}
    #endif

#endif


	if (g_dev == nullptr)
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(MAG_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct mag_report report;
	ssize_t sz;
	int ret;

	int fd = open(MAG_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'hmc5883 start' if the driver is not running", MAG_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10))
		errx(1, "failed to set queue depth");

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("periodic read %u", i);
		warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}


/**
 * Automatic scale calibration.
 *
 * Basic idea:
 *
 *   output = (ext field +- 1.1 Ga self-test) * scale factor
 *
 * and consequently:
 *
 *   1.1 Ga = (excited - normal) * scale factor
 *   scale factor = (excited - normal) / 1.1 Ga
 *
 *   sxy = (excited - normal) / 766	| for conf reg. B set to 0x60 / Gain = 3
 *   sz  = (excited - normal) / 713	| for conf reg. B set to 0x60 / Gain = 3
 *
 * By subtracting the non-excited measurement the pure 1.1 Ga reading
 * can be extracted and the sensitivity of all axes can be matched.
 *
 * SELF TEST OPERATION
 * To check the HMC5883L for proper operation, a self test feature in incorporated
 * in which the sensor offset straps are excited to create a nominal field strength
 * (bias field) to be measured. To implement self test, the least significant bits
 * (MS1 and MS0) of configuration register A are changed from 00 to 01 (positive bias)
 * or 10 (negetive bias), e.g. 0x11 or 0x12.
 * Then, by placing the mode register into single-measurement mode (0x01),
 * two data acquisition cycles will be made on each magnetic vector.
 * The first acquisition will be a set pulse followed shortly by measurement
 * data of the external field. The second acquisition will have the offset strap
 * excited (about 10 mA) in the positive bias mode for X, Y, and Z axes to create
 * about a ±1.1 gauss self test field plus the external field. The first acquisition
 * values will be subtracted from the second acquisition, and the net measurement
 * will be placed into the data output registers.
 * Since self test adds ~1.1 Gauss additional field to the existing field strength,
 * using a reduced gain setting prevents sensor from being saturated and data registers
 * overflowed. For example, if the configuration register B is set to 0x60 (Gain=3),
 * values around +766 LSB (1.16 Ga * 660 LSB/Ga) will be placed in the X and Y data
 * output registers and around +713 (1.08 Ga * 660 LSB/Ga) will be placed in Z data
 * output register. To leave the self test mode, change MS1 and MS0 bit of the
 * configuration register A back to 00 (Normal Measurement Mode), e.g. 0x10.
 * Using the self test method described above, the user can scale sensor
 */
int calibrate()
{
	int ret;

	int fd = open(MAG_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'hmc5883 start' if the driver is not running", MAG_DEVICE_PATH);

	if (OK != (ret = ioctl(fd, MAGIOCCALIBRATE, fd))) {
		warnx("failed to enable sensor calibration mode");
	}

	close(fd);

	if (ret == OK) {
		errx(0, "PASS");

	} else {
		errx(1, "FAIL");
	}
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(MAG_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int
hmc5883_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		hmc5883::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		hmc5883::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		hmc5883::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status"))
		hmc5883::info();

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(argv[1], "calibrate")) {
		if (hmc5883::calibrate() == 0) {
			errx(0, "calibration successful");

		} else {
			errx(1, "calibration failed");
		}
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' 'calibrate' or 'info'");
}
