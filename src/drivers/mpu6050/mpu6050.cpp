/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 TMR Development Team. All rights reserved.
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
 * @file mpu6050.cpp
 *
 * Driver for the Invensense MPU6050 connected via I2C.
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <debug.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include "mpu6050.h"

#define MPU6050_ADDRESS        TMR_I2C_OBDEV_MPU6050

// MPU 6050 registers
#define MPUREG_WHOAMI           0x75
#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
#define MPUREG_FIFO_EN          0x23
#define MPUREG_INT_PIN_CFG      0x37
#define MPUREG_INT_ENABLE       0x38
#define MPUREG_INT_STATUS       0x3A
#define MPUREG_ACCEL_XOUT_H     0x3B
#define MPUREG_ACCEL_XOUT_L     0x3C
#define MPUREG_ACCEL_YOUT_H     0x3D
#define MPUREG_ACCEL_YOUT_L     0x3E
#define MPUREG_ACCEL_ZOUT_H     0x3F
#define MPUREG_ACCEL_ZOUT_L     0x40
#define MPUREG_TEMP_OUT_H       0x41
#define MPUREG_TEMP_OUT_L       0x42
#define MPUREG_GYRO_XOUT_H      0x43
#define MPUREG_GYRO_XOUT_L      0x44
#define MPUREG_GYRO_YOUT_H      0x45
#define MPUREG_GYRO_YOUT_L      0x46
#define MPUREG_GYRO_ZOUT_H      0x47
#define MPUREG_GYRO_ZOUT_L      0x48
#define MPUREG_USER_CTRL        0x6A
#define MPUREG_PWR_MGMT_1       0x6B
#define MPUREG_PWR_MGMT_2       0x6C
#define MPUREG_FIFO_COUNTH      0x72
#define MPUREG_FIFO_COUNTL      0x73
#define MPUREG_FIFO_R_W         0x74
#define MPUREG_WHO_AM_I         0x75
#define MPUREG_PRODUCT_ID       0x0C

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01

// Product ID Description for MPU6000
// high 4 bits  low 4 bits
// Product Name Product Revision
#define MPU6000ES_REV_C4        0x14
#define MPU6000ES_REV_C5        0x15
#define MPU6000ES_REV_D6        0x16
#define MPU6000ES_REV_D7        0x17
#define MPU6000ES_REV_D8        0x18
#define MPU6000_REV_C4          0x54
#define MPU6000_REV_C5          0x55
#define MPU6000_REV_D6          0x56
#define MPU6000_REV_D7          0x57
#define MPU6000_REV_D8          0x58
#define MPU6000_REV_D9          0x59
#define MPU6000_REV_D10         0x5A

#define MPU6050_WHO_AM_I        0x68

#define MPU6050_ACCEL_DEFAULT_RANGE_G                8
#define MPU6050_ACCEL_DEFAULT_RATE                1000
#define MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ    30

#define MPU6050_GYRO_DEFAULT_RANGE_G                 8
#define MPU6050_GYRO_DEFAULT_RATE                 1000
#define MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ     30

#define MPU6050_DEFAULT_ONCHIP_FILTER_FREQ          42

#define MPU6050_ONE_G                         9.80665f

class MPU6050_gyro;

class MPU6050 : public device::I2C
{
public:
    MPU6050(int bus);
    virtual ~MPU6050();

    virtual int     init();

    virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
    virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

    /**
     * Diagnostics - print some basic information about the driver.
     */
    void            print_info();

protected:
    virtual int     probe();

    friend class MPU6050_gyro;

    virtual ssize_t     gyro_read(struct file *filp, char *buffer, size_t buflen);
    virtual int     gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
    MPU6050_gyro        *_gyro;
    uint8_t         _product;   /** product code */

    struct hrt_call     _call;
    unsigned        _call_interval;
    work_s          _work;

	RingBuffer		*_accel_reports;

    struct accel_scale  _accel_scale;
    float           _accel_range_scale;
    float           _accel_range_m_s2;
    orb_advert_t        _accel_topic;

	RingBuffer		*_gyro_reports;

    struct gyro_scale   _gyro_scale;
    float           _gyro_range_scale;
    float           _gyro_range_rad_s;
    orb_advert_t        _gyro_topic;

    unsigned        _reads;
    unsigned        _sample_rate;
    perf_counter_t      _sample_perf;

    math::LowPassFilter2p   _accel_filter_x;
    math::LowPassFilter2p   _accel_filter_y;
    math::LowPassFilter2p   _accel_filter_z;
    math::LowPassFilter2p   _gyro_filter_x;
    math::LowPassFilter2p   _gyro_filter_y;
    math::LowPassFilter2p   _gyro_filter_z;

    /**
     * Start automatic measurement.
     */
    void            start();

    /**
     * Stop automatic measurement.
     */
    void            stop();

    /**
     * Reset chip.
     *
     * Resets the chip and measurements ranges, but not scale and offset.
     */
    void            reset();

    /**
     * Static trampoline from the hrt_call context; because we don't have a
     * generic hrt wrapper yet.
     *
     * Called by the HRT in interrupt context at the specified rate if
     * automatic polling is enabled.
     *
     * @param arg       Instance pointer for the driver that is polling.
     */
    static void     measure_trampoline(void *arg);

    /**
     * Fetch measurements from the sensor and update the report buffers.
     */
    void            measure();

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
    void            cycle();

    /**
     * Static trampoline from the workq context; because we don't have a
     * generic workq wrapper yet.
     *
     * @param arg       Instance pointer for the driver that is polling.
     */
    static void     cycle_trampoline(void *arg);

    /**
     * Collect the result of the most recent measurement.
     */
    int             collect();

    /**
     * Read a register from the MPU6050
     *
     * @param       The register to read.
     * @return      The value that was read.
     */
    uint8_t         read_reg(uint8_t reg);

    /**
     * Write a register in the MPU6050
     *
     * @param reg       The register to write.
     * @param value     The new value to write.
     */
    void            write_reg(uint8_t reg, uint8_t value);

    /**
     * Modify a register in the MPU6050
     *
     * Bits are cleared before bits are set.
     *
     * @param reg       The register to modify.
     * @param clearbits Bits in the register to clear.
     * @param setbits   Bits in the register to set.
     */
    void            modify_reg(uint8_t reg, uint8_t clearbits, uint8_t setbits);

    /**
     * Set the MPU6050 measurement range.
     *
     * @param max_g     The maximum G value the range must support.
     * @return      OK if the value can be supported, -ERANGE otherwise.
     */
    int         set_range(unsigned max_g);

    /**
     * Swap a 16-bit value read from the MPU6050 to native byte order.
     */
    uint16_t        swap16(uint16_t val) { return (val >> 8) | (val << 8);  }

    /**
     * Self test
     *
     * @return 0 on success, 1 on failure
     */
     int            self_test();

    /**
     * Accel self test
     *
     * @return 0 on success, 1 on failure
     */
    int             accel_self_test();

    /**
     * Gyro self test
     *
     * @return 0 on success, 1 on failure
     */
     int            gyro_self_test();

    /*
      set low pass filter frequency
     */
    void _set_dlpf_filter(uint16_t frequency_hz);

    /*
      set sample rate (approximate) - 1kHz to 5Hz
    */
    void _set_sample_rate(uint16_t desired_sample_rate_hz);

};

/**
 * Helper class implementing the gyro driver node.
 */
class MPU6050_gyro : public device::CDev
{
public:
    MPU6050_gyro(MPU6050 *parent);
    ~MPU6050_gyro();

    virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
    virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

protected:
    friend class MPU6050;

    void            parent_poll_notify();
private:
    MPU6050         *_parent;
};

/** driver 'main' command */
extern "C" { __EXPORT int mpu6050_main(int argc, char *argv[]); }

MPU6050::MPU6050(int bus) :
    I2C("MPU6050", ACCEL_DEVICE_PATH, bus, MPU6050_ADDRESS, 400000),
    _gyro(new MPU6050_gyro(this)),
    _product(0),
    _call_interval(0),
    _accel_reports(nullptr),
    _accel_range_scale(0.0f),
    _accel_range_m_s2(0.0f),
    _accel_topic(-1),
    _gyro_reports(nullptr),
    _gyro_range_scale(0.0f),
    _gyro_range_rad_s(0.0f),
    _gyro_topic(-1),
    _reads(0),
    _sample_rate(1000),//MPU6050_GYRO_DEFAULT_RATE
    _sample_perf(perf_alloc(PC_ELAPSED, "mpu6050_read")),
    _accel_filter_x(MPU6050_ACCEL_DEFAULT_RATE, MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_y(MPU6050_ACCEL_DEFAULT_RATE, MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_z(MPU6050_ACCEL_DEFAULT_RATE, MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_x(MPU6050_GYRO_DEFAULT_RATE, MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_y(MPU6050_GYRO_DEFAULT_RATE, MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_z(MPU6050_GYRO_DEFAULT_RATE, MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ)
{
    // disable debug() calls
    _debug_enabled = false;

    // default accel scale factors
    _accel_scale.x_offset = 0;
    _accel_scale.x_scale  = 1.0f;
    _accel_scale.y_offset = 0;
    _accel_scale.y_scale  = 1.0f;
    _accel_scale.z_offset = 0;
    _accel_scale.z_scale  = 1.0f;

    // default gyro scale factors
    _gyro_scale.x_offset = 0;
    _gyro_scale.x_scale  = 1.0f;
    _gyro_scale.y_offset = 0;
    _gyro_scale.y_scale  = 1.0f;
    _gyro_scale.z_offset = 0;
    _gyro_scale.z_scale  = 1.0f;

    memset(&_call, 0, sizeof(_call));
    memset(&_work, 0, sizeof(_work));
}

MPU6050::~MPU6050()
{
    /* make sure we are truly inactive */
    stop();

    /* delete the gyro subdriver */
    delete _gyro;

    /* free any existing reports */
    if (_accel_reports != nullptr)
        delete _accel_reports;
    if (_gyro_reports != nullptr)
        delete _gyro_reports;

    /* delete the perf counter */
    perf_free(_sample_perf);
}

int
MPU6050::init()
{
    int ret;
    int gyro_ret;

    /* do I2C init (and probe) first */
    ret = I2C::init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        debug("I2C setup failed");
        return ret;
    }

    /* allocate basic report buffers */
	_accel_reports = new RingBuffer(2, sizeof(accel_report));
    if (_accel_reports == nullptr)
        goto out;

	_gyro_reports = new RingBuffer(2, sizeof(gyro_report));
    if (_gyro_reports == nullptr)
        goto out;

    reset();

    /* Initialize offsets and scales */
    _accel_scale.x_offset = 0;
    _accel_scale.x_scale  = 1.0f;
    _accel_scale.y_offset = 0;
    _accel_scale.y_scale  = 1.0f;
    _accel_scale.z_offset = 0;
    _accel_scale.z_scale  = 1.0f;

    _gyro_scale.x_offset = 0;
    _gyro_scale.x_scale  = 1.0f;
    _gyro_scale.y_offset = 0;
    _gyro_scale.y_scale  = 1.0f;
    _gyro_scale.z_offset = 0;
    _gyro_scale.z_scale  = 1.0f;

    /* do CDev init for the gyro device node, keep it optional */
    gyro_ret = _gyro->init();

    /* fetch an initial set of measurements for advertisement */
    measure();

    if (gyro_ret != OK) {
        _gyro_topic = -1;
    } else {
        gyro_report gr;
		_gyro_reports->get(&gr);

        _gyro_topic = orb_advertise(ORB_ID(sensor_gyro), &gr);
    }

    /* advertise accel topic */
    accel_report ar;
	_accel_reports->get(&ar);
    _accel_topic = orb_advertise(ORB_ID(sensor_accel), &ar);

out:
    return ret;
}

void MPU6050::reset()
{

    // Chip reset
    write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    up_udelay(10000);

    // Wake up device and select GyroZ clock (better performance)
    write_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    up_udelay(1000);

    // SAMPLE RATE
    _set_sample_rate(_sample_rate); // default sample rate = 200Hz
    usleep(1000);

    // FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)
    // was 90 Hz, but this ruins quality and does not improve the
    // system response
    _set_dlpf_filter(MPU6050_DEFAULT_ONCHIP_FILTER_FREQ);
    usleep(1000);
    // Gyro scale 2000 deg/s ()
    write_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
    usleep(1000);

    // correct gyro scale factors
    // scale to rad/s in SI units
    // 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
    // scaling factor:
    // 1/(2^15)*(2000/180)*PI
    _gyro_range_scale = (0.0174532 / 16.4);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
    _gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;

    // product-specific scaling
    switch (_product) {
    case MPU6000ES_REV_C4:
    case MPU6000ES_REV_C5:
    case MPU6000_REV_C4:
    case MPU6000_REV_C5:
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        write_reg(MPUREG_ACCEL_CONFIG, 1 << 3);
        break;

    case MPU6000ES_REV_D6:
    case MPU6000ES_REV_D7:
    case MPU6000ES_REV_D8:
    case MPU6000_REV_D6:
    case MPU6000_REV_D7:
    case MPU6000_REV_D8:
    case MPU6000_REV_D9:
    case MPU6000_REV_D10:
    // default case to cope with new chip revisions, which
    // presumably won't have the accel scaling bug      
    default:
        // Accel scale 8g (4096 LSB/g)
        write_reg(MPUREG_ACCEL_CONFIG, 2 << 3);
        break;
    }

    // Correct accel scale factors of 4096 LSB/g
    // scale to m/s^2 ( 1g = 9.81 m/s^2)
    _accel_range_scale = (MPU6050_ONE_G / 4096.0f);
    _accel_range_m_s2 = 8.0f * MPU6050_ONE_G;

    usleep(1000);

    // INT CFG => Interrupt on Data Ready
    write_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready
    usleep(1000);
    write_reg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR | (1 << MPU6050_INTCFG_I2C_BYPASS_EN_BIT)); // INT: Clear on any read
    usleep(1000);

    // Oscillator set
    // write_reg(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
    usleep(1000);

}

int
MPU6050::probe()
{
    /* look for a product ID we recognise */
    _product = read_reg(MPUREG_WHO_AM_I);

    if(_product == MPU6050_WHO_AM_I)
    {
        return OK;
    }

    debug("unexpected WHO_AM_I 0x%02x", _product);
    return -EIO;

}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
MPU6050::_set_sample_rate(uint16_t desired_sample_rate_hz)
{
  uint8_t div = 1000 / desired_sample_rate_hz;
  if(div>200) div=200;
  if(div<1) div=1;
  write_reg(MPUREG_SMPLRT_DIV, div-1);
  _sample_rate = 1000 / div;
}

/*
  set the DLPF filter frequency. This affects both accel and gyro.
 */
void
MPU6050::_set_dlpf_filter(uint16_t frequency_hz)
{
    uint8_t filter;

    /* 
       choose next highest filter frequency available
     */
    if (frequency_hz <= 5) {
        filter = BITS_DLPF_CFG_5HZ;
    } else if (frequency_hz <= 10) {
        filter = BITS_DLPF_CFG_10HZ;
    } else if (frequency_hz <= 20) {
        filter = BITS_DLPF_CFG_20HZ;
    } else if (frequency_hz <= 42) {
        filter = BITS_DLPF_CFG_42HZ;
    } else if (frequency_hz <= 98) {
        filter = BITS_DLPF_CFG_98HZ;
    } else if (frequency_hz <= 188) {
        filter = BITS_DLPF_CFG_188HZ;
    } else if (frequency_hz <= 256) {
        filter = BITS_DLPF_CFG_256HZ_NOLPF2;
    } else {
        filter = BITS_DLPF_CFG_2100HZ_NOLPF;
    }
    write_reg(MPUREG_CONFIG, filter);
}

ssize_t
MPU6050::read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(accel_report);

    /* buffer must be large enough */
    if (count < 1)
        return -ENOSPC;

    /* if automatic measurement is not enabled, get a fresh measurement into the buffer */
    if (_call_interval == 0) {
        _accel_reports->flush();
        measure();
    }

    /* if no data, error (we could block here) */
    if (_accel_reports->empty())
        return -EAGAIN;

    /* copy reports out of our buffer to the caller */
    accel_report *arp = reinterpret_cast<accel_report *>(buffer);
    int transferred = 0;
    while (count--) {
		if (!_accel_reports->get(arp))
            break;
        transferred++;
		arp++;
    }

    /* return the number of bytes transferred */
    return (transferred * sizeof(accel_report));
}

int
MPU6050::self_test()
{
    if (_reads == 0) {
        measure();
    }

    /* return 0 on success, 1 else */
    return (_reads > 0) ? 0 : 1;
}

int
MPU6050::accel_self_test()
{
    if (self_test())
        return 1;

    /* inspect accel offsets */
    if (fabsf(_accel_scale.x_offset) < 0.000001f)
        return 1;
    if (fabsf(_accel_scale.x_scale - 1.0f) > 0.4f || fabsf(_accel_scale.x_scale - 1.0f) < 0.000001f)
        return 1;

    if (fabsf(_accel_scale.y_offset) < 0.000001f)
        return 1;
    if (fabsf(_accel_scale.y_scale - 1.0f) > 0.4f || fabsf(_accel_scale.y_scale - 1.0f) < 0.000001f)
        return 1;

    if (fabsf(_accel_scale.z_offset) < 0.000001f)
        return 1;
    if (fabsf(_accel_scale.z_scale - 1.0f) > 0.4f || fabsf(_accel_scale.z_scale - 1.0f) < 0.000001f)
        return 1;

    return 0;
}

int
MPU6050::gyro_self_test()
{
    if (self_test())
        return 1;

    /* evaluate gyro offsets, complain if offset -> zero or larger than 6 dps */
    if (fabsf(_gyro_scale.x_offset) > 0.1f || fabsf(_gyro_scale.x_offset) < 0.000001f)
        return 1;
    if (fabsf(_gyro_scale.x_scale - 1.0f) > 0.3f)
        return 1;

    if (fabsf(_gyro_scale.y_offset) > 0.1f || fabsf(_gyro_scale.y_offset) < 0.000001f)
        return 1;
    if (fabsf(_gyro_scale.y_scale - 1.0f) > 0.3f)
        return 1;

    if (fabsf(_gyro_scale.z_offset) > 0.1f || fabsf(_gyro_scale.z_offset) < 0.000001f)
        return 1;
    if (fabsf(_gyro_scale.z_scale - 1.0f) > 0.3f)
        return 1;

    return 0;
}

ssize_t
MPU6050::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(gyro_report);

    /* buffer must be large enough */
    if (count < 1)
        return -ENOSPC;

    /* if automatic measurement is not enabled, get a fresh measurement into the buffer */
    if (_call_interval == 0) {
        _gyro_reports->flush();
        measure();
    }

    /* if no data, error (we could block here) */
    if (_gyro_reports->empty())
        return -EAGAIN;

    /* copy reports out of our buffer to the caller */
	gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
    int transferred = 0;
    while (count--) {
		if (!_gyro_reports->get(grp))
            break;
        transferred++;
		grp++;
    }

    /* return the number of bytes transferred */
    return (transferred * sizeof(gyro_report));
}

int
MPU6050::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

    case SENSORIOCRESET:
        reset();
        return OK;

    case SENSORIOCSPOLLRATE: {
            switch (arg) {

                /* switching to manual polling */
            case SENSOR_POLLRATE_MANUAL:
                stop();
                _call_interval = 0;
                return OK;

                /* external signalling not supported */
            case SENSOR_POLLRATE_EXTERNAL:

                /* zero would be bad */
            case 0:
                return -EINVAL;

                /* set default/max polling rate */
            case SENSOR_POLLRATE_MAX:
                return ioctl(filp, SENSORIOCSPOLLRATE, 1000);

            case SENSOR_POLLRATE_DEFAULT:
                return ioctl(filp, SENSORIOCSPOLLRATE, MPU6050_ACCEL_DEFAULT_RATE);

                /* adjust to a legal polling interval in Hz */
            default: {
                    /* do we need to start internal polling? */
                    bool want_start = (_call_interval == 0);

                    /* convert hz to hrt interval via microseconds */
                    unsigned ticks = 1000000 / arg;

                    /* check against maximum sane rate */
                    if (ticks < 1000)
                        return -EINVAL;

                    // adjust filters
                    float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
                    float sample_rate = 1.0e6f/ticks;
                    _accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
                    _accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
                    _accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


                    float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
                    _gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
                    _gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
                    _gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

                    /* update interval for next measurement */
                    /* XXX this is a bit shady, but no other way to adjust... */
                    _call.period = _call_interval = ticks;

                    /* if we need to start the poll state machine, do it */
                    if (want_start)
                        start();

                    return OK;
                }
            }
        }

    case SENSORIOCGPOLLRATE:
        if (_call_interval == 0)
            return SENSOR_POLLRATE_MANUAL;

        return 1000000 / _call_interval;

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100))
                return -EINVAL;
		
		irqstate_t flags = irqsave();
		if (!_accel_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);
		
            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _accel_reports->size();

    case ACCELIOCGSAMPLERATE:
        return _sample_rate;

    case ACCELIOCSSAMPLERATE:
        _set_sample_rate(arg);
        return OK;

    case ACCELIOCGLOWPASS:
        return _accel_filter_x.get_cutoff_freq();

    case ACCELIOCSLOWPASS:
        
        // XXX decide on relationship of both filters
        // i.e. disable the on-chip filter
        //_set_dlpf_filter((uint16_t)arg);
                _accel_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
                _accel_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
                _accel_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        return OK;

    case ACCELIOCSSCALE:
        {
            /* copy scale, but only if off by a few percent */
            struct accel_scale *s = (struct accel_scale *) arg;
            float sum = s->x_scale + s->y_scale + s->z_scale;
            if (sum > 2.0f && sum < 4.0f) {
                memcpy(&_accel_scale, s, sizeof(_accel_scale));
                return OK;
            } else {
                return -EINVAL;
            }
        }

    case ACCELIOCGSCALE:
        /* copy scale out */
        memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
        return OK;

    case ACCELIOCSRANGE:
        /* XXX not implemented */
        // XXX change these two values on set:
        // _accel_range_scale = (9.81f / 4096.0f);
        // _accel_range_m_s2 = 8.0f * 9.81f;
        return -EINVAL;
    case ACCELIOCGRANGE:
        return (unsigned long)((_accel_range_m_s2)/MPU6050_ONE_G + 0.5f);

    case ACCELIOCSELFTEST:
        return accel_self_test();

    default:
        /* give it to the superclass */
        return I2C::ioctl(filp, cmd, arg);
    }
}

int
MPU6050::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

        /* these are shared with the accel side */
    case SENSORIOCSPOLLRATE:
    case SENSORIOCGPOLLRATE:
    case SENSORIOCRESET:
        return ioctl(filp, cmd, arg);

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!_gyro_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);

            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _gyro_reports->size();

    case GYROIOCGSAMPLERATE:
        return _sample_rate;

    case GYROIOCSSAMPLERATE:
        _set_sample_rate(arg);
        return OK;

    case GYROIOCGLOWPASS:
        return _gyro_filter_x.get_cutoff_freq();
    case GYROIOCSLOWPASS:
        _gyro_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        _gyro_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        _gyro_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        // XXX check relation to the internal lowpass
        //_set_dlpf_filter((uint16_t)arg);
        return OK;

    case GYROIOCSSCALE:
        /* copy scale in */
        memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
        return OK;

    case GYROIOCGSCALE:
        /* copy scale out */
        memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
        return OK;

    case GYROIOCSRANGE:
        /* XXX not implemented */
        // XXX change these two values on set:
        // _gyro_range_scale = xx
        // _gyro_range_rad_s = xx
        return -EINVAL;
    case GYROIOCGRANGE:
        return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

    case GYROIOCSELFTEST:
        return gyro_self_test();

    default:
        /* give it to the superclass */
        return I2C::ioctl(filp, cmd, arg);
    }
}

uint8_t
MPU6050::read_reg(uint8_t reg)
{
    uint8_t val;

    transfer(&reg, 1, &val, 1);
    return val;
}

void
MPU6050::write_reg(uint8_t reg, uint8_t value)
{
    uint8_t cmd[] = { reg, value };

    transfer(&cmd[0], 2, nullptr, 0);
}

void
MPU6050::modify_reg(uint8_t reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t val;

    val = read_reg(reg);
    val &= ~clearbits;
    val |= setbits;
    write_reg(reg, val);
}

int
MPU6050::set_range(unsigned max_g)
{
#if 0
    uint8_t rangebits;
    float rangescale;

    if (max_g > 16) {
        return -ERANGE;

    } else if (max_g > 8) {     /* 16G */
        rangebits = OFFSET_LSB1_RANGE_16G;
        rangescale = 1.98;

    } else if (max_g > 4) {     /* 8G */
        rangebits = OFFSET_LSB1_RANGE_8G;
        rangescale = 0.99;

    } else if (max_g > 3) {     /* 4G */
        rangebits = OFFSET_LSB1_RANGE_4G;
        rangescale = 0.5;

    } else if (max_g > 2) {     /* 3G */
        rangebits = OFFSET_LSB1_RANGE_3G;
        rangescale = 0.38;

    } else if (max_g > 1) {     /* 2G */
        rangebits = OFFSET_LSB1_RANGE_2G;
        rangescale = 0.25;

    } else {            /* 1G */
        rangebits = OFFSET_LSB1_RANGE_1G;
        rangescale = 0.13;
    }

    /* adjust sensor configuration */
    modify_reg(ADDR_OFFSET_LSB1, OFFSET_LSB1_RANGE_MASK, rangebits);
    _range_scale = rangescale;
#endif
    return OK;
}

void
MPU6050::start()
{
    /* make sure we are stopped first */
    //stop();

    /* discard any stale data in the buffers */
    _accel_reports->flush();
    _gyro_reports->flush();

    #if 0
    /* start polling at the specified rate */
    hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&MPU6050::measure_trampoline, this);
    #else
    /* schedule a cycle to start things */
    work_queue(HPWORK, &_work, (worker_t)&MPU6050::cycle_trampoline, this, USEC2TICK(_call_interval));
    #endif
}

void
MPU6050::stop()
{
    #if 0
    hrt_cancel(&_call);
    #else
    work_cancel(HPWORK, &_work);
    #endif
}

void
MPU6050::measure_trampoline(void *arg)
{
    MPU6050 *dev = reinterpret_cast<MPU6050 *>(arg);

    /* make another measurement */
    dev->measure();
}

void
MPU6050::measure()
{
#pragma pack(push, 1)
    /**
     * Report conversation within the MPU6050, including command byte and
     * interrupt status.
     */
    struct MPUReport {
        uint8_t     cmd;
        uint8_t     status;
        uint8_t     accel_x[2];
        uint8_t     accel_y[2];
        uint8_t     accel_z[2];
        uint8_t     temp[2];
        uint8_t     gyro_x[2];
        uint8_t     gyro_y[2];
        uint8_t     gyro_z[2];
    } mpu_report;
#pragma pack(pop)

    struct Report {
        int16_t     accel_x;
        int16_t     accel_y;
        int16_t     accel_z;
        int16_t     temp;
        int16_t     gyro_x;
        int16_t     gyro_y;
        int16_t     gyro_z;
    } report;

    /* start measuring */
    perf_begin(_sample_perf);

    /*
     * Fetch the full set of measurements from the MPU6050 in one pass.
     */
    mpu_report.cmd = MPUREG_INT_STATUS;
    if (OK != transfer((uint8_t *)&mpu_report, 1, ((uint8_t *)&(mpu_report.status)), (sizeof(mpu_report) - 1 )))
        return;

    /* count measurement */
    _reads++;
#if 0
    /*
     * Convert from big to little endian
     */
    report.accel_x = int16_t_from_bytes(mpu_report.accel_y);
    report.accel_y = int16_t_from_bytes(mpu_report.accel_x);
    //report.accel_z = (-1) * (int16_t_from_bytes(mpu_report.accel_z));
    report.accel_z = ~(int16_t_from_bytes(mpu_report.accel_z)) + 1;

    report.temp = int16_t_from_bytes(mpu_report.temp);

    report.gyro_x = int16_t_from_bytes(mpu_report.gyro_y);
    report.gyro_y = int16_t_from_bytes(mpu_report.gyro_x);
    //report.gyro_z = (-1) * (int16_t_from_bytes(mpu_report.gyro_z));
    report.gyro_z = ~(int16_t_from_bytes(mpu_report.gyro_z)) + 1;
#else
    /*
     * Convert from big to little endian
     */
    report.accel_x = int16_t_from_bytes(mpu_report.accel_x);
    report.accel_y = int16_t_from_bytes(mpu_report.accel_y);
    report.accel_z = int16_t_from_bytes(mpu_report.accel_z);

    report.temp = int16_t_from_bytes(mpu_report.temp);

    report.gyro_x = int16_t_from_bytes(mpu_report.gyro_x);
    report.gyro_y = int16_t_from_bytes(mpu_report.gyro_y);
    report.gyro_z = int16_t_from_bytes(mpu_report.gyro_z);

    /*
     * Swap x,y axe and negate x,z
     */
    int16_t accel_xt = report.accel_y;
    int16_t accel_yt = report.accel_x;
    int16_t accel_zt = ((report.accel_z == -32768) ? 32767 : -report.accel_z);

    int16_t gyro_xt = report.gyro_y;
    int16_t gyro_yt = report.gyro_x;
    int16_t gyro_zt = ((report.gyro_z == -32768) ? 32767 : -report.gyro_z);

    /*
     * Apply the swap
     */
    report.accel_x = accel_xt;
    report.accel_y = accel_yt;
    report.accel_z = accel_zt;

    report.gyro_x = gyro_xt;
    report.gyro_y = gyro_yt;
    report.gyro_z = gyro_zt;
	
#endif

#if 0
    /*
     * Swap x,y axe and negate x,z
     */
    //int16_t accel_xt = report.accel_y;
    //int16_t accel_yt = ((report.accel_x == -32768) ? 32767 : -report.accel_x);
    int16_t accel_zt = ((report.accel_z == -32768) ? 32767 : -report.accel_z);

    //int16_t gyro_xt = report.gyro_y;
    //int16_t gyro_yt = ((report.gyro_x == -32768) ? 32767 : -report.gyro_x);
    int16_t gyro_zt = ((report.gyro_z == -32768) ? 32767 : -report.gyro_z);

    /*
     * Apply the swap
     */
    //report.accel_x = accel_xt;
    //report.accel_y = accel_yt;
    report.accel_z = accel_zt;

    //report.gyro_x = gyro_xt;
    //report.gyro_y = gyro_yt;
    report.gyro_z = gyro_zt;
#endif

    /*
     * Report buffers.
     */
    accel_report        arb;
    gyro_report         grb;

    /*
     * Adjust and scale results to m/s^2.
     */
    grb.timestamp = arb.timestamp = hrt_absolute_time();


    /*
     * 1) Scale raw value to SI units using scaling from datasheet.
     * 2) Subtract static offset (in SI units)
     * 3) Scale the statically calibrated values with a linear
     *    dynamically obtained factor
     *
     * Note: the static sensor offset is the number the sensor outputs
     *   at a nominally 'zero' input. Therefore the offset has to
     *   be subtracted.
     *
     *   Example: A gyro outputs a value of 74 at zero angular rate
     *        the offset is 74 from the origin and subtracting
     *        74 from all measurements centers them around zero.
     */


    /* NOTE: Axes have been swapped to match the board a few lines above. */

    arb.x_raw = report.accel_x;
    arb.y_raw = report.accel_y;
    arb.z_raw = report.accel_z;

    float x_in_new = ((report.accel_x * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
    float y_in_new = ((report.accel_y * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
    float z_in_new = ((report.accel_z * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;
    
    arb.x = _accel_filter_x.apply(x_in_new);
    arb.y = _accel_filter_y.apply(y_in_new);
    arb.z = _accel_filter_z.apply(z_in_new);

    arb.scaling = _accel_range_scale;
    arb.range_m_s2 = _accel_range_m_s2;

    arb.temperature_raw = report.temp;
    arb.temperature = (report.temp) / 361.0f + 35.0f;

    grb.x_raw = report.gyro_x;
    grb.y_raw = report.gyro_y;
    grb.z_raw = report.gyro_z;

    float x_gyro_in_new = ((report.gyro_x * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
    float y_gyro_in_new = ((report.gyro_y * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
    float z_gyro_in_new = ((report.gyro_z * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;
    
    grb.x = _gyro_filter_x.apply(x_gyro_in_new);
    grb.y = _gyro_filter_y.apply(y_gyro_in_new);
    grb.z = _gyro_filter_z.apply(z_gyro_in_new);

    grb.scaling = _gyro_range_scale;
    grb.range_rad_s = _gyro_range_rad_s;

    grb.temperature_raw = report.temp;
    grb.temperature = (report.temp) / 361.0f + 35.0f;

	_accel_reports->force(&arb);
	_gyro_reports->force(&grb);

	/* notify anyone waiting for data */
    poll_notify(POLLIN);
    _gyro->parent_poll_notify();

    /* and publish for subscribers */
    orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
    if (_gyro_topic != -1) {
        orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &grb);
    }

    /* stop measuring */
    perf_end(_sample_perf);
}

void
MPU6050::cycle_trampoline(void *arg)
{
    MPU6050 *dev = (MPU6050 *)arg;

    dev->cycle();
}

void
MPU6050::cycle()
{
    measure();

    /* schedule a fresh cycle call when the measurement is done */
    work_queue(HPWORK,
           &_work,
           (worker_t)&MPU6050::cycle_trampoline,
           this,
           USEC2TICK(_call_interval));
}

int
MPU6050::collect()
{
    int ret = -EIO;

    perf_begin(_sample_perf);


    ret = OK;

    perf_end(_sample_perf);
    return ret;
}

void
MPU6050::print_info()
{
    printf("reads:          %u\n", _reads);
}

MPU6050_gyro::MPU6050_gyro(MPU6050 *parent) :
    CDev("MPU6050_gyro", GYRO_DEVICE_PATH),
    _parent(parent)
{
}

MPU6050_gyro::~MPU6050_gyro()
{
}

void
MPU6050_gyro::parent_poll_notify()
{
    poll_notify(POLLIN);
}

ssize_t
MPU6050_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
    return _parent->gyro_read(filp, buffer, buflen);
}

int
MPU6050_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    return _parent->gyro_ioctl(filp, cmd, arg);
}

/**
 * Local functions in support of the shell command.
 */
namespace mpu6050
{

MPU6050 *g_dev;

void    start();
void    test();
void    reset();
void    info();

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
    g_dev = new MPU6050(TMR_I2C_BUS_ONBOARD);

    if (g_dev != nullptr && OK != g_dev->init()) {
        delete g_dev;
        g_dev = nullptr;
    }

    #if defined(TMR_I2C_BUS_ONBOARD)
    /* if this failed, attempt onboard sensor */
    if (g_dev == nullptr) {
        g_dev = new MPU6050(TMR_I2C_BUS_ONBOARD);

        if (g_dev != nullptr && OK != g_dev->init()) {
            goto fail;
        }
    }
    #endif

    /* set the poll rate to default, starts automatic data collection */
    fd = open(ACCEL_DEVICE_PATH, O_RDONLY);

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
    accel_report a_report;
    gyro_report g_report;
    ssize_t sz;

    /* get the driver */
    int fd = open(ACCEL_DEVICE_PATH, O_RDONLY);

    if (fd < 0)
        err(1, "%s open failed (try 'mpu6050 start' if the driver is not running)",
            ACCEL_DEVICE_PATH);

    /* get the driver */
    int fd_gyro = open(GYRO_DEVICE_PATH, O_RDONLY);

    if (fd_gyro < 0)
        err(1, "%s open failed", GYRO_DEVICE_PATH);

    /* reset to manual polling */
    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0)
        err(1, "reset to manual polling");

    /* do a simple demand read */
    sz = read(fd, &a_report, sizeof(a_report));

    if (sz != sizeof(a_report)) {
        warnx("ret: %d, expected: %d", sz, sizeof(a_report));
        err(1, "immediate acc read failed");
    }

    warnx("single read");
    warnx("time:     %lld", a_report.timestamp);
    warnx("acc  x:  \t%9.4f\tm/s^2", (double)a_report.x);
    warnx("acc  y:  \t%9.4f\tm/s^2", (double)a_report.y);
    warnx("acc  z:  \t%9.4f\tm/s^2", (double)a_report.z);
    warnx("acc  x:  \t%9d\traw           (0x%04X)", (short)a_report.x_raw, (unsigned short)a_report.x_raw);
    warnx("acc  y:  \t%9d\traw           (0x%04X)", (short)a_report.y_raw, (unsigned short)a_report.y_raw);
    warnx("acc  z:  \t%9d\traw           (0x%04X)", (short)a_report.z_raw, (unsigned short)a_report.z_raw);
    warnx("acc range: \t%9.4f\tm/s^2         (%.4f g)", (double)a_report.range_m_s2,
          (double)(a_report.range_m_s2 / MPU6050_ONE_G));

    /* do a simple demand read */
    sz = read(fd_gyro, &g_report, sizeof(g_report));

    if (sz != sizeof(g_report)) {
        warnx("ret: %d, expected: %d", sz, sizeof(g_report));
        err(1, "immediate gyro read failed");
    }

    warnx("gyro x: \t%9.5f\trad/s", (double)g_report.x);
    warnx("gyro y: \t%9.5f\trad/s", (double)g_report.y);
    warnx("gyro z: \t%9.5f\trad/s", (double)g_report.z);
    warnx("gyro x: \t%9d\traw", (int)g_report.x_raw);
    warnx("gyro y: \t%9d\traw", (int)g_report.y_raw);
    warnx("gyro z: \t%9d\traw", (int)g_report.z_raw);
    warnx("gyro range: \t%9.4f\trad/s         (%d deg/s)", (double)g_report.range_rad_s,
          (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

    warnx("temp:  \t%9.4f\tdeg celsius", (double)a_report.temperature);
    warnx("temp:  \t%9d\traw           (0x%04X)", (short)a_report.temperature_raw, (unsigned short)a_report.temperature_raw);


    /* XXX add poll-rate tests here too */

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
        err(1, "reset to manual polling");

    //reset();
    errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
    int fd = open(ACCEL_DEVICE_PATH, O_RDONLY);

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
mpu6050_main(int argc, char *argv[])
{
    /*
     * Start/load the driver.

     */
    if (!strcmp(argv[1], "start"))
        mpu6050::start();

    /*
     * Test the driver/device.
     */
    if (!strcmp(argv[1], "test"))
        mpu6050::test();

    /*
     * Reset the driver.
     */
    if (!strcmp(argv[1], "reset"))
        mpu6050::reset();

    /*
     * Print driver information.
     */
    if (!strcmp(argv[1], "info"))
        mpu6050::info();

    errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
