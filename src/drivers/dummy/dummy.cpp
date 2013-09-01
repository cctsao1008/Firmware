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
 * @file dummy.cpp
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

#define DUMMY_DEVICE_PATH "/dev/dummy"
#define DUMMY_ADDRESS 0x00

class DUMMY : public device::I2C
{
public:
    DUMMY(int bus);
    virtual ~DUMMY();


    virtual int     init();
    virtual int     probe();
    virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

private:
    work_s          _work;

    /**
     * Initialise the automatic measurement state machine and start it.
     *
     * @note This function is called at open and error time.  It might make sense
     *       to make it more aggressive about resetting the bus in case of errors.
     */
    void            start();

    /**
     * Stop the automatic measurement state machine.
     */
    void            stop();

    /**
     * Reset the device
     */
    int         reset();
};

namespace dummy
{
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
        errx(1, "start()");
    }

    /**
    * Perform some basic functional tests on the driver;
    * make sure we can collect data from the sensor in polled
    * and automatic modes.
    */
    void
    test()
    {
        errx(1, "test()");
    }

    /**
    * Reset the driver.
    */
    void
    reset()
    {
        errx(1, "reset()");
    }

    /**
    * Print a little info about the driver.
    */
    void
    info()
    {
        errx(1, "info()");
    }
}

extern "C" __EXPORT int dummy_main(int argc, char *argv[]);

DUMMY::DUMMY(int bus) :
    I2C("dummy", DUMMY_DEVICE_PATH, bus, DUMMY_ADDRESS, 400000)
{
    memset(&_work, 0, sizeof(_work));
}

DUMMY::~DUMMY()
{
}

int
DUMMY::init()
{
    int ret;
    ret = I2C::init();

    if (ret != OK) {
        return ret;
    }

    return OK;
}

int
DUMMY::probe()
{
    int ret = true;

    return ret;
}

int
DUMMY::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    int ret = ENOTTY;
    switch (cmd) {

    default:
        break;
    }

    return ret;
}

int
dummy_main(int argc, char *argv[])
{
    /*
     * Start/load the driver.
     */
    if (!strcmp(argv[1], "start"))
        dummy::start();

    /*
     * Test the driver/device.
     */
    if (!strcmp(argv[1], "test"))
        dummy::test();

    /*
     * Reset the driver.
     */
    if (!strcmp(argv[1], "reset"))
        dummy::reset();

    /*
     * Print driver information.
     */
    if (!strcmp(argv[1], "info"))
        dummy::info();

    errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
