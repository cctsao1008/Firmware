/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file i2c.c
 *
 * i2c hacking tool
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/i2c.h>

#include <board_config.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"

#if defined(CONFIG_ARCH_BOARD_TMRFC_V1)

#define PCA9536DP_ADDRESS    TMR_I2C_OBDEV_PCA9536
#define PCA9533DP_ADDRESS    TMR_I2C_OBDEV_PCA9533
#define MPU6050_ADDRESS      TMR_I2C_OBDEV_MPU6050
#define HMC5883L_ADDRESS     TMR_I2C_OBDEV_HMC5883
#define MS5611_ADDRESS       TMR_I2C_OBDEV_MS5611

/* Registers and Settings For MPU6050 */
#define MPU6050_RA_INT_PIN_CFG              0x37

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_RA_PWR_MGMT_1               0x6B

#define MPU6050_PWR1_DEVICE_RESET_BIT       7
#define MPU6050_PWR1_SLEEP_BIT              6
#define MPU6050_PWR1_CYCLE_BIT              5
#define MPU6050_PWR1_TEMP_DIS_BIT           3
#define MPU6050_PWR1_CLK_SEL_BIT            2
#define MPU6050_PWR1_CLK_SEL_LENGTH         3

/* Commands For MS5611 */
#define CMD_RESET                           0x1E

#ifndef TMR_I2C_BUS_ONBOARD
#  error TMR_I2C_BUS_ONBOARD not defined, no device interface
#endif
 
#else

#ifndef PX4_I2C_BUS_ONBOARD
#  error PX4_I2C_BUS_ONBOARD not defined, no device interface
#endif

#ifndef PX4_I2C_OBDEV_PX4IO
#  error PX4_I2C_OBDEV_PX4IO not defined
#endif

#endif

__EXPORT int i2c_main(int argc, char *argv[]);

static int transfer(uint8_t address, uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len);

static struct i2c_dev_s *i2c;

int i2c_main(int argc, char *argv[])
{
	/* find the right I2C */
    uint32_t val, ret;
    uint8_t buf[] = { 0, 4};

    #if defined(CONFIG_ARCH_BOARD_TMRFC_V1)

    i2c = up_i2cinitialize(TMR_I2C_BUS_ONBOARD);
    usleep(100000);

    if (i2c == NULL)
		errx(1, "failed to locate I2C bus\n");

    printf("Detecting on board sensors on I2C bus(I2C2) ......\n\n");

    ret = transfer(MPU6050_ADDRESS, buf, sizeof(buf), NULL, 0);
    if (ret)
		printf("MPU6050 send failed - %d\n", ret);

    ret = transfer(PCA9533DP_ADDRESS, NULL, 0, (uint8_t *)&val, sizeof(val));
	if (ret)
        printf("PCA9533DP recive failed - %d\n", ret);
    else
        printf("( 0x%x ) Have PCA9533DP\n", PCA9533DP_ADDRESS);

    ret = transfer(PCA9536DP_ADDRESS, NULL, 0, (uint8_t *)&val, sizeof(val));
	if (ret)
		printf("PCA9536DP recive failed - %d\n", ret);
	else
	    printf("( 0x%x ) Have PCA9536DP\n", PCA9536DP_ADDRESS);
                
	ret = transfer(MPU6050_ADDRESS, NULL, 0, (uint8_t *)&val, sizeof(val));
	if (ret)
		printf("MPU6050 recive failed - %d\n", ret);
    else
    {
	    printf("( 0x%x ) Have MPU6050\n\n", MPU6050_ADDRESS);
        printf("  Set MPU6050 auxiliary I2C bus to bypass mode......\n\n");

        buf[0] = MPU6050_RA_PWR_MGMT_1;
        buf[1] = (1 << MPU6050_PWR1_DEVICE_RESET_BIT);

        ret = transfer(MPU6050_ADDRESS, buf, sizeof(buf), NULL, 0); /* Reset command*/
        if(ret)
           printf("  MPU6050 send failed - %d\n", ret);

        usleep(100000);

        buf[0] = MPU6050_RA_PWR_MGMT_1;
        buf[1] = 0x0;

        ret = transfer(MPU6050_ADDRESS, buf, sizeof(buf), NULL, 0); /* Leave from sleep mode */
        if(ret)
           printf("  MPU6050 send failed - %d\n", ret);

        usleep(100000);

        buf[0] = MPU6050_RA_INT_PIN_CFG;
        buf[1] = (1 << MPU6050_INTCFG_I2C_BYPASS_EN_BIT);

        ret = transfer(MPU6050_ADDRESS, buf, sizeof(buf), NULL, 0); /* Disable MST I2C */
        if(ret)
           printf("  MPU6050 send failed - %d\n", ret);
        else
        {
            printf("  Detecting sensors on MPU6050 auxiliary I2C bus......\n\n");

            usleep(100000);

            ret = transfer(HMC5883L_ADDRESS, NULL, 0, (uint8_t *)&val, sizeof(val));
            if (ret)
                printf("  HMC5883 recive failed - %d\n", ret);
            else
                printf("  ( 0x%x ) Have HMC5883\n", HMC5883L_ADDRESS);

            buf[0] = CMD_RESET;

            ret = transfer(MS5611_ADDRESS, buf, 1, NULL, 0); /* Reset command*/
            if (ret)
                printf("  MS5611 recive failed - %d\n", ret);
            else
                printf("  ( 0x%x ) Have MS5611\n\n", MS5611_ADDRESS);
        }
            
            
	}

	errx(0, "Done\n");
    
    #else

    i2c = up_i2cinitialize(PX4_I2C_BUS_ONBOARD);

    if (i2c == NULL)
		errx(1, "failed to locate I2C bus");

	usleep(100000);

	ret = transfer(PX4_I2C_OBDEV_PX4IO, buf, sizeof(buf), NULL, 0);

	if (ret)
		errx(1, "send failed - %d", ret);

	ret = transfer(PX4_I2C_OBDEV_PX4IO, NULL, 0, (uint8_t *)&val, sizeof(val));
	if (ret)
		errx(1, "recive failed - %d", ret);
    

	errx(0, "got 0x%08x", val);

	#endif
}

static int
transfer(uint8_t address, uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	struct i2c_msg_s msgv[2];
	unsigned msgs;
	int ret;

	//	debug("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);

	msgs = 0;

	if (send_len > 0) {
		msgv[msgs].addr = address;
		msgv[msgs].flags = 0;
		msgv[msgs].buffer = send;
		msgv[msgs].length = send_len;
		msgs++;
	}

	if (recv_len > 0) {
		msgv[msgs].addr = address;
		msgv[msgs].flags = I2C_M_READ;
		msgv[msgs].buffer = recv;
		msgv[msgs].length = recv_len;
		msgs++;
	}

	if (msgs == 0)
		return -1;

	/*
	 * I2C architecture means there is an unavoidable race here
	 * if there are any devices on the bus with a different frequency
	 * preference.  Really, this is pointless.
	 */
	I2C_SETFREQUENCY(i2c, 400000);
	ret = I2C_TRANSFER(i2c, &msgv[0], msgs);

	// reset the I2C bus to unwedge on error
	if (ret != OK)
		up_i2creset(i2c);

	return ret;
}
