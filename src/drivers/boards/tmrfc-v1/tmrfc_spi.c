/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 TMR Development Team. All rights reserved.
 *
 *   Author : CHIA-CHENG, TSAO
 *
 *   E-Mail : chiacheng.tsao@gmail.com
 *
 *   Date :07/09/2013
 *
 ****************************************************************************/

/**
 * @file spi.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32.h"
#include "board_config.h"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the TMRFC board.
 *
 ************************************************************************************/

__EXPORT void weak_function stm32_spiinitialize(void)
{
    #if 0
    stm32_configgpio(GPIO_SPI_CS_GYRO);
    stm32_configgpio(GPIO_SPI_CS_ACCEL);
    stm32_configgpio(GPIO_SPI_CS_MPU);
    stm32_configgpio(GPIO_SPI_CS_SDCARD);
    #endif

    /* De-activate all peripherals,
     * required for some peripheral
     * state machines
     */
    #if 0
    stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
    stm32_gpiowrite(GPIO_SPI_CS_ACCEL, 1);
    stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);
    stm32_gpiowrite(GPIO_SPI_CS_SDCARD, 1);
    #endif
}

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
    /* SPI select is active low, so write !selected to select the device */
    switch (devid) {
    #if 0
    case TMR_SPIDEV_GYRO:
        /* Making sure the other peripherals are not selected */
        stm32_gpiowrite(GPIO_SPI_CS_GYRO, !selected);
        stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);
        stm32_gpiowrite(GPIO_SPI_CS_ACCEL, 1);
        break;

    case TMR_SPIDEV_ACCEL:
        /* Making sure the other peripherals are not selected */
        stm32_gpiowrite(GPIO_SPI_CS_ACCEL, !selected);
        stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);
        stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
        break;

    case TMR_SPIDEV_MPU:
        /* Making sure the other peripherals are not selected */
        stm32_gpiowrite(GPIO_SPI_CS_ACCEL, 1);
        stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
        stm32_gpiowrite(GPIO_SPI_CS_MPU, !selected);
        break;
    #endif

    default:
        break;

    }
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
    return SPI_STATUS_PRESENT;
}

__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
    /* SPI select is active low, so write !selected to select the device */

    switch (devid) {
        break;

    default:
        break;

    }
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
    return SPI_STATUS_PRESENT;
}


__EXPORT void stm32_spi3select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
    /* there can only be one device on this bus, so always select it */
    #if 0
    stm32_gpiowrite(GPIO_SPI_CS_SDCARD, !selected);
    #endif
}

__EXPORT uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
    /* this is actually bogus, but TMR has no way to sense the presence of an SD card */
    return SPI_STATUS_PRESENT;
}

