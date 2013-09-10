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
 * @file usb.c
 *
 * Board-specific USB functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "up_arch.h"
#include "stm32.h"
#include "board_config.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the TMRFC board.
 *
 ************************************************************************************/

__EXPORT void stm32_usbinitialize(void)
{
    /* The OTG FS has an internal soft pull-up */

    /* Configure the OTG FS VBUS sensing GPIO, Power On, and Overcurrent GPIOs */

#ifdef CONFIG_STM32_OTGFS
    stm32_configgpio(GPIO_OTGFS_VBUS);
    stm32_configgpio(GPIO_OTGFS_PWRON);
    stm32_configgpio(GPIO_OTGFS_OVER);
#endif
}

/************************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

__EXPORT void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
    ulldbg("resume: %d\n", resume);
}

