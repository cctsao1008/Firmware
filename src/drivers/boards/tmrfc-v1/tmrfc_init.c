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
 * @file tmrfc_init.c
 *
 * TMRFC-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbdev_trace.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/i2c.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include "stm32.h"
#include "board_config.h"
#include "stm32_uart.h"
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>
#include <systemlib/cpuload.h>

#include <termios.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <apps/nsh.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/storage.h>
#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init();
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{
    /* configure SPI interfaces */
    stm32_spiinitialize();

    /* configure USB interfaces */
    stm32_usbinitialize();

    /* configure LEDs (empty call to NuttX' ledinit) */
    up_ledinit();
}

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/
#if 0
static struct spi_dev_s *spi1;
static struct spi_dev_s *spi2;
#endif
static struct spi_dev_s *spi;
static struct spi_dev_s *spi3;
static struct sdio_dev_s *sdio;

#include <math.h>

#ifdef __cplusplus
__EXPORT int matherr(struct __exception *e)
{
    return 1;
}
#else
__EXPORT int matherr(struct exception *e)
{
    return 1;
}
#endif

__EXPORT int nsh_archinitialize(void)
{
    int result;

    #if 0
    /* configure always-on ADC pins */
    stm32_configgpio(GPIO_ADC1_IN10);
    stm32_configgpio(GPIO_ADC1_IN11);
    /* IN12 and IN13 further below */
    #endif

    message("[boot] Initializing (HRT) 1...\n");
    usleep(200000);

    /* configure the high-resolution time/callout interface */
    hrt_init();

    /* configure CPU load estimation */
    #ifdef CONFIG_SCHED_INSTRUMENTATION
    message("[boot] Initializing (CPU) 2...\n");
    usleep(200000);
    cpuload_initialize_once();
    #endif

    /* set up the serial DMA polling */
    static struct hrt_call serial_dma_call;
    struct timespec ts;

    /*
     * Poll at 1ms intervals for received bytes that have not triggered
     * a DMA event.
     */
    ts.tv_sec = 0;
    ts.tv_nsec = 1000000;

    #ifdef SERIAL_HAVE_DMA
    message("[boot] Initializing (DMA) 3...\n");
    usleep(200000);
    hrt_call_every(&serial_dma_call,
               ts_to_abstime(&ts),
               ts_to_abstime(&ts),
               (hrt_callout)stm32_serial_dma_poll,
               NULL);
   #endif

    /* All LEDs controlled by PCA9533 and PCA9536 in TMR-FC via I2C */
    /* use pca953x as default LED driver(cdev), and register it to "/dev/led", and disable original LED driver (led.cpp) */
    message("[boot] Initializing pca953x driver\n");
    drv_pca953x_start();

    #if 0 /* disable original LED driver (led.cpp), because it don't work, don't know why it?? */
    /* initial LED state */
    message("[boot] Initializing Generic LED driver\n");
    drv_led_start();
    #endif

    led_off(LED_AMBER);
    led_off(LED_BLUE);

    /* OS in running state */
    led_on(LED_GREEN);

    #if defined(CONFIG_SPI_BITBANG) && defined(CONFIG_MMCSD_SPI)  /* Use SPI to access Micro SD card  */
    
    /* Get the SPI driver instance for the SD chip select */
    message("[boot] Initializing soft SPI for the MMC/SD slot\n");

    spi = spi_initialize();

    if (!spi)
    {
        message("[boot] Failed to initialize soft SPI\n");
            return -ENODEV;
    }
    else
    {
        message("[boot] Successfully initialized soft SPI for the MMC/SD slot\n");
    }

    /* Bind the SPI device for the chip select to the slot */

    message("[boot] Binding soft SPI device to MMC/SD slot %d\n",
        CONFIG_NSH_MMCSDSLOTNO);

    result = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi);
    if (result < 0)
    {
        message("[boot] Failed to bind soft SPI device to MMC/SD slot %d: %d\n",
            CONFIG_NSH_MMCSDSLOTNO, result);
        return result;
    }

    message("[boot] Successfuly bound Soft SPI device to MMC/SD slot %d\n",
        CONFIG_NSH_MMCSDSLOTNO);
        
    #else  /* Use SDIO to access Micro SD card  */
    
    /* Mount the SDIO-based MMC/SD block driver first and get an instance of the SDIO interface */
    message("[boot] Initializing SDIO slot %d\n", CONFIG_NSH_MMCSDSLOTNO);
    sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);

    if (!sdio) {
        message("[boot] Failed to initialize SDIO slot %d\n", CONFIG_NSH_MMCSDSLOTNO);
        return -ENODEV;
    }

    /* Now bind the SDIO interface to the MMC/SD driver */
    message("[boot] Bind SDIO to the MMC/SD driver, minor=%d\n", CONFIG_NSH_MMCSDMINOR);
    result = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);
    if (result != OK) {
        message("[boot] Failed to bind SDIO to the MMC/SD driver: %d\n", result);
        return result;
    }
    
    message("[boot] Successfully bound SDIO to the MMC/SD driver\n");
  
    /* Then let's guess and say that there is a card in the slot */
    sdio_mediachange(sdio, true);
    #endif

    /* Initializing SPI port 3 */
    message("[boot] Initializing SPI3\n");
    spi3 = up_spiinitialize(3);

    if (!spi3) {
        message("[boot] Failed to initialize SPI port 3\n");
        return -ENODEV;
    }

    return OK;
}


/****************************************************************************
 * Name: usbmsc_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/
__EXPORT int usbmsc_archinitialize(void)
{
    return OK;
}

#if defined(CONFIG_USBDEV_COMPOSITE)
/****************************************************************************
 * Name: composite_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/
__EXPORT int composite_archinitialize(void)
{
    return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: usbmsc_exportluns
 *
 * Description:
 *   After all of the LUNs have been bound, this function may be called
 *   in order to export those LUNs in the USB storage device.
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to usbmsc_configure().
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/
__EXPORT int usbmsc_exportluns(FAR void *handle)
{
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: cdcacm_initialize
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 * Input Parameter:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/ttyACM0.
 *   handle - An optional opaque reference to the CDC/ACM class object that
 *     may subsequently be used with cdcacm_uninitialize().
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is retured.
 *
 ****************************************************************************/
__EXPORT int cdcacm_initialize(int minor, FAR void **handle)
{
    return EXIT_SUCCESS;
}

#endif

