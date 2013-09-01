/****************************************************************************
 * src/drivers/boards/tmrfc/spi_bitbang.c
 *
 *   Copyright (C) 2012 TMR Development Team. All rights reserved.
 *   Author: CHIA CHENG, TSAO <chiacheng.tsao@gmail.com>
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
 * @file tmrfc_spi_bitbang.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_bitbang.h>

#include <arch/board/board.h>
#include <stm32_gpio.h>

#include "stm32.h"
#include "board_config.h"


/* In order to use the SD card on the ITEAD shield, you must enable the SPI bit-bang
 * driver as well as support for SPI-based MMC/SD cards.
 */

#if defined(CONFIG_SPI_BITBANG) && defined(CONFIG_MMCSD_SPI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error Mountpoints are disabled (CONFIG_DISABLE_MOUNTPOINT=y)
#endif

/* Definitions for include/nuttx/spi/spi_bitbang.c. */

#define SPI_SETSCK    stm32_gpiowrite(BITBANG_GPIO_SPI_SCLK, true)
#define SPI_CLRSCK    stm32_gpiowrite(BITBANG_GPIO_SPI_SCLK, false)
#define SPI_SETMOSI   stm32_gpiowrite(BITBANG_GPIO_SPI_MOSI, true)
#define SPI_CLRMOSI   stm32_gpiowrite(BITBANG_GPIO_SPI_MOSI, false)
#define SPI_GETMISO   stm32_gpioread(BITBANG_GPIO_SPI_MISO)
#define SPI_SETCS     stm32_gpiowrite(BITBANG_GPIO_SPI_CS, true)
#define SPI_CLRCS     stm32_gpiowrite(BITBANG_GPIO_SPI_CS, false)

/* Only mode 0 */

#undef  SPI_BITBANG_DISABLEMODE0
#define SPI_BITBANG_DISABLEMODE1 1
#define SPI_BITBANG_DISABLEMODE2 1
#define SPI_BITBANG_DISABLEMODE3 1

/* Only 8-bit data width */

#undef SPI_BITBANG_VARWIDTH

/* Calibration value for timing loop */

#define SPI_BITBAND_LOOPSPERMSEC CONFIG_BOARD_LOOPSPERMSEC

/* SPI_PERBIT_NSEC is the minimum time to transfer one bit.  This determines
 * the maximum frequency and is also used to calculate delays to achieve
 * other SPI frequencies.
 */

#define SPI_PERBIT_NSEC      100

/* Misc definitions */


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void spi_select(FAR struct spi_bitbang_s *priv, enum spi_dev_e devid,
                       bool selected);
static uint8_t spi_status(FAR struct spi_bitbang_s *priv, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(FAR struct spi_bitbang_s *priv, enum spi_dev_e devid,
                       bool cmd);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Include the bit-band skeleton logic
 ****************************************************************************/

#include <nuttx/spi/spi_bitbang.c>

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Select or de-selected the SPI device specified by 'devid'
 *
 * Input Parameters:
 *   priv     - An instance of the bit-bang driver structure
 *   devid    - The device to select or de-select
 *   selected - True:select false:de-select
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(FAR struct spi_bitbang_s *priv, enum spi_dev_e devid,
                       bool selected)
{
  if (devid == SPIDEV_MMCSD)
    {
      if (selected)
        {
          SPI_CLRCS;
        }
      else
        {
          SPI_SETCS;
        }
    }
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Return status of the SPI device specified by 'devid'
 *
 * Input Parameters:
 *   priv     - An instance of the bit-bang driver structure
 *   devid    - The device to select or de-select
 *
 * Returned Value:
 *   An 8-bit, bit-encoded status byte
 *
 ****************************************************************************/

static uint8_t spi_status(FAR struct spi_bitbang_s *priv, enum spi_dev_e devid)
{
  if (devid == SPIDEV_MMCSD)
    {
      return SPI_STATUS_PRESENT;
    }

  return 0;
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   If there were was a CMD/DATA line, this function would manage it
 *
 * Input Parameters:
 *   priv  - An instance of the bit-bang driver structure
 *   devid - The device to use
 *   cmd   - True=MCD false=DATA
 *
 * Returned Value:
 *  OK
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(FAR struct spi_bitbang_s *priv, enum spi_dev_e devid,
                       bool cmd)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: spi_initialize
 *
 * Description:
 *   Initialize the SPI bit-bang driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A non-NULL reference to the SPI driver on success
 *
 ****************************************************************************/
FAR struct spi_dev_s *spi_initialize(void)
{
  /* Initialize GPIOs */

  stm32_configgpio(BITBANG_GPIO_SPI_SCLK);
  stm32_configgpio(BITBANG_GPIO_SPI_MISO);
  stm32_configgpio(BITBANG_GPIO_SPI_MOSI);
  stm32_configgpio(BITBANG_GPIO_SPI_CS);

  /* Create the SPI driver instance */

  return spi_create_bitbang(&g_spiops);
}

#endif /* CONFIG_SPI_BITBANG && CONFIG_MMC_SPI */
