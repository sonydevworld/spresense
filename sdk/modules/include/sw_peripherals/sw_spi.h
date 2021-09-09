/****************************************************************************
 * modules/include/sw_peripherals/sw_spi.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
 * @file sw_spi.h
 */

#ifndef __MODULES_SWPERIPHERALS_SW_SPI_H__
#define __MODULES_SWPERIPHERALS_SW_SPI_H__

/**
 * @defgroup sw_spi Software SPI
 * @{
 *
 * Software emulated SPI Driver.
 */

#include <nuttx/spi/spi.h>

/**
 * @defgroup sw_spi_datatype Data Types
 * @{
 */

/**
 * @struct swspi_s
 *
 * Instance of software SPI device.
 */

struct swspi_s
{
  int cs;
  int sck;
  int mosi;
  int miso;

  enum spi_mode_e mode;
  int bits;
  int delay_us;
};

/** @} sw_spi_datatype */

#  ifdef __cplusplus
#    define EXTERN extern "C"
extern "C"
{
#  else
#    define EXTERN extern
#  endif

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

/**
 * @defgroup sw_spi_func Functions
 * @{
 */

/**
 * Setup Software emulated SPI driver instance
 *
 * @param [in] dev: Instance of Software SPI to setup.
 * @param [in] cs_pin: Chip Select pin number.
 * @param [in] sck_pin: SCK signal pin number.
 * @param [in] mosi_pin: MOSI signal pin number.
 * @param [in] miso_pin: MISO signal pin number.
 */

void SWSPI_SETUP(FAR struct swspi_s *dev, int cs_pin,
    int sck_pin, int mosi_pin, int miso_pin);

/**
 * Select the device.
 *
 * @param [in] dev: Instance of Software SPI.
 * @param [in] devid: Device ID to select. Now this is just dummy.
 * @param [in] selected: Set true to activate Chip Select pin.
 */

void SWSPI_SELECT(FAR struct swspi_s *dev, uint32_t devid, bool selected);

/**
 * Set SPI mode.
 * Now this supports only mode0.
 *
 * @param [in] dev: Instance of Software SPI.
 * @param [in] mode: SPI Mode selection. Choose @ref SPIDEV_MODE0,
 *                  @ref SPIDEV_MODE1, @ref SPIDEV_MODE2 or @ref SPIDEV_MODE3
 */

void SWSPI_SETMODE(FAR struct swspi_s *dev, enum spi_mode_e mode);

/**
 * Set SPI bits per a word.
 * Now this supports only 8 bits.
 *
 * @param [in] dev: Instance of Software SPI.
 * @param [in] bits: Bits per words. Select 8, 16 or 32.
 */

void SWSPI_SETBITS(FAR struct swspi_s *dev, int bits);

/**
 * Set SPI Frequency.
 * Now this supports only maximum.
 *
 * @param [in] dev: Instance of Software SPI.
 * @param [in] freq: Frequency to set.

 * @return Actual frequency.
 */

uint32_t SWSPI_SETFREQUENCY(FAR struct swspi_s *dev, uint32_t freq);

/**
 * Execute SPI transaction.
 *
 * @param [in] dev: Instance of Software SPI.
 * @param [in] txbuff: Data memory address to send.
 *                     If set NULL, MOSI signal is always Low.
 * @param [in] rxbuff: Memory address to store received data.
 *                     If set NULL, MISO signal is avoided.
 * @param [in] nwords: Words to send/receive.
 *                     txbuff and rxbuff size is required
 *                     more than bits * nwords.
 */

void SWSPI_EXCHANGE(FAR struct swspi_s *dev,
    FAR const void *txbuff, FAR void *rxbuff, size_t nwords);

/** @} sw_spi_func */

#  undef EXTERN
#  ifdef __cplusplus
}
#  endif

/** @} sw_spi */

#endif  /* __MODULES_SWPERIPHERALS_SW_SPI_H__ */
