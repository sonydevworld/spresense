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

#ifndef __MODULES_SW_PERIPHERALS_SW_SPI_H__
#define __MODULES_SW_PERIPHERALS_SW_SPI_H__

/**
 * @defgroup sw_spi Software SPI
 * @{
 *
 * Software emulated SPI Driver.
 */

#include <nuttx/spi/spi.h>

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
 * Create software emulated SPI driver.
 *
 * @param [in] cs_pin: Chip Select pin number.
 * @param [in] sck_pin: SCK signal pin number.
 * @param [in] mosi_pin: MOSI signal pin number.
 * @param [in] miso_pin: MISO signal pin number.
 */

FAR struct spi_dev_s *create_swspi(int cs_pin, int sck_pin, int mosi_pin,
    int miso_pin);

/**
 * Destroy an instance of software emulated SPI driver.
 *
 * @param [in] dev: Instance of Software SPI.
 */

void destroy_swspi(FAR struct spi_dev_s *dev);

/** @} sw_spi_func */

#  undef EXTERN
#  ifdef __cplusplus
}
#  endif

/** @} sw_spi */

#endif  /* __MODULES_SW_PERIPHERALS_SW_SPI_H__ */
