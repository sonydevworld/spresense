/****************************************************************************
 * nrf52/nRF5_SDK_17.1.0_ddde560/config/nrf52832/config/nrf_porting.h
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

#ifndef NRF_PORTING_H
#define NRF_PORTING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define __STATIC_INLINE static __inline

#ifdef CONFIG_EXTERNALS_NRF52_OUTPUT_LOG_LEVEL
#  if CONFIG_EXTERNALS_NRF52_OUTPUT_LOG_LEVEL > 0
#    define NRF_LOG_ENABLED 1
#    define NRF_LOG_DEFAULT_LEVEL CONFIG_EXTERNALS_NRF52_OUTPUT_LOG_LEVEL

/* configuration for nrf_queue component */

#    ifdef CONFIG_EXTERNALS_NRF52_NRF_QUEUE_LOG_LEVEL
#      define NRF_QUEUE_CONFIG_LOG_LEVEL \
              CONFIG_EXTERNALS_NRF52_NRF_QUEUE_LOG_LEVEL
#      if NRF_QUEUE_CONFIG_LOG_LEVEL > 0
#        define NRF_QUEUE_CONFIG_LOG_ENABLED 1
#      endif /* NRF_QUEUE_CONFIG_LOG_LEVEL > 0 */
#    endif /* CONFIG_EXTERNALS_NRF52_NRF_QUEUE_LOG_LEVEL */

/* configuration for nrf_sdh_ble component */

#    ifdef CONFIG_EXTERNALS_NRF52_NRF_SDH_BLE_LOG_LEVEL
#      define NRF_SDH_BLE_CONFIG_LOG_LEVEL \
              CONFIG_EXTERNALS_NRF52_NRF_SDH_BLE_LOG_LEVEL
#      if NRF_SDH_BLE_CONFIG_LOG_LEVEL!=0
#        define NRF_SDH_BLE_CONFIG_LOG_ENABLED 1
#      endif /* NRF_SDH_BLE_CONFIG_LOG_LEVEL > 0 */
#    endif /* CONFIG_EXTERNALS_NRF52_NRF_SDH_BLE_LOG_LEVEL */

/* configuration for ser_hal_transport component */

#    ifdef CONFIG_EXTERNALS_NRF52_SER_HAL_TRANSPORT_LOG_LEVEL
#      define SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL \
              CONFIG_EXTERNALS_NRF52_SER_HAL_TRANSPORT_LOG_LEVEL
#      if SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL > 0
#        define SER_HAL_TRANSPORT_CONFIG_LOG_ENABLED 1
#      endif /* SER_HAL_TRANSPORT_CONFIG_LOG_LEVELi > 0 */
#    endif /* CONFIG_EXTERNALS_NRF52_SER_HAL_TRANSPORT_LOG_LEVEL */

/* configuration for ser_sd_transport component */

#    ifdef CONFIG_EXTERNALS_NRF52_SER_SD_TRANSPORT_LOG_LEVEL
#      define SER_SD_TRANSPORT_CONFIG_LOG_LEVEL \
              CONFIG_EXTERNALS_NRF52_SER_SD_TRANSPORT_LOG_LEVEL
#      if SER_SD_TRANSPORT_CONFIG_LOG_LEVEL > 0
#        define SER_SD_TRANSPORT_CONFIG_LOG_ENABLED 1
#      endif /* SER_SD_TRANSPORT_CONFIG_LOG_LEVEL > 0 */
#    endif /* CONFIG_EXTERNALS_NRF52_SER_SD_TRANSPORT_LOG_LEVEL */

/* configuration for ser_sd_handler component */

#    ifdef CONFIG_EXTERNALS_NRF52_SER_SD_HANDLER_LOG_LEVEL
#      define SER_SD_HANDLER_CONFIG_LOG_LEVEL \
              CONFIG_EXTERNALS_NRF52_SER_SD_HANDLER_LOG_LEVEL
#      if SER_SD_HANDLER_CONFIG_LOG_LEVEL > 0
#        define SER_SD_HANDLER_CONFIG_LOG_ENABLED 1
#      endif /* SER_SD_HANDLER_CONFIG_LOG_LEVEL > 0 */
#    endif /* CONFIG_EXTERNALS_NRF52_SER_SD_HANDLER_LOG_LEVEL */

/* configuration for ser_phy_uart component */

#    ifdef CONFIG_EXTERNALS_NRF52_SER_PHY_UART_LOG_LEVEL
#      define SER_PHY_UART_CONFIG_LOG_LEVEL \
              CONFIG_EXTERNALS_NRF52_SER_PHY_UART_LOG_LEVEL
#      if SER_PHY_UART_CONFIG_LOG_LEVEL > 0
#        define SER_PHY_UART_CONFIG_LOG_ENABLED 1
#      endif /* SER_PHY_UART_CONFIG_LOG_LEVEL > 0 */
#    endif /* CONFIG_EXTERNALS_NRF52_SER_PHY_UART_LOG_LEVEL */
#  endif /* CONFIG_EXTERNALS_NRF52_OUTPUT_LOG_LEVEL > 0 */
#endif /* CONFIG_EXTERNALS_NRF52_OUTPUT_LOG_LEVEL */

#ifdef CONFIG_EXTERNALS_NRF52_CRYPTOGRAPHY_MBEDTLS
#  define NRF52_CRYPTO_BACKEND_MBEDTLS   1
#else
#  define NRF52_CRYPTO_BACKEND_MBEDTLS   0
#endif /* CONFIG_EXTERNALS_NRF52_CRYPTOGRAPHY_MBEDTLS */

#ifdef CONFIG_EXTERNALS_NRF52_CRYPTOGRAPHY_MICRO_ECC
#  define NRF52_CRYPTO_BACKEND_MICRO_ECC 1
#else
#  define NRF52_CRYPTO_BACKEND_MICRO_ECC 0
#endif /* CONFIG_EXTERNALS_NRF52_CRYPTOGRAPHY_MICRO_ECC */
#endif /* NRF_PORTING_H */
