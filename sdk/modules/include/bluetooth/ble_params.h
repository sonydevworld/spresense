/****************************************************************************
 * modules/include/bluetooth/ble_params.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * @file ble_params.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief Bluetooth LE header for SDK on Spresense.
 * @details This header file includes bluetooth low energy API/HAL common definitions.
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_BLE_PARAMS_H
#define __MODULES_INCLUDE_BLUETOOTH_BLE_PARAMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct BLE_CONN_PARAMS
 * @brief Connection parameters for PPCP
 */
typedef struct {
  uint16_t minConnInterval; /**< Minimum Connection Interval in 1.25 ms units. (7.5ms - 4s) */
  uint16_t maxConnInterval; /**< Maximum Connection Interval in 1.25 ms units. (7.5ms - 4s) */
  uint16_t slaveLatency;    /**< Slave Latency in number of connection events. (max 499) */
  uint16_t connSupTimeout;  /**< Connection Supervision Timeout in 10 ms unit. (100ms - 32s) */
} BLE_CONN_PARAMS;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_INCLUDE_BLUETOOTH_BLE_PARAMS_H */
