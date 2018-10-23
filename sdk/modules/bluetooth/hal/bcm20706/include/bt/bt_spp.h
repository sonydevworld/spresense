/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/bt/bt_spp.h
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
 * @file       bt_spp.h
 */

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_SPP_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_SPP_H

/**
 * @defgroup BT
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <bt/bt_comm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup bt_defs Defines
 * @{
 */

#define MAX_SPP_LEN 998 /**< SPP payload data max length */
#define BT_SPP_UUID128_LEN 16

/**
 * @name SPP Group Event Opcode Type
 * @{
 */
#define BT_EVT_SPP_CONNECT             0x01  /**< SPP connection opened */
#define BT_EVT_SPP_SERVICE_NOT_FOUND   0x02  /**< SDP record with SPP service not found */
#define BT_EVT_SPP_CONNECTION_FAILED   0x03  /**< Connection attempt failed  */
#define BT_EVT_SPP_DISCONNECT          0x04  /**< SPP connection closed */
#define BT_EVT_SPP_TX_COMPLETE         0x05  /**< Data packet has been queued for transmission */
#define BT_EVT_SPP_RX_DATA             0x06  /**< SPP rx data*/
#define BT_EVT_SPP_STATUS              0x07  /**< SPP connect status*/
/** @} */

/** @} bt_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

typedef struct
{
  uint16_t len;
  uint8_t data[MAX_SPP_LEN];
} BT_SPP_DATA;

/**@brief Spp 128-bit UUID types
 */
typedef struct
{
  uint8_t uuid128[BT_SPP_UUID128_LEN];
} BT_SPP_UUID;

/**@brief Spp rx callback function
 */
typedef void (*sppCallBack)(BT_SESSION_EVT* data);

/** @} bt_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup bt_funcs Functions
 * @{
 */

/**@brief   Create spp connection to remote device
 *
 * @param[in]  addr: remote device address
 *
 * @return     0 on success,otherwise error
 * @retval     -ENOENT: can't find nearby device or remote device is not a spp device
 * @retval     -EBUSY: system error
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SppConnect(BT_ADDR *addr);

/**@brief   distroy spp connection to remote device
 *
 * @param[in]  addr: remote device address
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find this connected device
 * @retval     -ENOMEM: no more memory
 * @retval     -EBUSY: system error
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SppDisconnect(BT_ADDR *addr);

/**@brief   send spp data to remote device
 *
 * @param[in]  addr: remote device address
 * @param[in]  data: pointer to data to be send
 * @param[in]  len: length of data to be send
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find this connected device
 * @retval     -EINVAL: invalid argument
 * @retval     -EBUSY: system error
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SppSendData(BT_ADDR *addr, uint8_t *data, uint16_t len);

/**@brief   set spp rx callback
 *
 * @param[in]  addr: remote device
 * @param[in]  cb: spp rx callback to set
 * @return     0: success
 * @retval     -EINVAL: invalid argument
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SppSetRxCallBack(BT_ADDR *addr, sppCallBack cb);

/**@brief   set vendor spp 128-bit UUID
 *          This API shall be called before BT_CommonInitializeWithBtBinary()
 *
 * @param[in]  uuid: vendor spp 128-bit UUID
 * @param[in]  cb: spp rx callback to set
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SppSetUuid(BT_SPP_UUID *uuid);

/** @} bt_funcs */

/** @} BT */

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_SPP_H */
