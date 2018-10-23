/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/ble/ble_comm.h
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
 * @file       ble_comm.h
 */

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BLE_BLE_COMM_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BLE_BLE_COMM_H

/**
 * @defgroup BLE Bluetooth LE
 *
 * #include <ble/ble_comm.h>
 *
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <errno.h>
#include <bt/bt_comm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup ble_defs Defines
 * @{
 */

/**
 *@name BLE firmware
 *@{
 */
#define FIRMWARE_NAME_BLE                  "bcm20706fw" /**< Firmware name for BLE*/
/** @} */

/**
 *@name BLE Work Role
 *@{
 */
#define BLE_ROLE_PERIPHERAL                0x00 /**< Peripheral role */
#define BLE_ROLE_CENTRAL                   0x01 /**< Central role */
#define BLE_ROLE_PERIPHERAL_AND_CENTRAL    0x02 /**< Peripheral and Central role */
/** @} */

/**
 *@name Error Codes
 *@{
 */
#define BLE_SUCCESS                        0x0000 /**< Successful command */
/** @} */

/**
 *@name BLE Event Base Number
 *@{
 */
#define BLE_EVENT_BASE                     0x10 /**< Common BLE event base. */
#define BLE_GAP_EVENT_BASE                 0x30 /**< GAP BLE event base */
#define BLE_GATTS_EVENT_BASE               0x60 /**< GATTS BLE event base */
#define BLE_GATTC_EVENT_BASE               0x90 /**< GATTC BLE event base */
/** @} */

/**
 *@name Maximum Event Data Size
 *@{
 */
#define BLE_EVT_DAT_SIZE_MAX                (BT_EVT_DATA_LEN - 6) /**< Event max data size */
/** @} */

/** @} ble_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup ble_datatypes Data types
 * @{
 */

/**@brief BLE event types
 */
typedef enum BLE_EventTypes_t
{
  BLE_EVENT_TX_COMPLETE = BLE_EVENT_BASE,             /**< Transmission Complete. */
  BLE_GAP_EVENT_CONNECTED = BLE_GAP_EVENT_BASE,       /**< Connection established, see @ref BLE_EvtConnected */
  BLE_GAP_EVENT_CONN_PARAM_UPDATE,                    /**< Connection parameters updated, see @ref BLE_EvtConnParamUpdate */
  BLE_GAP_EVENT_CONN_PEER_MTU,                        /**< Peer device MTU, see @ref BLE_EvtConnParamUpdate */
  BLE_GAP_EVENT_DISCONNECTED,                         /**< Disconnected from peer, see @ref BLE_EvtDisconnected */
  BLE_GAP_EVENT_EXCHANGE_FEATURE,                     /**< Exchange pairing feature, see @ref BLE_EvtExchangeFeature */
  BLE_GAP_EVENT_DISPLAY_PASSKEY,                      /**< Display a passkey to user, see @ref BLE_EvtDisplayPasskey */
  BLE_GAP_EVENT_AUTH_STATUS,                          /**< Authentication procedure completed with status, see @ref BLE_EvtAuthStatus */
  BLE_GAP_EVENT_ADV_REPORT,                           /**< Advertising report, see @ref BLE_EvtAdvReportData */
  BLE_GAP_EVENT_AUTH_KEY_REQUEST,                     /**< Request to provide an authentication key, see @ref BLE_EvtAuthKey */
  BLE_GAP_EVENT_CONN_SEC_UPDATE,                      /**< Connection security updated */
  BLE_GAP_EVENT_TIMEOUT,                              /**< Operation is timeout, see @ref BLE_EvtTimeout */
  BLE_GATTS_EVENT_WRITE = BLE_GATTS_EVENT_BASE,       /**< Write operation performed for peer, see @ref BLE_EvtGattsWrite */
  BLE_GATTS_EVENT_CFM,                                /**< Confirmation event of indication operation, see @ref BLE_EvtGattsIndConfirm */
  BLE_GATTC_EVENT_DBDISCOVERY = BLE_GATTC_EVENT_BASE, /**< Attribute database discovery event, see @ref BLE_EvtGattcDbDiscovery */
  BLE_GATTC_EVENT_READ,                               /**< Read characteristic response event, see @ref BLE_EvtGattcRead */
  BLE_GATTC_EVENT_WRITE_RSP,                          /**< Write characteristic response event, see @ref BLE_EvtGattcWriteRsp */
  BLE_GATTC_EVENT_NTFIND,                             /**< Notification/Indication event from GATT server, see @ref BLE_EvtGattcNtfInd */
} BLE_EventTypes;

/**@brief BLE initialize parameter
 */
typedef struct BLE_InitializeParams_t
{
  uint8_t role; /**< BLE device work role. Peripheral, central or peripheral & central. @ref BLE_ROLE_PERIPHERAL, @ref BLE_ROLE_CENTRAL, @ref BLE_ROLE_PERIPHERAL_AND_CENTRAL*/
  BT_FIRMWARE_INFO info;
} BLE_InitializeParams;

/**@brief Event callback structure
 */
typedef struct BLE_Evt_t
{
  uint8_t opcode;
  uint8_t group;
  BLE_EventTypes evtHeader;                      /**< Event header */
  uint32_t       evtDataSize;                    /**< Event data size */
  uint8_t        evtData[BLE_EVT_DAT_SIZE_MAX];  /**< Event data */
} BLE_Evt;

typedef struct BLE_EvtTxComplete_t
{
  uint16_t connHandle; /**< Connection handle */
  uint8_t  count;      /**< Number of packets transmitted. */
} BLE_EvtTxComplete;

/**@brief Event context structure
 */
typedef struct BLE_EvtCtx_t
{
  BLE_Evt evt;   /**< Event callback structure */
  void    *data; /**< Event callback data. This field is totally user defined. For example you can define data as your application state. */
} BLE_EvtCtx;

/**@brief Event callback function
 */
typedef void (*BLE_EvtHandler)(BLE_Evt *bleEvt, void *data);

/**@brief BLE event handler
 */
typedef int (*BLE_EfCb)(void *data);

/** @} ble_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup ble_funcs Functions
 * @{
 */

/**@brief   Initialize the BLE stack
 *
 * @param[in]  initializeParams: Initialize stack parameter
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 * @note    This call initializes the BLE stack, no other BLE related function can be called before this one.
 */
int BLE_CommonInitializeStack(BLE_InitializeParams *initializeParams);

/**@brief Register a BLE event callback function
 * @details This call registers a BLE event (@ref BLE_EvtCtx) callback function.
 *
 * @param[in]  funcCb: BLE event handler
 * @param[in]  ctx: BLE event context
 * @retval     0: success
 * @retval     -EEXIST: buffer(ctx) already allocated, the exiting buffer will be used instead of replacing it,
 *              funcCb is set successfully
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 * @note   As this function may be called by event framework, app user may not call this function directly.
 */
int BLE_CommonSetBleEvtCallback(BLE_EfCb funcCb, BLE_EvtCtx *ctx);

/**@brief Finalize the BLE stack
 *
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
int BLE_CommonFinalizeStack(void);

/** @} ble_funcs */

#endif  /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BLE_BLE_COMM_H */
