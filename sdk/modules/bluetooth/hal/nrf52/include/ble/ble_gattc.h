/****************************************************************************
 * modules/bluetooth/hal/nrf52/include/ble/ble_gattc.h
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

/**
 * @file       ble_gattc.h
 */

#ifndef __MODULES_BLUETOOTH_HAL_NRF52_INCLUDE_BLE_BLE_GATTC_H
#define __MODULES_BLUETOOTH_HAL_NRF52_INCLUDE_BLE_BLE_GATTC_H

/**
 * @defgroup BLE Bluetooth LE GATT client
 *
 * #include <ble/ble_gattc.h>
 *
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <ble/ble_gatts.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup ble_defs Defines
 * @{
 */

// #define BLE_DB_DISCOVERY_MAX_SRV          10 /**< Support Max services */
#define BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV 4 /**< Support Max characteristics per service */
#ifdef CONFIG_BCM20707
#define MAX_VAL_DATA_LENGTH              512 /**< Max value data length. */
#elif defined(CONFIG_BLUETOOTH_NRF52)
#define MAX_VAL_DATA_LENGTH              247 /**< Max receive data length. */
#else
#define MAX_VAL_DATA_LENGTH              20 /**< Max value data length. */
#endif

/**
 * @name GATT Client DB Discovery Result Code
 * @{
 * @sa @ref BLE_GATTC_EVENT_DBDISCOVERY
 */
#define BLE_GATTC_RESULT_SUCCESS                0x00 /**< GATTC attribute database discovery success */
#define BLE_GATTC_RESULT_FAILED                 0x01 /**< GATTC attribute database discovery failed */
/** @} */

/**
 * @name GATT Client DB Discovery Reason Code
 * @{
 * @sa @ref BLE_GATTC_EVENT_DBDISCOVERY
 */
#define BLE_GATTC_REASON_SERVICE                0x01 /**< GATTC db discovery failed reason: service discovery failed */
#define BLE_GATTC_REASON_CHARACTERISTIC         0x02 /**< GATTC db discovery failed reason: characteristic discovery failed */
#define BLE_GATTC_REASON_DESCRIPTOR             0x03 /**< GATTC db discovery failed reason: descriptor discovery failed */
/** @} */

/** @} ble_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup ble_datatypes Data types
 * @{
 */

/**@brief BLE write operation */
typedef enum
{
  BLE_GATTC_WRITE_CMD = 0,
  BLE_GATTC_WRITE_REQ
} BLE_GATTC_WRITE_OP;

/**@brief GATTC handle range structure*/
typedef struct
{
  uint16_t                startHandle; /**< Start handle */
  uint16_t                endHandle;   /**< End handle */
} BLE_GattcHandleRange;

/**@brief GATTC characteristic structure*/
typedef struct
{
  BLE_CHAR_PROP           charPrope;      /**< Characteristic property */
  uint8_t                 reserve;        /**< Reserve */
  uint16_t                charValhandle;  /**< Characteristic value handle */
  uint16_t                charDeclhandle; /**< Characteristic declaration handle */
  BLE_Uuid                charValUuid;    /**< Characteristic value uuid */
} BLE_GattcChar;

/**@brief GATTC discovered characteristic data structure
 */
typedef struct
{
  uint16_t                cccdHandle;     /**< Handle of client configuration characteristic descriptor in characteristic */
  uint8_t                 reserve[2];     /**< Reserve */
  BLE_GattcChar           characteristic; /**< Characteristic information */
} BLE_GattcDbDiscChar;

/**@brief GATTC discovered service data structure
 */
typedef struct
{
  uint8_t                 charCount;                                          /**< Discovered characteristic count in service */
  uint8_t                 reserve[3];                                         /**< Reserve */
  BLE_GattcHandleRange    srvHandleRange;                                     /**< The handle range of this service */
  BLE_GattcDbDiscChar     characteristics[BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV]; /**< Discovered characteristics in this service */
  BLE_Uuid                srvUuid;                                            /**< UUID of discovered service */
} BLE_GattcDbDiscSrv;

/**@brief GATTC discovered attribute database data structure
 */
typedef struct
{
  uint8_t                 srvCount;                           /**< Discovered services count */
  uint8_t                 reserve;                            /**< Reserve */
  uint16_t                connHandle;                         /**< Connection handle */
  BLE_GattcDbDiscSrv      services[BLE_DB_DISCOVERY_MAX_SRV]; /**< Discovered services in attribute database */
} BLE_GattcDbDiscovery;

/**@brief GATTC read parameters */
typedef struct
{
  uint16_t charValHandle; /**< Characteristic value handle */
} BLE_GattcReadParams;

/**@brief GATTC write parameters */
typedef struct
{
  BLE_GATTC_WRITE_OP      writeOp;       /**< Write operation */
  uint16_t                charValHandle; /**< Characteristic value handle */
  uint16_t                charValLen;    /**< Characteristic value data length */
  uint8_t                 *charValData;  /**< Characteristic value data content */
} BLE_GattcWriteParams;


/**@brief GATTC attribute database discovery event structure
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GATTC_EVENT_DBDISCOVERY, and the member evtData is point to @ref BLE_EvtGattcDbDiscovery structure.
 */
typedef struct
{
  uint8_t                 result;       /**< GATTC attribute database discovery result*/
  uint8_t                 reserve;      /**< Reserve */
  uint16_t                connHandle;   /**< Connection handle */
  union
  {
    uint32_t             reason;      /**< Indicate db discovery failed reason */
    BLE_GattcDbDiscovery dbDiscovery; /**< Discovered db information */
  } params;     /**< params union */
} BLE_EvtGattcDbDiscovery;

/**@brief GATTC read characteristic response event structure
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GATTC_EVENT_READ, and the member evtData is point to @ref BLE_EvtGattcRead structure.
 * */
typedef struct
{
  uint16_t                connHandle;    /**< Connection handle */
  uint16_t                charValHandle; /**< Characteristic value handle */
  uint16_t                charValLen;    /**< Characteristic value data length */
  uint8_t                 charValData[MAX_VAL_DATA_LENGTH];  /**< Characteristic value data content */
} BLE_EvtGattcRead;

/**@brief GATTC read characteristic response event structure
 */
typedef struct
{
  uint16_t                connHandle;    /**< Connection handle */
  uint16_t                charValHandle; /**< Characteristic value handle */
  uint8_t                 status;        /**< 0 success, other failed */
} BLE_EvtGattcWriteRsp;

/**@brief GATTC notification/indication event structure
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GATTC_EVENT_NTFIND, and the member evtData is point to @ref BLE_EvtGattcNtfInd structure.
 */
typedef struct
{
  uint16_t                connHandle;                       /**< Connection handle */
  BLE_GattNtyIndType      type;                             /**< Indication or notification */
  uint16_t                attrHandle;                       /**< Attribute value data length */
  uint16_t                attrValLen;                       /**< Attribute value data length */
  uint8_t                 attrValData[MAX_VAL_DATA_LENGTH]; /**< Attribute value data content */
} BLE_EvtGattcNtfInd;

/** @} ble_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup ble_funcs Functions
 * @{
 */

/**@brief   GATT client start attribute database discovery
 * @details This call allows the application to start attribute database discovery.The following events may be triggered: @ref  BLE_GATTC_EVENT_DBDISCOVERY, @ref BLE_GATTC_EVENT_NTFIND.
 * @param[in]  connHandle: Connection handle
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GattcStartDbDiscovery(uint16_t connHandle);

/**
 * @defgroup ble_funcs Functions
 * @{
 */

/**@brief   GATT client continue attribute database discovery
 * @details This call allows the application to start attribute database discovery.The following events may be triggered: @ref  BLE_GATTC_EVENT_DBDISCOVERY, @ref BLE_GATTC_EVENT_NTFIND.
 * @param[in]  connHandle: Connection handle
 * @param[in]  startHandle: start handle of discover
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GattcContinueDbDiscovery(uint16_t connHandle, uint16_t startHandle);

/**@brief GATT client read characteristic
 * @details This call allows the application to read a characteristic. The following events may be triggered: @ref BLE_GATTC_EVENT_READ.
 * @param[in]  connHandle: Connection handle
 * @param[in]  readParams: Parameter of read operation
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 * @note Wait a bit and retry if the return value is -EBUSY.
 */
int BLE_GattcRead(uint16_t connHandle, BLE_GattcReadParams const *readParams);

/**@brief GATT client write characteristic
 *
 * @param[in]  connHandle: Connection handle
 * @param[in]  writeParams: Parameter of write operation
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 * @note Wait a bit and retry if the return value is -EBUSY.
 */
int BLE_GattcWrite(uint16_t connHandle, BLE_GattcWriteParams const *writeParams);

/**@brief GATT client handle value confirmation of indication
 * @details This call allows the application to send confirmation of an indication.
 *
 * @param[in]  connHandle: Connection handle
 * @param[in]  attrHandle: Attribute handle
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
int BLE_GattcConfirm(uint16_t connHandle, uint16_t attrHandle);

#ifdef BLE_ENABLE_NORDIC_ORIGINAL
/**@brief GATT client register uuid128
 * @details User can get the type of uuid128(e.g. characteristic.charValUuid.type) when @BLE_GATTC_EVENT_DBDISCOVERY if registered. And user can get bytes 12 and 13(e.g. characteristic.charValUuid.value) of uuid128 when @BLE_GATTC_EVENT_DBDISCOVERY if registered.
 *
 * @param[in]  uuid128: A little endian Vendor Specific UUID disregarding bytes 12 and 13.
 * @param[out] type: Index number of the Vendor Specific UUID
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
int BLE_GattcRegisterUuid128(BLE_Uuid128* uuid128, uint8_t* type);
#endif

/** @} ble_funcs */

/** @} ble_gattc */
#endif  /* __MODULES_BLUETOOTH_HAL_NRF52_INCLUDE_BLE_BLE_GATTC_H */
