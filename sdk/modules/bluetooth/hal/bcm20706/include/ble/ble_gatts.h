/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/ble/ble_gatts.h
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
 * @file       ble_gatts.h
 */

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BLE_BLE_GATTS_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BLE_BLE_GATTS_H

/**
 * @defgroup BLE Bluetooth LE GATT server
 *
 * #include <ble/ble_gatts.h>
 *
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup ble_defs Defines
 * @{
 */

#define UUID_128BIT_LENGTH  16 /**< 16 bytes of 128 bit UUID length */
#ifndef CONFIG_NRF51822
#define MAX_RCV_DATA_LENGTH 512 /**< Max receive data length. */
#else
#define MAX_RCV_DATA_LENGTH 20  /**< Max receive data length. */
#endif

/** @} ble_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup ble_datatypes Data types
 * @{
 */

/**@brief Gatts write event structure
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GATTS_EVENT_WRITE, and the member evtData is point to @ref BLE_EvtGattsWrite structure.
 */
typedef struct
{
   uint16_t   connHandle;                     /**< Connection handle */
   uint16_t   handle;                         /**< Attribute handle */
   uint16_t   offset;                         /**< Offset for the write operation */
   uint16_t   dataLen;                        /**< Length of the received data */
   uint8_t    reserve[2];                     /**< Reserve */
   uint8_t    data[MAX_RCV_DATA_LENGTH];      /**< Received data */
} BLE_EvtGattsWrite;

/**@brief Indication confirm event
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GATTS_EVENT_CFM, and the member evtData is point to @ref BLE_EvtGattsIndConfirm structure.
 */
typedef struct
{
  uint16_t    connHandle; /**< Connection handle */
  uint16_t    handle;     /**< Attribute handle */
} BLE_EvtGattsIndConfirm;

/**@brief BLE UUID structure
 */
typedef struct
{
  BLE_GATT_UUID_TYPE type;      /**< UUID type */
  union
  {
    BLE_UUID128   uuid128;   /**< 128-bit UUID */
    BLE_UUID_ALIAS baseAlias; /**< UUID base alias */
  }value;   /**< value union */
} BLE_Uuid;

/**@brief Characteristic meta datastructure
 */
typedef struct
{
  BLE_CHAR_PROP                 charPrope; /**< Characteristic Properties */
  uint8_t                         reserve; /**< Reserve */
  uint16_t               charUserDescSize; /**< The size of the user description */
  uint8_t                   *charUserDesc; /**< Pointer to a UTF-8 encoded string */
  BLE_ATTR_PERM                userDescDpr; /**< User Description descriptor, not supported now */
  BLE_ATTR_PERM           clientCharCfgDpr; /**< Client Characteristic Configuration Descriptor */
  BLE_ATTR_PERM           serverCharCfgDpr; /**< Server Characteristic Configuration Descriptor, not supported now */
} BLE_CharMeta;

/**@brief the GATTS descriptor structure
 */
typedef struct
{
  uint16_t                userDescHandle; /**< Handle to the User Description descriptor */
  uint16_t                    cccdHandle; /**< Handle to the Client Characteristic Configuration Descriptor */
  uint16_t                    sccdHandle; /**< Handle to the Server Characteristic Configuration Descriptor */
} BLE_GattsDprHandles;

/**@brief GATT Characteristic Definition Handles
 */
typedef struct
{
  uint16_t                  charHandle; /**< Handle to the characteristic value */
  BLE_GattsDprHandles       dprHandle; /**< Handle to the descriptor structure belongs to characteristic */
} BLE_GattsCharHandles;

/**@brief characteristic value attribute structure
 */
typedef struct BLE_GattsAttr_t
{
  BLE_Uuid                 valueUuid; /**< Attribute value UUID */
  BLE_ATTR_PERM              attrPerm; /**< Attribute permission structure */
  uint8_t                 *attrValue; /**< Pointer to attribute value */
  uint16_t                  valueLen; /**< Attribute value length */
} BLE_GattsAttr;

/**@brief Notification or Indication Type
 * @sa @ref BLE_EvtGattcNtfInd
 */
typedef enum
{
  BLE_GATT_NOTIFICATION = 0x01, /**< GATT type notification */
  BLE_GATT_INDICATION,          /**< GATT type indication */
} BLE_GattNtyIndType;

/**@brief GATTS handle value notification or indication parameters structure
 */
typedef struct
{
  BLE_GattNtyIndType      type;         /**< Indication or notification */
  uint8_t                 reserve;      /**< Reserve */
  uint8_t                 *attrValData; /**< Attribute value data content */
  uint16_t                attrValLen;   /**< Attribute value length in bytes */
  uint16_t                attrHandle;   /**< Attribute handle */
} BLE_GattsHandleValueNfyIndParams;


/**@brief GATTS Attribute value parameters structure
 */
typedef struct
{
  uint16_t  connHandle;   /**< Connection handle */
  uint16_t  attrHandle;   /**< Attribute handle */
  uint16_t  attrValLen;   /**< Attribute value length */
  uint16_t  reserve;      /**< Reserve */
  uint8_t   *attrValData; /**< Attribute value data */
} BLE_GattsAttrValueParam;

/** @} ble_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup ble_funcs Functions
 * @{
 */

/**@brief GATT server add service
 * @details Add a service declaration to the attribute table. There is not a maximum number of services. The number is varied depending on number of attributes in each service and its size. So user could add services until space runs out and no more can be added.
 *
 * @param[in]  type: Primary or secondary service
 * @param[in]  uuid: Pointer to service UUID
 * @param[out]  serviceHandle: Pointer to a 16-bit word where the assigned handle will be stored
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
int BLE_GattsAddService(BLE_GATT_TYPE type, BLE_Uuid const* uuid, uint16_t* serviceHandle);

/**@brief GATT server add characteristic
 * @details Add a characteristic declaration. This call allows the application to add a characteristic declaration, a characteristic value declaration and option characteristic descriptor declarations to the attribute table. There is not a maximum number of characteristic, it depends on the available resources.
 *
 * @param[in]  serviceHandle: Service handle that the characteristic to be added belongs to
 * @param[in]  charMeta: Characteristic meta data
 * @param[in]  charValue: Characteristic value structure
 * @param[out]  handles: Handles of added characteristic
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
int BLE_GattsAddCharacteristic(uint16_t serviceHandle, BLE_CharMeta const* charMeta, BLE_GattsAttr const* charValue, BLE_GattsCharHandles* handles);

/**@brief GATT server handle value notification/indication
 * @details Handle an attribute value notification or indication.
 *
 * @param[in]  connHandle: Connection handle
 * @param[in]  handleValueNfyInd: Characteristic meta data pointer to an handle value notify or indicate parameters structure
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GattsHandleValueNfyInd(uint16_t connHandle, BLE_GattsHandleValueNfyIndParams const* handleValueNfyInd);

/**@brief GATT server update attribute value
 * @details This call allows the application to update an attribute value. for example, if you want to update your characteristic value(an attribute), you can use this call with attribute handle set to characteristic value handle.
 *
 * @param[in]  attrValueParam: Attribute value, must be existing connHandle and attrHandle.
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GattsUpdateAttrValue(BLE_GattsAttrValueParam *attrValueParam);

/** @} ble_funcs */

/** @} ble_gatts */
#endif  /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BLE_BLE_GATTS_H */
