/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bcm20706_ble_gatts.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <ble/ble_comm.h>
#include <ble/ble_gatts.h>
#include <ble/ble_gattc.h>

#include "bt_util.h"
#include "bcm20706_ble_internal.h"
#include "manager/bt_uart_manager.h"
#include "bt_debug.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DBG_LOG_DEBUG btdbg
#define DBG_LOG_ERROR btdbg

#define BLE_UUID_TYPE_UNKNOWN      0x00
#define BLE_UUID_TYPE_BLE          0x01
#define BLE_UUID_TYPE_VENDOR_BEGIN 0x02

#define BLE_SERV_PACKET_DATA_LEN        22
#define BLE_CHAR_PACKET_UUID_DATA_LEN   25
#define GATTS_HANDLE_SERV_OFFSET        6
#define GATTS_HANDLE_CHAR_OFFSET        5
#define GATTS_HANDLE_CHAR_VALUE_OFFSET  1
#define GATTS_HANDLE_DESP_CCC_OFFSET    2
#define GATTS_HANDLE_DESP_SCC_OFFSET    3
#define GATTS_HANDLE_DESP_DESC_OFFSET   4

#define CONN_HANDLE_INVALED 0xffff

#define SRV_DISC_START_HANDLE  0x0001
#define SRV_DISC_END_HANDLE    0xFFFF

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t gatts_handle = 0x31;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bleAddService(uint8_t type, ble_uuid_t* uuidBaseAlias, ble_uuid128_t *uuid128, uint16_t* serviceHandle )
{
  uint8_t buff[BT_MID_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_ADD_SERVICE);
  UINT16_TO_STREAM(p, BLE_SERV_PACKET_DATA_LEN);
  UINT16_TO_STREAM(p, *serviceHandle);
  UINT8_TO_STREAM(p, type);
  UINT8_TO_STREAM(p, uuidBaseAlias->type);
  UINT16_TO_STREAM(p, uuidBaseAlias->uuid);
  memcpy(p, uuid128, BLE_UUID128_LEN);
  p += BLE_UUID128_LEN;
  return btUartSendData(buff, p - buff);
}

static int bleAddCharacteristic(ble_uuid_t* uuidBaseAlias, ble_uuid128_t *uuid128, ble_char_t* bleChar, ble_attr_t* bleAttr)
{
  uint8_t buff[BT_LONG_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_ADD_CHAR);
  UINT16_TO_STREAM(p, BLE_CHAR_PACKET_UUID_DATA_LEN + bleAttr->attrLen);
  UINT8_TO_STREAM(p, uuidBaseAlias->type);
  UINT16_TO_STREAM(p, uuidBaseAlias->uuid);
  memcpy(p, uuid128, BLE_UUID128_LEN);
  p += BLE_UUID128_LEN;
  UINT16_TO_STREAM(p, bleChar->charHandle);
  UINT8_TO_STREAM(p, bleChar->charProperties);
  UINT8_TO_STREAM(p, bleChar->charPermission);
  UINT16_TO_STREAM(p, bleAttr->attrLen);
  memcpy(p, bleAttr->attrValue, bleAttr->attrLen);
  p += bleAttr->attrLen;
  return btUartSendData(buff, p - buff);
}

static int bleGattsHandleValueNfyInd(uint16_t connHandle, BLE_GattsHandleValueNfyIndParams const* handleValueNfyInd)
{
  uint8_t buff[BT_LONG_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  if (handleValueNfyInd->type == BLE_GATT_NOTIFICATION)
    {
      UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_NOTIFY);
    }
  else
    {
      UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_INDICATE);
    }
  UINT16_TO_STREAM(p, BLE_HANDLE_LEN * 2 + handleValueNfyInd->attrValLen);
  UINT16_TO_STREAM(p, connHandle);
  UINT16_TO_STREAM(p, handleValueNfyInd->attrHandle);
  memcpy(p, handleValueNfyInd->attrValData, handleValueNfyInd->attrValLen);
  p += handleValueNfyInd->attrValLen;
  return btUartSendData(buff, p - buff);
}

static int bcm20706_ble_copy_uuid(BLE_Uuid *dest_uuid, BLE_UUID *src_uuid)
{
  int ret = BT_SUCCESS;

  if (!dest_uuid || !src_uuid)
    {
      DBG_LOG_DEBUG("UUID not allocated.");
      return BT_FAIL;
    }

  /* Create UUID for cypress HAL */

  dest_uuid->type = src_uuid->type;

  switch (dest_uuid->type)
    {
      case BLE_UUID_TYPE_BASEALIAS_BTSIG:
      case BLE_UUID_TYPE_BASEALIAS_VENDOR:
        memcpy(&dest_uuid->value.baseAlias, &src_uuid->value.alias, sizeof(BLE_UUID_ALIAS));
        break;

      case BLE_UUID_TYPE_UUID128:
        memcpy(&dest_uuid->value.uuid128, &src_uuid->value.uuid128, sizeof(BLE_UUID128));
        break;

      default:
        DBG_LOG_DEBUG("UUID not specified.");
        return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_add_service
 *
 * Description:
 *   Bluetooth LE GATT add service.
 *   Add service and get service handle ID.
 *
 ****************************************************************************/

static int bcm20706_ble_add_service(struct ble_gatt_service_s *ble_gatt_service)
{
  int ret = BT_SUCCESS;
  BLE_Uuid uuid;

  ret = bcm20706_ble_copy_uuid(&uuid, &ble_gatt_service->uuid);

  if (ret != BT_SUCCESS)
    {
        DBG_LOG_DEBUG("Copy UUID failed.");
        return BT_FAIL;
    }

  /* Add service to cypress HAL */

  ret = BLE_GattsAddService(BLE_GATTS_SRVTYP_PRIMARY, &uuid, &ble_gatt_service->handle);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_add_characteristic
 *
 * Description:
 *   Bluetooth LE GATT add characteristic..
 *   Add characteristic to setvice and get handle ID.
 *
 ****************************************************************************/

static int bcm20706_ble_add_characteristic(uint16_t serv_handle, struct ble_gatt_char_s *ble_gatt_char)
{
  int ret = BT_SUCCESS;
  BLE_CharMeta char_meta;
  BLE_GattsAttr char_value;
  BLE_GattsCharHandles char_handle;

  /* Setup char meta data */

  memset(&char_meta, 0, sizeof(BLE_CharMeta));

  memcpy(&char_meta.charPrope, &ble_gatt_char->property, sizeof(BLE_CHAR_PROP));

  /* Setup char value data */

  memset(&char_value, 0, sizeof(BLE_GattsAttr));

  /* Setup char handle */

  memset(&char_handle, 0, sizeof(BLE_GattsCharHandles));

  ret = bcm20706_ble_copy_uuid(&char_value.valueUuid, &ble_gatt_char->uuid);

  if (ret != BT_SUCCESS)
    {
        DBG_LOG_DEBUG("Copy UUID failed.");
        return BT_FAIL;
    }

  char_value.attrValue          = ble_gatt_char->value.data;
  char_value.valueLen           = ble_gatt_char->value.length;
  char_value.attrPerm.readPerm  = ble_gatt_char->value.attrPerm.readPerm;
  char_value.attrPerm.writePerm = ble_gatt_char->value.attrPerm.writePerm;

  /* Add characteristic to cypress HAL */

  ret = BLE_GattsAddCharacteristic(serv_handle, &char_meta, &char_value, &char_handle);

  /* Copy handle ID to characteristic context */

  ble_gatt_char->handle = char_handle.charHandle;

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_write
 *
 * Description:
 *   Bluetooth LE GATT write response.
 *   Reply write request from client.
 *
 ****************************************************************************/

static int bcm20706_ble_gatts_write(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle)
{
  int ret = BT_SUCCESS;

  /* No operation (firmware will do) */

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_read
 *
 * Description:
 *   Bluetooth LE GATT read response.
 *   Reply read request from client.
 *
 ****************************************************************************/

static int bcm20706_ble_gatts_read(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle)
{
  int ret = BT_SUCCESS;

  /* No operation (firmware will do) */

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_notify
 *
 * Description:
 *   Bluetooth LE GATT notify response.
 *   Reply notify request from client.
 *
 ****************************************************************************/

static int bcm20706_ble_notify(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle)
{
  int ret = BT_SUCCESS;
  BLE_GattsHandleValueNfyIndParams param = {0};
  int32_t size = (int32_t) ble_gatt_char->value.length;

  if (BLE_MAX_CHAR_SIZE >= size)
    {
      param.type = BLE_GATT_NOTIFICATION;
      param.attrHandle = ble_gatt_char->handle;
      param.attrValData = ble_gatt_char->value.data;
      param.attrValLen = size;

      ret = BLE_GattsHandleValueNfyInd(handle, &param);

      if (ret)
        {
          DBG_LOG_DEBUG("gatts handle value nfy/ind failed, ret=%d\n", ret);
        }
    }
  else
    {
      DBG_LOG_DEBUG("outcoming data exceeds buffer length\n");
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_start_db_discovery
 *
 * Description:
 *   Bluetooth LE GATT client start attribute database discovery
 *
 ****************************************************************************/

static int bcm20706_ble_start_db_discovery(uint16_t conn_handle)
{
  return BLE_GattcStartDbDiscovery(conn_handle);
}

/****************************************************************************
 * Name: bcm20706_ble_continue_db_discovery
 *
 * Description:
 *   Bluetooth LE GATT client continue attribute database discovery
 *   Continue discovery from start_handle
 *
 ****************************************************************************/

static int bcm20706_ble_continue_db_discovery(uint16_t start_handle, uint16_t conn_handle)
{
  return BLE_GattcContinueDbDiscovery(conn_handle, start_handle);
}

/****************************************************************************
 * Name: bcm20706_ble_write
 *
 * Description:
 *   Bluetooth LE GATT client write request.
 *   Request GATT server to write attributes.
 *
 ****************************************************************************/

static int bcm20706_ble_gattc_write(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle)
{
  BLE_GattcWriteParams param = {0};

  param.writeOp       = (ble_gatt_char->property.writeWoResp ? BLE_GATTC_WRITE_CMD : BLE_GATTC_WRITE_REQ);
  param.charValHandle = ble_gatt_char->handle;
  param.charValLen    = ble_gatt_char->value.length;
  param.charValData   = ble_gatt_char->value.data;

  return BLE_GattcWrite(handle, &param);
}

/****************************************************************************
 * Name: bcm20706_ble_read
 *
 * Description:
 *   Bluetooth LE GATT client read request.
 *   Request GATT server to read attributes.
 *
 ****************************************************************************/

static int bcm20706_ble_gattc_read(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle)
{
  BLE_GattcReadParams param = {0};

  param.charValHandle = ble_gatt_char->handle;

  return BLE_GattcRead(handle, &param);
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ble_hal_gatt_ops_s ble_hal_gatt_ops =
{
  .gatts.addService          = bcm20706_ble_add_service,
  .gatts.addChar             = bcm20706_ble_add_characteristic,
  .gatts.write               = bcm20706_ble_gatts_write,
  .gatts.read                = bcm20706_ble_gatts_read,
  .gatts.notify              = bcm20706_ble_notify,
  .gattc.startDbDiscovery    = bcm20706_ble_start_db_discovery,
  .gattc.continueDbDiscovery = bcm20706_ble_continue_db_discovery,
  .gattc.write               = bcm20706_ble_gattc_write,
  .gattc.read                = bcm20706_ble_gattc_read
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int BLE_GattsAddService(BLE_GATT_TYPE type, BLE_Uuid const* uuid, uint16_t* serviceHandle)
{
  ble_uuid_t        uuidBaseAlias = {0};
  ble_uuid128_t     uuid128Ble    = {{0}};
  uint16_t          temp          = 0;

  if(uuid == NULL)
    {
      return -EINVAL;
    }
  memset(&uuidBaseAlias, 0, sizeof(ble_uuid_t));
  switch( uuid->type )
    {
      case BLE_UUID_TYPE_BASEALIAS_BTSIG:
          uuidBaseAlias.type = BLE_UUID_TYPE_BLE;
          uuidBaseAlias.uuid = uuid->value.baseAlias.uuidAlias;
          break;

      case BLE_UUID_TYPE_BASEALIAS_VENDOR:
          uuidBaseAlias.type = BLE_UUID_TYPE_VENDOR_BEGIN;
          uuidBaseAlias.uuid = uuid->value.baseAlias.uuidAlias;
          memcpy(uuid128Ble.uuid128, (ble_uuid128_t*)&uuid->value.baseAlias.uuidBase, 16);
          temp =  uuidBaseAlias.uuid;
          uuid128Ble.uuid128[BLE_UUID128_13_BYTE] = (uint8_t)(temp&0xff);
          uuid128Ble.uuid128[BLE_UUID128_14_BYTE] = (uint8_t)(temp>>8);
          break;

      case BLE_UUID_TYPE_UUID128:
          uuidBaseAlias.type = BLE_UUID_TYPE_VENDOR_BEGIN;
          /* octecs 12-13 of 128-bit UUID,16bits */
          memcpy(uuid128Ble.uuid128, (ble_uuid128_t*)&uuid->value.uuid128.uuid128, 16);
          break;

      default:
        break;
    }

  gatts_handle += GATTS_HANDLE_SERV_OFFSET;
  *serviceHandle = gatts_handle;
  return bleAddService(type, &uuidBaseAlias, &uuid128Ble, serviceHandle);
}

int BLE_GattsAddCharacteristic(uint16_t serviceHandle, BLE_CharMeta const* charMeta,
    BLE_GattsAttr const* attrCharValue, BLE_GattsCharHandles* handles)
{
  ble_uuid_t        uuidBaseAlias = {0};
  ble_uuid128_t     uuid128Ble = {{0}};
  ble_char_t bleChar = {0};
  ble_attr_t bleAttr = {0};
  uint16_t temp = 0;

  bleChar.charHandle = handles->charHandle;
  memcpy(&bleChar.charProperties, &charMeta->charPrope, 1);
  bleChar.charPermission = 0;
  if((attrCharValue->attrPerm.readPerm == BLE_SEC_MODE_NO_ACCESS)
      || (attrCharValue->attrPerm.writePerm == BLE_SEC_MODE_NO_ACCESS))
    {
      bleChar.charPermission |= LEGATTDB_PERM_NONE;
    }
  if (attrCharValue->attrPerm.readPerm == BLE_SEC_MODE1LV1_NO_SEC)
    {
      bleChar.charPermission |= LEGATTDB_PERM_READABLE;
      btdbg("permission1 = %x\n", bleChar.charPermission);
    }
  if ((attrCharValue->attrPerm.readPerm == BLE_SEC_MODE1LV3_MITM_ENC)
      || (attrCharValue->attrPerm.readPerm == BLE_SEC_MODE1LV2_NO_MITM_ENC))
    {
      bleChar.charPermission |= (LEGATTDB_PERM_READABLE | LEGATTDB_PERM_AUTH_READABLE);
    }

  if (attrCharValue->attrPerm.writePerm == BLE_SEC_MODE1LV1_NO_SEC)
    {
      bleChar.charPermission |= (LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ);
    }

  if ((attrCharValue->attrPerm.writePerm == BLE_SEC_MODE1LV3_MITM_ENC)
      || (attrCharValue->attrPerm.writePerm == BLE_SEC_MODE1LV2_NO_MITM_ENC))
    {
      bleChar.charPermission |=
        (LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ |LEGATTDB_PERM_AUTH_WRITABLE);
      btdbg("auth write enable\n");
    }

  btdbg("permission2 = %x\n", bleChar.charPermission);
  bleAttr.charHandle = handles->charHandle;
  bleAttr.attrLen = attrCharValue->valueLen;
  bleAttr.attrValue = attrCharValue->attrValue;

  switch (attrCharValue->valueUuid.type)
    {
      case BLE_UUID_TYPE_BASEALIAS_BTSIG:
          uuidBaseAlias.type = BLE_UUID_TYPE_BLE;
          uuidBaseAlias.uuid = attrCharValue->valueUuid.value.baseAlias.uuidAlias;
          break;

      case BLE_UUID_TYPE_BASEALIAS_VENDOR:
          uuidBaseAlias.type = BLE_UUID_TYPE_VENDOR_BEGIN;
          uuidBaseAlias.uuid = attrCharValue->valueUuid.value.baseAlias.uuidAlias;
          memcpy(uuid128Ble.uuid128, (ble_uuid128_t*)&attrCharValue->valueUuid.value.baseAlias.uuidBase, BLE_UUID128_LEN);
          temp = uuidBaseAlias.uuid;
          uuid128Ble.uuid128[BLE_UUID128_13_BYTE] = (uint8_t)(temp & 0xff);
          uuid128Ble.uuid128[BLE_UUID128_14_BYTE] = (uint8_t)(temp >> 8);
          break;

      case BLE_UUID_TYPE_UUID128:
          uuidBaseAlias.type = BLE_UUID_TYPE_VENDOR_BEGIN;
          memcpy(uuid128Ble.uuid128, (ble_uuid128_t*)&attrCharValue->valueUuid.value.uuid128.uuid128, BLE_UUID128_LEN);
          break;

      default:
        break;
    }
  gatts_handle += GATTS_HANDLE_CHAR_OFFSET;
  handles->charHandle = gatts_handle + GATTS_HANDLE_CHAR_VALUE_OFFSET;
  handles->dprHandle.cccdHandle     = gatts_handle + GATTS_HANDLE_DESP_CCC_OFFSET;
  handles->dprHandle.sccdHandle     = gatts_handle + GATTS_HANDLE_DESP_SCC_OFFSET;
  handles->dprHandle.userDescHandle = gatts_handle + GATTS_HANDLE_DESP_DESC_OFFSET;

  return bleAddCharacteristic(&uuidBaseAlias, &uuid128Ble, &bleChar, &bleAttr);
}

int BLE_GattsUpdateAttrValue(BLE_GattsAttrValueParam *attrValueParam)
{
  ble_gatts_value_t gattsValue = {0};

  if( attrValueParam == NULL)
    {
      return -EINVAL;
    }

  memset(&gattsValue, 0, sizeof(gattsValue));
  gattsValue.len      = attrValueParam->attrValLen;
  gattsValue.p_value  = attrValueParam->attrValData;

  return bleGattsUpdateAttrValue(attrValueParam->connHandle, attrValueParam->attrHandle, &gattsValue);
}

int BLE_GattsHandleValueNfyInd(uint16_t connHandle, BLE_GattsHandleValueNfyIndParams const* handleValueNfyInd)
{
  if ( handleValueNfyInd == NULL)
    {
      return -EINVAL;
    }
  return bleGattsHandleValueNfyInd(connHandle, handleValueNfyInd);
}

int bleGattsUpdateAttrValue(uint16_t connHandle, uint16_t attrHandle, ble_gatts_value_t* gattsValue)
{
  uint8_t buff[BT_LONG_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_UPDATE_ATTR);
  UINT16_TO_STREAM(p, BLE_HANDLE_LEN * 3 + gattsValue->len);
  UINT16_TO_STREAM(p, connHandle);
  UINT16_TO_STREAM(p, attrHandle);
  UINT16_TO_STREAM(p, gattsValue->len);
  memcpy(p, gattsValue->p_value, gattsValue->len);
  p += gattsValue->len;
  return btUartSendData(buff, p - buff);
}

int bleServiceDiscover(uint16_t connHandle, uint16_t startHandle, uint16_t endHandle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_DISCOVER_SERVICES);
  UINT16_TO_STREAM(p, 6);
  UINT16_TO_STREAM(p, connHandle);
  UINT16_TO_STREAM(p, startHandle);
  UINT16_TO_STREAM(p, endHandle);
  return btUartSendData(buff, p - buff);
}

int bleCharDiscover(uint16_t connHandle, uint16_t startHandle, uint16_t endHandle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_DISCOVER_CHARACTERISTICS);
  UINT16_TO_STREAM(p, 6);
  UINT16_TO_STREAM(p, connHandle);
  UINT16_TO_STREAM(p, startHandle);
  UINT16_TO_STREAM(p, endHandle);
  return btUartSendData(buff, p - buff);
}

int bleDescriptorsDiscover(uint16_t connHandle, uint16_t startHandle, uint16_t endHandle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_DISCOVER_DESCRIPTORS);
  UINT16_TO_STREAM(p, 6);
  UINT16_TO_STREAM(p, connHandle);
  UINT16_TO_STREAM(p, startHandle);
  UINT16_TO_STREAM(p, endHandle);
  return btUartSendData(buff, p - buff);
}

int bleGattcReadResponse(uint16_t connHandle, uint16_t attrHandle)
{
  return 0;
}

int bleGattcRead(uint16_t connHandle, BLE_GattcReadParams const *readParams)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_READ_REQUEST);
  UINT16_TO_STREAM(p, 4);
  UINT16_TO_STREAM(p, connHandle);
  UINT16_TO_STREAM(p, readParams->charValHandle);
  btdbg("read connHandle = %x,val handle = %x\n",connHandle,readParams->charValHandle);
  return btUartSendData(buff, p - buff);
}

int bleGattcWriteResponse(uint16_t connHandle, uint16_t attrHandle)
{
  return 0;
}

int bleGattcWrite(uint16_t connHandle, BLE_GattcWriteParams const *writeParams)
{
  uint8_t buff[BT_LONG_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  if (writeParams->writeOp == BLE_GATTC_WRITE_CMD)
    {
      UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_WRITE_COMMAND);
    }
  else if (writeParams->writeOp == BLE_GATTC_WRITE_REQ)
    {
      UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_WRITE_REQUEST);
    }
  else
    {
      return -EINVAL;
    }
  UINT16_TO_STREAM(p, 4 + writeParams->charValLen);
  UINT16_TO_STREAM(p, connHandle);
  UINT16_TO_STREAM(p, writeParams->charValHandle);
  memcpy(p, writeParams->charValData, writeParams->charValLen);
  p += writeParams->charValLen;
  btdbg("write connHandle = %x,val handle = %x, len = %x\n",
        connHandle,writeParams->charValHandle, writeParams->charValLen);
  btdbg("write data[0] = %x,data[1] = %x\n",
        writeParams->charValData[0],writeParams->charValData[1]);
  return btUartSendData(buff, p - buff);
}

int bleGattcConfirm(uint16_t connHandle, uint16_t attrHandle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_INDICATE_CONFIRM);
  UINT16_TO_STREAM(p, 4);
  UINT16_TO_STREAM(p, connHandle);
  UINT16_TO_STREAM(p, attrHandle);
  return btUartSendData(buff, p - buff);
}

int BLE_GattcStartDbDiscovery(uint16_t connHandle)
{
  return bleServiceDiscover(connHandle, SRV_DISC_START_HANDLE, SRV_DISC_END_HANDLE);
}

int BLE_GattcContinueDbDiscovery(uint16_t connHandle, uint16_t startHandle)
{
  return bleServiceDiscover(connHandle, startHandle, SRV_DISC_END_HANDLE);
}

int BLE_GattcRead(uint16_t connHandle, BLE_GattcReadParams const *readParams)
{
  return bleGattcRead(connHandle, readParams);
}

int BLE_GattcWrite(uint16_t connHandle, BLE_GattcWriteParams const *writeParams)
{
  return bleGattcWrite(connHandle, writeParams);
}

int BLE_GattcConfirm(uint16_t connHandle, uint16_t attrHandle)
{
  return bleGattcConfirm(connHandle, attrHandle);
}

