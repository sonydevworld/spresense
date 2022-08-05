/****************************************************************************
 * modules/bluetooth/hal/nrf52/ble_gattc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <ble/ble_comm.h>
#include <ble/ble_gattc.h>
#include "ble.h"
#include "ble_err.h"
#include "ble_comm_internal.h"

// #define BLE_DBGPRT_ENABLE
#ifdef BLE_DBGPRT_ENABLE
#include <stdio.h>
#define BLE_PRT printf
#else
#define BLE_PRT(...)
#endif

/******************************************************************************
 * externs
 *****************************************************************************/
extern int bleConvertErrorCode(uint32_t errCode);
extern bleCommMem commMem;
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define SRV_DISC_START_HANDLE  0x0001

/******************************************************************************
 * Private Function
 *****************************************************************************/


static int nrf52_ble_copy_uuid(BLE_Uuid *dest_uuid, BLE_UUID *src_uuid)
{
  int ret = BT_SUCCESS;

  if (!dest_uuid || !src_uuid)
    {
      BLE_PRT("UUID not allocated.");
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
        BLE_PRT("UUID not specified.");
        return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_add_service
 *
 * Description:
 *   Bluetooth LE GATT add service.
 *   Add service and get service handle ID.
 *
 ****************************************************************************/

static int nrf52_ble_add_service(struct ble_gatt_service_s *ble_gatt_service)
{
  int ret = BT_SUCCESS;
  BLE_Uuid uuid;

  ret = nrf52_ble_copy_uuid(&uuid, &ble_gatt_service->uuid);

  if (ret != BT_SUCCESS)
    {
        BLE_PRT("Copy UUID failed.");
        return BT_FAIL;
    }

  /* Add service to cypress HAL */

  ret = BLE_GattsAddService(BLE_GATTS_SRVTYP_PRIMARY, &uuid, &ble_gatt_service->handle);

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_add_characteristic
 *
 * Description:
 *   Bluetooth LE GATT add characteristic..
 *   Add characteristic to setvice and get handle ID.
 *
 ****************************************************************************/

static int nrf52_ble_add_characteristic(uint16_t serv_handle, struct ble_gatt_char_s *ble_gatt_char)
{
  int ret = BT_SUCCESS;
  BLE_CharMeta char_meta;
  BLE_GattsAttr char_value;
  
  BLE_SrvSds* srv_sds = &g_ble_context.ble_srv_sds;
  BLE_GattsCharHandles *char_handle = &(srv_sds->char_ri_handle);
  

  /* Setup char meta data */

  memset(&char_meta, 0, sizeof(BLE_CharMeta));

  memcpy(&char_meta.charPrope, &ble_gatt_char->property, sizeof(BLE_CHAR_PROP));

  /* Setup char value data */

  memset(&char_value, 0, sizeof(BLE_GattsAttr));

  /* Setup char handle */

  memset(char_handle, 0, sizeof(BLE_GattsCharHandles));

  ret = nrf52_ble_copy_uuid(&char_value.valueUuid, &ble_gatt_char->uuid);

  if (ret != BT_SUCCESS)
    {
        BLE_PRT("Copy UUID failed.");
        return BT_FAIL;
    }

  char_value.attrValue          = ble_gatt_char->value.data;
  char_value.valueLen           = ble_gatt_char->value.length;
  char_value.attrPerm.readPerm  = ble_gatt_char->value.attrPerm.readPerm;
  char_value.attrPerm.writePerm = ble_gatt_char->value.attrPerm.writePerm;

  /* Add characteristic to cypress HAL */

  ret = BLE_GattsAddCharacteristic(serv_handle, &char_meta, &char_value, char_handle);

  /* Copy handle ID to characteristic context */

  ble_gatt_char->handle = char_handle->charHandle;

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_write
 *
 * Description:
 *   Bluetooth LE GATT write response.
 *   Reply write request from client.
 *
 ****************************************************************************/

static int nrf52_ble_gatts_write(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle)
{
  int ret = BT_SUCCESS;

  /* No operation (firmware will do) */

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_read
 *
 * Description:
 *   Bluetooth LE GATT read response.
 *   Reply read request from client.
 *
 ****************************************************************************/

static int nrf52_ble_gatts_read(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle)
{
  int ret = BT_SUCCESS;

  /* No operation (firmware will do) */

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_notify
 *
 * Description:
 *   Bluetooth LE GATT notify response.
 *   Reply notify request from client.
 *
 ****************************************************************************/

static int nrf52_ble_notify(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle)
{
  int ret = BT_SUCCESS;
  BLE_GattsHandleValueNfyIndParams param = {0};
  int32_t size = (int32_t) ble_gatt_char->value.length;

  if (BLE_MAX_CHAR_SIZE >= size)
    {
      BLE_PRT("size=%ld\n",size);
      BLE_PRT("ble_gatt_char->handle=%d\n", ble_gatt_char->handle);
      BLE_PRT("ble_gatt_char->value.data=%.*s\n", size, ble_gatt_char->value.data);
      param.type = BLE_GATT_NOTIFICATION;
      param.attrHandle = ble_gatt_char->handle;
      param.attrValData = ble_gatt_char->value.data;
      param.attrValLen = size;

      ret = BLE_GattsHandleValueNfyInd(handle, &param);

      if (ret)
        {
          BLE_PRT("gatts handle value nfy/ind failed, ret=%d\n", ret);
        }
    }
  else
    {
      BLE_PRT("outcoming data exceeds buffer length\n");
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_start_db_discovery
 *
 * Description:
 *   Bluetooth LE GATT client start attribute database discovery
 *
 ****************************************************************************/

static int nrf52_ble_start_db_discovery(uint16_t conn_handle)
{
  return BLE_GattcStartDbDiscovery(conn_handle);
}

/****************************************************************************
 * Name: nrf52_ble_continue_db_discovery
 *
 * Description:
 *   Bluetooth LE GATT client continue attribute database discovery
 *   Continue discovery from start_handle
 *
 ****************************************************************************/

static int nrf52_ble_continue_db_discovery(uint16_t start_handle, uint16_t conn_handle)
{
  return BLE_GattcContinueDbDiscovery(conn_handle, start_handle);
}

/****************************************************************************
 * Name: nrf52_ble_write
 *
 * Description:
 *   Bluetooth LE GATT client write request.
 *   Request GATT server to write attributes.
 *
 ****************************************************************************/

static int nrf52_ble_gattc_write(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle)
{
  BLE_GattcWriteParams param = {0};

  param.writeOp       = (ble_gatt_char->property.writeWoResp ? BLE_GATTC_WRITE_CMD : BLE_GATTC_WRITE_REQ);
  param.charValHandle = ble_gatt_char->handle;
  param.charValLen    = ble_gatt_char->value.length;
  param.charValData   = ble_gatt_char->value.data;

  return BLE_GattcWrite(handle, &param);
}

/****************************************************************************
 * Name: nrf52_ble_read
 *
 * Description:
 *   Bluetooth LE GATT client read request.
 *   Request GATT server to read attributes.
 *
 ****************************************************************************/

static int nrf52_ble_gattc_read(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle)
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
  .gatts.addService          = nrf52_ble_add_service,
  .gatts.addChar             = nrf52_ble_add_characteristic,
  .gatts.write               = nrf52_ble_gatts_write,
  .gatts.read                = nrf52_ble_gatts_read,
  .gatts.notify              = nrf52_ble_notify,
  .gattc.startDbDiscovery    = nrf52_ble_start_db_discovery,
  .gattc.continueDbDiscovery = nrf52_ble_continue_db_discovery,
  .gattc.write               = nrf52_ble_gattc_write,
  .gattc.read                = nrf52_ble_gattc_read
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/


int BLE_GattcStartDbDiscovery(uint16_t connHandle)
{
  int ret      = BLE_SUCCESS;
  int errCode = 0;

  errCode = sd_ble_gattc_primary_services_discover(connHandle,SRV_DISC_START_HANDLE,NULL);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

int BLE_GattcContinueDbDiscovery(uint16_t connHandle, uint16_t startHandle)
{
  int ret      = BLE_SUCCESS;
  int errCode = 0;

  errCode = sd_ble_gattc_primary_services_discover(connHandle,startHandle,NULL);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

int BLE_GattcRead(uint16_t connHandle, BLE_GattcReadParams const *readParams)
{
#define BLE_GATTC_READ_BUSY_CHECK_MAX 5
  int ret      = BLE_SUCCESS;
  int errCode = 0;

  if (readParams == NULL) {
    return -EINVAL;
  }

  for (int i=0; i<BLE_GATTC_READ_BUSY_CHECK_MAX; i++) {
    errCode = sd_ble_gattc_read(connHandle, readParams->charValHandle, 0);
    if (errCode != NRF_ERROR_BUSY) {
      ret = bleConvertErrorCode((uint32_t)errCode);
      return ret;
    }
    int msec = (commMem.gapMem->connParams.max_conn_interval * 1250 + (1000 - 1)) / 1000;
    BLE_PRT("BLE_GattcRead: delay %d\n", msec);
    usleep(msec * 1000);
  }
  return -EBUSY;
}

int BLE_GattcWrite(uint16_t connHandle, BLE_GattcWriteParams const *writeParams)
{
#define BLE_GATTC_TX_BUFFER_CHECK_MAX 5
  int ret      = BLE_SUCCESS;
  int errCode = 0;
  ble_gattc_write_params_t gattcWriteParams = {0};

  if (writeParams == NULL) {
    return -EINVAL;
  }

  memset(&gattcWriteParams,0,sizeof(gattcWriteParams));
  gattcWriteParams.handle   = writeParams->charValHandle;
  gattcWriteParams.len      = writeParams->charValLen;
  gattcWriteParams.p_value  = writeParams->charValData;
  gattcWriteParams.offset   = 0;
  if (writeParams->writeOp == BLE_GATTC_WRITE_CMD) {
    gattcWriteParams.write_op = BLE_GATT_OP_WRITE_CMD;
  }
  else if (writeParams->writeOp == BLE_GATTC_WRITE_REQ) {
    gattcWriteParams.write_op = BLE_GATT_OP_WRITE_REQ;
  }
  else {
    return -EINVAL;
  }
  gattcWriteParams.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_CANCEL;
  for (int i=0; i<BLE_GATTC_TX_BUFFER_CHECK_MAX; i++) {
    errCode = sd_ble_gattc_write(connHandle, &gattcWriteParams);
    if (errCode != NRF_ERROR_RESOURCES) {
      ret = bleConvertErrorCode((uint32_t)errCode);
      return ret;
    }
    int msec = (commMem.gapMem->connParams.max_conn_interval * 1250 + (1000 - 1)) / 1000;
    BLE_PRT("BLE_GattcWrite: delay %d\n", msec);
    usleep(msec * 1000);
  }
  return -EBUSY;

}

int BLE_GattcConfirm(uint16_t connHandle, uint16_t attrHandle)
{
  int ret      = BLE_SUCCESS;
  int errCode = 0;

  errCode = sd_ble_gattc_hv_confirm(connHandle, attrHandle);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

#ifdef BLE_ENABLE_NORDIC_ORIGINAL
int BLE_GattcRegisterUuid128(BLE_Uuid128* uuidBase, uint8_t* type)
{
  int ret      = BLE_SUCCESS;
  int errCode = 0;
  *type = BLE_UUID_TYPE_VENDOR_BEGIN;
  errCode  = sd_ble_uuid_vs_add((ble_uuid128_t *)uuidBase, type);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}
#endif
