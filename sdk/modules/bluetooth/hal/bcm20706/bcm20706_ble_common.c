/****************************************************************************
 * modules/bluetooth/hal/bcm20706/manager/bt_uart_manager.c
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
#include <unistd.h>
#include <debug.h>
#include <ble/ble_comm.h>
#include <ble/ble_gap.h>
#include <ble/ble_gatts.h>
#include <ble/ble_gattc.h>

#include "manager/bt_storage_manager.h"
#include "manager/bt_uart_manager.h"
#include "bcm20706_ble_internal.h"
#include "bcm20706_bt_internal.h"
#include "bt_util.h"
#include "bt_debug.h"

/******************************************************************************
 * externs
 *****************************************************************************/

extern bleGapMem *bleGetGapMem(void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Include or not the service_changed characteristic */

#define DBG_LOG_DEBUG btdbg
#define DBG_LOG_ERROR btdbg

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0
#define BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG  0x2902
#define BLE_GATTC_HANDLE_END     0xFFFF
#define BLE_GATT_HANDLE_INVALID  0x0000
#define BLE_ADV_RSSI_LEN  1
#define BLE_SERV_UUID_BTSIG_PACKET_LEN    8
#define BLE_SERV_UUID_VENDOR_PACKET_LEN   22
#define BLE_CHAR_UUID_BTSIG_PACKET_LEN    9
#define BLE_CHAR_UUID_VENDOR_PACKET_LEN   23
#define BLE_DESP_UUID_BTSIG_PACKET_LEN    6
#define BLE_DESP_UUID_VENDOR_PACKET_LEN   20
#define PASSKEY_LEN                       4

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bleCommMem commMem;

static uint8_t g_manufacturer_adv_data[] = {
  0xf3,
  0x10, 0x02,
  0x12,
  0xe4, 0x62, 0x6b, 0xb7, 0x0c, 0xe4 /* same with g_addr[] */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int setAppearanceValue(BLE_GAP_APPEARANCE *bleApprValue)
{
#define BLE_APPEARANCE_ATTR_HANDLE  0x18
  ble_gatts_value_t gattsValue = {0};
  uint16_t bleConnHandle = 0;

  if( bleApprValue == NULL)
    {
      return -EINVAL;
    }

  gattsValue.len      = sizeof(BLE_GAP_APPEARANCE);
  gattsValue.p_value  = (uint8_t*)bleApprValue;

  return bleGattsUpdateAttrValue(bleConnHandle, BLE_APPEARANCE_ATTR_HANDLE, &gattsValue);
}

static int setPpcpValue(BLE_GapConnParams *bleConnParams)
{
#define BLE_PPCP_ATTR_HANDLE  0x20
  ble_gatts_value_t gattsValue = {0};
  uint16_t bleConnHandle = 0;

  if( bleConnParams == NULL)
    {
      return -EINVAL;
    }

  gattsValue.len      = sizeof(BLE_GapConnParams);
  gattsValue.p_value  = (uint8_t*)bleConnParams;

  return bleGattsUpdateAttrValue(bleConnHandle, BLE_PPCP_ATTR_HANDLE, &gattsValue);
}

static int32_t set_adv_data(void)
{
#define BUF_LEN_MAX 32
#define BLE_TX_POWER_LEVEL     1
  int32_t ret             = 0;
  int8_t tx_power         = BLE_TX_POWER_LEVEL;
  BLE_GapAdvData adv_data = {0};

  adv_data.flags = BLE_GAP_ADV_LE_GENERAL_DISC_MODE | BLE_GAP_ADV_BR_EDR_NOT_SUPPORTED;
  adv_data.txPower = tx_power;
  adv_data.complete32Uuid = 0;
  adv_data.completeLocalName.advData = (uint8_t*) bt_common_context.ble_name;
  adv_data.completeLocalName.advLength =
    strnlen((char*)adv_data.completeLocalName.advData, BUF_LEN_MAX);
  adv_data.manufacturerSpecificData.advData = g_manufacturer_adv_data;
  adv_data.manufacturerSpecificData.advLength = sizeof(g_manufacturer_adv_data)/sizeof(uint8_t);
  ret = BLE_GapSetAdvData(&adv_data);

  if (BLE_SUCCESS != ret)
    {
      DBG_LOG_DEBUG("set ble adv data failed, ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_set_dev_addr
 *
 * Description:
 *   Bluetooth LE set device address.
 *   Set device address for BLE.
 *
 ****************************************************************************/

static int bcm20706_ble_set_dev_addr(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;

  /* Store input address to local address */

  memcpy(&bt_common_context.bt_addr, addr, BT_ADDR_LEN);

  /* Send BT Address to chip */

  ret = btSetBtAddress(addr);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_set_dev_name
 *
 * Description:
 *   Bluetooth LE set device name.
 *   Set device name for BLE.
 *
 ****************************************************************************/

static int bcm20706_ble_set_dev_name(char *name)
{
  int ret = BT_SUCCESS;
  size_t nameSize = 0;

  /* Get name length */

  nameSize = strlen(name);

  /* If invalid size, retrun NG */

  if (!name || nameSize > BT_MAX_NAME_LEN)
    {
      return -EINVAL;
    }

  /* Copy device name to local name */

  strncpy(bt_common_context.ble_name, name, nameSize);

  /* Send device name to chip */

  ret = btSetBtName(bt_common_context.ble_name);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_set_appearance
 *
 * Description:
 *   Bluetooth LE set appearance value.
 *   Set appearance value(generic phone, etc) to this device.
 *
 ****************************************************************************/

static int bcm20706_ble_set_appearance(BLE_APPEARANCE appearance)
{
  int ret = BT_SUCCESS;
  BLE_GAP_APPEARANCE gap_appearance;

  gap_appearance = (BLE_GAP_APPEARANCE) appearance;

  ret = setAppearanceValue(&gap_appearance);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_set_ppcp
 *
 * Description:
 *   Bluetooth LE set PPCP value.
 *
 ****************************************************************************/

static int bcm20706_ble_set_ppcp(BLE_CONN_PARAMS ppcp)
{
  int ret = BT_SUCCESS;
  BLE_GapConnParams gap_conn_param;

  gap_conn_param.minConnInterval = ppcp.minConnInterval;
  gap_conn_param.maxConnInterval = ppcp.maxConnInterval;
  gap_conn_param.slaveLatency    = ppcp.slaveLatency;;
  gap_conn_param.connSupTimeout  = ppcp.connSupTimeout;

  ret = setPpcpValue(&gap_conn_param);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_advertise
 *
 * Description:
 *   Bluetooth LE start/stop advertise.
 *   Start/Stop advertise
 *
 ****************************************************************************/

static int bcm20706_ble_advertise(bool enable)
{
  int ret = BT_SUCCESS;

  /* Set device information advertise data */

  if (enable)
    {
      set_adv_data();
    }

  /* Start/Stop advertise */

  ret = bleGapAdv(enable ? BLE_ENABLE : BLE_DISABLE);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_scan
 *
 * Description:
 *   Bluetooth LE start/stop scan.
 *   Start/Stop scan.
 *
 ****************************************************************************/

static int bcm20706_ble_scan(bool enable)
{
  int ret = BT_SUCCESS;

  ret = bleGapScan(enable ? BLE_ENABLE : BLE_DISABLE);

  return ret;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ble_hal_common_ops_s ble_hal_common_ops =
{
  .setDevAddr    = bcm20706_ble_set_dev_addr,
  .setDevName    = bcm20706_ble_set_dev_name,
  .setAppearance = bcm20706_ble_set_appearance,
  .setPPCP       = bcm20706_ble_set_ppcp,
  .advertise     = bcm20706_ble_advertise,
  .scan          = bcm20706_ble_scan
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Publict functions for BLE HAL */

int bleGapAdv(BLE_BOOL bleBool)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_LE_COMMAND_ADVERTISE);
  UINT16_TO_STREAM(p, 1);
  UINT8_TO_STREAM(p, bleBool);
  return btUartSendData(buff, p - buff);
}

int bleGapScan(BLE_BOOL bleBool)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_LE_COMMAND_SCAN);
  UINT16_TO_STREAM(p, 1);
  UINT8_TO_STREAM(p, bleBool);
  return btUartSendData(buff, p - buff);
}

void bleRecvAuthStatus(ble_evt_t *pBleBcmEvt)
{
  uint8_t *rp = NULL;
  BLE_GapBondInfo info;

  rp = pBleBcmEvt->evtData + 1;
  btdbg("auth status addr = %x,%x,%x,%x,%x,%x\n",rp[0],rp[1],rp[2],rp[3],rp[4],rp[5]);
  memcpy(&info.addr, rp, BT_ADDR_LEN);

  BLE_GapSaveBondInfo(&info);
}

void bleRecvNvramData(ble_evt_t *pBleBcmEvt, uint16_t len)
{
  uint8_t *rp = NULL;

  btdbg("Nvram data len = %d\n", len);
  rp = pBleBcmEvt->evtData;
  commMem.gapMem = bleGetGapMem();
  commMem.gapMem->wrapperBondInfo.dataLen = (uint8_t)len;
  memcpy(commMem.gapMem->wrapperBondInfo.bondData, rp, commMem.gapMem->wrapperBondInfo.dataLen);
  btdbg("nvId = %d, addr = %x, %x, %x, %x, %x, %x\n", rp[0]|(rp[1]<<8), rp[2],
      rp[3], rp[4],rp[5],rp[6],rp[7]);
  btdbg("nv data len = %d\n", len);
}

