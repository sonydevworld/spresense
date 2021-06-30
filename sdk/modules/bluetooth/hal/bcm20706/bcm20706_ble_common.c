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
extern uint8_t sdsBaseuuid[BASE_UUID_LEN];
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
      DBG_LOG_DEBUG("set ble adv data failed, ret=%ld\n", ret);
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
 * Name: bcm20706_ble_connect
 *
 * Description:
 *   Bluetooth LE connect to target device
 *
 ****************************************************************************/

static int bcm20706_ble_connect(const BT_ADDR *addr)
{
  int ret = BT_SUCCESS;
  BLE_GapAddr gap_addr = {0};

  memcpy(gap_addr.addr, addr->address, sizeof(gap_addr.addr));

  ret = BLE_GapConnect(&gap_addr);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_ble_disconnect
 *
 * Description:
 *   Bluetooth LE disconnect link to target device
 *
 ****************************************************************************/

static int bcm20706_ble_disconnect(const uint16_t conn_handle)
{
  int ret = BT_SUCCESS;
  BLE_GapConnHandle handle = conn_handle;

  ret = BLE_GapDisconnectLink(handle);

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
  .scan          = bcm20706_ble_scan,
  .connect       = bcm20706_ble_connect,
  .disconnect    = bcm20706_ble_disconnect
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

static void bleCopyUuid(uint8_t *pDest, uint8_t *pSrc, uint16_t len)
{
  int i = 0;
  for (i = 0; i < len; i++)
    {
      pDest[i] = pSrc[len - 1 - i];
    }
  return;
}

void bleRecvGattServiceDiscovered(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt, uint16_t len)
{
  bleGattcDb *gattcDbDiscovery = &commMem.gattcDb;
  uint8_t *rp = NULL;
  uint16_t connHandle = 0;
  BLE_GattcDbDiscSrv  *srvBeingDiscovered = NULL;
  uint16_t tempServUuid = 0;
  uint8_t tempServUuid128[BASE_UUID_LEN] = {0};

  rp = pBleBcmEvt->evtData;
  STREAM_TO_UINT16(connHandle, rp);

  if (gattcDbDiscovery->dbDiscovery.srvCount >= BLE_DB_DISCOVERY_MAX_SRV)
    {
      gattcDbDiscovery->state.srvCount += 1;
      return;
    }
  gattcDbDiscovery->dbDiscovery.srvCount +=1;
  gattcDbDiscovery->state.srvCount += 1;
  gattcDbDiscovery->dbDiscovery.connHandle = connHandle;
  srvBeingDiscovered = &(gattcDbDiscovery->
                         dbDiscovery.services[gattcDbDiscovery->dbDiscovery.srvCount - 1]);

  if (len == BLE_SERV_UUID_BTSIG_PACKET_LEN)
    {
      srvBeingDiscovered->srvUuid.type = BLE_UUID_TYPE_BASEALIAS_BTSIG;
      STREAM_TO_UINT16(tempServUuid, rp);
      if ((tempServUuid < MIN_UUID_SERVICE) && (tempServUuid > MAX_UUID_SERVICE))
        {
          srvBeingDiscovered->srvUuid.type = BLE_UUID_TYPE_BASEALIAS_VENDOR;
          memcpy(tempServUuid128, sdsBaseuuid, BASE_UUID_LEN);
          tempServUuid128[BLE_UUID128_13_BYTE] = (uint8_t)(tempServUuid && 0xff);
          tempServUuid128[BLE_UUID128_14_BYTE] = (uint8_t)(tempServUuid >> 8);
          bleCopyUuid(srvBeingDiscovered->
                      srvUuid.value.uuid128.uuid128, tempServUuid128, BLE_UUID128_LEN);
        }
      else
        {
          srvBeingDiscovered->srvUuid.type = BLE_UUID_TYPE_BASEALIAS_BTSIG;
          srvBeingDiscovered->srvUuid.value.baseAlias.uuidAlias = tempServUuid;
        }
    }
  else if (len == BLE_SERV_UUID_VENDOR_PACKET_LEN)
    {
      srvBeingDiscovered->srvUuid.type = BLE_UUID_TYPE_UUID128;
      bleCopyUuid(srvBeingDiscovered->srvUuid.value.uuid128.uuid128, rp, BLE_UUID128_LEN);
      rp += BLE_UUID128_LEN;
    }
  else
    {
      /* nothing */
    }
  STREAM_TO_UINT16(srvBeingDiscovered->srvHandleRange.startHandle, rp);
  STREAM_TO_UINT16(srvBeingDiscovered->srvHandleRange.endHandle, rp);
  gattcDbDiscovery->state.endHandle =
    srvBeingDiscovered->srvHandleRange.endHandle;
  btdbg("service discover,startHandle = %x,endHandle = %x.\n",
        srvBeingDiscovered->srvHandleRange.startHandle,
        srvBeingDiscovered->srvHandleRange.endHandle);
}

void bleRecvGattCharDiscovered(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt, uint16_t len)
{
  bleGattcDb *gattcDbDiscovery = &commMem.gattcDb;
  uint8_t *rp = NULL;
  uint8_t numCharsPrevDisc = 0;
  uint8_t numCharsCurrDisc = 0;
  uint16_t tempCharUuid = 0;
  uint8_t tempCharUuid128[BASE_UUID_LEN] = {0};

  BLE_GattcDbDiscSrv *srvBeingDiscovered =
    &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
  rp = pBleBcmEvt->evtData + BLE_HANDLE_LEN;

  if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
    {
      gattcDbDiscovery->discoveryInProgress = false;
      return;
    }

  numCharsPrevDisc = srvBeingDiscovered->charCount;
  numCharsCurrDisc = 1;

  /* Check if the total number of discovered characteristics are supported */
  if ((numCharsPrevDisc + numCharsCurrDisc) <= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV)
    {
      /* Update the characteristics count. */
      srvBeingDiscovered->charCount += numCharsCurrDisc;
    }
  else
    {
      /* The number of characteristics discovered at the peer is more than the supported maximum. */
      srvBeingDiscovered->charCount = BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV;
    }

  STREAM_TO_UINT16(srvBeingDiscovered->
                   characteristics[srvBeingDiscovered->charCount - 1].
                   characteristic.charDeclhandle, rp);
  btdbg("char discover,handle = %x.\n",
        srvBeingDiscovered->
        characteristics[srvBeingDiscovered->charCount - 1].
        characteristic.charDeclhandle);
  if (len == BLE_CHAR_UUID_BTSIG_PACKET_LEN)
    {
      STREAM_TO_UINT16(tempCharUuid, rp);
      if ((tempCharUuid < MIN_UUID_CHAR) && (tempCharUuid > MAX_UUID_CHAR))
        {
          srvBeingDiscovered->
            characteristics[srvBeingDiscovered->
                            charCount - 1].characteristic.charValUuid.type
            = BLE_UUID_TYPE_BASEALIAS_VENDOR;
          memcpy(tempCharUuid128, sdsBaseuuid, BASE_UUID_LEN);
          tempCharUuid128[BLE_UUID128_13_BYTE] = (uint8_t)(tempCharUuid && 0xff);
          tempCharUuid128[BLE_UUID128_14_BYTE] = (uint8_t)(tempCharUuid >> 8);
          bleCopyUuid(srvBeingDiscovered->
                      characteristics[srvBeingDiscovered->charCount - 1].
                      characteristic.charValUuid.value.uuid128.uuid128,
                      tempCharUuid128, BLE_UUID128_LEN);
        }
      else
        {
          srvBeingDiscovered->
            characteristics[srvBeingDiscovered->charCount - 1].
            characteristic.charValUuid.type
            = BLE_UUID_TYPE_BASEALIAS_BTSIG;
          srvBeingDiscovered->
            characteristics[srvBeingDiscovered->charCount - 1].
            characteristic.charValUuid.value.baseAlias.uuidAlias
            = tempCharUuid;
        }
    }
  else if (len == BLE_CHAR_UUID_VENDOR_PACKET_LEN)
    {
      srvBeingDiscovered->
        characteristics[srvBeingDiscovered->charCount - 1].
        characteristic.charValUuid.type
        = BLE_UUID_TYPE_UUID128;
      bleCopyUuid(srvBeingDiscovered->
                  characteristics[srvBeingDiscovered->charCount - 1].
                  characteristic.charValUuid.value.uuid128.uuid128,
                  rp, BLE_UUID128_LEN);
      rp += BLE_UUID128_LEN;
    }
  else
    {
      /* nothing */
    }
  memcpy(&srvBeingDiscovered->
         characteristics[srvBeingDiscovered->charCount - 1].characteristic.charPrope, rp, 1);
  rp += 1;
  STREAM_TO_UINT16(srvBeingDiscovered->
                   characteristics[srvBeingDiscovered->charCount - 1].
                   characteristic.charValhandle, rp);
  btdbg("char discover,valueHandle = %x.\n",
        srvBeingDiscovered->
        characteristics[srvBeingDiscovered->charCount - 1].characteristic.charValhandle);

  srvBeingDiscovered->
    characteristics[srvBeingDiscovered->charCount - 1].cccdHandle = BLE_GATT_HANDLE_INVALID;
}

void bleRecvGattDescriptorDiscovered(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt, uint16_t len)
{
  bleGattcDb *gattcDbDiscovery = &commMem.gattcDb;
  uint8_t *rp = NULL;
  uint16_t descriptorHandle = 0,uuid16 = 0;
  BLE_GattcDbDiscSrv *srvBeingDiscovered = NULL;

  rp = pBleBcmEvt->evtData + 2;
  if (len == BLE_DESP_UUID_BTSIG_PACKET_LEN)
    {
      STREAM_TO_UINT16(uuid16, rp);
    }
  else if (len == BLE_DESP_UUID_VENDOR_PACKET_LEN)
    {
      rp += BLE_UUID128_LEN;
    }
  STREAM_TO_UINT16(descriptorHandle, rp);

  srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
  BLE_GattcDbDiscChar * charBeingDiscovered =
    &(srvBeingDiscovered->characteristics[gattcDbDiscovery->currCharInd]);

  if ((gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
      || (gattcDbDiscovery->currCharInd >= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV))
    {
      gattcDbDiscovery->discoveryInProgress = false;
      return;
    }
  if (uuid16 == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG)
    {
      charBeingDiscovered->cccdHandle = descriptorHandle;
    }
  btdbg("descriptor discover,handle = %x.\n", descriptorHandle);
}

static void bleCharDiscoverd(bleGattcDb *gattcDbDiscovery)
{
  uint16_t connHandle = gattcDbDiscovery->dbDiscovery.connHandle;
  int ret;
  BLE_GattcHandleRange handleRange = {0};

  btdbg("char discover,handle = %x.\n", connHandle);

  if (gattcDbDiscovery->currSrvInd < gattcDbDiscovery->dbDiscovery.srvCount)
    {
      handleRange.startHandle = gattcDbDiscovery->
        dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.startHandle;
      handleRange.endHandle = gattcDbDiscovery->
        dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle;
      ret = bleCharDiscover(connHandle, handleRange.startHandle, handleRange.endHandle);
      if (ret != BLE_SUCCESS)
        {
          gattcDbDiscovery->discoveryInProgress = false;
          return;
        }
      gattcDbDiscovery->discoverState = CHAR_DISCOVER;
    }
  else
    {
      gattcDbDiscovery->discoverState = COMPLETE_DISCOVER;
    }
}

static int isDescDiscoveryReqd(bleGattcDb *gattcDbDiscovery,
                               BLE_GattcDbDiscChar  *currChar,
                               BLE_GattcDbDiscChar  *nextChar,
                               BLE_GattcHandleRange *handleRange)
{
  if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
    {
      return false;
    }
  btdbg("charValhandle=%d,srvInd = %d\n",
        currChar->characteristic.charValhandle,gattcDbDiscovery->currSrvInd);
  if (nextChar == NULL)
    {
      /* Current characteristic is the last characteristic in the service. Check if the value
       * handle of the current characteristic is equal to the service end handle.
       */
      if (currChar->characteristic.charValhandle ==
          gattcDbDiscovery->
          dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle)
        {
          /* No descriptors can be present for the current characteristic. currChar is the last
           * characteristic with no descriptors.
	   */
          return false;
        }

      handleRange->startHandle = currChar->characteristic.charValhandle + 1;

      /* Since the current characteristic is the last characteristic in the service, the end
       * handle should be the end handle of the service.
       */
      handleRange->endHandle = gattcDbDiscovery->
        dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle;
      return true;
    }

  /* nextChar != NULL. Check for existence of descriptors between the current and the next
   * characteristic.
   */
  if ((currChar->characteristic.charValhandle + 1) == nextChar->characteristic.charDeclhandle)
    {
      /* No descriptors can exist between the two characteristic. */
      return false;
    }

  handleRange->startHandle = currChar->characteristic.charValhandle + 1;
  handleRange->endHandle   = nextChar->characteristic.charDeclhandle - 1;

  return true;
}

static void onSrvDiscCompletion(bleGattcDb *gattcDbDiscovery)
{
  /* Reset the current characteristic index since a new service discovery is about to start. */
  gattcDbDiscovery->currCharInd = 0;

  /* Initiate discovery of the next service. */
  gattcDbDiscovery->currSrvInd++;

  if (gattcDbDiscovery->currSrvInd >= gattcDbDiscovery->dbDiscovery.srvCount)
    {
      gattcDbDiscovery->discoverState = COMPLETE_DISCOVER;
    }
}

static int bleDiscriptorDiscoverd(bleGattcDb *const gattcDbDiscovery, int *raiseDiscovComplete)
{
  int                        ret = 0;
  uint8_t                    i = 0;
  uint16_t                   connHandle = 0;
  BLE_GattcHandleRange       handleRange = {0};
  BLE_GattcDbDiscChar        *currCharBeingDiscovered = NULL;
  BLE_GattcDbDiscSrv         *srvBeingDiscovered = NULL;
  BLE_GattcDbDiscChar        *nextChar = NULL;
  int                       isDiscoveryReqd = 0;
  connHandle = gattcDbDiscovery->dbDiscovery.connHandle;

  btdbg("start descriptor discover\n");

  if ((gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
      || (gattcDbDiscovery->currCharInd >= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV))
    {
      return BLE_FAILED;
    }

  srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
  currCharBeingDiscovered = &(srvBeingDiscovered->characteristics[gattcDbDiscovery->currCharInd]);

  btdbg("start descriptor discover1,%d,charInd=%d,charCount=%d\n",
        isDiscoveryReqd,gattcDbDiscovery->currCharInd,
        srvBeingDiscovered->charCount);
  if ((gattcDbDiscovery->currCharInd + 1) == srvBeingDiscovered->charCount)
    {
      /* This is the last characteristic of this service. */
      isDiscoveryReqd = isDescDiscoveryReqd(gattcDbDiscovery,
                                            currCharBeingDiscovered,
                                            NULL, &handleRange);
    }
  else
    {
      for (i = gattcDbDiscovery->currCharInd; i < srvBeingDiscovered->charCount; i++)
        {
          if (i == (srvBeingDiscovered->charCount - 1))
            {
              /* The current characteristic is the last characteristic in the service. */
              nextChar = NULL;
            }
          else
            {
              nextChar = &(srvBeingDiscovered->characteristics[i + 1]);
            }

          /* Check if it is possible for the current characteristic to have a descriptor. */
          if (isDescDiscoveryReqd(gattcDbDiscovery,
                                  currCharBeingDiscovered, nextChar, &handleRange))
            {
              isDiscoveryReqd = true;
              gattcDbDiscovery->currCharInd = i;
              btdbg("start descriptor discover2,%d\n",isDiscoveryReqd);
              break;
            }
          else
            {
              /* No descriptors can exist. */
              currCharBeingDiscovered = nextChar;
            }
        }
    }

  if (!isDiscoveryReqd)
    {
      /* No more descriptor discovery required. Discovery is complete. */
      *raiseDiscovComplete = true;
      btdbg("start descriptor discover1 over\n");
      return ret;
    }

  *raiseDiscovComplete = false;
  btdbg("start descriptor discover start,connecthandle= %x,start handle= %x,end handle=%x\n",
        connHandle, handleRange.startHandle, handleRange.endHandle);
  ret = bleDescriptorsDiscover(connHandle, handleRange.startHandle, handleRange.endHandle);
  if (ret != BLE_SUCCESS)
    {
      gattcDbDiscovery->discoveryInProgress = false;
      return BLE_FAILED;
    }
  return ret;
}

void bleRecvGattCompleteDiscovered(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt)
{
  bleGattcDb *gattcDbDiscovery = &commMem.gattcDb;
  BLE_GattcDbDiscSrv *srvBeingDiscovered = NULL;
  BLE_EvtGattcDbDiscovery *db = &commMem.gattcDbDiscoveryData;
  int raiseDiscovComplete = 0;

  srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
  btdbg("discover complete,state = %d.\n", gattcDbDiscovery->discoverState);
  if (gattcDbDiscovery->discoverState == SERV_DISCOVER
      && (gattcDbDiscovery->dbDiscovery.srvCount != 0))
    {
      gattcDbDiscovery->discoverState = CHAR_DISCOVER;
      gattcDbDiscovery->currSrvInd = 0;
      bleCharDiscoverd(gattcDbDiscovery);
      btdbg("serv discover0,state = %d.\n", gattcDbDiscovery->discoverState);
    }
  else if ((gattcDbDiscovery->discoverState == SERV_DISCOVER)
           && (gattcDbDiscovery->dbDiscovery.srvCount == 0))
    {
      pBleEvent->evtHeader = BLE_GATTC_EVENT_DBDISCOVERY;
      db->result     = BLE_GATTC_RESULT_FAILED;
      db->params.reason = BLE_GATTC_REASON_SERVICE;
      pBleEvent->evtDataSize = sizeof(BLE_EvtGattcDbDiscovery);
      memcpy(pBleEvent->evtData, db, pBleEvent->evtDataSize);
      btdbg("serv discover1,state = %d.\n", gattcDbDiscovery->discoverState);
    }
  else if ((gattcDbDiscovery->discoverState == CHAR_DISCOVER)
           && (gattcDbDiscovery->
               dbDiscovery.services[gattcDbDiscovery->currSrvInd].charCount == 0) )
    {
      onSrvDiscCompletion(gattcDbDiscovery);
      if (gattcDbDiscovery->discoverState == COMPLETE_DISCOVER)
        {
          goto discover_over;
        }
      bleCharDiscoverd(gattcDbDiscovery);
      btdbg("char discover,state = %d.\n", gattcDbDiscovery->discoverState);
    }
  else if ((gattcDbDiscovery->discoverState == CHAR_DISCOVER)
           && (gattcDbDiscovery->
               dbDiscovery.services[gattcDbDiscovery->currSrvInd].charCount != 0) )
    {
      bleDiscriptorDiscoverd(gattcDbDiscovery, &raiseDiscovComplete);
      gattcDbDiscovery->discoverState = DESCRIPTOR_DISCOVER;
      btdbg("des discover0,state = %d.\n", gattcDbDiscovery->discoverState);
    }
  else if (gattcDbDiscovery->discoverState == DESCRIPTOR_DISCOVER
           && (gattcDbDiscovery->currCharInd < srvBeingDiscovered->charCount - 1))
    {
      gattcDbDiscovery->currCharInd++;
      btdbg("des discover ,state = %d.\n", gattcDbDiscovery->discoverState);
      bleDiscriptorDiscoverd(gattcDbDiscovery, &raiseDiscovComplete);
    }
  else if (gattcDbDiscovery->discoverState == DESCRIPTOR_DISCOVER
           && (gattcDbDiscovery->currCharInd >= srvBeingDiscovered->charCount - 1))
    {
      btdbg("start discover char0\n");
      gattcDbDiscovery->discoverState = CHAR_DISCOVER;
      onSrvDiscCompletion(gattcDbDiscovery);
      if (gattcDbDiscovery->discoverState == COMPLETE_DISCOVER) {
        goto discover_over;
      }
      bleCharDiscoverd(gattcDbDiscovery);
    }

  if (raiseDiscovComplete)
    {
      btdbg("start discover char\n");
      gattcDbDiscovery->discoverState = CHAR_DISCOVER;
      onSrvDiscCompletion(gattcDbDiscovery);
      if (gattcDbDiscovery->discoverState == COMPLETE_DISCOVER) {
        goto discover_over;
      }
      bleCharDiscoverd(gattcDbDiscovery);
      raiseDiscovComplete = false;
    }

 discover_over:
  if (gattcDbDiscovery->discoverState == COMPLETE_DISCOVER)
    {
      btdbg("discover over,serv count = %d\n", gattcDbDiscovery->dbDiscovery.srvCount);
      pBleEvent->evtHeader = BLE_GATTC_EVENT_DBDISCOVERY;
      if (gattcDbDiscovery->state.srvCount >
          BLE_DB_DISCOVERY_MAX_SRV)
        {
          db->result = BLE_GATTC_RESULT_OVERRUN;
          db->state.srvCount = gattcDbDiscovery->state.srvCount;
          db->state.endHandle = gattcDbDiscovery->state.endHandle;
        }
      else
        {
          db->result = BLE_GATTC_RESULT_SUCCESS;
        }
      pBleEvent->evtDataSize = sizeof(BLE_EvtGattcDbDiscovery);
      memcpy(&db->params.dbDiscovery,
             &gattcDbDiscovery->dbDiscovery,
             sizeof(BLE_GattcDbDiscovery));
      db->connHandle = db->params.dbDiscovery.connHandle;
      memcpy(pBleEvent->evtData, db, pBleEvent->evtDataSize);
      memset(gattcDbDiscovery, 0, sizeof(bleGattcDb));
    }
}
