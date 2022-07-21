/****************************************************************************
 * modules/bluetooth/hal/nrf52/ble_comm.c
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
#include <unistd.h>
#include <ble/ble_comm.h>
#include <ble/ble_gap.h>
#include <ble/ble_gatts.h>
#include <ble/ble_gattc.h>
#include "ble_storage_operations.h"
#include "ble_comm_internal.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include <arch/board/board.h>
#include <system/readline.h>

/******************************************************************************
 * externs
 *****************************************************************************/
extern bleGapMem *bleGetGapMem(void);
extern void bleMngEvtDispatch(ble_evt_t *nrfEvt);
extern void board_nrf52_initialize(void);
extern void board_nrf52_reset(bool en);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
// #define BLE_DBGPRT_ENABLE
#ifdef BLE_DBGPRT_ENABLE
#include <stdio.h>
#define BLE_PRT printf
#define BLE_PRT2(...)
#define BLE_ERR printf
#else
#define BLE_PRT(...)
#define BLE_PRT2(...)
#define BLE_ERR(...)
#endif

 /******************************************************************************
 * Structure define
 *****************************************************************************/

 /******************************************************************************
 * Function prototype declaration
 *****************************************************************************/
static int blePowerOn(void);
static int bleStackInit(void);
static int bleStackFin(void);
#ifdef BLE_USE_SECTION
static void bleEvtDispatch(ble_evt_t *pBleNrfEvt);
#endif
#if NRF_SD_BLE_API_VERSION > 5
static void onAdvSetTerminate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
#endif
static void onScanReqReport(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onPhyUpdateReq(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onPhyUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDataLengthUpdateRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDataLengthUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onExchangeMtuRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onTxComplete(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnParamUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnParamUpdateRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDisconnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onSecParamsRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onAuthStatus(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDispPasskey(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onAdvReport(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onAuthKeyRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnSecUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onSecInfoRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onTimeout(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onGattsTimeout(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onRssiChanged(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onGattsWrite(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onGattsIndConfirm(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onPrimarySrvDiscoveryRsp(bleGattcDb *gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onCharacteristicDiscoveryRsp(bleGattcDb *const gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDescriptorDiscoveryRsp(bleGattcDb *const gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onHvx(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onSysAttrMissing(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onReadRsp(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onWriteRsp(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onSrvDiscCompletion(BLE_Evt *pBleEvent, bleGattcDb *gattcDbDiscovery);
static  int characteristicsDiscover(BLE_Evt *pBleEvent, bleGattcDb *const gattcDbDiscovery);
static bool isCharDiscoveryReqd(bleGattcDb *const gattcDbDiscovery, BLE_GattcChar *afterChar);
static bool isDescDiscoveryReqd(bleGattcDb *gattcDbDiscovery, BLE_GattcDbDiscChar *currChar, BLE_GattcDbDiscChar *nextChar, BLE_GattcHandleRange *handleRange);
static  int descriptorsDiscover(BLE_Evt *pBleEvent, bleGattcDb *const gattcDbDiscovery, bool *raiseDiscovComplete);

static int nrf52_ble_set_dev_addr(BT_ADDR *addr);
static int nrf52_bt_init(void);
static int nrf52_bt_finalize(void);
static int nrf52_bt_enable(bool enable);

/******************************************************************************
 * Private Data
 *****************************************************************************/
bleCommMem commMem = {0};
bleGapWrapperBondInfo BondInfoInFlash[BLE_SAVE_BOND_DEVICE_MAX_NUM] = {{0}};
uint32_t bleBondEnableList = 0;
bleBondInfoKey bleKey = {0};
struct bt_common_context_s bt_common_context = {0};

#define BUF_LEN_MAX 32

#ifndef CONFIG_NRF51822
#define READ_BUFF_LEN 512
#else
#define READ_BUFF_LEN 20
#endif

#define WAIT_TIME (100*1000)
#define LOG_OUT printf
#define AUTH_KEY_SIZE   6
static BLE_GattcDbDiscovery gattc_db_discovery = {0};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int blePowerOff(void)
{
  int ret = 0;

//  for support reset pin. (Will be valid later)
//  board_nrf52_reset(true);
  ret = board_power_control(POWER_BTBLE, false);
  if (ret)
    {
      BLE_PRT("board_power_control(off): NG %d\n", ret);
      return ret;
    }
  BLE_PRT("Power off BLE!!\n");
  return BLE_SUCCESS;
}

static void bleInitBondInfoKey(void)
{
  bleKey.enable_key = BSO_GenerateRegistryKey(BOND_ENABLE_KEY_NAME, 0);
  bleKey.info_key[0] = BSO_GenerateRegistryKey(BOND_INFO_1_KEY_NAME, 0);
  bleKey.info_key[1] = BSO_GenerateRegistryKey(BOND_INFO_2_KEY_NAME, 0);
  bleKey.info_key[2] = BSO_GenerateRegistryKey(BOND_INFO_3_KEY_NAME, 0);
  bleKey.info_key[3] = BSO_GenerateRegistryKey(BOND_INFO_4_KEY_NAME, 0);
  bleKey.info_key[4] = BSO_GenerateRegistryKey(BOND_INFO_5_KEY_NAME, 0);
  bleKey.info_key[5] = BSO_GenerateRegistryKey(BOND_INFO_6_KEY_NAME, 0);
  bleKey.info_key[6] = BSO_GenerateRegistryKey(BOND_INFO_7_KEY_NAME, 0);
  bleKey.info_key[7] = BSO_GenerateRegistryKey(BOND_INFO_8_KEY_NAME, 0);
#ifdef BLE_DBGPRT_ENABLE
  BLE_PRT("bleInitBondInfoKey: enable_key 0x%lx\n", bleKey.enable_key);
  for (int i=0; i<BLE_SAVE_BOND_DEVICE_MAX_NUM; i++) {
    BLE_PRT("bleInitBondInfoKey: info_key[%d] 0x%lx\n", i, bleKey.info_key[i]);
  }
#endif
}

static int bleGetBondInfo(void)
{
  int ret = 0;
  bleInitBondInfoKey();
  ret = BSO_GetRegistryValue(bleKey.enable_key, (void *)&bleBondEnableList, sizeof(uint32_t));
  if (ret == -ENOENT)
    {
      BLE_PRT("bleGetBondInfo: Set bleBondEnableList = 0\n");
      bleBondEnableList = 0;
      BSO_SetRegistryValue(bleKey.enable_key, (const void*)&bleBondEnableList, sizeof(uint32_t));
      BSO_Sync();
    }
  else if (0 != ret)
    {
      BLE_PRT("bleGetBondInfo: Get bleBondEnableList NG %d\n", ret);
      return ret;
    }
  
  BLE_PRT("bleGetBondInfo: bleBondEnableList 0x%lx\n", bleBondEnableList);
  for (int i=0; i<BLE_SAVE_BOND_DEVICE_MAX_NUM; i++)
    {
      ret = BSO_GetRegistryValue(bleKey.info_key[i], (void *)&BondInfoInFlash[i], sizeof(bleGapWrapperBondInfo));
      if(ret == -ENOENT)
        {
          BLE_PRT("bleGetBondInfo: Set bleBondEnableList = 0\n");
          bleBondEnableList = 0;
          BSO_SetRegistryValue(bleKey.info_key[i], (const void*)&BondInfoInFlash[i], sizeof(bleGapWrapperBondInfo));
          BSO_Sync();
        }
      else if (0 != ret)
        {
          BLE_PRT("bleGetBondInfo: Get BondInfoInFlash[%d] NG %d\n", i, ret);
          return ret;
        }
    }
  return 0;
}

int BLE_CommonInitializeStack(BLE_InitializeParams *initializeParams)
{
  uint8_t role = 0;
  int ret = BLE_SUCCESS;

  if (initializeParams == NULL)
    {
      return -EINVAL;
    }
  if (commMem.stackInited)
    {
      return -EPERM;
    }

//  for pin configuration. (Will be valid later)
//  board_nrf52_initialize();

  commMem.gapMem = bleGetGapMem();
  role = initializeParams->role;
  ret = BSO_Init(NULL);
  if (ret)
    {
      BLE_PRT("BSO_Init failed\n");
      return ret;
    }
  ret = bleGetBondInfo();
  if (ret)
    {
      BLE_PRT("bleGetBondInfo: NG %d\n", ret);
      return ret;
    }
  switch (role)
    {
      case BLE_ROLE_PERIPHERAL:
      case BLE_ROLE_CENTRAL:
      case BLE_ROLE_PERIPHERAL_AND_CENTRAL:
        ret = blePowerOn();
        if (ret)
          {
            goto errPower;
          }
        ret = bleStackInit();
        if (ret)
          {
            BLE_PRT("bleStackInit: NG %d\n", ret);
            goto errInit;
          }
        else
          {
            commMem.stackInited = true;
          }
          break;
      default:
        ret = -EINVAL;
        break;
    }
  return ret;
errInit:
  bleStackFin();
  blePowerOff();
errPower:
  return ret;

}

int BLE_CommonSetBleEvtCallback(BLE_EfCb funcCb, BLE_EvtCtx *ctx)
{
  if (ctx == NULL)
    {
      return -EINVAL;
    }
  if (commMem.managerCbFlag)
    {
      return -EPERM;
    }

  commMem.cbId = funcCb;
  commMem.bleEvtCtx = ctx;
  commMem.callbackFlag = 1;
  BLE_PRT("%s:commMem.callbackFlag=%d\n", __func__, commMem.callbackFlag);
  return BLE_SUCCESS;
}

int BLE_CommonFinalizeStack(void)
{
  int ret = BLE_SUCCESS;
  if (!commMem.stackInited)
    {
      return ret;
    }
  commMem.callbackFlag = 0;
  commMem.gapMem = NULL;
  if (bleStackFin())
    {
      ret = -EPERM;
    }
  if (blePowerOff())
    {
      ret = -EPERM;
    }
  commMem.stackInited = false;
  return ret;
}

static
int blePowerOn(void)
{
  int ret = 0;
  ret = board_power_control(POWER_BTBLE, true);
  if (ret)
    {
      BLE_PRT("board_power_control(on): NG %d\n", ret);
      goto errPower;
    }
//  for support reset pin. (Will be valid later)
//  board_nrf52_reset(false);
  BLE_PRT("Power on BLE!!\n");
  return BLE_SUCCESS;
errPower:
  (void)board_power_control(POWER_BTBLE, false);
  return ret;

}

int bleConvertErrorCode(uint32_t errCode)
{
  int ret = 0;
  switch (errCode)
    {
      case NRF_SUCCESS:
        ret = BLE_SUCCESS;
        break;
      case NRF_ERROR_INVALID_PARAM:
        BLE_ERR("errno: 0x%lx, Invalid parameter.\n", errCode);
        ret = -EINVAL;
        break;
      case NRF_ERROR_INVALID_STATE:
        BLE_ERR("errno: 0x%lx, Invalid state.\n", errCode);
        ret = -EPERM;
        break;
      case NRF_ERROR_INVALID_ADDR:
        BLE_ERR("errno: 0x%lx, Bad Memory Address.\n", errCode);
        ret = -EINVAL;
        break;
      case NRF_ERROR_INVALID_FLAGS:
        BLE_ERR("errno: 0x%lx, Invalid flags.\n", errCode);
        ret = -EINVAL;
        break;
      case NRF_ERROR_INVALID_DATA:
        BLE_ERR("errno: 0x%lx, Invalid data.\n", errCode);
        ret = -EINVAL;
        break;
      case NRF_ERROR_DATA_SIZE:
        BLE_ERR("errno: 0x%lx, Data size exceeds limit.\n", errCode);
        ret = -EINVAL;
        break;
      case NRF_ERROR_INVALID_LENGTH:
        BLE_ERR("errno: 0x%lx, Invalid length.\n", errCode);
        ret = -EINVAL;
        break;
      case NRF_ERROR_NOT_SUPPORTED:
        BLE_ERR("errno: 0x%lx, Not supported.\n", errCode);
        ret = -ENOTSUP;
        break;
      case NRF_ERROR_BUSY:
        BLE_ERR("errno: 0x%lx, Busy.\n", errCode);
        ret = -EBUSY;
        break;
      case NRF_ERROR_NO_MEM:
        BLE_ERR("errno: 0x%lx, No Memory for operation.\n", errCode);
        ret = -ENOMEM;
        break;
      case NRF_ERROR_FORBIDDEN:
        BLE_ERR("errno: 0x%lx, Forbidden Operation.\n", errCode);
        ret = -EPERM;
        break;
      case BLE_ERROR_INVALID_CONN_HANDLE:
        BLE_ERR("errno: 0x%lx, Invalid connection handler.\n", errCode);
        ret = -EPERM;
        break;
      case BLE_ERROR_GAP_UUID_LIST_MISMATCH:
        BLE_ERR("errno: 0x%lx, BLE_ERROR_GAP_UUID_LIST_MISMATCH.\n", errCode);
        ret = -EINVAL;
        break;
      case BLE_ERROR_GAP_INVALID_BLE_ADDR:
        BLE_ERR("errno: 0x%lx, BLE_ERROR_GAP_INVALID_BLE_ADDR.\n", errCode);
        ret = -EPERM;
        break;
      case BLE_ERROR_GAP_WHITELIST_IN_USE:
        BLE_ERR("errno: 0x%lx, BLE_ERROR_GAP_WHITELIST_IN_USE.\n", errCode);
        ret = -EPERM;
        break;
      case BLE_ERROR_GAP_DISCOVERABLE_WITH_WHITELIST:
        BLE_ERR("errno: 0x%lx, BLE_ERROR_GAP_DISCOVERABLE_WITH_WHITELIST.\n", errCode);
        ret = -EPERM;
        break;
      case NRF_ERROR_TIMEOUT:
        BLE_ERR("errno: 0x%lx, NRF_ERROR_TIMEOUT.\n", errCode);
        ret = -ETIMEDOUT;
        break;
      default:
        if (errCode > 0)
          {
            BLE_ERR("errno: 0x%lx, Stack error.\n", errCode);
            ret = -EPERM;
          }
        break;
    }
  return ret;
}

static
int bleStackInit(void)
{
  int ret;
  BLE_PRT("nrf_sdh_enable_request\n");
  ret = nrf_sdh_enable_request();
  if (ret)
    {
      return bleConvertErrorCode(ret);
    }
  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  BLE_PRT("nrf_sdh_ble_default_cfg_set\n");
  ret = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, NULL);
  if (ret)
    {
      goto err;
    }
  // Enable BLE stack.
  BLE_PRT("nrf_sdh_ble_enable\n");
  ret = nrf_sdh_ble_enable(NULL);
  if (ret)
    {
      goto err;
    }
  // Register a handler for BLE events.
#ifdef BLE_USE_SECTION
  BLE_PRT("NRF_SDH_BLE_OBSERVER\n");
  NRF_SDH_BLE_OBSERVER(m_ble_observer, 0, (void *)bleEvtDispatch, NULL);
#endif
  return ret;
err:
  nrf_sdh_disable_request();
  return bleConvertErrorCode(ret);
}

static
int bleStackFin(void)
{
  int ret = 0;
  ret = nrf_sdh_disable_request();
  return bleConvertErrorCode(ret);
}

void bleNrfEvtHandler(BLE_Evt *bleEvent, ble_evt_t *pBleNrfEvt)
{
  BLE_PRT("bleNrfEvtHandler 0x%x\n", pBleNrfEvt->header.evt_id);
  memset(bleEvent, 0x00, sizeof(BLE_Evt));
  switch(pBleNrfEvt->header.evt_id)
    {
      case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
      case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        onTxComplete(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_CONNECTED:
        onConnect(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        onConnParamUpdate(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        onConnParamUpdateRequest(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_DISCONNECTED:
        onDisconnect(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        onSecParamsRequest(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_AUTH_STATUS:
        onAuthStatus(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_PASSKEY_DISPLAY:
        onDispPasskey(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_ADV_REPORT:
        onAdvReport(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        onAuthKeyRequest(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_SEC_INFO_REQUEST:
        onSecInfoRequest(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_RSSI_CHANGED:
        onRssiChanged(bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTS_EVT_WRITE:
        onGattsWrite(bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTS_EVT_HVC:
        onGattsIndConfirm(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_CONN_SEC_UPDATE:
        onConnSecUpdate(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_TIMEOUT:
        onTimeout(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        onDataLengthUpdateRequest(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
        onDataLengthUpdate(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_SCAN_REQ_REPORT:
        onScanReqReport(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        onPhyUpdateReq(bleEvent, pBleNrfEvt);
        break;
      case BLE_GAP_EVT_PHY_UPDATE:
        onPhyUpdate(bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
        onPrimarySrvDiscoveryRsp(&commMem.gattcDb, bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTC_EVT_CHAR_DISC_RSP:
        onCharacteristicDiscoveryRsp(&commMem.gattcDb, bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTC_EVT_DESC_DISC_RSP:
        onDescriptorDiscoveryRsp(&commMem.gattcDb, bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTC_EVT_READ_RSP:
        onReadRsp(bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTC_EVT_WRITE_RSP:
        onWriteRsp(bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTC_EVT_HVX:
        onHvx(bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        onSysAttrMissing(bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTS_EVT_TIMEOUT:
        onGattsTimeout(bleEvent, pBleNrfEvt);
        break;
      case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
        onExchangeMtuRequest(bleEvent, pBleNrfEvt);
        break;
    #if NRF_SD_BLE_API_VERSION > 5
      case BLE_GAP_EVT_ADV_SET_TERMINATED:
        onAdvSetTerminate(bleEvent, pBleNrfEvt);
        break;
    #endif
      default:
        bleEvent->evtHeader = (BLE_EventTypes)pBleNrfEvt->header.evt_id;
        BLE_ERR("Unknown event 0x%x\n", bleEvent->evtHeader);
        break;
      }
}

static void ble_advertise(int mode)
{
  int ret = 0;

  if (mode == 2)
    {
      LOG_OUT("ble_peripheral: Long range\n");
      BLE_GapAdvParams p = {0x40, 0, BLE_GAP_PHY_CODED, 0};
      ret = BLE_GapStartAdvExt(&p);

      if (BLE_SUCCESS != ret)
        {
          LOG_OUT("ble enable advertise failed, ret=%d\n", ret);
        }
    }
  else if (mode == 1)
    {
      ret = BLE_GapStartAdv();

      if (BLE_SUCCESS != ret)
        {
          LOG_OUT("ble enable advertise failed, ret=%d\n", ret);
        }
    }
  else
    {
      ret = BLE_GapStopAdv();

      if (BLE_SUCCESS != ret)
        {
          LOG_OUT("ble disable advertise failed, ret=%d\n", ret);
        }
    }
}

static void on_connected(const BLE_EvtConnected* evt)
{
  int ret = 0;
  LOG_OUT("Connected: handle %d, role %d\n", evt->handle, evt->role);
  g_ble_context.ble_srv_sds.conn_handle = evt->handle;
  g_ble_context.ble_conn_handle         = evt->handle;
  g_ble_context.ble_role                = evt->role;

  struct ble_event_conn_stat_t conn_stat_evt;
  conn_stat_evt.connected = true;
  conn_stat_evt.handle = evt->handle;
  conn_stat_evt.group_id = BLE_GROUP_COMMON;
  conn_stat_evt.event_id = BLE_COMMON_EVENT_CONN_STAT_CHANGE;

  ble_common_event_handler((struct bt_event_t *) &conn_stat_evt);

  if (evt->role != BLE_ROLE_CENTRAL)
    {
      return;
    }

  /* TODO BLE device needs to configure the MTU after connection,
   * wait for the finish of configuring.
   */

  usleep(500 * 1000);

  ret = BLE_GattcStartDbDiscovery(g_ble_context.ble_conn_handle);
  if (BLE_SUCCESS == ret)
    {
      LOG_OUT("[BLE][SUCCESS]BLE_GattcStartDbDiscovery\n");
    }
  else
    {
      LOG_OUT("[BLE][ERROR]BLE_GattcStartDbDiscovery\n");
      LOG_OUT("[BLE][ERROR]Error number = %d\n", ret);
    }
}

static void on_disconnected(const BLE_EvtDisconnected* evt)
{
  LOG_OUT("Disconnected: HCI status 0x%x\n", evt->reason);
  g_ble_context.ble_conn_handle = CONN_HANDLE_INVALED;
  struct ble_event_conn_stat_t conn_stat_evt;

  conn_stat_evt.connected = false;

  conn_stat_evt.group_id = BLE_GROUP_COMMON;
  conn_stat_evt.event_id = BLE_COMMON_EVENT_CONN_STAT_CHANGE;

  ble_common_event_handler((struct bt_event_t *) &conn_stat_evt);
  if (g_ble_context.ble_role == BLE_ROLE_PERIPHERAL)
    {
      int ret;
      g_ble_context.ble_srv_sds.notify_enabled = 0;
      LOG_OUT("Advertising restart !!\n");
      ret = BLE_GapStartAdv();
      if (BLE_SUCCESS != ret)
        {
          LOG_OUT("BLE_GapStartAdv failed, ret=%d\n", ret);
        }
    }
}

static uint8_t wait_on_read_rsp_flag = 0;

static void on_db_discovery(BLE_EvtGattcDbDiscovery *db_discovery)
{
  if (db_discovery->result != BLE_GATTC_RESULT_SUCCESS)
    {
      LOG_OUT("[BLE][ERROR]DB Discovery Failed\n");
    }
  else
    {
      LOG_OUT("[BLE][SUCCESS]DB Discovery Success.service count= %d\n",
          db_discovery->params.dbDiscovery.srvCount);
      memcpy(&gattc_db_discovery, &db_discovery->params.dbDiscovery,
          sizeof(BLE_GattcDbDiscovery));
    }
}

static void on_read_rsp(BLE_EvtGattcRead *readRsp)
{
  LOG_OUT("read peer device name:%s\n", readRsp->charValData);
  wait_on_read_rsp_flag = 0;
}

static void on_exchange_feature(const BLE_EvtExchangeFeature* exchange_feature)
{
  BLE_GapPairingFeature* pf = &g_ble_context.pairing_feature;
  int ret = BLE_GapExchangePairingFeature(exchange_feature->handle, pf);

  if (BLE_SUCCESS != ret)
    {
      LOG_OUT("exchange pairing feature failed, ret=%d\n", ret);
    }
  else
    {
      LOG_OUT("exchange pairing feature success\n");
    }
}

static void on_rssi_changed(const BLE_EvtRssiChanged* evt)
{
  LOG_OUT("[BLE][LOG]RSSI changed, value: %d\n", evt->rssi);
}

static void on_gatts_evt_write(BLE_EvtGattsWrite* gatts_write)
{
  BLE_SrvSds* srv_sds = &g_ble_context.ble_srv_sds;
  LOG_OUT("gatts_write->data:%.*s\n",gatts_write->dataLen, gatts_write->data);
  LOG_OUT("gatts_write->handle:%x\n",gatts_write->handle);
  LOG_OUT("srv_sds->char_ri_handle.dprHandle.cccdHandle:%x\n",srv_sds->char_ri_handle.dprHandle.cccdHandle);
  LOG_OUT("gatts_write->data[0] & 0x01:%d\n",gatts_write->data[0] & 0x01);
  if (gatts_write->handle == srv_sds->char_ri_handle.dprHandle.cccdHandle)
    {
      /* client characteristic configuration.bit 0: Notification bit 1: Indication.other reserved.
        see core spec 4.1 Vol3,PartG,3,3,3,3 client characteristic
        configuration.Table 3.11
      */
      srv_sds->notify_enabled = (bool)gatts_write->data[0] & 0x01;
      if (srv_sds->notify_enabled)
        {
          LOG_OUT("notification enabled!\n");
        } 
      else
        {
          LOG_OUT("notification disabled!\n");
        }
    }
#if 0
  else {
    if (CHAR_BUF_MAX >= gatts_write->dataLen) {
      /* TODO: blocking wait */
      /* TODO: protect multi-thread accessing */
      while (0 != srv_sds->char_rw_curr_idx) {
        usleep(WAIT_TIME);
      }

      (void)sem_wait(&srv_sds->char_rw_buf_sem);
      memcpy(srv_sds->char_rw_buf, gatts_write->data, gatts_write->dataLen);
      srv_sds->char_rw_curr_idx = gatts_write->dataLen;
      (void)sem_post(&srv_sds->char_rw_buf_sem);
    }
    else {
      LOG_OUT("incoming data exceeds buffer length\n");
    }
  }
#endif
}

static void on_disp_passkey(BLE_EvtDisplayPasskey* disp_passkey)
{
  LOG_OUT("[BLE][LOG]Passkey: %s\n", disp_passkey->passkey);
}

static void on_auth_status(BLE_EvtAuthStatus* auth_status)
{
  int ret   = BLE_SUCCESS;
  int index = 0;
  BLE_GapBondInfoList bond_id = {0};
  if (auth_status->status != BLE_GAP_SM_STATUS_SUCCESS)
    {
      LOG_OUT("[BLE][LOG]Pairing failed! ErrCode: %x\n", auth_status->status);
      return;
    }
  else
    {
      LOG_OUT("[BLE][LOG]Pairing success!\n");
    }
  LOG_OUT("[BLE][LOG]save bond address:%02X:%02X:%02X:%02X:%02X:%02X\n",\
      auth_status->bondInfo.addr[5],\
      auth_status->bondInfo.addr[4],\
      auth_status->bondInfo.addr[3],\
      auth_status->bondInfo.addr[2],\
      auth_status->bondInfo.addr[1],\
      auth_status->bondInfo.addr[0]);
  ret = BLE_GapSaveBondInfo(&auth_status->bondInfo);
  if (BLE_SUCCESS == ret)
    {
      LOG_OUT("[BLE][SUCCESS]BLE_GapSaveBondInfo\n");
    }
  else
    {
      LOG_OUT("[BLE][ERROR]BLE_GapSaveBondInfo\n");
      LOG_OUT("[BLE][ERROR]Error number = %d\n", ret);
    }

  ret = BLE_GapGetBondInfoIdList(&bond_id);
  if (BLE_SUCCESS == ret)
    {
      LOG_OUT("[BLE][SUCCESS]BLE_GapGetBondInfoIdList\n");
    }
  else
    {
      LOG_OUT("[BLE][ERROR]BLE_GapGetBondInfoIdList\n");
      LOG_OUT("[BLE][ERROR]Error number = %d\n", ret);
    }

  LOG_OUT("[BLE][LOG]bond_id.bondNum = %ld\n", bond_id.bondNum);

  for(index=0; index<bond_id.bondNum; index++)
    {
      LOG_OUT("[BLE][LOG]Address[%d]:%02X:%02X:%02X:%02X:%02X:%02X\n",\
          index,\
          bond_id.bondInfoId[index][5],\
          bond_id.bondInfoId[index][4],\
          bond_id.bondInfoId[index][3],\
          bond_id.bondInfoId[index][2],\
          bond_id.bondInfoId[index][1],\
          bond_id.bondInfoId[index][0]);
    }
}

static void on_timeout(BLE_EvtTimeout *timeout)
{
  int ret = BLE_SUCCESS;
  switch(timeout->timeoutSrc)
    {
      case BLE_GAP_TIMEOUT_ADVERTISING:
        LOG_OUT("[BLE][LOG]Timeout reason: Advertising timeout!\n");
        LOG_OUT("[BLE][LOG]Start advertise\n");
        ble_advertise(1);
        break;
      case BLE_GAP_TIMEOUT_SECURITY_REQUEST:
        LOG_OUT("[BLE][LOG]Timeout reason: Security request timeout!\n");
        break;
      case BLE_GAP_TIMEOUT_SCAN:
        LOG_OUT("[BLE][LOG]Timeout reason: Scanning timeout!\n");
        ret = BLE_GapStartScan();
        if (BLE_SUCCESS == ret)
          {
            LOG_OUT("[BLE][SUCCESS]BLE_GapStartScan\n");
          }
        else
          {
            LOG_OUT("[BLE][ERROR]BLE_GapStartScan\n");
            LOG_OUT("[BLE][ERROR]Error number = %d\n", ret);
          }
        break;
      case BLE_GAP_TIMEOUT_CONN:
        LOG_OUT("[BLE][LOG]Timeout reason: Connection timeout!\n");
        break;
      default:
        LOG_OUT("[BLE][LOG]Timeout reason: Error\n");
        break;
    }
}

static int input_passkey(uint8_t *key)
{
  LOG_OUT("Please enter passkey:\n");
  const char* name = "passkey_input";
  int len = 0;

  LOG_OUT("%s> ", name);
  fflush(stdout);
  len = readline((char*)key, AUTH_KEY_SIZE, stdin, stdout);
  if (len < 1)
    {
      LOG_OUT("readpasskey error, ret:%d\n", len);
      return -1;
    }
  key[AUTH_KEY_SIZE] = '\0';
  LOG_OUT("passkey = %s", key);
  return BLE_SUCCESS;
}

static void on_auth_key_request(BLE_EvtAuthKey *auth_key)
{
  int ret = BLE_SUCCESS;
  uint8_t key[AUTH_KEY_SIZE + 1] = {0};
  BLE_GapAuthKey key_data = {0};

  if (BLE_GAP_AUTH_KEY_TYPE_PASSKEY != auth_key->keyType)
    {
      LOG_OUT("[BLE][LOG]Authentication key type is not BLE_GAP_AUTH_KEY_TYPE_PASSKEY!!!\n");
      LOG_OUT("[BLE][LOG]Key type number = %d\n", auth_key->keyType);
      return;
    }
  ret = input_passkey((uint8_t*)key);
  if (BLE_SUCCESS == ret)
    {
      LOG_OUT("[BLE][SUCCESS]input_passkey\n");
    }
  else
    {
      LOG_OUT("[BLE][ERROR]input_passkey\n");
      LOG_OUT("[BLE][ERROR]Error number = %d\n", ret);
    }

  key_data.handle  = auth_key->handle;
  key_data.keyType = BLE_GAP_AUTH_KEY_TYPE_PASSKEY;
  key_data.keyLen  = AUTH_KEY_SIZE;
  key_data.key = key;
  ret = BLE_GapReplyAuthKey(&key_data);

  if (BLE_SUCCESS == ret)
    {
      LOG_OUT("[BLE][SUCCESS]BLE_GapReplyAuthKey\n");
    }
  else
    {
      LOG_OUT("[BLE][ERROR]BLE_GapReplyAuthKey\n");
      LOG_OUT("[BLE][ERROR]Error number = %d\n", ret);
    }
}

static void on_conn_params_update(BLE_EvtConnParamUpdate* param)
{
  LOG_OUT("[BLE][LOG]Connection parameters updated\n");
  LOG_OUT("[BLE][LOG]Status: %d\n", param->status);
  LOG_OUT("[BLE][LOG]Minimum interval: %d\n", param->connParams.minConnInterval);
  LOG_OUT("[BLE][LOG]Maximum interval: %d\n", param->connParams.maxConnInterval);
  LOG_OUT("[BLE][LOG]Latency: %d\n", param->connParams.slaveLatency);
  LOG_OUT("[BLE][LOG]Timeout: %d\n", param->connParams.connSupTimeout);
}

typedef struct
{
  uint8_t *data;
  uint16_t data_len;
} data_t;

static int adv_report_parse(uint8_t type, data_t *adv_data, data_t *type_data)
{
  uint16_t index = 0;
  uint8_t *data = adv_data->data;

  while (index < adv_data->data_len)
    {
      uint8_t field_length = data[index];
      uint8_t field_type   = data[index+1];

      if (field_type == type)
        {
          type_data->data   = &data[index+2];
          type_data->data_len = field_length-1;
          type_data->data[type_data->data_len] = '\0';
          return 0;
        }

      index += field_length + 1;
    }

  return -1;
}

static void on_adv_report(BLE_EvtAdvReportData *adv_report)
{
  data_t advReport = {adv_report->data, adv_report->dlen};
  data_t localName;
  uint8_t localNameData[32] = {0};
  localName.data = localNameData;
  localName.data_len = 0;
  int ret;

  if (adv_report_parse(0x09 /* COMPLETE_LOCAL_NAME */, &advReport, &localName) ||
    strncmp((const char *)localName.data, "SONY", strlen("SONY")))
    {
      return;
    }

  LOG_OUT("[BLE][LOG]Peer RSSI:%d Name:%s\n", adv_report->rssi, localName.data);

  ret = BLE_GapStopScan();
  if (BLE_SUCCESS != ret)
    {
      LOG_OUT("BLE_GapStopScan failed, ret=%d\n", ret);
    }

  /* If immediatey disconnect, change slave_latency. */

  ret = BLE_GapConnect(&adv_report->addr);
  if (BLE_SUCCESS != ret)
    {
      LOG_OUT("BLE_GapConnect failed, ret=%d\n", ret);
    }
}

static void on_phy_update_request(BLE_EvtPhyUpdate *phy_update)
{
  LOG_OUT("PHY update request: txPhy 0x%x, rxPhy 0x%x\n",
    phy_update->txPhy, phy_update->rxPhy);

  BLE_GapPhys phy = {phy_update->txPhy, phy_update->rxPhy};
  int ret = BLE_GapPhyUpdate(g_ble_context.ble_conn_handle, &phy);
  if (BLE_SUCCESS != ret)
    {
      LOG_OUT("BLE_GapPhyUpdate failed, ret=%d\n", ret);
    }
}

static void on_phy_update(BLE_EvtPhyUpdate *phy_update)
{
  LOG_OUT("PHY updated: txPhy 0x%x, rxPhy 0x%x\n",
    phy_update->txPhy, phy_update->rxPhy);
}

static void on_data_len_update(BLE_EvtDataLengthUpdate *data_len)
{
  LOG_OUT("Data length updated: maxTx %d, maxRx %d, maxTxTime %d ms, maxRxTime %d ms\n",
    data_len->maxTx, data_len->maxRx, data_len->maxTxTime, data_len->maxRxTime);
}

static
void bleCommEvtDispatch(ble_evt_t *pBleNrfEvt)
{
  BLE_Evt *bleEvent = &commMem.bleEvtCtx->evt;
  bleNrfEvtHandler(bleEvent, pBleNrfEvt);
}

#ifdef BLE_USE_SECTION
static
#endif
void bleEvtDispatch(ble_evt_t *pBleNrfEvt)
{
  BLE_PRT("bleEvtDispatch callbackFlag %d managerCbFlag %d\n",
    commMem.callbackFlag, commMem.managerCbFlag);
  if (commMem.callbackFlag)
    {
      bleCommEvtDispatch(pBleNrfEvt);
    }
  if (commMem.managerCbFlag)
    {
      commMem.managerCb(pBleNrfEvt);
    }
}

#if NRF_SD_BLE_API_VERSION > 5
static
void onAdvSetTerminate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
#ifdef BLE_DBGPRT_ENABLE
  ble_gap_evt_adv_set_terminated_t *set = &pBleNrfEvt->evt.gap_evt.params.adv_set_terminated;
  BLE_PRT("onAdvSetTerminate reason=%d handle=%d num=%d\n",
    set->reason, set->adv_handle, set->num_completed_adv_events);
#endif
  pBleNrfEvt->evt.gap_evt.params.timeout.src = BLE_GAP_TIMEOUT_ADVERTISING;
  onTimeout(pBleEvent, pBleNrfEvt);
}
#endif

static
void onScanReqReport(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  BLE_PRT("onScanReqReort\n");
}

static
void onPhyUpdateReq(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_PHY_UPDATE_REQUEST;
  ble_gap_phys_t *phys = &pBleNrfEvt->evt.gap_evt.params.phy_update_request.peer_preferred_phys;
  BLE_PRT("onPhyUpdateReq tx_phy %d rx_phy %d\n", phys->tx_phys, phys->rx_phys);
  commMem.phyUpdate.txPhy = phys->tx_phys;
  commMem.phyUpdate.rxPhy = phys->rx_phys;
  commMem.phyUpdate.status = 0;
  pBleEvent->evtDataSize = sizeof(BLE_EvtPhyUpdate);
  memcpy(pBleEvent->evtData, &commMem.phyUpdate, pBleEvent->evtDataSize);

  on_phy_update_request((BLE_EvtPhyUpdate *)pBleEvent->evtData);
}

static
void onPhyUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_PHY_UPDATE;
  ble_gap_evt_phy_update_t *phys = &pBleNrfEvt->evt.gap_evt.params.phy_update;
  BLE_PRT("onPhyUpdate stat %d tx_phy %d rx_phy %d\n", phys->status, phys->tx_phy, phys->rx_phy);
  commMem.phyUpdate.txPhy = phys->tx_phy;
  commMem.phyUpdate.rxPhy = phys->rx_phy;
  commMem.phyUpdate.status = phys->status;
  pBleEvent->evtDataSize = sizeof(BLE_EvtPhyUpdate);
  memcpy(pBleEvent->evtData, &commMem.phyUpdate, pBleEvent->evtDataSize);

  on_phy_update((BLE_EvtPhyUpdate *)pBleEvent->evtData);
}

static
void onDataLengthUpdateRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  int ret = 0;
#ifdef BLE_DBGPRT_ENABLE
  ble_gap_data_length_params_t *req = &pBleNrfEvt->evt.gap_evt.params.data_length_update_request.peer_params;
  BLE_PRT("onLenUpReq: max_tx=%d\n", req->max_tx_octets);
  BLE_PRT("onLenUpReq: max_rx=%d\n", req->max_rx_octets);
  BLE_PRT("onLenUpReq: tx_time=%d\n", req->max_tx_time_us);
  BLE_PRT("onLenUpReq: rx_time=%d\n", req->max_rx_time_us);
#endif
  ret = sd_ble_gap_data_length_update(pBleNrfEvt->evt.gap_evt.conn_handle, NULL, NULL);
  if (ret)
    {
      BLE_ERR("onLenUp: sd_ble_gap_data_length_update %d\n", ret);
    }
}

static
void onDataLengthUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_DATA_LENGTH_UPDATE;
  ble_gap_data_length_params_t *datalen = &pBleNrfEvt->evt.gap_evt.params.data_length_update.effective_params;
  BLE_PRT("onLenUp: max_tx=%d\n", datalen->max_tx_octets);
  BLE_PRT("onLenUp: max_rx=%d\n", datalen->max_rx_octets);
  BLE_PRT("onLenUp: tx_time=%d\n", datalen->max_tx_time_us);
  BLE_PRT("onLenUp: rx_time=%d\n", datalen->max_rx_time_us);
  commMem.dataLen = *(BLE_EvtDataLengthUpdate *)datalen;
  pBleEvent->evtDataSize = sizeof(BLE_EvtDataLengthUpdate);
  memcpy(pBleEvent->evtData, &commMem.dataLen, pBleEvent->evtDataSize);

  on_data_len_update((BLE_EvtDataLengthUpdate *)pBleEvent->evtData);
}

static
void onExchangeMtuRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  int ret = 0;
  ble_gatts_evt_exchange_mtu_request_t *req = &pBleNrfEvt->evt.gatts_evt.params.exchange_mtu_request;
  BLE_PRT("onMtuReq: mtu=%d\n", req->client_rx_mtu);
  if (NRF_SDH_BLE_GATT_MAX_MTU_SIZE > req->client_rx_mtu)
    {
      commMem.client_rx_mtu = req->client_rx_mtu;
    }
  else
    {
      commMem.client_rx_mtu = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
    }
  ret = sd_ble_gatts_exchange_mtu_reply(pBleNrfEvt->evt.gatts_evt.conn_handle, commMem.client_rx_mtu);
  if (ret)
    {
      BLE_ERR("onLenUp: sd_ble_gatts_exchange_mtu_reply %d\n", ret);
    }

  pBleEvent->evtHeader = BLE_GATTS_EVENT_EXCHANGE_MTU;
  commMem.gattsExchangeMTU.connHandle = pBleNrfEvt->evt.gatts_evt.conn_handle;
  commMem.gattsExchangeMTU.client_rx_mtu = req->client_rx_mtu;
  pBleEvent->evtDataSize = sizeof(BLE_EvtGattsExchangeMTU);
  memcpy(pBleEvent->evtData, &commMem.gattsExchangeMTU, pBleEvent->evtDataSize);
}

static
void onTxComplete(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_EVENT_TX_COMPLETE;
  if (pBleNrfEvt->header.evt_id == BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE)
    {
      commMem.txCompleteData.connHandle = pBleNrfEvt->evt.gattc_evt.conn_handle;
      commMem.txCompleteData.count = pBleNrfEvt->evt.gattc_evt.params.write_cmd_tx_complete.count;
      commMem.txCompleteData.role = BLE_ROLE_CENTRAL;
    }
  else
    {
      commMem.txCompleteData.connHandle = pBleNrfEvt->evt.gattc_evt.conn_handle;
      commMem.txCompleteData.count = pBleNrfEvt->evt.gattc_evt.params.write_cmd_tx_complete.count;
      commMem.txCompleteData.role = BLE_ROLE_PERIPHERAL;
    }
  pBleEvent->evtDataSize = sizeof(BLE_EvtTxComplete);
  memcpy(pBleEvent->evtData, &commMem.txCompleteData, pBleEvent->evtDataSize);
}

static void onConnect_hal(void)
{
  struct bt_event_conn_stat_t con_stat_evt;

  #ifdef BLE_DBGPRT_ENABLE
  uint8_t connect = 0;
  uint8_t type = 0;
  uint8_t reason = 0;
  #endif
  /* Copy device address */

  memcpy(&con_stat_evt.addr, commMem.connectData.addr.addr, sizeof(BT_ADDR));

  /* Copy connect status */

  con_stat_evt.connected = true;

  /* Copy type */
//   type = commMem.connectData.addr.type;
//   con_stat_evt.status = reason;

  con_stat_evt.group_id = BT_GROUP_COMMON;
  con_stat_evt.event_id = BT_COMMON_EVENT_CONN_STAT_CHANGE;

  BLE_PRT("addr: %02x.%02x.%02x.%02x.%02x,%02x is connect:%d,"
      "transport type: %x, disconnect reason: %x.\n",
    con_stat_evt.addr.address[0], con_stat_evt.addr.address[1],
    con_stat_evt.addr.address[2], con_stat_evt.addr.address[3],
    con_stat_evt.addr.address[4], con_stat_evt.addr.address[5],
    connect, type, reason);

  bt_common_event_handler((struct bt_event_t *) &con_stat_evt);
}

static
void onConnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_CONNECTED;
  ble_gap_evt_connected_t *connected = &pBleNrfEvt->evt.gap_evt.params.connected;
  commMem.connectData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
  commMem.connectData.addr.type = connected->peer_addr.addr_type;
  memcpy(commMem.connectData.addr.addr, connected->peer_addr.addr, BLE_GAP_ADDR_LENGTH);
  if (connected->role == BLE_GAP_ROLE_PERIPH )
    {
      commMem.connectData.role   = BLE_ROLE_PERIPHERAL;
    }
  else if (connected->role == BLE_GAP_ROLE_CENTRAL)
    {
      commMem.connectData.role   = BLE_ROLE_CENTRAL;
    }
  commMem.gapMem->connParams = *(ble_gap_conn_params_t *)&connected->conn_params;
#ifdef BLE_DBGPRT_ENABLE
  BLE_PRT("onConnect: handle=%d\n", commMem.connectData.handle);
  BLE_PRT("onConnect: role=%d\n", commMem.connectData.role);
  BLE_PRT("onConnect: type=%d\n", commMem.connectData.addr.type);
  BLE_PRT("onConnect: addr=0x");
  for (int i=0; i<6; i++) {
    BLE_PRT("%02x", commMem.connectData.addr.addr[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("onConnect: minConnInterval=%d\n", commMem.gapMem->connParams.min_conn_interval);
  BLE_PRT("onConnect: maxConnInterval=%d\n", commMem.gapMem->connParams.max_conn_interval);
  BLE_PRT("onConnect: slaveLatency=%d\n", commMem.gapMem->connParams.slave_latency);
  BLE_PRT("onConnect: connSupTimeout=%d\n", commMem.gapMem->connParams.conn_sup_timeout);
#endif
  commMem.gapMem->wrapperBondInfo.connHandle = pBleNrfEvt->evt.gap_evt.conn_handle;
  commMem.gapMem->wrapperBondInfo.bondInfo.addrType = connected->peer_addr.addr_type;
  memcpy(commMem.gapMem->wrapperBondInfo.bondInfo.addr, connected->peer_addr.addr, BLE_GAP_ADDR_LENGTH);
  pBleEvent->evtDataSize = sizeof(BLE_EvtConnected);
  memcpy(pBleEvent->evtData, &commMem.connectData, pBleEvent->evtDataSize);

  //notify to hal layer
  onConnect_hal();

  on_connected((BLE_EvtConnected*)pBleEvent->evtData);
}

static
void onConnParamUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_CONN_PARAM_UPDATE;
  ble_gap_conn_params_t *params = &pBleNrfEvt->evt.gap_evt.params.conn_param_update.conn_params;
  commMem.connParams.connParams.minConnInterval = params->min_conn_interval;
  commMem.connParams.connParams.maxConnInterval = params->max_conn_interval;
  commMem.connParams.connParams.slaveLatency = params->slave_latency;
  commMem.connParams.connParams.connSupTimeout = params->conn_sup_timeout;
  commMem.gapMem->connParams = *(ble_gap_conn_params_t *)params;
  pBleEvent->evtDataSize = sizeof(BLE_EvtConnParamUpdate);
  memcpy(pBleEvent->evtData, &commMem.connParams, pBleEvent->evtDataSize);

  on_conn_params_update((BLE_EvtConnParamUpdate*)pBleEvent->evtData);
}

static
void onConnParamUpdateRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_CONN_PARAM_UPDATE_REQUEST;
  ble_gap_conn_params_t *params = &pBleNrfEvt->evt.gap_evt.params.conn_param_update_request.conn_params;
  commMem.connParams.connParams.minConnInterval = params->min_conn_interval;
  commMem.connParams.connParams.maxConnInterval = params->max_conn_interval;
  commMem.connParams.connParams.slaveLatency = params->slave_latency;
  commMem.connParams.connParams.connSupTimeout = params->conn_sup_timeout;
  commMem.gapMem->connParams = *(ble_gap_conn_params_t *)params;
  pBleEvent->evtDataSize = sizeof(BLE_EvtConnParamUpdate);
  memcpy(pBleEvent->evtData, &commMem.connParams, pBleEvent->evtDataSize);
}

static
void onDisconnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_DISCONNECTED;
  commMem.disconnectData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
  commMem.disconnectData.reason = pBleNrfEvt->evt.gap_evt.params.disconnected.reason;
  BLE_PRT("onDisconnect: reason=%d\n", pBleNrfEvt->evt.gap_evt.params.disconnected.reason);
  pBleEvent->evtDataSize = sizeof(BLE_EvtDisconnected);
  memcpy(pBleEvent->evtData, &commMem.disconnectData, pBleEvent->evtDataSize);
  commMem.gapMem->is_connected = false;

  on_disconnected((BLE_EvtDisconnected*)pBleEvent->evtData);
}

static
void onSecParamsRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  ble_gap_evt_sec_params_request_t *secParamReq = &pBleNrfEvt->evt.gap_evt.params.sec_params_request;
  pBleEvent->evtHeader = BLE_GAP_EVENT_EXCHANGE_FEATURE;
  commMem.exchangeFeatureData.handle              = pBleNrfEvt->evt.gap_evt.conn_handle;
  commMem.exchangeFeatureData.peerFeature.oob     = (BLE_GapOOB)secParamReq->peer_params.oob;
  commMem.exchangeFeatureData.peerFeature.ioCap   = (BLE_GAP_IO_CAP)secParamReq->peer_params.io_caps;
  commMem.exchangeFeatureData.peerFeature.authReq = (BLE_GapAuth)((secParamReq->peer_params.mitm)<<1 |(secParamReq->peer_params.bond));
  commMem.exchangeFeatureData.peerFeature.maxKeySize = secParamReq->peer_params.max_key_size;
  commMem.exchangeFeatureData.peerFeature.minKeySize = secParamReq->peer_params.min_key_size;
  pBleEvent->evtDataSize = sizeof(BLE_EvtExchangeFeature);
  memcpy(pBleEvent->evtData, &commMem.exchangeFeatureData, pBleEvent->evtDataSize);

  on_exchange_feature((BLE_EvtExchangeFeature*)pBleEvent->evtData);
}

static
void onAuthStatus(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_AUTH_STATUS;
  ble_gap_evt_auth_status_t *authStatus = &pBleNrfEvt->evt.gap_evt.params.auth_status;
  commMem.authStatusData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
#ifdef BLE_DBGPRT_ENABLE
  BLE_PRT("onAuthStatus: handle=%d\n", pBleNrfEvt->evt.gap_evt.conn_handle);
  BLE_PRT("onAuthStatus: auth_status=%d\n", authStatus->auth_status);
  BLE_PRT("onAuthStatus: error_src=%d\n", authStatus->error_src);
  BLE_PRT("onAuthStatus: bonded=%d\n", authStatus->bonded);
  BLE_PRT("onAuthStatus: lesc=%d\n", authStatus->lesc);
  BLE_PRT("onAuthStatus: sm1_levels_lv1=%d\n", authStatus->sm1_levels.lv1);
  BLE_PRT("onAuthStatus: sm1_levels_lv2=%d\n", authStatus->sm1_levels.lv2);
  BLE_PRT("onAuthStatus: sm1_levels_lv3=%d\n", authStatus->sm1_levels.lv3);
  BLE_PRT("onAuthStatus: sm2_levels_lv1=%d\n", authStatus->sm2_levels.lv1);
  BLE_PRT("onAuthStatus: sm2_levels_lv2=%d\n", authStatus->sm2_levels.lv2);
  BLE_PRT("onAuthStatus: sm2_levels_lv3=%d\n", authStatus->sm2_levels.lv3);
  BLE_PRT("onAuthStatus: own.enc=%d\n", authStatus->kdist_own.enc);
  BLE_PRT("onAuthStatus: own.id=%d\n", authStatus->kdist_own.id);
  BLE_PRT("onAuthStatus: own.sign=%d\n", authStatus->kdist_own.sign);
  BLE_PRT("onAuthStatus: own.link=%d\n", authStatus->kdist_own.link);
  BLE_PRT("onAuthStatus: peer.enc=%d\n", authStatus->kdist_peer.enc);
  BLE_PRT("onAuthStatus: peer.id=%d\n", authStatus->kdist_peer.id);
  BLE_PRT("onAuthStatus: peer.sign=%d\n", authStatus->kdist_peer.sign);
  BLE_PRT("onAuthStatus: peer.link=%d\n", authStatus->kdist_peer.link);
#if 0
  // own
  BLE_PRT("onAuthStatus: own id_irk=0x");
  for (int i=0; i<16; i++) {
    BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownIdKey.id_info.irk[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("onAuthStatus: own id_addr_type=%d\n", commMem.gapMem->wrapperBondInfo.ownIdKey.id_addr_info.addr_type);
  BLE_PRT("onAuthStatus: own id_addr=0x");
  for (int i=0; i<6; i++) {
    BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownIdKey.id_addr_info.addr[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("onAuthStatus: own master_id_ediv=%d\n", commMem.gapMem->wrapperBondInfo.ownEncKey.master_id.ediv);
  BLE_PRT("onAuthStatus: own master_id_rand=0x");
  for (int i=0; i<8; i++) {
    BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownEncKey.master_id.rand[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("onAuthStatus: own enc_info_auth=%d\n", commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.auth);
  if (commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk_len) {
    BLE_PRT("onAuthStatus: own enc_info_ltk=0x");
    for (int i=0; i<commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk_len; i++) {
      BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk[i]);
    }
    BLE_PRT("\n");
  } else {
    BLE_PRT("onAuthStatus: own enc_info_ltk=%d\n", commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk_len);
  }
#endif
  // peer
  BLE_PRT("onAuthStatus: peer id_irk=0x");
  for (int i=0; i<16; i++)
    {
      BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerIdKey.id_info.irk[i]);
    }
  BLE_PRT("\n");
  BLE_PRT("onAuthStatus: peer id_addr_type=%d\n", commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr_type);
  BLE_PRT("onAuthStatus: peer id_addr=0x");
  for (int i=0; i<6; i++)
    {
      BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr[i]);
    }
  BLE_PRT("\n");

  BLE_PRT("onAuthStatus: peer master_id_ediv=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.master_id.ediv);
  BLE_PRT("onAuthStatus: peer master_id_rand=0x");
  for (int i=0; i<8; i++)
    {
      BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerEncKey.master_id.rand[i]);
    }
  BLE_PRT("\n");
  BLE_PRT("onAuthStatus: peer enc_info_auth=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.auth);
  if (commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len)
    {
      BLE_PRT("onAuthStatus: peer enc_info_ltk=0x");
      for (int i=0; i<commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len; i++)
        {
          BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk[i]);
        }
      BLE_PRT("\n");
    }
  else
    {
      BLE_PRT("onAuthStatus: peer enc_info_ltk=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len);
    }
#endif
  //Auth status code.
  if ( authStatus->auth_status < BLE_GAP_SEC_STATUS_PDU_INVALID )
    {
      commMem.authStatusData.status = authStatus->auth_status;
    }
  else if ((authStatus->auth_status > BLE_GAP_SEC_STATUS_RFU_RANGE1_END) && (authStatus->auth_status != BLE_GAP_SEC_STATUS_RFU_RANGE2_END))
    {
      commMem.authStatusData.status = authStatus->auth_status -(BLE_GAP_SEC_STATUS_RFU_RANGE1_END - BLE_GAP_SM_STATUS_RESERVED);
    }
  else
    {
      commMem.authStatusData.status = BLE_GAP_SM_STATUS_RESERVED;
    }
#if 1
  if (authStatus->kdist_peer.id)
    {
      BLE_PRT("update bondinfo\n");
      commMem.gapMem->wrapperBondInfo.bondInfo.addrType = commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr_type;
      memcpy(commMem.gapMem->wrapperBondInfo.bondInfo.addr, commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr, BLE_GAP_ADDR_LENGTH);
    }
#endif
  memcpy(&commMem.authStatusData.bondInfo, &commMem.gapMem->wrapperBondInfo.bondInfo, sizeof(BLE_GapBondInfo));
  pBleEvent->evtDataSize = sizeof(BLE_EvtAuthStatus);
  memcpy(pBleEvent->evtData, &commMem.authStatusData, pBleEvent->evtDataSize);

  on_auth_status((BLE_EvtAuthStatus*)pBleEvent->evtData);
}

static
void onDispPasskey(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_DISPLAY_PASSKEY;
  ble_gap_evt_passkey_display_t *dispPasskey = &pBleNrfEvt->evt.gap_evt.params.passkey_display;
  commMem.dispPasskeyData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
  memcpy(commMem.dispPasskeyData.passkey,dispPasskey->passkey,BLE_GAP_PASSKEY_LEN);
  commMem.dispPasskeyData.passkey[BLE_GAP_PASSKEY_LEN] = '\0';
  pBleEvent->evtDataSize = sizeof(BLE_EvtDisplayPasskey);
  memcpy(pBleEvent->evtData, &commMem.dispPasskeyData, pBleEvent->evtDataSize);

  on_disp_passkey((BLE_EvtDisplayPasskey*)pBleEvent->evtData);
}

#if NRF_SD_BLE_API_VERSION > 5
extern uint8_t bleAdvReportBuffer[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN];
#endif

static
void onAdvReport(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_ADV_REPORT;
  ble_gap_evt_adv_report_t *advReport = &pBleNrfEvt->evt.gap_evt.params.adv_report;
#if NRF_SD_BLE_API_VERSION > 5
  BLE_PRT("onAdvReport: rssi=%d scan_rsp=%d type=%d dlen=%d\n",
    advReport->rssi, advReport->type.scan_response, advReport->type.status, advReport->data.len);
  BLE_PRT("             connectable=%d scannable=%d directed=%d ext_pdu=%d set=%d dat=%d\n",
    advReport->type.connectable, advReport->type.scannable, advReport->type.directed,
    advReport->type.extended_pdu, advReport->set_id, advReport->data_id);

  if (advReport->type.status == BLE_GAP_ADV_DATA_STATUS_COMPLETE)
    {
      commMem.advReportData.dlen = advReport->data.len;
      memcpy(commMem.advReportData.data, advReport->data.p_data, advReport->data.len);
    }

  commMem.advReportData.scan_rsp = advReport->type.scan_response;
  commMem.advReportData.dlen = advReport->data.len;
  memcpy(commMem.advReportData.data, advReport->data.p_data, advReport->data.len);
  commMem.advReportData.rssi = advReport->rssi;
  memcpy(commMem.advReportData.addr.addr, advReport->peer_addr.addr, BLE_GAP_ADDR_LENGTH);
  commMem.advReportData.addr.type = advReport->peer_addr.addr_type;
  pBleEvent->evtDataSize = sizeof(BLE_EvtAdvReportData);
  memcpy(pBleEvent->evtData, &commMem.advReportData, pBleEvent->evtDataSize);
  ble_data_t bleAdvReport = {bleAdvReportBuffer, BLE_GAP_SCAN_BUFFER_EXTENDED_MIN};
  if (sd_ble_gap_scan_start(NULL, &bleAdvReport))
    {
      BLE_ERR("sd_ble_gap_scan_start failed.\n");
    }
#else
  BLE_PRT("onAdvReport: rssi=%d scan_rsp=%d type=%d dlen=%d\n",
    advReport->rssi, advReport->scan_rsp, advReport->type, advReport->dlen);
  commMem.advReportData.scan_rsp = advReport->scan_rsp;
  commMem.advReportData.dlen = advReport->dlen;
  memcpy(commMem.advReportData.data, advReport->data, BLE_GAP_ADV_MAX_SIZE);
  commMem.advReportData.rssi = advReport->rssi;
  memcpy(commMem.advReportData.addr.addr, advReport->peer_addr.addr, BLE_GAP_ADDR_LENGTH);
  commMem.advReportData.addr.type = advReport->peer_addr.addr_type;
  pBleEvent->evtDataSize = sizeof(BLE_EvtAdvReportData);
  memcpy(pBleEvent->evtData, &commMem.advReportData, pBleEvent->evtDataSize);
#endif

  on_adv_report((BLE_EvtAdvReportData *)pBleEvent->evtData);
}

static
void onAuthKeyRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_AUTH_KEY_REQUEST;
  ble_gap_evt_auth_key_request_t *authKey = &pBleNrfEvt->evt.gap_evt.params.auth_key_request;
  commMem.authKeyData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
  commMem.authKeyData.keyType = authKey->key_type;
  pBleEvent->evtDataSize = sizeof(BLE_EvtAuthKey);
  memcpy(pBleEvent->evtData, &commMem.authKeyData, pBleEvent->evtDataSize);

  on_auth_key_request((BLE_EvtAuthKey *)pBleEvent->evtData);
}

static
void onConnSecUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_CONN_SEC_UPDATE;
  pBleEvent->evtDataSize = 0;
#ifdef BLE_DBGPRT_ENABLE
  ble_gap_evt_conn_sec_update_t *sec = &pBleNrfEvt->evt.gap_evt.params.conn_sec_update;
  BLE_PRT("onConnSecUpdate: keysize=%d sm=%d lv=%d\n", sec->conn_sec.encr_key_size,
    sec->conn_sec.sec_mode.sm, sec->conn_sec.sec_mode.lv);
#endif
}

static
void onSecInfoRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  ble_gap_enc_info_t *enc_info = NULL;
  ble_gap_irk_t *id_info = NULL;
  ble_gap_sign_info_t *sign_info = NULL;
  ble_gap_evt_sec_info_request_t *secinfo = &pBleNrfEvt->evt.gap_evt.params.sec_info_request;
#ifdef BLE_DBGPRT_ENABLE
  BLE_PRT("onSecInfoRequest: handle=%d\n", pBleNrfEvt->evt.gap_evt.conn_handle);
  BLE_PRT("onSecInfoRequest: addr_type=%d\n", secinfo->peer_addr.addr_type);
  BLE_PRT("onSecInfoRequest: peer_addr=0x");
  for (int i=0; i<6; i++)
    {
      BLE_PRT("%02x", secinfo->peer_addr.addr[i]);
    }
  BLE_PRT("\n");
  BLE_PRT("onSecInfoRequest: master_id_ediv=%d\n", secinfo->master_id.ediv);
  BLE_PRT("onSecInfoRequest: master_id_rand=0x");
  for (int i=0; i<8; i++)
    {
      BLE_PRT("%02x", secinfo->master_id.rand[i]);
    }
  BLE_PRT("\n");
  BLE_PRT("onSecInfoRequest: enc_info=%d\n", secinfo->enc_info);
  BLE_PRT("onSecInfoRequest: id_info=%d\n", secinfo->id_info);
  BLE_PRT("onSecInfoRequest: sign_info=%d\n", secinfo->sign_info);
#if 0
  // own
  BLE_PRT("onSecInfoRequest: own id_irk=0x");
  for (int i=0; i<16; i++) {
    BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownIdKey.id_info.irk[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("onSecInfoRequest: own id_addr_type=%d\n", commMem.gapMem->wrapperBondInfo.ownIdKey.id_addr_info.addr_type);
  BLE_PRT("onSecInfoRequest: own id_addr=0x");
  for (int i=0; i<6; i++) {
    BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownIdKey.id_addr_info.addr[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("onSecInfoRequest: own master_id_ediv=%d\n", commMem.gapMem->wrapperBondInfo.ownEncKey.master_id.ediv);
  BLE_PRT("onSecInfoRequest: own master_id_rand=0x");
  for (int i=0; i<8; i++) {
    BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownEncKey.master_id.rand[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("onSecInfoRequest: own enc_info_auth=%d\n", commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.auth);
  if (commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk_len) {
    BLE_PRT("onSecInfoRequest: own enc_info_ltk=0x");
    for (int i=0; i<commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk_len; i++) {
      BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk[i]);
    }
    BLE_PRT("\n");
  } else {
    BLE_PRT("onSecInfoRequest: own enc_info_ltk=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len);
  }
#endif
  // peer
  BLE_PRT("onSecInfoRequest: peer id_irk=0x");
  for (int i=0; i<16; i++)
    {
      BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerIdKey.id_info.irk[i]);
    }
  BLE_PRT("\n");
  BLE_PRT("onSecInfoRequest: peer id_addr_type=%d\n", commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr_type);
  BLE_PRT("onSecInfoRequest: peer id_addr=0x");
  for (int i=0; i<6; i++)
    {
      BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr[i]);
    }
  BLE_PRT("\n");
  BLE_PRT("onSecInfoRequest: peer master_id_ediv=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.master_id.ediv);
  BLE_PRT("onSecInfoRequest: peer master_id_rand=0x");
  for (int i=0; i<8; i++)
    {
      BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerEncKey.master_id.rand[i]);
    }
  BLE_PRT("\n");
  BLE_PRT("onSecInfoRequest: peer enc_info_auth=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.auth);
  if (commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len)
    {
      BLE_PRT("onSecInfoRequest: peer enc_info_ltk=0x");
      for (int i=0; i<commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len; i++)
        {
          BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk[i]);
        }
      BLE_PRT("\n");
    }
  else
    {
      BLE_PRT("onSecInfoRequest: peer enc_info_ltk=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len);
    }
#endif
  uint32_t list = bleBondEnableList;
  for(int index = 0; index < BLE_SAVE_BOND_DEVICE_MAX_NUM; index++, list >>= 1)
    {
      if (!(list & 1))
        {
          continue;
        }
      if(!memcmp(BondInfoInFlash[index].peerEncKey.master_id.rand, secinfo->master_id.rand, BLE_GAP_SEC_RAND_LEN))
        {
          BLE_PRT("onSecInfoRequest: master_id exitsting index %d\n", index);
          enc_info = &BondInfoInFlash[index].peerEncKey.enc_info;
          id_info = &BondInfoInFlash[index].peerIdKey.id_info;
          break;
        }
    }
  sd_ble_gap_sec_info_reply(pBleNrfEvt->evt.gap_evt.conn_handle, enc_info, id_info, sign_info);
}

static
void onTimeout(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_TIMEOUT;
  ble_gap_evt_timeout_t *timeout = &pBleNrfEvt->evt.gap_evt.params.timeout;
  commMem.timeoutData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
  commMem.timeoutData.timeoutSrc = timeout->src;
  pBleEvent->evtDataSize = sizeof(BLE_EvtTimeout);
  memcpy(pBleEvent->evtData, &commMem.timeoutData, pBleEvent->evtDataSize);

  on_timeout((BLE_EvtTimeout *)pBleEvent->evtData);
}

static
void onGattsTimeout(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GATTS_EVENT_TIMEOUT;
  commMem.timeoutData.handle = pBleNrfEvt->evt.gatts_evt.conn_handle;
  commMem.timeoutData.timeoutSrc = BLE_GATT_TIMEOUT_PROTOCOL;
  pBleEvent->evtDataSize = sizeof(BLE_EvtTimeout);
  memcpy(pBleEvent->evtData, &commMem.timeoutData, pBleEvent->evtDataSize);
}

static
void onRssiChanged(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_RSSI_CHANGED;
  ble_gap_evt_rssi_changed_t *rssiChanged = &pBleNrfEvt->evt.gap_evt.params.rssi_changed;
  commMem.rssiChangeData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
  commMem.rssiChangeData.rssi = rssiChanged->rssi;
  commMem.gapMem->peerRssi = rssiChanged->rssi;
  pBleEvent->evtDataSize = sizeof(BLE_EvtRssiChanged);
  memcpy(pBleEvent->evtData, &commMem.rssiChangeData, pBleEvent->evtDataSize);

  on_rssi_changed((BLE_EvtRssiChanged*)pBleEvent->evtData);
}

static
void onGattsWrite(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GATTS_EVENT_WRITE;
  ble_gatts_evt_write_t * gattsWrite = &pBleNrfEvt->evt.gatts_evt.params.write;
  commMem.gattsWriteData.connHandle = pBleNrfEvt->evt.gatts_evt.conn_handle;
  commMem.gattsWriteData.handle  = gattsWrite->handle;  //attribute handle
  commMem.gattsWriteData.dataLen = gattsWrite->len;
  commMem.gattsWriteData.offset  = gattsWrite->offset;
  memcpy(commMem.gattsWriteData.data, gattsWrite->data, MAX_RCV_DATA_LENGTH);
  pBleEvent->evtDataSize = sizeof(BLE_EvtGattsWrite);
  memcpy(pBleEvent->evtData, &commMem.gattsWriteData, pBleEvent->evtDataSize);

  on_gatts_evt_write((BLE_EvtGattsWrite*)pBleEvent->evtData);
}

static
void onGattsIndConfirm(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GATTS_EVENT_CFM;
  ble_gatts_evt_hvc_t * gattsConfirm = &pBleNrfEvt->evt.gatts_evt.params.hvc;
  commMem.gattsIndConfirmData.connHandle    = pBleNrfEvt->evt.gatts_evt.conn_handle;
  commMem.gattsIndConfirmData.handle        = gattsConfirm->handle;
  pBleEvent->evtDataSize = sizeof(BLE_EvtGattsIndConfirm);
  memcpy(pBleEvent->evtData, &commMem.gattsIndConfirmData, pBleEvent->evtDataSize);
}

static void onReadRsp(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GATTC_EVENT_READ;
  ble_gattc_evt_t *bleGattcEvt                         = &(pBleNrfEvt->evt.gattc_evt);
  const ble_gattc_evt_read_rsp_t *bleGattcEvtReadRsp   = &(bleGattcEvt->params.read_rsp);
  commMem.gattcReadData.connHandle    = pBleNrfEvt->evt.gattc_evt.conn_handle;
  commMem.gattcReadData.charValHandle = bleGattcEvtReadRsp->handle;
  commMem.gattcReadData.charValLen    = bleGattcEvtReadRsp->len;
  memcpy(commMem.gattcReadData.charValData, bleGattcEvtReadRsp->data, MAX_VAL_DATA_LENGTH);
  pBleEvent->evtDataSize = sizeof(BLE_EvtGattcRead);
  memcpy(pBleEvent->evtData, &commMem.gattcReadData, pBleEvent->evtDataSize);

  on_read_rsp((BLE_EvtGattcRead *)pBleEvent->evtData);
}

static void onWriteRsp(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GATTC_EVENT_WRITE_RSP;
  pBleEvent->evtDataSize = sizeof(BLE_EvtGattcWriteRsp);

  BLE_EvtGattcWriteRsp* wrRsp = (BLE_EvtGattcWriteRsp*)pBleEvent->evtData;
  ble_gattc_evt_t *bleGattcEvt                         = &(pBleNrfEvt->evt.gattc_evt);
  const ble_gattc_evt_read_rsp_t *bleGattcEvtReadRsp   = &(bleGattcEvt->params.read_rsp);

  wrRsp->connHandle    = bleGattcEvt->conn_handle;
  wrRsp->charValHandle = bleGattcEvtReadRsp->handle;
  wrRsp->status        = bleGattcEvt->gatt_status;
}

static
void onHvx(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GATTC_EVENT_NTFIND;
  ble_gattc_evt_t *bleGattcEvt                = &(pBleNrfEvt->evt.gattc_evt);
  const ble_gattc_evt_hvx_t *bleGattcEvtHvx   = &(bleGattcEvt->params.hvx);
  commMem.gattcNtfIndData.connHandle   = pBleNrfEvt->evt.gattc_evt.conn_handle;
  commMem.gattcNtfIndData.attrHandle   = bleGattcEvtHvx->handle;
  commMem.gattcNtfIndData.type         = (BLE_GattNtyIndType)bleGattcEvtHvx->type;
  commMem.gattcNtfIndData.attrValLen   = bleGattcEvtHvx->len;
  memcpy(commMem.gattcNtfIndData.attrValData, bleGattcEvtHvx->data, MAX_VAL_DATA_LENGTH);
  pBleEvent->evtDataSize = sizeof(BLE_EvtGattcNtfInd);
  memcpy(pBleEvent->evtData, &commMem.gattcNtfIndData, pBleEvent->evtDataSize);
}

static
void onSysAttrMissing(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  int ret = BLE_SUCCESS;
  ret = sd_ble_gatts_sys_attr_set(commMem.connectData.handle, NULL, 0, 0);
  if (ret) {
    pBleEvent->evtHeader = BLE_GATTS_EVENT_SYS_ATTR_MISSING;
  }
}

static
void setDbDiscoveryEvent(BLE_Evt *pBleEvent, uint16_t connHandle, int result, int reason)
{
  commMem.gattcDbDiscoveryData.connHandle = connHandle;
  commMem.gattcDbDiscoveryData.result = result;
  commMem.gattcDbDiscoveryData.params.reason = reason;
  if (result == BLE_GATTC_RESULT_SUCCESS) {
    commMem.gattcDb.dbDiscovery.srvCount = commMem.gattcDb.currSrvInd;
    memcpy(&commMem.gattcDbDiscoveryData.params.dbDiscovery, &commMem.gattcDb.dbDiscovery, sizeof(BLE_GattcDbDiscovery));
  }
  pBleEvent->evtHeader = BLE_GATTC_EVENT_DBDISCOVERY;
  pBleEvent->evtDataSize = sizeof(BLE_EvtGattcDbDiscovery);
  memcpy(pBleEvent->evtData, &commMem.gattcDbDiscoveryData, pBleEvent->evtDataSize);
  commMem.gattcDb.currCharInd = 0;
  commMem.gattcDb.currSrvInd  = 0;

  on_db_discovery((BLE_EvtGattcDbDiscovery *)pBleEvent->evtData);
  return;
}

static
void onPrimarySrvDiscoveryRsp(bleGattcDb *gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  uint16_t connHandle = 0;
  ble_gattc_evt_t *bleGattcEvt = NULL;
  BLE_GattcDbDiscSrv                   *srvBeingDiscovered = NULL;

  bleGattcEvt = &(pBleNrfEvt->evt.gattc_evt);
  connHandle  = bleGattcEvt->conn_handle;
  BLE_PRT2("onPrimary start ind %d\n", gattcDbDiscovery->currSrvInd);
  if (bleGattcEvt->gatt_status == BLE_GATT_STATUS_SUCCESS)
    {
      if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
        {
          gattcDbDiscovery->discoveryInProgress = false;
          BLE_ERR("onPrimary MAX_SRV\n");
          setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_SERVICE);
          return;
        }
      gattcDbDiscovery->dbDiscovery.connHandle = connHandle;
      srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);

      const ble_gattc_evt_prim_srvc_disc_rsp_t *primSrvcDiscRspEvt = &(bleGattcEvt->params.prim_srvc_disc_rsp);
      srvBeingDiscovered->srvUuid.value.baseAlias.uuidAlias = primSrvcDiscRspEvt->services[0].uuid.uuid; //BLE_ENABLE_NORDIC_ORIGINAL
      srvBeingDiscovered->srvUuid.type = (BLE_GATT_UUID_TYPE)primSrvcDiscRspEvt->services[0].uuid.type; //BLE_ENABLE_NORDIC_ORIGINAL
      srvBeingDiscovered->srvHandleRange.startHandle =  primSrvcDiscRspEvt->services[0].handle_range.start_handle;
      srvBeingDiscovered->srvHandleRange.endHandle =  primSrvcDiscRspEvt->services[0].handle_range.end_handle;
      BLE_PRT2("onPrimary cnt %d type %d uuid 0x%04X sHdl %d eHdl %d\n",
        primSrvcDiscRspEvt->count,
        primSrvcDiscRspEvt->services[0].uuid.type,
        primSrvcDiscRspEvt->services[0].uuid.uuid,
        primSrvcDiscRspEvt->services[0].handle_range.start_handle,
        primSrvcDiscRspEvt->services[0].handle_range.end_handle);
      //get current service info finished. characteristic discovery which belongs to this service.
      (void)characteristicsDiscover(pBleEvent, gattcDbDiscovery);
    }
  else if (bleGattcEvt->gatt_status == BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND)
    {
      //db discovery complete.send success event to application
      gattcDbDiscovery->discoveryInProgress = true;
      setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_SUCCESS, 0);
    }
  else
    {
      //db discovery error.send event to application
      gattcDbDiscovery->discoveryInProgress = false;
      BLE_ERR("onPrimary Status error %d\n", bleGattcEvt->gatt_status);
      setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_SERVICE);
    }
}

static
void onCharacteristicDiscoveryRsp(bleGattcDb *const gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  uint8_t numCharsPrevDisc = 0;
  uint8_t numCharsCurrDisc = 0;
  uint16_t connHandle = 0;
  uint32_t i               = 0;
  uint32_t j               = 0;
  bool   performDescDiscov = false;
  bool raiseDiscovComplete = false;
  ble_gattc_evt_t                 *bleGattcEvt = &(pBleNrfEvt->evt.gattc_evt);
  BLE_GattcDbDiscSrv       *srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
  BLE_GattcChar            *lastKnownChar = NULL;

  connHandle  = bleGattcEvt->conn_handle;
  if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
    {
      BLE_ERR("onChar ind NG\n");
      gattcDbDiscovery->discoveryInProgress = false;
      setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_CHARACTERISTIC);
      return;
    }
  if (bleGattcEvt->gatt_status == BLE_GATT_STATUS_SUCCESS)
    {
      const ble_gattc_evt_char_disc_rsp_t * charDiscRspEvt;

      charDiscRspEvt = &(bleGattcEvt->params.char_disc_rsp);

      numCharsPrevDisc = srvBeingDiscovered->charCount;
      numCharsCurrDisc = charDiscRspEvt->count;
      BLE_PRT2("onChar preCnt %d curCnt %d\n", numCharsPrevDisc, numCharsCurrDisc);
      // Check if the total number of discovered characteristics are supported
      if ((numCharsPrevDisc + numCharsCurrDisc) <= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV)
        {
          // Update the characteristics count.
          srvBeingDiscovered->charCount += numCharsCurrDisc;
        }
      else
        {
          // The number of characteristics discovered at the peer is more than the supported maximum.
          srvBeingDiscovered->charCount = BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV;
        }

      for (i = numCharsPrevDisc, j = 0; i < srvBeingDiscovered->charCount; i++, j++)
        {
          memcpy(&srvBeingDiscovered->characteristics[i].characteristic.charPrope, &charDiscRspEvt->chars[j].char_props, sizeof(BLE_CharPrope));
          srvBeingDiscovered->characteristics[i].characteristic.charDeclhandle = charDiscRspEvt->chars[j].handle_decl;
          srvBeingDiscovered->characteristics[i].characteristic.charValhandle  = charDiscRspEvt->chars[j].handle_value;
          srvBeingDiscovered->characteristics[i].characteristic.charValUuid.type = (BLE_GATT_UUID_TYPE)charDiscRspEvt->chars[j].uuid.type; //BLE_ENABLE_NORDIC_ORIGINAL
          srvBeingDiscovered->characteristics[i].characteristic.charValUuid.value.baseAlias.uuidAlias = charDiscRspEvt->chars[j].uuid.uuid; //BLE_ENABLE_NORDIC_ORIGINAL
          srvBeingDiscovered->characteristics[i].cccdHandle = BLE_GATT_HANDLE_INVALID;

          BLE_PRT2("onChar type %d uuid 0x%04X ValHdl %d DecHdl %d\n",
            charDiscRspEvt->chars[j].uuid.type,
            charDiscRspEvt->chars[j].uuid.uuid,
            charDiscRspEvt->chars[j].handle_value,
            charDiscRspEvt->chars[j].handle_decl);
        }

      lastKnownChar = &(srvBeingDiscovered->characteristics[i - 1].characteristic);

      // If no more characteristic discovery is required, or if the maximum number of supported
      // characteristic per service has been reached, descriptor discovery will be performed.
      if (!isCharDiscoveryReqd(gattcDbDiscovery, lastKnownChar) ||
      (srvBeingDiscovered->charCount == BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV))
        {
          performDescDiscov = true;
        }
      else
        {
          // Update the current characteristic index.
          gattcDbDiscovery->currCharInd = srvBeingDiscovered->charCount;
          // Perform another round of characteristic discovery.
          BLE_PRT2("onChar charDiscover\n");
          (void)characteristicsDiscover(pBleEvent, gattcDbDiscovery);
        }
    }
  else
    {
      // The previous characteristic discovery resulted in no characteristics.
      // descriptor discovery should be performed.
      performDescDiscov = true;
    }

  if (performDescDiscov)
    {
      gattcDbDiscovery->currCharInd = 0;
      BLE_PRT2("onChar charDiscover\n");
      (void)descriptorsDiscover(pBleEvent, gattcDbDiscovery, &raiseDiscovComplete);
      if (raiseDiscovComplete)
        {
          BLE_PRT2("onChar onSrvDiscCompletion\n");
          onSrvDiscCompletion(pBleEvent, gattcDbDiscovery);
        }
    }
}

static
void onDescriptorDiscoveryRsp(bleGattcDb *const gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  uint32_t i = 0;
  uint16_t connHandle = 0;
  bool raiseDiscovComplete = false;
  BLE_GattcDbDiscSrv *srvBeingDiscovered = NULL;
  ble_gattc_evt_t *bleGattcEvt = NULL;
  bleGattcEvt = &(pBleNrfEvt->evt.gattc_evt);
  BLE_PRT2("onDesc srvInd %d charInd %d\n",
    gattcDbDiscovery->currSrvInd, gattcDbDiscovery->currCharInd);
  srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
  const ble_gattc_evt_desc_disc_rsp_t * descDiscRspEvt = &(bleGattcEvt->params.desc_disc_rsp);
  BLE_GattcDbDiscChar * charBeingDiscovered = &(srvBeingDiscovered->characteristics[gattcDbDiscovery->currCharInd]);

  connHandle  = bleGattcEvt->conn_handle;
  if ((gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV) ||
    (gattcDbDiscovery->currCharInd >= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV))
    {
      BLE_ERR("onDesc ind NG\n");
      gattcDbDiscovery->discoveryInProgress = false;
      setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_DESCRIPTOR);
      return;
    }
  if (bleGattcEvt->gatt_status == BLE_GATT_STATUS_SUCCESS)
    {
      // The descriptor was found at the peer.
      // If the descriptor was a Client Characteristic Configuration Descriptor, then the cccdHandle needs to be populated.
      // Loop through all the descriptors to find the Client Characteristic Configuration Descriptor.
      for (i = 0; i < descDiscRspEvt->count; i++)
        {
          if (descDiscRspEvt->descs[i].uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG)
            {
              charBeingDiscovered->cccdHandle = descDiscRspEvt->descs[i].handle;
              BLE_PRT2("onDesc ccc %d\n", descDiscRspEvt->descs[i].handle);
              break;
            }
        }
    }

  if ((gattcDbDiscovery->currCharInd + 1) == srvBeingDiscovered->charCount)
    {
      // No more characteristics and descriptors need to be discovered. Discovery is complete.
      raiseDiscovComplete = true;
    }
  else
    {
      BLE_PRT2("onDesc retry descriptorsDiscover\n");
      // Begin discovery of descriptors for the next characteristic.
      gattcDbDiscovery->currCharInd++;
      (void)descriptorsDiscover(pBleEvent, gattcDbDiscovery, &raiseDiscovComplete);
    }
  if (raiseDiscovComplete)
    {
      BLE_PRT2("onDesc onSrvDiscCompletion\n");
      onSrvDiscCompletion(pBleEvent, gattcDbDiscovery);
    }
}

static
void onSrvDiscCompletion(BLE_Evt *pBleEvent, bleGattcDb *gattcDbDiscovery)
{
  int ret = BLE_SUCCESS;
  uint16_t nextHandleStart = 0;
  BLE_GattcDbDiscSrv       *srvBeingDiscovered = NULL;
  BLE_GattcDbDiscSrv       *srvPrevDiscovered = NULL;

  // Reset the current characteristic index since a new service discovery is about to start.
  gattcDbDiscovery->currCharInd = 0;
  srvPrevDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);

  gattcDbDiscovery->currSrvInd++;
  if (gattcDbDiscovery->currSrvInd > BLE_DB_DISCOVERY_MAX_SRV)
    {
      BLE_ERR("onCompletion ind NG\n");
      goto err;
    }
  else
    {
      // Initiate discovery of the next service.
      if (gattcDbDiscovery->currSrvInd < BLE_DB_DISCOVERY_MAX_SRV)
        {
          srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
          // Reset the characteristic count in the current service to zero since a new service
          // discovery is about to start.
          srvBeingDiscovered->charCount = 0;
        }
    }
  if (srvPrevDiscovered->srvHandleRange.endHandle != BLE_GATTC_HANDLE_END)
    {
      nextHandleStart = srvPrevDiscovered->srvHandleRange.endHandle + 1;
    }
  else
    {
      nextHandleStart = BLE_GATTC_HANDLE_END;
    }
  BLE_PRT2("onCompletion nextHandle %d\n", nextHandleStart);
  ret = sd_ble_gattc_primary_services_discover(gattcDbDiscovery->dbDiscovery.connHandle, nextHandleStart, NULL);
  if (ret != NRF_SUCCESS)
    {
      BLE_ERR("sd_ble_gattc_primary_sd NG %d\n", ret);
      goto err;
    }
  return;
err:
  gattcDbDiscovery->discoveryInProgress = false;
  setDbDiscoveryEvent(pBleEvent, gattcDbDiscovery->dbDiscovery.connHandle,
    BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_SERVICE);
  return;
}

static
int characteristicsDiscover(BLE_Evt *pBleEvent, bleGattcDb *const gattcDbDiscovery)
{
  int ret = 0;
  uint8_t              prevCharInd = 0;
  BLE_GattcChar        *prevChar = NULL;
  BLE_GattcDbDiscSrv   *srvBeingDiscovered = NULL;
  BLE_GattcHandleRange handleRange = {0};
  BLE_PRT2("charDiscover start srvInd %d charInd %d\n",
    gattcDbDiscovery->currSrvInd, gattcDbDiscovery->currCharInd);
  if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
    {
      BLE_ERR("charDiscover NG %d\n", NRF_ERROR_FORBIDDEN);
      goto err;
    }
  srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
  if (gattcDbDiscovery->currCharInd != 0)
    {
      // This is not the first characteristic being discovered. Hence the 'start handle' to be
      // used must be computed using the charValhandle of the previous characteristic.

      prevCharInd = gattcDbDiscovery->currCharInd - 1;

      srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
      prevChar = &(srvBeingDiscovered->characteristics[prevCharInd].characteristic);
      handleRange.startHandle = prevChar->charValhandle + 1;
    }
  else
    {
      // This is the first characteristic of this service being discovered.
      handleRange.startHandle = srvBeingDiscovered->srvHandleRange.startHandle;
    }

  handleRange.endHandle = srvBeingDiscovered->srvHandleRange.endHandle;
  BLE_PRT2("charDiscover sHdl %d eHdl %d\n",
    handleRange.startHandle, handleRange.endHandle);
  ret = sd_ble_gattc_characteristics_discover(gattcDbDiscovery->dbDiscovery.connHandle,
    (ble_gattc_handle_range_t *)&handleRange);
  if (ret)
    {
      BLE_ERR("sd_ble_gattc_cdr NG %d\n", ret);
      goto err;
    }
  return 0;
err:
  gattcDbDiscovery->discoveryInProgress = false;
  setDbDiscoveryEvent(pBleEvent, gattcDbDiscovery->dbDiscovery.connHandle,
    BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_CHARACTERISTIC);
  return -1;
}

static
bool isCharDiscoveryReqd(bleGattcDb *const gattcDbDiscovery,
               BLE_GattcChar         *afterChar)
{
  BLE_PRT2("isChar charEndHdl %d srvEndHdl %d\n", afterChar->charValhandle,
    gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle);
  if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
    {
      return false;
    }
  if (afterChar->charValhandle <
  gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle)
    {
      // Handle value of the characteristic being discovered is less than the end handle of
      // the service being discovered. There is a possibility of more characteristics being
      // present. Hence a characteristic discovery is required.
      return true;
    }
  return false;
}

static
bool isDescDiscoveryReqd(bleGattcDb       *gattcDbDiscovery,
                BLE_GattcDbDiscChar  *currChar,
                BLE_GattcDbDiscChar  *nextChar,
                BLE_GattcHandleRange *handleRange)
{
  BLE_PRT2("isDesc charValHdl %d srvEndHdl %d\n",
    currChar->characteristic.charValhandle,
    gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle);

  if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
    {
      return false;
    }
  if (nextChar == NULL)
    {
      // Current characteristic is the last characteristic in the service. Check if the value
      // handle of the current characteristic is equal to the service end handle.
      if (currChar->characteristic.charValhandle ==
        gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle
      )
        {
          // No descriptors can be present for the current characteristic. currChar is the last
          // characteristic with no descriptors.
          return false;
        }

      handleRange->startHandle = currChar->characteristic.charValhandle + 1;

      // Since the current characteristic is the last characteristic in the service, the end
      // handle should be the end handle of the service.
      handleRange->endHandle = gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle;
      BLE_PRT2("isDesc start %d end %d\n", handleRange->startHandle, handleRange->endHandle);
      return true;
    }

  // nextChar != NULL. Check for existence of descriptors between the current and the next
  // characteristic.
  if ((currChar->characteristic.charValhandle + 1) == nextChar->characteristic.charDeclhandle)
    {
      // No descriptors can exist between the two characteristic.
      return false;
    }

  handleRange->startHandle = currChar->characteristic.charValhandle + 1;
  handleRange->endHandle   = nextChar->characteristic.charDeclhandle - 1;
  BLE_PRT2("isDesc start %d end %d\n", handleRange->startHandle, handleRange->endHandle);
  return true;
}
static
int descriptorsDiscover(BLE_Evt *pBleEvent, bleGattcDb *const gattcDbDiscovery, bool *raiseDiscovComplete)
{
  uint8_t                    i = 0;
  int                        ret = BLE_SUCCESS;
  BLE_GattcHandleRange       handleRange = {0};
  BLE_GattcDbDiscChar        *currCharBeingDiscovered = NULL;
  BLE_GattcDbDiscSrv         *srvBeingDiscovered = NULL;
  BLE_GattcDbDiscChar        *nextChar = NULL;
  bool                       isDiscoveryReqd = false;

  *raiseDiscovComplete = false;
  BLE_PRT2("descDiscover SrvInd %d CharInd %d\n",
    gattcDbDiscovery->currSrvInd, gattcDbDiscovery->currCharInd);
  if ((gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV) || (gattcDbDiscovery->currCharInd >= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV))
    {
      BLE_ERR("descDiscover NG %d\n", NRF_ERROR_FORBIDDEN);
      goto err;
    }

  srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
  currCharBeingDiscovered = &(srvBeingDiscovered->characteristics[gattcDbDiscovery->currCharInd]);

  if ((gattcDbDiscovery->currCharInd + 1) == srvBeingDiscovered->charCount)
    {
      // This is the last characteristic of this service.
      isDiscoveryReqd = isDescDiscoveryReqd(gattcDbDiscovery, currCharBeingDiscovered, NULL, &handleRange);
    }
  else
    {
      for (i = gattcDbDiscovery->currCharInd; i < srvBeingDiscovered->charCount; i++)
        {
          if (i == (srvBeingDiscovered->charCount - 1))
            {
              // The current characteristic is the last characteristic in the service.
              nextChar = NULL;
            }
          else
            {
              nextChar = &(srvBeingDiscovered->characteristics[i + 1]);
            }

          // Check if it is possible for the current characteristic to have a descriptor.
          if (isDescDiscoveryReqd(gattcDbDiscovery, currCharBeingDiscovered, nextChar, &handleRange))
            {
              isDiscoveryReqd = true;
              break;
            }
          else
            {
              // No descriptors can exist.
              currCharBeingDiscovered = nextChar;
              gattcDbDiscovery->currCharInd++;
            }
        }
    }

  if (!isDiscoveryReqd)
    {
      // No more descriptor discovery required. Discovery is complete.
      *raiseDiscovComplete = true;
      return NRF_SUCCESS;
    }

  BLE_PRT2("descDiscover sHdl %d eHdl %d\n", handleRange.startHandle, handleRange.endHandle);
  ret = sd_ble_gattc_descriptors_discover(gattcDbDiscovery->dbDiscovery.connHandle, (ble_gattc_handle_range_t *)&handleRange);
  if (ret)
    {
      BLE_ERR("sd_ble_gattc_dd NG %d\n", ret);
      goto err;
    }
  return 0;
err:
  gattcDbDiscovery->discoveryInProgress = false;
  setDbDiscoveryEvent(pBleEvent, gattcDbDiscovery->dbDiscovery.connHandle,
    BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_DESCRIPTOR);
  return -1;
}

/****************************************************************************
 * For HAL I/F
 ****************************************************************************/

static int32_t set_adv_data(void)
{
#define BLE_TX_POWER_LEVEL  0
  int ret             = 0;
  int8_t tx_power         = BLE_TX_POWER_LEVEL;
  BLE_GapAdvData adv_data = {0};

  adv_data.flags = BLE_GAP_ADV_LE_GENERAL_DISC_MODE | BLE_GAP_ADV_BR_EDR_NOT_SUPPORTED;
  adv_data.txPower = tx_power;
  adv_data.complete32Uuid = 0;
  adv_data.completeLocalName.advData = (uint8_t*) bt_common_context.ble_name;
  adv_data.completeLocalName.advLength =
    strnlen((char*)adv_data.completeLocalName.advData, BUF_LEN_MAX);
  ret = BLE_GapSetAdvData(&adv_data);

  if (BLE_SUCCESS != ret)
    {
      LOG_OUT("set ble adv data failed, ret=%d\n", ret);
    }

  return ret;
}

static int32_t init_pairing_mode(BLE_GAP_IO_CAP io_cap)
{
  BLE_GapPairingFeature* pf = &g_ble_context.pairing_feature;
  int ret                   = 0;
  BLE_GapOOB oob            = BLE_GAP_OOB_AUTH_DATA_NOT_PRESENT;
  BLE_GapAuth auth          = BLE_GAP_AUTH_REQ_NO_MITM_BOND;
  pf->oob                   = oob;
  pf->ioCap                 = io_cap;
  pf->authReq               = auth;
  pf->minKeySize            = BLE_GAP_MIN_KEY_SIZE;
  pf->maxKeySize            = BLE_GAP_MAX_KEY_SIZE;

  return ret;
}

 
static int ble_start(void)
{
  BLE_InitializeParams params       = {0};
  int ret                       = 0;

  ret = sem_init(&g_ble_context.ble_srv_sds.char_rw_buf_sem, 0, 1);
  if (ret)
    {
      LOG_OUT("service in semaphore init failed.\n");
      return ret;
    }

  params.role = BLE_ROLE_PERIPHERAL;

  ret = BLE_CommonInitializeStack(&params);

  if (BLE_SUCCESS != ret)
    {
      LOG_OUT("ble stack init failed, ret=%d\n", ret);
    }

  return ((BLE_SUCCESS == ret) ? 0 : -1);
}

static int ble_stop(void)
{
  int ret = BLE_CommonFinalizeStack();
  if (BLE_SUCCESS != ret)
    {
      LOG_OUT("ble stack fina failed, ret=%d\n", ret);
    }

  ret = sem_destroy(&g_ble_context.ble_srv_sds.char_rw_buf_sem);
  if (ret)
    {
      LOG_OUT("service in semaphore init failed.\n");
    }
  return ret;
}


/****************************************************************************
 * Name: nrf52_ble_scan
 *
 * Description:
 *   Bluetooth LE start/stop scan.
 *   Start/Stop scan.
 *
 ****************************************************************************/

static int nrf52_ble_scan(bool enable)
{
  int ret = BT_SUCCESS;
  if( true == enable)
    {
      ret = BLE_GapStartScan();
    }
  else
    {
      ret = BLE_GapStopScan();
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_connect
 *
 * Description:
 *   Bluetooth LE connect to target device
 *
 ****************************************************************************/

static int nrf52_ble_connect(const BT_ADDR *addr)
{
  int ret = BT_SUCCESS;
  BLE_GapAddr gap_addr = {0};

  memcpy(gap_addr.addr, addr->address, sizeof(gap_addr.addr));

  ret = BLE_GapConnect(&gap_addr);

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_disconnect
 *
 * Description:
 *   Bluetooth LE disconnect link to target device
 *
 ****************************************************************************/

static int nrf52_ble_disconnect(const uint16_t conn_handle)
{
  int ret = BT_SUCCESS;
  BLE_GapConnHandle handle = conn_handle;

  ret = BLE_GapDisconnectLink(handle);

  return ret;
}

static int nrf52_ble_advertise(bool enable)
{
  int ret = 0;
  if (true == enable)
    {
    ret = set_adv_data();
    
      ret = BLE_GapStartAdv();

      if (BLE_SUCCESS != ret)
        {
          LOG_OUT("ble enable advertise failed, ret=%d\n", ret);
        }
    }
  else
    {
      ret = BLE_GapStopAdv();

      if (BLE_SUCCESS != ret)
        {
          LOG_OUT("ble disable advertise failed, ret=%d\n", ret);
        }
    }
  return ret;
}

static int nrf52_ble_set_dev_addr(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;
  return ret;
}

static int nrf52_ble_set_dev_name(char *name)
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

  return ret;
}

static int nrf52_ble_set_appearance(BLE_APPEARANCE appearance)
{
  int ret = BT_SUCCESS;
  return ret;
}

static int nrf52_ble_set_ppcp(BLE_CONN_PARAMS ppcp)
{
  int ret = BT_SUCCESS;
  return ret;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/
BLE_Context g_ble_context;

struct ble_hal_common_ops_s ble_hal_common_ops =
{
  .setDevAddr    = nrf52_ble_set_dev_addr,
  .setDevName    = nrf52_ble_set_dev_name,
  .setAppearance = nrf52_ble_set_appearance,
  .setPPCP       = nrf52_ble_set_ppcp,
  .advertise     = nrf52_ble_advertise,
  .scan          = nrf52_ble_scan,
  .connect       = nrf52_ble_connect,
  .disconnect    = nrf52_ble_disconnect
};

struct bt_hal_common_ops_s bt_hal_common_ops =
{
  .init          = nrf52_bt_init,
  .finalize      = nrf52_bt_finalize,
  .enable        = nrf52_bt_enable,
  .setDevAddr    = NULL,
  .getDevAddr    = NULL,
  .setDevName    = NULL,
  .getDevName    = NULL,
  .paringEnable  = NULL,
  .getBondList   = NULL,
  .unBond        = NULL,
  .setVisibility = NULL,
  .inquiryStart  = NULL,
  .inquiryCancel = NULL
};

/****************************************************************************
 * BT Dummy
 ****************************************************************************/


/****************************************************************************
 * Name: nrf52_bt_init
 *
 * Description:
 *   Bluetooth Initialize
 *   Prepare NV storage etc for non power item initialize.
 *
 ****************************************************************************/

static int nrf52_bt_init(void)
{
  int ret = BT_SUCCESS;

  ret = BLE_CommonSetBleEvtCallback((BLE_EfCb)NULL, &g_ble_context.ble_evt_ctx);

  if (BLE_SUCCESS != ret)
    {
      LOG_OUT("set ble evt callback failed, ret=%d\n", ret);
    }
  init_pairing_mode(BLE_GAP_IO_CAP_NO_INPUT_NO_OUTPUT);
  return (BLE_SUCCESS == ret) ? 0 : -1;
}

/****************************************************************************
 * Name: nrf52_bt_finalize
 *
 * Description:
 *   Bluetooth Finalize
 *   Release NV etc.
 *
 ****************************************************************************/

static int nrf52_bt_finalize(void)
{
  int ret = BT_SUCCESS;

  ret = BSO_Finalize(NULL);
  if (ret)
    {
      ret = -ENXIO;
      BLE_PRT("BSO_Finalize failed\n");
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_bt_enable
 *
 * Description:
 *   Bluetooth Enable
 *   Turn ON/OFF bluetooth and operate UART and Receive thread.
 *
 ****************************************************************************/

static int nrf52_bt_enable(bool enable)
{
  int ret = BT_SUCCESS;

  if (enable == true)
    {
      ret = ble_start();
    }
  else
    {
      ret = ble_stop();
    }

  return ret;
}

