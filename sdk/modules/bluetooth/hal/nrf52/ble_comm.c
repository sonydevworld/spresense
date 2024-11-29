/****************************************************************************
 * modules/bluetooth/hal/nrf52/ble_comm.c
 *
 *   Copyright 2022, 2024 Sony Semiconductor Solutions Corporation
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
#include <nuttx/crc16.h>
#include <ble/ble_comm.h>
#include <ble/ble_gap.h>
#include <ble/ble_gatts.h>
#include <ble/ble_gattc.h>
#include "ble_storage_operations.h"
#include "ble_comm_internal.h"
#include "ble_debug.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include <arch/board/board.h>
#include <system/readline.h>
#include <nrf_crypto.h>
#include <nrf_crypto_ecc.h>
#include <nrf_crypto_ecdh.h>
#include <nrf_soc.h>

/******************************************************************************
 * externs
 *****************************************************************************/
extern bleGapMem *bleGetGapMem(void);
extern void bleMngEvtDispatch(ble_evt_t *nrfEvt);
extern void board_nrf52_initialize(void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF52_SYS_ATTR_DATA_OFFSET_HANDLE (0)
#define NRF52_SYS_ATTR_DATA_OFFSET_LEN    (2)
#define NRF52_SYS_ATTR_DATA_OFFSET_VALUE  (4)

#define NRF52_SYS_ATTR_DATA_LEN_HANDLE (2)
#define NRF52_SYS_ATTR_DATA_LEN_LEN    (2)
#define NRF52_SYS_ATTR_DATA_LEN_VALUE  (2)

#define IM_ADDR_CLEARTEXT_LENGTH  (3)
#define IM_ADDR_CIPHERTEXT_LENGTH (3)

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
static void onExchangeMtuResponse(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onTxComplete(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnParamUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnParamUpdateRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDisconnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onSecParamsRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
#ifdef CONFIG_NRF52_LESC
static void onLescDhkeyRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
#endif
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
static bool isCharDiscoveryReqd(bleGattcDb *const gattcDbDiscovery, uint16_t last_handle);
static bool isDescDiscoveryReqd(bleGattcDb *gattcDbDiscovery, BLE_GattcDbDiscChar *currChar, BLE_GattcDbDiscChar *nextChar, BLE_GattcHandleRange *handleRange);
static  int descriptorsDiscover(BLE_Evt *pBleEvent, bleGattcDb *const gattcDbDiscovery, bool *raiseDiscovComplete);

static int nrf52_ble_start_scan(bool duplicate_filter);
static int nrf52_ble_stop_scan(void);
static int nrf52_ble_connect(uint8_t addr_type, const BT_ADDR *addr);
static int nrf52_ble_disconnect(const uint16_t conn_handle);
static int nrf52_ble_advertise(bool enable);
static int nrf52_ble_set_dev_addr(BT_ADDR *addr, uint8_t type);
static int nrf52_ble_set_dev_name(char *name);
static int nrf52_ble_set_appearance(BLE_APPEARANCE appearance);
static int nrf52_ble_set_ppcp(BLE_CONN_PARAMS ppcp);
static uint16_t nrf52_ble_set_mtusize(uint16_t sz);
static uint16_t nrf52_ble_get_mtusize(void);
static int nrf52_ble_get_negotiated_mtusize(uint16_t handle);
static int nrf52_ble_pairing(uint16_t handle);
static int nrf52_ble_set_txpower(int8_t tx_power);
static int nrf52_ble_set_scan_param(struct ble_scan_param_s *param);
static int nrf52_ble_set_conn_param(struct ble_conn_param_s *param);

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
#define AUTH_KEY_SIZE   6

static struct ble_hal_common_ops_s ble_hal_common_ops =
{
  .setDevAddr           = nrf52_ble_set_dev_addr,
  .setDevName           = nrf52_ble_set_dev_name,
  .setAppearance        = nrf52_ble_set_appearance,
  .setPPCP              = nrf52_ble_set_ppcp,
  .advertise            = nrf52_ble_advertise,
  .startScan            = nrf52_ble_start_scan,
  .stopScan             = nrf52_ble_stop_scan,
  .connect              = nrf52_ble_connect,
  .disconnect           = nrf52_ble_disconnect,
  .setMtuSize           = nrf52_ble_set_mtusize,
  .getMtuSize           = nrf52_ble_get_mtusize,
  .getNegotiatedMtuSize = nrf52_ble_get_negotiated_mtusize,
  .pairing              = nrf52_ble_pairing,
  .setTxPower           = nrf52_ble_set_txpower,
  .setScanParam         = nrf52_ble_set_scan_param,
  .setConnParam         = nrf52_ble_set_conn_param,
};

static struct bt_hal_common_ops_s bt_hal_common_ops =
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
 * Private Functions
 ****************************************************************************/

static int blePowerOff(void)
{
  int ret = 0;

  /* Control the reset pin instead of the power.
   * true (HIGH) : reset assert
   * false (LOW) : reset deassert
   */

  ret = board_power_control(POWER_BTBLE, true);
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
  BLE_PRT("bleInitBondInfoKey: enable_key 0x%lx\n", bleKey.enable_key);
  for (int i=0; i<BLE_SAVE_BOND_DEVICE_MAX_NUM; i++) {
    BLE_PRT("bleInitBondInfoKey: info_key[%d] 0x%lx\n", i, bleKey.info_key[i]);
  }
}

static void set_EncKey_for_save_event(struct ble_idkey_s *apps,
                                      ble_gap_enc_key_t  *nrf52)
{
  memcpy(apps->ltk, nrf52->enc_info.ltk, BLE_LTK_LEN);
  apps->ediv = nrf52->master_id.ediv;
  memcpy(apps->rand, nrf52->master_id.rand, BLE_RAND_LEN);
}

static void set_bondinfo_for_save_event(struct ble_bondinfo_s *apps,
                                        bleGapWrapperBondInfo *nrf52)
{
  static struct ble_cccd_s cccd;

  apps->peer_addr.type = nrf52->bondInfo.addrType;
  memcpy(apps->peer_addr.addr,
         nrf52->bondInfo.addr,
         BLE_GAP_ADDR_LENGTH);

  memcpy(apps->own.irk, nrf52->ownIdKey.id_info.irk, BLE_GAP_SEC_KEY_LEN);
  memcpy(apps->peer.irk, nrf52->peerIdKey.id_info.irk, BLE_GAP_SEC_KEY_LEN);
  set_EncKey_for_save_event(&apps->own,  &nrf52->ownEncKey);
  set_EncKey_for_save_event(&apps->peer, &nrf52->peerEncKey);

  apps->cccd_num = 1;
  apps->cccd = &cccd;
  memcpy(&cccd.handle,
         &nrf52->sys_attr_data[NRF52_SYS_ATTR_DATA_OFFSET_HANDLE],
         NRF52_SYS_ATTR_DATA_LEN_HANDLE);
  memcpy(&cccd.value,
         &nrf52->sys_attr_data[NRF52_SYS_ATTR_DATA_OFFSET_VALUE],
         NRF52_SYS_ATTR_DATA_LEN_VALUE);
}

static void mk_save_bondinfo_event(void)
{
  int i;
  static struct ble_bondinfo_s bond[BLE_SAVE_BOND_DEVICE_MAX_NUM];
  struct ble_event_bondinfo_t evt;

  evt.group_id = BLE_GROUP_COMMON;
  evt.event_id = BLE_COMMON_EVENT_SAVE_BOND;
  evt.num = 0;

  for (i = 0; i < BLE_SAVE_BOND_DEVICE_MAX_NUM; i++)
    {
      if (((bleBondEnableList >> i) & 1) == 1)
        {
          set_bondinfo_for_save_event(&bond[evt.num],
                                      &BondInfoInFlash[i]);
          evt.num++;
        }
    }

  evt.bond = bond;

  ble_common_event_handler((struct bt_event_t *) &evt);
}

static int mk_load_bondinfo_event(struct ble_bondinfo_s *bond)
{
  struct ble_event_bondinfo_t evt;

  evt.group_id = BLE_GROUP_COMMON;
  evt.event_id = BLE_COMMON_EVENT_LOAD_BOND;
  evt.num = BLE_SAVE_BOND_DEVICE_MAX_NUM;
  evt.bond = bond;

  return ble_common_event_handler((struct bt_event_t *) &evt);
}

static void save_EncKey(ble_gap_enc_key_t  *nrf52,
                        struct ble_idkey_s *apps)
{
  memcpy(nrf52->enc_info.ltk, apps->ltk, BLE_GAP_SEC_KEY_LEN);
  nrf52->enc_info.ltk_len = BLE_GAP_SEC_KEY_LEN;
  nrf52->master_id.ediv = apps->ediv;
  memcpy(nrf52->master_id.rand, apps->rand, BLE_GAP_SEC_RAND_LEN);
}

static void save_cccd(int num,
                      struct ble_cccd_s *cccd,
                      uint8_t *sys_attr_data)
{
  int i;
  uint16_t *p = (uint16_t *)sys_attr_data;

  for (i = 0; i < num; i++)
    {
      *p++ = cccd[i].handle;
      *p++ = sizeof(cccd[i].value);
      *p++ = cccd[i].value;
    }

  /* Append CRC16 value for created sys_attr_data. */

  *p = crc16part(sys_attr_data, (uint8_t *)p - sys_attr_data, 0xFFFF);
}

static void save_apps_bondinfo(int num,
                               struct ble_bondinfo_s *apps_bond,
                               uint32_t *enable_list,
                               bleGapWrapperBondInfo *bond)
{
  int i;
  struct ble_bondinfo_s *apps = &apps_bond[0];
  bleGapWrapperBondInfo *nrf52 = &bond[0];

  *enable_list = 0;

  for (i = 0, apps = &apps_bond[0], nrf52 = &bond[0];
       i < num;
       i++, apps++, nrf52++)
    {
      *enable_list |= (1 << i);
      nrf52->bondInfo.addrType = apps->peer_addr.type;
      memcpy(nrf52->bondInfo.addr,
             apps->peer_addr.addr,
             BLE_GAP_ADDR_LENGTH);
      memcpy(nrf52->ownIdKey.id_info.irk, apps->own.irk, BLE_GAP_SEC_KEY_LEN);
      memcpy(nrf52->peerIdKey.id_info.irk, apps->peer.irk, BLE_GAP_SEC_KEY_LEN);
      nrf52->peerIdKey.id_addr_info.addr_id_peer = 1;
      nrf52->peerIdKey.id_addr_info.addr_type = apps->peer_addr.type;
      memcpy(nrf52->peerIdKey.id_addr_info.addr,
             apps->peer_addr.addr,
             BLE_GAP_ADDR_LENGTH);
      save_EncKey(&nrf52->ownEncKey,  &apps->own);
      save_EncKey(&nrf52->peerEncKey, &apps->peer);
      save_cccd(apps->cccd_num, apps->cccd, nrf52->sys_attr_data);
    }
}

static void load_apps_bondinfo(uint32_t *enable_list,
                               bleGapWrapperBondInfo *bond)
{
  int num;
  static struct ble_bondinfo_s apps_bond[BLE_SAVE_BOND_DEVICE_MAX_NUM];

  num = mk_load_bondinfo_event(apps_bond);
  save_apps_bondinfo(num, apps_bond, enable_list, bond);
}

static int bleGetBondInfo(void)
{
  bleInitBondInfoKey();
  load_apps_bondinfo(&bleBondEnableList, BondInfoInFlash);

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

        commMem.requested_mtu = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
        commMem.client_rx_mtu = BLE_GATT_ATT_MTU_DEFAULT;
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

  if (BSO_Finalize(NULL))
    {
      ret = -ENXIO;
    }

  commMem.stackInited = false;
  return ret;
}

static
int blePowerOn(void)
{
  int ret = 0;

  /* Control the reset pin instead of the power.
   * true (HIGH) : reset assert
   * false (LOW) : reset deassert
   */

  ret = board_power_control(POWER_BTBLE, false);

  if (ret)
    {
      BLE_PRT("board_power_control(on): NG %d\n", ret);
      goto errPower;
    }

  BLE_PRT("Power on BLE!!\n");
  return BLE_SUCCESS;
errPower:
  (void)board_power_control(POWER_BTBLE, true);
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

  switch (g_ble_context.conn_sts)
    {
      case BLE_CONN_STS_CONNECTED:
        BLE_GapDisconnectLink(g_ble_context.ble_conn_handle);
        g_ble_context.ble_conn_handle = CONN_HANDLE_INVALED;
        break;

      case BLE_CONN_STS_CONNECTING:
        BLE_GapCancelConnecting();
        break;

      default:
        break;
    }

  g_ble_context.conn_sts = BLE_CONN_STS_NOTCONNECTED;

  if (g_ble_context.is_advertising)
    {
      BLE_GapStopAdv();
      g_ble_context.is_advertising = false;
    }

  if (g_ble_context.is_scanning)
    {
      BLE_GapStopScan();
      g_ble_context.is_scanning = false;
    }

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
#ifdef CONFIG_NRF52_LESC
      case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        onLescDhkeyRequest(bleEvent, pBleNrfEvt);
        break;
#endif
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
      case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
        onExchangeMtuResponse(bleEvent, pBleNrfEvt);
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
  struct ble_event_conn_stat_t conn_stat_evt;

  LOG_OUT("Connected: handle %d, role %d\n", evt->handle, evt->role);
  g_ble_context.ble_srv_sds.conn_handle = evt->handle;
  g_ble_context.ble_conn_handle         = evt->handle;
  g_ble_context.ble_role                = evt->role;
  g_ble_context.conn_sts                = BLE_CONN_STS_CONNECTED;
  g_ble_context.is_scanning             = false;
  g_ble_context.is_advertising          = false;
  g_ble_context.ble_addr.type           = evt->addr.type;
  memcpy(g_ble_context.ble_addr.addr, evt->addr.addr, BT_ADDR_LEN);

  conn_stat_evt.connected = true;
  conn_stat_evt.status    = BLESTAT_SUCCESS;
  conn_stat_evt.handle = evt->handle;
  memcpy(conn_stat_evt.addr.address, evt->addr.addr, BT_ADDR_LEN);
  conn_stat_evt.group_id = BLE_GROUP_COMMON;
  conn_stat_evt.event_id = BLE_COMMON_EVENT_CONN_STAT_CHANGE;
  ble_common_event_handler((struct bt_event_t *) &conn_stat_evt);

  if ((evt->role == BLE_ROLE_CENTRAL) &&
      (commMem.requested_mtu != BLE_GATT_ATT_MTU_DEFAULT))
    {
      sd_ble_gattc_exchange_mtu_request(evt->handle, commMem.requested_mtu);
    }
}

static uint8_t convert_hcicode_to_mwvalue(uint8_t hcicode)
{
  switch (hcicode)
    {
      case BLE_HCI_STATUS_CODE_SUCCESS:
        return BLESTAT_SUCCESS;

      case BLE_HCI_MEMORY_CAPACITY_EXCEEDED:
        return BLESTAT_MEMCAP_EXCD;

      case BLE_HCI_CONNECTION_TIMEOUT:
        return BLESTAT_CONNECT_TIMEOUT;

      case BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION:
        return BLESTAT_PEER_TERMINATED;

      case BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES:
        return BLESTAT_PEER_TERM_LOWRES;

      case BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF:
        return BLESTAT_PEER_TERM_POFF;

      case BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION:
        return BLESTAT_TERMINATED;

      case BLE_HCI_CONTROLLER_BUSY:
        return BLESTAT_DEVICE_BUSY;

      case BLE_HCI_CONN_INTERVAL_UNACCEPTABLE:
        return BLESTAT_PARAM_REJECTED;

      case BLE_HCI_CONN_FAILED_TO_BE_ESTABLISHED:
        return BLESTAT_CONNECT_FAILED;

      default:
        return BLESTAT_UNSPEC_ERR;
    }
}

static void on_disconnected(const BLE_EvtDisconnected* evt)
{
  struct ble_event_conn_stat_t conn_stat_evt;

  LOG_OUT("Disconnected: HCI status 0x%x\n", evt->reason);
  g_ble_context.ble_conn_handle = CONN_HANDLE_INVALED;
  g_ble_context.conn_sts        = BLE_CONN_STS_NOTCONNECTED;

  conn_stat_evt.connected = false;

  conn_stat_evt.group_id = BLE_GROUP_COMMON;
  conn_stat_evt.event_id = BLE_COMMON_EVENT_CONN_STAT_CHANGE;
  conn_stat_evt.status   = convert_hcicode_to_mwvalue(evt->reason);
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
      else
        {
          g_ble_context.is_advertising = true;
        }
    }
}

static void on_read_rsp(BLE_EvtGattcRead *readRsp)
{
  struct ble_gatt_event_read_rsp_t evt;

  evt.group_id    = BLE_GROUP_GATT;
  evt.event_id    = BLE_GATT_EVENT_READ_RESP;
  evt.conn_handle = readRsp->connHandle;
  evt.char_handle = readRsp->charValHandle;
  evt.length      = readRsp->charValLen;
  memcpy(evt.data, readRsp->charValData, sizeof(evt.data));

  ble_gatt_event_handler((struct bt_event_t *)&evt);
}

static void on_exchange_feature(const BLE_EvtExchangeFeature* exchange_feature)
{
  int ret;
  BLE_GapPairingFeature* pf = NULL;

  /* In central case, pairing feature must not be set,
   * because it is transmitted in the parameter of
   * preceding sd_ble_gap_authenticate()
   */

  if (g_ble_context.ble_role == BLE_ROLE_PERIPHERAL)
    {
      pf = &g_ble_context.pairing_feature;
    }

  ret = BLE_GapExchangePairingFeature(exchange_feature->handle,
                                      pf,
                                      &exchange_feature->peerFeature);

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

static void on_disp_passkey(BLE_EvtDisplayPasskey* disp_passkey)
{
  LOG_OUT("[BLE][LOG]Passkey: %s\n", disp_passkey->passkey);
}

static void mk_encryption_result_event(uint16_t handle, bool result)
{
  struct ble_event_encryption_result_t evt;

  evt.group_id = BLE_GROUP_COMMON;
  evt.event_id = BLE_COMMON_EVENT_ENCRYPTION_RESULT;
  evt.conn_handle = handle;
  evt.result = result;

  ble_common_event_handler((struct bt_event_t *) &evt);
}

static void on_auth_status(BLE_EvtAuthStatus* auth_status)
{
  int ret   = BLE_SUCCESS;
  int index = 0;
  BLE_GapBondInfoList bond_id = {0};
  if (auth_status->status != BLE_GAP_SM_STATUS_SUCCESS)
    {
      LOG_OUT("[BLE][LOG]Pairing failed! ErrCode: %x\n", auth_status->status);
      mk_encryption_result_event(auth_status->handle, false);
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

  mk_save_bondinfo_event();
}

static void on_timeout(BLE_EvtTimeout *timeout)
{
  int ret = BLE_SUCCESS;
  struct ble_event_conn_stat_t conn_stat_evt;

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
        conn_stat_evt.connected = false;

        conn_stat_evt.group_id = BLE_GROUP_COMMON;
        conn_stat_evt.event_id = BLE_COMMON_EVENT_CONN_STAT_CHANGE;
        conn_stat_evt.status   = BLESTAT_CONNECT_TIMEOUT;
        ble_common_event_handler((struct bt_event_t *) &conn_stat_evt);

        break;
      default:
        LOG_OUT("[BLE][LOG]Timeout reason: Error\n");
        break;
    }
}

static int input_passkey(uint8_t *key)
{
  LOG_OUT("Please enter passkey:\n");
  int len = 0;

  LOG_OUT("passkey_input> ");
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

static void on_adv_report(BLE_EvtAdvReportData *adv_report)
{
  struct ble_event_adv_rept_t evt;

  evt.group_id = BLE_GROUP_COMMON;
  evt.event_id = BLE_COMMON_EVENT_SCAN_RESULT;
  evt.rssi = adv_report->rssi;
  evt.scan_rsp = adv_report->scan_rsp;
  evt.length = adv_report->dlen + 2;

  /* The first octet is used for notification of peer address type. */

  evt.data[0] = adv_report->addr.type;

  /* The second octet is used for notification of rssi. */

  evt.data[1] = (uint8_t)adv_report->rssi;

  /* The raw advertising data starts from the third octet. */

  memcpy(&evt.data[2], adv_report->data, adv_report->dlen);
  memcpy(evt.addr.address, adv_report->addr.addr, BLE_GAP_ADDR_LENGTH);
  ble_common_event_handler((struct bt_event_t *)&evt);
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
#ifdef CONFIG_BLUETOOTH_DEBUG_MSG
  ble_gap_evt_adv_set_terminated_t *set = &pBleNrfEvt->evt.gap_evt.params.adv_set_terminated;
#endif
  BLE_PRT("onAdvSetTerminate reason=%d handle=%d num=%d\n",
    set->reason, set->adv_handle, set->num_completed_adv_events);
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
#ifdef CONFIG_BLUETOOTH_DEBUG_MSG
  ble_gap_data_length_params_t *req = &pBleNrfEvt->evt.gap_evt.params.data_length_update_request.peer_params;
#endif
  BLE_PRT("onLenUpReq: max_tx=%d\n", req->max_tx_octets);
  BLE_PRT("onLenUpReq: max_rx=%d\n", req->max_rx_octets);
  BLE_PRT("onLenUpReq: tx_time=%d\n", req->max_tx_time_us);
  BLE_PRT("onLenUpReq: rx_time=%d\n", req->max_rx_time_us);
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

static void mk_mtusize_event(uint16_t handle, uint16_t sz)
{
  struct ble_event_mtusize_t evt;

  evt.mtusize  = sz;
  evt.handle   = handle;
  evt.group_id = BLE_GROUP_COMMON;
  evt.event_id = BLE_COMMON_EVENT_MTUSIZE;

  ble_common_event_handler((struct bt_event_t *) &evt);
}

static
void onExchangeMtuResponse(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  ble_gattc_evt_exchange_mtu_rsp_t *rsp;

  rsp = &pBleNrfEvt->evt.gattc_evt.params.exchange_mtu_rsp;
  BLE_PRT("onMtuRsp: mtu=%d\n", rsp->server_rx_mtu);
  if (commMem.requested_mtu > rsp->server_rx_mtu)
    {
      commMem.client_rx_mtu = rsp->server_rx_mtu;
    }
  else
    {
      commMem.client_rx_mtu = commMem.requested_mtu;
    }

  mk_mtusize_event(pBleNrfEvt->evt.gattc_evt.conn_handle, commMem.client_rx_mtu);
}

static
void onExchangeMtuRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  int ret = 0;
  ble_gatts_evt_exchange_mtu_request_t *req;

  req = &pBleNrfEvt->evt.gatts_evt.params.exchange_mtu_request;
  BLE_PRT("onMtuReq: mtu=%d\n", req->client_rx_mtu);
  if (commMem.requested_mtu > req->client_rx_mtu)
    {
      commMem.client_rx_mtu = req->client_rx_mtu;
    }
  else
    {
      commMem.client_rx_mtu = commMem.requested_mtu;
    }

  ret = sd_ble_gatts_exchange_mtu_reply(pBleNrfEvt->evt.gatts_evt.conn_handle,
                                        commMem.client_rx_mtu);
  if (ret)
    {
      BLE_ERR("onLenUp: sd_ble_gatts_exchange_mtu_reply %d\n", ret);
      return;
    }

  pBleEvent->evtHeader = BLE_GATTS_EVENT_EXCHANGE_MTU;
  commMem.gattsExchangeMTU.connHandle = pBleNrfEvt->evt.gatts_evt.conn_handle;
  commMem.gattsExchangeMTU.client_rx_mtu = req->client_rx_mtu;
  pBleEvent->evtDataSize = sizeof(BLE_EvtGattsExchangeMTU);
  memcpy(pBleEvent->evtData, &commMem.gattsExchangeMTU, pBleEvent->evtDataSize);

  mk_mtusize_event(pBleNrfEvt->evt.gatts_evt.conn_handle, commMem.client_rx_mtu);
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

/* Calculate the ah() hash function described in Bluetooth core specification */

static void ah(uint8_t const *p_k, uint8_t const *p_r, uint8_t *p_local_hash)
{
  nrf_ecb_hal_data_t ecb_hal_data;
  uint32_t i;

  for (i = 0; i < SOC_ECB_KEY_LENGTH; i++)
    {
      ecb_hal_data.key[i] = p_k[SOC_ECB_KEY_LENGTH - 1 - i];
    }

  memset(ecb_hal_data.cleartext, 0, SOC_ECB_KEY_LENGTH - IM_ADDR_CLEARTEXT_LENGTH);

  for (i = 0; i < IM_ADDR_CLEARTEXT_LENGTH; i++)
    {
      ecb_hal_data.cleartext[SOC_ECB_KEY_LENGTH - 1 - i] = p_r[i];
    }

  sd_ecb_block_encrypt(&ecb_hal_data);

  for (i = 0; i < IM_ADDR_CIPHERTEXT_LENGTH; i++)
    {
      p_local_hash[i] = ecb_hal_data.ciphertext[SOC_ECB_KEY_LENGTH - 1 - i];
    }
}

static bool im_address_resolve(ble_gap_addr_t const *p_addr, ble_gap_irk_t const *p_irk)
{
  uint8_t hash[IM_ADDR_CIPHERTEXT_LENGTH];
  uint8_t local_hash[IM_ADDR_CIPHERTEXT_LENGTH];
  uint8_t prand[IM_ADDR_CLEARTEXT_LENGTH];

  if (p_addr->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE)
    {
      return false;
    }

  memcpy(hash, p_addr->addr, IM_ADDR_CIPHERTEXT_LENGTH);
  memcpy(prand, &p_addr->addr[IM_ADDR_CIPHERTEXT_LENGTH], IM_ADDR_CLEARTEXT_LENGTH);
  ah(p_irk->irk, prand, local_hash);

  return (memcmp(hash, local_hash, IM_ADDR_CIPHERTEXT_LENGTH) == 0);
}

static int searchBondInfoIndexAddress(ble_gap_addr_t *addr)
{
  int i;
  uint32_t list = bleBondEnableList;

  for (i = 0; i < BLE_SAVE_BOND_DEVICE_MAX_NUM; i++, list >>= 1)
    {
      if (list & 1)
        {
          if ((addr->addr_type == BLE_GAP_ADDR_TYPE_PUBLIC) ||
              (addr->addr_type == BLE_GAP_ADDR_TYPE_RANDOM_STATIC))
            {
              if (memcmp(BondInfoInFlash[i].bondInfo.addr, addr->addr, BLE_GAP_ADDR_LEN) == 0)
                {
                  break;
                }
            }
          else if (addr->addr_type == BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE)
            {
              if (im_address_resolve(addr, &BondInfoInFlash[i].peerIdKey.id_info))
                {
                  break;
                }
            }
        }
    }

  return i;
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
  commMem.gapMem->wrapperBondInfo.connHandle = pBleNrfEvt->evt.gap_evt.conn_handle;
  commMem.gapMem->wrapperBondInfo.bondInfo.addrType = connected->peer_addr.addr_type;
  memcpy(commMem.gapMem->wrapperBondInfo.bondInfo.addr, connected->peer_addr.addr, BLE_GAP_ADDR_LENGTH);
  pBleEvent->evtDataSize = sizeof(BLE_EvtConnected);
  memcpy(pBleEvent->evtData, &commMem.connectData, pBleEvent->evtDataSize);

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

  /* Currently, not support the connection parameter settings.
   * So, accept the peer's setting as is.
   */

  sd_ble_gap_conn_param_update(pBleNrfEvt->evt.gap_evt.conn_handle, params);
}

static
void saveSysAttrData(ble_gap_master_id_t *id, uint8_t *sys_attr_data)
{
  int index;
  ble_gap_addr_t addr;

  addr.addr_type = g_ble_context.ble_addr.type;
  memcpy(addr.addr, g_ble_context.ble_addr.addr, BLE_GAP_ADDR_LENGTH);
  index = searchBondInfoIndexAddress(&addr);

  if (index < BLE_SAVE_BOND_DEVICE_MAX_NUM)
    {
      memcpy(BondInfoInFlash[index].sys_attr_data, sys_attr_data, BLE_GATTS_SYS_ATTR_DATA_TOTALLEN);
      BSO_SetRegistryValue(bleKey.info_key[index],
                           (const void*)&BondInfoInFlash[index],
                           sizeof(bleGapWrapperBondInfo));
      mk_save_bondinfo_event();
    }
}

static
void onDisconnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  uint16_t len = BLE_GATTS_SYS_ATTR_DATA_TOTALLEN;
  uint8_t sys_attr_data[BLE_GATTS_SYS_ATTR_DATA_TOTALLEN];

  pBleEvent->evtHeader = BLE_GAP_EVENT_DISCONNECTED;
  commMem.disconnectData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
  commMem.disconnectData.reason = pBleNrfEvt->evt.gap_evt.params.disconnected.reason;
  BLE_PRT("onDisconnect: reason=%d\n", pBleNrfEvt->evt.gap_evt.params.disconnected.reason);
  pBleEvent->evtDataSize = sizeof(BLE_EvtDisconnected);
  memcpy(pBleEvent->evtData, &commMem.disconnectData, pBleEvent->evtDataSize);
  commMem.gapMem->is_connected = false;

  on_disconnected((BLE_EvtDisconnected*)pBleEvent->evtData);

  /* Save system attribute data for next connection. */

  sd_ble_gatts_sys_attr_get(pBleNrfEvt->evt.gap_evt.conn_handle,
                            sys_attr_data,
                            &len,
                            0);

  saveSysAttrData(&commMem.gapMem->wrapperBondInfo.ownEncKey.master_id, sys_attr_data);
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
  commMem.exchangeFeatureData.peerFeature.lesc       = secParamReq->peer_params.lesc;
  pBleEvent->evtDataSize = sizeof(BLE_EvtExchangeFeature);
  memcpy(pBleEvent->evtData, &commMem.exchangeFeatureData, pBleEvent->evtDataSize);

  on_exchange_feature((BLE_EvtExchangeFeature*)pBleEvent->evtData);
}

#ifdef CONFIG_NRF52_LESC
static
void onLescDhkeyRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  uint16_t conn_handle = pBleNrfEvt->evt.gap_evt.conn_handle;
  uint8_t *recv_pk = pBleNrfEvt->evt.gap_evt.params.lesc_dhkey_request.p_pk_peer->pk;
  uint8_t pk_raw[BLE_GAP_LESC_P256_PK_LEN];
  nrf_crypto_ecc_public_key_t pk;
  ble_gap_lesc_dhkey_t dhkey;
  size_t dhkey_len = BLE_GAP_LESC_DHKEY_LEN;
  nrf_crypto_ecdh_context_t ctx;

  nrf_crypto_init();

  /* Convert the received public key from little endian to big-endian. */

  nrf_crypto_ecc_byte_order_invert(&g_nrf_crypto_ecc_secp256r1_curve_info,
                                   recv_pk,
                                   pk_raw,
                                   BLE_GAP_LESC_P256_PK_LEN);

  /* Convert received public key data to internal public key format. */

  nrf_crypto_ecc_public_key_from_raw(&g_nrf_crypto_ecc_secp256r1_curve_info,
                                     &pk,
                                     pk_raw,
                                     BLE_GAP_LESC_P256_PK_LEN);

  /* Calculate the DHKey. */

  nrf_crypto_ecdh_compute(&ctx,
                          &commMem.gapMem->lescKey.priv,
                          &pk,
                          dhkey.key,
                          &dhkey_len);

  /* Invert the shared secret for little endian format. */

  nrf_crypto_ecc_byte_order_invert(&g_nrf_crypto_ecc_secp256r1_curve_info,
                                   dhkey.key,
                                   dhkey.key,
                                   BLE_GAP_LESC_DHKEY_LEN);

  nrf_crypto_uninit();
  sd_ble_gap_lesc_dhkey_reply(conn_handle, &dhkey);
}
#endif

static
void onAuthStatus(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_AUTH_STATUS;
  ble_gap_evt_auth_status_t *authStatus = &pBleNrfEvt->evt.gap_evt.params.auth_status;
  commMem.authStatusData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
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

static int getAddrFmConnHandle(uint16_t handle, BT_ADDR *addr)
{
  if (handle != g_ble_context.ble_conn_handle)
    {
      /* Invalid connection handle. */

      return -EINVAL;
    }

  memcpy(addr->address, g_ble_context.ble_addr.addr, BT_ADDR_LEN);

  return BT_SUCCESS;
}

static int searchBondInfoIndexFmConnHandle(uint16_t handle)
{
  int i;
  int ret;
  uint32_t list = bleBondEnableList;
  BT_ADDR addr;

  ret = getAddrFmConnHandle(handle, &addr);
  if (ret != BT_SUCCESS)
    {
      return ret;
    }

  for (i = 0; i < BLE_SAVE_BOND_DEVICE_MAX_NUM; i++, list >>= 1)
    {
      if (list & 1)
        {
          if (memcmp(addr.address,
                     BondInfoInFlash[i].bondInfo.addr,
                     BT_ADDR_LEN)
               == 0)
            {
              break;
            }
        }
    }

  return i;
}

static void clearBondInfo(uint16_t handle)
{
  int ret;

  ret = searchBondInfoIndexFmConnHandle(handle);
  printf("searchBondInfoIndexFmConnHandle ret = %d\n", ret);
  if ((ret >= 0) && (ret < BLE_SAVE_BOND_DEVICE_MAX_NUM))
    {
      memset(&BondInfoInFlash[ret], 0, sizeof(bleGapWrapperBondInfo));
      bleBondEnableList &= ~(1 << ret);
      mk_save_bondinfo_event();
    }
}

static
void onConnSecUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  bool result;
  ble_gap_evt_t *evt = &pBleNrfEvt->evt.gap_evt;
  ble_gap_conn_sec_t *prm = &evt->params.conn_sec_update.conn_sec;

  pBleEvent->evtHeader = BLE_GAP_EVENT_CONN_SEC_UPDATE;
  pBleEvent->evtDataSize = 0;
  BLE_PRT("onConnSecUpdate: keysize=%d sm=%d lv=%d\n", prm->encr_key_size,
    prm->sec_mode.sm, prm->sec_mode.lv);

  if (prm->encr_key_size > 0)
    {
      /* Positive key size means that the connection is encrypted. */

      result = true;
    }
  else
    {
      /* Non-positive key size means that encryption fails.
       * In such a case, the corresponding bond information shall be cleared.
       */

      clearBondInfo(evt->conn_handle);
      result = false;
    }

  mk_encryption_result_event(evt->conn_handle, result);
}

static
void onSecInfoRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  ble_gap_enc_info_t *enc_info = NULL;
  ble_gap_irk_t *id_info = NULL;
  ble_gap_sign_info_t *sign_info = NULL;
  ble_gap_evt_sec_info_request_t *secinfo = &pBleNrfEvt->evt.gap_evt.params.sec_info_request;
  uint8_t *sys_attr_data = NULL;
  int index;

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

  index = searchBondInfoIndexAddress(&secinfo->peer_addr);

  if (index < BLE_SAVE_BOND_DEVICE_MAX_NUM)
    {
      BLE_PRT("onSecInfoRequest: master_id exitsting index %d\n", index);
      enc_info = &BondInfoInFlash[index].ownEncKey.enc_info;
      id_info = &BondInfoInFlash[index].ownIdKey.id_info;
      sys_attr_data = BondInfoInFlash[index].sys_attr_data;
    }

  memcpy(&commMem.gapMem->wrapperBondInfo.ownEncKey.master_id,
         &secinfo->master_id,
         sizeof(ble_gap_master_id_t));
  memcpy(&commMem.gapMem->wrapperBondInfo.sys_attr_data,
         sys_attr_data,
         BLE_GATTS_SYS_ATTR_DATA_TOTALLEN);
  sd_ble_gap_sec_info_reply(pBleNrfEvt->evt.gap_evt.conn_handle, enc_info, id_info, sign_info);
  sd_ble_gatts_sys_attr_set(pBleNrfEvt->evt.gap_evt.conn_handle,
                            sys_attr_data,
                            BLE_GATTS_SYS_ATTR_DATA_TOTALLEN,
                            0);
}

static
void onTimeout(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  pBleEvent->evtHeader = BLE_GAP_EVENT_TIMEOUT;
  ble_gap_evt_timeout_t *timeout = &pBleNrfEvt->evt.gap_evt.params.timeout;
  commMem.timeoutData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;

  switch (timeout->src)
    {
      case BLE_GAP_TIMEOUT_SRC_SCAN:
        commMem.timeoutData.timeoutSrc = BLE_GAP_TIMEOUT_SCAN;
        break;

      case BLE_GAP_TIMEOUT_SRC_CONN:
        commMem.timeoutData.timeoutSrc = BLE_GAP_TIMEOUT_CONN;
        break;

      case BLE_GAP_TIMEOUT_SRC_AUTH_PAYLOAD:
        commMem.timeoutData.timeoutSrc = BLE_GAP_TIMEOUT_SECURITY_REQUEST;
        break;

      default:

        /* It does not come here by interface specification. */

        commMem.timeoutData.timeoutSrc = timeout->src;
        break;
    }

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

static void mk_notify_req_event(BLE_SrvSds *srv)
{
  struct ble_gatt_event_notify_req_t notify;

  notify.group_id    = BLE_GROUP_GATT;
  notify.event_id    = BLE_GATT_EVENT_NOTIFY_REQ;
  notify.serv_handle = BLE_GATT_INVALID_SERVICE_HANDLE;
  notify.char_handle = srv->char_ri_handle.charHandle;
  notify.enable      = srv->notify_enabled;

  ble_gatt_event_handler((struct bt_event_t *)&notify);
}

static void mk_write_req_event(ble_gatts_evt_write_t *gattsWrite)
{
  struct ble_gatt_event_write_req_t write;

  write.group_id    = BLE_GROUP_GATT;
  write.event_id    = BLE_GATT_EVENT_WRITE_REQ;
  write.serv_handle = BLE_GATT_INVALID_SERVICE_HANDLE;
  write.char_handle = gattsWrite->handle;
  write.length      = gattsWrite->len;
  memcpy(write.data, gattsWrite->data, write.length);

  ble_gatt_event_handler((struct bt_event_t *)&write);
}

static
void onGattsWrite(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  ble_gatts_evt_write_t * gattsWrite = &pBleNrfEvt->evt.gatts_evt.params.write;
  BLE_SrvSds* srv_sds = &g_ble_context.ble_srv_sds;

  BLE_PRT("gattsWrite->data:%.*s\n",gattsWrite->len, gattsWrite->data);
  BLE_PRT("gattsWrite->handle:%x\n",gattsWrite->handle);
  BLE_PRT("srv_sds->char_ri_handle.dprHandle.cccdHandle:%x\n",
          srv_sds->char_ri_handle.dprHandle.cccdHandle);
  BLE_PRT("gattsWrite->data[0] & 0x01:%d\n",gattsWrite->data[0] & 0x01);

  if (gattsWrite->handle == srv_sds->char_ri_handle.dprHandle.cccdHandle)
    {
      /* client characteristic configuration.bit 0: Notification bit 1: Indication.other reserved.
       * see core spec 4.1 Vol3,PartG,3,3,3,3 client characteristic
       * configuration.Table 3.11
       */

      srv_sds->notify_enabled = (bool)gattsWrite->data[0] & 0x01;

      mk_notify_req_event(srv_sds);
    }
  else
    {
      if (BT_MAX_EVENT_DATA_LEN >= gattsWrite->len)
        {
          mk_write_req_event(gattsWrite);
        }
      else
        {
          BLE_PRT("incoming data exceeds buffer length\n");
        }
    }
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

  ble_gattc_evt_t *bleGattcEvt                         = &(pBleNrfEvt->evt.gattc_evt);
  const ble_gattc_evt_write_rsp_t *bleGattcEvtWriteRsp   = &(bleGattcEvt->params.write_rsp);
  struct ble_gatt_event_write_rsp_t evt;

  evt.group_id    = BLE_GROUP_GATT;
  evt.event_id    = BLE_GATT_EVENT_WRITE_RESP;
  evt.conn_handle = bleGattcEvt->conn_handle;
  evt.char_handle = bleGattcEvtWriteRsp->handle;
  evt.status      = bleGattcEvt->gatt_status;

  ble_gatt_event_handler((struct bt_event_t *)&evt);
}

static
void onHvx(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  ble_gattc_evt_t *bleGattcEvt                = &(pBleNrfEvt->evt.gattc_evt);
  const ble_gattc_evt_hvx_t *bleGattcEvtHvx   = &(bleGattcEvt->params.hvx);
  struct ble_gatt_event_notification_t evt;

  evt.group_id    = BLE_GROUP_GATT;
  evt.event_id    = BLE_GATT_EVENT_NOTIFICATION;
  evt.conn_handle = pBleNrfEvt->evt.gattc_evt.conn_handle;
  evt.char_handle = bleGattcEvtHvx->handle;
  evt.indicate    = (bleGattcEvtHvx->type == BLE_GATT_HVX_INDICATION);
  evt.length      = bleGattcEvtHvx->len;
  memcpy(evt.data, bleGattcEvtHvx->data, evt.length);

  ble_gatt_event_handler((struct bt_event_t *)&evt);
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

static void set_discoveried_data(BLE_GattcDbDiscovery *rcv,
                                 struct ble_gattc_db_discovery_s *evt)
{
  int i;
  int j;
  BLE_GattcDbDiscSrv  *rcv_srv;
  BLE_GattcDbDiscChar *rcv_ch;
  struct ble_gattc_db_disc_srv_s  *srv;
  struct ble_gattc_db_disc_char_s *ch;

  evt->srv_count = rcv->srvCount;

  srv = &evt->services[0];
  rcv_srv = &rcv->services[0];

  for (i = 0; i < evt->srv_count; i++, srv++, rcv_srv++)
    {
      srv->char_count = rcv_srv->charCount;
      srv->srv_handle_range.start_handle = rcv_srv->srvHandleRange.startHandle;
      srv->srv_handle_range.end_handle   = rcv_srv->srvHandleRange.endHandle;
      memcpy(&srv->srv_uuid, &rcv_srv->srvUuid, sizeof(BLE_UUID));

      ch = &srv->characteristics[0];
      rcv_ch = &rcv_srv->characteristics[0];

      for (j = 0; j < srv->char_count; j++, ch++, rcv_ch++)
        {
           ch->characteristic.char_prope      = rcv_ch->characteristic.charPrope;
           ch->characteristic.char_valhandle  = rcv_ch->characteristic.charValhandle;
           ch->characteristic.char_declhandle = rcv_ch->characteristic.charDeclhandle;
           memcpy(&ch->characteristic.char_valuuid,
                  &rcv_ch->characteristic.charValUuid,
                  sizeof(BLE_UUID));
           ch->cepd_handle = rcv_ch->cepdHandle;
           ch->cudd_handle = rcv_ch->cuddHandle;
           ch->cccd_handle = rcv_ch->cccdHandle;
           ch->sccd_handle = rcv_ch->sccdHandle;
           ch->cpfd_handle = rcv_ch->cpfdHandle;
           ch->cafd_handle = rcv_ch->cafdHandle;
        }
    }
}

static uint16_t get_last_handle(struct ble_gattc_db_disc_char_s *chrc)
{
  uint16_t biggest;

  biggest = chrc->characteristic.char_valhandle;
  biggest = MAX(biggest, chrc->cepd_handle);
  biggest = MAX(biggest, chrc->cudd_handle);
  biggest = MAX(biggest, chrc->cccd_handle);
  biggest = MAX(biggest, chrc->sccd_handle);
  biggest = MAX(biggest, chrc->cpfd_handle);
  biggest = MAX(biggest, chrc->cafd_handle);

  return biggest;
}

static
void setDbDiscoveryEvent(BLE_Evt *pBleEvent, uint16_t connHandle, int result, int reason)
{
  static struct ble_gatt_event_db_discovery_t evt;
  struct ble_gattc_db_disc_srv_s *last_srv;
  struct ble_gattc_db_disc_char_s *last_chrc;

  evt.group_id    = BLE_GROUP_GATT;
  evt.event_id    = BLE_GATT_EVENT_DB_DISCOVERY_COMPLETE;
  evt.result      = result;
  evt.conn_handle = connHandle;

  if (evt.result == BLE_GATTC_RESULT_SUCCESS)
    {
      set_discoveried_data(&commMem.gattcDb.dbDiscovery, &evt.params.db_discovery);
    }
  else
    {
      evt.params.reason = reason;
    }

  /* If the number of the last characteristic is BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV
   * and end_handle of the last service is BLE_GATTC_HANDLE_END,
   * continuation may be exists.
   * Then, the last handle value should be notified.
   */

  if (evt.params.db_discovery.srv_count == 0)
    {
      evt.state.end_handle = 0;
      commMem.disc.start = 0;
    }
  else
    {
      last_srv  = &evt.params.db_discovery.services[evt.params.db_discovery.srv_count - 1];
      last_chrc = &last_srv->characteristics[last_srv->char_count - 1];

      if (last_srv->char_count >= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV)
        {
          evt.state.end_handle = get_last_handle(last_chrc);
        }
      else
        {
          evt.state.end_handle = last_srv->srv_handle_range.end_handle;
          commMem.disc.start = 0;
        }
    }

  ble_gatt_event_handler((struct bt_event_t *)&evt);

  /* Initialize nrf52's local data. */

  memset(&commMem.gattcDb, 0, sizeof(commMem.gattcDb));
  commMem.gattcDb.currCharInd = 0;
  commMem.gattcDb.currSrvInd  = 0;
  return;
}

static bool is_target_uuid(const ble_uuid_t *notified, ble_uuid_t *target)
{
  if (target->type == BLE_UUID_TYPE_UNKNOWN)
    {
      /* This setting means that all information is discovered. */

      return true;
    }
  else
    {
      if (BLE_UUID_EQ(notified, target))
        {
          return true;
        }
      else
        {
          return false;
        }
    }
}

static void convert_uuid_nrf2mw(const ble_uuid_t *nrf52, BLE_Uuid *mw)
{
  uint8_t len;
  uint8_t uuid[16];

  if (nrf52->type == BLE_UUID_TYPE_BLE)
    {
      mw->value.baseAlias.uuidAlias = nrf52->uuid;
      mw->type = BLE_UUID_TYPE_BASEALIAS_BTSIG;
    }
  else if (nrf52->type == BLE_UUID_TYPE_UNKNOWN)
    {
      /* Unknown and non-encodable 128bit UUID case. */

      mw->type = BLE_UUID_TYPE_UUID128;
      memset(mw->value.uuid128.uuid128, 0, 16);
    }
  else
    {
      sd_ble_uuid_encode(nrf52, &len, uuid);
      if (len == 2)
        {
          mw->value.baseAlias.uuidAlias = (uuid[1] << 8) | uuid[0];
          mw->type = BLE_UUID_TYPE_BASEALIAS_VENDOR;
        }
      else
        {
          memcpy(mw->value.uuid128.uuid128, uuid, 16);
          mw->type = BLE_UUID_TYPE_UUID128;
        }
    }
}

static
void onPrimarySrvDiscoveryRsp(bleGattcDb *gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
  int i;
  uint16_t connHandle = 0;
  ble_gattc_evt_t *bleGattcEvt = &(pBleNrfEvt->evt.gattc_evt);
  ble_gattc_evt_prim_srvc_disc_rsp_t *srvs = &(bleGattcEvt->params.prim_srvc_disc_rsp);
  ble_gattc_service_t *srv;
  BLE_GattcDbDiscSrv *ltbl;
  int num;

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

      if (srvs->count > BLE_DB_DISCOVERY_MAX_SRV)
        {
          num = BLE_DB_DISCOVERY_MAX_SRV;
        }
      else
        {
          num = srvs->count;
        }

      gattcDbDiscovery->dbDiscovery.srvCount = num;

      for (i = 0, srv = &srvs->services[0], ltbl = &gattcDbDiscovery->dbDiscovery.services[0];
           i < num;
           i++, srv++, ltbl++)
        {
          if (is_target_uuid(&srv->uuid, &gattcDbDiscovery->target.srvUuid) == false)
            {
              continue;
            }

          convert_uuid_nrf2mw(&srv->uuid, &ltbl->srvUuid);
          ltbl->srvHandleRange.startHandle =  srv->handle_range.start_handle;
          ltbl->srvHandleRange.endHandle =  srv->handle_range.end_handle;
          BLE_PRT2("Primary Service[%d] type %d uuid 0x%04X sHdl %d eHdl %d\n",
                   i,
                   srv->uuid.type,
                   srv->uuid.uuid,
                   srv->handle_range.start_handle,
                   srv->handle_range.end_handle);
        }

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
  bool   performDescDiscov = false;
  bool raiseDiscovComplete = false;
  ble_gattc_evt_t                 *bleGattcEvt = &(pBleNrfEvt->evt.gattc_evt);
  BLE_GattcDbDiscSrv       *srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
  BLE_GattcDbDiscChar      *chtop;
  BLE_GattcChar            *ch = NULL;
  const ble_gattc_evt_char_disc_rsp_t *charDiscRspEvt;
  const ble_gattc_char_t   *rcvch;
  uint16_t                 last_handle = BLE_GATT_INVALID_ATTRIBUTE_HANDLE;
  connHandle  = bleGattcEvt->conn_handle;
  ble_gattc_handle_range_t range = {0};
  if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
    {
      BLE_ERR("onChar ind NG\n");
      gattcDbDiscovery->discoveryInProgress = false;
      setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_CHARACTERISTIC);
      return;
    }
  if (bleGattcEvt->gatt_status == BLE_GATT_STATUS_SUCCESS)
    {
      charDiscRspEvt = &(bleGattcEvt->params.char_disc_rsp);

      numCharsPrevDisc = srvBeingDiscovered->charCount;
      numCharsCurrDisc = charDiscRspEvt->count;
      BLE_PRT2("onChar preCnt %d curCnt %d\n", numCharsPrevDisc, numCharsCurrDisc);
      chtop = &srvBeingDiscovered->characteristics[numCharsPrevDisc];
      for (i = 0; i < numCharsCurrDisc; i++)
        {
          rcvch = &charDiscRspEvt->chars[i];
          last_handle = rcvch->handle_value;

          if (is_target_uuid(&rcvch->uuid, &gattcDbDiscovery->target.charUuid) == false)
            {
              continue;
            }

          ch    = &chtop->characteristic;
          memcpy(&ch->charPrope, &rcvch->char_props, sizeof(BLE_CharPrope));
          ch->charDeclhandle = rcvch->handle_decl;
          ch->charValhandle  = rcvch->handle_value;
          convert_uuid_nrf2mw(&rcvch->uuid, &ch->charValUuid);

          /* Initialize all descriptors handle with invalid value.
           * These handles are discovered later by descriptor discovery.
           */

          chtop->cccdHandle = BLE_GATT_INVALID_ATTRIBUTE_HANDLE;
          chtop->cepdHandle = BLE_GATT_INVALID_ATTRIBUTE_HANDLE;
          chtop->cuddHandle = BLE_GATT_INVALID_ATTRIBUTE_HANDLE;
          chtop->sccdHandle = BLE_GATT_INVALID_ATTRIBUTE_HANDLE;
          chtop->cpfdHandle = BLE_GATT_INVALID_ATTRIBUTE_HANDLE;
          chtop->cafdHandle = BLE_GATT_INVALID_ATTRIBUTE_HANDLE;

          BLE_PRT2("onChar type %d uuid 0x%04X ValHdl %d DecHdl %d\n",
                   rcvch->uuid.type,
                   rcvch->uuid.uuid,
                   rcvch->handle_value,
                   rcvch->handle_decl);

          srvBeingDiscovered->charCount++;
          if (srvBeingDiscovered->charCount >= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV)
            {
              break;
            }

          chtop++;
        }

      // If no more characteristic discovery is required, or if the maximum number of supported
      // characteristic per service has been reached, descriptor discovery will be performed.
      if (!isCharDiscoveryReqd(gattcDbDiscovery, last_handle) ||
      (srvBeingDiscovered->charCount >= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV))
        {
          performDescDiscov = true;
        }
      else
        {
          // Update the current characteristic index.
          gattcDbDiscovery->currCharInd = srvBeingDiscovered->charCount;
          // Perform another round of characteristic discovery.
          BLE_PRT2("onChar charDiscover\n");
          range.start_handle = last_handle + 1;
          range.end_handle   = srvBeingDiscovered->srvHandleRange.endHandle;
          sd_ble_gattc_characteristics_discover(
            gattcDbDiscovery->dbDiscovery.connHandle,
            &range);
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

static void saveDiscoveredDescriptor(uint16_t uuid,
                                     uint16_t handle,
                                     BLE_GattcDbDiscChar *charc)
{
  switch (uuid)
    {
      case BLE_UUID_DESCRIPTOR_CHAR_EXT_PROP:
        charc->cepdHandle = handle;
        break;

      case BLE_UUID_DESCRIPTOR_CHAR_USER_DESC:
        charc->cuddHandle = handle;
        break;

      case BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG:
        charc->cccdHandle = handle;
        break;

      case BLE_UUID_DESCRIPTOR_SERVER_CHAR_CONFIG:
        charc->sccdHandle = handle;
        break;

      case BLE_UUID_DESCRIPTOR_CHAR_PRESENTATION_FORMAT:
        charc->cpfdHandle = handle;
        break;

      case BLE_UUID_DESCRIPTOR_CHAR_AGGREGATE_FORMAT:
        charc->cafdHandle = handle;
        break;

      default:
        /* Unknown descriptor is discovered. */

        break;
    }

  BLE_PRT("discovered descriptor uuid:0x%04x,handle:0x%04x\n", uuid, handle);
}

static bool isLastCharacteristic(uint8_t cur_idx, uint8_t char_num)
{
  return ((cur_idx + 1) >= char_num);
}

static uint16_t getLastDescriptorHandle(bleGattcDb *db)
{
  uint16_t last;
  BLE_GattcDbDiscSrv *srv;
  BLE_GattcDbDiscChar *ch;

  srv = &db->dbDiscovery.services[db->currSrvInd];
  ch  = &srv->characteristics[db->currCharInd];

  if (isLastCharacteristic(db->currCharInd, srv->charCount))
    {
      last = srv->srvHandleRange.endHandle;
    }
  else
    {
      /* The last handle is the declaration handle of
       * next characteristic minus 1.
       */

      ch++;
      last = ch->characteristic.charDeclhandle - 1;
    }

  return last;
}

static int getDescNumOfCurrChar(const ble_gattc_evt_desc_disc_rsp_t *evt,
                                bool *incl)
{
  int i;

  *incl = false;

  /* In case of last characteristic or UUID-specified discovery,
   * the range of descriptor handle can be invalid.
   * In such a case, all attribute data is notify and
   * characteristic group is delimited by UUID = 0x2803 information,
   * that means next characteristic delaration.
   * So, delimit by 0x2803 information.
   */

  for (i = 0; i < evt->count; i++)
    {
      if (evt->descs[i].uuid.uuid == BLE_UUID_CHARACTERISTIC)
        {
          *incl = true;
          break;
        }
   }

  return i;
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
  const ble_gattc_desc_t *desc = NULL;
  ble_gattc_handle_range_t r;
  uint16_t last;
  bool includeNextChar;
  int num;

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

      num = getDescNumOfCurrChar(descDiscRspEvt, &includeNextChar);
      for (i = 0; i < num; i++)
        {
          desc = &descDiscRspEvt->descs[i];
          saveDiscoveredDescriptor(desc->uuid.uuid, desc->handle, charBeingDiscovered);
        }

      /* Continue descriptor discover of the same characteristic,
       * if the last handle of received data is different from
       * the end handle of descriptor discover request,
       */

      if (desc != NULL)
        {
          last = getLastDescriptorHandle(gattcDbDiscovery);
          if ((last != desc->handle) && !includeNextChar)
            {
              r.start_handle = desc->handle + 1;
              r.end_handle   = last;

              sd_ble_gattc_descriptors_discover(connHandle, &r);
              return;
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

static bool hasRemainingChar(bleGattcDb *gattcDbDiscovery)
{
  BLE_GattcDbDiscSrv  *curr_srv;
  BLE_GattcDbDiscChar *curr_ch;
  struct ble_gattc_db_disc_char_s chrc;
  uint16_t last;

  curr_srv = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
  curr_ch = &(curr_srv->characteristics[gattcDbDiscovery->currCharInd]);

  chrc.characteristic.char_valhandle = curr_ch->characteristic.charValhandle;
  chrc.cepd_handle = curr_ch->cepdHandle;
  chrc.cudd_handle = curr_ch->cuddHandle;
  chrc.cccd_handle = curr_ch->cccdHandle;
  chrc.sccd_handle = curr_ch->sccdHandle;
  chrc.cpfd_handle = curr_ch->cpfdHandle;
  chrc.cafd_handle = curr_ch->cafdHandle;

  last = get_last_handle(&chrc);

  if (last < curr_srv->srvHandleRange.endHandle)
    {
      return true;
    }
  else
    {
      return false;
    }
}

static
void onSrvDiscCompletion(BLE_Evt *pBleEvent, bleGattcDb *gattcDbDiscovery)
{
  BLE_GattcDbDiscSrv       *srvBeingDiscovered = NULL;

  /* If service discovery complete and this service has the more characteristics,
   * trigger event to encourage application to continue discover.
   */

  if (hasRemainingChar(gattcDbDiscovery))
    {
      gattcDbDiscovery->dbDiscovery.srvCount = gattcDbDiscovery->currSrvInd + 1;
      commMem.disc.start = gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.startHandle;

      setDbDiscoveryEvent(pBleEvent,
                    gattcDbDiscovery->dbDiscovery.connHandle,
                    BLE_GATTC_RESULT_SUCCESS, 0);
      return;
    }

  commMem.disc.start = 0;

  // Reset the current characteristic index since a new service discovery is about to start.
  gattcDbDiscovery->currCharInd = 0;

  gattcDbDiscovery->currSrvInd++;
  if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
    {
      BLE_PRT("onSrvDiscCompletion: no more service\n");
      setDbDiscoveryEvent(pBleEvent,
                      gattcDbDiscovery->dbDiscovery.connHandle,
                      BLE_GATTC_RESULT_SUCCESS, 0);
    }
  else
    {
      /* Reset the characteristic count in the next service to zero since a new service
       * discovery is about to start.
       */

      srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
      srvBeingDiscovered->charCount = 0;

      /* Discover characteristics of next service. */

      (void)characteristicsDiscover(pBleEvent, gattcDbDiscovery);
    }

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
      /* This is the first characteristic of this service being discovered.
       * In continue case, start discovery from application specified handle.
       */

      if (commMem.disc.req > BLE_GATTC_HANDLE_START)
        {
          handleRange.startHandle = commMem.disc.req;
          commMem.disc.req = BLE_GATTC_HANDLE_START;
        }
      else
        {
          handleRange.startHandle = srvBeingDiscovered->srvHandleRange.startHandle;

        }
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
                         uint16_t last_handle)
{
  BLE_PRT2("isChar charEndHdl %d srvEndHdl %d\n", last_handle,
    gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle);
  if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV)
    {
      return false;
    }
  if (last_handle <
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
      // No more descriptor discovery required.
      // Proceed to the characteristics discovery about next service.

      gattcDbDiscovery->currSrvInd++;
      gattcDbDiscovery->currCharInd = 0;

      if (gattcDbDiscovery->currSrvInd < gattcDbDiscovery->dbDiscovery.srvCount)
        {
          characteristicsDiscover(pBleEvent, gattcDbDiscovery);
        }
      else
        {
          setDbDiscoveryEvent(pBleEvent,
                              gattcDbDiscovery->dbDiscovery.connHandle,
                              BLE_GATTC_RESULT_SUCCESS, 0);
        }

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
  int ret             = 0;
  int8_t tx_power     = commMem.gapMem->txPower;
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
#ifdef CONFIG_NRF52_LESC
  pf->lesc                  = true;
#else
  pf->lesc                  = false;
#endif
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

  params.role = BLE_ROLE_PERIPHERAL_AND_CENTRAL;

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
 * Name: nrf52_ble_start_scan
 *
 * Description:
 *   Bluetooth LE start scan.
 *
 * Parameter:
 *   duplicate_filter:
 *           true means that duplicate scan results are filtered out.
 *           false means that all duplicate scan result are notified to
 *           applications.
 *           nrf52 HW do not support true value.
 *
 ****************************************************************************/

static int nrf52_ble_start_scan(bool duplicate_filter)
{
  int ret;

  /* Return error if duplicate_filter is true,
   * because nRF52 HW do not support duplicate scan result filter.
   */

  if (duplicate_filter)
    {
      return BT_FAIL;
    }

  ret = BLE_GapStartScan();
  if (ret == BLE_SUCCESS)
    {
      g_ble_context.is_scanning = true;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_stop_scan
 *
 * Description:
 *   Bluetooth LE stop scan.
 *
 ****************************************************************************/

static int nrf52_ble_stop_scan(void)
{
  int ret;
  ret = BLE_GapStopScan();
  if (ret == BLE_SUCCESS)
    {
      g_ble_context.is_scanning = false;
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

static int nrf52_ble_connect(uint8_t addr_type, const BT_ADDR *addr)
{
  int ret = BT_SUCCESS;
  BLE_GapAddr gap_addr = {0};

  gap_addr.type = addr_type;
  memcpy(gap_addr.addr, addr->address, sizeof(gap_addr.addr));

  ret = BLE_GapConnect(&gap_addr);
  if (ret == BT_SUCCESS)
    {
      g_ble_context.conn_sts = BLE_CONN_STS_CONNECTING;
    }

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
      else
        {
          g_ble_context.is_advertising = true;
        }
    }
  else
    {
      ret = BLE_GapStopAdv();

      if (BLE_SUCCESS != ret)
        {
          LOG_OUT("ble disable advertise failed, ret=%d\n", ret);
        }
      else
        {
          g_ble_context.is_advertising = false;
        }
    }
  return ret;
}

static int nrf52_ble_set_dev_addr(BT_ADDR *addr, uint8_t type)
{
  int ret;
  ble_gap_addr_t nrf52_addr;

  nrf52_addr.addr_type = type;
  memcpy(nrf52_addr.addr, addr->address, sizeof(nrf52_addr.addr));

  ret = sd_ble_gap_addr_set(&nrf52_addr);
  return bleConvertErrorCode(ret);
}

static int nrf52_ble_set_dev_name(char *name)
{
  int ret = BT_SUCCESS;
  size_t nameSize = 0;

  /* Get name length */

  nameSize = strlen(name);

  /* If invalid size, return error */

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
  uint32_t errCode;

  errCode = sd_ble_gap_appearance_set((uint16_t)appearance);
  return bleConvertErrorCode(errCode);
}

static int nrf52_ble_set_ppcp(BLE_CONN_PARAMS ppcp)
{
  int ret = BT_SUCCESS;
  return ret;
}

static uint16_t nrf52_ble_set_mtusize(uint16_t sz)
{
  uint16_t ret;

  if (sz > NRF_SDH_BLE_GATT_MAX_MTU_SIZE)
    {
      ret = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
    }
  else
    {
      ret = sz;
    }

  commMem.requested_mtu = ret;
  return ret;
}

static uint16_t nrf52_ble_get_mtusize(void)
{
  return commMem.requested_mtu;
}

static int nrf52_ble_get_negotiated_mtusize(uint16_t handle)
{
  if (handle != g_ble_context.ble_conn_handle)
    {
      return -EINVAL;
    }

  return commMem.client_rx_mtu;
}


static int nrf52_ble_pairing(uint16_t handle)
{
  int ret;

  ret = searchBondInfoIndexFmConnHandle(handle);
  if (ret < 0)
    {
      /* Invalid handle error */

      return ret;
    }
  else if (ret < BLE_SAVE_BOND_DEVICE_MAX_NUM)
    {
      /* If pairing information about this connection is stored,
       * skip pairing and only encrypt with stored key.
       */

      if (BondInfoInFlash[ret].ownEncKey.master_id.ediv != 0)
        {
          /* LE Legacy Pairing */

          ret = BLE_GapEncrypt(handle, &BondInfoInFlash[ret].peerEncKey);
        }
      else
        {
          /* LE Secure Connections Pairing */

          ret = BLE_GapEncrypt(handle, &BondInfoInFlash[ret].ownEncKey);
        }
    }
  else
    {
      /* Otherwise, execute pairing. */

      ret = BLE_GapAuthenticate(handle, &g_ble_context.pairing_feature);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_set_txpower
 *
 * Description:
 *   Bluetooth LE set Tx Power
 *   Supported tx_power values:
 *    -40, -20, -16, -12, -8, -4, 0, +3 and +4 [dBm]
 *
 ****************************************************************************/

static int nrf52_ble_set_txpower(int8_t tx_power)
{
  int ret = 0;

  if (!commMem.gapMem)
    {
      LOG_OUT("Must be called after BLE is enabled.\n");
      return -EPERM;
    }

  switch (tx_power)
    {
      case -40:
      case -20:
      case -16:
      case -12:
      case -8:
      case -4:
      case 0:
      case 3:
      case 4:
        commMem.gapMem->txPower = tx_power;
        break;
      default:
        commMem.gapMem->txPower = 0; /* set default if illegal value */
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_set_scan_param
 *
 * Description:
 *   Bluetooth LE set scan parameter
 *
 ****************************************************************************/

static int nrf52_ble_set_scan_param(struct ble_scan_param_s *param)
{
  return BLE_GapSetScanParam(param);
}

/****************************************************************************
 * Name: nrf52_ble_set_conn_param
 *
 * Description:
 *   Bluetooth LE set connection parameter
 *
 ****************************************************************************/

static int nrf52_ble_set_conn_param(struct ble_conn_param_s *param)
{
  return BLE_GapSetConnectionParams((BLE_GapConnParams *)param);
}

/****************************************************************************
 * Public Data
 ****************************************************************************/
BLE_Context g_ble_context;

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
  return BT_SUCCESS;
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

/****************************************************************************
 * Name: nrf52_ble_common_register
 *
 * Description:
 *   Register BLE common HAL I/F
 *
 ****************************************************************************/

int nrf52_ble_common_register(void)
{
  return ble_common_register_hal(&ble_hal_common_ops);
}

/****************************************************************************
 * Name: nrf52_bt_common_register
 *
 * Description:
 *   Register BT common HAL I/F
 *
 ****************************************************************************/

int nrf52_bt_common_register(void)
{
  return bt_common_register_hal(&bt_hal_common_ops);
}

