/****************************************************************************
 * modules/bluetooth/hal/nrf52/ble_comm_internal.h
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
 * @file       ble_common_internal.h
 * @brief      Sharing common data structures between ble_xxx.c internally.
 * @attention
 */
#ifndef __MODULES_BLUETOOTH_HAL_NRF52_BLE_COMM_INTERNAL_H
#define __MODULES_BLUETOOTH_HAL_NRF52_BLE_COMM_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <ble/ble_comm.h>
#include <ble/ble_gap.h>
#include <ble/ble_gatts.h>
#include <ble/ble_gattc.h>
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "app_util.h"
#include <semaphore.h>

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOND_SIZE_KEY_NAME       "0000"
#define BOND_INFO_KEY_NAME       "0001"
#define BOND_ENABLE_KEY_NAME     "1000"
#define BOND_INFO_1_KEY_NAME     "1001"
#define BOND_INFO_2_KEY_NAME     "1002"
#define BOND_INFO_3_KEY_NAME     "1003"
#define BOND_INFO_4_KEY_NAME     "1004"
#define BOND_INFO_5_KEY_NAME     "1005"
#define BOND_INFO_6_KEY_NAME     "1006"
#define BOND_INFO_7_KEY_NAME     "1007"
#define BOND_INFO_8_KEY_NAME     "1008"

#define BLE_GATTC_HANDLE_END 0xFFFF
#define APP_BLE_CONN_CFG_TAG 1
#define CONN_HANDLE_INVALED 0xffff
#ifndef BLE_USE_SECTION
extern void bleEvtDispatch(ble_evt_t *pBleNrfEvt);
#endif

typedef struct{
  uint32_t enable_key;
  uint32_t info_key[BLE_SAVE_BOND_DEVICE_MAX_NUM];
}bleBondInfoKey;

typedef struct
{
  BLE_GapConnHandle connHandle;
  BLE_GapBondInfo bondInfo;
  //ble_gap_id_key_t ownIdKey;
  //ble_gap_enc_key_t ownEncKey;
  ble_gap_id_key_t peerIdKey;
  ble_gap_enc_key_t peerEncKey;
} bleGapWrapperBondInfo;

/**@brief A collection of variables of gap. */
typedef struct
{
  bleGapWrapperBondInfo wrapperBondInfo;
  uint8_t advHandle;
  uint8_t gapAdvData[BLE_GAP_ADV_MAX_SIZE];
  uint16_t gapAdvLen;
  ble_gap_scan_params_t scanParams;
  ble_gap_conn_params_t connParams;
  ble_gap_sec_keyset_t keySet;
  uint8_t is_connected;
  int8_t peerRssi;
  uint8_t startRssi;
} bleGapMem;

typedef struct
{
  uint8_t                 currCharInd;
  uint8_t                 currSrvInd;
  bool                    discoveryInProgress;
  uint8_t                 reserve;
  BLE_GattcDbDiscovery    dbDiscovery;
} bleGattcDb;

/**@brief A collection of variables of common. */
 typedef struct
{
  uint8_t                 callbackFlag;
  uint8_t                 managerCbFlag;
  void                    (*managerCb)(ble_evt_t *nrfEvt);
  BLE_EfCb                cbId;
  uint32_t                bondInfoId;
  BLE_EvtCtx              *bleEvtCtx;
  BLE_EvtTxComplete       txCompleteData;
  BLE_EvtConnected        connectData;
  BLE_EvtConnParamUpdate  connParams;
  BLE_EvtDisconnected     disconnectData;
  BLE_EvtExchangeFeature  exchangeFeatureData;
  BLE_EvtAuthStatus       authStatusData;
  BLE_EvtDisplayPasskey   dispPasskeyData;
  BLE_EvtAdvReportData    advReportData;
  BLE_EvtAuthKey          authKeyData;
  BLE_EvtTimeout          timeoutData;
  BLE_EvtDataLengthUpdate dataLen;
  BLE_EvtRssiChanged      rssiChangeData;
  BLE_EvtGattsWrite       gattsWriteData;
  BLE_EvtGattsIndConfirm  gattsIndConfirmData;
  BLE_EvtGattsExchangeMTU gattsExchangeMTU;
  BLE_EvtGattcRead        gattcReadData;
  BLE_EvtGattcNtfInd      gattcNtfIndData;
  BLE_EvtGattcDbDiscovery gattcDbDiscoveryData;
  bleGattcDb              gattcDb;
  bleGapMem               *gapMem;
  uint8_t                 stackInited;
  uint16_t                client_rx_mtu;
  BLE_EvtPhyUpdate        phyUpdate;
} bleCommMem;

typedef struct ble_srv_sds
{
  bool notify_enabled;
  BLE_GapConnHandle conn_handle;
  uint16_t srv_handle;
  uint8_t char_rw_buf[MAX_RCV_DATA_LENGTH]; /* read / write */
  BLE_GattsCharHandles char_rw_handle;
  int32_t char_rw_curr_idx; /* current data length index */
  sem_t char_rw_buf_sem;
  uint8_t char_ri_buf[MAX_RCV_DATA_LENGTH]; /* read / indication */
  BLE_GattsCharHandles char_ri_handle;
} BLE_SrvSds;

typedef struct ble_context
{
  BLE_EvtCtx ble_evt_ctx;
  BLE_GapAddr ble_addr;
  BLE_GapDeviceConfig ble_dev_cfg;
  BLE_GapName ble_name;
  BLE_GapConnHandle ble_conn_handle;
  uint8_t ble_role;
  BLE_SrvSds ble_srv_sds;
  BLE_GapPairingFeature pairing_feature;
} BLE_Context;

extern BLE_Context g_ble_context;

struct bt_common_context_s  
{
  BT_ADDR bt_addr;  /* Common address for BT/BLE */
  char bt_name[BT_NAME_LEN];
  char ble_name[BT_NAME_LEN];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern struct bt_common_context_s bt_common_context;

#ifdef __cplusplus
}
#endif  /* __cplusplus */
/** @} ble_struct */
#endif  /* __MODULES_BLUETOOTH_HAL_NRF52_BLE_COMM_INTERNAL_H */

