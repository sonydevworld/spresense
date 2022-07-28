/****************************************************************************
 * modules/bluetooth/hal/nrf52/ble_gap.c
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

/******************************************************************************
 * Include
 *****************************************************************************/

#include <nuttx/config.h>
#include <string.h>
#include <ble/ble_comm.h>
#include <ble/ble_gap.h>
#include "ble_comm_internal.h"
#include "ble_storage_operations.h"

/******************************************************************************
 * externs
 *****************************************************************************/
extern int bleConvertErrorCode(uint32_t errCode);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADV_DATA_TYPE_FLAGS_SIZE                    0x02
#define ADV_DATA_TYPE_COMPLETE_16_UUIDS_SIZE        0x03
#define ADV_DATA_TYPE_COMPLETE_32_UUIDS_SIZE        0x05
#define ADV_DATA_TYPE_COMPLETE_128_UUIDS_SIZE       17

#define ADV_DEFAULT_INTERVAL            50
#if NRF_SD_BLE_API_VERSION > 5
#define ADV_DEFAULT_TIMEOUT             18000 // 10ms units
#else
#define ADV_DEFAULT_TIMEOUT             180
#endif

#define SCAN_INTERVAL                   0x0200
#define SCAN_WINDOW                     0x0020
#if NRF_SD_BLE_API_VERSION > 5
#define SCAN_TIMEOUT                    6000 // 10ms units
#else
#define SCAN_TIMEOUT                    60
#endif
#define MIN_SCAN_INTERVAL               0x0004
#define MAX_SCAN_INTERVAL               0x4000
#define MIN_SCAN_WINDOW                 0x0004
#define MAX_SCAN_WINDOW                 0x4000
#define MIN_SCAN_TIMEOUT                0x0000
#define MAX_SCAN_TIMEOUT                0xFFFF

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)
#define SLAVE_LATENCY                   0
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)

/* Flash handle key name size */
#define FLASH_KEY_NAME_SIZE      4
#define FLASH_KEY_NAME_2         2
#define FLASH_KEY_NAME_3         3
#define BLE_GAP_ADDR_LENGTH_4    4
#define BLE_GAP_ADDR_LENGTH_5    5
#define BLE_KEY_MASK             0x7F

// #define BLE_DBGPRT_ENABLE
#ifdef BLE_DBGPRT_ENABLE
#include <stdio.h>
#define BLE_PRT printf
#else
#define BLE_PRT(...)
#endif

 /******************************************************************************
 * Structre define
 *****************************************************************************/

 /******************************************************************************
 * Function prototype declaration
 *****************************************************************************/
bleGapMem *bleGetGapMem(void);
static int getAddress(BLE_GapAddr *addr);
static int setAddress(BLE_GapAddr *addr);
static int getName(BLE_GapName *name);
static int setName(BLE_GapName *name);
static int setCustomBaseuuid(uint8_t uuid[]);

/******************************************************************************
 * Variable
 *****************************************************************************/
static bleGapMem gapMem = {{0}};

extern bleGapWrapperBondInfo BondInfoInFlash[BLE_SAVE_BOND_DEVICE_MAX_NUM];
extern uint32_t bleBondEnableList;
extern bleBondInfoKey bleKey;

/******************************************************************************
 * Function
 *****************************************************************************/
bleGapMem *bleGetGapMem(void)
{
  return &gapMem;
}

int BLE_GapGetDeviceConfig(BLE_GapDeviceConfig *deviceConfig)
{
  int type = 0;
  int ret = BLE_SUCCESS;

  if((deviceConfig == NULL) || (deviceConfig->data == NULL)) {
    return -EINVAL;
  }

  type = deviceConfig->type;
  switch( type ) {
  case BLE_GAP_DEVICE_CONFIG_ADDR:
    ret = getAddress((BLE_GapAddr *)deviceConfig->data);
    break;
  case BLE_GAP_DEVICE_CONFIG_NAME:
    ret = getName((BLE_GapName *)deviceConfig->data);
    break;
  default:
    ret = -EINVAL;
    break;
  }
  return ret;
}

int BLE_GapSetDeviceConfig(BLE_GapDeviceConfig *deviceConfig)
{
  int type = 0;
  int ret = BLE_SUCCESS;

  if((deviceConfig == NULL)  || (deviceConfig->data == NULL)) {
    return -EINVAL;
  }

  type = deviceConfig->type;
  switch( type ) {
  case BLE_GAP_DEVICE_CONFIG_ADDR:
    ret = setAddress((BLE_GapAddr *)deviceConfig->data);
    break;
  case BLE_GAP_DEVICE_CONFIG_NAME:
    ret = setName((BLE_GapName *)deviceConfig->data);
    break;
  case BLE_GAP_DEVICE_CONFIG_UUID:
    ret = setCustomBaseuuid((uint8_t *)deviceConfig->data);
    break;
  case BLE_GAP_DEVICE_CONFIG_SERV_CHNG_ENABLE:
    /* TODO */
    break;
  case BLE_GAP_DEVICE_CONFIG_APPR_VALUE:
    /* TODO */
    break;
  case BLE_GAP_DEVICE_CONFIG_PPCP_VALUE:
    /* TODO */
    break;
  default:
    ret = -EINVAL;
    break;
  }
  return ret;
}

int BLE_GapSetAdvData(BLE_GapAdvData *advData)
{
  int errCode = BLE_SUCCESS, ret = BLE_SUCCESS;
  uint8_t index = 0;
  uint8_t advLen = 0;
  uint8_t *data = NULL;

  if(advData == NULL) {
    return -EINVAL;
  }

  if(advData->flags != 0) {
    gapMem.gapAdvData[index++] = ADV_DATA_TYPE_FLAGS_SIZE;
    gapMem.gapAdvData[index++] = ADV_DATA_TYPE_FLAGS;
    gapMem.gapAdvData[index++] = advData->flags;
  }
  if(advData->complete16Uuid != 0) {
    gapMem.gapAdvData[index++] = ADV_DATA_TYPE_COMPLETE_16_UUIDS_SIZE;
    gapMem.gapAdvData[index++] = ADV_DATA_TYPE_COMPLETE_16_UUIDS;
    gapMem.gapAdvData[index++] = (uint8_t)(advData->complete16Uuid >> 0);
    gapMem.gapAdvData[index++] = (uint8_t)(advData->complete16Uuid >> 8);
  }
  if(advData->complete32Uuid != 0) {
    gapMem.gapAdvData[index++] = ADV_DATA_TYPE_COMPLETE_32_UUIDS_SIZE;
    gapMem.gapAdvData[index++] = ADV_DATA_TYPE_COMPLETE_32_UUIDS;
    gapMem.gapAdvData[index++] = (uint8_t)(advData->complete32Uuid >> 0);
    gapMem.gapAdvData[index++] = (uint8_t)(advData->complete32Uuid >> 8);
    gapMem.gapAdvData[index++] = (uint8_t)(advData->complete32Uuid >> 16);
    gapMem.gapAdvData[index++] = (uint8_t)(advData->complete32Uuid >> 24);
  }
  if(advData->complete128Uuid != 0) {
    gapMem.gapAdvData[index++] = ADV_DATA_TYPE_COMPLETE_128_UUIDS_SIZE;
    gapMem.gapAdvData[index++] = ADV_DATA_TYPE_COMPLETE_128_UUIDS;
    memcpy(&gapMem.gapAdvData[index], advData->complete128Uuid, 16);
    index += 16;
  }
  advLen = advData->completeLocalName.advLength;
  data = advData->completeLocalName.advData;
  if((advLen != 0)&&(data != NULL)) {
    gapMem.gapAdvData[index++] = advLen+1;
    gapMem.gapAdvData[index++] = ADV_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(&gapMem.gapAdvData[index], data, advLen);
    index += advLen;
  }
  advLen = advData->manufacturerSpecificData.advLength;
  data = advData->manufacturerSpecificData.advData;
  if((advLen != 0)&&(data != NULL)) {
    gapMem.gapAdvData[index++] = advLen+1;
    gapMem.gapAdvData[index++] = ADV_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA;
    memcpy(&gapMem.gapAdvData[index], data, advLen);
    index += advLen;
  }
  if ((advData->serviceDataCount != 0) && (advData->serviceData != NULL)) {
    int i;

    for (i = 0; i < advData->serviceDataCount; i++) {
      BLE_GapAdvStructure *pServiceData = &(advData->serviceData[i]);

      advLen = pServiceData->advLength;
      data = pServiceData->advData;
      if ((advLen != 0) && (data != NULL)) {
        gapMem.gapAdvData[index++] = advLen + 1;
        gapMem.gapAdvData[index++] = ADV_DATA_TYPE_SERVICE_DATA;
        memcpy(&gapMem.gapAdvData[index], data, advLen);
        index += advLen;
      }
    }
  }
  gapMem.gapAdvLen = index+1;
  if(gapMem.gapAdvLen > BLE_GAP_ADV_MAX_SIZE) {
    return -EINVAL;
  }
#if NRF_SD_BLE_API_VERSION > 5
  // Configure a initial advertising configuration. The advertising data and and advertising
  // parameters will be changed later when we call @ref ble_advertising_start, but must be set
  // to legal values here to define an advertising handle.
  ble_gap_adv_params_t advParams = {0};
  advParams.primary_phy     = BLE_GAP_PHY_1MBPS;
  advParams.duration        = ADV_DEFAULT_TIMEOUT;
  advParams.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
  advParams.p_peer_addr     = NULL;
  advParams.filter_policy   = BLE_GAP_ADV_FP_ANY;
  advParams.interval        = ADV_DEFAULT_INTERVAL;
  errCode = sd_ble_gap_adv_set_configure(&gapMem.advHandle, NULL, &advParams);
  if (errCode) {
    BLE_PRT("BLE_GapSetAdvData:sd_ble_gap_adv_set_configure fail 0x%x\n", errCode);
    goto end;
  }
  errCode = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, gapMem.advHandle, advData->txPower);
  if (errCode) {
    BLE_PRT("BLE_GapStartAdv:sd_ble_gap_tx_power_set fail 0x%x\n", errCode);
  }
end:
#else
  errCode = sd_ble_gap_adv_data_set(gapMem.gapAdvData, gapMem.gapAdvLen, NULL, 0);
#endif
  ret = bleConvertErrorCode((uint32_t)errCode);

  return ret;
}

int BLE_GapStartAdvExt(BLE_GapAdvParams *advParams)
{
  int errCode = BLE_SUCCESS, ret = BLE_SUCCESS;

#if NRF_SD_BLE_API_VERSION > 5
  if (!((advParams->primary_phy == BLE_GAP_PHY_1MBPS) || (advParams->primary_phy == BLE_GAP_PHY_CODED))) {
    return -EINVAL;
  }

  ble_gap_adv_data_t advData = {0};
  advData.adv_data.p_data = gapMem.gapAdvData;
  advData.adv_data.len    = gapMem.gapAdvLen;

  ble_gap_adv_params_t params = {0};
  params.interval = advParams->interval;
  params.duration = advParams->duration;
  params.primary_phy = advParams->primary_phy;
  params.secondary_phy = advParams->primary_phy;
  params.p_peer_addr     = NULL;
  params.filter_policy   = BLE_GAP_ADV_FP_ANY;
  if (params.primary_phy == BLE_GAP_PHY_1MBPS) {
    params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
  }
  else {
    params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
  }

  errCode = sd_ble_gap_adv_set_configure(&gapMem.advHandle, &advData, &params);
  if (errCode) {
    BLE_PRT("BLE_GapStartAdv:sd_ble_gap_adv_set_configure fail 0x%x\n", errCode);
    goto end;
  }
  errCode = sd_ble_gap_adv_start(gapMem.advHandle, APP_BLE_CONN_CFG_TAG);
end:
#else
  errCode = NRF_ERROR_NOT_SUPPORTED;
#endif
  ret = bleConvertErrorCode((uint32_t)errCode);

  return ret;
}

int BLE_GapStartAdv(void)
{
  int errCode = BLE_SUCCESS, ret = BLE_SUCCESS;
  ble_gap_adv_params_t advParams = {0};

#if NRF_SD_BLE_API_VERSION > 5
  ble_gap_adv_data_t advData = {0};
  advData.adv_data.p_data = gapMem.gapAdvData;
  advData.adv_data.len    = gapMem.gapAdvLen;

  advParams.primary_phy     = BLE_GAP_PHY_1MBPS;
  advParams.duration        = ADV_DEFAULT_TIMEOUT;
  advParams.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
  advParams.p_peer_addr     = NULL;
  advParams.filter_policy   = BLE_GAP_ADV_FP_ANY;
  advParams.interval        = ADV_DEFAULT_INTERVAL;
  errCode = sd_ble_gap_adv_set_configure(&gapMem.advHandle, &advData, &advParams);
  if (errCode) {
    BLE_PRT("BLE_GapStartAdv:sd_ble_gap_adv_set_configure fail 0x%x\n", errCode);
    goto end;
  }
  errCode = sd_ble_gap_adv_start(gapMem.advHandle, APP_BLE_CONN_CFG_TAG);
end:
#else
  advParams.type = BLE_GAP_ADV_TYPE_ADV_IND;
  advParams.p_peer_addr = NULL;
  advParams.fp = BLE_GAP_ADV_FP_ANY;
  advParams.interval = ADV_DEFAULT_INTERVAL;
  advParams.timeout = ADV_DEFAULT_TIMEOUT;
  errCode = sd_ble_gap_adv_start(&advParams, APP_BLE_CONN_CFG_TAG);
#endif
  ret = bleConvertErrorCode((uint32_t)errCode);

  return ret;
}

int BLE_GapStopAdv(void)
{
  int errCode = BLE_SUCCESS;
#if NRF_SD_BLE_API_VERSION > 5
  // arg is tentative.
  errCode = sd_ble_gap_adv_stop(gapMem.advHandle);
#else
  errCode = sd_ble_gap_adv_stop();
#endif
  return bleConvertErrorCode((uint32_t)errCode);
}

int BLE_GapSetSecParam(BLE_GapSecCfg *param)
{
  int errCode  = 0;
  int ret      = BLE_SUCCESS;
  BLE_GapSecCfgPasskey *pkey = NULL;
  ble_opt_t    bleOpt = {0};

  if((param == NULL) || (param->data == NULL))
    {
      return -EINVAL;
    }
  memset(&bleOpt,0x00,sizeof(bleOpt));
  switch(param->type)
    {
      case SEC_CFG_PASSKEY:
        {
          pkey = (BLE_GapSecCfgPasskey *)param->data;
          bleOpt.gap_opt.passkey.p_passkey = (uint8_t*)(pkey->passkey);
          errCode = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &bleOpt);
          ret = bleConvertErrorCode((uint32_t)errCode);
          break;
        }

      default:
        ret = -EINVAL;
        break;
    }
  return ret;
}

int BLE_GapExchangePairingFeature(BLE_GapConnHandle connHandle, BLE_GapPairingFeature *pairingFeature)
{
  int ret      = BLE_SUCCESS;
  int errCode  = 0;
  ble_gap_sec_params_t secParams = {0};
  ble_gap_sec_keyset_t keysExchanged = {{0}};

  if(pairingFeature == NULL) {
    BLE_PRT("pairingFeature is NULL!!!\n");
    return -EINVAL;
  }
  if((pairingFeature->oob != BLE_GAP_OOB_AUTH_DATA_PRESENT) &&
     (pairingFeature->oob != BLE_GAP_OOB_AUTH_DATA_NOT_PRESENT)) {
    BLE_PRT("pairingFeature->oob=%d\n", pairingFeature->oob);
    return -EINVAL;
  }
  if((pairingFeature->maxKeySize < pairingFeature->minKeySize) ||
    (pairingFeature->maxKeySize > BLE_GAP_MAX_KEY_SIZE) ||
    (pairingFeature->minKeySize < BLE_GAP_MIN_KEY_SIZE)) {
    BLE_PRT("pairingFeature->maxKeySize=%d, pairingFeature->minKeySize=%d\n", pairingFeature->maxKeySize, pairingFeature->minKeySize);
    return -EINVAL;
  }

  memset(&secParams,0,sizeof(secParams));
  memset(&keysExchanged,0,sizeof(keysExchanged));

  //memset(&gapMem.wrapperBondInfo.ownEncKey,0,sizeof(gapMem.wrapperBondInfo.ownEncKey));
  //memset(&gapMem.wrapperBondInfo.ownIdKey,0,sizeof(gapMem.wrapperBondInfo.ownIdKey));
  memset(&gapMem.wrapperBondInfo.peerEncKey,0,sizeof(gapMem.wrapperBondInfo.peerEncKey));
  memset(&gapMem.wrapperBondInfo.peerIdKey,0,sizeof(gapMem.wrapperBondInfo.peerIdKey));

  secParams.oob          = pairingFeature->oob;
  secParams.io_caps      = pairingFeature->ioCap;
  secParams.max_key_size = pairingFeature->maxKeySize;
  secParams.min_key_size = pairingFeature->minKeySize;
  secParams.mitm         = (pairingFeature->authReq & BLE_GAP_AUTH_MITM) >> 1;
  secParams.bond         = (pairingFeature->authReq & BLE_GAP_AUTH_BOND) >> 0;

  //secParams.kdist_own.enc = 1;
  //secParams.kdist_own.id  = 1;
  secParams.kdist_peer.enc  = 1;
  secParams.kdist_peer.id   = 1;

  //keysExchanged.keys_own.p_enc_key  = &gapMem.wrapperBondInfo.ownEncKey;
  //keysExchanged.keys_own.p_id_key   = &gapMem.wrapperBondInfo.ownIdKey;
  keysExchanged.keys_peer.p_enc_key    = &gapMem.wrapperBondInfo.peerEncKey;
  keysExchanged.keys_peer.p_id_key    = &gapMem.wrapperBondInfo.peerIdKey;

  errCode = sd_ble_gap_sec_params_reply(connHandle, BLE_GAP_SEC_STATUS_SUCCESS, &secParams, &keysExchanged);
  ret = bleConvertErrorCode((uint32_t)errCode);
  memcpy(&gapMem.keySet, &keysExchanged, sizeof(ble_gap_sec_keyset_t));
#ifdef BLE_DBGPRT_ENABLE
#if 0
  // own
  BLE_PRT("ExchangePairing: own id_irk=0x");
  for (int i=0; i<16; i++) {
    BLE_PRT("%02x", gapMem.wrapperBondInfo.ownIdKey.id_info.irk[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("ExchangePairing: own id_addr_type=%d\n", gapMem.wrapperBondInfo.ownIdKey.id_addr_info.addr_type);
  BLE_PRT("ExchangePairing: own id_addr=0x");
  for (int i=0; i<6; i++) {
    BLE_PRT("%02x", gapMem.wrapperBondInfo.ownIdKey.id_addr_info.addr[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("ExchangePairing: own master_id_ediv=%d\n", gapMem.wrapperBondInfo.ownEncKey.master_id.ediv);
  BLE_PRT("ExchangePairing: own master_id_rand=0x");
  for (int i=0; i<8; i++) {
    BLE_PRT("%02x", gapMem.wrapperBondInfo.ownEncKey.master_id.rand[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("ExchangePairing: own enc_info_auth=%d\n", gapMem.wrapperBondInfo.ownEncKey.enc_info.auth);
  if (gapMem.wrapperBondInfo.ownEncKey.enc_info.ltk_len) {
    BLE_PRT("ExchangePairing: own enc_info_ltk=0x");
    for (int i=0; i<gapMem.wrapperBondInfo.ownEncKey.enc_info.ltk_len; i++) {
      BLE_PRT("%02x", gapMem.wrapperBondInfo.ownEncKey.enc_info.ltk[i]);
    }
    BLE_PRT("\n");
  } else {
    BLE_PRT("ExchangePairing: peer enc_info_ltk=%d\n", gapMem.wrapperBondInfo.peerEncKey.enc_info.ltk_len);
  }
#endif
  // peer
  BLE_PRT("ExchangePairing: peer id_irk=0x");
  for (int i=0; i<16; i++) {
    BLE_PRT("%02x", gapMem.wrapperBondInfo.peerIdKey.id_info.irk[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("ExchangePairing: peer id_addr_type=%d\n", gapMem.wrapperBondInfo.peerIdKey.id_addr_info.addr_type);
  BLE_PRT("ExchangePairing: peer id_addr=0x");
  for (int i=0; i<6; i++) {
    BLE_PRT("%02x", gapMem.wrapperBondInfo.peerIdKey.id_addr_info.addr[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("ExchangePairing: peer master_id_ediv=%d\n", gapMem.wrapperBondInfo.peerEncKey.master_id.ediv);
  BLE_PRT("ExchangePairing: peer master_id_rand=0x");
  for (int i=0; i<8; i++) {
    BLE_PRT("%02x", gapMem.wrapperBondInfo.peerEncKey.master_id.rand[i]);
  }
  BLE_PRT("\n");
  BLE_PRT("ExchangePairing: peer enc_info_auth=%d\n", gapMem.wrapperBondInfo.peerEncKey.enc_info.auth);
  if (gapMem.wrapperBondInfo.peerEncKey.enc_info.ltk_len) {
    BLE_PRT("ExchangePairing: peer enc_info_ltk=0x");
    for (int i=0; i<gapMem.wrapperBondInfo.peerEncKey.enc_info.ltk_len; i++) {
      BLE_PRT("%02x", gapMem.wrapperBondInfo.peerEncKey.enc_info.ltk[i]);
    }
    BLE_PRT("\n");
  } else {
    BLE_PRT("ExchangePairing: peer enc_info_ltk=%d\n", gapMem.wrapperBondInfo.peerEncKey.enc_info.ltk_len);
  }
#endif
  return ret;
}

#if NRF_SD_BLE_API_VERSION > 5
static int8_t ble_gap_scan_tx_power = 0;
uint8_t bleAdvReportBuffer[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN];
#endif

int BLE_GapSetScanParam(BLE_GapScanParams *scanParam)
{
  if(scanParam == NULL) {
    return -EINVAL;
  }
  if (!(scanParam->interval > MAX_SCAN_INTERVAL ||
      scanParam->interval < MIN_SCAN_INTERVAL ||
      scanParam->window > MAX_SCAN_WINDOW ||
      scanParam->window  < MIN_SCAN_WINDOW ||
      scanParam->timeout > MAX_SCAN_TIMEOUT ||
      scanParam->timeout < MIN_SCAN_TIMEOUT)) {
    gapMem.scanParams.active       = scanParam->active;
    //gapMem.scanParams.selective    = 0;
    gapMem.scanParams.interval     = scanParam->interval;
    gapMem.scanParams.window       = scanParam->window;
    //gapMem.scanParams.p_whitelist  = NULL;
    gapMem.scanParams.timeout      = scanParam->timeout;
  }
  else {
    return -EINVAL;
  }
#if NRF_SD_BLE_API_VERSION > 5
  gapMem.scanParams.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
  gapMem.scanParams.scan_phys     = scanParam->scan_phys;
  if (gapMem.scanParams.scan_phys == BLE_GAP_PHY_CODED) {
    gapMem.scanParams.extended    = 1;
  }
  else {
    gapMem.scanParams.extended    = 0;
  }
  ble_gap_scan_tx_power           = scanParam->tx_power;
#endif
  return 0;
}

int BLE_GapStartScan(void)
{
  int ret      = BLE_SUCCESS;
  int errCode  = 0;

  if ((gapMem.scanParams.interval > MAX_SCAN_INTERVAL ||
      gapMem.scanParams.interval < MIN_SCAN_INTERVAL ||
      gapMem.scanParams.window > MAX_SCAN_WINDOW ||
      gapMem.scanParams.window  < MIN_SCAN_WINDOW ||
      gapMem.scanParams.timeout > MAX_SCAN_TIMEOUT ||
      gapMem.scanParams.timeout < MIN_SCAN_TIMEOUT)) {
    // default
    gapMem.scanParams.active       = 0;
    gapMem.scanParams.interval     = SCAN_INTERVAL;
    gapMem.scanParams.window       = SCAN_WINDOW;
    gapMem.scanParams.timeout      = SCAN_TIMEOUT;
#if NRF_SD_BLE_API_VERSION > 5
    gapMem.scanParams.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
    gapMem.scanParams.scan_phys     = BLE_GAP_PHY_1MBPS;
    gapMem.scanParams.extended      = 0;
    ble_gap_scan_tx_power           = 0;
#endif
  }
#if NRF_SD_BLE_API_VERSION > 5
  errCode = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, 0,
    ble_gap_scan_tx_power);
  if (errCode) {
    BLE_PRT("sd_ble_gap_tx_power_set fail 0x%x\n", errCode);
    goto end;
  }

  ble_data_t reportBuf = {0};
  reportBuf.p_data = bleAdvReportBuffer;
  reportBuf.len = BLE_GAP_SCAN_BUFFER_EXTENDED_MIN;
  errCode = sd_ble_gap_scan_start(&gapMem.scanParams, &reportBuf);
end:
#else
  errCode = sd_ble_gap_scan_start(&gapMem.scanParams);
#endif
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

int BLE_GapStartScanExt(BLE_GapScanParams *scanparams)
{
  int ret      = BLE_SUCCESS;
  int errCode  = 0;

  if(scanparams == NULL) {
    return -EINVAL;
  }

  ble_gap_scan_params_t p = {0};
  p.active       = scanparams->active;
  p.interval     = scanparams->interval;
  p.window       = scanparams->window;
  p.timeout      = scanparams->timeout;
#if NRF_SD_BLE_API_VERSION > 5
  p.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
  p.scan_phys     = scanparams->scan_phys;
  if (p.scan_phys == BLE_GAP_PHY_CODED) {
    p.extended    = 1;
  }
  gapMem.scanParams = p;

  ble_gap_scan_tx_power = scanparams->tx_power;
  errCode = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, 0,
    ble_gap_scan_tx_power);
  if (errCode) {
    BLE_PRT("sd_ble_gap_tx_power_set fail 0x%x\n", errCode);
    goto end;
  }

  ble_data_t reportBuf = {0};
  reportBuf.p_data = bleAdvReportBuffer;
  reportBuf.len = BLE_GAP_SCAN_BUFFER_EXTENDED_MIN;
  errCode = sd_ble_gap_scan_start(&gapMem.scanParams, &reportBuf);
end:
#else
  gapMem.scanParams = p;
  errCode = sd_ble_gap_scan_start(&gapMem.scanParams);
#endif
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

int BLE_GapStopScan(void)
{
  int ret      = BLE_SUCCESS;
  int errCode  = 0;

  errCode = sd_ble_gap_scan_stop();
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

int BLE_GapConnect(BLE_GapAddr *addr)
{
  int ret      = BLE_SUCCESS;
  int errCode  = 0;
  ble_gap_addr_t peerAddr = {0};

  if(addr == NULL) {
    return -EINVAL;
  }

  gapMem.connParams.min_conn_interval = MIN_CONNECTION_INTERVAL;
  gapMem.connParams.max_conn_interval = MAX_CONNECTION_INTERVAL;
  gapMem.connParams.slave_latency = SLAVE_LATENCY;
  gapMem.connParams.conn_sup_timeout = SUPERVISION_TIMEOUT;
  peerAddr.addr_type = addr->type;
  memcpy(peerAddr.addr, addr->addr, BLE_GAP_ADDR_LENGTH);
  errCode = sd_ble_gap_connect(&peerAddr,
                  &gapMem.scanParams,
                  &gapMem.connParams,
                  APP_BLE_CONN_CFG_TAG);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

int BLE_GapUpdateConnectionParams(BLE_GapConnHandle connHandle, BLE_GapConnParams *connParams)
{
  int ret      = BLE_SUCCESS;
  int errCode  = 0;

  if(connParams == NULL)
    {
      return -EINVAL;
    }

  gapMem.connParams = *(ble_gap_conn_params_t *)connParams;
  errCode = sd_ble_gap_conn_param_update(connHandle, &gapMem.connParams);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

int BLE_GapCancelConnecting(void)
{
  int ret      = BLE_SUCCESS;
  int errCode  = 0;

  errCode = sd_ble_gap_connect_cancel();
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

int BLE_GapDisconnectLink(BLE_GapConnHandle connHandle)
{
  int errCode = sd_ble_gap_disconnect((uint16_t)connHandle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
  return bleConvertErrorCode((uint32_t)errCode);
}

int BLE_GapAuthenticate(BLE_GapConnHandle connHandle, BLE_GapPairingFeature *pairingFeature)
{
  int ret      = BLE_SUCCESS;
  int errCode  = 0;
  ble_gap_sec_params_t secParams = {0};

  if(pairingFeature == NULL) {
    return -EINVAL;
  }

  secParams.oob          = pairingFeature->oob;
  secParams.io_caps      = pairingFeature->ioCap;
  secParams.max_key_size = pairingFeature->maxKeySize;
  secParams.min_key_size = pairingFeature->minKeySize;
  secParams.mitm         = (pairingFeature->authReq & BLE_GAP_AUTH_MITM) >> 1;
  secParams.bond         = (pairingFeature->authReq & BLE_GAP_AUTH_BOND) >> 0;
  secParams.kdist_peer.enc  = 1;
  secParams.kdist_peer.id   = 1;
  errCode = sd_ble_gap_authenticate(connHandle, &secParams);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

int BLE_GapReplyPairingFeature(BLE_GapConnHandle connHandle)
{
  int ret      = BLE_SUCCESS;
  int errCode  = 0;
  ble_gap_sec_keyset_t secKeyset = {{0}};

  secKeyset.keys_peer.p_enc_key   = &gapMem.wrapperBondInfo.peerEncKey;
  secKeyset.keys_peer.p_id_key    = &gapMem.wrapperBondInfo.peerIdKey;

  errCode = sd_ble_gap_sec_params_reply(connHandle, BLE_GAP_SEC_STATUS_SUCCESS, NULL, &secKeyset);
  ret = bleConvertErrorCode((uint32_t)errCode);
  memcpy(&gapMem.keySet, &secKeyset, sizeof(ble_gap_sec_keyset_t));
  return ret;
}


int BLE_GapReplyAuthKey(BLE_GapAuthKey *authKey)
{
  int ret      = BLE_SUCCESS;
  int errCode  = 0;

  if((authKey == NULL) || (authKey->key == NULL)) {
    return -EINVAL;
  }

  errCode = sd_ble_gap_auth_key_reply(authKey->handle, authKey->keyType, authKey->key);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

int BLE_GapSaveBondInfo(BLE_GapBondInfo *info)
{
  int ret = 0;
  int index;
  BLE_GapBondInfo checkInfo={0};
  
  if (info == NULL)
    {
      return -EINVAL;
    }

  if ((gapMem.wrapperBondInfo.bondInfo.addrType != info->addrType) ||
    memcmp(gapMem.wrapperBondInfo.bondInfo.addr, info->addr, BLE_GAP_ADDR_LENGTH) ||
    !memcmp(checkInfo.addr, info->addr, BLE_GAP_ADDR_LENGTH))
    {
      ret = -EINVAL;
      BLE_PRT("BLE_GapSaveBondInfo: Not finished Authenticate %d\n", ret);
      return ret;
    }
  uint32_t list = bleBondEnableList;
  if (bleBondEnableList == 0xFF)
    {
      ret = -ENOMEM;
      BLE_PRT("BLE_GapSaveBondInfo: BondInfo full %d\n", ret);
      return ret;
    }
  if(bleBondEnableList) {
    for(index = 0; index < BLE_SAVE_BOND_DEVICE_MAX_NUM; index++, list >>= 1) {
      if (!(list & 1)) {
        continue;
      }
      if(!memcmp(BondInfoInFlash[index].bondInfo.addr, info->addr, BLE_GAP_ADDR_LENGTH)) {
        BLE_PRT("BLE_GapSaveBondInfo: Exsiting addr\n");
        BondInfoInFlash[index] = gapMem.wrapperBondInfo;
        goto save;
      }
    }
  }
  list = bleBondEnableList;
  for(index = 0; index < BLE_SAVE_BOND_DEVICE_MAX_NUM; index++, list >>= 1) {
    if (list & 1) {
      continue;
    }
    BondInfoInFlash[index] = gapMem.wrapperBondInfo;
    bleBondEnableList |= (1 << index);
    break;
  }
  BLE_PRT("BLE_GapSaveBondInfo: bleBondEnableList 0x%lx\n", bleBondEnableList);
  if (index == BLE_SAVE_BOND_DEVICE_MAX_NUM) {
    ret = -ENOMEM;
    BLE_PRT("BLE_GapSaveBondInfo: BondInfo full ?? %d\n", ret);
    return ret;
  }
save:
  ret = BSO_SetRegistryValue(bleKey.info_key[index],
    (const void*)&BondInfoInFlash[index], sizeof(bleGapWrapperBondInfo));
  if(0 != ret) {
    BLE_PRT("BLE_GapSaveBondInfo: Set info_key NG %d size %d\n", ret, sizeof(bleGapWrapperBondInfo));
    return ret;
  }
  ret = BSO_SetRegistryValue(bleKey.enable_key, (const void*)&bleBondEnableList, sizeof(uint32_t));
  if(0 != ret) {
    BLE_PRT("BLE_GapSaveBondInfo: Set enable_key NG %d\n", ret);
    return ret;
  }
  BSO_Sync();
  return 0;
}

int BLE_GapClearBondInfo(BLE_GapBondInfo *info)
{
  int ret = BLE_SUCCESS;
  int index;
  uint32_t list = bleBondEnableList;

  if (info == NULL)
    {
      return -EINVAL;
    }
  if (!bleBondEnableList)
    {
      ret = -ENOENT;
      BLE_PRT("BLE_GapClearBondInfo: BondInfo empty %d\n", ret);
      return ret;
    }

  for(index = 0; index < BLE_SAVE_BOND_DEVICE_MAX_NUM; index++, list >>= 1) {
    if (!(list & 1)) {
      continue;
    }
    if(!memcmp(BondInfoInFlash[index].bondInfo.addr, info->addr, BLE_GAP_ADDR_LENGTH)) {
      bleBondEnableList &= ~(1 << index);
      break;
    }
    BLE_PRT("BLE_GapClearBondInfo: Type:%d Addr[%d]:%02X:%02X:%02X:%02X:%02X:%02X\n",
      BondInfoInFlash[index].bondInfo.addrType,
      index,
      BondInfoInFlash[index].bondInfo.addr[5],
      BondInfoInFlash[index].bondInfo.addr[4],
      BondInfoInFlash[index].bondInfo.addr[3],
      BondInfoInFlash[index].bondInfo.addr[2],
      BondInfoInFlash[index].bondInfo.addr[1],
      BondInfoInFlash[index].bondInfo.addr[0]);
  }
  BLE_PRT("BLE_GapClearBondInfo: new bleBondEnableList 0x%lx\n", bleBondEnableList);
  if (index == BLE_SAVE_BOND_DEVICE_MAX_NUM) {
    ret = -ENOENT;
    BLE_PRT("BLE_GapSaveBondInfo: Not entry %d\n", ret);
    return ret;
  }
  //BLE_PRT("BLE_GapClearBondInfo: enable_key 0x%x\n", bleKey.enable_key);
  ret = BSO_SetRegistryValue(bleKey.enable_key, (const void*)&bleBondEnableList, sizeof(uint32_t));
  if(0 != ret) {
    BLE_PRT("BLE_GapClearBondInfo: Set enable_key index %d NG %d\n", index, ret);
    return ret;
  }
  BSO_Sync();
  return ret;
}

int BLE_GapGetBondInfoIdList(BLE_GapBondInfoList *bondInfo)
{
  uint32_t list = bleBondEnableList;

  if (bondInfo == NULL)
    {
      return -EINVAL;
    }

  bondInfo->bondNum = 0;
  for(int index = 0; index < BLE_SAVE_BOND_DEVICE_MAX_NUM; index++, list >>= 1)
    {
      if (list & 1)
        {
          BLE_PRT("BLE_GapGetBondInfoIdList: exist index %d\n", index);
          memcpy(bondInfo->bondInfoId[bondInfo->bondNum],
          BondInfoInFlash[index].bondInfo.addr, BLE_GAP_ADDR_LENGTH);
          bondInfo->bondNum++;
         }
    }
  return 0;
}

int BLE_GapEncrypt(BLE_GapConnHandle connHandle)
{
  int index;
  uint32_t list = bleBondEnableList;
  int errCode = 0;
  if (gapMem.is_connected) {
    for(index = 0; index < BLE_SAVE_BOND_DEVICE_MAX_NUM; index++, list >>= 1) {
      if (!(list & 1)) {
        continue;
      }
      if (BondInfoInFlash[index].bondInfo.addrType != gapMem.wrapperBondInfo.bondInfo.addrType) {
        continue;
      }
      if(!memcmp(BondInfoInFlash[index].bondInfo.addr,
        gapMem.wrapperBondInfo.bondInfo.addr, BLE_GAP_ADDR_LENGTH)) {
        gapMem.wrapperBondInfo = BondInfoInFlash[index];
        break;
      }
    }
  }
  errCode = sd_ble_gap_encrypt(connHandle,
    &gapMem.wrapperBondInfo.peerEncKey.master_id, &gapMem.wrapperBondInfo.peerEncKey.enc_info);
  return bleConvertErrorCode((uint32_t)errCode);
}

int BLE_GapStartRssi(BLE_GapConnHandle connHandle, uint8_t thresholdDbm, uint8_t skipCount)
{
  int ret     = BLE_SUCCESS;
  int errCode = 0;

  errCode = sd_ble_gap_rssi_start(connHandle, thresholdDbm, skipCount);
  ret = bleConvertErrorCode((uint32_t)errCode);
  if (ret == BLE_SUCCESS) {
    gapMem.startRssi = true;
  }
  return ret;
}

int BLE_GapStopRssi(BLE_GapConnHandle connHandle)
{
  int ret     = BLE_SUCCESS;
  int errCode = 0;

  errCode = sd_ble_gap_rssi_stop(connHandle);
  ret = bleConvertErrorCode((uint32_t)errCode);
  if (ret == BLE_SUCCESS) {
    gapMem.startRssi = false;
  }
  return ret;
}

int BLE_GapGetRssi(BLE_GapConnHandle connHandle, int8_t *rssi)
{
  int ret = BLE_SUCCESS;

  if (rssi == NULL) {
    return -EINVAL;
  }
  if (gapMem.peerRssi == 0) {
    return -EPERM;
  }
  if (gapMem.startRssi == false) {
    return -EINVAL;
  }
  *rssi = gapMem.peerRssi;
  return ret;
}

int BLE_GapPhyUpdate(BLE_GapConnHandle connHandle, BLE_GapPhys *gapPhys)
{
  int ret = 0;

  if(gapPhys == NULL) {
    return -EINVAL;
  }

  ble_gap_phys_t phys;
  phys.tx_phys = gapPhys->txPhy;
  phys.rx_phys = gapPhys->rxPhy;
  ret = sd_ble_gap_phy_update(connHandle, &phys);
  return bleConvertErrorCode((uint32_t)ret);
}

static
int getAddress(BLE_GapAddr *addr)
{
  int errCode = BLE_SUCCESS, ret = BLE_SUCCESS;
  ble_gap_addr_t bleGapAddr = {0};

  if(addr == NULL) {
    return -EINVAL;
  }

  errCode = sd_ble_gap_addr_get(&bleGapAddr);
  ret = bleConvertErrorCode((uint32_t)errCode);
  memcpy(addr->addr, bleGapAddr.addr, BLE_GAP_ADDR_LENGTH);

  return ret;
}

static
int setAddress(BLE_GapAddr *addr)
{
  int errCode = 0, ret = 0;
  ble_gap_addr_t bleGapAddr = {0};
  ble_gap_privacy_params_t privacy = {0};

  if(addr == NULL) {
    return -EINVAL;
  }
  switch( addr->type ) {
  case BLE_GAP_ADDR_TYPE_PUBLIC:
    bleGapAddr.addr_type = addr->type;
    memcpy(bleGapAddr.addr, addr->addr, BLE_GAP_ADDR_LENGTH);
    errCode = sd_ble_gap_addr_set(&bleGapAddr);
    break;
  case BLE_GAP_ADDR_TYPE_RANDOM_STATIC:
    //default
    break;
  case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE:
  case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
    privacy.privacy_mode = BLE_GAP_PRIVACY_MODE_DEVICE_PRIVACY;
    privacy.private_addr_type = addr->type;
    privacy.private_addr_cycle_s = 900; //sec
    privacy.p_device_irk = NULL;
    errCode = sd_ble_gap_privacy_set(&privacy);
    break;
  default:
    return -EINVAL;
  }
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

static
int getName(BLE_GapName *name)
{
  int errCode = BLE_SUCCESS, ret = BLE_SUCCESS;

  if(name == NULL) {
    return -EINVAL;
  }
  errCode = sd_ble_gap_device_name_get(name->name, &name->size);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

static
int setName(BLE_GapName *name)
{
  int errCode = BLE_SUCCESS, ret = BLE_SUCCESS;

  if(name == NULL) {
    return -EINVAL;
  }
  ble_gap_conn_sec_mode_t secMode = {0};
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&secMode);
  errCode = sd_ble_gap_device_name_set(&secMode, name->name, name->size);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}

static
int setCustomBaseuuid(uint8_t uuid[])
{
  int errCode = BLE_SUCCESS, ret = BLE_SUCCESS;
  ble_uuid128_t *sdsBaseuuid = (ble_uuid128_t *)uuid;
  uint8_t uuidType;

  if(uuid == NULL) {
    return -EINVAL;
  }
  BLE_PRT("sd_ble_uuid_vs_add\n");
  errCode = sd_ble_uuid_vs_add(sdsBaseuuid, &uuidType);
  ret = bleConvertErrorCode((uint32_t)errCode);
  return ret;
}
