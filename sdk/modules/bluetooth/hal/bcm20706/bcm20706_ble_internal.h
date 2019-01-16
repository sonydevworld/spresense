/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bcm20706_ble_internal.h
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
 * @file       ble_common_internal.h
 * @brief      Sharing common data structures between ble_xxx.c internally.
 * @attention
 */
#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_BCM20706_BLE_INTERNAL_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_BCM20706_BLE_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <ble/ble_comm.h>
#include <ble/ble_gap.h>
#include <ble/ble_gatts.h>
#include <ble/ble_gattc.h>
#include <bt/bt_comm.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLE_FAILED   -1
#define BOND_SIZE_KEY_NAME       "0000"
#define BOND_INFO_KEY_NAME       "0001"
#define PACKET_LEN               251
#define BOND_DATA_LEN            152

/* The permission bits */
#define LEGATTDB_PERM_NONE                             (0x00)
#define LEGATTDB_PERM_VARIABLE_LENGTH                  (0x1 << 0)
#define LEGATTDB_PERM_READABLE                         (0x1 << 1)
#define LEGATTDB_PERM_WRITE_CMD                        (0x1 << 2)
#define LEGATTDB_PERM_WRITE_REQ                        (0x1 << 3)
#define LEGATTDB_PERM_AUTH_READABLE                    (0x1 << 4)
#define LEGATTDB_PERM_RELIABLE_WRITE                   (0x1 << 5)
#define LEGATTDB_PERM_AUTH_WRITABLE                    (0x1 << 6)

#define BLE_HANDLE_LEN    2
#define BASE_UUID_LEN     16
#define BLE_UUID128_LEN   16
#define MIN_UUID_SERVICE  0x1800
#define MAX_UUID_SERVICE  0x1826
#define MIN_UUID_CHAR     0x2A00
#define MAX_UUID_CHAR     0x2ADA
#define BYTES_OF_PACKET_LEN   2
#define BLE_UUID128_13_BYTE   12
#define BLE_UUID128_14_BYTE   13
#define NV_DATA_LEN           139

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  BLE_DISABLE = 0,
  BLE_ENABLE,
} BLE_BOOL;

typedef enum
{
  SERV_DISCOVER = 0,
  CHAR_DISCOVER,
  DESCRIPTOR_DISCOVER,
  COMPLETE_DISCOVER,
} DISCOVER_STATE;

typedef struct
{
  uint8_t evtData[PACKET_LEN];
} ble_evt_t;

typedef struct
{
  BLE_GapBondInfo bondInfo;
  uint8_t dataLen;
  uint8_t  bondData[BOND_DATA_LEN];
} bleGapWrapperBondInfo;

typedef struct
{
  uint16_t uuid;
  uint8_t type;
} ble_uuid_t;

typedef struct
{
  uint8_t uuid128[BLE_UUID128_LEN]; /* little endian */
} ble_uuid128_t;

typedef struct
{
  uint16_t charHandle;
  uint8_t charProperties;
  uint8_t charPermission;
} ble_char_t;

typedef struct
{
  uint16_t charHandle;
  uint16_t attrLen;
  uint8_t *attrValue;
} ble_attr_t;

typedef struct
{
  uint16_t len;
  uint8_t *p_value;
} ble_gatts_value_t;

typedef struct
{
  uint8_t status;
  BT_ADDR btAddr;
  uint8_t transport;
} pair_result_t;

typedef struct
{
  BT_ADDR btAddr;
  uint8_t transport;
} passkey_request_t;

typedef struct
{
  uint8_t status;
  BT_ADDR btAddr;
  uint8_t transport;
} encp_change_t;

/**@brief A collection of variables of gap. */
typedef struct
{
  bleGapWrapperBondInfo wrapperBondInfo;
  BT_DID_VendorInfo btDidInfo;
  uint8_t gapAdvData[BLE_GAP_ADV_MAX_SIZE];
  uint32_t sizeKey;
  uint32_t infoKey;
  BLE_EvtConnected peerInfo;
  uint8_t  deviceRole;
  BLE_BOOL scanState;
} bleGapMem;

typedef struct
{
  uint8_t                 currCharInd;
  uint8_t                 currSrvInd;
  int                     discoveryInProgress;
  uint8_t                 reserve;
  DISCOVER_STATE          discoverState;
  BLE_GattcOverRunState   state;
  BLE_GattcDbDiscovery    dbDiscovery;
} bleGattcDb;

/**@brief A collection of variables of common. */
 typedef struct
{
  uint32_t                bondInfoId;
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
  BLE_EvtGattsWrite       gattsWriteData;
  BLE_EvtGattsIndConfirm  gattsIndConfirmData;
  BLE_EvtGattcRead        gattcReadData;
  BLE_EvtGattcNtfInd      gattcNtfIndData;
  BLE_EvtGattcDbDiscovery gattcDbDiscoveryData;
  bleGattcDb              gattcDb;
  bleGapMem               *gapMem;
  uint8_t                 stackInited;
} bleCommMem;

/****************************************************************************
 * Public Functions prototype
 ****************************************************************************/

int bleGapAdv(BLE_BOOL bleBool);
int bleGapScan(BLE_BOOL bleBool);
int bleGattsUpdateAttrValue(uint16_t connHandle, uint16_t attrHandle,
    ble_gatts_value_t *gattsValue);
void bleRecvAuthStatus(ble_evt_t *pBleBcmEvt);
void bleRecvNvramData(ble_evt_t *pBleBcmEvt, uint16_t len);
int bleSetAdvData(uint8_t *advData, uint8_t size);
int bleGapReplySecurity(BT_ADDR bleAddr,  uint8_t pairEnable,
		BLE_GapPairingFeature pairingFeature);
int bleCharDiscover(uint16_t connHandle, uint16_t startHandle, uint16_t endHandle);
int bleDescriptorsDiscover(uint16_t connHandle, uint16_t startHandle, uint16_t endHandle);
void bleRecvGattServiceDiscovered(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt, uint16_t len);
void bleRecvGattCharDiscovered(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt, uint16_t len);
void bleRecvGattDescriptorDiscovered(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt, uint16_t len);
void bleRecvGattCompleteDiscovered(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt);

#endif  /* __MODULES_BLUETOOTH_HAL_BCM20706_BCM20706_BLE_INTERNAL_H */

