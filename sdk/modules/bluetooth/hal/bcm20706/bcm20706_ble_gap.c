/****************************************************************************
 * modules/bluetooth/hal/bcm20706/manager/bcm20706_ble_gap.c
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

/******************************************************************************
 * Include
 *****************************************************************************/

#include <nuttx/config.h>
#include <string.h>
#include <ble/ble_comm.h>
#include <ble/ble_gap.h>
#include <bt/bt_comm.h>

#include "manager/bt_storage_manager.h"
#include "manager/bt_uart_manager.h"
#include "bcm20706_ble_internal.h"
#include "bcm20706_bt_internal.h"
#include "bt_util.h"
#include "bt_debug.h"

/******************************************************************************
 * externs
 *****************************************************************************/

extern int bleConvertErrorCode(uint32_t errCode);
extern int btSetBtAddress(BT_ADDR *addr);
extern int btSetPairingEnable(uint8_t isEnable);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADV_DATA_TYPE_FLAGS_SIZE                    0x02
#define ADV_DATA_TYPE_TX_POWER_SIZE                 0x02
#define ADV_DATA_TYPE_COMPLETE_32_UUIDS_SIZE        0x05
#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))

#define ADV_DEFAULT_INTERVAL            50
#define ADV_DEFAULT_TIMEOUT             180

#define SCAN_INTERVAL                   0x0200
#define SCAN_WINDOW                     0x0020
#define SCAN_TIMEOUT                    60
#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)
#define SLAVE_LATENCY                   0
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)
#define FLASH_DEFAULT_VALUE             0xA5A5A5
/* Flash handle key name size */
#define FLASH_KEY_NAME_SIZE      4
#define FLASH_KEY_NAME_2         2
#define FLASH_KEY_NAME_3         3
#define BLE_GAP_ADDR_LENGTH_4    4
#define BLE_GAP_ADDR_LENGTH_5    5
#define BLE_KEY_MASK             0x7F

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum
{
  UINT_0_625_MS = 625,
  UNIT_1_25_MS = 1250,
  UNIT_10_MS = 10000,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

bleGapMem *bleGetGapMem(void);
uint32_t generateSaveHandle(BLE_GapBondInfo *info);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bleGapMem gapMem =
{
  .sizeKey = 0,
  .infoKey = 0,
};

uint8_t sdsBaseuuid[BASE_UUID_LEN] = {0xfb, 0x34, 0x9b, 0x5f,  \
                                      0x80, 0x00, 0x00, 0x80,  \
                                      0x00, 0x10, 0x00, 0x00,  \
                                      0x00, 0x00, 0x00, 0x00};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bleGapMem *bleGetGapMem(void)
{
  return &gapMem;
}

void generateKey(void)
{
  if((0 == gapMem.sizeKey) || (0 == gapMem.infoKey))
    {
      gapMem.sizeKey = BSO_GenerateRegistryKey(BOND_SIZE_KEY_NAME,0);
      gapMem.infoKey = BSO_GenerateRegistryKey(BOND_INFO_KEY_NAME,0);
    }
}

uint32_t generateSaveHandle(BLE_GapBondInfo *info)
{
  uint32_t index = 0;
  uint8_t keyName[FLASH_KEY_NAME_SIZE] = {0};
  memcpy(keyName, info->addr, FLASH_KEY_NAME_SIZE);
  keyName[FLASH_KEY_NAME_2] |= info->addr[BLE_GAP_ADDR_LENGTH_4];
  keyName[FLASH_KEY_NAME_3] |= info->addr[BLE_GAP_ADDR_LENGTH_5];
  for (index = 0; index < FLASH_KEY_NAME_SIZE; index++)
    {
      keyName[index] &= BLE_KEY_MASK;
    }
  return BSO_GenerateRegistryKey((const char* )keyName,0);
}

int bleGattsSetBleNamePermission(BLE_SEC_MODE pWritePerm)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_SET_BLE_NAME_PERMISSION);
  UINT16_TO_STREAM(p, sizeof(BLE_SEC_MODE));
  UINT8_TO_STREAM(p, pWritePerm);
  return btUartSendData(buff, p - buff);
}

int bleGattsSetServiceChangedEnable(void)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_GATT_COMMAND_ENABLE_SERVICE_CHANGED);
  UINT16_TO_STREAM(p, 0);
  btUartSendData(buff, p - buff);
  return 0;
}

int bleSetAdvData(uint8_t* advData, uint8_t size)
{
  uint8_t buff[BT_MID_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_LE_COMMAND_SET_RAW_ADVERTISE_DATA);
  UINT16_TO_STREAM(p, size);
  memcpy(p, advData, size);
  p += size;
  btdbg("ble send adv data,size = %d\n", size);
  return btUartSendData(buff, p - buff);
}

int bleGapSetConnectPara(uint16_t connHandle, BLE_GapConnParams *connParams)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_LE_COMMAND_SET_CONN_PARAMS);
  UINT16_TO_STREAM(p, 10);
  UINT16_TO_STREAM(p, connHandle);
  UINT16_TO_STREAM(p, connParams->minConnInterval);
  UINT16_TO_STREAM(p, connParams->maxConnInterval);
  UINT16_TO_STREAM(p, connParams->slaveLatency);
  UINT16_TO_STREAM(p, connParams->connSupTimeout);
  return btUartSendData(buff, p - buff);
}

int bleGapConnect(BLE_GapAddr *bleGapAddr)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_LE_COMMAND_CONNECT);
  UINT16_TO_STREAM(p, BT_ADDR_LEN + 1);
  UINT8_TO_STREAM(p, 1);
  memcpy(p, bleGapAddr->addr, BT_ADDR_LEN);
  p += BT_ADDR_LEN;
  return btUartSendData(buff, p - buff);
}

int bleGapCancelConnecting(void)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_LE_COMMAND_CANCEL_CONNECT);
  UINT16_TO_STREAM(p, 0);
  return btUartSendData(buff, p - buff);
}

int bleGapDisconnect(BLE_GapConnHandle connHandle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_LE_COMMAND_DISCONNECT);
  UINT16_TO_STREAM(p, 2);
  UINT16_TO_STREAM(p, connHandle);
  return btUartSendData(buff, p - buff);
}

int bleGapSetPinCode(uint32_t pinCode)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_SET_PINCODE);
  UINT16_TO_STREAM(p, 4);
  UINT32_TO_STREAM(p, pinCode);
  return btUartSendData(buff, p - buff);
}

int bleGapReplyPairingFeature(BT_ADDR bleAddr, uint8_t pairEnable, BLE_GapPairingFeature pairingFeature)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  uint8_t *a = bleAddr.address;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_REPLY_FEATURE);
  UINT16_TO_STREAM(p, BT_ADDR_LEN + 6);
  STREAM_TO_BDADDR(p, a);
  p += BT_ADDR_LEN;
  UINT8_TO_STREAM(p, pairEnable);
  UINT8_TO_STREAM(p, pairingFeature.ioCap);
  UINT8_TO_STREAM(p, pairingFeature.oob);
  UINT8_TO_STREAM(p, pairingFeature.authReq);
  UINT8_TO_STREAM(p, pairingFeature.minKeySize);
  UINT8_TO_STREAM(p, pairingFeature.maxKeySize);
  return btUartSendData(buff, p - buff);
}

int bleGapReplySecurity(BT_ADDR bleAddr, uint8_t pairEnable, BLE_GapPairingFeature pairingFeature)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  uint8_t *a = bleAddr.address;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_REPLY_SECURITY);
  UINT16_TO_STREAM(p, BT_ADDR_LEN + 6);
  STREAM_TO_BDADDR(p, a);
  p += BT_ADDR_LEN;
  UINT8_TO_STREAM(p, pairEnable);
  UINT8_TO_STREAM(p, pairingFeature.ioCap);
  UINT8_TO_STREAM(p, pairingFeature.oob);
  UINT8_TO_STREAM(p, pairingFeature.authReq);
  UINT8_TO_STREAM(p, pairingFeature.minKeySize);
  UINT8_TO_STREAM(p, pairingFeature.maxKeySize);
  return btUartSendData(buff, p - buff);
}

int blePushNvData(bleGapWrapperBondInfo *bleBondInfo)
{
  uint8_t buff[BT_LONG_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_PUSH_NVRAM_DATA);
  UINT16_TO_STREAM(p, bleBondInfo->dataLen);
  memcpy(p, bleBondInfo->bondData, bleBondInfo->dataLen);
  btdbg("nvId = %d, addr = %x, %x, %x, %x, %x, %x\n", p[0]|(p[1]<<8), p[2],
      p[3], p[4],p[5],p[6],p[7]);
  btdbg("nv data len = %d\n", bleBondInfo->dataLen);
  p += bleBondInfo->dataLen;
  return btUartSendData(buff, p - buff);
}

int bleDeleteNvData(uint16_t nvDataId)
{
  uint8_t buff[BT_LONG_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_DELETE_NVRAM_DATA);
  UINT16_TO_STREAM(p, sizeof(nvDataId));
  UINT16_TO_STREAM(p, nvDataId);
  return btUartSendData(buff, p - buff);
}

int BLE_GapSetAdvData(BLE_GapAdvData *advData)
{
#define MIN_TX_POWER  -127
#define MAX_TX_POWER  127
  uint8_t index = 0;
  uint8_t size = 0;
  uint8_t advLen = 0;
  uint8_t *data = NULL;

  if(advData == NULL)
    {
      btdbg("advData = NULL\n");
      return -EINVAL;
    }
  if(advData->flags != 0)
    {
      gapMem.gapAdvData[index++] = ADV_DATA_TYPE_FLAGS_SIZE;
      gapMem.gapAdvData[index++] = ADV_DATA_TYPE_FLAGS;
      gapMem.gapAdvData[index++] = advData->flags;
    }
  if (advData->txPower < MAX_TX_POWER && advData->txPower > MIN_TX_POWER)
    {
      gapMem.gapAdvData[index++] = ADV_DATA_TYPE_TX_POWER_SIZE;
      gapMem.gapAdvData[index++] = ADV_DATA_TYPE_TX_POWER;
      gapMem.gapAdvData[index++] = (uint8_t)advData->txPower;
    }
  if(advData->complete32Uuid != 0)
    {
      gapMem.gapAdvData[index++] = ADV_DATA_TYPE_COMPLETE_32_UUIDS_SIZE;
      gapMem.gapAdvData[index++] = ADV_DATA_TYPE_COMPLETE_32_UUIDS;
      gapMem.gapAdvData[index++] = (uint8_t)(advData->complete32Uuid >> 0);
      gapMem.gapAdvData[index++] = (uint8_t)(advData->complete32Uuid >> 8);
      gapMem.gapAdvData[index++] = (uint8_t)(advData->complete32Uuid >> 16);
      gapMem.gapAdvData[index++] = (uint8_t)(advData->complete32Uuid >> 24);
    }
  advLen = advData->completeLocalName.advLength;
  btdbg("advlen = %d.\n", advLen);
  data = advData->completeLocalName.advData;
  if((advLen != 0)&&(data != NULL))
    {
      gapMem.gapAdvData[index++] = advLen + 1;
      gapMem.gapAdvData[index++] = ADV_DATA_TYPE_COMPLETE_LOCAL_NAME;
      memcpy(&gapMem.gapAdvData[index], data, advLen);
      index += advLen;
    }
  advLen = advData->manufacturerSpecificData.advLength;
  btdbg("advlen = %d.\n", advLen);
  data = advData->manufacturerSpecificData.advData;
  if((advLen != 0)&&(data != NULL))
    {
      gapMem.gapAdvData[index++] = advLen + 1;
      gapMem.gapAdvData[index++] = ADV_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA;
      memcpy(&gapMem.gapAdvData[index], data, advLen);
      index += advLen;
    }
  if ((advData->serviceDataCount != 0) && (advData->serviceData != NULL))
    {
      int i;

      for (i = 0; i < advData->serviceDataCount; i++)
        {
          BLE_GapAdvStructure *pServiceData = &(advData->serviceData[i]);

          advLen = pServiceData->advLength;
          data = pServiceData->advData;
          if ((advLen != 0) && (data != NULL))
            {
              gapMem.gapAdvData[index++] = advLen + 1;
              gapMem.gapAdvData[index++] = ADV_DATA_TYPE_SERVICE_DATA;
              memcpy(&gapMem.gapAdvData[index], data, advLen);
              index += advLen;
            }
        }
    }
  size = index + 1;
  btdbg("adv size = %d.\n", size);
  if(size > BLE_GAP_ADV_MAX_SIZE)
    {
      btdbg("size > BLE_GAP_ADV_MAX_SIZE\n");
      return -EINVAL;
    }

  return bleSetAdvData(gapMem.gapAdvData, size);
}

int BLE_GapStartAdv(void)
{
  return bleGapAdv(BLE_ENABLE);
}

int BLE_GapStopAdv(void)
{
  return bleGapAdv(BLE_DISABLE);
}

int BLE_GapSetSecParam(BLE_GapSecCfg *param)
{
  int ret    = BLE_SUCCESS;
  uint8_t *p = NULL;
  uint32_t pinCode = 0;

  if((param == NULL) || (param->data == NULL))
    {
      return -EINVAL;
    }
  switch(param->type)
    {
      case SEC_CFG_PASSKEY:
        {
          p = (uint8_t *)param->data;
          pinCode += p[5] - '0';
          pinCode += (p[4] - '0') * 10;
          pinCode += (p[3] - '0') * 100;
          pinCode += (p[2] - '0') * 1000;
          pinCode += (p[1] - '0') * 10000;
          pinCode += (p[0] - '0') * 100000;
          ret = bleGapSetPinCode(pinCode);
          break;
        }

      default:
        ret = -EINVAL;
        break;
    }
  return ret;
}

int BLE_GapStartScan(void)
{
  gapMem.scanState = BLE_ENABLE;
  return bleGapScan(BLE_ENABLE);
}

int BLE_GapStopScan(void)
{
  gapMem.scanState = BLE_DISABLE;
  return bleGapScan(BLE_DISABLE);
}

int BLE_GapConnect(BLE_GapAddr *addr)
{
  int ret = 0;

  if (!addr)
    {
      return -EINVAL;
    }
  ret = btSetPairingEnable(BLE_DISABLE);
  if (ret != BLE_SUCCESS)
    {
      return ret;
    }
  return bleGapConnect(addr);
}

int btReplyPasskey(BT_REPLY_PASSKEY replyPasskey)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_REPLY_PASSKEY);
  UINT16_TO_STREAM(p, 11);
  memcpy(p,&replyPasskey.addr, BT_ADDR_LEN);
  p += BT_ADDR_LEN;
  UINT8_TO_STREAM(p, replyPasskey.btAccept);
  UINT32_TO_STREAM(p, replyPasskey.passKey);
  return btUartSendData(buff, p - buff);
}


int BLE_GapCancelConnecting(void)
{
  return bleGapCancelConnecting();
}

int BLE_GapDisconnectLink(BLE_GapConnHandle connHandle)
{
  return bleGapDisconnect(connHandle);
}

int BLE_GapSaveBondInfo(BLE_GapBondInfo *info)
{
  int ret = BLE_SUCCESS;
  int size = 0;
  int index = 0;
  uint32_t saveHandle = 0;
  BLE_GapBondInfoList bondInfo = {0};
  uint16_t bondInfoSize = 0;

  bondInfoSize = (BLE_GAP_ADDR_LENGTH + BT_DID_INFO_LEN) * BT_SAVE_BOND_DEVICE_MAX_NUM;
  generateKey();
  size = sizeof(bondInfo.bondNum);
  ret = BSO_GetRegistryValue(gapMem.sizeKey, (void *)&bondInfo.bondNum, size);
  btdbg("bondNum = %ld\n", bondInfo.bondNum);
  if(0 != ret)
    {
      if (-ENOENT == ret)
        {
          ret = BSO_SetRegistryValue(gapMem.sizeKey, (void *)&bondInfo.bondNum, size);
          if(0 != ret)
            {
              goto exit_bond_failure;
            }
        }
      else
        {
          goto exit_bond_failure;
        }
    }
  if(FLASH_DEFAULT_VALUE == bondInfo.bondNum)
    {
      bondInfo.bondNum = 0;
    }
  if(BLE_SAVE_BOND_DEVICE_MAX_NUM <= bondInfo.bondNum)
    {
      ret = ENOMEM;
      goto exit_bond_failure;
    }
  if(0 != bondInfo.bondNum)
    {
      ret = BSO_GetRegistryValue(gapMem.infoKey, (void *)bondInfo.bondInfoId, bondInfoSize);
      if(0 != ret)
        {
          if (-ENOENT == ret)
            {
              ret = 0;
            }
          else
            {
              goto exit_bond_failure;
            }
        }
      for(index = 0; index < bondInfo.bondNum; index++)
        {
          if(!memcmp(bondInfo.bondInfoId[index], info->addr, BLE_GAP_ADDR_LENGTH))
            {
              goto start_bonding;
            }
        }
    }
  memcpy(bondInfo.bondInfoId[bondInfo.bondNum], info->addr, BLE_GAP_ADDR_LENGTH);
  memcpy(bondInfo.didInfo[bondInfo.bondNum], &gapMem.btDidInfo, BT_DID_INFO_LEN);
  bondInfo.bondNum++;
  ret = BSO_SetRegistryValue(gapMem.infoKey, (const void*)bondInfo.bondInfoId, bondInfoSize);
  if(0 != ret)
    {
      goto exit_bond_failure;
    }
  btdbg("bond infomation success\n");
  size = sizeof(bondInfo.bondNum);
  ret = BSO_SetRegistryValue(gapMem.sizeKey, (const void*)&bondInfo.bondNum, size);
  if(0 != ret)
    {
      goto exit_bond_failure;
    }
  btdbg("bond num success\n");
start_bonding:
  saveHandle = generateSaveHandle(info);
  ret = BSO_SetRegistryValue(saveHandle, (const void*)&gapMem.wrapperBondInfo, sizeof(bleGapWrapperBondInfo));
  if(0 != ret)
    {
      goto exit_bond_failure;
    }
  btdbg("bond complete information success\n");
exit_bond_failure:
  return ret;
}

int BLE_GapClearBondInfo(BLE_GapBondInfo *info)
{
  int ret = BLE_SUCCESS;
  int size = 0;
  int index = 0;
  uint32_t saveHandle = 0;
  BLE_GapBondInfoList bondInfo = {0};
  uint16_t nvDataId;
  uint16_t bondInfoSize = 0;

  bondInfoSize = (BLE_GAP_ADDR_LENGTH + BT_DID_INFO_LEN) * BT_SAVE_BOND_DEVICE_MAX_NUM;
  generateKey();
  size = sizeof(bondInfo.bondNum);
  ret = BSO_GetRegistryValue(gapMem.sizeKey, (void *)&bondInfo.bondNum, size);
  if(0 != ret)
    {
      goto exit_clear_failure;
    }
  if(0 == bondInfo.bondNum)
    {
      ret = -ENOENT;
      goto exit_clear_failure;
    }
  btdbg("bond num = %ld\n", bondInfo.bondNum);
  ret = BSO_GetRegistryValue(gapMem.infoKey, (void *)bondInfo.bondInfoId, bondInfoSize);
  if(0 != ret)
    {
      goto exit_clear_failure;
    }
  for(index = 0; index < bondInfo.bondNum; index++)
    {
      if(!memcmp(bondInfo.bondInfoId[index], info->addr, BLE_GAP_ADDR_LENGTH))
        {
          btdbg("Find bond address\n");
          goto start_clear_bond_info;
        }
    }
  ret = -ENOENT;
  goto exit_clear_failure;
start_clear_bond_info:
  saveHandle = generateSaveHandle(info);
  memset(&gapMem.wrapperBondInfo, 0x00, sizeof(bleGapWrapperBondInfo));
  ret = BSO_GetRegistryValue(saveHandle, (void*)&gapMem.wrapperBondInfo, sizeof(bleGapWrapperBondInfo));
  if(0 != ret)
    {
      goto exit_clear_failure;
    }
  memcpy(&nvDataId, gapMem.wrapperBondInfo.bondData, sizeof(nvDataId));
  ret = bleDeleteNvData(nvDataId);
  if(0 != ret)
    {
      goto exit_clear_failure;
    }
  ret = BSO_DeleteRegistryKey(saveHandle);
  if(0 != ret)
    {
      goto exit_clear_failure;
    }
  while(index < bondInfo.bondNum - 1)
    {
      memcpy(bondInfo.bondInfoId[index], bondInfo.bondInfoId[index + 1], BLE_GAP_ADDR_LENGTH);
      memcpy(bondInfo.didInfo[index], bondInfo.didInfo[index + 1], BT_DID_INFO_LEN);
      index++;
    }
  ret = BSO_SetRegistryValue(gapMem.infoKey, (const void*)bondInfo.bondInfoId, bondInfoSize);
  if(0 != ret)
    {
      goto exit_clear_failure;
    }
  bondInfo.bondNum--;
  size = sizeof(bondInfo.bondNum);
  ret = BSO_SetRegistryValue(gapMem.sizeKey, (const void*)&bondInfo.bondNum, size);
  if(0 != ret)
    {
      goto exit_clear_failure;
    }
exit_clear_failure:
  return ret;
}

int BLE_GapGetBondInfoIdList(BLE_GapBondInfoList *bondInfo)
{
  int ret     = 0;
  int size    = 0;
  int index   = 0;
  int bondNum = 0;
  BLE_GapBondInfoList infoList = {0};
  uint16_t bondInfoSize = 0;

  bondInfoSize = (BLE_GAP_ADDR_LENGTH + BT_DID_INFO_LEN) * BT_SAVE_BOND_DEVICE_MAX_NUM;
  if (bondInfo == NULL)
    {
      return -EINVAL;
    }
  memset(bondInfo, 0, sizeof(BLE_GapBondInfoList));
  generateKey();
  size = sizeof(bondInfo->bondNum);
  ret = BSO_GetRegistryValue(gapMem.sizeKey, (void *)&bondInfo->bondNum, size);
  if(0 != ret)
    {
      if (-ENOENT == ret)
        {
          bondInfo->bondNum = 0;
          ret = 0;
          return ret;
        }
      else
        {
          return -ENOENT;
        }
    }
  if (bondInfo->bondNum == 0)
    {
      return 0;
    }
  if(FLASH_DEFAULT_VALUE == bondInfo->bondNum)
    {
      bondInfo->bondNum = 0;
      return 0;
    }
  if(BLE_SAVE_BOND_DEVICE_MAX_NUM < bondInfo->bondNum)
    {
      bondInfo->bondNum = BLE_SAVE_BOND_DEVICE_MAX_NUM;
    }
  infoList.bondNum = bondInfo->bondNum;
  ret = BSO_GetRegistryValue(gapMem.infoKey, (void *)infoList.bondInfoId, bondInfoSize);
  if(0 != ret)
    {
      bondInfo->bondNum = 0;
      BSO_SetRegistryValue(gapMem.sizeKey, (const void*)&bondInfo->bondNum, sizeof(bondInfo->bondNum));
      return ret;
    }
  memcpy(bondInfo, &infoList, sizeof(BLE_GapBondInfoList));

  bondNum = bondInfo->bondNum;
  for(index = 0; index < bondNum; index++)
    {
      memcpy(bondInfo->bondInfoId[index], infoList.bondInfoId[bondNum - 1 - index], BLE_GAP_ADDR_LENGTH);
      memcpy(bondInfo->didInfo[index], infoList.didInfo[bondNum - 1 - index], BLE_DID_INFO_LEN);
    }
  return ret;
}

int BLE_GapEncrypt(BLE_GapConnHandle connHandle)
{
  int ret                 = BLE_SUCCESS;
  return ret;
}

int BLE_GapUpdateConnectionParams(BLE_GapConnHandle connHandle, BLE_GapConnParams *connParams)
{
  if (!connParams)
    {
      return -EINVAL;
    }
   return bleGapSetConnectPara(connHandle, connParams);
}
