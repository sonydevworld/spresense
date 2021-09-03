/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bcm20706_bt_common.c
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

#include <unistd.h>
#include <arch/board/board.h>
#include <bt/bt_comm.h>
#include <ble/ble_comm.h>
#include <ble/ble_gap.h>

#include "bt_debug.h"
#include "bt_util.h"
#include "bcm20706_bt_internal.h"
#include "bcm20706_ble_internal.h"
#include "manager/bt_hci_manager.h"
#include "manager/bt_storage_manager.h"
#include "manager/bt_uart_manager.h"

#include <arch/chip/pm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DBG_LOG_DEBUG btdbg
#define DBG_LOG_ERROR btdbg

#define BTM_DEFAULT_DISC_WINDOW       0x0012
#define BTM_DEFAULT_DISC_INTERVAL     0x0800
#define BTM_DEFAULT_CONN_WINDOW       0x0012
#define BTM_DEFAULT_CONN_INTERVAL     0x0800

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

extern bleGapMem *bleGetGapMem(void);
extern uint32_t generateSaveHandle(BLE_GapBondInfo *info);
extern void generateKey(void);
extern int btRecvTaskEntry(void);
extern int btRecvTaskEnd(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Workaround for BT Hot Sleep Issue. After it is resolved, wakelock will be removed */
static struct pm_cpu_wakelock_s g_wake_lock =
{
  .count = 0,
  .info = PM_CPUWAKELOCK_TAG('B', 'T', 0),
};

struct bt_common_context_s bt_common_context = {};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int btSetVisibility(BT_VISIBILITY isVisible)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  uint8_t tConnect = (isVisible & 0x02) >> 1;
  uint8_t tDiscovery = isVisible & 0x01;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_SET_VISIBILITY);
  UINT16_TO_STREAM(p, 2);
  UINT8_TO_STREAM(p, tDiscovery);
  UINT8_TO_STREAM(p, tConnect);
  return btUartSendData(buff, p - buff);
}

static int btSetVisibilityParam(BT_VISIBILITY_PARAM *btVisilityParam)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_SET_VISIBILITY_PARAM);
  UINT16_TO_STREAM(p, sizeof(BT_VISIBILITY_PARAM));
  UINT16_TO_STREAM(p, btVisilityParam->discWindow);
  UINT16_TO_STREAM(p, btVisilityParam->discInterval);
  UINT16_TO_STREAM(p, btVisilityParam->connWindow);
  UINT16_TO_STREAM(p, btVisilityParam->connInterval);
  return btUartSendData(buff, p - buff);
}

static int btSetParingMode(BT_IO_CAP ioCapability)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_SET_PAIR_MODE);
  UINT16_TO_STREAM(p, 1);
  UINT8_TO_STREAM(p, ioCapability);
  return btUartSendData(buff, p - buff);
}

static int btStartInquiry(void)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_INQUIRY);
  UINT16_TO_STREAM(p, 1);
  UINT8_TO_STREAM(p, 1);
  return btUartSendData(buff, p - buff);
}

static int btCancelInquiry(void)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_INQUIRY);
  UINT16_TO_STREAM(p, 1);
  UINT8_TO_STREAM(p, 0);
  return btUartSendData(buff, p - buff);
}

static int btStopBond(BT_ADDR *addr)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_UNBOND);
  UINT16_TO_STREAM(p, BT_ADDR_LEN);
  memcpy(p, addr, BT_ADDR_LEN);
  p += BT_ADDR_LEN;
  return btUartSendData(buff, p - buff);
}

static int btPushNvData(bleGapWrapperBondInfo *bleBondInfo)
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

static int btPushNvAllBondInfo(void)
{
  int ret                 = BT_SUCCESS;
  uint32_t saveHandle     = 0;
  BLE_GapBondInfo info    = {0};
  bleGapMem *btGapMem    = bleGetGapMem();
  int size = 0;
  int index = 0;
  BLE_GapBondInfoList bondInfo = {0};

  generateKey();
  size = sizeof(bondInfo.bondNum);
  ret = BSO_GetRegistryValue(btGapMem->sizeKey, (void *)&bondInfo.bondNum, size);
  btdbg("bondNum = %ld\n", bondInfo.bondNum);
  if(0 != ret)
    {
      if (-ENOENT == ret)
        {
          bondInfo.bondNum = 0;
          ret = 0;
        }
      else
        {
          btdbg("get sizekey failed\n");
          return -ENOENT;
        }
    }

  if(0 != bondInfo.bondNum)
    {
      size = BLE_GAP_ADDR_LENGTH * (bondInfo.bondNum);
      ret = BSO_GetRegistryValue(btGapMem->infoKey, (void *)bondInfo.bondInfoId, size);
      if(0 != ret)
        {
          btdbg("get infokey failed\n");
          return -ENOENT;
        }
      for(index = 0; index < bondInfo.bondNum; index++)
        {
          memcpy(info.addr, bondInfo.bondInfoId[index], BT_ADDR_LEN);
          saveHandle = generateSaveHandle(&info);
          memset(&btGapMem->wrapperBondInfo,0x00,sizeof(bleGapWrapperBondInfo));
          ret = BSO_GetRegistryValue(saveHandle, (void*)&btGapMem->wrapperBondInfo, sizeof(bleGapWrapperBondInfo));
          if(0 != ret)
            {

              /* If the return value is not 0, the file of bond information is broken,
               * So stop the function and return error.
               */

              return -ENOENT;
            }
          ret = btPushNvData(&btGapMem->wrapperBondInfo);

          if (ret)
            {
              return ret;
            }
        }
    }
  return ret;
}

/* For HAL I/F */

/****************************************************************************
 * Name: bcm20706_bt_init
 *
 * Description:
 *   Bluetooth Initialize
 *   Prepare NV storage etc for non power item initialize.
 *
 ****************************************************************************/

static int bcm20706_bt_init(void)
{
  int ret = BT_SUCCESS;

  ret = BSO_Init(NULL);
  if (ret)
    {
      ret = -ENXIO;
      btdbg("BSO_Init failed\n");
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_finalize
 *
 * Description:
 *   Bluetooth Finalize
 *   Release NV etc.
 *
 ****************************************************************************/

static int bcm20706_bt_finalize(void)
{
  int ret = BT_SUCCESS;

  ret = BSO_Finalize(NULL);
  if (ret)
    {
      ret = -ENXIO;
      btdbg("BSO_Finalize failed\n");
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_enable
 *
 * Description:
 *   Bluetooth Enable
 *   Turn ON/OFF bluetooth and operate UART and Receive thread.
 *
 ****************************************************************************/

static int bcm20706_bt_enable(bool enable)
{
  int ret = BT_SUCCESS;

  if (enable == true)
    {
      /* Workaround for BT Hot Sleep Issue. After it is resolved, wakelock will be removed
       * get wake lock
       */
      up_pm_acquire_wakelock(&g_wake_lock);

      /* Power on BT */
      ret = board_bluetooth_power_control(true);
      ret |= board_bluetooth_pin_cfg();
      if (ret)
        {
          DBG_LOG_DEBUG("board_bluetooth_power_control(on): NG %d\n", ret);
          return -EIO;
        }
      board_bluetooth_reset();
      board_bluetooth_enable_sleep(true);
      DBG_LOG_DEBUG("Power on BT!!\n");

      ret = btUartInitialization();
      if (ret)
        {
          ret = -ENXIO;
          goto err;
        }

      /* UART post pin configuration */
      ret = board_bluetooth_uart_pin_cfg();
      if (ret)
        {
          ret = -ENXIO;
          goto err;
        }

      DBG_LOG_DEBUG("uart init success\n");

      ret = bt_hci_start_boot();
      if (ret)
        {
          DBG_LOG_DEBUG("btHciStartBoot: NG %d\n", ret);
          ret = -ENXIO;
          goto err;
        }
      DBG_LOG_DEBUG("hci boot success\n");

      ret = btRecvTaskEntry();
      if (ret)
        {
          DBG_LOG_DEBUG("btRecvTaskEntry: NG %d\n", ret);
          ret = -ENXIO;
          goto err;
        }

      ret = btPushNvAllBondInfo();
      if (ret)
        {
          DBG_LOG_DEBUG("btPushNvAllBondInfo: NG %d\n", ret);
          if (ret == -ENOENT)
            {
              /* The bond info file is broken, just clean the registry and delete the bond info file */

              BSO_CleanRegistry();
              ret = 0;
            }
          else
            {
              goto release;
            }
        }
      /* TODO: Will set sniff parameter */
    }
  else
    {
      btUartFinalization();
      btRecvTaskEnd();
      board_bluetooth_power_control(false);
      /* Workaround for BT Hot Sleep Issue. After it is resolved, wakelock will be removed */
      up_pm_release_wakelock(&g_wake_lock);
    }

  return ret;

release:
      btRecvTaskEnd();
err:
      board_bluetooth_power_control(false);
  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_set_device_addr
 *
 * Description:
 *   Bluetooth set device address
 *   Set device address to chip and store local address.
 *
 ****************************************************************************/

static int bcm20706_bt_set_device_addr(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;

  /* Store input address to local address */

  memcpy(&bt_common_context.bt_addr, addr, BT_ADDR_LEN);

  /* Send BT Address to chip */

  ret = btSetBtAddress(&bt_common_context.bt_addr);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_get_device_addr
 *
 * Description:
 *   Bluetooth get device address
 *   Get device address from local address.
 *
 ****************************************************************************/

static int bcm20706_bt_get_device_addr(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;

  /* Copy device address from local address */

  memcpy(addr, &bt_common_context.bt_addr, BT_ADDR_LEN);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_set_device_name
 *
 * Description:
 *   Bluetooth set device name
 *   Set device name to chip and store local name.
 *
 ****************************************************************************/

static int bcm20706_bt_set_device_name(char *name)
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

  strncpy(bt_common_context.bt_name, name, nameSize);

  /* Send device name to chip */

  ret = btSetBtName(bt_common_context.bt_name);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_get_device_name
 *
 * Description:
 *   Bluetooth get device name
 *   Get device name from local name.
 *
 ****************************************************************************/

static int bcm20706_bt_get_device_name(char *name)
{
  int ret = BT_SUCCESS;
  size_t nameSize = 0;

  /* Get local name length */

  nameSize = strlen(bt_common_context.bt_name);

  /* Copy local name to name */

  strncpy(name, bt_common_context.bt_name, nameSize);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_paring_enable
 *
 * Description:
 *   Bluetooth pairing mode enable/disable
 *   Set pairing mode and enable/disable.
 *
 ****************************************************************************/

static int bcm20706_bt_paring_enable(bool enable)
{
  int ret = BT_SUCCESS;

  ret = btSetParingMode(BT_INPUT_OUTPUT);  /* TODO: Configurable */

  if (ret != BT_SUCCESS)
    {
      DBG_LOG_DEBUG("pairing mode change failed.");
      return ret;
    }

  ret = btSetPairingEnable(enable ? BT_TRUE : BT_FALSE);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_get_bond_list
 *
 * Description:
 *   Bluetooth get bondling address list.
 *   Get bonding address list.
 *
 ****************************************************************************/

static int bcm20706_bt_get_bond_list(BT_ADDR *addrs, int *num)
{
  int ret = BT_SUCCESS;
  int n;
  BLE_GapBondInfoList bondInfo;
  BT_ADDR addr_list[BLE_SAVE_BOND_DEVICE_MAX_NUM];

  ret = BLE_GapGetBondInfoIdList(&bondInfo);

  if (ret != BT_SUCCESS)
    {
      DBG_LOG_DEBUG("get bondling list failed.");
      return ret;
    }

  *num = bondInfo.bondNum;

  for (n = 0; n < *num && n < BLE_SAVE_BOND_DEVICE_MAX_NUM; n ++)
    {
      memcpy(&addr_list[n].address, &bondInfo.bondInfoId[n], BT_ADDR_LEN);
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_un_bond
 *
 * Description:
 *   Bluetooth unbond by BT_ADDR.
 *   Remove bondling information by BT_ADDR.
 *
 ****************************************************************************/

static int bcm20706_bt_un_bond(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;
  BLE_GapBondInfo info = {0};

  memcpy(info.addr, addr, BT_ADDR_LEN);
  ret = BLE_GapClearBondInfo(&info);

  if (ret != BT_SUCCESS)
    {
      DBG_LOG_DEBUG("unbond failed(%d).\n", ret);
      return ret;
    }

  return btStopBond(addr);
}

/****************************************************************************
 * Name: bcm20706_bt_set_visibility
 *
 * Description:
 *   Bluetooth set visibility.
 *   Set visibility from other device.
 *
 ****************************************************************************/

static int bcm20706_bt_set_visibility(BT_VISIBILITY visibility)
{
  int ret = BT_SUCCESS;
  BT_VISIBILITY_PARAM btVisilityParam =
  {
    BTM_DEFAULT_DISC_WINDOW,
    BTM_DEFAULT_DISC_INTERVAL,
    BTM_DEFAULT_CONN_WINDOW,
    BTM_DEFAULT_CONN_INTERVAL
  };

  ret = btSetVisibilityParam(&btVisilityParam);

  if (ret != BT_SUCCESS)
    {
      DBG_LOG_DEBUG("set visibility parameter failed.");
      return ret;
    }

  ret = btSetVisibility(visibility);

  if (ret != BT_SUCCESS)
    {
      DBG_LOG_DEBUG("set visibility failed.");
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_inquiry_start
 *
 * Description:
 *   Bluetooth start inquiry.
 *   Start inquiry for search near devices.
 *
 ****************************************************************************/

static int bcm20706_bt_inquiry_start(void)
{
  int ret = BT_SUCCESS;

  ret = btStartInquiry();

  if (ret != BT_SUCCESS)
    {
      DBG_LOG_DEBUG("start inquiry failed.");
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_inquiry_cancel
 *
 * Description:
 *   Bluetooth cancel inquiry.
 *   Cancel inquiry to stop search.
 *
 ****************************************************************************/

static int bcm20706_bt_inquiry_cancel(void)
{
  int ret = BT_SUCCESS;

  ret = btCancelInquiry();

  if (ret != BT_SUCCESS)
    {
      DBG_LOG_DEBUG("cancel inquiry failed.");
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct bt_hal_common_ops_s bt_hal_common_ops =
{
  .init          = bcm20706_bt_init,
  .finalize      = bcm20706_bt_finalize,
  .enable        = bcm20706_bt_enable,
  .setDevAddr    = bcm20706_bt_set_device_addr,
  .getDevAddr    = bcm20706_bt_get_device_addr,
  .setDevName    = bcm20706_bt_set_device_name,
  .getDevName    = bcm20706_bt_get_device_name,
  .paringEnable  = bcm20706_bt_paring_enable,
  .getBondList   = bcm20706_bt_get_bond_list,
  .unBond        = bcm20706_bt_un_bond,
  .setVisibility = bcm20706_bt_set_visibility,
  .inquiryStart  = bcm20706_bt_inquiry_start,
  .inquiryCancel = bcm20706_bt_inquiry_cancel
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Public fot BLE */

int btSetBtAddress(BT_ADDR *addr)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_SET_LOCAL_BDA);
  UINT16_TO_STREAM(p, BT_ADDR_LEN);
  memcpy(p,addr, BT_ADDR_LEN);
  p += BT_ADDR_LEN;
  return btUartSendData(buff, p - buff);
}

int btSetBtName(char *name)
{
  uint8_t buff[BT_MID_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_SET_BT_NAME);
  UINT16_TO_STREAM(p, strlen(name) + 1);
  memcpy(p, name, strlen(name));
  p += strlen(name);
  UINT8_TO_STREAM(p, '\0');
  return btUartSendData(buff, p - buff);
}

int btSetPairingEnable(uint8_t isEnable)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_SET_PAIRING_MODE);
  UINT16_TO_STREAM(p, 1);
  UINT8_TO_STREAM(p, isEnable);
  return btUartSendData(buff, p - buff);
}
