/****************************************************************************
 * modules/bluetooth/bluetooth_common.c
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

#include <string.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/hal/bt_if.h>

#include "bluetooth_hal_init.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BLUETOOTH_NAME
#define CONFIG_BLUETOOTH_NAME    "SONY-BT-CLASSIC"
#endif

#ifndef CONFIG_BLUETOOTH_LE_NAME
#define CONFIG_BLUETOOTH_LE_NAME "SONY-BLE-CLASSIC"
#endif

/****************************************************************************
 * Function prototypes
 ****************************************************************************/

extern void ble_gatt_init(struct ble_state_s *ble_state);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_common_state_s g_bt_common_state =
{
  .bt_name  = CONFIG_BLUETOOTH_NAME,
  .ble_name = CONFIG_BLUETOOTH_LE_NAME,
  .bt_addr  = {{0x20, 0x70, 0x3A, 0x10, 0x00, 0x01}},
  .ble_addr = {{0x20, 0x70, 0x3A, 0x10, 0x00, 0x01}}
};

static struct bt_acl_state_s g_bt_acl_state =
{
  .bt_acl_connection = BT_DISCONNECTED,
  .bt_common_state   = &g_bt_common_state
};

static struct ble_state_s g_ble_state =
{
  .ble_connection    = BT_DISCONNECTED,
  .bt_common_state   = &g_bt_common_state
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bt_event_cmd_status(struct bt_event_cmd_stat_t *cmd_stat_evt)
{
  int ret = BT_SUCCESS;
  struct bt_common_ops_s *bt_common_ops = g_bt_common_state.bt_common_ops;

  if (bt_common_ops && bt_common_ops->command_status)
    {
      bt_common_ops->command_status(cmd_stat_evt->cmd_status);
    }
  else
    {
      _err("%s [BT][Common] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int bt_event_pairing_complete(struct bt_event_pair_cmplt_t *pair_cmplt)
{
  int ret = BT_SUCCESS;
  struct bt_common_ops_s *bt_common_ops = g_bt_common_state.bt_common_ops;

  if (bt_common_ops && bt_common_ops->pairing_complete)
    {
      bt_common_ops->pairing_complete(pair_cmplt->addr, pair_cmplt->status);
    }
  else
    {
      _err("%s [BT][Common] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int bt_event_inquiry_result(struct bt_event_inquiry_rslt_t *inq_result_evt)
{
  int ret = BT_SUCCESS;
  struct bt_common_ops_s *bt_common_ops = g_bt_common_state.bt_common_ops;

  if (bt_common_ops && bt_common_ops->inquiry_result)
    {
      bt_common_ops->inquiry_result(inq_result_evt->addr, inq_result_evt->name);
    }
  else
    {
      _err("%s [BT][Common] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int bt_event_inquiry_complete(void)
{
  int ret = BT_SUCCESS;
  struct bt_common_ops_s *bt_common_ops = g_bt_common_state.bt_common_ops;

  if (bt_common_ops && bt_common_ops->inquiry_complete)
    {
      bt_common_ops->inquiry_complete();
    }
  else
    {
      _err("%s [BT][Common] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int bt_event_conn_stat_change(struct bt_event_conn_stat_t *conn_stat_evt)
{
  int ret = BT_SUCCESS;
  struct bt_common_ops_s *bt_common_ops = g_bt_common_state.bt_common_ops;

  if (conn_stat_evt->connected)
    {
      g_bt_acl_state.bt_target_addr = conn_stat_evt->addr;
      g_bt_acl_state.bt_acl_connection = BT_CONNECTED;
    }
  else
    {
      g_bt_acl_state.bt_acl_connection = BT_DISCONNECTED;
    }

  if (bt_common_ops && bt_common_ops->connect_status_changed)
    {
      bt_common_ops->connect_status_changed(&g_bt_acl_state, conn_stat_evt->connected, conn_stat_evt->status);
    }
  else
    {
      _err("%s [BT][Common] Connect status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int bt_event_conn_dev_name(struct bt_event_dev_name_t *dev_name_evt)
{
  int ret = BT_SUCCESS;
  struct bt_common_ops_s *bt_common_ops = g_bt_common_state.bt_common_ops;

  if (bt_common_ops && bt_common_ops->connected_device_name)
    {
      memcpy(g_bt_acl_state.bt_target_name, dev_name_evt->name, BT_NAME_LEN);
      bt_common_ops->connected_device_name(dev_name_evt->name);
    }
  else
    {
      _err("%s [BT][Common] Connected device name callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int bt_event_bond_info(struct bt_event_bond_info_t *bond_info_evt)
{
  int ret = BT_SUCCESS;
  struct bt_common_ops_s *bt_common_ops = g_bt_common_state.bt_common_ops;

  if (bt_common_ops && bt_common_ops->bond_info)
    {
      g_bt_acl_state.bt_target_addr = bond_info_evt->addr;
      bt_common_ops->bond_info(bond_info_evt->addr);
    }
  else
    {
      _err("%s [BT][Common] Bonding information callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int ble_event_connect_stat_change(struct ble_event_conn_stat_t *conn_stat_evt)
{
  int ret = BT_SUCCESS;
  struct ble_common_ops_s *ble_common_ops = g_bt_common_state.ble_common_ops;

  if (conn_stat_evt->connected)
    {
      g_ble_state.bt_target_addr     = conn_stat_evt->addr;
      g_ble_state.ble_connect_handle = conn_stat_evt->handle;
      g_ble_state.ble_connection     = BT_CONNECTED;
    }
  else
    {
      g_ble_state.ble_connection = BT_DISCONNECTED;
    }

  if (ble_common_ops && ble_common_ops->connect_status_changed)
    {
      ble_common_ops->connect_status_changed(&g_ble_state, conn_stat_evt->connected);
    }
  else
    {
      _err("%s [BLE][Common] Connect status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

static int ble_event_connect_dev_name(struct ble_event_dev_name_t *dev_name_evt)
{
  int ret = BT_SUCCESS;
  struct ble_common_ops_s *ble_common_ops = g_bt_common_state.ble_common_ops;

  if (ble_common_ops && ble_common_ops->connected_device_name_resp)
    {
      memcpy(g_ble_state.bt_target_name, dev_name_evt->name, BT_NAME_LEN);
      ble_common_ops->connected_device_name_resp(dev_name_evt->name);
    }
  else
    {
      _err("%s [BLE][Common] Connected device name callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

static int ble_event_advertise_report(struct ble_event_adv_rept_t *adv_rept_evt)
{
  int ret = BT_SUCCESS;
  struct ble_common_ops_s *ble_common_ops = g_bt_common_state.ble_common_ops;

  if (ble_common_ops && ble_common_ops->scan_result)
    {
      ble_common_ops->scan_result(adv_rept_evt->addr, (char*)(adv_rept_evt->data));
    }
  else
    {
      _err("%s [BLE][Common] Advertise report callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_init
 *
 * Description:
 *   Initialize BT module(HAL, File system, Pin config, UART config, etc).
 *
 ****************************************************************************/

int bt_init(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops;

  /* Initialize HAL module */

  ret = bt_hal_init();
  if (ret != BT_SUCCESS)
    {
      _err("%s [BT][Common] Initialization failed(HAL initialize failed).\n", __func__);
      return ret;
    }

  /* Pick HAL callbacks */

  bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->init)
    {
      ret = bt_hal_common_ops->init();
    }

  return ret;
}

/****************************************************************************
 * Name: bt_finalize
 *
 * Description:
 *   Finalize BT module.
 *
 ****************************************************************************/

int bt_finalize(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->finalize)
    {
      ret = bt_hal_common_ops->finalize();
    }

  return ret;
}

/****************************************************************************
 * Name: bt_set_address
 *
 * Description:
 *   Set bluetooth device address.
 *   If not set address, use default address .
 *
 ****************************************************************************/

int bt_set_address(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;

  if (!addr)
    {
      _err("%s [BT][Common] Set local BT address failed(addr not set).\n", __func__);
      return BT_FAIL;
    }

  memcpy(&g_bt_common_state.bt_addr, addr, sizeof(BT_ADDR));
  return ret;
}

/****************************************************************************
 * Name: bt_get_address
 *
 * Description:
 *   Get current bluetooth device address.
 *
 ****************************************************************************/

int bt_get_address(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;
  BT_ADDR target_addr;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (!addr)
    {
      _err("%s [BT][Common] addr not allocated.\n", __func__);
      return BT_FAIL;
    }

  if (bt_hal_common_ops && bt_hal_common_ops->getDevAddr)
    {
      ret = bt_hal_common_ops->getDevAddr(&target_addr);

      memcpy(addr, &target_addr, sizeof(BT_ADDR));
    }
  else
    {
      _err("%s [BT][Common] Get local address failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_set_name
 *
 * Description:
 *   Set bluetooth device name.
 *
 ****************************************************************************/

int bt_set_name(char *name)
{
  int ret = BT_SUCCESS;

  if (!name)
    {
      _err("%s [BT][Common] Set BT name failed(name not set).\n", __func__);
      return BT_FAIL;
    }

  memcpy(g_bt_common_state.bt_name, name, sizeof(char) * BT_NAME_LEN);
  return ret;
}

/****************************************************************************
 * Name: bt_get_name
 *
 * Description:
 *   Get current bluetooth device name.
 *
 ****************************************************************************/

int bt_get_name(char *name)
{
  int ret = BT_SUCCESS;
  char target_name[BT_NAME_LEN];
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (!name)
    {
      _err("%s [BT][Common] name not allocated.\n", __func__);
      return BT_FAIL;
    }

  if (bt_hal_common_ops && bt_hal_common_ops->getDevName)
    {
      ret = bt_hal_common_ops->getDevName(target_name);
      memcpy(name, target_name, sizeof(char) * BT_NAME_LEN);
    }
  else
    {
      _err("%s [BT][Common] Get local name failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_enable
 *
 * Description:
 *   Enable bluetooth module(Power ON, Reset, Firmware load, set device name, etc).
 *
 ****************************************************************************/

int bt_enable(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->enable)
    {
      ret = bt_hal_common_ops->enable(true);
    }
  else
    {
      _err("%s [BT][Common] Enabling failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  if (bt_hal_common_ops && bt_hal_common_ops->setDevAddr)
    {
      ret = bt_hal_common_ops->setDevAddr(&g_bt_common_state.bt_addr);
    }

  if (bt_hal_common_ops && bt_hal_common_ops->setDevName)
    {
      ret = bt_hal_common_ops->setDevName(g_bt_common_state.bt_name);
    }

  return ret;
}

/****************************************************************************
 * Name: bt_disable
 *
 * Description:
 *   Disable bluetooth module(Power OFF).
 *
 ****************************************************************************/

int bt_disable(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->enable)
    {
      ret = bt_hal_common_ops->enable(false);
    }
  else
    {
      _err("%s [BT][Common] Disabling failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_pairing_enable
 *
 * Description:
 *   Start pairing mode with pairing .
 *
 ****************************************************************************/

int bt_pairing_enable(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->paringEnable)
    {
      ret = bt_hal_common_ops->paringEnable(true);
    }
  else
    {
      _err("%s [BT][Common] Paring enable failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_paring_disable
 *
 * Description:
 *   Cancel pairing mode.
 *
 ****************************************************************************/

int bt_paring_disable(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->paringEnable)
    {
      ret = bt_hal_common_ops->paringEnable(false);
    }
  else
    {
      _err("%s [BT][Common] Paring disable failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_get_bond_list
 *
 * Description:
 *   Get bonding device BT_ADDR list.
 *
 ****************************************************************************/

int bt_get_bond_list(BT_ADDR *addr, int *num)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->getBondList)
    {
      ret = bt_hal_common_ops->getBondList(addr, num);
    }
  else
    {
      _err("%s [BT][Common] Get bonding list failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_unbond
 *
 * Description:
 *   Unbond device by BT_ADDR.
 *
 ****************************************************************************/

int bt_unbond(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->unBond)
    {
      ret = bt_hal_common_ops->unBond(addr);
    }
  else
    {
      _err("%s [BT][Common] Get bonding list failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_set_visible
 *
 * Description:
 *   Set bluetooth device to visible for connect from peer device.
 *
 ****************************************************************************/

int bt_set_visibility(BT_VISIBILITY visibility)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->setVisibility)
    {
      ret = bt_hal_common_ops->setVisibility(visibility);
    }
  else
    {
      _err("%s [BT][Common] Set visibility failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_start_inquiry
 *
 * Description:
 *   Start inquiry to device by name.
 *
 ****************************************************************************/

int bt_start_inquiry(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->inquiryStart)
    {
      ret = bt_hal_common_ops->inquiryStart();
    }
  else
    {
      _err("%s [BT][Common] Inquiry start(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_cancel_inquiry
 *
 * Description:
 *   Cancel to current inquiry.
 *
 ****************************************************************************/

int bt_cancel_inquiry(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->inquiryCancel)
    {
      ret = bt_hal_common_ops->inquiryCancel();
    }
  else
    {
      _err("%s [BT][Common] Inquiry cancel failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_register_common_cb
 *
 * Description:
 *   Register A2DP generic BT operation callback for Application.
 *
 ****************************************************************************/

int bt_register_common_cb(struct bt_common_ops_s *bt_common_ops)
{
  if (!bt_common_ops)
    {
      _err("%s [BT][Common] Set application callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_common_state.bt_common_ops = bt_common_ops;

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_common_register_hal
 *
 * Description:
 *   Register common HAl interface.
 *
 ****************************************************************************/

int bt_common_register_hal(struct bt_hal_common_ops_s *bt_hal_common_ops)
{
  if (!bt_hal_common_ops)
    {
      _err("%s [BT][Common] Set HAL callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_common_state.bt_hal_common_ops = bt_hal_common_ops;

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_common_event_handler
 *
 * Description:
 *   Handler of generic event.
 *   Receive generic event from HAL and dispatch event for application.
 *
 ****************************************************************************/

int bt_common_event_handler(struct bt_event_t *bt_event)
{
  switch (bt_event->event_id)
    {
      case BT_COMMON_EVENT_CMD_STATUS:
        return bt_event_cmd_status((struct bt_event_cmd_stat_t *) bt_event);

      case BT_COMMON_EVENT_PAIRING_COMPLETE:
        return bt_event_pairing_complete((struct bt_event_pair_cmplt_t *) bt_event);

      case BT_COMMON_EVENT_INQUIRY_RESULT:
        return bt_event_inquiry_result((struct bt_event_inquiry_rslt_t *) bt_event);

      case BT_COMMON_EVENT_INQUIRY_COMPLETE:
        return bt_event_inquiry_complete();

      case BT_COMMON_EVENT_CONN_STAT_CHANGE:
        return bt_event_conn_stat_change((struct bt_event_conn_stat_t *) bt_event);

      case BT_COMMON_EVENT_CONN_DEV_NAME:
        return bt_event_conn_dev_name((struct bt_event_dev_name_t *) bt_event);

      case BT_COMMON_EVENT_BOND_INFO:
        return bt_event_bond_info((struct bt_event_bond_info_t *) bt_event);

      default:
        break;
    }
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_set_address
 *
 * Description:
 *   Set Bluetooth LE module address
 *   This is Spresense side address and should be call before bt_enable.
 *
 ****************************************************************************/

int ble_set_address(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;

  if (!addr)
    {
      _err("%s [BLE][Common] Set local BT address failed(addr not set).\n", __func__);
      return BT_FAIL;
    }

  memcpy(&g_bt_common_state.ble_addr, addr, sizeof(BT_ADDR));
  return ret;
}

/****************************************************************************
 * Name: ble_get_address
 *
 * Description:
 *   Get Bluetooth LE module address
 *
 ****************************************************************************/

int ble_get_address(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;

  if (!addr)
    {
      _err("%s [BLE][Common] addr not allocated.\n", __func__);
      return BT_FAIL;
    }

  memcpy(addr, &g_bt_common_state.ble_addr, sizeof(BT_ADDR));
  return ret;
}

/****************************************************************************
 * Name: ble_set_name
 *
 * Description:
 *   Set Bluetooth LE module name
 *   This name visible for other devices and should be call before bt_enable.
 *
 ****************************************************************************/

int ble_set_name(char *name)
{
  int ret = BT_SUCCESS;

  if (!name)
    {
      _err("%s [BLE][Common] Set BT name failed(name not set).\n", __func__);
      return BT_FAIL;
    }

  memcpy(g_bt_common_state.ble_name, name, sizeof(char) * BT_NAME_LEN);
  return ret;
}

/****************************************************************************
 * Name: ble_get_name
 *
 * Description:
 *   Get Bluetooth LE module name
 *
 ****************************************************************************/

int ble_get_name(char *name)
{
  int ret = BT_SUCCESS;

  if (!name)
    {
      _err("%s [BLE][Common] name not allocated.\n", __func__);
      return BT_FAIL;
    }

  memcpy(name, g_bt_common_state.ble_name, sizeof(char) * BT_NAME_LEN);
  return ret;
}

/****************************************************************************
 * Name: ble_enable
 *
 * Description:
 *   Bluetooth LE enable
 *   Set initialize parameter to HAL(Name/Addr/PPCP/Appearance)
 *
 ****************************************************************************/

int ble_enable(void)
{
  int ret = BT_SUCCESS;
  struct ble_hal_common_ops_s *ble_hal_common_ops = g_bt_common_state.ble_hal_common_ops;
  BLE_CONN_PARAMS ppcp = {0xFFFF, 0xFFFF, 0x00, 0xFFFF};

  if (ble_hal_common_ops && ble_hal_common_ops->setDevName &&
      ble_hal_common_ops->setDevAddr &&
      ble_hal_common_ops->setAppearance &&
      ble_hal_common_ops->setPPCP)
    {
      ret = ble_hal_common_ops->setDevName(g_bt_common_state.ble_name);

      if (ret != BT_SUCCESS)
        {
          _err("%s [BLE][Common] BLE set name failed.\n", __func__);
          return ret;
        }

      ret = ble_hal_common_ops->setDevAddr(&g_bt_common_state.ble_addr);

      if (ret != BT_SUCCESS)
        {
          _err("%s [BLE][Common] BLE set address failed.\n", __func__);
          return ret;
        }

      ret = ble_hal_common_ops->setAppearance(BLE_APPEARANCE_GENERIC_PHONE);

      if (ret != BT_SUCCESS)
        {
          _err("%s [BLE][Common] BLE set appearance failed.\n", __func__);
          return ret;
        }

      ret = ble_hal_common_ops->setPPCP(ppcp);

      if (ret != BT_SUCCESS)
        {
          _err("%s [BLE][Common] BLE set PPCP failed.\n", __func__);
          return ret;
        }

      /* Initialize BLE GATT */

      ble_gatt_init(&g_ble_state);
    }
  else
    {
      _err("%s [BLE][Common] BLE enable failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_disable
 *
 * Description:
 *   Bluetooth LE disable
 *
 ****************************************************************************/

int ble_disable(void)
{
  /* nop */
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_connect
 *
 * Description:
 *   Bluetooth LE connect for Central
 *   This function is for Central role.
 *
 ****************************************************************************/

int ble_connect(struct ble_state_s *ble_state)
{
  int ret = BT_SUCCESS;
  struct ble_hal_common_ops_s *ble_hal_common_ops = g_bt_common_state.ble_hal_common_ops;

  if (ble_hal_common_ops && ble_hal_common_ops->connect)
    {
      ret = ble_hal_common_ops->connect(&ble_state->bt_target_addr);
    }
  else
    {
      _err("%s [BLE][Common] BLE connect failed(Central not supported yet).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_disconnect
 *
 * Description:
 *   Bluetooth LE disconnect for Central
 *   This function is for Central role.
 *
 ****************************************************************************/

int ble_disconnect(struct ble_state_s *ble_state)
{
  int ret = BT_SUCCESS;
  struct ble_hal_common_ops_s *ble_hal_common_ops = g_bt_common_state.ble_hal_common_ops;

  if (ble_hal_common_ops && ble_hal_common_ops->disconnect)
    {
      ret = ble_hal_common_ops->disconnect(ble_state->ble_connect_handle);
    }
  else
    {
      _err("%s [BLE][Common] BLE connect failed(Central not supported yet).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_start_advertise
 *
 * Description:
 *   Start BLE advertise mode.
 *   This function is for Peripheral role.
 *
 ****************************************************************************/

int ble_start_advertise(void)
{
  int ret = BT_SUCCESS;
  struct ble_hal_common_ops_s *ble_hal_common_ops = g_bt_common_state.ble_hal_common_ops;

  if (ble_hal_common_ops && ble_hal_common_ops->advertise)
    {
      ret = ble_hal_common_ops->advertise(true);
    }
  else
    {
      _err("%s [BLE][Common] BLE start advertise(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_cancel_advertise
 *
 * Description:
 *   Cancel BLE advertise mode.
 *   This function is for Peripheral role.
 *
 ****************************************************************************/

int ble_cancel_advertise(void)
{
  int ret = BT_SUCCESS;
  struct ble_hal_common_ops_s *ble_hal_common_ops = g_bt_common_state.ble_hal_common_ops;

  if (ble_hal_common_ops && ble_hal_common_ops->advertise)
    {
      ret = ble_hal_common_ops->advertise(false);
    }
  else
    {
      _err("%s [BLE][Common] BLE cancel advertise(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_start_scan
 *
 * Description:
 *   Start BLE scan mode.
 *   This function is for Central role.
 *
 ****************************************************************************/

int ble_start_scan(void)
{
  int ret = BT_SUCCESS;
  struct ble_hal_common_ops_s *ble_hal_common_ops = g_bt_common_state.ble_hal_common_ops;

  if (ble_hal_common_ops && ble_hal_common_ops->scan)
    {
      ret = ble_hal_common_ops->scan(true);
    }
  else
    {
      _err("%s [BLE][Common] BLE scan failed(Central not supported yet).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_cancel_scan
 *
 * Description:
 *   Cancel BLE scan mode.
 *   This function is for Central role.
 *
 ****************************************************************************/

int ble_cancel_scan(void)
{
  int ret = BT_SUCCESS;
  struct ble_hal_common_ops_s *ble_hal_common_ops = g_bt_common_state.ble_hal_common_ops;

  if (ble_hal_common_ops && ble_hal_common_ops->scan)
    {
      ret = ble_hal_common_ops->scan(false);
    }
  else
    {
      _err("%s [BLE][Common] BLE scan failed(Central not supported yet).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_register_common_cb
 *
 * Description:
 *   Bluetooth LE register common callbacks
 *   Register Connect/Advertise/Scan callback.
 *
 ****************************************************************************/

int ble_register_common_cb(struct ble_common_ops_s *ble_common_ops)
{
  if (!ble_common_ops)
    {
      _err("%s [BLE][Common] Set application callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_common_state.ble_common_ops = ble_common_ops;

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_common_register_hal
 *
 * Description:
 *   Bluetooth LE common function HAL register
 *   This is Spresense side address and should be call before bt_enable.
 *
 ****************************************************************************/

int ble_common_register_hal(struct ble_hal_common_ops_s *ble_hal_common_ops)
{
  if (!ble_hal_common_ops)
    {
      _err("%s [BLE][Common] Set HAL callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_common_state.ble_hal_common_ops = ble_hal_common_ops;

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_common_event_handler
 *
 * Description:
 *   BLE common event handler
 *   HAL should call this function if receive BLE common event(@ref BLE_COMMON_EVENT_ID).
 *
 ****************************************************************************/

int ble_common_event_handler(struct bt_event_t *bt_event)
{
  switch (bt_event->event_id)
    {
      case BLE_COMMON_EVENT_CONN_STAT_CHANGE:
        return ble_event_connect_stat_change((struct ble_event_conn_stat_t *) bt_event);

      case BLE_COMMON_EVENT_CONN_DEV_NAME:
        return ble_event_connect_dev_name((struct ble_event_dev_name_t *) bt_event);

      case BLE_COMMON_EVENT_SCAN_RESULT:
        return ble_event_advertise_report((struct ble_event_adv_rept_t *) bt_event);

      default:
        break;
    }
  return BT_SUCCESS;
}

