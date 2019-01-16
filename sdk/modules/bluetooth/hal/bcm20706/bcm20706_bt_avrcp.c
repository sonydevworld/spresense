/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bcm20706_bt_avrcp.c
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
#include <stdlib.h>
#include <bt/bt_avrc_con.h>

#include "bt_util.h"
#include "manager/bt_uart_manager.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int btAvrcControllerConnect(BT_ADDR *addr)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_AVRC_CONTROLLER_COMMAND_CONNECT);
  UINT16_TO_STREAM(p, BT_ADDR_LEN);
  memcpy(p, addr, BT_ADDR_LEN);
  p += BT_ADDR_LEN;
  return btUartSendData(buff, p - buff);
}

static int btAvrcControllerDisconnect(uint16_t handle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_AVRC_CONTROLLER_COMMAND_DISCONNECT);
  UINT16_TO_STREAM(p, 2);
  UINT16_TO_STREAM(p, handle);
  return btUartSendData(buff, p - buff);
}

static int btAvrcTargetConnect(BT_ADDR *addr)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_AVRC_TARGET_COMMAND_CONNECT);
  UINT16_TO_STREAM(p, BT_ADDR_LEN);
  memcpy(p, addr, BT_ADDR_LEN);
  return btUartSendData(buff, p - buff);
}

static int btAvrcTargetDisconnect(void)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_AVRC_TARGET_COMMAND_DISCONNECT);
  UINT16_TO_STREAM(p, 0);
  return btUartSendData(buff, p - buff);
}

static int btAvrcSendCommand(uint16_t handle, uint16_t cmd)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, cmd);
  UINT16_TO_STREAM(p, 2);
  UINT16_TO_STREAM(p, handle);
  btUartSendData(buff, p - buff);
  return 0;
}


/****************************************************************************
 * Name: bcm20706_bt_inquiry_cancel
 *
 * Description:
 *   Bluetooth cancel inquiry.
 *   Cancel inquiry to stop search.
 *
 ****************************************************************************/

static int bcm20706_bt_avrcp_controller_connect(BT_ADDR *addr, uint16_t handle, bool connect)
{
  int ret = BT_SUCCESS;

  if (connect)
    {
      /* Connect */

      ret = btAvrcControllerConnect(addr);
    }
  else
    {
      /* Disconnect */

      ret = btAvrcControllerDisconnect(handle);
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

static int bcm20706_bt_avrcp_target_connect(BT_ADDR *addr, uint16_t handle, bool connect)
{
  int ret = BT_SUCCESS;

  if (connect)
    {
      /* Connect */

      ret = btAvrcTargetConnect(addr);
    }
  else
    {
      /* Disconnect */

      ret = btAvrcTargetDisconnect();
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

static int bcm20706_bt_avrcp_send_avrcp_command(BT_ADDR *addr, BT_AVRCP_CMD_ID cmd_id, bool press, uint16_t handle)
{
  uint16_t cmd = 0x0FFFF;
  int ret = BT_SUCCESS;

  switch (cmd_id)
    {
      case BT_AVRCP_CMD_VOL_UP:
        cmd = BT_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP;
        break;

      case BT_AVRCP_CMD_VOL_DOWN:
        cmd = BT_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN;
        break;

      case BT_AVRCP_CMD_PLAY:
        cmd = BT_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY;
        break;

      case BT_AVRCP_CMD_STOP:
        cmd = BT_CONTROL_AVRC_CONTROLLER_COMMAND_STOP;
        break;

      case BT_AVRCP_CMD_PAUSE:
        cmd = BT_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE;
        break;

      default:
        printf("%s [BT][AVRCP] Command ID (%d) not supported wit HAL.\n", __func__, cmd_id);
        ret = BT_FAIL;
        goto bye;
    }

  if ((cmd ^ 0x0FFFF) == 0)
    {
      printf("%s [BT][AVRCP] Command (%d) invalid.\n", __func__, cmd);
      ret = BT_FAIL;
      goto bye;
    }

  ret = btAvrcSendCommand(handle, cmd);

bye:
  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_avrcp_configure_notification
 *
 * Description:
 *   Bluetooth avrcp configure notification.
 *   Register for the avrcp configure notifiction.
 *
 ****************************************************************************/

static int bcm20706_bt_avrcp_configure_notification(BT_AVRC_SUPPORT_NOTIFY_EVENT *notification_list)
{
  int ret = BT_SUCCESS;

  /* Not supported yet */

  return ret;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct bt_hal_avrcp_ops_s bt_hal_avrcp_ops =
{
  .avrcc_connect          = bcm20706_bt_avrcp_controller_connect,
  .avrct_connect          = bcm20706_bt_avrcp_target_connect,
  .send_avrcp_command     = bcm20706_bt_avrcp_send_avrcp_command,
  .configure_notification = bcm20706_bt_avrcp_configure_notification
};

