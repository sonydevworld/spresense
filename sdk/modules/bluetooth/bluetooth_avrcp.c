/****************************************************************************
 * modules/bluetooth/bluetooth_avrcp.c
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

#include <bluetooth/bt_avrcp.h>
#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_avrcp_state_s g_bt_avrcp_state =
{
  .bt_avrcc_connection = BT_DISCONNECTED,
  .bt_avrct_connection = BT_DISCONNECTED
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int event_cmd_status(struct bt_event_cmd_stat_t *cmd_stat_evt)
{
  int ret = BT_SUCCESS;
  struct bt_avrcp_ops_s *bt_avrcp_ops = g_bt_avrcp_state.bt_avrcp_ops;

  if (bt_avrcp_ops && bt_avrcp_ops->command_status)
    {
      bt_avrcp_ops->command_status(cmd_stat_evt->cmd_status);
    }
  else
    {
      _err("%s [BT][AVRCP] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_connect(struct bt_avrcp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_avrcp_ops_s *bt_avrcp_ops = g_bt_avrcp_state.bt_avrcp_ops;
  BT_AVRCP_ROLE role = event_connect->event_id == BT_AVRCC_EVENT_CONNECT ?
                       BT_AVRCP_CONTROLLER : BT_AVRCP_TARGET;

  if (role == BT_AVRCP_CONTROLLER)
    {
      g_bt_avrcp_state.bt_avrcc_connection = BT_CONNECTED;
      g_bt_avrcp_state.bt_avrcc_handle = event_connect->handle;
    }
  else
    {
      g_bt_avrcp_state.bt_avrct_connection = BT_CONNECTED;
      g_bt_avrcp_state.bt_avrct_handle = event_connect->handle;
    }

  if (bt_avrcp_ops && bt_avrcp_ops->connect)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_avrcp_ops->connect(g_bt_avrcp_state.bt_acl_state, role);
    }
  else
    {
      _err("%s [BT][AVRCP] AVRCP connect callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_disconnect(struct bt_avrcp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_avrcp_ops_s *bt_avrcp_ops = g_bt_avrcp_state.bt_avrcp_ops;
  BT_AVRCP_ROLE role = event_connect->event_id == BT_AVRCC_EVENT_DISCONNECT ?
                       BT_AVRCP_CONTROLLER : BT_AVRCP_TARGET;

  if (role == BT_AVRCP_CONTROLLER)
    {
      g_bt_avrcp_state.bt_avrcc_connection = BT_DISCONNECTED;
    }
  else
    {
      g_bt_avrcp_state.bt_avrct_connection = BT_DISCONNECTED;
    }

  if (bt_avrcp_ops && bt_avrcp_ops->disconnect)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_avrcp_ops->disconnect(g_bt_avrcp_state.bt_acl_state, role);
    }
  else
    {
      _err("%s [BT][AVRCP] AVRCP disconnect callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_play_position(struct bt_avrcp_event_play_position_t *event_pos)
{
  int ret = BT_SUCCESS;
  struct bt_avrcp_notify_ops_s *bt_avrcp_notify_ops = g_bt_avrcp_state.bt_avrcp_notify_ops;


  if (bt_avrcp_notify_ops && bt_avrcp_notify_ops->playPosChanged)
    {
      bt_avrcp_notify_ops->playPosChanged((uint8_t *)&event_pos->position, sizeof(event_pos->position));
    }
  else
    {
      _err("%s [BT][AVRCP] AVRCP play position callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_avrcp_is_supported
 *
 * Description:
 *   Return AVRCP support status by bool.
 *
 ****************************************************************************/

bool bt_avrcp_is_supported(void)
{
  /* If HAL interface was registered, return true. */

  return !(!g_bt_avrcp_state.bt_hal_avrcp_ops);
}

/****************************************************************************
 * Name: bt_avrcp_connect
 *
 * Description:
 *   Connect to AVRCP(AVRCC/AVRCT) to peer device.
 *
 ****************************************************************************/

int bt_avrcp_connect(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_avrcp_ops_s *bt_hal_avrcp_ops = g_bt_avrcp_state.bt_hal_avrcp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][AVRCP] Connect failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_avrcp_state.bt_avrcc_connection != BT_DISCONNECTED &&
      g_bt_avrcp_state.bt_avrct_connection != BT_DISCONNECTED)
    {
      _err("%s [BT][AVRCP] AVRCP not disconnected(AVRCC: %d, AVRCT: %d).\n",
      __func__, g_bt_avrcp_state.bt_avrcc_connection, g_bt_avrcp_state.bt_avrct_connection);
    }

  if (bt_hal_avrcp_ops && bt_hal_avrcp_ops->avrcc_connect &&
      bt_hal_avrcp_ops->avrct_connect)
    {
      ret = bt_hal_avrcp_ops->avrcc_connect(&bt_acl_state->bt_target_addr, g_bt_avrcp_state.bt_avrcc_handle, true);
      ret |= bt_hal_avrcp_ops->avrct_connect(&bt_acl_state->bt_target_addr, g_bt_avrcp_state.bt_avrct_handle, true);
      g_bt_avrcp_state.bt_acl_state = bt_acl_state;
      g_bt_avrcp_state.bt_avrcc_connection = BT_CONNECTING;
      g_bt_avrcp_state.bt_avrct_connection = BT_CONNECTING;
    }
  else
    {
      _err("%s [BT][AVRCP] Connect failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_avrcp_disconnect
 *
 * Description:
 *   Disconnect to AVRCP(AVRCC/AVRCT) to peer device.
 *
 ****************************************************************************/

int bt_avrcp_disconnect(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_avrcp_ops_s *bt_hal_avrcp_ops = g_bt_avrcp_state.bt_hal_avrcp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][AVRCP] Disconnect failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_avrcp_state.bt_avrcc_connection != BT_CONNECTED &&
      g_bt_avrcp_state.bt_avrct_connection != BT_CONNECTED)
    {
      _err("%s [BT][AVRCP] AVRCP not connected(AVRCC: %d, AVRCT: %d).\n",
      __func__, g_bt_avrcp_state.bt_avrcc_connection, g_bt_avrcp_state.bt_avrct_connection);
    }

  if (bt_hal_avrcp_ops && bt_hal_avrcp_ops->avrcc_connect &&
      bt_hal_avrcp_ops->avrct_connect)
    {
      ret = bt_hal_avrcp_ops->avrcc_connect(&bt_acl_state->bt_target_addr, g_bt_avrcp_state.bt_avrcc_handle, false);
      ret |= bt_hal_avrcp_ops->avrct_connect(&bt_acl_state->bt_target_addr, g_bt_avrcp_state.bt_avrct_handle, false);
      g_bt_avrcp_state.bt_avrcc_connection = BT_DISCONNECTING;
      g_bt_avrcp_state.bt_avrct_connection = BT_DISCONNECTING;
    }
  else
    {
      _err("%s [BT][AVRCP] Disconnect failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_avrcp_send_command
 *
 * Description:
 *   Send AVRCC event to AVRCP(AVRCC/AVRCT) to peer device.
 *
 ****************************************************************************/

int bt_avrcp_send_command(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_CMD_ID cmd_id, bool press)
{
  int ret = BT_SUCCESS;
  struct bt_hal_avrcp_ops_s *bt_hal_avrcp_ops = g_bt_avrcp_state.bt_hal_avrcp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][AVRCP] Send command failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_avrcp_state.bt_avrcc_connection != BT_CONNECTED)
    {
      _err("%s [BT][AVRCP] AVRCP not connected(AVRCC: %d,).\n",
      __func__, g_bt_avrcp_state.bt_avrcc_connection);
    }

  if (bt_hal_avrcp_ops && bt_hal_avrcp_ops->send_avrcp_command)
    {
      ret = bt_hal_avrcp_ops->send_avrcp_command(&bt_acl_state->bt_target_addr, cmd_id, press, g_bt_avrcp_state.bt_avrcc_handle);
    }
  else
    {
      _err("%s [BT][AVRCP] Send command failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_register_notification
 *
 * Description:
 *   Register notification callbacks and set notification list to HAL.
 *
 ****************************************************************************/

int bt_register_notification(struct bt_avrcp_notify_ops_s *bt_avrcp_notify_ops)
{
  int ret = BT_SUCCESS;
  BT_AVRC_SUPPORT_NOTIFY_EVENT notification_list;
  struct bt_hal_avrcp_ops_s *bt_hal_avrcp_ops = g_bt_avrcp_state.bt_hal_avrcp_ops;

  if (!bt_avrcp_notify_ops)
    {
      _err("%s [BT][AVRCP] Set notification callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_avrcp_state.bt_avrcp_notify_ops = bt_avrcp_notify_ops;

  notification_list.playStatusChange    = (bt_avrcp_notify_ops->playStatusChange != NULL);
  notification_list.trackChange         = (bt_avrcp_notify_ops->trackChange != NULL);
  notification_list.trackReachedEnd     = (bt_avrcp_notify_ops->trackReachedEnd != NULL);
  notification_list.trackReachedStart   = (bt_avrcp_notify_ops->trackReachedStart != NULL);
  notification_list.playPosChanged      = (bt_avrcp_notify_ops->playPosChanged != NULL);
  notification_list.batteryStatusChange = (bt_avrcp_notify_ops->batteryStatusChange != NULL);
  notification_list.systemStatusChange  = (bt_avrcp_notify_ops->systemStatusChange != NULL);
  notification_list.appSettingChange    = (bt_avrcp_notify_ops->appSettingChange != NULL);
  notification_list.nowPlayingChange    = (bt_avrcp_notify_ops->nowPlayingChange != NULL);
  notification_list.avalPlayerChange    = (bt_avrcp_notify_ops->avalPlayerChange != NULL);
  notification_list.addrPlayChange      = (bt_avrcp_notify_ops->addrPlayChange != NULL);
  notification_list.uidsChange          = (bt_avrcp_notify_ops->uidsChange != NULL);
  notification_list.volumeChange        = (bt_avrcp_notify_ops->volumeChange != NULL);

  if (bt_hal_avrcp_ops && bt_hal_avrcp_ops->configure_notification)
    {
      ret = bt_hal_avrcp_ops->configure_notification(&notification_list);
    }
  else
    {
      _err("%s [BT][AVRCP] Set notification callback failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_avrcp_register_cb
 *
 * Description:
 *   Register AVRCP connection callback for Application.
 *
 ****************************************************************************/

int bt_avrcp_register_cb(struct bt_avrcp_ops_s *bt_avrcp_ops)
{
  if (!bt_avrcp_ops)
    {
      _err("%s [BT][AVRCP] Set application callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_avrcp_state.bt_avrcp_ops = bt_avrcp_ops;
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_avrcp_register_hal
 *
 * Description:
 *   Register AVRCP HAl interface.
 *
 ****************************************************************************/

int bt_avrcp_register_hal(struct bt_hal_avrcp_ops_s *bt_hal_avrcp_ops)
{
  if (!bt_hal_avrcp_ops)
    {
      _err("%s [BT][AVRCP] Set HAL callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_avrcp_state.bt_hal_avrcp_ops = bt_hal_avrcp_ops;
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_avrcp_event_handler
 *
 * Description:
 *   Handler of AVRCP event.
 *   Receive AVRCP event from HAL and dispatch event for application.
 *
 ****************************************************************************/

int bt_avrcp_event_handler(struct bt_event_t *bt_event)
{
  switch (bt_event->event_id)
    {
      case BT_AVRCP_EVENT_CMD_STATUS:
        return event_cmd_status((struct bt_event_cmd_stat_t *) bt_event);

      case BT_AVRCC_EVENT_CONNECT:
      case BT_AVRCT_EVENT_CONNECT:
        return event_connect((struct bt_avrcp_event_connect_t *) bt_event);

      case BT_AVRCC_EVENT_DISCONNECT:
      case BT_AVRCT_EVENT_DISCONNECT:
        return event_disconnect((struct bt_avrcp_event_connect_t *) bt_event);

      case BT_AVRCP_EVENT_PLAY_STAT_CHANGE:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_TRACK_CHANGE:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_TRACK_REACH_END:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_TRACK_REACH_START:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_PLAY_POS_CHANGE:
        return event_play_position((struct bt_avrcp_event_play_position_t *) bt_event);

      case BT_AVRCP_EVENT_BATT_STAT_CHANGE:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_SYS_STATUS_CHANGE:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_APP_SETT_CHANGE:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_NOW_PLAY_CHANGE:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_AVAI_PLAYER_CHANGE:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_ADDR_PLAYER_CHANGE:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_UIDS_CHANGE:
        /* Not supported yet */
        break;
      case BT_AVRCP_EVENT_VOLUME_CHANGE:
        /* Not supported yet */
        break;
      default:
        break;
    }
  return BT_SUCCESS;
}

