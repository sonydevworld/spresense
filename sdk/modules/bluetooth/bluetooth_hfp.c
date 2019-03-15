/****************************************************************************
 * modules/bluetooth/bluetooth_hfp.c
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

#include <bluetooth/bt_hfp.h>
#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_hfp_state_s g_bt_hfp_state =
{
  .bt_hfp_connection       = BT_DISCONNECTED,
  .bt_hfp_audio_connection = BT_DISCONNECTED,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int event_cmd_status(struct bt_event_cmd_stat_t *cmd_stat_evt)
{
  int ret = BT_SUCCESS;
  struct bt_hfp_ops_s *bt_hfp_ops = g_bt_hfp_state.bt_hfp_ops;

  if (bt_hfp_ops && bt_hfp_ops->command_status)
    {
      bt_hfp_ops->command_status(cmd_stat_evt->cmd_status);
    }
  else
    {
      _err("%s [BT][HFP] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_hf_connect(struct bt_hfp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_hfp_ops_s *bt_hfp_ops = g_bt_hfp_state.bt_hfp_ops;

  g_bt_hfp_state.bt_hfp_connection = BT_CONNECTED;

  if (bt_hfp_ops && bt_hfp_ops->connect)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_hfp_ops->connect(g_bt_hfp_state.bt_acl_state, event_connect->hfp_type);

      g_bt_hfp_state.bt_hfp_handle = event_connect->handle;
      g_bt_hfp_state.bt_hfp_audio_handle = event_connect->handle;
    }
  else
    {
      _err("%s [BT][HFP] HFP connect callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_hf_disconnect(struct bt_hfp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_hfp_ops_s *bt_hfp_ops = g_bt_hfp_state.bt_hfp_ops;

  g_bt_hfp_state.bt_hfp_connection = BT_DISCONNECTED;

  if (bt_hfp_ops && bt_hfp_ops->disconnect)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_hfp_ops->disconnect(g_bt_hfp_state.bt_acl_state);
    }
  else
    {
      _err("%s [BT][HFP] HFP connect callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_audio_connect(struct bt_hfp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_hfp_ops_s *bt_hfp_ops = g_bt_hfp_state.bt_hfp_ops;

  g_bt_hfp_state.bt_hfp_audio_connection = BT_CONNECTED;

  if (bt_hfp_ops && bt_hfp_ops->audio_connect)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_hfp_ops->audio_connect(g_bt_hfp_state.bt_acl_state);

      g_bt_hfp_state.bt_hfp_audio_handle = event_connect->handle;
    }
  else
    {
      _err("%s [BT][HFP] HFP audio connect callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_audio_disconnect(struct bt_hfp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_hfp_ops_s *bt_hfp_ops = g_bt_hfp_state.bt_hfp_ops;

  g_bt_hfp_state.bt_hfp_audio_connection = BT_DISCONNECTED;

  if (bt_hfp_ops && bt_hfp_ops->audio_disconnect)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_hfp_ops->audio_disconnect(g_bt_hfp_state.bt_acl_state);
    }
  else
    {
      _err("%s [BT][HFP] HFP audio connect callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_ag_feature_resp(struct bt_hfp_event_ag_feature_t *event_ag_flag)
{
  int ret = BT_SUCCESS;
  struct bt_hfp_ops_s *bt_hfp_ops = g_bt_hfp_state.bt_hfp_ops;

  if (bt_hfp_ops && bt_hfp_ops->ag_feature)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_hfp_ops->ag_feature(g_bt_hfp_state.bt_acl_state, event_ag_flag->ag_flag);
    }
  else
    {
      _err("%s [BT][HFP] HFP AG feature response callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_at_cmd_resp(struct bt_hfp_event_at_cmd_t *event_at_resp)
{
  int ret = BT_SUCCESS;
  struct bt_hfp_ops_s *bt_hfp_ops = g_bt_hfp_state.bt_hfp_ops;

  if (bt_hfp_ops && bt_hfp_ops->hf_at_response)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_hfp_ops->hf_at_response(g_bt_hfp_state.bt_acl_state, event_at_resp->at_resp);
    }
  else
    {
      _err("%s [BT][HFP] HFP AT command response callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_hfp_is_supported
 *
 * Description:
 *   Return HFP support status by bool.
 *
 ****************************************************************************/

bool bt_hfp_is_supported(void)
{
  /* If HAL interface was registered, return true. */

  return !(!g_bt_hfp_state.bt_hal_hfp_ops);
}

/****************************************************************************
 * Name: bt_hfp_connect
 *
 * Description:
 *   Connect to peer device on HFP.
 *
 ****************************************************************************/

int bt_hfp_connect(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_hfp_ops_s *bt_hal_hfp_ops = g_bt_hfp_state.bt_hal_hfp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][HFP] Connect failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_hfp_state.bt_hfp_connection != BT_DISCONNECTED)
    {
      _err("%s [BT][HFP] Connect failed(Not disconnected).\n", __func__);
      return -EINVAL;
    }

  if (bt_hal_hfp_ops && bt_hal_hfp_ops->connect)
    {
      ret = bt_hal_hfp_ops->connect(&bt_acl_state->bt_target_addr, g_bt_hfp_state.bt_hfp_handle, true);
      g_bt_hfp_state.bt_acl_state = bt_acl_state;
      g_bt_hfp_state.bt_hfp_connection = BT_CONNECTING;
    }
  else
    {
      _err("%s [BT][HFP] Connect failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_hfp_disconnect
 *
 * Description:
 *   Disconnect to peer device on HFP.
 *
 ****************************************************************************/

int bt_hfp_disconnect(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_hfp_ops_s *bt_hal_hfp_ops = g_bt_hfp_state.bt_hal_hfp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][HFP] Disconnect failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_hfp_state.bt_hfp_connection != BT_CONNECTED)
    {
      _err("%s [BT][HFP] Disconnect failed(Not connected).\n", __func__);
      return -EINVAL;
    }

  if (bt_hal_hfp_ops && bt_hal_hfp_ops->connect)
    {
      ret = bt_hal_hfp_ops->connect(&bt_acl_state->bt_target_addr, g_bt_hfp_state.bt_hfp_handle, false);
      g_bt_hfp_state.bt_hfp_connection = BT_DISCONNECTING;
    }
  else
    {
      _err("%s [BT][HFP] Disconnect failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_hfp_audio_connect
 *
 * Description:
 *   Connect to peer device on HFP audio.
 *
 ****************************************************************************/

int bt_hfp_audio_connect(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_hfp_ops_s *bt_hal_hfp_ops = g_bt_hfp_state.bt_hal_hfp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][HFP] Connect audio failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_hfp_state.bt_hfp_audio_connection != BT_DISCONNECTED)
    {
      _err("%s [BT][HFP] Connect audio failed(Not disconnected).\n", __func__);
      return -EINVAL;
    }

  if (bt_hal_hfp_ops && bt_hal_hfp_ops->audio_connect)
    {
      ret = bt_hal_hfp_ops->audio_connect(&bt_acl_state->bt_target_addr, g_bt_hfp_state.bt_hfp_audio_handle, true);
      g_bt_hfp_state.bt_hfp_audio_connection = BT_CONNECTING;
    }
  else
    {
      _err("%s [BT][HFP] Connect audio failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_hfp_audio_disconnect
 *
 * Description:
 *   Disconnect to peer device on HFP audio.
 *
 ****************************************************************************/

int bt_hfp_audio_disconnect(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_hfp_ops_s *bt_hal_hfp_ops = g_bt_hfp_state.bt_hal_hfp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][HFP] Disconnect audio failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_hfp_state.bt_hfp_audio_connection != BT_CONNECTED)
    {
      _err("%s [BT][HFP] Disconnect audio failed(Not connected).\n", __func__);
      return -EINVAL;
    }

  if (bt_hal_hfp_ops && bt_hal_hfp_ops->audio_connect)
    {
      ret = bt_hal_hfp_ops->audio_connect(&bt_acl_state->bt_target_addr, g_bt_hfp_state.bt_hfp_audio_handle, false);
      g_bt_hfp_state.bt_hfp_audio_connection = BT_DISCONNECTING;
    }
  else
    {
      _err("%s [BT][HFP] Disconnect audio failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_hfp_send_at_command
 *
 * Description:
 *   Send AT command via HFP to peer device
 *
 ****************************************************************************/

int bt_hfp_send_at_command(struct bt_acl_state_s *bt_acl_state, char *at_cmd_str)
{
  int ret = BT_SUCCESS;
  struct bt_hal_hfp_ops_s *bt_hal_hfp_ops = g_bt_hfp_state.bt_hal_hfp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][HFP] Send AT command failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (!at_cmd_str)
    {
      _err("%s [BT][HFP] Send AT command failed(Command is not set).\n", __func__);
      return BT_FAIL;
    }

  if (bt_hal_hfp_ops && bt_hal_hfp_ops->send_at_command)
    {
      ret = bt_hal_hfp_ops->send_at_command(&bt_acl_state->bt_target_addr, at_cmd_str, g_bt_hfp_state.bt_hfp_handle);
    }
  else
    {
      _err("%s [BT][HFP] Send AT command failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_hfp_press_button
 *
 * Description:
 *   Set pressing button for HFP profile
 *
 ****************************************************************************/

int bt_hfp_press_button(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_hfp_ops_s *bt_hal_hfp_ops = g_bt_hfp_state.bt_hal_hfp_ops;

  if (bt_hal_hfp_ops && bt_hal_hfp_ops->press_button)
    {
      bt_hal_hfp_ops->press_button(&bt_acl_state->bt_target_addr, g_bt_hfp_state.bt_hfp_handle);
    }
  else
    {
      _err("%s [BT][HFP] Press button failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

/****************************************************************************
 * Name: bt_hfp_set_feature
 *
 * Description:
 *   Set feature for HFP profile
 *
 ****************************************************************************/

int bt_hfp_set_feature(BT_HFP_HF_FEATURE_FLAG flag)
{
  int ret = BT_SUCCESS;
  struct bt_hal_hfp_ops_s *bt_hal_hfp_ops = g_bt_hfp_state.bt_hal_hfp_ops;

  if (bt_hal_hfp_ops && bt_hal_hfp_ops->set_hf_feature)
    {
      bt_hal_hfp_ops->set_hf_feature(flag);
      g_bt_hfp_state.bt_hfp_supported_feature = flag;
    }
  else
    {
      _err("%s [BT][HFP] Set default supported feature failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}


/****************************************************************************
 * Name: bt_hfp_register_cb
 *
 * Description:
 *   Register connection callback for application.
 *
 ****************************************************************************/

int bt_hfp_register_cb(struct bt_hfp_ops_s *bt_hfp_ops)
{
  if (!bt_hfp_ops)
    {
      _err("%s [BT][HFP] Set application callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_hfp_state.bt_hfp_ops = bt_hfp_ops;
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_hfp_register_hal
 *
 * Description:
 *   Register HFP HAL interface.
 *
 ****************************************************************************/

int bt_hfp_register_hal(struct bt_hal_hfp_ops_s *bt_hal_hfp_ops)
{
  if (!bt_hal_hfp_ops)
    {
      _err("%s [BT][HFP] Set HAL callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_hfp_state.bt_hal_hfp_ops = bt_hal_hfp_ops;

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_spp_event_handler
 *
 * Description:
 *   Handler of HFP event.
 *   Receive HFP event from HAL and dispatch event for application.
 *
 ****************************************************************************/

int bt_hfp_event_handler(struct bt_event_t *bt_event)
{
  switch (bt_event->event_id)
    {
      case BT_HFP_EVENT_CMD_STATUS:
        return event_cmd_status((struct bt_event_cmd_stat_t *) bt_event);

      case BT_HFP_EVENT_HF_CONNECT:
        return event_hf_connect((struct bt_hfp_event_connect_t *) bt_event);

      case BT_HFP_EVENT_HF_DISCONNECT:
        return event_hf_disconnect((struct bt_hfp_event_connect_t *) bt_event);

      case BT_HFP_EVENT_AUDIO_CONNECT:
        return event_audio_connect((struct bt_hfp_event_connect_t *) bt_event);

      case BT_HFP_EVENT_AUDIO_DISCONNECT:
        return event_audio_disconnect((struct bt_hfp_event_connect_t *) bt_event);

      case BT_HFP_EVENT_AG_FEATURE_RESP:
        return event_ag_feature_resp((struct bt_hfp_event_ag_feature_t *) bt_event);

      case BT_HFP_EVENT_AT_CMD_RESP:
        return event_at_cmd_resp((struct bt_hfp_event_at_cmd_t *) bt_event);

      default:
        break;
    }
  return BT_SUCCESS;
}


