/****************************************************************************
 * modules/bluetooth/bluetooth_a2dp.c
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

#include <bluetooth/bt_a2dp.h>
#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup bt_defs Defines
 * @{
 */

/**
 *@name BT success code
 *@{
 */
#define BT_A2DP_SCHED_EXEC_MEDIA_DATA SCHED_FIFO
/** @} */

/**
 *@name BT success code
 *@{
 */
#define BT_A2DP_SCHED_THREAD_NAME "bt_a2dp_media"
/** @} */

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_BLUETOOTH_A2DP_USE_THREAD
/** Bluetooth A2DP application callbacks
 */
struct bt_a2dp_media_thread_s
{
  struct sched_param param;
  pthread_mutex_t mutex;
  pthread_attr_t attr;
  pthread_t thread;
};
#endif /* CONFIG_BLUETOOTH_A2DP_USE_THREAD */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_a2dp_state_s g_bt_a2dp_state =
{
  .bt_a2dp_connection = BT_DISCONNECTED
};

#ifdef CONFIG_BLUETOOTH_A2DP_USE_THREAD
static struct bt_a2dp_media_thread_s g_bt_a2dp_media_thread;
#endif /* CONFIG_BLUETOOTH_A2DP_USE_THREAD */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int event_cmd_status(struct bt_event_cmd_stat_t *cmd_stat_evt)
{
  int ret = BT_SUCCESS;
  struct bt_a2dp_ops_s *bt_a2dp_ops = g_bt_a2dp_state.bt_a2dp_ops;

  if (bt_a2dp_ops && bt_a2dp_ops->command_status)
    {
      bt_a2dp_ops->command_status(cmd_stat_evt->cmd_status);
    }
  else
    {
      _err("%s [BT][A2DP] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_connect(struct bt_a2dp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_a2dp_ops_s *bt_a2dp_ops = g_bt_a2dp_state.bt_a2dp_ops;

  g_bt_a2dp_state.bt_a2dp_connection = BT_CONNECTED;

  if (bt_a2dp_ops && bt_a2dp_ops->connect)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_a2dp_ops->connect(g_bt_a2dp_state.bt_acl_state, event_connect->codecInfo);

      g_bt_a2dp_state.bt_a2dp_handle = event_connect->handle;
    }
  else
    {
      _err("%s [BT][A2DP] A2DP connect callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_disconnect(struct bt_a2dp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_a2dp_ops_s *bt_a2dp_ops = g_bt_a2dp_state.bt_a2dp_ops;

  g_bt_a2dp_state.bt_a2dp_connection = BT_DISCONNECTED;

  if (bt_a2dp_ops && bt_a2dp_ops->disconnect)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_a2dp_ops->disconnect(g_bt_a2dp_state.bt_acl_state);
    }
  else
    {
      _err("%s [BT][A2DP] A2DP disconnect callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static pthread_addr_t receive_thread(pthread_addr_t arg)
{
  int ret = BT_SUCCESS;
  struct bt_a2dp_ops_s *bt_a2dp_ops = g_bt_a2dp_state.bt_a2dp_ops;
  struct bt_a2dp_event_recv_t *event_recv = (struct bt_a2dp_event_recv_t *) arg;

  if (bt_a2dp_ops && bt_a2dp_ops->receive_media_pkt)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_a2dp_ops->receive_media_pkt(g_bt_a2dp_state.bt_acl_state, event_recv->data, event_recv->len);
    }
  else
    {
      _err("%s [BT][A2DP] A2DP receive media packet callback failed(CB not registered).\n", __func__);
      return (pthread_addr_t) BT_FAIL;
    }
  return (pthread_addr_t) ret;
}

static int event_recv_data(struct bt_a2dp_event_recv_t *event_recv)
{
  int ret = BT_SUCCESS;

#ifdef CONFIG_BLUETOOTH_A2DP_USE_THREAD
  if (!g_bt_a2dp_media_thread.mutex.pid)
    {
      pthread_mutex_init(&g_bt_a2dp_media_thread.mutex, NULL);
      sched_getparam(0, &g_bt_a2dp_media_thread.param);
      pthread_attr_init(&g_bt_a2dp_media_thread.attr);
      pthread_attr_setschedpolicy(&g_bt_a2dp_media_thread.attr,
                                  BT_A2DP_SCHED_EXEC_MEDIA_DATA);
      pthread_attr_setschedparam(&g_bt_a2dp_media_thread.attr,
                                 &g_bt_a2dp_media_thread.param);
    }
  pthread_create(&g_bt_a2dp_media_thread.thread, &g_bt_a2dp_media_thread.attr,
                 receive_thread, (pthread_addr_t)event_recv);
  pthread_setname_np(g_bt_a2dp_media_thread.thread, BT_A2DP_SCHED_THREAD_NAME);
#else
  receive_thread((pthread_addr_t) event_recv);
#endif /* CONFIG_BLUETOOTH_A2DP_USE_THREAD */

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_a2dp_is_supported
 *
 * Description:
 *   Return A2DP support status by bool.
 *
 ****************************************************************************/

bool bt_a2dp_is_supported(void)
{
  /* If HAL interface was registered, return true. */

  return !(!g_bt_a2dp_state.bt_hal_a2dp_ops);
}

/****************************************************************************
 * Name: bt_a2dp_connect
 *
 * Description:
 *   Connect A2DP by Bluetooth Address.
 *
 ****************************************************************************/

int bt_a2dp_connect(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_a2dp_ops_s *bt_hal_a2dp_ops = g_bt_a2dp_state.bt_hal_a2dp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][A2DP] Connect failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_a2dp_state.bt_a2dp_connection != BT_DISCONNECTED)
    {
      _err("%s [BT][A2DP] Connect failed(Not disconnected).\n", __func__);
      return -EINVAL;
    }

  if (bt_hal_a2dp_ops && bt_hal_a2dp_ops->connect)
    {
      ret = bt_hal_a2dp_ops->connect(&bt_acl_state->bt_target_addr, g_bt_a2dp_state.bt_a2dp_handle, true);
      g_bt_a2dp_state.bt_acl_state = bt_acl_state;
      g_bt_a2dp_state.bt_a2dp_connection = BT_CONNECTING;
    }
  else
    {
      _err("%s [BT][A2DP] Connect failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_a2dp_connect
 *
 * Description:
 *   Connect A2DP by Bluetooth Address.
 *
 ****************************************************************************/

int bt_a2dp_disconnect(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_a2dp_ops_s *bt_hal_a2dp_ops = g_bt_a2dp_state.bt_hal_a2dp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][A2DP] Disconnect failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_a2dp_state.bt_a2dp_connection != BT_CONNECTED)
    {
      _err("%s [BT][A2DP] Disconnect failed(Not connected).\n", __func__);
      return -EINVAL;
    }

  if (bt_hal_a2dp_ops && bt_hal_a2dp_ops->connect)
    {
      ret = bt_hal_a2dp_ops->connect(&bt_acl_state->bt_target_addr, g_bt_a2dp_state.bt_a2dp_handle, false);
      g_bt_a2dp_state.bt_a2dp_connection = BT_DISCONNECTING;
    }
  else
    {
      _err("%s [BT][A2DP] Disconnect failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_a2dp_set_codec_capability
 *
 * Description:
 *   Set A2DP supported codecs by codec flag.
 *
 ****************************************************************************/

int bt_a2dp_set_codec_capability(BT_AUDIO_CODEC_INFO *codec_capabilities, uint8_t num)
{
  int ret = BT_SUCCESS;
  int n;
  struct bt_hal_a2dp_ops_s *bt_hal_a2dp_ops = g_bt_a2dp_state.bt_hal_a2dp_ops;

  for (n = 0; n < num; n ++)
    {
      ret = bt_hal_a2dp_ops->set_codec(&codec_capabilities[n]);
      if (ret != BT_SUCCESS)
        {
          _err("%s [BT][A2DP] Cannot set codec info index = %d. ret = %d\n", __func__, n, ret);
          goto error;
        }

      if (codec_capabilities[n].codecId == BT_A2DP_SINK_CODEC_AAC)
        {
          ret = bt_hal_a2dp_ops->aacEnable(true);
          if (ret != BT_SUCCESS)
            {
              _err("%s [BT][A2DP] Cannot set AAC codec enable. ret = %d\n", __func__, ret);
            }
        }
    }

error:
  return ret;
}

/****************************************************************************
 * Name: bt_a2dp_register_callback
 *
 * Description:
 *   Register A2DP connection callback for Application.
 *
 ****************************************************************************/

int bt_a2dp_register_callback(struct bt_a2dp_ops_s *bt_a2dp_ops)
{
  if (!bt_a2dp_ops)
    {
      _err("%s [BT][A2DP] Set application callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_a2dp_state.bt_a2dp_ops = bt_a2dp_ops;
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_a2dp_register_hal
 *
 * Description:
 *   Register A2DP HAl interface.
 *
 ****************************************************************************/

int bt_a2dp_register_hal(struct bt_hal_a2dp_ops_s *bt_hal_a2dp_ops)
{
  if (!bt_hal_a2dp_ops)
    {
      _err("%s [BT][A2DP] Set HAL callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_a2dp_state.bt_hal_a2dp_ops = bt_hal_a2dp_ops;
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_a2dp_event_handler
 *
 * Description:
 *   Handler of A2DP event.
 *   Receive A2DP event from HAL and dispatch event for application.
 *
 ****************************************************************************/

int bt_a2dp_event_handler(struct bt_event_t *bt_event)
{
  switch (bt_event->event_id)
    {
      case BT_A2DP_EVENT_CMD_STATUS:
        return event_cmd_status((struct bt_event_cmd_stat_t *) bt_event);

      case BT_A2DP_EVENT_CONNECT:
        return event_connect((struct bt_a2dp_event_connect_t *) bt_event);

      case BT_A2DP_EVENT_DISCONNECT:
        return event_disconnect((struct bt_a2dp_event_connect_t *) bt_event);

      case BT_A2DP_EVENT_MEDIA_PACKET:
        return event_recv_data((struct bt_a2dp_event_recv_t *) bt_event);

      default:
        break;
    }
  return BT_SUCCESS;
}

