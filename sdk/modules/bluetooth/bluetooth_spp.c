/****************************************************************************
 * modules/bluetooth/bluetooth_spp.c
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
#include <bluetooth/bt_spp.h>
#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_spp_state_s g_bt_spp_state =
{
  .bt_spp_connection = BT_DISCONNECTED,
  .spp_uuid          = {{0x00, 0x00, 0x11, 0x01,
                         0x00, 0x00, 0x10, 0x00,
                         0x80, 0x00, 0x00, 0x80,
                         0x5f, 0x9b, 0x34, 0xfb}}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int event_connect(struct bt_spp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_spp_ops_s *bt_spp_ops = g_bt_spp_state.bt_spp_ops;

  g_bt_spp_state.bt_spp_connection = BT_CONNECTED;

  if (bt_spp_ops && bt_spp_ops->connect)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_spp_ops->connect(g_bt_spp_state.bt_acl_state);

      g_bt_spp_state.bt_spp_handle = event_connect->handle;
    }
  else
    {
      _err("%s [BT][SPP] SPP connect callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_disconnect(struct bt_spp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_spp_ops_s *bt_spp_ops = g_bt_spp_state.bt_spp_ops;

  g_bt_spp_state.bt_spp_connection = BT_DISCONNECTED;

  if (bt_spp_ops && bt_spp_ops->disconnect)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_spp_ops->disconnect(g_bt_spp_state.bt_acl_state);
    }
  else
    {
      _err("%s [BT][SPP] SPP disconnect callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_connect_fail(struct bt_spp_event_connect_t *event_connect)
{
  int ret = BT_SUCCESS;
  struct bt_spp_ops_s *bt_spp_ops = g_bt_spp_state.bt_spp_ops;

  if (bt_spp_ops && bt_spp_ops->connection_fail)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_spp_ops->connection_fail(g_bt_spp_state.bt_acl_state, event_connect->reason);
    }
  else
    {
      _err("%s [BT][SPP] SPP connection faill callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_receive_data(struct bt_spp_event_recv_data_t *event_rx)
{
  int ret = BT_SUCCESS;
  struct bt_spp_ops_s *bt_spp_ops = g_bt_spp_state.bt_spp_ops;

  if (bt_spp_ops && bt_spp_ops->receive_data)
    {
      /* Need to search ACL context by BT_ADDR for multipoint. Will be implement. */

      bt_spp_ops->receive_data(g_bt_spp_state.bt_acl_state, event_rx->data, event_rx->len);
    }
  else
    {
      _err("%s [BT][SPP] SPP connection faill callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_spp_is_supported
 *
 * Description:
 *   Return SPP support status by bool.
 *
 ****************************************************************************/

bool bt_spp_is_supported(void)
{
  /* If HAL interface was registered, return true. */

  return !(!g_bt_spp_state.bt_hal_spp_ops);
}

/****************************************************************************
 * Name: bt_spp_connect
 *
 * Description:
 *   Connect Serial port profile with BT_ADDR.
 *
 ****************************************************************************/

int bt_spp_connect(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_spp_ops_s *bt_hal_spp_ops = g_bt_spp_state.bt_hal_spp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][SPP] Connect failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_spp_state.bt_spp_connection != BT_DISCONNECTED)
    {
      _err("%s [BT][SPP] Connect failed(Not disconnected).\n", __func__);
      return -EINVAL;
    }

  if (bt_hal_spp_ops && bt_hal_spp_ops->setUuid)
    {
      ret = bt_hal_spp_ops->setUuid(&g_bt_spp_state.spp_uuid);

      if (ret != BT_SUCCESS)
        {
          _err("%s [BT][SPP] Set UUID failed(From HAL response %d).\n", __func__, ret);
        }
    }
  else
    {
      _err("%s [BT][SPP] Connect failed(HAL not implemented).\n", __func__);
    }

  if (bt_hal_spp_ops && bt_hal_spp_ops->connect)
    {
      ret = bt_hal_spp_ops->connect(&bt_acl_state->bt_target_addr, g_bt_spp_state.bt_spp_handle, true);
      g_bt_spp_state.bt_acl_state = bt_acl_state;
      g_bt_spp_state.bt_spp_connection = BT_CONNECTING;
    }
  else
    {
      _err("%s [BT][SPP] Connect failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_spp_disconnect
 *
 * Description:
 *   Disconnect Serial port profile with BT_ADDR.
 *
 ****************************************************************************/

int bt_spp_disconnect(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;
  struct bt_hal_spp_ops_s *bt_hal_spp_ops = g_bt_spp_state.bt_hal_spp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][SPP] Disconnect failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_spp_state.bt_spp_connection != BT_CONNECTED)
    {
      _err("%s [BT][SPP] Disconnect failed(Not connected).\n", __func__);
      return -EINVAL;
    }

  if (bt_hal_spp_ops && bt_hal_spp_ops->connect)
    {
      ret = bt_hal_spp_ops->connect(&bt_acl_state->bt_target_addr, g_bt_spp_state.bt_spp_handle, false);
      g_bt_spp_state.bt_spp_connection = BT_DISCONNECTING;
    }
  else
    {
      _err("%s [BT][SPP] Disconnect failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_spp_set_uuid
 *
 * Description:
 *   Set UUID for Serial port profile.
 *   If not set, use SPP default UUID will set.
 *
 ****************************************************************************/

int bt_spp_set_uuid(BT_UUID *uuid)
{
  int ret = BT_SUCCESS;
  struct bt_hal_spp_ops_s *bt_hal_spp_ops = g_bt_spp_state.bt_hal_spp_ops;

  if (!bt_hal_spp_ops || !bt_hal_spp_ops->setUuid)
    {
      _err("%s [BT][SPP] Set uuid(HAL not registered).\n", __func__);
      return -EINVAL;
    }

  ret = bt_hal_spp_ops->setUuid((uuid ? uuid : &g_bt_spp_state.spp_uuid));
  if (ret)
    {
      _err("%s [BT][SPP] Set uuid().\n", __func__);
      return BT_FAIL;
    }
  else
    {
      memcpy(&g_bt_spp_state.spp_uuid, uuid, BT_UUID128_LEN);
    }

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_spp_send_tx_data
 *
 * Description:
 *   Send Tx Data.
 *
 ****************************************************************************/

int bt_spp_send_tx_data(struct bt_acl_state_s *bt_acl_state, uint8_t *data, int len)
{
  int ret = BT_SUCCESS;
  struct bt_hal_spp_ops_s *bt_hal_spp_ops = g_bt_spp_state.bt_hal_spp_ops;

  if (!bt_acl_state->bt_acl_connection)
    {
      _err("%s [BT][SPP] Disconnect failed(ACL not connected).\n", __func__);
      return -EINVAL;
    }

  if (g_bt_spp_state.bt_spp_connection != BT_CONNECTED)
    {
      _err("%s [BT][SPP] Send Tx Data failed(Not connected).\n", __func__);
      return -EINVAL;
    }

  if (bt_hal_spp_ops && bt_hal_spp_ops->sendTxData)
    {
      ret = bt_hal_spp_ops->sendTxData(&bt_acl_state->bt_target_addr, data, len, g_bt_spp_state.bt_spp_handle);
    }
  else
    {
      _err("%s [BT][SPP] Send Tx Data failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_spp_register_cb
 *
 * Description:
 *   Register connection callback for application.
 *
 ****************************************************************************/

int bt_spp_register_cb(struct bt_spp_ops_s *bt_spp_ops)
{
  if (!bt_spp_ops)
    {
      _err("%s [BT][Common] Set application callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_spp_state.bt_spp_ops = bt_spp_ops;

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_spp_register_hal
 *
 * Description:
 *   Register SPP HAL interface.
 *
 ****************************************************************************/

int bt_spp_register_hal(struct bt_hal_spp_ops_s *bt_hal_spp_ops)
{
  if (!bt_hal_spp_ops)
    {
      _err("%s [BT][SPP] Set HAL callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_spp_state.bt_hal_spp_ops = bt_hal_spp_ops;

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_spp_event_handler
 *
 * Description:
 *   Handler of SPP event.
 *   Receive SPP event from HAL and dispatch event for application.
 *
 ****************************************************************************/

int bt_spp_event_handler(struct bt_event_t *bt_event)
{
  switch (bt_event->event_id)
    {
      case BT_SPP_EVENT_CONNECT:
        return event_connect((struct bt_spp_event_connect_t *) bt_event);

      case BT_SPP_EVENT_DISCONNECT:
        return event_disconnect((struct bt_spp_event_connect_t *) bt_event);

      case BT_SPP_EVENT_CONNECT_FAIL:
        return event_connect_fail((struct bt_spp_event_connect_t *) bt_event);

      case BT_SPP_EVENT_RX_DATA:
        return event_receive_data((struct bt_spp_event_recv_data_t *) bt_event);

      default:
        break;
    }
  return BT_SUCCESS;
}

