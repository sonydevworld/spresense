/****************************************************************************
 * modules/bluetooth/bluetooth_le_gatt.c
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
#include <bluetooth/ble_gatt.h>
#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ble_gatt_state_s g_ble_gatt_state =
{
  .num = 0,
  .services = {{0}}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct ble_gatt_char_s *ble_search_characteristic(uint16_t serv_handle, uint16_t char_handle)
{
  struct ble_gatt_service_s *ble_gatt_service = NULL;
  struct ble_gatt_char_s *ble_gatt_char = NULL;
  int n, m;

  if (serv_handle != BLE_GATT_INVALID_SERVICE_HANDLE)
    {
      /* If HAL return Service handle ID */

      /* Seach service from GATT context */

      for (n = 0; n < g_ble_gatt_state.num; n ++)
        {
          if (g_ble_gatt_state.services[n].handle == serv_handle)
            {
              ble_gatt_service = &g_ble_gatt_state.services[n];
              break;
            }
        }

      if (ble_gatt_service)
        {
          /*Search characteristic from service */

          for (n = 0; n < ble_gatt_service->num; n ++)
            {
              if (ble_gatt_service->chars[n]->handle == char_handle)
                {
                  ble_gatt_char = ble_gatt_service->chars[n];
                  break;
                }
            }
        }
    }
  else
    {
      /* If HAL not return Service handle ID */

      for (m = 0; m < g_ble_gatt_state.num; m++)
        {
          ble_gatt_service = &g_ble_gatt_state.services[m];

          for (n = 0; n < ble_gatt_service->num; n ++)
            {
              if (ble_gatt_service->chars[n]->handle == char_handle)
                {
                  ble_gatt_char = ble_gatt_service->chars[n];
                  break;
                }
            }
        }
    }

  return ble_gatt_char;
}

static int event_write_req(struct ble_gatt_event_write_req_t *write_req_evt)
{
  int ret = BT_SUCCESS;
  struct ble_gatt_char_s *ble_gatt_char;
  struct ble_gatt_peripheral_ops_s *ble_gatt_peripheral_ops;
  BLE_CHAR_VALUE  *value;

  /* Search characteristic */

  ble_gatt_char = ble_search_characteristic(write_req_evt->serv_handle, write_req_evt->char_handle);

  if (!ble_gatt_char)
    {
      _err("%s [BLE][GATT] characteristic search failed(Not found).\n", __func__);
      return BT_FAIL;
    }

  /* Copy write data */

  value = &ble_gatt_char->value;

  memcpy(value->data, write_req_evt->data, write_req_evt->length);

  value->length = write_req_evt->length;

  /* Callback to application with updated characteristic */

  ble_gatt_peripheral_ops = ble_gatt_char->ble_gatt_peripheral_ops;

  if (ble_gatt_peripheral_ops && ble_gatt_peripheral_ops->write)
    {
      ble_gatt_peripheral_ops->write(ble_gatt_char);
    }
  else
    {
      _err("%s [BLE][GATT] Write request event callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

static int event_read_req(struct ble_gatt_event_read_req_t *read_req_evt)
{
  int ret = BT_SUCCESS;
  struct ble_gatt_char_s *ble_gatt_char;
  struct ble_gatt_peripheral_ops_s *ble_gatt_peripheral_ops;

  /* Search characteristic */

  ble_gatt_char = ble_search_characteristic(read_req_evt->serv_handle, read_req_evt->char_handle);

  if (!ble_gatt_char)
    {
      _err("%s [BLE][GATT] characteristic search failed(Not found).\n", __func__);
      return BT_FAIL;
    }

  /* Callback to application with searched characteristic */

  ble_gatt_peripheral_ops = ble_gatt_char->ble_gatt_peripheral_ops;

  if (ble_gatt_peripheral_ops && ble_gatt_peripheral_ops->read)
    {
      ble_gatt_peripheral_ops->read(ble_gatt_char);
    }
  else
    {
      _err("%s [BLE][GATT] Read request event callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

static int event_notify_req(struct ble_gatt_event_notify_req_t *notify_req_evt)
{
  int ret = BT_SUCCESS;
  struct ble_gatt_char_s *ble_gatt_char;
  struct ble_gatt_peripheral_ops_s *ble_gatt_peripheral_ops;

  /* Search characteristic */

  ble_gatt_char = ble_search_characteristic(notify_req_evt->serv_handle, notify_req_evt->char_handle);

  if (!ble_gatt_char)
    {
      _err("%s [BLE][GATT] characteristic search failed(Not found).\n", __func__);
      return BT_FAIL;
    }

  /* Callback to application with searched characteristic */

  ble_gatt_peripheral_ops = ble_gatt_char->ble_gatt_peripheral_ops;

  if (ble_gatt_peripheral_ops && ble_gatt_peripheral_ops->notify)
    {
      ble_gatt_peripheral_ops->notify(ble_gatt_char, notify_req_evt->enable);
    }
  else
    {
      _err("%s [BLE][GATT] Notify request event callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

static int event_write_rsp(struct ble_gatt_event_write_rsp_t *write_rsp_evt)
{
  int ret = BT_SUCCESS;
  struct ble_gatt_char_s *ble_gatt_char = NULL;
  struct ble_gatt_central_ops_s *ble_gatt_central_ops = NULL;

  /* Search characteristic */

  ble_gatt_char = ble_search_characteristic(write_rsp_evt->serv_handle, write_rsp_evt->char_handle);

  if (!ble_gatt_char)
    {
      _err("%s [BLE][GATT] characteristic search failed(Not found).\n", __func__);
      return BT_FAIL;
    }

  ble_gatt_char->status = write_rsp_evt->status;
  ble_gatt_char->handle = write_rsp_evt->char_handle;

  /* Callback to application with updated characteristic */

  ble_gatt_central_ops = ble_gatt_char->ble_gatt_central_ops;

  if (ble_gatt_central_ops && ble_gatt_central_ops->write)
    {
      ble_gatt_central_ops->write(ble_gatt_char);
    }
  else
    {
      _err("%s [BLE][GATT] Write response event callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

static int event_read_rsp(struct ble_gatt_event_read_rsp_t *read_rsp_evt)
{
  int ret = BT_SUCCESS;
  struct ble_gatt_char_s *ble_gatt_char = NULL;
  struct ble_gatt_central_ops_s *ble_gatt_central_ops = NULL;
  BLE_CHAR_VALUE  *value = NULL;

  /* Search characteristic */

  ble_gatt_char = ble_search_characteristic(read_rsp_evt->serv_handle, read_rsp_evt->char_handle);

  if (!ble_gatt_char)
    {
      _err("%s [BLE][GATT] characteristic search failed(Not found).\n", __func__);
      return BT_FAIL;
    }

  ble_gatt_char->handle = read_rsp_evt->char_handle;

  /* Copy write data */

  value = &ble_gatt_char->value;

  memcpy(value->data, read_rsp_evt->data, read_rsp_evt->length);

  value->length = read_rsp_evt->length;

  /* Callback to application with searched characteristic */

  ble_gatt_central_ops = ble_gatt_char->ble_gatt_central_ops;

  if (ble_gatt_central_ops && ble_gatt_central_ops->read)
    {
      ble_gatt_central_ops->read(ble_gatt_char);
    }
  else
    {
      _err("%s [BLE][GATT] Read response event callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

static int event_db_discovery(struct ble_gatt_event_db_discovery_t *db_disc_evt)
{
  int ret = BT_SUCCESS;
  /* TODO: gatt database discovery can't look up central_ops by ble_search_characteristic(),
   * because the peer device service/characteristic database has not been built yet while this
   * callback is calling. We should consider where to store struct ble_gatt_central_ops.
   */
  struct ble_gatt_central_ops_s *ble_gatt_central_ops = g_ble_gatt_state.ble_gatt_central_ops;

  /* Callback to application */

  if (ble_gatt_central_ops && ble_gatt_central_ops->database_discovery)
    {
      ble_gatt_central_ops->database_discovery(db_disc_evt);
    }
  else
    {
      _err("%s [BLE][GATT] Db discovery event callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_gatt_is_supported
 *
 * Description:
 *   Get Bluetooth Low Ennergy GATT support or not support
 *
 ****************************************************************************/

bool ble_gatt_is_supported(void)
{
  /* If HAL interface was registered, return true. */

  return !(!g_ble_gatt_state.ble_hal_gatt_ops);
}

/****************************************************************************
 * Name: ble_gatt_init
 *
 * Description:
 *   Set BLE instance into GATT module.
 *
 ****************************************************************************/

void ble_gatt_init(struct ble_state_s *ble_state)
{
  g_ble_gatt_state.ble_state = ble_state;
}

/****************************************************************************
 * Name: ble_create_service
 *
 * Description:
 *   BLE Create GATT Service
 *   Create GATT Service instance and return instance pointer via *service.
 *
 ****************************************************************************/

int ble_create_service(struct ble_gatt_service_s **service)
{
  if (g_ble_gatt_state.num < BLE_MAX_SERVICES)
    {
      *service = &g_ble_gatt_state.services[g_ble_gatt_state.num];
      g_ble_gatt_state.num ++;
    }
  else
    {
      _err("%s [BLE][GATT] BLE create service failed(max services reached).\n", __func__);
      return BT_FAIL;
    }
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_register_servce
 *
 * Description:
 *   BLE Register GATT Service
 *   Register GATT Service to HAL.
 *
 ****************************************************************************/

int ble_register_servce(struct ble_gatt_service_s *service)
{
  int ret = BT_SUCCESS;
  int n;
  struct ble_hal_gatts_ops_s *ble_hal_gatts_ops = &(g_ble_gatt_state.ble_hal_gatt_ops->gatts);

  if (!service)
    {
      _err("%s [BLE][GATT] BLE register service failed(service not created).\n", __func__);
      return BT_FAIL;
    }

  if (service->handle)
    {
      _err("%s [BLE][GATT] BLE register service failed(service already registered).\n", __func__);
      return BT_FAIL;
    }

  if (ble_hal_gatts_ops && ble_hal_gatts_ops->addService &&
      ble_hal_gatts_ops->addChar)
    {
      ret = ble_hal_gatts_ops->addService(service);

      if (ret != BT_SUCCESS)
        {
          _err("%s [BLE][GATT] Register service failed(Cannot get service handle).\n", __func__);
          return ret;
        }

      for (n = 0; n < service->num; n ++)
        {
          ret = ble_hal_gatts_ops->addChar(service->handle, service->chars[n]);

          if (ret != BT_SUCCESS)
            {
              _err("%s [BLE][GATT] Register service failed(Cannot add characteristic).\n", __func__);
              return ret;
            }
        }
    }
  else
    {
      _err("%s [BLE][GATT] Register service failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_add_characteristic
 *
 * Description:
 *   BLE add Characteristic to service
 *
 ****************************************************************************/

int ble_add_characteristic(struct ble_gatt_service_s *service, struct ble_gatt_char_s *charc)
{
  if (!service)
    {
      _err("%s [BLE][GATT] BLE add characteristic failed(service not created).\n", __func__);
      return BT_FAIL;
    }

  if (service->num < BLE_MAX_CHARACTERISTICS)
    {
      service->chars[service->num] = charc;
      service->num ++;
    }
  else
    {
      _err("%s [BLE][GATT] BLE create service failed(max services reached).\n", __func__);
      return BT_FAIL;
    }
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_characteristic_notify
 *
 * Description:
 *   BLE Notify Characteristic value
 *   Notify characteristic value to Central (For Peripheral role)
 *
 ****************************************************************************/

int ble_characteristic_notify(struct ble_gatt_char_s *charc, uint8_t *data, int len)
{
  int ret = BT_SUCCESS;
  struct ble_hal_gatts_ops_s *ble_hal_gatts_ops = &(g_ble_gatt_state.ble_hal_gatt_ops->gatts);
  BLE_CHAR_VALUE *value;

  if (!charc)
    {
      _err("%s [BLE][GATT] BLE notify failed(characteristic not created).\n", __func__);
      return BT_FAIL;
    }

  value = &charc->value;

  if (BLE_MAX_CHAR_SIZE < len)
    {
      _err("%s [BLE][GATT] BLE notify failed(value size is too big).\n", __func__);
      return BT_FAIL;
    }

  if (ble_hal_gatts_ops && ble_hal_gatts_ops->notify)
    {
      memcpy(value->data, data, len);

      value->length = len;

      ret = ble_hal_gatts_ops->notify(charc, g_ble_gatt_state.ble_state->ble_connect_handle);
    }
  else
    {
      _err("%s [BLE][GATT] Notify failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_characteristic_read
 *
 * Description:
 *   BLE Read Characteristic value
 *   Send read characteristic request to peripheral (For Central role)
 *
 ****************************************************************************/

int ble_characteristic_read(struct ble_gatt_char_s *charc)
{
  int ret = BT_SUCCESS;
  struct ble_hal_gattc_ops_s *ble_hal_gattc_ops = &(g_ble_gatt_state.ble_hal_gatt_ops->gattc);

  if (!charc)
    {
      _err("%s [BLE][GATT] BLE characteristic read failed.\n", __func__);
      return BT_FAIL;
    }

  if (ble_hal_gattc_ops && ble_hal_gattc_ops->read)
    {
      ret = ble_hal_gattc_ops->read(charc, g_ble_gatt_state.ble_state->ble_connect_handle);
    }
  else
    {
      _err("%s [BLE][GATT] read characteristic failed.\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_characteristic_write
 *
 * Description:
 *   BLE Write Characteristic value
 *   Send write characteristic request to peripheral (For Central role)
 *
 ****************************************************************************/

int ble_characteristic_write(struct ble_gatt_char_s *charc, uint8_t *data, int len)
{
  int ret = BT_SUCCESS;
  struct ble_hal_gattc_ops_s *ble_hal_gattc_ops = &(g_ble_gatt_state.ble_hal_gatt_ops->gattc);

  if (!charc)
    {
      _err("%s [BLE][GATT] BLE characteristic write failed.\n", __func__);
      return BT_FAIL;
    }

  if (ble_hal_gattc_ops && ble_hal_gattc_ops->write)
    {
      ret = ble_hal_gattc_ops->write(charc, g_ble_gatt_state.ble_state->ble_connect_handle);
    }
  else
    {
      _err("%s [BLE][GATT] write characteristic failed.\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_start_db_discovery
 *
 * Description:
 *   BLE start database discovery
 *   Send database discovery request to peripheral (For Central role)
 *
 ****************************************************************************/

int ble_start_db_discovery(uint16_t conn_handle)
{
  int ret = BT_SUCCESS;
  struct ble_hal_gattc_ops_s *ble_hal_gattc_ops = &(g_ble_gatt_state.ble_hal_gatt_ops->gattc);

  if (ble_hal_gattc_ops && ble_hal_gattc_ops->startDbDiscovery)
    {
      ret = ble_hal_gattc_ops->startDbDiscovery(conn_handle);
    }
  else
    {
      _err("%s [BLE][GATT] start db discovery failed.\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_continue_db_discovery
 *
 * Description:
 *   BLE continue database discovery
 *   Send continue database discovery request to peripheral (For Central role)
 *
 ****************************************************************************/

int ble_continue_db_discovery(uint16_t start_handle, uint16_t conn_handle)
{
  int ret = BT_SUCCESS;
  struct ble_hal_gattc_ops_s *ble_hal_gattc_ops = &(g_ble_gatt_state.ble_hal_gatt_ops->gattc);

  if (ble_hal_gattc_ops && ble_hal_gattc_ops->continueDbDiscovery)
    {
      ret = ble_hal_gattc_ops->continueDbDiscovery(start_handle, conn_handle);
    }
  else
    {
      _err("%s [BLE][GATT] continue db discovery failed.\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: ble_gatt_register_hal
 *
 * Description:
 *   Bluetooth LE GATT function HAL register
 *
 ****************************************************************************/

int ble_gatt_register_hal(struct ble_hal_gatt_ops_s *ble_hal_gatt_ops)
{
  if (!ble_hal_gatt_ops)
    {
      _err("%s [BLE][GATT] Set HAL callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_ble_gatt_state.ble_hal_gatt_ops = ble_hal_gatt_ops;

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_gatt_event_handler
 *
 * Description:
 *   BLE GATT event handler
 *   HAL should call this function if receive BLE GATT event(@ref BLE_GATT_EVENT_ID).
 *
 ****************************************************************************/

int ble_gatt_event_handler(struct bt_event_t *bt_event)
{
  switch (bt_event->event_id)
    {
      case BLE_GATT_EVENT_WRITE_REQ:
        return event_write_req((struct ble_gatt_event_write_req_t *) bt_event);

      case BLE_GATT_EVENT_READ_REQ:
        return event_read_req((struct ble_gatt_event_read_req_t *) bt_event);

      case BLE_GATT_EVENT_NOTIFY_REQ:
        return event_notify_req((struct ble_gatt_event_notify_req_t *) bt_event);

      case BLE_GATT_EVENT_WRITE_RESP:
        return event_write_rsp((struct ble_gatt_event_write_rsp_t *) bt_event);

      case BLE_GATT_EVENT_READ_RESP:
        return event_read_rsp((struct ble_gatt_event_read_rsp_t *) bt_event);

      case BLE_GATT_EVENT_NOTIFY_RESP:
        /* Central role not supported yet */
        break;

      case BLE_GATT_EVENT_DB_DISCOVERY_COMPLETE:
        return event_db_discovery((struct ble_gatt_event_db_discovery_t *) bt_event);

      default:
        break;
    }
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_register_gatt_central_cb
 *
 * Description:
 *   Temporary I/F helps app hook central callbacks to framework stored state
 *   Should be removed after consider overall design about how to manage app state
 *
 ****************************************************************************/

int ble_register_gatt_central_cb(struct ble_gatt_central_ops_s *central_ops)
{
  if (!central_ops)
    {
      _err("%s [BLE][GATT] Set central callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_ble_gatt_state.ble_gatt_central_ops = central_ops;

  return BT_SUCCESS;
}
