/****************************************************************************
 * bluetooth_le_central/bluetooth_le_central_main.c
 *
 *   Copyright 2018, 2022 Sony Semiconductor Solutions Corporation
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

#include <stdio.h>
#include <string.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/hal/bt_if.h>
#include <assert.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


#define BLE_UUID_SDS_SERVICE_IN  0x3802
#define BLE_UUID_SDS_CHAR_IN     0x4a02

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* BLE common callbacks */

/* Connection status change */

static void on_le_connect_status_change(struct ble_state_s *ble_state,
                                        bool connected);

/* Device name change */

static void on_connected_device_nameresp(const char *name);

/* Result callback for scan */

static void on_scan_result(BT_ADDR addr, uint8_t *data, uint8_t len);

/* MTU size callback */

static void on_mtusize(uint16_t handle, uint16_t sz);

/* BLE GATT callbacks */

/* Write response */

static void on_write(struct ble_gatt_char_s *ble_gatt_char);

/* Read response */

static void on_read(struct ble_gatt_char_s *ble_gatt_char);

/* Receive notification */

static void on_notify(struct ble_gatt_char_s *ble_gatt_char);

/* DB discovery result */

static void on_db_discovery(struct ble_gatt_event_db_discovery_t *db_disc);

/* Descriptor write response */

static void on_descriptor_write(uint16_t conn_handle,
                                uint16_t handle,
                                int      result);

/* Descriptor read response */

static void on_descriptor_read(uint16_t conn_handle,
                               uint16_t handle,
                               uint8_t  *data,
                               uint16_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ble_common_ops_s ble_common_ops =
  {
    .connect_status_changed     = on_le_connect_status_change,
    .connected_device_name_resp = on_connected_device_nameresp,
    .scan_result                = on_scan_result,
    .mtusize                    = on_mtusize,
  };

static struct ble_gatt_central_ops_s ble_gatt_central_ops =
  {
    .write              = on_write,
    .read               = on_read,
    .notify             = on_notify,
    .database_discovery = on_db_discovery,
    .descriptor_write   = on_descriptor_write,
    .descriptor_read    = on_descriptor_read,
  };

static BT_ADDR local_addr               = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};

static char local_ble_name[BT_NAME_LEN] = "SONY_BLE";
static struct ble_state_s *s_ble_state = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ble_state_add_bt_addr(struct ble_state_s *state, BT_ADDR *addr)
{
  ASSERT(state);

  memcpy(&state->bt_target_addr, addr, sizeof(state->bt_target_addr));
}

static void on_le_connect_status_change(struct ble_state_s *ble_state,
                                      bool connected)
{
  BT_ADDR addr = ble_state->bt_target_addr;

  /* If receive connected status data, this function will call. */

  printf("[BLE_GATT] Connect status ADDR:%02X:%02X:%02X:%02X:%02X:%02X, status:%s\n",
          addr.address[5], addr.address[4], addr.address[3],
          addr.address[2], addr.address[1], addr.address[0],
          connected ? "Connected" : "Disconnected");

  s_ble_state = ble_state;
}

static void on_connected_device_nameresp(const char *name)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] Receive connected device name = %s\n", __func__, name);
}

static void on_scan_result(BT_ADDR addr, uint8_t *data, uint8_t len)
{
  const char target_device_name[] = "SONY-PERIPHERAL";
  struct ble_state_s state = {0};
  int ret = BT_SUCCESS;
  struct bt_eir_s eir;

  printf("[BLE] Scan result ADDR:%02X:%02X:%02X:%02X:%02X:%02X\n",
          addr.address[5], addr.address[4], addr.address[3], addr.address[2],
          addr.address[1], addr.address[0]);

  ret = ble_parse_advertising_data(BLE_AD_TYPE_COMPLETE_LOCAL_NAME, data, len, &eir);
  if (ret != BT_SUCCESS)
    {
      printf("This advertising data do not have device name.\n");
      return;
    }

  printf("[BLE] Scan device name: %s\n", eir.data);

  if (0 != strcmp((char *)eir.data, target_device_name))
    {
      printf("%s [BLE_GATT] BLE target_device_name %s ignored.\n", __func__, eir.data);
      goto bye;
    }

  ret = ble_cancel_scan();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE_GATT] Cancel scan failed. ret = %d\n", __func__, ret);
      goto bye;
    }

  ble_state_add_bt_addr(&state, &addr);

  ret = ble_connect(&state);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE_GATT] connect target device failed. ret = %d\n", __func__, ret);
      goto bye;
    }
  else
    {
      printf("%s [BLE_GATT] connect target device. ret = %d\n", __func__, ret);
    }

bye:
  return;
}

static void on_mtusize(uint16_t handle, uint16_t sz)
{
  printf("mtusize = %d\n", sz);
  ble_start_db_discovery(s_ble_state->ble_connect_handle);
}

static void on_write(struct ble_gatt_char_s *ble_gatt_char)
{
  BLE_CHAR_VALUE *value = &ble_gatt_char->value;

  printf("%s [BLE] data[0] = 0x%02X, Length = %d\n", __func__, value->data[0], value->length);
}

static void on_read(struct ble_gatt_char_s *ble_gatt_char)
{
  int i;

  printf("%s [BLE] \n", __func__);
  printf("handle : 0x%04x\n", ble_gatt_char->handle);
  printf("value len : %d\n",  ble_gatt_char->value.length);
  for (i = 0; i < ble_gatt_char->value.length; i++)
    {
      printf("%02x ", ble_gatt_char->value.data[i]);
    }

  printf("\n");
}

static void on_notify(struct ble_gatt_char_s *ble_gatt_char)
{
  int i;

  printf("%s [BLE] \n", __func__);
  printf("handle : 0x%04x\n", ble_gatt_char->handle);
  printf("value len : %d\n",  ble_gatt_char->value.length);
  for (i = 0; i < ble_gatt_char->value.length; i++)
    {
      printf("%02x ", ble_gatt_char->value.data[i]);
    }

  printf("\n");
}

void on_db_discovery(struct ble_gatt_event_db_discovery_t *db_disc)
{
  int i;
  int j;
  struct ble_gattc_db_discovery_s *db;
  struct ble_gattc_db_disc_srv_s  *srv;
  struct ble_gattc_db_disc_char_s *chr;
  db = &db_disc->params.db_discovery;

  srv = &db->services[0];
  for (i = 0; i < db->srv_count; i++, srv++)
    {
      printf("=== SRV[%d] ===\n", i);

      chr = &srv->characteristics[0];

      for (j = 0; j < srv->char_count; j++, chr++)
        {
          printf("   === CHR[%d] ===\n", j);
          printf("      decl  handle : 0x%04x\n", chr->characteristic.char_declhandle);
          printf("      value handle : 0x%04x\n", chr->characteristic.char_valhandle);
          printf("      uuid         : 0x%04x\n", chr->characteristic.char_valuuid.value.alias.uuidAlias);
          printf("      cccd  handle : 0x%04x\n", chr->cccd_handle);
          printf("\n");
        }

      printf("\n");
    }

  /* The end_handle of the last service is 0xFFFF.
   * So, if end_handle is not 0xFFFF, the continuous service information exists.
   */

  if (db_disc->state.end_handle != 0xFFFF)
    {
      ble_continue_db_discovery(s_ble_state->ble_connect_handle,
                                db_disc->state.end_handle + 1);
    }
}

static void on_descriptor_write(uint16_t conn_handle, uint16_t handle, int result)
{
  printf("%s [BLE] conn_handle = 0x%04x, handle = 0x%04x, result = %d\n",
         __func__, conn_handle, handle, result);
}

static void on_descriptor_read(uint16_t conn_handle,
                               uint16_t handle,
                               uint8_t  *data,
                               uint16_t len)
{
  int i;

  printf("%s [BLE] \n", __func__);

  printf("conn_handle : 0x%04x, handle : 0x%04x\n", conn_handle, handle);
  printf("len : %d\n",  len);
  for (i = 0; i < len; i++)
    {
      printf("%02x ", data[i]);
    }

  printf("\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * BLE_GATT_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret = 0;

  /* Initialize BT HAL */

  ret = bt_init();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Initialization failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Turn ON BT */

  ret = bt_enable();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Enabling failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Register BLE common callbacks */

  ret = ble_register_common_cb(&ble_common_ops);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Register common call back failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Register BLE gatt callbacks */

  ret = ble_register_gatt_central_cb(&ble_gatt_central_ops);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Register central call back failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE set name */

  ret = ble_set_name(local_ble_name);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Set name failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE set address */

  ret = ble_set_address(&local_addr);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Set address failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE enable */

  ret = ble_enable();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Enable failed. ret = %d\n", __func__, ret);
      goto error;
    }

  ret = ble_start_scan(false);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Start scan failed. ret = %d\n", __func__, ret);
      goto error;
    }

error:
  return ret;
}
