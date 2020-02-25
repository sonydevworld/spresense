/****************************************************************************
 * bluetooth_le_central/bluetooth_le_central_main.c
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

#include <stdio.h>
#include <string.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


#define BLE_UUID_SDS_SERVICE_IN  0x3802
#define BLE_UUID_SDS_CHAR_IN     0x4a02

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* BT common callbacks */

static void on_command_status(BT_CMD_STATUS status);                      /**< Command status */
static void on_pairing_complete(BT_ADDR addr, BT_PAIR_STATUS status);     /**< Pairing complete */
static void on_inquiry_result(BT_ADDR addr, char *name);                  /**< Inquiry data result */
static void on_inquiry_complete(void);                                    /**< Coplete inquiry */
static void on_connect_status_changed(struct bt_acl_state_s *bt_acl_state,
                                    bool connected, int status);          /**< Connection status change */
static void on_connected_device_name(const char *name);                   /**< Device name change */
static void on_bond_info(BT_ADDR addr);                                   /**< Bonding information */

/* BLE common callbacks */

static void on_le_connect_status_change(struct ble_state_s *ble_state,
                                      bool connected);                    /**< Connection status change */
static void on_connected_device_nameresp(const char *name);               /**< Device name change */
static void on_scan_result(BT_ADDR addr, char *dev_name);                 /**< Result callback for scan */

/* BLE GATT callbacks */

static void on_write(struct ble_gatt_char_s *ble_gatt_char);                /**< Write response */
static void on_read(struct ble_gatt_char_s *ble_gatt_char);                 /**< Read response */
static void on_notify(struct ble_gatt_char_s *ble_gatt_char);               /**< Notify response */
static void on_db_discovery(struct ble_gatt_event_db_discovery_t *db_disc); /**< Db discovery result */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_common_ops_s bt_common_ops =
  {
    .command_status         = on_command_status,
    .pairing_complete       = on_pairing_complete,
    .inquiry_result         = on_inquiry_result,
    .inquiry_complete       = on_inquiry_complete,
    .connect_status_changed = on_connect_status_changed,
    .connected_device_name  = on_connected_device_name,
    .bond_info              = on_bond_info
  };

static struct ble_common_ops_s ble_common_ops =
  {
    .connect_status_changed     = on_le_connect_status_change,
    .connected_device_name_resp = on_connected_device_nameresp,
    .scan_result                = on_scan_result
  };

static struct ble_gatt_central_ops_s ble_gatt_central_ops =
  {
    .write              = on_write,
    .read               = on_read,
    .notify             = on_notify,
    .database_discovery = on_db_discovery
  };

static BT_ADDR local_addr               = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};

static char local_bt_name[BT_NAME_LEN]  = "SONY_BT";
static char local_ble_name[BT_NAME_LEN] = "SONY_BLE";
static struct ble_state_s *s_ble_state = NULL;
#if 0
static struct ble_gatt_service_s *g_ble_gatt_service;

static BLE_UUID128 service_in_uuid = {{0xfb, 0x34, 0x9b, 0x5f,  \
                                       0x80, 0x00, 0x00, 0x80,  \
                                       0x00, 0x10, 0x00, 0x00,  \
                                       0x00, 0x00, 0x00, 0x00}};

static BLE_UUID128 char_in_uuid = {{0x00, 0x34, 0x9b, 0x5f,  \
                                    0x80, 0x00, 0x00, 0x80,  \
                                    0x00, 0x10, 0x00, 0x00,  \
                                    0x00, 0x00, 0x00, 0x00}};

static BLE_ATTR_PERM attr_param =
  {
    .readPerm  = BLE_SEC_MODE1LV2_NO_MITM_ENC,
    .writePerm = BLE_SEC_MODE1LV2_NO_MITM_ENC
  };

static uint8_t char_data[BLE_MAX_CHAR_SIZE];

static BLE_CHAR_VALUE char_value =
  {
    .length = BLE_MAX_CHAR_SIZE
  };

static BLE_CHAR_PROP char_property =
  {
    .read   = 1,
    .write  = 1,
    .notify = 1
  };

static struct ble_gatt_char_s g_ble_gatt_char =
  {
    .handle = 0,
    .ble_gatt_central_ops = &ble_gatt_central_ops
  };
#endif
/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ble_state_add_bt_addr(struct ble_state_s *state, BT_ADDR *addr)
{
  ASSERT(state);

  memcpy(&state->bt_target_addr, addr, sizeof(state->bt_target_addr));
}

static void on_command_status(BT_CMD_STATUS status)
{
  /* If receive command status event, this function will call. */

  printf("%s [BT] Command status = %d\n", __func__, status);
}

static void on_pairing_complete(BT_ADDR addr, BT_PAIR_STATUS status)
{
  /* If pairing task complete, this function will call.
   * Print receive event data.
   */

  printf("[BT] Pairing complete ADDR:%02X:%02X:%02X:%02X:%02X:%02X, status=%d\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5],
          status);
}

static void on_inquiry_result(BT_ADDR addr, char *name)
{
  /* If receive inquiry search result, this function will call. */

  printf("[BT] Inquiry result ADDR:%02X:%02X:%02X:%02X:%02X:%02X, name:%s\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5],
          name);
}

static void on_inquiry_complete(void)
{
  /* If receive inquiry complete event, this function will call. */

  printf("%s [BT] Inquiry complete\n", __func__);
}

static void on_connect_status_changed(struct bt_acl_state_s *bt_acl_state,
                                    bool connected, int status)
{
  /* If conection status changed, this function will call. */

  printf("%s [BT] Connect status changed\n", __func__);
}

static void on_connected_device_name(const char *name)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BT] Receive connected device name = %s\n", __func__, name);
}

static void on_bond_info(BT_ADDR addr)
{
  /* If new bonding is comming, this function will call.
   * Print new bonding information.
   */

  printf("[BLE_GATT] Bonding information ADDR:%02X:%02X:%02X:%02X:%02X:%02X\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5]);
}

static void on_le_connect_status_change(struct ble_state_s *ble_state,
                                      bool connected)
{
  int ret = BT_SUCCESS;
  BT_ADDR addr = ble_state->bt_target_addr;

  /* If receive connected status data, this function will call. */

  printf("[BLE_GATT] Connect status ADDR:%02X:%02X:%02X:%02X:%02X:%02X, status:%s\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5],
          connected ? "Connected" : "Disconnected");

  s_ble_state = ble_state;

  if (connected)
    {
      /* TODO BLE device needs to configure the MTU after connection,
       * wait for the finish of configuring.
       */

      usleep(500 * 1000);

      ret = ble_start_db_discovery(ble_state->ble_connect_handle);
      if (ret != BT_SUCCESS)
        {
          printf("%s [BLE_GATT] start db discovery failed. ret = %d\n", __func__, ret);
          return;
        }
      else
        {
          printf("%s [BLE_GATT] start db discovery. ret = %d\n", __func__, ret);
        }
    }
}

static void on_connected_device_nameresp(const char *name)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] Receive connected device name = %s\n", __func__, name);
}

static void on_scan_result(BT_ADDR addr, char *dev_name)
{
  const char target_device_name[] = "SONY-CENTRAL";

  struct ble_state_s state = {0};
  int ret = BT_SUCCESS;

  /* If receive connected device name data, this function will call. */
  printf("[BLE_GATT] Scan result ADDR:%02X:%02X:%02X:%02X:%02X:%02X, name:%s\n",
          addr.address[0], addr.address[1], addr.address[2], addr.address[3],
          addr.address[4], addr.address[5], dev_name);

  if (0 != strncmp(dev_name, target_device_name, sizeof(target_device_name)))
    {
      printf("%s [BLE_GATT] BLE target_device_name %s ignored.\n", __func__, dev_name);
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

static void on_write(struct ble_gatt_char_s *ble_gatt_char)
{
  BLE_CHAR_VALUE *value = &ble_gatt_char->value;
  uint8_t data[2] = {0x06, 0x14};
  int len = 2;

  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] data[0] = 0x%02X, Length = %d\n", __func__, value->data[0], value->length);

  /* After 2 seconds, send data to characteristic */

  sleep(2);

  ble_characteristic_notify(ble_gatt_char, data, len);
}

static void on_read(struct ble_gatt_char_s *ble_gatt_char)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] \n", __func__);
}

static void on_notify(struct ble_gatt_char_s *ble_gatt_char)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] \n", __func__);
}

void on_db_discovery(struct ble_gatt_event_db_discovery_t *db_disc)
{
  /* discovered services/characteristics in peer device
   * TODO: build local state for peer device services/characteristics
   * The following utilizes g_ble_gatt_state to store peer device state,
   * but this causes central/peripheral roles could not run at the same
   * time.
   *
   * ble_create_service(): create peer device services
   * ble_add_charateristic(): add peer device charateristics to service
   */
  (void) db_disc;

  printf("%s [BLE] \n", __func__);
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

  /* Register BT event callback function */

  ret = bt_register_common_cb(&bt_common_ops);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Register common call back failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Initialize BT HAL */

  ret = bt_init();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Initialization failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Set local device address */

  ret = bt_set_address(&local_addr);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Set local address failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Set local device name */

  ret = bt_set_name(local_bt_name);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Set local name failed. ret = %d\n", __func__, ret);
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

  ret = ble_start_scan();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Start scan failed. ret = %d\n", __func__, ret);
      goto error;
    }

error:
  return ret;
}
