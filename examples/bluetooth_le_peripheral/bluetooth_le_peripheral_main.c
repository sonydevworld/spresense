/****************************************************************************
 * bluetooth_le_peripheral/bluetooth_le_peripheral_main.c
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
#include <bluetooth/ble_gatt.h>

#include "system/readline.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLE_MAX_TX_DATA_SIZE     1024

#define BLE_UUID_SDS_SERVICE_IN  0x3802
#define BLE_UUID_SDS_CHAR_IN     0x4a02

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* BT common callbacks */

static void onCommandStatus(BT_CMD_STATUS status);                      /**< Command status */
static void onPairingComplete(BT_ADDR addr, BT_PAIR_STATUS status);     /**< Pairing complete */
static void onInquiryResult(BT_ADDR addr, char *name);                  /**< Inquiry data result */
static void onInquiryComplete(void);                                    /**< Coplete inquiry */
static void onConnectStatusChanged(struct bt_acl_state_s *bt_acl_state,
                                    bool connected, int status);        /**< Connection status change */
static void onConnectedDeviceName(const char *name);                    /**< Device name change */
static void onBondInfo(BT_ADDR addr);                                   /**< Bonding information */

/* BLE common callbacks */

static void onLeConnectStatusChanged(struct ble_state_s *ble_state,
                                      bool connected);                  /**< Connection status change */
static void onConnectedDeviceNameResp(const char *name);                /**< Device name change */
static void onScanResult(BT_ADDR addr, char *dev_name);                 /**< Result callback for scan */

/* BLE GATT callbacks */

static void onWrite(struct ble_gatt_char_s *ble_gatt_char);               /**< Write request */
static void onRead(struct ble_gatt_char_s *ble_gatt_char);                /**< Read request */
static void onNotify(struct ble_gatt_char_s *ble_gatt_char, bool enable); /**< Notify request */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_common_ops_s bt_common_ops =
  {
    .command_status         = onCommandStatus,
    .pairing_complete       = onPairingComplete,
    .inquiry_result         = onInquiryResult,
    .inquiry_complete       = onInquiryComplete,
    .connect_status_changed = onConnectStatusChanged,
    .connected_device_name  = onConnectedDeviceName,
    .bond_info              = onBondInfo
  };

static struct ble_common_ops_s ble_common_ops =
  {
    .connect_status_changed     = onLeConnectStatusChanged,
    .connected_device_name_resp = onConnectedDeviceNameResp,
    .scan_result                = onScanResult
  };

static struct ble_gatt_peripheral_ops_s ble_gatt_peripheral_ops =
  {
    .write  = onWrite,
    .read   = onRead,
    .notify = onNotify
  };

static BT_ADDR local_addr               = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};

static char local_bt_name[BT_NAME_LEN]  = "SONY_BT";
static char local_ble_name[BT_NAME_LEN] = "SONY_BLE";

static struct bt_acl_state_s *s_bt_acl_state = NULL;

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
    .ble_gatt_peripheral_ops = &ble_gatt_peripheral_ops
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void onCommandStatus(BT_CMD_STATUS status)
{
  /* If receive command status event, this function will call. */

  printf("%s [BT] Command status = %d\n", __func__, status);
}

static void onPairingComplete(BT_ADDR addr, BT_PAIR_STATUS status)
{
  /* If pairing task complete, this function will call.
   * Print receive event data.
   */

  printf("[BT] Pairing complete ADDR:%02X:%02X:%02X:%02X:%02X:%02X, status=%d\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5],
          status);
}

static void onInquiryResult(BT_ADDR addr, char *name)
{
  /* If receive inquiry search result, this function will call. */

  printf("[BT] Inquiry result ADDR:%02X:%02X:%02X:%02X:%02X:%02X, name:%s\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5],
          name);
}

static void onInquiryComplete(void)
{
  /* If receive inquiry complete event, this function will call. */

  printf("%s [BT] Inquiry complete\n", __func__);
}

static void onConnectStatusChanged(struct bt_acl_state_s *bt_acl_state,
                                    bool connected, int status)
{
  /* If ACL is connected, SPP can start connect */

  s_bt_acl_state = bt_acl_state;
}

static void onConnectedDeviceName(const char *name)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BT] Receive connected device name = %s\n", __func__, name);
}

static void onBondInfo(BT_ADDR addr)
{
  /* If new bonding is comming, this function will call.
   * Print new bonding information.
   */

  printf("[BLE_GATT] Bonding information ADDR:%02X:%02X:%02X:%02X:%02X:%02X\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5]);
}

static void onLeConnectStatusChanged(struct ble_state_s *ble_state,
                                      bool connected)
{
  BT_ADDR addr = ble_state->bt_target_addr;

  /* If receive connected status data, this function will call. */

  printf("[BLE_GATT] Connect status ADDR:%02X:%02X:%02X:%02X:%02X:%02X, status:%s\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5],
          connected ? "Connected" : "Disconnected");
}

static void onConnectedDeviceNameResp(const char *name)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] Receive connected device name = %s\n", __func__, name);
}

static void onScanResult(BT_ADDR addr, char *dev_name)
{
  /* If receive connected device name data, this function will call. */

  printf("[BLE_GATT] Scan result ADDR:%02X:%02X:%02X:%02X:%02X:%02X, name:%s\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5],
          dev_name);
}


static void onWrite(struct ble_gatt_char_s *ble_gatt_char)
{
  BLE_CHAR_VALUE *value = &ble_gatt_char->value;

  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] data[0] = 0x%02X, Length = %d\n", __func__, value->data[0], value->length);
}

static void onRead(struct ble_gatt_char_s *ble_gatt_char)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] \n", __func__);
}

static void onNotify(struct ble_gatt_char_s *ble_gatt_char, bool enable)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] \n", __func__);
}

static void ble_peripheral_exit(void)
{
  int ret;

  /* Turn OFF BT */

  ret = bt_disable();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] BT disable failed. ret = %d\n", __func__, ret);
    }

  /* Finalize BT */

  ret = bt_finalize();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] BT finalize failed. ret = %d\n", __func__, ret);
    }
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
  int len = 0;
  char buffer[BLE_MAX_TX_DATA_SIZE] = {0};
  BLE_UUID *s_uuid;
  BLE_UUID *c_uuid;

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

  /* BLE create GATT service instance */

  ret = ble_create_service(&g_ble_gatt_service);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Create GATT service failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Setup Service */

  /* Get Service UUID pointer */

  s_uuid = &g_ble_gatt_service->uuid;

  /* Setup Service UUID */

  s_uuid->type                  = BLE_UUID_TYPE_BASEALIAS_BTSIG;
  s_uuid->value.alias.uuidAlias = BLE_UUID_SDS_SERVICE_IN;
  memcpy(&s_uuid->value.alias.uuidBase, &service_in_uuid, sizeof(BLE_UUID128));

  /* Setup Characteristic */

  /* Get Characteristic UUID pointer */

  c_uuid = &g_ble_gatt_char.uuid;

  /* Setup Characteristic UUID */

  c_uuid->type =BLE_UUID_TYPE_BASEALIAS_VENDOR;
  c_uuid->value.alias.uuidAlias = BLE_UUID_SDS_CHAR_IN;
  memcpy(&c_uuid->value.alias.uuidBase, &char_in_uuid, sizeof(BLE_UUID128));

  /* Set data point */

  char_value.data = char_data;

  /* Setup Characteristic BLE_ATTR_PERM */

  memcpy(&char_value.attrPerm, &attr_param, sizeof(BLE_ATTR_PERM));

  /* Setup Characteristic BLE_CHAR_VALUE */

  memcpy(&g_ble_gatt_char.value, &char_value, sizeof(BLE_CHAR_VALUE));

  /* Setup Characteristic BLE_CHAR_PROP */

  memcpy(&g_ble_gatt_char.property, &char_property, sizeof(BLE_CHAR_PROP));

  /* BLE add GATT characteristic into service */

  ret = ble_add_characteristic(g_ble_gatt_service, &g_ble_gatt_char);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Add GATT characteristic failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE register GATT service */

  ret = ble_register_servce(g_ble_gatt_service);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Register GATT service failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE start advertise */

  ret = ble_start_advertise();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Start advertise failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Send Tx data by using readline */

  while(1)
    {
      printf("ble_peripheral>");
      fflush(stdout);

      len = readline(buffer, sizeof(buffer) - 1, stdin, stdout);

      if (s_bt_acl_state && s_bt_acl_state->bt_acl_connection == BT_CONNECTED)
        {
          ret = ble_characteristic_notify(&g_ble_gatt_char, (uint8_t *) buffer, len);
          if (ret != BT_SUCCESS)
            {
              printf("%s [BLE] Send data failed. ret = %d\n", __func__, ret);
            }
        }

      if (!strcmp(buffer, "quit\n"))
        {
          printf("Quit.\n");
          break;
        }
    }

  /* Quit this application */

  ble_peripheral_exit();

error:
  return ret;
}
