/****************************************************************************
 * bluetooth_le_peripheral/bluetooth_le_peripheral_main.c
 *
 *   Copyright 2018, 2022, 2025 Sony Semiconductor Solutions Corporation
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
#include <stdlib.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/ble_util.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLE_MAX_TX_DATA_SIZE     1024

#define BLE_UUID_SDS_SERVICE_IN  0x3802
#define BLE_UUID_SDS_CHAR_IN     0x4a02

#define BONDINFO_FILENAME "/mnt/spif/BONDINFO"

#define CHAR_ACCESS_COUNT 10

#define CHAR_SIZE 20

#if CHAR_SIZE > 244
#  error "Characteritic size must not exceed max MTU size - 3 (= 244)."
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* BLE common callbacks */

/* Connection status change */

static void onLeConnectStatusChanged(struct ble_state_s *ble_state,
                                     bool connected, uint8_t reason);

/* Device name change */

static void onConnectedDeviceNameResp(const char *name);

/* Save bonding information */

static void onSaveBondInfo(int num, struct ble_bondinfo_s *bond);

/* Load bonding information */

static int onLoadBondInfo(int num, struct ble_bondinfo_s *bond);

/* Negotiated MTU size */

static void onMtuSize(uint16_t handle, uint16_t sz);

/* Encryption result */

static void onEncryptionResult(uint16_t, bool result);

/* BLE GATT callbacks */

/* Write request */

static void onWrite(struct ble_gatt_char_s *ble_gatt_char);

/* Read request */

static void onRead(struct ble_gatt_char_s *ble_gatt_char);

/* Notify request */

static void onNotify(struct ble_gatt_char_s *ble_gatt_char, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ble_common_ops_s ble_common_ops =
  {
    .connect_status_changed     = onLeConnectStatusChanged,
    .connected_device_name_resp = onConnectedDeviceNameResp,
    .save_bondinfo              = onSaveBondInfo,
    .load_bondinfo              = onLoadBondInfo,
    .mtusize                    = onMtuSize,
    .encryption_result          = onEncryptionResult,
  };

static struct ble_gatt_peripheral_ops_s ble_gatt_peripheral_ops =
  {
    .write  = onWrite,
    .read   = onRead,
    .notify = onNotify
  };

static BT_ADDR local_addr               = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};

static char local_ble_name[BT_NAME_LEN] = "SONY-PERIPHERAL";

static bool ble_is_connected = false;

static uint16_t ble_conn_handle;

static struct ble_gatt_service_s *g_ble_gatt_service;

static BLE_UUID128 service_in_uuid = {{0xfb, 0x34, 0x9b, 0x5f,  \
                                       0x80, 0x00, 0x00, 0x80,  \
                                       0x00, 0x10, 0x00, 0x00,  \
                                       0x00, 0x00, 0x00, 0x00}};

static BLE_UUID128 char_in_uuid = {{0xfb, 0x34, 0x9b, 0x5f,  \
                                    0x80, 0x00, 0x00, 0x80,  \
                                    0x00, 0x10, 0x00, 0x00,  \
                                    0x00, 0x00, 0x00, 0x00}};

static BLE_ATTR_PERM attr_param =
  {
    .readPerm  = BLE_SEC_MODE1LV2_NO_MITM_ENC,
    .writePerm = BLE_SEC_MODE1LV2_NO_MITM_ENC
  };

static uint8_t char_data[CHAR_SIZE];

static BLE_CHAR_VALUE char_value =
  {
    .length = CHAR_SIZE
  };

static BLE_CHAR_PROP char_property =
  {
    .read        = 1,
    .write       = 1,
    .writeWoResp = 1,
    .notify      = 1
  };

static struct ble_gatt_char_s g_ble_gatt_char =
  {
    .handle = 0,
    .ble_gatt_peripheral_ops = &ble_gatt_peripheral_ops
  };

static int g_ble_bonded_device_num;
static struct ble_cccd_s **g_cccd = NULL;

static int g_dbaccess_cnt;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void onLeConnectStatusChanged(struct ble_state_s *ble_state,
                                     bool connected, uint8_t cause)
{
  BT_ADDR addr = ble_state->bt_target_addr;

  /* If receive connected status data, this function will call. */

  printf("[BLE] Connect ADDR:%02X:%02X:%02X:%02X:%02X:%02X\n",
          addr.address[5], addr.address[4], addr.address[3],
          addr.address[2], addr.address[1], addr.address[0]);
  printf("[BLE] status:%s", connected ? "Connected" : "Disconnected");
  printf("  : Cause <%s> :\n",
         cause == BLESTAT_SUCCESS          ? "Success"                 :
         cause == BLESTAT_MEMCAP_EXCD      ? "Mem capacity exceed"     :
         cause == BLESTAT_CONNECT_TIMEOUT  ? "Connetion timeout"       :
         cause == BLESTAT_PEER_TERMINATED  ? "Central terminated"   :
         cause == BLESTAT_PEER_TERM_LOWRES ? "Central low resource" :
         cause == BLESTAT_PEER_TERM_POFF   ? "Central power off"    :
         cause == BLESTAT_TERMINATED       ? "Terminated by self"      :
         cause == BLESTAT_DEVICE_BUSY      ? "Device busy"             :
         cause == BLESTAT_PARAM_REJECTED   ? "Negotiation breakup"     :
         cause == BLESTAT_CONNECT_FAILED   ? "Connection failed"       :
                                             "Unspecified error"       );

  ble_conn_handle = ble_state->ble_connect_handle;
  ble_is_connected = connected;
}

static void onConnectedDeviceNameResp(const char *name)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] Receive connected device name = %s\n", __func__, name);
}

static void onSaveBondInfo(int num, struct ble_bondinfo_s *bond)
{
  int i;
  FILE *fp;
  int sz;

  /* In this example, save the parameter `num` and each members of
   * the parameter `bond` in order to the file.
   */

  fp = fopen(BONDINFO_FILENAME, "wb");
  if (fp == NULL)
    {
      printf("Error: could not create file %s\n", BONDINFO_FILENAME);
      return;
    }

  fwrite(&num, 1, sizeof(int), fp);

  for (i = 0; i < num; i++)
    {
      fwrite(&bond[i], 1, sizeof(struct ble_bondinfo_s), fp);

      /* Because only cccd is pointer member, save it individually. */

      sz = bond[i].cccd_num * sizeof(struct ble_cccd_s);
      fwrite(bond[i].cccd, 1, sz, fp);
    }

  fclose(fp);
}

static int onLoadBondInfo(int num, struct ble_bondinfo_s *bond)
{
  int i;
  FILE *fp;
  int stored_num;
  int sz;
  size_t ret;

  fp = fopen(BONDINFO_FILENAME, "rb");
  if (fp == NULL)
    {
      return 0;
    }

  ret = fread(&stored_num, 1, sizeof(int), fp);
  if (ret != sizeof(int))
    {
      printf("Error: could not load due to %s read error.\n",
             BONDINFO_FILENAME);
      fclose(fp);
      return 0;
    }

  g_ble_bonded_device_num = (stored_num < num) ? stored_num : num;
  sz = g_ble_bonded_device_num * sizeof(struct ble_cccd_s *);
  g_cccd = (struct ble_cccd_s **)malloc(sz);
  if (g_cccd == NULL)
    {
      printf("Error: could not load due to malloc error.\n");
      g_ble_bonded_device_num = 0;
    }

  for (i = 0; i < g_ble_bonded_device_num; i++)
    {
      ret = fread(&bond[i], 1, sizeof(struct ble_bondinfo_s), fp);
      if (ret != sizeof(struct ble_bondinfo_s))
        {
          printf("Error: could not load all data due to %s read error.\n",
                 BONDINFO_FILENAME);
          printf("The number of loaded device is %d\n", i);
          g_ble_bonded_device_num = i;
          break;
        }

      if (bond[i].cccd_num > 1)
        {
          printf("Error: could not load all data due to invalid data.\n");
          printf("cccd_num does not exceed the number of characteristics\n");
          printf("that is set by this application.\n");
          printf("The number of loaded device is %d\n", i);

          g_ble_bonded_device_num = i;
          break;
        }

      /* Because only cccd is pointer member, load it individually. */

      sz = bond[i].cccd_num * sizeof(struct ble_cccd_s);
      g_cccd[i] = (struct ble_cccd_s *)malloc(sz);

      if (g_cccd[i] == NULL)
        {
          printf("Error: could not load all data due to malloc error.");
          printf("The number of loaded device is %d\n", i);

          g_ble_bonded_device_num = i;
          break;
        }

      bond[i].cccd = g_cccd[i];
      ret = fread(bond[i].cccd, 1, sz, fp);
      if (ret != sz)
        {
          printf("Error: could not load all data due to %s read error.\n",
                 BONDINFO_FILENAME);
          printf("The number of loaded device is %d\n", i);
          g_ble_bonded_device_num = i;
          break;
        }
    }

  fclose(fp);

  return g_ble_bonded_device_num;
}

static void free_cccd(void)
{
  int i;

  if (g_cccd)
    {
      for (i = 0; i < g_ble_bonded_device_num; i++)
        {
          if (g_cccd[i])
            {
              free(g_cccd[i]);
            }
        }

      free(g_cccd);
      g_cccd = NULL;
    }
}

static void onMtuSize(uint16_t handle, uint16_t sz)
{
  printf("negotiated MTU size(connection handle = %d) : %d\n", handle, sz);
}

static void onEncryptionResult(uint16_t handle, bool result)
{
  printf("Encryption result(connection handle = %d) : %s\n",
         handle, (result) ? "Success" : "Fail");
}

static void onWrite(struct ble_gatt_char_s *ble_gatt_char)
{
  int i;
  char str[BLE_UUID_128BIT_STRING_BUFSIZE];

  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] start\n", __func__);
  printf("   handle : %d\n", ble_gatt_char->handle);
  bleutil_convert_uuid2str(&ble_gatt_char->uuid, str, BLE_UUID_128BIT_STRING_LENGTH);
  printf("   uuid : %s\n", str);
  printf("   value_len : %d\n", ble_gatt_char->value.length);
  printf("   value : ");
  for (i = 0; i < ble_gatt_char->value.length; i++)
    {
      printf("%02x ", ble_gatt_char->value.data[i]);
    }

  printf("\n");

  printf("%s [BLE] end\n", __func__);

  g_dbaccess_cnt++;
}

static void onRead(struct ble_gatt_char_s *ble_gatt_char)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] \n", __func__);
}

static void onNotify(struct ble_gatt_char_s *ble_gatt_char, bool enable)
{
  char str[BLE_UUID_128BIT_STRING_BUFSIZE];

  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] start \n", __func__);
  printf("   handle : %d\n", ble_gatt_char->handle);
  bleutil_convert_uuid2str(&ble_gatt_char->uuid, str, BLE_UUID_128BIT_STRING_LENGTH);
  printf("   uuid : %s\n", str);

  if (enable)
    {
      printf("   notification enabled\n");
    }
  else
    {
      printf("   notification disabled\n");
    }

  printf("%s [BLE] end \n", __func__);
}

static void ble_peripheral_exit(void)
{
  int ret;

  /* Wait for disconnection by BLE central. */

  while (ble_is_connected)
    {
      usleep(1); /* usleep for task dispatch */
    };

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

static uint16_t wait_connection(void)
{
  /* connection establishment is initiated by BLE central device.
   * So, BLE peripheral only waits for connection status to be "connected".
   */

  while (!ble_is_connected)
    {
      usleep(1); /* usleep for task dispatch */
    }

  return ble_conn_handle;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret = 0;
  BLE_UUID *s_uuid;
  BLE_UUID *c_uuid;
  uint16_t conn_handle;

  g_dbaccess_cnt = 0;

  /* Initialize BT HAL */

  ret = bt_init();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Initialization failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Register BLE common callbacks */

  ret = ble_register_common_cb(&ble_common_ops);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Register common call back failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Turn ON BT */

  ret = bt_enable();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Enabling failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Free memory that is allocated in onLoadBond() callback function. */

  free_cccd();

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

  c_uuid->type                  = BLE_UUID_TYPE_BASEALIAS_BTSIG;
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

  conn_handle = wait_connection();

  /* First, wait for BLE central to write characteristic CHAR_ACCESS_COUNT times. */

  while (g_dbaccess_cnt < CHAR_ACCESS_COUNT)
    {
      usleep(1); /* usleep for task dispatch */
    }

  /* Second, update characteristic CHAR_ACCESS_COUNT times every second.
   * The data is decremented with each update.
   */

  while(g_dbaccess_cnt--)
    {
      ret = ble_characteristic_notify(conn_handle,
                                      &g_ble_gatt_char,
                                      (uint8_t *)&g_dbaccess_cnt,
                                      sizeof(g_dbaccess_cnt));
      if (ret != BT_SUCCESS)
        {
          printf("%s [BLE] Send data failed. ret = %d\n", __func__, ret);
        }

      sleep(1);
    }

  /* Quit this application */

  ble_peripheral_exit();

error:
  return ret;
}
