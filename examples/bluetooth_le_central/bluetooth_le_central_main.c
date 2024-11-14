/****************************************************************************
 * bluetooth_le_central/bluetooth_le_central_main.c
 *
 *   Copyright 2018, 2022, 2024 Sony Semiconductor Solutions Corporation
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
#include <bluetooth/bt_common.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/ble_util.h>
#include <bluetooth/hal/bt_if.h>
#include <assert.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BONDINFO_FILENAME "/mnt/spif/BONDINFO"
#define RESULT_NOT_RECEIVE 1
#define CHAR_ACCESS_COUNT 10
#define INVALID_CONN_HANDLE 0xffff
#define NOTIFICATION_ENABLED 1
#define NOTIFICATION_DISABLED 0

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* BLE common callbacks */

/* Connection status change */

static void on_le_connect_status_change(struct ble_state_s *ble_state,
                                        bool connected, uint8_t reason);

/* Device name change */

static void on_connected_device_nameresp(const char *name);

/* Result callback for scan */

static void on_scan_result(BT_ADDR addr, uint8_t *data, uint8_t len);

/* MTU size callback */

static void on_mtusize(uint16_t handle, uint16_t sz);

/* Save bonding information */

static void on_savebond(int num, struct ble_bondinfo_s *bond);

/* Load bonding information */

static int on_loadbond(int num, struct ble_bondinfo_s *bond);

/* Encryption result */

static void encryption_result(uint16_t handle, bool result);

/* BLE GATT callbacks */

/* Write response */

static void on_write(uint16_t conn_handle, struct ble_gatt_char_s *ble_gatt_char);

/* Read response */

static void on_read(uint16_t conn_handle, struct ble_gatt_char_s *ble_gatt_char);

/* Receive notification */

static void on_notify(uint16_t conn_handle, struct ble_gatt_char_s *ble_gatt_char);

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
    .save_bondinfo              = on_savebond,
    .load_bondinfo              = on_loadbond,
    .encryption_result          = encryption_result,
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

static char local_ble_name[BT_NAME_LEN] = "SONY-CENTRAL";
static struct ble_state_s *s_ble_state = NULL;

static int g_ble_bonded_device_num;
static struct ble_cccd_s **g_cccd = NULL;

static bool g_target = false;
static BLE_UUID g_target_srv_uuid;
static BLE_UUID g_target_char_uuid;
static char g_target_device_name[BT_NAME_LEN] = "SONY-PERIPHERAL";

static uint8_t g_read_data[BLE_MAX_GATT_DATA_LEN];
static uint16_t g_read_datalen;
static int      g_charwr_result;
static int      g_charrd_result;
static int      g_descwr_result;
static int      g_descrd_result;

static struct ble_gattc_db_disc_char_s g_nrw_char;
static volatile int    g_discovered = false;
static int    g_notify_cnt = 0;
static int    g_dbaccess_cnt = 0;

static bool g_mtusz_exchg = false;
static int g_encrypted = -1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void on_le_connect_status_change(struct ble_state_s *ble_state,
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
         cause == BLESTAT_CONNECT_TIMEOUT  ? "Connection timeout"      :
         cause == BLESTAT_PEER_TERMINATED  ? "Peripheral terminated"   :
         cause == BLESTAT_PEER_TERM_LOWRES ? "Peripheral low resource" :
         cause == BLESTAT_PEER_TERM_POFF   ? "Peripheral power off"    :
         cause == BLESTAT_TERMINATED       ? "Terminated by self"      :
         cause == BLESTAT_DEVICE_BUSY      ? "Device busy"             :
         cause == BLESTAT_PARAM_REJECTED   ? "Negotiation breakup"     :
         cause == BLESTAT_CONNECT_FAILED   ? "Connection failed"       :
                                             "Unspecified error"       );

  s_ble_state = connected ? ble_state : NULL;
}

static void on_connected_device_nameresp(const char *name)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] Receive connected device name = %s\n", __func__, name);
}

static void on_scan_result(BT_ADDR addr, uint8_t *data, uint8_t len)
{
  struct ble_state_s state = {0};
  int ret = BT_SUCCESS;
  char devname[BT_EIR_LEN];

  devname[0] = '\0';
  printf("[BLE] Scan result ADDR:%02X:%02X:%02X:%02X:%02X:%02X (RSSI:%ddB)\n",
         addr.address[5], addr.address[4], addr.address[3],
         addr.address[2], addr.address[1], addr.address[0],
         bleutil_get_rssi(data, len));

  /* If peer device has the device name, print it. */

  if (bleutil_get_devicename(data, len, devname))
    {
      printf("[BLE] Scan device name: %s\n", devname);
    }

  if (g_target)
    {
      /* If target UUID is specified, display only peripheral devices
       * that transmit advertising with target UUID and
       * connect to the device with target UUID.
       */

      if (bleutil_find_srvc_uuid(&g_target_srv_uuid, data, len) == 0)
        {
          printf("Service UUID is not matched\n");
          return;
        }
    }
  else
    {
      if (strcmp((char *)devname, g_target_device_name) != 0)
        {
          printf("%s [BLE] DevName is not matched: %s expect: %s\n",
                 __func__, devname, g_target_device_name);
          goto bye;
        }
    }

  ret = ble_cancel_scan();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE_GATT] Cancel scan failed. ret = %d\n", __func__, ret);
      goto bye;
    }

  bleutil_add_btaddr(&state, &addr, bleutil_get_addrtype(data, len));

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

  g_mtusz_exchg = true;
}

static void encryption_result(uint16_t handle, bool result)
{
  printf("Encryption result(connection handle = %d) : %s\n",
         handle, (result) ? "Success" : "Fail");

  g_encrypted = result;
}

static void on_savebond(int num, struct ble_bondinfo_s *bond)
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

static int on_loadbond(int num, struct ble_bondinfo_s *bond)
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
          printf("Error: could not load all data due to %s read error.\n"
                 "The number of loaded device is %d\n", BONDINFO_FILENAME, i);
          g_ble_bonded_device_num = i;
          break;
        }

      if (bond[i].cccd_num > 1)
        {
          printf("Error: could not load all data due to invalid data.\n"
                 "       cccd_num does not exceed the number of characteristics\n"
                 "       that is set by this application.\n"
                 "       The number of loaded device is %d\n", i);

          g_ble_bonded_device_num = i;
          break;
        }

      /* Because only cccd is pointer member, load it individually. */

      sz = bond[i].cccd_num * sizeof(struct ble_cccd_s);
      g_cccd[i] = (struct ble_cccd_s *)malloc(sz);

      if (g_cccd[i] == NULL)
        {
          printf("Error: could not load all data due to malloc error.\n"
                 "       The number of loaded device is %d\n", i);

          g_ble_bonded_device_num = i;
          break;
        }

      bond[i].cccd = g_cccd[i];
      ret = fread(bond[i].cccd, 1, sz, fp);
      if (ret != sz)
        {
          printf("Error: could not load all data due to %s read error.\n"
                 "       The number of loaded device is %d\n",
                 BONDINFO_FILENAME, i);
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

static void on_write(uint16_t conn_handle, struct ble_gatt_char_s *ble_gatt_char)
{
  printf("%s [BLE] result = %d\n", __func__, ble_gatt_char->status);

  g_charwr_result = ble_gatt_char->status;
}

static void on_read(uint16_t conn_handle, struct ble_gatt_char_s *ble_gatt_char)
{
  int i;

  printf("%s [BLE] \n", __func__);
  printf("   handle : 0x%04x\n", ble_gatt_char->handle);
  printf("   value len : %d\n",  ble_gatt_char->value.length);
  printf("   value : ");

  for (i = 0; i < ble_gatt_char->value.length; i++)
    {
      printf("%02x ", ble_gatt_char->value.data[i]);
    }

  printf("\n");

  g_read_datalen = ble_gatt_char->value.length;
  memcpy(g_read_data, ble_gatt_char->value.data, g_read_datalen);
  g_charrd_result = (g_read_datalen > 0) ? BT_SUCCESS : BT_FAIL;
}

static void on_notify(uint16_t conn_handle, struct ble_gatt_char_s *ble_gatt_char)
{
  int i;

  printf("%s [BLE] \n", __func__);
  printf("   handle : 0x%04x\n", ble_gatt_char->handle);
  printf("   value len : %d\n",  ble_gatt_char->value.length);
  printf("   value data :");
  for (i = 0; i < ble_gatt_char->value.length; i++)
    {
      printf("%02x ", ble_gatt_char->value.data[i]);
    }

  printf("\n");

  g_notify_cnt++;
}

static void print_descriptor_handle(const char *name, uint16_t handle)
{
  if (handle == BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      return;
    }

  printf("      %s  handle : 0x%04x\n", name, handle);
}

static void on_db_discovery(struct ble_gatt_event_db_discovery_t *db_disc)
{
  int i;
  int j;
  struct ble_gattc_db_discovery_s *db;
  struct ble_gattc_db_disc_srv_s  *srv;
  struct ble_gattc_db_disc_char_s *ch;
  char   uuid[BLE_UUID_128BIT_STRING_BUFSIZE];
  BLE_CHAR_PROP prop;

  db = &db_disc->params.db_discovery;
  srv = &db->services[0];

  /* If any error occurs, finish discovery */

  if (db_disc->result != BT_SUCCESS)
    {
      printf("Discovery failed result=%d\n", db_disc->result);
      goto finish;
    }

  for (i = 0; i < db->srv_count; i++, srv++)
    {
      printf("=== SRV[%d] ===\n", i);

      bleutil_convert_uuid2str(&srv->srv_uuid, uuid, BLE_UUID_128BIT_STRING_BUFSIZE);
      printf("   uuid : %s\n", uuid);

      ch = &srv->characteristics[0];

      for (j = 0; j < srv->char_count; j++, ch++)
        {
          prop = ch->characteristic.char_prope;

          printf("   === CHR[%d] ===\n", j);
          printf("      decl  handle : 0x%04x\n", ch->characteristic.char_declhandle);
          printf("      value handle : 0x%04x\n", ch->characteristic.char_valhandle);
          bleutil_convert_uuid2str(&ch->characteristic.char_valuuid,
                                   uuid,
                                   BLE_UUID_128BIT_STRING_BUFSIZE);
          printf("      uuid         : %s\n", uuid);
          printf("      property     : %s%s%s%s%s\n",
                 prop.notify ? "notify," : "",
                 prop.indicate ? "indicate," : "",
                 prop.read   ? "read," : "",
                 prop.write  ? "write," : "",
                 prop.writeWoResp ? "write w/o rsp" : "");

          print_descriptor_handle("cccd", ch->cccd_handle);
          print_descriptor_handle("cepd", ch->cepd_handle);
          print_descriptor_handle("cudd", ch->cudd_handle);
          print_descriptor_handle("sccd", ch->sccd_handle);
          print_descriptor_handle("cpfd", ch->cpfd_handle);
          print_descriptor_handle("cafd", ch->cafd_handle);
          printf("\n");

          /* In this application, use a characteristic that has
           * read/write/notify property.
           * So, store only such data.
           */

          if (prop.notify && prop.read && (prop.write || prop.writeWoResp))
            {
              memcpy(&g_nrw_char, ch, sizeof(struct ble_gattc_db_disc_char_s));
            }
        }

      printf("\n");
    }

  /* The end_handle of the last service is 0xFFFF.
   * When end_handle is 0x0000, no more services are found.
   * So, if end_handle is not 0xFFFF or 0x0000, the continuous service information exists.
   */

  if ((db_disc->state.end_handle != 0xFFFF) && (db_disc->state.end_handle != 0x0000))
    {
      if (g_target == false)
        {
          ble_continue_db_discovery(db_disc->state.end_handle + 1,
                                    s_ble_state->ble_connect_handle);
          return;
        }
    }

finish:
  g_discovered = true;
}

static struct ble_gattc_db_disc_char_s *get_nrw_characteristic(uint16_t conn_handle)
{
  g_discovered = false;

  if (g_target)
    {
      ble_discover_uuid(conn_handle,
                        &g_target_srv_uuid,
                        &g_target_char_uuid);
    }
  else
    {
      ble_start_db_discovery(conn_handle);
    }

  /* Wait for discovery result to be notified */

  while (g_discovered == false)
    {
      usleep(1); /* usleep for task dispatch */
    }

  return (g_nrw_char.characteristic.char_valhandle != 0) ? &g_nrw_char : NULL;
}

static void on_descriptor_write(uint16_t conn_handle, uint16_t handle, int result)
{
  printf("%s [BLE] conn_handle = 0x%04x, handle = 0x%04x, result = %d\n",
         __func__, conn_handle, handle, result);

  g_descwr_result = result;
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

  g_read_datalen = len;
  memcpy(g_read_data, data, g_read_datalen);
  g_descrd_result = (len) ? BT_SUCCESS : BT_FAIL;
}

static int parse_argument(int argc, FAR char *argv[])
{
  int ret;

  if (argc == 3)
    {
      ret = bleutil_convert_str2uuid(argv[1], &g_target_srv_uuid);
      if (ret != BT_SUCCESS)
        {
          printf("Invalid service UUID.\n");
          return ret;
        }

      ret = bleutil_convert_str2uuid(argv[2], &g_target_char_uuid);
      if (ret != BT_SUCCESS)
        {
          printf("Invalid characteristic UUID.\n");
          return ret;
        }

      g_target = true;
    }
  else if (argc == 2)
    {
      strncpy(g_target_device_name, argv[1], BT_NAME_LEN);
      g_target_device_name[BT_NAME_LEN - 1] = '\0';
      g_target = false;
    }
  else
    {
      g_target = false;
    }

  return OK;
}

static uint16_t scan_and_connect(void)
{
  int ret;

  ret = ble_start_scan(false);
  if (ret != BT_SUCCESS)
    {
      return INVALID_CONN_HANDLE;
    }

  /* If ble_start_scan() is executed, scan result is notified by
   * scan_result callback of struct ble_common_ops_s.
   * This application executes ble_connect() API in scan_result callback.
   * After that, connection will be established and MTU size will be
   * exchanged automatically.
   * So, wait for MTU size to be exchanged here.
   */

  while (!g_mtusz_exchg)
    {
      usleep(1); /* usleep for task dispatch */
    }

  return s_ble_state->ble_connect_handle;
}

static int pairing(uint16_t conn_handle)
{
  int ret;

  g_encrypted = -1;
  ret = ble_pairing(conn_handle);
  if (ret != BT_SUCCESS)
    {
      return ret;
    }

  /* Wait for encryption result to be notified. */

  while (g_encrypted == -1)
    {
      usleep(1); /* usleep for task dispatch */
    }

  return (g_encrypted) ? BT_SUCCESS : BT_FAIL;
}

static int write_descriptor(uint16_t conn_handle,
                            uint16_t desc_handle,
                            uint8_t  *buf,
                            uint16_t len)
{
  int ret;

  g_descwr_result = RESULT_NOT_RECEIVE;

  ret = ble_descriptor_write(conn_handle, desc_handle, buf, len);
  if (ret != BT_SUCCESS)
    {
      return ret;
    }

  /* Wait for write response. */

  while (g_descwr_result == RESULT_NOT_RECEIVE)
    {
      usleep(1); /* usleep for task dispatch */
    }

  return g_descwr_result;
}

static int read_descriptor(uint16_t conn_handle,
                           uint16_t desc_handle,
                           uint8_t  *buf,
                           uint16_t *len)
{
  int ret;

  g_descrd_result = RESULT_NOT_RECEIVE;
  ret = ble_descriptor_read(conn_handle, desc_handle);
  if (ret != BT_SUCCESS)
    {
      return ret;
    }

  /* Wait for read response. */

  while (g_descrd_result == RESULT_NOT_RECEIVE)
    {
      usleep(1);
    }

  if (g_descrd_result == BT_SUCCESS)
    {
      *len = g_read_datalen;
      memcpy(buf, g_read_data, g_read_datalen);
    }

  return g_descrd_result;
}

static int write_characteristic(uint16_t conn_handle,
                                uint16_t char_handle,
                                uint8_t  *buf,
                                uint16_t len,
                                bool     rsp)
{
  int ret;

  g_charwr_result = RESULT_NOT_RECEIVE;

  ret = ble_write_characteristic(conn_handle, char_handle, buf, len, rsp);
  if (ret != BT_SUCCESS)
    {
      return ret;
    }

  if (!rsp)
    {
      /* In write w/o response case, not wait response. */

      return OK;
    }

  /* Wait for write response. */

  while (g_charwr_result == RESULT_NOT_RECEIVE)
    {
      usleep(1); /* usleep for task dispatch */
    }

  return g_charwr_result;
}

static int read_characteristic(uint16_t conn_handle,
                               uint16_t char_handle,
                               uint8_t  *buf,
                               uint16_t *len)
{
  int ret;

  g_charrd_result = RESULT_NOT_RECEIVE;

  ret = ble_read_characteristic(conn_handle, char_handle);
  if (ret != BT_SUCCESS)
    {
      return ret;
    }

  /* Wait for read response. */

  while (g_charrd_result == RESULT_NOT_RECEIVE)
    {
      usleep(1); /* usleep for task dispatch */
    }

  if (g_charrd_result == BT_SUCCESS)
    {
      *len = g_read_datalen;
      memcpy(buf, g_read_data, g_read_datalen);
    }

  return g_charrd_result;
}

static int access_descriptor(uint16_t conn_handle,
                             struct ble_gattc_db_disc_char_s *char_db)
{
  int ret = BT_SUCCESS;
  uint16_t buf = 0;
  uint16_t len = 0;

  if (char_db->cccd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      ret = read_descriptor(conn_handle, char_db->cccd_handle, (uint8_t *)&buf, &len);
      if (ret != BT_SUCCESS)
        {
          return ret;
        }

      if (buf == NOTIFICATION_DISABLED)
        {
          buf = NOTIFICATION_ENABLED;
          ret = write_descriptor(conn_handle, char_db->cccd_handle, (uint8_t *)&buf, len);
        }
    }

  if (char_db->cepd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      ret = read_descriptor(conn_handle, char_db->cepd_handle, (uint8_t *)&buf, &len);
      if (ret != BT_SUCCESS)
        {
          return ret;
        }
    }

  if (char_db->cudd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      ret = read_descriptor(conn_handle, char_db->cudd_handle, (uint8_t *)&buf, &len);
      if (ret != BT_SUCCESS)
        {
          return ret;
        }
    }

  if (char_db->sccd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      ret = read_descriptor(conn_handle, char_db->sccd_handle, (uint8_t *)&buf, &len);
      if (ret != BT_SUCCESS)
        {
          return ret;
        }
    }

  if (char_db->cpfd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      ret = read_descriptor(conn_handle, char_db->cpfd_handle, (uint8_t *)&buf, &len);
      if (ret != BT_SUCCESS)
        {
          return ret;
        }
    }

  if (char_db->cafd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      ret = read_descriptor(conn_handle, char_db->cafd_handle, (uint8_t *)&buf, &len);
      if (ret != BT_SUCCESS)
        {
          return ret;
        }
    }

  return ret;
}

static int access_characteristic(uint16_t conn_handle,
                                 uint16_t char_handle,
                                 int data,
                                 bool rsp)
{
  int ret;
  int buf = 0;
  uint16_t len;

  ret = write_characteristic(conn_handle,
                             char_handle,
                             (uint8_t *)&data,
                             sizeof(data),
                             rsp);
  if (ret != BT_SUCCESS)
    {
      return ret;
    }

  ret = read_characteristic(conn_handle,
                            char_handle,
                            (uint8_t *)&buf,
                            &len);
  if (ret != BT_SUCCESS)
    {
      return ret;
    }

  if (buf != data)
    {
      printf("read result error. value = %d, expect = %d\n", buf, data);
      ret = BT_FAIL;
    }

  return ret;
}

static void finish_ble(uint16_t conn_handle)
{
  struct ble_state_s state;

  state.ble_connect_handle = conn_handle;
  ble_disconnect(&state);

  /* Wait for connection status to be disconnected status. */

  while (s_ble_state)
    {
      usleep(1); /* usleep for task dispatch */
    }

  bt_disable();
  bt_finalize();
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
  int i;
  uint16_t conn_handle;
  struct ble_gattc_db_disc_char_s *nrw_char;

  ret = parse_argument(argc, argv);
  if (ret != OK)
    {
      return ret;
    }

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

  /* Free memory that is allocated in on_loadbond() callback function. */

  free_cccd();

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

  conn_handle = scan_and_connect();
  if (conn_handle != BT_SUCCESS)
    {
      printf("%s [BLE] Connection to the target failed. ret = %d\n", __func__, ret);
      goto error;
    }

  nrw_char = get_nrw_characteristic(conn_handle);
  if (nrw_char == NULL)
    {
      printf("%s [BLE] Can not find the characteristic that "
             "has the notify/read/write property. ret = %d\n", __func__, ret);
      goto error;
    }

  ret = pairing(conn_handle);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Pairing failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Read and write descriptor of discovered characteristic.
   * This application increments written data each write.
   */

  ret = access_descriptor(conn_handle, nrw_char);

  if (nrw_char->characteristic.char_prope.write)
    {
      /* write(w/ rsp) and read characteristic */

      for (i = 0; i < CHAR_ACCESS_COUNT / 2; i++)
        {
          ret = access_characteristic(conn_handle,
                                      nrw_char->characteristic.char_valhandle,
                                      g_dbaccess_cnt++,
                                      TRUE);
          if (ret != BT_SUCCESS)
            {
              goto error;
            }
        }
    }

  if (nrw_char->characteristic.char_prope.writeWoResp)
    {
      /* write(w/o rsp) and read characteristic */

      for (i = 0; i < CHAR_ACCESS_COUNT / 2; i++)
        {
          ret = access_characteristic(conn_handle,
                                      nrw_char->characteristic.char_valhandle,
                                      g_dbaccess_cnt++,
                                      FALSE);
          if (ret != BT_SUCCESS)
            {
              goto error;
            }
        }
    }

  /* This application wait receive notify CHAR_ACCESS_COUNT times. */

  while (g_notify_cnt < CHAR_ACCESS_COUNT)
    {
      usleep(1); /* usleep for task dispatch */
    }

  finish_ble(conn_handle);

error:
  return ret;
}
