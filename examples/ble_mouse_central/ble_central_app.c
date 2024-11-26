/****************************************************************************
 * examples/ble_mouse_central/ble_central_app.c
 *
 *   Copyright 2024 Sony Semiconductor Solutions Corporation
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

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/queue.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/hal/bt_if.h>
#include <bluetooth/ble_util.h>
#include "ble_central_app.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLE_APP_BUFFER_LEN  64
#define BLE_APP_DEVNAME_LEN 32

#define BLE_APP_DEFAULT_MTU_SIZE 23

/* Maximum number of handles with the same UUID */

#define MAX_HANDLES_OF_UUID 8

/* Print the detail log (1:enable, 0:disable) */

#define LOG_SCAN      1
#define LOG_DISCOVERY 1
#define LOG_NOTIFY    0
#define LOG_READ      0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct handle_set_s
{
  uint16_t handle;
  uint16_t cccd_handle;
};

/* Discovery list node */
struct discover_node_s
{
  discover_cb_t                cb;
  BLE_UUID                     uuid;
  int                          nhandle;
  struct handle_set_s          handles[MAX_HANDLES_OF_UUID];
  SLIST_ENTRY(discover_node_s) entries;
};

/* Notify list node */

struct notify_node_s
{
  notify_cb_t                cb;
  BLE_UUID                   uuid;
  int                        nhandle;
  struct handle_set_s        handles[MAX_HANDLES_OF_UUID];
  SLIST_ENTRY(notify_node_s) entries;
};

/* Private structure for BLE central app */

struct ble_app_private_t
{
  enum ble_app_event event;
  sem_t              event_sync;
  int                result;
  uint8_t            buffer[BLE_APP_BUFFER_LEN];
  int                len;
  sem_t              api_sync;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void on_scan(BT_ADDR addr, uint8_t *data, uint8_t len);
static void on_state_change(struct ble_state_s *state, bool connected, uint8_t cause);
static void on_mtusize(uint16_t handle, uint16_t sz);
static void on_discovered(struct ble_gatt_event_db_discovery_t *db_disc);
static void on_enc_result(uint16_t handle, bool result);
static void on_devicename(const char *name);

static void on_write(uint16_t conn_handle, struct ble_gatt_char_s *gatt_char);
static void on_read(uint16_t conn_handle, struct ble_gatt_char_s *gatt_char);
static void on_notify(uint16_t conn_handle, struct ble_gatt_char_s *gatt_char);
static void on_desc_write(uint16_t connhdl, uint16_t charhdl, int result);
static void on_desc_read(uint16_t connhdl, uint16_t charhdl, uint8_t *data, uint16_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ble_common_ops_s ble_common_ops =
  {
    .scan_result                = on_scan,
    .mtusize                    = on_mtusize,
    .connect_status_changed     = on_state_change,
    .encryption_result          = on_enc_result,
    .connected_device_name_resp = on_devicename,
    .save_bondinfo              = NULL,
    .load_bondinfo              = NULL,
  };

static struct ble_gatt_central_ops_s ble_central_ops =
  {
    .write              = on_write,
    .read               = on_read,
    .notify             = on_notify,
    .database_discovery = on_discovered,
    .descriptor_write   = on_desc_write,
    .descriptor_read    = on_desc_read,
  };

/* Connection handle */

static uint16_t g_connecthdl;

/* Scan filter parameters */

static char     g_filter_devname[BLE_APP_DEVNAME_LEN];
static BLE_UUID g_filter_uuid;

/* Private data */

static struct ble_app_private_t g_ble_app_priv;

/* Discovery list */

static SLIST_HEAD(discover_list_s, discover_node_s) g_discover_list =
  SLIST_HEAD_INITIALIZER(g_discover_list);

/* Notify list */

static SLIST_HEAD(notify_list_s, notify_node_s) g_notify_list =
  SLIST_HEAD_INITIALIZER(g_notify_list);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* BLE common operations */

static void unused_code
display_devinfo(uint8_t *addr, uint8_t type, const char *name)
{
  printf(name[0] != '\0' ? "Addr %s %02X:%02X:%02X:%02X:%02X:%02X >>%s<<\n" :
                           "Addr %s %02X:%02X:%02X:%02X:%02X:%02X\n",
         type == BLE_ADDRTYPE_PUBLIC                  ? "Pub" :
         type == BLE_ADDRTYPE_RAND_STATIC             ? "Rnd" :
         type == BLE_ADDRTYPE_RAND_PRIV_RESOLVABLE    ? "Prv" :
         type == BLE_ADDRTYPE_RAND_PRIV_NONRESOLVABLE ? "NRs" : "Unk",
         addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],
         name[0] != '\0' ? name : "");
}

static void on_scan(BT_ADDR addr, uint8_t *data, uint8_t len)
{
  uint8_t addr_type;
  struct ble_state_s state = {0};
  bool match = false;
  char devname[BLE_APP_DEVNAME_LEN];

  /* Get the BLE address in scanned data. */

  addr_type = bleutil_get_addrtype(data, len);

  /* Search for the BLE device name in scanned data. */

  memset(devname, 0, sizeof(devname));
  if (bleutil_get_devicename(data, len, devname))
    {
      if (0 == strncmp(g_filter_devname, devname, sizeof(g_filter_devname)))
        {
          match = true;
        }
    }

#if LOG_SCAN == 1
  display_devinfo(addr.address, addr_type, devname);
#endif

  /* Search for the service UUID in scanned data. */

  if (bleutil_find_srvc_uuid(&g_filter_uuid, data, len))
    {
      match = true;
    }

  /* If the specified device is found by scan filter, connect the address */

  if (match)
    {
      /* Stop scanning */

      ble_cancel_scan();

      /* Connect a found device. */

      bleutil_add_btaddr(&state, &addr, addr_type);
      ble_connect(&state);
    }
}

static void on_state_change(struct ble_state_s *state, bool connected,
                            uint8_t cause)
{
  struct ble_app_private_t *priv = &g_ble_app_priv;

  printf(connected ? "Connected " : "Disconnected ");
  printf(" : Cause <%s>  :",
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
                                             "Unspecified error");
  display_devinfo(state->bt_target_addr.address,
                  state->bt_target_addr_type,
                  state->bt_target_name);

  if (!connected)
    {
      /* Send the disconnected event to user application. */

      priv->event = BLE_APP_DISCONNECTED;
      sem_post(&priv->event_sync);
    }
  else
    {
      /* Keep connected device handle */

      g_connecthdl = state->ble_connect_handle;
    }
}

static void on_mtusize(uint16_t handle, uint16_t sz)
{
  printf("[%s] MTU size is negotiated as :%d\n", __func__, sz);

  /* Start discovery */

  ble_start_db_discovery(g_connecthdl);
}

static void on_enc_result(uint16_t handle, bool result)
{
  struct ble_app_private_t *priv = &g_ble_app_priv;

  printf("[%s] Encryption : %s\n", __func__, result ? "Success" : "Fail");

  /* Send the connected event to user application. */

  priv->event = BLE_APP_CONNECTED;
  sem_post(&priv->event_sync);
}

static void on_devicename(const char *name)
{
  printf("[%s] Got device name : %s\n", __func__, name);
}

/* BLE GATT Central operations */

static void on_write(uint16_t conn_handle,
                     struct ble_gatt_char_s *gatt_char)
{
  struct ble_app_private_t *priv = &g_ble_app_priv;

  priv->result = gatt_char->status;
  sem_post(&priv->api_sync);
}

static void unused_code
display_dumpdata(const char *funcname, struct ble_gatt_char_s *gatt_char)
{
  int i;

  printf("[%s]\n", funcname);
  printf("   handle : 0x%04x\n", gatt_char->handle);
  printf("   value len : %d\n",  gatt_char->value.length);
  printf("   value data: ");
  for (i = 0; i < gatt_char->value.length; i++)
    {
      printf("%02x ", gatt_char->value.data[i]);
    }

  printf("\n");
}

static void on_read(uint16_t conn_handle,
                    struct ble_gatt_char_s *gatt_char)
{
  struct ble_app_private_t *priv = &g_ble_app_priv;

  priv->len = gatt_char->value.length;
  if (priv->len <= BLE_APP_BUFFER_LEN)
    {
#if LOG_READ == 1
      display_dumpdata(__func__, gatt_char);
#endif
      memcpy(priv->buffer, gatt_char->value.data, priv->len);
      priv->result = 0;
    }
  else
    {
      priv->result = -1;
    }

  sem_post(&priv->api_sync);
}

static void on_notify(uint16_t conn_handle,
                      struct ble_gatt_char_s *gatt_char)
{
  struct notify_node_s *node;
  int i;

#if LOG_NOTIFY == 1
  display_dumpdata(__func__, gatt_char);
#endif

  SLIST_FOREACH(node, &g_notify_list, entries)
    {
      for (i = 0; i < node->nhandle; i++)
        {
          if (node->handles[i].handle == gatt_char->handle)
            {
              node->cb(gatt_char);
            }
        }
    }
}

static void unused_code
display_discovered(struct ble_gattc_db_disc_srv_s *srv,
                   struct ble_gattc_db_disc_char_s *ch)
{
  char srv_uuid[BLE_UUID_128BIT_STRING_BUFSIZE];
  char chr_uuid[BLE_UUID_128BIT_STRING_BUFSIZE];
  BLE_CHAR_PROP prop;

  bleutil_convert_uuid2str(&srv->srv_uuid, srv_uuid, BLE_UUID_128BIT_STRING_BUFSIZE);
  bleutil_convert_uuid2str(&ch->characteristic.char_valuuid, chr_uuid, BLE_UUID_128BIT_STRING_BUFSIZE);

  printf("Service UUID: %s\n", srv_uuid);
  printf(" Handle: 0x%04x (UUID: %s)\n", ch->characteristic.char_valhandle, chr_uuid);

  prop = ch->characteristic.char_prope;
  printf("  Properties: %s%s%s%s%s%s%s%s\n",
          prop.reserve      ? "Extended, " : "",
          prop.authSignedWr ? "Authenticated Signed Writes, " : "",
          prop.indicate     ? "Indicate, " : "",
          prop.notify       ? "Notify, " : "",
          prop.write        ? "Write, " : "",
          prop.writeWoResp  ? "Write Without Response, " : "",
          prop.read         ? "Read, " : "",
          prop.broadcast    ? "Broadcast, " : "");

  if (ch->cafd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      printf("  Handle: 0x%04x (Characteristic Aggregate Format)\n",
             ch->cafd_handle);
    }

  if (ch->cpfd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      printf("  Handle: 0x%04x (Characteristic Presentation Format)\n",
             ch->cpfd_handle);
    }

  if (ch->sccd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      printf("  Handle: 0x%04x (Server Characteristic Configuration)\n",
             ch->sccd_handle);
    }

  if (ch->cccd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      printf("  Handle: 0x%04x (Client Characteristic Configuration)\n",
             ch->cccd_handle);
    }

  if (ch->cudd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      printf("  Handle: 0x%04x (Characteristic User Description)\n",
             ch->cudd_handle);
    }

  if (ch->cepd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
    {
      printf("  Handle: 0x%04x (Characteristic Extended Properties)\n",
             ch->cepd_handle);
    }
}

static void on_discovered(struct ble_gatt_event_db_discovery_t *db_disc)
{
  int svcs;
  int chrs;
  int srv_num = db_disc->params.db_discovery.srv_count;
  struct ble_gattc_db_disc_srv_s  *srv = db_disc->params.db_discovery.services;
  struct ble_gattc_db_disc_char_s *ch;
  struct discover_node_s *node;
  int i;
  uint16_t handle;
  uint16_t cccd_handle;

  for (svcs = 0; svcs < srv_num; svcs++, srv++)
    {
      ch = srv->characteristics;
      for (chrs = 0; chrs < srv->char_count; chrs++, ch++)
        {
#if LOG_DISCOVERY == 1
          display_discovered(srv, ch);
#endif
          SLIST_FOREACH(node, &g_discover_list, entries)
            {
              if (bleutil_uuidcmp(&node->uuid, &ch->characteristic.char_valuuid) == 0)
                {
                  handle = ch->characteristic.char_valhandle;
                  cccd_handle = ch->cccd_handle;

                  for (i = 0; i < node->nhandle; i++)
                    {
                      /* Check if already registered. */

                      if ((node->handles[i].handle == handle) &&
                          (node->handles[i].cccd_handle == cccd_handle))
                        {
                          break;
                        }
                    }

                  if ((i == node->nhandle) && (i < MAX_HANDLES_OF_UUID))
                    {
                      /* Register new handle */

                      node->handles[i].handle = handle;
                      node->handles[i].cccd_handle = cccd_handle;
                      node->nhandle++;
                    }

                  node->cb(ch);
                }
            }
        }
    }

  if ((db_disc->state.end_handle != 0xffff) &&
      (db_disc->state.end_handle != 0x0000))
    {
      /* Continue to find next handle. */

      ble_continue_db_discovery(db_disc->state.end_handle + 1,
                                g_connecthdl);
    }
  else
    {
      /* Pairing after discovery is complete. */

      printf("[%s] Paring to BLE device\n", __func__);
      ble_pairing(g_connecthdl);
    }
}

static void on_desc_write(uint16_t connhdl, uint16_t charhdl, int result)
{
  struct ble_app_private_t *priv = &g_ble_app_priv;

  priv->result = result;
  sem_post(&priv->api_sync);
}

static void on_desc_read(uint16_t connhdl, uint16_t charhdl,
                         uint8_t *data, uint16_t len)
{
  struct ble_app_private_t *priv = &g_ble_app_priv;

  priv->len = len;
  if (priv->len <= BLE_APP_BUFFER_LEN)
    {
      memcpy(priv->buffer, data, priv->len);
      priv->result = 0;
    }
  else
    {
      priv->result = -1;
    }

  sem_post(&priv->api_sync);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* BLE GATT synchronous API */

int write_gatt_descriptor(uint16_t handle,
                          uint8_t *data, int len)
{
  int ret;
  struct ble_app_private_t *priv = &g_ble_app_priv;

  ret = ble_descriptor_write(g_connecthdl, handle, data, len);
  if (ret == BT_SUCCESS)
    {
      sem_wait(&priv->api_sync);
      ret = priv->result;
    }

  return ret;
}

int read_gatt_descriptor(uint16_t handle, uint8_t *data, int *len)
{
  int ret;
  struct ble_app_private_t *priv = &g_ble_app_priv;

  *len = 0;

  ret = ble_descriptor_read(g_connecthdl, handle);
  if (ret == BT_SUCCESS)
    {
      sem_wait(&priv->api_sync);
      ret = priv->result;
      if (ret == 0)
        {
          *len = priv->len;
          memcpy(data, priv->buffer, priv->len);
        }
    }

  return ret;
}

int write_gatt_char(uint16_t handle, uint8_t *data, int len, bool response)
{
  int ret;
  struct ble_app_private_t *priv = &g_ble_app_priv;

  ret = ble_write_characteristic(g_connecthdl, handle, data, len, response);
  if (response && (ret == BT_SUCCESS))
    {
      sem_wait(&priv->api_sync);
      ret = priv->result;
    }

  return ret;
}

int read_gatt_char(uint16_t handle, uint8_t *data, int *len)
{
  int ret;
  struct ble_app_private_t *priv = &g_ble_app_priv;

  *len = 0;

  ret = ble_read_characteristic(g_connecthdl, handle);
  if (ret == BT_SUCCESS)
    {
      sem_wait(&priv->api_sync);
      ret = priv->result;
      if (ret == 0)
        {
          *len = priv->len;
          memcpy(data, priv->buffer, priv->len);
        }
    }

  return ret;
}

int scan_filter_uuid(BLE_UUID *uuid)
{
  g_filter_uuid = *uuid;
  return 0;
}

int scan_filter_device_name(const char *name)
{
  strncpy(g_filter_devname, name, sizeof(g_filter_devname) - 1);
  return 0;
}

int register_discover(BLE_UUID *uuid, discover_cb_t cb)
{
  struct discover_node_s *node;

  node = malloc(sizeof(struct discover_node_s));
  if (!node)
    {
      return -ENOMEM;
    }

  /* Insert into discover list. */

  node->cb          = cb;
  node->uuid        = *uuid;
  node->nhandle     = 0;
  SLIST_INSERT_HEAD(&g_discover_list, node, entries);

  return 0;
}

int unregister_discover(BLE_UUID *uuid)
{
  struct discover_node_s *node;
  struct discover_node_s *tmp;

  /* Remove from discover list. */

  SLIST_FOREACH_SAFE(node, &g_discover_list, entries, tmp)
    {
      if (0 == bleutil_uuidcmp(&node->uuid, uuid))
        {
          SLIST_REMOVE(&g_discover_list, node, discover_node_s, entries);
          free(node);
        }
    }

  return 0;
}

int start_notify(BLE_UUID *uuid, notify_cb_t cb)
{
  int ret = 0;
  uint16_t notify_en = 1;
  struct notify_node_s *nnode;
  struct discover_node_s *dnode;
  int nhandle = 0;
  int i;
  uint16_t cccd_handle;

  /* Get the handle numbers from discovered UUID. */

  SLIST_FOREACH(dnode, &g_discover_list, entries)
    {
      if (bleutil_uuidcmp(&dnode->uuid, uuid) == 0)
        {
          nhandle = dnode->nhandle;
          break;
        }
    }

  if (nhandle == 0)
    {
      return -ENOENT;
    }

  /* Insert into notify list. */

  nnode = malloc(sizeof(struct notify_node_s));
  if (!nnode)
    {
      return -ENOMEM;
    }

  nnode->cb      = cb;
  nnode->uuid    = *uuid;
  nnode->nhandle = nhandle;
  memcpy(nnode->handles, dnode->handles, sizeof(nnode->handles));
  SLIST_INSERT_HEAD(&g_notify_list, nnode, entries);

  /* Enable notification on CCCD handle. */

  for (i = 0; i < nhandle; i++)
    {
      cccd_handle = dnode->handles[i].cccd_handle;
      if (cccd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
        {
          ret = write_gatt_descriptor(cccd_handle,
                                      (uint8_t *)&notify_en, sizeof(notify_en));
          if (ret != BT_SUCCESS)
            {
              printf("ERROR: Failed to enable notification ret=%d\n", ret);
            }
        }
    }

  return ret;
}

int stop_notify(BLE_UUID *uuid)
{
  int ret = 0;
  uint16_t notify_en = 0;
  struct notify_node_s *node;
  struct notify_node_s *tmp;
  int i;
  uint16_t cccd_handle;

  /* Remove from notify list. */

  SLIST_FOREACH_SAFE(node, &g_notify_list, entries, tmp)
    {
      if (bleutil_uuidcmp(&node->uuid, uuid) == 0)
        {
          for (i = 0; i < node->nhandle; i++)
            {
              cccd_handle = node->handles[i].cccd_handle;
              if (cccd_handle != BLE_GATT_INVALID_ATTRIBUTE_HANDLE)
                {
                  ret = write_gatt_descriptor(cccd_handle,
                                              (uint8_t *)&notify_en, sizeof(notify_en));
                }
            }

          SLIST_REMOVE(&g_notify_list, node, notify_node_s, entries);
          free(node);
        }
    }

  return ret;
}

int ble_app_init(void)
{
  int ret = 0;
  struct ble_app_private_t *priv = &g_ble_app_priv;

  /* Initialize private data. */

  sem_init(&priv->event_sync, 0, 0);
  sem_init(&priv->api_sync, 0, 0);

  memset(g_filter_devname, 0, sizeof(g_filter_devname));

  /* Initialize BT/BLE */

  bt_init();

  ble_register_common_cb(&ble_common_ops);
  ble_register_gatt_central_cb(&ble_central_ops);

  return ret;
}

enum ble_app_event ble_app_wait_event(void)
{
  struct ble_app_private_t *priv = &g_ble_app_priv;

  sem_wait(&priv->event_sync);
  return priv->event;
}
