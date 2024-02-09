/****************************************************************************
 * ble_midi_central/ble_midi_central.c
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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
#include <bluetooth/bt_common.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/hal/bt_if.h>
#include <bluetooth/ble_util.h>

#include "led_indicator.h"
#include "midi_play.h"
#include "midi_parser.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MIDI BLE Service UUID:03B80E5A-EDE8-4B33-A751-6CE34EC4C700 */

#define MIDI_SRV_UUID { 0x00, 0xc7, 0xc4, 0x4e, \
                        0xe3, 0x6c, 0x51, 0xa7, \
                        0x33, 0x4b, 0xe8, 0xed, \
                        0x5a, 0x0e, 0xb8, 0x03  }

/* MIDI BLE Characteristic UUID:7772E5DB-3868-4112-A1A9-F2669D106BF3 */

#define MIDI_CHR_UUID  { 0xF3, 0x6B, 0x10, 0x9D, \
                         0x66, 0xF2, 0xA9, 0xA1, \
                         0x12, 0x41, 0x68, 0x38, \
                         0xDB, 0xE5, 0x72, 0x77  }

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void on_scan(BT_ADDR addr, uint8_t *data, uint8_t len);
static void on_state_change(struct ble_state_s *state, bool connected,
                            uint8_t cause);
static void on_mtusize(uint16_t handle, uint16_t sz);
static void on_discovered(struct ble_gatt_event_db_discovery_t *db_disc);
static void on_enc_result(uint16_t handle, bool result);
static void on_devicename(const char *name);

static void on_read(uint16_t conn_handle,
                    struct ble_gatt_char_s *gatt_char);
static void on_notify(uint16_t conn_handle,
                      struct ble_gatt_char_s *ble_gatt_char);
static void on_desc_write(uint16_t connhdl, uint16_t charhdl, int result);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static BLE_UUID g_midi_svcuuid = { BLE_UUID_TYPE_UUID128, {{MIDI_SRV_UUID}} };
static BLE_UUID g_midi_chruuid = { BLE_UUID_TYPE_UUID128, {{MIDI_CHR_UUID}} };

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
    .write              = NULL,
    .read               = on_read,
    .notify             = on_notify,
    .database_discovery = on_discovered,
    .descriptor_write   = on_desc_write,
    .descriptor_read    = NULL,
  };

static BT_ADDR g_mybleaddr = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};
static uint16_t g_connecthdl;
static uint16_t g_charhdl;
static uint16_t g_char_cccdhdl;
static char g_devname[256];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void display_devinfo(uint8_t *addr, uint8_t type, const char *name)
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
  int ret;
  uint8_t addr_type;
  struct ble_state_s state = {0};

  addr_type = bleutil_get_addrtype(data, len);
  if (bleutil_get_devicename(data, len, g_devname))
    {
      printf("[%s] MSG: Device name : %s\n", __func__, g_devname);
      g_devname[0] = '\0';
    }

  display_devinfo(addr.address, addr_type, g_devname);

  if (bleutil_find_srvc_uuid(&g_midi_svcuuid, data, len))
    {
      /* Stop scanning */

      ret = ble_cancel_scan();
      if (ret != BT_SUCCESS)
        {
          printf("[%s] ERROR: ble_cancel_scan() failed. ret = %d\n",
                  __func__, ret);
        }

      /* Connect found device. */

      bleutil_add_btaddr(&state, &addr, addr_type);
      ret = ble_connect(&state);
      if (ret != BT_SUCCESS)
        {
          printf("[%s] ERROR: ble_connect() failed. ret = %d\n",
                 __func__, ret);
          return;
        }

      printf("[%s] MSG: Connecting...\n", __func__);
    }
}

static void on_state_change(struct ble_state_s *state, bool connected,
                            uint8_t cause)
{
  printf(connected ? "Connected " : "Disconnected ");
  printf(" : Cause <%s>  :", 
         cause == BLESTAT_SUCCESS          ? "Success"                 :
         cause == BLESTAT_MEMCAP_EXCD      ? "Mem capacity exceed"     :
         cause == BLESTAT_CONNECT_TIMEOUT  ? "Connetion timeout"       :
         cause == BLESTAT_PEER_TERMINATED  ? "Peripheral terminated"   :
         cause == BLESTAT_PEER_TERM_LOWRES ? "Peripheral low resource" :
         cause == BLESTAT_PEER_TERM_POFF   ? "Peripheral power off"    :
         cause == BLESTAT_TERMINATED       ? "Terminated by self"      :
         cause == BLESTAT_DEVICE_BUSY      ? "Device busy"             :
         cause == BLESTAT_PARAM_REJECTED   ? "Negotiation breakup"     :
         cause == BLESTAT_CONNECT_FAILED   ? "Connection failed"       :
                                             "Unspecified error"       );
  display_devinfo(state->bt_target_addr.address,
                  state->bt_target_addr_type,
                  state->bt_target_name);
  
  if (!connected)
    {
      ble_start_scan(false); /* Scan again.. */
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

  /* Start discover characteristics */

  ble_discover_uuid(g_connecthdl, &g_midi_svcuuid, &g_midi_chruuid);
}

static uint16_t search_charuuid(BLE_UUID *uuid,
                                struct ble_gatt_event_db_discovery_t *db_disc,
                                uint16_t *cccd)
{
  int svcs;
  int chrs;
  int srv_num = db_disc->params.db_discovery.srv_count;
  struct ble_gattc_db_disc_srv_s  *srv = db_disc->params.db_discovery.services;
  struct ble_gattc_db_disc_char_s *ch;

  for (svcs = 0; svcs < srv_num; svcs++)
    {
      ch = srv->characteristics;
      for (chrs = 0; chrs < srv->char_count; chrs++, ch++)
        {
          if (bleutil_uuidcmp(uuid, &ch->characteristic.char_valuuid) == 0)
            {
              /* Found the characteristics */

              *cccd = ch->cccd_handle;
              return ch->characteristic.char_valhandle;
            }
        }
    }

  return 0;
}

static void on_discovered(struct ble_gatt_event_db_discovery_t *db_disc)
{
  /* display_svcinfo(db_disc); */

  g_charhdl = search_charuuid(&g_midi_chruuid, db_disc, &g_char_cccdhdl);
  if (g_charhdl != 0)
    {
      /* Found MIDI Charactor */

      printf("[%s] Paring to BLE MIDI device\n", __func__);
      ble_pairing(g_connecthdl);
      return;
    }

  /* Couldn't find.. */

  if (db_disc->state.end_handle != 0xFFFF)
    {
      /* Continue to find next handle id */

      ble_continue_db_discovery(db_disc->state.end_handle + 1,
                                g_connecthdl);
    }
  else
    {
      printf("[%s] No more charactoristics...\n", __func__);
      /* ble_start_scan(false); */
    }
}

static void on_enc_result(uint16_t handle, bool result)
{
  int ret;

  printf("[%s] Encryption : %s\n", __func__, result ? "Success" : "Fail");

  /* Now paring is done.
   * Read the characteristic once because of MIDI BLE spec.
   */

  ret = ble_read_characteristic(g_connecthdl, g_charhdl);
  if (ret != BT_SUCCESS)
    {
      printf("[%s] gle_charactaristic_read() Failed : %d\n", __func__, ret);
    }
}

static void on_devicename(const char *name)
{
  printf("[%s] Got device name : %s\n", __func__, name);
}

static void on_read(uint16_t conn_handle,
                    struct ble_gatt_char_s *ble_gatt_char)
{
  int ret;
  uint8_t notice_en = 1;

  /* Dummy read is completed.
   * Enable notification to receive asynchronouse midi packet.
   */

  printf("[%s] Notify Enable\n", __func__);
  ret = ble_descriptor_write(g_connecthdl,
                             g_char_cccdhdl, &notice_en, 1);
  if (ret != BT_SUCCESS)
    {
      printf("[%s] Failed to notification enabled\n", __func__);
    }
}

static void on_desc_write(uint16_t connhdl, uint16_t charhdl, int result)
{
  printf("[%s] Enabling notify result: %d\n", __func__, result);
  led_stop_waitconnection();
}

static void on_notify(uint16_t conn_handle,
                      struct ble_gatt_char_s *gatt_char)
{
  /* Received nodify from MIDI device. */

  parse_midicmd(gatt_char->value.data, gatt_char->value.length);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  int ret;

  init_midiparser();
  init_led_indicator();

  /* Initialize BT/BLE */

  bt_init();
  ble_register_common_cb(&ble_common_ops);
  ble_register_gatt_central_cb(&ble_central_ops);
  ble_set_name("BLE_MIDI_CENTRAL");
  ble_set_address(&g_mybleaddr);

  /* Turn ON BT */

  ret = bt_enable();
  if (ret != BT_SUCCESS)
    {
      printf("[%s] bt_enable() failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE enable */

  ret = ble_enable();
  if (ret != BT_SUCCESS)
    {
      printf("[%s] ble_enable() failed. ret = %d\n", __func__, ret);
      goto error;
    }

  ret = ble_start_scan(false);
  if (ret != BT_SUCCESS)
    {
      printf("[%s] ble_start_scan() failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BT/BLE is an event driven framework.
   * So this task will handling actual play of sound of MIDI command.
   */

  play_midi();

error:
  ble_disable();
  bt_disable();

  return 0;
}
