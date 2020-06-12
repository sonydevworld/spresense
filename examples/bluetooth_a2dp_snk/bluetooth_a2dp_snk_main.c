/****************************************************************************
 * bluetooth_a2dp_snk/bluetooth_a2dp_snk_main.c
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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
#include "system/readline.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/bt_spp.h>
#include <bluetooth/bt_a2dp.h>
#include <bluetooth/bt_avrcp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAY_SIZE(a)   ((sizeof(a))/(sizeof(a[0])))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

typedef struct
{
  const char   *cmd;                        /* The command text */
  const char   *arghelp;                    /* Text describing the args */
  void (*pFunc)(int argc, char *argv[]);    /* Pointer to command handler */
  const char   *help;                       /* The help text */
}BtA2dpSnkCmd;

/* BT common callbacks */

static void on_command_status(BT_CMD_STATUS status);                      /**< Command status */
static void on_pairing_complete(BT_ADDR addr, BT_PAIR_STATUS status);     /**< Pairing complete */
static void on_inquiry_result(BT_ADDR addr, char *name);                  /**< Inquiry data result */
static void on_inquiry_complete(void);                                    /**< Coplete inquiry */
static void on_connect_status_changed(struct bt_acl_state_s *bt_acl_state,
                                    bool connected, int status);          /**< Connection status change */
static void on_connected_devicename(const char *name);                    /**< Device name change */
static void on_bond_info(BT_ADDR addr);                                   /**< Bonding information */

/* BT A2DP callbacks */

static void on_a2dp_command_status(BT_CMD_STATUS status);                 /**< A2dp command status */
static void on_a2dp_connected(struct bt_acl_state_s *bt_acl_state,
                             BT_AUDIO_CODEC_INFO codecInfo);              /**< A2dp connected */
static void on_a2dp_disconnected(struct bt_acl_state_s *bt_acl_state);    /**< A2dp disconnected */
static void on_a2dp_media_received(struct bt_acl_state_s *bt_acl_state,
                                 uint8_t *data, int len);

/* BT AVRCP callbacks */

static void on_AVRCP_command_status(BT_CMD_STATUS status);                                   /**< Command status notification */
static void on_AVRCP_connected(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_ROLE role);     /**< Connection status notification */
static void on_AVRCP_disconnected(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_ROLE role);  /**< Disconnection status notification */


/* BT A2DP_SNK commands definitions */

static void bt_connect_a2dp_sink(int argc, char *argv[]);
static void bt_disconnect_a2dp_sink(int argc, char *argv[]);
static void bt_connect_avrc_controller(int argc, char *argv[]);
static void bt_disconnect_avrc_controller(int argc, char *argv[]);
static void bt_avrc_controller_send_command(int argc, char *argv[]);

/* BT AVRCP notifications */

static void on_play_status_change(BT_AVRC_TRACK_INFO *trackInfo);  /**< Play status change */
static void on_track_change(BT_AVRC_TRACK_INFO *trackInfo);        /**< Track change */
static void on_play_pos_changed(uint8_t *pdata, int len);          /**< Play position change */
static void on_app_setting_change(uint8_t *pdata, int len);        /**< Application settings change */
static void on_volume_change(uint8_t *pdata, int len);             /**< Volume change on the target device */

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
    .connected_device_name  = on_connected_devicename,
    .bond_info              = on_bond_info
  };

static struct bt_a2dp_ops_s bt_a2dp_ops =
  {
    .command_status     = on_a2dp_command_status,
    .connect            = on_a2dp_connected,
    .disconnect         = on_a2dp_disconnected,
    .receive_media_pkt  = on_a2dp_media_received,
  };

static struct bt_avrcp_ops_s bt_avrcp_ops =
  {
    .command_status     = on_AVRCP_command_status,
    .connect            = on_AVRCP_connected,
    .disconnect         = on_AVRCP_disconnected,
  };

static struct bt_avrcp_notify_ops_s bt_avrcp_notify_ops =
  {
    .playStatusChange    = on_play_status_change,
    .trackChange         = on_track_change,
    .playPosChanged      = on_play_pos_changed,
    .appSettingChange    = on_app_setting_change,
    .volumeChange        = on_volume_change,
  };

/****************************************************************************

command/Arguments          Description

connectsink                Connect a2dp sink
disconnectsink             Disconnect a2dp sink
conavrccon                 Connect avrc Controller
disconavrccon              Disconnect avrc Controller

avrcSendCommand/arg1       Avrc controller send command, arg1:
                           65 volume up
                           66 volume down
                           68 play
                           69 stop
                           70 pause

******************************************************************************/

const static BtA2dpSnkCmd btA2dpCmds[] =
  {
    { "connectsink",          "",           bt_connect_a2dp_sink,             "Emulate BT event" },
    { "disconnectsink",       "",           bt_disconnect_a2dp_sink,          "Emulate BT event" },
    { "conavrccon",           "",           bt_connect_avrc_controller,       "Emulate BT event" },
    { "disconavrccon",        "",           bt_disconnect_avrc_controller,    "Emulate BT event" },
    { "avrcSendCommand",      "",           bt_avrc_controller_send_command,   "Emulate BT event" },
  };

static BT_ADDR local_addr           = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};
static char local_name[BT_NAME_LEN] = "SONY_BT_A2DP_SNK_SAMPLE";
struct bt_acl_state_s *s_bt_acl_state;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  printf("%s [BT] Inquiry complete\n");
}

static void on_connect_status_changed(struct bt_acl_state_s *bt_acl_state,
                                    bool connected, int status)
{
  /* If connect status changed, this function will call. */
  printf("%s [BT] Connect status changed\n", __func__);

  s_bt_acl_state = bt_acl_state;
}

static void on_connected_devicename(const char *name)
{
  /* If receive connected device name data, this function will call. */
  printf("%s [BT] Receive connected device name = %s\n", __func__, name);
}

static void on_bond_info(BT_ADDR addr)
{
  /* If new bonding is comming, this function will call.
   * Print new bonding information.
   */
  printf("[BT] Bonding information ADDR:%02X:%02X:%02X:%02X:%02X:%02X\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5]);
}

/* BT A2DP callbacks */

static void on_a2dp_command_status(BT_CMD_STATUS status)
{
  /* If AT command has been run, this function will call
   * Print the result of AT command
   */
  printf("%s [BT_A2DP] Command status = %d\n", __func__, status);
}

static void on_a2dp_connected(struct bt_acl_state_s *bt_acl_state,
                             BT_AUDIO_CODEC_INFO codecInfo)
{
  /* If A2DP is connected, this function will call
   * Print the A2DP connected message
   */
  printf("%s [BT_A2DP] A2DP connected\n", __func__);
}

static void on_a2dp_disconnected(struct bt_acl_state_s *bt_acl_state)
{
  /* If A2DP is disconnected, this function will call
   * Print the A2DP connected message
   */
  printf("%s [BT_A2DP] A2DP disconnected\n", __func__);
}

static void on_a2dp_media_received(struct bt_acl_state_s *bt_acl_state,
                                 uint8_t *data, int len)
{
   /* If media package is arrived, this function will call
    * Print media's length
    */
   printf("%s [BT_A2DP] Media length = %d\n", __func__, len);
}


/* BT AVRCP callbacks */

static void on_AVRCP_command_status(BT_CMD_STATUS status)
{
  /* If AT command has been run, this function will call
   * Print the result of AT command
   */
  printf("%s [BT_AVRCP] Command status = %d\n", __func__, status);
}

static void on_AVRCP_connected(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_ROLE role)
{
  /* If AVRCP is connected, this function will call
   * Print the AVRCP connected message
   */
  printf("%s [BT_AVRCP] AVRCP connected. Role: %d\n", __func__, role);
}

static void on_AVRCP_disconnected(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_ROLE role)
{
  /* If AVRCP is disconnected, this function will call
   * Print the AVRCP disconnected message
   */
  printf("%s [BT_AVRCP] AVRCP disconnected. Role: %d\n", __func__, role);
}

static void on_play_status_change(BT_AVRC_TRACK_INFO *trackInfo)
{
  printf("%s [BT_AVRCP] AVRCP play status change. AttrId = %u, attrLen = %u, attrValue = %s.\n",
           __func__, trackInfo->attrId, trackInfo->attrLen, trackInfo->attrValue);
}

static void on_track_change(BT_AVRC_TRACK_INFO *trackInfo)
{
  printf("%s [BT_AVRCP] AVRCP on_track_change. AttrId = %u, attrLen = %u, attrValue = %s.\n",
           __func__, trackInfo->attrId, trackInfo->attrLen, trackInfo->attrValue);
}

static void on_play_pos_changed(uint8_t *pdata, int len)
{
  uint32_t pos = *pdata;
  printf(" %s play position: %d\n", __func__, pos);
}

static void on_app_setting_change(uint8_t *pdata, int len)
{
  printf("%s [BT_AVRCP] AVRCP setting changed.\n", __func__);
}

static void on_volume_change(uint8_t *pdata, int len)
{
  printf("%s [BT_AVRCP] AVRCP volume changed.\n", __func__);
}

/* BT A2DP_SNK command function */

static void bt_connect_a2dp_sink(int argc, char *argv[])
{
   ASSERT(s_bt_acl_state);

   if (bt_a2dp_connect(s_bt_acl_state))
     {
       printf("%s Command error.\n", __func__);
     }
}

static void bt_disconnect_a2dp_sink(int argc, char *argv[])
{
   ASSERT(s_bt_acl_state);

   if (bt_a2dp_disconnect(s_bt_acl_state))
     {
       printf("%s Command error.\n", __func__);
     }
}

/* BT AVRCP function */

static void bt_connect_avrc_controller(int argc, char *argv[])
{
  ASSERT(s_bt_acl_state);

  if (bt_avrcp_connect(s_bt_acl_state))
    {
      printf("%s Command error.\n", __func__);
    }
}
static void bt_disconnect_avrc_controller(int argc, char *argv[])
{
  ASSERT(s_bt_acl_state);

  if (bt_avrcp_disconnect(s_bt_acl_state))
    {
      printf("%s Command error.\n", __func__);
    }
}

static void bt_avrc_controller_send_command(int argc, char *argv[])
{
  ASSERT(s_bt_acl_state);

  if (argc < 2)
    {
      printf("%s Param error (%d)\n", __func__, argc);
      goto bye;
    }

  if (bt_avrcp_send_command(s_bt_acl_state, atoi(argv[1]), true))
    {
      printf("%s Command error.\n", __func__);
    }

bye:
  return;
}

static void bt_a2dp_snk_exit(void)
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

static void main_loop(void)
{
  char buffer[50] = {0};
  char *token = NULL;
  char *argv[10] = {NULL};
  char *str = NULL;
  char *save_str = NULL;
  int i = 0;
  ssize_t len = -1;
  ssize_t num = -1;

  for(; ; token = NULL)
    {
      printf("a2dp_snk>");
      fflush(stdout);

      memset(buffer, sizeof(buffer), 0);
      memset(argv, sizeof(argv), 0);

      len = readline(buffer, sizeof(buffer) - 1, stdin, stdout);
      if (len <= 0)
        {
          printf("Input error\n");
          continue;
        }

      if (strcmp(buffer, "quit\n") == 0)
        {
          printf("Quit\n");
          break;
        }

      buffer[len] = '\0';

      for (num = 0, str = buffer; num < sizeof(argv); num++, str = NULL)
        {
          token = strtok_r(str, " \n", &save_str);
          if (token == NULL)
            break;

          argv[num] = token;
        }

      for (i = 0; argv[0] != NULL && i < ARRAY_SIZE(btA2dpCmds); i++)
        {
            if (strncmp(argv[0], btA2dpCmds[i].cmd, sizeof(buffer)) == 0
                    && btA2dpCmds[i].pFunc != NULL)
              {
                btA2dpCmds[i].pFunc(num, &argv[0]);
              }
        }
    }
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * bt_a2dp_snk_main
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

  ret = bt_set_name(local_name);
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

  /* Register A2DP ops */

  ret = bt_a2dp_register_callback(&bt_a2dp_ops);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Register A2DP callback failed. ret = %d\n", __func__, ret);
    }

  /* Register AVRCP ops */

  ret = bt_avrcp_register_cb(&bt_avrcp_ops);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Register AVRCP callback failed. ret = %d\n", __func__, ret);
    }

  ret = bt_register_notification(&bt_avrcp_notify_ops);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Register AVRCP notification callback failed. ret = %d\n", __func__, ret);
    }

  /* Start pairing mode */

  ret = bt_pairing_enable();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Pairing enable failed. ret = %d\n", __func__, ret);
      goto error;
    }

  bt_spp_set_uuid(NULL);

  /* Set visible from other devices */

  ret = bt_set_visibility(BT_VIS_DISCOVERY_CONNECTABLE);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Set visible failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Start inquiry */

  ret = bt_start_inquiry();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Start inquiry failed. ret = %d\n", __func__, ret);
      goto error;
    }

  main_loop();

  bt_a2dp_snk_exit();

error:
  return ret;
}
