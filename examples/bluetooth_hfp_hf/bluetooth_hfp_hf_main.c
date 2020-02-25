/****************************************************************************
 * bluetooth_hfp_hf/bluetooth_hfp_hf_main.c
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
#include "system/readline.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/bt_spp.h>
#include <bluetooth/bt_hfp_features.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/bt_hfp.h>

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
}BtHfpHfCmd;

/* BT common callbacks */

static void onCommandStatus(BT_CMD_STATUS status);                      /**< Command status */
static void onPairingComplete(BT_ADDR addr, BT_PAIR_STATUS status);     /**< Pairing complete */
static void onInquiryResult(BT_ADDR addr, char *name);                  /**< Inquiry data result */
static void onInquiryComplete(void);                                    /**< Coplete inquiry */
static void onConnectStatusChanged(struct bt_acl_state_s *bt_acl_state,
                                    bool connected, int status);        /**< Connection status change */
static void onConnectedDeviceName(const char *name);                    /**< Device name change */
static void onBondInfo(BT_ADDR addr);                                   /**< Bonding information */

/* HFP callbacks */

static void onHfpCommandStatus(BT_CMD_STATUS status);                   /**< HFP command status */
static void onHfpConnect(struct bt_acl_state_s *bt_acl_state,
                            BT_PROFILE_TYPE btProfileType);             /**< HFP connect */
static void onHfpDisconnect(struct bt_acl_state_s *bt_acl_state);       /**< HFP disconnect*/
static void onHfpAudioConnected(struct bt_acl_state_s *bt_acl_state);   /**< HFP audio connect */
static void onHfpAudioDisconnected(struct bt_acl_state_s *bt_acl_state);/**< HFP audio disconnect */
static void onHfpFeature(struct bt_acl_state_s *bt_acl_state,
                            BT_HFP_AG_FEATURE_FLAG feature);            /**< HFP AG feature */
static void onHfAtResponse(struct bt_acl_state_s *bt_acl_state,
                            char *at_resp);                             /**< HFP at response */

/* SpritzerCommand Functions */

static void btConnectHf(int argc, char *argv[]);
static void btDisconnectHf(int argc, char *argv[]);
static void btConnectHfAudio(int argc, char *argv[]);
static void btDisconnectHfAudio(int argc, char *argv[]);
static void btButtonPress(int argc, char *argv[]);
static void btSendAtCommand(int argc, char *argv[]);
static void btGetBondlist(int argc, char *argv[]);
static void btUnbond(int argc, char *argv[]);

static int connectHF(struct bt_acl_state_s *bt_acl_state);
static void bt_hfp_hf_exit(void);

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

static struct bt_hfp_ops_s bt_hfp_ops =
  {
    .command_status     = onHfpCommandStatus,
    .connect            = onHfpConnect,
    .disconnect         = onHfpDisconnect,
    .audio_connect      = onHfpAudioConnected,
    .audio_disconnect   = onHfpAudioDisconnected,
    .ag_feature         = onHfpFeature,
    .hf_at_response     = onHfAtResponse,
  };

const static BtHfpHfCmd btHfCmds[] =
  {
    { "connectHf",          "",           btConnectHf,             "Emulate BT event" },
    { "disconnectHf",       "",           btDisconnectHf,          "Emulate BT event" },
    { "connectAudio",       "",           btConnectHfAudio,        "Emulate BT event" },
    { "disconAudio",        "",           btDisconnectHfAudio,     "Emulate BT event" },
    { "buttonPress",        "",           btButtonPress,           "Emulate BT event" },
    { "sendAtComm",         "",           btSendAtCommand,         "Emulate BT event" },
    { "getbondlist",        "",           btGetBondlist,           "Emulate BT event" },
    { "unbond",             "",           btUnbond,                "Emulate BT event" }
  };

static BT_ADDR local_addr           = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};
static char local_name[BT_NAME_LEN] = "SONY_BT_HFP_HF_SAMPLE";
struct bt_acl_state_s *s_bt_acl_state;

/****************************************************************************
 * SpritzerCommand Map

command/Arguments          Description

connectHf                  Connect handsfree/headset
disconnectHf               Disconnect handsfree/headset
connectHf                  Connect handsfree audio
disconnectHf               Disconnect handsfree audio
buttonPress                Headset button press
sendAtComm/arg1/arg2/arg3  Send At command,
                           arg1:                                        arg2:                           arg3:
                           32: set speaker gain volume                  0-15                            -
                           33: set microphone gain volume               0-15                            -
                           34: answer incoming call                     -                               -
                           35: Get number from voice tag                1                               -
                           36: Voice recognition                        0: enable                       -
                                                                        1: disable
                           37: Last number redial                       -                               -
                           38: Call hold                                0: release all held calls       -
                                                                        1: release all active calls
                                                                        2: swap active and held calls
                                                                        3: heldactive calls
                           39: Hang up                                  -                               -
                           40: Read indicator status                    -                               -
                           41: Retrieve subscriber number               -                               -
                           42: Dial                                     -                               Number to dial terminated by a semicolon
                           43: Noise/Echo control                       0: disable                      -
                                                                        1: disable                      -
                           44: Dial                                     -                               Digits to transmit via DTMF
                           47: Enable/disable extended AG result codes  0: disable                      -
                                                                        1: enable                       -
                           48: Query list of current calls in AG        -                               -
                           51: Send proprietary AT command              -                               The full AT command string
getbondlist                Get bt bond list, include the index and address
unbond/arg1                Release the specified BT device, arg1:
                           The bond BT device index.
 ****************************************************************************/

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

  printf("%s [BT] Inquiry complete\n");

}

static void onConnectStatusChanged(struct bt_acl_state_s *bt_acl_state,
                                    bool connected, int status)
{
  /* If ACL is connected, HFP_AG can start connect */

  printf("%s [BT] Connect status changed\n", __func__);

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

  printf("[BT] Bonding information ADDR:%02X:%02X:%02X:%02X:%02X:%02X\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5]);
}


/* HFP callbacks */

static void onHfpCommandStatus(BT_CMD_STATUS status)
{
  /* After HFP command is sendt, this function will call. */
  printf("%s [BT_HFP] Command status = %d\n", __func__, status);
}

static void onHfpConnect(struct bt_acl_state_s *bt_acl_state, BT_PROFILE_TYPE btProfileType)
{
  /* If HFP connection is finished, this function will call. */

  printf("%s [BT_HFP] HFP_AG connected\n", __func__);
}

static void onHfpDisconnect(struct bt_acl_state_s *bt_acl_state)
{
  /* If HFP disconnection is finished, this function will call. */

  printf("%s [BT_HFP] HFP_AG disconnected\n", __func__);
}

static void onHfpAudioConnected(struct bt_acl_state_s *bt_acl_state)
{
  /* If HFP Audio connection is finished, this function will call. */

  printf("%s [BT_HFP] HFP_AG Audio connected\n", __func__);
}

static void onHfpAudioDisconnected(struct bt_acl_state_s *bt_acl_state)
{
  /* If HFP Audio disconnection is finished, this function will call. */

  printf("%s [BT_HFP] HFP_AG Audio disconnected\n", __func__);
}

static void onHfpFeature(struct bt_acl_state_s *bt_acl_state, BT_HFP_AG_FEATURE_FLAG feature)
{
  /* If AG feature has been changed, this function will call. */

  printf("%s [BT_HFP] AG Feature changed\n", __func__);
}

static void onHfAtResponse(struct bt_acl_state_s *bt_acl_state, char *at_resp)
{
  /* If HF response, this function will call. */

  printf("%s [BT_HFP] HF at response\n", __func__);
}

static void btConnectHf(int argc, char *argv[])
{
  ASSERT(s_bt_acl_state);

  if (connectHF(s_bt_acl_state) < 0)
    {
      printf("%s Command error\n", __func__);
    }
}

static void btDisconnectHf(int argc, char *argv[])
{
  ASSERT(s_bt_acl_state);

  if (bt_hfp_is_supported())
    {
      if (bt_hfp_disconnect(s_bt_acl_state))
        {
          printf("%s Command error\n", __func__);
        }
    }
}

static void btConnectHfAudio(int argc, char *argv[])
{
  ASSERT(s_bt_acl_state);

  if (bt_hfp_audio_connect(s_bt_acl_state) < 0)
    {
      printf("%s Command error\n", __func__);
    }
}

static void btDisconnectHfAudio(int argc, char *argv[])
{
  ASSERT(s_bt_acl_state);

  if (bt_hfp_audio_disconnect(s_bt_acl_state) < 0)
    {
      printf("%s Command error\n", __func__);
    }
}

static void btButtonPress(int argc, char *argv[])
{
  ASSERT(s_bt_acl_state);

  if (bt_hfp_press_button(s_bt_acl_state) < 0)
    {
      printf("%s Command error\n", __func__);
    }
}

static void btSendAtCommand(int argc, char *argv[])
{
  int i = 0;
  size_t offset = -1;
  size_t len = -1;
  char param[40] = {0};

  ASSERT(s_bt_acl_state);

  if (argc < 2)
    {
      printf("%s Param error (%d)\n", __func__, argc);
      goto bye;
    }

  for (i = 1, offset = 0; argv[i] && offset < sizeof(param); i++, offset += len)
    {
      len = snprintf(param + offset, sizeof(param) - offset, "%s ", argv[i]);
      if (len < 0)
        {
          printf("%s Param format error (%d).\n", __func__, len);
          goto bye;
        }
    }
  param[sizeof(param) - 1] = '\0';

  if (bt_hfp_send_at_command(s_bt_acl_state, param) < 0)
    {
      printf("%s Command error\n", __func__);
    }

bye:
  return;
}

static void btGetBondlist(int argc, char *argv[])
{
  int num = -1;

  ASSERT(s_bt_acl_state);

  if (bt_get_bond_list(&local_addr, &num) < 0)
    {
      printf("%s Command error\n", __func__);
    }
  else
    {
      printf("Bond list number %d\n", num);
    }
}

static void btUnbond(int argc, char *argv[])
{
  ASSERT(s_bt_acl_state);

  if (bt_unbond(&local_addr) < 0)
    {
      printf("%s Command error\n", __func__);
    }
}

static int connectHF(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;

  /* Check HAL supported HFP */

  if (bt_hfp_is_supported())
    {
      /* Start to connect HFP */

      ret = bt_hfp_connect(bt_acl_state);
      if (ret != BT_SUCCESS)
        {
          printf("%s Connection request failed. ret = %d\n", __func__, ret);
        }
    }
  return ret;
}

static void bt_hfp_hf_exit(void)
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
      printf("hfp_hf>");
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

      for (i = 0; argv[0] != NULL && i < ARRAY_SIZE(btHfCmds); i++)
        {
            if (strncmp(argv[0], btHfCmds[i].cmd, sizeof(buffer)) == 0
                    && btHfCmds[i].pFunc != NULL)
              {
                btHfCmds[i].pFunc(num, &argv[0]);
              }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * bt_spp_main
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

  /* Set HFP feature */

  ret = bt_hfp_set_feature(BT_HFP_HF_FEATURE_3WAY_CALLING
                             | BT_HFP_HF_FEATURE_CLIP_CAPABILITY
                             | BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION
                             | BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL
                             | BT_HFP_HF_FEATURE_CODEC_NEGOTIATION);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Set HFP feature failed. ret = %d\n", __func__, ret);
    }

  /* Register HFP callback function */

  ret = bt_hfp_register_cb(&bt_hfp_ops);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Register callback failed. ret = %d\n", __func__, ret);
    }

  /* Start pairing mode */

  ret = bt_pairing_enable();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Pairing enable failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Set SPP UUID */

  ret = bt_spp_set_uuid(NULL);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Set SPP UUID failed. ret = %d\n", __func__, ret);
    }

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

  bt_hfp_hf_exit();

error:
  return ret;
}
