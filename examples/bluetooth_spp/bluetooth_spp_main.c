/****************************************************************************
 * bluetooth_spp/bluetooth_main.c
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
#include <bluetooth/bt_spp.h>

#include "system/readline.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPP_MAX_TX_DATA_SIZE 1024

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

/* SPP callbacks */

static void onSppConnect(struct bt_acl_state_s *bt_acl_state);        /**< Connection status */
static void onSppDisconnect(struct bt_acl_state_s *bt_acl_state);     /**< Disconnection status */
static void onSppConnectionFail(struct bt_acl_state_s *bt_acl_state,
                                  BT_CONNECT_FAIL_REASON_ID fail_id); /**< Connection fail */
static void onSppReceiveData(struct bt_acl_state_s *bt_acl_state,
                              uint8_t *data, int len);                /**< Receive SPP data */

static int connectSPP(struct bt_acl_state_s *bt_acl_state);
static void bt_spp_exit(void);

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

static struct bt_spp_ops_s bt_spp_ops =
  {
    .connect         = onSppConnect,
    .disconnect      = onSppDisconnect,
    .connection_fail = onSppConnectionFail,
    .receive_data    = onSppReceiveData
  };

static BT_ADDR local_addr           = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};

static char local_name[BT_NAME_LEN] = "SONY_BT_SPP_SAMPLE";

static struct bt_acl_state_s *s_bt_acl_state = NULL;

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

  printf("[BT_SPP] Pairing complete ADDR:%02X:%02X:%02X:%02X:%02X:%02X, status=%d\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5],
          status);
}

static void onInquiryResult(BT_ADDR addr, char *name)
{
  /* If receive inquiry search result, this function will call. */

  printf("[BT_SPP] Inquiry result ADDR:%02X:%02X:%02X:%02X:%02X:%02X, name:%s\n",
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

  if (connected)
    {
      /* Start to connect SPP */

      connectSPP(bt_acl_state);
    }
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

  printf("[BT_SPP] Bonding information ADDR:%02X:%02X:%02X:%02X:%02X:%02X\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5]);
}

static void onSppConnect(struct bt_acl_state_s *bt_acl_state)
{
  /* If SPP connection is finished, this function will call. */

  printf("%s [BT] SPP connected\n", __func__);
}

static void onSppDisconnect(struct bt_acl_state_s *bt_acl_state)
{
  /* If SPP connection is finished, this function will call. */

  printf("%s [BT] SPP Disconnected\n", __func__);
}

static void onSppConnectionFail(struct bt_acl_state_s *bt_acl_state,
                                  BT_CONNECT_FAIL_REASON_ID fail_id)
{
  /* If SPP connection is finished, this function will call. */

  printf("%s [BT] SPP Connection failed reason = %d\n", __func__, fail_id);
}

static void onSppReceiveData(struct bt_acl_state_s *bt_acl_state,
                              uint8_t *data, int len)
{
  int ret;
  /* If receive SPP data, this function will call. */

  printf("%s [BT] Receive Data data[0] = 0x%02X, Length = %d \n", __func__, data[0], len);

  /* Loop back */

  ret = bt_spp_send_tx_data(bt_acl_state, data, len);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Send data failed. ret = %d\n", __func__, ret);
    }
}

static int connectSPP(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;

  /* Check HAL supported SPP */

  if (bt_spp_is_supported())
    {
      /* Register SPP callback function */

      ret = bt_spp_register_cb(&bt_spp_ops);
      if (ret != BT_SUCCESS)
        {
          printf("%s [BT] Register callback failed. ret = %d\n", __func__, ret);
        }

      /* Start to connect SPP */

      ret = bt_spp_connect(bt_acl_state);
      if (ret != BT_SUCCESS)
        {
          printf("%s [BT] Connection request failed. ret = %d\n", __func__, ret);
        }
    }
  return ret;
}

static void bt_spp_exit(void)
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
 * bt_spp_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret = 0;
  int len = 0;
  char buffer[SPP_MAX_TX_DATA_SIZE] = {0};

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

  /* Start pairing mode */

  ret = bt_pairing_enable();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Pairing enable failed. ret = %d\n", __func__, ret);
      goto error;
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

  /* Send Tx data by using readline */

  while(1)
    {
      printf("spp>");
      fflush(stdout);

      len = readline(buffer, sizeof(buffer) - 1, stdin, stdout);

      if (s_bt_acl_state && s_bt_acl_state->bt_acl_connection == BT_CONNECTED)
        {
          ret = bt_spp_send_tx_data(s_bt_acl_state, (uint8_t *) buffer, len);
          if (ret != BT_SUCCESS)
            {
              printf("%s [BT] Send data failed. ret = %d\n", __func__, ret);
            }
        }

      if (!strcmp(buffer, "quit\n"))
        {
          printf("Quit.");
          break;
        }
    }

  /* Quit this application */

  bt_spp_exit();

error:
  return ret;
}
