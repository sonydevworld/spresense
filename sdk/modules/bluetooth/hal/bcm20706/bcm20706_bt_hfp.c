/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bcm20706_bt_hfp.c
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

#include <stdlib.h>
#include <stdio.h>
#include "bt_util.h"
#include <bt/bt_hfp_hf.h>
#include "manager/bt_uart_manager.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int btHfConnect(BT_ADDR *addr)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_HF_COMMAND_CONNECT);
  UINT16_TO_STREAM(p, BT_ADDR_LEN);
  memcpy(p, addr, BT_ADDR_LEN);
  p += BT_ADDR_LEN;
  return btUartSendData(buff, p-buff);
}

static int btHfDisconnect(uint16_t handle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_HF_COMMAND_DISCONNECT);
  UINT16_TO_STREAM(p, sizeof(uint16_t));
  UINT16_TO_STREAM(p, handle);
  return btUartSendData(buff, p-buff);
}

static int btHfConnectAudio(uint16_t handle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_HF_COMMAND_OPEN_AUDIO);
  UINT16_TO_STREAM(p, sizeof(uint16_t));
  UINT16_TO_STREAM(p, handle);
  return btUartSendData(buff, p-buff);
}

static int btHfDisconnectAudio(uint16_t handle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_HF_COMMAND_CLOSE_AUDIO);
  UINT16_TO_STREAM(p, sizeof(uint16_t));
  UINT16_TO_STREAM(p, handle);
  return btUartSendData(buff, p-buff);
}

static int btHfSetSupportedFeature(uint32_t *btHfFeature)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_HF_COMMAND_SET_FEATURE);
  UINT16_TO_STREAM(p, sizeof(uint32_t));
  UINT32_TO_STREAM(p, *btHfFeature);
  btUartSendData(buff, p - buff);
  return 0;
}

static int btHfSendATCommand(uint16_t handle, uint16_t code, uint16_t numeric, char *optionalString)
{
  uint8_t buff[BT_LONG_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  size_t atSize = (optionalString ? strnlen(optionalString, BT_HF_MAX_OPTIONAL_STRING_LEN - 1) : 0);
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, (BT_CONTROL_GROUP_HF << 8) | code);
  UINT16_TO_STREAM(p, sizeof(uint16_t) + sizeof(uint16_t) + atSize + 1);
  UINT16_TO_STREAM(p, handle);
  UINT16_TO_STREAM(p, numeric);
  memcpy(p, (uint8_t*)optionalString, atSize);
  p[atSize] = '\0';
  p += atSize + 1;
  return btUartSendData(buff, p - buff);
}

static int btHfButtonPress(uint16_t handle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_HF_COMMAND_BUTTON_PRESS);
  UINT16_TO_STREAM(p, sizeof(uint16_t));
  UINT16_TO_STREAM(p, handle);
  return btUartSendData(buff, p - buff);
}


/****************************************************************************
 * Name: bcm20706_bt_hfp_connect
 *
 * Description:
 *   Bluetooth HFP connect/disconnect.
 *   Connect/Disconnect HFP with target device address.
 *
 ****************************************************************************/

static int bcm20706_bt_hfp_connect(BT_ADDR *addr, uint16_t handle, bool connect)
{
  int ret = BT_SUCCESS;

  if (connect)
    {
      /* Connect */

      ret = btHfConnect(addr);
    }
  else
    {
      /* Disconnect */

      ret = btHfDisconnect(handle);
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_hfp_audio_connect
 *
 * Description:
 *   Bluetooth HFP audio connect/disconnect.
 *   Connect/Disconnect HFP audio (This function must call after
 *   bcm20706_bt_hfp_connect).
 *
 ****************************************************************************/

static int bcm20706_bt_hfp_audio_connect(BT_ADDR *addr, uint16_t handle, bool connect)
{
  int ret = BT_SUCCESS;

  if (connect)
    {
      /* Connect */

      ret = btHfConnectAudio(handle);
    }
  else
    {
      /* Disconnect */

      ret = btHfDisconnectAudio(handle);
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_hfp_hf_feature
 *
 * Description:
 *   Bluetooth HFP set HF feature.
 *   Set hf feature by support flag.
 *
 ****************************************************************************/

static int bcm20706_bt_hfp_hf_feature(BT_HFP_HF_FEATURE_FLAG hf_heature)
{
  int ret = BT_SUCCESS;
  uint32_t flag = (uint32_t) hf_heature;
  

  /* Flag structure is same as official */

  ret = btHfSetSupportedFeature(&flag);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_hfp_send_at_command
 *
 * Description:
 *   Send AT command to BT
 *
 ****************************************************************************/

static int bcm20706_bt_hfp_send_at_command(BT_ADDR *addr, char *at_str, uint16_t handle)
{
  int ret = BT_SUCCESS;
  int i = 0;
  uint16_t code = 0;
  uint16_t numberic = 0;
  char cmd[20] = {0};
  char *str = NULL;
  char *save_ptr = NULL;
  char *token = NULL;
  char *str_opt = NULL;

  strncpy(cmd, at_str, sizeof(cmd));
  cmd[sizeof(cmd)-1] = '\0';

  /* AT command '<code> [numberic] [str_opt]' */

  for (i = 0, str = cmd; ; i++, str = NULL)
    {
      token = strtok_r(str, " \n", &save_ptr);
      if (token == NULL)
        break;

      switch(i)
        {
          case 0: /* <code> */
            code = atoi(token);
            break;

          case 1: /* [numberic] */
            numberic = atoi(token);
            break;

          case 2: /* [str_opt] */
            if (code == BT_HF_AT_COMMAND_SPK || code == BT_HF_AT_COMMAND_CHUP
                || code == BT_HF_AT_COMMAND_MAX)
              {
                printf("%s Unexpected param\n", __func__);
                ret = BT_FAIL;
                goto bye;
              }
            str_opt = token;
            break;

          default:
            break;
        }

    }


  if (btHfSendATCommand(handle, code, numberic, str_opt) < 0)
    {
      printf("%s Send AT command failed\n", __func__);
      ret = BT_FAIL;
    }

bye:
  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_hfp_press_button
 *
 * Description:
 *   Send pressing button command to BT
 *
 ****************************************************************************/

static int bcm20706_bt_hfp_press_button(BT_ADDR *addr, uint16_t handle)
{
  int ret = BT_SUCCESS;

  ret = btHfButtonPress(handle);
  if (ret < 0)
    {
      printf("%s Send pressing button failed\n", __func__);
    }

  return ret;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct bt_hal_hfp_ops_s bt_hal_hfp_ops =
{
  .connect         = bcm20706_bt_hfp_connect,
  .audio_connect   = bcm20706_bt_hfp_audio_connect,
  .set_hf_feature  = bcm20706_bt_hfp_hf_feature,
  .send_at_command = bcm20706_bt_hfp_send_at_command,
  .press_button    = bcm20706_bt_hfp_press_button,
};
