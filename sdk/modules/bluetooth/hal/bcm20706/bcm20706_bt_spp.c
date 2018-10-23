/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bcm20706_bt_spp.c
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

#include <bt/bt_spp.h>

#include "bcm20706_bt_internal.h"
#include "bt_util.h"
#include "manager/bt_uart_manager.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int btSppConnect(BT_ADDR *addr)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_SPP_COMMAND_CONNECT);
  UINT16_TO_STREAM(p, BT_ADDR_LEN);
  memcpy(p, addr, BT_ADDR_LEN);
  p += BT_ADDR_LEN;
  return btUartSendData(buff, p - buff);
}

static int btSppDisconnect(uint16_t handle)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_SPP_COMMAND_DISCONNECT);
  UINT16_TO_STREAM(p, 2);
  UINT16_TO_STREAM(p, handle);
  return btUartSendData(buff, p - buff);
}

static int btSppSetUuid(BT_SPP_UUID *uuid)
{
  uint8_t buff[BT_MID_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_SPP_COMMAND_UUID);
  UINT16_TO_STREAM(p, BT_SPP_UUID128_LEN);
  memcpy(p, uuid, BT_SPP_UUID128_LEN);
  p += BT_SPP_UUID128_LEN;
  return btUartSendData(buff, p - buff);
}

static int sppSendData(uint16_t handle,uint8_t *data,uint16_t len)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  int ret = 0;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_SPP_COMMAND_DATA);
  UINT16_TO_STREAM(p, 2 + len);
  UINT16_TO_STREAM(p, handle);
  ret = btUartSendData(buff, p - buff);
  return (ret | btUartSendData(data, len));
}

/****************************************************************************
 * Name: bcm20706_bt_spp_connect
 *
 * Description:
 *   Bluetooth Connect/Disconnect SPP.
 *   Connect/Disconnect SPP from target device.
 *
 ****************************************************************************/

static int bcm20706_bt_spp_connect(BT_ADDR *addr, uint16_t handle, bool connect)
{
  int ret = BT_SUCCESS;

  if (connect)
    {
      /* Connect */

      ret = btSppConnect(addr);
    }
  else
    {
      /* Disconnect */

      ret = btSppDisconnect(handle);
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_spp_set_uuid
 *
 * Description:
 *   Bluetooth set SPP unique UUID.
 *   Set unique SPP UUID.
 *
 ****************************************************************************/

static int bcm20706_bt_spp_set_uuid(BT_UUID *uuid)
{
  int ret = BT_SUCCESS;
  int n;
  BT_SPP_UUID spp_uuid;

  /* Copy UUID from Framework to HAL */

  for (n = 0; n < BT_SPP_UUID128_LEN; n ++)
    {
      spp_uuid.uuid128[n] = uuid->uuid128[n];
    }

  ret = btSppSetUuid(&spp_uuid);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_spp_send_tx_data
 *
 * Description:
 *   Bluetooth send Tx Data.
 *   Send data for target device.
 *
 ****************************************************************************/

static int bcm20706_bt_spp_send_tx_data(BT_ADDR *addr, uint8_t *data, int len, uint16_t handle)
{
  int ret = BT_SUCCESS;

  ret = sppSendData(handle, data, len);

  return ret;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct bt_hal_spp_ops_s bt_hal_spp_ops =
{
  .connect    = bcm20706_bt_spp_connect,
  .setUuid    = bcm20706_bt_spp_set_uuid,
  .sendTxData = bcm20706_bt_spp_send_tx_data
};

