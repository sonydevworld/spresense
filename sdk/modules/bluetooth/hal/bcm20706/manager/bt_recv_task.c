/****************************************************************************
 * modules/bluetooth/hal/bcm20706/manager/bt_recv_task.c
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
#include <pthread.h>
#include <debug.h>
#include <bt/bt_comm.h>
#include <bt/bt_a2dp_sink.h>
#include <bt/bt_avrc_tar.h>
#include <bt/bt_avrc_con.h>
#include <ble/ble_comm.h>
#include <arch/board/board.h>
#include <assert.h>

#include "manager/bt_freq_lock.h"
#include "manager/bt_uart_manager.h"
#include "bt_util.h"
#include "bt_debug.h"
#include "bcm20706_ble_internal.h"
#include "bcm20706_bt_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DBG_LOG_DEBUG btdbg
#define DBG_LOG_ERROR btdbg

#define HANDLE_SIZE    2
#define BT_RESULT_SIZE 2
#define NVRAM_ID_LEN   2
#define HASH_POS_IN_VERSION 8
#define BLE_ADV_RSSI_LEN  1

struct bt_hfp_event_t
{
  union _head
  {
    uint8_t command_status;
    uint16_t handle;
  } head;

  union _event
  {
    struct _hf_open_event
    {
      uint8_t addr[BT_ADDR_LEN];
      uint8_t status;
    } hf_open_event;

    struct _hf_ag_indicator
    {
      uint8_t numberic[2];
      char opt_str[4];
    } hf_ag_indicator;

    uint8_t profile_type;
    uint16_t ag_feature;
  } event;

} __attribute__((__packed__));

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_t gRecvTask;
static BT_EVT bt_evt_buff;
uint8_t *appEvtBuff = (uint8_t *) &bt_evt_buff;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void btRecvDeviceStatus(uint8_t *p, uint16_t len, int group)
{
  struct bt_event_cmd_stat_t cmd_stat_evt;
  
  memcpy(&cmd_stat_evt.cmd_status, p,  sizeof(cmd_stat_evt.cmd_status));

  switch (group)
    {
      case BT_CONTROL_GROUP_DEVICE:
        cmd_stat_evt.group_id = BT_GROUP_COMMON;
        cmd_stat_evt.event_id = BT_COMMON_EVENT_CMD_STATUS;
        bt_common_event_handler((struct bt_event_t *) &cmd_stat_evt);
        break;

      case BT_CONTROL_GROUP_HF:
        cmd_stat_evt.group_id = BT_GROUP_HFP;
        cmd_stat_evt.event_id = BT_HFP_EVENT_CMD_STATUS;
        bt_hfp_event_handler((struct bt_event_t *) &cmd_stat_evt);
        break;

      case BT_CONTROL_GROUP_AVRC_TARGET:
      case BT_CONTROL_GROUP_AVRC_CONTROLLER:
        cmd_stat_evt.group_id = BT_GROUP_AVRCP;
        cmd_stat_evt.event_id = BT_AVRCP_EVENT_CMD_STATUS;
        bt_avrcp_event_handler((struct bt_event_t *) &cmd_stat_evt);
        break;

      case BT_CONTROL_GROUP_A2DP_SINK:
      case BT_CONTROL_GROUP_AUDIO_SINK:
        cmd_stat_evt.group_id = BT_GROUP_A2DP;
        cmd_stat_evt.event_id = BT_A2DP_EVENT_CMD_STATUS;
        bt_a2dp_event_handler((struct bt_event_t *) &cmd_stat_evt);
        break;

      default:
        break;
    }
}

static void btSaveBondInfo(BT_ADDR *addr)
{
  BLE_GapBondInfo info = {0};

  info.addrType = 0; /* Only use in BLE */
  memcpy(info.addr, addr, BT_ADDR_LEN);

  BLE_GapSaveBondInfo(&info);
}

static void btRecvBondInfo(uint8_t *p, uint16_t len)
{
  uint8_t *rp = NULL;
  struct bt_event_bond_info_t bond_info_evt;

  /* Copy target device address */
  rp = p + NVRAM_ID_LEN;
  memcpy(&bond_info_evt.addr, rp, BT_ADDR_LEN);

  bond_info_evt.group_id = BT_GROUP_COMMON;
  bond_info_evt.event_id = BT_COMMON_EVENT_BOND_INFO;

  /* Save bonding information to filesystem */
  btSaveBondInfo(&bond_info_evt.addr);

  bt_common_event_handler((struct bt_event_t *) &bond_info_evt);
}

static void recevNvramData(uint8_t evtCode, uint8_t *p, uint16_t len)
{
  uint8_t *wp = NULL;
  ble_evt_t *pBleBcmEvt = (ble_evt_t*)p;
  wp = appEvtBuff;
  UINT16_TO_STREAM(wp,(BT_CONTROL_GROUP_DEVICE<< 8) | evtCode);

  btdbg("receive nv data,transport = %d, len = %d\n", p[len-1], len);
  if (p[len - 1] == BT_TRANSPORT_BR_EDR)
    {
      btdbg("BT key type = %02x, key = %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, \
          %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n",
          p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15], p[16], p[17], p[18],
          p[19], p[20], p[21], p[22], p[23], p[24]);
      bleRecvNvramData(pBleBcmEvt, len);
      btdbg("receive NV data.\n");
      btRecvBondInfo(p, len);
      btdbg("receive bond information.\n");
    }
  else if (p[len - 1] == BT_TRANSPORT_LE)
    {
      bleRecvNvramData(pBleBcmEvt, len);
      btdbg("BLE ltk = %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, \
          %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n",
          p[52], p[53], p[54], p[55], p[56], p[57], p[58], p[59], p[60], p[61],
          p[62], p[63], p[64], p[65], p[66], p[67]);
    }
}

static void btRecvInquiryResult(uint8_t *p, uint16_t len)
{
#define RSSI_LEN 1
#define TYPE_COMPLETE_NAME 0x09
  uint8_t *rp = NULL;
  uint8_t value_length = 0;
  uint8_t type_id = 0;
  struct bt_event_inquiry_rslt_t inq_rslt_evt;

  /* Get target device address */

  rp = p;
  memcpy(&inq_rslt_evt.addr, rp, BT_ADDR_LEN);

  rp += BT_ADDR_LEN;
  rp += BT_CLASS_LEN;
  rp += RSSI_LEN;

  /* Value length(contain '0') */

  STREAM_TO_UINT8(value_length, rp);

  /* Value type */

  STREAM_TO_UINT8(type_id, rp);

  if (type_id == TYPE_COMPLETE_NAME)
    {
      memcpy(inq_rslt_evt.name, rp, value_length);

      /* Insert '\0' at the end of name */

      inq_rslt_evt.name[value_length - 1] = '\0';
    }
  else
    {
      /* If type ID is not Complete name ID, return null */

      inq_rslt_evt.name[0] = '\0';
    }

  inq_rslt_evt.group_id = BT_GROUP_COMMON;
  inq_rslt_evt.event_id = BT_COMMON_EVENT_INQUIRY_RESULT;

  bt_common_event_handler((struct bt_event_t *) &inq_rslt_evt);
}

static void btRecvInquiryComplete(uint8_t *p, uint16_t len)
{
  struct bt_event_t bt_evt;

  bt_evt.group_id = BT_GROUP_COMMON;
  bt_evt.event_id = BT_COMMON_EVENT_INQUIRY_COMPLETE;

  bt_common_event_handler((struct bt_event_t *) &bt_evt);
}

static void pairingComplete(uint8_t evtCode, uint8_t *p)
{
  struct bt_event_pair_cmplt_t pair_cmplt_evt;
  ble_evt_t *pBleBcmEvt = (ble_evt_t*)p;

  btdbg("pair complete,status = %x, addr = %x,%x, %x, %x, %x, %x, transport = %x\n",
      p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);

  if (((pair_result_t*)p)->transport == BT_TRANSPORT_BR_EDR)
    {
      memcpy(&pair_cmplt_evt.status, p, sizeof(uint8_t));
      p += sizeof(uint8_t);
      STREAM_TO_BDADDR(&pair_cmplt_evt.addr, p);
    }
  else if (((pair_result_t*)p)->transport == BT_TRANSPORT_LE)
    {
      bleRecvAuthStatus(pBleBcmEvt);
    }

  pair_cmplt_evt.group_id = BT_GROUP_COMMON;
  pair_cmplt_evt.event_id = BT_COMMON_EVENT_PAIRING_COMPLETE;

  bt_common_event_handler((struct bt_event_t *) &pair_cmplt_evt);
}

static void btRecvConnDevName(uint8_t evtCode, uint8_t *p)
{
  struct bt_event_dev_name_t dev_name_evt;

  /* Skip status */

  p += 2;

  /* Skip address */

  p += BT_ADDR_LEN;

  /* Copy device name */

  strncpy(dev_name_evt.name, (const char*)p, BT_MAX_NAME_LEN - 1);
  dev_name_evt.name[BT_MAX_NAME_LEN - 1] = '\0';

  dev_name_evt.group_id = BT_GROUP_COMMON;
  dev_name_evt.event_id = BT_COMMON_EVENT_CONN_DEV_NAME;

  bt_common_event_handler((struct bt_event_t *) &dev_name_evt);
}

static void btRecvUserConfirmation(uint8_t *ptr)
{
  BT_REPLY_CONFIRM btReplyConfirm;
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  uint8_t *rp = NULL;
  uint8_t *wp = ((BT_EVT *)appEvtBuff)->evtData;
  BT_EVT_CONFIRMATION_REQ *tncqEvt = (BT_EVT_CONFIRMATION_REQ *)wp;

  /* Get target address */

  rp = ptr;
  memcpy(&tncqEvt->addr, rp, BT_ADDR_LEN);

  /* Get numeric */

  rp += BT_ADDR_LEN;
  memcpy(&tncqEvt->numeric, rp, sizeof(tncqEvt->numeric));

  /* Infuture, will send event to application */

  memcpy(&btReplyConfirm.addr, &tncqEvt->addr, BT_ADDR_LEN);
  btReplyConfirm.btAccept = BT_TRUE;

  /* Send accept command */

  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_REPLY_CONFIRMATION);
  UINT16_TO_STREAM(p, 7);
  memcpy(p, &btReplyConfirm.addr, BT_ADDR_LEN);
  p += BT_ADDR_LEN;
  UINT8_TO_STREAM(p, btReplyConfirm.btAccept);
  btUartSendData(buff, p - buff);
}

static void btRecvAclConnStatus(uint8_t *p, uint16_t len)
{
  struct bt_event_conn_stat_t con_stat_evt;
  uint8_t connect;
  uint8_t type;
  uint8_t reason;

  /* Copy device address */

  memcpy(&con_stat_evt.addr, p, sizeof(BT_ADDR));

  /* Copy connect status */

  p += sizeof(BT_ADDR);
  memcpy(&connect, p, sizeof(uint8_t));
  con_stat_evt.connected = connect == 1;

  /* Copy type */

  p += sizeof(uint8_t);
  memcpy(&type, p, sizeof(uint8_t));

  /* Copy reason */

  p += sizeof(uint8_t);
  memcpy(&reason, p, sizeof(uint8_t));
  con_stat_evt.status = reason;

  con_stat_evt.group_id = BT_GROUP_COMMON;
  con_stat_evt.event_id = BT_COMMON_EVENT_CONN_STAT_CHANGE;

  btdbg("addr: %02x.%02x.%02x.%02x.%02x,%02x is connect:%d,"
      "transport type: %x, disconnect reason: %x.\n",
    con_stat_evt.addr.address[0], con_stat_evt.addr.address[1],
    con_stat_evt.addr.address[2], con_stat_evt.addr.address[3],
    con_stat_evt.addr.address[4], con_stat_evt.addr.address[5],
    connect, type, reason);

  bt_common_event_handler((struct bt_event_t *) &con_stat_evt);
}

static void bleRecvSecurityRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt)
{
  uint8_t *rp     = pBleBcmEvt->evtData;
  BT_ADDR bleAddr = {{0}};
  BLE_GapPairingFeature pfeature;

  /* Setup security feature */

  pfeature.oob        = BLE_GAP_OOB_AUTH_DATA_NOT_PRESENT;
  pfeature.ioCap      = BLE_GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
  pfeature.authReq    = BLE_GAP_AUTH_REQ_NO_MITM_BOND;
  pfeature.minKeySize = BLE_GAP_MIN_KEY_SIZE;
  pfeature.maxKeySize = BLE_GAP_MAX_KEY_SIZE;

  btdbg("notify addr = %x,%x,%x,%x,%x,%x\n",rp[0],rp[1],rp[2],rp[3],rp[4],rp[5]);
  memcpy(&bleAddr, rp, BT_ADDR_LEN);

  /* Reply security feature */

  bleGapReplySecurity(bleAddr, BLE_ENABLE, pfeature);
}

static void btRecvSppEvtConnected(uint8_t *p)
{
  struct bt_spp_event_connect_t connect_evt;

  /* Copy device address */

  memcpy(&connect_evt.addr, p, BT_ADDR_LEN);

  /* Copy Handle ID */

  p += BT_ADDR_LEN;
  STREAM_TO_UINT16(connect_evt.handle, p);

  connect_evt.group_id = BT_GROUP_SPP;
  connect_evt.event_id = BT_SPP_EVENT_CONNECT;

  bt_spp_event_handler((struct bt_event_t *) &connect_evt);
}

static void btRecvSppEvtDisconnect(uint8_t *p)
{
  struct bt_spp_event_connect_t connect_evt;

  connect_evt.group_id = BT_GROUP_SPP;
  connect_evt.event_id = BT_SPP_EVENT_DISCONNECT;

  bt_spp_event_handler((struct bt_event_t *) &connect_evt);
}

static void btRecvSppEvtRxData(uint8_t *p, uint16_t len)
{
  struct bt_spp_event_recv_data_t recv_evt;
  uint16_t handle = 0;

  /* Get handle ID */

  STREAM_TO_UINT16(handle, p);
  (void) handle;

  /* Get data length */
  len = MIN(len, BT_EVT_DATA_LEN);
  recv_evt.len = len - HANDLE_SIZE;

  /* Get data body */
  memcpy(recv_evt.data, p, recv_evt.len);

  recv_evt.group_id = BT_GROUP_SPP;
  recv_evt.event_id = BT_SPP_EVENT_RX_DATA;

  bt_spp_event_handler((struct bt_event_t *) &recv_evt);
}

static void btRecvHfpCommandStatus(uint8_t *p)
{
   const struct bt_hfp_event_t *pack = (const struct bt_hfp_event_t*)p;

   struct bt_event_t app_evt = {0};

   app_evt.group_id = BT_GROUP_HFP;
   app_evt.event_id = BT_HFP_EVENT_CMD_STATUS;
   memcpy(app_evt.data, pack, MIN(sizeof(app_evt.data), sizeof(pack->head.command_status)));

   bt_hfp_event_handler(&app_evt);
}

static void btRecvHfpEvtOpen(uint8_t *p)
{
  const struct bt_hfp_event_t *pack = (const struct bt_hfp_event_t*)p;

  btdbg("connection open handle = %04x, addr = %02x:%02x:%02x:%02x:%02x:%02x, status = %01x\n",
        pack->head.handle,
        pack->event.hf_open_event.addr[0],
        pack->event.hf_open_event.addr[1],
        pack->event.hf_open_event.addr[2],
        pack->event.hf_open_event.addr[3],
        pack->event.hf_open_event.addr[4],
        pack->event.hf_open_event.addr[5],
        pack->event.hf_open_event.status);

}

static void btRecvHfpEvtClose(uint8_t *p)
{
  const struct bt_hfp_event_t *pack = (const struct bt_hfp_event_t*)p;

  struct bt_event_t app_evt = {0};

  btdbg("connection closed handle = %04x\n", pack->head.handle);

  app_evt.group_id = BT_GROUP_HFP;
  app_evt.event_id = BT_HFP_EVENT_HF_DISCONNECT;
  memcpy(app_evt.data, pack, MIN(sizeof(app_evt.data), sizeof(pack->head.handle)));

  bt_hfp_event_handler(&app_evt);
}

static void btRecvHfpAudioEvtOpen(uint8_t *p)
{
   const struct bt_hfp_event_t *pack = (const struct bt_hfp_event_t*)p;

   struct bt_event_t app_evt = {0};

   btdbg("audio open handle = %04x\n", pack->head.handle);

   app_evt.group_id = BT_GROUP_HFP;
   app_evt.event_id = BT_HFP_EVENT_AUDIO_CONNECT;
   memcpy(app_evt.data, pack, MIN(sizeof(app_evt.data), sizeof(pack->head.handle)));

   bt_hfp_event_handler(&app_evt);
}

static void btRecvHfpAudioEvtClose(uint8_t *p)
{
   const struct bt_hfp_event_t *pack = (const struct bt_hfp_event_t*)p;

   struct bt_event_t app_evt = {0};

   btdbg("audio close handle = %04x\n", pack->head.handle);

   app_evt.group_id = BT_GROUP_HFP;
   app_evt.event_id = BT_HFP_EVENT_AUDIO_DISCONNECT;
   memcpy(app_evt.data, pack, MIN(sizeof(app_evt.data), sizeof(pack->head.handle)));

   bt_hfp_event_handler(&app_evt);
}

static void btRecvHfpEvtConnected(uint8_t *p)
{
  const struct bt_hfp_event_t *pack = (const struct bt_hfp_event_t*)p;

  struct bt_event_t app_evt = {0};

  btdbg("connection connected handle = %04x, profile_type = %02x\n",
        pack->head.handle, pack->event.profile_type);

  app_evt.group_id = BT_GROUP_HFP;
  app_evt.event_id = BT_HFP_EVENT_HF_CONNECT;
  memcpy(app_evt.data, pack, MIN(sizeof(app_evt.data), sizeof(pack->event.profile_type)));

  bt_hfp_event_handler(&app_evt);
}

static void btRecvHfpEvtAgFeature(uint8_t *p)
{
  const struct bt_hfp_event_t *pack = (const struct bt_hfp_event_t*)p;

  struct bt_event_t app_evt = {0};

  btdbg("connection AG handle = %04x, feature = %04x\n",
        pack->head.handle, pack->event.ag_feature);

  app_evt.group_id = BT_GROUP_HFP;
  app_evt.event_id = BT_HFP_EVENT_AG_FEATURE_RESP;
  memcpy(app_evt.data, pack, MIN(sizeof(app_evt.data), sizeof(pack->event.ag_feature)));

  bt_hfp_event_handler(&app_evt);
}

static void btRecvHfpEvtAgIndicator(uint8_t *p)
{
  const struct bt_hfp_event_t *pack = (const struct bt_hfp_event_t*)p;

  struct bt_event_t app_evt = {0};
  size_t data_size = 0;

  btdbg("AG indicator handle = %04x, numberic = %02x,%02x, opt_str = '%s'\n",
        pack->head.handle,
        pack->event.hf_ag_indicator.numberic[0],
        pack->event.hf_ag_indicator.numberic[1],
        pack->event.hf_ag_indicator.opt_str);

 data_size = sizeof(pack->head.handle) + sizeof(pack->event.hf_ag_indicator.numberic)
                + sizeof(pack->event.hf_ag_indicator.opt_str);

 app_evt.group_id = BT_GROUP_HFP;
 app_evt.event_id = BT_HFP_EVENT_AT_CMD_RESP;
 memcpy(app_evt.data, pack, MIN(sizeof(app_evt.data), data_size));

 bt_hfp_event_handler(&app_evt);
}

static void btRecvA2dpSnkEvtConnected(uint8_t *p)
{
  struct bt_a2dp_event_connect_t connect_evt = {0};

  /* Copy device address */
  memcpy(&connect_evt.addr, p, BT_ADDR_LEN);
  p += BT_ADDR_LEN;

  /* Copy Handle ID */
  STREAM_TO_UINT16(connect_evt.handle, p);

  connect_evt.group_id = BT_GROUP_A2DP;
  connect_evt.event_id = BT_A2DP_EVENT_CONNECT;

  bt_a2dp_event_handler((struct bt_event_t *) &connect_evt);
}

static void btRecvA2dpAvrcControllerConnected(uint8_t *p)
{
  struct bt_avrcp_event_connect_t connect_evt = {0};

  /* Copy device address */
  memcpy(&connect_evt.addr, p, BT_ADDR_LEN);
  p += BT_ADDR_LEN + 1;

  /* Copy Handle ID */
  STREAM_TO_UINT16(connect_evt.handle, p);

  connect_evt.group_id = BT_GROUP_AVRCP;
  connect_evt.event_id = BT_AVRCC_EVENT_CONNECT;
  bt_avrcp_event_handler((struct bt_event_t *) &connect_evt);
}

static void btRecvA2dpAvrcControllerDisconnected(uint8_t *p)
{
  struct bt_avrcp_event_connect_t connect_evt = {0};

  /* Copy Handle ID */
  STREAM_TO_UINT16(connect_evt.handle, p);

  connect_evt.group_id = BT_GROUP_AVRCP;
  connect_evt.event_id = BT_AVRCC_EVENT_DISCONNECT;
  bt_avrcp_event_handler((struct bt_event_t *) &connect_evt);
}

static void btRecvA2dpAvrcControllerPlayPosition(uint8_t *p, uint8_t evtCode)
{
  struct bt_avrcp_event_play_position_t evt = {0};

  /* Copy Handle ID */
  STREAM_TO_UINT16(evt.handle, p);

  evt.group_id = BT_GROUP_AVRCP;
  evt.event_id = BT_AVRCP_EVENT_PLAY_POS_CHANGE;
  memcpy(&evt.position, p, sizeof(evt.position));
  bt_avrcp_event_handler((struct bt_event_t *) &evt);
}

static void btRecvA2dpSnkEvtDisconnect(uint8_t *p)
{
  struct bt_a2dp_event_connect_t connect_evt = {0};

  /* Copy Handle ID */
  STREAM_TO_UINT16(connect_evt.handle, p);

  connect_evt.group_id = BT_GROUP_A2DP;
  connect_evt.event_id = BT_A2DP_EVENT_DISCONNECT;

  bt_a2dp_event_handler((struct bt_event_t *) &connect_evt);
}

void bleRecvLeConnected(BLE_Evt *bleEvent, ble_evt_t *pBleBcmEvt)
{
  struct ble_event_conn_stat_t conn_stat_evt;
  uint8_t *rp = NULL;

  rp = pBleBcmEvt->evtData + BT_ADDR_LEN + 1;
  STREAM_TO_UINT16(conn_stat_evt.handle, rp);
  btdbg("connect handle = %d\n", conn_stat_evt.handle);
  rp = pBleBcmEvt->evtData + 1;
  btdbg("connect addr = %x,%x,%x,%x,%x,%x\n",rp[0],rp[1],rp[2],rp[3],rp[4],rp[5]);
  memcpy(&conn_stat_evt.addr, rp, sizeof(BT_ADDR));

  conn_stat_evt.connected = true;

  conn_stat_evt.group_id = BLE_GROUP_COMMON;
  conn_stat_evt.event_id = BLE_COMMON_EVENT_CONN_STAT_CHANGE;

  ble_common_event_handler((struct bt_event_t *) &conn_stat_evt);
}

void bleRecvLeDisconnected(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt)
{
  struct ble_event_conn_stat_t conn_stat_evt;

  conn_stat_evt.connected = false;

  conn_stat_evt.group_id = BLE_GROUP_COMMON;
  conn_stat_evt.event_id = BLE_COMMON_EVENT_CONN_STAT_CHANGE;

  ble_common_event_handler((struct bt_event_t *) &conn_stat_evt);
}

void bleRecvLeAdverReport(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt, uint16_t len)
{
  (void) pBleEvent;

  struct ble_event_adv_rept_t adv_rept_evt = {0};
  uint8_t *rp = pBleBcmEvt->evtData + 1;

  adv_rept_evt.group_id = BLE_GROUP_COMMON;
  adv_rept_evt.event_id = BLE_COMMON_EVENT_SCAN_RESULT;

  /* not used
   * STREAM_TO_UINT8(commMem.advReportData.addr.type, rp);
   */
  ++rp; /* skip addr type */
  memcpy(&adv_rept_evt.addr, rp, BLE_GAP_ADDR_LENGTH);
  rp += BLE_GAP_ADDR_LENGTH;
  STREAM_TO_UINT8(adv_rept_evt.rssi, rp);

  adv_rept_evt.length = len - BLE_GAP_ADDR_LENGTH - BLE_HANDLE_LEN - BLE_ADV_RSSI_LEN;
  memcpy(adv_rept_evt.data, rp, BLE_GAP_ADV_MAX_SIZE);

#ifndef REPORT_ALL_ADV_DATA /* report device name only */
  uint32_t idx = 0;
  uint8_t field_len = 0;
  uint8_t field_type = 0;
  uint8_t adv_data[BLE_GAP_ADV_MAX_SIZE] = {0};

  memcpy(adv_data, adv_rept_evt.data, adv_rept_evt.length);

  while (idx < adv_rept_evt.length)
    {
      field_len = adv_data[idx];
      field_type = adv_data[idx + 1];

      if (0x09 == field_type) { /* 0x09: Complete local name */
        adv_rept_evt.length = field_len;
        memset(adv_rept_evt.data, 0, sizeof(adv_rept_evt.data));
        memcpy(adv_rept_evt.data, &adv_data[idx + 2], field_len - 1);
        ble_common_event_handler((struct bt_event_t *) &adv_rept_evt);
        return;
      }
      idx += field_len + 1;
    }
#else
  ble_common_event_handler((struct bt_event_t *) &adv_rept_evt);
#endif /* REPORT_ALL_ADV_DATA */


}

void bleRecvGattReadRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt)
{
  /* Read request will operate by firmware side */
  /* nop */
}

void bleRecvGattWriteRequest(BLE_Evt *pBleEvent,
    ble_evt_t *pBleBcmEvt, uint16_t len)
{
  struct ble_gatt_event_write_req_t write_req_evt;
  uint8_t *rp = NULL;
  uint16_t conn_handle;

  /* Get characteristic handle ID */
  rp = pBleBcmEvt->evtData;
  STREAM_TO_UINT16(conn_handle, rp);
  STREAM_TO_UINT16(write_req_evt.char_handle, rp);
  (void) conn_handle;

  /* Get length of data */
  write_req_evt.length = len - BLE_HANDLE_LEN - BLE_HANDLE_LEN;

  /* Get data body */
  memcpy(write_req_evt.data, rp, write_req_evt.length);

  /* This HAL doesn't support service handle ID while write request */

  write_req_evt.serv_handle = BLE_GATT_INVALID_SERVICE_HANDLE;

  write_req_evt.group_id = BLE_GROUP_GATT;
  write_req_evt.event_id = BLE_GATT_EVENT_WRITE_REQ;

  ble_gatt_event_handler((struct bt_event_t *) &write_req_evt);
}

void bleRecvGattReadResponse(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt, uint16_t len)
{
  (void) pBleEvent;

  struct ble_gatt_event_read_rsp_t read_rsp_evt = {0};
  uint8_t *rp = pBleBcmEvt->evtData;

  STREAM_TO_UINT16(read_rsp_evt.conn_handle, rp);
  STREAM_TO_UINT16(read_rsp_evt.char_handle, rp);

  read_rsp_evt.length   = len - BLE_HANDLE_LEN - BLE_HANDLE_LEN;
  memcpy(read_rsp_evt.data, rp, read_rsp_evt.length);

  /* This HAL doesn't support service handle ID while write request */
  read_rsp_evt.serv_handle = BLE_GATT_INVALID_SERVICE_HANDLE;

  read_rsp_evt.group_id = BLE_GROUP_GATT;
  read_rsp_evt.event_id = BLE_GATT_EVENT_READ_RESP;

  ble_gatt_event_handler((struct bt_event_t *) &read_rsp_evt);

  btdbg("read reponse\n");
}

void bleRecvGattWriteResponse(BLE_Evt *pBleEvent, ble_evt_t *pBleBcmEvt)
{
  (void) pBleEvent;

  struct ble_gatt_event_write_rsp_t write_rsp_evt = {0};
  uint8_t *rp = pBleBcmEvt->evtData;

  STREAM_TO_UINT16(write_rsp_evt.conn_handle, rp);
  STREAM_TO_UINT16(write_rsp_evt.char_handle, rp);
  STREAM_TO_UINT8(write_rsp_evt.status, rp);

  /* This HAL doesn't support service handle ID while write request */
  write_rsp_evt.serv_handle = BLE_GATT_INVALID_SERVICE_HANDLE;

  write_rsp_evt.group_id = BLE_GROUP_GATT;
  write_rsp_evt.event_id = BLE_GATT_EVENT_WRITE_RESP;

  ble_gatt_event_handler((struct bt_event_t *) &write_rsp_evt);

  btdbg("write response,status.\n");
}

/* Event spliter */

static void btRecvDeviceControlPacket(uint8_t evtCode, uint8_t *p,
    uint16_t len)
{
  uint8_t *wp = NULL;
  BLE_Evt *bleEvent = &(((BLE_EvtCtx*)appEvtBuff)->evt);
  ble_evt_t *pBleBcmEvt = (ble_evt_t*)p;

  wp = appEvtBuff;
  UINT16_TO_STREAM(wp,(BT_CONTROL_GROUP_DEVICE<< 8) | evtCode);

  switch (evtCode)
    {
      case BT_CONTROL_EVENT_COMMAND_STATUS:
        btRecvDeviceStatus(p, len, BT_CONTROL_GROUP_DEVICE);
        break;

      case BT_CONTROL_EVENT_NVRAM_DATA:
        recevNvramData(evtCode, p, len);
        break;

      case BT_CONTROL_EVENT_INQUIRY_RESULT:
        btRecvInquiryResult(p, len);
        break;

      case BT_CONTROL_EVENT_INQUIRY_COMPLETE:
        btRecvInquiryComplete(p,len);
        break;

      case BT_CONTROL_EVENT_PAIRING_COMPLETE:
        pairingComplete(evtCode, p);
        break;

      case BT_EVT_DEV_CONNECTED_DEVICE_NAME:
        btRecvConnDevName(evtCode, p);
        break;
      case BT_CONTROL_EVENT_USER_CONFIRMATION:
        btRecvUserConfirmation(p);
        break;

      case BT_EVT_DEV_ACL_CONNECTION_STATUS:
        btRecvAclConnStatus(p, len);
        break;

      case BT_CONTROL_EVENT_SECURITY_REQUEST:
        btdbg("security request\n");
        bleRecvSecurityRequest(bleEvent, pBleBcmEvt);
        break;

      case BT_CONTROL_EVENT_USER_PASSKEY:
      case BT_CONTROL_EVENT_REPLY_BT_VERSION:
      case BT_CONTROL_EVENT_DEVICE_STARTED:
      case BT_CONTROL_EVENT_WICED_TRACE:
      case BT_CONTROL_EVENT_HCI_TRACE:
      case BT_CONTROL_EVENT_ENCRYPTION_CHANGED:
      case BT_CONTROL_EVENT_PASSKEY_NOTIFICATION:
      case BT_CONTROL_EVENT_POWER_MANAGER_STATUS:
      case BT_CONTROL_EVENT_REQUEST_FEATURE:
      case BT_CONTROL_EVENT_REPLY_BAUD_RATE:
      case BT_CONTROL_EVENT_REPLY_I2S_ROLE:
      case BT_CONTROL_EVENT_REPLY_VENDORID:
        /* Not supported yet */
        break;

      default:
        break;
    }
}

static void btRecvHfpControlPacket(uint8_t evtCode, uint8_t *p, uint16_t len)
{

  uint8_t *wp = NULL;

  wp = appEvtBuff;
  UINT16_TO_STREAM(wp, ((BT_CONTROL_GROUP_HF << 8) | evtCode));

  ASSERT(len <= sizeof(struct bt_hfp_event_t));

  switch (evtCode)
    {
      case BT_CONTROL_HF_EVENT_COMMAND_STATUS:
        btRecvHfpCommandStatus(p);
        break;

      case BT_CONTROL_HF_EVENT_OPEN:
        btRecvHfpEvtOpen(p);
        break;

      case BT_CONTROL_HF_EVENT_CLOSE:
        btRecvHfpEvtClose(p);
        break;

      case BT_CONTROL_HF_AUDIO_EVENT_OPEN:
        btRecvHfpAudioEvtOpen(p);
        break;

      case BT_CONTROL_HF_AUDIO_EVENT_CLOSE:
        btRecvHfpAudioEvtClose(p);
        break;

      case BT_CONTROL_HF_EVENT_CONNECTED:
        btRecvHfpEvtConnected(p);
        break;

      case BT_CONTROL_HF_EVENT_AG_FEATURE:
        btRecvHfpEvtAgFeature(p);
        break;

      case BT_CONTROL_HF_EVENT_AG_INDICATOR:
        btRecvHfpEvtAgIndicator(p);
        break;

      default:
        printf("[EVENT] HFP event code %02x not supported\n", evtCode);
        break;
    }
}

static void btRecvSppControlPacket(uint8_t evtCode, uint8_t *p, uint16_t len)
{
  uint8_t *wp = NULL;

  wp = appEvtBuff;
  UINT16_TO_STREAM(wp, ((BT_CONTROL_GROUP_SPP << 8) | evtCode));
  switch (evtCode)
    {
      case BT_CONTROL_SPP_EVENT_CONNECTED:
        btRecvSppEvtConnected(p);
        break;

      case BT_CONTROL_SPP_EVENT_DISCONNECTED:
        btRecvSppEvtDisconnect(p);
        break;

      case BT_CONTROL_SPP_EVENT_CONNECTION_FAILED:
        /* Not supported yet */
        break;

      case BT_CONTROL_SPP_EVENT_RX_DATA:
        btRecvSppEvtRxData(p, len);
        break;

      case BT_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND:
      case BT_CONTROL_SPP_EVENT_TX_COMPLETE:
      case BT_CONTROL_SPP_EVENT_STATUS:
        /* Not supported yet */
        break;

      default:
        break;
    }
}

static void btRecvA2dpSnkControlPacket(uint8_t evtCode, uint8_t *p, uint16_t len)
{
  uint8_t *wp = NULL;

  wp = appEvtBuff;
  UINT16_TO_STREAM(wp, ((BT_CONTROL_GROUP_A2DP_SINK) | evtCode));

  switch(evtCode)
    {
      case BT_CONTROL_SINK_EVENT_COMMAND_STATUS:
          btRecvDeviceStatus(p, len, BT_CONTROL_GROUP_DEVICE);
          break;
      case BT_CONTROL_SINK_EVENT_CONNECTED:
          btRecvA2dpSnkEvtConnected(p);
          break;
      case BT_CONTROL_SINK_EVENT_CONNECTION_FAILED:
          /* Not supported yet */
          break;
      case BT_CONTROL_SINK_EVENT_DISCONNECTED:
          btRecvA2dpSnkEvtDisconnect(p);
          break;
      case BT_CONTROL_SINK_EVENT_RECEIVE_DATA:
          /* Not supported yet */
          break;
      case BT_CONTROL_SINK_EVENT_STARTED:
          /* Not supported yet */
          break;
      case BT_CONTROL_SINK_EVENT_STOPPED:
          /* Not supported yet */
          break;
      default:
          break;
    }
}

static void btRecvA2dpAvrcControllerControlPacket(uint8_t evtCode, uint8_t *p, uint16_t len)
{
  uint8_t *wp = NULL;
  wp = appEvtBuff;
  UINT16_TO_STREAM(wp, (BT_CONTROL_GROUP_AVRC_CONTROLLER << 8) | evtCode);

  switch (evtCode)
    {
      case BT_CONTROL_AVRC_CONTROLLER_EVENT_CMD_STATUS:
        btRecvDeviceStatus(p, len, BT_CONTROL_GROUP_DEVICE);
        break;

      case BT_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED:
        btRecvA2dpAvrcControllerConnected(p);
        break;

      case BT_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED:
        btRecvA2dpAvrcControllerDisconnected(p);
        break;

      case BT_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_INFO:
        /* Not supported yet */
        break;

      case BT_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS:
        /* Not supported yet */
        break;

      case BT_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_POSITION:
        btRecvA2dpAvrcControllerPlayPosition(p, evtCode);
        break;

      case BT_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_CHANGE:
        /* Not supported yet */
        break;

      case BT_CONTROL_AVRC_CONTROLLER_EVENT_VOLUME_LEVEL:
        /* Not supported yet */
        break;

      case BT_EVT_AVRC_CONTROLLER_VOLUME_UP:
      case BT_EVT_AVRC_CONTROLLER_VOLUME_DOWN:
        /* Not supported yet */
        break;

      default:
        break;
    }
}

void bleRecvLeControlPacket(uint8_t evtCode, uint8_t *p, uint16_t len)
{
  BLE_Evt *bleEvent = &(((BLE_EvtCtx*)appEvtBuff)->evt);
  ble_evt_t *pBleBcmEvt = (ble_evt_t*)p;

  switch (evtCode)
    {
      case BT_CONTROL_LE_EVENT_CONNECTED:
        bleRecvLeConnected(bleEvent, pBleBcmEvt);
        btdbg("ble connect\n");
        break;
      case BT_CONTROL_LE_EVENT_DISCONNECTED:
        bleRecvLeDisconnected(bleEvent, pBleBcmEvt);
        btdbg("ble disconnect\n");
        break;
      case BT_CONTROL_LE_EVENT_ADVERTISEMENT_REPORT:
        bleRecvLeAdverReport(bleEvent, pBleBcmEvt, len);
        break;
      case BT_CONTROL_LE_EVENT_ADVERTISEMENT_STATE:
      case BT_CONTROL_LE_EVENT_SCAN_STATUS:
      case BT_CONTROL_LE_EVENT_CONN_PARAMS:
      case BT_CONTROL_LE_EVENT_CONNECT_TIMEOUT:
      case BT_CONTROL_LE_EVENT_COMMAND_STATUS:
        break;
      default:
        break;
    }
}

void bleRecvGattControlPacket(uint8_t evtCode, uint8_t *p, uint16_t len)
{
  BLE_Evt *bleEvent = &(((BLE_EvtCtx*)appEvtBuff)->evt);
  ble_evt_t *pBleBcmEvt = (ble_evt_t*)p;

  switch (evtCode)
    {
      case BT_CONTROL_GATT_EVENT_READ_REQUEST:
        bleRecvGattReadRequest(bleEvent, pBleBcmEvt);
        break;

      case BT_CONTROL_GATT_EVENT_WRITE_REQUEST:
        bleRecvGattWriteRequest(bleEvent, pBleBcmEvt, len);
        break;

      case BT_CONTROL_GATT_EVENT_READ_RESPONSE:
        bleRecvGattReadResponse(bleEvent, pBleBcmEvt, len);
        break;

      case BT_CONTROL_GATT_EVENT_WRITE_RESPONSE:
        bleRecvGattWriteResponse(bleEvent, pBleBcmEvt);
        break;
      case BT_CONTROL_GATT_EVENT_COMMAND_STATUS:
        break;
      case BT_CONTROL_GATT_EVENT_DISCOVERY_COMPLETE:
        {
          bleRecvGattCompleteDiscovered(bleEvent, pBleBcmEvt);
          uint32_t i = 0, j = 0;
          struct ble_gatt_event_db_discovery_t db_disc_evt = {0};
          BLE_EvtGattcDbDiscovery *db = (BLE_EvtGattcDbDiscovery *)(bleEvent->evtData);

          db_disc_evt.group_id = BLE_GROUP_GATT;
          db_disc_evt.event_id = BLE_GATT_EVENT_DB_DISCOVERY_COMPLETE;

          db_disc_evt.result                          = db->result;
          db_disc_evt.conn_handle                     = db->connHandle;
          db_disc_evt.state.srv_count                 = db->state.srvCount;
          db_disc_evt.state.end_handle                = db->state.endHandle;
          db_disc_evt.params.db_discovery.srv_count   = db->params.dbDiscovery.srvCount;
          db_disc_evt.params.db_discovery.conn_handle = db->params.dbDiscovery.connHandle;

          for (i = 0; i < BLE_DB_DISCOVERY_MAX_SRV; ++i)
            {
              struct ble_gattc_db_disc_srv_s *srvs_dst = db_disc_evt.params.db_discovery.services;
              BLE_GattcDbDiscSrv *srvs_src = db->params.dbDiscovery.services;

              srvs_dst[i].char_count = srvs_src[i].charCount;
              srvs_dst[i].srv_handle_range.start_handle = srvs_src[i].srvHandleRange.startHandle;
              srvs_dst[i].srv_handle_range.end_handle = srvs_src[i].srvHandleRange.endHandle;

              for (j = 0; j < BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV; ++j)
                {
                  struct ble_gattc_db_disc_char_s *chars_dst = srvs_dst[i].characteristics;
                  BLE_GattcDbDiscChar *chars_src = srvs_src[i].characteristics;

                  chars_dst[j].cccd_handle = chars_src[j].cccdHandle;
                  chars_dst[j].characteristic.char_prope.broadcast = chars_src[j].characteristic.charPrope.broadcast;
                  chars_dst[j].characteristic.char_prope.read = chars_src[j].characteristic.charPrope.read;
                  chars_dst[j].characteristic.char_prope.writeWoResp = chars_src[j].characteristic.charPrope.writeWoResp;
                  chars_dst[j].characteristic.char_prope.write = chars_src[j].characteristic.charPrope.write;
                  chars_dst[j].characteristic.char_prope.notify = chars_src[j].characteristic.charPrope.notify;
                  chars_dst[j].characteristic.char_prope.indicate = chars_src[j].characteristic.charPrope.indicate;
                  chars_dst[j].characteristic.char_prope.authSignedWr = chars_src[j].characteristic.charPrope.authSignedWr;
                  chars_dst[j].characteristic.char_valhandle = chars_src[j].characteristic.charValhandle;
                  chars_dst[j].characteristic.char_declhandle = chars_src[j].characteristic.charDeclhandle;
                  memcpy(&chars_dst[j].characteristic.char_valuuid,
                         &chars_src[j].characteristic.charValUuid,
                         sizeof(BLE_UUID));
                }
              /* BLE_Uuid and BLE_UUID must keeps the same */
              memcpy(&srvs_dst[i].srv_uuid, &srvs_src[i].srvUuid, sizeof(BLE_UUID));
            }
          ble_gatt_event_handler((struct bt_event_t *) &db_disc_evt);
        }
        break;
      case BT_CONTROL_GATT_EVENT_SERVICE_DISCOVERED:
        bleRecvGattServiceDiscovered(bleEvent, pBleBcmEvt, len);
        break;
      case BT_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED:
        bleRecvGattCharDiscovered(bleEvent, pBleBcmEvt, len);
        break;
      case BT_CONTROL_GATT_EVENT_DESCRIPTOR_DISCOVERED:
        bleRecvGattDescriptorDiscovered(bleEvent, pBleBcmEvt, len);
        break;
      case BT_CONTROL_GATT_EVENT_WRITE_COMPLETE:
      case BT_CONTROL_GATT_EVENT_INDICATION:
      case BT_CONTROL_GATT_EVENT_NOTIFICATION:
        /* Not supported yet */
        break;

      default:
        break;
    }
}

static void btRecvControlPacket(uint16_t opcode, uint8_t *p, uint16_t len)
{
  uint8_t evtCode = (opcode)&0xff;

  switch (opcode >> 8)
    {
      case BT_CONTROL_GROUP_DEVICE:
        btRecvDeviceControlPacket(evtCode, p, len);
        break;

      case BT_CONTROL_GROUP_HF:
        btRecvHfpControlPacket(evtCode, p, len);
        break;

      case BT_CONTROL_GROUP_LE:
        bleRecvLeControlPacket(evtCode, p, len);
        break;

      case BT_CONTROL_GROUP_GATT:
        bleRecvGattControlPacket(evtCode, p, len);
        break;

      case BT_CONTROL_GROUP_SPP:
        btRecvSppControlPacket(evtCode, p, len);
        break;

      case BT_CONTROL_GROUP_AVRC_TARGET:
        /* Not supported yet */
        break;

      case BT_CONTROL_GROUP_AVRC_CONTROLLER:
        btRecvA2dpAvrcControllerControlPacket(evtCode, p, len);
        break;

      case BT_CONTROL_GROUP_A2DP_SINK:
        btRecvA2dpSnkControlPacket(evtCode, p, len);
        break;

      case BT_CONTROL_GROUP_AUDIO_SINK:
        /* Not supported yet */
        break;

      case BT_CONTROL_GROUP_A2DP_SRC:
      case BT_CONTROL_GROUP_AG:
        /* nop */
        break;

      default:
        break;

    }
}

static void* btRecvTaskSingleStep(void *param)
{
  uint8_t *p = NULL;
  uint16_t opcode = 0;
  uint16_t packetLen = 0;

  while (true)
    {
      p = btUartGetCompleteBuff(&packetLen);
      if (p == NULL)
        {
          break;
        }
      switch (*p++)
        {
          case PACKET_CONTROL:
            STREAM_TO_UINT16(opcode, p);
            STREAM_TO_UINT16(packetLen, p);
            btRecvControlPacket(opcode, p, packetLen);
            break;

          case PACKET_MEDIA:
            /* nop */
            break;

          case PACKET_HCI:
            /* nop */
            break;

          default:
            break;
        }
      btUartReleaseCompleteBuff();
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btRecvTaskEntry
 *
 * Description:
 *   Start receive thread to get data from target.
 *
 ****************************************************************************/

int btRecvTaskEntry(void)
{
  int errCode =  BT_SUCCESS;
  errCode = pthread_create(&gRecvTask, NULL, btRecvTaskSingleStep, NULL);

  if (errCode)
    {
      errCode = -ENXIO;
    }

  return errCode;
}

/****************************************************************************
 * Name: btRecvTaskEnd
 *
 * Description:
 *   Stop receive thread to get data from target.
 *
 ****************************************************************************/

int btRecvTaskEnd(void)
{
  int ret = 0;

  ret = pthread_join(gRecvTask, NULL);

  if (ret != 0)
    {
      btdbg("Receive task finalize error.\n");
    }

  return ret;
}
