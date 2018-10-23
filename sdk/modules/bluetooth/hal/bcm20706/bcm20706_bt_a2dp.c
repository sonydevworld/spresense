/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bcm20706_bt_a2dp.c
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

#include <string.h>
#include <stdlib.h>
#include <bt/bt_a2dp_sink.h>

#include "bt_util.h"
#include "manager/bt_uart_manager.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int btA2dpSinkConnect(BT_ADDR *addr)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_AUDIO_SINK_COMMAND_CONNECT);
  UINT16_TO_STREAM(p, BT_ADDR_LEN);
  memcpy(p, addr, BT_ADDR_LEN);
  p += BT_ADDR_LEN;
  return btUartSendData(buff, p - buff);
}

static int btA2dpSinkDisconnect(BT_ADDR *addr)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_AUDIO_SINK_COMMAND_DISCONNECT);
  UINT16_TO_STREAM(p, BT_ADDR_LEN);
  memcpy(p, addr, BT_ADDR_LEN);
  p += BT_ADDR_LEN;
  return btUartSendData(buff, p - buff);
}

static int btA2dpSinkEnableAac(BT_BOOL btAacIsEnable)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_AUDIO_SINK_COMMAND_AAC_ENABLE);
  UINT16_TO_STREAM(p, sizeof(BT_BOOL));
  UINT8_TO_STREAM(p, btAacIsEnable);
  return btUartSendData(buff, p - buff);
}

static int btA2dpSinkSetCodec(BT_BCM_AUDIO_CODEC_INFO *btCodecInfo)
{
  uint8_t buff[BT_MID_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p, PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_AUDIO_SINK_COMMAND_SET_CODEC);
  UINT16_TO_STREAM(p, sizeof(BT_BCM_AUDIO_CODEC_INFO));
  memcpy(p, btCodecInfo, sizeof(BT_BCM_AUDIO_CODEC_INFO));
  p += sizeof(BT_BCM_AUDIO_CODEC_INFO);
  return btUartSendData(buff, p - buff);
}

static uint8_t bt_a2dp_sbc_sample_frequency(BT_A2DP_SAMPLE_FREQ sampFreq)
{
  switch(sampFreq)
    {
      case BT_A2DP_SAMPLE_FREQ_16000HZ:
        return 0x80;

      case BT_A2DP_SAMPLE_FREQ_32000HZ:
        return 0x40;

      case BT_A2DP_SAMPLE_FREQ_44100HZ:
        return 0x20;

      case BT_A2DP_SAMPLE_FREQ_48000HZ:
        return 0x10;

      default:
        return 0x00;
    }
}

static uint8_t bt_a2dp_sbc_channel_mode(BT_A2DP_CHANNEL_MODE chMode)
{
  switch(chMode)
    {
      case BT_A2DP_CHANNEL_MONO:
        return 0x08;

      case BT_A2DP_CHANNEL_DUAL:
        return 0x04;

      case BT_A2DP_CHANNEL_STEREO:
        return 0x02;

      case BT_A2DP_CHANNEL_JOINTSTEREO:
        return 0x01;

      default:
        return 0x00;
    }
}

static uint8_t bt_a2dp_sbc_alloc_method(BT_A2DP_SBC_ALLOC_MODE allocMthd)
{
  switch(allocMthd)
    {
      case BT_A2DP_CHANNEL_SNR:
        return 0x02;

      case BT_A2DP_CHANNEL_LOUDNESS:
        return 0x01;

      default:
        return 0x00;
    }
}

static uint8_t bt_a2dp_aac_profile(BT_A2DP_AAC_PROFILE profile)
{
  switch(profile)
    {
      case BT_A2DP_MPEG2_AAC_LC:
        return 0x80;

      case BT_A2DP_MPEG4_AAC_LC:
        return 0x40;

      case BT_A2DP_MPEG4_AAC_LTP:
        return 0x20;

      case BT_A2DP_MPEG4_AAC_SSR:
        return 0x10;

      default:
        return 0x00;
    }
}

static uint16_t bt_a2dp_aac_sample_frequency(BT_A2DP_SAMPLE_FREQ sampFreq)
{
  switch(sampFreq)
    {
      case BT_A2DP_SAMPLE_FREQ_8000HZ:
        return 0x8000;

      case BT_A2DP_SAMPLE_FREQ_11000HZ:
        return 0x4000;

      case BT_A2DP_SAMPLE_FREQ_12000HZ:
        return 0x2000;

      case BT_A2DP_SAMPLE_FREQ_16000HZ:
        return 0x1000;

      case BT_A2DP_SAMPLE_FREQ_22050HZ:
        return 0x0800;

      case BT_A2DP_SAMPLE_FREQ_24000HZ:
        return 0x0400;

      case BT_A2DP_SAMPLE_FREQ_32000HZ:
        return 0x0200;

      case BT_A2DP_SAMPLE_FREQ_44100HZ:
        return 0x0100;

      case BT_A2DP_SAMPLE_FREQ_48000HZ:
        return 0x0080;

      case BT_A2DP_SAMPLE_FREQ_64000HZ:
        return 0x0040;

      case BT_A2DP_SAMPLE_FREQ_88000HZ:
        return 0x0020;

      case BT_A2DP_SAMPLE_FREQ_96000HZ:
        return 0x0010;

      default:
        return 0x00;
    }
}

static uint8_t bt_a2dp_aac_channel(BT_A2DP_CHANNEL_MODE chnl)
{
  switch(chnl)
    {
      case BT_A2DP_CHANNEL_MONO:
        return 0x08;

      case BT_A2DP_CHANNEL_DUAL:
      case BT_A2DP_CHANNEL_STEREO:
      case BT_A2DP_CHANNEL_JOINTSTEREO:
        return 0x04;

      default:
        return 0x00;
    }
}

/****************************************************************************
 * Name: bcm20706_bt_a2dp_connect
 *
 * Description:
 *   Bluetooth A2DP connect/disconnect.
 *   Connect/Disconnect A2DP by device address.
 *
 ****************************************************************************/

static int bcm20706_bt_a2dp_connect(BT_ADDR *addr, uint16_t handle, bool connect)
{
  int ret = BT_SUCCESS;

  if (connect)
    {
      /* Connect */

      ret = btA2dpSinkConnect(addr);
    }
  else
    {
      /* Disconnect */

      ret = btA2dpSinkDisconnect(addr);
    }

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_a2dp_aac_enable
 *
 * Description:
 *   Bluetooth AAC codec enable via A2DP.
 *   Enabling/Disabling AAC codec for A2DP.
 *
 ****************************************************************************/

static int bcm20706_bt_a2dp_aac_enable(bool enable)
{
  int ret = BT_SUCCESS;

  ret = btA2dpSinkEnableAac(enable ? BT_TRUE :   BT_FALSE);

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_a2dp_codec_enable
 *
 * Description:
 *   Bluetooth Vendor codec enable via A2DP.
 *   Enabling/Disabling Vendor specific codec for A2DP.
 *
 ****************************************************************************/

static int bcm20706_bt_a2dp_codec_enable(bool enable)
{
  int ret = BT_SUCCESS;

  /* Not supported yet */

  return ret;
}

/****************************************************************************
 * Name: bcm20706_bt_a2dp_set_codec
 *
 * Description:
 *   Bluetooth set A2DP codecs.
 *   Setup codec parameters from BT_AUDIO_CODEC_INFO.
 *
 ****************************************************************************/

static int bcm20706_bt_a2dp_set_codec(BT_AUDIO_CODEC_INFO *codec_info)
{
  int ret = BT_SUCCESS;
  BT_BCM_AUDIO_CODEC_INFO codecInfo;
  BT_A2D_SBC_CIE *sbc_cie;
  BT_A2D_M24_CIE *aac_cie;

  if (codec_info->codecId == BT_A2DP_SINK_CODEC_SBC)
    {
      codecInfo.codecId = BT_BCM_A2DP_SINK_CODEC_SBC;

      sbc_cie = &codecInfo.cie.sbc;

      sbc_cie->sampFreq    = bt_a2dp_sbc_sample_frequency(codec_info->codec_info.sbc.sampFreq);
      sbc_cie->chMode      = bt_a2dp_sbc_channel_mode(codec_info->codec_info.sbc.chMode);
      sbc_cie->blockLen    = codec_info->codec_info.sbc.blockLen;
      sbc_cie->numSubbands = codec_info->codec_info.sbc.numSubbands;
      sbc_cie->allocMthd   = bt_a2dp_sbc_alloc_method(codec_info->codec_info.sbc.allocMthd);
      sbc_cie->maxBitpool  = codec_info->codec_info.sbc.maxBitpool;
      sbc_cie->minBitpool  = codec_info->codec_info.sbc.minBitpool;
    }
  else if (codec_info->codecId == BT_A2DP_SINK_CODEC_AAC)
    {
      codecInfo.codecId = BT_BCM_A2DP_SINK_CODEC_M24;

      aac_cie = &codecInfo.cie.m24;

      aac_cie->objType  = bt_a2dp_aac_profile(codec_info->codec_info.aac.profile);
      aac_cie->sampFreq = bt_a2dp_aac_sample_frequency(codec_info->codec_info.aac.sampFreq);
      aac_cie->chnl     = bt_a2dp_aac_channel(codec_info->codec_info.aac.chnl);
      aac_cie->vbr      = codec_info->codec_info.aac.isVbrSupported;
      aac_cie->bitrate  = codec_info->codec_info.aac.bitrate;
    }
  else
    {
        /* Not supoprt codec */

        return BT_FAIL;
    }

  ret = btA2dpSinkSetCodec(&codecInfo);

  return ret;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct bt_hal_a2dp_ops_s bt_hal_a2dp_ops =
{
  .connect           = bcm20706_bt_a2dp_connect,
  .aacEnable         = bcm20706_bt_a2dp_aac_enable,
  .vendorCodecEnable = bcm20706_bt_a2dp_codec_enable,
  .set_codec         = bcm20706_bt_a2dp_set_codec
};


