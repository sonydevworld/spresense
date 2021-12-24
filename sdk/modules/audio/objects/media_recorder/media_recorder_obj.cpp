/****************************************************************************
 * modules/audio/objects/media_recorder/media_recorder_obj.cpp
 *
 *   Copyright 2018-2021 Sony Semiconductor Solutions Corporation
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
#include <nuttx/arch.h>
#include <stdlib.h>
#include "memutils/common_utils/common_assert.h"
#include "media_recorder_obj.h"
#include "dsp_driver/include/dsp_drv.h"
#include "debug/dbg_log.h"

__USING_WIEN2
using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*--------------------------------------------------------------------------*/
static bool filter_done_callback(ComponentCbParam *cmplt, void* p_requester)
{
  err_t er;

  switch (cmplt->event_type)
    {
      case ComponentExec:
        {
          MediaRecorderObject::FilterDoneCmd result_param;

          result_param.event_type = cmplt->event_type;
          result_param.result     = cmplt->result;

          er = MsgLib::send<MediaRecorderObject::FilterDoneCmd>
                                                 (MediaRecorderObject::get_msgq_id(),
                                                  MsgPriNormal,
                                                  MSG_AUD_MRC_RST_FILTER,
                                                  NULL,
                                                  result_param);

          F_ASSERT(er == ERR_OK);
        }
        break;

      case ComponentFlush:
        {
          MediaRecorderObject::FilterDoneCmd result_param;

          result_param.event_type = cmplt->event_type;
          result_param.result     = cmplt->result;

          er = MsgLib::send<MediaRecorderObject::FilterDoneCmd>
                                                 (MediaRecorderObject::get_msgq_id(),
                                                  MsgPriNormal,
                                                  MSG_AUD_MRC_RST_FILTER,
                                                  NULL,
                                                  result_param);
          F_ASSERT(er == ERR_OK);
        }
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
        return false;
    }
  return true;
}

/*--------------------------------------------------------------------------*/
static bool encoder_done_callback(void* p_response)
{
  err_t er;
  EncCmpltParam cmplt;
  DspDrvComPrm_t* p_param = (DspDrvComPrm_t *)p_response;
  D_ASSERT2(DSP_COM_DATA_TYPE_STRUCT_ADDRESS == p_param->type,
            AssertParamLog(AssertIdTypeUnmatch, p_param->type));

  Apu::Wien2ApuCmd* packet = reinterpret_cast<Apu::Wien2ApuCmd*>
    (p_param->data.pParam);

  cmplt.event_type = static_cast<Wien2::Apu::ApuEventType>
    (packet->header.event_type);
  cmplt.result = (Apu::ApuExecOK == packet->result.exec_result)
    ? true : false;

  switch (packet->header.event_type)
    {
      case Apu::ExecEvent:
        {
          cmplt.exec_enc_cmplt.input_buffer  =
            packet->exec_enc_cmd.input_buffer;
          cmplt.exec_enc_cmplt.output_buffer =
            packet->exec_enc_cmd.output_buffer;

          MEDIA_RECORDER_VDBG("Enc s %ld\n",
                              cmplt.exec_enc_cmplt.output_buffer.size);

          er = MsgLib::send<EncCmpltParam>(MediaRecorderObject::get_msgq_id(),
                                           MsgPriNormal,
                                           MSG_AUD_MRC_RST_ENC,
                                           NULL,
                                           cmplt);
          F_ASSERT(er == ERR_OK);
        }
        break;

      case Apu::FlushEvent:
        {
          cmplt.stop_enc_cmplt.output_buffer =
            packet->flush_enc_cmd.output_buffer;

          MEDIA_RECORDER_VDBG("FlsEnc s %ld\n",
                              cmplt.stop_enc_cmplt.output_buffer.size);

          er = MsgLib::send<EncCmpltParam>(MediaRecorderObject::get_msgq_id(),
                                           MsgPriNormal,
                                           MSG_AUD_MRC_RST_ENC,
                                           NULL,
                                           cmplt);
          F_ASSERT(er == ERR_OK);
        }
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
        return false;
    }
  return true;
}

/*--------------------------------------------------------------------------*/
uint32_t MediaRecorderObject::loadCodec(AudioCodec codec,
                                            char *path,
                                            int32_t sampling_rate,
                                            int32_t bit_length,
                                            uint32_t* dsp_inf)
{
  uint32_t rst = AS_ECODE_OK;
  if ((codec == AudCodecMP3) || (codec == AudCodecOPUS))
    {
      rst = AS_encode_activate(codec,
                               (path) ? path : CONFIG_AUDIOUTILS_DSP_MOUNTPT,
                               m_msgq_id.cmp,
                               m_pool_id.cmp,
                               dsp_inf);
      if(rst != AS_ECODE_OK)
        {
          return rst;
        }
    }
  else if (codec == AudCodecLPCM)
    {
      char fullpath[64] = { 0 };

      if (isNeedUpsampling(sampling_rate))
        {
          m_filter_instance = m_src_instance;
          snprintf(fullpath,
                   sizeof(fullpath),
                   "%s/SRC",
                   ((path) ? path : CONFIG_AUDIOUTILS_DSP_MOUNTPT));
        }
      else
        {
          if (bit_length == AS_BITLENGTH_24)
            {
              m_filter_instance = m_packing_instance;
            }
          else
            {
              m_filter_instance = m_thruproc_instance;
            }
        }

      rst =  m_filter_instance->activate(filter_done_callback,
                                         fullpath,
                                         static_cast<void *>(this),
                                         dsp_inf);
      if (rst != AS_ECODE_OK)
        {
          return rst;
        }
    }
  else
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }

  m_codec_type = codec;

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
bool MediaRecorderObject::unloadCodec(void)
{
  if ((m_codec_type == AudCodecMP3) || (m_codec_type == AudCodecOPUS))
    {
      if (!AS_encode_deactivate())
        {
          return false;
        }
    }
  else if (m_codec_type == AudCodecLPCM)
    {
      bool ret = true;

      if (m_filter_instance)
        {
          ret = m_filter_instance->deactivate();

          m_filter_instance = NULL;
        }

      if (!ret)
        {
          return false;
        }
    }
  else
    {
      /* No need to unload DSP because it is not loaded */
    }

  m_codec_type = InvalidCodecType;

  return true;
}

/*--------------------------------------------------------------------------*/
FAR void AS_MediaRecorderObjEntry(FAR void *arg)
{
  MediaRecorderObject::create((AsObjectParams_t *)arg);
}

/*--------------------------------------------------------------------------*/
MediaRecorderObject::MsgProc
  MediaRecorderObject::RsltProcTbl[AUD_MRC_RST_MSG_NUM][StateNum] =
{
  /* Message Type: MSG_AUD_MRC_RST_FILTER. */

  {                                              /* Recorder status: */
    &MediaRecorderObject::illegalFilterDone,     /*   Booted.        */
    &MediaRecorderObject::illegalFilterDone,     /*   Ready.         */
    &MediaRecorderObject::illegalFilterDone,     /*   PreActive.     */
    &MediaRecorderObject::filterDoneOnActive,    /*   Active.        */
    &MediaRecorderObject::filterDoneOnStop,      /*   Stopping.      */
    &MediaRecorderObject::filterDoneOnErrorStop, /*   ErrorStopping. */
    &MediaRecorderObject::illegalFilterDone      /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_MRC_RST_ENC. */

  {                                              /* Recorder status: */
    &MediaRecorderObject::illegalEncDone,        /*   Booted.        */
    &MediaRecorderObject::illegalEncDone,        /*   Ready.         */
    &MediaRecorderObject::illegalEncDone,        /*   PreActive.     */
    &MediaRecorderObject::encDoneOnActive,       /*   Active.        */
    &MediaRecorderObject::encDoneOnStop,         /*   Stopping.      */
    &MediaRecorderObject::encDoneOnErrorStop,    /*   ErrorStopping. */
    &MediaRecorderObject::illegalEncDone         /*   WaitStop.      */
  }
};


/*--------------------------------------------------------------------------*/
void MediaRecorderObject::parseResult(MsgPacket *msg)
{
  uint32_t event = MSG_GET_SUBTYPE(msg->getType());
  F_ASSERT((event < AUD_MRC_RST_MSG_NUM));

  (this->*RsltProcTbl[event][m_state.get()])(msg);
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::reply(AsRecorderEvent evtype,
                                    MsgType msg_type,
                                    uint32_t result)
{
  if (m_callback != NULL)
    {
      m_callback(evtype, result, 0);
    }
  else if (m_msgq_id.from != MSG_QUE_NULL)
    {
      AudioObjReply cmplt((uint32_t)msg_type,
                           AS_OBJ_REPLY_TYPE_REQ,
                           AS_MODULE_ID_MEDIA_RECORDER_OBJ,
                           result);
      err_t er = MsgLib::send<AudioObjReply>(m_msgq_id.from,
                                             MsgPriNormal,
                                             MSG_TYPE_AUD_RES,
                                             m_msgq_id.self,
                                             cmplt);
      if (ERR_OK != er)
        {
          F_ASSERT(0);
        }
    }
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::illegal(MsgPacket *msg)
{
  uint msgtype = msg->getType();
  msg->moveParam<RecorderCommand>();

  uint32_t idx = msgtype - MSG_AUD_MRC_CMD_ACTIVATE;

  AsRecorderEvent table[] =
  {
    AsRecorderEventAct,
    AsRecorderEventDeact,
    AsRecorderEventInit,
    AsRecorderEventStart,
    AsRecorderEventStop,
    AsRecorderEventReqEncode
  };

  reply(table[idx], msg->getType(), AS_ECODE_STATE_VIOLATION);
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::activateOnBooted(MsgPacket *msg)
{
  uint32_t rst;
  AsActivateRecorder act = msg->moveParam<RecorderCommand>().act_param;

  MEDIA_RECORDER_DBG("ACT: outdev %d\n", act.param.output_device);

  /* Set event callback */

  m_callback = act.cb;

  if (!checkAndSetMemPool())
    {
      reply(AsRecorderEventAct,
            msg->getType(),
            AS_ECODE_CHECK_MEMORY_POOL_ERROR);
      return;
    }

  rst = isValidActivateParam(act.param);
  if (rst != AS_ECODE_OK)
    {
      reply(AsRecorderEventAct, msg->getType(), rst);
      return;
    }

  switch (m_output_device)
    {
      case AS_SETRECDR_STS_OUTPUTDEVICE_RAM:
        m_p_output_device_handler =
          act.param.output_device_handler;
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        reply(AsRecorderEventAct, msg->getType(), AS_ECODE_COMMAND_PARAM_OUTPUT_DEVICE);
        return;
    }

  /* Init Sink */

  InitAudioRecSinkParam_s init_sink;
  if (m_output_device == AS_SETRECDR_STS_OUTPUTDEVICE_RAM)
    {
      init_sink.init_audio_ram_sink.output_device_hdlr =
        *m_p_output_device_handler;
    }

  m_rec_sink.init(init_sink);

  /* Transit to Ready */

  m_state = Ready;

  /* Reply */

  reply(AsRecorderEventAct, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::deactivateOnReady(MsgPacket *msg)
{
  msg->moveParam<RecorderCommand>();

  MEDIA_RECORDER_DBG("DEACT:\n");

  if (!unloadCodec())
    {
      reply(AsRecorderEventDeact, msg->getType(), AS_ECODE_DSP_UNLOAD_ERROR);
      return;
    }

  m_state = Booted;

  reply(AsRecorderEventDeact, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::initOnReady(MsgPacket *msg)
{
  RecorderCommand cmd = msg->moveParam<RecorderCommand>();
  uint32_t rst = AS_ECODE_OK;

  MEDIA_RECORDER_DBG("INIT: fs %ld, ch num %d, bit len %d, codec %d(%s),"
                     "complexity %d, bitrate %ld\n",
                     cmd.init_param.sampling_rate,
                     cmd.init_param.channel_number,
                     cmd.init_param.bit_length,
                     cmd.init_param.codec_type,
                     cmd.init_param.dsp_path,
                     cmd.init_param.computational_complexity,
                     cmd.init_param.bitrate);

  rst = isValidInitParam(cmd);
  if (rst != AS_ECODE_OK)
    {
      reply(AsRecorderEventInit, msg->getType(), rst);
      return;
    }

  m_channel_num   = cmd.init_param.channel_number;
  m_pcm_bit_width =
    ((cmd.init_param.bit_length == AS_BITLENGTH_16)
      ? AudPcmFormatInt16 : (cmd.init_param.bit_length == AS_BITLENGTH_24)
                        ? AudPcmFormatInt24 : AudPcmFormatInt32);
  m_bit_rate      = cmd.init_param.bitrate;
  m_complexity    = cmd.init_param.computational_complexity;
  AudioCodec cmd_codec_type = InvalidCodecType;
  switch (cmd.init_param.codec_type)
    {
      case AS_CODECTYPE_MP3:
        cmd_codec_type = AudCodecMP3;
        break;
      case AS_CODECTYPE_LPCM:
        cmd_codec_type = AudCodecLPCM;
        break;
      case AudCodecOPUS:
        cmd_codec_type = AudCodecOPUS;
        break;
      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        reply(AsRecorderEventInit, msg->getType(), AS_ECODE_COMMAND_PARAM_CODEC_TYPE);
        return;
    }
  if (m_codec_type != cmd_codec_type)
    {
      if (!unloadCodec())
        {
          reply(AsRecorderEventInit, msg->getType(), AS_ECODE_DSP_UNLOAD_ERROR);
          return;
        }

      uint32_t dsp_inf = 0;
      rst = loadCodec(cmd_codec_type,
                      cmd.init_param.dsp_path,
                      cmd.init_param.sampling_rate,
                      cmd.init_param.bit_length,
                      &dsp_inf);
      if (rst != AS_ECODE_OK)
        {
          reply(AsRecorderEventInit, msg->getType(), rst);
          return;
        }
    }
  else
    {
      if (m_codec_type == AudCodecLPCM &&
          (isNeedUpsampling(m_sampling_rate) &&
           !isNeedUpsampling(cmd.init_param.sampling_rate)))
        {
          if (!unloadCodec())
            {
              reply(AsRecorderEventInit, msg->getType(), AS_ECODE_DSP_UNLOAD_ERROR);
              return;
            }
          m_codec_type = cmd_codec_type;
        }

      if (m_codec_type == AudCodecLPCM &&
          !isNeedUpsampling(m_sampling_rate) &&
          isNeedUpsampling(cmd.init_param.sampling_rate))
        {
          uint32_t dsp_inf = 0;
          rst = loadCodec(cmd_codec_type,
                          cmd.init_param.dsp_path,
                          cmd.init_param.sampling_rate,
                          cmd.init_param.bit_length,
                          &dsp_inf);
          if (rst != AS_ECODE_OK)
            {
              reply(AsRecorderEventInit, msg->getType(), rst);
              return;
            }
        }
    }
  m_sampling_rate = cmd.init_param.sampling_rate;

  /* Init encoder */

  rst = initEnc(&cmd.init_param);
  m_output_buf_mh_que.clear();

  /* Reply */

  reply(AsRecorderEventInit, msg->getType(), rst);
}

/*--------------------------------------------------------------------------*/
uint32_t MediaRecorderObject::initEnc(AsInitRecorderParam *param)
{
  bool result = true;
  uint32_t apu_result = AS_ECODE_OK;
  uint32_t dsp_inf = 0;

  if (m_codec_type == AudCodecLPCM)
    {
      InitComponentParam init_param;

      if (isNeedUpsampling(param->sampling_rate))
        {
          init_param.common.samples       =
            getCapSampleNumPerFrame(param->codec_type, param->sampling_rate);
          init_param.common.in_fs         =
            (cxd56_audio_get_clkmode() == CXD56_AUDIO_CLKMODE_HIRES)
              ? AS_SAMPLINGRATE_192000 : AS_SAMPLINGRATE_48000;
          init_param.common.out_fs        = param->sampling_rate;
          init_param.common.in_bitlength  = param->bit_length;
          init_param.common.out_bitlength = param->bit_length;
          init_param.common.ch_num        = param->channel_number;
        }
      else
        {
          if (m_pcm_bit_width == AudPcmFormatInt24)
            {
              init_param.common.in_bitlength  = AS_BITLENGTH_32;
              init_param.common.out_bitlength = AS_BITLENGTH_24;
            }
        }

      if (m_filter_instance)
        {
          apu_result = m_filter_instance->init(init_param);
          result = m_filter_instance->recv_done();
          if (!result)
            {
              return AS_ECODE_QUEUE_OPERATION_ERROR;
            }

          if (apu_result != AS_ECODE_OK)
            {
              return apu_result;
            }
        }
    }
  else if ((m_codec_type == AudCodecMP3) || (m_codec_type == AudCodecOPUS))
    {
      InitEncParam enc_param;
      enc_param.codec_type           = m_codec_type;
      enc_param.input_sampling_rate  = AS_SAMPLINGRATE_48000;
      enc_param.output_sampling_rate = param->sampling_rate;
      enc_param.bit_width            = m_pcm_bit_width;
      enc_param.channel_num          = param->channel_number;
      enc_param.callback             = encoder_done_callback;
      enc_param.bit_rate             = m_bit_rate;
      enc_param.complexity           =
        (m_codec_type == AudCodecOPUS) ? m_complexity : 0;

      apu_result = AS_encode_init(&enc_param, &dsp_inf);
      result = AS_encode_recv_done();
      if (!result)
        {
          return AS_ECODE_QUEUE_OPERATION_ERROR;
        }

      if (apu_result != AS_ECODE_OK)
        {
          return apu_result;
        }
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::startOnReady(MsgPacket *msg)
{
  bool result = true;
  uint32_t apu_result = AS_ECODE_OK;
  uint32_t dsp_inf = 0;

  msg->moveParam<RecorderCommand>();

  /* Transit to Active */

  m_state = Active;

  /* Codec DSP processing resumed */

  if ((m_codec_type == AudCodecMP3) || (m_codec_type == AudCodecOPUS))
    {
      InitEncParam enc_param;
      enc_param.codec_type           = m_codec_type;
      enc_param.input_sampling_rate  = AS_SAMPLINGRATE_48000;
      enc_param.output_sampling_rate = m_sampling_rate;
      enc_param.bit_width            = m_pcm_bit_width;
      enc_param.channel_num          = m_channel_num;
      enc_param.callback             = encoder_done_callback;
      enc_param.bit_rate             = m_bit_rate;
      enc_param.complexity           =
        (m_codec_type == AudCodecOPUS) ? m_complexity : 0;

      apu_result = AS_encode_init(&enc_param, &dsp_inf);
      result = AS_encode_recv_done();
      if (!result)
        {
          apu_result = AS_ECODE_QUEUE_OPERATION_ERROR;
        }
    }

  /* Reply */

  reply(AsRecorderEventStart, msg->getType(), apu_result);
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::illegalReqEnc(MsgPacket *msg)
{
  AsPcmDataParam pcmparam = msg->moveParam<AsPcmDataParam>();

  pcmparam.callback(0, pcmparam.is_end);
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::execOnActive(MsgPacket *msg)
{
  AsPcmDataParam pcmparam = msg->moveParam<AsPcmDataParam>();

  /* Encode Mic-in pcm data.
   * But size 0 PCM cannot encode. (It will cause encode error.)
   */

  bool exec_result = true;

  if (pcmparam.size)
    {
      exec_result = execEnc(&pcmparam);
    }
  else
    {
      exec_result = false;

      pcmparam.callback(0, false);
    }

  /* If end frame or exec failed, flush encode */

  if (pcmparam.is_end || (!exec_result))
    {
      bool stop_result = stopEnc();

      if (!stop_result)
        {
          /* If stop failed, transition to WaitStop.
           * Because reply of stop never returns.
           */

          m_state = WaitStop;
        }
      else
        {
          if (pcmparam.is_end)
            {
              /* End frame and stop success, wait reply from encode. */

              m_state = Stopping;
            }
          else
            {
              /* If not end frame and exec error, wait under Error state. */

              m_state = (exec_result)
                ? Stopping : ErrorStopping;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::stopOnReady(MsgPacket *msg)
{
  msg->moveParam<RecorderCommand>();

  reply(AsRecorderEventStop, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::stopOnActive(MsgPacket *msg)
{
  MEDIA_RECORDER_DBG("FLUSH:\n");

  msg->moveParam<RecorderCommand>();

  /* Hold external command */

  if (!setExternalCmd(AsRecorderEventStop))
    {
      reply(AsRecorderEventStop, msg->getType(), AS_ECODE_QUEUE_OPERATION_ERROR);
      return;
    }
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::stopOnStopping(MsgPacket *msg)
{
  stopOnActive(msg);
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::stopOnErrorStopping(MsgPacket *msg)
{
  stopOnActive(msg);
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::stopOnWaitStop(MsgPacket *msg)
{
  MEDIA_RECORDER_DBG("FLUSH:\n");

  msg->moveParam<RecorderCommand>();

  if (m_output_buf_mh_que.empty())
    {
      /* If all of encode request was returned, transit to Ready. */

      m_state = Ready;

      reply(AsRecorderEventStop, msg->getType(), AS_ECODE_OK);
    }
  else
    {
      /* If encode request which was issued is remaining, wait them. */

      if (!setExternalCmd(AsRecorderEventStop))
        {
          reply(AsRecorderEventStop, msg->getType(), AS_ECODE_QUEUE_OPERATION_ERROR);
          return;
        }
    }
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::illegalFilterDone(MsgPacket *msg)
{
  FilterDoneCmd filter_result =
    msg->moveParam<FilterDoneCmd>();

  /* Even if illegal reply, but need to do post handling same as usual.
   * Because allocated areas and queue for encodeing should be free.
   */

  if (m_filter_instance)
    {
      m_filter_instance->recv_done();
    }

  dequeEncOutBuf();

  if (filter_result.event_type == ComponentExec)
    {
      if (m_output_buf_mh_que.empty())
        {
          /* In this case, return of flush request never comes.
           * If all of encode request was returned and external
           * command exists, transit to Ready.
           */

          if (checkExternalCmd())
            {
              AsRecorderEvent ext_evt = getExternalCmd();
              reply(ext_evt, MSG_AUD_MRC_CMD_STOP, AS_ECODE_OK);

              m_state = Ready;
            }
        }

      dequeEncInBuf();
    }
  else if (filter_result.event_type == ComponentFlush)
    {
      if (checkExternalCmd())
        {
          AsRecorderEvent ext_evt = getExternalCmd();
          reply(ext_evt, MSG_AUD_MRC_CMD_STOP, AS_ECODE_OK);

          m_state = Ready;
        }
    }
  else
    {
      /* nop */
    }
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::filterDoneOnActive(MsgPacket *msg)
{
  FilterDoneCmd filter_result =
    msg->moveParam<FilterDoneCmd>();

  ComponentCmpltParam cmplt;

  if (m_filter_instance)
    {
      m_filter_instance->recv_done(&cmplt);
    }

  if (filter_result.result)
    {
      bool is_end = m_cnv_in_que.top().is_end;

      bool write_result =
        writeToDataSinker(cmplt.output.mh,
                          cmplt.output.size);

      /* If write error, stop encoding and transit to ErrorStopping */

      if (!write_result)
        {
          bool stop_result = stopEnc();

          /* If stop failed, succeeding data should dispose */

          if (!stop_result)
            {
              if (checkExternalCmd())
                {
                  AsRecorderEvent ext_evt = getExternalCmd();
                  reply(ext_evt, MSG_AUD_MRC_CMD_STOP, AS_ECODE_OK);

                  m_state = Ready;
                }
              else
                {
                  m_state = WaitStop;
                }
            }
          else
            {
              m_state = ErrorStopping;
            }

          is_end = true;
        }

      /* Notify that the PCM data finished role */

      m_cnv_in_que.top().callback(0, is_end);
    }

  dequeEncInBuf();
  dequeEncOutBuf();
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::filterDoneOnStop(MsgPacket *msg)
{
  FilterDoneCmd filter_result =
    msg->moveParam<FilterDoneCmd>();

  ComponentCmpltParam cmplt;

  if (m_filter_instance)
    {
      m_filter_instance->recv_done(&cmplt);
    }

  if (filter_result.event_type == ComponentExec)
    {
      bool is_end = m_cnv_in_que.top().is_end;

      if (filter_result.result)
        {
          bool write_result =
            writeToDataSinker(cmplt.output.mh,
                              cmplt.output.size);

          if (!write_result)
            {
              /* Here, in stopping sequece, flush was already issued. */

              m_state = ErrorStopping;

              is_end = true;
            }
        }

      /* Notify that the PCM data finished role */

      m_cnv_in_que.top().callback(0, is_end);

      /* Free PCM and ES data area */

      dequeEncInBuf();
      dequeEncOutBuf();
    }
  else if (filter_result.event_type == ComponentFlush)
    {
      if (filter_result.result && (cmplt.output.size > 0))
        {
          writeToDataSinker(m_output_buf_mh_que.top(),
                            cmplt.output.size);
        }

      dequeEncOutBuf();

      m_rec_sink.finalize();

      if (checkExternalCmd())
        {
          AsRecorderEvent ext_evt = getExternalCmd();
          reply(ext_evt, MSG_AUD_MRC_CMD_STOP, AS_ECODE_OK);
        }

      m_state = Ready;
    }
  else
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::filterDoneOnErrorStop(MsgPacket *msg)
{
  FilterDoneCmd filter_result =
    msg->moveParam<FilterDoneCmd>();

  ComponentCmpltParam cmplt;

  if (m_filter_instance)
    {
      m_filter_instance->recv_done(&cmplt);
    }

  if (filter_result.event_type == ComponentExec)
    {
      bool is_end = m_cnv_in_que.top().is_end;

      if (filter_result.result && (cmplt.output.size > 0))
        {
          bool write_result =
            writeToDataSinker(cmplt.output.mh,
                              cmplt.output.size);

          if (!write_result)
            {
              /* Here, in error stopping sequece, flush was already issued. */
            }
        }

      /* Notify that the PCM data finished the role */

      m_cnv_in_que.top().callback(0, is_end);

      /* Free PCM and ES data area */

      dequeEncOutBuf();
      dequeEncInBuf();
    }
  else if (filter_result.event_type == ComponentFlush)
    {
      if (filter_result.result && (cmplt.output.size > 0))
        {
          writeToDataSinker(m_output_buf_mh_que.top(),
                            cmplt.output.size);
        }

      dequeEncOutBuf();

      m_rec_sink.finalize();

      /* Even if on error stopping, check exeternal command que and reply if
       * request is there. This suppose follow cases, Error on stopping and
       * stop request on error stopping. Other case will be transit to WaitStop,
       */

      if (checkExternalCmd())
        {
          AsRecorderEvent ext_evt = getExternalCmd();
          reply(ext_evt, MSG_AUD_MRC_CMD_STOP, AS_ECODE_OK);

          m_state = Ready;
        }
      else
        {
          m_state = WaitStop;
        }
    }
  else
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::illegalEncDone(MsgPacket *msg)
{
  EncCmpltParam enc_result = msg->moveParam<EncCmpltParam>();

  /* Even if illegal reply, but need to do post handling same as usual.
   * Because allocated areas and queue for encodeing should be free.
   */

  AS_encode_recv_done();

  dequeEncOutBuf();

  if (enc_result.event_type == Apu::ExecEvent)
    {
      if (m_output_buf_mh_que.empty())
        {
          /* In this case, return of flush request never comes.
           * If all of encode request was returned and external
           * command exists, transit to Ready.
           */

          if (checkExternalCmd())
            {
              AsRecorderEvent ext_evt = getExternalCmd();
              reply(ext_evt, MSG_AUD_MRC_CMD_STOP, AS_ECODE_OK);

              m_state = Ready;
            }
        }

      dequeEncInBuf();
    }
  else if (enc_result.event_type == Apu::FlushEvent)
    {
      if (checkExternalCmd())
        {
          AsRecorderEvent ext_evt = getExternalCmd();
          reply(ext_evt, MSG_AUD_MRC_CMD_STOP, AS_ECODE_OK);

          m_state = Ready;
        }
    }
  else
    {
      /* nop */
    }
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::encDoneOnActive(MsgPacket *msg)
{
  EncCmpltParam enc_result = msg->moveParam<EncCmpltParam>();
  AS_encode_recv_done();

  if (enc_result.result)
    {
      bool is_end = m_cnv_in_que.top().is_end;

      bool write_result =
        writeToDataSinker(m_output_buf_mh_que.top(),
                          enc_result.exec_enc_cmplt.output_buffer.size);

      /* If write error, transit to ErrorStopping state */

      if (!write_result)
        {
          bool stop_result = stopEnc();

          /* If stop failed, succeeding data should dispose */

          if (!stop_result)
            {
              if (checkExternalCmd())
                {
                  AsRecorderEvent ext_evt = getExternalCmd();
                  reply(ext_evt, MSG_AUD_MRC_CMD_STOP, AS_ECODE_OK);

                  m_state = Ready;
                }
              else
                {
                  m_state = WaitStop;
                }
            }
          else
            {
              m_state = ErrorStopping;
            }

          is_end = true;
        }

      /* Notify that the PCM data finished the role */

      m_cnv_in_que.top().callback(0, is_end);
    }

  /* Free PCM and ES data area */

  dequeEncInBuf();
  dequeEncOutBuf();
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::encDoneOnStop(MsgPacket *msg)
{
  EncCmpltParam enc_result = msg->moveParam<EncCmpltParam>();
  AS_encode_recv_done();

  if (enc_result.event_type == Apu::ExecEvent)
    {
      bool is_end = m_cnv_in_que.top().is_end;

      if (enc_result.result)
        {
          bool write_result =
            writeToDataSinker(m_output_buf_mh_que.top(),
                              enc_result.exec_enc_cmplt.output_buffer.size);

          if (!write_result)
            {
              /* Here, in stopping sequece, flush was already issued. */

              m_state = ErrorStopping;

              is_end = true;
            }
        }

      /* Notify that the PCM data finished the role */

      m_cnv_in_que.top().callback(0, is_end);

      /* Free PCM and ES data area */

      dequeEncInBuf();
      dequeEncOutBuf();
    }
  else if (enc_result.event_type == Apu::FlushEvent)
    {
      if (enc_result.result && (enc_result.stop_enc_cmplt.output_buffer.size > 0))
        {
          writeToDataSinker(m_output_buf_mh_que.top(),
                            enc_result.stop_enc_cmplt.output_buffer.size);
        }

      /* Free ES data area */

      dequeEncOutBuf();

      m_rec_sink.finalize();

      /* Transit to Ready state */

      m_state = Ready;

      if (checkExternalCmd())
        {
          AsRecorderEvent ext_evt = getExternalCmd();
          reply(ext_evt, MSG_AUD_MRC_CMD_STOP, AS_ECODE_OK);
        }
    }
  else
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::encDoneOnErrorStop(MsgPacket *msg)
{
  EncCmpltParam enc_result = msg->moveParam<EncCmpltParam>();
  AS_encode_recv_done();

  if (enc_result.event_type == Apu::ExecEvent)
    {
      bool is_end = m_cnv_in_que.top().is_end;

      if (enc_result.result)
        {
          bool write_result =
            writeToDataSinker(m_output_buf_mh_que.top(),
                              enc_result.exec_enc_cmplt.output_buffer.size);

          if (!write_result)
            {
              /* Here, in error stopping sequece, flush was already issued. */
            }
        }

      /* Notify that the PCM data finished the role */

      m_cnv_in_que.top().callback(0, is_end);

      /* Free PCM and ES data area */

      dequeEncOutBuf();
      dequeEncInBuf();
    }
  else if (enc_result.event_type == Apu::FlushEvent)
    {
      if (enc_result.result && (enc_result.stop_enc_cmplt.output_buffer.size > 0))
        {
          writeToDataSinker(m_output_buf_mh_que.top(),
                            enc_result.stop_enc_cmplt.output_buffer.size);
        }

      dequeEncOutBuf();

      m_rec_sink.finalize();

      /* Even if on error stopping, check exeternal command que and reply if
       * request is there. This suppose follow cases, Error on stopping and
       * stop request on error stopping. Other case will be transit to WaitStop,
       */

      if (checkExternalCmd())
        {
          AsRecorderEvent ext_evt = getExternalCmd();
          reply(ext_evt, MSG_AUD_MRC_CMD_STOP, AS_ECODE_OK);

          m_state = Ready;
        }
      else
        {
          m_state = WaitStop;
        }
    }
  else
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }
}

/*--------------------------------------------------------------------------*/
bool MediaRecorderObject::execEnc(AsPcmDataParam *inpcm)
{
  MemMgrLite::MemHandle outmh = getOutputBufAddr();

  if (m_codec_type == AudCodecLPCM)
    {
      ExecComponentParam param;

      param.input     = *inpcm;
      param.output = outmh;

      if ((m_filter_instance)
       && (param.input.mh.getPa())
       && (param.output.getPa()))
        {
          if (m_filter_instance->exec(param))
            {
              enqueEncInBuf(*inpcm);
              enqueEncOutBuf(outmh);
            }
          else
            {
              return false;
            }
        }
      else
        {
          return false;
        }
    }
  else if ((m_codec_type == AudCodecMP3) || (m_codec_type == AudCodecOPUS))
    {
      ExecEncParam param;

      param.input_buffer.p_buffer  =
        static_cast<unsigned long *>(inpcm->mh.getPa());
      param.input_buffer.size      = inpcm->size;
      param.output_buffer.p_buffer =
        static_cast<unsigned long *>((outmh.isNull()) ? NULL : outmh.getPa());
      param.output_buffer.size     = m_max_output_pcm_size;

      if ((param.input_buffer.p_buffer)
       && (param.output_buffer.p_buffer))
        {
          if (AS_encode_exec(&param))
            {
              enqueEncInBuf(*inpcm);
              enqueEncOutBuf(outmh);
            }
          else
            {
              return false;
            }
        }
      else
        {
          return false;
        }
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool MediaRecorderObject::stopEnc(void)
{
  MemMgrLite::MemHandle outmh = getOutputBufAddr();

  if (m_codec_type == AudCodecLPCM)
    {
      FlushComponentParam param;

      param.output = outmh;

      if (m_filter_instance)
        {
          /* Request flush even if it is not memory allocated.
           * At Flush request, Spcifing NULL addrrdd is allowed
           * and will be replyed with "ERROR".
           */

          if (m_filter_instance->flush(param))
            {
              enqueEncOutBuf(outmh);
            }
          else
            {
              return false;
            }
        }
    }
  else if ((m_codec_type == AudCodecMP3) || (m_codec_type == AudCodecOPUS))
    {
      StopEncParam param;

      param.output_buffer.p_buffer =
        static_cast<unsigned long *>((outmh.isNull()) ? NULL : outmh.getPa());
      param.output_buffer.size     = m_max_output_pcm_size;

      /* Request flush even if it is not memory allocated.
       * Spcifing NULL addr is allowed and will be replyed with "ERROR".
       */

      if (AS_encode_stop(&param))
        {
          enqueEncOutBuf(outmh);
        }
      else
        {
          return false;
        }
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool MediaRecorderObject::setExternalCmd(AsRecorderEvent ext_event)
{
  if (!m_external_cmd_que.push(ext_event))
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
AsRecorderEvent MediaRecorderObject::getExternalCmd(void)
{
  AsRecorderEvent ext_cmd = AsRecorderEventAct;

  if (m_external_cmd_que.empty())
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
    }
  else
    {
      ext_cmd = m_external_cmd_que.top();

      if (!m_external_cmd_que.pop())
        {
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }
    }

  return ext_cmd;
}

/*--------------------------------------------------------------------------*/
uint32_t MediaRecorderObject::checkExternalCmd(void)
{
  return m_external_cmd_que.size();
}

/*--------------------------------------------------------------------------*/
MemMgrLite::MemHandle MediaRecorderObject::getOutputBufAddr()
{
  MemMgrLite::MemHandle mh;
  if (mh.allocSeg(m_pool_id.output, m_max_output_pcm_size) != ERR_OK)
    {
      MEDIA_RECORDER_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
    }

  return mh;
}

/*--------------------------------------------------------------------------*/
uint32_t MediaRecorderObject::isValidActivateParam(
  const AsActivateRecorderParam &param)
{
  switch (param.output_device)
    {
      case AS_SETRECDR_STS_OUTPUTDEVICE_RAM:
        m_output_device = AS_SETRECDR_STS_OUTPUTDEVICE_RAM;
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_OUTPUT_DEVICE;
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t MediaRecorderObject::isValidInitParam(
  const RecorderCommand& cmd)
{
  uint32_t rst = AS_ECODE_OK;
  switch(cmd.init_param.codec_type)
    {
      case AS_CODECTYPE_MP3:
        rst = isValidInitParamMP3(cmd);
        break;

      case AS_CODECTYPE_LPCM:
        rst = isValidInitParamLPCM(cmd);
        break;

      case AS_CODECTYPE_OPUS:
        rst = isValidInitParamOPUS(cmd);
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }
  return rst;
}

/*--------------------------------------------------------------------------*/
uint32_t MediaRecorderObject::isValidInitParamMP3(
  const RecorderCommand& cmd)
{
  switch(cmd.init_param.channel_number)
    {
      case AS_CHANNEL_MONO:
      case AS_CHANNEL_STEREO:
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
    }
  switch(cmd.init_param.bit_length)
    {
      case AS_BITLENGTH_16:
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_BIT_LENGTH;
    }
  switch(cmd.init_param.sampling_rate)
    {
      case AS_SAMPLINGRATE_16000:
        {
          switch(cmd.init_param.bitrate)
            {
              case AS_BITRATE_8000:
                if (cmd.init_param.channel_number ==
                    AS_CHANNEL_STEREO)
                  {
                    MEDIA_RECORDER_ERR(
                      AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
                    return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
                  }
                break;

              case AS_BITRATE_16000:
              case AS_BITRATE_24000:
              case AS_BITRATE_32000:
              case AS_BITRATE_40000:
              case AS_BITRATE_48000:
              case AS_BITRATE_56000:
              case AS_BITRATE_64000:
              case AS_BITRATE_80000:
              case AS_BITRATE_96000:
              case AS_BITRATE_112000:
              case AS_BITRATE_128000:
              case AS_BITRATE_144000:
              case AS_BITRATE_160000:
                break;

              default:
                MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
                return AS_ECODE_COMMAND_PARAM_BIT_RATE;
            }
        }
        break;

      case AS_SAMPLINGRATE_48000:
        {
          switch(cmd.init_param.bitrate)
            {
              case AS_BITRATE_32000:
              case AS_BITRATE_40000:
              case AS_BITRATE_48000:
              case AS_BITRATE_56000:
              case AS_BITRATE_64000:
              case AS_BITRATE_80000:
              case AS_BITRATE_96000:
              case AS_BITRATE_112000:
              case AS_BITRATE_128000:
              case AS_BITRATE_160000:
              case AS_BITRATE_192000:
              case AS_BITRATE_224000:
              case AS_BITRATE_256000:
              case AS_BITRATE_320000:
                break;

              default:
                MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
                return AS_ECODE_COMMAND_PARAM_BIT_RATE;
            }
        }
        break;
      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t MediaRecorderObject::isValidInitParamLPCM(
  const RecorderCommand& cmd)
{
  switch(cmd.init_param.channel_number)
    {
      case AS_CHANNEL_MONO:
      case AS_CHANNEL_STEREO:
      case AS_CHANNEL_4CH:
      case AS_CHANNEL_6CH:
      case AS_CHANNEL_8CH:
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
    }
  switch(cmd.init_param.bit_length)
    {
      case AS_BITLENGTH_16:
        break;

      case AS_BITLENGTH_24:
      case AS_BITLENGTH_32:
        {
          cxd56_audio_clkmode_t clock_mode = cxd56_audio_get_clkmode();
          if ((CXD56_AUDIO_CLKMODE_HIRES == clock_mode
            && cmd.init_param.sampling_rate == AS_SAMPLINGRATE_192000)
           || (CXD56_AUDIO_CLKMODE_NORMAL == clock_mode
            && cmd.init_param.sampling_rate == AS_SAMPLINGRATE_48000))
            {
              break;
            }
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          return AS_ECODE_COMMAND_PARAM_BIT_LENGTH;
        }
      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_BIT_LENGTH;
    }
  switch(cmd.init_param.sampling_rate)
    {
      case AS_SAMPLINGRATE_16000:
      case AS_SAMPLINGRATE_48000:
        break;

      case AS_SAMPLINGRATE_192000:
        {
          cxd56_audio_clkmode_t clock_mode = cxd56_audio_get_clkmode();
          if (CXD56_AUDIO_CLKMODE_HIRES == clock_mode)
            {
              break;
            }
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
        }
      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t MediaRecorderObject::isValidInitParamOPUS(
  const RecorderCommand& cmd)
{
  switch(cmd.init_param.channel_number)
    {
      case AS_CHANNEL_MONO:
      case AS_CHANNEL_STEREO:
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
    }
  switch(cmd.init_param.bit_length)
    {
      case AS_BITLENGTH_16:
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_BIT_LENGTH;
    }
  switch(cmd.init_param.sampling_rate)
    {
      case AS_SAMPLINGRATE_8000:
      case AS_SAMPLINGRATE_16000:
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
    }
  switch(cmd.init_param.bitrate)
    {
      case AS_BITRATE_8000:
      case AS_BITRATE_16000:
        break;

      default:
        MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_BIT_RATE;
    }
  if (cmd.init_param.computational_complexity >
      AS_INITREC_COMPLEXITY_10)
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_COMPLEXITY;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
bool MediaRecorderObject::writeToDataSinker(
  const MemMgrLite::MemHandle& mh,
  uint32_t byte_size)
{
  AudioRecSinkData_s sink_data;
  sink_data.mh        = mh;
  sink_data.byte_size = byte_size;

  return m_rec_sink.write(sink_data);
}

/*--------------------------------------------------------------------------*/
bool MediaRecorderObject::checkAndSetMemPool(void)
{
  if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.output))
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  m_max_output_pcm_size = (MemMgrLite::Manager::getPoolSize(m_pool_id.output)) /
    (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.output));

  if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.cmp))
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  if ((int)(sizeof(Apu::Wien2ApuCmd)) >
      (MemMgrLite::Manager::getPoolSize(m_pool_id.cmp))/
      (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.cmp)))
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.cmp))
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool MediaRecorderObject::isNeedUpsampling(int32_t sampling_rate)
{
  /* The condition that the sampling rate converter is unnecessary is
   * when input sampling and output sampling are equal.
   * If clock mode is normal, the input sampling rate is 48000.
   * Also, if the clock mode is HI-Res, the input sampling rate is 192000.
   * Therefore, if the clock mode is normal and output sampling is 48000,
   * and clock mode is Hi-Res and output sampling is 192000,
   * upsampling is unnecessary.
   */

  cxd56_audio_clkmode_t clock_mode = cxd56_audio_get_clkmode();
  if (((sampling_rate == AS_SAMPLINGRATE_48000) &&
       (clock_mode == CXD56_AUDIO_CLKMODE_NORMAL)) ||
      ((sampling_rate == AS_SAMPLINGRATE_192000) &&
       (clock_mode == CXD56_AUDIO_CLKMODE_HIRES)))
    {
      /* No need upsampling. */

      return false;
    }
  /* Need upsampoing. */

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*--------------------------------------------------------------------------*/
static bool CreateMediaRecorder(AsObjectParams_t params, AudioAttentionCb attcb)
{
  /* Register attention callback */

  MEDIA_RECORDER_REG_ATTCB(attcb);

  /* Reset Message queue. */

  FAR MsgQueBlock *que;
  err_t err_code = MsgLib::referMsgQueBlock(params.msgq_id.self, &que);
  F_ASSERT(err_code == ERR_OK);
  que->reset();

  /* Init pthread attributes object. */
  pthread_attr_t attr;

  pthread_attr_init(&attr);

  pthread_t pid;

  /* Set pthread scheduling parameter. */

  struct sched_param sch_param;

  sch_param.sched_priority = 150;
  attr.stacksize           = 1024 * 2;

  pthread_attr_setschedparam(&attr, &sch_param);

  /* Create thread. */

  int ret = pthread_create(&pid,
                           &attr,
                           (pthread_startroutine_t)AS_MediaRecorderObjEntry,
                           (pthread_addr_t) &params);
  if (ret < 0)
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  pthread_setname_np(pid, "media_recorder");

  MediaRecorderObject::set_pid(pid);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_CreateMediaRecorder(FAR AsCreateRecorderParams_t *param, AudioAttentionCb attcb)
{
  AsObjectParams_t params;

  if (param == NULL)
    {
      return false;
    }

  params.msgq_id.self = param->msgq_id.recorder;
  params.msgq_id.from = param->msgq_id.mng;
  params.msgq_id.cmp  = param->msgq_id.dsp;

  params.pool_id.input  = param->pool_id.input;
  params.pool_id.output = param->pool_id.output;
  params.pool_id.cmp    = param->pool_id.dsp;;

  return CreateMediaRecorder(params, attcb);

}

/*--------------------------------------------------------------------------*/
bool AS_CreateMediaRecorder(FAR AsCreateRecorderParams_t *param)
{
  return AS_CreateMediaRecorder(param, NULL);
}

/*--------------------------------------------------------------------------*/
bool AS_ActivateMediaRecorder(FAR AsActivateRecorder *actparam)
{
  /* Parameter check */

  if ( (actparam == NULL) || !AS_checkAvailabilityMediaRecorder() )
    {
      return false;
    }

  /* Activate */

  RecorderCommand cmd;

  cmd.act_param = *actparam;

  err_t er = MsgLib::send<RecorderCommand>(MediaRecorderObject::get_msgq_id(),
                                           MsgPriNormal,
                                           MSG_AUD_MRC_CMD_ACTIVATE,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_InitMediaRecorder(FAR AsInitRecorderParam *initparam)
{
  /* Parameter check */

  if ( (initparam == NULL) || !AS_checkAvailabilityMediaRecorder() )
    {
      return false;
    }

  /* Init */

  RecorderCommand cmd;

  cmd.init_param = *initparam;

  err_t er = MsgLib::send<RecorderCommand>(MediaRecorderObject::get_msgq_id(),
                                           MsgPriNormal,
                                           MSG_AUD_MRC_CMD_INIT,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_ReqEncodeMediaRecorder(AsPcmDataParam *pcmparam)
{
  /* Parameter check */

  if ( (pcmparam == NULL) || !AS_checkAvailabilityMediaRecorder() )
    {
      return false;
    }

  /* Request Encode */

  err_t er = MsgLib::send<AsPcmDataParam>(MediaRecorderObject::get_msgq_id(),
                                          MsgPriNormal,
                                          MSG_AUD_MRC_CMD_ENCODE,
                                          NULL,
                                          *pcmparam);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StartMediaRecorder(void)
{
  /* Start */

  if ( !AS_checkAvailabilityMediaRecorder() )
    {
      return false;
    }

  RecorderCommand cmd;

  err_t er = MsgLib::send<RecorderCommand>(MediaRecorderObject::get_msgq_id(),
                                           MsgPriNormal,
                                           MSG_AUD_MRC_CMD_START,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StopMediaRecorder(void)
{
  /* Stop */

  if ( !AS_checkAvailabilityMediaRecorder() )
    {
      return false;
    }

  RecorderCommand cmd;

  err_t er = MsgLib::send<RecorderCommand>(MediaRecorderObject::get_msgq_id(),
                                           MsgPriNormal,
                                           MSG_AUD_MRC_CMD_STOP,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivateMediaRecorder(void)
{
  if ( !AS_checkAvailabilityMediaRecorder() )
    {
      return false;
    }

  RecorderCommand cmd;

  err_t er = MsgLib::send<RecorderCommand>(MediaRecorderObject::get_msgq_id(),
                                           MsgPriNormal,
                                           MSG_AUD_MRC_CMD_DEACTIVATE,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeleteMediaRecorder(void)
{

  pid_t pid = MediaRecorderObject::get_pid();
  MediaRecorderObject::destory();

  if (pid == INVALID_PROCESS_ID)
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  pthread_cancel(pid);
  pthread_join(pid, NULL);

  MediaRecorderObject::set_pid(INVALID_PROCESS_ID);

  /* Unregister attention callback */

  MEDIA_RECORDER_UNREG_ATTCB();

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_checkAvailabilityMediaRecorder(void)
{
  return (MediaRecorderObject::get_instance() != NULL);
}

/*--------------------------------------------------------------------------*/
void MediaRecorderObject::create(AsObjectParams_t* params)
{
  MediaRecorderObject* inst = new(MediaRecorderObject::get_adr()) MediaRecorderObject(params->msgq_id, params->pool_id);

  if(inst != NULL)
    {
      inst->run();
    }
  else
    {
      MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }
}
