/****************************************************************************
 * modules/audio/components/encoder/encoder_component.cpp
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

#include <arch/chip/backuplog.h>
#include <sdk/debug.h>

#include "encoder_component.h"
#include "apus/cpuif_cmd.h"

#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "audio/audio_message_types.h"
#include "dsp_driver/include/dsp_drv.h"
#include "apus/dsp_audio_version.h"
#include "wien2_internal_packet.h"

#define DBG_MODULE DBG_MODULE_AS

__WIEN2_BEGIN_NAMESPACE

static EncoderComponent *s_instance = NULL;

extern "C" {
/*--------------------------------------------------------------------
  C Interface
  --------------------------------------------------------------------*/
uint32_t AS_encode_activate(AudioCodec param,
                            const char *path,
                            MsgQueId apu_dtq,
                            PoolId apu_pool_id,
                            uint32_t *dsp_inf)
{
  if (s_instance == NULL)
    {
      s_instance = new EncoderComponent(apu_dtq,apu_pool_id);
    }

  return s_instance->activate_apu(param, path, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_encode_deactivate()
{
  bool result = s_instance->deactivate_apu();

  delete s_instance;
  s_instance = NULL;

  return result;
}

/*--------------------------------------------------------------------*/
uint32_t AS_encode_init(const InitEncParam *param, uint32_t *dsp_inf)
{
  /* Parameter check */

  if (param == NULL || dsp_inf == NULL)
    {
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Init */

  return s_instance->init_apu(*param, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_encode_exec(const ExecEncParam *param)
{
  /* Parameter check */

  if (param == NULL)
    {
      return false;
    }

  /* Execute */

  return s_instance->exec_apu(*param);
}

/*--------------------------------------------------------------------*/
bool AS_encode_stop(const StopEncParam *param)
{
  /* Parameter check */

  if (param == NULL)
    {
      return false;
    }

  /* Stop */

  return s_instance->flush_apu(*param);
}

/*--------------------------------------------------------------------*/
bool AS_encode_recv_done(void)
{
  return s_instance->recv_done();
}

/*--------------------------------------------------------------------*/
bool AS_encode_recv_apu(void *p_param)
{
  return s_instance->recv_apu(p_param);
}

/*--------------------------------------------------------------------*/

} /* extern "C" */

/*--------------------------------------------------------------------*/
/* callback function for DSP Driver */
/*--------------------------------------------------------------------*/
void enc_dsp_done_callback(void *p_response, void *p_instance)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
          if (p_param->event_type == Apu::BootEvent)
            {
              err_t er = MsgLib::send<uint32_t>(((EncoderComponent*)p_instance)->get_apu_mid(),
                                                MsgPriNormal,
                                                MSG_ISR_APU0,
                                                0,
                                                p_param->data.value);
              F_ASSERT(er == ERR_OK);
            }
          else if (p_param->event_type == Apu::ErrorEvent)
            {
              ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_ASSETION_FAIL);
            }
          break;

      case Apu::EncMode:
          AS_encode_recv_apu(p_response);
          break;

      default:
          ENCODER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          break;
    }
}

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
uint32_t EncoderComponent::activate_apu(AudioCodec param,
                                        const char *path,
                                        uint32_t *dsp_inf)
{
  char filepath[64];
  uint32_t encoder_dsp_version;

  ENCODER_DBG("ACT: codec %d\n", param);

  switch (param)
    {
      case AudCodecMP3:
          snprintf(filepath, sizeof(filepath), "%s/MP3ENC", path);
          encoder_dsp_version = DSP_MP3ENC_VERSION;
          break;

      case AudCodecOPUS:
          snprintf(filepath, sizeof(filepath), "%s/OPUSENC", path);
          encoder_dsp_version = DSP_OPUSENC_VERSION;
          break;

      default:
          ENCODER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }

  /* Load DSP binary */

  int ret = DD_Load(filepath, 
                    enc_dsp_done_callback, 
                    (void *)this, 
                    &m_dsp_handler,
                    DspBinTypeELFwoBind);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_Load() failure. %d\n", ret);
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  /* wait for DSP boot up... */

  dsp_boot_check(m_apu_dtq, dsp_inf);

  /* DSP version check */

  if (encoder_dsp_version != *dsp_inf)
    {
      logerr("DSP version unmatch. expect %08lx / actual %08lx",
              encoder_dsp_version, *dsp_inf);

      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);
    }

  ENCODER_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  memset(&m_debug_log_info, 0, sizeof(m_debug_log_info));
#endif

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool EncoderComponent::deactivate_apu(void)
{
  ENCODER_DBG("DEACT:\n");

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  if (m_debug_log_info.addr)
    {
      up_backuplog_free(m_debug_log_info.name);
    }
#endif

  int ret = DD_Unload(m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_UnLoad() failure. %d\n", ret);
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      return false;
    }
  ENCODER_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t EncoderComponent::init_apu(const InitEncParam& param, uint32_t *dsp_inf)
{
  ENCODER_DBG("INIT: codec %d, infs %ld, outfs %ld, bit len %d, ch num %d, complexity %d, bit rate %ld, cb %p\n",
              param.codec_type, param.input_sampling_rate, param.output_sampling_rate, param.bit_width,
              param.channel_num, param.complexity, param.bit_rate, param.callback);

  m_callback = param.callback;

  Apu::Wien2ApuCmd* p_apu_cmd = static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return AS_ECODE_ENCODER_LIB_INITIALIZE_ERROR;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::EncMode;
  p_apu_cmd->header.event_type   = Apu::InitEvent;

  /* When MP3ENC, channel_config and bit_rate is not referenced */

  p_apu_cmd->init_enc_cmd.codec_type          = param.codec_type;
  p_apu_cmd->init_enc_cmd.channel_num         = param.channel_num;
  p_apu_cmd->init_enc_cmd.input_sampling_rate = param.input_sampling_rate;

  if (param.channel_num == 2)
    {
      p_apu_cmd->init_enc_cmd.channel_config = AUD_PCM_CH_CONFIG_2_0;
    }
  else
    {
      p_apu_cmd->init_enc_cmd.channel_config = AUD_PCM_CH_CONFIG_1_0;
    }

  p_apu_cmd->init_enc_cmd.bit_length           = param.bit_width;
  p_apu_cmd->init_enc_cmd.output_sampling_rate = param.output_sampling_rate;

  if (param.codec_type == AudCodecMP3)
    {
      p_apu_cmd->init_enc_cmd.bit_rate = param.bit_rate;
    }

  if (param.codec_type == AudCodecSBC)
    {
      p_apu_cmd->init_enc_cmd.init_sbc_enc_param.block_len   = 16;
      p_apu_cmd->init_enc_cmd.init_sbc_enc_param.subband_num = 8;
      p_apu_cmd->init_enc_cmd.init_sbc_enc_param.enc_type    = 1; /* type SNR */
    }

  if (param.codec_type == AudCodecOPUS)
    {
      p_apu_cmd->init_enc_cmd.bit_rate                                = param.bit_rate;
      p_apu_cmd->init_enc_cmd.init_opus_enc_param.complexity          = param.complexity;
#ifdef CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT
      p_apu_cmd->init_enc_cmd.init_opus_enc_param.use_original_format = false;
#else
      p_apu_cmd->init_enc_cmd.init_opus_enc_param.use_original_format = true;
#endif /* CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT */
    }

  p_apu_cmd->init_enc_cmd.debug_dump_info.addr = NULL;
  p_apu_cmd->init_enc_cmd.debug_dump_info.size = 0;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  /* initialization of DSP debug dump */

  if (m_debug_log_info.addr == NULL)
    {
      if (param.codec_type == AudCodecMP3)
        {
          strncpy(m_debug_log_info.name, "MP3ENC", sizeof(m_debug_log_info.name));
        }
      else
        {
          strncpy(m_debug_log_info.name, "OPUSENC", sizeof(m_debug_log_info.name));
        }

      m_debug_log_info.addr = up_backuplog_alloc(m_debug_log_info.name,
                                                 AUDIOUTILS_DSP_DEBUG_DUMP_SIZE);

      if (m_debug_log_info.addr == NULL)
        {
          ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR);
          return AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR;
        }
    }

  if (m_debug_log_info.addr != NULL)
    {
      p_apu_cmd->init_enc_cmd.debug_dump_info.addr = m_debug_log_info.addr;
      p_apu_cmd->init_enc_cmd.debug_dump_info.size = AUDIOUTILS_DSP_DEBUG_DUMP_SIZE;
    }
  else
    {
      memset(m_debug_log_info.name, 0, sizeof(m_debug_log_info.name));
    }
#endif

  send_apu(p_apu_cmd);

  /* Wait init completion and receive reply information */

  Apu::InternalResult internal_result;
  uint32_t rst = dsp_init_check<Apu::InternalResult>(m_apu_dtq, &internal_result);
  *dsp_inf = internal_result.value;

  return rst;
}

/*--------------------------------------------------------------------*/
bool EncoderComponent::exec_apu(const ExecEncParam& param)
{
  /* Encode data area check */

  if ((param.input_buffer.p_buffer == NULL)
   || (param.output_buffer.p_buffer == NULL))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Execute encode */

  Apu::Wien2ApuCmd* p_apu_cmd = static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::EncMode;
  p_apu_cmd->header.event_type   = Apu::ExecEvent;

  p_apu_cmd->exec_enc_cmd.input_buffer  = param.input_buffer;
  p_apu_cmd->exec_enc_cmd.output_buffer = param.output_buffer;

  send_apu(p_apu_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool EncoderComponent::flush_apu(const StopEncParam& param)
{
  ENCODER_DBG("FLUSH:\n");

  /* Regardless of output buffer is not allocated, send Flush Request
   * to DSP. Because it is needed by DSP to finish process correctly.
   */

  /* Flush */

  Apu::Wien2ApuCmd* p_apu_cmd = static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::EncMode;
  p_apu_cmd->header.event_type   = Apu::FlushEvent;

  p_apu_cmd->flush_enc_cmd.output_buffer = param.output_buffer;

  send_apu(p_apu_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool EncoderComponent::recv_apu(void *p_response)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;
  Apu::Wien2ApuCmd *packet = static_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  if (Apu::ApuExecOK != packet->result.exec_result)
    {
      ENCODER_WARN(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  if (Apu::InitEvent == packet->header.event_type)
    {
      /* Notify init completion to myself */

      Apu::InternalResult internal_result = packet->result.internal_result[0];
      dsp_init_complete<Apu::InternalResult>(m_apu_dtq, packet->result.exec_result, &internal_result);

      return true;
    }

  return m_callback(p_param);
}

/*--------------------------------------------------------------------*/
void EncoderComponent::send_apu(Apu::Wien2ApuCmd* p_cmd)
{
  DspDrvComPrm_t com_param;
  com_param.event_type = p_cmd->header.event_type;
  com_param.process_mode = p_cmd->header.process_mode;
  com_param.type = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam = reinterpret_cast<void*>(p_cmd);

  int ret = DD_SendCommand(m_dsp_handler, &com_param);
  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_SendCommand() failure. %d\n", ret);
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_SEND_ERROR);
      return;
    }
}

__WIEN2_END_NAMESPACE

