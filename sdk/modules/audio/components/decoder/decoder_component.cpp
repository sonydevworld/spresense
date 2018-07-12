/****************************************************************************
 * modules/audio/components/decoder/decoder_component.cpp
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

#include "decoder_component.h"

#include "memutils/common_utils/common_assert.h"
#include "common/audio_internal_message_types.h"
#include "memutils/message/Message.h"

#include "apus/cpuif_cmd.h"
#include "dsp_driver/include/dsp_drv.h"
#include <arch/chip/pm.h>
#include "apus/dsp_audio_version.h"
#include "wien2_internal_packet.h"

#define DBG_MODULE DBG_MODULE_AS

__USING_WIEN2

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
static struct pm_cpu_freqlock_s g_decode_hvlock;
#endif

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
#define LOG_ENTRY_NUM 4

static const char g_entry_name[LOG_ENTRY_NUM][LOG_ENTRY_NAME] =
{
  "MP3DEC",
  "WAVDEC",
  "AACDEC",
  "OPUSDEC"
};

static uint32_t g_namemap = 0x00000000;
#endif

extern "C" {
/*--------------------------------------------------------------------
    C Interface
  --------------------------------------------------------------------*/
uint32_t AS_decode_init(const InitDecCompParam *param,
                        void *p_instance,
                        uint32_t *dsp_inf)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL || dsp_inf == NULL)
    {
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Init */

  return ((DecoderComponent *)p_instance)->init_apu(*param, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_decode_exec(const ExecDecCompParam *param, void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      return false;
    }

  /* Execute */

  return ((DecoderComponent *)p_instance)->exec_apu(*param);
}

/*--------------------------------------------------------------------*/
bool AS_decode_stop(const StopDecCompParam *param, void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      return false;
    }

  /* Stop */

  return ((DecoderComponent *)p_instance)->flush_apu(*param);
}

/*--------------------------------------------------------------------*/
bool AS_decode_setparam(const SetDecCompParam *param, void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      return false;
    }

  /* Set param */

  return ((DecoderComponent *)p_instance)->setparam_apu(*param);
}

/*--------------------------------------------------------------------*/
bool AS_decode_recv_apu(void *p_param, void *p_instance)
{
  return ((DecoderComponent *)p_instance)->recv_apu(p_param);
}

/*--------------------------------------------------------------------*/
bool AS_decode_recv_done(void *p_instance)
{
  return ((DecoderComponent *)p_instance)->recv_done();
}

/*--------------------------------------------------------------------*/
uint32_t AS_decode_activate(AudioCodec param,
                            const char *path,
                            void **p_instance,
                            MemMgrLite::PoolId apu_pool_id,
                            MsgQueId apu_mid,
                            uint32_t *dsp_inf)
{
  if (p_instance == NULL)
    {
      DECODER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Reply pointer of self instance, which is used for API call. */

  *p_instance = (void*)(new DecoderComponent(apu_pool_id,apu_mid));

  if (*p_instance == NULL)
    {
      DECODER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  return ((DecoderComponent*)*p_instance)->activate(param, path, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_decode_deactivate(void *p_instance)
{

  if ((DecoderComponent *)p_instance != NULL)
    {
      ((DecoderComponent *)p_instance)->deactivate();
      delete (DecoderComponent*)p_instance;
      return true;
    }

  DECODER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);

  return false;
}

/*--------------------------------------------------------------------*/
void cbRcvDspRes(void *p_response, void *p_instance)
{
  /* Callback from DSP when DSP process is done. */

  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
        if (p_param->event_type == Apu::BootEvent)
          {
            err_t er = MsgLib::send<uint32_t>
                      (((DecoderComponent*)p_instance)->get_apu_mid(),
                       MsgPriNormal,
                       MSG_ISR_APU0,
                       0,
                       p_param->data.value);

            if (er != ERR_OK)
              {
                F_ASSERT(0);
              }
          }
        else if (p_param->event_type == Apu::ErrorEvent)
          {
            DECODER_ERR(AS_ATTENTION_SUB_CODE_DSP_ASSETION_FAIL);
          }
        break;

      case Apu::DecMode:
        AS_decode_recv_apu(p_response, p_instance);
        break;

      default:
        DECODER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}

} /* extern "C" */

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
uint32_t DecoderComponent::init_apu(const InitDecCompParam& param,
                                    uint32_t *dsp_inf)
{
  DECODER_DBG("INIT: code %d, fs %d, ch num %d, bit len %d, "
              "sample num %d, cb %08x, req %08x\n",
               param.codec_type, param.input_sampling_rate, param.channel_num,
               param.bit_width, param.frame_sample_num, param.callback,
               param.p_requester);

  m_callback = param.callback;
  m_p_requester = param.p_requester;

  Apu::Wien2ApuCmd *p_apu_cmd =
    reinterpret_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return AS_ECODE_DECODER_LIB_INITIALIZE_ERROR;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::DecMode;
  p_apu_cmd->header.event_type   = Apu::InitEvent;

  /* Note: CXD5602 supports only stereo format. */

  p_apu_cmd->init_dec_cmd.codec_type     = param.codec_type;
  p_apu_cmd->init_dec_cmd.channel_num    = param.channel_num;
  p_apu_cmd->init_dec_cmd.sampling_rate  = param.input_sampling_rate;
  p_apu_cmd->init_dec_cmd.channel_config = AUD_PCM_CH_CONFIG_2_0;

  p_apu_cmd->init_dec_cmd.out_pcm_param.bit_length     = param.bit_width;
  p_apu_cmd->init_dec_cmd.out_pcm_param.channel_format = Aud2ChannelFormat;
  p_apu_cmd->init_dec_cmd.out_pcm_param.channel_select = AudChSelThrough;

  if ((p_apu_cmd->init_dec_cmd.codec_type == AudCodecLPCM
    || p_apu_cmd->init_dec_cmd.codec_type == AudCodecFLAC)
   && ((p_apu_cmd->init_dec_cmd.sampling_rate == 64000)
    || (p_apu_cmd->init_dec_cmd.sampling_rate == 88200)
    || (p_apu_cmd->init_dec_cmd.sampling_rate == 96000)
    || (p_apu_cmd->init_dec_cmd.sampling_rate == 176400)
    || (p_apu_cmd->init_dec_cmd.sampling_rate == 192000)))
    {
      p_apu_cmd->init_dec_cmd.out_pcm_param.sampling_rate = 192000;
    }
  else
    {
      p_apu_cmd->init_dec_cmd.out_pcm_param.sampling_rate = 48000;
    }

  /* Note: CXD5602 supports only stereo format. */

  p_apu_cmd->init_dec_cmd.out_pcm_param.channel_config =
    AUD_PCM_CH_CONFIG_2_0;

  p_apu_cmd->init_dec_cmd.out_pcm_param.decoder_output_sample =
    param.frame_sample_num;

  p_apu_cmd->init_dec_cmd.debug_dump_info.addr = NULL;
  p_apu_cmd->init_dec_cmd.debug_dump_info.size = 0;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  /* Initialization for DSP debug dump. */

  if (m_debug_log_info.info.addr == NULL)
    {
      strncpy(m_debug_log_info.info.name,
              g_entry_name[param.codec_type],
              sizeof(m_debug_log_info.info.name));

      m_debug_log_info.namemap_ps = 1 << (param.codec_type * 2);

      if ((g_namemap & m_debug_log_info.namemap_ps) == 0)
        {
          strncat(m_debug_log_info.info.name, "0", sizeof(char));
        }
      else
        {
          strncat(m_debug_log_info.info.name, "1", sizeof(char));
          m_debug_log_info.namemap_ps <<= 1;
        }

      g_namemap |= m_debug_log_info.namemap_ps;

      m_debug_log_info.info.addr = up_backuplog_alloc
                                            (m_debug_log_info.info.name,
                                             AUDIOUTILS_DSP_DEBUG_DUMP_SIZE);

      if (m_debug_log_info.info.addr == NULL)
        {
          DECODER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR);
          return AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR;
        }
    }

  if (m_debug_log_info.info.addr != NULL)
    {
      p_apu_cmd->init_dec_cmd.debug_dump_info.addr =
        m_debug_log_info.info.addr;

      p_apu_cmd->init_dec_cmd.debug_dump_info.size =
        AUDIOUTILS_DSP_DEBUG_DUMP_SIZE;
    }
  else
    {
      memset(m_debug_log_info.info.name,
             0,
             sizeof(m_debug_log_info.info.name));
    }
#endif

  send_apu(p_apu_cmd);

  uint32_t rst = dsp_init_check(m_apu_mid, dsp_inf);

  return rst;
}

/*--------------------------------------------------------------------*/
bool DecoderComponent::exec_apu(const ExecDecCompParam& param)
{
  Apu::Wien2ApuCmd *p_apu_cmd =
    reinterpret_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::DecMode;
  p_apu_cmd->header.event_type   = Apu::ExecEvent;

  p_apu_cmd->exec_dec_cmd.input_buffer  = param.input_buffer;
  p_apu_cmd->exec_dec_cmd.output_buffer = param.output_buffer;
  p_apu_cmd->exec_dec_cmd.num_of_au     = param.num_of_au;

  send_apu(p_apu_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool DecoderComponent::flush_apu(const StopDecCompParam& param)
{
  /* The number of time to send FLUSH is depend on type of codec or filter */

  Apu::Wien2ApuCmd *p_apu_cmd =
    reinterpret_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::DecMode;
  p_apu_cmd->header.event_type   = Apu::FlushEvent;

  p_apu_cmd->flush_dec_cmd.output_buffer = param.output_buffer;

  send_apu(p_apu_cmd);
  return true;
}

/*--------------------------------------------------------------------*/
bool DecoderComponent::setparam_apu(const SetDecCompParam& param)
{
  Apu::Wien2ApuCmd *p_apu_cmd =
    reinterpret_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::DecMode;
  p_apu_cmd->header.event_type   = Apu::SetParamEvent;

  p_apu_cmd->setparam_dec_cmd.l_gain = param.l_gain;
  p_apu_cmd->setparam_dec_cmd.r_gain = param.r_gain;

  send_apu(p_apu_cmd);
  return true;
}

/*--------------------------------------------------------------------*/
void DecoderComponent::send_apu(Apu::Wien2ApuCmd *p_cmd)
{
  DspDrvComPrm_t com_param;

  com_param.event_type   = p_cmd->header.event_type;
  com_param.process_mode = p_cmd->header.process_mode;
  com_param.type         = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam  = reinterpret_cast<void*>(p_cmd);

  int ret = DD_SendCommand(m_dsp_handler, &com_param);
  if (ret != DSPDRV_NOERROR)
    {
      _err("DD_SendCommand() failure. %d\n", ret);
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_SEND_ERROR);
      return;
    }
}

/*--------------------------------------------------------------------*/
bool DecoderComponent::recv_apu(void *p_response)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;
  Apu::Wien2ApuCmd *packet =
    static_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  if (Apu::ApuExecOK != packet->result.exec_result)
    {
      DECODER_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  if (Apu::InitEvent == packet->header.event_type)
    {
      dsp_init_complete(m_apu_mid, packet);
      return true;
    }

  /* Notify to requester */

  return m_callback(p_param, m_p_requester);
}

/*--------------------------------------------------------------------*/
uint32_t DecoderComponent::activate(AudioCodec param,
                                    const char *path,
                                    uint32_t *dsp_inf)
{
  char filepath[64];
  uint32_t decoder_dsp_version;

  DECODER_DBG("ACT: codec %d\n", param);

  switch (param)
    {
      case AudCodecMP3:
        snprintf(filepath, sizeof(filepath), "%s/MP3DEC", path);
        decoder_dsp_version = DSP_MP3DEC_VERSION;
        break;

      case AudCodecLPCM:
        snprintf(filepath, sizeof(filepath), "%s/WAVDEC", path);
        decoder_dsp_version = DSP_WAVDEC_VERSION;
        break;

      case AudCodecAAC:
        snprintf(filepath, sizeof(filepath), "%s/AACDEC", path);
        decoder_dsp_version = DSP_AACDEC_VERSION;
        break;

      case AudCodecOPUS:
        snprintf(filepath, sizeof(filepath), "%s/OPUSDEC", path);
        decoder_dsp_version = DSP_OPUSDEC_VERSION;
        break;

      default:
        DECODER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
  /* Lock HV performance to avoid loading time becomes too long */

  g_decode_hvlock.count = 0;
  g_decode_hvlock.info = PM_CPUFREQLOCK_TAG('D', 'C', 0x1199);
  g_decode_hvlock.flag = PM_CPUFREQLOCK_FLAG_HV;

  up_pm_acquire_freqlock(&g_decode_hvlock);
#endif

  /* Load DSP */

  int ret = DD_Load(filepath, cbRcvDspRes, (void*)this, &m_dsp_handler);

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
  up_pm_release_freqlock(&g_decode_hvlock);
#endif

  if (ret != DSPDRV_NOERROR)
    {
      _err("DD_Load() failure. %d\n", ret);
      DECODER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  if (!dsp_boot_check(m_apu_mid, decoder_dsp_version, dsp_inf))
    {
      DECODER_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);

      ret = DD_Unload(m_dsp_handler);

      if (ret != DSPDRV_NOERROR)
        {
          _err("DD_UnLoad() failure. %d\n", ret);
          DECODER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
        }

      return AS_ECODE_DSP_VERSION_ERROR;
    }

  DECODER_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  memset(&m_debug_log_info, 0, sizeof(m_debug_log_info));
#endif

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool DecoderComponent::deactivate(void)
{
  bool result = true;

  DECODER_DBG("DEACT:\n");

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  if (m_debug_log_info.info.addr)
    {
      up_backuplog_free(m_debug_log_info.info.name);
      g_namemap &= ~m_debug_log_info.namemap_ps;
    }
#endif

  int ret = DD_Unload(m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      _err("DD_UnLoad() failure. %d\n", ret);
      DECODER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      result = false;
    }

  DECODER_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);

  return result;
}

