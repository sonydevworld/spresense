/****************************************************************************
 * modules/audio/components/filter/mpp_filter_component.cpp
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

#include <sdk/debug.h>
#include "components/filter/mpp_filter_component.h"
#include "apus/cpuif_cmd.h"

#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "audio/audio_message_types.h"
#include "apus/apu_context_ids.h"
#include "debug/dbg_log.h"
#include "apus/dsp_audio_version.h"

#define DBG_MODULE DBG_MODULE_AS

__WIEN2_BEGIN_NAMESPACE

#define DSP_CORE_ID_DUMMY 3

#define XLOUD_DEFAULT_LEVEL 30
#define XLOUD_DEFAULT_HEADROOM_IN 0
#define XLOUD_DEFAULT_HEADROOM_OUT 0
#define XLOUD_DEFAULT_EAX_LEVEL 30

/*--------------------------------------------------------------------*/
void mpp_filter_dsp_done_callback(void *p_response, void *p_instance)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;
  Apu::Wien2ApuCmd *packet =
    reinterpret_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
        switch (p_param->event_type)
          {
            case Apu::BootEvent:
              {
                err_t er = MsgLib::send<uint32_t>
                        (((MPPComponent*)p_instance)->get_apu_mid(),
                         MsgPriNormal,
                         MSG_ISR_APU0,
                         0,
                         p_param->data.value);

                F_ASSERT(er == ERR_OK);
              }
              break;

            case Apu::ErrorEvent:
                FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_ASSETION_FAIL);
                break;

            default:
                break;
          }
        break;

      case Apu::FilterMode:
        switch (packet->init_filter_cmd.filter_type)
          {
            case Apu::XLOUD:
                ((MPPComponent *)p_instance)->recv_apu(p_param);
                break;

            default:
                FILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
                break;
          }
        break;

      default:
        FILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        F_ASSERT(0);
        break;
    }
}

/*--------------------------------------------------------------------*/
/* Methods of MPPComponent class */
/*--------------------------------------------------------------------*/
uint32_t MPPComponent::activate_apu(const char *path,
                                    uint32_t *dsp_inf)
{
  char filepath[64];
  FILTER_DBG("ACT MPP:\n");

  snprintf(filepath, sizeof(filepath), "%s/MPPEAX", path);

  int ret = DD_Load(filepath,
                    mpp_filter_dsp_done_callback,
                    (void *)this,
                    &m_dsp_handler,
                    DspBinTypeELFwoBind);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_Load() failure. %d\n", ret);
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  /* wait for DSP boot up... */

  dsp_boot_check(m_apu_dtq, dsp_inf);

  /* DSP version check */

  if (DSP_MPPEAX_VERSION != *dsp_inf)
    {
      logerr("DSP version unmatch. expect %08x / actual %08lx",
              DSP_MPPEAX_VERSION, *dsp_inf);

      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);
    }

  FILTER_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::deactivate_apu(void)
{
  FILTER_DBG("DEACT MPP:\n");

  int ret = DD_Unload(m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_UnLoad() failure. %d\n", ret);
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      return false;
    }

  FILTER_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);
  return true;
}

/*--------------------------------------------------------------------*/
uint32_t MPPComponent::init_apu(InitXLOUDParam *param, uint32_t* dsp_inf)
{
  FILTER_DBG("INIT MPP: ch num %d, sample num %ld, infs %ld, mode %d, "
             "bit len <in %d/out %d>, xloud coef <tbl %p/size %ld>, "
             "eax coef <tbl %p/size %ld>, sel out %p\n",
             param->ch_num, param->sample_per_frame, param->in_fs,
             param->mode, param->in_bytelength, param->out_bytelength,
             param->p_xloud_coef_image, param->xloud_coef_size,
             param->p_eax_coef_image, param->eax_coef_size,
             param->p_sel_out_param);

  m_exec_queue.clear();
  m_buf_idx = 0;

  Apu::Wien2ApuCmd *p_apu_cmd = &m_apu_cmd_buf[m_buf_idx];

  p_apu_cmd->header.core_id      = DSP_CORE_ID_DUMMY;
  p_apu_cmd->header.context_id   = DSP_MPPEAX_CONTEXT_MPPEAX;
  p_apu_cmd->header.process_mode = Apu::FilterMode;
  p_apu_cmd->header.event_type   = Apu::InitEvent;

  p_apu_cmd->init_filter_cmd.filter_type = Apu::XLOUD;

  p_apu_cmd->init_filter_cmd.channel_num = param->ch_num;
  p_apu_cmd->init_filter_cmd.sample      = param->sample_per_frame;

  p_apu_cmd->init_filter_cmd.init_xloud_param.input_sampling_rate =
    param->in_fs;

  p_apu_cmd->init_filter_cmd.init_xloud_param.mode =
    param->mode;

  p_apu_cmd->init_filter_cmd.init_xloud_param.in_pcm_bit_len =
    static_cast<AudioPcmFormat>((param->in_bytelength) << 3);

  p_apu_cmd->init_filter_cmd.init_xloud_param.out_pcm_bit_len =
    static_cast<AudioPcmFormat>((param->out_bytelength) << 3);

  p_apu_cmd->init_filter_cmd.init_xloud_param.p_coef_image =
    reinterpret_cast<uint8_t*>(param->p_xloud_coef_image);

  p_apu_cmd->init_filter_cmd.init_xloud_param.coef_size =
    param->xloud_coef_size;

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_input_sampling_rate =
    AudioFs2ApuValue[AudFs_16000];

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_mic_channel_num =
    AudioIOValidChannelNum[TwoChannels];

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_sample = 80;

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_mode =
    static_cast<Apu::AudioEaxMode>(param->mode);

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_enable_external_analysis =
    false;

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_in_pcm_bit_len =
    AudPcmFormatInt16;

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_out_pcm_bit_len =
    AudPcmFormatInt16;

  p_apu_cmd->init_filter_cmd.init_xloud_param.p_eax_coef_image =
    reinterpret_cast<uint8_t*>(param->p_eax_coef_image);

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_coef_size =
    param->eax_coef_size;

  p_apu_cmd->init_filter_cmd.init_xloud_param.p_dma_info = NULL;

  p_apu_cmd->init_filter_cmd.debug_dump_info.addr = NULL;
  p_apu_cmd->init_filter_cmd.debug_dump_info.size = 0;

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  m_buf_idx = (m_buf_idx + 1) % APU_COMMAND_QUEUE_SIZE;

  /* Wait init completion and receive reply information */

  Apu::InternalResult internal_result;
  uint32_t rst = dsp_init_check(m_apu_dtq, &internal_result);
  *dsp_inf = internal_result.value;

  return rst;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::exec_apu(ExecXLOUDParam *param)
{
  if (m_buf_idx >= APU_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  /* Filter data area check */

  if ((param->in_buffer.p_buffer == NULL)
   || (param->out_buffer.p_buffer == NULL))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Exec MFE */

  m_apu_cmd_buf[m_buf_idx].header.core_id      = DSP_CORE_ID_DUMMY;
  m_apu_cmd_buf[m_buf_idx].header.context_id   = DSP_MPPEAX_CONTEXT_MPPEAX;
  m_apu_cmd_buf[m_buf_idx].header.process_mode = Apu::FilterMode;
  m_apu_cmd_buf[m_buf_idx].header.event_type   = Apu::ExecEvent;

  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.filter_type   = Apu::XLOUD;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.input_buffer  = param->in_buffer;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.output_buffer = param->out_buffer;

  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.level = XLOUD_DEFAULT_LEVEL;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.headroom_in = XLOUD_DEFAULT_HEADROOM_IN;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.headroom_out = XLOUD_DEFAULT_HEADROOM_OUT;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.enable_mute = false;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.enable_hybrid_gain_mode =
    true;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.eax_level = XLOUD_DEFAULT_EAX_LEVEL;

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  m_buf_idx = (m_buf_idx + 1) % APU_COMMAND_QUEUE_SIZE;

  return true;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::flush_apu()
{
  if (m_buf_idx >= APU_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  m_apu_cmd_buf[m_buf_idx].header.core_id      = DSP_CORE_ID_DUMMY;
  m_apu_cmd_buf[m_buf_idx].header.context_id   = DSP_MPPEAX_CONTEXT_MPPEAX;
  m_apu_cmd_buf[m_buf_idx].header.process_mode = Apu::FilterMode;
  m_apu_cmd_buf[m_buf_idx].header.event_type   = Apu::FlushEvent;

  m_apu_cmd_buf[m_buf_idx].flush_filter_cmd.filter_type = Apu::XLOUD;

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  m_buf_idx = (m_buf_idx + 1) % APU_COMMAND_QUEUE_SIZE;

  return true;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::setparam_apu(SetXLOUDParam *param)
{
  FILTER_DBG("SET MPP: param idx %d, xloud vol %d\n",
             param->param_idx,
             param->xloud_vol);

  if (m_buf_idx >= APU_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  m_apu_cmd_buf[m_buf_idx].header.core_id      = DSP_CORE_ID_DUMMY;
  m_apu_cmd_buf[m_buf_idx].header.context_id   = DSP_MPPEAX_CONTEXT_MPPEAX;
  m_apu_cmd_buf[m_buf_idx].header.process_mode = Apu::FilterMode;
  m_apu_cmd_buf[m_buf_idx].header.event_type   = Apu::SetParamEvent;

  m_apu_cmd_buf[m_buf_idx].setparam_filter_cmd.filter_type       = Apu::XLOUD;
  m_apu_cmd_buf[m_buf_idx].setparam_filter_cmd.set_mpp.param_idx =
    param->param_idx;

  if (param->param_idx == Apu::AudXloudSetIndividual)
    {
      m_apu_cmd_buf[m_buf_idx].setparam_filter_cmd.set_mpp.xloud_vol =
        param->xloud_vol;
    }

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  m_buf_idx = (m_buf_idx + 1) % APU_COMMAND_QUEUE_SIZE;

  return true;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::tuning_apu(TuningXLOUDParam *param)
{
  FILTER_DBG("TUNING MPP: param idx %d, "
             "xloud tbl <conf %08lx/param %08lx>, "
             "eax tbl <conf %08lx/param %08lx>\n",
             param->param_idx,
             param->xloud_config_table,
             param->xloud_param_table,
             param->eax_config_table,
             param->eax_param_table);

  if (m_buf_idx >= APU_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  Apu::Wien2ApuCmd *apu_cmd = &m_apu_cmd_buf[m_buf_idx];

  apu_cmd->header.core_id      = DSP_CORE_ID_DUMMY;
  apu_cmd->header.context_id   = DSP_MPPEAX_CONTEXT_MPPEAX;
  apu_cmd->header.process_mode = Apu::FilterMode;
  apu_cmd->header.event_type   = Apu::TuningEvent;

  apu_cmd->tuning_filter_cmd.filter_type          = Apu::XLOUD;
  apu_cmd->tuning_filter_cmd.tuning_mpp.param_idx = param->param_idx;

  if (param->param_idx == Apu::AudXloudSetIndividual)
    {
      apu_cmd->tuning_filter_cmd.tuning_mpp.tuning_xloud.xloud_config_table =
        param->xloud_config_table;

      apu_cmd->tuning_filter_cmd.tuning_mpp.tuning_xloud.xloud_param_table =
        param->xloud_param_table;

      apu_cmd->tuning_filter_cmd.tuning_mpp.tuning_xloud.eax_config_table =
        param->eax_config_table;

      apu_cmd->tuning_filter_cmd.tuning_mpp.tuning_xloud.eax_param_table =
        param->eax_param_table;
    }

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  m_buf_idx = (m_buf_idx + 1) % APU_COMMAND_QUEUE_SIZE;

  return true;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::recv_apu(DspDrvComPrm_t *p_param)
{
  Apu::Wien2ApuCmd* packet =
    reinterpret_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  if (Apu::ApuExecOK != packet->result.exec_result)
    {
      FILTER_WARN(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  if (Apu::InitEvent == packet->header.event_type)
    {
      /* Notify init completion to myself */

      Apu::InternalResult internal_result = packet->result.internal_result[0];
      dsp_init_complete(m_apu_dtq,  packet->result.exec_result, &internal_result);
    }
  else
    {
      xLoudCmpltParam cmplt;

      switch (packet->header.event_type)
      {
        case Apu::InitEvent:
          cmplt.event_type = InitEvent;
          break;
    
        case Apu::ExecEvent:
          cmplt.event_type = ExecEvent;
          break;
    
        case Apu::FlushEvent:
          cmplt.event_type = StopEvent;
          break;
      }

      cmplt.filter_type = MediaPlayerPost;
      cmplt.out_buffer  = packet->exec_filter_cmd.output_buffer;
      cmplt.result      = (Apu::ApuExecOK == packet->result.exec_result) ? true : false;

      if(!m_callback(&cmplt))
        {
          FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
          return false;
        }
    }

  if (!m_exec_queue.pop())
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
void MPPComponent::send_apu(Apu::Wien2ApuCmd& cmd)
{
  F_ASSERT(!m_exec_queue.full());

  if (!m_exec_queue.push(&cmd))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return;
    }

  DspDrvComPrm_t com_param;

  com_param.process_mode  = cmd.header.process_mode;
  com_param.event_type    = cmd.header.event_type;
  com_param.type          = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam   = reinterpret_cast<void*>(&cmd);

  int ret = DD_SendCommand(m_dsp_handler, &com_param);
  
  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_SendCommand() failure. %d\n", ret);
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_SEND_ERROR);
      return;
    }
}

__WIEN2_END_NAMESPACE

