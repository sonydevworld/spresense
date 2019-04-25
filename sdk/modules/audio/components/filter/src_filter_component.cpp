/****************************************************************************
 * modules/audio/components/filter/src_filter_component.cpp
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

#include "components/filter/src_filter_component.h"
#include "apus/cpuif_cmd.h"

#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "audio/audio_message_types.h"

#include "apus/dsp_audio_version.h"
#include "wien2_internal_packet.h"

#define DBG_MODULE DBG_MODULE_AS

__WIEN2_BEGIN_NAMESPACE

#define DSP_CORE_OF_SRC 3

/*--------------------------------------------------------------------*/
/* callback function for DSP Driver */
/*--------------------------------------------------------------------*/
void src_filter_dsp_done_callback(void *p_response, void *p_instance)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
        switch (p_param->event_type)
          {
            case Apu::BootEvent:
              {
                err_t er = MsgLib::send<uint32_t>
                        (((SRCComponent*)p_instance)->get_apu_mid(),
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
        ((SRCComponent*)p_instance)->recv_apu(p_param);
        break;

      default:
        FILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}

/*--------------------------------------------------------------------*/
/* Methods of SRCComponent class */
/*--------------------------------------------------------------------*/
uint32_t SRCComponent::activate_apu(const char *path,
                                    uint32_t *dsp_inf)
{
  char filepath[64];
  FILTER_DBG("ACT SRC:\n");

  snprintf(filepath, sizeof(filepath), "%s/SRC", path);

  int ret = DD_Load(filepath,
                    src_filter_dsp_done_callback,
                    (void *)this,
                    &m_dsp_handler);
  
  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_Load() failure. %d\n", ret);
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  /* wait for DSP boot up */

  dsp_boot_check(m_apu_dtq, dsp_inf);

  /* DSP version check */

  if (DSP_SRC_VERSION != *dsp_inf)
    {
      logerr("DSP version unmatch. expect %08x / actual %08x",
              DSP_SRC_VERSION, *dsp_inf);

      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);
    }

  FILTER_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  memset(&m_debug_log_info, 0, sizeof(m_debug_log_info));
#endif

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool SRCComponent::deactivate_apu(void)
{
  FILTER_DBG("DEACT SRC:\n");

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
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      return false;
    }

  FILTER_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t SRCComponent::init_apu(InitSRCParam *param, uint32_t *dsp_inf)
{
  Apu::Wien2ApuCmd* p_apu_cmd =
    static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  FILTER_DBG("INIT SRC: sample num %d, fs <in %d/out %d>, "
             "byte len <in %d/out %d>, ch num %d\n",
             param->sample_per_frame, param->in_fs,
             param->out_fs,
             param->in_bytelength,
             param->out_bytelength, param->ch_num);

  if (p_apu_cmd == NULL)
    {
      return AS_ECODE_FILTER_LIB_INITIALIZE_ERROR;
    }

  p_apu_cmd->header.core_id      = DSP_CORE_OF_SRC;
  p_apu_cmd->header.process_mode = Apu::FilterMode;
  p_apu_cmd->header.event_type   = Apu::InitEvent;

  p_apu_cmd->init_filter_cmd.filter_type = Apu::SRC;
  p_apu_cmd->init_filter_cmd.channel_num = param->ch_num;
  p_apu_cmd->init_filter_cmd.sample      = param->sample_per_frame;

  /* cut_off, attenuation parameter is set to recommended value of SRC library */

  p_apu_cmd->init_filter_cmd.init_src_param.input_sampling_rate  =
    param->in_fs;

  p_apu_cmd->init_filter_cmd.init_src_param.output_sampling_rate =
    param->out_fs;

  p_apu_cmd->init_filter_cmd.init_src_param.cut_off =
    SRC_CUT_OFF;

  p_apu_cmd->init_filter_cmd.init_src_param.attenuation =
    SRC_ATTENUATION;

  p_apu_cmd->init_filter_cmd.init_src_param.in_word_len =
    param->in_bytelength;

  p_apu_cmd->init_filter_cmd.init_src_param.out_word_len =
    param->out_bytelength;

  p_apu_cmd->init_filter_cmd.debug_dump_info.addr = NULL;
  p_apu_cmd->init_filter_cmd.debug_dump_info.size = 0;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  /* initialization for DSP debug dump */

  if (m_debug_log_info.addr == NULL)
    {
      strncpy(m_debug_log_info.name, "SRC", sizeof(m_debug_log_info.name));

      m_debug_log_info.addr = up_backuplog_alloc(m_debug_log_info.name,
                                                 AUDIOUTILS_DSP_DEBUG_DUMP_SIZE);
      if (m_debug_log_info.addr == NULL)
        {
          FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR);
          return AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR;
        }
    }

  if (m_debug_log_info.addr != NULL)
    {
      p_apu_cmd->init_filter_cmd.debug_dump_info.addr = m_debug_log_info.addr;
      p_apu_cmd->init_filter_cmd.debug_dump_info.size =
        AUDIOUTILS_DSP_DEBUG_DUMP_SIZE;
    }
  else
    {
      memset(m_debug_log_info.name, 0, sizeof(m_debug_log_info.name));
    }
#endif

  send_apu(p_apu_cmd);

  /* Wait init completion and receive reply information */

  Apu::InternalResult internal_result;
  uint32_t rst = dsp_init_check(m_apu_dtq, &internal_result);
  *dsp_inf = internal_result.value;

  return rst;
}

/*--------------------------------------------------------------------*/
bool SRCComponent::exec_apu(ExecSRCParam *param)
{
  Apu::Wien2ApuCmd* p_apu_cmd =
    static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
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

  /* SRC processing */

  p_apu_cmd->header.core_id      = DSP_CORE_OF_SRC;
  p_apu_cmd->header.process_mode = Apu::FilterMode;
  p_apu_cmd->header.event_type   = Apu::ExecEvent;

  p_apu_cmd->exec_filter_cmd.filter_type   = Apu::SRC;
  p_apu_cmd->exec_filter_cmd.input_buffer  = param->in_buffer;
  p_apu_cmd->exec_filter_cmd.output_buffer = param->out_buffer;

  send_apu(p_apu_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool SRCComponent::flush_apu(StopSRCParam *param)
{
  /* Regardless of output buffer is not allocated, send Flush Request
   * to DSP. Because it is needed by DSP to finish process correctly.
   */

  Apu::Wien2ApuCmd* p_apu_cmd =
    static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return false;
    }

  /* SRC processing */

  p_apu_cmd->header.core_id      = DSP_CORE_OF_SRC;
  p_apu_cmd->header.process_mode = Apu::FilterMode;
  p_apu_cmd->header.event_type   = Apu::FlushEvent;

  p_apu_cmd->flush_filter_cmd.filter_type                 = Apu::SRC;
  p_apu_cmd->flush_filter_cmd.flush_src_cmd.output_buffer =
    param->out_buffer;

  send_apu(p_apu_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool SRCComponent::recv_apu(DspDrvComPrm_t *p_param)
{
  D_ASSERT(DSP_COM_DATA_TYPE_STRUCT_ADDRESS == p_param->type);

  Apu::Wien2ApuCmd* packet =
    static_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  if (Apu::ApuExecOK != packet->result.exec_result)
    {
      FILTER_WARN(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  if (Apu::InitEvent == packet->header.event_type)
    {
      /* Notify init completion to myself */

      Apu::InternalResult internal_result = packet->result.internal_result[0];
      dsp_init_complete(m_apu_dtq, packet->result.exec_result, &internal_result);
      return true;
    }

  SrcCmpltParam cmplt;

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

  cmplt.filter_type = SampleRateConv;
  cmplt.out_buffer = packet->exec_filter_cmd.output_buffer;
  cmplt.result = (Apu::ApuExecOK == packet->result.exec_result) ? true : false;

  return m_callback(&cmplt);
}

/*--------------------------------------------------------------------*/
void SRCComponent::send_apu(Apu::Wien2ApuCmd *p_cmd)
{
  DspDrvComPrm_t com_param;

  com_param.process_mode  = p_cmd->header.process_mode;
  com_param.event_type    = p_cmd->header.event_type;
  com_param.type          = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam   = reinterpret_cast<void*>(p_cmd);

  int ret = DD_SendCommand(m_dsp_handler, &com_param);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_SendCommand() failure. %d\n", ret);
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_SEND_ERROR);
      return;
    }
}

__WIEN2_END_NAMESPACE

