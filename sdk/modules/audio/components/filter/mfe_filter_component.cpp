/****************************************************************************
 * modules/audio/components/filter/mfe_filter_component.cpp
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

#include "components/filter/mfe_filter_component.h"
#include "apus/cpuif_cmd.h"

#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "common/audio_internal_message_types.h"
#include "apus/apu_context_ids.h"
#include "debug/dbg_log.h"
#include "apus/dsp_audio_version.h"

#define DBG_MODULE DBG_MODULE_AS

__WIEN2_BEGIN_NAMESPACE

static MFEComponent *sp_mfe_component = NULL;
static MPPComponent *sp_mpp_component = NULL; /* MFE/SRC 1core Only */

#define DSP_CORE_ID_DUMMY 3

/*--------------------------------------------------------------------*/
/* callback function for DSP Driver */
/*--------------------------------------------------------------------*/
void mfe_filter_dsp_done_callback(void *p_response, void *p_instance)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;
  Apu::Wien2ApuCmd *packet =
    reinterpret_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  if (sp_mfe_component == NULL)
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    };

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
        switch(p_param->event_type)
          {
            case Apu::BootEvent:
              {
                err_t er = MsgLib::send<uint32_t>
                            (((MFEComponent*)p_instance)->get_apu_mid(),
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
            case Apu::MFE:
            case Apu::SRC:
              if (!sp_mfe_component->recv_apu(p_param))
                {
                  return;
                }
              break;

            case Apu::XLOUD: /* MFE/SRC 1core Only */
              if (!sp_mpp_component->recv_apu(p_param))
                {
                  return;
                }
              break;

            default:
              FILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
              break;
          }
        break;

      default:
          FILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          break;
    }
}

/*--------------------------------------------------------------------*/
/* Methods of MFEComponent class */
/*--------------------------------------------------------------------*/
uint32_t MFEComponent::activate_apu(MFEComponent *p_component,
                                    uint32_t *dsp_inf)
{
  FILTER_DBG("ACT MFE:\n");

  sp_mfe_component = p_component;

  int ret = DD_Load_Secure("MFESRC", 
                           mfe_filter_dsp_done_callback,
                           (void*)this, 
                           &m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_Load_Secure() failure. %d\n", ret);
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  if (sp_mpp_component != NULL)
    {
      sp_mpp_component->m_dsp_handler = m_dsp_handler;
    }

  /* wait for DSP boot up... */

  dsp_boot_check(m_apu_dtq, dsp_inf);

  /* DSP version check */

  if (DSP_MFESRC_VERSION != *dsp_inf)
    {
      logerr("DSP version unmatch. expect %08x / actual %08x",
              DSP_MFESRC_VERSION, *dsp_inf);

      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);
    }

  FILTER_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  memset(&m_debug_log_info, 0, sizeof(m_debug_log_info));
#endif

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
uint32_t MFEComponent::activate_apu(MFEComponent *p_mfe_component,
                                    MPPComponent *p_mpp_component,
                                    uint32_t *dsp_inf)
{
  /* MFE/SRC 1core Only */

  FILTER_DBG("ACT MFE:\n");

  sp_mpp_component = p_mpp_component;

  return activate_apu(p_mfe_component, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool MFEComponent::deactivate_apu(void)
{
  FILTER_DBG("DEACT MFE:\n");

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  if (m_debug_log_info.addr)
    {
      up_backuplog_free(m_debug_log_info.name);
    }
#endif

  int ret = DD_Unload(m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_Unload() failure. %d\n", ret);
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      return false;
    }

  FILTER_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t MFEComponent::init_apu(InitMFEParam param, uint32_t* dsp_inf)
{
  FILTER_DBG("INIT MFE: sample num %d, proc mode %d, ch num <mic %d/ref %d>,"
             " fs %d, aec <use %d/enable %d>, conf tbl %08x\n",
             param.sample_num, param.proc_mode, param.mic_channel_num,
             param.ref_channel_num, param.sampling_rate,
             param.use_aec, param.enable_mfe_aec, param.config_table);

  m_exec_queue.clear();
  m_buf_idx = 0;

  m_apu_cmd_buf[m_buf_idx].header.core_id      = DSP_CORE_ID_DUMMY;
  m_apu_cmd_buf[m_buf_idx].header.context_id   = DSP_MPPEAX_CONTEXT_MFE;
  m_apu_cmd_buf[m_buf_idx].header.process_mode = Apu::FilterMode;
  m_apu_cmd_buf[m_buf_idx].header.event_type   = Apu::InitEvent;

  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.filter_type = Apu::MFE;
  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.channel_num = param.mic_channel_num;
  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.sample      = param.sample_num;

  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.init_mfe_param.proc_mode =
    param.proc_mode;

  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.init_mfe_param.ref_channel_num =
    param.ref_channel_num;

  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.init_mfe_param.sampling_rate =
    param.sampling_rate;

  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.init_mfe_param.use_aec =
    param.use_aec;

  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.init_mfe_param.enable_mfe_aec =
    param.enable_mfe_aec;

  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.init_mfe_param.config_table =
    param.config_table;

  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.debug_dump_info.addr = NULL;
  m_apu_cmd_buf[m_buf_idx].init_filter_cmd.debug_dump_info.size = 0;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  /* initialization for DSP debug dump */

  if (m_debug_log_info.addr == NULL)
    {
      strncpy(m_debug_log_info.name, "MFESRC", sizeof(m_debug_log_info.name));

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
      m_apu_cmd_buf[m_buf_idx].init_filter_cmd.debug_dump_info.addr =
        m_debug_log_info.addr;

      m_apu_cmd_buf[m_buf_idx].init_filter_cmd.debug_dump_info.size =
        AUDIOUTILS_DSP_DEBUG_DUMP_SIZE;
    }
  else
    {
      memset(m_debug_log_info.name, 0, sizeof(m_debug_log_info.name));
    }
#endif

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  increment_buf_idx();

  uint32_t rst = dsp_init_check(m_apu_dtq, dsp_inf);

  return rst;
}

/*--------------------------------------------------------------------*/
bool MFEComponent::exec_apu(ExecMFEParam param)
{
  if (m_buf_idx >= MFE_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  /* allocate command address for APU notification MFE done */

  Apu::Wien2ApuCmd *p_apu_cmd = &m_apu_cmd_buf[m_buf_idx];
  increment_buf_idx();

  /* For cast due to limitation of column character num. */

  Apu::Wien2ApuCmd *p_cmd = &m_apu_cmd_buf[m_buf_idx];

  if (m_exec_queue.full() || !m_exec_queue.push(p_apu_cmd))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  p_cmd->header.core_id      = DSP_CORE_ID_DUMMY;
  p_cmd->header.context_id   = DSP_MPPEAX_CONTEXT_MFE;
  p_cmd->header.process_mode = Apu::FilterMode;
  p_cmd->header.event_type   = Apu::ExecEvent;

  p_cmd->exec_filter_cmd.filter_type   = Apu::MFE;
  p_cmd->exec_filter_cmd.input_buffer  = param.input_buffer;
  p_cmd->exec_filter_cmd.output_buffer = param.output_buffer;

  p_cmd->exec_filter_cmd.exec_mfe_cmd.notification.output_buffer =
    param.notification_buffer;

  p_cmd->exec_filter_cmd.exec_mfe_cmd.notification.p_apu_cmd =
    p_apu_cmd;

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  increment_buf_idx();

  return true;
}

/*--------------------------------------------------------------------*/
bool MFEComponent::flush_apu()
{
  if (m_buf_idx >= MFE_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  m_apu_cmd_buf[m_buf_idx].header.core_id = DSP_CORE_ID_DUMMY;
  m_apu_cmd_buf[m_buf_idx].header.context_id = DSP_MPPEAX_CONTEXT_MFE;
  m_apu_cmd_buf[m_buf_idx].header.process_mode = Apu::FilterMode;
  m_apu_cmd_buf[m_buf_idx].header.event_type = Apu::FlushEvent;
  m_apu_cmd_buf[m_buf_idx].flush_filter_cmd.filter_type = Apu::MFE;

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  increment_buf_idx();

  return true;
}

/*--------------------------------------------------------------------*/
bool MFEComponent::setparam_apu(Apu::ApuSetParamFilterCmd param)
{
  FILTER_DBG("SET MFE: filter type %d, param idx %d, eax handle %08x\n",
             param.filter_type, param.set_mfe.param_idx,
             param.set_mfe.p_eax_handle);

  return true;
}

/*--------------------------------------------------------------------*/
bool MFEComponent::tuning_apu(Apu::ApuTuningFilterCmd param)
{
  FILTER_DBG("TUNING MFE: filter type %d, mic delay %d, "
             "ref delay %d, conf tbl %08x\n",
             param.filter_type, param.tuning_mfe.mic_delay,
             param.tuning_mfe.ref_delay,param.tuning_mfe.mfe_config_table);

  if (m_buf_idx >= MFE_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  m_apu_cmd_buf[m_buf_idx].header.core_id      = DSP_CORE_ID_DUMMY;
  m_apu_cmd_buf[m_buf_idx].header.context_id   = DSP_MPPEAX_CONTEXT_MFE;
  m_apu_cmd_buf[m_buf_idx].header.process_mode = Apu::FilterMode;
  m_apu_cmd_buf[m_buf_idx].header.event_type   = Apu::TuningEvent;

  m_apu_cmd_buf[m_buf_idx].tuning_filter_cmd.filter_type = Apu::MFE;

  m_apu_cmd_buf[m_buf_idx].tuning_filter_cmd.tuning_mfe.mic_delay =
    param.tuning_mfe.mic_delay;

  m_apu_cmd_buf[m_buf_idx].tuning_filter_cmd.tuning_mfe.ref_delay =
    param.tuning_mfe.ref_delay;

  m_apu_cmd_buf[m_buf_idx].tuning_filter_cmd.tuning_mfe.mfe_config_table =
    param.tuning_mfe.mfe_config_table;

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  increment_buf_idx();

  return true;
}

/*--------------------------------------------------------------------*/
bool MFEComponent::recv_apu(DspDrvComPrm_t *p_param)
{
  Apu::Wien2ApuCmd *packet =
    reinterpret_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  if (Apu::ApuExecOK != packet->result.exec_result)
    {
      FILTER_WARN(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  if ((Apu::InitEvent == packet->header.event_type)
   && (Apu::MFE == packet->init_filter_cmd.filter_type))
    {
      dsp_init_complete(m_apu_dtq, packet);
    }
  else
    {
      if (!m_callback(p_param))
        {
          FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
          return false;
        }
    }

  if (!m_exec_queue.pop()) {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return false;
  }

  return true;
}

/*--------------------------------------------------------------------*/
void MFEComponent::send_apu(Apu::Wien2ApuCmd& cmd)
{
  if (m_exec_queue.full() || !m_exec_queue.push(&cmd))
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

