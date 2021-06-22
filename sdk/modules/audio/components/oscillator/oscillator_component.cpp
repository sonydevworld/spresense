/****************************************************************************
 * modules/audio/components/oscillator/oscillator_component.cpp
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#include "oscillator_component.h"
#include "apus/cpuif_cmd.h"
#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "audio/audio_message_types.h"
#include "dsp_driver/include/dsp_drv.h"
#include "apus/dsp_audio_version.h"
#include "wien2_internal_packet.h"

#include "apus/apu_cmd.h"

__WIEN2_BEGIN_NAMESPACE

/* ------------------------------------------------------------------------ */
/* callback function for DSP Driver                                         */
/* ------------------------------------------------------------------------ */
void osc_dsp_done_callback(void *p_response, void *p_instance)
{
  DspDrvComPrm_t      *p_param = (DspDrvComPrm_t *)p_response;
  OscillatorComponent *p_inst  = (OscillatorComponent *)p_instance;

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
        if (p_param->event_type == Apu::BootEvent)
          {
            err_t er = MsgLib::send<uint32_t>(p_inst->get_msgq_id(),
                                              MsgPriNormal,
                                              MSG_ISR_APU0,
                                              0,
                                              p_param->data.value);
            F_ASSERT(er == ERR_OK);
          }
        else if (p_param->event_type == Apu::ErrorEvent)
          {
            OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_ASSETION_FAIL);
          }
        break;

      case Apu::OscMode:
        p_inst->recv(p_response);
        break;

      default:
        OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}

/* ------------------------------------------------------------------------ *
 * Class Methods                                                            *
 * ------------------------------------------------------------------------ */
uint32_t OscillatorComponent::activate(MsgQueId    msgq_id,
                                       PoolId      pool_id,
                                       const char *path,
                                       uint32_t   *dsp_inf)
{
  uint32_t osc_dsp_version = DSP_OSC_VERSION;

  /* Setting id */

  m_msgq_id = msgq_id;
  m_req_que.set_pool_id(pool_id);

  /* Load DSP binary */

  int ret = DD_Load(path,
                    osc_dsp_done_callback,
                    (void *)this,
                    &m_dsp_handler,
                    DspBinTypeELF);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_Load() failure. %d\n", ret);
      OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  /* wait for DSP boot up... */

  dsp_boot_check(m_msgq_id, dsp_inf);

  /* DSP version check */

  if (osc_dsp_version != *dsp_inf)
    {
      logerr("DSP version unmatch. expect %08lx / actual %08lx",
              osc_dsp_version, *dsp_inf);

      OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);
    }

  OSCILLATOR_CMP_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

  return AS_ECODE_OK;
}

/* ------------------------------------------------------------------------ */
bool OscillatorComponent::deactivate(void)
{
  OSCILLATOR_CMP_DBG("DEACT:\n");

  if (m_dsp_handler == NULL)
    {
      /* DSP is not loaded */

      logerr("DD_SendCommand() failure.\n");
      OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);

      return false;
    }

  int ret = DD_force_Unload(m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_force_Unload() failure. %d\n", ret);
      OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      return false;
    }

  m_dsp_handler = NULL;

  OSCILLATOR_CMP_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);

  return true;
}

/* ------------------------------------------------------------------------ */
uint32_t OscillatorComponent::init(const InitOscParam& param, uint32_t *dsp_inf)
{
  OSCILLATOR_CMP_DBG("INIT: WaveMode %d, channel_num %d, bit len %d, sampling_rate %ld\n",
                     param.type,
                     param.channel_num,
                     param.bit_length,
                     param.sampling_rate);

  m_callback = param.callback;
  m_instance = param.instance;


  Apu::Wien2ApuCmd* p_apu_cmd = m_req_que.alloc();
  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::OscMode;
  p_apu_cmd->header.event_type   = Apu::InitEvent;

  p_apu_cmd->init_osc_cmd = param;

  send_apu(p_apu_cmd);

  /* Wait init completion and receive reply information */

  Apu::InternalResult internal_result;
  uint32_t rst = dsp_init_check<Apu::InternalResult>(m_msgq_id, &internal_result);
  *dsp_inf = internal_result.value;

  /* Free message que */
  if (!m_req_que.free())
    {
      OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
    }

  return rst;
}

/* ------------------------------------------------------------------------ */
bool OscillatorComponent::exec(const ExecOscParam& param)
{
  /* Data check */

  if (param.buffer.p_buffer == NULL)
    {
       OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
       return false;
    }

  Apu::Wien2ApuCmd* p_apu_cmd = m_req_que.alloc();
  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::OscMode;
  p_apu_cmd->header.event_type   = Apu::ExecEvent;

  p_apu_cmd->exec_osc_cmd = param;

  return send_apu(p_apu_cmd);
}

/* ------------------------------------------------------------------------ */
bool OscillatorComponent::set(const SetOscParam& param)
{
  /* You should check date */

  Apu::Wien2ApuCmd* p_apu_cmd = m_req_que.alloc();
  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::OscMode;
  p_apu_cmd->header.event_type   = Apu::SetParamEvent;

  p_apu_cmd->setparam_osc_cmd = param;

  return send_apu(p_apu_cmd);
}

/* ------------------------------------------------------------------------ */
bool OscillatorComponent::flush()
{
  /* Regardless of output buffer is not allocated, send Flush Request
   * to DSP. Because it is needed by DSP to finish process correctly.
   */

  /* Flush */
  Apu::Wien2ApuCmd* p_apu_cmd = m_req_que.alloc();
  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::OscMode;
  p_apu_cmd->header.event_type   = Apu::FlushEvent;

  return send_apu(p_apu_cmd);
}

/* ------------------------------------------------------------------------ */
bool OscillatorComponent::recv(void *p_response)
{
  DspDrvComPrm_t   *p_param = (DspDrvComPrm_t *)p_response;
  Apu::Wien2ApuCmd *packet  = static_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);
  OscCmpltParam     comple;

  if (Apu::ApuExecOK != packet->result.exec_result)
    {
      OSCILLATOR_CMP_WARN(AS_ATTENTION_SUB_CODE_DSP_RESULT_ERROR);
    }

  switch (packet->header.event_type)
    {
      case Apu::InitEvent:

        /* Notify init completion to myself */

        dsp_init_complete<Apu::InternalResult>(m_msgq_id,
                          packet->result.exec_result,
                         &packet->result.internal_result[0]);
        return true;

      case Apu::ExecEvent:
        comple.exec_osc_param = *(ExecOscParam *)packet;
        break;

      case Apu::SetParamEvent:
        comple.set_osc_param = *(SetOscParam *)packet;
        break;

      default:
        break;
    }

  comple.event_type = packet->header.event_type;
  comple.result     = packet->result.exec_result;

  /* Free message que */
  if (!m_req_que.free())
    {
      OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
    }

  return m_callback(&comple, m_instance);
}

/* ------------------------------------------------------------------------ */
bool OscillatorComponent::send_apu(Apu::Wien2ApuCmd* p_cmd)
{
  DspDrvComPrm_t com_param;

  if (m_dsp_handler == NULL)
    {
      /* DSP is not loaded */

      logerr("DD_SendCommand() failure.\n");
      OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);

      return false;
    }

  com_param.event_type   = p_cmd->header.event_type;
  com_param.process_mode = p_cmd->header.process_mode;
  com_param.type         = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam  = reinterpret_cast<void*>(p_cmd);

  int ret = DD_SendCommand(m_dsp_handler, &com_param);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_SendCommand() failure. %d\n", ret);
      OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_SEND_ERROR);

      return false;
    }

  return true;
}

__WIEN2_END_NAMESPACE
