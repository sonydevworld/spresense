/****************************************************************************
 * modules/audio/components/customproc/usercustom_component.cpp
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

#include "usercustom_component.h"

#include "memutils/common_utils/common_assert.h"
#include "audio/audio_message_types.h"
#include "memutils/message/Message.h"

#include "dsp_driver/include/dsp_drv.h"
#include <arch/chip/pm.h>
#include <sdk/debug.h>
#include "apus/dsp_audio_version.h"

__USING_WIEN2

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
static struct pm_cpu_freqlock_s g_decode_hvlock;
#endif

extern "C"
{
/*--------------------------------------------------------------------
    C Interface
  --------------------------------------------------------------------*/
bool AS_customproc_recv_dsp(void *p_param, void *p_instance)
{
  return ((UserCustomComponent *)p_instance)->recv_dsp(p_param);
}

/*--------------------------------------------------------------------*/
static void cbRcvDspRes(void *p_response, void *p_instance)
{
  /* Callback from DSP when DSP process is done. */

  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  switch (p_param->process_mode)
    {
      case CustomprocCommand::CommonMode:
        {
          err_t er = MsgLib::send<uint32_t>
                    (((UserCustomComponent*)p_instance)->get_msgq_id(),
                     MsgPriNormal,
                     MSG_ISR_APU0,
                     0,
                     p_param->data.value);

          if (er != ERR_OK)
            {
              F_ASSERT(0);
            }
        }
        break;

      case CustomprocCommand::FilterMode:
        AS_customproc_recv_dsp(p_response, p_instance);
        break;

      default:
        POSTPROC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}
} /* extern "C" */

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
uint32_t UserCustomComponent::init(const InitComponentParam& param)
{
  POSTPROC_DBG("INIT:\n");

  /* Send command to PostprocDSP, Contents in custom are "don't care". */

  CustomprocCommand::CmdBase *p_cmd = m_req_que.alloc();

  if (p_cmd == NULL)
    {
      return AS_ECODE_CHECK_MEMORY_POOL_ERROR;
    }

  /* Load custom data */

  memcpy(p_cmd, param.custom.addr, param.custom.size);

  /* Edit command base (hearder) */

  p_cmd->header.cmd_type    = CustomprocCommand::Init;
  p_cmd->result.result_code = CustomprocCommand::ExecError;

  /* Send to Post DSP */

  send(p_cmd);

  uint32_t dsp_inf;

  /* Wait for init completion, and check result */

  if (CustomprocCommand::ExecOk != dsp_init_check<uint32_t>(m_msgq_id, &dsp_inf))
    {
      return AS_ECODE_DSP_SET_ERROR;
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool UserCustomComponent::exec(const ExecComponentParam& param)
{
  void *p_cmd = m_req_que.alloc(param.input, param.output);

  if (p_cmd == NULL)
    {
      return false;
    }

  /* Set fixed parameter */

  CustomprocCommand::CmdBase *cmd
    = reinterpret_cast<CustomprocCommand::CmdBase *>(p_cmd);

  cmd->header.cmd_type      = CustomprocCommand::Exec;
  cmd->exec_cmd.input.addr  = param.input.mh.getPa();
  cmd->exec_cmd.input.size  = param.input.size;
  cmd->exec_cmd.output.addr = param.output.getPa();
  cmd->exec_cmd.output.size = param.output.getSize();

  send(p_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool UserCustomComponent::flush(const FlushComponentParam& param)
{
  /* The number of time to send FLUSH is depend on type of codec or filter */

  void *p_cmd = m_req_que.alloc(param.output);

  if (p_cmd == NULL)
    {
      return false;
    }

  /* Set fixed parameter */

  CustomprocCommand::CmdBase *cmd
    = reinterpret_cast<CustomprocCommand::CmdBase *>(p_cmd);

  cmd->header.cmd_type       = CustomprocCommand::Flush;
  cmd->flush_cmd.output.addr = param.output.getPa();
  cmd->flush_cmd.output.size = param.output.getSize();

  send(p_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool UserCustomComponent::set(const SetComponentParam& param)
{
  POSTPROC_DBG("SET:\n");

  CustomprocCommand::CmdBase *p_cmd = m_req_que.alloc();

  if (p_cmd == NULL)
    {
      return false;
    }

  /* Load custom data */

  memcpy(p_cmd, param.custom.addr, param.custom.size);

  /* Edit command base (hearder) */

  p_cmd->header.cmd_type    = CustomprocCommand::Set;
  p_cmd->result.result_code = CustomprocCommand::ExecError;

  /* Send to Post DSP */

  send(p_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
void UserCustomComponent::send(void *p_cmd)
{
  DspDrvComPrm_t com_param;

  com_param.event_type   = 0; /* Don't care */
  com_param.process_mode = 0; /* Don't care */
  com_param.type         = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam  = p_cmd;

  /* Send command to PostFilterDSP via DSP driver */

  int ret = DD_SendCommand(m_dsp_handler, &com_param);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_SendCommand() failure. %d\n", ret);
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_SEND_ERROR);
      return;
    }
}

/*--------------------------------------------------------------------*/
bool UserCustomComponent::recv_dsp(void *p_response)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  /* Check command data type */

  if (p_param->type != DSP_COM_DATA_TYPE_STRUCT_ADDRESS)
    {
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
      return false;
    }

  /* Get custom (APU command) */

  CustomprocCommand::CmdBase *custom =
    static_cast<CustomprocCommand::CmdBase *>(p_param->data.pParam);

  if (CustomprocCommand::ExecOk != custom->result.result_code)
    {
      POSTPROC_WARN(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  if (CustomprocCommand::Init == custom->header.cmd_type)
    {
      uint32_t dmy = 0;
      dsp_init_complete<uint32_t>(m_msgq_id, custom->result.result_code, &dmy);
      return true;
    }

  /* Notify to requester */

  ComponentCbParam cbpram;

  switch (custom->header.cmd_type)
    {
      case CustomprocCommand::Init:
        cbpram.event_type = ComponentInit;
        break;

      case CustomprocCommand::Exec:
        cbpram.event_type = ComponentExec;
        break;

      case CustomprocCommand::Flush:
        cbpram.event_type = ComponentFlush;
        break;

      case CustomprocCommand::Set:
        cbpram.event_type = ComponentSet;
        break;

      default:
        cbpram.event_type = ComponentInit;
        break;
    }

  cbpram.result =
    (custom->result.result_code == CustomprocCommand::ExecOk) ? true : false;

  return m_callback(&cbpram, m_p_requester);
}

/*--------------------------------------------------------------------*/
bool UserCustomComponent::recv_done(ComponentCmpltParam *cmplt)
{
  CustomprocCommand::CmdBase *custom = m_req_que.top_cmd();

  /* Set output pcm parameters (even if is not there) */

  cmplt->output          = m_req_que.top_input();
  cmplt->output.mh       = m_req_que.top_output();
  cmplt->output.size     = (custom->header.cmd_type == CustomprocCommand::Exec) ?
                             custom->exec_cmd.output.size :
                             custom->flush_cmd.output.size;
  cmplt->output.is_valid = (custom->result.result_code == CustomprocCommand::ExecOk) ? true : false;

  /* Set function type and result */

  cmplt->result = (custom->result.result_code == CustomprocCommand::ExecOk) ? true : false;

  if (!m_req_que.free())
    {
      CUSTOM_CMP_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
      return false;
    }
  return true;
}

/*--------------------------------------------------------------------*/
bool UserCustomComponent::recv_done(ComponentInformParam *info)
{
  CustomprocCommand::CmdBase *custom = m_req_que.top_cmd();

  /* Set inform parameters. */

  info->inform_req       = custom->result.inform_req;
  info->inform_data.mh   = m_req_que.top_output();
  info->inform_data.size = custom->exec_cmd.output.size;

  /* Set function type and result */

  info->result = (custom->result.result_code == CustomprocCommand::ExecOk) ? true : false;

  if (!m_req_que.free())
    {
      CUSTOM_CMP_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
      return false;
    }
  return true;
}

/*--------------------------------------------------------------------*/
uint32_t UserCustomComponent::activate(ComponentCallback callback,
                                     const char *dsp_name,
                                     void *p_requester,
                                     uint32_t *dsp_inf)
{
  POSTPROC_DBG("ACT:\n");

  m_p_requester = p_requester;
  m_callback = callback;

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
  /* Lock HV performance to avoid loading time becomes too long */

#  ifndef CONFIG_PM_DISABLE_FREQLOCK_COUNT
  g_decode_hvlock.count = 0;
#  endif
  g_decode_hvlock.info = PM_CPUFREQLOCK_TAG('D', 'C', 0x1199);
  g_decode_hvlock.flag = PM_CPUFREQLOCK_FLAG_HV;

  up_pm_acquire_freqlock(&g_decode_hvlock);
#endif

  /* Load DSP */

  int ret = DD_Load(dsp_name, cbRcvDspRes, (void*)this, &m_dsp_handler, DspBinTypeELF);

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
  up_pm_release_freqlock(&g_decode_hvlock);
#endif

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_Load() failure. %d\n", ret);
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  /* wait for DSP boot up... */

  dsp_boot_check(m_msgq_id, dsp_inf);

  POSTPROC_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool UserCustomComponent::deactivate(void)
{
  bool result = true;

  POSTPROC_DBG("DEACT:\n");

  int ret = DD_force_Unload(m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_UnLoad() failure. %d\n", ret);
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      result = false;
    }

  POSTPROC_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);

  return result;
}

