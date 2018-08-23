/****************************************************************************
 * modules/audio/components/postproc/postproc_component.cpp
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

#include "postproc_component.h"

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
bool AS_postproc_recv_apu(void *p_param, void *p_instance)
{
  return ((PostprocComponent *)p_instance)->recv_apu(p_param);
}

/*--------------------------------------------------------------------*/
static void cbRcvDspRes(void *p_response, void *p_instance)
{
  /* Callback from DSP when DSP process is done. */

  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  switch (p_param->process_mode)
    {
      case PostprocCommand::CommonMode:
        {
          err_t er = MsgLib::send<uint32_t>
                    (((PostprocComponent*)p_instance)->get_apu_mid(),
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

      case PostprocCommand::FilterMode:
        AS_postproc_recv_apu(p_response, p_instance);
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
uint32_t PostprocComponent::init_apu(const InitPostprocParam& param)
{
  POSTPROC_DBG("INIT:\n");

  m_callback = param.callback;
  m_p_requester = param.p_requester;

  return 0;
}

/*--------------------------------------------------------------------*/
bool PostprocComponent::sendcmd_apu(const SendPostprocParam& param)
{
  POSTPROC_DBG("SEND PFCMD: user draw? %d, type %d size %d addr %08x\n",
                 param.is_userdraw,
                 param.cmd_type,
                 param.packet.size,
                 param.packet.addr);

  /* Send command to PostprocDSP, Contents in packet are "don't care". */

  PostprocCommand::CmdBase *p_cmd =
    static_cast<PostprocCommand::CmdBase *>(allocApuBufs(PostprocSendCmd));

  if (p_cmd == NULL)
    {
      return false;
    }

  /* Load packet data */

  memcpy(p_cmd, param.packet.addr, param.packet.size);

  /* Edit command base (hearder) */

  if (!param.is_userdraw)
    {
      p_cmd->header.cmd_type    = param.cmd_type; 
      p_cmd->result.result_code = PostprocCommand::ExecError;
    }

  /* Send to Post */

  send(p_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool PostprocComponent::exec_apu(const ExecPostprocParam& param)
{
  void *p_cmd = allocApuBufs(PostprocExec, param.input, param.output_mh);

  if (p_cmd == NULL)
    {
      return false;
    }

  /* Set fixed parameter */

  PostprocCommand::CmdBase *cmd
    = reinterpret_cast<PostprocCommand::CmdBase *>(p_cmd);

  cmd->header.cmd_type      = PostprocCommand::Exec;
  cmd->exec_cmd.input.addr  = param.input.mh.getPa();
  cmd->exec_cmd.input.size  = param.input.size;
  cmd->exec_cmd.output.addr = param.output_mh.getPa();
  cmd->exec_cmd.output.size = param.output_mh.getSize();;

  send(p_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool PostprocComponent::flush_apu(const FlushPostprocParam& param)
{
  /* The number of time to send FLUSH is depend on type of codec or filter */

  void *p_cmd = allocApuBufs(PostprocFlush, param.output_mh);

  if (p_cmd == NULL)
    {
      return false;
    }

  /* Set fixed parameter */

  PostprocCommand::CmdBase *cmd
    = reinterpret_cast<PostprocCommand::CmdBase *>(p_cmd);

  cmd->header.cmd_type       = PostprocCommand::Flush;
  cmd->flush_cmd.output.addr = param.output_mh.getPa();
  cmd->flush_cmd.output.size = param.output_mh.getSize();;

  send(p_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
void PostprocComponent::send(void *p_cmd)
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
bool PostprocComponent::recv_apu(void *p_response)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  /* Check command data type */

  if (p_param->type != DSP_COM_DATA_TYPE_STRUCT_ADDRESS)
    {
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
      return false;
    }

  /* Get packet (APU command) */

  PostprocCommand::CmdBase *packet =
    static_cast<PostprocCommand::CmdBase *>(p_param->data.pParam);

  if (PostprocCommand::ExecOk != packet->result.result_code)
    {
      POSTPROC_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  /* Notify to requester */

  PostprocCbParam cbpram;
  cbpram.event_type = static_cast<PostprocCommand::CmdType>(packet->header.cmd_type);
  cbpram.result     = static_cast<PostprocCommand::ResultCode>(packet->result.result_code);

  return m_callback(&cbpram, m_p_requester);
}

/*--------------------------------------------------------------------*/
bool PostprocComponent::recv_done(PostprocCmpltParam *cmplt)
{
  PostprocCommand::CmdBase *packet =
    static_cast<PostprocCommand::CmdBase *>(m_apu_req_mh_que.top().cmd_mh.getPa());
  
  /* Set output pcm parameters (even if is not there) */

  cmplt->output          = m_apu_req_mh_que.top().input;
  cmplt->output.mh       = m_apu_req_mh_que.top().output_mh;
  cmplt->output.size     = (packet->header.cmd_type == PostprocCommand::Exec) ?
                             packet->exec_cmd.output.size :
                             packet->flush_cmd.output.size;
  cmplt->output.is_valid = (packet->result.result_code == PostprocCommand::ExecOk) ? true : false;

  /* Set function type and result */

  cmplt->ftype = m_apu_req_mh_que.top().func_type;
  cmplt->result = (packet->result.result_code == PostprocCommand::ExecOk) ? true : false;

  return freeApuCmdBuf();
};

/*--------------------------------------------------------------------*/
uint32_t PostprocComponent::activate(uint32_t *dsp_inf)
{
  char filename[32];

  POSTPROC_DBG("ACT:\n");

  snprintf(filename, sizeof(filename), "%s/POSTPROC", CONFIG_AUDIOUTILS_DSP_MOUNTPT);

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

  int ret = DD_Load(filename, cbRcvDspRes, (void*)this, &m_dsp_handler);

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

  dsp_boot_check(m_apu_mid, dsp_inf);

  POSTPROC_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool PostprocComponent::deactivate(void)
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

