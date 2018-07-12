/****************************************************************************
 * modules/audio/components/postfilter/postfilter_component.cpp
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

#include "postfilter_component.h"

#include "memutils/common_utils/common_assert.h"
#include "common/audio_internal_message_types.h"
#include "memutils/message/Message.h"

#include "dsp_driver/include/dsp_drv.h"
#include <arch/chip/pm.h>
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
bool AS_postfilter_recv_apu(void *p_param, void *p_instance)
{
  return ((PostfilterComponent *)p_instance)->recv_apu(p_param);
}

/*--------------------------------------------------------------------*/
static void cbRcvDspRes(void *p_response, void *p_instance)
{
  /* Callback from DSP when DSP process is done. */

  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
        if (p_param->event_type == Apu::BootEvent)
          {
            err_t er = MsgLib::send<uint32_t>
                      (((PostfilterComponent*)p_instance)->get_apu_mid(),
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
            POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_ASSETION_FAIL);
          }
        break;

      case Apu::FilterMode:
        AS_postfilter_recv_apu(p_response, p_instance);
        break;

      default:
        POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}
} /* extern "C" */

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
uint32_t PostfilterComponent::init_apu(const InitPostfilterParam& param,
                                       uint32_t *dsp_inf)
{
  POSTFILTER_DBG("INIT: ch num %d, bit len %d, "
              "sample num %d, cb %08x, req %08x\n",
               param.channel_num, param.bit_width, param.sample_num,
               param.callback, param.p_requester);

  m_callback = param.callback;
  m_p_requester = param.p_requester;

  Wien2::Apu::Wien2ApuCmd *p_cmd =
    reinterpret_cast<Wien2::Apu::Wien2ApuCmd*>(allocApuBufs());

  if (p_cmd == NULL)
    {
      return AS_ECODE_DECODER_LIB_INITIALIZE_ERROR;
    }

  memset(p_cmd, 0x00, sizeof(Wien2::Apu::Wien2ApuCmd));

  p_cmd->header.process_mode = Wien2::Apu::FilterMode;
  p_cmd->header.event_type   = Wien2::Apu::InitEvent;

  /* Note: CXD5602 supports only stereo format. */

  p_cmd->init_postfilter_cmd.ch_num    = param.channel_num;
  p_cmd->init_postfilter_cmd.bit_width = static_cast<Wien2::AudioPcmBitWidth>(param.bit_width);
  p_cmd->init_postfilter_cmd.sample    = param.sample_num;

  send_apu(p_cmd);

  uint32_t rst = dsp_init_check(m_apu_mid, dsp_inf);

  return rst;
}

/*--------------------------------------------------------------------*/
bool PostfilterComponent::exec_apu(const ExecPostfilterParam& param)
{
  Wien2::Apu::Wien2ApuCmd *p_cmd =
    reinterpret_cast<Wien2::Apu::Wien2ApuCmd*>(allocApuBufs(param.input,
                                                            param.output_mh));

  if (p_cmd == NULL)
    {
      return false;
    }

  memset(p_cmd, 0x00, sizeof(Wien2::Apu::Wien2ApuCmd));

  p_cmd->header.process_mode = Wien2::Apu::FilterMode;
  p_cmd->header.event_type   = Wien2::Apu::ExecEvent;

  p_cmd->exec_postfilter_cmd.input_buffer.size      = param.input.size;
  p_cmd->exec_postfilter_cmd.input_buffer.p_buffer  = reinterpret_cast<unsigned long*>(param.input.mh.getPa());
  p_cmd->exec_postfilter_cmd.output_buffer.size     = param.output_mh.getSize();;
  p_cmd->exec_postfilter_cmd.output_buffer.p_buffer = reinterpret_cast<unsigned long*>(param.output_mh.getPa());

  send_apu(p_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool PostfilterComponent::flush_apu(const FlushPostfilterParam& param)
{
  /* The number of time to send FLUSH is depend on type of codec or filter */

  Wien2::Apu::Wien2ApuCmd *p_cmd =
    reinterpret_cast<Wien2::Apu::Wien2ApuCmd*>(allocApuBufs(param.output_mh));

  if (p_cmd == NULL)
    {
      return false;
    }

  memset(p_cmd, 0x00, sizeof(Wien2::Apu::Wien2ApuCmd));

  p_cmd->header.process_mode = Wien2::Apu::FilterMode;
  p_cmd->header.event_type   = Wien2::Apu::FlushEvent;

  p_cmd->exec_postfilter_cmd.output_buffer.size     = param.output_mh.getSize();;
  p_cmd->exec_postfilter_cmd.output_buffer.p_buffer = reinterpret_cast<unsigned long*>(param.output_mh.getPa());

  send_apu(p_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
void PostfilterComponent::send_apu(Wien2::Apu::Wien2ApuCmd *p_cmd)
{
  DspDrvComPrm_t com_param;

  com_param.event_type   = p_cmd->header.event_type;
  com_param.process_mode = p_cmd->header.process_mode;
  com_param.type         = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam  = reinterpret_cast<void*>(p_cmd);

  /* Send command to PostFileterDSP via DSP driver */

  int ret = DD_SendCommand(m_dsp_handler, &com_param);

  if (ret != DSPDRV_NOERROR)
    {
      _err("DD_SendCommand() failure. %d\n", ret);
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_SEND_ERROR);
      return;
    }
}

/*--------------------------------------------------------------------*/
bool PostfilterComponent::recv_apu(void *p_response)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  /* Check command data type */

  if (p_param->type != DSP_COM_DATA_TYPE_STRUCT_ADDRESS)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
      return false;
    }

  Wien2::Apu::Wien2ApuCmd *packet =
    static_cast<Wien2::Apu::Wien2ApuCmd *>(p_param->data.pParam);

  if (Apu::ApuExecOK != packet->result.exec_result)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  /* Send self-sync message when InitEvent */

  if (Apu::InitEvent == packet->header.event_type)
    {
      dsp_init_complete(m_apu_mid, packet);
      return true;
    }

  /* Notify to requester */

  PostfilterCbParam cbpram;
  cbpram.event_type = static_cast<Wien2::Apu::ApuEventType>(packet->header.event_type);

  return m_callback(&cbpram, m_p_requester);
}

/*--------------------------------------------------------------------*/
bool PostfilterComponent::recv_done(PostfilterCmpltParam *cmplt)
{
  Wien2::Apu::Wien2ApuCmd *packet =
    static_cast<Wien2::Apu::Wien2ApuCmd *>(m_apu_req_mh_que.top().cmd_mh.getPa());

  cmplt->output          = m_apu_req_mh_que.top().input;
  cmplt->output.mh       = m_apu_req_mh_que.top().output_mh;
  cmplt->output.sample   = (packet->header.event_type == Apu::ExecEvent) ?
                             packet->exec_postfilter_cmd.output_buffer.size :
                             packet->flush_postfilter_cmd.output_buffer.size;
  cmplt->output.is_valid = (packet->result.exec_result == Wien2::Apu::ApuExecOK) ? true : false;

  cmplt->result = packet->result.exec_result;

  return freeApuCmdBuf();
};

/*--------------------------------------------------------------------*/
uint32_t PostfilterComponent::activate(uint32_t *dsp_inf)
{
  char filename[32];
  uint32_t postfilter_dsp_version;

  POSTFILTER_DBG("ACT:\n");

  snprintf(filename, sizeof(filename), "%s/POSTFILTER", CONFIG_AUDIOUTILS_DSP_MOUNTPT);
  postfilter_dsp_version = DSP_POSTFLTR_VERSION;

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
      _err("DD_Load() failure. %d\n", ret);
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  if (!dsp_boot_check(m_apu_mid, postfilter_dsp_version, dsp_inf))
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);

      /* PostFilter DSP(worker) isn't be "EXIT" state is ASMP framework.
       * And cannot unload normaly. Therefore, unload it force here.
       */

      ret = DD_force_Unload(m_dsp_handler);

      if (ret != DSPDRV_NOERROR)
        {
          _err("DD_UnLoad() failure. %d\n", ret);
          POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
        }

      return AS_ECODE_DSP_VERSION_ERROR;
    }

  POSTFILTER_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool PostfilterComponent::deactivate(void)
{
  bool result = true;

  POSTFILTER_DBG("DEACT:\n");

  int ret = DD_force_Unload(m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      _err("DD_UnLoad() failure. %d\n", ret);
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      result = false;
    }

  POSTFILTER_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);

  return result;
}

