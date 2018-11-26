/****************************************************************************
 * modules/audio/components/recognition/voice_recognition_command_component.cpp
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

/*===================================================================
 * include
 *===================================================================
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <arch/chip/backuplog.h>
#include <sdk/debug.h>
#include "apus/apu_cmd.h"
#include "objects/sound_recognizer/voice_recognition_command_object.h"
#include "voice_recognition_command_component.h"
#include "debug/dbg_log.h"
#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "audio/audio_message_types.h"
#include "apus/dsp_audio_version.h"

using namespace Wien2;
using namespace Wien2::Apu;

static VoiceCmdComponent *s_instance = NULL;

extern "C" {

/*--------------------------------------------------------------------*/
/* callback function for DSP Driver */
/*--------------------------------------------------------------------*/
void voiceCmdCmpDspDoneCallback(void *p_response, void *p_instance)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
        switch(p_param->event_type)
          {
            case Apu::BootEvent:
              {
                err_t er =
                  MsgLib::send<uint32_t>
                       (((VoiceCmdComponent*)p_instance)->get_apu_mid(),
                        MsgPriNormal,
                        MSG_ISR_APU0,
                        0,
                        p_param->data.value);

                F_ASSERT(er == ERR_OK);
              }
              break;

            case Apu::ErrorEvent:
              RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_ASSETION_FAIL);
              break;

            default:

              break;
          }
        break;

      case Apu::RecognitionMode:
        s_instance->recv_apu(p_param);
        break;

      default:
        RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}

/*--------------------------------------------------------------------*/
/*   global interface                                                 */
/*--------------------------------------------------------------------*/
uint32_t AS_voiceCmdCmpActivate(MsgQueId recognizer_dtq,
                                MsgQueId dsp_dtq,
                                uint32_t *dsp_inf)
{
  if (s_instance == NULL)
    {
      s_instance = new VoiceCmdComponent(recognizer_dtq, dsp_dtq);
    }

  return s_instance->act(dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_voiceCmdCmpDeactivate(void)
{
  if (s_instance == NULL)
    {
      RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  if (s_instance->deact())
    {
      s_instance = NULL;
    }
  else
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
int AS_voiceCmdCmpInit(InitVoiceCmdCompReqParam_t *pCmdInit)
{
  if (s_instance == NULL)
    {
      RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return -1;
    }

  return s_instance->init(pCmdInit);
}

/*--------------------------------------------------------------------*/
int AS_voiceCmdCmpExec(ExecVoiceCmdCompReqParam_t *pCmdExec)
{
  if (s_instance == NULL)
    {
      RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return -1;
    }

  return s_instance->exec(pCmdExec);
}

/*--------------------------------------------------------------------*/
int AS_voiceCmdCmpFlush(void)
{
  if (s_instance == NULL)
    {
      RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return -1;
    }

  return s_instance->flush();
}

} /* extern "C" */

/*******************************************************
    private function
*******************************************************/
static Wien2ApuCmd s_apu_init_cmd_param;
static Wien2ApuCmd s_apu_exec_cmd_param;
static Wien2ApuCmd s_apu_flush_cmd_param;

uint32_t VoiceCmdComponent::act(uint32_t *dsp_inf)
{
  RECOGNITION_CMP_DBG("ACT:\n");

  /* DSP Load */

  int ret =  DD_Load_Secure("VADWUW",
                            voiceCmdCmpDspDoneCallback,
                            (void *)this,
                            &m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_Load_Secure() failure. %d\n", ret);
      RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  /* wait for DSP boot up... */

  dsp_boot_check(m_dsp_dtq, dsp_inf);

  /* DSP version check */

  if (DSP_VADWUW_VERSION != *dsp_inf)
    {
      logerr("DSP version unmatch. expect %08x / actual %08x",
              DSP_VADWUW_VERSION, *dsp_inf);

      RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);
    }

  RECOGNITION_CMP_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  memset(&m_debug_log_info, 0, sizeof(m_debug_log_info));
#endif

  return AS_ECODE_OK;
}

bool VoiceCmdComponent::deact()
{
  RECOGNITION_CMP_DBG("DEACT:\n");

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  if (m_debug_log_info.addr)
    {
      up_backuplog_free(m_debug_log_info.name);
    }
#endif

  int ret = DD_Unload(m_dsp_handler);

  if (m_dsp_handler != NULL && ret != DSPDRV_NOERROR)
    {
      logerr("DD_UnLoad() failure. %d\n", ret);
      RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      return -1;
    }

  RECOGNITION_CMP_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);
  return 0;
}

int VoiceCmdComponent::init(InitVoiceCmdCompReqParam_t *p_param)
{
  RECOGNITION_CMP_DBG("INIT: vad only %d, vad param %08x\n",
                      p_param->vad_only, p_param->p_vad_param);

  /* Recognition type */

  m_recognition_type = (p_param->vad_only == 1) ? Vad : VadWuwsr;

  /* Set parameters */

  s_apu_init_cmd_param.header.process_mode = RecognitionMode;
  s_apu_init_cmd_param.header.event_type   = InitEvent;

  s_apu_init_cmd_param.init_recognition_cmd.recognition_type =
    (ApuRecognitionType)m_recognition_type;

  s_apu_init_cmd_param.init_recognition_cmd.sampling_rate =
    AudFs_16000;

  s_apu_init_cmd_param.init_recognition_cmd.p_vad_param =
    p_param->p_vad_param;

  s_apu_init_cmd_param.init_recognition_cmd.debug_dump_info.addr = NULL;
  s_apu_init_cmd_param.init_recognition_cmd.debug_dump_info.size = 0;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP

    if (m_debug_log_info.addr == NULL)
      {
        strncpy(m_debug_log_info.name, "VADWUW", sizeof(m_debug_log_info.name));

        m_debug_log_info.addr = up_backuplog_alloc(m_debug_log_info.name,
                                                   AUDIOUTILS_DSP_DEBUG_DUMP_SIZE);

        if (m_debug_log_info.addr == NULL)
          {
            RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR);
            return AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR;
          }
      }

    if (m_debug_log_info.addr != NULL)
      {
        s_apu_init_cmd_param.init_recognition_cmd.debug_dump_info.addr =
          m_debug_log_info.addr;

        s_apu_init_cmd_param.init_recognition_cmd.debug_dump_info.size =
          AUDIOUTILS_DSP_DEBUG_DUMP_SIZE;
      }
    else
      {
        memset(m_debug_log_info.name, 0, sizeof(m_debug_log_info.name));
      }
#endif

        /* Send INIT command */

        sendApu(&s_apu_init_cmd_param);

        return 0;
}

int VoiceCmdComponent::exec(ExecVoiceCmdCompReqParam_t *p_param)
{
  /* Set parameters */

  s_apu_exec_cmd_param.header.process_mode = RecognitionMode;
  s_apu_exec_cmd_param.header.event_type   = ExecEvent;

  s_apu_exec_cmd_param.exec_recognition_cmd.input_buffer.size =
    p_param->sample_num;

  s_apu_exec_cmd_param.exec_recognition_cmd.input_buffer.p_buffer =
    (unsigned long *)p_param->address;

  s_apu_exec_cmd_param.exec_recognition_cmd.output_buffer.size =
    0;

  s_apu_exec_cmd_param.exec_recognition_cmd.output_buffer.p_buffer =
    (unsigned long *)&m_is_found;

  /* Send recorded one frame data to DSP. */
  sendApu(&s_apu_exec_cmd_param);

  return 0;
}

int VoiceCmdComponent::flush()
{
  /* Set parameters */

  s_apu_flush_cmd_param.header.process_mode = RecognitionMode;
  s_apu_flush_cmd_param.header.event_type   = FlushEvent;

  /* Send flush command */

  sendApu(&s_apu_flush_cmd_param);

  return 0;
}

void VoiceCmdComponent::sendApu(Apu::Wien2ApuCmd *p_cmd)
{
  DspDrvComPrm_t com_param;

  com_param.process_mode = p_cmd->header.process_mode;
  com_param.event_type   = p_cmd->header.event_type;
  com_param.type         = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam  = reinterpret_cast<void*>(p_cmd);

  int ret = DD_SendCommand(m_dsp_handler, &com_param);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_SendCommand() failure. %d\n", ret);
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_SEND_ERROR);
      return;
    }
}

void VoiceCmdComponent::recv_apu(DspDrvComPrm_t *p_dsp_param)
{
  Wien2ApuCmd *p_cmd = (Wien2ApuCmd *)p_dsp_param->data.pParam;
  VoiceRecognitionCommandObject::CommandResultParam_t result_param;
  result_param.result = 0;

  if (p_cmd->result.exec_result == ApuExecError)
    {
      RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
      result_param.result = -1;
    }
  else if (p_cmd->result.exec_result == ApuWarning)
    {
      RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
      result_param.result = -1;
    }
  else
    {
      switch(p_cmd->header.event_type )
        {
          case InitEvent:
            result_param.type =
              VoiceRecognitionCommandObject::EventCmpltInit;
            break;

          case ExecEvent:
            {
              result_param.type =
                VoiceRecognitionCommandObject::EventCmpltExec;

              if (m_recognition_type == Vad)
                {
                  result_param.is_vad_found =
                    *(char *)p_cmd->exec_recognition_cmd.output_buffer.
                    p_buffer;

                  result_param.is_wuwsr_found = false;
                }
              else if (m_recognition_type == Wuwsr)
                {
                  result_param.is_vad_found   = false;
                  result_param.is_wuwsr_found =
                    *(char *)p_cmd->exec_recognition_cmd.output_buffer.
                    p_buffer;
                }
              else
                {
                  result_param.is_vad_found =
                    *(((char *)p_cmd->exec_recognition_cmd.output_buffer.
                     p_buffer) + 0);

                  result_param.is_wuwsr_found =
                    *(((char *)p_cmd->exec_recognition_cmd.output_buffer.
                     p_buffer) + 1);
                }
            }
            break;

          case FlushEvent:
            result_param.type =
              VoiceRecognitionCommandObject::EventCmpltFlush;
            break;

          default:
            RECOGNITION_CMP_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
            result_param.result = -1;
            break;
        }
    }

  err_t err = MsgLib::send<VoiceRecognitionCommandObject::CommandResultParam_t>
                                                 (m_recognizer_dtq,
                                                  MsgPriNormal,
                                                  MSG_AUD_RCG_VOICE_CMPLT,
                                                  NULL,
                                                  result_param);
  F_ASSERT(err == ERR_OK);
}

