/****************************************************************************
 * modules/audio/components/recognition/voice_recognition_command_component.h
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

#ifndef _VOICE_RECOGNITION_COMMAND_COMPONENT_H_
#define _VOICE_RECOGNITION_COMMAND_COMPONENT_H_

#include <stdint.h>

#ifdef __cplusplus

#include <string.h>
#include "memutils/message/Message.h"
#include "apus/apu_cmd.h"

#include "dsp_driver/include/dsp_drv.h"
#include "components/common/component_common.h"

typedef struct
{
  uint8_t vad_only;     /* Is VAD process onlu ? */
  uint8_t *p_vad_param; /* Cotrol parameter for VAD */
} InitVoiceCmdCompReqParam_t;

typedef struct
{
  uint32_t address;
  uint32_t sample_num;
} ExecVoiceCmdCompReqParam_t;

class VoiceCmdComponent : public ComponentCommon
{
public:
  VoiceCmdComponent(MsgQueId recognizer_dtq, MsgQueId dsp_dtq)
    : m_is_found(0)
  {
    m_recognizer_dtq = recognizer_dtq;
    m_dsp_dtq = dsp_dtq;
    m_dsp_handler = NULL;
  }

  ~VoiceCmdComponent(){}

  /*-structure----------------------*/
  /* message parameter */

  uint32_t act(uint32_t *dsp_inf);
  bool deact();
  int  init(InitVoiceCmdCompReqParam_t *p_param);
  int  exec(ExecVoiceCmdCompReqParam_t *p_param);
  int  flush();
  void recv_apu(DspDrvComPrm_t *p_dsp_param);
  MsgQueId get_apu_mid(void) { return m_dsp_dtq; };

private:
  /*-function----------------------*/

  void sendApu(Apu::Wien2ApuCmd* p_cmd);

  /*-variable-----------------------*/

  uint32_t m_recognition_type; /* VAD or VAD+WUWSR */
  uint32_t m_is_found;

  void *m_dsp_handler;

  MsgQueId m_recognizer_dtq;
  MsgQueId m_dsp_dtq;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  DebugLogInfo m_debug_log_info;
#endif

};
#endif

#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t AS_voiceCmdCmpActivate(MsgQueId recognizer_dtq,
                                       MsgQueId dsp_dtq,
                                       uint32_t *dsp_inf);
extern bool AS_voiceCmdCmpDeactivate(void);
extern int  AS_voiceCmdCmpInit(InitVoiceCmdCompReqParam_t *pCmdInit);
extern int  AS_voiceCmdCmpExec(ExecVoiceCmdCompReqParam_t *pCmdExec);
extern int  AS_voiceCmdCmpFlush(void);
#ifdef __cplusplus
}
#endif

#endif /* _VOICE_RECOGNITION_COMMAND_COMPONENT_H_ */
