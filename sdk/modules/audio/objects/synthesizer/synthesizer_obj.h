/****************************************************************************
 * modules/audio/objects/synthesizer/synthesizer_obj.h
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

#ifndef __MODULES_AUDIO_OBJECTS_SYNTHESIZER_OBJ_H
#define __MODULES_AUDIO_OBJECTS_SYNTHESIZER_OBJ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/os_utils/chateau_osal.h"
#include "memutils/message/Message.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"
#include "audio/audio_synthesizer_api.h"
#include "audio/audio_message_types.h"
#include "components/oscillator/oscillator_component.h"
#include "audio_state.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  MAX_OUT_BUFF_NUM  4  /* Number of PCM buffer. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef s_std::Queue<MemMgrLite::MemHandle, MAX_OUT_BUFF_NUM + 1> PcmMhQueue;

typedef struct
{
  uint32_t  size;
  void*     addr;
  bool      is_end;

} OscCompComplete;

typedef union
{
  OscCompComplete        comp_param;
  AsActivateSynthesizer  act_param;
  AsInitSynthesizerParam init_param;
  SetOscParam            set_param;

} SynthesizerCommand;

/****************************************************************************
 * Public Types
 ****************************************************************************/

class SynthesizerObject
{
public:
  static SynthesizerObject *create(AsSynthesizerMsgQueId_t msgq_id,
                                   AsSynthesizerPoolId_t   pool_id);

  static SynthesizerObject *get_instance(void)
  {
    static SynthesizerObject  sng_ogj;

    return &sng_ogj;
  }

  static pthread_t get_pid(void)
  {
    return get_instance()->m_pid;
  }

  static void set_pid(pthread_t id)
  {
    get_instance()->m_pid = id;
  }

  void run(void);

  err_t send(MsgType type, const SynthesizerCommand& param)
  {
    return MsgLib::send<SynthesizerCommand>(m_msgq_id.synthesizer,
                                            MsgPriNormal,
                                            type,
                                            NULL,
                                            param);
  }

private:
  SynthesizerObject(void)
    : m_state(AS_MODULE_ID_SYNTHESIZER_OBJ, "", SynthsizerStateBooted)
    , m_bit_length(AS_BITLENGTH_16)
    , m_pid(INVALID_PROCESS_ID)
    , m_stock_event(AsSynthesizerEventNum)
    , m_oscillator(0, NullPoolId)
  {
    memset(m_dsp_path, 0, AS_AUDIO_DSP_PATH_LEN);
  }

  enum SynthesizerState_e
  {
    SynthsizerStateBooted = 0,
    SynthsizerStateReady,
    SynthsizerStatePreActive,
    SynthsizerStateActive,
    SynthsizerStateStopping,
    SynthsizerStateErrorStopping,
    SynthsizerStateWaitStop,
    SynthsizerStateNum
  };

  AsSynthesizerMsgQueId_t        m_msgq_id;
  AsSynthesizerPoolId_t          m_pool_id;
  AudioState<SynthesizerState_e> m_state;
  OscllicatorComponentHandler    m_osc_hdlr;
  SynthesizerCallback            m_callback;
  void                          *m_param;
  PcmMhQueue                     m_pcm_buf_mh_que;
  uint8_t                        m_bit_length;
  uint32_t                       m_sample_size;
  uint32_t                       m_pcm_buff_size;
  char                           m_dsp_path[AS_AUDIO_DSP_PATH_LEN];
  pthread_t                      m_pid;
  AsSynthesizerEvent             m_stock_event;
  AsSynthesizerDataPath          m_data_path;
  AsSynthesizerDataDest          m_dest;

  typedef void (SynthesizerObject::*MsgProc)(MsgPacket*);

  static  MsgProc MsgProcTbl[AUD_SYN_MSG_NUM][SynthsizerStateNum];
  static  MsgProc MsgResultTbl[AUD_SYN_RST_MSG_NUM][SynthsizerStateNum];

  void reply(AsSynthesizerEvent evtype,
             MsgType            msg_type,
             uint32_t           result);

  void parse(MsgPacket *);

  void illegalEvt(MsgPacket *);
  void activate(MsgPacket *);
  void deactivate(MsgPacket *);

  void init(MsgPacket *);
  void execOnReady(MsgPacket *);
  void stopOnExec(MsgPacket *);
  void stopOnWait(MsgPacket *);
  void set(MsgPacket *);

  void illegalCompDone(MsgPacket *);
  void stopOnErrorStop(MsgPacket *);
  void cmpDoneOnErrorStop(MsgPacket *);

  void nextReqOnExec(MsgPacket *msg);
  void nextReqOnStopping(MsgPacket *msg);
  void cmpDoneOnExec(MsgPacket *msg);
  void cmpDoneOnStopping(MsgPacket *msg);
  void cmpDoneOnSet(MsgPacket *msg);

  bool checkAndSetMemPool();

  void sendPcmToOwner(MemHandle mh, bool is_end);
  uint32_t check_parameter(AsInitSynthesizerParam *p);

  /* Components */

  OscillatorComponent m_oscillator;

  bool oscillator_exec(void);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_SYNTHESIZER_OBJ_H */
