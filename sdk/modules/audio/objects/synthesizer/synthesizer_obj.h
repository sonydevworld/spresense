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

#include "../object_base.h"

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

class SynthesizerObject:ObjectBase
{
public:
  static void create(AsObjectParams_t*);
  static void destory() { get_instance()->m_is_created = false; }

  static SynthesizerObject *get_adr(void)
  {
    static SynthesizerObject  s_inst;
    return &s_inst;
  }

  static SynthesizerObject *get_instance(void)
  {
    SynthesizerObject* s_inst = get_adr();
    if(s_inst->m_is_created){
      return s_inst;
    }
    return 0;
  }

  static pthread_t get_pid(void)
  {
    return (get_instance() == 0) ? INVALID_PROCESS_ID : get_instance()->m_pid;
  }

  static void set_pid(pthread_t id)
  {
    if (get_instance() != 0) { get_instance()->m_pid = id; }
  }

  err_t send(MsgType type, const SynthesizerCommand& param)
  {
    return MsgLib::send<SynthesizerCommand>(m_msgq_id.self,
                                            MsgPriNormal,
                                            type,
                                            NULL,
                                            param);
  }

private:

  SynthesizerObject(void)
    : ObjectBase()
    , m_bit_length(AS_BITLENGTH_16)
    , m_stock_event(AsSynthesizerEventNum)
    , m_oscillator(NullPoolId, 0) {}

  SynthesizerObject(AsObjectMsgQueId_t msgq_id, AsObjectPoolId_t pool_id)
    : ObjectBase(AS_MODULE_ID_SYNTHESIZER_OBJ, msgq_id,pool_id)
    , m_bit_length(AS_BITLENGTH_16)
    , m_stock_event(AsSynthesizerEventNum)
    , m_oscillator(pool_id.cmp, 0)
  {
    memset(m_dsp_path, 0, AS_AUDIO_DSP_PATH_LEN);
  }


  OscllicatorComponentHandler    m_osc_hdlr;
  SynthesizerCallback            m_callback;
  void                          *m_param;
  PcmMhQueue                     m_pcm_buf_mh_que;
  uint8_t                        m_bit_length;
  uint32_t                       m_sample_size;
  uint32_t                       m_pcm_buff_size;
  char                           m_dsp_path[AS_AUDIO_DSP_PATH_LEN];
  AsSynthesizerEvent             m_stock_event;
  AsSynthesizerDataPath          m_data_path;
  AsSynthesizerDataDest          m_dest;

  typedef void (SynthesizerObject::*MsgProc)(MsgPacket*);

  static  MsgProc MsgResultTbl[AUD_SYN_RST_MSG_NUM][StateNum];

  void reply(AsSynthesizerEvent evtype,
             MsgType            msg_type,
             uint32_t           result);

  void parseResult(MsgPacket *);

  void illegal(MsgPacket *);
	
  void activateOnBooted(MsgPacket *);
  void deactivateOnReady(MsgPacket *);
  void initOnReady(MsgPacket *);
  void startOnReady(MsgPacket *);
  void stopOnActive(MsgPacket *);
  void stopOnErrorStopping(MsgPacket *);
  void stopOnWait(MsgPacket *);
  void setOnReady(MsgPacket *msg) { set(msg); }
  void setOnPreActive(MsgPacket *msg) { set(msg); }
  void setOnActive(MsgPacket *msg) { set(msg); }
  void setOnErrorStopping(MsgPacket *msg) { cmpDoneOnSet(msg); }

  void set(MsgPacket *);

  void illegalCompDone(MsgPacket *);
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
