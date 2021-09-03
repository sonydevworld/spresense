/****************************************************************************
 * modules/audio/objects/sound_recognizer/recognizer_object.h
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

#ifndef __MODULES_AUDIO_OBJECTS_SOUND_RECOGNIZER_RECOGNIZER_OBJECT_H
#define __MODULES_AUDIO_OBJECTS_SOUND_RECOGNIZER_RECOGNIZER_OBJECT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <string.h>
#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/s_stl/queue.h"
#include "audio/audio_high_level_api.h"
#include "wien2_internal_packet.h"
#include "audio/audio_message_types.h"
#include "audio_state.h"

#include "../object_base.h"

//#include "components/recognition/voice_recognition_command_component.h"
#include "components/customproc/usercustom_component.h"

__USING_WIEN2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VAD_IN_SAMPLE_SIZE        320
#define VAD_BYTE_PER_SAMPLE_SIZE  2
#define VAD_IN_DATA_SIZE          (VAD_IN_SAMPLE_SIZE * \
                                    VAD_BYTE_PER_SAMPLE_SIZE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

class RecognizerObject:ObjectBase
{
public:

  typedef struct
  {
    ComponentEventType event_type;
    bool                result;
  } RecognitionDoneCmd;

  static void create(AsObjectParams_t*);
  static void destory() { get_instance()->m_is_created = false; }

  static RecognizerObject *get_adr(void)
  {
    static RecognizerObject s_inst;
    return &s_inst;
  }

  static RecognizerObject *get_instance(void)
  {
    RecognizerObject* s_inst = get_adr();
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

  static pthread_t get_msgq_id(void)
  {
    return (get_instance() == 0) ? 0 : get_instance()->m_msgq_id.self;
  }

private:

  typedef void (RecognizerObject::*ctl_func)(void);

  typedef void (RecognizerObject::*MsgProc)(MsgPacket *);
  static MsgProc MsgParamTbl[AUD_RCG_PRM_NUM][StateNum];
  static MsgProc RsltProcTbl[AUD_RCG_RES_MSG_NUM][StateNum];

  RecognizerObject(void)
    : ObjectBase()
    , m_callback(NULL)
    , m_recognizer_type(AsRecognizerTypeInvalid)
    , m_notify_path(AsNotifyPathCallback) {}

  RecognizerObject(AsObjectMsgQueId_t msgq_id, AsObjectPoolId_t pool_id)
    : ObjectBase(AS_MODULE_ID_RECOGNITION_OBJ, msgq_id,pool_id)
    , m_callback(NULL)
    , m_recognizer_type(AsRecognizerTypeInvalid)
    , m_notify_path(AsNotifyPathCallback)
  {
    memset(m_dsp_path, 0, sizeof(m_dsp_path));
  }

  void parseResult(MsgPacket *msg);

  void illegal(MsgPacket *msg);
  void ignore(MsgPacket *msg);

  void activateOnBooted(MsgPacket *msg);
  void deactivateOnReady(MsgPacket *msg);
  void initOnReady(MsgPacket *msg);
  void startOnReady(MsgPacket *msg);
  void stopOnActive(MsgPacket *msg);
  void execOnActive(MsgPacket *msg);
  void setOnReady(MsgPacket *msg) { set(msg); }
  void setOnActive(MsgPacket *msg) { set(msg); }
  void setOnErrorStoppping(MsgPacket *msg) { set(msg); }
  void set(MsgPacket *msg);

  void execOnReady(MsgPacket *msg) { ignore(msg); }
  void execOnStopping(MsgPacket *msg) { ignore(msg); }
  void execOnErrorStopping(MsgPacket *msg) { ignore(msg); }

  void initRcgproc(MsgPacket *msg);
  void setRcgproc(MsgPacket *msg);

  void recognizeDoneOnReady(MsgPacket *msg);
  void recognizeDoneOnActive(MsgPacket *msg);
  void recognizeDoneOnStopping(MsgPacket *msg);
  void illegalRecognizeDone(MsgPacket *msg);

  uint32_t loadComponent(AsRecognizerType type, char *dsp_path);
  uint32_t unloadComponent(void);

  bool notify(AsRecognitionInfo info);
  void reply(AsRecognizerEvent event, uint32_t command_id, uint32_t result);

  ComponentBase *m_p_rcgproc_instance;

  RecognizerCallback m_callback;
  AsRecognizerType m_recognizer_type;
  char m_dsp_path[AS_RECOGNIZER_FILE_PATH_LEN];
  uint8_t m_notify_path;
  AsNotifyDest m_notify_dest;
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

#endif /* __MODULES_AUDIO_OBJECTS_SOUND_RECOGNIZER_RECOGNIZER_OBJECT_H */
