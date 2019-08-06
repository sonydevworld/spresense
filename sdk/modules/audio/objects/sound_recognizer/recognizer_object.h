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

class RecognizerObject
{
public:
  typedef struct
  {
    ComponentEventType event_type;
    bool                result;
  } RecognitionDoneCmd;

  static void create(MsgQueId, MsgQueId);

private:
  enum RecognizerState_e
  {
    StateBooted = 0,
    StateReady,
    StateActive,
    StateStopping,
    StateNum
  };

  typedef void (RecognizerObject::*ctl_func)(void);

  typedef void (RecognizerObject::*MsgProc)(MsgPacket *);
  static MsgProc MsgProcTbl[AUD_RCG_REQ_MSG_NUM][StateNum];
  static MsgProc RsltProcTbl[AUD_RCG_RES_MSG_NUM][StateNum];

  RecognizerObject(MsgQueId msg_id, MsgQueId manager_msg_id) :
    m_p_rcgproc_instance(NULL),
    m_self_msgq_id(msg_id),
    m_parent_msgq_id(manager_msg_id),
    m_state(AS_MODULE_ID_RECOGNITION_OBJ, "", StateBooted),
    m_callback(NULL),
    m_recognizer_type(AsRecognizerTypeInvalid),
    m_notify_path(AsNotifyPathCallback)
  { 
    memset(m_dsp_path, 0, sizeof(m_dsp_path));
  }

  void run();
  void parse(MsgPacket *msg);
  void illegal(MsgPacket *msg);
  void init(MsgPacket *msg);
  void exec(MsgPacket *msg);
  void illegalexec(MsgPacket *msg);
  void activate(MsgPacket *msg);
  void deactivate(MsgPacket *msg);
  void start(MsgPacket *msg);
  void stop(MsgPacket *msg);
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

  MsgQueId m_self_msgq_id;    /* Message ID of myself. */
  MsgQueId m_parent_msgq_id;
  AudioState<RecognizerState_e> m_state;
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
