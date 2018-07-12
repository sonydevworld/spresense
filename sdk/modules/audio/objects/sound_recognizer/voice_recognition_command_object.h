/****************************************************************************
 * modules/audio/objects/sound_recognizer/voice_recognition_command_object.h
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

#ifndef __MODULES_AUDIO_OBJECTS_SOUND_RECOGNIZER_VOICE_RECOGNITION_COMMAND_OBJECT_H
#define __MODULES_AUDIO_OBJECTS_SOUND_RECOGNIZER_VOICE_RECOGNITION_COMMAND_OBJECT_H

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
#include "common/audio_internal_message_types.h"
#include "audio_state.h"

__USING_WIEN2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MFE_OUTPUT_SAMPLE_SIZE    80
#define MFE_BYTE_PER_SAMPLE_SIZE  2
#define MFE_OUTPUT_DATA_SIZE     (MFE_OUTPUT_SAMPLE_SIZE * \
                                   MFE_BYTE_PER_SAMPLE_SIZE)
#define VAD_IN_SAMPLE_SIZE        320
#define VAD_BYTE_PER_SAMPLE_SIZE  2
#define VAD_IN_DATA_SIZE          (VAD_IN_SAMPLE_SIZE * \
                                    VAD_BYTE_PER_SAMPLE_SIZE)

#define VAD_INPUT_BUF_NUM         17 /* 15 frames for reference
                                      * and 2 frames for parallel processing.
                                      */

#define VAD_FILL_CNT              4  /* A value obtained by dividing
                                      * the number 320 of samples processe
                                      * by vad by the number of notify
                                      * samples 80.
                                      */

/****************************************************************************
 * Public Types
 ****************************************************************************/

class VoiceRecognitionCommandObject
{
public:
  struct exec_param_s
  {
    uint32_t  address;
    uint32_t  size;
  };
  typedef struct exec_param_s CommandExecParam_t;

  enum event_type_e
  {
    EventCmpltInit = 0,
    EventCmpltExec,
    EventCmpltFlush,
    EventMax
  };

  struct result_param_s
  {
    int          result;
    event_type_e type;
    bool         is_vad_found;
    bool         is_wuwsr_found;
  };
  typedef struct result_param_s CommandResultParam_t;

  struct set_param_s
  {
    uint16_t key_word;
    uint8_t vad_only;
    uint8_t *p_vad_param;
  };
  typedef struct set_param_s CommandSetParam_t;

  static void create(MsgQueId, MsgQueId);

private:
  enum state_e
  {
    StateStop = 0,
    StateStart,
    StateMax
  };

  typedef void (VoiceRecognitionCommandObject::*ctl_func)(void);

  struct state_table_param_s
  {
    bool transition;
    ctl_func p_func;
  };
  typedef struct state_table_param_s state_table_param_t;

  static state_table_param_t stateTbl[StateMax][StateMax];

  VoiceRecognitionCommandObject(MsgQueId msg_id, MsgQueId manager_msg_id) :
    m_self(msg_id),
    m_manager(manager_msg_id),
    m_status(AS_MODULE_ID_RECOGNITION_OBJ, "", StateStop),
    m_key_word(0),
    m_vad_only(0),
    m_vad_previous_status(0),
    m_vad_param(NULL),
    m_vad_buf_index(0),
    m_vad_buf_pointer(NULL) {}
  void run();
  void parse(MsgPacket *msg);
  void exec(MsgPacket *msg);
  void notify(MsgPacket *msg);
  void setStatus(state_e status);
  void setCommandParam(MsgPacket *msg);
  void act();
  void deact();
  void start();
  void stop();
  void *allocVadInBuf();
  void freeVadInBuf();

  void sendAudioCmdCmplt(uint8_t command_code,
                         uint32_t result,
                         uint32_t sub_result = 0)
    {
      AudioMngCmdCmpltResult cmplt(command_code,
                                   0,
                                   result,
                                   AS_MODULE_ID_RECOGNITION_OBJ,
                                   sub_result);
      err_t er = MsgLib::send<AudioMngCmdCmpltResult>(m_manager,
                                                      MsgPriNormal,
                                                      MSG_TYPE_AUD_RES,
                                                      m_self,
                                                      cmplt);
      F_ASSERT(er == ERR_OK);
    }

  void notifyAudioCommandInfo(const AudioFindCommandInfo& info)
    {
      err_t er = MsgLib::send<AudioFindCommandInfo>(m_manager,
                                                    MsgPriNormal,
                                                    MSG_AUD_RCG_FIND_COMMAND,
                                                    m_self,
                                                    info);
      F_ASSERT(er == ERR_OK);
    }

  struct vad_in_data_s
    {
      MemMgrLite::MemHandle mh;
    };
  typedef s_std::Queue<vad_in_data_s, VAD_INPUT_BUF_NUM> VadInMhQueue;

  MsgQueId m_self;    /* Message ID of myself. */
  MsgQueId m_manager; /* Message ID of AudioManager. */
  AudioState<state_e> m_status;
  uint16_t m_key_word;
  uint8_t  m_vad_only;
  bool     m_vad_previous_status;
  uint8_t *m_vad_param;
  VadInMhQueue m_vad_in_buf_mh_que;
  uint8_t  m_vad_buf_index;
  uint8_t *m_vad_buf_pointer;
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

#endif /* __MODULES_AUDIO_OBJECTS_SOUND_RECOGNIZER_VOICE_RECOGNITION_COMMAND_OBJECT_H */
