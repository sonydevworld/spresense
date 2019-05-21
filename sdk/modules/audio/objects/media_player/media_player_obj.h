/****************************************************************************
 * modules/audio/objects/media_player/media_player_obj.h
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

#ifndef __MODULES_AUDIO_OBJECTS_MEDIA_PLAYER_MEDIA_PLAYER_OBJ_H
#define __MODULES_AUDIO_OBJECTS_MEDIA_PLAYER_MEDIA_PLAYER_OBJ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "audio/audio_player_api.h"
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/message/Message.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"
#include "wien2_common_defs.h"
#include "audio_state.h"
#include "audio/audio_message_types.h"
#include "player_input_device_handler.h"
#include "wien2_internal_packet.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MEDIA_PLAYER_FATAL(code) MEDIA_PLAYERS_FATAL(m_player_id, code)
#define MEDIA_PLAYER_ERR(code) MEDIA_PLAYERS_ERR(m_player_id, code)
#define MEDIA_PLAYER_WARN(code) MEDIA_PLAYERS_WARN(m_player_id, code)
#define MEDIA_PLAYER_INF(code) MEDIA_PLAYERS_INF(m_player_id, code)

/****************************************************************************
 * Public Types
 ****************************************************************************/

class PlayerObj
{
public:
  static void create(void **obj,
                     AsPlayerMsgQueId_t msgq_id,
                     AsPlayerPoolId_t pool_id,
                     AsPlayerId player_id);

  MsgQueId get_selfId()
    {
      return m_msgq_id.player;
    }
  MsgQueId get_apuId()
    {
      return m_msgq_id.dsp;
    }
  AsPlayerId get_playerId()
    {
      return m_player_id;
    }

private:
  PlayerObj(AsPlayerMsgQueId_t msgq_id, AsPlayerPoolId_t pool_id, AsPlayerId player_id);

  AsPlayerMsgQueId_t m_msgq_id;
  AsPlayerPoolId_t   m_pool_id;

  enum PlayerState
  {
    BootedState = 0,
    ReadyState,
    PrePlayParentState,
    PlayState,
    StoppingState,
    WaitEsEndState,
    UnderflowState,
    WaitStopState,
    PlayerStateNum
  };

  enum PlayerSubState
  {
    InvalidSubState = 0xffffffff,
    SubStatePrePlay = 0,
    SubStatePrePlayStopping,
    SubStatePrePlayWaitEsEnd,
    SubStatePrePlayUnderflow,
    SubStateNum
  };

  AudioState<PlayerState>    m_state;
  AudioState<PlayerSubState> m_sub_state;

  AsPlayerId                m_player_id;
  PlayerInputDeviceHandler *m_input_device_handler;
  InputHandlerOfRAM         m_in_ram_device_handler;
  void*                     m_p_dec_instance;

  uint32_t  m_max_es_buff_size;
  uint32_t  m_max_pcm_buff_size;
  uint32_t  m_max_src_work_buff_size;
  AudioCodec  m_codec_type;

  #define  MAX_EXEC_COUNT    2   /* Number of audio frames to be prior introduced. */
  #define  MAX_OUT_BUFF_NUM  10  /* Number of PCM buffer. */
  #define  MAX_SRC_WORK_BUFF_NUM 1 /* Number of SRC work buffer. */

  typedef s_std::Queue<MemMgrLite::MemHandle, MAX_EXEC_COUNT + 1> EsMhQueue;
  EsMhQueue m_es_buf_mh_que;

  typedef s_std::Queue<MemMgrLite::MemHandle, MAX_OUT_BUFF_NUM> PcmMhQueue;
  PcmMhQueue m_pcm_buf_mh_que;

  typedef s_std::Queue<AsPcmDataParam, MAX_OUT_BUFF_NUM + 1> DecodecPcmMhQueue;
  DecodecPcmMhQueue m_decoded_pcm_mh_que;

  typedef s_std::Queue<MemMgrLite::MemHandle, MAX_SRC_WORK_BUFF_NUM> SrcWorkMhQueue;
  SrcWorkMhQueue m_src_work_buf_mh_que;
  void *m_src_work_buf;

  s_std::Queue<AsPlayerEvent, 1> m_external_cmd_que;

  MediaPlayerCallback m_callback;

  AsPcmDataDest m_pcm_dest;
  AsPcmDataPath m_pcm_path;

  void run(void);
  void parse(MsgPacket *);
  void parseSubState(MsgPacket *);

  typedef void (PlayerObj::*MsgProc)(MsgPacket *);
  static MsgProc MsgProcTbl[AUD_PLY_MSG_NUM][PlayerStateNum];
  static MsgProc PlayerSubStateTbl[AUD_PLY_MSG_NUM][SubStateNum];
  static MsgProc PlayerResultTbl[AUD_PLY_RST_MSG_NUM][PlayerStateNum];
  static MsgProc PlayerResultSubTbl[AUD_PLY_RST_MSG_NUM][SubStateNum];

  void reply(AsPlayerEvent evtype, MsgType msg_type, uint32_t result);
  void illegalEvt(MsgPacket *);

  void activate(MsgPacket *);
  void deactivate(MsgPacket *);

  void init(MsgPacket *);

  void playOnReady(MsgPacket *);

  void stopOnPlay(MsgPacket *);
  void stopOnWait(MsgPacket *);
  void stopOnUnderflow(MsgPacket *);
  void stopOnWaitEsEnd(MsgPacket *);
  void stopOnPrePlay(MsgPacket *);
  void stopOnPrePlayUnderflow(MsgPacket *);
  void stopOnPrePlayWaitEsEnd(MsgPacket *);

  void illegalSinkDone(MsgPacket *);
  void nextReqOnPlay(MsgPacket *);
  void nextReqOnStopping(MsgPacket *);
  void nextReqOnUnderflow(MsgPacket *);
  void nextReqOnWaitEsEnd(MsgPacket *);

  void illegalDecDone(MsgPacket *);
  void decDoneOnPlay(MsgPacket *);
  void decDoneOnWaitStop(MsgPacket *);
  void decDoneOnWaitEsEnd(MsgPacket *);
  void decDoneOnPrePlay(MsgPacket *);
  void decDoneOnPrePlayStopping(MsgPacket *);
  void decDoneOnPrePlayUnderflow(MsgPacket *);
  void decDoneOnPrePlayWaitEsEnd(MsgPacket *);

  void decSetDone(MsgPacket *);

  void setGain(MsgPacket *);

  uint32_t loadCodec(AudioCodec codec,
                     AsInitPlayerParam *param,
                     uint32_t* dsp_inf);
  uint32_t unloadCodec();

  uint32_t startPlay(uint32_t* dsp_inf);
  void stopPlay(void);

  void sendPcmToOwner(AsPcmDataParam& data);

  void decode(void* p_es, uint32_t es_size);

  void *allocPcmBuf(uint32_t size);
  bool  freePcmBuf() {
  if (!m_pcm_buf_mh_que.pop())
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
      return false;
    }
    return true;
  }

  void *getEs(uint32_t* size);
  bool freeEsBuf()
    {
      if (!m_es_buf_mh_que.pop())
        {
        MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }
      return true;
    }

  void *allocSrcWorkBuf(uint32_t size);
  bool  freeSrcWorkBuf()
    {
      if (m_src_work_buf == NULL)
        {
          if (!m_src_work_buf_mh_que.pop())
            {
              MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
              return false;
            }
        }
      else
        {
          kmm_free(m_src_work_buf);
        }
      return true;
    }

  void finalize();
  bool checkAndSetMemPool();
  bool judgeMultiCore(uint32_t sampling_rate, uint8_t bit_length);
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

#endif /* __MODULES_AUDIO_OBJECTS_MEDIA_PLAYER_MEDIA_PLAYER_OBJ_H */
