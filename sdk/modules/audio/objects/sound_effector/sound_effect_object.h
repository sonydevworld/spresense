/****************************************************************************
 * modules/audio/objects/sound_effector/sound_effect_object.h
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
 
#ifndef __MODULES_AUDIO_OBJECTS_SOUND_EFFECTOR_SOUND_EFFECT_OBJECT_H
#define __MODULES_AUDIO_OBJECTS_SOUND_EFFECTOR_SOUND_EFFECT_OBJECT_H

#include "wien2_common_defs.h"

#include <audio/audio_high_level_api.h>
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/s_stl/queue.h"
#include "audio_state.h"

#include "audio/audio_message_types.h"
#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"

#include "components/capture/capture_component.h"
#include "components/filter/filter_api.h"
#include "components/renderer/renderer_component.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

#define CAPTURE_PRESET_NUM (3)
#define CAP_BUF_QUEUE_SIZE  (CAPTURE_PRESET_NUM+2)

#define MAX_MIC_IN_CH_NUM   (1)

#define INPUT_DEVICE_MASK_AMIC_CH  (0x0F00)

#define FILTER_MODE_THROUGH (0x00)
#define FILTER_MODE_MFE     (0x01)
#define FILTER_MODE_MPPEAX  (0x02)

class SoundEffectObject
{
public:
  static void create(MsgQueId self_dtq,
                     MsgQueId manager_dtq,
                     MsgQueId voice_recognition_dtq);

private:
  SoundEffectObject(MsgQueId self_dtq,
                    MsgQueId manager_dtq,
                    MsgQueId voice_recognition_dtq)
    : m_self_dtq(self_dtq)
    , m_manager_dtq(manager_dtq)
    , m_voice_recognition_dtq(voice_recognition_dtq)
    , m_state(AS_MODULE_ID_SOUND_EFFECT_OBJ, "", SoundFXBootedState)
    , m_filter_mode(FILTER_MODE_THROUGH)
    , m_mic_in_ch_num(MAX_MIC_IN_CH_NUM)
    , m_i2s_in_ch_num(MAX_I2S_IN_CH_NUM)
    , m_sp_hp_out_ch_num(MAX_I2S_IN_CH_NUM)
    , m_i2s_out_ch_num(2)
    , m_select_output_mic(AS_SELECT_MIC1_OR_MIC2)
    , m_mic_in_sync_cnt(0)
    , m_i2s_in_sync_cnt(0)
    , m_capt_sync_wait_flg(true)
    , m_mfe_instance(NULL)
    , m_mpp_instance(NULL)
    {}

  enum SoundEffectState
  {
    SoundFXBootedState,
    SoundFXReadyState,
    SoundFXRunState,
    SoundFXStoppingState,
    SoundFXStateNum
  };

  MsgQueId m_self_dtq;
  MsgQueId m_manager_dtq;
  MsgQueId m_voice_recognition_dtq;

  AudioState<SoundEffectState> m_state;
  uint8_t m_filter_mode;

  uint8_t m_mic_in_ch_num;
  uint8_t m_i2s_in_ch_num;
  uint8_t m_sp_hp_out_ch_num;
  uint8_t m_i2s_out_ch_num;

  /* TODO: Should be removed. Should initialize component
   *       when initXXX command has been issued.
   */
  InitMFEParam m_init_mfe_param;

  /* TODO: Should be removed. Should initialize component
   *       when initXXX command has been issued.
   */
  InitXLOUDParam m_init_xloud_param;

  AsSelectOutputMic m_select_output_mic;

  AsI2sOutputData m_I2S_output_data;

  uint8_t m_mic_in_sync_cnt;
  uint8_t m_i2s_in_sync_cnt;

  bool m_capt_sync_wait_flg;

  struct SoundFxBufParam
  {
    MemMgrLite::MemHandle mh;
    uint32_t sample;
    bool is_end;
  };

  /* TODO: Check validity of queue num. */
  typedef s_std::Queue<SoundFxBufParam, CAP_BUF_QUEUE_SIZE> InDataMhQueue;
  typedef s_std::Queue<SoundFxBufParam, CAP_BUF_QUEUE_SIZE> OutDataMhQueue;
  typedef s_std::Queue<SoundFxBufParam, CAP_BUF_QUEUE_SIZE> RcgDataMhQueue;

  InDataMhQueue  m_mfe_in_buf_mh_que;
  InDataMhQueue  m_mpp_in_buf_mh_que;
  OutDataMhQueue m_hp_out_buf_mh_que;
  OutDataMhQueue m_i2s_out_buf_mh_que;
  RcgDataMhQueue m_mfe_out_buf_mh_que;

  s_std::Queue<AudioCommand, 1> m_external_cmd_que;

  RenderComponentHandler m_i2s_render_comp_handler, m_hp_render_comp_handler;
  CaptureComponentHandler m_capture_from_mic_hdlr, m_capture_from_i2s_hdlr;

  FilterComponent *m_mfe_instance;
  FilterComponent *m_mpp_instance;

  typedef void (SoundEffectObject::*MsgProc)(MsgPacket*);
  static  MsgProc MsgProcTbl[AUD_SEF_MSG_NUM][SoundFXStateNum];

  void run();
  void parse(MsgPacket* msg);

  void illegal(MsgPacket* msg);
  void init(MsgPacket* msg);
  void act(MsgPacket* msg);
  void deact(MsgPacket* msg);
  void startOnReady(MsgPacket* msg);
  void stopOnActive(MsgPacket* msg);
  void illegalInput(MsgPacket* msg);
  void inputOnActive(MsgPacket* msg);
  void inputOnStopping(MsgPacket* msg);
  void filterRstOnActive(MsgPacket* msg);
  void filterRstOnStopping(MsgPacket* msg);
  void dmaOutDoneCmpltOnActive(MsgPacket* msg);
  void dmaOutDoneCmpltOnStopping(MsgPacket* msg);
  void setParam(MsgPacket* msg);
  void filterDoneCmplt(MsgPacket* msg);

  void input(CaptureDataParam& param);

  uint32_t initMfe(const AudioCommand& cmd);
  uint32_t initMpp(const AudioCommand& cmd);

  void execI2SOutRender(FilterCompCmpltParam *param);
  void execHpSpOutRender(FilterCompCmpltParam *param);
  void execMfe(MemMgrLite::MemHandle mh, int32_t sample, bool is_end);
  void execMpp(MemMgrLite::MemHandle mh, int32_t sample, bool is_end);

  void* allocHpOutBuf();
  void* allocI2SOutBuf();
  void* allocMfeOutBuf();

  bool freeMfeInBuf()
  {
    if (!m_mfe_in_buf_mh_que.pop())
      {
        SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }
    return true;
  }

  bool freeMppInBuf()
  {
    if (!m_mpp_in_buf_mh_que.pop())
      {
        SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }
    return true;
  }

  bool freeHpOutBuf()
  {
    if (!m_hp_out_buf_mh_que.pop())
      {
        SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }
    return true;
  }

  bool freeI2SOutBuf()
  {
    if (!m_i2s_out_buf_mh_que.pop())
      {
        SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }
    return true;
  }

  bool freeMfeOutBuf()
  {
    if (!m_mfe_out_buf_mh_que.pop())
      {
        SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }
    return true;
  }

  void freeOutBuf(cxd56_audio_dma_t dmac_select_id);

  void sendAudioCmdCmplt(const AudioCommand& cmd,
                         uint32_t result,
                         uint32_t sub_result = 0)
  {
    AudioMngCmdCmpltResult cmplt(cmd.header.command_code,
                                 cmd.header.sub_code,
                                 result,
                                 AS_MODULE_ID_SOUND_EFFECT_OBJ, sub_result);

    err_t er = MsgLib::send<AudioMngCmdCmpltResult>(m_manager_dtq,
                                                    MsgPriNormal,
                                                    MSG_TYPE_AUD_RES,
                                                    m_self_dtq, cmplt);

    F_ASSERT(ERR_OK == er);
  }

  void selectCh4to2(uint16_t *p_src, uint16_t *p_dst, uint32_t sample_num);
  void convertCh1to2(uint16_t *p_src, uint16_t *p_dst, uint32_t sample_num);
};

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_SOUND_EFFECTOR_SOUND_EFFECT_OBJECT_H */
