/****************************************************************************
 * modules/audio/manager/audio_manager.h
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

#ifndef __AUDIO_MANAGER_H
#define __AUDIO_MANAGER_H

#include <audio/audio_high_level_api.h>
#include <arch/chip/cxd56_audio.h>
#include "attention.h"
#include "wien2_common_defs.h"
#include "audio/audio_message_types.h"
#include "memutils/s_stl/queue.h"
#include "wien2_internal_packet.h"

#ifndef __ASSEMBLY__

#  undef EXTERN
#  if defined(__cplusplus)
#    define EXTERN extern "C"
extern "C"
{
#  else
#    define EXTERN extern
#  endif

int cxd56_audio_bb_register(FAR const char *devpath);
int cxd56_audio_bb_unregister(FAR const char *devpath);

#  undef EXTERN
#  if defined(__cplusplus)
}
#  endif

#endif /* __ASSEMBLY__ */

__WIEN2_BEGIN_NAMESPACE

/*--------------------------------------------------------------------*/
class AudioManager
{
public:
  static void create(MsgQueId selfDtq,
                     MsgQueId playerDtq,
                     MsgQueId subplayerDtq,
                     MsgQueId outMixerDtq,
                     AudioAttentionCb att_cb,
                     obs_AudioAttentionCb obs_att_cb);

  ~AudioManager()
  {
  };

private:
  AudioManager(MsgQueId selfDtq,
               MsgQueId playerDtq,
               MsgQueId subplayerDtq,
               MsgQueId outMixerDtq,
               AudioAttentionCb att_cb,
               obs_AudioAttentionCb obs_att_cb) :
    m_selfDtq(selfDtq),
    m_playerDtq(playerDtq),
    m_subplayerDtq(subplayerDtq),
    m_outMixerDtq(outMixerDtq),
    m_State(AS_MNG_STATUS_POWEROFF),
    m_SubState(AS_MNG_SUB_STATUS_NONE),
    m_attentionCBFunc(att_cb),
    m_obs_attentionCBFunc(obs_att_cb),
    m_active_player(0),
    m_player_transition_count(0),
    m_input_en(false),
    m_output_en(false)
#ifdef AS_FEATURE_OUTPUTMIX_ENABLE
    , m_output_device(HPOutputDevice)
#endif /* AS_FEATURE_OUTPUTMIX_ENABLE */
  {
  };

  MsgQueId m_selfDtq;
  MsgQueId m_playerDtq;
  MsgQueId m_subplayerDtq;
  MsgQueId m_outMixerDtq;

  AsMngStatus    m_State;
  AsMngSubStatus m_SubState;

  static AsVadStatus m_VadState;

  enum MngAllState
  {
      MNG_ALLSTATE_READY = 0,
      MNG_ALLSTATE_PLAYREADY,
      MNG_ALLSTATE_PLAYACTIVE,
      MNG_ALLSTATE_PLAYPAUSE,
      MNG_ALLSTATE_RECODERREADY,
      MNG_ALLSTATE_RECODERACTIVE,
      MNG_ALLSTATE_BBREADY,
      MNG_ALLSTATE_BBACTIVE,
      MNG_ALLSTATE_WAITCMDWORD,
      MNG_ALLSTATE_POWEROFF,
      MNG_ALLSTATE_THROUGH,
      MNG_ALLSTATE_NUM
  };

  AudioAttentionCb m_attentionCBFunc;
  obs_AudioAttentionCb m_obs_attentionCBFunc;
#ifdef CONFIG_AUDIOUTILS_VOICE_COMMAND
  static AudioFindCommandCallbackFunction m_findCommandCBFunc;
#endif
  uint32_t m_active_player;
  uint32_t m_command_code;
  int8_t m_player_transition_count;
  bool m_input_en;
  bool m_output_en;

#ifdef AS_FEATURE_OUTPUTMIX_ENABLE
  AsOutputMixDevice m_output_device;
#endif /* AS_FEATURE_OUTPUTMIX_ENABLE */

  typedef void (AudioManager::*MsgProc)(AudioCommand &cmd);
  typedef void (AudioManager::*RstProc)(const AudioMngCmdCmpltResult &result);
  static MsgProc MsgProcTbl[AUD_MGR_MSG_NUM][MNG_ALLSTATE_NUM];
  static RstProc RstProcTbl[1][AS_MNG_STATUS_NUM];

  void run(void);
  void parse(FAR MsgPacket *, FAR MsgQueBlock *);

  void illegal(AudioCommand &cmd);
  void ignore(AudioCommand &cmd);
  void powerOn(AudioCommand &cmd);
  void powerOff(AudioCommand &cmd);
  void soundFx(AudioCommand &cmd);
  void mfe(AudioCommand &cmd);
  void mpp(AudioCommand &cmd);
  void player(AudioCommand &cmd);
  void outputmixer(AudioCommand &cmd);
  void recorder(AudioCommand &cmd);
  void setMicMap(AudioCommand &cmd);
  void setMicGain(AudioCommand &cmd);
  void initI2SParam(AudioCommand &cmd);
  void setI2SParam(AudioCommand &cmd);
  void initDEQParam(AudioCommand &cmd);
  void setOutputSelect(AudioCommand &cmd);
  void initDNCParam(AudioCommand &cmd);
  void setClearStereo(AudioCommand &cmd);
  void setVolume(AudioCommand &cmd);
  void setVolumeMute(AudioCommand &cmd);
  void setBeep(AudioCommand &cmd);
  void setRenderingClk(AudioCommand &cmd);
  void setRdyOnAct(AudioCommand &cmd);
  void setRdyOnPlay(AudioCommand &cmd);
  void setRdyOnRecorder(AudioCommand &cmd);
  void setRdyOnThrough(AudioCommand &cmd);
  void setBaseBandStatus(AudioCommand &cmd);
  void setPlayerStatus(AudioCommand &cmd);
  void setRecorder(AudioCommand &cmd);
  void voiceCommand(AudioCommand &cmd);
  void setThroughStatus(AudioCommand &cmd);
  void setThroughPath(AudioCommand &cmd);
  void getstatus(AudioCommand &cmd);
  void setSpDrvMode(AudioCommand &cmd);

  void illegalCmplt(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnReady(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnSoundFx(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnPlayer(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnRecorder(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnPowerOff(const AudioMngCmdCmpltResult &cmd);

  int getAllState(void);

  static void execFindCommandCallback(uint16_t key_word, uint8_t status);

  void execAttentions(const ErrorAttentionParam& info);
  static void execAttentionsCallback(FAR void *);

  uint32_t powerOnBaseBand(uint8_t power_id);
  uint32_t powerOffBaseBand(uint8_t power_id);
  
  void sendResult(uint8_t code, uint8_t sub_code = 0);
  void sendErrRespResult(uint8_t  sub_code,
                         uint8_t  module_id,
                         uint32_t error_code,
                         uint32_t error_sub_code = 0);
  bool deactivatePlayer();
  bool deactivateRecorder();
  bool deactivateSoundFx();
  bool deactivateOutputMix();

  uint32_t setPlayerStatusParamCheck(uint8_t  input_dev,
                                     uint8_t  output_dev,
                                     FAR void *input_handler);
  bool packetCheck(uint8_t length, uint8_t command_code, AudioCommand &cmd);
};

__WIEN2_END_NAMESPACE

#endif /* __AUDIO_MANAGER_H */
