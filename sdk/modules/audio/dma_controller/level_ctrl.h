/****************************************************************************
 * modules/audio/dma_controller/level_ctrl.h
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

#ifndef LEVEL_CTRL_H
#define LEVEL_CTRL_H

#include "wien2_common_defs.h"

/*--------------------------------------------------------------------*/

class LevelCtrl
{
public:

  enum LevelCtrlCmd
  {
    CmdMuteOff = 0,
    CmdMuteOn,
    CmdStay,
    CmdNum
  };

  LevelCtrl() :
      m_state(StatusMuteOn)
    , m_auto_fade(true)
    , m_tablePtr(NULL)
  {
  };

  ~LevelCtrl(){};

  bool init(cxd56_audio_dma_t dmac_id, bool auto_fade, bool fade_enable);
  bool setFadeRamp(uint32_t* sample_per_frame);
  bool exec(LevelCtrlCmd request, bool is_wait);

  bool getAutoFade(void) { return m_auto_fade; }

private:

  enum LevelCtrlState
  {
    StatusMuteOff = 0,
    StatusFadeOut,
    StatusMuteOn,
    StatusFadeIn,
    StatusLevel,
    StatusNum
  };

  typedef void (LevelCtrl::*ActProc)(bool);
  static ActProc ActProcTbl[StatusNum][StatusNum];

  LevelCtrlState   m_state;
  cxd56_audio_dma_t m_dmac_id;
  bool m_auto_fade;

  static LevelCtrlState chgMuteState[CmdNum][StatusNum];
  static LevelCtrlState chgNoFadeMuteState[CmdNum][StatusNum];

  LevelCtrlState (*m_tablePtr)[StatusNum];

  void muteSdinVol(bool wait_flag);
  void unMuteSdinVol(bool wait_flag);
  void no_action(bool wait_flg);
};

#endif /* LEVEL_CTRL_H */

