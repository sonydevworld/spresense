/****************************************************************************
 * modules/include/audio/dsp_framework/customproc_dsp_ctrl.h
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

#ifndef __CUSTOMPROC_DSP_CTRL_H__
#define __CUSTOMPROC_DSP_CTRL_H__

#include <string.h>

#include <audio/dsp_framework/customproc_command_base.h>
#include <audio/dsp_framework/customproc_dsp_userproc_if.h>

class CustomprocDspCtrl
{
public:
  void parse(CustomprocCommand::CmdBase *cmd);

  CustomprocDspCtrl(CustomprocDspUserProcIf *p_userproc_ins)
    : m_p_userproc(p_userproc_ins)
    , m_state(Booted)
  {}

private:

  typedef enum
  {
    Booted = 0,
    Ready,
    Active,

    DspStateNum
  } DspState;

  CustomprocDspUserProcIf *m_p_userproc;
  DspState m_state;

  typedef void (CustomprocDspCtrl::*CtrlProc)(CustomprocCommand::CmdBase *cmd);
  static CtrlProc CtrlFuncTbl[CustomprocCommand::CmdTypeNum][DspStateNum];

  void init(CustomprocCommand::CmdBase *cmd);
  void exec(CustomprocCommand::CmdBase *cmd);
  void flush(CustomprocCommand::CmdBase *cmd);
  void set(CustomprocCommand::CmdBase *cmd);
  void illegal(CustomprocCommand::CmdBase *cmd);
};

#endif /* __USERCUSTOM_DSP_CTRL_H__ */

