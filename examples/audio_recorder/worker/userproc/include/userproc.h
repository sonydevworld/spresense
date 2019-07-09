/****************************************************************************
 * audio_recorder/worker/userproc/include/userproc.h
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

#ifndef __USERPROC_H__
#define __USERPROC_H__

#include <string.h>

#include <audio/dsp_framework/customproc_dsp_userproc_if.h>
#include "userproc_command.h"
#include "rcfilter.h"

class UserProc : public CustomprocDspUserProcIf
{
public:

  UserProc() :
    m_enable(true)
  {}

  virtual void init(CustomprocCommand::CmdBase *cmd) { init(static_cast<InitParam *>(cmd)); }
  virtual void exec(CustomprocCommand::CmdBase *cmd) { exec(static_cast<ExecParam *>(cmd)); }
  virtual void flush(CustomprocCommand::CmdBase *cmd) { flush(static_cast<FlushParam *>(cmd)); }
  virtual void set(CustomprocCommand::CmdBase *cmd) { set(static_cast<SetParam *>(cmd)); }

private:

  bool m_enable;
  RCfilter m_filter_ins;

  void init(InitParam *param);
  void exec(ExecParam *param);
  void flush(FlushParam *param);
  void set(SetParam *param);

};

#endif /* __USERPROC_H__ */

