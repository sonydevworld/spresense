/****************************************************************************
 * modules/audio/components/postfilter/dsp_framework/postproc_dsp_ctrl.cpp
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

#include "postproc_dsp_ctrl.h"

/*--------------------------------------------------------------------*/
PostprocDspCtrl::CtrlProc PostprocDspCtrl::CtrlFuncTbl[PostprocCommand::CmdTypeNum] =
{
  &PostprocDspCtrl::init,
  &PostprocDspCtrl::exec,
  &PostprocDspCtrl::flush,
  &PostprocDspCtrl::set,
};

/*--------------------------------------------------------------------*/
void PostprocDspCtrl::parse(PostprocCommand::CmdBase *cmd)
{
  (this->*CtrlFuncTbl[cmd->header.cmd_type])(cmd);
}

/*--------------------------------------------------------------------*/
void PostprocDspCtrl::init(PostprocCommand::CmdBase *cmd)
{
  m_p_userproc->init(cmd);
}

/*--------------------------------------------------------------------*/
void PostprocDspCtrl::exec(PostprocCommand::CmdBase *cmd)
{
  m_p_userproc->exec(cmd);
}

/*--------------------------------------------------------------------*/
void PostprocDspCtrl::flush(PostprocCommand::CmdBase *cmd)
{
  m_p_userproc->flush(cmd);
}

/*--------------------------------------------------------------------*/
void PostprocDspCtrl::set(PostprocCommand::CmdBase *cmd)
{
  m_p_userproc->set(cmd);
}

/*--------------------------------------------------------------------*/
void PostprocDspCtrl::illegal(PostprocCommand::CmdBase *cmd)
{
  cmd->result.result_code = PostprocCommand::ExecError;
}

