/****************************************************************************
 * modules/audio/dsp/worker/postfilter_ctrl.cpp
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

#include "postfilter_ctrl.h"

/*--------------------------------------------------------------------*/
PostFilterCtrl::CtrlProc PostFilterCtrl::CtrlFuncTbl[Wien2::Apu::ApuEventTypeNum][StateNum] =
{
  /* boot */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::illegal,  /* Ready  */
    &PostFilterCtrl::illegal   /* Exec   */
  },

  /* init */
  {
    &PostFilterCtrl::init,     /* Booted */
    &PostFilterCtrl::init,     /* Ready  */
    &PostFilterCtrl::illegal   /* Exec   */
  },

  /* exec */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::exec,     /* Ready  */
    &PostFilterCtrl::exec      /* Exec   */
  },

  /* flush */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::illegal,  /* Ready  */
    &PostFilterCtrl::flush     /* Exec   */
  },

  /* setParam */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::setparam, /* Ready  */
    &PostFilterCtrl::setparam  /* Exec   */
  },

  /* tuning */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::illegal,  /* Ready  */
    &PostFilterCtrl::illegal   /* Exec   */
  },

  /* error */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::illegal,  /* Ready  */
    &PostFilterCtrl::illegal   /* Exec   */
  },
};

/*--------------------------------------------------------------------*/
void PostFilterCtrl::parse(Wien2::Apu::Wien2ApuCmd *cmd)
{
  (this->*CtrlFuncTbl[cmd->header.event_type][m_state])(cmd);
}

/*--------------------------------------------------------------------*/
void PostFilterCtrl::init(Wien2::Apu::Wien2ApuCmd *cmd)
{
  Wien2::Apu::ApuInitPostFilterCmd *init_cmd = &cmd->init_postfilter_cmd;

  (void)init_cmd;
  cmd->result.exec_result = Wien2::Apu::ApuExecOK;

  m_state = ReadyStatus;
}

/*--------------------------------------------------------------------*/
void PostFilterCtrl::exec(Wien2::Apu::Wien2ApuCmd *cmd)
{
  Wien2::Apu::ApuExecPostFilterCmd *exec_cmd = &cmd->exec_postfilter_cmd;

  /* !!tentative!! simply copy from input to output */

  memcpy(exec_cmd->output_buffer.p_buffer,
         exec_cmd->input_buffer.p_buffer,
         exec_cmd->input_buffer.size);

  //{
  //  /* RC filter example */

  //  int16_t *ls = (int16_t*)exec_cmd->output_buffer.p_buffer;
  //  int16_t *rs = ls + 1;

  //  static int16_t ls_l = 0;
  //  static int16_t rs_l = 0;

  //  if (!ls_l && !rs_l)
  //    {
  //      ls_l = *ls;
  //      rs_l = *rs;
  //    }

  //  for (uint32_t cnt = 0; cnt < exec_cmd->input_buffer.size; cnt += 4)
  //    {
  //      *ls = (ls_l * 99 / 100) + (*ls * 1 / 100);
  //      *rs = (rs_l * 99 / 100) + (*rs * 1 / 100);

  //      ls_l = *ls;
  //      rs_l = *rs;

  //      ls += 2;
  //      rs += 2;
  //    }
  //}

  cmd->result.exec_result = Wien2::Apu::ApuExecOK;

  m_state = ExecStatus;
}

/*--------------------------------------------------------------------*/
void PostFilterCtrl::flush(Wien2::Apu::Wien2ApuCmd *cmd)
{
  Wien2::Apu::ApuFlushPostFilterCmd *flush_cmd = &cmd->flush_postfilter_cmd;

  (void)flush_cmd;
  cmd->result.exec_result = Wien2::Apu::ApuExecOK;

  m_state = ReadyStatus;
}

/*--------------------------------------------------------------------*/
void PostFilterCtrl::setparam(Wien2::Apu::Wien2ApuCmd *cmd)
{
  cmd->result.exec_result = Wien2::Apu::ApuExecOK;
}

/*--------------------------------------------------------------------*/
void PostFilterCtrl::illegal(Wien2::Apu::Wien2ApuCmd *cmd)
{
  cmd->result.exec_result = Wien2::Apu::ApuExecError;
}

