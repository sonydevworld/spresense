/****************************************************************************
 * audio_recorder/worker/userproc/src/userproc.cpp
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

#include "userproc.h"
#include "asmp.h"

/*--------------------------------------------------------------------*/
/*                                                                    */
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
void UserProc::init(InitParam *param)
{
  /* Init signal process. */

  param->result.result_code = CustomprocCommand::ExecOk;
}

/*--------------------------------------------------------------------*/
void UserProc::exec(ExecParam *param)
{
  /* Execute signal process to input audio data. */

  wk_memcpy(param->exec_cmd.output.addr,
            param->exec_cmd.input.addr,
            param->exec_cmd.input.size);

  param->exec_cmd.output.size = param->exec_cmd.input.size;

  if (m_enable)
    {
      param->exec_cmd.output.size =
        m_filter_ins.exec((int16_t *)param->exec_cmd.input.addr,
                          param->exec_cmd.input.size,
                          (int16_t *)param->exec_cmd.output.addr,
                          param->exec_cmd.output.size);
    }

  param->result.result_code = CustomprocCommand::ExecOk;
}

/*--------------------------------------------------------------------*/
void UserProc::flush(FlushParam *param)
{
  /* Flush signal process. */

  param->flush_cmd.output.size = 
    m_filter_ins.flush((int16_t *)param->flush_cmd.output.addr,
                       param->flush_cmd.output.size);

  param->result.result_code = CustomprocCommand::ExecOk;
}

/*--------------------------------------------------------------------*/
void UserProc::set(SetParam *param)
{
  /* Set signal process parameters.
   * Enable/Disable and Coef.
   */

  m_enable = param->enable;

  m_filter_ins.set(param->coef);

  param->result.result_code = CustomprocCommand::ExecOk;
}

