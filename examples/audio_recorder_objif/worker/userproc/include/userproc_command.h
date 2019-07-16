/****************************************************************************
 * audio_recorder/worker/userproc/include/userproc_command.h
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

#ifndef __USERPROC_COMMAND_H__
#define __USERPROC_COMMAND_H__

#include <stdint.h>
#include <audio/dsp_framework/customproc_command_base.h>

struct InitParam : public CustomprocCommand::CmdBase
{
  uint32_t reserve0;
  uint32_t reserve1;
  uint32_t reserve2;
  uint32_t reserve3;
};

struct ExecParam : public CustomprocCommand::CmdBase
{
  uint32_t reserve0;
  uint32_t reserve1;
  uint32_t reserve2;
  uint32_t reserve3;
};

struct FlushParam : public CustomprocCommand::CmdBase
{
  uint32_t reserve0;
  uint32_t reserve1;
  uint32_t reserve2;
  uint32_t reserve3;
};

struct SetParam : public CustomprocCommand::CmdBase
{
  uint32_t enable;
  uint32_t coef;
  uint32_t reserve2;
  uint32_t reserve3;
};

#endif /* __USERPROC_COMMAND_H__ */

