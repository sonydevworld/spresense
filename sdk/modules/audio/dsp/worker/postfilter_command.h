/****************************************************************************
 * modules/audio/dsp/worker/postfilter_command.h
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

#ifndef __POSTFILTER_COMMAND_H__
#define __POSTFILTER_COMMAND_H__

#include <stdint.h>

class PostFilterCommand
{
public:

  enum command_type
  {
    Boot = 0,
    Init,
    Exec,
    Flush,
    SetParam,
    Tuning,
    Error,
    CmdTypeNum
  };
  typedef command_type CmdType;

  enum process_mode 
  {
    CommonMode = 0,
    FilterMode = 2,
  };
  typedef process_mode ProcMode;

  enum data_type
  {
    DataTypeAddr = 0,
    DataTypeValue,
  };
  typedef data_type DataType;

  enum result_code
  {
    ExecOk = 0,
    ExecWarn,
    ExecError,
  };
  typedef result_code ResultCode;

  struct Buffer
  {
    void     *addr;
    uint32_t sample;
  };

  /*! Command header */

  struct command_header_s
  {
    uint8_t cmd_type;
    uint8_t process_mode;
  };
  typedef command_header_s CmdHeader;

  /*! Initialize command */

  struct init_command_s
  {
    uint32_t ch_num;
    uint32_t bit_width;
    uint32_t sample;
  };
  typedef init_command_s InitCmd;

  /*! Execution command */

  struct exec_command_s
  {
    Buffer input;
    Buffer output;
  };
  typedef exec_command_s ExecCmd;

  /*! Flush command */

  struct flush_command_s
  {
    Buffer output;
  };
  typedef flush_command_s FlushCmd;

  /*! Result */

  struct result_s
  {
    ResultCode result_code;
  };
  typedef result_s RestltParam;


  /*! Command structure definition */

  struct command_s
  {
    CmdHeader header;

    union
    {
      InitCmd  init_cmd;
      ExecCmd  exec_cmd;
      FlushCmd flush_cmd;
    };

    RestltParam result;
  };
  typedef command_s Cmd;
};


#endif /* __POSTFILTER_COMMAND_H__ */

