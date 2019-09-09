/****************************************************************************
 * modules/include/audio/dsp_framework/customproc_command_base.h
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

#ifndef __CUSTOMPROC_COMMAND_BASE_H__
#define __CUSTOMPROC_COMMAND_BASE_H__

#include <stdint.h>

namespace CustomprocCommand
{
  enum command_type
  {
    Init,
    Exec,
    Flush,
    Set,
    CmdTypeNum
  };
  typedef command_type CmdType;

  enum process_mode
  {
    CommonMode = 0,
    FilterMode = 1,
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
    uint32_t size;
  };

  /*! Command header */

  struct command_header_s
  {
    uint32_t cmd_type;
  };
  typedef command_header_s CmdHeader;

  /*! Initialize command */

  struct init_command_base_s
  {
    /* reserve */

    uint32_t reserve0;
    uint32_t reserve1;
    uint32_t reserve2;
    uint32_t reserve3;
  };
  typedef init_command_base_s InitParamBase;

  /*! Execution command */

  struct exec_command_base_s
  {
    /* Fixed parameters */

    Buffer input;
    Buffer output;
  };
  typedef exec_command_base_s ExecParamBase;

  /*! Flush command */

  struct flush_command_base_s
  {
    /* Fixed parameters */

    Buffer output;

    /* reserve*/

    uint32_t reserve0;
    uint32_t reserve1;
  };
  typedef flush_command_base_s FlushParamBase;

  /*! Set command */

  struct set_command_base_s
  {
    /* reserve*/

    uint32_t reserve0;
    uint32_t reserve1;
    uint32_t reserve2;
    uint32_t reserve3;
  };
  typedef set_command_base_s SetParamBase;

  /*! Result */

  struct result_s
  {
    uint32_t result_code;
    uint32_t inform_req;
  };
  typedef result_s ResultParam;


  /*! Command structure definition */

  struct command_base_s
  {
    CmdHeader header;
    ResultParam result;

    union
    {
      InitParamBase  init_cmd;
      ExecParamBase  exec_cmd;
      FlushParamBase flush_cmd;
      SetParamBase   set_cmd;
    };

  };
  typedef command_base_s CmdBase;

};

#endif /* __CUSTOMPROC_COMMAND_BASE_H__ */

