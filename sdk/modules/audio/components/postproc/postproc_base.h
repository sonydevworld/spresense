/****************************************************************************
 * modules/audio/components/postproc/postproc_base.h
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

#ifndef _POSTPROC_BASE_H_
#define _POSTPROC_BASE_H_

#include "dsp_framework/postproc_command_base.h"
#include "audio/audio_common_defs.h"
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/memory_manager/MemHandle.h"

enum PostCompEventType
{
  PostprocInit = 0,
  PostprocExec,
  PostprocFlush,
  PostprocSet,
};

struct PostprocCbParam
{
  PostCompEventType event_type;
  bool              result;
};

typedef bool (*PostprocCallback)(PostprocCbParam*, void*);

struct PostprocPacket
{
  uint8_t  *addr;
  uint32_t size;
};

struct InitPostprocParam
{
  uint8_t        cmd_type;
  bool           is_userdraw;
  PostprocPacket packet;
};
typedef InitPostprocParam SetPostprocParam;

struct ExecPostprocParam
{
  AsPcmDataParam        input;
  MemMgrLite::MemHandle output_mh;
};

struct FlushPostprocParam
{
  PostprocPacket        packet;
  MemMgrLite::MemHandle output_mh;
};

struct PostprocCmpltParam
{
  bool                 result;
  AsPcmDataParam       output;
};

class PostprocBase
{
public:
  PostprocBase() {}
  virtual ~PostprocBase() {}

  virtual uint32_t init_apu(const InitPostprocParam& param) = 0;
  virtual bool exec_apu(const ExecPostprocParam& param) = 0;
  virtual bool flush_apu(const FlushPostprocParam& param) = 0;
  virtual bool set_apu(const SetPostprocParam& param) = 0;
  virtual bool recv_done(PostprocCmpltParam *cmplt) = 0;
  virtual bool recv_done(void) = 0;
  virtual uint32_t activate(PostprocCallback callback,
                            void *p_requester,
                            uint32_t *dsp_inf) = 0;
  virtual bool deactivate() = 0;

protected:
  PostprocCallback m_callback;

  void *m_p_requester;
};

#endif /* _POSTPROC_BASE_H_ */

