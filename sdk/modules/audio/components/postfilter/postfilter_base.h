/****************************************************************************
 * modules/audio/components/postfilter/postfilter_base.h
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

#ifndef _POSTFILTER_BASE_H_
#define _POSTFILTER_BASE_H_

#include "apus/apu_cmd.h"
#include "audio/audio_common_defs.h"
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/memory_manager/MemHandle.h"

struct PostfilterCbParam
{
  Wien2::Apu::ApuEventType event_type;
};

typedef bool (*PostFilterCallback)(PostfilterCbParam*, void*);

struct InitPostfilterParam
{
  uint32_t           channel_num;
  uint32_t           bit_width;
  uint32_t           sample_num;
  PostFilterCallback callback;
  void               *p_requester;
};

struct ExecPostfilterParam
{
  AsPcmDataParam        input;
  MemMgrLite::MemHandle output_mh;
};

struct FlushPostfilterParam
{
  MemMgrLite::MemHandle output_mh;
};

struct PostfilterCmpltParam
{
  bool           result;
  AsPcmDataParam output;
};

class PostfilterBase
{
public:
  PostfilterBase() {}
  virtual ~PostfilterBase() {}

  virtual uint32_t init_apu(const InitPostfilterParam& param, uint32_t *dsp_inf) = 0;
  virtual bool exec_apu(const ExecPostfilterParam& param) = 0;
  virtual bool flush_apu(const FlushPostfilterParam& param) = 0;
  virtual bool recv_done(PostfilterCmpltParam *cmplt) = 0;
  virtual bool recv_done(void) = 0;
  virtual uint32_t activate(uint32_t *dsp_inf) = 0;
  virtual bool deactivate() = 0;

protected:
  PostFilterCallback m_callback;

  void *m_p_requester;
};

#endif /* _POSTFILTER_BASE_H_ */

