/****************************************************************************
 * modules/audio/components/postfilter/postfilter_through.h
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

#ifndef _POSTFILTER_THROUGH_H_
#define _POSTFILTER_THROUGH_H_

#include "audio/audio_high_level_api.h"
#include "memutils/s_stl/queue.h"
#include "postfilter_base.h"

class PostfilterThrough : public PostfilterBase
{
public:
  PostfilterThrough() {}
  ~PostfilterThrough() {}

  virtual uint32_t init_apu(const InitPostfilterParam& param, uint32_t *dsp_inf);
  virtual bool exec_apu(const ExecPostfilterParam& param);
  virtual bool flush_apu(const FlushPostfilterParam& param);
  virtual bool recv_done(PostfilterCmpltParam *cmplt);
  virtual bool recv_done(void) { return true; };
  virtual uint32_t activate(uint32_t *dsp_inf);
  virtual bool deactivate();

private:
  #define REQ_QUEUE_SIZE 7 
  typedef s_std::Queue<AsPcmDataParam, REQ_QUEUE_SIZE> ReqQue;
  ReqQue m_req_que;
};

#endif /* _POSTFILTER_THROUGH_H_ */

