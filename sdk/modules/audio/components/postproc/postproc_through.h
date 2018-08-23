/****************************************************************************
 * modules/audio/components/postproc/postproc_through.h
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

#ifndef _POSTPROC_THROUGH_H_
#define _POSTPROC_THROUGH_H_

#include "audio/audio_high_level_api.h"
#include "memutils/s_stl/queue.h"
#include "postproc_base.h"

class PostprocThrough : public PostprocBase
{
public:
  PostprocThrough() {}
  ~PostprocThrough() {}

  virtual uint32_t init_apu(const InitPostprocParam& param);
  virtual bool sendcmd_apu(const SendPostprocParam& param);
  virtual bool exec_apu(const ExecPostprocParam& param);
  virtual bool flush_apu(const FlushPostprocParam& param);
  virtual bool recv_done(PostprocCmpltParam *cmplt);
  virtual uint32_t activate(uint32_t *dsp_inf);
  virtual bool deactivate();

private:
  #define REQ_QUEUE_SIZE 7 

  struct ApuReqData
  {
    PostCompFunctionType func_type;
    AsPcmDataParam       pcm;
  };

  typedef s_std::Queue<ApuReqData, REQ_QUEUE_SIZE> ReqQue;
  ReqQue m_req_que;
};

#endif /* _POSTPROC_THROUGH_H_ */

