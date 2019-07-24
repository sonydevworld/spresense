/****************************************************************************
 * modules/audio/components/customproc/srcwrap_component.h
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

#ifndef _SRCWRAP_COMPONENT_H_
#define _SRCWRAP_COMPONENT_H_

#include "audio/audio_high_level_api.h"
#include "memutils/s_stl/queue.h"
#include "components/filter/src_filter_component.h"
#include "customproc_base.h"

/* Wrapping of SRC initialize. */

typedef struct
{
  /* SRC initialize paramters. */

  Wien2::InitFilterParam param;

} InitSrcParam;

/* This class is wrapper of proprietary SRCComponent to use SRC DSP. */

class SrcWrapComponent : public CustomProcBase
{
public:
  SrcWrapComponent(MemMgrLite::PoolId apu_pool_id,MsgQueId apu_mid)
  {
    m_p_src_instance = new Wien2::SRCComponent(apu_mid, apu_pool_id);
  }
  ~SrcWrapComponent() {}

  virtual uint32_t init(const InitCustomProcParam& param);
  virtual bool exec(const ExecCustomProcParam& param);
  virtual bool flush(const FlushCustomProcParam& param);
  virtual bool set(const SetCustomProcParam& param);
  virtual bool recv_done(CustomProcCmpltParam *cmplt);
  virtual bool recv_done(CustomProcInformParam *info);
  virtual bool recv_done(void);
  virtual uint32_t activate(CustomProcCallback callback,
                            const char *image_name,
                            void *p_requester,
                            uint32_t *dsp_inf);
  virtual bool deactivate();

  bool recv_apu(Wien2::FilterCompCmpltParam *srccmplt);

  uint32_t init_src(InitSrcParam& param);

private:
  Wien2::SRCComponent *m_p_src_instance;

  /* Request queue */

  #define REQ_QUEUE_SIZE 7 

  struct ApuReqData
  {
    AsPcmDataParam        input;
    MemMgrLite::MemHandle output_mh;
  };

  typedef s_std::Queue<ApuReqData, REQ_QUEUE_SIZE> ReqQue;
  ReqQue m_req_que;

  /* Complete queue */

  #define CMPLT_QUEUE_SIZE 3 

  typedef s_std::Queue<CustomProcCmpltParam, CMPLT_QUEUE_SIZE> CmpltQue;
  CmpltQue m_cmplt_que;
};

#endif /* _SRCWRAP_COMPONENT_H_ */

