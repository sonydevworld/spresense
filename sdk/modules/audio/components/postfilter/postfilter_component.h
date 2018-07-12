/****************************************************************************
 * modules/audio/components/postfilter/postfilter_component.h
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

#ifndef _POSTFILTER_COMPONENT_H_
#define _POSTFILTER_COMPONENT_H_

#include "memutils/s_stl/queue.h"
#include "memutils/message/Message.h"
#include "debug/dbg_log.h"
#include "components/common/component_common.h"
#include "postfilter_base.h"

extern "C" {

bool AS_postfilter_recv_apu(void *p_param, void *p_instance);

} /* extern "C" */

class PostfilterComponent : public PostfilterBase,
                            public Wien2::ComponentCommon
{
public:
  PostfilterComponent(MemMgrLite::PoolId apu_pool_id,MsgQueId apu_mid):
      m_apu_pool_id(apu_pool_id)
    , m_apu_mid(apu_mid)
  {}
  ~PostfilterComponent() {}

  virtual uint32_t init_apu(const InitPostfilterParam& param, uint32_t *dsp_inf);
  virtual bool exec_apu(const ExecPostfilterParam& param);
  virtual bool flush_apu(const FlushPostfilterParam& param);
  virtual bool recv_done(PostfilterCmpltParam *cmplt);
  virtual bool recv_done(void) { return freeApuCmdBuf(); };
  virtual uint32_t activate(uint32_t *dsp_inf);
  virtual bool deactivate();

  bool recv_apu(void *p_param);
  MsgQueId get_apu_mid(void) { return m_apu_mid; };

  void *m_dsp_handler;

private:
  MemMgrLite::PoolId m_apu_pool_id;

  MsgQueId m_apu_mid;

  #define APU_DEC_QUEUE_SIZE 7

  struct ApuReqData
  {
    MemMgrLite::MemHandle cmd_mh;
    AsPcmDataParam        input;
    MemMgrLite::MemHandle output_mh;
  };

  typedef s_std::Queue<ApuReqData, APU_DEC_QUEUE_SIZE> ApuReqMhQue;
  ApuReqMhQue m_apu_req_mh_que;

  void send_apu(Wien2::Apu::Wien2ApuCmd*);

  void* allocApuBufs(AsPcmDataParam input, MemMgrLite::MemHandle output)
  {
    ApuReqData req;

    req.input     = input;
    req.output_mh = output;

    return pushqueue(req);
  }

  void* allocApuBufs(MemMgrLite::MemHandle output)
  {
    ApuReqData req;

    req.output_mh = output;

    return pushqueue(req);
  }

  void* allocApuBufs(void)
  {
    ApuReqData req;

    return pushqueue(req);
  }

  void* pushqueue(ApuReqData reqdata)
  {
    if (reqdata.cmd_mh.allocSeg(m_apu_pool_id, sizeof(Wien2::Apu::Wien2ApuCmd)) != ERR_OK)
      {
        DECODER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
        return NULL;
      }

    if (!m_apu_req_mh_que.push(reqdata))
      {
        DECODER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
        return NULL;
      }

    return reqdata.cmd_mh.getPa();
  }

  bool freeApuCmdBuf()
  {
    if (!m_apu_req_mh_que.pop())
      {
        DECODER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }

    return true;
  }
};

#endif /* _POSTFILTER_COMPONENT_H_ */

