/****************************************************************************
 * modules/audio/components/dsp_req_que.h
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#ifndef _DSP_REQ_QUE_H_
#define _DSP_REQ_QUE_H_


#include "memutils/os_utils/chateau_osal.h"
#include "memutils/common_utils/common_assert.h"
#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/s_stl/queue.h"
#include "debug/dbg_log.h"

using namespace MemMgrLite;
__WIEN2_BEGIN_NAMESPACE


template<class T, int N>
class DspReqQue
{

public:

  T* alloc(AsPcmDataParam input, MemHandle output)
  {
    DspReqData req;
    req.input  = input;
    req.output = output;
    return pushqueue(req);
  }

  T* alloc(AsPcmDataParam input)
  {
    DspReqData req;
    req.input  = input;
    return pushqueue(req);
  }

  T* alloc(MemHandle output)
  {
    DspReqData req;
    req.output = output;
    return pushqueue(req);
  }

  T* alloc(void)
  {
    DspReqData req;
    return pushqueue(req);
  }

  T* top_cmd(void)
  {
    return static_cast<T *>(m_req_que.top().cmd.getPa());
  }

  AsPcmDataParam top_input(void)
  {
    return m_req_que.top().input;
  }

  MemHandle top_output(void)
  {
    return m_req_que.top().output;
  }

  bool free()
  {
    if (!m_req_que.pop())
      {
        CUSTOM_CMP_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }
    return true;
  }

  void set_pool_id(PoolId pool_id) { m_pool_id = pool_id; }

private:

  PoolId   m_pool_id;

  struct DspReqData
  {
    MemHandle       cmd;
    AsPcmDataParam  input;
    MemHandle       output;
  };

  typedef s_std::Queue<DspReqData, N> ReqQueue;
  ReqQueue m_req_que;

  T* pushqueue(DspReqData req)
  {
    if(m_pool_id != NullPoolId)
      {
        if (req.cmd.allocSeg(m_pool_id, sizeof(T)) != ERR_OK)
          {
            CUSTOM_CMP_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
            return NULL;
          }
        if (!m_req_que.push(req))
          {
            CUSTOM_CMP_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
            return NULL;
          }
        return static_cast<T *>(req.cmd.getPa());
    } else {
      if (!m_req_que.push(req))
        {
          CUSTOM_CMP_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
        }
      return NULL;
    }
  }

};

__WIEN2_END_NAMESPACE

#endif /* _DSP_REQ_QUE_H_ */

