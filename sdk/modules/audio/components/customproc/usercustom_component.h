/****************************************************************************
 * modules/audio/components/customproc/usercustom_component.h
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

#ifndef _USERCUSTOM_COMPONENT_H_
#define _USERCUSTOM_COMPONENT_H_

#include "audio/dsp_framework/customproc_command_base.h"
#include "memutils/s_stl/queue.h"
#include "memutils/message/Message.h"
#include "debug/dbg_log.h"
#include "components/component_base.h"
#include "dsp_req_que.h"

extern "C" {

bool AS_postproc_recv_apu(void *p_param, void *p_instance);

} /* extern "C" */

using namespace MemMgrLite;
__USING_WIEN2

class UserCustomComponent : public ComponentBase
{
public:
  UserCustomComponent(PoolId pool_id, MsgQueId msgq_id):
      ComponentBase(pool_id, msgq_id)
  { m_req_que.set_pool_id(pool_id); }

  ~UserCustomComponent() {}

  virtual uint32_t init(const InitComponentParam& param);
  virtual bool exec(const ExecComponentParam& param);
  virtual bool flush(const FlushComponentParam& param);
  virtual bool set(const SetComponentParam& param);
  virtual bool recv_done(ComponentCmpltParam *cmplt);
  virtual bool recv_done(ComponentInformParam *info);
  virtual bool recv_done(void) { return m_req_que.free(); }
  virtual uint32_t activate(ComponentCallback callback,
                            const char *image_name,
                            void *p_requester,
                            uint32_t *dsp_inf);
  virtual bool deactivate();

  bool recv_dsp(void *p_param);

  void *m_dsp_handler;

private:

  #define REQ_QUEUE_SIZE 7
  
  DspReqQue<CustomprocCommand::CmdBase, REQ_QUEUE_SIZE> m_req_que;

  void send(void *);

};

#endif /* _USERCUSTOM_COMPONENT_H_ */

