/****************************************************************************
 * modules/audio/components/filter/src_filter_component.h
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

#ifndef SRC_FILTER_COMPONENT_H
#define SRC_FILTER_COMPONENT_H

#include "memutils/os_utils/chateau_osal.h"
#include "wien2_common_defs.h"
#include "memutils/s_stl/queue.h"
#include "apus/apu_cmd.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "dsp_driver/include/dsp_drv.h"
#include "components/component_base.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE
using namespace MemMgrLite;

/*--------------------------------------------------------------------*/
/* Data structure definitions                                         */
/*--------------------------------------------------------------------*/


/*--------------------------------------------------------------------*/
/* Class definitions                                                  */
/*--------------------------------------------------------------------*/

class SRCComponent : public ComponentBase
{
private:

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  DebugLogInfo m_debug_log_info;
#endif

  /* Request queue */

  static const uint32_t ReqQueueSize = 7;

 struct ApuReqData
  {
    MemMgrLite::MemHandle cmd_mh;
    AsPcmDataParam        input;
    MemMgrLite::MemHandle output_mh;
  };

  s_std::Queue<ApuReqData, ReqQueueSize> m_apu_req_que;

  MsgQueId m_apu_dtq;
  PoolId m_apu_pool_id;

  void send_apu(Apu::Wien2ApuCmd*);

public:

  SRCComponent(PoolId apu_pool_id, MsgQueId apu_dtq)
    : m_apu_dtq(apu_dtq)
    , m_apu_pool_id(apu_pool_id)
    , m_dsp_handler(NULL) {}
  ~SRCComponent() {}

  virtual uint32_t init(const InitComponentParam& param);
  virtual bool exec(const ExecComponentParam& param);
  virtual bool flush(const FlushComponentParam& param);
  virtual bool set(const SetComponentParam& param);
  virtual bool recv_done(ComponentCmpltParam *cmplt);
  virtual bool recv_done(ComponentInformParam *info);
  virtual bool recv_done(void);
  virtual uint32_t activate(ComponentCallback callback,
                            const char *image_name,
                            void *p_requester,
                            uint32_t *dsp_inf);
  virtual bool deactivate();

  bool recv_apu(DspDrvComPrm_t*);

  MsgQueId get_apu_mid(void) { return m_apu_dtq; };

  void *m_dsp_handler;
};

__WIEN2_END_NAMESPACE

#endif /* SRC_FILTER_COMPONENT_H */

