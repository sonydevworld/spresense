/****************************************************************************
 * modules/audio/components/common/component_base.h
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

#ifndef _CUSTOMPROC_BASE_H_
#define _CUSTOMPROC_BASE_H_

#include "audio/audio_common_defs.h"
#include "audio/audio_message_types.h"

#include "memutils/os_utils/chateau_osal.h"
#include "memutils/common_utils/common_assert.h"
#include "memutils/message/Message.h"

#include "apus/dsp_audio_version.h"
#include "audio_component_api.h"

using namespace MemMgrLite;
__WIEN2_BEGIN_NAMESPACE

typedef bool (*ComponentCallback)(ComponentCbParam*, void*);

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
#define AUDIOUTILS_DSP_DEBUG_DUMP_SIZE  (1948)
#define LOG_ENTRY_NAME                  (8)

struct DebugLogInfo
{
  char name[LOG_ENTRY_NAME];
  void *addr;
};
#endif

template<typename T>
struct DspResult
{
  uint32_t exec_result;
  T        internal_result;
};

class ComponentBase
{
public:
  ComponentBase() {}

  ComponentBase(PoolId pool_id, MsgQueId msgq_id):
    m_pool_id(pool_id),
    m_msgq_id(msgq_id)
  {}


  virtual ~ComponentBase() {}

  virtual uint32_t init(const InitComponentParam& param) {F_ASSERT(0);}
  virtual bool exec(const ExecComponentParam& param) {F_ASSERT(0);}
  virtual bool flush(const FlushComponentParam& param) {F_ASSERT(0);}
  virtual bool set(const SetComponentParam& param) {F_ASSERT(0);}
  virtual bool recv_done(ComponentCmpltParam *cmplt) {F_ASSERT(0);}
  virtual bool recv_done(ComponentInformParam *info) {F_ASSERT(0);}
  virtual bool recv_done(void) {F_ASSERT(0);}
  virtual uint32_t activate(ComponentCallback callback,
                            const char *image_name,
                            void *p_requester,
                            uint32_t *dsp_inf) {F_ASSERT(0);}
  virtual bool deactivate() {F_ASSERT(0);}

  bool dsp_boot_check(MsgQueId dsp_dtq, uint32_t *dsp_inf);

  MsgQueId get_msgq_id(void) { return m_msgq_id; };

protected:

  PoolId   m_pool_id;
  MsgQueId m_msgq_id;

  template<typename T>
  uint32_t dsp_init_check(MsgQueId dsp_dtq, T *internal)
  {
    err_t        err_code;
    MsgQueBlock  *que;
    MsgPacket    *msg;

    err_code = MsgLib::referMsgQueBlock(dsp_dtq, &que);
    F_ASSERT(err_code == ERR_OK);

    err_code = que->recv(TIME_FOREVER, &msg);
    F_ASSERT(err_code == ERR_OK);
    F_ASSERT(msg->getType() == MSG_ISR_APU0);

    DspResult<T> dsp_result = msg->moveParam< DspResult<T> >();
    err_code = que->pop();
    F_ASSERT(err_code == ERR_OK);

    *internal = dsp_result.internal_result;

    return dsp_result.exec_result;
  }

  template<typename T>
  void dsp_init_complete(MsgQueId dsp_dtq, uint32_t result, T *internal)
  {
    DspResult<T> dsp_result;

    dsp_result.exec_result     = result;
    dsp_result.internal_result = *internal;

    err_t er = MsgLib::send< DspResult<T> >(dsp_dtq,
                                            MsgPriNormal,
                                            MSG_ISR_APU0,
                                            0,
                                            dsp_result);
    F_ASSERT(er == ERR_OK);
  }

  ComponentCallback m_callback;

  void *m_p_requester;

};

__WIEN2_END_NAMESPACE

#endif /* _CUSTOMPROC_BASE_H_ */

