/****************************************************************************
 * modules/audio/components/filter/packing_component.h
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

#ifndef PACKING_COMPONENT_H
#define PACKING_COMPONENT_H


#include "wien2_common_defs.h"
#include "debug/dbg_log.h"
#include "memutils/s_stl/queue.h"
#include "components/component_base.h"

__WIEN2_BEGIN_NAMESPACE
using namespace MemMgrLite;

/*--------------------------------------------------------------------*/
enum BitWidth
{
  BitWidth24bit = 24,
  BitWidth32bit = 32,
};

/*--------------------------------------------------------------------*/
/* Data structure definitions                                         */
/*--------------------------------------------------------------------*/


/*--------------------------------------------------------------------*/
/* Class definitions                                                  */
/*--------------------------------------------------------------------*/

class PackingComponent : public ComponentBase
{
private:

  /* Request queue */

  static const uint32_t ReqQueueSize = 7;

  s_std::Queue<AsPcmDataParam, ReqQueueSize> m_req_que;

  MsgQueId m_apu_dtq;
  PoolId m_apu_pool_id;

  uint16_t m_in_bitwidth;
  uint16_t m_out_bitwidth;

  void cnv32to24(uint32_t samples, int8_t *in, int8_t *out);
  void cnv24to32(uint32_t samples, int8_t *in, int8_t *out);
  void send_resp(ComponentEventType evt, bool result);

public:

  PackingComponent() :
      m_in_bitwidth(32)
    , m_out_bitwidth(24)
    {}
  ~PackingComponent() {}

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
};

__WIEN2_END_NAMESPACE

#endif /* PACKING_COMPONENT_H */

