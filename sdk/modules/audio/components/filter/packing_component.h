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
#include "apus/apu_cmd.h"
#include "dsp_driver/include/dsp_drv.h"


#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE
using namespace MemMgrLite;

/*--------------------------------------------------------------------*/
enum BitWidth
{
  BitWidth24bit = 24,
  BitWidth32bit = 32,
};

/*--------------------------------------------------------------------*/
struct InitPackingParam
{
  BitWidth in_bitwidth;
  BitWidth out_bitwidth;
};

struct ExecPackingParam
{
  BufferHeader in_buffer;
  BufferHeader out_buffer;
};

struct StopPackingParam
{
  BufferHeader out_buffer;
};

/*--------------------------------------------------------------------*/
class PackingComponent
{
public:

  PackingComponent()
    : m_in_bitwidth(32)
    , m_out_bitwidth(24)
    {}

  ~PackingComponent() {}

  typedef bool (*PackingCompCallback)(DspDrvComPrm_t*);

  uint32_t activate_apu(PackingComponent *p_component);
  bool deactivate_apu();
  uint32_t init_apu(InitPackingParam param);
  bool exec_apu(ExecPackingParam param);
  bool flush_apu(StopPackingParam param);

  bool setCallBack(PackingCompCallback func) { m_callback = func; return true; };
  bool recv_done(void) { return true; };

private:

  PackingCompCallback m_callback;

  uint16_t m_in_bitwidth;
  uint16_t m_out_bitwidth;

  void cnv32to24(uint32_t samples, int8_t *in, int8_t *out);
  void cnv24to32(uint32_t samples, int8_t *in, int8_t *out);
  void notify_reply(uint8_t evt, bool result, BufferHeader outbuf);
};

__WIEN2_END_NAMESPACE

#endif /* PACKING_COMPONENT_H */

