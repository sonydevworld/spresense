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
#include "filter_component.h"

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

/* Init PackingComponent Parameters */

struct InitPackingParam : public InitFilterParam
{
};

/* Exec PackingComponent Parameters */

struct ExecPackingParam : public ExecFilterParam
{
};

/* Stop PackingComponent Parameters */

struct StopPackingParam : public StopFilterParam
{
};

/* PackingComponent Complete Reply Parameters */

struct PackingCmpltParam : public FilterCompCmpltParam
{
};

/*--------------------------------------------------------------------*/
/* Class definitions                                                  */
/*--------------------------------------------------------------------*/

class PackingComponent : public FilterComponent
{
private:

  uint16_t m_in_bitwidth;
  uint16_t m_out_bitwidth;

  uint32_t init_apu(InitPackingParam *param);
  bool exec_apu(ExecPackingParam *param);
  bool flush_apu(StopPackingParam *param);

  void cnv32to24(uint32_t samples, int8_t *in, int8_t *out);
  void cnv24to32(uint32_t samples, int8_t *in, int8_t *out);
  void send_resp(FilterComponentEvent evt, bool result, BufferHeader outbuf);

public:

  PackingComponent() :
      m_in_bitwidth(32)
    , m_out_bitwidth(24)
    {}
  ~PackingComponent() {}

  virtual uint32_t activate_apu(const char *path, uint32_t *dsp_inf);
  virtual bool deactivate_apu();
  virtual uint32_t init_apu(InitFilterParam *param, uint32_t *dsp_inf)
  {
    return init_apu(static_cast<InitPackingParam *>(param));
  }

  virtual bool exec_apu(ExecFilterParam *param)
  {
    return exec_apu(static_cast<ExecPackingParam *>(param));
  }

  virtual bool flush_apu(StopFilterParam *param)
  {
    return flush_apu(static_cast<StopPackingParam *>(param));
  }

  virtual bool setparam_apu(SetFilterParam *param) { return true; }
  virtual bool tuning_apu(TuningFilterParam *param) { return true; }
  virtual bool recv_done(void) { return true; };
};

__WIEN2_END_NAMESPACE

#endif /* PACKING_COMPONENT_H */

