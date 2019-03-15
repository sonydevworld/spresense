/****************************************************************************
 * modules/audio/components/filter/through_component.h
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

#ifndef THROUGH_COMPONENT_H
#define THROUGH_COMPONENT_H


#include "wien2_common_defs.h"
#include "debug/dbg_log.h"
#include "filter_component.h"

__WIEN2_BEGIN_NAMESPACE
using namespace MemMgrLite;

/*--------------------------------------------------------------------*/
/* Data structure definitions                                         */
/*--------------------------------------------------------------------*/

/* Init ThroughComponent Parameters */

struct InitThroughParam : public InitFilterParam
{
};

/* Exec ThroughComponent Parameters */

struct ExecThroughParam : public ExecFilterParam
{
};

/* Stop ThroughComponent Parameters */

struct StopThroughParam : public StopFilterParam
{
};

/* ThroughComponent Complete Reply Parameters */

struct ThroughCmpltParam : public FilterCompCmpltParam
{
};

/*--------------------------------------------------------------------*/
/* Class definitions                                                  */
/*--------------------------------------------------------------------*/

class ThroughComponent : public FilterComponent
{
private:

  uint32_t init_apu(InitThroughParam *param, uint32_t *dsp_inf);
  bool exec_apu(ExecThroughParam *param);
  bool flush_apu(StopThroughParam *param);

  void send_resp(FilterComponentEvent evt, bool result, BufferHeader outbuf);

public:

  ThroughComponent() {}
  ~ThroughComponent() {}

  virtual uint32_t activate_apu(const char *path, uint32_t *dsp_inf);
  virtual bool deactivate_apu();

  virtual uint32_t init_apu(InitFilterParam *param, uint32_t *dsp_inf)
  {
    return init_apu(static_cast<InitThroughParam *>(param), dsp_inf);
  }

  virtual bool exec_apu(ExecFilterParam *param)
  {
    return exec_apu(static_cast<ExecThroughParam *>(param));
  }

  virtual bool flush_apu(StopFilterParam *param)
  {
    return flush_apu(static_cast<StopThroughParam *>(param));
  }

  virtual bool setparam_apu(SetFilterParam *param) { return true; }
  virtual bool tuning_apu(TuningFilterParam *param) { return true; }
  virtual bool recv_done(void) { return true; };
};

__WIEN2_END_NAMESPACE

#endif /* THROUGH_COMPONENT_H */

