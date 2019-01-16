/****************************************************************************
 * modules/audio/components/filter/filter_component.h
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

#ifndef FILTER_COMPONENT_H
#define FILTER_COMPONENT_H

#include "memutils/os_utils/chateau_osal.h"
#include "apus/apu_cmd.h"
#include "wien2_common_defs.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"

#include "dsp_driver/include/dsp_drv.h"

__WIEN2_BEGIN_NAMESPACE

using namespace MemMgrLite;

/* This enumertation defines all the types of filter component can provide. 
 * For component's user, it is not necessary to know how this component is
 * built, is it tunnelled or non-tunnelled. User only focus on what kinds of
 * functions they need to use.
 */
enum FilterComponentType
{
  SampleRateConv = 0,    /* Use single core to process SRC. */
  MicFrontEnd,           /* Use MFE-SRC tunnelled modules. */
  MediaPlayerPost,       /* Use master: XLOUD-SRC tunnelled
                          * and slave: MFE mechenism.
                          */
  MediaPlayerPostAsSub,
  Packing,               /* Use BitWidth Converter
                          * (DSP will not be loaded)
                          */
  Through,               /* Through */
  FilterComponentTypeNum
};

/* Filter Events */

enum FilterComponentEvent
{
  InitEvent = 0,
  ExecEvent,
  StopEvent,
  SetParamEvent,
  TuningEvent
};

/*--------------------------------------------------------------------*/
/* Data structure definitions                                         */
/*--------------------------------------------------------------------*/

/* Base of Init Filter Parameters */

struct InitFilterParam
{
  int32_t  sample_per_frame;
  uint32_t in_fs;
  uint32_t out_fs;
  uint16_t in_bytelength;
  uint16_t out_bytelength;
  uint8_t  ch_num;
};

/* Base of Exec Filter Parameters */

struct ExecFilterParam
{
  BufferHeader in_buffer;
  BufferHeader out_buffer;
};

/* Base of Stop Filter Parameters */

struct StopFilterParam
{
  BufferHeader out_buffer;
};

/* Base of Set Filter Parameters */

struct SetFilterParam
{
};

/* Base of Tuning Filter Parameters */

struct TuningFilterParam
{
};

/* Base of Filter Complete Reply Parameters */

struct FilterCompCmpltParam
{
  FilterComponentEvent event_type;
  FilterComponentType  filter_type;
  bool result;

  BufferHeader out_buffer;
};

/* Filter Complete Reply Callback */

typedef bool (*FilterCompCallback)(FilterCompCmpltParam *);

/*--------------------------------------------------------------------*/
/* Class definitions                                                  */
/*--------------------------------------------------------------------*/

/* Filter Component Class */

class FilterComponent
{
public:

  FilterComponent() {}
  virtual ~FilterComponent() {}

  virtual bool setCallBack(FilterCompCallback func) { m_callback = func; return true; }

  virtual uint32_t activate_apu(const char *path, uint32_t *dsp_inf) = 0;
  virtual bool deactivate_apu(void) = 0;
  virtual uint32_t init_apu(InitFilterParam *param, uint32_t *dsp_inf) = 0;
  virtual bool exec_apu(ExecFilterParam *param) = 0;
  virtual bool flush_apu(StopFilterParam *param) = 0;

  virtual bool setparam_apu(SetFilterParam *param) = 0;
  virtual bool tuning_apu(TuningFilterParam *param) = 0;

  virtual bool recv_done(void) = 0;

private:

protected:

  FilterCompCallback m_callback;
};

__WIEN2_END_NAMESPACE

#endif /* FILTER_COMPONENT_H */

