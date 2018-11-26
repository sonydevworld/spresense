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
#include "wien2_common_defs.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"

#include "packing_component.h"
#ifdef CONFIG_AUDIOUTILS_SRC
#include "src_filter_component.h"
#endif
#ifdef CONFIG_AUDIOUTILS_MFE
#include "mfe_filter_component.h"
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
#include "mpp_filter_component.h"
#endif

#include "dsp_driver/include/dsp_drv.h"

__WIEN2_BEGIN_NAMESPACE

using namespace MemMgrLite;


typedef bool (*FilterCompCallback)(DspDrvComPrm_t*);

/* This enumertation defines all the types of filter component can provide. 
 * For component's user, it is not necessary to know how this component is
 * built, is it tunnelled or non-tunnelled. User only focus on what kinds of
 * functions they need to use.
 */
enum FilterComponentType
{
  SRCOnly = 0,           /* Use single core to process SRC. */
  MfeOnly,               /* Use MFE-SRC tunnelled modules. */
  MppEax,                /* Use master: XLOUD-SRC tunnelled
                          * and slave: MFE mechenism.
                          */
  BitWidthConv,          /* Use BitWidth Converter
                          * (DSP will not be loaded)
                          */
  FilterComponentTypeNum
};

struct FilterComponentParam
{
  Apu::ApuFilterType filter_type;
  FilterCompCallback callback;

  union
  {
    InitPackingParam init_packing_param;
    ExecPackingParam exec_packing_param;
    StopPackingParam stop_packing_param;

#ifdef CONFIG_AUDIOUTILS_SRC
    InitSRCParam init_src_param;
    ExecSRCParam exec_src_param;
    StopSRCParam stop_src_param;
#endif
#ifdef CONFIG_AUDIOUTILS_MFE
    InitMFEParam              init_mfe_param;
    ExecMFEParam              exec_mfe_param;
    Apu::ApuSetParamFilterCmd set_mfe_param;
    Apu::ApuTuningFilterCmd   tuning_mfe_param;
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
    InitXLOUDParam            init_xloud_param;
    ExecXLOUDParam            exec_xloud_param;
    Apu::ApuSetParamFilterCmd set_mpp_param;
    Apu::ApuTuningFilterCmd   tuning_mpp_param;
#endif
  };
};

struct FilterCompCmpltParam
{
  Apu::ApuEventType event_type;

  BufferHeader output_buffer;
};

/*--------------------------------------------------------------------*/
extern "C" {

uint32_t AS_filter_activate(FilterComponentType,
                            const char *,
                            MsgQueId,
                            PoolId,
                            uint32_t*);
bool AS_filter_deactivate(FilterComponentType);
uint32_t AS_filter_init(FilterComponentParam, uint32_t*);
bool AS_filter_exec(FilterComponentParam);
bool AS_filter_stop(FilterComponentParam);
bool AS_filter_setparam(FilterComponentParam);
bool AS_filter_tuning(FilterComponentParam);
bool AS_filter_recv_done(Apu::ApuFilterType type);

} /* extern "C" */

__WIEN2_END_NAMESPACE

#endif /* FILTER_COMPONENT_H */
