/****************************************************************************
 * modules/audio/components/filter/filter_api.h
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

#ifndef FILTER_API_H
#define FILTER_API_H

#include "memutils/os_utils/chateau_osal.h"
#include "apus/apu_cmd.h"
#include "wien2_common_defs.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"

#include "packing_component.h"
#include "src_filter_component.h"
#ifdef CONFIG_AUDIOUTILS_MFE
#include "mfe_filter_component.h"
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
#include "mpp_filter_component.h"
#endif

__WIEN2_BEGIN_NAMESPACE

/*--------------------------------------------------------------------*/
extern "C" {

uint32_t AS_filter_activate(FilterComponentType,
                            const char *,
                            MsgQueId,
                            PoolId,
                            uint32_t *,
                            FilterCompCallback,
                            FilterComponent **);
bool AS_filter_deactivate(FilterComponent *p_ins, FilterComponentType type);
uint32_t AS_filter_init(InitFilterParam *, uint32_t *, FilterComponent *);
bool AS_filter_exec(ExecFilterParam *, FilterComponent *);
bool AS_filter_stop(StopFilterParam *, FilterComponent *);
bool AS_filter_setparam(SetFilterParam *, FilterComponent *);
bool AS_filter_tuning(TuningFilterParam *, FilterComponent *);
bool AS_filter_recv_done(FilterComponent *p_ins);

} /* extern "C" */

__WIEN2_END_NAMESPACE

#endif /* FILTER_COMPONENT_H */
