/****************************************************************************
 * modules/audio/components/postproc/postproc_api.h
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

#ifndef _POSTPROC_API_H_
#define _POSTPROC_API_H_

#include "postproc_component.h"
#include "postproc_through.h"

extern "C" {

uint32_t AS_postproc_init(const InitPostprocParam *param,
                          void *p_instance,
                          uint32_t *dsp_inf);

bool AS_postproc_sendcmd(const SendPostprocParam *param, void *p_instance);

bool AS_postproc_exec(const ExecPostprocParam *param, void *p_instance);

bool AS_postproc_flush(const FlushPostprocParam *param, void *p_instance);

bool AS_postproc_recv_done(void *p_instance, PostprocCmpltParam *cmplt);

uint32_t AS_postproc_activate(void **p_instance,
                              MemMgrLite::PoolId apu_pool_id,
                              MsgQueId apu_mid,
                              uint32_t *dsp_inf,
                              bool through);

bool AS_postproc_deactivate(void *p_instance);

} /* extern "C" */

#endif /* _POSTPROC_API_H_ */

