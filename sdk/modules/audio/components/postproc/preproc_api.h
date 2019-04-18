/****************************************************************************
 * modules/audio/components/postproc/preproc_api.h
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

#ifndef _PREPROC_API_H_
#define _PREPROC_API_H_

#include "postproc_api.h"

/* Wrap of APIs */

#define InitPreprocParam InitPostprocParam
#define AS_preproc_init(a,b) AS_postproc_init(a,b)

#define ExecPreprocParam ExecPostprocParam
#define AS_preproc_exec(a,b) AS_postproc_exec(a,b)

#define FlushPreprocParam FlushPostprocParam
#define AS_preproc_flush(a,b) AS_postproc_flush(a,b)

#define SetPreprocParam SetPostprocParam
#define AS_preproc_setparam(a,b) AS_postproc_setparam(a,b)

#define AS_preproc_is_enable(a) AS_postproc_is_enable(a)

#define PreprocCmpltParam PostprocCmpltParam
#define AS_preproc_recv_done(a, b) AS_postproc_recv_done(a, b)

#define AS_preproc_activate(a,b,c,d,e,f,g,h) AS_postproc_activate(a,b,c,d,e,f,g,h)

#define AS_preproc_deactivate(a) AS_postproc_deactivate(a)


#define PreprocCbParam PostprocCbParam

#define PreCompEventType PostCompEventType
#define PreprocInit PostprocInit
#define PreprocExec PostprocExec
#define PreprocFlush PostprocFlush
#define PreprocSet PostprocSet


#endif /* _PREPROC_API_H_ */

