/****************************************************************************
 * modules/audiolite/worker/mp3dec/sprmp3_debug.h
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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

#ifndef __AUDIOLITE_WORKER_MP3DEC_SPRMP3_DEBUG_H
#define __AUDIOLITE_WORKER_MP3DEC_SPRMP3_DEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef BUILD_TGT_ASMPWORKER
# include <asmp/stdio.h>
#else
# include <stdio.h>
#endif

#include "minimp3_spresense.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef SPRMP3_DEBUG
#  define sprmp3_dprintf(...) printf(__VA_ARGS__)
#  ifndef SPRMP3_DEBUG_DETAIL
#    define print_status(s)
#    define print_buffer_status(s)
#  endif
#else
#  define sprmp3_dprintf(...)
#  define print_status(s)
#  define print_buffer_status(s)
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#ifdef SPRMP3_DEBUG

#ifdef SPRMP3_DEBUG_COMPARE
int dbg_load_mp3frame(const char *fname, unsigned char **fmem);
#endif

#ifdef SPRMP3_DEBUG_DETAIL
void print_status(sprmp3_sys_t *sys);
void print_buffer_status(sprmp3_sys_t *sys);
#endif

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __AUDIOLITE_WORKER_MP3DEC_SPRMP3_DEBUG_H */
