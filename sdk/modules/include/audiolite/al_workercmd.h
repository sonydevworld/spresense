/****************************************************************************
 * modules/include/audiolite/al_workercmd.h
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

#ifndef __INCLUDE_AUDIOLITE_WORKERCMD_H
#define __INCLUDE_AUDIOLITE_WORKERCMD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <audiolite/al_memalloc.h>
#include <audiolite/alworker_comm.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int alworker_send_systemparam(al_wtask_t *wtask,
                              int chnum, int hz, int mode);
int alworker_send_startframe(al_wtask_t *wtask);
int alworker_send_instgain(al_wtask_t *wtask, float gain);
int alworker_send_start(al_wtask_t *wtask,
                        al_comm_msgopt_t *opts = NULL);
int alworker_send_stop(al_wtask_t *wtask);
int alworker_send_term(al_wtask_t *wtask);
int alworker_inject_omem(al_wtask_t *wtask, audiolite_mem *mem);
int alworker_inject_imem(al_wtask_t *wtask, audiolite_mem *mem);
int alworker_send_resp(al_wtask_t *wtask, al_comm_msghdr_t hdr, int ret);
int alworker_send_usrcmd(al_wtask_t *wtask, al_comm_msgopt_t *opt);

#endif  /* __INCLUDE_AUDIOLITE_WORKERCMD_H */

