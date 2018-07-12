/****************************************************************************
 * modules/asmp/supervisor/mptask.h
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

#ifndef __ASMP_SUPERVISOR_MPTASK_H
#define __ASMP_SUPERVISOR_MPTASK_H

/****************************************************************************
 * Include Files
 ****************************************************************************/

#include <sdk/debug.h>

#include <stdint.h>
#include <errno.h>
#include <semaphore.h>
#include <assert.h>

#include <asmp/mptask.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define task_set_exec(t)           do { (t)->attr.status = STATE_EXEC; } while (0)
#define task_set_pause(t)          do { (t)->attr.status = STATE_PAUSED; } while (0)
#define task_set_exit(t)           do { (t)->attr.status = STATE_EXIT; } while (0)
#define task_set_exit_status(t, c) do { (t)->attr.exit_status = (c); } while (0)
#define task_set_secure(t)         do { (t)->attr.secured = 1; } while (0)

#define task_is_init(t)   ((t)->attr.status == STATE_INIT)
#define task_is_exec(t)   ((t)->attr.status == STATE_EXEC)
#define task_is_paused(t) ((t)->attr.status == STATE_PAUSED)
#define task_is_exit(t)   ((t)->attr.status == STATE_EXIT)
#define task_is_secure(t) ((t)->attr.secured == 1)

#define mptask_semgive(id) sem_post(id)

#ifdef CONFIG_ASMP_DEBUG_ERROR
#  define mperr(fmt, ...)  logerr(fmt, ## __VA_ARGS__)
#else
#  define mperr(fmt, ...)
#endif
#ifdef CONFIG_ASMP_DEBUG_WARN
#  define mpwarn(fmt, ...)  logwarn(fmt, ## __VA_ARGS__)
#else
#  define mpwarn(fmt, ...)
#endif
#ifdef CONFIG_ASMP_DEBUG_INFO
#  define mpinfo(fmt, ...)  loginfo(fmt, ## __VA_ARGS__)
#else
#  define mpinfo(fmt, ...)
#endif

/* Number of MP CPUs */

#define NMPCPUS   6

static inline int mptask_semtake(sem_t *id)
{
  while (sem_wait(id) != 0)
    {
      ASSERT(errno == EINTR);
    }
  return OK;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct pm_cpu_wakelock_s g_mptask_wlock;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mptask_initialize(void);
int mptask_sighandler(int8_t signo, uint16_t sigdata, uint32_t data,
                      FAR void *userdata);
cpuid_t mptask_getcpuid(mptask_t *task);
void mptask_cpu_free(mptask_t *task);
int mptask_map(mptask_t *task);
void mptask_unmap(mptask_t *task);
void mptask_mapclear(mptask_t *task);
int mptask_exec_secure(mptask_t *task);

#endif
