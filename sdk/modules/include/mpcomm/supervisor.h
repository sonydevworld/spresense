/****************************************************************************
 * mpcomm/supervisor/supervisor.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

/**
 * @file supervisor.h
 */

#ifndef __MODULES_INCLUDE_MPCOMM_SUPERVISOR_H
#define __MODULES_INCLUDE_MPCOMM_SUPERVISOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <pthread.h>

#include <asmp/types.h>
#include <asmp/mptask.h>
#include <asmp/mpmq.h>

#include <mpcomm/mpcomm.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct worker_info
 *
 * Information about worker (controller or helper).
 *
 * @typedef worker_info_t
 * See @ref worker_info
 */

typedef struct worker_info
{
  /** MP task object. See @ref mptask_t */

  mptask_t mptask;

  /** MP message queue object. See @ref mpmq_t */

  mpmq_t mq;

  /** CPU ID. See @ref cpuid_t */

  cpuid_t cpuid;
} worker_info_t;

/**
 * @struct supervisor_context
 *
 * Structure of information needed by a supervisor in the MPCOMM framework.
 *
 * @typedef mpcomm_supervisor_context_t
 * See @ref supervisor_context
 */

typedef struct supervisor_context
{
  /** Information about controller. See @ref worker_info_t */

  worker_info_t controller;

  /** Information about helpers. See @ref worker_info_t */

  worker_info_t helpers[MPCOMM_MAX_HELPERS];

  /** Set of CPUs used as a helper. See @ref cpu_set_t */

  cpu_set_t helpers_cpuset;

  /** Number of helpers used. */

  uint8_t helper_num;

  /** Controller listener thread identifier. */

  pthread_t controller_listener_pid;

  /** Semaphore to wait for controller done messages. */

  sem_t sem_done;
} mpcomm_supervisor_context_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * Initialize the MPCOMM framework.
 *
 * @param [in] ctx: Context with information about MPCOMM workers.
 * @param [in] filepath: Path to controller and helper binary file.
 * @param [in] helper_num: Number of helpers to be used.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_supervisor_init(mpcomm_supervisor_context_t **ctx,
                           const char *filepath,
                           uint8_t helper_num);

/**
 * Deinitialize the MPCOMM framework.
 *
 * @param [in] ctx: Context with information about MPCOMM workers.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_supervisor_deinit(mpcomm_supervisor_context_t *ctx);

/**
 * Send the user data to the controller for processing.
 *
 * @param [in] ctx: Context with information about MPCOMM workers.
 * @param [in] data: User data.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_supervisor_send_controller(mpcomm_supervisor_context_t *ctx,
                                      void *data);

/**
 * Wait until user data has been processed by controller.
 *
 * @param [in] ctx: Context with information about MPCOMM workers.
 * @param [in] abstime: The absolute time to wait until a timeout
 *                      is declared.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_supervisor_wait_controller_done(mpcomm_supervisor_context_t *ctx,
                                           const struct timespec *abstime);

#endif /* __MODULES_INCLUDE_MPCOMM_SUPERVISOR_H */
