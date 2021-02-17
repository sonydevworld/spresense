/****************************************************************************
 * mpcomm/controller/controller.c
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
 * @file controller.h
 */

#ifndef __MODULES_INCLUDE_MPCOMM_CONTROLLER_H
#define __MODULES_INCLUDE_MPCOMM_CONTROLLER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <asmp/types.h>
#include <asmp/mpmq.h>

#include <mpcomm/common.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/** Convert address from virtual to physical */

#define MEM_V2P(addr) controller_memory_virt_to_phys(addr)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Definition of callback function.
 *
 *  User-defined callback that is called by controller_main()
 *  after receiving data from the supervisor.
 *
 * @param[in] data : User data received from the supervisor.
 */

typedef void (*controller_user_func_t)(void *data);

/**
 * @struct helper_info
 *
 * Information about helper.
 *
 * @typedef helper_info_t
 * See @ref helper_info
 */

typedef struct helper_info
{
  /** MP message queue object. See @ref mpmq_t */

  mpmq_t mq;

  /** CPU ID. See @ref cpuid_t */

  cpuid_t cpuid;
} helper_info_t;

/**
 * @struct controller_context
 *
 * Structure of information needed by a controller in the MPCOMM framework.
 *
 * @typedef controller_context_t
 * See @ref controller_context
 */

typedef struct controller_context
{
  /** MP message queue to supervisor. See @ref mpmq_t */

  mpmq_t mq_2_supervisor;

  /** Information about helpers. See @ref helper_info_t */

  helper_info_t helpers[MPCOMM_MAX_HELPERS];

  /** Set of CPUs used as a helper. See @ref cpu_set_t */

  cpu_set_t helpers_cpuset;

  /**
   * Set of CPUs used as a helper that sends a MPCOMM_MSG_ID_DONE message.
   * See @ref cpu_set_t
   */

  cpu_set_t helpers_doneset;

  /**
   * If 1 then controller received MPCOMM_MSG_ID_DONE message from
   * supervisor, if 0 it did not.
   */

  uint8_t supervisor_done;

  /** Number of helpers used. */

  uint8_t helpers_num;

  /** User-defined callback. See @ref controller_user_func_t */

  controller_user_func_t user_func;

  /** Controller load address. */

  uintptr_t loadaddr;

  /** If 0, the helper should stay in the loop and if 1 it should quit. */

  uint8_t quit_loop;
} controller_context_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * Start the MPCOMM framework on the controller. The user can register
 * a callback that will handle the data received from the supervisor.
 *
 * @param [in] user_func: User-defined callback.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int controller_main(controller_user_func_t user_func);

/**
 * Send the user data to the helper for processing.
 *
 * @param [in] helper_index: Helper index.
 * @param [in] data: User data.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int controller_send_helper(uint8_t helper_index, void *data);

/**
 * Wait until user data has been processed by helper.
 *
 * @param [in] helper_index: Helper index.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int controller_wait_helper_done(uint8_t helper_index);

/**
 * Wait until user data has been processed by all helpers.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int controller_wait_helpers_done(void);

/**
 * Return number of helpers.
 *
 * @return Number of helpers.
 */

int controller_get_helpers_num(void);

/**
 * Convert address from virtual to physical.
 *
 * @param [in] addr: Virtual address.
 *
 * @return Physical address.
 */

void *controller_memory_virt_to_phys(void *addr);

/**
 * Send a request to the supervisor to call malloc() and
 * save the address of the allocated memory in the ptr.
 *
 * @param [out] ptr: The address of the allocated memory
 *                   received from the supervisor.
 * @param [in] size: Requested size.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int controller_send_malloc(void **ptr, size_t size);

/**
 * Send a request to the supervisor to call free() on the ptr.
 *
 * @param [in] ptr: The address of the allocated memory to be
 *                  freed by the supervisor.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int controller_send_free(void *ptr);

/**
 * Send the error number to the supervisor.
 *
 * @param [in] err: Error number.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int controller_send_error(int err);

#endif /* __MODULES_INCLUDE_MPCOMM_CONTROLLER_H */
