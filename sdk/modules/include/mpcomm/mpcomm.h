/****************************************************************************
 * mpcomm/mpcomm.h
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
 * @file mpcomm.h
 */

#ifndef __MODULES_INCLUDE_MPCOMM_MPCOMM_H
#define __MODULES_INCLUDE_MPCOMM_MPCOMM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#include <asmp/types.h>
#include <asmp/mpmq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/** The helper and controller initialization message ID. */

#define MPCOMM_MSG_ID_INIT 1

/** The helper and controller deinitialization message ID. */

#define MPCOMM_MSG_ID_DEINIT 2

/** Message ID to send user data to controller and helper. */

#define MPCOMM_MSG_ID_USER_FUNC 3

/** Message ID confirming that the message has been processed. */

#define MPCOMM_MSG_ID_DONE 4

/** Message ID to request malloc() from supervisor. */

#define MPCOMM_MSG_ID_MALLOC 5

/** Message ID to request free() from supervisor. */

#define MPCOMM_MSG_ID_FREE 6

/** Message ID to send error code to supervisor. */

#define MPCOMM_MSG_ID_ERROR 7

/** Message ID to send log message to supervisor. */

#define MPCOMM_MSG_ID_LOG 8

/** Maximum number of helpers. */

#define MPCOMM_MAX_HELPERS 4

/** Maximum CPU ID. */

#define MPCOMM_CPUID_MAX 8

/** Controller mode. */

#define MPCOMM_MODE_CONTROLLER 0

/** Helper mode. */

#define MPCOMM_MODE_HELPER 1

/** Convert address from virtual to physical */

#define MEM_V2P(addr) mpcomm_memory_virt_to_phys(addr)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Definition of callback function.
 *
 *  User-defined callback that is called by mpcomm_main()
 *  after receiving data from the supervisor or controller.
 *
 * @param[in] data : User data.
 */

typedef void (*mpcomm_user_func_t)(void *data);

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
 * @struct mpcomm_context
 *
 * Structure of information needed by a controller and helpers
 * in the MPCOMM framework.
 *
 * @typedef mpcomm_context_t
 * See @ref mpcomm_context
 */

typedef struct mpcomm_context
{
  /** Worker mode. */

  uint8_t mode;

  /** MP message queue to supervisor. See @ref mpmq_t */

  mpmq_t mq_2_supervisor;

  /** MP message queue to controller. See @ref mpmq_t */

  mpmq_t mq_2_controller;

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

  /** User-defined callback for controller. See @ref mpcomm_user_func_t */

  mpcomm_user_func_t controller_user_func;

  /** User-defined callback for helpers. See @ref mpcomm_user_func_t */

  mpcomm_user_func_t helper_user_func;

  /** Controller/Helper load address. */

  uintptr_t loadaddr;

  /** If 0, the helper should stay in the loop and if 1 it should quit. */

  uint8_t quit_loop;
} mpcomm_context_t;

/**
 * @struct mpcomm_init_msg
 *
 * Structure for controller and helper initialization.
 *
 * @typedef mpcomm_init_msg_t
 * See @ref mpcomm_init_msg
 */

typedef struct mpcomm_init_msg
{
  /** Worker mode. */

  uint8_t mode;

  /** Controller CPU ID. See @ref cpuid_t */

  cpuid_t controller_cpuid;

  /** Set of CPUs used as a helper. See @ref cpu_set_t */

  cpu_set_t helpers_cpuset;

  /** Controller/Helper load address. */

  uintptr_t loadaddr;
} mpcomm_init_msg_t;

/**
 * @struct mpcomm_malloc_msg
 *
 * Structure for malloc() request.
 *
 * @typedef mpcomm_malloc_msg_t
 * See @ref mpcomm_malloc_msg
 */

typedef struct mpcomm_malloc_msg
{
  /** Core CPU ID that is requesting malloc. See @ref cpuid_t */

  cpuid_t cpuid;

  /** The address of the allocated memory received from the supervisor. */

  void **ptr;

  /** Requested size. */

  size_t size;
} mpcomm_malloc_msg_t;

/**
 * @struct mpcomm_free_msg
 *
 * Structure for free() request.
 *
 * @typedef mpcomm_free_msg_t
 * See @ref mpcomm_free_msg
 */

typedef struct mpcomm_free_msg
{
  /** Core CPU ID that is requesting free. See @ref cpuid_t */

  cpuid_t cpuid;

  /** The address of the allocated memory to be freed by the supervisor. */

  void *ptr;
} mpcomm_free_msg_t;

/**
 * @struct mpcomm_error_msg
 *
 * Structure for error message.
 *
 * @typedef mpcomm_error_msg_t
 * See @ref mpcomm_error_msg
 */

typedef struct mpcomm_error_msg
{
  /** Core CPU ID that is sending error code. See @ref cpuid_t */

  cpuid_t cpuid;

  /** The error code. */

  int error;
} mpcomm_error_msg_t;

/**
 * @struct mpcomm_log_msg
 *
 * Structure for error message.
 *
 * @typedef mpcomm_log_msg_t
 * See @ref mpcomm_log_msg
 */

typedef struct mpcomm_log_msg
{
  /** Core CPU ID that is sending error code. See @ref cpuid_t */

  cpuid_t cpuid;

  /** Log to be printed. */

  char *log;
} mpcomm_log_msg_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * Start the MPCOMM framework on the controller or the helper. The user can
 * register a callbacks that will handle the data received from
 * the supervisor and controller.
 *
 * @param [in] controller_user_func: User-defined callback for the contoller.
 * @param [in] helper_user_func: User-defined callback for the helper.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_main(mpcomm_user_func_t controller_user_func,
                mpcomm_user_func_t helper_user_func);

/**
 * Send the user data to the helper for processing.
 *
 * @param [in] helper_index: Helper index.
 * @param [in] data: User data.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_send_helper(uint8_t helper_index, void *data);

/**
 * Wait until user data has been processed by helper.
 *
 * @param [in] helper_index: Helper index.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_wait_helper_done(uint8_t helper_index);

/**
 * Wait until user data has been processed by all helpers.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_wait_helpers_done(void);

/**
 * Return number of helpers.
 *
 * @return Number of helpers.
 */

int mpcomm_get_helpers_num(void);

/**
 * Return true if it is controller. Otherwise, return false.
 *
 * @return True, if it is controller.
 */

bool mpcomm_is_controller(void);

/**
 * Convert address from virtual to physical.
 *
 * @param [in] addr: Virtual address.
 *
 * @return Physical address.
 */

void *mpcomm_memory_virt_to_phys(void *addr);

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

int mpcomm_send_malloc(void **ptr, size_t size);

/**
 * Send a request to the supervisor to call free() on the ptr.
 *
 * @param [in] ptr: The address of the allocated memory to be
 *                  freed by the supervisor.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_send_free(void *ptr);

/**
 * Send the error number to the supervisor.
 *
 * @param [in] err: Error code.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_send_error(int err);

/**
 * Send the log for printing by supervisor.
 *
 * @param [in] log: Log to be printed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int mpcomm_send_log(char *log);

#endif /* __MODULES_INCLUDE_MPCOMM_MPCOMM_H */
