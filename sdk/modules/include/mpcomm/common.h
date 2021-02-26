/****************************************************************************
 * mpcomm/common/common.h
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
 * @file common.h
 */

#ifndef __MODULES_INCLUDE_MPCOMM_COMMON_H
#define __MODULES_INCLUDE_MPCOMM_COMMON_H

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

/** Message ID to send error number to supervisor. */

#define MPCOMM_MSG_ID_ERROR 7

/** Maximum number of helpers. */

#define MPCOMM_MAX_HELPERS 4

/** Maximum CPU ID. */

#define MPCOMM_CPUID_MAX 8

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct controller_init_msg
 *
 * Structure for controller initialization.
 *
 * @typedef controller_init_msg_t
 * See @ref controller_init_msg
 */

typedef struct controller_init_msg
{
  /** Set of CPUs used as a helper. See @ref cpu_set_t */

  cpu_set_t helpers_cpuset;

  /** Controller load address. */

  uintptr_t loadaddr;
} controller_init_msg_t;

/**
 * @struct helper_init_msg
 *
 * Structure for helper initialization.
 *
 * @typedef helper_init_msg_t
 * See @ref helper_init_msg
 */

typedef struct helper_init_msg
{
  /** Controller CPU ID. See @ref cpuid_t */

  cpuid_t controller_cpuid;

  /** Helper load address. */

  uintptr_t loadaddr;
} helper_init_msg_t;

/**
 * @struct supervisor_malloc_msg
 *
 * Structure for malloc() request.
 *
 * @typedef supervisor_malloc_msg_t
 * See @ref supervisor_malloc_msg
 */

typedef struct supervisor_malloc_msg
{
  /** The address of the allocated memory received from the supervisor. */

  void **ptr;

  /** Requested size. */

  size_t size;
} supervisor_malloc_msg_t;

/**
 * @struct supervisor_free_msg
 *
 * Structure for free() request.
 *
 * @typedef supervisor_free_msg_t
 * See @ref supervisor_free_msg
 */

typedef struct supervisor_free_msg
{
  /** The address of the allocated memory to be freed by the supervisor. */

  void *ptr;
} supervisor_free_msg_t;

#endif /* __MODULES_INCLUDE_MPCOMM_COMMON_H */
