/****************************************************************************
 * mpcomm/helper/helper.h
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
 * @file helper.h
 */

#ifndef __MODULES_INCLUDE_MPCOMM_HELPER_H
#define __MODULES_INCLUDE_MPCOMM_HELPER_H

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

#define MEM_V2P(addr) helper_memory_virt_to_phys(addr)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Definition of callback function.
 *
 *  User-defined callback that is called by helper_main()
 *  after receiving data from the controller.
 *
 * @param[in] data : User data received from the controller.
 */

typedef void (*helper_user_func_t)(void *data);

/**
 * @struct helper_context
 *
 * Structure of information needed by a helper in the MPCOMM framework.
 *
 * @typedef helper_context_t
 * See @ref helper_context
 */

typedef struct helper_context
{
  /** MP message queue to supervisor. See @ref mpmq_t */

  mpmq_t mq_2_supervisor;

  /** MP message queue to controller. See @ref mpmq_t */

  mpmq_t mq_2_controller;

  /** User-defined callback. See @ref helper_user_func_t */

  helper_user_func_t user_func;

  /** Helper load address. */

  uintptr_t loadaddr;

  /** If 0, the helper should stay in the loop and if 1 it should quit. */

  uint8_t quit_loop;
} helper_context_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * Start the MPCOMM framework on the helper. The user can register a callback
 * that will handle the data received from the controller.
 *
 * @param [in] user_func: User-defined callback.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int helper_main(helper_user_func_t user_func);

/**
 * Convert address from virtual to physical.
 *
 * @param [in] addr: Virtual address.
 *
 * @return Physical address.
 */

void *helper_memory_virt_to_phys(void *addr);

#endif /* __MODULES_INCLUDE_MPCOMM_HELPER_H */
