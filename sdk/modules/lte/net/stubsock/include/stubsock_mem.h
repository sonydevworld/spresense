/****************************************************************************
 * modules/lte/net/stubsock/include/stubsock_mem.h
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

#ifndef __MODULES_LTE_NET_STUBSOCK_INCLUDE_STUBSOCK_MEM_H
#define __MODULES_LTE_NET_STUBSOCK_INCLUDE_STUBSOCK_MEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sdk/config.h>

#ifdef CONFIG_NET

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STUBSOCK_MEM_INIT     stubsock_mem_initialize
#define STUBSOCK_MEM_FIN      stubsock_mem_finalize
#define STUBSOCK_MEM_ALOC     stubsock_mem_alloc
#define STUBSOCK_MEM_FREE     stubsock_mem_free

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stubsock_mem_initialize()
 *
 * Description:
 *   Initialize the memory for stub socket.
 *
 ****************************************************************************/

void stubsock_mem_initialize(void);

/****************************************************************************
 * Name: stubsock_mem_finalize()
 *
 * Description:
 *   Finalize the memory for stub socket.
 *
 ****************************************************************************/

void stubsock_mem_finalize(void);

/****************************************************************************
 * Name: stubsock_mem_alloc()
 *
 * Description:
 *   Allocate the memory for stub socket.
 *
 ****************************************************************************/

FAR void *stubsock_mem_alloc(uint32_t reqsize);

/****************************************************************************
 * Name: stubsock_mem_free()
 *
 * Description:
 *   Free the memory for stub socket.
 *
 ****************************************************************************/

void stubsock_mem_free(FAR void *mem);

#endif

#endif /* __MODULES_LTE_NET_STUBSOCK_INCLUDE_STUBSOCK_H */
