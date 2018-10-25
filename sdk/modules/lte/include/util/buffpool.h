/****************************************************************************
 * modules/lte/include/util/buffpool.h
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

#ifndef __MODULES_LTE_INCLUDE_UTIL_BUFFPOOL_H
#define __MODULES_LTE_INCLUDE_UTIL_BUFFPOOL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include "osal.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct buffpool_blockset_s
{
  uint32_t size;
  uint16_t num;
};

typedef FAR void *buffpool_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: buffpool_create
 *
 * Description:
 *   Create the bufferpool.
 *
 * Input Parameters:
 *   set     List of size and number for creating the buffer.
 *   setnum  Number of @set.
 *
 * Returned Value:
 *   If the process succeeds, it returns Object of buffpool.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

buffpool_t buffpool_create(
  FAR struct buffpool_blockset_s set[], uint8_t setnum);

/****************************************************************************
 * Name: buffpool_delete
 *
 * Description:
 *   Delete the bufferpool.
 *
 * Input Parameters:
 *   thiz  Object of bufferpool.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t buffpool_delete(buffpool_t thiz);

/****************************************************************************
 * Name: buffpool_alloc
 *
 * Description:
 *   Allocate buffer from bufferpool.
 *   This function is blocking.
 *
 * Input Parameters:
 *   thiz     Object of bufferpool.
 *   reqsize  Buffer size.
 *
 * Returned Value:
 *   Buffer address.
 *   If can't get available buffer, returned NULL.
 *
 ****************************************************************************/

FAR void *buffpool_alloc(buffpool_t thiz, uint32_t reqsize);

/****************************************************************************
 * Name: buffpool_free
 *
 * Description:
 *   Free the allocated buffer from bufferpool.
 *
 * Input Parameters:
 *   thiz  Object of bufferpool.
 *   buff  Buffer address.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   And when request buffer is NULL to returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t buffpool_free(buffpool_t thiz, FAR void *buff);

#endif /* __MODULES_LTE_INCLUDE_UTIL_BUFFPOOL_H */
