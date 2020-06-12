/****************************************************************************
 * modules/lte/altcom/include/evtdisp/buffpoolwrapper.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_EVTDISP_BUFFPOOLWRAPPER_H
#define __MODULES_LTE_ALTCOM_INCLUDE_EVTDISP_BUFFPOOLWRAPPER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include "buffpool.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUFFPOOL_ALLOC(reqsize) \
    (buffpoolwrapper_alloc(reqsize))

#define BUFFPOOL_FREE(buff) (buffpoolwrapper_free(buff))

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern buffpool_t g_buffpoolwrapper_obj;

/****************************************************************************
 * Inline functions
 ****************************************************************************/

FAR static inline void * buffpoolwrapper_alloc(uint32_t reqsize)
{

#ifdef CONFIG_LTE_USE_BUFFPOOL

  return buffpool_alloc(g_buffpoolwrapper_obj, reqsize);

#else
  FAR void *ptr;

  ptr = SYS_MALLOC(reqsize);
  if (ptr)
    {
      memset(ptr, 0, reqsize);
    }
  return ptr;

#endif

}

static inline int32_t buffpoolwrapper_free(FAR void *buff)
{

#ifdef CONFIG_LTE_USE_BUFFPOOL

  return buffpool_free(g_buffpoolwrapper_obj, buff);

#else

  SYS_FREE(buff);
  return 0;

#endif

}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: buffpoolwrapper_init
 *
 * Description:
 *   Initialize the bufferpool
 *
 * Input Parameters:
 *   set     List of size and number for creating the buffer.
 *   setnum  Number of @set.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t buffpoolwrapper_init(
  FAR struct buffpool_blockset_s set[], uint8_t setnum);

/****************************************************************************
 * Name: buffpoolwrapper_fin
 *
 * Description:
 *   Finalize the bufferpool.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t buffpoolwrapper_fin(void);

#endif
