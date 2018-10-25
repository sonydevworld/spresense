/****************************************************************************
 * modules/lte/include/util/thrdpool.h
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

#ifndef __MODULES_LTE_INCLUDE_UTIL_THRDPOOL_H
#define __MODULES_LTE_INCLUDE_UTIL_THRDPOOL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include "osal.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE void (*thrdpool_jobif_t)(FAR void *arg);

struct thrdpool_set_s
{
  uint32_t thrdstacksize;
  int32_t  thrdpriority;
  uint8_t  maxthrdnum;
  uint8_t  maxquenum;
};

struct thrdpool_s
{
  CODE int32_t (*runjob)(
    FAR struct thrdpool_s *thiz, CODE thrdpool_jobif_t job, FAR void *arg);
  CODE uint32_t (*getfreethrds)(FAR struct thrdpool_s *thiz);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: thrdpool_create
 *
 * Description:
 *   Create an object of threadpool and get the instance.
 *
 * Input Parameters:
 *   set  Threadpool create setting.
 *
 * Returned Value:
 *   struct thrdpool_s pointer(i.e. instance of threadpool).
 *
 ****************************************************************************/

FAR struct thrdpool_s *thrdpool_create(FAR struct thrdpool_set_s *set);

/****************************************************************************
 * Name: thrdpool_delete
 *
 * Description:
 *   Delete object of threadpool.
 *
 * Input Parameters:
 *   thiz  struct thrdpool_s pointer(i.e. instance of threadpool).
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t thrdpool_delete(FAR struct thrdpool_s *thiz);

#endif /* __MODULES_LTE_INCLUDE_UTIL_THRDPOOL_H */
