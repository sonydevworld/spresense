/****************************************************************************
 * modules/lte/altcom/api/apiutil.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>

#include "dbg_if.h"
#include "thrdfctry.h"
#include "osal.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_runjob
 *
 * Description:
 *  run job to the worker.
 *
 * Input Parameters:
 *  id   workerid
 *  job  workin job pointer.
 *  arg  job parameters pointer.
 *
 * Returned Value:
 *   If the process succeeds, it returns APIUTIL_SUCCESS.
 *   Otherwise APIUTIL_FAILURE is returned.
 *
 ****************************************************************************/

int32_t altcom_runjob(
  int8_t id,  CODE thrdpool_jobif_t job, FAR void *arg)
{
  int32_t               ret;
  FAR struct thrdpool_s *pool = NULL;

  if (!job)
    {
      DBGIF_LOG_ERROR("NULL parameter.\n");
      return -EINVAL;
    }

  pool = thrdfctry_getwrkr(id);
  if (!pool)
    {
      DBGIF_LOG_ERROR("thrdfctry_getwrkr()\n");
      return -EINVAL;;
    }

  ret = pool->runjob(pool, job, arg);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("runjob() [errno=%d]\n", ret);
      return ret;
    }

  return 0;
}
