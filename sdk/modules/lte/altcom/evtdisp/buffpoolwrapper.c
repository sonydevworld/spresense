/****************************************************************************
 * modules/lte/altcom/evtdisp/buffpoolwrapper.c
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

#include "buffpoolwrapper.h"
#include "dbg_if.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR buffpool_t g_buffpoolwrapper_obj = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: buffpoolwrapper_init
 *
 * Description:
 *   Initialize the buffpoolwrapper.
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
  FAR struct buffpool_blockset_s set[], uint8_t setnum)
{

  /* Create buffpool only if CONFIG_LTE_USE_BUFFPOOL is enabled. */

#ifdef CONFIG_LTE_USE_BUFFPOOL

  if (g_buffpoolwrapper_obj)
    {
      DBGIF_LOG_ERROR("buffpool object has already been created.\n");
      return -EPERM;
    }

  g_buffpoolwrapper_obj = buffpool_create(set, setnum);
  if (!g_buffpoolwrapper_obj)
    {
      return -errno;
    }

#endif

  return 0;
}

/****************************************************************************
 * Name: buffpoolwrapper_fin
 *
 * Description:
 *   Finalize the buffpoolwrapper.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise BUFFPOOL_FAILURE is returned.
 *
 ****************************************************************************/

int32_t buffpoolwrapper_fin(void)
{

#ifdef CONFIG_LTE_USE_BUFFPOOL

  int32_t ret = 0;

  if (!g_buffpoolwrapper_obj)
    {
      DBGIF_LOG_ERROR("buffpool object has not been created.\n");
      return -EPERM;
    }

  ret = buffpool_delete(g_buffpoolwrapper_obj);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Delete buffpool object failed.\n");
      return ret;
    }

  g_buffpoolwrapper_obj = NULL;

#endif

  return 0;
}
