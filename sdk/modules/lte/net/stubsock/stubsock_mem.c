/****************************************************************************
 * modules/lte/net/stubsock/stubsock_mem.c
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

#include <nuttx/config.h>
#include <sdk/config.h>

#if defined(CONFIG_NET) && defined(CONFIG_NET_DEV_SPEC_SOCK)

#include "stubsock.h"
#include "dbg_if.h"
#include "buffpool.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLOCKSETLIST_NUM (sizeof(g_stubsock_blk_settings) / sizeof(g_stubsock_blk_settings[0]))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct buffpool_blockset_s g_stubsock_blk_settings[] =
{
  {
      32, 20
  }
};

FAR buffpool_t g_stubsockbuffpl_obj = NULL;

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

void stubsock_mem_initialize(void)
{
  FAR struct buffpool_blockset_s *pset = &g_stubsock_blk_settings[0];

  g_stubsockbuffpl_obj = buffpool_create(pset, BLOCKSETLIST_NUM);
  if (!g_stubsockbuffpl_obj)
    {
      DBGIF_LOG_ERROR("Failed to create buffpool\n");
    }
}

/****************************************************************************
 * Name: stubsock_mem_finalize()
 *
 * Description:
 *   Finalize the memory for stub socket.
 *
 ****************************************************************************/

void stubsock_mem_finalize(void)
{
  int32_t ret;

  if (g_stubsockbuffpl_obj)
    {
      ret = buffpool_delete(g_stubsockbuffpl_obj);
      if (0 > ret)
        {
          DBGIF_LOG_ERROR("Delete buffpool object failed.\n");
          return;
        }
      g_stubsockbuffpl_obj = NULL;
    }
}

/****************************************************************************
 * Name: stubsock_mem_alloc()
 *
 * Description:
 *   Allocate the memory for stub socket.
 *
 ****************************************************************************/

FAR void *stubsock_mem_alloc(uint32_t reqsize)
{
  return buffpool_alloc(g_stubsockbuffpl_obj, reqsize);
}

/****************************************************************************
 * Name: stubsock_mem_free()
 *
 * Description:
 *   Free the memory for stub socket.
 *
 ****************************************************************************/

void stubsock_mem_free(FAR void *mem)
{
  buffpool_free(g_stubsockbuffpl_obj, mem);
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
