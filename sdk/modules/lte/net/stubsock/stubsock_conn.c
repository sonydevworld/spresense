/****************************************************************************
 * modules/lte/net/stubsock/stubsock_conn.c
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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "devspecsock/devspecsock.h"
#include "stubsock.h"
#include "dbg_if.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct devspecsock_sockif_s g_ds_sockif;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stubsock_alloc()
 ****************************************************************************/

FAR struct stubsock_conn_s *stubsock_alloc(void)
{
  FAR struct stubsock_conn_s *conn;

  conn = (FAR struct stubsock_conn_s*)STUBSOCK_MEM_ALOC
           (sizeof(struct stubsock_conn_s));
  if (conn)
    {
      memset(conn, 0, sizeof(*conn));
      conn->stubsockid = -1;
    }
  else
    {
      DBGIF_LOG_ERROR("Failed to allocate memory\n");
    }

  return conn;
}

/****************************************************************************
 * Name: stubsock_free()
 ****************************************************************************/

void stubsock_free(FAR struct stubsock_conn_s *conn)
{
  STUBSOCK_MEM_FREE(conn);
}

/****************************************************************************
 * Name: stubsock_initialize()
 ****************************************************************************/

void stubsock_initialize(void)
{
  STUBSOCK_MEM_INIT();

  g_ds_sockif.sockif        = &g_stubsock_sockif;
  g_ds_sockif.si_getsockopt = stubsock_getsockopt;
  g_ds_sockif.si_setsockopt = stubsock_setsockopt;

  devspecsock_register(&g_ds_sockif);
}

/****************************************************************************
 * Name: stubsock_finalize()
 ****************************************************************************/

void stubsock_finalize(void)
{
  STUBSOCK_MEM_FIN();
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
