/****************************************************************************
 * modules/lte/net/stubsock/stubsock_socket.c
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

#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "devspecsock/devspecsock.h"
#include "stubsock.h"
#include "altcom_socket.h"
#include "altcom_errno.h"
#include "dbg_if.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stubsock_socket
 ****************************************************************************/

int stubsock_socket(int domain, int type, int protocol,
                    FAR struct socket *psock)
{
  FAR struct devspecsock_conn_s *ds_conn;
  FAR struct stubsock_conn_s    *conn;
  int                            sockfd;
  int                            conv_domain;
  int                            conv_type;
  int                            conv_proto;
  int                            err;

  /* Convert domain to altcom definition */

  conv_domain = stubsock_convdomain_remote(domain);
  if (conv_domain < 0)
    {
      DBGIF_LOG1_ERROR("socket address family unsupported: %d\n", domain);
      err = conv_domain;
      goto errout;
    }

  /* Convert type to altcom definition */

  conv_type = stubsock_convtype_remote(type);
  if (conv_type < 0)
    {
      DBGIF_LOG1_ERROR("ERROR: Unsupported type: %d\n", type);
      err = conv_type;
      goto errout;
    }

  /* Convert protocol to altcom definition */

  conv_proto = stubsock_convproto_remote(protocol);
  if (conv_proto < 0)
    {
      DBGIF_LOG1_ERROR("ERROR: Unsupported protocol: %d\n", type);
      err = conv_proto;
      goto errout;
    }

  /* Allocate the stubsock socket connection structure and save in the new
   * socket instance.
   */

  conn = stubsock_alloc();
  if (!conn)
    {
      /* Failed to reserve a connection structure */

      return -ENOMEM;
    }

  sockfd = altcom_socket(conv_domain, conv_type, conv_proto);
  if (sockfd < 0)
    {
      err = altcom_errno();
      err = -err;
      goto errout_free_conn;
    }

  ds_conn               = psock->s_conn;
  ds_conn->devspec_conn = conn;
  conn->stubsockid      = sockfd;

  return OK;

errout_free_conn:
  stubsock_free(conn);
errout:

  return err;
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
