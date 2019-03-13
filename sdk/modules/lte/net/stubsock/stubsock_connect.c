/****************************************************************************
 * modules/lte/net/stubsock/stubsock_connect.c
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
#include <netinet/in.h>

#include <string.h>

#include "socket/socket.h"
#include "devspecsock/devspecsock.h"
#include "stubsock.h"
#include "altcom_socket.h"
#include "altcom_in.h"
#include "altcom_errno.h"
#include "dbg_if.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stubsock_connect
 ****************************************************************************/

int stubsock_connect(FAR struct socket *psock,
                     FAR const struct sockaddr *addr,
                     socklen_t addrlen)
{
  FAR struct devspecsock_conn_s  *ds_conn =
    (FAR struct devspecsock_conn_s*)psock->s_conn;
  FAR struct stubsock_conn_s     *conn = ds_conn->devspec_conn;
  struct altcom_sockaddr_storage  storage;
  altcom_socklen_t                addr_len;
  int                             sockfd;
  int                             ret;
  int                             val = 0;

  DBGIF_ASSERT(conn, "conn == NULL\n");

  sockfd = conn->stubsockid;

  /* Check addrlen */

  if (addr->sa_family == AF_INET)
    {
      if (addrlen != sizeof(struct sockaddr_in))
        {
          return -EINVAL;
        }
      else
        {
          /* Adjust the length. Because the size of the structure is
           * different between local and remote. */

          addr_len = sizeof(struct altcom_sockaddr_in);
        }
    }
  else if (addr->sa_family == AF_INET6)
    {
      if (addrlen != sizeof(struct sockaddr_in6))
        {
          return -EINVAL;
        }
      else
        {
          /* Adjust the length. Because the size of the structure is
           * different between local and remote. */

          addr_len = sizeof(struct altcom_sockaddr_in6);
        }
    }
  else
    {
      return -EINVAL;
    }

  if (_SS_ISNONBLOCK(psock->s_flags))
    {
      val |= ALTCOM_O_NONBLOCK;
    }

  /* Whether it is blocking or not,
   * change the behavior of sokcet with fcntl */

  altcom_fcntl(sockfd, ALTCOM_SETFL, val);

  /* Convert to altcom's sockaddr structure */

  memset(&storage, 0, sizeof(struct altcom_sockaddr_storage));
  stubsock_convsockaddr_remote(addr, &storage);

  ret = altcom_connect(sockfd, (FAR const struct altcom_sockaddr*)&storage,
                       addr_len);
  if (ret < 0)
    {
      ret = altcom_errno();
      ret = -ret;
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
