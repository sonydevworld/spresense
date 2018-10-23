/****************************************************************************
 * modules/lte/net/stubsock/stubsock_getsockname.c
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
#include <sys/time.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/net/net.h>

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
 * Name: stubsock_getsockname
 *
 * Description:
 *   The getsockname() function retrieves the locally-bound name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   conn     usrsock socket connection structure
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 ****************************************************************************/

int stubsock_getsockname(FAR struct socket *psock,
                         FAR struct sockaddr *addr, FAR socklen_t *addrlen)
{
  FAR struct devspecsock_conn_s  *ds_conn =
    (FAR struct devspecsock_conn_s*)psock->s_conn;
  FAR struct stubsock_conn_s     *conn = ds_conn->devspec_conn;
  int                             sockfd;
  int                             ret;
  altcom_socklen_t                altcom_addrlen;
  socklen_t                       output_addrlen;
  struct sockaddr_in6             tmpaddr;
  struct altcom_sockaddr_storage  storage;

  DBGIF_ASSERT(conn, "conn == NULL\n");

  if (addr == NULL)
    {
      return -EINVAL;
    }

  if ((addrlen == NULL) || (*addrlen == 0))
    {
      return -EINVAL;
    }

  /* Adjust the length. Because the size of the structure is
   * different between NuttX and remote. */

  if (psock->s_domain == AF_INET)
    {
      altcom_addrlen = sizeof(struct altcom_sockaddr_in);
      output_addrlen = sizeof(struct sockaddr_in);
    }
  else
    {
      altcom_addrlen = sizeof(struct altcom_sockaddr_in6);
      output_addrlen = sizeof(struct sockaddr_in6);
    }
  memset(&storage, 0, sizeof(struct altcom_sockaddr_storage));
  memset(&tmpaddr, 0, sizeof(struct sockaddr_in6));

  sockfd = conn->stubsockid;

  ret = altcom_getsockname(sockfd, (FAR struct altcom_sockaddr*)&storage,
                           &altcom_addrlen);
  if (ret < 0)
    {
      ret = altcom_errno();
      ret = -ret;
    }
  else
    {
      /* Convert remote address to NuttX */

      stubsock_convstorage_local(&storage, (FAR struct sockaddr*)&tmpaddr);

      /* This function is supposed to return the partial address if
       * a smaller buffer has been provided. */

      memcpy(addr, &tmpaddr, *addrlen);

      *addrlen = output_addrlen;
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
