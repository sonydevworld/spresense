/****************************************************************************
 * modules/lte/net/stubsock/stubsock_accept.c
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
 * Name: stubsock_accept
 *
 * Description:
 *   The stubsock_accept function is used with connection-based socket
 *   types (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an inet_accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, inet_accept blocks the caller until a
 *   connection is present. If the socket is marked non-blocking and no
 *   pending connections are present on the queue, inet_accept returns
 *   EAGAIN.
 *
 * Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *   newsock  Location to return the accepted socket information.
 *
 * Returned Value:
 *   Returns 0 (OK) on success.  On failure, it returns a negated errno
 *   value.  See accept() for a desrciption of the approriate error value.
 *
 ****************************************************************************/

int stubsock_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                    FAR socklen_t *addrlen, FAR struct socket *newsock)
{
  FAR struct devspecsock_conn_s  *ds_conn =
    (FAR struct devspecsock_conn_s*)psock->s_conn;
  FAR struct devspecsock_conn_s  *new_ds_conn =
    (FAR struct devspecsock_conn_s*)newsock->s_conn;
  FAR struct stubsock_conn_s     *conn = ds_conn->devspec_conn;
  FAR struct stubsock_conn_s     *newconn;
  int                             ret = OK;
  int                             sockfd;
  int                             newsockfd;
  altcom_socklen_t                altcom_addrlen;
  socklen_t                       output_addrlen;
  struct sockaddr_in6             tmpaddr;
  struct altcom_sockaddr_storage  storage;
  int                             val = 0;
#ifdef CONFIG_NET_SOCKOPTS
  struct timeval                  tv_val;
#endif

  DBGIF_ASSERT(conn, "conn == NULL\n");

  if (addr != NULL)
    {
      if ((addrlen == NULL) || (*addrlen == 0))
        {
          return -EINVAL;
        }
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

  if (_SS_ISNONBLOCK(psock->s_flags))
    {
      val |= ALTCOM_O_NONBLOCK;
    }

  /* Whether it is blocking or not,
   * change the behavior of sokcet with fcntl */

  altcom_fcntl(sockfd, ALTCOM_SETFL, val);

#ifdef CONFIG_NET_SOCKOPTS
  tv_val.tv_sec  = psock->s_rcvtimeo / DSEC_PER_SEC;
  tv_val.tv_usec = (psock->s_rcvtimeo % DSEC_PER_SEC) * USEC_PER_DSEC;

  altcom_setsockopt(sockfd, ALTCOM_SOL_SOCKET, ALTCOM_SO_RCVTIMEO, &tv_val,
                    sizeof(tv_val));
#endif

  newsockfd = altcom_accept(sockfd, (FAR struct altcom_sockaddr*)&storage,
                            &altcom_addrlen);
  if (newsockfd < 0)
    {
      ret = altcom_errno();
      ret = -ret;
    }
  else if (addr != NULL)
    {
      /* Convert remote address to NuttX */

      stubsock_convstorage_local(&storage, (FAR struct sockaddr*)&tmpaddr);

      /* This function is supposed to return the partial address if
       * a smaller buffer has been provided. */

      memcpy(addr, &tmpaddr, *addrlen);

      *addrlen = output_addrlen;
    }

  if (newsockfd >= 0)
    {
      /* Allocate the stubsock socket connection structure and save in the new
       * socket instance.
       */

      newconn = stubsock_alloc();
      if (!newconn)
        {
          altcom_close(newsockfd);

          /* Failed to reserve a connection structure */

          return -ENOMEM;
        }

      new_ds_conn->devspec_conn = newconn;

      newconn->stubsockid = newsockfd;
    }

  return ret;
}

#endif /* CONFIG_NET */
