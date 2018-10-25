/****************************************************************************
 * modules/lte/net/stubsock/stubsock_recvfrom.c
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
 * Name: stubsock_recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags (ignored)
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 ****************************************************************************/

ssize_t stubsock_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                          int flags, FAR struct sockaddr *from,
                          FAR socklen_t *fromlen)
{
  FAR struct devspecsock_conn_s  *ds_conn =
    (FAR struct devspecsock_conn_s*)psock->s_conn;
  FAR struct stubsock_conn_s     *conn = ds_conn->devspec_conn;
  int                             sockfd;
  int                             ret;
  int                             val = 0;
  altcom_socklen_t                altcom_fromlen;
  socklen_t                       output_fromlen;
  struct sockaddr_in6             tmpaddr;
  struct altcom_sockaddr_storage  storage;
#ifdef CONFIG_NET_SOCKOPTS
  struct timeval                  tv_val;
#endif
  

  DBGIF_ASSERT(conn, "conn == NULL\n");

  if (from != NULL)
    {
      if ((fromlen == NULL) || (*fromlen == 0))
        {
          return -EINVAL;
        }
    }

  /* Adjust the length. Because the size of the structure is
   * different between NuttX and remote. */

  if (psock->s_domain == AF_INET)
    {
      altcom_fromlen = sizeof(struct altcom_sockaddr_in);
      output_fromlen = sizeof(struct sockaddr_in);
    }
  else
    {
      altcom_fromlen = sizeof(struct altcom_sockaddr_in6);
      output_fromlen = sizeof(struct sockaddr_in6);
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
  tv_val.tv_sec = psock->s_rcvtimeo / DSEC_PER_SEC;
  tv_val.tv_usec = (psock->s_rcvtimeo % DSEC_PER_SEC) * USEC_PER_DSEC;

  altcom_setsockopt(sockfd, ALTCOM_SOL_SOCKET, ALTCOM_SO_RCVTIMEO, &tv_val,
                    sizeof(tv_val));
#endif

  ret = altcom_recvfrom(sockfd, buf, len,
                        stubsock_convflags_remote(flags),
                        (FAR struct altcom_sockaddr *)&storage,
                        &altcom_fromlen);
  if (ret < 0)
    {
      ret = altcom_errno();
      ret = -ret;
    }
  else if (from != NULL)
    {
      /* Convert remote address to NuttX */

      stubsock_convstorage_local(&storage, (FAR struct sockaddr*)&tmpaddr);

      /* This function is supposed to return the partial address if
       * a smaller buffer has been provided. */

      memcpy(from, &tmpaddr, *fromlen);

      *fromlen = output_fromlen;
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
