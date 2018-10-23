/****************************************************************************
 * modules/lte/net/stubsock/stubsock_setsockopt.c
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
 * Name: stubsock_setsockopt
 *
 * Description:
 *   stubsock_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the socket on the 'psock' argument.
 *
 *   The 'level' argument specifies the protocol level of the option. To set
 *   options at the socket level, specify the level argument as SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Parameters:
 *   psock     Socket instance
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 ****************************************************************************/

int stubsock_setsockopt(FAR struct socket *psock, int level, int option,
                        FAR const void *value, FAR socklen_t value_len)
{
  FAR struct devspecsock_conn_s *ds_conn =
    (FAR struct devspecsock_conn_s*)psock->s_conn;
  FAR struct stubsock_conn_s    *conn = ds_conn->devspec_conn;
  int32_t                        sockfd;
  int32_t                        ret = 0;
  int32_t                        lev;
  int32_t                        opt;

  DBGIF_ASSERT(conn, "conn == NULL\n");

  sockfd = conn->stubsockid;

  switch(level)
    {
      case SOL_SOCKET:
        lev = ALTCOM_SOL_SOCKET;

        switch(option)
          {
            case SO_BROADCAST:
              opt = ALTCOM_SO_BROADCAST;
              break;
            case SO_REUSEADDR:
              opt = ALTCOM_SO_REUSEADDR;
              break;
            case SO_KEEPALIVE:
              opt = ALTCOM_SO_KEEPALIVE;
              break;
            case SO_RCVBUF:
              opt = ALTCOM_SO_RCVBUF;
              break;

#ifdef CONFIG_NET_SOLINGER
            case SO_LINGER:
              opt = ALTCOM_SO_LINGER;
              /* linger structure is same as remote */

              break;
#endif

            /* The following are not yet implemented */

            case SO_DEBUG:
            case SO_OOBINLINE:
            case SO_SNDBUF:
            case SO_DONTROUTE:
            case SO_RCVLOWAT:
            case SO_SNDLOWAT:

            /* These options do not reach here */

            case SO_RCVTIMEO:
            case SO_SNDTIMEO:

            /* There options are only valid when used with getopt */

            case SO_ACCEPTCONN:
            case SO_ERROR:
            case SO_TYPE:

            default:
              ret = -ENOPROTOOPT;
              break;
          }
        break;

      default:
        ret = -ENOPROTOOPT;
        break;
    }

  if (ret >= 0)
    {
      ret = altcom_setsockopt(sockfd, lev, opt, value,
                              (altcom_socklen_t)value_len);
      if (ret < 0)
        {
          ret = altcom_errno();
          ret = -ret;
        }
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
