/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_sock.c
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
#include "altcom_sock.h"
#include "altcom_socket.h"
#include "altcom_in.h"
#include "cc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct altcom_socket_s g_altcom_sockets[ALTCOM_NSOCKET];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_sockfd_socket
 ****************************************************************************/

FAR struct altcom_socket_s *altcom_sockfd_socket(int sockfd)
{
  if (sockfd >= 0 && sockfd < ALTCOM_NSOCKET)
    {
      return &g_altcom_sockets[sockfd];
    }

  return NULL;
}

/****************************************************************************
 * Name: altcom_sockaddr_to_sockstorage
 ****************************************************************************/

void altcom_sockaddr_to_sockstorage(FAR const struct altcom_sockaddr *addr,
                                    FAR struct altcom_sockaddr_storage *storage)
{
  FAR struct altcom_sockaddr_in  *inaddr;
  FAR struct altcom_sockaddr_in6 *in6addr;

  if (addr->sa_family == ALTCOM_PF_INET)
    {
      inaddr = (FAR struct altcom_sockaddr_in*)addr;

      /* These parameter are network byte order */

      memcpy((FAR struct altcom_sockaddr_in*)storage, inaddr,
             sizeof(struct altcom_sockaddr_in));
    }
  else if (addr->sa_family == ALTCOM_PF_INET6)
    {
      in6addr = (FAR struct altcom_sockaddr_in6*)addr;

      /* These parameter are network byte order */

      memcpy((FAR struct altcom_sockaddr_in6*)storage, in6addr,
             sizeof(struct altcom_sockaddr_in6));
    }
}
