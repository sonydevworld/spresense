/****************************************************************************
 * modules/lte/net/netdb/getaddrinfo.c
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

#if defined(CONFIG_NET) && defined(CONFIG_LTE_NETDB)

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <netdb.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "altcom_socket.h"
#include "altcom_netdb.h"
#include "altcom_errno.h"
#include "stubsock.h"
#include "dbg_if.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getaddrinfo
 *
 * Description:
 *   Given node and service, which identify an Internet host and a service,
 *   getaddrinfo() returns one or more addrinfo structures, each of
 *   which contains an Internet address that can be specified in a call to
 *   bind() or connect(). The getaddrinfo() is reentrant
 *   and allows programs to eliminate IPv4-versus-IPv6 dependencies. 
 *
 * Parameters:
 *   nodename - Specifies either a numerical network address, or a network
 *              hostname, whose network addresses are looked up and resolved
 *   servname - Sets the port in each returned address structure
 *   hints - Points to an addrinfo structure that specifies criteria for
 *           selecting the socket address structures returned in the list
 *           pointed to by res
 *   res - Pointer to the start of the list
 *
 * Returned Value:
 *  getaddrinfo() returns 0 if it succeeds, or one of the following
 *  nonzero error codes:
 *
 *     EAI_NONAME
 *     EAI_SERVICE
 *     EAI_FAIL
 *     EAI_MEMORY
 *     EAI_FAMILY
 *
 ****************************************************************************/

int getaddrinfo(const char *nodename, const char *servname,
                const struct addrinfo *hints, struct addrinfo **res)
{
  int                             ret;
  struct altcom_addrinfo         *ai;
  struct altcom_sockaddr_storage  ss;
  struct altcom_addrinfo          hint;
  FAR struct altcom_addrinfo     *phints = NULL;

  if (hints)
    {
      hint.ai_flags    = stubsock_convaiflags_remote(hints->ai_flags);
      hint.ai_family   = stubsock_convdomain_remote(hints->ai_family);
      hint.ai_socktype = stubsock_convtype_remote(hints->ai_socktype);
      hint.ai_protocol = stubsock_convproto_remote(hints->ai_protocol);

      phints = &hint;
    }

  ret = altcom_getaddrinfo(nodename, servname, phints,
                           (struct altcom_addrinfo**)res);
  if (ret == 0)
    {
      ai = (struct altcom_addrinfo*)(*res);

      while(ai)
        {
          ai->ai_family   = stubsock_convdomain_local(ai->ai_family);
          DBGIF_ASSERT(ai->ai_family != -EAFNOSUPPORT, "Invalid ai_family\n");

          ai->ai_socktype = stubsock_convtype_local(ai->ai_socktype);
          DBGIF_ASSERT(ai->ai_socktype != -EPROTONOSUPPORT, "Invalid ai_socktype\n");

          ai->ai_protocol = stubsock_convproto_local(ai->ai_protocol);
          DBGIF_ASSERT(ai->ai_protocol != -EPROTONOSUPPORT, "Invalid ai_protocol\n");


          if (ai->ai_addrlen <= sizeof(struct altcom_sockaddr_storage))
            {
              /* temporarily evacuate to the working memory */

              memcpy(&ss, ai->ai_addr,
                     sizeof(struct altcom_sockaddr_storage));

              stubsock_convstorage_local((FAR struct altcom_sockaddr_storage*)ai->ai_addr,
                                         (FAR struct sockaddr*)&ss);

              memcpy(ai->ai_addr, &ss, ai->ai_addrlen);

              if (ai->ai_family == ALTCOM_AF_INET)
                {
                  ai->ai_addrlen = sizeof(struct sockaddr_in);
                }
              else
                {
                  ai->ai_addrlen = sizeof(struct sockaddr_in6);
                }
            }
          ai = ai->ai_next;
        }
    }
  else
    {
      /* Convert return code */

      switch(ret)
        {
          case ALTCOM_EAI_NONAME:
            ret = EAI_NONAME;
            break;
          case ALTCOM_EAI_SERVICE:
            ret = EAI_SERVICE;
            break;
          case ALTCOM_EAI_FAIL:
            ret = EAI_SYSTEM;
            break;
          case ALTCOM_EAI_MEMORY:
            ret = EAI_MEMORY;
            break;
          case ALTCOM_EAI_FAMILY:
            ret = EAI_FAMILY;
            break;
          default:
            ret = EAI_SYSTEM;
            break;
        }
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_LTE_NETDB */
