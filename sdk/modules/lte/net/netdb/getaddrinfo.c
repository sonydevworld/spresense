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
#include "altcom_in.h"
#include "dbg_if.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: convaiflags_remote
 ****************************************************************************/

static int convaiflags_remote(int ai_flags)
{
  int ret = 0;

  if (ai_flags & AI_PASSIVE)
    {
      ret |= ALTCOM_AI_PASSIVE;
    }

  if (ai_flags & AI_CANONNAME)
    {
      ret |= ALTCOM_AI_CANONNAME;
    }

  if (ai_flags & AI_NUMERICHOST)
    {
      ret |= ALTCOM_AI_NUMERICHOST;
    }

  if (ai_flags & AI_NUMERICSERV)
    {
      ret |= ALTCOM_AI_NUMERICSERV;
    }

  if (ai_flags & AI_V4MAPPED)
    {
      ret |= ALTCOM_AI_V4MAPPED;
    }

  if (ai_flags & AI_ALL)
    {
      ret |= ALTCOM_AI_ALL;
    }

  if (ai_flags & AI_ADDRCONFIG)
    {
      ret |= ALTCOM_AI_ADDRCONFIG;
    }

  return ret;
}

/****************************************************************************
 * Name: convdomain_remote
 ****************************************************************************/

static int convdomain_remote(int domain)
{
  int ret;

  switch(domain)
    {
      case PF_UNSPEC:
        ret = ALTCOM_PF_UNSPEC;
        break;

      case PF_INET:
        ret = ALTCOM_PF_INET;
        break;

      case PF_INET6:
        ret = ALTCOM_PF_INET6;
        break;

      default:
        ret = -EAFNOSUPPORT;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: convtype_remote
 ****************************************************************************/

static int convtype_remote(int type)
{
  int ret;

  switch(type)
    {
      case SOCK_STREAM:
        ret = ALTCOM_SOCK_STREAM;
        break;

      case SOCK_DGRAM:
        ret = ALTCOM_SOCK_DGRAM;
        break;

      case SOCK_RAW:
        ret = ALTCOM_SOCK_RAW;
        break;

      default:
        ret = -EPROTONOSUPPORT;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: convproro_remote
 ****************************************************************************/

static int convproto_remote(int protocol)
{
  int ret;

  switch(protocol)
    {
      case IPPROTO_IP:
        ret = ALTCOM_IPPROTO_IP;
        break;

      case IPPROTO_ICMP:
        ret = ALTCOM_IPPROTO_ICMP;
        break;

      case IPPROTO_TCP:
        ret = ALTCOM_IPPROTO_TCP;
        break;

      case IPPROTO_UDP:
        ret = ALTCOM_IPPROTO_UDP;
        break;

      case IPPROTO_IPV6:
        ret = ALTCOM_IPPROTO_IPV6;
        break;

      case IPPROTO_ICMP6:
        ret = ALTCOM_IPPROTO_ICMPV6;
        break;

      case IPPROTO_UDPLITE:
        ret = ALTCOM_IPPROTO_UDPLITE;
        break;

      case IPPROTO_RAW:
        ret = ALTCOM_IPPROTO_RAW;
        break;

      default:
        ret = -EPROTONOSUPPORT;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: convstorage_local
 ****************************************************************************/

static void convstorage_local(FAR const struct altcom_sockaddr_storage *from,
                                FAR struct sockaddr *to)
{
  FAR struct altcom_sockaddr_in  *inaddr_from;
  FAR struct altcom_sockaddr_in6 *in6addr_from;
  FAR struct sockaddr_in         *inaddr_to;
  FAR struct sockaddr_in6        *in6addr_to;

  if (from->ss_family == ALTCOM_AF_INET)
    {
      inaddr_from = (FAR struct altcom_sockaddr_in*)from;
      inaddr_to   = (FAR struct sockaddr_in*)to;

      inaddr_to->sin_family = AF_INET;
      inaddr_to->sin_port   = inaddr_from->sin_port;
      memcpy(&inaddr_to->sin_addr, &inaddr_from->sin_addr,
             sizeof(struct in_addr));
    }
  else if (from->ss_family == ALTCOM_AF_INET6)
    {
      in6addr_from = (FAR struct altcom_sockaddr_in6*)from;
      in6addr_to   = (FAR struct sockaddr_in6*)to;

      in6addr_to->sin6_family = AF_INET6;
      in6addr_to->sin6_port   = in6addr_from->sin6_port;
      memcpy(&in6addr_to->sin6_addr, &in6addr_from->sin6_addr,
             sizeof(struct in6_addr));
    }
}

/****************************************************************************
 * Name: convdomain_local
 ****************************************************************************/

static int convdomain_local(int domain)
{
  int ret;

  switch(domain)
    {
      case ALTCOM_PF_UNSPEC:
        ret = PF_UNSPEC;
        break;

      case ALTCOM_PF_INET:
        ret = PF_INET;
        break;

      case ALTCOM_PF_INET6:
        ret = PF_INET6;
        break;

      default:
        ret = -EAFNOSUPPORT;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: convtype_local
 ****************************************************************************/

static int convtype_local(int type)
{
  int ret;

  switch(type)
    {
      case ALTCOM_SOCK_STREAM:
        ret = SOCK_STREAM;
        break;

      case ALTCOM_SOCK_DGRAM:
        ret = SOCK_DGRAM;
        break;

      case ALTCOM_SOCK_RAW:
        ret = SOCK_RAW;
        break;

      default:
        ret = -EPROTONOSUPPORT;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: convproto_local
 ****************************************************************************/

static int convproto_local(int protocol)
{
  int ret;

  switch(protocol)
    {
      case ALTCOM_IPPROTO_IP:
        ret = IPPROTO_IP;
        break;

      case ALTCOM_IPPROTO_ICMP:
        ret = IPPROTO_ICMP;
        break;

      case ALTCOM_IPPROTO_TCP:
        ret = IPPROTO_TCP;
        break;

      case ALTCOM_IPPROTO_UDP:
        ret = IPPROTO_UDP;
        break;

      case ALTCOM_IPPROTO_IPV6:
        ret = IPPROTO_IPV6;
        break;

      case ALTCOM_IPPROTO_ICMPV6:
        ret = IPPROTO_ICMP6;
        break;

      case ALTCOM_IPPROTO_UDPLITE:
        ret = IPPROTO_UDPLITE;
        break;

      case ALTCOM_IPPROTO_RAW:
        ret = IPPROTO_RAW;
        break;

      default:
        ret = -EPROTONOSUPPORT;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: getaddrinfo
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
      hint.ai_flags    = convaiflags_remote(hints->ai_flags);
      hint.ai_family   = convdomain_remote(hints->ai_family);
      hint.ai_socktype = convtype_remote(hints->ai_socktype);
      hint.ai_protocol = convproto_remote(hints->ai_protocol);

      phints = &hint;
    }

  ret = altcom_getaddrinfo(nodename, servname, phints,
                           (struct altcom_addrinfo**)res);
  if (ret == 0)
    {
      ai = (struct altcom_addrinfo*)(*res);

      while(ai)
        {
          ai->ai_family   = convdomain_local(ai->ai_family);
          DBGIF_ASSERT(ai->ai_family != -EAFNOSUPPORT, "Invalid ai_family\n");

          ai->ai_socktype = convtype_local(ai->ai_socktype);
          DBGIF_ASSERT(ai->ai_socktype != -EPROTONOSUPPORT, "Invalid ai_socktype\n");

          ai->ai_protocol = convproto_local(ai->ai_protocol);
          DBGIF_ASSERT(ai->ai_protocol != -EPROTONOSUPPORT, "Invalid ai_protocol\n");


          if (ai->ai_addrlen <= sizeof(struct altcom_sockaddr_storage))
            {
              /* temporarily evacuate to the working memory */

              memcpy(&ss, ai->ai_addr,
                     sizeof(struct altcom_sockaddr_storage));

              convstorage_local((FAR struct altcom_sockaddr_storage*)ai->ai_addr,
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
