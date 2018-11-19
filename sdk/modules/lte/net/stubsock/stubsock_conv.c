/****************************************************************************
 * modules/lte/net/stubsock/stubsock_conv.c
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
#include "stubsock.h"
#include "altcom_socket.h"
#include "altcom_in.h"
#include "dbg_if.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stubsock_convdomain_remote()
 *
 * Description:
 *   Convert domain to remote definition.
 *
 ****************************************************************************/

int stubsock_convdomain_remote(int domain)
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
 * Name: stubsock_convdomain_local()
 *
 * Description:
 *   Convert domain to local definition.
 *
 ****************************************************************************/

int stubsock_convdomain_local(int domain)
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
 * Name: stubsock_convtype_remote()
 *
 * Description:
 *   Convert type to remote definition.
 *
 ****************************************************************************/

int stubsock_convtype_remote(int type)
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
 * Name: stubsock_convtype_local()
 *
 * Description:
 *   Convert type to local definition.
 *
 ****************************************************************************/

int stubsock_convtype_local(int type)
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
 * Name: stubsock_convproto_remote()
 *
 * Description:
 *   Convert protocol to remote definition.
 *
 ****************************************************************************/

int stubsock_convproto_remote(int protocol)
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

      case IPPROTO_ICMPV6:
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
 * Name: stubsock_convproto_local()
 *
 * Description:
 *   Convert protocol to local definition.
 *
 ****************************************************************************/

int stubsock_convproto_local(int protocol)
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
        ret = IPPROTO_ICMPV6;
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
 * Name: stubsock_convflags_remote()
 *
 * Description:
 *   Convert flags to remote definition.
 *
 ****************************************************************************/

int stubsock_convflags_remote(int flags)
{
  int ret = 0;

  if (flags & MSG_PEEK)
    {
      ret |= ALTCOM_MSG_PEEK;
    }

  if (flags & MSG_WAITALL)
    {
      ret |= ALTCOM_MSG_WAITALL;
    }

  if (flags & MSG_OOB)
    {
      ret |= ALTCOM_MSG_OOB;
    }

  if (flags & MSG_DONTWAIT)
    {
      ret |= ALTCOM_MSG_DONTWAIT;
    }

  if (flags & MSG_MORE)
    {
      ret |= ALTCOM_MSG_MORE;
    }

  return ret;
}

/****************************************************************************
 * Name: stubsock_convflags_local()
 *
 * Description:
 *   Convert flags to local definition.
 *
 ****************************************************************************/

int stubsock_convflags_local(int flags)
{
  int ret = 0;

  if (flags & ALTCOM_MSG_PEEK)
    {
      ret |= MSG_PEEK;
    }

  if (flags & ALTCOM_MSG_WAITALL)
    {
      ret |= MSG_WAITALL;
    }

  if (flags & ALTCOM_MSG_OOB)
    {
      ret |= MSG_OOB;
    }

  if (flags & ALTCOM_MSG_DONTWAIT)
    {
      ret |= MSG_DONTWAIT;
    }

  if (flags & ALTCOM_MSG_MORE)
    {
      ret |= MSG_MORE;
    }

  return ret;
}


/****************************************************************************
 * Name: stubsock_convaiflags_remote()
 *
 * Description:
 *   Convert ai_flags to remote definition.
 *
 ****************************************************************************/

int stubsock_convaiflags_remote(int ai_flags)
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
 * Name: stubsock_convsockaddr_remote()
 *
 * Description:
 *   Convert sockaddr to remote definition.
 *
 ****************************************************************************/

void stubsock_convsockaddr_remote(FAR const struct sockaddr *from,
                                  FAR struct altcom_sockaddr_storage *to)
{
  FAR struct sockaddr_in         *inaddr_from;
  FAR struct sockaddr_in6        *in6addr_from;
  FAR struct altcom_sockaddr_in  *inaddr_to;
  FAR struct altcom_sockaddr_in6 *in6addr_to;

  if (from->sa_family == AF_INET)
    {
      inaddr_from = (FAR struct sockaddr_in*)from;
      inaddr_to   = (FAR struct altcom_sockaddr_in*)to;

      inaddr_to->sin_len    = sizeof(struct altcom_sockaddr_in);
      inaddr_to->sin_family = ALTCOM_AF_INET;
      inaddr_to->sin_port   = inaddr_from->sin_port;
      memcpy(&inaddr_to->sin_addr, &inaddr_from->sin_addr,
             sizeof(struct altcom_in_addr));
    }
  else if (from->sa_family == AF_INET6)
    {
      in6addr_from = (FAR struct sockaddr_in6*)from;
      in6addr_to   = (FAR struct altcom_sockaddr_in6*)to;

      in6addr_to->sin6_len    = sizeof(struct altcom_sockaddr_in6);
      in6addr_to->sin6_family = ALTCOM_AF_INET6;
      in6addr_to->sin6_port   = in6addr_from->sin6_port;
      memcpy(&in6addr_to->sin6_addr, &in6addr_from->sin6_addr,
             sizeof(struct altcom_in6_addr));
    }
}

/****************************************************************************
 * Name: stubsock_convstorage_local()
 *
 * Description:
 *   Convert sockaddr_storage to local definition.
 *
 ****************************************************************************/

void stubsock_convstorage_local(FAR const struct altcom_sockaddr_storage *from,
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
 * Name: stubsock_convherrno_local()
 *
 * Description:
 *   Convert h_errno to local definition.
 *
 ****************************************************************************/

int stubsock_convherrno_local(int herr)
{
  int ret;

  switch(herr)
    {
      case ALTCOM_HOST_NOT_FOUND:
        ret = HOST_NOT_FOUND;
        break;

      case ALTCOM_NO_DATA:
        ret = NO_DATA;
        break;

      case ALTCOM_NO_RECOVERY:
        ret = NO_RECOVERY;
        break;

      case ALTCOM_TRY_AGAIN:
        ret = TRY_AGAIN;
        break;

      default:
        ret = herr;
        break;
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
