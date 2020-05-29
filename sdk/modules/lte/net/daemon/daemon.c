/****************************************************************************
 * modules/lte/net/daemon/daemon.c
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#include <sdk/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <debug.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/net/usrsock.h>
#include <nuttx/net/netdev.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#ifdef CONFIG_LTE_DAEMON_SYNC_TIME
#  include <sys/time.h>
#endif

#include "lte/lte_api.h"
#include "lte/lte_daemon.h"

#include "altcom_socket.h"
#include "altcom_in.h"
#include "altcom_select.h"
#include "altcom_select_ext.h"
#include "altcom_errno.h"

#include "lte/altcom/altcom_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SOCKET_COUNT ALTCOM_NSOCKET

#define EVENT_PIPE "/tmp/lte_event_pipe"
#define APIREQ_PIPE "/tmp/lte_api_req_pipe"

#define DAEMONAPI_REQUEST_POWER_ON  128
#define DAEMONAPI_REQUEST_POWER_OFF 129
#define DAEMONAPI_REQUEST_FIN       130

#ifndef CONFIG_LTE_DAEMON_TASK_PRIORITY
#  define CONFIG_LTE_DAEMON_TASK_PRIORITY (110)
#endif

#ifndef MIN
#  define MIN(a,b)  (((a) < (b)) ? (a) : (b))
#endif

#ifdef CONFIG_LTE_DAEMON_DEBUG_MSG
#  define daemon_debug_printf(v, ...) printf("[DBG] "v, ##__VA_ARGS__)
#  define daemon_print_recvevt(v, ...) printf("[EVT recv] "v, ##__VA_ARGS__)
#  define daemon_print_sendevt(v, ...) printf("[EVT send] "v, ##__VA_ARGS__)
#else
#  define daemon_debug_printf(v, ...)
#  define daemon_print_recvevt(v, ...)
#  define daemon_print_sendevt(v, ...)
#endif

#ifdef CONFIG_LTE_DAEMON_DEBUG_ERR
#  define daemon_error_printf(v, ...) printf("[ERR] %d "v, __LINE__, ##__VA_ARGS__)
#else
#  define daemon_error_printf(v, ...)
#endif

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

enum sock_state_e
{
  CLOSED,
  OPENED,
  CONNECTED,
  CONNECTING,
};

struct usock_s
{
  int16_t           index;
  int               usockid;
  enum sock_state_e state;
  int16_t           domain;
  int               flags;
  uint8_t           xid;
};

struct daemon_s
{
  int                  selectid;
  int                  event_outfd;
  int                  event_infd;
  int                  apireq_outfd;
  int                  apireq_infd;
  int                  session_id;
  sem_t                sync_sem;
  int                  pid;
  lte_apn_setting_t    apn;
  struct usock_s       sockets[SOCKET_COUNT];
  struct net_driver_s  net_dev;
  void                 (*user_restart_cb)(uint32_t reason);
};

struct socket_param
{
  int domain;
  int type;
  int protocol;
};

struct setsockopt_param
{
  int16_t level;
  int16_t option;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int socket_request(int fd, struct daemon_s *priv,
                          FAR void *hdrbuf);
static int close_request(int fd, struct daemon_s *priv,
                          FAR void *hdrbuf);
static int connect_request(int fd, struct daemon_s *priv,
                            FAR void *hdrbuf);
static int sendto_request(int fd, struct daemon_s *priv,
                          FAR void *hdrbuf);
static int recvfrom_request(int fd, struct daemon_s *priv,
                            FAR void *hdrbuf);
static int setsockopt_request(int fd, struct daemon_s *priv,
                              FAR void *hdrbuf);
static int getsockopt_request(int fd, struct daemon_s *priv,
                              FAR void *hdrbuf);
static int getsockname_request(int fd, struct daemon_s *priv,
                                FAR void *hdrbuf);
static int getpeername_request(int fd, struct daemon_s *priv,
                                FAR void *hdrbuf);
static int bind_request(int fd, struct daemon_s *priv,
                        FAR void *hdrbuf);
static int listen_request(int fd, struct daemon_s *priv,
                          FAR void *hdrbuf);
static int accept_request(int fd, struct daemon_s *priv,
                          FAR void *hdrbuf);
static int ioctl_request(int fd, struct daemon_s *priv,
                          FAR void *hdrbuf);

static void select_async_callback(int32_t ret_code, int32_t err_code,
                                  int32_t id, FAR altcom_fd_set *readset,
                                  FAR altcom_fd_set *writeset,
                                  FAR altcom_fd_set *exceptset,
                                  FAR void* priv);
static void set_select_socket(struct daemon_s* priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct usrsock_req_handler_s
{
  uint32_t hdrlen;
  int (CODE *fn)(int fd, struct daemon_s *priv, FAR void *req);
} handlers[USRSOCK_REQUEST__MAX] =
{
  [USRSOCK_REQUEST_SOCKET] =
  {
    sizeof(struct usrsock_request_socket_s),
    socket_request,
  },
  [USRSOCK_REQUEST_CLOSE] =
  {
    sizeof(struct usrsock_request_close_s),
    close_request,
  },
  [USRSOCK_REQUEST_CONNECT] =
  {
    sizeof(struct usrsock_request_connect_s),
    connect_request,
  },
  [USRSOCK_REQUEST_SENDTO] =
  {
    sizeof(struct usrsock_request_sendto_s),
    sendto_request,
  },
  [USRSOCK_REQUEST_RECVFROM] =
  {
    sizeof(struct usrsock_request_recvfrom_s),
    recvfrom_request,
  },
  [USRSOCK_REQUEST_SETSOCKOPT] =
  {
    sizeof(struct usrsock_request_setsockopt_s),
    setsockopt_request,
  },
  [USRSOCK_REQUEST_GETSOCKOPT] =
  {
    sizeof(struct usrsock_request_getsockopt_s),
    getsockopt_request,
  },
  [USRSOCK_REQUEST_GETSOCKNAME] =
  {
    sizeof(struct usrsock_request_getsockname_s),
    getsockname_request,
  },
  [USRSOCK_REQUEST_GETPEERNAME] =
  {
    sizeof(struct usrsock_request_getpeername_s),
    getpeername_request,
  },
  [USRSOCK_REQUEST_BIND] =
  {
    sizeof(struct usrsock_request_bind_s),
    bind_request,
  },
  [USRSOCK_REQUEST_LISTEN] =
  {
    sizeof(struct usrsock_request_listen_s),
    listen_request,
  },
  [USRSOCK_REQUEST_ACCEPT] =
  {
    sizeof(struct usrsock_request_accept_s),
    accept_request,
  },
  [USRSOCK_REQUEST_IOCTL] =
  {
    sizeof(struct usrsock_request_ioctl_s),
    ioctl_request,
  },
};

struct daemon_s *g_daemon             = NULL;
static bool     g_daemonisrunnning    = false;
static int      g_altcomresult;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _write_to_usock
 ****************************************************************************/

static int _write_to_usock(int fd, void *buf, size_t count)
{
  ssize_t wlen;

#ifdef CONFIG_LTE_DAEMON_DEBUG_MSG
  FAR struct usrsock_message_common_s       *head;
  FAR struct usrsock_message_req_ack_s      *req_ack;
  FAR struct usrsock_message_socket_event_s *event;
  FAR struct usrsock_message_datareq_ack_s  *datareq_ack;
  static int remain_size = 0;

  head = buf;
  if (remain_size > 0 )
    {
      daemon_print_sendevt("datareq_ack data remain_size %d, count %d\n",
                            remain_size, count);
      remain_size -= count;
    }
  else
    {
      switch(head->msgid)
        {
          case USRSOCK_MESSAGE_RESPONSE_ACK:
            req_ack = buf;
            daemon_print_sendevt("USRSOCK_MESSAGE_RESPONSE_ACK, xid %d\n",
                                  req_ack->xid);
            break;

          case USRSOCK_MESSAGE_RESPONSE_DATA_ACK:
            datareq_ack = buf;
            remain_size = datareq_ack->valuelen;
            daemon_print_sendevt("USRSOCK_MESSAGE_RESPONSE_DATA_ACK, xid %d\n",
                                  datareq_ack->reqack.xid);
            break;

          case USRSOCK_MESSAGE_SOCKET_EVENT:
            event = buf;
            if (event->events & USRSOCK_EVENT_ABORT)
              {
                daemon_print_sendevt("USRSOCK_EVENT_ABORT, usockid %d\n",
                                      event->usockid);
              }
            if (event->events & USRSOCK_EVENT_SENDTO_READY)
              {
                daemon_print_sendevt("USRSOCK_EVENT_SENDTO_READY, usockid %d\n",
                                      event->usockid);
              }
            if (event->events & USRSOCK_EVENT_RECVFROM_AVAIL)
              {
                daemon_print_sendevt("USRSOCK_EVENT_RECVFROM_AVAIL, usockid %d\n",
                                      event->usockid);
              }
            if (event->events & USRSOCK_EVENT_REMOTE_CLOSED)
              {
                daemon_print_sendevt("USRSOCK_EVENT_REMOTE_CLOSED, usockid %d\n",
                                      event->usockid);
              }
          break;

          default:
          break;
        }
    }
#endif

  daemon_debug_printf("%s fd = %d, count = %d\n", __func__, fd, count);

  wlen = write(fd, buf, count);

  if (0 > wlen)
    {
      return -errno;
    }

  if (wlen != count)
    {
      return -ENOSPC;
    }

  return OK;
}

/****************************************************************************
 * Name: _send_ack_common
 ****************************************************************************/

static int _send_ack_common(int fd,
                            uint8_t xid,
                            FAR struct usrsock_message_req_ack_s *resp)
{
  resp->head.msgid = USRSOCK_MESSAGE_RESPONSE_ACK;
  resp->head.flags = 0;
  resp->xid = xid;

  /* Send ACK response. */

  return _write_to_usock(fd, resp, sizeof(*resp));
}

/****************************************************************************
 * Name: daemon_socket_new
 ****************************************************************************/

static FAR struct usock_s *daemon_socket_new(FAR struct daemon_s *priv)
{
  FAR struct usock_s *usock = NULL;
  int i;

  for (i = 0; i < SOCKET_COUNT; i++)
    {
      if (CLOSED == priv->sockets[i].state)
        {
          usock = &priv->sockets[i];
          memset(usock, 0, sizeof(struct usock_s));
          usock->usockid = -1;
          usock->index = i;
          usock->state = OPENED;
          return usock;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: daemon_socket_delete
 ****************************************************************************/

static void daemon_socket_delete(FAR struct daemon_s *priv,
                                  FAR struct usock_s *usock)
{
  if (usock != NULL)
    {
      usock->usockid = -1;
      usock->state = CLOSED;
    }
}

/****************************************************************************
 * Name: daemon_socket_send_abort
 ****************************************************************************/

static int daemon_socket_send_abort(FAR struct daemon_s *priv, int fd)
{
  int                                   i;
  int                                   ret    = 0;
  FAR struct usock_s                    *usock = NULL;
  struct usrsock_message_socket_event_s event;

  event.head.msgid = USRSOCK_MESSAGE_SOCKET_EVENT;
  event.head.flags = USRSOCK_MESSAGE_FLAG_EVENT;

  for (i = 0; i < SOCKET_COUNT; i++)
    {
      usock = &priv->sockets[i];
      if (CLOSED == usock->state)
        {
          continue;
        }

      daemon_socket_delete(priv, usock);

      event.usockid = i;
      event.events = USRSOCK_EVENT_ABORT;
      daemon_print_sendevt("USRSOCK_EVENT_ABORT, usockid %d\n",
                            event.usockid);
      ret = _write_to_usock(fd, &event, sizeof(event));
      if (0 > ret)
        {
          daemon_error_printf("_write_to_usock() ret = %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: daemon_socket_get
 ****************************************************************************/

static FAR struct usock_s *daemon_socket_get(FAR struct daemon_s *priv,
                                              int sockid)
{
  if (sockid > SOCKET_COUNT)
    {
      return NULL;
    }
  return &priv->sockets[sockid];
}

/****************************************************************************
 * Name: read_req
 ****************************************************************************/

static ssize_t
read_req(int fd, FAR const struct usrsock_request_common_s *com_hdr,
          FAR void *req, size_t reqsize)
{
  ssize_t rlen;

  rlen = read(fd, (uint8_t *)req + sizeof(*com_hdr),
              reqsize - sizeof(*com_hdr));

  if (rlen < 0)
    {
      return -errno;
    }
  if (rlen + sizeof(*com_hdr) != reqsize)
    {
      return -EMSGSIZE;
    }

  return rlen;
}

/****************************************************************************
 * Name: usrsock_request
 ****************************************************************************/

static int usrsock_request(int fd, FAR struct daemon_s *priv)
{
  FAR struct usrsock_request_common_s *com_hdr;
  uint8_t                             hdrbuf[16];
  ssize_t                             rlen;
  int                                 ret = 0;

  com_hdr = (FAR void *)hdrbuf;
  rlen = read(fd, com_hdr, sizeof(struct usrsock_request_common_s));

  if (rlen < 0)
    {
      return -errno;
    }

  if (rlen != sizeof(struct usrsock_request_common_s))
    {
      return -EMSGSIZE;
    }

  if (com_hdr->reqid >= USRSOCK_REQUEST__MAX ||
      !handlers[com_hdr->reqid].fn)
    {
      ASSERT(false);
      return -EIO;
    }

  assert(handlers[com_hdr->reqid].hdrlen < sizeof(hdrbuf));

  rlen = read_req(fd, com_hdr, hdrbuf,
    handlers[com_hdr->reqid].hdrlen);

  if (rlen < 0)
    {
      return rlen;
    }

  daemon_debug_printf("handlers[com_hdr->reqid] : %d\n",
                      com_hdr->reqid);
  ret = handlers[com_hdr->reqid].fn(fd, priv, hdrbuf);
  daemon_debug_printf("handlers[com_hdr->reqid] : %d, ret = %d\n",
                      com_hdr->reqid, ret);
  return ret;
}

/****************************************************************************
 * Name: usock_send_event
 ****************************************************************************/

static int usock_send_event(int fd, FAR struct daemon_s *priv,
                            FAR struct usock_s *usock, int events)
{
  FAR struct usrsock_message_socket_event_s event;
  int                                       i = 0;

  memset(&event, 0, sizeof(event));
  event.head.flags = USRSOCK_MESSAGE_FLAG_EVENT;
  event.head.msgid = USRSOCK_MESSAGE_SOCKET_EVENT;

  for (i = 0; i < SOCKET_COUNT; i++)
    {
      if (usock == &priv->sockets[i])
        {
          break;
        }
    }

  if (SOCKET_COUNT == i)
    {
      return -EINVAL;
    }

  event.usockid = i;
  event.events  = events;

  return _write_to_usock(fd, &event, sizeof(event));
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
 * Name: convproto_remote
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
 * Name: conv_socket_param
 ****************************************************************************/

static int conv_socket_param(int domain, int type, int protocol,
                              FAR struct socket_param *param)
{
  param->domain = convdomain_remote(domain);
  param->type = convtype_remote(type);
  param->protocol = convproto_remote(protocol);
  if ((-EPROTONOSUPPORT == param->domain) ||
      (-EPROTONOSUPPORT == param->type) ||
      (-EPROTONOSUPPORT == param->protocol))
    {
      return -EPROTONOSUPPORT;
    }

  return OK;
}

/****************************************************************************
 * Name: conv_setsockopt_param
 ****************************************************************************/

static int conv_setsockopt_param(int16_t level, int16_t option,
                                  FAR struct setsockopt_param *param)
{
  int ret = 0;
  switch(level)
    {
      case SOL_SOCKET:
        param->level = ALTCOM_SOL_SOCKET;

        switch(option)
          {
            case SO_BROADCAST:
              param->option = ALTCOM_SO_BROADCAST;
              break;
            case SO_REUSEADDR:
              param->option = ALTCOM_SO_REUSEADDR;
              break;
            case SO_KEEPALIVE:
              param->option = ALTCOM_SO_KEEPALIVE;
              break;
            case SO_RCVBUF:
              param->option = ALTCOM_SO_RCVBUF;
              break;

#ifdef CONFIG_NET_SOLINGER
            case SO_LINGER:
              param->option = ALTCOM_SO_LINGER;
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
  return ret;
}

/****************************************************************************
 * Name: conv_getsockopt_param
 ****************************************************************************/

static int conv_getsockopt_param(int16_t level, int16_t option,
                          FAR struct setsockopt_param *param)
{
  int ret = 0;
  switch(level)
    {
      case SOL_SOCKET:
        param->level = ALTCOM_SOL_SOCKET;

        switch(option)
          {
            case SO_ACCEPTCONN:
              param->option = ALTCOM_SO_ACCEPTCONN;
              break;
            case SO_ERROR:
              param->option = ALTCOM_SO_ERROR;
              break;
            case SO_BROADCAST:
              param->option = ALTCOM_SO_BROADCAST;
              break;
            case SO_KEEPALIVE:
              param->option = ALTCOM_SO_KEEPALIVE;
              break;
            case SO_REUSEADDR:
              param->option = ALTCOM_SO_REUSEADDR;
              break;
            case SO_TYPE:
              param->option = ALTCOM_SO_TYPE;
              break;
            case SO_RCVBUF:
              param->option = ALTCOM_SO_RCVBUF;
              break;

#ifdef CONFIG_NET_SOLINGER
            case SO_LINGER:
              param->option = ALTCOM_SO_LINGER;
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
            default:
              ret = -ENOPROTOOPT;
              break;
          }
        break;

      default:
        ret = -ENOPROTOOPT;
        break;
    }
  return ret;
}

/****************************************************************************
 * Name: convsockaddr_remote
 ****************************************************************************/

static void convsockaddr_remote(FAR const struct sockaddr *from,
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
 * Name: convflags_local
 ****************************************************************************/

static int convflags_local(int from_flags, int *to_flags)
{

  if (from_flags & (MSG_DONTROUTE | MSG_CTRUNC | MSG_PROXY | MSG_TRUNC |
                    MSG_EOR | MSG_FIN | MSG_SYN | MSG_CONFIRM |
                    MSG_RST | MSG_ERRQUEUE | MSG_NOSIGNAL) )
    {
      return -ENOPROTOOPT;
    }

  *to_flags = 0;

  if (from_flags & MSG_PEEK)
    {
      *to_flags |= ALTCOM_MSG_PEEK;
    }
  if (from_flags & MSG_WAITALL)
    {
      *to_flags |= ALTCOM_MSG_WAITALL;
    }
  if (from_flags & MSG_OOB)
    {
      *to_flags |= ALTCOM_MSG_OOB;
    }
  if (from_flags & MSG_DONTWAIT)
    {
      *to_flags |= ALTCOM_MSG_DONTWAIT;
    }
  if (from_flags & MSG_MORE)
    {
      *to_flags |= ALTCOM_MSG_MORE;
    }

  return 0;
}

/****************************************************************************
 * Name: set_select_socket
 ****************************************************************************/

static void set_select_socket(struct daemon_s* priv)
{
  altcom_fd_set local_readset;
  altcom_fd_set local_writeset;
  altcom_fd_set local_execset;

  ALTCOM_FD_ZERO(&local_readset);
  ALTCOM_FD_ZERO(&local_writeset);
  ALTCOM_FD_ZERO(&local_execset);
  int i     = 0;
  int maxfd = -1;

  for (i = 0; i < SOCKET_COUNT; i++)
    {
      if (CLOSED == priv->sockets[i].state)
        {
          continue;
        }
      if ( !(priv->sockets[i].flags & USRSOCK_EVENT_RECVFROM_AVAIL) )
        {
          ALTCOM_FD_SET(priv->sockets[i].usockid, &local_readset);
        }
      if ( !(priv->sockets[i].flags & USRSOCK_EVENT_SENDTO_READY) )
        {
          ALTCOM_FD_SET(priv->sockets[i].usockid, &local_writeset);
        }

      ALTCOM_FD_SET(priv->sockets[i].usockid, &local_execset);

      if (maxfd < priv->sockets[i].usockid)
        {
          maxfd = priv->sockets[i].usockid;
        }
    }

  if (maxfd != -1)
    {
      priv->selectid = altcom_select_async(maxfd + 1,
                        &local_readset, &local_writeset, &local_execset,
                        select_async_callback, (void *)priv);
    }
}

/****************************************************************************
 * Name: select_async_callback
 ****************************************************************************/

static void select_async_callback(int32_t ret_code, int32_t err_code,
                                  int32_t id, FAR altcom_fd_set *readset,
                                  FAR altcom_fd_set *writeset,
                                  FAR altcom_fd_set *exceptset,
                                  FAR void* priv)
{
  FAR struct daemon_s                  *info;
  struct     usrsock_message_req_ack_s resp;
  int32_t                              result = 0;
  int32_t                              val_len;
  int                                  ret  = 0;
  int                                  i;
  int                                  wlen = 0;

  info = (FAR struct daemon_s *)priv;

  if (0 > ret_code)
    {
      return;
    }

    for (i = 0; i < SOCKET_COUNT; i++)
      {
        if (CLOSED == info->sockets[i].state)
          {
            continue;
          }

        if (readset)
          {
            if (ALTCOM_FD_ISSET(info->sockets[i].usockid, readset))
              {
                wlen = usock_send_event(info->event_outfd, info,
                                        &info->sockets[i],
                                        USRSOCK_EVENT_RECVFROM_AVAIL);
                daemon_print_sendevt("USRSOCK_EVENT_RECVFROM_AVAIL\n");
                info->sockets[i].flags |= USRSOCK_EVENT_RECVFROM_AVAIL;
                if (0 > wlen)
                  {
                    return;
                  }
              }
          }

      if (writeset)
        {
          if (ALTCOM_FD_ISSET(info->sockets[i].usockid, writeset))
            {
              wlen = usock_send_event(info->event_outfd, info,
                                      &info->sockets[i],
                                      USRSOCK_EVENT_SENDTO_READY);
              daemon_print_sendevt("USRSOCK_EVENT_SENDTO_READY\n");
              info->sockets[i].flags |= USRSOCK_EVENT_SENDTO_READY;
              if (CONNECTING == info->sockets[i].state)
                {
                  val_len = sizeof(result);
                  ret = altcom_getsockopt(info->sockets[i].usockid,
                                          ALTCOM_SOL_SOCKET,
                                          ALTCOM_SO_ERROR,
                                          (FAR void*)&result,
                                          (altcom_socklen_t*)&val_len);
                  daemon_debug_printf("altcom_getsockopt() ret = %d\n", ret);
                  if (0 > ret)
                    {
                      ret = altcom_errno();
                      ret = -ret;
                      daemon_debug_printf("altcom_getsockopt() failed = %d\n",
                                          ret);
                    }
                  else if (0 == result)
                    {
                      info->sockets[i].state = CONNECTED;
                    }
                  else
                    {
                      info->sockets[i].state = OPENED;
                    }
                  memset(&resp, 0, sizeof(resp));
                  resp.result = result;
                  ret = _send_ack_common(info->event_outfd,
                                          info->sockets[i].xid, &resp);
                  if (0 > ret)
                    {
                      daemon_error_printf("_send_ack_common() ret = %d\n",
                                          ret);
                    }
                }
            }
        }
    }

  set_select_socket(info);
}

/****************************************************************************
 * Name: setup_event
 ****************************************************************************/

static int setup_event(struct daemon_s *priv)
{
  int ret = 0;

  if (priv->selectid != -1)
    {
      altcom_select_async_cancel(priv->selectid, true);
      priv->selectid = -1;
    }

  set_select_socket(priv);

  return ret;
}

#ifdef CONFIG_LTE_DAEMON_SYNC_TIME

/****************************************************************************
 * Name: localtime_callback
 ****************************************************************************/

static void localtime_callback(FAR lte_localtime_t *localtime)
{
  int            ret;
  struct tm      calTime;
  struct timeval current_time = {0};

  /* lte_localtime_t -> struct tm */

  memset(&calTime, 0, sizeof(struct tm));
  calTime.tm_year = localtime->year + 100; /* 1900 + 100 + year(0-99) */
  calTime.tm_mon  = localtime->mon - 1;    /* mon(1-12) - 1 */
  calTime.tm_mday = localtime->mday;
  calTime.tm_hour = localtime->hour;
  calTime.tm_min  = localtime->min;
  calTime.tm_sec  = localtime->sec;

  /* struct tm -> struct time_t */

  current_time.tv_sec = mktime(&calTime);

  /* Set time */

  ret = settimeofday(&current_time, NULL);
  if (ret < 0)
    {
      daemon_error_printf("settimeofday falied: %d\n", errno);
      return;
    }

  daemon_debug_printf("set localtime: %4d/%02d/%02d,%02d:%02d:%02d\n",
                      localtime->year + 1900 + 100, localtime->mon,
                      localtime->mday, localtime->hour, localtime->min,
                      localtime->sec);
}

#endif

/****************************************************************************
 * Name: socket_request
 ****************************************************************************/

static int socket_request(int fd, FAR struct daemon_s *priv,
                          FAR void *hdrbuf)
{
  FAR struct usrsock_request_socket_s  *req        = hdrbuf;
  FAR struct usock_s                   *usock      = NULL;
  struct     usrsock_message_req_ack_s resp;
  struct     socket_param              param;
  int                                  ret         = 0;
  int                                  val         = 0;
  int16_t                              usockid;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("socket, xid = %d\n", req->head.xid);

  /* Allocate socket. */
  usock = daemon_socket_new(priv);
  if (!usock)
    {
      ret = -ENFILE;
      daemon_error_printf("allocate usock table failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("allocate usock table index = %d\n", usock->index);

  ret = conv_socket_param(req->domain, req->type, req->protocol, &param);
  if (0 > ret)
    {
      daemon_error_printf("convert nuttx to altcom failed = %d\n", ret);
      goto send_resp;
    }

  usockid = altcom_socket(param.domain, param.type, param.protocol);
  if (0 > usockid)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_socket() failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("altcom_socket() altcom_fd = %d\n", usockid);

  usock->usockid = usockid;
  usock->domain = req->domain;

  val = altcom_fcntl(usockid, ALTCOM_GETFL, 0);
  if (0 > val)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_fcntl() errno = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("altcom_fcntl() val = %d\n", val);

  ret = altcom_fcntl(usock->usockid, ALTCOM_SETFL, (val | ALTCOM_O_NONBLOCK));
  if (0 > ret)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_fcntl() errno = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("altcom_fcntl() ret = %d\n", ret);

  ret = setup_event(priv);
  if (0 > ret)
    {
      ret = altcom_errno();
      ret = -ret;
      goto send_resp;
    }

send_resp:
  /* Send ACK response */
  memset(&resp, 0, sizeof(resp));
  resp.result = (0 > ret) ? ret : usock->index;
  daemon_debug_printf("%s resp.result = %d\n", __func__, ret);

  if (0 > ret)
    {
      if ((usock != NULL) && (usock->usockid != -1))
        {
          altcom_close(usock->usockid);
        }
      daemon_socket_delete(priv, usock);
    }

  ret = _send_ack_common(fd, req->head.xid, &resp);
  if (0 > ret)
    {
      daemon_error_printf("_send_ack_common() failed = %d\n", ret);
      return ret;
    }

  daemon_debug_printf("%s: end \n", __func__);
  return OK;
}

/****************************************************************************
 * Name: close_request
 ****************************************************************************/

static int close_request(int fd, FAR struct daemon_s *priv,
                          FAR void *hdrbuf)
{
  FAR struct usrsock_request_close_s   *req   = hdrbuf;
  FAR struct usock_s                   *usock = NULL;
  struct     usrsock_message_req_ack_s resp;
  int                                  ret    = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("close, xid = %d\n", req->head.xid);

  /* Check if this socket exists. */
  usock = daemon_socket_get(priv, req->usockid);
  if (!usock)
    {
      ret = 0;
      daemon_error_printf("get usock table failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("close_request usockid = %d\n", usock->usockid);

  if (CLOSED == usock->state)
    {
      ret = 0;
      daemon_error_printf("socket is already closed");
      goto send_resp;
    }

  ret = altcom_close(usock->usockid);
  daemon_socket_delete(priv, usock);
  if (0 > ret)
    {
      ret = 0;
      daemon_error_printf("altcom_close() errno = %d\n", altcom_errno());
      goto send_resp;
    }
  daemon_debug_printf("altcom_close() ret = %d\n", ret);

  setup_event(priv);

send_resp:
  /* Send ACK response */
  memset(&resp, 0, sizeof(resp));
  resp.result = ret;
  daemon_debug_printf("%s resp.result = %d\n", __func__, ret);

  ret = _send_ack_common(fd, req->head.xid, &resp);
  if (0 > ret)
    {
      daemon_error_printf("_send_ack_common() ret = %d\n", ret);
      return ret;
    }

  daemon_debug_printf("%s: end \n", __func__);
  return OK;
}

/****************************************************************************
 * Name: connect_request
 ****************************************************************************/

static int connect_request(int fd, struct daemon_s *priv, FAR void *hdrbuf)
{
  FAR struct usrsock_request_connect_s *req     = hdrbuf;
  struct     usrsock_message_req_ack_s resp;
  struct     sockaddr_storage          addr;
  struct     altcom_sockaddr_storage   storage;
  FAR struct usock_s                   *usock   = NULL;
  ssize_t                              rlen;
  int                                  ret      = 0;
  altcom_socklen_t                     addr_len = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("connect, xid = %d\n", req->head.xid);

  /* Check if this socket exists. */
  usock = daemon_socket_get(priv, req->usockid);
  if (!usock)
    {
      ret = -EBADFD;
      daemon_error_printf("get usock table failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("connect_request usockid = %d\n", usock->usockid);

  /* Check if this socket is not opened. */
  if (CLOSED == usock->state)
    {
      ret = -ENETDOWN;
      daemon_error_printf("socket is not opened.\n");
      goto send_resp;
    }

  /* Check if address size ok. */
  if (req->addrlen == sizeof(struct sockaddr_in))
    {
      addr_len = sizeof(struct altcom_sockaddr_in);
    }
  else if (req->addrlen == sizeof(struct sockaddr_in6))
    {
      addr_len = sizeof(struct altcom_sockaddr_in6);
    }
  else
    {
      ret = -EINVAL;
      daemon_error_printf("invalid argment request addrlen .\n");
      goto send_resp;
    }

  /* Read address. */
  rlen = read(fd, (FAR void *)&addr, req->addrlen);
  if (0 > rlen|| rlen < req->addrlen)
    {
      ret = -EFAULT;
      daemon_error_printf("read address failed.\n");
      goto send_resp;
    }

  memset(&storage, 0, sizeof(struct altcom_sockaddr_storage));
  convsockaddr_remote((struct sockaddr *)&addr, &storage);

  ret = altcom_connect(usock->usockid,
                        (FAR const struct altcom_sockaddr*)&storage,
                        addr_len);
  if (0 > ret)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_connect() failed = %d\n", ret);
      goto send_resp;
    }
  usock->state = CONNECTED;

  daemon_debug_printf("altcom_connect() ret = %d\n", ret);

send_resp:
  /* Send response. */
  memset(&resp, 0, sizeof(resp));
  resp.head.flags = 0;

  if ((usock != NULL) && (ret == -EINPROGRESS))
    {
      usock->state = CONNECTING;
      usock->xid = req->head.xid;
      resp.head.flags |= USRSOCK_MESSAGE_FLAG_REQ_IN_PROGRESS;
    }

  resp.head.msgid = USRSOCK_MESSAGE_RESPONSE_ACK;
  resp.xid = req->head.xid;
  resp.result = ret;

  ret = _write_to_usock(fd, &resp, sizeof(resp));
  if (0 > ret)
    {
      daemon_error_printf("_write_to_usock() ret = %d\n", ret);
      return ret;
    }

  daemon_debug_printf("%s: end \n", __func__);
  return OK;
}

/****************************************************************************
 * Name: sendto_request
 ****************************************************************************/

static int sendto_request(int fd, struct daemon_s *priv, FAR void *hdrbuf)
{
  FAR struct usrsock_request_sendto_s  *req       = hdrbuf;
  FAR struct usock_s                   *usock     = NULL;
  FAR struct altcom_sockaddr           *pto       = NULL;
  struct     usrsock_message_req_ack_s resp;
  struct     altcom_sockaddr_storage   storage;
  struct     sockaddr_storage          to;
  uint8_t                              *sendbuf   = NULL;
  ssize_t                              rlen;
  int                                  ret        = 0;
  altcom_socklen_t                     addr_len   = 0;
  int                                  flags      = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("sendto, xid = %d\n", req->head.xid);

  /* Check if this socket exists. */
  usock = daemon_socket_get(priv, req->usockid);
  if (!usock)
    {
      ret = -EBADFD;
      daemon_error_printf("get usock table failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("sendto_request usockid = %d\n", usock->usockid);

  /* Check if this socket is connected. */
  if (CLOSED == usock->state)
    {
      ret = -ENETDOWN;
      daemon_error_printf("socket is not opened.\n");
      goto send_resp;
    }

  /* Check if address size ok. */
  if (req->addrlen > 0)
    {
      if (req->addrlen == sizeof(struct sockaddr_in))
        {
          addr_len = sizeof(struct altcom_sockaddr_in);
        }
      else if (req->addrlen == sizeof(struct sockaddr_in6))
        {
          addr_len = sizeof(struct altcom_sockaddr_in6);
        }
      else
        {
          ret = -EINVAL;
          daemon_error_printf("invalid argment request addrlen.\n");
          goto send_resp;
        }

      rlen = read(fd, (FAR void *)&to, req->addrlen);
      if (0 > rlen|| rlen < req->addrlen)
        {
          ret = -EFAULT;
          daemon_error_printf("read address failed.\n");
          goto send_resp;
        }
      memset(&storage, 0, sizeof(struct altcom_sockaddr_storage));
      convsockaddr_remote((struct sockaddr *)&to, &storage);

      pto = (FAR struct altcom_sockaddr*)&storage;
    }

    /* Check if the request has data. */
    if (req->buflen > 0)
      {
        sendbuf = calloc(1, req->buflen);
        if (!sendbuf)
          {
            ret = -ENOBUFS;
            daemon_error_printf("buffer allocate failed.\n");
            goto send_resp;
          }

        /* Read data from usrsock. */
        rlen = read(fd, sendbuf, req->buflen);
        if (0 > rlen || rlen < req->buflen)
          {
            ret = -EFAULT;
            daemon_error_printf("read buffer failed.\n");
            goto send_resp;
          }
    }
  else
    {
      ret = -EINVAL;
      daemon_error_printf("invalid argment request addrlen.\n");
      goto send_resp;
    }

  ret = convflags_local(req->flags, &flags);
  if (0 > ret)
    {
      daemon_error_printf("convert nuttx to altcom failed = %d\n", ret);
      goto send_resp;
    }

  ret = altcom_sendto(usock->usockid, sendbuf, req->buflen, flags,
                      pto, addr_len);
  usock->flags &= ~USRSOCK_EVENT_SENDTO_READY;
  setup_event(priv);
  if (0 > ret)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_sendto() failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("altcom_sendto() ret = %d\n", ret);

send_resp:
  /* Send ACK response. */
  memset(&resp, 0, sizeof(resp));
  resp.result = ret;
  daemon_debug_printf("%s resp.result = %d\n", __func__, ret);

  if (sendbuf)
    {
      free(sendbuf);
      sendbuf = NULL;
    }

  ret = _send_ack_common(fd, req->head.xid, &resp);
  if (0 > ret)
    {
      daemon_error_printf("_send_ack_common() ret = %d\n", ret);
      return ret;
    }

  daemon_debug_printf("%s: end \n", __func__);
    return OK;
}

/****************************************************************************
 * Name: recvfrom_request
 ****************************************************************************/

static int recvfrom_request(int fd, struct daemon_s *priv, FAR void *hdrbuf)
{
  FAR struct usrsock_request_recvfrom_s *req           = hdrbuf;
  FAR struct usock_s                    *usock         = NULL;
  struct usrsock_message_datareq_ack_s  resp;
  struct altcom_sockaddr_storage        storage;
  struct sockaddr_in6                   tmpaddr;
  struct sockaddr_storage               from;
  int                                   ret            = 0;
  uint8_t                               *buf           = NULL;
  uint8_t                               dummy_buf      = 0;
  altcom_socklen_t                      altcom_fromlen = 0;
  socklen_t                             output_fromlen = 0;
  int                                   flags          = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("recvfrom, xid = %d\n", req->head.xid);

  /* Check if this socket exists. */
  usock = daemon_socket_get(priv, req->usockid);
  if (!usock)
    {
      ret = -EBADFD;
      daemon_error_printf("get usock table failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("recvfrom_request usockid = %d\n", usock->usockid);

  if (CLOSED == usock->state)
    {
      ret = -ENETDOWN;
      daemon_error_printf("socket is not opened.\n");
      goto send_resp;
    }

  if (usock->domain == AF_INET)
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

  /* Check if the request has data. */
  if (req->max_buflen > 0)
    {
      buf = calloc(1, req->max_buflen);
      if (!buf)
        {
          ret = -ENOBUFS;
          daemon_error_printf("buffer allocate failed.\n");
          goto send_resp;
        }
    }

  ret = convflags_local(req->flags, &flags);
  if (0 > ret)
    {
      daemon_error_printf("convert nuttx to altcom failed = %d\n", ret);
      goto send_resp;
    }

  ret = altcom_recvfrom(usock->usockid,
                        (buf != NULL) ? buf : (FAR void *)&dummy_buf,
                        req->max_buflen, flags,
                        (FAR struct altcom_sockaddr *)&storage,
                        &altcom_fromlen);
  usock->flags &= ~USRSOCK_EVENT_RECVFROM_AVAIL;
  setup_event(priv);
  if (0 > ret)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_recvform() failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("altcom_recvform() ret = %d\n", ret);

  convstorage_local(&storage, (FAR struct sockaddr*)&tmpaddr);
  memcpy(&from, &tmpaddr, req->max_addrlen);

send_resp:
  /* Send response. */
  memset(&resp, 0, sizeof(resp));
  resp.reqack.result = ret;

  resp.reqack.head.msgid = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  resp.reqack.head.flags = 0;
  resp.reqack.xid = req->head.xid;

  resp.valuelen = MIN(output_fromlen,
                      req->max_addrlen);
  resp.valuelen_nontrunc = output_fromlen;

  if (0 >= ret)
    {
      resp.valuelen_nontrunc = 0;
      resp.valuelen = 0;
      if ((0 == ret) && (req->max_buflen != 0))
        {
          usock_send_event(fd, priv, usock,
          USRSOCK_EVENT_REMOTE_CLOSED);
        }
    }

  daemon_debug_printf("%s resp.result = %d\n", __func__, ret);

  ret = _write_to_usock(fd, &resp, sizeof(resp));
  if (0 > ret)
    {
      daemon_error_printf("_write_to_usock() ret = %d\n", ret);
    }
  else
    {
      if (resp.valuelen > 0)
        {
          ret = _write_to_usock(fd, &from, resp.valuelen);
          if (0 > ret)
            {
              daemon_error_printf("_write_to_usock() ret = %d\n", ret);
            }
        }

      if (resp.reqack.result > 0)
        {
          ret = _write_to_usock(fd, buf, resp.reqack.result);
          if (0 > ret)
            {
              daemon_error_printf("_write_to_usock() ret = %d\n", ret);
            }
        }
    }

  if (buf)
    {
      free(buf);
      buf = NULL;
    }
  daemon_debug_printf("%s: end \n", __func__);
  return OK;
}

/****************************************************************************
 * Name: bind_request
 ****************************************************************************/

static int bind_request(int fd, struct daemon_s *priv, FAR void *hdrbuf)
{
  FAR struct usrsock_request_bind_s *req     = hdrbuf;
  FAR struct usock_s                *usock   = NULL;
  struct usrsock_message_req_ack_s  resp;
  struct altcom_sockaddr_storage    storage;
  struct sockaddr_storage           addr;
  ssize_t                           rlen;
  int                               ret      = 0;
  altcom_socklen_t                  addr_len = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("bind, xid = %d\n", req->head.xid);

  /* Check if this socket exists. */
  usock = daemon_socket_get(priv, req->usockid);
  if (!usock)
    {
      ret = -EBADFD;
      daemon_error_printf("get usock table failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("bind_request usockid = %d\n", usock->usockid);

  if (CLOSED == usock->state)
    {
      ret = -ENETDOWN;
      daemon_error_printf("socket is not opened.\n");
      goto send_resp;
    }

  /* Check if address size ok. */
  if (req->addrlen > 0)
    {
      if (req->addrlen == sizeof(struct sockaddr_in))
        {
          addr_len = sizeof(struct altcom_sockaddr_in);
        }
      else if (req->addrlen == sizeof(struct sockaddr_in6))
        {
          addr_len = sizeof(struct altcom_sockaddr_in6);
        }
      else
        {
          ret = -EINVAL;
          daemon_error_printf("invalid argment request addrlen = %d.\n",
                              req->addrlen);
          goto send_resp;
        }
    }
  else
    {
      ret = -EINVAL;
      daemon_error_printf("invalid argment request addrlen = %d.\n",
                          req->addrlen);
      goto send_resp;
    }

  rlen = read(fd, (FAR void *)&addr, req->addrlen);
  if (0 > rlen|| rlen < req->addrlen)
    {
      ret = -EFAULT;
      daemon_error_printf("read addr failed.\n");
      goto send_resp;
    }

  memset(&storage, 0, sizeof(struct altcom_sockaddr_storage));
  convsockaddr_remote((struct sockaddr *)&addr, &storage);

  ret = altcom_bind(usock->usockid,
                    (FAR const struct altcom_sockaddr*)&storage,
                    addr_len);
  if (0 > ret)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_bind() failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("altcom_bind() ret = %d\n", ret);

send_resp:
  /* Send ACK response. */
  memset(&resp, 0, sizeof(resp));
  resp.result = ret;
  daemon_debug_printf("%s resp.result = %d\n", __func__, ret);

  ret = _send_ack_common(fd, req->head.xid, &resp);
  if (0 > ret)
    {
      daemon_error_printf("_send_ack_common() ret = %d\n", ret);
      return ret;
    }

  daemon_debug_printf("%s: end \n", __func__);
  return OK;
}

/****************************************************************************
 * Name: listen_request
 ****************************************************************************/

static int listen_request(int fd, struct daemon_s *priv,  FAR void *hdrbuf)
{
  FAR struct usrsock_request_listen_s  *req   = hdrbuf;
  FAR struct usock_s                   *usock = NULL;
  struct     usrsock_message_req_ack_s resp;
  int                                  ret    = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("listen, xid = %d\n", req->head.xid);

  /* Check if this socket exists. */
  usock = daemon_socket_get(priv, req->usockid);
  if (!usock)
    {
      ret = -EBADFD;
      daemon_error_printf("get usock table failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("listen_request usockid = %d\n", usock->usockid);

  /* Check if this socket is connected. */
  if (CLOSED == usock->state)
    {
      ret = -ENETDOWN;
      daemon_error_printf("socket is not opened.\n");
      goto send_resp;
    }

  ret = altcom_listen(usock->usockid, req->backlog);
  if (0 > ret)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_listen() failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("altcom_listen() ret = %d\n", ret);

send_resp:
  /* Send ACK response. */
  memset(&resp, 0, sizeof(resp));
  resp.result = ret;
  daemon_debug_printf("%s resp.result = %d\n", __func__, ret);

  ret = _send_ack_common(fd, req->head.xid, &resp);
  if (0 > ret)
    {
      daemon_error_printf("_send_ack_common() ret = %d\n", ret);
      return ret;
    }

  daemon_debug_printf("%s: end \n", __func__);
  return OK;
}

/****************************************************************************
 * Name: accept_request
 ****************************************************************************/

static int accept_request(int fd, struct daemon_s *priv, FAR void *hdrbuf)
{
  FAR struct usrsock_request_accept_s      *req           = hdrbuf;
  FAR struct usock_s                       *usock         = NULL;
  FAR struct usock_s                       *new_usock     = NULL;
  struct     usrsock_message_datareq_ack_s resp;
  struct     altcom_sockaddr_storage       storage;
  struct     sockaddr_storage              tmpaddr;
  int                                      ret            = 0;
  int                                      val            = 0;
  int16_t                                  newsockfd      = -1;
  altcom_socklen_t                         addrlen;
  socklen_t                                output_addrlen = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("accept, xid = %d\n", req->head.xid);

  /* Check if this socket exists. */
  usock = daemon_socket_get(priv, req->usockid);
  if (!usock)
    {
      ret = -EBADFD;
      daemon_error_printf("get usock table failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("accept_request usockid = %d\n", usock->usockid);

  new_usock = daemon_socket_new(priv);
  if (!new_usock)
    {
      ret = -ENFILE;
      daemon_error_printf("allocate usock table failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("accept socket index = %d\n", new_usock->index);

  /* Check if this socket is connected. */
  if (CLOSED == usock->state)
    {
      ret = -ENETDOWN;
      daemon_error_printf("socket is not opened.\n");
      goto send_resp;
    }

  if (usock->domain == AF_INET)
    {
      addrlen = sizeof(struct altcom_sockaddr_in);
      output_addrlen = sizeof(struct sockaddr_in);
    }
  else
    {
      addrlen = sizeof(struct altcom_sockaddr_in6);
      output_addrlen = sizeof(struct sockaddr_in6);
    }

  memset(&storage, 0, sizeof(struct altcom_sockaddr_storage));
  newsockfd = altcom_accept(usock->usockid,
                            (FAR struct altcom_sockaddr*)&storage,
                            &addrlen);
  usock->flags &= ~USRSOCK_EVENT_RECVFROM_AVAIL;
  
  if (0 > newsockfd)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_accept() failed = %d\n", ret);
      setup_event(priv);
      goto send_resp;
    }
  daemon_debug_printf("altcom_accept(): newsockfd = %d\n", newsockfd);

  convstorage_local(&storage, (FAR struct sockaddr*)&tmpaddr);

  new_usock->usockid = newsockfd;
  new_usock->state = CONNECTED;
  new_usock->domain = usock->domain;
  new_usock->xid = req->head.xid;

  setup_event(priv);

  val = altcom_fcntl(new_usock->usockid, ALTCOM_GETFL, 0);
  if (0 > val)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_fcntl() errno = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("altcom_fcntl() val = %d\n", val);

  ret = altcom_fcntl(new_usock->usockid,
                      ALTCOM_SETFL, (val | ALTCOM_O_NONBLOCK));
  if (0 > ret)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_fcntl() errno = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("altcom_fcntl() ret = %d\n", ret);

send_resp:
  /* Send response. */
  memset(&resp, 0, sizeof(resp));
  resp.reqack.head.msgid = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  resp.reqack.head.flags = 0;
  resp.reqack.xid = req->head.xid;
  resp.reqack.result  = (0 > ret) ? ret : 2;

  if (ret < 0)
    {
      resp.valuelen = 0;
      resp.valuelen_nontrunc = 0;
      if ((new_usock != NULL) && (new_usock->usockid != -1))
        {
          altcom_close(new_usock->usockid);
        }
      daemon_socket_delete(priv, new_usock);
    }
  else
    {
      resp.valuelen = MIN(req->max_addrlen, output_addrlen);
      resp.valuelen_nontrunc = output_addrlen;
    }

  daemon_debug_printf("%s resp.result = %d\n", __func__, resp.reqack.result);

  ret = _write_to_usock(fd, &resp, sizeof(resp));
  if (0 > ret)
    {
      daemon_error_printf("_write_to_usock() ret = %d\n", ret);
      return ret;
    }

  if (resp.valuelen > 0)
    {
      ret = _write_to_usock(fd, &tmpaddr, resp.valuelen);
      if (0 > ret)
        {
          daemon_error_printf("_write_to_usock() ret = %d\n", ret);
        }
    }

  if (newsockfd >= 0)
    {
      ret = _write_to_usock(fd, &new_usock->index, sizeof(new_usock->index));
      if (0 > ret)
        {
          daemon_error_printf("_write_to_usock() ret = %d\n", ret);
        }
    }

  daemon_debug_printf("%s: end \n", __func__);
  return OK;
}

/****************************************************************************
 * Name: setsockopt_request
 ****************************************************************************/

static int setsockopt_request(int fd, struct daemon_s *priv, FAR void *hdrbuf)
{
  FAR struct usrsock_request_setsockopt_s *req   = hdrbuf;
  FAR struct usock_s                      *usock = NULL;
  struct     usrsock_message_req_ack_s    resp;
  struct     setsockopt_param             param;
  FAR void                                *value = NULL;
  int                                     ret    = 0;
  ssize_t                                 rlen;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("setsockopt, xid = %d\n", req->head.xid);

  /* Check if this socket exists. */
  usock = daemon_socket_get(priv, req->usockid);
  if (!usock)
    {
      ret = -EBADFD;
      daemon_error_printf("get usock table failed = %d\n", ret);
      goto send_resp;
    }

  /* Check if this socket is not opened. */
  if (CLOSED == usock->state)
    {
      ret = -ENETDOWN;
      daemon_error_printf("socket is not opened.\n");
      goto send_resp;
    }
  daemon_debug_printf("accept_request usockid = %d\n", usock->usockid);

  if (req->valuelen > 0)
    {
      value = malloc(req->valuelen);
      if (!value)
        {
          ret = -ENOBUFS;
          daemon_error_printf("buffer allocate failed.\n");
          goto send_resp;
        }

      rlen = read(fd, value, req->valuelen);
      if (0 > rlen || rlen < req->valuelen)
        {
          ret = -EFAULT;
          daemon_error_printf("read address failed.\n");
          goto send_resp;
        }
      ret = conv_setsockopt_param(req->level, req->option, &param);
      if (0 <= ret)
        {
          ret = altcom_setsockopt(usock->usockid, param.level,
                                  param.option, value,
                                  (altcom_socklen_t)req->valuelen);
          if (0 > ret)
            {
              ret = altcom_errno();
              ret = -ret;
              daemon_error_printf("altcom_setsockopt() ret = %d\n", ret);
              goto send_resp;
            }
          daemon_debug_printf("altcom_setsockopt() ret = %d\n", ret);
        }
    }
  else
    {
      ret = -EINVAL;
      daemon_error_printf("invalid argment request valuelen.\n");
      goto send_resp;
    }

send_resp:
  /* Send ACK response */
  memset(&resp, 0, sizeof(resp));
  resp.result = ret;
  daemon_debug_printf("%s resp.result = %d\n", __func__, ret);

  if (value)
    {
      free(value);
      value = NULL;
    }

  ret = _send_ack_common(fd, req->head.xid, &resp);
  if (0 > ret)
    {
      daemon_error_printf("_send_ack_common() ret = %d\n", ret);
      return ret;
    }

  daemon_debug_printf("%s: end \n", __func__);
  return ret;
}

/****************************************************************************
 * Name: getsockopt_request
 ****************************************************************************/

static int getsockopt_request(int fd, struct daemon_s *priv, FAR void *hdrbuf)
{
  FAR struct usrsock_request_getsockopt_s  *req   = hdrbuf;
  FAR struct usock_s                       *usock = NULL;
  struct     usrsock_message_datareq_ack_s resp;
  struct     setsockopt_param              param;
  FAR void                                 *value = NULL;
  altcom_socklen_t                         valuelen;
  int                                      ret    = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("getsockopt, xid = %d\n", req->head.xid);

  /* Check if this socket exists. */
  usock = daemon_socket_get(priv, req->usockid);
  if (!usock)
    {
      ret = -EBADFD;
      daemon_error_printf("get usock table failed = %d\n", ret);
      goto send_resp;
    }
  daemon_debug_printf("getsockopt_request usockid = %d\n", usock->usockid);

  /* Check if this socket is not opened. */
  if (CLOSED == usock->state)
    {
      ret = -ENETDOWN;
      daemon_error_printf("socket is not opened.\n");
      goto send_resp;
    }

  ret = conv_getsockopt_param(req->level, req->option, &param);
  if (ret < 0)
    {
      daemon_error_printf("invalid argment request.\n");
      goto send_resp;
    }

  if (req->max_valuelen > 0)
    {
      value = malloc(req->max_valuelen);
      valuelen = req->max_valuelen;
      if (!value)
        {
          ret = -ENOBUFS;
          daemon_error_printf("buffer allocate failed.\n");
          goto send_resp;
        }

      ret = altcom_getsockopt(usock->usockid,
                              param.level,
                              param.option,
                              value, &valuelen);
      daemon_debug_printf("altcom_getsockopt() ret = %d\n", ret);
      if (ret < 0)
        {
          ret = altcom_errno();
          ret = -ret;
          daemon_debug_printf("altcom_getsockopt() failed = %d\n", ret);
          goto send_resp;
        }
    }
  else
    {
      ret = -EINVAL;
      daemon_error_printf("invalid argment request max_valuelen.\n");
      goto send_resp;
    }

send_resp:
  /* Send response. */
  memset(&resp, 0, sizeof(resp));
  resp.reqack.head.msgid = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  resp.reqack.head.flags = 0;
  resp.reqack.result = ret;
  resp.reqack.xid = req->head.xid;

  if (ret >= 0)
    {
      resp.valuelen = MIN(req->max_valuelen, (socklen_t)valuelen);
      resp.valuelen_nontrunc = (socklen_t)valuelen;
    }
  else
    {
      resp.valuelen = 0;
      resp.valuelen_nontrunc = 0;
    }

  ret = _write_to_usock(fd, &resp, sizeof(resp));
  if (ret < 0)
    {
      daemon_error_printf("_write_to_usock() ret = %d\n", ret);
    }

  if (resp.valuelen > 0 && ret >= 0)
    {
      ret = _write_to_usock(fd, value, resp.valuelen);
      if (0 > ret)
        {
          daemon_error_printf("_write_to_usock() ret = %d\n", ret);
        }
    }

  if (value)
    {
      free(value);
      value = NULL;
    }

  daemon_debug_printf("%s: end \n", __func__);
  return ret;
}

/****************************************************************************
 * Name: getsockname_request
 ****************************************************************************/

static int getsockname_request(int fd, struct daemon_s *priv,
                                FAR void *hdrbuf)
{
  FAR struct usrsock_request_getsockname_s *req           = hdrbuf;
  FAR struct sockaddr_in6                  addr;
  FAR struct usock_s                       *usock;
  struct     usrsock_message_datareq_ack_s resp;
  struct     altcom_sockaddr_storage       storage;
  altcom_socklen_t                         altcom_addrlen = 0;
  socklen_t                                output_addrlen = 0;
  int                                      ret            = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("getsockname, xid = %d\n", req->head.xid);

  /* Check if this socket exists. */
  usock = daemon_socket_get(priv, req->usockid);
  if (!usock)
    {
      ret = -EBADFD;
      goto send_resp;
    }
  daemon_debug_printf("getsockname_request usockid = %d\n", usock->usockid);

  /* Check if this socket is connected. */
  if (CLOSED == usock->state)
    {
      ret = -ENETDOWN;
      daemon_debug_printf("socket is not opened.\n");
      goto send_resp;
    }

  if (usock->domain == AF_INET)
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
  memset(&addr, 0, sizeof(struct sockaddr_in6));

  ret = altcom_getsockname(usock->usockid,
                            (FAR struct altcom_sockaddr*)&storage,
                            &altcom_addrlen);
  if (0 > ret)
    {
      ret = altcom_errno();
      ret = -ret;
      daemon_error_printf("altcom_getsockname() failed = %d\n", ret);
      goto send_resp;
    }

  convstorage_local(&storage, (FAR struct sockaddr*)&addr);

send_resp:
  /* Send response. */
  memset(&resp, 0, sizeof(resp));
  resp.reqack.head.msgid = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  resp.reqack.head.flags = 0;
  resp.reqack.xid = req->head.xid;
  resp.reqack.result = ret;

  if (0 > ret)
    {
      resp.valuelen = 0;
      resp.valuelen_nontrunc = 0;
    }
  else
    {
      resp.valuelen = MIN(req->max_addrlen, output_addrlen);
      resp.valuelen_nontrunc = output_addrlen;
    }

  ret = _write_to_usock(fd, &resp, sizeof(resp));
  if (0 > ret)
    {
      daemon_error_printf("_write_to_usock() ret = %d\n", ret);
      return ret;
    }

  if (0 < resp.valuelen)
    {
      ret = _write_to_usock(fd, &addr, resp.valuelen);
      if (0 > ret)
        {
          daemon_error_printf("_write_to_usock() ret = %d\n", ret);
          return ret;
        }
    }

  daemon_debug_printf("%s: end \n", __func__);
  return OK;
}

/****************************************************************************
 * Name: getsockname_request
 ****************************************************************************/

static int getpeername_request(int fd, struct daemon_s *priv,
                                FAR void *hdrbuf)
{
  FAR struct usrsock_request_getpeername_s *req = hdrbuf;
  struct      usrsock_message_req_ack_s    resp;
  int                                      ret  = 0;

  /* altcom_getpeername() is not support */
  daemon_print_recvevt("getpeername, xid = %d\n", req->head.xid);

  memset(&resp, 0, sizeof(resp));
  resp.result = -ENOTSUP;
  ret = _send_ack_common(fd, req->head.xid, &resp);
  if (0 > ret)
    {
      daemon_error_printf("_send_ack_common() ret = %d\n", ret);
      return ret;
    }

  daemon_debug_printf("%s: end \n", __func__);
  return OK;
}

/****************************************************************************
 * Name: ioctl_request
 ****************************************************************************/

static int ioctl_request(int fd, struct daemon_s *priv, FAR void *hdrbuf)
{
  FAR struct usrsock_request_ioctl_s   *req = hdrbuf;
  struct usrsock_message_req_ack_s     resp;
  struct usrsock_message_datareq_ack_s resp_data;
  struct ifreq                         if_req;
  lte_pdn_t                            pdn_info = {};
  int                                  ret      = -EINVAL;
  int                                  i;
  bool                                 getreq   = false;

  DEBUGASSERT(priv);
  DEBUGASSERT(req);

  daemon_print_recvevt("ioctl, xid = %d\n", req->head.xid);

  switch (req->cmd)
    {
      case SIOCSIFFLAGS:
        break;

      default:
        break;
    }

  if (sizeof(struct ifreq) < req->arglen)
    {
      ret = -EFAULT;
      daemon_error_printf("read ifreq failed.\n");
      getreq = true;
    }
  else
    {
      ret = read(fd, (FAR void *)&if_req, req->arglen);
      if (0 > ret || ret < req->arglen)
        {
          ret = -EFAULT;
          daemon_error_printf("read ifreq failed.\n");
          getreq = true;
        }
    }

  if (false == getreq)
    {
      if (if_req.ifr_flags & IFF_UP)
        {
#ifdef CONFIG_LTE_DAEMON_SYNC_TIME
          altcom_set_report_localtime(localtime_callback);
#endif

          ret = altcom_activate_pdn_sync(&priv->apn, &pdn_info);
          if (0 > ret)
            {
              ret = altcom_errno();
              ret = -ret;
              daemon_error_printf("lte_activate_pdn_sync() failed = %d\n",
                                  ret);
              goto send_resp;
            }
          priv->net_dev.d_flags = IFF_UP;
          priv->session_id = pdn_info.session_id;
          for (i = 0; i < pdn_info.ipaddr_num; i++)
            {
#ifdef CONFIG_NET_IPv4
              if (LTE_IPTYPE_V4 == pdn_info.address[i].ip_type)
                {
                  inet_pton(AF_INET,
                            (FAR const char *)pdn_info.address[i].address,
                            (FAR void *)&priv->net_dev.d_ipaddr);
                }
#endif
#ifdef CONFIG_NET_IPv6
              if (LTE_IPTYPE_V6 == pdn_info.address[i].ip_type)
                {
                  inet_pton(AF_INET6,
                            (FAR const char *)pdn_info.address[i].address,
                            (FAR void *)&priv->net_dev.d_ipv6addr);
                }
#endif
            }
        }
      if (if_req.ifr_flags & IFF_DOWN)
        {
#ifdef CONFIG_LTE_DAEMON_SYNC_TIME
          altcom_set_report_localtime(NULL);
#endif

          ret = altcom_deactivate_pdn_sync(priv->session_id);
          if (0 > ret)
            {
              daemon_error_printf("lte_deactivate_pdn_sync() failed = %d\n",
                                  ret);
            }
          priv->net_dev.d_flags = IFF_DOWN;
          priv->session_id = -1;
#ifdef CONFIG_NET_IPv4
          memset(&priv->net_dev.d_ipaddr, 0,
                  sizeof(priv->net_dev.d_ipaddr));
#endif
#ifdef CONFIG_NET_IPv6
          memset(&priv->net_dev.d_ipv6addr, 0,
                  sizeof(priv->net_dev.d_ipv6addr));
#endif
        }
send_resp:
      /* Send ACK response */
      memset(&resp, 0, sizeof(resp));
      resp.result = ret;

      daemon_debug_printf("%s resp.result = %d\n", __func__, ret);

      ret = _send_ack_common(fd, req->head.xid, &resp);
      if (0 > ret)
        {
          return ret;
        }
    }
  else
    {
      memset(&resp_data, 0, sizeof(resp_data));
      resp_data.reqack.result = ret;

      resp_data.reqack.head.msgid = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
      resp_data.reqack.head.flags = 0;
      resp_data.reqack.xid = req->head.xid;

      resp_data.valuelen = MIN(req->arglen,
                                sizeof(if_req));
      resp_data.valuelen_nontrunc = sizeof(if_req);

      daemon_debug_printf("%s resp.result = %d\n", __func__, ret);

      ret = _write_to_usock(fd, &resp_data, sizeof(resp_data));
      if (0 > ret)
        {
          daemon_error_printf("_write_to_usock() ret = %d\n", ret);
          return ret;
        }

      if (0 <= resp_data.reqack.result)
        {
          ret = _write_to_usock(fd, (FAR void*)&if_req, resp_data.valuelen);
          if (0 > ret)
            {
              daemon_error_printf("_write_to_usock() ret = %d\n", ret);
              return ret;
            }
        }
    }

  daemon_debug_printf("%s: end \n", __func__);
  return OK;
}

/****************************************************************************
 * Name: forwarding_usock
 ****************************************************************************/

static int forwarding_usock(int dst_fd, int src_fd, struct daemon_s* priv)
{
  int ret = 0;
  union usock_msg_u
    {
      struct usrsock_message_common_s head;
      struct usrsock_message_socket_event_s event;
      struct usrsock_message_req_ack_s req_ack;
    } usock_msg;
  FAR struct usock_s *usock = NULL;

  ret = read(src_fd, &usock_msg.head, sizeof(struct usrsock_message_common_s));
  if (0 > ret)
    {
      return -errno;
    }

  if (usock_msg.head.msgid == USRSOCK_MESSAGE_SOCKET_EVENT)
    {
      ret = read(src_fd, &usock_msg.event.usockid,
                 sizeof(struct usrsock_message_socket_event_s)
                 - sizeof(struct usrsock_message_common_s));
      if (0 > ret)
        {
          return -errno;
        }

      usock = daemon_socket_get(priv, usock_msg.event.usockid);
      DEBUGASSERT(usock);

      if (usock->state != CLOSED)
        {
          ret = _write_to_usock(dst_fd, &usock_msg.event,
                                sizeof(usock_msg.event));
          if (0 > ret)
            {
              daemon_error_printf("_write_to_usock() ret = %d\n", ret);
              return ret;
            }
          daemon_print_sendevt("usock_msg.event.usockid = %d\n",
                                usock_msg.event.usockid );
          daemon_print_sendevt("usock_msg.event.events = %d\n",
                                usock_msg.event.events );
        }
      else
        {
          daemon_debug_printf("Not send because it is an event for closed fd\n");
        }
    }
  else if (usock_msg.head.msgid == USRSOCK_MESSAGE_RESPONSE_ACK)
    {
      ret = read(src_fd, &usock_msg.req_ack.xid,
                  sizeof(struct usrsock_message_req_ack_s)
                  - sizeof(struct usrsock_message_common_s));
      if (0 > ret)
        {
          return -errno;
        }

      ret = _write_to_usock(dst_fd, &usock_msg.req_ack,
                            sizeof(struct usrsock_message_req_ack_s));
      if (0 > ret)
        {
          daemon_error_printf("_write_to_usock() ret = %d\n", ret);
          return ret;
        }
      daemon_print_sendevt("usock_msg.req_ack.xid = %d\n",
                            usock_msg.req_ack.xid);
      daemon_print_sendevt("usock_msg.req_ack.result = %d\n",
                            usock_msg.req_ack.result);
    }

  return ret;
}

/****************************************************************************
 * Name: daemon_api_request
 ****************************************************************************/

static int daemon_api_request(int read_fd, struct daemon_s* priv)
{
  int ret = 0;
  int daemon_api_cmd;

  ret = read(read_fd, &daemon_api_cmd, sizeof(daemon_api_cmd));
  if (0 > ret || ret != sizeof(daemon_api_cmd))
    {
      return -errno;
    }

  daemon_print_recvevt("receive daemonapi_request %d\n", daemon_api_cmd);

  switch (daemon_api_cmd)
    {
      case DAEMONAPI_REQUEST_POWER_ON:
        ret = altcom_power_on();
        g_altcomresult = ret;
        break;
      case DAEMONAPI_REQUEST_POWER_OFF:
        ret = altcom_power_off();
        break;
      case DAEMONAPI_REQUEST_FIN:
        g_daemonisrunnning = false;
        break;
      default:
        daemon_error_printf("no match daemon_api_cmd\n");
        ret = -ENOTSUP;
        break;
    }

  if (0 > ret)
    {
      return -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: daemon_restart_cb
 ****************************************************************************/

static void daemon_restart_cb(uint32_t reason)
{
  int ret;

  daemon_debug_printf("daemon_restart_cb called reason by %d\n", reason);

  if (reason == LTE_RESTART_MODEM_INITIATED)
    {
      ret = daemon_socket_send_abort(g_daemon, g_daemon->event_outfd);
      if (0 > ret)
        {
          daemon_error_printf("daemon_socket_send_abort() ret = %d\n", ret);
        }
      if (g_daemon->selectid != -1)
        {
          altcom_select_async_cancel(g_daemon->selectid, false);
          g_daemon->selectid = -1;
        }
      g_daemon->net_dev.d_flags = IFF_DOWN;
#ifdef CONFIG_NET_IPv4
      memset(&g_daemon->net_dev.d_ipaddr, 0,
              sizeof(g_daemon->net_dev.d_ipaddr));
#endif
#ifdef CONFIG_NET_IPv6
      memset(&g_daemon->net_dev.d_ipv6addr, 0,
              sizeof(g_daemon->net_dev.d_ipv6addr));
#endif
    }

  if (reason == LTE_RESTART_USER_INITIATED)
    {
      sem_post(&g_daemon->sync_sem);
    }

  if (g_daemon->user_restart_cb != NULL)
    {
      g_daemon->user_restart_cb(reason);
    }

  return;
}

/****************************************************************************
 * Name: main_loop
 ****************************************************************************/

static int main_loop(FAR struct daemon_s *priv)
{
  struct pollfd fds[3];
  int  fd[3];
  int  ret;

  ret = netdev_register(&priv->net_dev, NET_LL_ETHERNET);
  if (0 > ret)
    {
      daemon_debug_printf("Failed to register network device = %d\n", ret);
      return ret;
    }

  fd[0] = open("/dev/usrsock", O_RDWR);
  ASSERT(fd[0] >= 0);

  ret = mkfifo(EVENT_PIPE, 0666);
  ASSERT(ret >= 0);

  priv->event_outfd = open(EVENT_PIPE, O_WRONLY);
  ASSERT(priv->event_outfd >= 0);

  fd[1] = open(EVENT_PIPE, O_RDONLY);
  ASSERT(fd[1] >= 0);

  daemon_debug_printf("open event pipe event_outfd: %d,\n",
                      priv->event_outfd);
  daemon_debug_printf("open event pipe event_infd: %d\n",
                      fd[1]);


  fd[2] = g_daemon->apireq_infd;

  ret = altcom_initialize();
  ASSERT(ret >= 0);

  close(priv->event_outfd);
  close(priv->apireq_outfd);

  ret = altcom_set_report_restart(daemon_restart_cb);
  ASSERT(ret >= 0);

  sem_post(&priv->sync_sem);

  daemon_debug_printf("LTE daemon is starting !\n");
  while (g_daemonisrunnning)
    {
      memset(fds, 0, sizeof(fds));

      /* Check events */

      fds[0].fd     = fd[0];
      fds[0].events = POLLIN;
      fds[1].fd     = fd[1];
      fds[1].events = POLLIN;
      fds[2].fd     = fd[2];
      fds[2].events = POLLIN;

      ret = poll(fds, sizeof(fds)/sizeof(fds[0]), -1);
      if (0 > ret)
        {
          daemon_error_printf("poll failed: %d\n",  errno);
          continue;
        }

      if (fds[0].revents & POLLIN)
        {
          ret = usrsock_request(fd[0], priv);
        }

      if (fds[1].revents & POLLIN)
        {
          ret = forwarding_usock(fd[0], fd[1], priv);
        }

      if (fds[2].revents & POLLIN)
        {
          ret = daemon_api_request(fd[2], priv);
        }
    }

  daemon_socket_send_abort(priv, fd[0]);
  if (priv->selectid != -1)
    {
      altcom_select_async_cancel(priv->selectid, true);
      priv->selectid = -1;
    }
  altcom_finalize();

  close(fd[1]);
  close(fd[0]);

  close(priv->apireq_infd);

  sem_destroy(&priv->sync_sem);
  netdev_unregister(&priv->net_dev);

  daemon_debug_printf("daemon finished.\n");

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_daemon
 ****************************************************************************/

int lte_daemon(int argc, FAR char *argv[])
{
  int  ret;

  ret = main_loop(g_daemon);

  return ret;
}

/****************************************************************************
 * Name: lte_daemon_init
 ****************************************************************************/

int32_t lte_daemon_init(lte_apn_setting_t *apn)
{
  int ret;
  int local_errno;

  if (false == g_daemonisrunnning)
    {
      g_daemon = calloc(sizeof(struct daemon_s), 1);
      if (!g_daemon)
        {
          daemon_error_printf("daemon allocate failed.\n");
          return -ENOBUFS;
        }
      g_daemon->selectid = -1;
      g_daemon->pid = -1;

      if (apn)
        {
          memcpy(&g_daemon->apn, apn, sizeof(lte_apn_setting_t));
        }

      ret = mkfifo(APIREQ_PIPE, 0666);
      if (0 > ret)
        {
          local_errno = errno;
          free(g_daemon);
          g_daemon = NULL;
          daemon_error_printf("mkfifo failed = %d\n", errno);
          return -local_errno;
        }

      g_daemon->apireq_outfd = open(APIREQ_PIPE, (O_WRONLY | O_CREAT));
      if (0 > g_daemon->apireq_outfd)
        {
          local_errno = errno;
          remove(APIREQ_PIPE);
          free(g_daemon);
          g_daemon = NULL;
          daemon_error_printf("pipe open failed = %d\n", errno);
          return -local_errno;
        }

      g_daemon->apireq_infd = open(APIREQ_PIPE, (O_RDONLY | O_CREAT));
      if (0 > g_daemon->apireq_infd)
        {
          local_errno = errno;
          close(g_daemon->apireq_outfd);
          remove(APIREQ_PIPE);
          free(g_daemon);
          g_daemon = NULL;
          daemon_error_printf("pipe open failed = %d\n", errno);
          return -local_errno;
        }
      daemon_debug_printf("open apireq pipe apireq_outfd: %d\n",
                          g_daemon->apireq_outfd);
      daemon_debug_printf("open apireq pipe apireq_infd: %d\n",
                          g_daemon->apireq_infd);


      ret = sem_init(&g_daemon->sync_sem, 0, 0);
      if (0 > ret)
        {
          local_errno = errno;
          close(g_daemon->apireq_outfd);
          close(g_daemon->apireq_infd);
          remove(APIREQ_PIPE);
          free(g_daemon);
          g_daemon = NULL;
          daemon_error_printf("semaphore init fail %d\n", errno);
          return -local_errno;
        }

      g_daemonisrunnning = true;
      g_daemon->pid = task_create("lte_daemon", CONFIG_LTE_DAEMON_TASK_PRIORITY,
                                  4096, lte_daemon, NULL);
      if (0 > g_daemon->pid)
        {
          local_errno = errno;
          close(g_daemon->apireq_outfd);
          close(g_daemon->apireq_infd);
          remove(APIREQ_PIPE);
          g_daemonisrunnning = false;
          sem_destroy(&g_daemon->sync_sem);
          free(g_daemon);
          g_daemon = NULL;
          return -local_errno;
        }
      sem_wait(&g_daemon->sync_sem);
    }
  else
    {
      return -EALREADY;
    }

  return 0;
}

/****************************************************************************
 * Name: daemon_power_on
 ****************************************************************************/

int32_t lte_daemon_power_on(void)
{
  int ret;
  int daemon_cmd_id = DAEMONAPI_REQUEST_POWER_ON;

  if (g_daemonisrunnning)
    {
      ret = write(g_daemon->apireq_outfd, &daemon_cmd_id,
                  sizeof(daemon_cmd_id));
      if (0 > ret)
        {
          daemon_error_printf("%s write failed: %d\n", __func__, errno);
          return -errno;
        }
      sem_wait(&g_daemon->sync_sem);
      ret = g_altcomresult;
      daemon_debug_printf("lte_daemon_power_on() =  %d\n", ret);
    }
  else
    {
      daemon_error_printf("lte_daemon is not running\n");
      ret = -EOPNOTSUPP;
    }

  return ret;
}



/****************************************************************************
 * Name: lte_daemon_set_cb
 ****************************************************************************/

int32_t lte_daemon_set_cb(restart_report_cb_t restart_callback)
{
  if (restart_callback == NULL)
    {
      return -EINVAL;
    }

  if (g_daemon == NULL)
    {
      return -EOPNOTSUPP;
    }
  else if (g_daemon->pid != -1)
    {
      g_daemon->user_restart_cb = restart_callback;
    }

  return 0;
}

/****************************************************************************
 * Name: lte_daemon_fin
 ****************************************************************************/

int32_t lte_daemon_fin(void)
{
  int ret;
  int rc;

  if (g_daemonisrunnning)
    {
      g_daemonisrunnning = false;
      ret = socket(AF_INET, SOCK_STREAM, 0);
      if (ret >= 0)
        {
          close(ret);
        }

      ret = waitpid(g_daemon->pid, &rc, 0);
      if (0 > ret)
        {
          if (errno != ECHILD)
            {
              daemon_error_printf("%s waitpid failed: %d\n", __func__,
                                  -errno);
              return -errno;
            }
          ret = 0;
        }
      close(g_daemon->apireq_infd);
      close(g_daemon->apireq_outfd);

      remove(EVENT_PIPE);
      remove(APIREQ_PIPE);
      g_daemon->pid = -1;

      if (g_daemon)
        {
          free(g_daemon);
          g_daemon = NULL;
        }
      ret = 0;
    }
  else
    {
      ret = -EALREADY;
      daemon_error_printf("lte_daemon_fin() failed =  %d\n", ret);
    }

  return ret;
}
