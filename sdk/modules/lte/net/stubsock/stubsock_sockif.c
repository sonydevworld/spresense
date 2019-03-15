/****************************************************************************
 * modules/lte/net/stubsock/stubsock_sockif.c
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
#include "dbg_if.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        stubsock_sockif_setup(FAR struct socket *psock,
                                        int protocol);
static sockcaps_t stubsock_sockif_sockcaps(FAR struct socket *psock);
static void       stubsock_sockif_addref(FAR struct socket *psock);
static int        stubsock_sockif_bind(FAR struct socket *psock,
                                       FAR const struct sockaddr *addr,
                                       socklen_t addrlen);
static int        stubsock_sockif_getsockname(FAR struct socket *psock,
                                              FAR struct sockaddr *addr,
                                              FAR socklen_t *addrlen);
static int        stubsock_sockif_listen(FAR struct socket *psock,
                                         int backlog);
static int        stubsock_sockif_connect(FAR struct socket *psock,
                                          FAR const struct sockaddr *addr,
                                          socklen_t addrlen);
static int        stubsock_sockif_accept(FAR struct socket *psock,
                                         FAR struct sockaddr *addr,
                                         FAR socklen_t *addrlen,
                                         FAR struct socket *newsock);
#ifndef CONFIG_DISABLE_POLL
static int        stubsock_sockif_poll(FAR struct socket *psock,
                                       FAR struct pollfd *fds, bool setup);
#endif
static ssize_t    stubsock_sockif_send(FAR struct socket *psock,
                                       FAR const void *buf, size_t len,
                                       int flags);
static ssize_t    stubsock_sockif_sendto(FAR struct socket *psock,
                                         FAR const void *buf, size_t len,
                                         int flags,
                                         FAR const struct sockaddr *to,
                                         socklen_t tolen);
static ssize_t    stubsock_sockif_recvfrom(FAR struct socket *psock,
                                           FAR void *buf, size_t len,
                                           int flags,
                                           FAR struct sockaddr *from,
                                           FAR socklen_t *fromlen);
static int        stubsock_sockif_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_stubsock_sockif =
{
  stubsock_sockif_setup,       /* si_setup */
  stubsock_sockif_sockcaps,    /* si_sockcaps */
  stubsock_sockif_addref,      /* si_addref */
  stubsock_sockif_bind,        /* si_bind */
  stubsock_sockif_getsockname, /* si_getsockname */
  stubsock_sockif_listen,      /* si_listen */
  stubsock_sockif_connect,     /* si_connect */
  stubsock_sockif_accept,      /* si_accept */
#ifndef CONFIG_DISABLE_POLL
  stubsock_sockif_poll,        /* si_poll */
#endif
  stubsock_sockif_send,        /* si_send */
  stubsock_sockif_sendto,      /* si_sendto */
#ifdef CONFIG_NET_SENDFILE
  NULL,                        /* si_sendfile */
#endif
  stubsock_sockif_recvfrom,    /* si_recvfrom */
  stubsock_sockif_close        /* si_close */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stubsock_sockif_setup
 ****************************************************************************/

static int stubsock_sockif_setup(FAR struct socket *psock, int protocol)
{
  FAR struct devspecsock_conn_s *ds_conn =
    (FAR struct devspecsock_conn_s*)psock->s_conn;
  int domain = psock->s_domain;
  int type = ds_conn->s_type;
  int ret;

  DBGIF_LOG1_INFO("%s\n", __func__);

  ret = stubsock_socket(domain, type, protocol, psock);

  return ret;
}

/****************************************************************************
 * Name: stubsock_sockif_sockcaps
 ****************************************************************************/

static sockcaps_t stubsock_sockif_sockcaps(FAR struct socket *psock)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return SOCKCAP_NONBLOCKING;
}


/****************************************************************************
 * Name: stubsock_sockif_addref
 ****************************************************************************/

static void stubsock_sockif_addref(FAR struct socket *psock)
{
  DBGIF_LOG1_INFO("%s\n", __func__);
}

/****************************************************************************
 * Name: stubsock_sockif_bind
 ****************************************************************************/

static int stubsock_sockif_bind(FAR struct socket *psock,
                                FAR const struct sockaddr *addr,
                                socklen_t addrlen)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_bind(psock, addr, addrlen);
}

/****************************************************************************
 * Name: stubsock_sockif_getsockname
 ****************************************************************************/

static int stubsock_sockif_getsockname(FAR struct socket *psock,
                                       FAR struct sockaddr *addr,
                                       FAR socklen_t *addrlen)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_getsockname(psock, addr, addrlen);
}

/****************************************************************************
 * Name: stubsock_sockif_listen
 ****************************************************************************/

static int stubsock_sockif_listen(FAR struct socket *psock,
                                  int backlog)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_listen(psock, backlog);
}

/****************************************************************************
 * Name: stubsock_sockif_connect
 ****************************************************************************/

static int stubsock_sockif_connect(FAR struct socket *psock,
                                   FAR const struct sockaddr *addr,
                                   socklen_t addrlen)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_connect(psock, addr, addrlen);
}

/****************************************************************************
 * Name: stubsock_sockif_accept
 ****************************************************************************/

static int stubsock_sockif_accept(FAR struct socket *psock,
                                  FAR struct sockaddr *addr,
                                  FAR socklen_t *addrlen,
                                  FAR struct socket *newsock)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_accept(psock, addr, addrlen, newsock);
}

#ifndef CONFIG_DISABLE_POLL

/****************************************************************************
 * Name: stubsock_sockif_poll
 ****************************************************************************/

static int stubsock_sockif_poll(FAR struct socket *psock,
                                FAR struct pollfd *fds, bool setup)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_poll(psock, fds, setup);
}

#endif

/****************************************************************************
 * Name: stubsock_sockif_send
 ****************************************************************************/

static ssize_t stubsock_sockif_send(FAR struct socket *psock,
                                    FAR const void *buf, size_t len,
                                    int flags)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_send(psock, buf, len, flags);
}

/****************************************************************************
 * Name: stubsock_sockif_sendto
 ****************************************************************************/

static ssize_t stubsock_sockif_sendto(FAR struct socket *psock,
                                      FAR const void *buf, size_t len,
                                      int flags,
                                      FAR const struct sockaddr *to,
                                      socklen_t tolen)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_sendto(psock, buf, len, flags, to, tolen);
}

/****************************************************************************
 * Name: stubsock_sockif_recvfrom
 ****************************************************************************/

static ssize_t stubsock_sockif_recvfrom(FAR struct socket *psock,
                                        FAR void *buf, size_t len,
                                        int flags,
                                        FAR struct sockaddr *from,
                                        FAR socklen_t *fromlen)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_recvfrom(psock, buf, len, flags, from, fromlen);
}

/****************************************************************************
 * Name: stubsock_sockif_close
 ****************************************************************************/

static int stubsock_sockif_close(FAR struct socket *psock)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_close(psock);
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
