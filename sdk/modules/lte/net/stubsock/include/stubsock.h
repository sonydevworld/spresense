/****************************************************************************
 * modules/lte/net/stubsock/include/stubsock.h
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

#ifndef __MODULES_LTE_NET_STUBSOCK_INCLUDE_STUBSOCK_H
#define __MODULES_LTE_NET_STUBSOCK_INCLUDE_STUBSOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sdk/config.h>

#ifdef CONFIG_NET

#include <netdb.h>
#include "socket/socket.h"
#include "stubsock_mem.h"
#include "altcom_socket.h"
#include "altcom_netdb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SOCK_SUTBSOCK_TYPE  0x6f

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct stubsock_conn_s
{
  int32_t  stubsockid; /* Used for altcom socket */
  uint16_t flags;      /* Socket state flags */
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const struct sock_intf_s g_stubsock_sockif;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stubsock_initialize()
 ****************************************************************************/

void stubsock_initialize(void);

/****************************************************************************
 * Name: stubsock_finalize()
 ****************************************************************************/

void stubsock_finalize(void);

/****************************************************************************
 * Name: stubsock_alloc()
 ****************************************************************************/

FAR struct stubsock_conn_s *stubsock_alloc(void);

/****************************************************************************
 * Name: stubsock_free()
 ****************************************************************************/

void stubsock_free(FAR struct stubsock_conn_s *conn);

/****************************************************************************
 * Name: stubsock_convdomain_remote()
 ****************************************************************************/

int stubsock_convdomain_remote(int domain);

/****************************************************************************
 * Name: stubsock_convdomain_local()
 ****************************************************************************/

int stubsock_convdomain_local(int domain);

/****************************************************************************
 * Name: stubsock_convtype_remote()
 ****************************************************************************/

int stubsock_convtype_remote(int type);

/****************************************************************************
 * Name: stubsock_convtype_local()
 ****************************************************************************/

int stubsock_convtype_local(int type);

/****************************************************************************
 * Name: stubsock_convproto_remote()
 ****************************************************************************/

int stubsock_convproto_remote(int protocol);

/****************************************************************************
 * Name: stubsock_convproto_local()
 ****************************************************************************/

int stubsock_convproto_local(int protocol);

/****************************************************************************
 * Name: stubsock_convflags_remote()
 ****************************************************************************/

int stubsock_convflags_remote(int flags);

/****************************************************************************
 * Name: stubsock_convflags_local()
 ****************************************************************************/

int stubsock_convflags_local(int flags);

/****************************************************************************
 * Name: stubsock_convaiflags_remote()
 ****************************************************************************/

int stubsock_convaiflags_remote(int ai_flags);

/****************************************************************************
 * Name: stubsock_convsockaddr_remote()
 ****************************************************************************/

void stubsock_convsockaddr_remote(FAR const struct sockaddr *from,
                                  FAR struct altcom_sockaddr_storage *to);

/****************************************************************************
 * Name: stubsock_convstorage_local()
 ****************************************************************************/

void stubsock_convstorage_local(FAR const struct altcom_sockaddr_storage *from,
                                FAR struct sockaddr *to);

/****************************************************************************
 * Name: stubsock_convherrno_local()
 ****************************************************************************/

int stubsock_convherrno_local(int herr);

/****************************************************************************
 * Name: stubsock_accept
 ****************************************************************************/

int stubsock_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                    FAR socklen_t *addrlen, FAR struct socket *newsock);

/****************************************************************************
 * Name: stubsock_bind
 ****************************************************************************/

int stubsock_bind(FAR struct socket *psock, FAR const struct sockaddr *addr,
                  socklen_t addrlen);

/****************************************************************************
 * Name: stubsock_close
 ****************************************************************************/

int stubsock_close(FAR struct socket *psock);

/****************************************************************************
 * Name: stubsock_connect
 ****************************************************************************/

int stubsock_connect(FAR struct socket *psock,
                     FAR const struct sockaddr *addr,
                     socklen_t addrlen);

/****************************************************************************
 * Name: stubsock_getsockname
 ****************************************************************************/

int stubsock_getsockname(FAR struct socket *psock,
                         FAR struct sockaddr *addr, FAR socklen_t *addrlen);

/****************************************************************************
 * Name: stubsock_getsockopt
 ****************************************************************************/

int stubsock_getsockopt(FAR struct socket *psock, int level, int option,
                        FAR void *value, FAR socklen_t *value_len);

/****************************************************************************
 * Name: stubsock_listen
 ****************************************************************************/

int stubsock_listen(FAR struct socket *psock, int backlog);

/****************************************************************************
 * Name: stubsock_poll
 ****************************************************************************/

int stubsock_poll(FAR struct socket *psock, FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Name: stubsock_recvfrom
 ****************************************************************************/

ssize_t stubsock_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                          int flags, FAR struct sockaddr *from,
                          FAR socklen_t *fromlen);

/****************************************************************************
 * Name: stubsock_send
 ****************************************************************************/

ssize_t stubsock_send(FAR struct socket *psock, FAR const void *buf,
                      size_t len, int flags);

/****************************************************************************
 * Name: stubsock_sendto
 ****************************************************************************/

ssize_t stubsock_sendto(FAR struct socket *psock, FAR const void *buf,
                        size_t len, int flags, FAR const struct sockaddr *to,
                        socklen_t tolen);

/****************************************************************************
 * Name: stubsock_setsockopt
 ****************************************************************************/

int stubsock_setsockopt(FAR struct socket *psock, int level, int option,
                        FAR const void *value, FAR socklen_t value_len);

/****************************************************************************
 * Name: stubsock_socket
 ****************************************************************************/

int stubsock_socket(int domain, int type, int protocol,
                    FAR struct socket *psock);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif

#endif /* __MODULES_LTE_NET_STUBSOCK_INCLUDE_STUBSOCK_H */
