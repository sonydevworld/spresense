/****************************************************************************
 * modules/include/lte/altcom/net/altcom_socket.h
 *
 *   Copyright 2018, 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_INCLUDE_LTE_ALTCOM_NET_ALTCOM_SOCKET_H
#define __MODULES_INCLUDE_LTE_ALTCOM_NET_ALTCOM_SOCKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of sockets */

#define ALTCOM_NSOCKET             10

/* Address family */

#define ALTCOM_AF_UNSPEC           0
#define ALTCOM_AF_INET             2
#define ALTCOM_AF_INET6            10
#define ALTCOM_PF_INET             ALTCOM_AF_INET
#define ALTCOM_PF_INET6            ALTCOM_AF_INET6
#define ALTCOM_PF_UNSPEC           ALTCOM_AF_UNSPEC

/* Socket protocol type */

#define ALTCOM_SOCK_STREAM         1
#define ALTCOM_SOCK_DGRAM          2
#define ALTCOM_SOCK_RAW            3
#define ALTCOM_SOCK_DGRAM_DTLS     130

/* Protocol */

#define ALTCOM_IPPROTO_IP          0
#define ALTCOM_IPPROTO_ICMP        1
#define ALTCOM_IPPROTO_TCP         6
#define ALTCOM_IPPROTO_UDP         17
#define ALTCOM_IPPROTO_IPV6        41
#define ALTCOM_IPPROTO_ICMPV6      58
#define ALTCOM_IPPROTO_UDPLITE     136
#define ALTCOM_IPPROTO_RAW         255

/* Flags */

#define ALTCOM_MSG_PEEK            0x01
#define ALTCOM_MSG_WAITALL         0x02
#define ALTCOM_MSG_OOB             0x04
#define ALTCOM_MSG_DONTWAIT        0x08
#define ALTCOM_MSG_MORE            0x10

/* How */

#define ALTCOM_SHUT_RD             0
#define ALTCOM_SHUT_WR             1
#define ALTCOM_SHUT_RDWR           2

/* Level */

#define ALTCOM_SOL_SOCKET          0xfff

/* Option flags per-socket */

#define ALTCOM_SO_REUSEADDR        0x0004
#define ALTCOM_SO_KEEPALIVE        0x0008
#define ALTCOM_SO_BROADCAST        0x0020

/* Additional options, not kept in so_options */

#define ALTCOM_SO_ACCEPTCONN       0x0002
#define ALTCOM_SO_LINGER           0x0080
#define ALTCOM_SO_RCVBUF           0x1002
#define ALTCOM_SO_SNDTIMEO         0x1005
#define ALTCOM_SO_RCVTIMEO         0x1006
#define ALTCOM_SO_ERROR            0x1007
#define ALTCOM_SO_TYPE             0x1008
#define ALTCOM_SO_NO_CHECK         0x100a

/* Options for level IPPROTO_IP */

#define ALTCOM_IP_TOS              1
#define ALTCOM_IP_TTL              2

/* Options and types related to multicast membership */

#define ALTCOM_IP_ADD_MEMBERSHIP   3
#define ALTCOM_IP_DROP_MEMBERSHIP  4

/* Options and types for UDP multicast traffic handling */

#define ALTCOM_IP_MULTICAST_TTL    5
#define ALTCOM_IP_MULTICAST_IF     6
#define ALTCOM_IP_MULTICAST_LOOP   7

/* Options for level ALTCOM_IPPROTO_TCP */

#define ALTCOM_TCP_NODELAY         0x01
#define ALTCOM_TCP_KEEPALIVE       0x02
#define ALTCOM_TCP_KEEPIDLE        0x03
#define ALTCOM_TCP_KEEPINTVL       0x04
#define ALTCOM_TCP_KEEPCNT         0x05

/* Options for level ALTCOM_IPPROTO_IPV6 */

#define ALTCOM_IPV6_CHECKSUM       7
#define ALTCOM_IPV6_V6ONLY         27

/* Options for level ALTCOM_IPPROTO_UDP */

#define ALTCOM_UDP_DTLS_SRTP_RECEPTION 0x01

/* Options for level ALTCOM_IPPROTO_UDPLITE */

#define ALTCOM_UDPLITE_SEND_CSCOV  0x01
#define ALTCOM_UDPLITE_RECV_CSCOV  0x02

/* Macros for fcntl */

#define ALTCOM_GETFL               3
#define ALTCOM_SETFL               4

#define ALTCOM_O_NONBLOCK          0x01

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint8_t  altcom_sa_family_t;
typedef uint32_t altcom_socklen_t;
typedef uint16_t altcom_in_port_t;

struct altcom_sockaddr
{
  uint8_t            sa_len;
  altcom_sa_family_t sa_family;
  char               sa_data[14];
};

struct altcom_sockaddr_storage
{
  uint8_t            s2_len;
  altcom_sa_family_t ss_family;
  char               s2_data1[2];
  uint32_t           s2_data2[3];
  uint32_t           s2_data3[3];
};

struct altcom_linger
{
  int                l_onoff;
  int                l_linger;
};

struct altcom_timeval
{
  long               tv_sec;
  long               tv_usec;
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_accept
 ****************************************************************************/

int altcom_accept(int sockfd, struct altcom_sockaddr *addr,
                  altcom_socklen_t *addrlen);

/****************************************************************************
 * Name: altcom_bind
 ****************************************************************************/

int altcom_bind(int sockfd, const struct altcom_sockaddr *addr,
                altcom_socklen_t addrlen);

/****************************************************************************
 * Name: altcom_close
 ****************************************************************************/

int altcom_close(int sockfd);

/****************************************************************************
 * Name: altcom_connect
 ****************************************************************************/

int altcom_connect(int sockfd, const struct altcom_sockaddr *addr,
                   altcom_socklen_t addrlen);

/****************************************************************************
 * Name: altcom_fcntl
 ****************************************************************************/

int altcom_fcntl(int sockfd, int cmd, int val);

/****************************************************************************
 * Name: altcom_getsockname
 ****************************************************************************/

int altcom_getsockname(int sockfd, struct altcom_sockaddr *addr,
                       altcom_socklen_t *addrlen);

/****************************************************************************
 * Name: altcom_getsockopt
 ****************************************************************************/

int altcom_getsockopt(int sockfd, int level, int option, void *value,
                      altcom_socklen_t *value_len);

/****************************************************************************
 * Name: altcom_ioctl
 ****************************************************************************/

int altcom_ioctl(int sockfd, long req, void *argp);

/****************************************************************************
 * Name: altcom_listen
 ****************************************************************************/

int altcom_listen(int sockfd, int backlog);

/****************************************************************************
 * Name: altcom_read
 ****************************************************************************/

int altcom_read(int sockfd, void *buf, size_t len);

/****************************************************************************
 * Name: altcom_recv
 ****************************************************************************/

int altcom_recv(int sockfd, void *buf, size_t len, int flags);

/****************************************************************************
 * Name: altcom_recvfrom
 ****************************************************************************/

int altcom_recvfrom(int sockfd, void *buf, size_t len, int flags,
                    struct altcom_sockaddr *from, altcom_socklen_t *fromlen);

/****************************************************************************
 * Name: altcom_send
 ****************************************************************************/

int altcom_send(int sockfd, const void *buf, size_t len, int flags);

/****************************************************************************
 * Name: altcom_sendto
 ****************************************************************************/

int altcom_sendto(int sockfd, const void *buf, size_t len, int flags,
                  const struct altcom_sockaddr *to, altcom_socklen_t tolen);

/****************************************************************************
 * Name: altcom_setsockopt
 ****************************************************************************/

int altcom_setsockopt(int sockfd, int level, int option, const void *value,
                      altcom_socklen_t value_len);

/****************************************************************************
 * Name: altcom_shutdown
 ****************************************************************************/

int altcom_shutdown(int sockfd, int how);

/****************************************************************************
 * Name: altcom_socket
 ****************************************************************************/

int altcom_socket(int domain, int type, int protocol);

/****************************************************************************
 * Name: altcom_write
 ****************************************************************************/

int altcom_write(int sockfd, const void *buf, size_t len);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_INCLUDE_LTE_ALTCOM_NET_ALTCOM_SOCKET_H */
