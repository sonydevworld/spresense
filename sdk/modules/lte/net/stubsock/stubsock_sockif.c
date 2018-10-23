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
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Parameters:
 *   psock    A pointer to a user allocated socket structure to be
 *            initialized.
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negater errno value is
 *   returned.
 *
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
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The non-negative set of socket cababilities is returned.
 *
 ****************************************************************************/

static sockcaps_t stubsock_sockif_sockcaps(FAR struct socket *psock)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return SOCKCAP_NONBLOCKING;
}


/****************************************************************************
 * Name: stubsock_sockif_addref
 *
 * Description:
 *   Increment the reference count on the underlying connection structure.
 *
 * Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stubsock_sockif_addref(FAR struct socket *psock)
{
  DBGIF_LOG1_INFO("%s\n", __func__);
}

/****************************************************************************
 * Name: stubsock_sockif_bind
 *
 * Description:
 *   stubsock_sockif_bind() gives the socket 'conn' the local address 'addr'.
 *   'addr' is 'addrlen' bytes long. Traditionally, this is called
 *   "assigning a name to a socket." When a socket is created with socket,
 *   it exists in a name space (address family) but has no name assigned.
 *
 * Parameters:
 *   psock    A reference to the socket structure of the socket to be bound
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     psock is a descriptor for a file, not a socket.
 *
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
 *
 * Description:
 *   The stubsock_sockif_getsockname() function retrieves the locally-bound
 *   name of the specified socket, stores this address in the sockaddr
 *   structure pointed to by the 'addr' argument, and stores the length of
 *   this address in the object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   psock    A reference to the socket structure
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
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
 *
 * Description:
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept().  For the case of AFINET
 *   and AFINET6 sockets, psock_listen() calls this function.  The
 *   psock_listen() call applies only to sockets of type SOCK_STREAM or
 *   SOCK_SEQPACKET.
 *
 * Parameters:
 *   psock    Reference to an internal, bound socket structure.
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, a negated errno value is
 *   returned.  See listen() for the set of appropriate error values.
 *
 ****************************************************************************/

static int stubsock_sockif_listen(FAR struct socket *psock,
                                  int backlog)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_listen(psock, backlog);
}

/****************************************************************************
 * Name: stubsock_sockif_connect
 *
 * Description:
 *   Perform a usrsock connection
 *
 * Parameters:
 *   psock - A reference to the socket structure of the socket to be connected
 *   addr    The address of the remote server to connect to
 *   addrlen Length of address buffer
 *
 * Returned Value:
 *   On success, zero is returned. On error, a negated errno value is
 *   returned.  See connect() for the set of appropriate error values.
 *
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
 *
 * Description:
 *   The stubsock_sockif_accept function is used with connection-based socket
 *   types (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an inet_accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, inet_accept blocks the caller until a
 *   connection is present. If the socket is marked non-blocking and no
 *   pending connections are present on the queue, inet_accept returns
 *   EAGAIN.
 *
 * Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *   newsock  Location to return the accepted socket information.
 *
 * Returned Value:
 *   Returns 0 (OK) on success.  On failure, it returns a negated errno
 *   value.  See accept() for a desrciption of the approriate error value.
 *
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
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
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
 *
 * Description:
 *   The stubsock_sockif_send() call may be used only when the socket is in
 *   a connected state  (so that the intended recipient is known).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send() for the list of appropriate error
 *   values.
 *
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
 *
 * Description:
 *   If stubsock_sockif_sendto() is used on a connection-mode (SOCK_STREAM,
 *   SOCK_SEQPACKET) socket, the parameters to and 'tolen' are ignored
 *   (and the error EISCONN may be returned when they are not NULL and 0),
 *   and the error ENOTCONN is returned when the socket was not actually
 *   connected.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags (ignored)
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
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
 *
 * Description:
 *   Performs the close operation on an socket instance
 *
 * Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 ****************************************************************************/

static int stubsock_sockif_close(FAR struct socket *psock)
{
  DBGIF_LOG1_INFO("%s\n", __func__);

  return stubsock_close(psock);
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
