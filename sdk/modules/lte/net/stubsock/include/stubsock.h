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
 *
 * Description:
 *   Initialize the stub Socket connection structures.
 *
 ****************************************************************************/

void stubsock_initialize(void);

/****************************************************************************
 * Name: stubsock_finalize()
 *
 * Description:
 *   Finalize the stub Socket connection structures.
 *
 ****************************************************************************/

void stubsock_finalize(void);

/****************************************************************************
 * Name: stubsock_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized stubsock connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct stubsock_conn_s *stubsock_alloc(void);

/****************************************************************************
 * Name: stubsock_free()
 *
 * Description:
 *   Free a stubsock connection structure that is no longer in use. This should
 *   be done by the implementation of close().
 *
 ****************************************************************************/

void stubsock_free(FAR struct stubsock_conn_s *conn);

/****************************************************************************
 * Name: stubsock_convdomain_remote()
 *
 * Description:
 *   Convert domain to remote definition.
 *
 ****************************************************************************/

int stubsock_convdomain_remote(int domain);

/****************************************************************************
 * Name: stubsock_convdomain_local()
 *
 * Description:
 *   Convert domain to local definition.
 *
 ****************************************************************************/

int stubsock_convdomain_local(int domain);

/****************************************************************************
 * Name: stubsock_convtype_remote()
 *
 * Description:
 *   Convert type to remote definition.
 *
 ****************************************************************************/

int stubsock_convtype_remote(int type);

/****************************************************************************
 * Name: stubsock_convtype_local()
 *
 * Description:
 *   Convert type to local definition.
 *
 ****************************************************************************/

int stubsock_convtype_local(int type);

/****************************************************************************
 * Name: stubsock_convproto_remote()
 *
 * Description:
 *   Convert protocol to remote definition.
 *
 ****************************************************************************/

int stubsock_convproto_remote(int protocol);

/****************************************************************************
 * Name: stubsock_convproto_local()
 *
 * Description:
 *   Convert protocol to local definition.
 *
 ****************************************************************************/

int stubsock_convproto_local(int protocol);

/****************************************************************************
 * Name: stubsock_convflags_remote()
 *
 * Description:
 *   Convert flags to remote definition.
 *
 ****************************************************************************/

int stubsock_convflags_remote(int flags);

/****************************************************************************
 * Name: stubsock_convflags_local()
 *
 * Description:
 *   Convert flags to local definition.
 *
 ****************************************************************************/

int stubsock_convflags_local(int flags);

/****************************************************************************
 * Name: stubsock_convaiflags_remote()
 *
 * Description:
 *   Convert ai_flags to remote definition.
 *
 ****************************************************************************/

int stubsock_convaiflags_remote(int ai_flags);

/****************************************************************************
 * Name: stubsock_convsockaddr_remote()
 *
 * Description:
 *   Convert sockaddr to remote definition.
 *
 ****************************************************************************/

void stubsock_convsockaddr_remote(FAR const struct sockaddr *from,
                                  FAR struct altcom_sockaddr_storage *to);

/****************************************************************************
 * Name: stubsock_convstorage_local()
 *
 * Description:
 *   Convert sockaddr_storage to local definition.
 *
 ****************************************************************************/

void stubsock_convstorage_local(FAR const struct altcom_sockaddr_storage *from,
                                FAR struct sockaddr *to);

/****************************************************************************
 * Name: stubsock_convherrno_local()
 *
 * Description:
 *   Convert h_errno to local definition.
 *
 ****************************************************************************/

int stubsock_convherrno_local(int herr);

/****************************************************************************
 * Name: stubsock_accept
 *
 * Description:
 *   The stubsock_accept function is used with connection-based socket
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

int stubsock_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                    FAR socklen_t *addrlen, FAR struct socket *newsock);

/****************************************************************************
 * Name: stubsock_bind
 *
 * Description:
 *   stubsock_bind() gives the socket 'conn' the local address 'addr'. 'addr'
 *   is 'addrlen' bytes long. Traditionally, this is called "assigning a name
 *   to a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Parameters:
 *   conn     usrsock socket connection structure
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 ****************************************************************************/

int stubsock_bind(FAR struct socket *psock, FAR const struct sockaddr *addr,
                  socklen_t addrlen);

/****************************************************************************
 * Name: stubsock_close
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

int stubsock_close(FAR struct socket *psock);

/****************************************************************************
 * Name: stubsock_connect
 *
 * Description:
 *   Perform a stubsock connection
 *
 * Parameters:
 *   psock - A reference to the socket structure of the socket to be connected
 *   addr    The address of the remote server to connect to
 *   addrlen Length of address buffer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int stubsock_connect(FAR struct socket *psock,
                     FAR const struct sockaddr *addr,
                     socklen_t addrlen);

/****************************************************************************
 * Name: stubsock_getsockname
 *
 * Description:
 *   The getsockname() function retrieves the locally-bound name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   conn     usrsock socket connection structure
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 ****************************************************************************/

int stubsock_getsockname(FAR struct socket *psock,
                         FAR struct sockaddr *addr, FAR socklen_t *addrlen);

/****************************************************************************
 * Name: stubsock_getsockopt
 *
 * Description:
 *   getsockopt() retrieve thse value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument. If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Parameters:
 *   psock     Socket instance
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 ****************************************************************************/

int stubsock_getsockopt(FAR struct socket *psock, int level, int option,
                        FAR void *value, FAR socklen_t *value_len);

/****************************************************************************
 * Name: stubsock_listen
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

int stubsock_listen(FAR struct socket *psock, int backlog);

/****************************************************************************
 * Name: stubsock_poll
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

int stubsock_poll(FAR struct socket *psock, FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Name: stubsock_recvfrom
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

ssize_t stubsock_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                          int flags, FAR struct sockaddr *from,
                          FAR socklen_t *fromlen);

/****************************************************************************
 * Name: stubsock_send
 *
 * Description:
 *   The stubsock_send() call may be used only when the socket is in
 *   a connected state  (so that the intended recipient is known).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 ****************************************************************************/

ssize_t stubsock_send(FAR struct socket *psock, FAR const void *buf,
                      size_t len, int flags);

/****************************************************************************
 * Name: stubsock_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
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

ssize_t stubsock_sendto(FAR struct socket *psock, FAR const void *buf,
                        size_t len, int flags, FAR const struct sockaddr *to,
                        socklen_t tolen);

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
                        FAR const void *value, FAR socklen_t value_len);

/****************************************************************************
 * Name: stubsock_socket
 *
 * Description:
 *   socket() creates an endpoint for communication and returns a socket
 *   structure.
 *
 * Parameters:
 *   domain   (see sys/socket.h)
 *   type     (see sys/socket.h)
 *   protocol (see sys/socket.h)
 *   psock    A pointer to a user allocated socket structure to be initialized.
 *
 * Returned Value:
 *   0 on success; negative error-code on error
 *
 *   EACCES
 *     Permission to create a socket of the specified type and/or protocol
 *     is denied.
 *   EAFNOSUPPORT
 *     The implementation does not support the specified address family.
 *   EINVAL
 *     Unknown protocol, or protocol family not available.
 *   EMFILE
 *     Process file table overflow.
 *   ENFILE
 *     The system limit on the total number of open files has been reached.
 *   ENOBUFS or ENOMEM
 *     Insufficient memory is available. The socket cannot be created until
 *     sufficient resources are freed.
 *   EPROTONOSUPPORT
 *     The protocol type or the specified protocol is not supported within
 *     this domain.
 *
 * Assumptions:
 *
 ****************************************************************************/

int stubsock_socket(int domain, int type, int protocol,
                    FAR struct socket *psock);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif

#endif /* __MODULES_LTE_NET_STUBSOCK_INCLUDE_STUBSOCK_H */
