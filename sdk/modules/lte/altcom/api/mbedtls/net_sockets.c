/****************************************************************************
 * modules/lte/altcom/api/mbedtls/net_sockets.c
 *
 *   Copyright 2018 Sony Corporation
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
//#include "dbg_if.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "altcom_socket.h"
#include "altcom_in.h"
#include "altcom_inet.h"
#include "altcom_netdb.h"
#include "mbedtls/net.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FCNTL2(fd, cmd)       altcom_fcntl(fd, cmd, 0)
#define FCNTL3(fd, cmd, val)  altcom_fcntl(fd, cmd, val)

#define MSVC_INT_CAST

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
// modified
static int net_prepare( void )
{
    return( 0 );
}

// modified
static int net_would_block( const mbedtls_net_context *ctx )
{
	int32_t err = altcom_errno();
    /*
     * Never return 'WOULD BLOCK' on a non-blocking socket
     */
    if( ( FCNTL2( ctx->alt_fd, ALTCOM_GETFL ) & ALTCOM_O_NONBLOCK ) != ALTCOM_O_NONBLOCK )
        return( 0 );

    switch( err )
    {
        case ALTCOM_EAGAIN:
            return( 1 );
    }
    return( 0 );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
// original
void mbedtls_net_init(mbedtls_net_context *ctx)
{
    ctx->fd = -1;
    ctx->alt_fd = -1;
}

// original
int mbedtls_net_connect(mbedtls_net_context *ctx, const char *host, const char *port, int proto)
{
    int ret;
    struct altcom_addrinfo hints, *addr_list, *cur;

    if( ( ret = net_prepare() ) != 0 )
        return( ret );

    /* Do name resolution with both IPv6 and IPv4 */
    memset( &hints, 0, sizeof( hints ) );
    hints.ai_family = ALTCOM_AF_UNSPEC;
    hints.ai_socktype = proto == MBEDTLS_NET_PROTO_UDP ? ALTCOM_SOCK_DGRAM : ALTCOM_SOCK_STREAM;
    hints.ai_protocol = proto == MBEDTLS_NET_PROTO_UDP ? ALTCOM_IPPROTO_UDP : ALTCOM_IPPROTO_TCP;

    if( altcom_getaddrinfo( host, port, &hints, &addr_list ) != 0 )
        return( MBEDTLS_ERR_NET_UNKNOWN_HOST );

    /* Try the sockaddrs until a connection succeeds */
    ret = MBEDTLS_ERR_NET_UNKNOWN_HOST;
    for( cur = addr_list; cur != NULL; cur = cur->ai_next )
    {
        ctx->alt_fd = (int) altcom_socket( cur->ai_family, cur->ai_socktype,
                            cur->ai_protocol );
        if( ctx->alt_fd < 0 )
        {
            ret = MBEDTLS_ERR_NET_SOCKET_FAILED;
            continue;
        }
        mbedtls_net_alt2fd(ctx);

        if( altcom_connect( ctx->alt_fd, cur->ai_addr, MSVC_INT_CAST cur->ai_addrlen ) == 0 )
        {
            ret = 0;
            break;
        }

        altcom_close( ctx->alt_fd );
        ret = MBEDTLS_ERR_NET_CONNECT_FAILED;
    }

    altcom_freeaddrinfo( addr_list );
    return( ret );
}

int mbedtls_net_dtls_connect(mbedtls_net_context *ctx, const char *host, const char *port, int proto, int source_port)
{
    int ret;
    struct altcom_addrinfo hints, *addr_list, *cur;
    struct altcom_sockaddr_in sin;

    if( ( ret = net_prepare() ) != 0 )
        return( ret );

    /* Do name resolution with both IPv6 and IPv4 */
    memset( &hints, 0, sizeof( hints ) );
    hints.ai_family = ALTCOM_AF_UNSPEC;
    hints.ai_socktype = proto == MBEDTLS_NET_PROTO_UDP ? ALTCOM_SOCK_DGRAM : ALTCOM_SOCK_STREAM;
    hints.ai_protocol = proto == MBEDTLS_NET_PROTO_UDP ? ALTCOM_IPPROTO_UDP : ALTCOM_IPPROTO_TCP;

    if( altcom_getaddrinfo( host, port, &hints, &addr_list ) != 0 )
        return( MBEDTLS_ERR_NET_UNKNOWN_HOST );

    /* Try the sockaddrs until a connection succeeds */
    ret = MBEDTLS_ERR_NET_UNKNOWN_HOST;
    for( cur = addr_list; cur != NULL; cur = cur->ai_next )
    {
        ctx->alt_fd = (int) altcom_socket( cur->ai_family, ALTCOM_SOCK_DGRAM_DTLS,
                            cur->ai_protocol );
        if( ctx->alt_fd < 0 )
        {
            ret = MBEDTLS_ERR_NET_SOCKET_FAILED;
            continue;
        }
        mbedtls_net_alt2fd(ctx);

        memset(&sin, 0, sizeof(struct altcom_sockaddr_in));
        sin.sin_port = altcom_ntohs(source_port);
        sin.sin_family = ALTCOM_AF_INET;
        sin.sin_addr.s_addr = altcom_inet_addr("0.0.0.0");
        if( altcom_bind( ctx->alt_fd, (struct altcom_sockaddr *)&sin, sizeof(struct altcom_sockaddr_in) ) != 0 )
        {
            altcom_close( ctx->alt_fd );
            ret = MBEDTLS_ERR_NET_BIND_FAILED;
            continue;
        }

        if( altcom_connect( ctx->alt_fd, cur->ai_addr, MSVC_INT_CAST cur->ai_addrlen ) == 0 )
        {
            ret = 0;
            break;
        }

        altcom_close( ctx->alt_fd );
        ret = MBEDTLS_ERR_NET_CONNECT_FAILED;
    }

    altcom_freeaddrinfo( addr_list );
    return( ret );
}


// original
int mbedtls_net_bind( mbedtls_net_context *ctx, const char *bind_ip, const char *port, int proto )
{
    int n, ret;
    struct altcom_addrinfo hints, *addr_list, *cur;

    if( ( ret = net_prepare() ) != 0 )
        return( ret );

    /* Bind to IPv6 and/or IPv4, but only in the desired protocol */
    memset( &hints, 0, sizeof( hints ) );
    hints.ai_family = ALTCOM_AF_UNSPEC;
    hints.ai_socktype = proto == MBEDTLS_NET_PROTO_UDP ? ALTCOM_SOCK_DGRAM : ALTCOM_SOCK_STREAM;
    hints.ai_protocol = proto == MBEDTLS_NET_PROTO_UDP ? ALTCOM_IPPROTO_UDP : ALTCOM_IPPROTO_TCP;
    if( bind_ip == NULL )
        hints.ai_flags = ALTCOM_AI_PASSIVE;

    if( altcom_getaddrinfo( bind_ip, port, &hints, &addr_list ) != 0 )
        return( MBEDTLS_ERR_NET_UNKNOWN_HOST );

    /* Try the sockaddrs until a binding succeeds */
    ret = MBEDTLS_ERR_NET_UNKNOWN_HOST;
    for( cur = addr_list; cur != NULL; cur = cur->ai_next )
    {
        ctx->alt_fd = (int) altcom_socket( cur->ai_family, cur->ai_socktype,
                            cur->ai_protocol );
        if( ctx->alt_fd < 0 )
        {
            ret = MBEDTLS_ERR_NET_SOCKET_FAILED;
            continue;
        }
        mbedtls_net_alt2fd(ctx);

        n = 1;
        if( altcom_setsockopt( ctx->alt_fd, ALTCOM_SOL_SOCKET, ALTCOM_SO_REUSEADDR,
                        (const char *) &n, sizeof( n ) ) != 0 )
        {
            altcom_close( ctx->alt_fd );
            ret = MBEDTLS_ERR_NET_SOCKET_FAILED;
            continue;
        }

        if( altcom_bind( ctx->alt_fd, cur->ai_addr, MSVC_INT_CAST cur->ai_addrlen ) != 0 )
        {
            altcom_close( ctx->alt_fd );
            ret = MBEDTLS_ERR_NET_BIND_FAILED;
            continue;
        }

        /* Listen only makes sense for TCP */
        if( proto == MBEDTLS_NET_PROTO_TCP )
        {
            if( altcom_listen( ctx->alt_fd, MBEDTLS_NET_LISTEN_BACKLOG ) != 0 )
            {
                altcom_close( ctx->alt_fd );
                ret = MBEDTLS_ERR_NET_LISTEN_FAILED;
                continue;
            }
        }

        /* Bind was successful */
        ret = 0;
        break;
    }

    altcom_freeaddrinfo( addr_list );

    return( ret );

}

int mbedtls_net_accept( mbedtls_net_context *bind_ctx,
                        mbedtls_net_context *client_ctx,
                        void *client_ip, size_t buf_size, size_t *ip_len )
{
    int ret;
    int type;

    struct altcom_sockaddr_storage client_addr;

#if defined(__socklen_t_defined) || defined(_SOCKLEN_T) ||  \
    defined(_SOCKLEN_T_DECLARED) || defined(__DEFINED_socklen_t)
    socklen_t n = (socklen_t) sizeof( client_addr );
    socklen_t type_len = (socklen_t) sizeof( type );
#else
    //int n = (int) sizeof( client_addr ); // modified.
    //int type_len = (int) sizeof( type ); // modified.
	size_t n = sizeof( client_addr );
    size_t type_len = sizeof( type );

#endif

    mbedtls_net_fd2alt(bind_ctx);

    /* Is this a TCP or UDP socket? */
    if( altcom_getsockopt( bind_ctx->alt_fd, ALTCOM_SOL_SOCKET, ALTCOM_SO_TYPE,
                    (void *) &type, &type_len ) != 0 ||
        ( type != ALTCOM_SOCK_STREAM && type != ALTCOM_SOCK_DGRAM ) )
    {
        return( MBEDTLS_ERR_NET_ACCEPT_FAILED );
    }

    if( type == ALTCOM_SOCK_STREAM )
    {
        /* TCP: actual accept() */
        ret = client_ctx->alt_fd = (int) altcom_accept( bind_ctx->alt_fd,
                                             (struct altcom_sockaddr *) &client_addr, &n );
    }
    else
    {
        /* UDP: wait for a message, but keep it in the queue */
        char buf[1] = { 0 };

        ret = (int) altcom_recvfrom( bind_ctx->alt_fd, buf, sizeof( buf ), ALTCOM_MSG_PEEK,
                        (struct altcom_sockaddr *) &client_addr, &n );

#if defined(_WIN32)
        if( ret == SOCKET_ERROR &&
            WSAGetLastError() == WSAEMSGSIZE )
        {
            /* We know buf is too small, thanks, just peeking here */
            ret = 0;
        }
#endif
    }

    if( ret < 0 )
    {
        if( net_would_block( bind_ctx ) != 0 )
            return( MBEDTLS_ERR_SSL_WANT_READ );

        return( MBEDTLS_ERR_NET_ACCEPT_FAILED );
    }
    mbedtls_net_alt2fd(client_ctx);

    /* UDP: hijack the listening socket to communicate with the client,
     * then bind a new socket to accept new connections */
    if( type != ALTCOM_SOCK_STREAM )
    {
        struct altcom_sockaddr_storage local_addr;
        int one = 1;

        if( altcom_connect( bind_ctx->alt_fd, (struct altcom_sockaddr *) &client_addr, n ) != 0 )
            return( MBEDTLS_ERR_NET_ACCEPT_FAILED );

        client_ctx->fd = bind_ctx->fd;
        client_ctx->alt_fd = bind_ctx->alt_fd;
        bind_ctx->fd     = -1; /* In case we exit early */
        bind_ctx->alt_fd = -1; /* In case we exit early */

        n = sizeof( struct altcom_sockaddr_storage );
        if( altcom_getsockname( client_ctx->alt_fd,
                         (struct altcom_sockaddr *) &local_addr, &n ) != 0 ||
            ( bind_ctx->alt_fd = (int) altcom_socket( local_addr.ss_family,
                                           ALTCOM_SOCK_DGRAM, ALTCOM_IPPROTO_UDP ) ) < 0 ||
            altcom_setsockopt( bind_ctx->alt_fd, ALTCOM_SOL_SOCKET, ALTCOM_SO_REUSEADDR,
                        (const char *) &one, sizeof( one ) ) != 0 )
        {
            return( MBEDTLS_ERR_NET_SOCKET_FAILED );
        }
        mbedtls_net_alt2fd(bind_ctx);

        if( altcom_bind( bind_ctx->alt_fd, (struct altcom_sockaddr *) &local_addr, n ) != 0 )
        {
            return( MBEDTLS_ERR_NET_BIND_FAILED );
        }
    }

    if( client_ip != NULL )
    {
        if( client_addr.ss_family == ALTCOM_AF_INET )
        {
            struct altcom_sockaddr_in *addr4 = (struct altcom_sockaddr_in *) &client_addr;
            *ip_len = sizeof( addr4->sin_addr.s_addr );

            if( buf_size < *ip_len )
                return( MBEDTLS_ERR_NET_BUFFER_TOO_SMALL );

            memcpy( client_ip, &addr4->sin_addr.s_addr, *ip_len );
        }
        else
        {
            struct altcom_sockaddr_in6 *addr6 = (struct altcom_sockaddr_in6 *) &client_addr;
            *ip_len = sizeof( addr6->sin6_addr.altcom_s6_addr );

            if( buf_size < *ip_len )
                return( MBEDTLS_ERR_NET_BUFFER_TOO_SMALL );

            memcpy( client_ip, &addr6->sin6_addr.altcom_s6_addr, *ip_len);
        }
    }

    return( 0 );
}

// modified
int mbedtls_net_set_block( mbedtls_net_context *ctx )
{
    mbedtls_net_fd2alt(ctx);
#if ( defined(_WIN32) || defined(_WIN32_WCE) ) && !defined(EFIX64) && \
    !defined(EFI32)
    u_long n = 0;
    return( ioctlsocket( ctx->alt_fd, FIONBIO, &n ) );
#else
    return( FCNTL3( ctx->alt_fd, ALTCOM_SETFL, FCNTL2( ctx->alt_fd, ALTCOM_GETFL ) & ~ALTCOM_O_NONBLOCK ) );
#endif
}

// modified
int mbedtls_net_set_nonblock( mbedtls_net_context *ctx )
{
    mbedtls_net_fd2alt(ctx);
#if ( defined(_WIN32) || defined(_WIN32_WCE) ) && !defined(EFIX64) && \
    !defined(EFI32)
    u_long n = 1;
    return( ioctlsocket( ctx->alt_fd, FIONBIO, &n ) );
#else
    return( FCNTL3( ctx->alt_fd, ALTCOM_SETFL, FCNTL2( ctx->alt_fd, ALTCOM_GETFL ) | ALTCOM_O_NONBLOCK ) );
#endif
}

// modified
int mbedtls_net_recv( void *ctx, unsigned char *buf, size_t len )
{
    return MBEDTLS_ERR_NET_RECV_FAILED;
}

// modified
int mbedtls_net_recv_timeout( void *ctx, unsigned char *buf,
                              size_t len, uint32_t timeout )
{
    return MBEDTLS_ERR_NET_RECV_FAILED;
}

// modified
int mbedtls_net_send( void *ctx, const unsigned char *buf, size_t len )
{
    return MBEDTLS_ERR_NET_SEND_FAILED;
}

// original
void mbedtls_net_free( mbedtls_net_context *ctx )
{
    if( ctx->fd == -1 )
        return;

    mbedtls_net_fd2alt(ctx);
    altcom_shutdown( ctx->alt_fd, 2 );
    altcom_close( ctx->alt_fd );

    ctx->fd = -1;
    ctx->alt_fd = -1;
}

// new
void mbedtls_net_alt2fd( mbedtls_net_context *ctx )
{
#if CONFIG_NFILE_DESCRIPTORS
  if ((0 <= ctx->alt_fd) && (ctx->alt_fd < CONFIG_NFILE_DESCRIPTORS)) {
    ctx->fd = ctx->alt_fd + CONFIG_NFILE_DESCRIPTORS;
  }
  else {
    ctx->fd = ctx->alt_fd;
  }
#else
  ctx->fd = ctx->alt_fd;
#endif
}

// new
void mbedtls_net_fd2alt( mbedtls_net_context *ctx )
{
#if CONFIG_NFILE_DESCRIPTORS
  if (CONFIG_NFILE_DESCRIPTORS <= ctx->fd) {
    ctx->alt_fd = ctx->fd - CONFIG_NFILE_DESCRIPTORS;
  }
  else {
    ctx->alt_fd = ctx->fd;
  }
#else
  ctx->alt_fd = ctx->fd;
#endif
}
