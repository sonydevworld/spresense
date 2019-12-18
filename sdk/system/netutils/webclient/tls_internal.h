/****************************************************************************
 * system/netutils/webclient/tls_internal.h
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

#ifndef __SYSTEM_NETUTILS_WEBCLIENT_TLS_INTERNAL_H
#define __SYSTEM_NETUTILS_WEBCLIENT_TLS_INTERNAL_H

#include "mbedtls/ssl.h"

/**
 * Initialize the TLS socket module
 */
void tls_socket_init(void);

/**
 * Set root CA certificate information for mbedTLS.
 * @param g_ssl_ca root CA certificate information
 */
void tls_set_crt_info(mbedtls_x509_crt *g_ssl_ca);

/**
 * Create a new TLS socket
 * Returns -1 on error, and the socket number otherwise.
 */
int tls_socket_create(int domain, int type, int protocol);

/**
 * Close a TLS socket
 */
void tls_socket_close(int s);

/**
 * Connect a TLS socket to the given address and port number.
 * Returns -1 on error, 0 otherwise.
 */
int tls_socket_connect(int s, const char *hostname, const struct sockaddr *addr);

/**
 * Read len bytes of data from the TLS socket and store in buf.
 * Returns -1 on error, otherwise the number of bytes that were read.
 */
int tls_socket_read(int s, char *buf, size_t len);

/**
 * Write len bytes of data from buf to the TLS socket.
 * Returns -1 on error, otherwise the number of bytes that were written.
 */
int tls_socket_write(int s, const char *buf, size_t len);

#endif /* __SYSTEM_NETUTILS_WEBCLIENT_TLS_INTERNAL_H */
