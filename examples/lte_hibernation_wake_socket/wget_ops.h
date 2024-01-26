/****************************************************************************
 * examples/lte_hibernation_wake_socket/wget_ops.h
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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

#ifndef __EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_WGET_OPS_H
#define __EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_WGET_OPS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <mbedtls/config.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/platform.h>
#include <mbedtls/ssl.h>
#include <mbedtls/pk_internal.h>
#include <mbedtls/md_internal.h>

#include "wget_utils.h"

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

struct wgetops_sock_context_s
{
  int sockfd;
};

struct wgetops_tls_context_s
{
  mbedtls_net_context server_fd;
  mbedtls_x509_crt ca;
  mbedtls_entropy_context entropy;
  mbedtls_ctr_drbg_context ctr_drbg;
  mbedtls_ssl_context ssl;
  mbedtls_ssl_config conf;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/* wget operations for normal socket */

int wgetops_sock_connect(FAR void *ctx, FAR const char *hostname,
                         FAR const char *port);
ssize_t wgetops_sock_send(FAR void *ctx, FAR const void *buf, size_t len);
ssize_t wgetops_sock_recv(FAR void *ctx, FAR void *buf, size_t len);
int wgetops_sock_close(FAR void *ctx);

/* wget operations for secure socket */

int wgetops_tls_connect(FAR void *ctx, FAR const char *hostname,
                        FAR const char *port);
ssize_t wgetops_tls_send(FAR void *ctx, FAR const void *buf, size_t len);
ssize_t wgetops_tls_recv(FAR void *ctx, FAR void *buf, size_t len);
int wgetops_tls_close(FAR void *ctx);

#endif /* __EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_WGET_OPS_H */
