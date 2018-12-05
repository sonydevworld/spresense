/****************************************************************************
 * examples/tlstest/ssl_sub.h
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
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef __EXAMPLES_TLSTEST_SSl_SUB_H
#define __EXAMPLES_TLSTEST_SSl_SUB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TLSTEST_SSL_BUFFER_SIZE        1024
#define TLSTEST_SSL_TCP                0
#define TLSTEST_SSL_UDP                1
#define TLSTEST_SSL_GET                0
#define TLSTEST_SSL_POST               1
#define TLSTEST_SSL_HEAD               2
#define TLSTEST_SSL_VERIFY_NONE        0
#define TLSTEST_SSL_VERIFY_REQUIRED    1
#define TLSTEST_SSL_VERIFY_CALLBACK    2


/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void ssl_init(const char** cafile, int verify);
void ssl_fin(void);
int ssl_connect(int tcp, const char *host, const char *port, int *sock);
int ssl_disconnect(int sock);

int ssl_recv(int sock, char *buf, size_t len);
int ssl_send(int sock, const char *buf, size_t len);
int ssl_sendrecv(int sock, int get, const char* host, const char* path,
                 const char* data);

#endif /* __EXAMPLES_TLSTEST_SSL_SUB_H */
