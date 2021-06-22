/****************************************************************************
 * externals/sslutils/include/sslutils/sslutil.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#ifndef __EXTERNALS_SSLUTILS_SSLUTIL_H__
#define __EXTERNALS_SSLUTILS_SSLUTIL_H__

#include "netutils/webclient.h"

struct sslutil_tls_context
{
  FAR const char *ca_dir;
  FAR const char *ca_file;
  FAR const char *cli_file;
  FAR const char *privkey;
  FAR const char *custom_id;
};

#define SSLUTIL_CTX_INIT(p_ctx) \
  do { \
    (p_ctx)->ca_dir = NULL; \
    (p_ctx)->ca_file = NULL;  \
    (p_ctx)->cli_file = NULL; \
    (p_ctx)->privkey = NULL;  \
    (p_ctx)->custom_id = NULL;  \
  } while(0)

#define SSLUTIL_CTX_SET_CADIR(p_ctx, path) \
  do { (p_ctx)->ca_dir = path; }while(0)

#define SSLUTIL_CTX_SET_CAFILE(p_ctx, path) \
  do { (p_ctx)->ca_file = path; }while(0)

#define SSLUTIL_CTX_SET_CLIENTCAFILE(p_ctx, path) \
  do { (p_ctx)->cli_file = path; }while(0)

#define SSLUTIL_CTX_SET_PRIVKEY(p_ctx, path) \
  do { (p_ctx)->privkey = path; }while(0)

#define SSLUTIL_CTX_SET_CUSTOMID(p_ctx, id) \
  do { (p_ctx)->custom_id = id; }while(0)

struct webclient_tls_ops *sslutil_webclient_tlsops(void);

#endif  /* __EXTERNALS_SSLUTILS_SSLUTIL_H__ */
