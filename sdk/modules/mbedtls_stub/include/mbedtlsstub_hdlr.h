/****************************************************************************
 * modules/mbedtls_stub/include/mbedtlsstub_hdlr.h
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

#ifndef __MODULES_MBEDTLS_STUB_INCLUDE_MBEDTLSSTUB_HDLR_H
#define __MODULES_MBEDTLS_STUB_INCLUDE_MBEDTLSSTUB_HDLR_H

 /****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include "mbedtlsstub_hdlr_ssl.h"
#include "mbedtlsstub_hdlr_ssl_conf.h"
#include "mbedtlsstub_hdlr_ssl_session.h"
#include "mbedtlsstub_hdlr_x509_crt.h"
#include "mbedtlsstub_hdlr_pk.h"
#include "mbedtlsstub_hdlr_ctr_drbg.h"
#include "mbedtlsstub_hdlr_entropy.h"
#include "mbedtlsstub_hdlr_cipher.h"
#include "mbedtlsstub_hdlr_md.h"
#include "mbedtlsstub_hdlr_base64.h"
#include "mbedtlsstub_hdlr_sha1.h"
#include "mbedtlsstub_hdlr_bignum.h"
#include "mbedtlsstub_hdlr_x509_csr.h"
#include "mbedtlsstub_hdlr_x509write_crt.h"
#include "mbedtlsstub_hdlr_rsa.h"
#include "mbedtlsstub_hdlr_ssl_srtp.h"
#include "mbedtlsstub_hdlr_x509_dn.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum mbedtlsstub_ctx_id_e
{
  MBEDTLSSTUB_SSL_CTX = 0,
  MBEDTLSSTUB_SSL_SESSION_CTX,
  MBEDTLSSTUB_SSL_CONFIG_CTX,
  MBEDTLSSTUB_SSL_X509_CRT_CTX,
  MBEDTLSSTUB_SSL_PK_CTX,
  MBEDTLSSTUB_SSL_CTR_DRBG_CTX,
  MBEDTLSSTUB_SSL_ENTROPY_CTX,
  MBEDTLSSTUB_SSL_CIPHER_CTX,
  MBEDTLSSTUB_SSL_MPI_CTX,
  MBEDTLSSTUB_SSL_X509_CSR_CTX,
  MBEDTLSSTUB_SSL_X509WRITE_CRT_CTX,
  MBEDTLSSTUB_SSL_RSA_CTX,
  MBEDTLSSTUB_SSL_CTX_MAX,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t mbedtlsstub_get_mbedtls_ctx_id(enum mbedtlsstub_ctx_id_e id);

#endif /* __MODULES_MBEDTLS_STUB_INCLUDE_MBEDTLSSTUB_HDLR_H */
