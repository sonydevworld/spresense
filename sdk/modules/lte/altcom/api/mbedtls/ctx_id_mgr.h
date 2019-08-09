/****************************************************************************
 * modules/lte/altcom/api/mbedtls/ctx_id_mgr.h
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

#ifndef __MODULES_LTE_ALTCOM_API_MBEDTLS_CTX_ID_MGR_H
#define __MODULES_LTE_ALTCOM_API_MBEDTLS_CTX_ID_MGR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MBEDTLS_INVALID_CTX_ID (0xFFFFFFFF)
#define MBEDTLS_MINIMUM_CTX_ID (0x00000001)
#define MBEDTLS_MAXIMUM_CTX_ID (0x00FFFFFF)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum mbedtls_ctx_id_e
  {
    SSL_CTX = 0,
    SSL_SESSION_CTX,
    SSL_CONFIG_CTX,
    SSL_X509_CRT_CTX,
    SSL_PK_CTX,
    SSL_CTR_DRBG_CTX,
    SSL_ENTROPY_CTX,
    SSL_CIPHER_CTX,
    SSL_MPI_CTX,
    SSL_X509_CSR_CTX,
    SSL_X509WRITE_CRT_CTX,
    SSL_RSA_CTX,
    SSL_CTX_MAX,
  };

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t get_mbedtls_ctx_id(enum mbedtls_ctx_id_e id);

#endif /* __MODULES_LTE_ALTCOM_API_MBEDTLS_CTX_ID_MGR_H */
