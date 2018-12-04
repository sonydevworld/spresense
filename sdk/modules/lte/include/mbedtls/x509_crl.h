/**
 * \file x509_crl.h
 *
 * \brief X.509 certificate revocation list parsing
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */
#ifndef MBEDTLS_X509_CRL_H
#define MBEDTLS_X509_CRL_H

#if !defined(MBEDTLS_CONFIG_FILE)
#include "config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include "x509.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup x509_module
 * \{ */

/**
 * \name Structures and functions for parsing CRLs
 * \{
 */

/**
 * Certificate revocation list entry.
 * Contains the CA-specific serial numbers and revocation dates.
 */
typedef struct mbedtls_x509_crl_entry
{
    mbedtls_x509_buf raw;

    mbedtls_x509_buf serial;

    mbedtls_x509_time revocation_date;

    mbedtls_x509_buf entry_ext;

    struct mbedtls_x509_crl_entry *next;
}
mbedtls_x509_crl_entry;

/**
 * Certificate revocation list structure.
 * Every CRL may have multiple entries.
 */
typedef struct mbedtls_x509_crl
{
  uint32_t id;
}
mbedtls_x509_crl;

/**
 * \brief          Parse a DER-encoded CRL and append it to the chained list
 *
 * \param chain    points to the start of the chain
 * \param buf      buffer holding the CRL data in DER format
 * \param buflen   size of the buffer
 *                 (including the terminating null byte for PEM data)
 *
 * \return         0 if successful, or a specific X509 or PEM error code
 */
int mbedtls_x509_crl_parse_der( mbedtls_x509_crl *chain,
                        const unsigned char *buf, size_t buflen );
/**
 * \brief          Parse one or more CRLs and append them to the chained list
 *
 * \note           Mutliple CRLs are accepted only if using PEM format
 *
 * \param chain    points to the start of the chain
 * \param buf      buffer holding the CRL data in PEM or DER format
 * \param buflen   size of the buffer
 *                 (including the terminating null byte for PEM data)
 *
 * \return         0 if successful, or a specific X509 or PEM error code
 */
int mbedtls_x509_crl_parse( mbedtls_x509_crl *chain, const unsigned char *buf, size_t buflen );

#if defined(MBEDTLS_FS_IO)
/**
 * \brief          Load one or more CRLs and append them to the chained list
 *
 * \note           Mutliple CRLs are accepted only if using PEM format
 *
 * \param chain    points to the start of the chain
 * \param path     filename to read the CRLs from (in PEM or DER encoding)
 *
 * \return         0 if successful, or a specific X509 or PEM error code
 */
int mbedtls_x509_crl_parse_file( mbedtls_x509_crl *chain, const char *path );
#endif /* MBEDTLS_FS_IO */

/**
 * \brief          Returns an informational string about the CRL.
 *
 * \param buf      Buffer to write to
 * \param size     Maximum size of buffer
 * \param prefix   A line prefix
 * \param crl      The X509 CRL to represent
 *
 * \return         The length of the string written (not including the
 *                 terminated nul byte), or a negative error code.
 */
int mbedtls_x509_crl_info( char *buf, size_t size, const char *prefix,
                   const mbedtls_x509_crl *crl );

/**
 * \brief          Initialize a CRL (chain)
 *
 * \param crl      CRL chain to initialize
 */
void mbedtls_x509_crl_init( mbedtls_x509_crl *crl );

/**
 * \brief          Unallocate all CRL data
 *
 * \param crl      CRL chain to free
 */
void mbedtls_x509_crl_free( mbedtls_x509_crl *crl );

/* \} name */
/* \} addtogroup x509_module */

#ifdef __cplusplus
}
#endif

#endif /* mbedtls_x509_crl.h */
