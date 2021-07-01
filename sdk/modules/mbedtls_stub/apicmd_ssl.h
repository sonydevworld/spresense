/****************************************************************************
 * modules/mbedtls_stub/apicmd_ssl.h
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_MBEDTLS_STUB_APICMD_SSL_H
#define __MODULES_MBEDTLS_STUB_APICMD_SSL_H

 /****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* API sub command id */
#define APISUBCMDID_TLS_SSL_INIT  (0x01)
#define APISUBCMDID_TLS_SSL_FREE   (0x02)
#define APISUBCMDID_TLS_SSL_SETUP (0x03)
#define APISUBCMDID_TLS_SSL_HOSTNAME (0x04)
#define APISUBCMDID_TLS_SSL_BIO      (0x05)
#define APISUBCMDID_TLS_SSL_HANDSHAKE (0x06)
#define APISUBCMDID_TLS_SSL_WRITE (0x07)
#define APISUBCMDID_TLS_SSL_READ (0x08)
#define APISUBCMDID_TLS_SSL_CLOSE_NOTIFY (0x09)
#define APISUBCMDID_TLS_SSL_VERSION (0x0A)
#define APISUBCMDID_TLS_SSL_CIPHERSUITE (0x0B)
#define APISUBCMDID_TLS_SSL_CIPHERSUITE_ID (0x0C)
#define APISUBCMDID_TLS_SSL_RECORD_EXPANSION (0x0D)
#define APISUBCMDID_TLS_SSL_VERIFY_RESULT (0x0E)
#define APISUBCMDID_TLS_SSL_TIMER_CB (0x0F)
#define APISUBCMDID_TLS_SSL_PEER_CERT (0x10)
#define APISUBCMDID_TLS_SSL_BYTES_AVAIL (0x11)

#define APICMD_TLS_SSL_CMD_DATA_SIZE (sizeof(struct apicmd_sslcmd_s) - \
                                      sizeof(union apicmd_ssl_subcmd_data_s))
#define APICMD_TLS_SSL_CMDRES_DATA_SIZE (sizeof(struct apicmd_sslcmdres_s) - \
                                         sizeof(union apicmd_ssl_subcmdres_data_s))

#define APICMD_SSL_HOSTNAME_LEN (256)
#define APICMD_SSL_WRITE_BUF_LEN (2000)
#define APICMD_SSL_READ_BUF_LEN (2000)
#define APICMD_SSL_VERSION_LEN (16)
#define APICMD_SSL_CIPHERSUITE_REQLEN (64)
#define APICMD_SSL_CIPHERSUITE_RESLEN (64)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

/* APISUBCMDID_TLS_SSL_SETUP */

begin_packed_struct struct apicmd_ssl_setup_v4_s
{
  uint32_t conf;  
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_HOSTNAME */

begin_packed_struct struct apicmd_ssl_hostname_v4_s
{
  int8_t hostname[APICMD_SSL_HOSTNAME_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_BIO */

begin_packed_struct struct apicmd_ssl_bio_v4_s
{
  uint32_t p_bio;
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_WRITE */

begin_packed_struct struct apicmd_ssl_write_v4_s
{
  uint32_t len;
  int8_t buf[APICMD_SSL_WRITE_BUF_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_READ */

begin_packed_struct struct apicmd_ssl_read_v4_s
{
  uint32_t len;
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_CIPHERSUITE_ID */

begin_packed_struct struct apicmd_ssl_ciphersuite_id_v4_s
{
  int8_t ciphersuite[APICMD_SSL_CIPHERSUITE_REQLEN];
} end_packed_struct;

/* SSL handle sub command data */

begin_packed_struct union apicmd_ssl_subcmd_data_s
{
  struct apicmd_ssl_setup_v4_s          setup;
  struct apicmd_ssl_hostname_v4_s       hostname;
  struct apicmd_ssl_bio_v4_s            bio;
  struct apicmd_ssl_write_v4_s          write;
  struct apicmd_ssl_read_v4_s           read;
  struct apicmd_ssl_ciphersuite_id_v4_s ciphersuite_id;
} end_packed_struct;

/* SSL handle command */

begin_packed_struct struct apicmd_sslcmd_s
{
  uint32_t ssl;
  uint32_t subcmd_id;
  union apicmd_ssl_subcmd_data_s u;
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_READRES */

begin_packed_struct struct apicmd_ssl_readres_v4_s
{
  int8_t buf[APICMD_SSL_READ_BUF_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_VERSIONRES */

begin_packed_struct struct apicmd_ssl_versionres_v4_s
{
  int8_t version[APICMD_SSL_VERSION_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_CIPHERSUITERES */

begin_packed_struct struct apicmd_ssl_ciphersuiteres_v4_s
{
  int8_t ciphersuite[APICMD_SSL_CIPHERSUITE_RESLEN];
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_CIPHERSUITE_IDRES */

begin_packed_struct struct apicmd_ssl_ciphersuite_idres_v4_s
{
  int32_t id;
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_PEER_CERTRES */

begin_packed_struct struct apicmd_ssl_peer_certres_v4_s
{
  uint32_t crt;
} end_packed_struct;

/* APISUBCMDID_TLS_SSL_BYTES_AVAILRES */

begin_packed_struct struct apicmd_ssl_bytes_availres_v4_s
{
  uint32_t avail_bytes;
} end_packed_struct;

/* SSL handle sub command response data */

begin_packed_struct union apicmd_ssl_subcmdres_data_s
{
  struct apicmd_ssl_readres_v4_s           readres;
  struct apicmd_ssl_versionres_v4_s        versionres;
  struct apicmd_ssl_ciphersuiteres_v4_s    ciphersuiteres;
  struct apicmd_ssl_ciphersuite_idres_v4_s ciphersuite_idres;
  struct apicmd_ssl_peer_certres_v4_s      peer_certres;
  struct apicmd_ssl_bytes_availres_v4_s    bytes_availres;
} end_packed_struct;

/* SSL handle command response */

begin_packed_struct struct apicmd_sslcmdres_s
{
  int32_t ret_code;
  uint32_t subcmd_id;
  union apicmd_ssl_subcmdres_data_s u;
} end_packed_struct;

#endif /* __MODULES_MBEDTLS_STUB_APICMD_SSL_H */
