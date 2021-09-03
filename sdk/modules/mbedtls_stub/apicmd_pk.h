/****************************************************************************
 * modules/mbedtls_stub/apicmd_pk.h
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

#ifndef __MODULES_MBEDTLS_STUB_APICMD_PK_H
#define __MODULES_MBEDTLS_STUB_APICMD_PK_H

 /****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APISUBCMDID_TLS_PK_INIT (0x01)
#define APISUBCMDID_TLS_PK_FREE (0x02)
#define APISUBCMDID_TLS_PK_PARSE_KEYFILE (0x03)
#define APISUBCMDID_TLS_PK_PARSE_KEY (0x04)
#define APISUBCMDID_TLS_PK_CHECK_PAIR (0x05)
#define APISUBCMDID_TLS_PK_SETUP (0x06)
#define APISUBCMDID_TLS_PK_INFO_FROM_TYPE (0x07)
#define APISUBCMDID_TLS_PK_WRITE_KEY_PEM (0x08)
#define APISUBCMDID_TLS_PK_WRITE_KEY_DER (0x09)
#define APISUBCMDID_TLS_PK_RSA (0x0a)

#define APICMD_TLS_PK_CMD_DATA_SIZE (sizeof(struct apicmd_pkcmd_s) - \
                                     sizeof(union apicmd_pk_subcmd_data_s))
#define APICMD_TLS_PK_CMDRES_DATA_SIZE (sizeof(struct apicmd_pkcmdres_s) - \
                                        sizeof(union apicmd_pk_subcmdres_data_s))

#define APICMD_PK_PARSE_KEYFILE_PATH_LEN (256)
#define APICMD_PK_PARSE_KEYFILE_PWD_LEN  (64)
#define APICMD_PK_PARSE_KEY_KEY_LEN (1920)
#define APICMD_PK_PARSE_KEY_PWD_LEN (64)
#define APICMD_PK_WRITE_KEY_PEM_BUF_LEN (2000)
#define APICMD_PK_WRITE_KEY_DER_BUF_LEN (2000)


/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

/* APISUBCMDID_TLS_PK_PARSE_KEYFILE */

begin_packed_struct struct apicmd_pk_parse_keyfile_v4_s
{
  int8_t path[APICMD_PK_PARSE_KEYFILE_PATH_LEN];
  int8_t pwd[APICMD_PK_PARSE_KEYFILE_PWD_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_PK_PARSE_KEY */

begin_packed_struct struct apicmd_pk_parse_key_v4_s
{
  uint32_t keylen;
  uint32_t pwdlen;
  int8_t key[APICMD_PK_PARSE_KEY_KEY_LEN];
  int8_t pwd[APICMD_PK_PARSE_KEY_PWD_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_PK_CHECK_PAIR */

begin_packed_struct struct apicmd_pk_check_pair_v4_s
{
  uint32_t prv;
} end_packed_struct;

/* APISUBCMDID_TLS_PK_SETUP */

begin_packed_struct struct apicmd_pk_setup_v4_s
{
  uint32_t info;
} end_packed_struct;

/* APISUBCMDID_TLS_PK_INFO_FROM_TYPE */

begin_packed_struct struct apicmd_pk_info_from_type_v4_s
{
  uint32_t pk_type;
} end_packed_struct;

/* APISUBCMDID_TLS_PK_WRITE_KEY_PEM */

begin_packed_struct struct apicmd_pk_write_key_pem_v4_s
{
  uint32_t size;
} end_packed_struct;

/* APISUBCMDID_TLS_PK_WRITE_KEY_DER */

begin_packed_struct struct apicmd_pk_write_key_der_v4_s
{
  uint32_t size;
} end_packed_struct;

/* pk handle sub command data */

begin_packed_struct union apicmd_pk_subcmd_data_s
{
  struct apicmd_pk_parse_keyfile_v4_s  parse_keyfile;
  struct apicmd_pk_parse_key_v4_s      parse_key;
  struct apicmd_pk_check_pair_v4_s     check_pair;
  struct apicmd_pk_setup_v4_s          setup;
  struct apicmd_pk_info_from_type_v4_s info_from_type;
  struct apicmd_pk_write_key_pem_v4_s  write_key_pem;
  struct apicmd_pk_write_key_der_v4_s  write_key_der;
} end_packed_struct;

/* pk handle command */

begin_packed_struct struct apicmd_pkcmd_s
{
  uint32_t ctx;
  uint32_t subcmd_id;
  union apicmd_pk_subcmd_data_s u;
} end_packed_struct;

/* APISUBCMDID_TLS_PK_INFO_FROM_TYPERES */

begin_packed_struct struct apicmd_pk_info_from_typeres_v4_s
{
  uint32_t pk_info;
} end_packed_struct;

/* APISUBCMDID_TLS_PK_WRITE_KEY_PEMRES */

begin_packed_struct struct apicmd_pk_write_key_pemres_v4_s
{
  int8_t  buf[APICMD_PK_WRITE_KEY_PEM_BUF_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_PK_WRITE_KEY_DERRES */

begin_packed_struct struct apicmd_pk_write_key_derres_v4_s
{
  int8_t  buf[APICMD_PK_WRITE_KEY_DER_BUF_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_PK_RSARES */

begin_packed_struct struct apicmd_pk_rsares_v4_s
{
  uint32_t rsa;
} end_packed_struct;

/* pk handle sub command response data */

begin_packed_struct union apicmd_pk_subcmdres_data_s
{
  struct apicmd_pk_info_from_typeres_v4_s info_from_typeres;
  struct apicmd_pk_write_key_pemres_v4_s  write_key_pemres;
  struct apicmd_pk_write_key_derres_v4_s  write_key_derres;
  struct apicmd_pk_rsares_v4_s            rsares;
} end_packed_struct;

/* pk handle command response */

begin_packed_struct struct apicmd_pkcmdres_s
{
  int32_t ret_code;
  uint32_t subcmd_id;
  union apicmd_pk_subcmdres_data_s u;
} end_packed_struct;


#endif /* __MODULES_MBEDTLS_STUB_APICMD_PK_H */
