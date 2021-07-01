/****************************************************************************
 * modules/mbedtls_stub/apicmd_cipher.h
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

#ifndef __MODULES_MBEDTLS_STUB_APICMD_CIPHER_H
#define __MODULES_MBEDTLS_STUB_APICMD_CIPHER_H

 /****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APISUBCMDID_TLS_MD_INFO_FROM_TYPE (0x01)
#define APISUBCMDID_TLS_MD_GET_SIZE (0x02)
#define APISUBCMDID_TLS_MD (0x03)
#define APISUBCMDID_TLS_MD_DIGEST (0x04)
#define APISUBCMDID_TLS_BASE64_ENCODE (0x05)
#define APISUBCMDID_TLS_SHA1 (0x06)

#define APICMD_TLS_CIPHER_CMD_DATA_SIZE (sizeof(struct apicmd_ciphercmd_s) - \
                                         sizeof(union apicmd_cipher_subcmd_data_s))
#define APICMD_TLS_CIPHER_CMDRES_DATA_SIZE (sizeof(struct apicmd_ciphercmdres_s) - \
                                            sizeof(union apicmd_cipher_subcmdres_data_s))

#define APICMD_MD_INPUT_LEN  (2000)
#define APICMD_MD_OUTPUT_LEN_V4 (64)
#define APICMD_MD_DIGEST_OUTPUT_LEN_V4 (64)
#define APICMD_BASE64_ENCODE_SRC_LEN (2000)
#define APICMD_BASE64_ENCODE_DST_LEN (2000)
#define APICMD_SHA1_INPUT_LEN  (2000)
#define APICMD_SHA1_OUTPUT_LEN (20)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

/* APISUBCMDID_TLS_MD_INFO_FROM_TYPE */

begin_packed_struct struct apicmd_md_info_from_type_v4_s
{
  int32_t md_type;
} end_packed_struct;

/* APISUBCMDID_TLS_MD */

begin_packed_struct struct apicmd_md_v4_s
{
  uint32_t ilen;
  int8_t input[APICMD_MD_INPUT_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_MD_DIGEST */

begin_packed_struct struct apicmd_md_digest_v4_s
{
  uint32_t chain;
} end_packed_struct;

/* APISUBCMDID_TLS_BASE64_ENCODE */

begin_packed_struct struct apicmd_base64_encode_v4_s
{
  int32_t dlen;
  int32_t slen;
  int8_t src[APICMD_BASE64_ENCODE_SRC_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_SHA1 */

begin_packed_struct struct apicmd_sha1_v4_s
{
  int32_t ilen;
  int8_t input[APICMD_SHA1_INPUT_LEN];
} end_packed_struct;

/* cipher handle sub command data */

begin_packed_struct union apicmd_cipher_subcmd_data_s
{
  struct apicmd_md_info_from_type_v4_s md_info_from_type;
  struct apicmd_md_v4_s                md;
  struct apicmd_md_digest_v4_s         md_digest;
  struct apicmd_base64_encode_v4_s     base64_encode;
  struct apicmd_sha1_v4_s              sha1;
} end_packed_struct;

/* cipher handle command */

begin_packed_struct struct apicmd_ciphercmd_s
{
  uint32_t md_info;
  uint32_t subcmd_id;
  union apicmd_cipher_subcmd_data_s u;
} end_packed_struct;

/* APISUBCMDID_TLS_MD_INFO_FROM_TYPERES */

begin_packed_struct struct apicmd_md_info_from_typeres_v4_s
{
  uint32_t md_info;
} end_packed_struct;

/* APISUBCMDID_TLS_MD_GET_SIZERES */

begin_packed_struct struct apicmd_md_get_sizeres_v4_s
{
  uint8_t md_size;
} end_packed_struct;

/* APISUBCMDID_TLS_MDRES */

begin_packed_struct struct apicmd_mdres_v4_s
{
  int8_t output[APICMD_MD_OUTPUT_LEN_V4];
} end_packed_struct;

/* APISUBCMDID_TLS_MD_DIGESTRES */

begin_packed_struct struct apicmd_md_digestres_v4_s
{
  int8_t output[APICMD_MD_DIGEST_OUTPUT_LEN_V4];
} end_packed_struct;

/* APISUBCMDID_TLS_BASE64_ENCODERES */

begin_packed_struct struct apicmd_base64_encoderes_v4_s
{
  int32_t olen;
  int8_t dst[APICMD_BASE64_ENCODE_DST_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_SHA1RES */

begin_packed_struct struct apicmd_sha1res_v4_s
{
  int8_t output[APICMD_SHA1_OUTPUT_LEN];
} end_packed_struct;

/* cipher handle sub command response data */

begin_packed_struct union apicmd_cipher_subcmdres_data_s
{
  struct apicmd_md_info_from_typeres_v4_s md_info_from_typeres;
  struct apicmd_md_get_sizeres_v4_s       md_get_sizeres;
  struct apicmd_mdres_v4_s                mdres;
  struct apicmd_md_digestres_v4_s         md_digestres;
  struct apicmd_base64_encoderes_v4_s     base64_encoderes;
  struct apicmd_sha1res_v4_s              sha1res;
} end_packed_struct;

/* cipher handle command response */

begin_packed_struct struct apicmd_ciphercmdres_s
{
  int32_t ret_code;
  uint32_t subcmd_id;
  union apicmd_cipher_subcmdres_data_s u;
} end_packed_struct;


#endif /* __MODULES_MBEDTLS_STUB_APICMD_CIPHER_H */
