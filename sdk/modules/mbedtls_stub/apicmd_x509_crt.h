/****************************************************************************
 * modules/mbedtls_stub/apicmd_x509_crt.h
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

#ifndef __MODULES_MBEDTLS_STUB_APICMD_X509_CRT_H
#define __MODULES_MBEDTLS_STUB_APICMD_X509_CRT_H

 /****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APISUBCMDID_TLS_X509_CRT_INIT (0x01)
#define APISUBCMDID_TLS_X509_CRT_FREE (0x02)
#define APISUBCMDID_TLS_X509_CRT_PARSE_FILE (0x03)
#define APISUBCMDID_TLS_X509_CRT_PARSE_DER (0x04)
#define APISUBCMDID_TLS_X509_CRT_PARSE (0x05)
#define APISUBCMDID_TLS_X509_CRT_VERIFY_INFO (0x06)

#define APICMD_TLS_X509_CRT_CMD_DATA_SIZE (sizeof(struct apicmd_x509_crtcmd_s) - \
                                           sizeof(union apicmd_x509_crt_subcmd_data_s))
#define APICMD_TLS_X509_CRT_CMDRES_DATA_SIZE (sizeof(struct apicmd_x509_crtcmdres_s) - \
                                              sizeof(union apicmd_x509_crt_subcmdres_data_s))

#define APICMD_X509_CRT_PARSE_FILE_PATH_LEN (256)
#define APICMD_X509_CRT_VERIFY_INFO_PREFIX_LEN  (256)
#define APICMD_X509_CRT_VERIFY_INFO_RET_BUF_LEN (2000)
#define APICMD_X509_CRT_PARSE_BUF_LEN (2000)
#define APICMD_X509_CRT_PARSE_DER_LEN (2000)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

/* APISUBCMDID_TLS_X509_CRT_PARSE_FILE */

begin_packed_struct struct apicmd_x509_crt_parse_file_v4_s
{
  int8_t path[APICMD_X509_CRT_PARSE_FILE_PATH_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_X509_CRT_PARSE_DER */

begin_packed_struct struct apicmd_x509_crt_parse_der_v4_s
{
  uint32_t buflen;
  int8_t buf[APICMD_X509_CRT_PARSE_DER_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_X509_CRT_PARSE */

begin_packed_struct struct apicmd_x509_crt_parse_v4_s
{
  uint32_t buflen;
  int8_t buf[APICMD_X509_CRT_PARSE_BUF_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_X509_CRT_VERIFY_INFO */

begin_packed_struct struct apicmd_x509_crt_verify_info_v4_s
{
  uint32_t flags;
  uint32_t size;
  int8_t prefix[APICMD_X509_CRT_VERIFY_INFO_PREFIX_LEN];
} end_packed_struct;

/* x509crt handle sub command data */

begin_packed_struct union apicmd_x509_crt_subcmd_data_s
{
  struct apicmd_x509_crt_parse_file_v4_s  parse_file;
  struct apicmd_x509_crt_parse_der_v4_s   parse_der;
  struct apicmd_x509_crt_parse_v4_s       parse;
  struct apicmd_x509_crt_verify_info_v4_s verify_info;
} end_packed_struct;

/* x509crt handle comannd */

begin_packed_struct struct apicmd_x509_crtcmd_s
{
  uint32_t crt;
  uint32_t subcmd_id;
  union apicmd_x509_crt_subcmd_data_s u;
} end_packed_struct;

/* APISUBCMDID_TLS_X509_CRT_VERIFY_INFORES */

begin_packed_struct struct apicmd_x509_crt_verify_infores_v4_s
{
  int8_t buf[APICMD_X509_CRT_VERIFY_INFO_RET_BUF_LEN];
} end_packed_struct;

/* x509crt handle sub command response data */

begin_packed_struct union apicmd_x509_crt_subcmdres_data_s
{
  struct apicmd_x509_crt_verify_infores_v4_s verify_infores;
} end_packed_struct;

/* x509crt handle comannd response */

begin_packed_struct struct apicmd_x509_crtcmdres_s
{
  int32_t ret_code;
  uint32_t subcmd_id;
  union apicmd_x509_crt_subcmdres_data_s u;
} end_packed_struct;


#endif /* __MODULES_MBEDTLS_STUB_APICMD_X509_CRT_H */
