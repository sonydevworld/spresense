/****************************************************************************
 * modules/mbedtls_stub/apicmd_config.h
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

#ifndef __MODULES_MBEDTLS_STUB_APICMD_CONFIG_H
#define __MODULES_MBEDTLS_STUB_APICMD_CONFIG_H

 /****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APISUBCMDID_TLS_CONFIG_INIT (0x01)
#define APISUBCMDID_TLS_CONFIG_FREE (0x02)
#define APISUBCMDID_TLS_CONFIG_DEFAULTS (0x03)
#define APISUBCMDID_TLS_CONFIG_AUTHMODE (0x04)
#define APISUBCMDID_TLS_CONFIG_RNG (0x05)
#define APISUBCMDID_TLS_CONFIG_CA_CHAIN (0x06)
#define APISUBCMDID_TLS_CONFIG_OWN_CERT (0x07)
#define APISUBCMDID_TLS_CONFIG_READ_TIMEOUT (0x08)
#define APISUBCMDID_TLS_CONFIG_VERIFY (0x09)
#define APISUBCMDID_TLS_CONFIG_VERIFY_CALLBACK (0x0A)
#define APISUBCMDID_TLS_CONFIG_ALPN_PROTOCOLS (0x0B)
#define APISUBCMDID_TLS_CONFIG_CIPHERSUITES (0x0C)

#define APICMD_TLS_CONFIG_CMD_DATA_SIZE (sizeof(struct apicmd_configcmd_s) - \
                                         sizeof(union apicmd_config_subcmd_data_s))
#define APICMD_TLS_CONFIG_CMDRES_DATA_SIZE (sizeof(struct apicmd_configcmdres_s))

#define APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN (256)
#define APICMD_CONFIG_CIPHERSUITES_COUNT (4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

/* APISUBCMDID_TLS_CONFIG_DEFAULTS */

begin_packed_struct struct apicmd_config_defaults_v4_s
{
  int32_t endpoint;
  int32_t transport;
  int32_t preset;
} end_packed_struct;

/* APISUBCMDID_TLS_CONFIG_AUTHMODE */

begin_packed_struct struct apicmd_config_authmode_v4_s
{
  int32_t authmode;
} end_packed_struct;

/* APISUBCMDID_TLS_CONFIG_RNG */

begin_packed_struct struct apicmd_config_rng_v4_s
{
  uint32_t p_rng;
} end_packed_struct;

/* APISUBCMDID_TLS_CONFIG_CA_CHAIN */

begin_packed_struct struct apicmd_config_ca_chain_v4_s
{
  uint32_t ca_chain;
  uint32_t ca_crl;
} end_packed_struct;

/* APISUBCMDID_TLS_CONFIG_OWN_CERT */

begin_packed_struct struct apicmd_config_own_cert_v4_s
{
  uint32_t own_cert;
  uint32_t pk_key;
} end_packed_struct;

/* APISUBCMDID_TLS_CONFIG_READ_TIMEOUT */

begin_packed_struct struct apicmd_config_read_timeout_v4_s
{
  uint32_t timeout;
} end_packed_struct;

/* APISUBCMDID_TLS_CONFIG_ALPN_PROTOCOLS */

begin_packed_struct struct apicmd_config_alpn_protocols_v4_s
{
  int8_t protos1[APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN];
  int8_t protos2[APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN];
  int8_t protos3[APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN];
  int8_t protos4[APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN];
} end_packed_struct;

/* APISUBCMDID_TLS_CONFIG_CIPHERSUITES */

begin_packed_struct struct apicmd_config_ciphersuites_v4_s
{
  int32_t ciphersuites[APICMD_CONFIG_CIPHERSUITES_COUNT];
} end_packed_struct;

/* config handle sub command data */

begin_packed_struct union apicmd_config_subcmd_data_s
{
  struct apicmd_config_defaults_v4_s       defaults;
  struct apicmd_config_authmode_v4_s       authmode;
  struct apicmd_config_rng_v4_s            rng;
  struct apicmd_config_ca_chain_v4_s       ca_chain;
  struct apicmd_config_own_cert_v4_s       own_cert;
  struct apicmd_config_read_timeout_v4_s   read_timeout;
  struct apicmd_config_alpn_protocols_v4_s alpn_protocols;
  struct apicmd_config_ciphersuites_v4_s   ciphersuites;
} end_packed_struct;

/* config handle command */

begin_packed_struct struct apicmd_configcmd_s
{
  uint32_t conf;
  uint32_t subcmd_id;
  union apicmd_config_subcmd_data_s u;
} end_packed_struct;

/* config handle command response */

begin_packed_struct struct apicmd_configcmdres_s
{
  int32_t ret_code;
} end_packed_struct;


#endif /* __MODULES_MBEDTLS_STUB_APICMD_CONFIG_H */
