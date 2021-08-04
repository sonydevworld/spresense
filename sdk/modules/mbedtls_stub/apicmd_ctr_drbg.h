/****************************************************************************
 * modules/mbedtls_stub/apicmd_ctr_drbg.h
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

#ifndef __MODULES_MBEDTLS_STUB_APICMD_CTR_DRBG_H
#define __MODULES_MBEDTLS_STUB_APICMD_CTR_DRBG_H

 /****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APISUBCMDID_TLS_CTR_DRBG_INIT (0x01)
#define APISUBCMDID_TLS_CTR_DRBG_FREE (0x02)
#define APISUBCMDID_TLS_CTR_DRBG_SEED (0x03)

#define APICMD_TLS_CTR_DRBG_CMD_DATA_SIZE (sizeof(struct apicmd_ctr_drbgcmd_s) - \
                                       sizeof(union apicmd_ctr_drbg_subcmd_data_s))
#define APICMD_TLS_CTR_DRBG_CMDRES_DATA_SIZE (sizeof(struct apicmd_ctr_drbgcmdres_s))

#define APICMD_CTR_DRBG_SEED_CUSTOM_LEN (256)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

/* APISUBCMDID_TLS_CTR_DRBG_SEED */

begin_packed_struct struct apicmd_ctr_drbg_seed_v4_s
{
  uint32_t p_entropy;
  uint32_t len;
  int8_t custom[APICMD_CTR_DRBG_SEED_CUSTOM_LEN];
} end_packed_struct;

/* ctr drbg handle sub command data */

begin_packed_struct union apicmd_ctr_drbg_subcmd_data_s
{
  struct apicmd_ctr_drbg_seed_v4_s seed;
} end_packed_struct;

/* ctr drbg handle command */

begin_packed_struct struct apicmd_ctr_drbgcmd_s
{
  uint32_t ctx;
  uint32_t subcmd_id;
  union apicmd_ctr_drbg_subcmd_data_s u;
} end_packed_struct;

/* ctr drbg handle command response */

begin_packed_struct struct apicmd_ctr_drbgcmdres_s
{
  int32_t ret_code;
} end_packed_struct;


#endif /* __MODULES_MBEDTLS_STUB_APICMD_CTR_DRBG_H */
