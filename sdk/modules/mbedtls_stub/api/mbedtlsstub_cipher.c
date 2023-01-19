/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_cipher.c
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wireless/lte/lte_ioctl.h>
#include <mbedtls/cipher.h>

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mbedtls_cipher_info_t g_cipher_info = {0};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_cipher_init(mbedtls_cipher_context_t *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_cipher_free(mbedtls_cipher_context_t *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

const mbedtls_cipher_info_t *mbedtls_cipher_info_from_string(const char *cipher_name)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)cipher_name};
  FAR void *outarg[] = {&result, &g_cipher_info};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_INFO_FROM_STRING,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return &g_cipher_info;
    }
  else
    {
      return NULL;
    }
}

int mbedtls_cipher_setup(mbedtls_cipher_context_t *ctx, const mbedtls_cipher_info_t *cipher_info)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)cipher_info};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_SETUP,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_cipher_setkey(mbedtls_cipher_context_t *ctx, const unsigned char *key, int key_bitlen, const mbedtls_operation_t operation)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)key, &key_bitlen, (void *)&operation};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_SETKEY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_cipher_set_iv(mbedtls_cipher_context_t *ctx, const unsigned char *iv, size_t iv_len)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)iv, &iv_len};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_SET_IV,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_cipher_update(mbedtls_cipher_context_t *ctx, const unsigned char *input, size_t ilen, unsigned char *output, size_t *olen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)input, &ilen};
  FAR void *outarg[] = {&result, output, olen};

  if (!ctx || !input || !output || !olen)
    {
      return MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
    }

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_UPDATE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

