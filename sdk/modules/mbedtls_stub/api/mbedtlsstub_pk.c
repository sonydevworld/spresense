/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_pk.c
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

#include <stdlib.h>
#include <nuttx/wireless/lte/lte_ioctl.h>
#include <mbedtls/pk_internal.h>

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mbedtls_pk_info_t g_pk_info = {0};
static mbedtls_rsa_context g_rsa_context  = {0};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_pk_init(mbedtls_pk_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_PK_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_pk_free(mbedtls_pk_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_PK_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_pk_parse_keyfile(mbedtls_pk_context *ctx, const char *path, const char *password)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)path, (void *)password};
  FAR void *outarg[] = {&result};

  FAR unsigned char *parse_buf = NULL;
  size_t parse_len;

  result = mbedtls_load_local_file(path, &parse_buf, &parse_len);
  if (result == 0)
    {
      if (password == NULL)
        {
          ret = mbedtls_pk_parse_key(ctx, parse_buf, parse_len, NULL, 0);
        }
      else
        {
          ret = mbedtls_pk_parse_key(ctx, parse_buf, parse_len,
                      (const unsigned char*)password, strlen(password));
        }

      free(parse_buf);
    }
  else
    {
      ret = lapi_req(LTE_CMDID_TLS_PK_PARSE_KEYFILE,
                     (FAR void *)inarg, ARRAY_SZ(inarg),
                     (FAR void *)outarg, ARRAY_SZ(outarg),
                     NULL);
      if (ret == 0)
        {
          ret = result;
        }
    }

  return ret;
}

int mbedtls_pk_parse_key(mbedtls_pk_context *ctx, const unsigned char *key, size_t keylen, const unsigned char *pwd, size_t pwdlen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)key, &keylen, (void *)pwd, &pwdlen};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_PK_PARSE_KEY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_pk_check_pair(const mbedtls_pk_context *pub, const mbedtls_pk_context *prv)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)pub, (void *)prv};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_PK_CHECK_PAIR,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_pk_setup(mbedtls_pk_context *ctx, const mbedtls_pk_info_t *info)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)info};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_PK_SETUP,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

const mbedtls_pk_info_t *mbedtls_pk_info_from_type(mbedtls_pk_type_t pk_type)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&pk_type};
  FAR void *outarg[] = {&result, &g_pk_info};

  ret = lapi_req(LTE_CMDID_TLS_PK_INFO_FROM_TYPE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return &g_pk_info;
    }
  else
    {
      return NULL;
    }
}

int mbedtls_pk_write_key_pem(mbedtls_pk_context *ctx, unsigned char *buf, size_t size)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &size};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_PK_WRITE_KEY_PEM,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_pk_write_key_der(mbedtls_pk_context *ctx, unsigned char *buf, size_t size)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &size};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_PK_WRITE_KEY_DER,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

mbedtls_rsa_context *mbedtls_pk_rsa(const mbedtls_pk_context pk)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)&pk};
  FAR void *outarg[] = {&result, &g_rsa_context};

  ret = lapi_req(LTE_CMDID_TLS_PK_RSA,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return &g_rsa_context;
    }
  else
    {
      return NULL;
    }
}

int mbedtls_pk_getctx(mbedtls_pk_context *pk, uint8_t *buff, size_t size)
{
  int ret = sizeof(mbedtls_pk_context);

  if (pk && buff && size >= sizeof(mbedtls_pk_context))
    {
      mbedtls_pk_context *ctx = (mbedtls_pk_context *)buff;
      ctx->id = pk->id;
    }
  else if (pk && buff)
    {
      ret = MBEDTLS_ERR_PK_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_pk_setctx(mbedtls_pk_context *pk, uint8_t *buff, size_t size)
{
  int ret = 0;

  if (pk && buff && size >= sizeof(mbedtls_pk_context))
    {
      mbedtls_pk_context *ctx = (mbedtls_pk_context *)buff;
      pk->id = ctx->id;
    }
  else
    {
      ret = MBEDTLS_ERR_PK_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_pk_getctxsize(mbedtls_pk_context *pk)
{
  return mbedtls_pk_getctx(NULL, NULL, 0);
}

int mbedtls_pk_info_getctx(mbedtls_pk_info_t *info, uint8_t *buff, size_t size)
{
  int ret = sizeof(mbedtls_pk_info_t);

  if (info && buff && size >= sizeof(mbedtls_pk_info_t))
    {
      mbedtls_pk_info_t *ctx = (mbedtls_pk_info_t *)buff;
      ctx->id = info->id;
    }
  else if (info && buff)
    {
      ret = MBEDTLS_ERR_PK_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_pk_info_setctx(mbedtls_pk_info_t *info, uint8_t *buff, size_t size)
{
  int ret = 0;

  if (info && buff && size >= sizeof(mbedtls_pk_info_t))
    {
      mbedtls_pk_info_t *ctx = (mbedtls_pk_info_t *)buff;
      info->id = ctx->id;
    }
  else
    {
      ret = MBEDTLS_ERR_PK_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_pk_info_getctxsize(mbedtls_pk_info_t *info)
{
  return mbedtls_pk_info_getctx(NULL, NULL, 0);
}
