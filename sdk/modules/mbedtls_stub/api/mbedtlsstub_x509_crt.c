/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_x509_crt.c
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
#include <mbedtls/x509_crt.h>

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_x509_crt_init(mbedtls_x509_crt *crt)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&crt->id};
  FAR void *outarg[] = {&result, crt};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_x509_crt_free(mbedtls_x509_crt *crt)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {crt};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_x509_crt_parse_file(mbedtls_x509_crt *chain, const char *path)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {chain, (void *)path};
  FAR void *outarg[] = {&result};

  FAR unsigned char *parse_buf = NULL;
  size_t parse_len;

  result = mbedtls_load_local_file(path, &parse_buf, &parse_len);
  if (result == 0)
    {
      ret = mbedtls_x509_crt_parse(chain, parse_buf, parse_len);
      free(parse_buf);
    }
  else
    {
      ret = lapi_req(LTE_CMDID_TLS_X509_CRT_PARSE_FILE,
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

int mbedtls_x509_crt_parse_der(mbedtls_x509_crt *chain, const unsigned char *buf, size_t buflen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {chain, (void *)buf, &buflen};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_PARSE_DER,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_crt_parse(mbedtls_x509_crt *chain, const unsigned char *buf, size_t buflen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {chain, (void *)buf, &buflen};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_PARSE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_crt_info(char *buf, size_t size, const char *prefix, const mbedtls_x509_crt *crt)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&size, (void *)prefix, (void *)crt};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_INFO,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_crt_verify_info(char *buf, size_t size, const char *prefix, uint32_t flags)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&size, (void *)prefix, &flags};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_VERIFY_INFO,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_crt_getctx(mbedtls_x509_crt *crt, uint8_t *buff, size_t size)
{
  int ret = sizeof(mbedtls_x509_crt);

  if (crt && buff && size >= sizeof(mbedtls_x509_crt))
    {
      mbedtls_x509_crt *ctx = (mbedtls_x509_crt *)buff;
      ctx->id = crt->id;
    }
  else if (crt && buff)
    {
      ret = MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_x509_crt_setctx(mbedtls_x509_crt *crt, uint8_t *buff, size_t size)
{
  int ret = 0;

  if (crt && buff && size >= sizeof(mbedtls_x509_crt))
    {
      mbedtls_x509_crt *ctx = (mbedtls_x509_crt *)buff;
      crt->id = ctx->id;
    }
  else
    {
      ret = MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_x509_crt_getctxsize(mbedtls_x509_crt *crt)
{
  return mbedtls_x509_crt_getctx(NULL, NULL, 0);
}

