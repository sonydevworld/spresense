/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_md.c
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
#include <mbedtls/x509_crt.h>
#include <mbedtls/md_internal.h>

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mbedtls_md_info_t g_md_info = {0};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

const mbedtls_md_info_t *mbedtls_md_info_from_type(mbedtls_md_type_t md_type)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&md_type};
  FAR void *outarg[] = {&result, &g_md_info};

  ret = lapi_req(LTE_CMDID_TLS_MD_INFO_FROM_TYPE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return &g_md_info;
    }
  else
    {
      return NULL;
    }
}

unsigned char mbedtls_md_get_size(const mbedtls_md_info_t *md_info)
{
  int ret;
  int32_t result;
  uint8_t md_size;
  FAR void *inarg[] = {(void *)md_info};
  FAR void *outarg[] = {&result, &md_size};

  ret = lapi_req(LTE_CMDID_TLS_MD_GET_SIZE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return md_size;
    }
  else
    {
      return 0;
    }
}

int mbedtls_md(const mbedtls_md_info_t *md_info, const unsigned char *input, size_t ilen, unsigned char *output)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)md_info, (void *)input, &ilen};
  FAR void *outarg[] = {&result, output};

  ret = lapi_req(LTE_CMDID_TLS_MD,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_md_digest(const mbedtls_md_info_t *md_info, mbedtls_x509_crt *chain, unsigned char *output)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)md_info, chain};
  FAR void *outarg[] = {&result, output};

  ret = lapi_req(LTE_CMDID_TLS_MD_DIGEST,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_md_info_getctx(mbedtls_md_info_t *md_info, uint8_t *buff, size_t size)
{
  int ret = sizeof(mbedtls_md_info_t);

  if (md_info && buff && size >= sizeof(mbedtls_md_info_t))
    {
      mbedtls_md_info_t *ctx = (mbedtls_md_info_t *)buff;
      ctx->id = md_info->id;
    }
  else if (md_info && buff)
    {
      ret = MBEDTLS_ERR_MD_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_md_info_setctx(mbedtls_md_info_t *md_info, uint8_t *buff, size_t size)
{
  int ret = 0;

  if (md_info && buff && size >= sizeof(mbedtls_md_info_t))
    {
      mbedtls_md_info_t *ctx = (mbedtls_md_info_t *)buff;
      md_info->id = ctx->id;
    }
  else
    {
      ret = MBEDTLS_ERR_MD_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_md_info_getctxsize(mbedtls_md_info_t *md_info)
{
  return mbedtls_md_info_getctx(NULL, NULL, 0);
}

