/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_ctr_drbg.c
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
#include <mbedtls/ctr_drbg.h>

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_ctr_drbg_init(mbedtls_ctr_drbg_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_CTR_DRBG_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_ctr_drbg_free(mbedtls_ctr_drbg_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CTR_DRBG_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ctr_drbg_seed(mbedtls_ctr_drbg_context *ctx, int (*f_entropy)(void *, unsigned char *, size_t), void *p_entropy, const unsigned char *custom, size_t len)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, f_entropy, p_entropy, (void *)custom, &len};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CTR_DRBG_SEED,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ctr_drbg_random(void *p_rng, unsigned char *output, size_t output_len)
{
    return MBEDTLS_ERR_CTR_DRBG_ENTROPY_SOURCE_FAILED;
}

int mbedtls_ctr_drbg_getctx(mbedtls_ctr_drbg_context *ctr_drbg, uint8_t *buff, size_t size)
{
  int ret = sizeof(mbedtls_ctr_drbg_context);

  if (ctr_drbg && buff && size >= sizeof(mbedtls_ctr_drbg_context))
    {
      mbedtls_ctr_drbg_context *ctx = (mbedtls_ctr_drbg_context *)buff;
      ctx->id = ctr_drbg->id;
    }
  else if (ctr_drbg && buff)
    {
      ret = MBEDTLS_ERR_CTR_DRBG_FILE_IO_ERROR;
    }

  return ret;
}

int mbedtls_ctr_drbg_setctx(mbedtls_ctr_drbg_context *ctr_drbg, uint8_t *buff, size_t size)
{
  int ret = 0;

  if (ctr_drbg && buff && size >= sizeof(mbedtls_ctr_drbg_context))
    {
      mbedtls_ctr_drbg_context *ctx = (mbedtls_ctr_drbg_context *)buff;
      ctr_drbg->id = ctx->id;
    }
  else
    {
      ret = MBEDTLS_ERR_CTR_DRBG_FILE_IO_ERROR;
    }

  return ret;
}

int mbedtls_ctr_drbg_getctxsize(mbedtls_ctr_drbg_context *ctr_drbg)
{
  return mbedtls_ctr_drbg_getctx(NULL, NULL, 0);
}
