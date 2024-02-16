/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_entropy.c
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
#include <mbedtls/entropy.h>

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_entropy_init(mbedtls_entropy_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_ENTROPY_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_entropy_free(mbedtls_entropy_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_ENTROPY_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_entropy_func(void *data, unsigned char *output, size_t len)
{
  return MBEDTLS_ERR_ENTROPY_SOURCE_FAILED;
}

int mbedtls_entropy_getctx(mbedtls_entropy_context *entropy, uint8_t *buff, size_t size)
{
  int ret = sizeof(mbedtls_entropy_context);

  if (entropy && buff && size >= sizeof(mbedtls_entropy_context))
    {
      mbedtls_entropy_context *ctx = (mbedtls_entropy_context *)buff;
      ctx->id = entropy->id;
    }
  else if (entropy && buff)
    {
      ret = MBEDTLS_ERR_ENTROPY_FILE_IO_ERROR;
    }

  return ret;
}

int mbedtls_entropy_setctx(mbedtls_entropy_context *entropy, uint8_t *buff, size_t size)
{
  int ret = 0;

  if (entropy && buff && size >= sizeof(mbedtls_entropy_context))
    {
      mbedtls_entropy_context *ctx = (mbedtls_entropy_context *)buff;
      entropy->id = ctx->id;
    }
  else
    {
      ret = MBEDTLS_ERR_ENTROPY_FILE_IO_ERROR;
    }

  return ret;
}

int mbedtls_entropy_getctxsize(mbedtls_entropy_context *entropy)
{
  return mbedtls_entropy_getctx(NULL, NULL, 0);
}

