/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_bignum.c
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
#include <mbedtls/bignum.h>

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_mpi_init(mbedtls_mpi *X)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&X->id};
  FAR void *outarg[] = {&result, X};

  ret = lapi_req(LTE_CMDID_TLS_MPI_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_mpi_free(mbedtls_mpi *X)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {X};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_MPI_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_mpi_read_string(mbedtls_mpi *X, int radix, const char *s)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {X, &radix, (void *)s};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_MPI_READ_STRING,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_mpi_write_string(const mbedtls_mpi *X, int radix, char *buf, size_t buflen, size_t *olen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)X, &radix, &buflen};
  FAR void *outarg[] = {&result, buf, &buflen, olen};

  if (!X || !buf || !olen)
    {
      return MBEDTLS_ERR_MPI_BAD_INPUT_DATA;
    }

  ret = lapi_req(LTE_CMDID_TLS_MPI_WRITE_STRING,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

