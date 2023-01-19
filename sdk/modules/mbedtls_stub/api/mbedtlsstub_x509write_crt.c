/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_x509write_crt.c
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

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_x509write_crt_init(mbedtls_x509write_cert *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_x509write_crt_free(mbedtls_x509write_cert *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_x509write_crt_der(mbedtls_x509write_cert *ctx, unsigned char *buf, size_t size, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &size, f_rng, p_rng};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_DER,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_pem(mbedtls_x509write_cert *ctx, unsigned char *buf, size_t size, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &size, f_rng, p_rng};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_PEM,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_x509write_crt_set_subject_key(mbedtls_x509write_cert *ctx, mbedtls_pk_context *key)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, key};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_KEY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_x509write_crt_set_issuer_key(mbedtls_x509write_cert *ctx, mbedtls_pk_context *key)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, key};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_ISSUER_KEY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_x509write_crt_set_subject_name(mbedtls_x509write_cert *ctx, const char *subject_name)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)subject_name};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_NAME,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_issuer_name(mbedtls_x509write_cert *ctx, const char *issuer_name)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)issuer_name};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_ISSUER_NAME,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_x509write_crt_set_version(mbedtls_x509write_cert *ctx, int version)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &version};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_VERSION,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_x509write_crt_set_md_alg(mbedtls_x509write_cert *ctx, mbedtls_md_type_t md_alg)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &md_alg};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_MD_ALG,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_x509write_crt_set_serial(mbedtls_x509write_cert *ctx, const mbedtls_mpi *serial)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)serial};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_SERIAL,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_validity(mbedtls_x509write_cert *ctx, const char *not_before, const char *not_after)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)not_before, (void *)not_after};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_VALIDITY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_basic_constraints(mbedtls_x509write_cert *ctx, int is_ca, int max_pathlen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &is_ca, &max_pathlen};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_CONSTRAINTS,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_subject_key_identifier(mbedtls_x509write_cert *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_ID,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_authority_key_identifier(mbedtls_x509write_cert *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_AUTHORITY_ID,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_key_usage(mbedtls_x509write_cert *ctx, unsigned int key_usage)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &key_usage};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_KEY_USAGE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_ns_cert_type(mbedtls_x509write_cert *ctx, unsigned char ns_cert_type)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &ns_cert_type};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

