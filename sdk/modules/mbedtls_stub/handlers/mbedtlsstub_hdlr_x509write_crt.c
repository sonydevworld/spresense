/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_x509write_crt.c
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

#include <stdint.h>
#include <stddef.h>
#include <arpa/inet.h>
#include <nuttx/modem/alt1250.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/ctr_drbg.h>

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_x509write_crt_init.h"
#include "include/apicmd_x509write_crt_free.h"
#include "include/apicmd_x509write_crt_der.h"
#include "include/apicmd_x509write_crt_pem.h"
#include "include/apicmd_x509write_crt_subject_key.h"
#include "include/apicmd_x509write_crt_issuer_key.h"
#include "include/apicmd_x509write_crt_subject_name.h"
#include "include/apicmd_x509write_crt_issuer_name.h"
#include "include/apicmd_x509write_crt_version.h"
#include "include/apicmd_x509write_crt_md_alg.h"
#include "include/apicmd_x509write_crt_serial.h"
#include "include/apicmd_x509write_crt_validity.h"
#include "include/apicmd_x509write_crt_basic_constraints.h"
#include "include/apicmd_x509write_crt_subject_key_identifier.h"
#include "include/apicmd_x509write_crt_authority_key_identifier.h"
#include "include/apicmd_x509write_crt_key_usage.h"
#include "include/apicmd_x509write_crt_ns_cert_type.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_x509writecrtinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_X509WRITE_CRT_CTX);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_init_s *out =
        (FAR struct apicmd_x509write_crt_init_s *)pktbuf;

      out->ctx = htonl(*id);

      TLS_DEBUG("[x509write_crt_init]ctx id: %ld\n", *id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_INIT;
      size = sizeof(struct apicmd_x509write_crt_init_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_free_s *out =
        (FAR struct apicmd_x509write_crt_free_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      TLS_DEBUG("[x509write_crt_free]ctx id: %lu\n", ctx->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_FREE;
      size = sizeof(struct apicmd_x509write_crt_free_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  size_t req_buf_len = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];
  FAR void *p_rng = (FAR void *)arg[3];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_der_s *out =
        (FAR struct apicmd_x509write_crt_der_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      req_buf_len = (*size <= APICMD_X509WRITE_CRT_DER_BUF_LEN)
        ? *size : APICMD_X509WRITE_CRT_DER_BUF_LEN;
      out->size = htonl(req_buf_len);

      TLS_DEBUG("[x509write_crt_der]config id: %lu\n", ctx->id);

      if (p_rng != NULL)
        {
          mbedtls_ctr_drbg_context *lctx = (mbedtls_ctr_drbg_context*)p_rng;
          uint32_t id = lctx->id;
          out->p_rng = htonl(id);
          TLS_DEBUG("[x509write_crt_der]p_rng(id): %lu\n", id);
        }

      *altcid = APICMDID_TLS_X509WRITE_CRT_DER;
      ret_size = sizeof(struct apicmd_x509write_crt_der_s);
    }
  else
#endif
    {
      ret_size = -ENOTSUP;
    }

  return ret_size;
}

int32_t mbedtlsstub_x509writecrtpem_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  size_t req_buf_len = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];
  FAR void *p_rng = (FAR void *)arg[3];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_pem_s *out =
        (FAR struct apicmd_x509write_crt_pem_s *)pktbuf;

      out->crt = htonl(ctx->id);

      req_buf_len = (*size <= APICMD_X509WRITE_CRT_PEM_BUF_LEN)
        ? *size : APICMD_X509WRITE_CRT_PEM_BUF_LEN;
      out->size = htonl(req_buf_len);

      TLS_DEBUG("[x509write_crt_pem]config id: %lu\n", ctx->id);

      if (p_rng != NULL)
        {
          mbedtls_ctr_drbg_context *lctx = (mbedtls_ctr_drbg_context*)p_rng;
          uint32_t id = lctx->id;
          out->p_rng = htonl(id);
          TLS_DEBUG("[x509write_crt_pem]p_rng(id): %lu\n", id);
        }

      *altcid = APICMDID_TLS_X509WRITE_CRT_PEM;
      ret_size = sizeof(struct apicmd_x509write_crt_pem_s);
    }
  else
#endif
    {
      ret_size = -ENOTSUP;
    }

  return ret_size;
}

int32_t mbedtlsstub_x509writecrtsubkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR mbedtls_pk_context *key = (FAR mbedtls_pk_context *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_key_s *out =
        (FAR struct apicmd_x509write_crt_subject_key_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->key = htonl(key->id);

      TLS_DEBUG("[x509write_crt_subject_key]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_subject_key]key id: %lu\n", key->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_SUBJECT_KEY;
      size = sizeof(struct apicmd_x509write_crt_subject_key_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtissuerkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR mbedtls_pk_context *key = (FAR mbedtls_pk_context *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_issuer_key_s *out =
        (FAR struct apicmd_x509write_crt_issuer_key_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->key = htonl(key->id);

      TLS_DEBUG("[x509write_crt_issuer_key]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_issuer_key]key id: %lu\n", key->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_ISSUER_KEY;
      size = sizeof(struct apicmd_x509write_crt_issuer_key_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtsubname_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  int32_t buflen = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR const char *issuer_name = (FAR const char *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_name_s *out =
        (FAR struct apicmd_x509write_crt_subject_name_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      memset(out->subject_name, '\0',
        APICMD_X509WRITE_CRT_SUBJECT_NAME_BUF_LEN);
      if (issuer_name != NULL)
        {
          buflen = strlen(issuer_name);
          if (buflen > APICMD_X509WRITE_CRT_SUBJECT_NAME_BUF_LEN)
            {
              return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
            }
          memcpy(out->subject_name, issuer_name, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }

      TLS_DEBUG("[x509write_crt_subject_name]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_subject_name]subject_name: %s\n", issuer_name);

      *altcid = APICMDID_TLS_X509WRITE_CRT_SUBJECT_NAME;
      size = sizeof(struct apicmd_x509write_crt_subject_name_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtissuername_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  int32_t buflen = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR const char *issuer_name = (FAR const char *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_issuer_name_s *out =
        (FAR struct apicmd_x509write_crt_issuer_name_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      memset(out->issuer_name, '\0', APICMD_X509WRITE_CRT_ISSUER_NAME_BUF_LEN);
      if (issuer_name != NULL)
        {
          buflen = strlen(issuer_name);
          if (buflen > APICMD_X509WRITE_CRT_ISSUER_NAME_BUF_LEN)
            {
              return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
            }
          memcpy(out->issuer_name, issuer_name, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }

      TLS_DEBUG("[x509write_crt_issuer_name]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_issuer_name]issuer_name: %s\n",
        issuer_name);

      *altcid = APICMDID_TLS_X509WRITE_CRT_ISSUER_NAME;
      size = sizeof(struct apicmd_x509write_crt_issuer_name_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtver_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR int *version = (FAR int *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_version_s *out =
        (FAR struct apicmd_x509write_crt_version_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->version = htonl(*version);

      TLS_DEBUG("[x509write_crt_version]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_version]version: %d\n", *version);

      *altcid = APICMDID_TLS_X509WRITE_CRT_VERSION;
      size = sizeof(struct apicmd_x509write_crt_version_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtmdalg_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR mbedtls_md_type_t *md_alg = (FAR mbedtls_md_type_t *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_md_alg_s *out =
        (FAR struct apicmd_x509write_crt_md_alg_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->md_alg = htonl(*md_alg);

      TLS_DEBUG("[x509write_crt_md_alg]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_md_alg]md_alg: %d\n", *md_alg);

      *altcid = APICMDID_TLS_X509WRITE_CRT_MD_ALG;
      size = sizeof(struct apicmd_x509write_crt_md_alg_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtserial_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR const mbedtls_mpi *serial = (FAR const mbedtls_mpi *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_serial_s *out =
        (FAR struct apicmd_x509write_crt_serial_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->serial = htonl(serial->id);

      TLS_DEBUG("[x509write_crt_serial]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_serial]serial id: %lu\n", serial->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_SERIAL;
      size = sizeof(struct apicmd_x509write_crt_serial_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtvalidity_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  int32_t buflen = 0;
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR const char *not_before = (FAR const char *)arg[1];
  FAR const char *not_after = (FAR const char *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_validity_s *out =
        (FAR struct apicmd_x509write_crt_validity_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      memset(out->not_before, '\0', APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN);
      if (not_before != NULL)
        {
          buflen = strlen(not_before);
          if (buflen > APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN)
            {
              return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
            }
          memcpy(out->not_before, not_before, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }

      memset(out->not_after, '\0', APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN);
      if (not_after != NULL)
        {
          buflen = strlen(not_after);
          if (buflen > APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN)
            {
              return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
            }
          memcpy(out->not_after, not_after, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }

      TLS_DEBUG("[x509write_crt_validity]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_validity]not_before: %s\n", not_before);
      TLS_DEBUG("[x509write_crt_validity]not_after: %s\n", not_after);

      *altcid = APICMDID_TLS_X509WRITE_CRT_VALIDITY;
      size = sizeof(struct apicmd_x509write_crt_validity_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtconst_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR int *is_ca = (FAR int *)arg[1];
  FAR int *max_pathlen = (FAR int *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_basic_constraints_s *out =
        (FAR struct apicmd_x509write_crt_basic_constraints_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->is_ca = htonl(*is_ca);
      out->max_pathlen = htonl(*max_pathlen);

      TLS_DEBUG("[x509write_crt_basic_constraints]ctx id: %lu\n",
        ctx->id);
      TLS_DEBUG("[x509write_crt_basic_constraints]is_ca: %d\n",
        *is_ca);
      TLS_DEBUG("[x509write_crt_basic_constraints]max_pathlen: %d\n",
        *max_pathlen);

      *altcid = APICMDID_TLS_X509WRITE_CRT_CONSTRAINTS;
      size = sizeof(struct apicmd_x509write_crt_basic_constraints_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtsubid_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_key_identifier_s *out =
        (FAR struct apicmd_x509write_crt_subject_key_identifier_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      TLS_DEBUG("[x509write_crt_subject_key_identifier]ctx id: %lu\n",
        ctx->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_SUBJECT_ID;
      size = sizeof(struct apicmd_x509write_crt_subject_key_identifier_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtauthid_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_authority_key_identifier_s *out =
        (FAR struct apicmd_x509write_crt_authority_key_identifier_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      TLS_DEBUG("[x509write_crt_authority_key_identifier]ctx id: %lu\n",
        ctx->id);

      *altcid = APICMDID_TLS_X509WRITE_CRT_AUTHORITY_ID;
      size = sizeof(struct apicmd_x509write_crt_authority_key_identifier_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtkeyusage_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR unsigned int *key_usage = (FAR unsigned int *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_key_usage_s *out =
        (FAR struct apicmd_x509write_crt_key_usage_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->key_usage = htonl(*key_usage);

      TLS_DEBUG("[x509write_crt_key_usage]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_key_usage]key_usage: %d\n",
        *key_usage);

      *altcid = APICMDID_TLS_X509WRITE_CRT_KEY_USAGE;
      size = sizeof(struct apicmd_x509write_crt_key_usage_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509writecrtnscerttype_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509write_cert *ctx = (FAR mbedtls_x509write_cert *)arg[0];
  FAR unsigned char *ns_cert_type = (FAR unsigned char *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_ns_cert_type_s *out =
        (FAR struct apicmd_x509write_crt_ns_cert_type_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->ns_cert_type = *ns_cert_type;

      TLS_DEBUG("[x509write_crt_ns_cert_type]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[x509write_crt_ns_cert_type]ns_cert_type: %d\n",
        *ns_cert_type);

      *altcid = APICMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE;
      size = sizeof(struct apicmd_x509write_crt_ns_cert_type_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}


int32_t mbedtlsstub_x509writecrtinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_initres_s *in =
        (FAR struct apicmd_x509write_crt_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_init res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_freeres_s *in =
        (FAR struct apicmd_x509write_crt_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_free res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  size_t req_buf_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];

  req_buf_len = (*size <= APICMD_X509WRITE_CRT_DER_BUF_LEN)
    ? *size : APICMD_X509WRITE_CRT_DER_BUF_LEN;
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_derres_s *in =
        (FAR struct apicmd_x509write_crt_derres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (*ret <= 0)
        {
          /* Nothing to do */
        }
      else if ((0 < *ret) && (*ret <= req_buf_len))
        {
          memcpy(buf, in->buf, *ret);
        }
      else
        {
          TLS_ERROR("Unexpected buffer length: %ld\n", *ret);
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }

      TLS_DEBUG("[x509write_crt_der res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtpem_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  size_t req_buf_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];

  req_buf_len = (*size <= APICMD_X509WRITE_CRT_PEM_BUF_LEN)
    ? *size : APICMD_X509WRITE_CRT_PEM_BUF_LEN;
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_pemres_s *in =
        (FAR struct apicmd_x509write_crt_pemres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (*ret == 0)
        {
          memcpy(buf, in->buf, req_buf_len);
        }

      TLS_DEBUG("[x509write_crt_pem res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtsubkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_keyres_s *in =
        (FAR struct apicmd_x509write_crt_subject_keyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_subject_key res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtissuerkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_issuer_keyres_s *in =
        (FAR struct apicmd_x509write_crt_issuer_keyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_issuer_key res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtsubname_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_nameres_s *in =
        (FAR struct apicmd_x509write_crt_subject_nameres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_subject_name res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtissuername_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_issuer_nameres_s *in =
        (FAR struct apicmd_x509write_crt_issuer_nameres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_issuer_name res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtver_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_versionres_s *in =
        (FAR struct apicmd_x509write_crt_versionres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_version res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtmdalg_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_md_algres_s *in =
        (FAR struct apicmd_x509write_crt_md_algres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_md_alg res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtserial_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_serialres_s *in =
        (FAR struct apicmd_x509write_crt_serialres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_serial res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtvalidity_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_validityres_s *in =
        (FAR struct apicmd_x509write_crt_validityres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_validity res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtconst_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_basic_constraintsres_s *in =
        (FAR struct apicmd_x509write_crt_basic_constraintsres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_basic_constraints res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtsubid_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_subject_key_identifierres_s *in =
        (FAR struct apicmd_x509write_crt_subject_key_identifierres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_subject_key_identifier res]ret: %ld\n",
        *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtauthid_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_authority_key_identifierres_s *in =
        (FAR struct apicmd_x509write_crt_authority_key_identifierres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_authority_key_identifier res]ret: %ld\n", *ret);
    }
  else
#endif
    {
     return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtkeyusage_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_key_usageres_s *in =
        (FAR struct apicmd_x509write_crt_key_usageres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_key_usage res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509writecrtnscerttype_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509write_crt_ns_cert_typeres_s *in =
        (FAR struct apicmd_x509write_crt_ns_cert_typeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509write_crt_ns_cert_type res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
