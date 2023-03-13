/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_rsa.c
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
#include <mbedtls/rsa.h>
#include <mbedtls/ctr_drbg.h>

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_rsa_init.h"
#include "include/apicmd_rsa_free.h"
#include "include/apicmd_rsa_gen_key.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_rsainit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int *padding = (FAR int *)arg[0];
  FAR int *hash_id = (FAR int *)arg[1];
  FAR int32_t *id = (FAR int32_t *)arg[2];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_RSA_CTX);
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_init_s *out =
        (FAR struct apicmd_rsa_init_s *)pktbuf;

      out->ctx = htonl(*id);
      out->padding = htonl(*padding);
      out->hash_id = htonl(*hash_id);
    
      TLS_DEBUG("[rsa_init]ctx id: %ld\n", *id);
      TLS_DEBUG("[rsa_init]padding: %d\n", *padding);
      TLS_DEBUG("[rsa_init]hash_id: %d\n", *hash_id);

      *altcid = APICMDID_TLS_RSA_INIT;
      size = sizeof(struct apicmd_rsa_init_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_rsafree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_rsa_context *ctx = (FAR mbedtls_rsa_context *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_free_s *out =
        (FAR struct apicmd_rsa_free_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      TLS_DEBUG("[rsa_free]ctx id: %lu\n", ctx->id);

      *altcid = APICMDID_TLS_RSA_FREE;
      size = sizeof(struct apicmd_rsa_free_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_rsagenkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_rsa_context *ctx = (FAR mbedtls_rsa_context *)arg[0];
  FAR void *p_rng = (FAR void *)arg[2];
  FAR unsigned int *nbits = (FAR unsigned int *)arg[3];
  FAR int *exponent = (FAR int *)arg[4];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_gen_key_s *out =
        (FAR struct apicmd_rsa_gen_key_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->nbits = htonl(*nbits);
      out->exponent = htonl(*exponent);

      TLS_DEBUG("[rsa_gen_key]config id: %lu\n", ctx->id);
      TLS_DEBUG("[rsa_gen_key]nbits: %d\n", *nbits);
      TLS_DEBUG("[rsa_gen_key]exponent: %d\n", *exponent);

      if (p_rng != NULL)
        {
          mbedtls_ctr_drbg_context *lctx = (mbedtls_ctr_drbg_context*)p_rng;
          uint32_t id = lctx->id;
          out->p_rng = htonl(id);
          TLS_DEBUG("[rsa_gen_key]p_rng(id): %lu\n", id);
        }

      *altcid = APICMDID_TLS_RSA_GEN_KEY;
      size = sizeof(struct apicmd_rsa_gen_key_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_rsainit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_initres_s *in =
        (FAR struct apicmd_rsa_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[rsa_init res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_rsafree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_freeres_s *in =
        (FAR struct apicmd_rsa_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[rsa_free res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_rsagenkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_rsa_gen_keyres_s *in =
        (FAR struct apicmd_rsa_gen_keyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[rsa_gen_key res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
