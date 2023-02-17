/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_ctr_drbg.c
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
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_ctr_drbg.h"
#include "include/apicmd_ctr_drbg_init.h"
#include "include/apicmd_ctr_drbg_free.h"
#include "include/apicmd_ctr_drbg_seed.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_ctrdrbginit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_CTR_DRBG_CTX);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_init_s *out =
        (FAR struct apicmd_ctr_drbg_init_s *)pktbuf;

      out->ctx = htonl(*id);

      *altcid = APICMDID_TLS_CTR_DRBG_INIT;
      size = sizeof(struct apicmd_ctr_drbg_init_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ctr_drbgcmd_s *out =
        (FAR struct apicmd_ctr_drbgcmd_s *)pktbuf;

      out->ctx = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CTR_DRBG_INIT);

      *altcid = APICMDID_TLS_CTR_DRBG_CMD_V4;
      size = sizeof(struct apicmd_ctr_drbgcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ctr_drbg_init]ctx id: %ld\n", *id);

  return size;
}

int32_t mbedtlsstub_ctrdrbgfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ctr_drbg_context *ctx =
    (FAR mbedtls_ctr_drbg_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_free_s *out =
        (FAR struct apicmd_ctr_drbg_free_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      *altcid = APICMDID_TLS_CTR_DRBG_FREE;
      size = sizeof(struct apicmd_ctr_drbg_free_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ctr_drbgcmd_s *out =
        (FAR struct apicmd_ctr_drbgcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CTR_DRBG_FREE);

      *altcid = APICMDID_TLS_CTR_DRBG_CMD_V4;
      size = sizeof(struct apicmd_ctr_drbgcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ctr_drbg_free]ctx id: %lu\n", ctx->id);

  return size;
}

int32_t mbedtlsstub_ctrdrbgseed_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ctr_drbg_context *ctx =
    (FAR mbedtls_ctr_drbg_context *)arg[0];
  FAR void *p_entropy = (FAR void *)arg[2];
  FAR const unsigned char *custom = (FAR const unsigned char *)arg[3];
  FAR size_t *len = (FAR size_t *)arg[4];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_seed_s *out =
        (FAR struct apicmd_ctr_drbg_seed_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      TLS_DEBUG("[ctr_drbg_seed]ctx id: %lu\n", ctx->id);

      if (p_entropy != NULL)
        {
          mbedtls_entropy_context *lctx =
            (mbedtls_entropy_context*) p_entropy;
          out->p_entropy = htonl(lctx->id);
          TLS_DEBUG("[ctr_drbg_seed]p_entropy: %lu\n", lctx->id);
        }
      else
        {
          out->p_entropy = 0;
          TLS_DEBUG("[ctr_drbg_seed]p_entropy: 0\n");
        }

      memset(out->custom, 0, APICMD_CTR_DRBG_SEED_CUSTOM_LEN);
      memcpy(out->custom, custom, *len);

      out->len = htonl(*len);

      *altcid = APICMDID_TLS_CTR_DRBG_SEED;
      size = sizeof(struct apicmd_ctr_drbg_seed_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ctr_drbgcmd_s *out =
        (FAR struct apicmd_ctr_drbgcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      TLS_DEBUG("[ctr_drbg_seed]ctx id: %lu\n", ctx->id);

      out->subcmd_id = htonl(APISUBCMDID_TLS_CTR_DRBG_SEED);

      if (p_entropy != NULL)
        {
          mbedtls_entropy_context *lctx =
            (mbedtls_entropy_context*)p_entropy;
          out->u.seed.p_entropy = htonl(lctx->id);
          TLS_DEBUG("[ctr_drbg_seed]p_entropy: %lu\n", lctx->id);
        }
      else
        {
          out->u.seed.p_entropy = 0;
          TLS_DEBUG("[ctr_drbg_seed]p_entropy: 0\n");
        }

      memset(out->u.seed.custom, 0, APICMD_CTR_DRBG_SEED_CUSTOM_LEN);
      memcpy(out->u.seed.custom, custom, *len);

      out->u.seed.len = htonl(*len);

      *altcid = APICMDID_TLS_CTR_DRBG_CMD_V4;
      size = sizeof(struct apicmd_ctr_drbgcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ctr_drbg_seed]len: %zu\n", *len);

  return size;
}

int32_t mbedtlsstub_ctrdrbginit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_initres_s *in =
        (FAR struct apicmd_ctr_drbg_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ctr_drbg_init res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_ctrdrbgfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_freeres_s *in =
        (FAR struct apicmd_ctr_drbg_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ctr_drbg_free res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_ctrdrbgseed_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ctr_drbg_seedres_s *in =
        (FAR struct apicmd_ctr_drbg_seedres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ctr_drbg_seed res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_ctrdrbgcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      return -1;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ctr_drbgcmdres_s *in =
        (FAR struct apicmd_ctr_drbgcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ctrdrbgcmd_pkt_parse res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
