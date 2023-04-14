/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_conf.c
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
#include <mbedtls/ssl.h>
#include <mbedtls/ctr_drbg.h>

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_config.h"
#include "include/apicmd_config_init.h"
#include "include/apicmd_config_free.h"
#include "include/apicmd_config_defaults.h"
#include "include/apicmd_config_authmode.h"
#include "include/apicmd_config_rng.h"
#include "include/apicmd_config_ca_chain.h"
#include "include/apicmd_config_own_cert.h"
#include "include/apicmd_config_read_timeout.h"
#include "include/apicmd_config_verify.h"
#include "include/apicmd_config_verify_callback.h"
#include "include/apicmd_config_alpn_protocols.h"
#include "include/apicmd_config_ciphersuites.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_confinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_CONFIG_CTX);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_init_s *out =
        (FAR struct apicmd_config_init_s *)pktbuf;

      out->conf = htonl(*id);

      *altcid = APICMDID_TLS_CONFIG_INIT;
      size = sizeof(struct apicmd_config_init_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_INIT);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[config_init]config id: %ld\n", *id);

  return size;
}

int32_t mbedtlsstub_conffree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_free_s *out =
        (FAR struct apicmd_config_free_s *)pktbuf;

      out->conf = htonl(conf->id);

      *altcid = APICMDID_TLS_CONFIG_FREE;
      size = sizeof(struct apicmd_config_free_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_FREE);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[config_free]config id: %lu\n", conf->id);

  return size;
}

int32_t mbedtlsstub_confdefauts_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR int *endpoint = (FAR int *)arg[1];
  FAR int *transport = (FAR int *)arg[2];
  FAR int *preset = (FAR int *)arg[3];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_defaults_s *out =
        (FAR struct apicmd_config_defaults_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->endpoint = (int32_t) htonl(*endpoint);
      out->transport = (int32_t) htonl(*transport);
      out->preset = (int32_t) htonl(*preset);

      *altcid = APICMDID_TLS_CONFIG_DEFAULTS;
      size = sizeof(struct apicmd_config_defaults_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_DEFAULTS);
      out->u.defaults.endpoint = (int32_t) htonl(*endpoint);
      out->u.defaults.transport = (int32_t) htonl(*transport);
      out->u.defaults.preset = (int32_t) htonl(*preset);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[config_defaults]config id: %lu\n", conf->id);
  TLS_DEBUG("[config_defaults]endpoint: %d\n", *endpoint);
  TLS_DEBUG("[config_defaults]transport: %d\n", *transport);
  TLS_DEBUG("[config_defaults]preset: %d\n", *preset);

  return size;
}

int32_t mbedtlsstub_confauth_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR int *authmode = (FAR int *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_authmode_s *out =
        (FAR struct apicmd_config_authmode_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->authmode = (int32_t) htonl(*authmode);

      *altcid = APICMDID_TLS_CONFIG_AUTHMODE;
      size = sizeof(struct apicmd_config_authmode_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_AUTHMODE);
      out->u.authmode.authmode = (int32_t) htonl(*authmode);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[config_authmode]config id: %lu\n", conf->id);
  TLS_DEBUG("[config_authmode]authmode: %d\n", *authmode);

  return size;
}

int32_t mbedtlsstub_confrng_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR void *p_rng = (FAR void *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_rng_s *out =
        (FAR struct apicmd_config_rng_s *)pktbuf;

      out->conf = htonl(conf->id);

      *altcid = APICMDID_TLS_CONFIG_RNG;
      size = sizeof(struct apicmd_config_rng_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_RNG);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[config_rng]config id: %lu\n", conf->id);

  if (p_rng != NULL)
    {
      mbedtls_ctr_drbg_context *ctx = (mbedtls_ctr_drbg_context*)p_rng;
      uint32_t id = ctx->id;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
      if (altver == ALTCOM_VER1)
        {
        FAR struct apicmd_config_rng_s *out =
          (FAR struct apicmd_config_rng_s *)pktbuf;

          out->p_rng = htonl(id);
        }
      else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
      if (altver == ALTCOM_VER4)
        {
        FAR struct apicmd_configcmd_s *out =
          (FAR struct apicmd_configcmd_s *)pktbuf;

          out->u.rng.p_rng = htonl(id);
        }
      else
#endif
        {
          return -ENOSYS;
        }

      TLS_DEBUG("[config_rng]p_rng(id): %lu\n", id);
    }

  return size;
}

int32_t mbedtlsstub_confcachain_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR mbedtls_x509_crt *ca_chain = (FAR mbedtls_x509_crt *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_ca_chain_s *out =
        (FAR struct apicmd_config_ca_chain_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->ca_chain = (ca_chain != NULL) ? htonl(ca_chain->id) : 0;
      out->ca_crl = 0; // force crl_id zero

      *altcid = APICMDID_TLS_CONFIG_CA_CHAIN;
      size = sizeof(struct apicmd_config_ca_chain_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_CA_CHAIN);
      out->u.ca_chain.ca_chain = (ca_chain != NULL) ? htonl(ca_chain->id) : 0;
      out->u.ca_chain.ca_crl = 0; // force crl_id zero

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[config_ca_chain]config id: %lu\n", conf->id);
  TLS_DEBUG("[config_ca_chain]crt id: %lu\n", (ca_chain != NULL) ? ca_chain->id : 0);

  return size;
}

int32_t mbedtlsstub_confowncert_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR mbedtls_x509_crt *own_cert = (FAR mbedtls_x509_crt *)arg[1];
  FAR mbedtls_pk_context *pk_key = (FAR mbedtls_pk_context *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_own_cert_s *out =
        (FAR struct apicmd_config_own_cert_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->own_cert = (own_cert != NULL) ? htonl(own_cert->id) : 0;
      out->pk_key = htonl(pk_key->id);

      *altcid = APICMDID_TLS_CONFIG_OWN_CERT;
      size = sizeof(struct apicmd_config_own_cert_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_OWN_CERT);
      out->u.own_cert.own_cert = (own_cert != NULL) ? htonl(own_cert->id) : 0;
      out->u.own_cert.pk_key = htonl(pk_key->id);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[config_own_cert]config id: %lu\n", conf->id);
  TLS_DEBUG("[config_own_cert]own_cert: %lu\n", (own_cert != NULL) ? own_cert->id : 0);
  TLS_DEBUG("[config_own_cert]pk_key: %lu\n", pk_key->id);

  return size;
}

int32_t mbedtlsstub_confreadtimeo_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR uint32_t *timeout = (FAR uint32_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_read_timeout_s *out =
        (FAR struct apicmd_config_read_timeout_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->timeout = htonl(*timeout);

      *altcid = APICMDID_TLS_CONFIG_READ_TIMEOUT;
      size = sizeof(struct apicmd_config_read_timeout_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_READ_TIMEOUT);
      out->u.read_timeout.timeout = htonl(*timeout);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(FAR struct apicmd_configcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[config_read_timeout]config id: %lu\n", conf->id);
  TLS_DEBUG("[config_read_timeout]timeout: %lu\n", *timeout);

  return size;
}

int32_t mbedtlsstub_confvrfy_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_verify_s *out =
        (FAR struct apicmd_config_verify_s *)pktbuf;

      out->conf = htonl(conf->id);

      *altcid = APICMDID_TLS_CONFIG_VERIFY;
      size = sizeof(struct apicmd_config_verify_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_VERIFY);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[config_verify]config id: %lu\n", conf->id);

  return size;
}

int32_t mbedtlsstub_confvrfycb_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int *ret = (FAR int *)arg[0];
  FAR uint32_t *flags = (FAR uint32_t *)arg[1];

  FAR struct apicmd_config_verify_callback_s *out =
    (FAR struct apicmd_config_verify_callback_s *)pktbuf;

  size = sizeof(struct apicmd_config_verify_callback_s);

  out->ret_code = htonl(*ret);
  out->flags = htonl(*flags);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_TLS_CONFIG_VERIFY_CALLBACK;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_TLS_CONFIG_VERIFY_CALLBACK_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t mbedtlsstub_confalpnproto_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  char **p;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR const char **protos = (FAR const char **)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_alpn_protocols_s *out =
        (FAR struct apicmd_config_alpn_protocols_s *)pktbuf;

      out->conf = htonl(conf->id);
      p = (char **)protos;

      memset(out->protos1, 0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->protos1, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->protos2, 0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->protos2, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->protos3, 0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->protos3, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->protos4, 0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->protos4, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }

      TLS_DEBUG("[config_alpn_protocols]config id: %lu\n", conf->id);
      TLS_DEBUG("[config_alpn_protocols]protos1: %s\n", out->protos1);
      TLS_DEBUG("[config_alpn_protocols]protos2: %s\n", out->protos2);
      TLS_DEBUG("[config_alpn_protocols]protos3: %s\n", out->protos3);
      TLS_DEBUG("[config_alpn_protocols]protos4: %s\n", out->protos4);

      *altcid = APICMDID_TLS_CONFIG_ALPN_PROTOCOLS;
      size = sizeof(struct apicmd_config_alpn_protocols_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      p = (char **)protos;

      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_ALPN_PROTOCOLS);
      memset(out->u.alpn_protocols.protos1,
        0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->u.alpn_protocols.protos1, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->u.alpn_protocols.protos2,
        0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->u.alpn_protocols.protos2, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->u.alpn_protocols.protos3,
        0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->u.alpn_protocols.protos3, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      memset(out->u.alpn_protocols.protos4,
        0, APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN);
      if (*p != NULL)
        {
          strncpy((char *)out->u.alpn_protocols.protos4, *p,
            APICMD_CONFIG_ALPN_PROTOCOLS_PROTOS_LEN-1);
          p++;
        }
      TLS_DEBUG("[config_alpn_protocols]config id: %lu\n", conf->id);
      TLS_DEBUG("[config_alpn_protocols]protos1: %s\n",
        out->u.alpn_protocols.protos1);
      TLS_DEBUG("[config_alpn_protocols]protos2: %s\n",
        out->u.alpn_protocols.protos2);
      TLS_DEBUG("[config_alpn_protocols]protos3: %s\n",
        out->u.alpn_protocols.protos3);
      TLS_DEBUG("[config_alpn_protocols]protos4: %s\n",
        out->u.alpn_protocols.protos4);

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t mbedtlsstub_confciphersuites_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  int cnt;
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[0];
  FAR const int *ciphersuites = (FAR const int *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_ciphersuites_s *out =
        (FAR struct apicmd_config_ciphersuites_s *)pktbuf;

      out->conf = htonl(conf->id);
      memset(out->ciphersuites,
        0, sizeof(int32_t)*APICMD_CONFIG_CIPHERSUITES_COUNT);

      cnt = 0;
      for (cnt = 0; cnt < APICMD_CONFIG_CIPHERSUITES_COUNT; cnt++)
        {
          if (ciphersuites[cnt] == 0)
            {
              break;
            }

          out->ciphersuites[cnt] = htonl(ciphersuites[cnt]);
        }

      *altcid = APICMDID_TLS_CONFIG_CIPHERSUITES;
      size = sizeof(struct apicmd_config_ciphersuites_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_configcmd_s *out =
        (FAR struct apicmd_configcmd_s *)pktbuf;

      out->conf = htonl(conf->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_CONFIG_CIPHERSUITES);
      memset(out->u.ciphersuites.ciphersuites, 0,
        sizeof(int32_t)*APICMD_CONFIG_CIPHERSUITES_COUNT);

      cnt = 0;
      for (cnt = 0; cnt < APICMD_CONFIG_CIPHERSUITES_COUNT; cnt++)
        {
          if (ciphersuites[cnt] == 0)
            {
              break;
            }

          out->u.ciphersuites.ciphersuites[cnt] =
             htonl(ciphersuites[cnt]);
        }

      *altcid = APICMDID_TLS_CONFIG_CMD_V4;
      size = sizeof(struct apicmd_configcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[config_ciphersuites]config id: %lu\n", conf->id);

  return size;
}

int32_t mbedtlsstub_confinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_initres_s *in =
        (FAR struct apicmd_config_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_init res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_conffree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_freeres_s *in =
        (FAR struct apicmd_config_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_free res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_confdefauts_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_defaultsres_s *in =
        (FAR struct apicmd_config_defaultsres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_defaults res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_confauth_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_authmoderes_s *in =
        (FAR struct apicmd_config_authmoderes_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_authmode res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_confrng_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_rngres_s *in =
        (FAR struct apicmd_config_rngres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_rng res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_confcachain_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_ca_chainres_s *in =
        (FAR struct apicmd_config_ca_chainres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_ca_chain res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_confowncert_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_own_certres_s *in =
        (FAR struct apicmd_config_own_certres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_own_cert res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_confreadtimeo_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_read_timeoutres_s *in =
        (FAR struct apicmd_config_read_timeoutres_s *)pktbuf;

      *ret = ntohl( in->ret_code);
      TLS_DEBUG("[config_read_timeout res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_confvrfy_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_verifyres_s *in =
        (FAR struct apicmd_config_verifyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_verify res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_confvrfycb_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR uint32_t *crt = (FAR uint32_t *)arg[0];
  FAR int32_t *depth = (FAR int32_t *)arg[1];

  FAR struct apicmd_config_verify_callbackres_s *in =
    (FAR struct apicmd_config_verify_callbackres_s *)pktbuf;

  *crt = htonl(in->crt);
  *depth = htonl(in->certificate_depth);

  return 0;
}

int32_t mbedtlsstub_confalpnproto_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_alpn_protocolsres_s *in =
        (FAR struct apicmd_config_alpn_protocolsres_s *)pktbuf;

      *ret = ntohl( in->ret_code);
      TLS_DEBUG("[config_alpn_protocols res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_confciphersuites_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_config_ciphersuitesres_s *in =
        (FAR struct apicmd_config_ciphersuitesres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[config_ciphersuites res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_configcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
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
      FAR struct apicmd_configcmdres_s *in =
        (FAR struct apicmd_configcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[configcmd_pkt_parse res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
