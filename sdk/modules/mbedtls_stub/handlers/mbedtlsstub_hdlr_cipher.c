/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_cipher.c
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
#include <mbedtls/cipher.h>

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_pk.h"
#include "include/apicmd_cipher.h"
#include "include/apicmd_cipher_init.h"
#include "include/apicmd_cipher_free.h"
#include "include/apicmd_cipher_info_from_string.h"
#include "include/apicmd_cipher_setup.h"
#include "include/apicmd_cipher_setkey.h"
#include "include/apicmd_cipher_set_iv.h"
#include "include/apicmd_cipher_update.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_cipherinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_CIPHER_CTX);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_init_s *out =
        (FAR struct apicmd_cipher_init_s *)pktbuf;

      out->ctx = htonl(*id);

      *altcid = APICMDID_TLS_CIPHER_INIT;
      size = sizeof(struct apicmd_cipher_init_s);
    }
  else
#endif
    {
      return -ENOTSUP;
    }

  TLS_DEBUG("[cipher_init]ctx id: %ld\n", *id);

  return size;
}

int32_t mbedtlsstub_cipherfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_cipher_context_t *ctx =
    (FAR mbedtls_cipher_context_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_free_s *out =
        (FAR struct apicmd_cipher_free_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      *altcid = APICMDID_TLS_CIPHER_FREE;
      size = sizeof(struct apicmd_cipher_free_s);
    }
  else
#endif
    {
      return -ENOTSUP;
    }

  TLS_DEBUG("[cipher_free]ctx id: %lu\n", ctx->id);

  return size;
}

int32_t mbedtlsstub_cipherinfofromstr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR const char *cipher_name = (FAR const char *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_info_from_string_s *out =
        (FAR struct apicmd_cipher_info_from_string_s *)pktbuf;

      memset(out->cipher_name, '\0', APICMD_CIPHER_INFO_NAME_LEN);
      strncpy((char*) out->cipher_name, cipher_name,
        APICMD_CIPHER_INFO_NAME_LEN-1);
      TLS_DEBUG("[cipher_info_from_string]cipher name: %s\n",
        out->cipher_name);

      *altcid = APICMDID_TLS_CIPHER_INFO_FROM_STRING;
      size = sizeof(struct apicmd_cipher_info_from_string_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_ciphersetup_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_cipher_context_t *ctx =
    (FAR mbedtls_cipher_context_t *)arg[0];
  FAR mbedtls_cipher_info_t *cipher_info =
    (FAR mbedtls_cipher_info_t *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_setup_s *out =
        (FAR struct apicmd_cipher_setup_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->cipher_info = htonl(cipher_info->id);

      TLS_DEBUG("[cipher_setup]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[cipher_setup]info id: %lu\n", cipher_info->id);

      *altcid = APICMDID_TLS_CIPHER_SETUP;
      size = sizeof(struct apicmd_cipher_setup_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_ciphersetkey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_cipher_context_t *ctx =
    (FAR mbedtls_cipher_context_t *)arg[0];
  FAR const unsigned char *key = (FAR const unsigned char *)arg[1];
  FAR int *key_bitlen = (FAR int *)arg[2];
  FAR mbedtls_operation_t *operation = (FAR mbedtls_operation_t *)arg[3];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_setkey_s *out =
        (FAR struct apicmd_cipher_setkey_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      memset(out->key, 0, APICMD_CIPHER_SETKEY_LEN);
      memcpy(out->key, key, APICMD_CIPHER_SETKEY_LEN);
      out->key_bitlen = htonl(*key_bitlen);
      out->operation = htonl(*operation);

      TLS_DEBUG("[cipher_setkey]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[cipher_setkey]key_bitlen: %d\n", *key_bitlen);
      TLS_DEBUG("[cipher_setkey]operation: %d\n", *operation);

      *altcid = APICMDID_TLS_CIPHER_SETKEY;
      size = sizeof(struct apicmd_cipher_setkey_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_ciphersetiv_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_cipher_context_t *ctx = (FAR mbedtls_cipher_context_t *)arg[0];
  FAR const unsigned char *iv = (FAR const unsigned char *)arg[1];
  FAR size_t *iv_len = (FAR size_t *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_set_iv_s *out =
        (FAR struct apicmd_cipher_set_iv_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      memset(out->iv, 0, APICMD_CIPHER_SET_IV_LEN);
      memcpy(out->iv, iv, *iv_len);
      out->iv_len = htonl(*iv_len);

      TLS_DEBUG("[cipher_set_iv]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[cipher_set_iv]iv_len: %zu\n", *iv_len);

      *altcid = APICMDID_TLS_CIPHER_SET_IV;
      size = sizeof(struct apicmd_cipher_set_iv_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_cipherupdate_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_cipher_context_t *ctx =
    (FAR mbedtls_cipher_context_t *)arg[0];
  FAR const unsigned char *input = (FAR const unsigned char *)arg[1];
  FAR size_t *ilen = (FAR size_t *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_update_s *out =
        (FAR struct apicmd_cipher_update_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      if (*ilen <= APICMD_CIPHER_UPDATE_INPUT_LEN)
        {
          memset(out->input, 0, APICMD_CIPHER_UPDATE_INPUT_LEN);
          memcpy(out->input, input, *ilen);
        }
      else
        {
          return MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
        }

      out->ilen = htonl(*ilen);

      TLS_DEBUG("[cipher_update]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[cipher_update]ilen: %d\n", *ilen);

      *altcid = APICMDID_TLS_CIPHER_UPDATE;
      size = sizeof(struct apicmd_cipher_update_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_cipherinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_initres_s *in =
        (FAR struct apicmd_cipher_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[cipher_init res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_cipherfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_freeres_s *in =
        (FAR struct apicmd_cipher_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[cipher_free res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_cipherinfofromstr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR mbedtls_cipher_info_t *cipher_info =
    (FAR mbedtls_cipher_info_t *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_info_from_stringres_s *in =
        (FAR struct apicmd_cipher_info_from_stringres_s *)pktbuf;

      *ret = ntohl(in->cipher_info);
      cipher_info->id = *ret;

      TLS_DEBUG("[cipher_info_from_string res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_ciphersetup_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_setupres_s *in =
        (FAR struct apicmd_cipher_setupres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[cipher_setup res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_ciphersetkey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_setkeyres_s *in =
        (FAR struct apicmd_cipher_setkeyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[cipher_setkey res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_ciphersetiv_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_set_ivres_s *in =
        (FAR struct apicmd_cipher_set_ivres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[cipher_set_iv res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_cipherupdate_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *output = (FAR unsigned char *)arg[1];
  FAR size_t *olen = (FAR size_t *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cipher_updateres_s *in =
        (FAR struct apicmd_cipher_updateres_s *)pktbuf;
      size_t len = ntohl(in->olen);

      *ret = ntohl(in->ret_code);
      if (*olen < len)
        {
          *ret = MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
        }
      else
        {
          memcpy(output, in->output, len);
        }

      *olen = len;

      TLS_DEBUG("[cipher_update res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_ciphercmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
  int32_t ret = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      TLS_ERROR("Unexpected ALTCOM version: %u\n",altver);
      ret = -1;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      uint32_t subcmd_id = ntohl(in->subcmd_id);

      if (subcmd_id == 0 || subcmd_id > APISUBCMDID_TLS_SHA1)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n", subcmd_id);
          return -1;
        }
      if (subcmd_id == APISUBCMDID_TLS_MD_INFO_FROM_TYPE)
        {
          ret = mbedtlsstub_mdinfofromtype_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen,
                                         bitmap);
        }
      else if (subcmd_id == APISUBCMDID_TLS_MD_GET_SIZE)
        {
          ret = mbedtlsstub_mdgetsize_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen,
                                    bitmap);
        }
      else if (subcmd_id == APISUBCMDID_TLS_MD)
        {
          ret = mbedtlsstub_md_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen, bitmap);
        }
      else if (subcmd_id == APISUBCMDID_TLS_MD_DIGEST)
        {
          ret = mbedtlsstub_mddigest_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen,
                                   bitmap);
        }
      else if (subcmd_id == APISUBCMDID_TLS_BASE64_ENCODE)
        {
          ret = mbedtlsstub_base64enc_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen,
                                    bitmap);
        }
      else if (subcmd_id == APISUBCMDID_TLS_SHA1)
        {
          ret = mbedtlsstub_sha1_pkt_parse(dev, pktbuf, pktsz, altver, arg, arglen, bitmap);
          TLS_DEBUG("[sha1 res]ret: %ld\n", ntohl(in->ret_code));
        }
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return ret;
}
