/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_pk.c
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
#include <mbedtls/pk_internal.h>

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_pk.h"
#include "include/apicmd_pk_init.h"
#include "include/apicmd_pk_free.h"
#include "include/apicmd_pk_parse_keyfile.h"
#include "include/apicmd_pk_parse_key.h"
#include "include/apicmd_pk_check_pair.h"
#include "include/apicmd_pk_setup.h"
#include "include/apicmd_pk_info_from_type.h"
#include "include/apicmd_pk_write_key_pem.h"
#include "include/apicmd_pk_write_key_der.h"
#include "include/apicmd_pk_rsa.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_pkinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_PK_CTX);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_init_s *out =
        (FAR struct apicmd_pk_init_s *)pktbuf;

      out->ctx = htonl(*id);

      *altcid = APICMDID_TLS_PK_INIT;
      size = sizeof(struct apicmd_pk_init_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_INIT);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[pk_init]ctx id: %ld\n", *id);

  return size;
}

int32_t mbedtlsstub_pkfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_crt *crt = (FAR mbedtls_x509_crt *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_free_s *out =
        (FAR struct apicmd_pk_free_s *)pktbuf;

      out->ctx = htonl(crt->id);

      *altcid = APICMDID_TLS_PK_FREE;
      size = sizeof(struct apicmd_pk_free_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(crt->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_FREE);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[pk_free]ctx id: %lu\n", crt->id);

  return size;
}

int32_t mbedtlsstub_pkparsekeyfile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_pk_context *ctx = (FAR mbedtls_pk_context *)arg[0];
  FAR const char *path = (FAR const char *)arg[1];
  FAR const char *password = (FAR const char *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_parse_keyfile_s *out =
        (FAR struct apicmd_pk_parse_keyfile_s *)pktbuf;

      out->ctx = htonl(ctx->id);
  
      memset(out->path, '\0', APICMD_PK_PARSE_KEYFILE_PATH_LEN);
      if (path != NULL)
        {
          strncpy((char*) out->path,
            path, APICMD_PK_PARSE_KEYFILE_PATH_LEN-1);
        }
  
      memset(out->pwd, '\0', APICMD_PK_PARSE_KEYFILE_PWD_LEN);
      if (password != NULL)
        {
          strncpy((char*)out->pwd,
            password, APICMD_PK_PARSE_KEYFILE_PWD_LEN-1);
        }
  
      TLS_DEBUG("[pk_parse_keyfile]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[pk_parse_keyfile]path: %s\n", out->path);
      TLS_DEBUG("[pk_parse_keyfile]pwd: %s\n", out->pwd);

      *altcid = APICMDID_TLS_PK_PARSE_KEYFILE;
      size = sizeof(struct apicmd_pk_parse_keyfile_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_PARSE_KEYFILE);
      memset(out->u.parse_keyfile.path, '\0',
        APICMD_PK_PARSE_KEYFILE_PATH_LEN);
      if (path != NULL)
        {
          strncpy((char *)out->u.parse_keyfile.path, path,
            APICMD_PK_PARSE_KEYFILE_PATH_LEN-1);
        }

      memset(out->u.parse_keyfile.pwd, '\0',
        APICMD_PK_PARSE_KEYFILE_PWD_LEN);
      if (password != NULL)
        {
          strncpy((char *)out->u.parse_keyfile.pwd, password,
            APICMD_PK_PARSE_KEYFILE_PWD_LEN-1);
        }

      TLS_DEBUG("[pk_parse_keyfile]ctx id: %lu\n", ctx->id);
      TLS_DEBUG("[pk_parse_keyfile]path: %s\n",
        out->u.parse_keyfile.path);
      TLS_DEBUG("[pk_parse_keyfile]pwd: %s\n",
        out->u.parse_keyfile.pwd);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t mbedtlsstub_pkparsekey_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  uint32_t buflen = 0;
  FAR mbedtls_pk_context *ctx = (FAR mbedtls_pk_context *)arg[0];
  FAR const unsigned char *key = (FAR const unsigned char *)arg[1];
  FAR size_t *keylen = (FAR size_t *)arg[2];
  FAR const unsigned char *pwd = (FAR const unsigned char *)arg[3];
  FAR size_t *pwdlen = (FAR size_t *)arg[4];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_parse_key_s *out =
        (FAR struct apicmd_pk_parse_key_s *)pktbuf;

      out->ctx = htonl(ctx->id);

      if (*keylen <= APICMD_PK_PARSE_KEY_KEY_LEN)
        {
          buflen = *keylen;
        }
      else
        {
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }
      memset(out->key, '\0', APICMD_PK_PARSE_KEY_KEY_LEN);
      if (key != NULL)
        {
          memcpy(out->key, key, buflen);
        }
      out->keylen = htonl(buflen);

      if (*pwdlen <= APICMD_PK_PARSE_KEY_PWD_LEN)
        {
          buflen = *pwdlen;
        }
      else
        {
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }
      memset(out->pwd, '\0', APICMD_PK_PARSE_KEY_PWD_LEN);
      if (pwd != NULL)
        {
          memcpy(out->pwd, pwd, buflen);
        }
      out->pwdlen = htonl(buflen);

      *altcid = APICMDID_TLS_PK_PARSE_KEY;
      size = sizeof(struct apicmd_pk_parse_key_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_PARSE_KEY);
      if (*keylen <= APICMD_PK_PARSE_KEY_KEY_LEN)
        {
          buflen = *keylen;
        }
      else
        {
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }
      memset(out->u.parse_key.key, '\0', APICMD_PK_PARSE_KEY_KEY_LEN);
      if (key != NULL)
        {
          memcpy(out->u.parse_key.key, key, buflen);
        }
      out->u.parse_key.keylen = htonl(buflen);

      if (*pwdlen <= APICMD_PK_PARSE_KEY_PWD_LEN)
        {
          buflen = *pwdlen;
        }
      else
        {
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }
      memset(out->u.parse_key.pwd, '\0', APICMD_PK_PARSE_KEY_PWD_LEN);
      if (pwd != NULL)
        {
          memcpy(out->u.parse_key.pwd, pwd, buflen);
        }
      out->u.parse_key.pwdlen = htonl(buflen);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[pk_parse_key]ctx id: %lu\n", ctx->id);
  TLS_DEBUG("[pk_parse_key]key: %s\n", key);
  TLS_DEBUG("[pk_parse_key]keylen: %d\n", *keylen);
  TLS_DEBUG("[pk_parse_key]pwd: %s\n", pwd);
  TLS_DEBUG("[pk_parse_key]pwdlen: %d\n", *pwdlen);

  return size;
}

int32_t mbedtlsstub_pkcheckpair_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_pk_context *pub =
    (FAR const mbedtls_pk_context *)arg[0];
  FAR const mbedtls_pk_context *prv =
    (FAR const mbedtls_pk_context *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_check_pair_s *out =
        (FAR struct apicmd_pk_check_pair_s *)pktbuf;

      out->pub = htonl(pub->id);
      out->prv = htonl(prv->id);

      *altcid = APICMDID_TLS_PK_CHECK_PAIR;
      size = sizeof(struct apicmd_pk_check_pair_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(pub->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_CHECK_PAIR);
      out->u.check_pair.prv = htonl(prv->id);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[pk_check_pair]pub id: %lu\n", pub->id);
  TLS_DEBUG("[pk_check_pair]prv id: %lu\n", prv->id);

  return size;
}

int32_t mbedtlsstub_pksetup_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_pk_context *ctx = (FAR mbedtls_pk_context *)arg[0];
  FAR const struct mbedtls_pk_info_t *info =
    (FAR const struct mbedtls_pk_info_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_setup_s *out =
        (FAR struct apicmd_pk_setup_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->info = htonl(info->id);

      *altcid = APICMDID_TLS_PK_SETUP;
      size = sizeof(struct apicmd_pk_setup_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_SETUP);
      out->u.setup.info = htonl(info->id);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[pk_setup]ctx id: %lu\n", ctx->id);
  TLS_DEBUG("[pk_setup]info id: %lu\n", info->id);

  return size;
}

int32_t mbedtlsstub_pkinfofromtype_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_pk_type_t *pk_type = (FAR mbedtls_pk_type_t *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_info_from_type_s *out =
        (FAR struct apicmd_pk_info_from_type_s *)pktbuf;

      out->pk_type = htonl(*pk_type);

      *altcid = APICMDID_TLS_PK_INFO_FROM_TYPE;
      size = sizeof(struct apicmd_pk_info_from_type_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->u.info_from_type.pk_type = htonl(*pk_type);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_INFO_FROM_TYPE);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[pk_info_from_type]pk_type id: %d\n", *pk_type);

  return size;
}

int32_t mbedtlsstub_pkwritekeypem_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  size_t   req_buf_len = 0;
  FAR mbedtls_pk_context *ctx = (FAR mbedtls_pk_context *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_write_key_pem_s *out =
        (FAR struct apicmd_pk_write_key_pem_s *)pktbuf;

      out->ctx = htonl(ctx->id);
  
      req_buf_len = (*size <= APICMD_PK_WRITE_KEY_PEM_BUF_LEN)
        ? *size : APICMD_PK_WRITE_KEY_PEM_BUF_LEN;
  
      out->size = htonl(req_buf_len);

      *altcid = APICMDID_TLS_PK_WRITE_KEY_PEM;
      ret_size = sizeof(struct apicmd_pk_write_key_pem_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
  
      req_buf_len = (*size <= APICMD_PK_WRITE_KEY_PEM_BUF_LEN)
        ? *size : APICMD_PK_WRITE_KEY_PEM_BUF_LEN;
  
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_WRITE_KEY_PEM);
      out->u.write_key_pem.size = htonl(req_buf_len);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      ret_size = sizeof(struct apicmd_pkcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[pk_write_key_pem]config id: %lu\n", ctx->id);

  return ret_size;
}

int32_t mbedtlsstub_pkwritekeyder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  size_t   req_buf_len = 0;
  FAR mbedtls_pk_context *ctx = (FAR mbedtls_pk_context *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_write_key_der_s *out =
        (FAR struct apicmd_pk_write_key_der_s *)pktbuf;

      out->ctx = htonl(ctx->id);
  
      req_buf_len = (*size <= APICMD_PK_WRITE_KEY_DER_BUF_LEN)
        ? *size : APICMD_PK_WRITE_KEY_DER_BUF_LEN;
  
      out->size = htonl(req_buf_len);

      *altcid = APICMDID_TLS_PK_WRITE_KEY_DER;
      ret_size = sizeof(struct apicmd_pk_write_key_der_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(ctx->id);
  
      req_buf_len = (*size <= APICMD_PK_WRITE_KEY_DER_BUF_LEN)
        ? *size : APICMD_PK_WRITE_KEY_DER_BUF_LEN;
  
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_WRITE_KEY_DER);
      out->u.write_key_der.size = htonl(req_buf_len);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      ret_size = sizeof(struct apicmd_pkcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[pk_write_key_der]config id: %lu\n", ctx->id);

  return ret_size;
}

int32_t mbedtlsstub_pkrsa_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_pk_context *pk =
    (FAR const mbedtls_pk_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_rsa_s *out =
        (FAR struct apicmd_pk_rsa_s *)pktbuf;

      out->pk = htonl(pk->id);

      *altcid = APICMDID_TLS_PK_RSA;
      size = sizeof(struct apicmd_pk_rsa_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmd_s *out =
        (FAR struct apicmd_pkcmd_s *)pktbuf;

      out->ctx = htonl(pk->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_PK_RSA);

      *altcid = APICMDID_TLS_PK_CMD_V4;
      size = sizeof(struct apicmd_pkcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[pk_rsa]pk id: %lu\n", pk->id);

  return size;
}


int32_t mbedtlsstub_pkinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_initres_s *in =
        (FAR struct apicmd_pk_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_init res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_pkfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_freeres_s *in =
        (FAR struct apicmd_pk_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_free res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_pkparsekeyfile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_parse_keyfileres_s *in =
        (FAR struct apicmd_pk_parse_keyfileres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_parse_keyfile res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_pkparsekey_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_parse_keyres_s *in =
        (FAR struct apicmd_pk_parse_keyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_parse_key res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_pkcheckpair_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_check_pairres_s *in =
        (FAR struct apicmd_pk_check_pairres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_check_pair res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_pksetup_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_setupres_s *in =
        (FAR struct apicmd_pk_setupres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_setup res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_pkinfofromtype_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR mbedtls_pk_info_t *pk_info = (FAR mbedtls_pk_info_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_info_from_typeres_s *in =
        (FAR struct apicmd_pk_info_from_typeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (*ret == 0)
        {
          pk_info->id = ntohl(in->pk_info);
        }
      TLS_DEBUG("[pk_info_from_type res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_PK_INFO_FROM_TYPE)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }

      pk_info->id = ntohl(in->u.info_from_typeres.pk_info);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_pkwritekeypem_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  size_t req_buf_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];

  req_buf_len = (*size <= APICMD_PK_WRITE_KEY_PEM_BUF_LEN)
    ? *size : APICMD_PK_WRITE_KEY_PEM_BUF_LEN;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_write_key_pemres_s *in =
        (FAR struct apicmd_pk_write_key_pemres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (*ret == 0)
        {
          memcpy(buf, in->buf, req_buf_len);
        }
      TLS_DEBUG("[pk_write_key_pem res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_pkcmdres_s *in =
        (FAR struct apicmd_pkcmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_PK_WRITE_KEY_PEM)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
        }

      memcpy(buf, in->u.write_key_pemres.buf, req_buf_len);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_pkwritekeyder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  size_t req_buf_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_write_key_derres_s *in =
        (FAR struct apicmd_pk_write_key_derres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[pk_write_key_der res]ret: %ld\n", *ret);
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

  req_buf_len = (*size <= APICMD_PK_WRITE_KEY_DER_BUF_LEN)
    ? *size : APICMD_PK_WRITE_KEY_DER_BUF_LEN;

  if (*ret <= 0)
    {
      /* Nothing to do */
    }
  else if ((0 < *ret) && (*ret <= req_buf_len))
    {
      FAR struct apicmd_pk_write_key_derres_s *in =
        (FAR struct apicmd_pk_write_key_derres_s *)pktbuf;

        memcpy(buf, in->buf, *ret);
    }
  else
    {
      TLS_ERROR("Unexpected buffer length: %ld\n", *ret);
      return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
    }

  return 0;
}

int32_t mbedtlsstub_pkrsa_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR mbedtls_rsa_context *rsa_context =
    (FAR mbedtls_rsa_context *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_pk_rsares_s *in =
        (FAR struct apicmd_pk_rsares_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (*ret == 0)
        {
          rsa_context->id = ntohl(in->rsa);
        }
      TLS_DEBUG("[pk_rsa res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_pkcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                             size_t pktsz, uint8_t altver, FAR void **arg,
                             size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  FAR int32_t *ret_code = (FAR int32_t *)arg[0];
#endif
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
      TLS_DEBUG("[pkcmd_pkt_parse res]ret: %ld\n", ntohl(in->ret_code));

      if (subcmd_id == 0 || subcmd_id > APISUBCMDID_TLS_PK_RSA)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n", subcmd_id);
          return -1;
        }
      if (subcmd_id == APISUBCMDID_TLS_PK_INFO_FROM_TYPE)
        {
          FAR mbedtls_pk_info_t *pk_info = (FAR mbedtls_pk_info_t *)arg[1];

          *ret_code = ntohl(in->ret_code);
          if (*ret_code == 0)
            {
              pk_info->id = ntohl(in->u.info_from_typeres.pk_info);
            }
          TLS_DEBUG("[pk_info_from_type res]ret: %ld\n", ntohl(in->ret_code));
        }
      else if (subcmd_id == APISUBCMDID_TLS_PK_WRITE_KEY_PEM)
        {
          size_t req_buf_len = 0;
          FAR unsigned char *buf = (FAR unsigned char *)arg[1];
          FAR size_t *size = (FAR size_t *)arg[2];

          req_buf_len = (*size <= APICMD_PK_WRITE_KEY_PEM_BUF_LEN)
            ? *size : APICMD_PK_WRITE_KEY_PEM_BUF_LEN;

          *ret_code = ntohl(in->ret_code);
          memcpy(buf, in->u.write_key_pemres.buf, req_buf_len);

          TLS_DEBUG("[pk_write_key_pem res]ret: %ld\n", ntohl(in->ret_code));
        }
      else if (subcmd_id == APISUBCMDID_TLS_PK_WRITE_KEY_DER)
        {
          size_t req_buf_len = 0;
          FAR unsigned char *buf = (FAR unsigned char *)arg[1];
          FAR size_t *size = (FAR size_t *)arg[2];

          req_buf_len = (*size <= APICMD_PK_WRITE_KEY_DER_BUF_LEN)
            ? *size : APICMD_PK_WRITE_KEY_DER_BUF_LEN;

          *ret_code = ntohl(in->ret_code);
          if (*ret_code <= 0)
            {
              /* Nothing to do */
            }
          else if ((0 < *ret_code) && (*ret_code <= req_buf_len))
            {
              memcpy(buf, in->u.write_key_derres.buf, *ret_code);
            }
          else
            {
              TLS_ERROR("Unexpected buffer length: %ld\n", *ret_code);
              return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
            }

          TLS_DEBUG("[pk_write_key_der res]ret: %ld\n", ntohl(in->ret_code));
        }
      else if (subcmd_id == APISUBCMDID_TLS_PK_RSA)
        {
          FAR mbedtls_rsa_context *rsa_context =
            (FAR mbedtls_rsa_context *)arg[1];

          *ret_code = ntohl(in->ret_code);
          rsa_context->id = ntohl(in->u.rsares.rsa);

          TLS_DEBUG("[pk_rsa res]ret: %ld\n", ntohl(in->ret_code));
        }
      else
        {
          *ret_code = ntohl(in->ret_code);
        }
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return ret;
}
