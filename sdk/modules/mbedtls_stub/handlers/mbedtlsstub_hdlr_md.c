/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_md.c
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
#include <mbedtls/md_internal.h>
#include <mbedtls/x509_crt.h>

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_cipher.h"
#include "include/apicmd_md_info_from_type.h"
#include "include/apicmd_md_get_size.h"
#include "include/apicmd_md.h"
#include "include/apicmd_md_digest.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_mdinfofromtype_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_md_type_t *md_type = (FAR mbedtls_md_type_t *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_info_from_type_s *out =
        (FAR struct apicmd_md_info_from_type_s *)pktbuf;

      out->md_type = htonl(*md_type);

      *altcid = APICMDID_TLS_MD_INFO_FROM_TYPE;
      size = sizeof(struct apicmd_md_info_from_type_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmd_s *out =
        (FAR struct apicmd_ciphercmd_s *)pktbuf;

      out->subcmd_id = htonl(APISUBCMDID_TLS_MD_INFO_FROM_TYPE);
      out->u.md_info_from_type.md_type = htonl(*md_type);

      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
      size = sizeof(struct apicmd_ciphercmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[md_info_from_type]md_type: %d\n", *md_type);

  return size;
}

int32_t mbedtlsstub_mdgetsize_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_md_info_t *md_info = (FAR mbedtls_md_info_t *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_get_size_s *out =
        (FAR struct apicmd_md_get_size_s *)pktbuf;

      out->md_info = htonl(md_info->id);

      *altcid = APICMDID_TLS_MD_GET_SIZE;
      size = sizeof(struct apicmd_md_get_size_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmd_s *out =
        (FAR struct apicmd_ciphercmd_s *)pktbuf;

      out->md_info = htonl(md_info->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_MD_GET_SIZE);

      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
      size = sizeof(struct apicmd_ciphercmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[md_get_size]md_info id: %lu\n", md_info->id);

  return size;
}

int32_t mbedtlsstub_md_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_md_info_t *md_info =
    (FAR const mbedtls_md_info_t *)arg[0];
  FAR const unsigned char *input = (FAR const unsigned char *)arg[1];
  FAR size_t *ilen = (FAR size_t *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_s *out =
        (FAR struct apicmd_md_s *)pktbuf;

      out->md_info = htonl(md_info->id);

      if (*ilen <= APICMD_MD_INPUT_LEN)
        {
          memset(out->input, 0, APICMD_MD_INPUT_LEN);
          memcpy(out->input, input, *ilen);
        }
      else
        {
          return MBEDTLS_ERR_MD_BAD_INPUT_DATA;
        }

      out->ilen = htonl(*ilen);

      *altcid = APICMDID_TLS_MD;
      size = sizeof(struct apicmd_md_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmd_s *out =
        (FAR struct apicmd_ciphercmd_s *)pktbuf;

      out->md_info = htonl(md_info->id);
      out->subcmd_id =
        htonl(APISUBCMDID_TLS_MD);
      if (*ilen <= APICMD_MD_INPUT_LEN)
        {
          memset(out->u.md.input, 0, APICMD_MD_INPUT_LEN);
          memcpy(out->u.md.input, input, *ilen);
        }
      else
        {
          return MBEDTLS_ERR_MD_BAD_INPUT_DATA;
        }

      out->u.md.ilen = htonl(*ilen);

      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
      size = sizeof(struct apicmd_ciphercmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[md]md_info id: %lu\n", md_info->id);

  return size;
}

int32_t mbedtlsstub_mddigest_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_md_info_t *md_info =
    (FAR const mbedtls_md_info_t *)arg[0];
  FAR mbedtls_x509_crt *chain = (FAR mbedtls_x509_crt *)arg[1];

  FAR struct apicmd_md_digest_s *out =
    (FAR struct apicmd_md_digest_s *)pktbuf;

  out->md_info = htonl(md_info->id);
  out->chain = htonl(chain->id);

  size = sizeof(struct apicmd_md_digest_s);

  TLS_DEBUG("[md_digest]md_info id: %lu\n", md_info->id);
  TLS_DEBUG("[md_digest]chain id: %lu\n", chain->id);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_TLS_MD_DIGEST;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}


int32_t mbedtlsstub_mdinfofromtype_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR mbedtls_md_info_t *md_info = (FAR mbedtls_md_info_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_info_from_typeres_s *in =
        (FAR struct apicmd_md_info_from_typeres_s *)pktbuf;

      *ret = ntohl(in->md_info);
      md_info->id = *ret;
      TLS_DEBUG("[md_info_from_type res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_MD_INFO_FROM_TYPE)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return -1;
        }

      *ret = ntohl(in->u.md_info_from_typeres.md_info);
      md_info->id = *ret;
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_mdgetsize_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR uint8_t *md_size = (FAR uint8_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_get_sizeres_s *in =
        (FAR struct apicmd_md_get_sizeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      *md_size = in->md_size;

      TLS_DEBUG("[md_get_size res]ret: %ld\n", *ret);
      TLS_DEBUG("[md_get_size res]md_size: %d\n", (int) in->md_size);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_MD_GET_SIZE)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return -1;
        }

      *md_size = in->u.md_get_sizeres.md_size;

      TLS_DEBUG("[md_get_size res]ret: %ld\n", *ret);
      TLS_DEBUG("[md_get_size res]md_size: %d\n",
        (int)in->u.md_get_sizeres.md_size);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_md_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *output = (FAR unsigned char *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_mdres_s *in =
        (FAR struct apicmd_mdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      memcpy(output, in->output, APICMD_MD_OUTPUT_LEN);
      TLS_DEBUG("[md res]ret: %ld\n", *ret);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_MD)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return *ret;
        }

      memcpy(output, in->u.mdres.output, APICMD_MD_OUTPUT_LEN_V4);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_mddigest_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_md_digestres_s *in =
        (FAR struct apicmd_md_digestres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      memcpy(buf, in->output, APICMD_MD_DIGEST_OUTPUT_LEN);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      memcpy(buf, in->u.md_digestres.output, APICMD_MD_DIGEST_OUTPUT_LEN);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[md_digest res]ret: %ld\n", *ret);

  return 0;
}
