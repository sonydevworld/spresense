/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_x509_crt.c
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

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_x509_crt.h"
#include "include/apicmd_x509_crt_init.h"
#include "include/apicmd_x509_crt_free.h"
#include "include/apicmd_x509_crt_parse_file.h"
#include "include/apicmd_x509_crt_parse_der.h"
#include "include/apicmd_x509_crt_parse.h"
#include "include/apicmd_x509_crt_info.h"
#include "include/apicmd_x509_crt_verify_info.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_x509crtinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_X509_CRT_CTX);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_init_s *out =
        (FAR struct apicmd_x509_crt_init_s *)pktbuf;

      out->crt = htonl(*id);

      *altcid = APICMDID_TLS_X509_CRT_INIT;
      size = sizeof(struct apicmd_x509_crt_init_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->crt = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_X509_CRT_INIT);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[x509_crt_init]ctx id: %ld\n", *id);

  return size;
}

int32_t mbedtlsstub_x509crtfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_crt *crt = (FAR mbedtls_x509_crt *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_free_s *out =
        (FAR struct apicmd_x509_crt_free_s *)pktbuf;

      out->crt = htonl(crt->id);

      *altcid = APICMDID_TLS_X509_CRT_FREE;
      size = sizeof(struct apicmd_x509_crt_free_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->crt = htonl(crt->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_X509_CRT_FREE);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[x509_crt_free]ctx id: %lu\n", crt->id);

  return size;
}

int32_t mbedtlsstub_x509crtparsefile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_crt *chain = (FAR mbedtls_x509_crt *)arg[0];
  FAR const char *path = (FAR const char *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_file_s *out =
        (FAR struct apicmd_x509_crt_parse_file_s *)pktbuf;

      out->chain = htonl(chain->id);
      memset(out->path, '\0', APICMD_X509_CRT_PARSE_FILE_PATH_LEN);
      if (path != NULL)
        {
          strncpy((char *)out->path, path,
            APICMD_X509_CRT_PARSE_FILE_PATH_LEN-1);
        }

      TLS_DEBUG("[x509_crt_parse_file]ctx id: %lu\n", chain->id);
      TLS_DEBUG("[x509_crt_parse_file]path: %s\n", out->path);

      *altcid = APICMDID_TLS_X509_CRT_PARSE_FILE;
      size = sizeof(struct apicmd_x509_crt_parse_file_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->crt = htonl(chain->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_X509_CRT_PARSE_FILE);
      memset(out->u.parse_file.path,
        '\0', APICMD_X509_CRT_PARSE_FILE_PATH_LEN);
      if (path != NULL)
        {
          strncpy((char *)out->u.parse_file.path, path,
            APICMD_X509_CRT_PARSE_FILE_PATH_LEN-1);
        }

      TLS_DEBUG("[x509_crt_parse_file]ctx id: %lu\n", chain->id);
      TLS_DEBUG("[x509_crt_parse_file]path: %s\n",
        out->u.parse_file.path);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t mbedtlsstub_x509crtparseder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_x509_crt *chain = (FAR mbedtls_x509_crt *)arg[0];
  FAR const unsigned char *buf = (FAR const unsigned char *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_der_s *out =
        (FAR struct apicmd_x509_crt_parse_der_s *)pktbuf;

      out->chain = htonl(chain->id);
      *buflen = (*buflen <= APICMD_X509_CRT_PARSE_DER_LEN)
        ? *buflen : APICMD_X509_CRT_PARSE_DER_LEN;
      if (buf != NULL)
        {
          memcpy(out->buf, buf, *buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->buflen = htonl(*buflen);

      *altcid = APICMDID_TLS_X509_CRT_PARSE_DER;
      size = sizeof(struct apicmd_x509_crt_parse_der_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->crt = htonl(chain->id);
      out->subcmd_id =
        htonl(APISUBCMDID_TLS_X509_CRT_PARSE_DER);
      *buflen = (*buflen <= APICMD_X509_CRT_PARSE_DER_LEN)
        ? *buflen : APICMD_X509_CRT_PARSE_DER_LEN;
      if (buf != NULL)
        {
          memcpy(out->u.parse_der.buf, buf, *buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->u.parse_der.buflen = htonl(*buflen);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[x509_crt_parse_der]ctx id: %lu\n", chain->id);
  TLS_DEBUG("[x509_crt_parse_der]buflen: %d\n", *buflen);

  return size;
}

int32_t mbedtlsstub_x509crtparse_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  uint32_t lbuflen = 0;
  FAR mbedtls_x509_crt *chain = (FAR mbedtls_x509_crt *)arg[0];
  FAR const unsigned char *buf = (FAR const unsigned char *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_s *out =
        (FAR struct apicmd_x509_crt_parse_s *)pktbuf;

      out->chain = htonl(chain->id);

      *altcid = APICMDID_TLS_X509_CRT_PARSE;
      size = sizeof(struct apicmd_x509_crt_parse_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->crt = htonl(chain->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_X509_CRT_PARSE);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  if (*buflen <= APICMD_X509_CRT_PARSE_BUF_LEN)
    {
      lbuflen = *buflen;
    }
  else
    {
      return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_s *out =
        (FAR struct apicmd_x509_crt_parse_s *)pktbuf;

      memset(out->buf, '\0', APICMD_X509_CRT_PARSE_BUF_LEN);
      if (buf != NULL)
        {
          memcpy(out->buf, buf, lbuflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->buflen = htonl(lbuflen);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      memset(out->u.parse.buf, '\0', APICMD_X509_CRT_PARSE_BUF_LEN);
      if (buf != NULL)
        {
          memcpy(out->u.parse.buf, buf, lbuflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->u.parse.buflen = htonl(lbuflen);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[x509_crt_parse]ctx id: %lu\n", chain->id);
  TLS_DEBUG("[x509_crt_parse]buflen: %lu\n", lbuflen);

  return size;
}

int32_t mbedtlsstub_x509crtinfo_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  uint32_t buflen = 0;
  FAR size_t *size = (FAR size_t *)arg[0];
  FAR const char *prefix = (FAR const char *)arg[1];
  FAR const mbedtls_x509_crt *crt = (FAR const mbedtls_x509_crt *)arg[2];

  FAR struct apicmd_x509_crt_info_s *out =
    (FAR struct apicmd_x509_crt_info_s *)pktbuf;

  out->crt = htonl(crt->id);
  buflen = (*size <= APICMD_X509_CRT_INFO_RET_BUF_LEN)
    ? *size : APICMD_X509_CRT_INFO_RET_BUF_LEN;
  out->size = htonl(buflen);
  memset(out->prefix, '\0', APICMD_X509_CRT_INFO_PREFIX_LEN);
  if (prefix != NULL)
    {
      strncpy((char*) out->prefix, prefix,
        APICMD_X509_CRT_INFO_PREFIX_LEN-1);
    }
  ret_size = sizeof(struct apicmd_x509_crt_info_s);

  TLS_DEBUG("[x509_crt_info]ctx id: %lu\n", crt->id);
  TLS_DEBUG("[x509_crt_info]size: %d\n", *size);
  TLS_DEBUG("[x509_crt_info]prefix: %s\n", out->prefix);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_TLS_X509_CRT_INFO;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_TLS_X509_CRT_INFO_V4;
    }
  else
#endif
    {
      ret_size = -ENOSYS;
    }

  return ret_size;
}

int32_t mbedtlsstub_x509crtvrfyinfo_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
  uint32_t buflen = 0;
  FAR size_t *size = (FAR size_t *)arg[0];
  FAR const char *prefix = (FAR const char *)arg[1];
  FAR uint32_t *flags = (FAR uint32_t *)arg[2];

  buflen = (*size <= APICMD_X509_CRT_VERIFY_INFO_RET_BUF_LEN)
    ? *size : APICMD_X509_CRT_VERIFY_INFO_RET_BUF_LEN;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_verify_info_s *out =
        (FAR struct apicmd_x509_crt_verify_info_s *)pktbuf;

      out->size = htonl(buflen);
      memset(out->prefix, '\0', APICMD_X509_CRT_VERIFY_INFO_PREFIX_LEN);
      if (prefix != NULL)
        {
          strncpy((char *)out->prefix, prefix,
            APICMD_X509_CRT_VERIFY_INFO_PREFIX_LEN-1);
        }
      out->flags = htonl(*flags);
  
      TLS_DEBUG("[x509_crt_verify_info]size: %d\n", *size);
      TLS_DEBUG("[x509_crt_verify_info]prefix: %s\n",
        out->prefix);
      TLS_DEBUG("[x509_crt_verify_info]flags: %ld\n", *flags);

      *altcid = APICMDID_TLS_X509_CRT_VERIFY_INFO;
      ret_size = sizeof(struct apicmd_x509_crt_verify_info_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_x509_crtcmd_s *out =
        (FAR struct apicmd_x509_crtcmd_s *)pktbuf;

      out->subcmd_id = htonl(APISUBCMDID_TLS_X509_CRT_VERIFY_INFO);
      out->u.verify_info.size = htonl(buflen);
      memset(out->u.verify_info.prefix,
        '\0', APICMD_X509_CRT_VERIFY_INFO_PREFIX_LEN);
      if (prefix != NULL)
        {
          strncpy((char *)out->u.verify_info.prefix, prefix,
            APICMD_X509_CRT_VERIFY_INFO_PREFIX_LEN-1);
        }
      out->u.verify_info.flags = htonl(*flags);

      TLS_DEBUG("[x509_crt_verify_info]size: %d\n", *size);
      TLS_DEBUG("[x509_crt_verify_info]prefix: %s\n",
        out->u.verify_info.prefix);
      TLS_DEBUG("[x509_crt_verify_info]flags: %ld\n", *flags);

      *altcid = APICMDID_TLS_X509_CRT_CMD_V4;
      ret_size = sizeof(struct apicmd_x509_crtcmd_s);
    }
  else
#endif
    {
      ret_size = -ENOSYS;
    }

  return ret_size;
}

int32_t mbedtlsstub_x509crtinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_initres_s *in =
        (FAR struct apicmd_x509_crt_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[x509_crt_init res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_x509crtfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_freeres_s *in =
        (FAR struct apicmd_x509_crt_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[x509_crt_free res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_x509crtparsefile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_fileres_s *in =
        (FAR struct apicmd_x509_crt_parse_fileres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[x509_crt_parse_file res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_x509crtparseder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parse_derres_s *in =
        (FAR struct apicmd_x509_crt_parse_derres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[x509_crt_parse_der res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_x509crtparse_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_parseres_s *in =
        (FAR struct apicmd_x509_crt_parseres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[x509_crt_parse res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_x509crtinfo_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  uint32_t buflen = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];
  FAR struct apicmd_x509_crt_infores_s *in =
    (FAR struct apicmd_x509_crt_infores_s *)pktbuf;

  buflen = (*size <= APICMD_X509_CRT_INFO_RET_BUF_LEN)
    ? *size : APICMD_X509_CRT_INFO_RET_BUF_LEN;

  *ret = ntohl(in->ret_code);
  *ret = (*ret <= buflen) ? *ret : buflen;
  memcpy(buf, in->buf, *ret);

  TLS_DEBUG("[x509_crt_info res]ret: %ld\n", *ret);

  return 0;
}

int32_t mbedtlsstub_x509crtvrfyinfo_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  uint32_t buflen = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *size = (FAR size_t *)arg[2];

  buflen = (*size <= APICMD_X509_CRT_VERIFY_INFO_RET_BUF_LEN)
    ? *size : APICMD_X509_CRT_VERIFY_INFO_RET_BUF_LEN;
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_crt_verify_infores_s *in =
        (FAR struct apicmd_x509_crt_verify_infores_s *)pktbuf;

      *ret = ntohl( in->ret_code);
      *ret = (*ret <= buflen) ? *ret : buflen;
      memcpy(buf, in->buf, *ret);
      TLS_DEBUG("[x509_crt_verify_info res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_x509crtcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
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
      FAR struct apicmd_x509_crtcmdres_s *in =
        (FAR struct apicmd_x509_crtcmdres_s *)pktbuf;

      uint32_t subcmd_id = ntohl(in->subcmd_id);
      TLS_DEBUG("[x509crtcmd_pkt_parse res]ret: %ld\n", ntohl(in->ret_code));

      if (subcmd_id == 0 || subcmd_id > APISUBCMDID_TLS_X509_CRT_VERIFY_INFO)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(subcmd_id));
          return -1;
        }

      if (subcmd_id == APISUBCMDID_TLS_X509_CRT_VERIFY_INFO)
        {
          uint32_t buflen = 0;
          FAR unsigned char *buf = (FAR unsigned char *)arg[1];
          FAR size_t *size = (FAR size_t *)arg[2];

          buflen = (*size <= APICMD_X509_CRT_INFO_RET_BUF_LEN)
            ? *size : APICMD_X509_CRT_INFO_RET_BUF_LEN;

          *ret_code = ntohl(in->ret_code);
          *ret_code = (*ret_code <= buflen) ? *ret_code : buflen;
          memcpy(buf, in->u.verify_infores.buf, *ret_code);

          TLS_DEBUG("[x509_crt_verify_info res]ret: %lu\n", *ret_code);
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
