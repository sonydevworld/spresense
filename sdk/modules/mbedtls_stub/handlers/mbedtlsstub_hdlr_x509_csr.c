/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_x509_csr.c
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
#include <mbedtls/x509_csr.h>

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_x509_csr_init.h"
#include "include/apicmd_x509_csr_free.h"
#include "include/apicmd_x509_csr_parse_file.h"
#include "include/apicmd_x509_csr_parse_der.h"
#include "include/apicmd_x509_csr_parse.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_x509csrinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_X509_CSR_CTX);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_init_s *out =
        (FAR struct apicmd_x509_csr_init_s *)pktbuf;

      out->csr = htonl(*id);

      TLS_DEBUG("[x509_csr_init]csr id: %ld\n", *id);

      *altcid = APICMDID_TLS_X509_CSR_INIT;
      size = sizeof(struct apicmd_x509_csr_init_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509csrfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509_csr *csr = (FAR mbedtls_x509_csr *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_free_s *out =
        (FAR struct apicmd_x509_csr_free_s *)pktbuf;

      out->csr = htonl(csr->id);

      TLS_DEBUG("[x509_csr_free]ctx id: %lu\n", csr->id);

      *altcid = APICMDID_TLS_X509_CSR_FREE;
      size = sizeof(struct apicmd_x509_csr_free_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509csrparsefile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509_csr *csr = (FAR mbedtls_x509_csr *)arg[0];
  FAR const char *path = (FAR const char *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parse_file_s *out =
        (FAR struct apicmd_x509_csr_parse_file_s *)pktbuf;

      out->csr = htonl(csr->id);
      memset(out->path, '\0', APICMD_X509_CSR_PARSE_FILE_PATH_LEN);
      if (path != NULL)
        {
          strncpy((char*) out->path, path,
            APICMD_X509_CSR_PARSE_FILE_PATH_LEN-1);
        }

      TLS_DEBUG("[x509_csr_parse_file]ctx id: %lu\n", csr->id);
      TLS_DEBUG("[x509_csr_parse_file]path: %s\n", path);

      *altcid = APICMDID_TLS_X509_CSR_PARSE_FILE;
      size = sizeof(struct apicmd_x509_csr_parse_file_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509csrparseder_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509_csr *csr = (FAR mbedtls_x509_csr *)arg[0];
  FAR const unsigned char *buf = (FAR const unsigned char *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parse_der_s *out =
        (FAR struct apicmd_x509_csr_parse_der_s *)pktbuf;

      out->csr = htonl(csr->id);
      *buflen = (*buflen <= APICMD_X509_CSR_PARSE_DER_LEN)
        ? *buflen : APICMD_X509_CSR_PARSE_DER_LEN;
      if (buf != NULL)
        {
          memcpy(out->buf, buf, *buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->buflen = htonl(*buflen);

      TLS_DEBUG("[x509_csr_parse_der]ctx id: %lu\n", csr->id);
      TLS_DEBUG("[x509_csr_parse_der]buflen: %d\n", *buflen);

      *altcid = APICMDID_TLS_X509_CSR_PARSE_DER;
      size = sizeof(struct apicmd_x509_csr_parse_der_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_x509csrparse_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_x509_csr *csr = (FAR mbedtls_x509_csr *)arg[0];
  FAR const unsigned char *buf = (FAR const unsigned char *)arg[1];
  FAR size_t *buflen = (FAR size_t *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parse_s *out =
        (FAR struct apicmd_x509_csr_parse_s *)pktbuf;

      out->csr = htonl(csr->id);
      if (*buflen > APICMD_X509_CSR_PARSE_BUF_LEN)
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      memset(out->buf, '\0', APICMD_X509_CSR_PARSE_BUF_LEN);
      if (buf != NULL)
        {
          memcpy(out->buf, buf, *buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->buflen = htonl(*buflen);

      TLS_DEBUG("[x509_csr_parse]ctx id: %lu\n", csr->id);
      TLS_DEBUG("[x509_csr_parse]buflen: %d\n", *buflen);

      *altcid = APICMDID_TLS_X509_CSR_PARSE;
      size = sizeof(struct apicmd_x509_csr_parse_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}


int32_t mbedtlsstub_x509csrinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_initres_s *in =
        (FAR struct apicmd_x509_csr_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_csr_init res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509csrfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_freeres_s *in =
        (FAR struct apicmd_x509_csr_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_csr_free res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509csrparsefile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parse_fileres_s *in =
        (FAR struct apicmd_x509_csr_parse_fileres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_csr_parse_file res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509csrparseder_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parse_derres_s *in =
        (FAR struct apicmd_x509_csr_parse_derres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_csr_parse_der res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509csrparse_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_csr_parseres_s *in =
        (FAR struct apicmd_x509_csr_parseres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_csr_parse res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
