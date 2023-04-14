/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_x509_dn.c
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
#include <mbedtls/x509_csr.h>

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_x509_dn_gets_crt.h"
#include "include/apicmd_x509_dn_gets_csr.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_x509dngetscrt_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  uint32_t buflen = 0;
  FAR char *buf = (FAR char *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];
  FAR const mbedtls_x509_crt *crt = (FAR const mbedtls_x509_crt *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_dn_gets_crt_s *out =
        (FAR struct apicmd_x509_dn_gets_crt_s *)pktbuf;

      out->crt = htonl(crt->id);
      if (*size <= APICMD_X509_DN_GETS_CRT_BUF_LEN)
        {
          buflen = *size;
        }
      else
            {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      memset(out->buf, '\0', APICMD_X509_DN_GETS_CRT_BUF_LEN);
      if (buf != NULL)
        {
          memcpy(out->buf, buf, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->size = htonl(buflen);

      TLS_DEBUG("[x509_dn_gets_crt]ctx id: %lu\n", crt->id);
      TLS_DEBUG("[x509_dn_gets_crt]size: %lu\n", buflen);

      *altcid = APICMDID_TLS_X509_DN_GETS_CRT;
      ret_size = sizeof(struct apicmd_x509_dn_gets_crt_s);
    }
  else
#endif
    {
      ret_size = -ENOTSUP;
    }

  return ret_size;
}

int32_t mbedtlsstub_x509dngetscsr_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t ret_size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  uint32_t buflen = 0;
  FAR char *buf = (FAR char *)arg[0];
  FAR size_t *size = (FAR size_t *)arg[1];
  FAR mbedtls_x509_csr *csr = (FAR mbedtls_x509_csr *)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_dn_gets_csr_s *out =
        (FAR struct apicmd_x509_dn_gets_csr_s *)pktbuf;

      out->csr = htonl(csr->id);
      if (*size <= APICMD_X509_DN_GETS_CSR_BUF_LEN)
        {
          buflen = *size;
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      memset(out->buf, '\0', APICMD_X509_DN_GETS_CSR_BUF_LEN);
      if (buf != NULL)
        {
          memcpy(out->buf, buf, buflen);
        }
      else
        {
          return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
        }
      out->size = htonl(buflen);

      TLS_DEBUG("[x509_dn_gets_csr]ctx id: %lu\n", csr->id);
      TLS_DEBUG("[x509_dn_gets_csr]size: %lu\n", buflen);

      *altcid = APICMDID_TLS_X509_DN_GETS_CSR;
      ret_size = sizeof(struct apicmd_x509_dn_gets_csr_s);
    }
  else
#endif
    {
      ret_size = -ENOTSUP;
    }

  return ret_size;
}

int32_t mbedtlsstub_x509dngetscrt_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_dn_gets_crtres_s *in =
        (FAR struct apicmd_x509_dn_gets_crtres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_dn_gets_crt res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_x509dngetscsr_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_x509_dn_gets_csrres_s *in =
        (FAR struct apicmd_x509_dn_gets_csrres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[x509_dn_gets_csr res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
