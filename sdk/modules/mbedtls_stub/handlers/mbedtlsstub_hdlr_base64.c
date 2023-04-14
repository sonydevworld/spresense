/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_base64.c
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

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_cipher.h"
#include "include/apicmd_base64_encode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_base64enc_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR size_t *dlen = (FAR size_t *)arg[0];
  FAR const unsigned char *src = (FAR const unsigned char *)arg[1];
  FAR size_t *slen = (FAR size_t *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_base64_encode_s *out =
        (FAR struct apicmd_base64_encode_s *)pktbuf;

      memcpy(out->src, src, *slen);
      out->slen = htonl(*slen);
      out->dlen = htonl(*dlen);

      *altcid = APICMDID_TLS_BASE64_ENCODE;
      size = sizeof(struct apicmd_base64_encode_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmd_s *out =
        (FAR struct apicmd_ciphercmd_s *)pktbuf;

      out->subcmd_id = htonl(APISUBCMDID_TLS_BASE64_ENCODE);
      memcpy(out->u.base64_encode.src, src, *slen);
      out->u.base64_encode.slen = htonl(*slen);
      out->u.base64_encode.dlen = htonl(*dlen);

      *altcid = APICMDID_TLS_CIPHER_CMD_V4;
      size = sizeof(struct apicmd_ciphercmd_s);
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t mbedtlsstub_base64enc_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  uint32_t out_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR char *buf = (FAR char *)arg[1];
  FAR size_t *olen = (FAR size_t *)arg[2];
  FAR size_t *dlen = (FAR size_t *)arg[3];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_base64_encoderes_s *in =
        (FAR struct apicmd_base64_encoderes_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      out_len = ntohl(in->olen);
      if (out_len <= *dlen)
        {
          memcpy(buf, in->dst, out_len);
          *olen = out_len;
        }
      else
        {
          TLS_ERROR("Unexpected output length: %lu\n", out_len);
          *olen = 0;
          return -1;
        }
      TLS_DEBUG("[base64_encode res]ret: %ld\n", *ret);
      TLS_DEBUG("[base64_encode res]length: %lu\n", out_len);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_BASE64_ENCODE)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return -1;
        }

      out_len = ntohl(in->u.base64_encoderes.olen);
      if (out_len <= *dlen)
        {
          memcpy(buf, in->u.base64_encoderes.dst, out_len);
          *olen = out_len;
        }
      else
        {
          TLS_ERROR("Unexpected output length: %lu\n", out_len);
          *olen = 0;
          return -1;
        }
      TLS_DEBUG("[base64_encode res]ret: %ld\n", *ret);
      TLS_DEBUG("[base64_encode res]length: %lu\n", out_len);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
