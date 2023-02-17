/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_sha1.c
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
#include "include/apicmd_sha1.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_sha1_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const unsigned char *input = (FAR const unsigned char *)arg[0];
  FAR size_t *ilen = (FAR size_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_sha1_s *out =
        (FAR struct apicmd_sha1_s *)pktbuf;

      memcpy(out->input, input, *ilen);
      out->ilen = htonl(*ilen);

      *altcid = APICMDID_TLS_SHA1;
      size = sizeof(struct apicmd_sha1_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmd_s *out =
        (FAR struct apicmd_ciphercmd_s *)pktbuf;

      out->subcmd_id = htonl(APISUBCMDID_TLS_SHA1);
      memcpy(out->u.sha1.input, input, *ilen);
      out->u.sha1.ilen = htonl(*ilen);

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

int32_t mbedtlsstub_sha1_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *output = (FAR unsigned char *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_sha1res_s *in =
        (FAR struct apicmd_sha1res_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      memcpy(output, in->output, APICMD_SHA1_OUTPUT_LEN);
      TLS_DEBUG("[sha1 res]ret: %ld\n", *ret);

    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_ciphercmdres_s *in =
        (FAR struct apicmd_ciphercmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      if (ntohl(in->subcmd_id) != APISUBCMDID_TLS_SHA1)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",
            ntohl(in->subcmd_id));
          return -1;
        }

      memcpy(output, in->u.sha1res.output, APICMD_SHA1_OUTPUT_LEN);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
