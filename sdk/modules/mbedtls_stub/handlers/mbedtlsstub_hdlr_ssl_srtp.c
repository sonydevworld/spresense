/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_ssl_srtp.c
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

#include "include/mbedtlsstub_debug.h"
#include "include/mbedtlsstub_utils.h"
#include "include/mbedtlsstub_hdlr.h"
#include "include/altcom_cmdid.h"
#include "include/apicmd_ssl_export_srtp_keys.h"
#include "include/apicmd_ssl_use_srtp.h"
#include "include/apicmd_ssl_srtp_profile.h"
#include "include/apicmd_ssl_turn.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_sslexportsrtpkeys_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_export_srtp_keys_s *out =
        (FAR struct apicmd_ssl_export_srtp_keys_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      TLS_DEBUG("[ssl_export_srtp_keys]ctx id: %lu\n", ssl->id);

      *altcid = APICMDID_TLS_SSL_EXPORT_SRTP_KEYS;
      size = sizeof(struct apicmd_ssl_export_srtp_keys_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_sslusesrtp_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_use_srtp_s *out =
        (FAR struct apicmd_ssl_use_srtp_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      TLS_DEBUG("[ssl_use_srtp]ctx id: %lu\n", ssl->id);

      *altcid = APICMDID_TLS_SSL_USE_SRTP;
      size = sizeof(struct apicmd_ssl_use_srtp_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_srtpprofile_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_srtp_profile_s *out =
        (FAR struct apicmd_ssl_srtp_profile_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      TLS_DEBUG("[ssl_srtp_profile]ctx id: %lu\n", ssl->id);

      *altcid = APICMDID_TLS_SSL_SRTP_PROFILE;
      size = sizeof(struct apicmd_ssl_srtp_profile_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_sslturn_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR uint16_t *turn_channel = (FAR uint16_t *)arg[1];
  FAR uint32_t *peer_addr = (FAR uint32_t *)arg[2];
  FAR uint16_t *peer_port = (FAR uint16_t *)arg[3];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_turn_s *out =
        (FAR struct apicmd_ssl_turn_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->turn_channel = htons(*turn_channel);
      out->peer_addr = htonl(*peer_addr);
      out->peer_port = htons(*peer_port);

      TLS_DEBUG("[ssl_turn]ctx id: %lu\n", ssl->id);

      *altcid = APICMDID_TLS_SSL_TURN;
      size = sizeof(struct apicmd_ssl_turn_s);
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t mbedtlsstub_sslexportsrtpkeys_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  uint32_t buflen = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR uint16_t *size = (FAR uint16_t*)arg[2];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_export_srtp_keysres_s *in =
        (FAR struct apicmd_ssl_export_srtp_keysres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      buflen = (*size < APICMD_EXPORT_SRTP_KEY_LEN) ?
        *size : APICMD_EXPORT_SRTP_KEY_LEN;
      memcpy(buf, in->key, buflen);

      TLS_DEBUG("[ssl_export_srtp_keys res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_sslusesrtp_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_use_srtpres_s *in =
        (FAR struct apicmd_ssl_use_srtpres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[ssl_use_srtp res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t mbedtlsstub_srtpprofile_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
  int32_t profile = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_srtp_profileres_s *in =
        (FAR struct apicmd_ssl_srtp_profileres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      profile = ntohl(in->profile);

      TLS_DEBUG("[ssl_srtp_profile res]ret: %ld\n", *ret);
      TLS_DEBUG("[ssl_srtp_profile res]profile: %ld\n", profile);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return profile;
}

int32_t mbedtlsstub_sslturn_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_turn_res_s *in =
        (FAR struct apicmd_ssl_turn_res_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      TLS_DEBUG("[ssl_turn res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
