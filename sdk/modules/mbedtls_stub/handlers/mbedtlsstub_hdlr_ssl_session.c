/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_ssl_session.c
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
#include "include/apicmd_session.h"
#include "include/apicmd_session_init.h"
#include "include/apicmd_session_free.h"
#include "include/apicmd_session_get.h"
#include "include/apicmd_session_set.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_ssesioninit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_SESSION_CTX);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_init_s *out =
        (FAR struct apicmd_session_init_s *)pktbuf;

      out->session = htonl(*id);

      *altcid = APICMDID_TLS_SESSION_INIT;
      size = sizeof(struct apicmd_session_init_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmd_s *out =
        (FAR struct apicmd_sessioncmd_s *)pktbuf;

      out->session = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_INIT);

      *altcid = APICMDID_TLS_SESSION_CMD_V4;
      size = sizeof(struct apicmd_sessioncmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[session_init]session id: %ld\n", *id);

  return size;
}

int32_t mbedtlsstub_sessionfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_session *session = (FAR mbedtls_ssl_session *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_free_s *out =
        (FAR struct apicmd_session_free_s *)pktbuf;

      out->session = htonl(session->id);

      *altcid = APICMDID_TLS_SESSION_FREE;
      size = sizeof(struct apicmd_session_free_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmd_s *out =
        (FAR struct apicmd_sessioncmd_s *)pktbuf;

      out->session = htonl(session->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_FREE);

      *altcid = APICMDID_TLS_SESSION_CMD_V4;
      size = sizeof(struct apicmd_sessioncmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[session_free]session id: %lu\n", session->id);

  return size;
}

int32_t mbedtlsstub_sessionget_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];
  FAR mbedtls_ssl_session *session = (FAR mbedtls_ssl_session *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_get_s *out =
        (FAR struct apicmd_session_get_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->dst = htonl(session->id);

      *altcid = APICMDID_TLS_SESSION_GET;
      size = sizeof(struct apicmd_session_get_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmd_s *out =
        (FAR struct apicmd_sessioncmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->session = htonl(session->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_GET);

      *altcid = APICMDID_TLS_SESSION_CMD_V4;
      size = sizeof(struct apicmd_sessioncmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[session_get]ssl id: %lu\n", ssl->id);
  TLS_DEBUG("[session_get]session id: %lu\n", session->id);

  return size;
}

int32_t mbedtlsstub_sessionset_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR const mbedtls_ssl_session *session =
    (FAR const mbedtls_ssl_session *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_set_s *out =
        (FAR struct apicmd_session_set_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->session = htonl(session->id);

      *altcid = APICMDID_TLS_SESSION_SET;
      size = sizeof(struct apicmd_session_set_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmd_s *out =
        (FAR struct apicmd_sessioncmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->session = htonl(session->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_SET);

      *altcid = APICMDID_TLS_SESSION_CMD_V4;
      size = sizeof(struct apicmd_sessioncmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[session_set]ssl id: %lu\n", ssl->id);
  TLS_DEBUG("[session_set]session id: %lu\n", session->id);

  return size;
}

int32_t mbedtlsstub_sessionreset_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sessioncmd_s *out =
        (FAR struct apicmd_sessioncmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_RESET);

      *altcid = APICMDID_TLS_SESSION_CMD_V4;
      size = sizeof(struct apicmd_sessioncmd_s);
    }
  else
#endif
    {
      return -ENOTSUP;
    }

  TLS_DEBUG("[session_reset]ssl id: %lu\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_ssesioninit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_initres_s *in =
        (FAR struct apicmd_session_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[session_init res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sessionfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_freeres_s *in =
        (FAR struct apicmd_session_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[session_free res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sessionget_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_getres_s *in =
        (FAR struct apicmd_session_getres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[session_get res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sessionset_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_session_setres_s *in =
        (FAR struct apicmd_session_setres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[session_set res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sessioncmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
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
      FAR struct apicmd_sessioncmdres_s *in =
        (FAR struct apicmd_sessioncmdres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[sessioncmd_pkt_parse res]ret: %ld\n", *ret);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
