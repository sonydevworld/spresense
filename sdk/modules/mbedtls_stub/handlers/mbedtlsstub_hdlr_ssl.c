/****************************************************************************
 * modules/mbedtls_stub/handlers/mbedtlsstub_hdlr_ssl.c
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
#include "include/apicmd_ssl.h"
#include "include/apicmd_ssl_init.h"
#include "include/apicmd_ssl_free.h"
#include "include/apicmd_ssl_setup.h"
#include "include/apicmd_ssl_hostname.h"
#include "include/apicmd_ssl_bio.h"
#include "include/apicmd_ssl_handshake.h"
#include "include/apicmd_ssl_write.h"
#include "include/apicmd_ssl_read.h"
#include "include/apicmd_ssl_close_notify.h"
#include "include/apicmd_ssl_version.h"
#include "include/apicmd_ssl_ciphersuite.h"
#include "include/apicmd_ssl_ciphersuite_id.h"
#include "include/apicmd_ssl_record_expansion.h"
#include "include/apicmd_ssl_verify_result.h"
#include "include/apicmd_ssl_timer_cb.h"
#include "include/apicmd_ssl_peer_cert.h"
#include "include/apicmd_ssl_bytes_avail.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SSL_CIPHER_STR_BUF     64

#define SSL_V_3_0              "SSLv3.0"
#define TLS_V_1_0              "TLSv1.0"
#define TLS_V_1_1              "TLSv1.1"
#define TLS_V_1_2              "TLSv1.2"
#define TLS_UNKNOWN            "unknown"

#define DTLS_V_1_0             "DTLSv1.0"
#define DTLS_V_1_2             "DTLSv1.2"
#define DTLS_UNKNOWN           "unknown (DTLS)"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char* ssl_tls_ver_tbl[] =
{
  SSL_V_3_0,
  TLS_V_1_0,
  TLS_V_1_1,
  TLS_V_1_2,
  DTLS_V_1_0,
  DTLS_V_1_2,
  TLS_UNKNOWN,
  DTLS_UNKNOWN
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *get_ssl_tls_version(const int8_t *ver_name)
{
  int i = 0;
  int size = sizeof(ssl_tls_ver_tbl) / sizeof(ssl_tls_ver_tbl[0]);

  for (i = 0; i < size; i++)
    {
      if (strcmp((const char*) ver_name, ssl_tls_ver_tbl[i]) == 0)
        {
          return ssl_tls_ver_tbl[i];
        }
    }

  return TLS_UNKNOWN;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t mbedtlsstub_sslinit_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *id = (FAR int32_t *)arg[0];

  *id = mbedtlsstub_get_mbedtls_ctx_id(MBEDTLSSTUB_SSL_CTX);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_init_s *out =
        (FAR struct apicmd_ssl_init_s *)pktbuf;

      out->ssl = htonl(*id);

      *altcid = APICMDID_TLS_SSL_INIT;
      size = sizeof(struct apicmd_ssl_init_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(*id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_INIT);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_init]ctx id: %ld\n", *id);

  return size;
}

int32_t mbedtlsstub_sslfree_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_free_s *out =
        (FAR struct apicmd_ssl_free_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_FREE;
      size = sizeof(FAR struct apicmd_ssl_free_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
          (struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_FREE);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_free]ctx id: %ld\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_sslsetup_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR mbedtls_ssl_config *conf = (FAR mbedtls_ssl_config *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_setup_s *out =
        (FAR struct apicmd_ssl_setup_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->conf = htonl(conf->id);

      *altcid = APICMDID_TLS_SSL_SETUP;
      size = sizeof(struct apicmd_ssl_setup_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_SETUP);
      out->u.setup.conf = htonl(conf->id);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_setup]ssl id: %ld\n", ssl->id);
  TLS_DEBUG("[ssl_setup]conf id: %ld\n", conf->id);

  return size;
}

int32_t mbedtlsstub_sslhostname_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR const char *hostname = (FAR const char *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_hostname_s *out =
        (FAR struct apicmd_ssl_hostname_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      memset(out->hostname, '\0', APICMD_SSL_HOSTNAME_LEN);
      if (hostname != NULL)
        {
          size_t hostname_len = strnlen(hostname,
            APICMD_SSL_HOSTNAME_LEN-1);
          strncpy((char*)out->hostname, hostname, hostname_len);
        }

      TLS_DEBUG("[ssl_hostname]id: %ld\n", ssl->id);
      TLS_DEBUG("[ssl_hostname]hostname: %s\n", out->hostname);

      *altcid = APICMDID_TLS_SSL_HOSTNAME;
      size = sizeof(struct apicmd_ssl_hostname_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_HOSTNAME);
      memset(out->u.hostname.hostname, '\0', APICMD_SSL_HOSTNAME_LEN);
      if (hostname != NULL)
        {
          size_t hostname_len = strnlen(hostname, APICMD_SSL_HOSTNAME_LEN-1);
          strncpy((char *)out->u.hostname.hostname, hostname, hostname_len);
        }

      TLS_DEBUG("[ssl_hostname]id: %ld\n", ssl->id);
      TLS_DEBUG("[ssl_hostname]hostname: %s\n",
        out->u.hostname.hostname);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t mbedtlsstub_sslbio_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR void *p_bio = (FAR void *)arg[1];
  uint32_t fd = *(uint32_t *)arg[5];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_bio_s *out =
        (FAR struct apicmd_ssl_bio_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      TLS_DEBUG("[ssl_bio]id: %ld\n", ssl->id);

      if (p_bio != NULL)
        {
          out->p_bio = htonl(fd);

          TLS_DEBUG("[ssl_bio]p_bio(fd): %lu\n", fd);
        }
      else
        {
          out->p_bio = 0;
          TLS_DEBUG("[ssl_bio]p_bio: %lu\n", out->p_bio);
        }

      *altcid = APICMDID_TLS_SSL_BIO;
      size = sizeof(struct apicmd_ssl_bio_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_BIO);
      TLS_DEBUG("[ssl_bio]id: %lu\n", ssl->id);

      if (p_bio != NULL)
        {
          out->u.bio.p_bio = htonl(fd);

          TLS_DEBUG("[ssl_bio]p_bio(fd): %lu\n", fd);
        }
      else
        {
          out->u.bio.p_bio = 0;
          TLS_DEBUG("[ssl_bio]p_bio: %lu\n", out->u.bio.p_bio);
        }

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t mbedtlsstub_sslhandshake_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_handshake_s *out =
        (FAR struct apicmd_ssl_handshake_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_HANDSHAKE;
      size = sizeof(struct apicmd_ssl_handshake_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_HANDSHAKE);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_handshake]id: %lu\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_sslwrite_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  size_t   writelen = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR const unsigned char *buf = (FAR const unsigned char *)arg[1];
  FAR size_t *len = (FAR size_t *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_write_s *out =
        (FAR struct apicmd_ssl_write_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      writelen = (*len <= APICMD_SSL_WRITE_BUF_LEN)
        ? *len : APICMD_SSL_WRITE_BUF_LEN;
      memcpy(out->buf, buf, writelen);
      out->len = htonl(writelen);

      *altcid = APICMDID_TLS_SSL_WRITE;
      size = sizeof(struct apicmd_ssl_write_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id =
        htonl(APISUBCMDID_TLS_SSL_WRITE);
      writelen = (*len <= APICMD_SSL_WRITE_BUF_LEN)
        ? *len : APICMD_SSL_WRITE_BUF_LEN;
      memcpy(out->u.write.buf, buf, writelen);
      out->u.write.len = htonl(writelen);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_write]ctx id: %lu\n", ssl->id);
  TLS_DEBUG("[ssl_write]write len: %d\n", writelen);

  return size;
}

int32_t mbedtlsstub_sslread_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  size_t req_buf_len = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];
  FAR size_t *len = (FAR size_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_read_s *out =
        (FAR struct apicmd_ssl_read_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      req_buf_len = (*len <= APICMD_SSL_READ_BUF_LEN)
        ? *len : APICMD_SSL_READ_BUF_LEN;
      out->len = htonl(req_buf_len);

      *altcid = APICMDID_TLS_SSL_READ;
      size = sizeof(struct apicmd_ssl_read_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_READ);

      req_buf_len = (*len <= APICMD_SSL_READ_BUF_LEN)
        ? *len : APICMD_SSL_READ_BUF_LEN;
      out->u.read.len = htonl(req_buf_len);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_read]ctx id: %lu\n", ssl->id);
  TLS_DEBUG("[ssl_read]read len: %d\n", req_buf_len);

  return size;
}

int32_t mbedtlsstub_sslclosenotif_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_close_notify_s *out =
        (FAR struct apicmd_ssl_close_notify_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_CLOSE_NOTIFY;
      size = sizeof(struct apicmd_ssl_close_notify_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_CLOSE_NOTIFY);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_close_notify]id: %lu\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_sslver_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_version_s *out =
        (FAR struct apicmd_ssl_version_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_VERSION;
      size = sizeof(struct apicmd_ssl_version_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id =
        htonl(APISUBCMDID_TLS_SSL_VERSION);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_version]ctx id: %lu\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_sslciphersuite_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_ciphersuite_s *out =
        (FAR struct apicmd_ssl_ciphersuite_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_CIPHERSUITE;
      size = sizeof(struct apicmd_ssl_ciphersuite_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_CIPHERSUITE);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_ciphersuite]ctx id: %lu\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_sslciphersuiteid_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const char *ciphersuite_name = (FAR const char *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_ciphersuite_id_s *out =
        (FAR struct apicmd_ssl_ciphersuite_id_s *)pktbuf;

      memset(out->ciphersuite, '\0', APICMD_SSL_CIPHERSUITE_REQLEN);
      strncpy((char *)out->ciphersuite, ciphersuite_name,
        APICMD_SSL_CIPHERSUITE_REQLEN);

      TLS_DEBUG("[ssl_ciphersuite_id]ciphersuite: %s\n",
        out->ciphersuite);

      *altcid = APICMDID_TLS_SSL_CIPHERSUITE_ID;
      size = sizeof(struct apicmd_ssl_ciphersuite_id_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_CIPHERSUITE_ID);
      memset(out->u.ciphersuite_id.ciphersuite,
        '\0', APICMD_SSL_CIPHERSUITE_REQLEN);
      strncpy((char *)out->u.ciphersuite_id.ciphersuite, ciphersuite_name,
        APICMD_SSL_CIPHERSUITE_REQLEN);

      TLS_DEBUG("[ssl_ciphersuite_id]ciphersuite: %s\n",
        out->u.ciphersuite_id.ciphersuite);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t mbedtlsstub_sslrecexp_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_record_expansion_s *out =
        (FAR struct apicmd_ssl_record_expansion_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_RECORD_EXPANSION;
      size = sizeof(struct apicmd_ssl_record_expansion_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_RECORD_EXPANSION);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_record_expansion]ctx id: %lu\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_sslvrfyresult_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl = (FAR const mbedtls_ssl_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_verify_result_s *out =
        (FAR struct apicmd_ssl_verify_result_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_VERIFY_RESULT;
      size = sizeof(struct apicmd_ssl_verify_result_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id =
        htonl(APISUBCMDID_TLS_SSL_VERIFY_RESULT);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_verify_result]ctx id: %lu\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_ssltimercb_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR mbedtls_ssl_context *ssl = (FAR mbedtls_ssl_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_timer_cb_s *out =
        (FAR struct apicmd_ssl_timer_cb_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_TIMER_CB;
      size = sizeof(struct apicmd_ssl_timer_cb_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_TIMER_CB);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_timer_cb]config id: %lu\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_sslpeercert_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_peer_cert_s *out =
        (FAR struct apicmd_ssl_peer_cert_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_PEER_CERT;
      size = sizeof(struct apicmd_ssl_peer_cert_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_PEER_CERT);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_peer_cert]ctx id: %lu\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_sslbytesavail_pkt_compose(FAR void **arg,
                              size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                              const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const mbedtls_ssl_context *ssl =
    (FAR const mbedtls_ssl_context *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_bytes_avail_s *out =
        (FAR struct apicmd_ssl_bytes_avail_s *)pktbuf;

      out->ssl = htonl(ssl->id);

      *altcid = APICMDID_TLS_SSL_BYTES_AVAIL;
      size = sizeof(struct apicmd_ssl_bytes_avail_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_sslcmd_s *out =
        (FAR struct apicmd_sslcmd_s *)pktbuf;

      out->ssl = htonl(ssl->id);
      out->subcmd_id = htonl(APISUBCMDID_TLS_SSL_BYTES_AVAIL);

      *altcid = APICMDID_TLS_SSL_CMD_V4;
      size = sizeof(struct apicmd_sslcmd_s);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  TLS_DEBUG("[ssl_bytes_avail]id: %lu\n", ssl->id);

  return size;
}

int32_t mbedtlsstub_sslinit_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_initres_s *in =
        (FAR struct apicmd_ssl_initres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_init res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslfree_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_freeres_s *in =
        (FAR struct apicmd_ssl_freeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_free res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslsetup_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_setupres_s *in =
        (FAR struct apicmd_ssl_setupres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_setup res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslhostname_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_hostnameres_s *in =
        (FAR struct apicmd_ssl_hostnameres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_hostname res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslbio_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_biores_s *in =
        (FAR struct apicmd_ssl_biores_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_bio res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslhandshake_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_handshakeres_s *in =
        (FAR struct apicmd_ssl_handshakeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_handshake res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslwrite_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_writeres_s *in =
        (FAR struct apicmd_ssl_writeres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_write res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslread_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  size_t req_buf_len = 0;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR unsigned char *buf = (FAR unsigned char *)arg[1];
  FAR size_t *len = (FAR size_t *)arg[2];

  req_buf_len = (*len <= APICMD_SSL_READ_BUF_LEN)
    ? *len : APICMD_SSL_READ_BUF_LEN;
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_readres_s *in =
        (FAR struct apicmd_ssl_readres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_read res]ret: %ld\n", *ret);
      if (*ret <= 0)
        {
          /* Nothing to do */
        }
      else if ((0 < *ret) && (*ret <= req_buf_len))
        {
          memcpy(buf, in->buf, *ret);
        }
      else
        {
          TLS_ERROR("Unexpected buffer length: %ld\n", *ret);
          return MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
        }
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

int32_t mbedtlsstub_sslclosenotif_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_close_notifyres_s *in =
        (FAR struct apicmd_ssl_close_notifyres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_close_notify res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslver_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR const char **version = (FAR const char **)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_versionres_s *in =
        (FAR struct apicmd_ssl_versionres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_version res]ret: %ld\n", *ret);
      in->version[APICMD_SSL_VERSION_LEN-1] = '\0';
      *version = get_ssl_tls_version(in->version);
      TLS_DEBUG("[ssl_version res]version: %s\n", *version);
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

int32_t mbedtlsstub_sslciphersuite_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR char *cipher_str_buf = (FAR char *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_ciphersuiteres_s *in =
        (FAR struct apicmd_ssl_ciphersuiteres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_ciphersuite res]ret: %ld\n", *ret);

      memset(cipher_str_buf, '\0', SSL_CIPHER_STR_BUF);
      strncpy((char *)cipher_str_buf, (const char *)in->ciphersuite,
        SSL_CIPHER_STR_BUF-1);
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

int32_t mbedtlsstub_sslciphersuiteid_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif
  int32_t  ret_cipher_id = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_ciphersuite_idres_s *in =
        (FAR struct apicmd_ssl_ciphersuite_idres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      ret_cipher_id = ntohl(in->id);
      TLS_DEBUG("[ssl_ciphersuite_id res]ret: %ld\n", *ret);
      TLS_DEBUG("[ssl_ciphersuite_id res]id: %ld\n", ret_cipher_id);
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

  return ret_cipher_id;
}

int32_t mbedtlsstub_sslrecexp_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_record_expansionres_s *in =
        (FAR struct apicmd_ssl_record_expansionres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_record_expansion res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslvrfyresult_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_verify_resultres_s *in =
        (FAR struct apicmd_ssl_verify_resultres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_verify_result res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_ssltimercb_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_timer_cbres_s *in =
        (FAR struct apicmd_ssl_timer_cbres_s *)pktbuf;

      *ret = ntohl(in->ret_code);
      TLS_DEBUG("[ssl_timer_cb res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslpeercert_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR mbedtls_x509_crt *x509_crt = (FAR mbedtls_x509_crt *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_peer_certres_s *in =
        (FAR struct apicmd_ssl_peer_certres_s *)pktbuf;

      *ret = ntohl(in->ret_code);

      x509_crt->id = ntohl(in->crt);
      TLS_DEBUG("[ssl_peer_cert res]ret: %ld\n", *ret);
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

int32_t mbedtlsstub_sslbytesavail_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
                              size_t pktsz, uint8_t altver, FAR void **arg,
                              size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR int32_t *ret = (FAR int32_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_ssl_bytes_availres_s *in =
        (FAR struct apicmd_ssl_bytes_availres_s *)pktbuf;

      *ret = ntohl( in->avail_bytes);
      TLS_DEBUG("[ssl_bytes_avail res]ret: %ld\n", *ret);
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


int32_t mbedtlsstub_sslcmd_pkt_parse(FAR struct alt1250_dev_s *dev, FAR uint8_t *pktbuf,
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
      FAR struct apicmd_sslcmdres_s *in =
        (FAR struct apicmd_sslcmdres_s *)pktbuf;

      uint32_t subcmd_id = ntohl(in->subcmd_id);
      TLS_DEBUG("[sslcmd_pkt_parse res]ret: %ld\n", ntohl(in->ret_code));

      if (subcmd_id == 0 || subcmd_id > APISUBCMDID_TLS_SSL_BYTES_AVAIL)
        {
          TLS_ERROR("Unexpected sub command id: %lu\n",subcmd_id);
          return -1;
        }
      if (subcmd_id == APISUBCMDID_TLS_SSL_READ)
        {
          size_t req_buf_len = 0;
          FAR unsigned char *buf = (FAR unsigned char *)arg[1];
          FAR size_t *len = (FAR size_t *)arg[2];

          req_buf_len = (*len <= APICMD_SSL_READ_BUF_LEN)
            ? *len : APICMD_SSL_READ_BUF_LEN;

          *ret_code = ntohl(in->ret_code);
          if (*ret_code <= 0)
            {
              /* Nothing to do */
            }
          else if ((0 < *ret_code) && (*ret_code <= req_buf_len))
            {
              memcpy(buf, in->u.readres.buf, *ret_code);
            }
          else
            {
              TLS_ERROR("Unexpected buffer length: %ld\n", *ret_code);
              return MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
            }

          TLS_DEBUG("[ssl_read res]ret: %ld\n", ntohl(in->ret_code));
        }
      else if (subcmd_id == APISUBCMDID_TLS_SSL_VERSION)
        {
          FAR const char **version = (FAR const char **)arg[1];

          *ret_code = ntohl(in->ret_code);

          TLS_DEBUG("[ssl_version res]ret: %ld\n", *ret_code);
          TLS_DEBUG("[ssl_version res]version: %s\n",
            in->u.versionres.version);

          in->u.versionres.version[APICMD_SSL_VERSION_LEN-1] = '\0';
          *version = get_ssl_tls_version(in->u.versionres.version);
        }
      else if (subcmd_id == APISUBCMDID_TLS_SSL_PEER_CERT)
        {
          FAR mbedtls_x509_crt *x509_crt = (FAR mbedtls_x509_crt *)arg[1];

          *ret_code = ntohl(in->ret_code);

          x509_crt->id = ntohl(in->u.peer_certres.crt);

          TLS_DEBUG("[ssl_peer_cert res]ret: %ld\n", ntohl(in->ret_code));

        }
      else if (subcmd_id == APISUBCMDID_TLS_SSL_BYTES_AVAIL)
        {
          *ret_code = ntohl( in->u.bytes_availres.avail_bytes);
          TLS_DEBUG("[ssl_bytes_avail res]avail_bytes: %ld\n", *ret_code);
        }
      else if (subcmd_id == APISUBCMDID_TLS_SSL_CIPHERSUITE)
        {
          FAR char *cipher_str_buf = (FAR char *)arg[1];

          *ret_code = ntohl(in->ret_code);
          TLS_DEBUG("[ssl_ciphersuite res]ret: %ld\n", *ret_code);

          memset(cipher_str_buf, '\0', SSL_CIPHER_STR_BUF);
          strncpy((char *)cipher_str_buf,
            (const char *)in->u.ciphersuiteres.ciphersuite,
            SSL_CIPHER_STR_BUF-1);
        }
      else if (subcmd_id == APISUBCMDID_TLS_SSL_CIPHERSUITE_ID)
        {
          *ret_code = ntohl(in->ret_code);
          ret = ntohl(in->u.ciphersuite_idres.id);

          TLS_DEBUG("[ssl_ciphersuite_id res]ret: %ld\n", *ret_code);
          TLS_DEBUG("[ssl_ciphersuite_id res]id: %ld\n", ret);
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
