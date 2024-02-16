/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_ssl.c
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

#include <sys/ioctl.h>
#include <nuttx/wireless/lte/lte_ioctl.h>
#include <mbedtls/ssl.h>
#include <mbedtls/net_sockets.h>

#include "include/mbedtlsstub_utils.h"
#include "lte/lapi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CIPHER_STR_BUF 64

#define TLS_UNKNOWN    "unknown"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char cipher_str_buf[CIPHER_STR_BUF] = {0};
static mbedtls_x509_crt g_x509_crt = {0};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_ssl_init(mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ssl->id};
  FAR void *outarg[] = {&result, ssl};

  ret = lapi_req(LTE_CMDID_TLS_SSL_INIT,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_ssl_free(mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ssl_setup(mbedtls_ssl_context *ssl, const mbedtls_ssl_config *conf)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, (void *)conf};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_SETUP,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_set_hostname(mbedtls_ssl_context *ssl, const char *hostname)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, (void *)hostname};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_HOSTNAME,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_ssl_set_bio(mbedtls_ssl_context *ssl, void *p_bio, mbedtls_ssl_send_t *f_send, mbedtls_ssl_recv_t *f_recv, mbedtls_ssl_recv_timeout_t *f_recv_timeout)
{
  int32_t result;
  int fd;
  FAR void *inarg[] = {ssl, p_bio, f_send, f_recv, f_recv_timeout, &fd};
  FAR void *outarg[] = {&result};
  struct lte_ioctl_data_s cmd;

  if (!p_bio)
    {
      return;
    }

  cmd.cmdid = LTE_CMDID_TLS_SSL_BIO;
  cmd.inparam = inarg;
  cmd.inparamlen = ARRAY_SZ(inarg);
  cmd.outparam = outarg;
  cmd.outparamlen = ARRAY_SZ(outarg);
  cmd.cb = NULL;

  ioctl(((mbedtls_net_context *)p_bio)->fd, SIOCLTECMD, (unsigned long)&cmd);

  return ;
}

int mbedtls_ssl_handshake(mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_HANDSHAKE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_write(mbedtls_ssl_context *ssl, const unsigned char *buf, size_t len)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, (void *)buf, &len};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_WRITE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_read(mbedtls_ssl_context *ssl, unsigned char *buf, size_t len)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, &len};
  FAR void *outarg[] = {&result, buf, &len};

  ret = lapi_req(LTE_CMDID_TLS_SSL_READ,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_close_notify(mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_CLOSE_NOTIFY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

const char *mbedtls_ssl_get_version(const mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  const char* res_version = TLS_UNKNOWN;
  FAR void *inarg[] = {(void *)ssl};
  FAR void *outarg[] = {&result, (void *)&res_version};

  ret = lapi_req(LTE_CMDID_TLS_SSL_VERSION,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return res_version;
    }
  else
    {
      return NULL;
    }
}

const char *mbedtls_ssl_get_ciphersuite(const mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)ssl};
  FAR void *outarg[] = {&result, cipher_str_buf};

  ret = lapi_req(LTE_CMDID_TLS_SSL_CIPHERSUITE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return cipher_str_buf;
    }
  else
    {
      return NULL;
    }

}

int mbedtls_ssl_get_ciphersuite_id(const char *ciphersuite_name)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)ciphersuite_name};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_CIPHERSUITE_ID,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_get_record_expansion(const mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_RECORD_EXPANSION,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

uint32_t mbedtls_ssl_get_verify_result(const mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_VERIFY_RESULT,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_ssl_set_timer_cb(mbedtls_ssl_context *ssl, void *p_timer, mbedtls_ssl_set_timer_t *f_set_timer, mbedtls_ssl_get_timer_t *f_get_timer)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, p_timer, f_set_timer, f_get_timer};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_TIMER_CB,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

const mbedtls_x509_crt *mbedtls_ssl_get_peer_cert(const mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)ssl};
  FAR void *outarg[] = {&result, &g_x509_crt};

  ret = lapi_req(LTE_CMDID_TLS_SSL_PEER_CERT,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return &g_x509_crt;
    }
  else
    {
      return NULL;
    }
}

size_t mbedtls_ssl_get_bytes_avail(const mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_BYTES_AVAIL,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return result;
    }
  else
    {
      return 0;
    }
}

int mbedtls_ssl_getctx(mbedtls_ssl_context *ssl, uint8_t *buff, size_t size)
{
  int ret = sizeof(mbedtls_ssl_context);

  if (ssl && buff && size >= sizeof(mbedtls_ssl_context))
    {
      mbedtls_ssl_context *ctx = (mbedtls_ssl_context *)buff;
      ctx->id = ssl->id;
    }
  else if (ssl && buff)
    {
      ret = MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_ssl_setctx(mbedtls_ssl_context *ssl, uint8_t *buff, size_t size)
{
  int ret = 0;

  if (ssl && buff && size >= sizeof(mbedtls_ssl_context))
    {
      mbedtls_ssl_context *ctx = (mbedtls_ssl_context *)buff;
      ssl->id = ctx->id;
    }
  else
    {
      ret = MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_ssl_getctxsize(mbedtls_ssl_context *ssl)
{
  return mbedtls_ssl_getctx(NULL, NULL, 0);
}
