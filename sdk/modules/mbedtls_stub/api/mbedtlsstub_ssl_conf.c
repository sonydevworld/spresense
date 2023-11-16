/****************************************************************************
 * modules/mbedtls_stub/api/mbedtlsstub_ssl_conf.c
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
 * Private Data
 ****************************************************************************/

static sem_t g_vrfylock = SEM_INITIALIZER(1);
static int (*g_f_vrfy)(void *, mbedtls_x509_crt *, int, uint32_t *);
static void *g_p_vrfy;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void ssllock(FAR sem_t *lock)
{
  int ret;

  do
    {
      ret = sem_wait(lock);
    }
  while (ret == -EINTR);
}

static inline void sslunlock(FAR sem_t *lock)
{
  sem_post(lock);
}

static void verifycb_event(FAR void **cbarg)
{
  int ret;
  uint32_t flags = 0;
  uint32_t id = *((FAR uint32_t *)cbarg[0]);
  int32_t depth = *((FAR int32_t *)cbarg[1]);
  int (*f_vrfy)(void *, mbedtls_x509_crt *, int, uint32_t *);
  void *p_vrfy;
  mbedtls_x509_crt crt =
    {
    };

  FAR void *inarg[] =
    {
      &ret, &flags
    };

  crt.id = id;

  ssllock(&g_vrfylock);
  f_vrfy = g_f_vrfy;
  p_vrfy = g_p_vrfy;
  sslunlock(&g_vrfylock);

  if (f_vrfy)
    {
      ret = f_vrfy(p_vrfy, &crt, depth, &flags);

      lapi_req(LTE_CMDID_TLS_CONFIG_VERIFY_CALLBACK | LTE_CMDOPT_ASYNC_BIT,
               (FAR void *)inarg, ARRAY_SZ(inarg),
               NULL, 0, NULL);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_ssl_config_init(mbedtls_ssl_config *conf)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&conf->id};
  FAR void *outarg[] = {&result, conf};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_ssl_config_free(mbedtls_ssl_config *conf)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {conf};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ssl_config_defaults(mbedtls_ssl_config *conf, int endpoint, int transport, int preset)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {conf, &endpoint, &transport, &preset};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_DEFAULTS,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_ssl_conf_authmode(mbedtls_ssl_config *conf, int authmode)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {conf, &authmode};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_AUTHMODE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_ssl_conf_rng(mbedtls_ssl_config *conf, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {conf, f_rng, p_rng};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_RNG,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_ssl_conf_ca_chain(mbedtls_ssl_config *conf, mbedtls_x509_crt *ca_chain, mbedtls_x509_crl *ca_crl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {conf, ca_chain, ca_crl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_CA_CHAIN,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ssl_conf_own_cert(mbedtls_ssl_config *conf, mbedtls_x509_crt *own_cert, mbedtls_pk_context *pk_key)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {conf, own_cert, pk_key};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_OWN_CERT,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_ssl_conf_read_timeout(mbedtls_ssl_config *conf, uint32_t timeout)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {conf, &timeout};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_READ_TIMEOUT,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_ssl_conf_verify(mbedtls_ssl_config *conf, int (*f_vrfy)(void *, mbedtls_x509_crt *, int, uint32_t *), void *p_vrfy)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {conf, f_vrfy, p_vrfy};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_VERIFY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 verifycb_event);
  if (ret == 0)
    {
      ret = result;

      /* Only one callback can be registered with mbedtls_ssl_conf_verify().
       * Therefore, note that if mbedtls_ssl_conf_verify() is executed,
       * it will be overwritten.
       */

      ssllock(&g_vrfylock);
      g_f_vrfy = f_vrfy;
      g_p_vrfy = p_vrfy;
      sslunlock(&g_vrfylock);
    }

  return ;
}

int mbedtls_ssl_conf_alpn_protocols(mbedtls_ssl_config *conf, const char **protos)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {conf, protos};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_ALPN_PROTOCOLS,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_ssl_conf_ciphersuites(mbedtls_ssl_config *conf, const int *ciphersuites)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {conf, (void *)ciphersuites};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CONFIG_CIPHERSUITES,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ssl_config_getctx(mbedtls_ssl_config *conf, uint8_t *buff, size_t size)
{
  int ret = sizeof(mbedtls_ssl_config);

  if (conf && buff && size >= sizeof(mbedtls_ssl_config))
    {
      mbedtls_ssl_config *ctx = (mbedtls_ssl_config *)buff;
      ctx->id = conf->id;
    }
  else if (conf && buff)
    {
      ret = MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_ssl_config_setctx(mbedtls_ssl_config *conf, uint8_t *buff, size_t size)
{
  int ret = 0;

  if (conf && buff && size >= sizeof(mbedtls_ssl_config))
    {
      mbedtls_ssl_config *ctx = (mbedtls_ssl_config *)buff;
      conf->id = ctx->id;
    }
  else
    {
      ret = MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_ssl_config_getctxsize(mbedtls_ssl_config *conf)
{
  return mbedtls_ssl_config_getctx(NULL, NULL, 0);
}

int mbedtls_x509_crl_getctx(mbedtls_x509_crl *crl, uint8_t *buff, size_t size)
{
  int ret = sizeof(mbedtls_x509_crl);

  if (crl && buff && size >= sizeof(mbedtls_x509_crl))
    {
      mbedtls_x509_crl *ctx = (mbedtls_x509_crl *)buff;
      ctx->id = crl->id;
    }
  else if (crl && buff)
    {
      ret = MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_x509_crl_setctx(mbedtls_x509_crl *crl, uint8_t *buff, size_t size)
{
  int ret = 0;

  if (crl && buff && size >= sizeof(mbedtls_x509_crl))
    {
      mbedtls_x509_crl *ctx = (mbedtls_x509_crl *)buff;
      crl->id = ctx->id;
    }
  else
    {
      ret = MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  return ret;
}

int mbedtls_x509_crl_getctxsize(mbedtls_x509_crl *crl)
{
  return mbedtls_x509_crl_getctx(NULL, NULL, 0);
}
