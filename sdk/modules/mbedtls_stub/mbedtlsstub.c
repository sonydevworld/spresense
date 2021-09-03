/****************************************************************************
 * modules/mbedtls_stub/mbedtlsstub.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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
#include <errno.h>
#include <sys/types.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <nuttx/wireless/lte/lte_ioctl.h>

#include <mbedtls/ssl.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/x509_csr.h>
#include <mbedtls/entropy.h>
#include <mbedtls/pk_internal.h>
#include <mbedtls/md_internal.h>
#include <mbedtls/net_sockets.h>

#include "lte/lapi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SZ
#  define ARRAY_SZ(array) (sizeof(array)/sizeof(array[0]))
#endif

#define CIPHER_STR_BUF 64

#define TLS_UNKNOWN    "unknown"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char cipher_str_buf[CIPHER_STR_BUF] = {0};
static mbedtls_x509_crt g_x509_crt         = {0};
static struct mbedtls_pk_info_t g_pk_info  = {0};
static mbedtls_rsa_context g_rsa_context   = {0};
static mbedtls_cipher_info_t g_cipher_info = {0};
static mbedtls_md_info_t g_md_info         = {0};
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

static int32_t mbedtls_load_local_file(const char *path, unsigned char **buf, size_t *len)
{
  FILE *fd;
  size_t size;
  fpos_t end_pos = 0;

  /* Check file size */

  if ((fd = fopen( path, "rb" )) == NULL)
    {
      return (MBEDTLS_ERR_PK_FILE_IO_ERROR);
    }

  fseek(fd, 0, SEEK_END);
  if (0 != fgetpos(fd, &end_pos))
    {
      fclose(fd);
      return(MBEDTLS_ERR_PK_FILE_IO_ERROR);
    }
  fseek(fd, 0, SEEK_SET);

  if (end_pos <= 0)
    {
      fclose(fd);
      return(MBEDTLS_ERR_PK_FILE_IO_ERROR);
    }

  size = end_pos;

  /* Malloc buffer */

  *len = (size_t) size;
  if ((*len+1 == 0) || ((*buf = malloc(*len+1)) == NULL ))
    {
      fclose(fd);
      return(MBEDTLS_ERR_PK_ALLOC_FAILED);
    }

  /* Read certificate file */

  if (fread(*buf, 1, *len, fd) != *len)
    {
      fclose(fd);
      free(*buf);
      return(MBEDTLS_ERR_PK_FILE_IO_ERROR);
    }

  fclose(fd);
  (*buf)[*len] = '\0';

  if (strstr((const char *)*buf, "-----BEGIN ") != NULL)
    {
      ++*len;
    }

  return(0);
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

void mbedtls_ssl_session_init(mbedtls_ssl_session *session)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&session->id};
  FAR void *outarg[] = {&result, session};

  ret = lapi_req(LTE_CMDID_TLS_SESSION_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_ssl_session_free(mbedtls_ssl_session *session)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {session};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SESSION_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ssl_get_session(const mbedtls_ssl_context *ssl, mbedtls_ssl_session *session)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)ssl, session};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SESSION_GET,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_set_session(mbedtls_ssl_context *ssl, const mbedtls_ssl_session *session)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, (void *)session};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SESSION_SET,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_session_reset(mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SESSION_RESET,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_x509_crt_init(mbedtls_x509_crt *crt)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&crt->id};
  FAR void *outarg[] = {&result, crt};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_x509_crt_free(mbedtls_x509_crt *crt)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {crt};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_x509_crt_parse_file(mbedtls_x509_crt *chain, const char *path)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {chain, (void *)path};
  FAR void *outarg[] = {&result};

  FAR unsigned char *parse_buf = NULL;
  size_t parse_len;

  result = mbedtls_load_local_file(path, &parse_buf, &parse_len);
  if (result == 0)
    {
      ret = mbedtls_x509_crt_parse(chain, parse_buf, parse_len);
      free(parse_buf);
    }
  else
    {
      ret = lapi_req(LTE_CMDID_TLS_X509_CRT_PARSE_FILE,
                     (FAR void *)inarg, ARRAY_SZ(inarg),
                     (FAR void *)outarg, ARRAY_SZ(outarg),
                     NULL);
      if (ret == 0)
        {
          ret = result;
        }
    }

  return ret;
}

int mbedtls_x509_crt_parse_der(mbedtls_x509_crt *chain, const unsigned char *buf, size_t buflen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {chain, (void *)buf, &buflen};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_PARSE_DER,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_crt_parse(mbedtls_x509_crt *chain, const unsigned char *buf, size_t buflen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {chain, (void *)buf, &buflen};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_PARSE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_crt_info(char *buf, size_t size, const char *prefix, const mbedtls_x509_crt *crt)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&size, (void *)prefix, (void *)crt};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_INFO,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_crt_verify_info(char *buf, size_t size, const char *prefix, uint32_t flags)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&size, (void *)prefix, &flags};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_X509_CRT_VERIFY_INFO,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_pk_init(mbedtls_pk_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_PK_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_pk_free(mbedtls_pk_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_PK_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_pk_parse_keyfile(mbedtls_pk_context *ctx, const char *path, const char *password)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)path, (void *)password};
  FAR void *outarg[] = {&result};

  FAR unsigned char *parse_buf = NULL;
  size_t parse_len;

  result = mbedtls_load_local_file(path, &parse_buf, &parse_len);
  if (result == 0)
    {
      if (password == NULL)
        {
          ret = mbedtls_pk_parse_key(ctx, parse_buf, parse_len, NULL, 0);
        }
      else
        {
          ret = mbedtls_pk_parse_key(ctx, parse_buf, parse_len,
                      (const unsigned char*)password, strlen(password));
        }

      free(parse_buf);
    }
  else
    {
      ret = lapi_req(LTE_CMDID_TLS_PK_PARSE_KEYFILE,
                     (FAR void *)inarg, ARRAY_SZ(inarg),
                     (FAR void *)outarg, ARRAY_SZ(outarg),
                     NULL);
      if (ret == 0)
        {
          ret = result;
        }
    }

  return ret;
}

int mbedtls_pk_parse_key(mbedtls_pk_context *ctx, const unsigned char *key, size_t keylen, const unsigned char *pwd, size_t pwdlen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)key, &keylen, (void *)pwd, &pwdlen};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_PK_PARSE_KEY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_pk_check_pair(const mbedtls_pk_context *pub, const mbedtls_pk_context *prv)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)pub, (void *)prv};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_PK_CHECK_PAIR,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_pk_setup(mbedtls_pk_context *ctx, const mbedtls_pk_info_t *info)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)info};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_PK_SETUP,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

const mbedtls_pk_info_t *mbedtls_pk_info_from_type(mbedtls_pk_type_t pk_type)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&pk_type};
  FAR void *outarg[] = {&result, &g_pk_info};

  ret = lapi_req(LTE_CMDID_TLS_PK_INFO_FROM_TYPE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return &g_pk_info;
    }
  else
    {
      return NULL;
    }
}

int mbedtls_pk_write_key_pem(mbedtls_pk_context *ctx, unsigned char *buf, size_t size)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &size};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_PK_WRITE_KEY_PEM,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_pk_write_key_der(mbedtls_pk_context *ctx, unsigned char *buf, size_t size)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &size};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_PK_WRITE_KEY_DER,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

mbedtls_rsa_context *mbedtls_pk_rsa(const mbedtls_pk_context pk)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)&pk};
  FAR void *outarg[] = {&result, &g_rsa_context};

  ret = lapi_req(LTE_CMDID_TLS_PK_RSA,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return &g_rsa_context;
    }
  else
    {
      return NULL;
    }
}

void mbedtls_ctr_drbg_init(mbedtls_ctr_drbg_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_CTR_DRBG_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_ctr_drbg_free(mbedtls_ctr_drbg_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CTR_DRBG_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ctr_drbg_seed(mbedtls_ctr_drbg_context *ctx, int (*f_entropy)(void *, unsigned char *, size_t), void *p_entropy, const unsigned char *custom, size_t len)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, f_entropy, p_entropy, (void *)custom, &len};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CTR_DRBG_SEED,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_entropy_init(mbedtls_entropy_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_ENTROPY_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_entropy_free(mbedtls_entropy_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_ENTROPY_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_cipher_init(mbedtls_cipher_context_t *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_cipher_free(mbedtls_cipher_context_t *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

const mbedtls_cipher_info_t *mbedtls_cipher_info_from_string(const char *cipher_name)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)cipher_name};
  FAR void *outarg[] = {&result, &g_cipher_info};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_INFO_FROM_STRING,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return &g_cipher_info;
    }
  else
    {
      return NULL;
    }
}

int mbedtls_cipher_setup(mbedtls_cipher_context_t *ctx, const mbedtls_cipher_info_t *cipher_info)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)cipher_info};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_SETUP,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_cipher_setkey(mbedtls_cipher_context_t *ctx, const unsigned char *key, int key_bitlen, const mbedtls_operation_t operation)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)key, &key_bitlen, (void *)&operation};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_SETKEY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_cipher_set_iv(mbedtls_cipher_context_t *ctx, const unsigned char *iv, size_t iv_len)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)iv, &iv_len};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_SET_IV,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_cipher_update(mbedtls_cipher_context_t *ctx, const unsigned char *input, size_t ilen, unsigned char *output, size_t *olen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)input, &ilen};
  FAR void *outarg[] = {&result, output, olen};

  if (!ctx || !input || !output || !olen)
    {
      return MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
    }

  ret = lapi_req(LTE_CMDID_TLS_CIPHER_UPDATE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

const mbedtls_md_info_t *mbedtls_md_info_from_type(mbedtls_md_type_t md_type)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&md_type};
  FAR void *outarg[] = {&result, &g_md_info};

  ret = lapi_req(LTE_CMDID_TLS_MD_INFO_FROM_TYPE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return &g_md_info;
    }
  else
    {
      return NULL;
    }
}

unsigned char mbedtls_md_get_size(const mbedtls_md_info_t *md_info)
{
  int ret;
  int32_t result;
  uint8_t md_size;
  FAR void *inarg[] = {(void *)md_info};
  FAR void *outarg[] = {&result, &md_size};

  ret = lapi_req(LTE_CMDID_TLS_MD_GET_SIZE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      return md_size;
    }
  else
    {
      return 0;
    }
}

int mbedtls_md(const mbedtls_md_info_t *md_info, const unsigned char *input, size_t ilen, unsigned char *output)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)md_info, (void *)input, &ilen};
  FAR void *outarg[] = {&result, output};

  ret = lapi_req(LTE_CMDID_TLS_MD,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_md_digest(const mbedtls_md_info_t *md_info, mbedtls_x509_crt *chain, unsigned char *output)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)md_info, chain};
  FAR void *outarg[] = {&result, output};

  ret = lapi_req(LTE_CMDID_TLS_MD_DIGEST,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_base64_encode(unsigned char *dst, size_t dlen, size_t *olen, const unsigned char *src, size_t slen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&dlen, (void *)src, &slen};
  FAR void *outarg[] = {&result, dst, olen, &dlen};

  ret = lapi_req(LTE_CMDID_TLS_BASE64_ENCODE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_sha1(const unsigned char *input, size_t ilen, unsigned char output[20])
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)input, &ilen};
  FAR void *outarg[] = {&result, output};

  ret = lapi_req(LTE_CMDID_TLS_SHA1,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ssl_export_srtp_keys( mbedtls_ssl_context *ssl, uint8_t* key_buffer, uint16_t key_buffer_size)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, &key_buffer_size};
  FAR void *outarg[] = {&result, key_buffer, &key_buffer_size};

  ret = lapi_req(LTE_CMDID_TLS_SSL_EXPORT_SRTP_KEYS,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);

  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_ssl_set_use_srtp(mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_USE_SRTP,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_ssl_get_srtp_profile(mbedtls_ssl_context *ssl)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_SRTP_PROFILE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_ssl_set_turn(mbedtls_ssl_context *ssl, uint16_t turn_channel, uint32_t peer_addr, uint16_t peer_port)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ssl, &turn_channel, &peer_addr, &peer_port};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_SSL_TURN,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_mpi_init(mbedtls_mpi *X)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&X->id};
  FAR void *outarg[] = {&result, X};

  ret = lapi_req(LTE_CMDID_TLS_MPI_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_mpi_free(mbedtls_mpi *X)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {X};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_MPI_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_mpi_read_string(mbedtls_mpi *X, int radix, const char *s)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {X, &radix, (void *)s};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_MPI_READ_STRING,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_mpi_write_string(const mbedtls_mpi *X, int radix, char *buf, size_t buflen, size_t *olen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {(void *)X, &radix, &buflen};
  FAR void *outarg[] = {&result, buf, &buflen, olen};

  if (!X || !buf || !olen)
    {
      return MBEDTLS_ERR_MPI_BAD_INPUT_DATA;
    }

  ret = lapi_req(LTE_CMDID_TLS_MPI_WRITE_STRING,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_x509_csr_init(mbedtls_x509_csr *csr)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&csr->id};
  FAR void *outarg[] = {&result, csr};

  ret = lapi_req(LTE_CMDID_TLS_X509_CSR_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_x509_csr_free(mbedtls_x509_csr *csr)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {csr};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_CSR_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_x509_csr_parse_file(mbedtls_x509_csr *csr, const char *path)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {csr, (void *)path};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_CSR_PARSE_FILE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_csr_parse_der(mbedtls_x509_csr *csr, const unsigned char *buf, size_t buflen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {csr, (void *)buf, &buflen};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_CSR_PARSE_DER,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_csr_parse(mbedtls_x509_csr *csr, const unsigned char *buf, size_t buflen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {csr, (void *)buf, &buflen};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_CSR_PARSE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_dn_gets_crt(char *buf, size_t size, const mbedtls_x509_crt *crt)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {buf, &size, (void *)crt};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_DN_GETS_CRT,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509_dn_gets_csr(char *buf, size_t size, const mbedtls_x509_csr *csr)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {buf, &size, (void *)csr};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509_DN_GETS_CSR,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_x509write_crt_init(mbedtls_x509write_cert *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_INIT,
		 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_x509write_crt_free(mbedtls_x509write_cert *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_x509write_crt_der(mbedtls_x509write_cert *ctx, unsigned char *buf, size_t size, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &size, f_rng, p_rng};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_DER,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_pem(mbedtls_x509write_cert *ctx, unsigned char *buf, size_t size, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &size, f_rng, p_rng};
  FAR void *outarg[] = {&result, buf, &size};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_PEM,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_x509write_crt_set_subject_key(mbedtls_x509write_cert *ctx, mbedtls_pk_context *key)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, key};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_KEY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_x509write_crt_set_issuer_key(mbedtls_x509write_cert *ctx, mbedtls_pk_context *key)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, key};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_ISSUER_KEY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_x509write_crt_set_subject_name(mbedtls_x509write_cert *ctx, const char *subject_name)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)subject_name};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_NAME,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_issuer_name(mbedtls_x509write_cert *ctx, const char *issuer_name)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)issuer_name};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_ISSUER_NAME,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_x509write_crt_set_version(mbedtls_x509write_cert *ctx, int version)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &version};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_VERSION,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_x509write_crt_set_md_alg(mbedtls_x509write_cert *ctx, mbedtls_md_type_t md_alg)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &md_alg};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_MD_ALG,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_x509write_crt_set_serial(mbedtls_x509write_cert *ctx, const mbedtls_mpi *serial)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)serial};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_SERIAL,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_validity(mbedtls_x509write_cert *ctx, const char *not_before, const char *not_after)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, (void *)not_before, (void *)not_after};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_VALIDITY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_basic_constraints(mbedtls_x509write_cert *ctx, int is_ca, int max_pathlen)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &is_ca, &max_pathlen};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_CONSTRAINTS,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_subject_key_identifier(mbedtls_x509write_cert *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_SUBJECT_ID,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_authority_key_identifier(mbedtls_x509write_cert *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_AUTHORITY_ID,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_key_usage(mbedtls_x509write_cert *ctx, unsigned int key_usage)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &key_usage};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_KEY_USAGE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_x509write_crt_set_ns_cert_type(mbedtls_x509write_cert *ctx, unsigned char ns_cert_type)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, &ns_cert_type};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_X509WRITE_CRT_NS_CERT_TYPE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

void mbedtls_rsa_init(mbedtls_rsa_context *ctx, int padding, int hash_id)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {&padding, &hash_id, &ctx->id};
  FAR void *outarg[] = {&result, ctx};

  ret = lapi_req(LTE_CMDID_TLS_RSA_INIT,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

void mbedtls_rsa_free(mbedtls_rsa_context *ctx)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_RSA_FREE,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ;
}

int mbedtls_rsa_gen_key(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, unsigned int nbits, int exponent)
{
  int ret;
  int32_t result;
  FAR void *inarg[] = {ctx, f_rng, p_rng, &nbits, &exponent};
  FAR void *outarg[] = {&result};

  ret = lapi_req(LTE_CMDID_TLS_RSA_GEN_KEY,
                 (FAR void *)inarg, ARRAY_SZ(inarg),
                 (FAR void *)outarg, ARRAY_SZ(outarg),
                 NULL);
  if (ret == 0)
    {
      ret = result;
    }

  return ret;
}

int mbedtls_entropy_func(void *data, unsigned char *output, size_t len)
{
  return MBEDTLS_ERR_ENTROPY_SOURCE_FAILED;
}

int mbedtls_ctr_drbg_random(void *p_rng, unsigned char *output, size_t output_len)
{
    return MBEDTLS_ERR_CTR_DRBG_ENTROPY_SOURCE_FAILED;
}
