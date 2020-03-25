/****************************************************************************
 * examples/lte_azureiot/mbedtls_if.c
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#include <string.h>
#include <dirent.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/certs.h>
#include "mbedtls_if.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_PORT_NO             "443"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mbedtls_net_context      g_server_fd;
static mbedtls_x509_crt         g_ca;
static mbedtls_entropy_context  g_entropy;
static mbedtls_ctr_drbg_context g_ctr_drbg;
static mbedtls_ssl_context      g_ssl;
static mbedtls_ssl_config       g_conf;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int setup_certificates(mbedtls_x509_crt *ca, char *certs_buf, int certs_buf_size)
{
  certs_buf[certs_buf_size - 1] = '\0';

  return mbedtls_x509_crt_parse(ca, (const unsigned char *)certs_buf, certs_buf_size);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tls_connect(const char *hostname, char *certs_buf, int certs_buf_size)
{
  FAR static const char *pers = "mbedtls";

  /* Initialize mbedTLS stuff */

  mbedtls_ctr_drbg_init(&g_ctr_drbg);
  mbedtls_net_init(&g_server_fd);
  mbedtls_ssl_init(&g_ssl);
  mbedtls_ssl_config_init(&g_conf);
  mbedtls_x509_crt_init(&g_ca);
  mbedtls_entropy_init(&g_entropy);

  /* Setup certificates. */

  if (setup_certificates(&g_ca, certs_buf, certs_buf_size))
    {
      printf("Fail: setup_certificates()\n");
      goto errout_disconnect;
    }

  mbedtls_ssl_conf_ca_chain(&g_conf, &g_ca, NULL);
  mbedtls_ssl_conf_authmode(&g_conf, MBEDTLS_SSL_VERIFY_REQUIRED);

  /* Setup mbedTLS stuff */

  if (mbedtls_ctr_drbg_seed(&g_ctr_drbg, mbedtls_entropy_func, &g_entropy,
                            (const unsigned char *)pers, strlen(pers)) != 0)
    {
      printf("mbedtls_ctr_drbg_seed() fail\n");
      goto errout_disconnect;
    }

  if (mbedtls_ssl_config_defaults(&g_conf,
                                  MBEDTLS_SSL_IS_CLIENT,
                                  MBEDTLS_SSL_TRANSPORT_STREAM,
                                  MBEDTLS_SSL_PRESET_DEFAULT) != 0)
    {
      printf("mbedtls_ssl_config_defaults() fail\n");
      goto errout_disconnect;
    }

  mbedtls_ssl_conf_rng(&g_conf, mbedtls_ctr_drbg_random, &g_ctr_drbg);

  if (mbedtls_ssl_setup(&g_ssl, &g_conf) != 0)
    {
      printf("mbedtls_ssl_setup() fail\n");
      goto errout_disconnect;
    }

  if (mbedtls_ssl_set_hostname(&g_ssl, hostname) != 0)
    {
      printf("mbedtls_ssl_set_hostname() fail\n");
      goto errout_disconnect;
    }

  /* Start the connection.
   * mbedtls_net_connect execute address resolution, socket create,
   * and connect.
   */

  int   i;

  for (i = 10; 0 < i; i--)
    {
      if (mbedtls_net_connect(&g_server_fd,
                              hostname,
                              APP_PORT_NO,    /* The type is not integer, but string. */
                              MBEDTLS_NET_PROTO_TCP) == 0)
        {
          break;
        }

      printf("mbedtls_net_connect() retry\n");
      sleep(1);
    }

  if (i == 0)
    {
      printf("mbedtls_net_connect() fail\n");
      goto errout_close;
    }

  mbedtls_ssl_set_bio(&g_ssl,
                      &g_server_fd,
                      mbedtls_net_send,
                      mbedtls_net_recv,
                      NULL);

  /* Do SSL handshake */

  for (i = 10; 0 < i; i--)
    {
      if (mbedtls_ssl_handshake(&g_ssl) == 0)
        {
          break;
        }

      printf("mbedtls_ssl_handshake() retry\n");
      sleep(1);
    }

  if (i == 0)
    {
      printf("mbedtls_ssl_handshake() fail\n");
      goto errout_close;
    }

  return OK;

errout_close:

  mbedtls_ssl_close_notify(&g_ssl);

  /* Close the connection */

errout_disconnect:

  /* Free mbedTLS stuff */

  mbedtls_net_free(&g_server_fd);
  mbedtls_ssl_free(&g_ssl);
  mbedtls_ssl_config_free(&g_conf);
  mbedtls_ctr_drbg_free(&g_ctr_drbg);
  mbedtls_entropy_free(&g_entropy);
  mbedtls_x509_crt_free(&g_ca);

  return ERROR;
}

void tls_disconnect(void)
{
  /* Close the connection */

  mbedtls_ssl_close_notify(&g_ssl);

  /* Free mbedTLS stuff */

  mbedtls_net_free(&g_server_fd);
  mbedtls_ssl_free(&g_ssl);
  mbedtls_ssl_config_free(&g_conf);
  mbedtls_ctr_drbg_free(&g_ctr_drbg);
  mbedtls_entropy_free(&g_entropy);
  mbedtls_x509_crt_free(&g_ca);
}

int tls_write(char *iobuffer, int iobuffer_size)
{
  int            ret;
  unsigned char *buf_ptr     = (unsigned char *)iobuffer;
  size_t         request_len = iobuffer_size;

  do
    {
      ret = mbedtls_ssl_write(&g_ssl, buf_ptr, request_len);

      /* Return value means actual written size or error_code */

      if (ret > 0)
        {
          /* Successful case */

          if (ret < request_len)
            {
              /* Written size may be smaller than requested size.
               * In such case, for examples, shift address and retry
               */

              buf_ptr     += ret;
              request_len -= ret;
              continue;
            }

          break;
        }
      else if (ret == 0)
        {
          /* written size = 0 means communication end */

          break;
        }
      else
        {
          /* Unsuccessful case.
           * retry or error return depending on error code.
           */

          if ((ret == MBEDTLS_ERR_SSL_WANT_READ) ||
              (ret == MBEDTLS_ERR_SSL_WANT_WRITE))
            {
              /* Please retry */

              continue;
            }
          else
            {
              /* Error case */

              printf("mbedtls_ssl_write() fail: ret = %x\n", ret);
              iobuffer_size = ret;
              break;
            }
        }
    }
  while (1);

  return iobuffer_size;
}

int tls_read(char *iobuffer, int iobuffer_size)
{
  return mbedtls_ssl_read(&g_ssl, (unsigned char *)iobuffer, iobuffer_size);
}
