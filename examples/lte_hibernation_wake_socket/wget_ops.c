/****************************************************************************
 * examples/ltehibernation_wake_socket/wget_ops.c
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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
#include <stdio.h>
#include <netdb.h>

#include "wget_ops.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_TLS_CERT_FILENAME_LEN   128

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_tls_cert_filename[APP_TLS_CERT_FILENAME_LEN];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wgetops_sock_connect
 ****************************************************************************/

int wgetops_sock_connect(FAR void *ctx, FAR const char *hostname,
                         FAR const char *port)
{
  FAR struct wgetops_sock_context_s *sock =
    (FAR struct wgetops_sock_context_s *)ctx;
  int ret;
  struct addrinfo hints;
  struct addrinfo *ainfo = NULL;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  ret = getaddrinfo(hostname, port, &hints, &ainfo);
  if (ret != 0)
    {
      printf("getaddrinfo error:%d\n", ret);
      return -1;
    }

  sock->sockfd = socket(ainfo->ai_family,
                        ainfo->ai_socktype,
                        ainfo->ai_protocol);
  if (sock->sockfd < 0)
    {
      printf("socket error:%d\n", errno);
      freeaddrinfo(ainfo);
      return -1;
    }

  ret = connect(sock->sockfd, ainfo->ai_addr, ainfo->ai_addrlen);
  if (ret < 0)
    {
      printf("connect error:%d\n", errno);
      freeaddrinfo(ainfo);
      close(sock->sockfd);
      sock->sockfd = -1;
      return -1;
    }

  freeaddrinfo(ainfo);

  return 0;
}

/****************************************************************************
 * Name: wgetops_sock_send
 ****************************************************************************/

ssize_t wgetops_sock_send(FAR void *ctx, FAR const void *buf, size_t len)
{
  FAR struct wgetops_sock_context_s *sock =
    (FAR struct wgetops_sock_context_s *)ctx;
  int ret;

  ret = send(sock->sockfd, buf, len, 0);
  if (ret < 0)
    {
      printf("send() fail: %d\n", errno);
    }

  return ret;
}

/****************************************************************************
 * Name: wgetops_sock_recv
 ****************************************************************************/

ssize_t wgetops_sock_recv(FAR void *ctx, FAR void *buf, size_t len)
{
  FAR struct wgetops_sock_context_s *sock =
    (FAR struct wgetops_sock_context_s *)ctx;
  int ret;

  ret = recv(sock->sockfd, (FAR void *)buf, len, 0);
  if (ret < 0)
    {
      printf("recv() fail: %d\n", errno);
    }

  return ret;
}

/****************************************************************************
 * Name: wgetops_sock_close
 ****************************************************************************/

int wgetops_sock_close(FAR void *ctx)
{
  FAR struct wgetops_sock_context_s *sock =
    (FAR struct wgetops_sock_context_s *)ctx;

  close(sock->sockfd);

  return 0;
}

/****************************************************************************
 * Name: wgetops_tls_connect
 ****************************************************************************/

int wgetops_tls_connect(FAR void *ctx, FAR const char *hostname,
                        FAR const char *port)
{
  FAR struct wgetops_tls_context_s *tlsctx =
    (FAR struct wgetops_tls_context_s *)ctx;
  int ret;
  FAR static const char *pers = "mbedtls";
  FAR DIR *dirp = NULL;  /* Directory for TLS certification files */
  FAR struct dirent *cert_info = NULL;

  /* Initialize mbedTLS stuff */

  mbedtls_ctr_drbg_init(&tlsctx->ctr_drbg);
  mbedtls_net_init(&tlsctx->server_fd);
  mbedtls_ssl_init(&tlsctx->ssl);
  mbedtls_ssl_config_init(&tlsctx->conf);
  mbedtls_x509_crt_init(&tlsctx->ca);
  mbedtls_entropy_init(&tlsctx->entropy);

  /* Setup certificates.
   *    In this examples, read the certificates files from the directory
   *     which is defined by Kconfig.
   */

  dirp = opendir(CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_CERTS_PATH);
  if (dirp == NULL)
    {
      cert_info = NULL;
    }
  else
    {
      cert_info = readdir(dirp);
    }

  if (cert_info != NULL)
    {
      do
        {
          snprintf(g_tls_cert_filename, APP_TLS_CERT_FILENAME_LEN,
                   "%s/%s",
                   CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_CERTS_PATH,
                   cert_info->d_name);
          if (mbedtls_x509_crt_parse_file(&tlsctx->ca,
                                          g_tls_cert_filename) == 0)
            {
              printf("mbedtls_x509_crt_parse_file() succeed: %s\n",
                     g_tls_cert_filename);
            }
        }
      while ((cert_info = readdir(dirp)) != NULL);
    }

  if (dirp != NULL)
    {
      closedir(dirp);
    }

  mbedtls_ssl_conf_ca_chain(&tlsctx->conf, &tlsctx->ca, NULL);
  mbedtls_ssl_conf_authmode(&tlsctx->conf, MBEDTLS_SSL_VERIFY_REQUIRED);

  /* Setup mbedTLS stuff */

  if ((ret = mbedtls_ctr_drbg_seed(&tlsctx->ctr_drbg, mbedtls_entropy_func,
                                   &tlsctx->entropy,
                                   (const unsigned char *)pers,
                                   strlen(pers))) != 0)
    {
      printf("mbedtls_ctr_drbg_seed() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  if ((ret = mbedtls_ssl_config_defaults(&tlsctx->conf,
                                         MBEDTLS_SSL_IS_CLIENT,
                                         MBEDTLS_SSL_TRANSPORT_STREAM,
                                         MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
    {
      printf("mbedtls_ssl_config_defaults() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  mbedtls_ssl_conf_rng(&tlsctx->conf, mbedtls_ctr_drbg_random,
                       &tlsctx->ctr_drbg);

  if ((ret = mbedtls_ssl_setup(&tlsctx->ssl, &tlsctx->conf)) != 0)
    {
      printf("mbedtls_ssl_setup() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  if ((ret = mbedtls_ssl_set_hostname(&tlsctx->ssl, hostname)) != 0)
    {
      printf("mbedtls_ssl_set_hostname() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  /* Start the connection.
   *    mbedtls_net_connect execute address resolution, socket create,
   *    and connect.
   */

  if ((ret = mbedtls_net_connect(&tlsctx->server_fd,
                          hostname,
                          port,    /* The type is not integer, but string. */
                          MBEDTLS_NET_PROTO_TCP)) != 0)
    {
      printf("mbedtls_net_connect() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  mbedtls_ssl_set_bio(&tlsctx->ssl,
                      &tlsctx->server_fd,
                      mbedtls_net_send,
                      mbedtls_net_recv,
                      NULL);

  /* Do SSL handshake */

  if ((ret = mbedtls_ssl_handshake(&tlsctx->ssl)) != 0)
    {
      printf("mbedtls_ssl_handshake() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  return 0;
exit:

  /* Free mbedTLS stuff */

  mbedtls_net_free(&tlsctx->server_fd);
  mbedtls_ssl_free(&tlsctx->ssl);
  mbedtls_ssl_config_free(&tlsctx->conf);
  mbedtls_ctr_drbg_free(&tlsctx->ctr_drbg);
  mbedtls_entropy_free(&tlsctx->entropy);
  mbedtls_x509_crt_free(&tlsctx->ca);
  return -1;
}

/****************************************************************************
 * Name: wgetops_tls_send
 ****************************************************************************/

ssize_t wgetops_tls_send(FAR void *ctx, FAR const void *buf, size_t len)
{
  FAR struct wgetops_tls_context_s *tlsctx =
    (FAR struct wgetops_tls_context_s *)ctx;
  int ret;

  ret = mbedtls_ssl_write(&tlsctx->ssl, buf, len);
  if (ret < 0)
    {
      printf("mbedtls_ssl_write() fail: -0x%X\n", -ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wgetops_tls_recv
 ****************************************************************************/

ssize_t wgetops_tls_recv(FAR void *ctx, FAR void *buf, size_t len)
{
  FAR struct wgetops_tls_context_s *tlsctx =
    (FAR struct wgetops_tls_context_s *)ctx;
  int ret;

  ret = mbedtls_ssl_read(&tlsctx->ssl, (FAR void *)buf, len);
  if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
    {
      /* Peer's disconnection */

      ret = 0;
    }
  else if (ret < 0)
    {
      printf("mbedtls_ssl_read() fail: -0x%X\n", -ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wgetops_tls_close
 ****************************************************************************/

int wgetops_tls_close(FAR void *ctx)
{
  FAR struct wgetops_tls_context_s *tlsctx =
    (FAR struct wgetops_tls_context_s *)ctx;

  mbedtls_ssl_close_notify(&tlsctx->ssl);

  /* Free mbedTLS stuff */

  mbedtls_net_free(&tlsctx->server_fd);
  mbedtls_ssl_free(&tlsctx->ssl);
  mbedtls_ssl_config_free(&tlsctx->conf);
  mbedtls_ctr_drbg_free(&tlsctx->ctr_drbg);
  mbedtls_entropy_free(&tlsctx->entropy);
  mbedtls_x509_crt_free(&tlsctx->ca);

  return 0;
}

