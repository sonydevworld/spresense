/****************************************************************************
 * externals/sslutils/mbedtls_webclient.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <queue.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>

#include <netutils/ssl_connection.h>

#include "mbedtls/config.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/platform.h"
#include "mbedtls/ssl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SSLUTIL_PORT_STRSIZE 8
#define SSLUTIL_CERTVERIFY_STAT_BUFFLEN  128

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sslutil_ssl_ctx_s
{
  mbedtls_ssl_context ssl;
  mbedtls_ssl_config conf;
  mbedtls_net_context server_fd;
  mbedtls_ctr_drbg_context ctr_drbg;
  mbedtls_entropy_context entropy;
  mbedtls_x509_crt ca_cert;
  mbedtls_x509_crt cli_cert;
  mbedtls_pk_context cli_key;
  const char *ca_certs_dir;
  const char *ca_certs_file;
  const char *cli_certs_file;
  const char *private_key_file;
};

struct sslutil_sock_s
{
  sq_entry_t node;
  int ndx;
  int sockfd;
  int use_ssl;
  FAR struct sslutil_ssl_ctx_s *ssl;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sq_queue_t g_sslutil_sockqueue = {};
static int g_sslutil_sockndx = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sslutil_socknew
 ****************************************************************************/

static FAR struct sslutil_sock_s *sslutil_socknew(int use_ssl)
{
  FAR struct sslutil_sock_s *sock;

  sock = (FAR struct sslutil_sock_s *)
           calloc(1, sizeof(struct sslutil_sock_s));
  if (sock)
    {
      sock->ndx = g_sslutil_sockndx++ & 0x7fffffff;
      sock->sockfd = -1;
      sock->use_ssl = use_ssl;
      if (use_ssl)
        {
          sock->ssl = (FAR struct sslutil_ssl_ctx_s *)
                        calloc(1, sizeof(struct sslutil_ssl_ctx_s));
          sock->ssl->ca_certs_dir =
            CONFIG_EXTERNALS_MBEDTLS_DEFAULT_CERTS_PATH;
        }
      sq_addlast(&sock->node, &g_sslutil_sockqueue);
    }

  return sock;
}

/****************************************************************************
 * Name: sslutil_sockfree
 ****************************************************************************/

static int sslutil_sockfree(FAR struct sslutil_sock_s *sock)
{
  if (!sock)
    {
      return -EINVAL;
    }
  sq_rem(&sock->node, &g_sslutil_sockqueue);
  if (sock->ssl)
    {
      free(sock->ssl);
    }
  free(sock);

  return 0;
}

/****************************************************************************
 * Name: sslutil_sockget
 ****************************************************************************/

static FAR struct sslutil_sock_s *sslutil_sockget(int ndx)
{
  FAR struct sslutil_sock_s *sock = NULL;

  sock = (FAR struct sslutil_sock_s *)sq_peek(&g_sslutil_sockqueue);

  while (sock != NULL)
    {
      if (ndx == sock->ndx)
        {
          break;
        }
      sock = (FAR struct sslutil_sock_s *)sq_next(&sock->node);
    }

  return sock;
}

/****************************************************************************
 * Name: sslutil_doconnect
 ****************************************************************************/

static int sslutil_doconnect(FAR struct sslutil_sock_s *sock,
                                  FAR const char *host, uint16_t port)
{
  struct addrinfo hints;
  struct addrinfo *ainfo = NULL;
  char port_char[SSLUTIL_PORT_STRSIZE] = {0};
  int ret;
  struct timeval tv;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  snprintf(port_char, SSLUTIL_PORT_STRSIZE, "%d", port);

  ret = getaddrinfo(host, port_char, &hints, &ainfo);
  if (ret != 0)
    {
      /* Could not resolve host (or malformed IP address) */

      nerr("getaddrinfo() error : %d\n", ret);
      return -EHOSTUNREACH;
    }

  sock->sockfd = socket(ainfo->ai_family, ainfo->ai_socktype,
                        ainfo->ai_protocol);
  if (sock->sockfd < 0)
    {
      ret = -errno;

      nerr("ERROR: socket failed: %d\n", errno);
      freeaddrinfo(ainfo);
      return ret;
    }

  /* Set send and receive timeout values */

  tv.tv_sec  = CONFIG_EXTERNALS_MBEDTLS_DEFAULT_TIMEOUT;
  tv.tv_usec = 0;

  setsockopt(sock->sockfd, SOL_SOCKET, SO_RCVTIMEO, (FAR const void *)&tv,
             sizeof(struct timeval));
  setsockopt(sock->sockfd, SOL_SOCKET, SO_SNDTIMEO, (FAR const void *)&tv,
             sizeof(struct timeval));

  /* Connect to server.  First we have to set some fields in the
   * 'server' address structure.  The system will assign me an arbitrary
   * local port that is not in use. */

  ret = connect(sock->sockfd, ainfo->ai_addr, ainfo->ai_addrlen);
  if (ret < 0)
    {
      ret = -errno;

      nerr("ERROR: connect failed: %d\n", errno);
      freeaddrinfo(ainfo);
      return ret;
    }

  freeaddrinfo(ainfo);

  return ret;
}

/****************************************************************************
 * Name: sslutil_dosend
 ****************************************************************************/

static int sslutil_dosend(FAR struct sslutil_sock_s *sock,
                               FAR const void *buf, size_t len, int flags)
{
  int ret;

  ret = send(sock->sockfd, buf, len, flags);
  if (ret < 0)
    {
      ret = -errno;
      nerr("ERROR: send failed: %d\n", errno);
    }

  return ret;
}

/****************************************************************************
 * Name: sslutil_dorecv
 ****************************************************************************/

static int sslutil_dorecv(FAR struct sslutil_sock_s *sock,
                               FAR void *buf, size_t len, int flags)
{
  int ret;

  ret = recv(sock->sockfd, buf, len, flags);
  if (ret < 0)
    {
      ret = -errno;
      nerr("ERROR: recv failed: %d\n", errno);
    }

  return ret;
}

/****************************************************************************
 * Name: sslutil_doclose
 ****************************************************************************/

static int sslutil_doclose(FAR struct sslutil_sock_s *sock)
{
  int ret;

  ret = close(sock->sockfd);
  if (ret < 0)
    {
      ret = -errno;
      nerr("ERROR: close failed: %d\n", errno);
    }

  return ret;
}

/****************************************************************************
 * Name: sslutil_sslsockinit
 ****************************************************************************/

static int sslutil_sslsockinit(struct sslutil_ssl_ctx_s *ssl)
{
  /* Initialize mbedTLS stuff */

  mbedtls_net_init(&ssl->server_fd);
  mbedtls_ssl_init(&ssl->ssl);
  mbedtls_ssl_config_init(&ssl->conf);
  mbedtls_ctr_drbg_init(&ssl->ctr_drbg);
  mbedtls_entropy_init(&ssl->entropy);

  return 0;
}

/****************************************************************************
 * Name: sslutil_sslsockfin
 ****************************************************************************/

static int sslutil_sslsockfin(struct sslutil_ssl_ctx_s *ssl)
{
  /* Free mbedTLS stuff */

  mbedtls_ssl_close_notify(&ssl->ssl);
  mbedtls_net_free(&ssl->server_fd);
  mbedtls_ssl_free(&ssl->ssl);
  mbedtls_ssl_config_free(&ssl->conf);
  mbedtls_ctr_drbg_free(&ssl->ctr_drbg);
  mbedtls_entropy_free(&ssl->entropy);

  return 0;
}

/****************************************************************************
 * Name: sslutil_sslconnect
 ****************************************************************************/

static int sslutil_sslconnect(struct sslutil_sock_s *sock,
                                   FAR const char *host, uint16_t port)
{
  FAR struct sslutil_ssl_ctx_s *ssl = sock->ssl;
  FAR static const char *pers = "tls_test";
  FAR char *certs_filename;
  FAR DIR *dirp = NULL;  /* Pointer to directory for certification files */
  FAR struct dirent *cert_dirent = NULL;
  int ret;
  char port_char[SSLUTIL_PORT_STRSIZE] = {0};
  char *buf;
  bool verify_ca = false;

  sslutil_sslsockinit(ssl);

  ret = mbedtls_ctr_drbg_seed(&ssl->ctr_drbg, mbedtls_entropy_func,
                              &ssl->entropy, (const unsigned char *)pers,
                              strlen(pers));
  if (ret != 0)
    {
      return ret;
    }

  ret = mbedtls_ssl_config_defaults(&ssl->conf, MBEDTLS_SSL_IS_CLIENT,
                                    MBEDTLS_SSL_TRANSPORT_STREAM,
                                    MBEDTLS_SSL_PRESET_DEFAULT);
  if (ret != 0)
    {
      nerr("mbedtls_ssl_config_defaults() error : -0x%x\n", -ret);
      return ret;
    }

  /* Setup CA certificates. */

  if (ssl->ca_certs_file)
    {
      mbedtls_x509_crt_init(&ssl->ca_cert);
      mbedtls_x509_crt_parse_file(&ssl->ca_cert,
                                  ssl->ca_certs_file);
      verify_ca = true;
    }
  else if (ssl->ca_certs_dir)
    {
      dirp = opendir(ssl->ca_certs_dir);
      if (dirp != NULL)
        {
          cert_dirent = readdir(dirp);
          if (cert_dirent != NULL)
            {
              certs_filename = (FAR char *)malloc(PATH_MAX);
              if (certs_filename)
                {
                  mbedtls_x509_crt_init(&ssl->ca_cert);

                  do
                    {
                      memset(certs_filename, 0, PATH_MAX);

                      snprintf(certs_filename, PATH_MAX,
                               "%s/%s",
                               ssl->ca_certs_dir, cert_dirent->d_name);
                      if (0 == mbedtls_x509_crt_parse_file(&ssl->ca_cert,
                                                  certs_filename))
                        {
                          verify_ca = true;
                        }
                    }
                  while ((cert_dirent = readdir(dirp)) != NULL);

                  free(certs_filename);
                }
            }
          closedir(dirp);
        }
    }

  if (verify_ca)
    {
      /* Peer must present a valid certificate,
       * handshake is aborted if verification failed.
       */

      mbedtls_ssl_conf_ca_chain(&ssl->conf, &ssl->ca_cert, NULL);
      mbedtls_ssl_conf_authmode(&ssl->conf, MBEDTLS_SSL_VERIFY_REQUIRED);
    }
  else
    {
      /* Peer certificate is not checked */

      nwarn("peer certificate is not checked\n");
      mbedtls_ssl_conf_authmode(&ssl->conf, MBEDTLS_SSL_VERIFY_NONE);
    }

  /* Setup client certificates. */

  if (ssl->cli_certs_file && ssl->private_key_file)
    {
      mbedtls_x509_crt_init(&ssl->cli_cert);
      mbedtls_pk_init(&ssl->cli_key);

      mbedtls_x509_crt_parse_file(&ssl->cli_cert,
                                  ssl->cli_certs_file);
      mbedtls_pk_parse_keyfile(&ssl->cli_key, ssl->private_key_file,
                               NULL);
      mbedtls_ssl_conf_own_cert(&ssl->conf, &ssl->cli_cert,
                                &ssl->cli_key);
    }

  mbedtls_ssl_conf_rng(&ssl->conf, mbedtls_ctr_drbg_random,
                       &ssl->ctr_drbg);
  mbedtls_ssl_conf_read_timeout(&ssl->conf,
    CONFIG_EXTERNALS_MBEDTLS_DEFAULT_TIMEOUT * 1000);
  mbedtls_ssl_setup(&ssl->ssl, &ssl->conf);
  ret = mbedtls_ssl_set_hostname(&ssl->ssl, host);
  if (ret != 0)
    {
      nerr("mbedtls_ssl_set_hostname() error : -0x%x\n", -ret);
      return ret;
    }

  snprintf(port_char, SSLUTIL_PORT_STRSIZE, "%d", port);

  /* Start the connection.
   * mbedtls_net_connect execute address resolution, socket create,
   * and connect. */

  ret = mbedtls_net_connect(&ssl->server_fd, host, port_char,
                            MBEDTLS_NET_PROTO_TCP);
  if (ret != 0)
    {
      nerr("mbedtls_net_connect() error : -0x%x\n", -ret);
      return ret;
    }

  mbedtls_ssl_set_bio(&ssl->ssl, &ssl->server_fd,
                      mbedtls_net_send, mbedtls_net_recv, NULL);

  ninfo("Performing the SSL/TLS handshake\n");

  /* Do SSL handshake */

  while ((ret = mbedtls_ssl_handshake(&ssl->ssl)) != 0)
    {
      if ((ret != MBEDTLS_ERR_SSL_WANT_READ) &&
          (ret != MBEDTLS_ERR_SSL_WANT_WRITE))
        {
          nerr("mbedtls_ssl_handshake() error : -0x%x\n", -ret);
          return ret;
        }
    }

  ret = mbedtls_ssl_get_verify_result(&ssl->ssl);
  if (ret != 0)
    {
      buf = calloc(1, SSLUTIL_CERTVERIFY_STAT_BUFFLEN);
      if (!buf)
        {
          nerr("failed to allocate memory\n");
          return -ENOMEM;
        }
      mbedtls_x509_crt_verify_info(buf, SSLUTIL_CERTVERIFY_STAT_BUFFLEN, " ", ret);
      nerr("Failed to verify peer certificates: %s\n", buf);
      free(buf);
      return -1;
    }

  if (ssl->ca_certs_file || ssl->ca_certs_dir)
    {
      mbedtls_x509_crt_free(&ssl->ca_cert);
    }

  if (ssl->cli_certs_file && ssl->private_key_file)
    {
      mbedtls_x509_crt_free(&ssl->cli_cert);
      mbedtls_pk_free(&ssl->cli_key);
    }

  return 0;
}

/****************************************************************************
 * Name: sslutil_sslsend
 ****************************************************************************/

static int sslutil_sslsend(struct sslutil_sock_s *sock,
                                FAR const void *buf, size_t len, int flags)
{
  FAR struct sslutil_ssl_ctx_s *ssl = sock->ssl;
  int ret;

  while ((ret = mbedtls_ssl_write(&ssl->ssl, buf, len)) <= 0)
    {
      if ((ret != MBEDTLS_ERR_SSL_WANT_READ) &&
          (ret != MBEDTLS_ERR_SSL_WANT_WRITE))
        {
          nerr("mbedtls_ssl_write() error : -0x%x\n", -ret);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sslutil_sslrecv
 ****************************************************************************/

static int sslutil_sslrecv(struct sslutil_sock_s *sock,
                                FAR void *buf, size_t len, int flags)
{
  FAR struct sslutil_ssl_ctx_s *ssl = sock->ssl;
  int ret;

  while ((ret = mbedtls_ssl_read(&ssl->ssl, buf, len)) <= 0)
    {
      if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
        {
          return 0;
        }
      if ((ret != MBEDTLS_ERR_SSL_WANT_READ) &&
          (ret != MBEDTLS_ERR_SSL_WANT_WRITE))
        {
          nerr("mbedtls_ssl_read() error : -0x%x\n", -ret);
          return ret;
        }
     }

  return ret;
}

/****************************************************************************
 * Name: sslutil_sslclose
 ****************************************************************************/

static int sslutil_sslclose(struct sslutil_sock_s *sock)
{
  FAR struct sslutil_ssl_ctx_s *ssl = sock->ssl;

  sslutil_sslsockfin(ssl);

  return 0;
}

/****************************************************************************
 * Name: sslutil_connect
 ****************************************************************************/

static int sslutil_connect(int use_ssl, FAR const char *host,
                                uint16_t port)
{
  FAR struct sslutil_sock_s *sock;
  int ret;

  sock = sslutil_socknew(use_ssl);
  if (!sock)
    {
      return -ENFILE;
    }

  if (use_ssl)
    {
      /* Perform SSL/TLS connect */

      ret = sslutil_sslconnect(sock, host, port);
      if (ret < 0)
        {
          nerr("sslutil_sslconnect() error : %d\n", ret);
          sslutil_sockfree(sock);
        }
    }
  else
    {
      /* Perform connect */

      ret = sslutil_doconnect(sock, host, port);
      if (ret < 0)
        {
          nerr("sslutil_doconnect() error : %d\n", ret);
          sslutil_sockfree(sock);
        }
    }

  if (ret >= 0)
    {
      /* If successful, the index is returned so that the sslutil_sock_s
       * context can be obtained when subsequent called
       * such as sslutil_send and sslutil_recv. */

      ret = sock->ndx;
    }

  return ret;
}

/****************************************************************************
 * Name: sslutil_send
 ****************************************************************************/

static ssize_t sslutil_send(int fd, FAR const void *buf, size_t len,
                                 int flags)
{
  FAR struct sslutil_sock_s *sock;
  ssize_t ret;

  sock = sslutil_sockget(fd);
  if (!sock)
    {
      return -EBADF;
    }

  if (sock->use_ssl)
    {
      /* Perform SSL/TLS send */

      ret = sslutil_sslsend(sock, buf, len, flags);
      if (ret < 0)
        {
          nerr("sslutil_sslsend() error : %d\n", ret);
        }
    }
  else
    {
      /* Perform send */

      ret = sslutil_dosend(sock, buf, len, flags);
      if (ret < 0)
        {
          nerr("sslutil_dosend() error : %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sslutil_recv
 ****************************************************************************/

static ssize_t sslutil_recv(int fd, FAR void *buf, size_t len, int flags)
{
  FAR struct sslutil_sock_s *sock;
  ssize_t ret;

  sock = sslutil_sockget(fd);
  if (!sock)
    {
      return -EBADF;
    }

  if (sock->use_ssl)
    {
      /* Perform SSL/TLS recv */

      ret = sslutil_sslrecv(sock, buf, len, flags);
      if (ret < 0)
        {
          nerr("sslutil_sslrecv() error : %d\n", ret);
        }
    }
  else
    {
      /* Perform recv */

      ret = sslutil_dorecv(sock, buf, len, flags);
      if (ret < 0)
        {
          nerr("sslutil_dorecv() error : %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sslutil_close
 ****************************************************************************/

static ssize_t sslutil_close(int fd)
{
  FAR struct sslutil_sock_s *sock;
  int ret;

  sock = sslutil_sockget(fd);
  if (!sock)
    {
      return -EBADF;
    }

  if (sock->use_ssl)
    {
      /* Perform SSL/TLS close */

      ret = sslutil_sslclose(sock);
      if (ret < 0)
        {
          nerr("sslutil_sslclose() error : %d\n", ret);
        }
    }
  else
    {
      /* Perform close */

      ret = sslutil_doclose(sock);
      if (ret < 0)
        {
          nerr("sslutil_doclose() error : %d\n", ret);
        }
    }

  sslutil_sockfree(sock);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_sslsock_connection_methods
 *
 * Description:
 *   Implementation of get_sslsock_connection_methods() by mbedTLS
 *
 ****************************************************************************/

void get_sslsocket_methods(struct sock_methods_s *methods)
{
  methods->connect = sslutil_connect;
  methods->send = sslutil_send;
  methods->recv = sslutil_recv;
  methods->close = sslutil_close;
}
