/****************************************************************************
 * externals/sslutils/mbedtls_webclient.c
 *
 *   Copyright 2020,2021 Sony Semiconductor Solutions Corporation
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
#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <debug.h>

#include <nuttx/queue.h>

#include "mbedtls/config.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/platform.h"
#include "mbedtls/ssl.h"
#include "mbedtls/debug.h"

#include "sslutils/sslutil.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SSLUTIL_CERTVERIFY_STAT_BUFFLEN  128

/****************************************************************************
 * Private Functions prototype
 ****************************************************************************/
static int sslutil_connect(FAR void *ctx,
                           FAR const char *hostname, FAR const char *port,
                           unsigned int timeout_second,
                           FAR struct webclient_tls_connection **connp);
static ssize_t sslutil_send(FAR void *ctx,
                       FAR struct webclient_tls_connection *conn,
                       FAR const void *buf, size_t len);
static ssize_t sslutil_recv(FAR void *ctx,
                       FAR struct webclient_tls_connection *conn,
                       FAR void *buf, size_t len);
static int sslutil_close(FAR void *ctx,
                    FAR struct webclient_tls_connection *conn);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct webclient_tls_ops g_tls_ops = 
{
  .connect = sslutil_connect,
  .send    = sslutil_send,
  .recv    = sslutil_recv,
  .close   = sslutil_close,
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct webclient_tls_connection
{
  mbedtls_ssl_context ssl;
  mbedtls_ssl_config conf;
  mbedtls_net_context server_fd;
  mbedtls_ctr_drbg_context ctr_drbg;
  mbedtls_entropy_context entropy;
  mbedtls_x509_crt ca_cert;
  mbedtls_x509_crt cli_cert;
  mbedtls_pk_context cli_key;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_EXTERNALS_MBEDTLS
/****************************************************************************
 * Name: sslutil_debuglog
 ****************************************************************************/

static void sslutil_debuglog(void *ctx, int level, const char *file,
    int line, const char *str)
{
  printf("mbedTLS[%d] %s(%d): %s", level, file, line, str);
}
#endif

/****************************************************************************
 * Name: sslutil_delete_connection
 ****************************************************************************/

static void sslutil_delete_connection(struct webclient_tls_connection *conn)
{
  if (conn != NULL)
    {
      /* Free mbedTLS stuff */

      mbedtls_ssl_close_notify(&conn->ssl);
      mbedtls_net_free(&conn->server_fd);
      mbedtls_ssl_free(&conn->ssl);
      mbedtls_ssl_config_free(&conn->conf);
      mbedtls_ctr_drbg_free(&conn->ctr_drbg);
      mbedtls_entropy_free(&conn->entropy);
      mbedtls_x509_crt_free(&conn->ca_cert);
      mbedtls_x509_crt_free(&conn->cli_cert);
      mbedtls_pk_free(&conn->cli_key);

      free(conn);
    }
}

/****************************************************************************
 * Name: sslutil_create_connection
 ****************************************************************************/

static struct webclient_tls_connection *sslutil_create_connection(
    FAR struct sslutil_tls_context *tls_ctx)
{
  struct webclient_tls_connection *conn;
  int ret;

  conn = (struct webclient_tls_connection *)
    malloc(sizeof(struct webclient_tls_connection));

  if (conn != NULL)
    {
      /* Initialize mbedTLS stuff */

      mbedtls_ssl_init(&conn->ssl);
      mbedtls_net_init(&conn->server_fd);
      mbedtls_ssl_config_init(&conn->conf);
      mbedtls_ctr_drbg_init(&conn->ctr_drbg);
      mbedtls_entropy_init(&conn->entropy);
      mbedtls_x509_crt_init(&conn->ca_cert);
      mbedtls_x509_crt_init(&conn->cli_cert);
      mbedtls_pk_init(&conn->cli_key);

      ret = mbedtls_ctr_drbg_seed(&conn->ctr_drbg, mbedtls_entropy_func,
          &conn->entropy, (const unsigned char *)tls_ctx->custom_id,
          tls_ctx->custom_id ? strlen(tls_ctx->custom_id) : 0);
      if (ret != 0)
        {
          nerr("mbedtls_ssl_config_defaults() error : -0x%x\n", -ret);
          sslutil_delete_connection(conn);
          return NULL;
        }

      ret = mbedtls_ssl_config_defaults(&conn->conf, MBEDTLS_SSL_IS_CLIENT,
                                    MBEDTLS_SSL_TRANSPORT_STREAM,
                                    MBEDTLS_SSL_PRESET_DEFAULT);
      if (ret != 0)
        {
          nerr("mbedtls_ssl_config_defaults() error : -0x%x\n", -ret);
          sslutil_delete_connection(conn);
          return NULL;
        }

#ifdef CONFIG_EXTERNALS_MBEDTLS
      /* Initialize mbedTLS debug log config */

      conn->conf.f_dbg = sslutil_debuglog;
      conn->conf.p_dbg = stdout;
      mbedtls_debug_set_threshold(CONFIG_EXTERNALS_SSLUTILS_LOGLEVEL);
#endif
    }

  return conn;
}

/****************************************************************************
 * Name: setup_ca_cert
 ****************************************************************************/

static bool setup_ca_cert(FAR mbedtls_x509_crt *cert, FAR const char *file)
{
  bool ret = false;

  if (file && cert && (mbedtls_x509_crt_parse_file(cert, file) == 0))
    {
      ret = true;
    }

  return ret;
}

/****************************************************************************
 * Name: setup_all_ca_cert
 ****************************************************************************/

static bool setup_all_ca_cert(FAR mbedtls_x509_crt *cert, FAR const char *dir_path)
{
  bool ret = false;
  FAR DIR *dirp;
  FAR struct dirent *dnet;
  FAR char *fname;

  if (!dir_path)
    {
      return ret;
    }

  dirp = opendir(dir_path);
  if (dirp)
    {
      fname = (FAR char *)malloc(PATH_MAX);
      if (fname)
        {
          while((dnet = readdir(dirp)) != NULL)
            {
              snprintf(fname, PATH_MAX, "%s/%s", dir_path, dnet->d_name);
              ret |= setup_ca_cert(cert, fname);
            }
          free(fname);
        }
      closedir(dirp);
    }

  return ret;
}

/****************************************************************************
 * Name: setup_ca_certs
 ****************************************************************************/

static int setup_ca_certs(FAR struct sslutil_tls_context *tls_ctx,
    FAR struct webclient_tls_connection *conn)
{
  bool has_ca = false;

  has_ca |= setup_ca_cert(&conn->ca_cert, tls_ctx->ca_file);
  has_ca |= setup_all_ca_cert(&conn->ca_cert, tls_ctx->ca_dir);

  if (has_ca)
    {
      /* Peer must present a valid certificate,
       * handshake is aborted if verification failed.
       */

      mbedtls_ssl_conf_ca_chain(&conn->conf, &conn->ca_cert, NULL);
      mbedtls_ssl_conf_authmode(&conn->conf, MBEDTLS_SSL_VERIFY_REQUIRED);
    }
  else
    {
      /* Peer certificate is not checked */

      nwarn("peer certificate is not checked\n");
      mbedtls_ssl_conf_authmode(&conn->conf, MBEDTLS_SSL_VERIFY_NONE);
    }

  return OK;
}

/****************************************************************************
 * Name: setup_client_certs
 ****************************************************************************/

static int setup_client_certs(FAR struct sslutil_tls_context *tls_ctx,
    FAR struct webclient_tls_connection *conn)
{
  if (tls_ctx->cli_file && tls_ctx->privkey)
    {

      mbedtls_x509_crt_parse_file(&conn->cli_cert, tls_ctx->cli_file);
      mbedtls_pk_parse_keyfile(&conn->cli_key, tls_ctx->privkey, NULL);
      mbedtls_ssl_conf_own_cert(&conn->conf, &conn->cli_cert,
                                &conn->cli_key);
    }

  return OK;
}

/****************************************************************************
 * Name: start_handshake
 ****************************************************************************/

static int start_handshake(FAR struct mbedtls_ssl_context *ssl_ctx)
{
  int ret;

  while ((ret = mbedtls_ssl_handshake(ssl_ctx)) != 0)
    {
      if ((ret != MBEDTLS_ERR_SSL_WANT_READ) &&
          (ret != MBEDTLS_ERR_SSL_WANT_WRITE))
        {
          nerr("mbedtls_ssl_handshake() error : -0x%x\n", -ret);
          return -1;
        }
    }
  return 0;
}

/****************************************************************************
 * Name: verify_handshake_result
 ****************************************************************************/

static int verify_handshake_result(FAR struct mbedtls_ssl_context *ssl_ctx)
{
  int ret;

  ret = mbedtls_ssl_get_verify_result(ssl_ctx);
  if (ret != 0)
    {
      char buf[SSLUTIL_CERTVERIFY_STAT_BUFFLEN];
      mbedtls_x509_crt_verify_info(buf, SSLUTIL_CERTVERIFY_STAT_BUFFLEN, " ", ret);
      nerr("Failed to verify peer certificates: %s\n", buf);
      return -1;
    }
  return 0;
}

/****************************************************************************
 * Name: sslutil_sslconnect
 ****************************************************************************/

static int sslutil_connect(FAR void *ctx,
                           FAR const char *hostname, FAR const char *port,
                           unsigned int timeout_second,
                           FAR struct webclient_tls_connection **connp)
{
  int ret = 0;
  FAR struct webclient_tls_connection *conn;
  FAR struct sslutil_tls_context *tls_ctx
    = (FAR struct sslutil_tls_context *)ctx;

  conn = sslutil_create_connection(tls_ctx);
  if (conn != NULL)
    {
      /* Setup Root CA certificates. */

      ret = setup_ca_certs(tls_ctx, conn);
      if (ret != 0)
        {
          goto err_with_clean;
        }

      /* Setup client certificates. */

      ret = setup_client_certs(tls_ctx, conn);
      if (ret != 0)
        {
          goto err_with_clean;
        }

      /* Set rundom number generator */

      mbedtls_ssl_conf_rng(&conn->conf, mbedtls_ctr_drbg_random,
                           &conn->ctr_drbg);

      /* Set timeout */

      mbedtls_ssl_conf_read_timeout(&conn->conf, timeout_second * 1000);
      mbedtls_ssl_setup(&conn->ssl, &conn->conf);

      /* Set hostname */

      ret = mbedtls_ssl_set_hostname(&conn->ssl, hostname);
      if (ret != 0)
        {
          nerr("mbedtls_ssl_set_hostname() error : -0x%x\n", -ret);
          goto err_with_clean;
        }

      /* Start the connection.
       * mbedtls_net_connect execute address resolution, socket create,
       * and connect. */

      ret = mbedtls_net_connect(&conn->server_fd, hostname, port,
                                MBEDTLS_NET_PROTO_TCP);
      if (ret != 0)
        {
          nerr("mbedtls_net_connect() error : -0x%x\n", -ret);
          goto err_with_clean;
        }

      /* Set transaction methods */

      mbedtls_ssl_set_bio(&conn->ssl, &conn->server_fd,
                          mbedtls_net_send, mbedtls_net_recv,
                          mbedtls_net_recv_timeout);

      ninfo("Performing the SSL/TLS handshake\n");

      ret = start_handshake(&conn->ssl);
      if (ret != 0)
        {
          goto err_with_clean;
        }

      ret = verify_handshake_result(&conn->ssl);
      if (ret != 0)
        {
          goto err_with_clean;
        }

      *connp = conn;
    }
  else
    {
      ret = -ENOMEM;
    }

  return ret;

err_with_clean:
  sslutil_delete_connection(conn);

  return ret;
}

/****************************************************************************
 * Name: sslutil_sslsend
 ****************************************************************************/

static ssize_t sslutil_send(FAR void *ctx,
                       FAR struct webclient_tls_connection *conn,
                       FAR const void *buf, size_t len)
{
  int ret;

  while ((ret = mbedtls_ssl_write(&conn->ssl, buf, len)) < 0)
    {
      /* If mbedtls_ssl_write returns MBEDTLS_ERR_SSL_WANT_READ or
       * MBEDTLS_ERR_SSL_WANT_WRITE, need to retry to call
       * mbedtls_ssl_write.
       */

      if ((ret != MBEDTLS_ERR_SSL_WANT_READ) &&
          (ret != MBEDTLS_ERR_SSL_WANT_WRITE))
        {
          /* The return code is defined by -0xXXXX in "mbedtls/ssl.h ".
           * e.g. MBEDTLS_ERR_SSL_TIMEOUT: -0x6800
           */

          nerr("mbedtls_ssl_write() error : -0x%x\n", -ret);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sslutil_sslrecv
 ****************************************************************************/

static ssize_t sslutil_recv(FAR void *ctx,
                       FAR struct webclient_tls_connection *conn,
                       FAR void *buf, size_t len)
{
  int ret;

  while ((ret = mbedtls_ssl_read(&conn->ssl, buf, len)) < 0)
    {
      /* If mbedtls_ssl_read returns MBEDTLS_ERR_SSL_WANT_READ or
       * MBEDTLS_ERR_SSL_WANT_WRITE, need to retry to call
       * mbedtls_ssl_read.
       */

      if ((ret != MBEDTLS_ERR_SSL_WANT_READ) &&
          (ret != MBEDTLS_ERR_SSL_WANT_WRITE))
        {
          if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
            {
              /* WebClient cannot handle this error code.
               * Return 0 to exit as same as normal situation.
               * Reference:
               *   https://github.com/ARMmbed/mbedtls
               *   /blob/development/programs/ssl/ssl_client1.c
               */

              return 0;
            }

          /* The return code is defined by -0xXXXX in "mbedtls/ssl.h ".
           * e.g. MBEDTLS_ERR_SSL_TIMEOUT: -0x6800
           */

          nerr("mbedtls_ssl_read() error : -0x%x\n", -ret);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sslutil_sslclose
 ****************************************************************************/

static int sslutil_close(FAR void *ctx,
                    FAR struct webclient_tls_connection *conn)
{

  sslutil_delete_connection(conn);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_webclient_tlsops
 *
 * Description:
 *   Implementation of get_webclient_tlsops() by mbedTLS
 *
 ****************************************************************************/

struct webclient_tls_ops *sslutil_webclient_tlsops(void)
{
  return &g_tls_ops;
}
