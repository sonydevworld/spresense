/****************************************************************************
 * examples/lte_hibernation_wake_socket/lte_hibernation_wake_socket_main.c
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
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <dirent.h>
#include <netdb.h>

#include <arch/chip/pm.h>
#include <nuttx/timers/rtc.h>
#include <arch/board/board.h>

#include <netutils/netlib.h>

#include "lte/lte_api.h"
#include "lte_connection.h"
#include "wget_utils.h"
#include "wget_ops.h"
#include "file_utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SAVE_DIR CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_SAVECONTEXT_PATH

#define LTE_HIBERNATION_CONTEXT_PATH              SAVE_DIR "/.ltectx"
#define LTE_HIBERNATION_SOCK_CONTEXT_PATH         SAVE_DIR "/.sockctx"
#define LTE_HIBERNATION_TLS_NET_CONTEXT_PATH      SAVE_DIR "/.tls_net"
#define LTE_HIBERNATION_TLS_X509CRT_CONTEXT_PATH  SAVE_DIR "/.tls_x509crt"
#define LTE_HIBERNATION_TLS_ENTROPY_CONTEXT_PATH  SAVE_DIR "/.tls_entropy"
#define LTE_HIBERNATION_TLS_CTRDRBG_CONTEXT_PATH  SAVE_DIR "/.tls_ctrdrbg"
#define LTE_HIBERNATION_TLS_SSL_CONTEXT_PATH      SAVE_DIR "/.tls_ssl"
#define LTE_HIBERNATION_TLS_SSLCONF_CONTEXT_PATH  SAVE_DIR "/.tls_sslconf"
#define LTE_HIBERNATION_INTERVAL                  (30)  /* seconds */

/* APN settings */

#ifdef CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_APN_IPTYPE_IPV6
#  define APP_APN_IPTYPE   LTE_IPTYPE_V6
#elif defined CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_APN_IPTYPE_IPV4V6
#  define APP_APN_IPTYPE   LTE_IPTYPE_V4V6
#else
#  define APP_APN_IPTYPE   LTE_IPTYPE_V4
#endif

#ifdef CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_APN_AUTHTYPE_PAP
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_PAP
#elif defined CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_APN_AUTHTYPE_CHAP
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_CHAP
#else
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_NONE
#endif

#define SD_MOUNT_POINT "/mnt/sd0"
#define ALARM_DEVPATH "/dev/rtc0"
#define ALARM_SIGNO 1

#define APP_IOBUFFER_LEN         512
#define APP_HTTPS_WGET_URL       "https://httpbin.org/delay/10"
#define APP_HTTP_WGET_URL        "http://httpbin.org/delay/10"

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#define MATCH_STRING(str1, str2) ((strlen(str1) == strlen(str2)) && \
                                  (strncmp(str1, str2, strlen(str2)) == 0))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct wgetops_sock_context_s g_sock_context;
static struct wgetops_tls_context_s g_tls_context;
static unsigned char g_iobuffer[APP_IOBUFFER_LEN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avail_sdcard
 ****************************************************************************/

static bool avail_sdcard(void)
{
  int ret;
  int retry;
  struct stat tmp;

  /* In case that SD card isn't inserted, it times out at max 2 sec */

  for (retry = 0; retry < 20; retry++)
    {
      ret = stat(SD_MOUNT_POINT, &tmp);
      if (ret == 0)
        {
          return true;
        }

      usleep(100 * 1000); /* 100 msec */
    }

  return false;
}

/****************************************************************************
 * Name: save_sock_context
 ****************************************************************************/

static int save_sock_context(int sockfd, FAR char *filename)
{
  int ret;
  FAR uint8_t *buff;
  struct socket_context_s sc =
    {
      0
    };

  /* ctx_size = 0 means get the context size. The obtained context size is
   * stored in ctx_size.
   */

  sc.ctx_size = 0;
  ret = ioctl(sockfd, SIOCGETCONTEXT, (unsigned long)&sc);
  if (ret < 0)
    {
      printf("SIOCGETCONTEXT failed:%d\n", errno);
      return -1;
    }

  buff = (FAR uint8_t *)malloc(sc.ctx_size);
  if (!buff)
    {
      printf("Failed allocate memory. size = %d\n", sc.ctx_size);
      return -1;
    }

  sc.ctx = (FAR struct socket_ctx_data_s *)buff;
  ret = ioctl(sockfd, SIOCGETCONTEXT, (unsigned long)&sc);
  if (ret < 0)
    {
      printf("SIOCGETCONTEXT failed:%d\n", errno);
      free(buff);
      return -1;
    }

  ret = app_save_file(filename, buff, sc.ctx_size);
  if (ret < 0)
    {
      free(buff);
      return ret;
    }

  free(buff);

  printf("Save socket context to file %s successfully\n", filename);

  return ret;
}

/****************************************************************************
 * Name: load_sock_context
 ****************************************************************************/

static int load_sock_context(FAR char *filename)
{
  int sockfd;
  struct socket_context_s sc;
  int ret = -1;
  ssize_t size;
  FAR struct socket_ctx_data_s *sdata;

  size = app_read_file(filename, g_iobuffer, sizeof(g_iobuffer));
  if (size < 0)
    {
      printf("Failed to read %s file \n", filename);
      return ret;
    }

  sdata = (FAR struct socket_ctx_data_s *)g_iobuffer;
  printf("socket context domain:%d\n", sdata->s_domain);
  printf("socket context type:%d\n", sdata->s_type);
  printf("socket context protocol:%d\n", sdata->s_proto);

  sockfd = socket(sdata->s_domain, sdata->s_type, sdata->s_proto);
  if (sockfd < 0)
    {
      printf("socket failed:%d\n", errno);
      return -1;
    }

  sc.ctx = (FAR struct socket_ctx_data_s *)g_iobuffer;
  sc.ctx_size = size;
  ret = ioctl(sockfd, SIOCSETCONTEXT, (unsigned long)&sc);
  if (ret < 0)
    {
      printf("SIOCSETCONTEXT failed:%d\n", errno);
      close(sockfd);
      return -1;
    }

  printf("Load socket context from file %s successfully\n", filename);

  return sockfd;
}

/****************************************************************************
 * Name: save_tls_contexts
 ****************************************************************************/

static int save_tls_contexts(FAR struct wgetops_tls_context_s *ctx)
{
  int ret;
  size_t size;
  uint8_t *buff;

  /* Select the largest context size */

  size = mbedtls_net_getctxsize(&ctx->server_fd);
  size = MAX(size, mbedtls_x509_crt_getctxsize(&ctx->ca));
  size = MAX(size, mbedtls_entropy_getctxsize(&ctx->entropy));
  size = MAX(size, mbedtls_ctr_drbg_getctxsize(&ctx->ctr_drbg));
  size = MAX(size, mbedtls_ssl_getctxsize(&ctx->ssl));
  size = MAX(size, mbedtls_ssl_config_getctxsize(&ctx->conf));

  buff = (uint8_t *)malloc(size);
  if (buff == NULL)
    {
      printf("Failed to allocate memory. size = %d\n", size);
      return -ENOMEM;
    }

  /* Get context of mbedtls_net_context */

  size = mbedtls_net_getctxsize(&ctx->server_fd);
  ret = mbedtls_net_getctx(&ctx->server_fd, buff, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_net_getctx() :%d\n", ret);
      goto exit;
    }

  ret = app_save_file(LTE_HIBERNATION_TLS_NET_CONTEXT_PATH, buff, size);
  if (ret < 0)
    {
      goto exit;
    }

  /* Get context of mbedtls_x509_crt */

  size = mbedtls_x509_crt_getctxsize(&ctx->ca);
  ret = mbedtls_x509_crt_getctx(&ctx->ca, buff, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_x509_crt_getctx() :%d\n", ret);
      goto exit;
    }

  ret = app_save_file(LTE_HIBERNATION_TLS_X509CRT_CONTEXT_PATH, buff, size);
  if (ret < 0)
    {
      goto exit;
    }

  /* Get context of mbedtls_entropy_context */

  size = mbedtls_entropy_getctxsize(&ctx->entropy);
  ret = mbedtls_entropy_getctx(&ctx->entropy, buff, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_entropy_getctx() :%d\n", ret);
      goto exit;
    }

  ret = app_save_file(LTE_HIBERNATION_TLS_ENTROPY_CONTEXT_PATH, buff, size);
  if (ret < 0)
    {
      goto exit;
    }

  /* Get context of mbedtls_ctr_drbg_context */

  size = mbedtls_ctr_drbg_getctxsize(&ctx->ctr_drbg);
  ret = mbedtls_ctr_drbg_getctx(&ctx->ctr_drbg, buff, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_ctr_drbg_getctx() :%d\n", ret);
      goto exit;
    }

  ret = app_save_file(LTE_HIBERNATION_TLS_CTRDRBG_CONTEXT_PATH, buff, size);
  if (ret < 0)
    {
      goto exit;
    }

  /* Get context of mbedtls_ssl_context */

  size = mbedtls_ssl_getctxsize(&ctx->ssl);
  ret = mbedtls_ssl_getctx(&ctx->ssl, buff, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_ssl_getctx() :%d\n", ret);
      goto exit;
    }

  ret = app_save_file(LTE_HIBERNATION_TLS_SSL_CONTEXT_PATH, buff, size);
  if (ret < 0)
    {
      goto exit;
    }

  /* Get context of mbedtls_ssl_config */

  size = mbedtls_ssl_config_getctxsize(&ctx->conf);
  ret = mbedtls_ssl_config_getctx(&ctx->conf, buff, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_ssl_config_getctx() :%d\n", ret);
      goto exit;
    }

  ret = app_save_file(LTE_HIBERNATION_TLS_SSLCONF_CONTEXT_PATH, buff, size);
  if (ret < 0)
    {
      goto exit;
    }

  free(buff);

  printf("Save secure socket context to files successfully\n");

  return 0;

exit:
  free(buff);
  remove(LTE_HIBERNATION_TLS_NET_CONTEXT_PATH);
  remove(LTE_HIBERNATION_TLS_X509CRT_CONTEXT_PATH);
  remove(LTE_HIBERNATION_TLS_ENTROPY_CONTEXT_PATH);
  remove(LTE_HIBERNATION_TLS_CTRDRBG_CONTEXT_PATH);
  remove(LTE_HIBERNATION_TLS_SSL_CONTEXT_PATH);
  remove(LTE_HIBERNATION_TLS_SSLCONF_CONTEXT_PATH);

  return ret;
}

/****************************************************************************
 * Name: load_tls_contexts
 ****************************************************************************/

static int load_tls_contexts(FAR struct wgetops_tls_context_s *ctx)
{
  ssize_t size;
  int ret = -1;

  /* Load context of mbedtls_net_context from file */

  size = app_read_file(LTE_HIBERNATION_TLS_NET_CONTEXT_PATH,
                       g_iobuffer, sizeof(g_iobuffer));
  if (size < 0)
    {
      goto exit;
    }

  ret = mbedtls_net_setctx(&ctx->server_fd, g_iobuffer, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_net_setctx() :%d\n", ret);
      goto exit;
    }

  /* Load context of mbedtls_x509_crt from file */

  size = app_read_file(LTE_HIBERNATION_TLS_X509CRT_CONTEXT_PATH,
                       g_iobuffer, sizeof(g_iobuffer));
  if (size < 0)
    {
      goto exit;
    }

  ret = mbedtls_x509_crt_setctx(&ctx->ca, g_iobuffer, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_x509_crt_setctx() :%d\n", ret);
      goto exit;
    }

  /* Load context of mbedtls_entropy_context from file */

  size = app_read_file(LTE_HIBERNATION_TLS_ENTROPY_CONTEXT_PATH,
                       g_iobuffer, sizeof(g_iobuffer));
  if (size < 0)
    {
      goto exit;
    }

  ret = mbedtls_entropy_setctx(&ctx->entropy, g_iobuffer, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_entropy_setctx() :%d\n", ret);
      goto exit;
    }

  /* Load context of mbedtls_ctr_drbg_context from file */

  size = app_read_file(LTE_HIBERNATION_TLS_CTRDRBG_CONTEXT_PATH,
                       g_iobuffer, sizeof(g_iobuffer));
  if (size < 0)
    {
      goto exit;
    }

  ret = mbedtls_ctr_drbg_setctx(&ctx->ctr_drbg, g_iobuffer, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_ctr_drbg_setctx() :%d\n", ret);
      goto exit;
    }

  /* Load context of mbedtls_ssl_context from file */

  size = app_read_file(LTE_HIBERNATION_TLS_SSL_CONTEXT_PATH,
                       g_iobuffer, sizeof(g_iobuffer));
  if (size < 0)
    {
      goto exit;
    }

  ret = mbedtls_ssl_setctx(&ctx->ssl, g_iobuffer, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_ssl_setctx() :%d\n", ret);
      goto exit;
    }

  /* Load context of mbedtls_ssl_config from file */

  size = app_read_file(LTE_HIBERNATION_TLS_SSLCONF_CONTEXT_PATH,
                       g_iobuffer, sizeof(g_iobuffer));
  if (size < 0)
    {
      goto exit;
    }

  ret = mbedtls_ssl_config_setctx(&ctx->conf, g_iobuffer, size);
  if (ret < 0)
    {
      printf("Failed to mbedtls_ssl_config_setctx() :%d\n", ret);
      goto exit;
    }

  printf("Load socket context from files successfully\n");

  return 0;

exit:
  remove(LTE_HIBERNATION_TLS_NET_CONTEXT_PATH);
  remove(LTE_HIBERNATION_TLS_X509CRT_CONTEXT_PATH);
  remove(LTE_HIBERNATION_TLS_ENTROPY_CONTEXT_PATH);
  remove(LTE_HIBERNATION_TLS_CTRDRBG_CONTEXT_PATH);
  remove(LTE_HIBERNATION_TLS_SSL_CONTEXT_PATH);
  remove(LTE_HIBERNATION_TLS_SSLCONF_CONTEXT_PATH);

  return ret;
}

/****************************************************************************
 * Name: send_request_common
 ****************************************************************************/

static int send_request_common(FAR struct wu_wget_context_s *ctx, char *url)
{
  int ret;

  ret = wu_perform_wget_connect(ctx, url);
  if (ret < 0)
    {
      printf("wu_perform_wget_connect() failed: %d\n", ret);
      return -1;
    }

  printf("Server connected\n");

  ret = wu_perform_wget_request(ctx, g_iobuffer, sizeof(g_iobuffer));
  if (ret < 0)
    {
      printf("wu_perform_wget_request() failed: %d\n", ret);
      return -1;
    }

  printf("HTTP request done\n");

  return 0;
}

/****************************************************************************
 * Name: send_http_request
 ****************************************************************************/

static int send_http_request(FAR struct wgetops_sock_context_s *ctx,
                             char *url)
{
  struct wu_wget_context_s wgetctx;

  wgetctx.ctx         = (FAR void *)ctx;
  wgetctx.ops.connect = wgetops_sock_connect;
  wgetctx.ops.send    = wgetops_sock_send;
  wgetctx.ops.recv    = wgetops_sock_recv;
  wgetctx.ops.close   = wgetops_sock_close;

  return send_request_common(&wgetctx, url);
}

/****************************************************************************
 * Name: send_https_request
 ****************************************************************************/

static int send_https_request(FAR struct wgetops_tls_context_s *ctx,
                              char *url)
{
  struct wu_wget_context_s wgetctx;

  wgetctx.ctx         = (FAR void *)ctx;
  wgetctx.ops.connect = wgetops_tls_connect;
  wgetctx.ops.send    = wgetops_tls_send;
  wgetctx.ops.recv    = wgetops_tls_recv;
  wgetctx.ops.close   = wgetops_tls_close;

  return send_request_common(&wgetctx, url);
}

/****************************************************************************
 * Name: recv_http_response
 ****************************************************************************/

static int recv_http_response(FAR struct wgetops_sock_context_s *ctx)
{
  struct wu_wget_context_s wgetctx;
  int ret;

  wgetctx.ctx         = (FAR void *)ctx;
  wgetctx.ops.connect = wgetops_sock_connect;
  wgetctx.ops.send    = wgetops_sock_send;
  wgetctx.ops.recv    = wgetops_sock_recv;
  wgetctx.ops.close   = wgetops_sock_close;

  ret = wu_perform_wget_response(&wgetctx, g_iobuffer, sizeof(g_iobuffer));
  if (ret < 0)
    {
      printf("wu_perform_wget_response() failed: %d\n", ret);
      return -1;
    }

  printf("HTTP response received\n");

  return 0;
}

/****************************************************************************
 * Name: recv_https_response
 ****************************************************************************/

static int recv_https_response(FAR struct wgetops_tls_context_s *ctx)
{
  struct wu_wget_context_s wgetctx;
  int ret;

  wgetctx.ctx         = (FAR void *)ctx;
  wgetctx.ops.connect = wgetops_tls_connect;
  wgetctx.ops.send    = wgetops_tls_send;
  wgetctx.ops.recv    = wgetops_tls_recv;
  wgetctx.ops.close   = wgetops_tls_close;

  ret = wu_perform_wget_response(&wgetctx, g_iobuffer, sizeof(g_iobuffer));
  if (ret < 0)
    {
      printf("wu_perform_wget_response() failed: %d\n", ret);
      return -1;
    }

  printf("HTTP response received\n");

  return 0;
}

/****************************************************************************
 * Name: context_save_cb
 *
 * Description:
 *   Function to receive context data that needs to be saved when the LTE
 *   enters hibernation mode.
 *   The context data needs to save in this function.
 ****************************************************************************/

static void context_save_cb(uint8_t *data, int size)
{
  app_save_file(LTE_HIBERNATION_CONTEXT_PATH, data, size);
}

/****************************************************************************
 * Name: lte_enter_hibernation
 *
 * Description:
 *   Function to transition LTE functions to hibernation mode.
 ****************************************************************************/

static int lte_enter_hibernation(void)
{
  int ret;
  struct boardioc_pm_ctrl_s pmc =
    {
      .action = BOARDIOC_PM_CHANGESTATE,
      .domain = BOARD_PM_APPS,
      .state  = PM_SLEEP
    };

  ret = lte_set_context_save_cb(context_save_cb);
  if (ret < 0)
    {
      printf("lte_set_context_save_cb failed (%d)\n", ret);
      return ret;
    }

  ret = boardctl(BOARDIOC_PM_CONTROL, (uintptr_t)&pmc);
  if (ret < 0)
    {
      printf("Sleep failed.\n");
      return ret;
    }

  printf("entering sleep...\n");

  return 0;
}

/****************************************************************************
 * Name: lte_resume_from_hibernation
 *
 * Description:
 *   Function to return LTE functions from hibernation mode.
 ****************************************************************************/

static int lte_resume_from_hibernation(void)
{
  int size = 0;
  int ctx_size;
  int ret = 0;
  int fd = 0;
  uint8_t data[64];

  if (strncmp(SD_MOUNT_POINT, SAVE_DIR, strlen(SD_MOUNT_POINT)) == 0)
    {
      if (!avail_sdcard())
        {
          printf("SD card is not inserted...\n");
          return -1;
        }
    }

  ret = lte_initialize();
  if (ret < 0 && ret != -EALREADY)
    {
      printf("lte_initialize failed.(%d)\n", ret);
      return -1;
    }

  size = app_file_size(LTE_HIBERNATION_CONTEXT_PATH);
  if (size > 0)
    {
      /* Get the context size required for resume. */

      ctx_size = lte_hibernation_resume(NULL, 0);
      if (ctx_size != size)
        {
          printf("File size does not match."
                 " file size = %d, context size = %d\n",
                 size, ctx_size);
          ret = -1;
        }
      else
        {
          fd = open(LTE_HIBERNATION_CONTEXT_PATH, O_RDONLY);

          do
            {
              size = read(fd, data, sizeof(data));
              ret = lte_hibernation_resume(data, size);
              if (ret < 0)
                {
                  printf("lte_hibernation_resume failed.\n");
                }
              else
                {
                  printf("remain size = %d\n", ret);
                }
            }
          while (ret > 0);

          close(fd);
        }

      unlink(LTE_HIBERNATION_CONTEXT_PATH);
    }
  else
    {
      ret = -1;
    }

  if (ret != 0)
    {
      lte_finalize();
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: ip_parse
 ****************************************************************************/

static int ip_parse(FAR char *s)
{
  int ret = -1;

  if (MATCH_STRING(s, "v4v6"))
    {
      ret = LTE_IPTYPE_V4V6;
    }
  else if (MATCH_STRING(s, "v4"))
    {
      ret = LTE_IPTYPE_V4;
    }
  else if (MATCH_STRING(s, "v6"))
    {
      ret = LTE_IPTYPE_V6;
    }

  return ret;
}

/****************************************************************************
 * Name: auth_parse
 ****************************************************************************/

static int auth_parse(FAR char *s)
{
  int ret = -1;

  if (MATCH_STRING(s, "none"))
    {
      ret = LTE_APN_AUTHTYPE_NONE;
    }
  else if (MATCH_STRING(s, "pap"))
    {
      ret = LTE_APN_AUTHTYPE_PAP;
    }
  else if (MATCH_STRING(s, "chap"))
    {
      ret = LTE_APN_AUTHTYPE_CHAP;
    }

  return ret;
}

/****************************************************************************
 * Name: show_usage
 ****************************************************************************/

static void show_usage(FAR const char *progname, int exitcode)
{
  fprintf(stderr, "\nUSAGE: %s command\n", progname);
  fprintf(stderr, " [-a <apn_name>] [-i <ip_type>] [-t <auth_type>]"
                  " [-u <user_name>] [-p <password>]\n");
  fprintf(stderr, "  -a: APN name\n");
  fprintf(stderr, "  -i: IP type : v4 or v6 or v4v6\n");
  fprintf(stderr, "  -t: Authenticaion type : none or pap or chap\n");
  fprintf(stderr, "  -u: User name for authenticaion\n");
  fprintf(stderr, "  -p: Password for authenticaion\n");
  fprintf(stderr, " [-h]: Show this message\n");
  exit(exitcode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  int opt;
  struct lte_apn_setting apnsetting;
  int ip_type;
  int auth_type;

  /* This sample executes the following sequence
   * 1. Connect LTE network
   * 2. send HTTP request (secure socket)
   * 3. spresense cold sleep (Keep the connection to LTE)
   * 4. 10 seconds later, HTTP response received (secure socket)
   * 5. send HTTP request (non-secure socket)
   * 6. spresense cold sleep (Keep the connection to LTE)
   * 7. 10 seconds later, HTTP response received (non-secure socket)
   * 8. end
   */

  printf("Wakeup spresense\n");

  apnsetting.apn =  CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_APN_NAME;
  apnsetting.ip_type   = APP_APN_IPTYPE;
  apnsetting.auth_type = APP_APN_AUTHTYPE;
  apnsetting.apn_type  = LTE_APN_TYPE_DEFAULT | LTE_APN_TYPE_IA;
  apnsetting.user_name =
    CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_APN_USERNAME;
  apnsetting.password  =
    CONFIG_EXAMPLES_LTE_HIBERNATION_WAKE_SOCKET_APN_PASSWD;

  optind = -1;
  while ((opt = getopt(argc, argv, "a:i:t:u:p:h")) != -1)
    {
      switch (opt)
        {
          case 'a':
            if (strlen(optarg) >= LTE_APN_LEN)
              {
                fprintf(stderr, "APN name is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            apnsetting.apn = optarg;
            break;
          case 'i':
            ip_type = ip_parse(optarg);
            if (ip_type == -1)
              {
                fprintf(stderr, "Invalid IP type:%s\n", optarg);
                show_usage(argv[0], EXIT_FAILURE);
              }

            apnsetting.ip_type = ip_type;
            break;
          case 't':
            auth_type = auth_parse(optarg);
            if (auth_type == -1)
              {
                fprintf(stderr, "Invalid authenticaion type:%s\n", optarg);
                show_usage(argv[0], EXIT_FAILURE);
              }

            apnsetting.auth_type = auth_type;
            break;
          case 'u':
            if (strlen(optarg) >= LTE_APN_USER_NAME_LEN)
              {
                fprintf(stderr, "User name is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            apnsetting.user_name = optarg;
            break;
          case 'p':
            if (strlen(optarg) >= LTE_APN_PASSWD_LEN)
              {
                fprintf(stderr, "Password is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            apnsetting.password = optarg;
            break;
          case 'h':
            show_usage(argv[0], EXIT_SUCCESS);
            break;
          default:
            show_usage(argv[0], EXIT_FAILURE);
            break;
        }
    }

  if (up_pm_get_bootcause() &
      (PM_BOOT_COLD_RTC | PM_BOOT_COLD_RTC_ALM0 | PM_BOOT_COLD_GPIO))
    {
      /* Resume from suspend mode */

      printf("Resume from LTE hibernation mode.\n");

      ret = lte_resume_from_hibernation();
      if (ret < 0)
        {
          goto normal_boot;
        }

      /* Find out if there is a file that stores the context of the secure
       * socket
       */

      if (app_file_exist(LTE_HIBERNATION_TLS_NET_CONTEXT_PATH))
        {
          /* -------------------------------------------------------------
           * 4. 10 seconds later, HTTP response received (secure socket)
           * -------------------------------------------------------------
           */

          /* Load the secure socket context needed to receive the
           * HTTP response from a file.
           */

          ret = load_tls_contexts(&g_tls_context);
          if (ret < 0)
            {
              return -1;
            }

          recv_https_response(&g_tls_context);

          /* ------------------------------------------
           * 5. send HTTP request (non-secure socket)
           * ------------------------------------------
           */

          /* HTTP request is sent using non-secure socket.
           * HTTP response will arrive 10 seconds later. In the meantime,
           * Spresense sleeps and waits for the HTTP response, and wakes up
           * when triggered by the HTTP response.
           */

          ret = send_http_request(&g_sock_context, APP_HTTP_WGET_URL);
          if (ret < 0)
            {
              return -1;
            }

          /* Save to file the non-secure socket context needed to receive
           * a HTTP response after resuming from sleep.
           */

          ret = save_sock_context(g_sock_context.sockfd,
                                  LTE_HIBERNATION_SOCK_CONTEXT_PATH);
          if (ret < 0)
            {
              return -1;
            }

          /* ------------------------------------------------------
           * 6. spresense cold sleep (Keep the connection to LTE)
           * ------------------------------------------------------
           */

          /* LTE module entering hibernation mode */

          printf("Entering LTE hibernation mode.\n");

          ret = lte_enter_hibernation();
          if (ret < 0)
            {
              return -1;
            }

          /* Entering cold sleep */

          boardctl(BOARDIOC_POWEROFF, 1);
        }

      /* Find out if there is a file that stores the context of the
       * non-secure socket
       */

      else if (app_file_exist(LTE_HIBERNATION_SOCK_CONTEXT_PATH))
        {
          /* -------------------------------------------------------------
           * 7. 10 sec later, HTTP response received (non-secure socket)
           * -------------------------------------------------------------
           */

          /* Load the non-secure socket context needed to receive the
           * HTTP response from a file.
           */

          g_sock_context.sockfd =
            load_sock_context(LTE_HIBERNATION_SOCK_CONTEXT_PATH);
          if (g_sock_context.sockfd < 0)
            {
              return -1;
            }

          recv_http_response(&g_sock_context);

          /* --------
           * 8. end
           * --------
           */

          printf("End of this sample\n");
        }

      return 0;
    }

normal_boot:

  /* Normal boot */

  printf("Turn On LTE.\n");

  /* ------------------------
   * 1. Connect LTE network
   * ------------------------
   */

  ret = app_connect_to_lte(&apnsetting);
  if (ret < 0)
    {
      return -1;
    }

  /* --------------------------------------
   * 2. send HTTP request (secure socket)
   * --------------------------------------
   */

  /* HTTP requests are sent using secure socket. HTTP response will
   * arrive 10 seconds later. In the meantime, Spresense sleeps and
   * waits for the HTTP response, and wakes up when triggered by
   * the HTTP response.
   */

  ret = send_https_request(&g_tls_context, APP_HTTPS_WGET_URL);
  if (ret < 0)
    {
      return -1;
    }

  /* Save to file the secure socket context needed to receive a HTTP
   * response after resuming from sleep.
   */

  ret = save_tls_contexts(&g_tls_context);
  if (ret < 0)
    {
      return -1;
    }

  /* ------------------------------------------------------
   * 3. spresense cold sleep (Keep the connection to LTE)
   * ------------------------------------------------------
   */

  /* LTE module entering hibernation mode */

  printf("Entering LTE hibernation mode.\n");

  ret = lte_enter_hibernation();
  if (ret < 0)
    {
      return -1;
    }

  /* Entering cold sleep */

  boardctl(BOARDIOC_POWEROFF, 1);

  return 0;
}

