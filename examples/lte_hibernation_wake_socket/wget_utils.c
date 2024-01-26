/****************************************************************************
 * test/sdk/lte/hibernation/secure_socket/wget_tls.c
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
#include <netutils/netlib.h>

#include "wget_utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_HTTP_STATUS_CODE_OFFSET 9
#define APP_HTTP_STATUS_CODE_LEN    3

#define APP_HTTPS_SCHEME            "https"
#define APP_HTTPS_SCHEME_LEN        6 /* length of "https" + \0 */
#define APP_HTTP_SCHEME             "http"
#define APP_HTTP_SCHEME_LEN         5 /* length of "https" + \0 */
#define APP_PORT_LEN                6
#define APP_HTTPS_WELLKNOWN_PORT    443
#define APP_HTTP_WELLKNOWN_PORT     80
#define APP_HOSTNAME_LEN            128
#define APP_FILENAME_LEN            128

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_hostname[APP_HOSTNAME_LEN];
static char g_filename[APP_FILENAME_LEN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int create_http_get(const char    *host,
                           const char    *path,
                           char          *buffer,
                           size_t        buffer_size)
{
  const char http_get_request[] = "GET %s HTTP/1.1\r\n"
                                  "HOST: %s\r\n"
                                  "Connection: close\r\n"
                                   "\r\n";

  return snprintf(buffer, buffer_size,
                  http_get_request,
                  path,
                  host);
}

static void print_http_status_code(const unsigned char *buffer)
{
  unsigned char status_code[APP_HTTP_STATUS_CODE_LEN + 1] =
    {
      0
    };

  /* Get HTTP status code.
   * For examples, HTTP 200 OK response starts from "HTTP/1.1 200"
   */

  memcpy(status_code,
         &buffer[APP_HTTP_STATUS_CODE_OFFSET],
         APP_HTTP_STATUS_CODE_LEN);
  printf("HTTP status code = %s\n", status_code);
  return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: perform_tls_wget_connect
 ****************************************************************************/

int wu_perform_wget_connect(FAR struct wu_wget_context_s *ctx, FAR char *url)
{
  int ret;
  struct url_s parsed_url;
  char scheme[APP_HTTPS_SCHEME_LEN] =
    {
      0
    };

  char port_char[APP_PORT_LEN] =
    {
      0
    };

  /* 1. Parse input URL */

  memset(&parsed_url, 0, sizeof(parsed_url));
  parsed_url.scheme    = scheme;
  parsed_url.schemelen = APP_HTTPS_SCHEME_LEN;
  parsed_url.host      = g_hostname;
  parsed_url.hostlen   = APP_HOSTNAME_LEN;
  parsed_url.path      = g_filename;
  parsed_url.pathlen   = APP_FILENAME_LEN;

  netlib_parseurl(url, &parsed_url);

  /* Check if url start from "https://" */

  if (strncmp(scheme, APP_HTTPS_SCHEME, APP_HTTPS_SCHEME_LEN) == 0)
    {
      parsed_url.port = (parsed_url.port == 0) ?
                          APP_HTTPS_WELLKNOWN_PORT : 0;
    }

  /* Check if url start from "http://" */

  else if (strncmp(scheme, APP_HTTP_SCHEME, APP_HTTP_SCHEME_LEN) == 0)
    {
      parsed_url.port = (parsed_url.port == 0) ? APP_HTTP_WELLKNOWN_PORT : 0;
    }
  else
    {
      /* Because this example want to use mbed TLS,
       * support only "https://" format URL.
       */

      printf("URL is not https or http\n");
      return ERROR;
    }

  /* Because mbedTLS need the port number with character string format
   * in mbedtls_net_connect() API, transform int -> char.
   */

  snprintf(port_char, APP_PORT_LEN, "%d", parsed_url.port);

  ret = ctx->ops.connect(ctx->ctx, parsed_url.host, port_char);
  if (ret < 0)
    {
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: wu_perform_wget_request
 ****************************************************************************/

int wu_perform_wget_request(FAR struct wu_wget_context_s *ctx,
                            FAR const void *buf, size_t len)
{
  int ret;
  unsigned char *buf_ptr;
  size_t        request_len;

  request_len = create_http_get(g_hostname, g_filename, (char *)buf, len);

  buf_ptr = (unsigned char *)buf;
  do
    {
      ret = ctx->ops.send(ctx->ctx, (FAR const void *)buf_ptr, request_len);

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

          printf("HTTP request sent:%d bytes\n", request_len);
          break;
        }
      else if (ret == 0)
        {
          /* written size = 0 means communication end */

          break;
        }
      else
        {
          printf("send fail: ret = %d\n", ret);
          break;
        }
    }
  while (1);

  return ret;
}

/****************************************************************************
 * Name: wu_perform_wget_response
 ****************************************************************************/

int wu_perform_wget_response(FAR struct wu_wget_context_s *ctx,
                             FAR void *buf, size_t len)
{
  int ret;
  unsigned char *buf_ptr;
  size_t        request_len;

  /* Read received data from server.
   * In this example, receive HTTP response.
   */

  memset(buf, 0, len);
  buf_ptr     = buf;
  request_len = len;
  do
    {
      ret = ctx->ops.recv(ctx->ctx, buf_ptr, request_len);

      /* Return value means actual read size or error_code */

      if (ret > 0)
        {
          printf("HTTP response recv %d bytes\n", ret);
          printf("%s", buf_ptr);

          /* Successful case */

          if (ret < request_len)
            {
              /* Read size may be smaller than requested size.
               * In such case, for examples, shift address and retry
               */

              buf_ptr     += ret;
              request_len -= ret;
              continue;
            }

          /* In this example,
           * print only HTTP status code for simplicity.
           */

          print_http_status_code((const unsigned char *)buf);
          break;
        }
      else if (ret == 0)
        {
          /* read size = 0 means communication end */

          print_http_status_code((const unsigned char *)buf);
          break;
        }
      else
        {
          /* Error case */

          printf("recv() fail: ret = %d\n", ret);
          break;
        }
    }
  while (1);

  ctx->ops.close(ctx->ctx);

  return 0;
}

