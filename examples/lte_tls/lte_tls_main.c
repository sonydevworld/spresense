/****************************************************************************
 * examples/lte_tls/lte_tls_main.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
#include <string.h>
#include <netdb.h>
#include <dirent.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <netutils/netlib.h>

#include <mbedtls/config.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/platform.h>
#include <mbedtls/ssl.h>

#include "lte_connection.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_IOBUFFER_LEN            512

#define APP_POST_URL                "https://example.com/post"
#define APP_HTTP_STATUS_CODE_OFFSET 9
#define APP_HTTP_STATUS_CODE_LEN    3

#define APP_SCHEME                 "https"
#define APP_SCHEME_LEN              6 /* length of "https" + \0 */
#define APP_PORT_LEN                5 
#define APP_HTTPS_WELLKNOWN_PORT    443
#define APP_HOSTNAME_LEN            128
#define APP_FILENAME_LEN            128
#define APP_TLS_CERT_FILENAME_LEN   128

#ifndef CONFIG_EXAMPLES_LTE_TLS_CERTS_PATH
#define CONFIG_EXAMPLES_LTE_TLS_CERTS_PATH   "/mnt/sd0/CERTS"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mbedtls_net_context      g_server_fd;
static mbedtls_x509_crt         g_ca;
static mbedtls_entropy_context  g_entropy;
static mbedtls_ctr_drbg_context g_ctr_drbg;
static mbedtls_ssl_context      g_ssl;
static mbedtls_ssl_config       g_conf;

static unsigned char g_iobuffer[APP_IOBUFFER_LEN];
static char g_hostname[APP_HOSTNAME_LEN] = {0};
static char g_filename[APP_FILENAME_LEN] = {0};

static char g_tls_cert_filename[APP_TLS_CERT_FILENAME_LEN];

/* subject:/C=US/O=GeoTrust Inc./CN=GeoTrust Global CA */
/* issuer :/C=US/O=GeoTrust Inc./CN=GeoTrust Global CA */
static const unsigned char GeoTrustGlobalCA_certificate[856]={
0x30,0x82,0x03,0x54,0x30,0x82,0x02,0x3C,0xA0,0x03,0x02,0x01,0x02,0x02,0x03,0x02,
0x34,0x56,0x30,0x0D,0x06,0x09,0x2A,0x86,0x48,0x86,0xF7,0x0D,0x01,0x01,0x05,0x05,
0x00,0x30,0x42,0x31,0x0B,0x30,0x09,0x06,0x03,0x55,0x04,0x06,0x13,0x02,0x55,0x53,
0x31,0x16,0x30,0x14,0x06,0x03,0x55,0x04,0x0A,0x13,0x0D,0x47,0x65,0x6F,0x54,0x72,
0x75,0x73,0x74,0x20,0x49,0x6E,0x63,0x2E,0x31,0x1B,0x30,0x19,0x06,0x03,0x55,0x04,
0x03,0x13,0x12,0x47,0x65,0x6F,0x54,0x72,0x75,0x73,0x74,0x20,0x47,0x6C,0x6F,0x62,
0x61,0x6C,0x20,0x43,0x41,0x30,0x1E,0x17,0x0D,0x30,0x32,0x30,0x35,0x32,0x31,0x30,
0x34,0x30,0x30,0x30,0x30,0x5A,0x17,0x0D,0x32,0x32,0x30,0x35,0x32,0x31,0x30,0x34,
0x30,0x30,0x30,0x30,0x5A,0x30,0x42,0x31,0x0B,0x30,0x09,0x06,0x03,0x55,0x04,0x06,
0x13,0x02,0x55,0x53,0x31,0x16,0x30,0x14,0x06,0x03,0x55,0x04,0x0A,0x13,0x0D,0x47,
0x65,0x6F,0x54,0x72,0x75,0x73,0x74,0x20,0x49,0x6E,0x63,0x2E,0x31,0x1B,0x30,0x19,
0x06,0x03,0x55,0x04,0x03,0x13,0x12,0x47,0x65,0x6F,0x54,0x72,0x75,0x73,0x74,0x20,
0x47,0x6C,0x6F,0x62,0x61,0x6C,0x20,0x43,0x41,0x30,0x82,0x01,0x22,0x30,0x0D,0x06,
0x09,0x2A,0x86,0x48,0x86,0xF7,0x0D,0x01,0x01,0x01,0x05,0x00,0x03,0x82,0x01,0x0F,
0x00,0x30,0x82,0x01,0x0A,0x02,0x82,0x01,0x01,0x00,0xDA,0xCC,0x18,0x63,0x30,0xFD,
0xF4,0x17,0x23,0x1A,0x56,0x7E,0x5B,0xDF,0x3C,0x6C,0x38,0xE4,0x71,0xB7,0x78,0x91,
0xD4,0xBC,0xA1,0xD8,0x4C,0xF8,0xA8,0x43,0xB6,0x03,0xE9,0x4D,0x21,0x07,0x08,0x88,
0xDA,0x58,0x2F,0x66,0x39,0x29,0xBD,0x05,0x78,0x8B,0x9D,0x38,0xE8,0x05,0xB7,0x6A,
0x7E,0x71,0xA4,0xE6,0xC4,0x60,0xA6,0xB0,0xEF,0x80,0xE4,0x89,0x28,0x0F,0x9E,0x25,
0xD6,0xED,0x83,0xF3,0xAD,0xA6,0x91,0xC7,0x98,0xC9,0x42,0x18,0x35,0x14,0x9D,0xAD,
0x98,0x46,0x92,0x2E,0x4F,0xCA,0xF1,0x87,0x43,0xC1,0x16,0x95,0x57,0x2D,0x50,0xEF,
0x89,0x2D,0x80,0x7A,0x57,0xAD,0xF2,0xEE,0x5F,0x6B,0xD2,0x00,0x8D,0xB9,0x14,0xF8,
0x14,0x15,0x35,0xD9,0xC0,0x46,0xA3,0x7B,0x72,0xC8,0x91,0xBF,0xC9,0x55,0x2B,0xCD,
0xD0,0x97,0x3E,0x9C,0x26,0x64,0xCC,0xDF,0xCE,0x83,0x19,0x71,0xCA,0x4E,0xE6,0xD4,
0xD5,0x7B,0xA9,0x19,0xCD,0x55,0xDE,0xC8,0xEC,0xD2,0x5E,0x38,0x53,0xE5,0x5C,0x4F,
0x8C,0x2D,0xFE,0x50,0x23,0x36,0xFC,0x66,0xE6,0xCB,0x8E,0xA4,0x39,0x19,0x00,0xB7,
0x95,0x02,0x39,0x91,0x0B,0x0E,0xFE,0x38,0x2E,0xD1,0x1D,0x05,0x9A,0xF6,0x4D,0x3E,
0x6F,0x0F,0x07,0x1D,0xAF,0x2C,0x1E,0x8F,0x60,0x39,0xE2,0xFA,0x36,0x53,0x13,0x39,
0xD4,0x5E,0x26,0x2B,0xDB,0x3D,0xA8,0x14,0xBD,0x32,0xEB,0x18,0x03,0x28,0x52,0x04,
0x71,0xE5,0xAB,0x33,0x3D,0xE1,0x38,0xBB,0x07,0x36,0x84,0x62,0x9C,0x79,0xEA,0x16,
0x30,0xF4,0x5F,0xC0,0x2B,0xE8,0x71,0x6B,0xE4,0xF9,0x02,0x03,0x01,0x00,0x01,0xA3,
0x53,0x30,0x51,0x30,0x0F,0x06,0x03,0x55,0x1D,0x13,0x01,0x01,0xFF,0x04,0x05,0x30,
0x03,0x01,0x01,0xFF,0x30,0x1D,0x06,0x03,0x55,0x1D,0x0E,0x04,0x16,0x04,0x14,0xC0,
0x7A,0x98,0x68,0x8D,0x89,0xFB,0xAB,0x05,0x64,0x0C,0x11,0x7D,0xAA,0x7D,0x65,0xB8,
0xCA,0xCC,0x4E,0x30,0x1F,0x06,0x03,0x55,0x1D,0x23,0x04,0x18,0x30,0x16,0x80,0x14,
0xC0,0x7A,0x98,0x68,0x8D,0x89,0xFB,0xAB,0x05,0x64,0x0C,0x11,0x7D,0xAA,0x7D,0x65,
0xB8,0xCA,0xCC,0x4E,0x30,0x0D,0x06,0x09,0x2A,0x86,0x48,0x86,0xF7,0x0D,0x01,0x01,
0x05,0x05,0x00,0x03,0x82,0x01,0x01,0x00,0x35,0xE3,0x29,0x6A,0xE5,0x2F,0x5D,0x54,
0x8E,0x29,0x50,0x94,0x9F,0x99,0x1A,0x14,0xE4,0x8F,0x78,0x2A,0x62,0x94,0xA2,0x27,
0x67,0x9E,0xD0,0xCF,0x1A,0x5E,0x47,0xE9,0xC1,0xB2,0xA4,0xCF,0xDD,0x41,0x1A,0x05,
0x4E,0x9B,0x4B,0xEE,0x4A,0x6F,0x55,0x52,0xB3,0x24,0xA1,0x37,0x0A,0xEB,0x64,0x76,
0x2A,0x2E,0x2C,0xF3,0xFD,0x3B,0x75,0x90,0xBF,0xFA,0x71,0xD8,0xC7,0x3D,0x37,0xD2,
0xB5,0x05,0x95,0x62,0xB9,0xA6,0xDE,0x89,0x3D,0x36,0x7B,0x38,0x77,0x48,0x97,0xAC,
0xA6,0x20,0x8F,0x2E,0xA6,0xC9,0x0C,0xC2,0xB2,0x99,0x45,0x00,0xC7,0xCE,0x11,0x51,
0x22,0x22,0xE0,0xA5,0xEA,0xB6,0x15,0x48,0x09,0x64,0xEA,0x5E,0x4F,0x74,0xF7,0x05,
0x3E,0xC7,0x8A,0x52,0x0C,0xDB,0x15,0xB4,0xBD,0x6D,0x9B,0xE5,0xC6,0xB1,0x54,0x68,
0xA9,0xE3,0x69,0x90,0xB6,0x9A,0xA5,0x0F,0xB8,0xB9,0x3F,0x20,0x7D,0xAE,0x4A,0xB5,
0xB8,0x9C,0xE4,0x1D,0xB6,0xAB,0xE6,0x94,0xA5,0xC1,0xC7,0x83,0xAD,0xDB,0xF5,0x27,
0x87,0x0E,0x04,0x6C,0xD5,0xFF,0xDD,0xA0,0x5D,0xED,0x87,0x52,0xB7,0x2B,0x15,0x02,
0xAE,0x39,0xA6,0x6A,0x74,0xE9,0xDA,0xC4,0xE7,0xBC,0x4D,0x34,0x1E,0xA9,0x5C,0x4D,
0x33,0x5F,0x92,0x09,0x2F,0x88,0x66,0x5D,0x77,0x97,0xC7,0x1D,0x76,0x13,0xA9,0xD5,
0xE5,0xF1,0x16,0x09,0x11,0x35,0xD5,0xAC,0xDB,0x24,0x71,0x70,0x2C,0x98,0x56,0x0B,
0xD9,0x17,0xB4,0xD1,0xE3,0x51,0x2B,0x5E,0x75,0xE8,0xD5,0xD0,0xDC,0x4F,0x34,0xED,
0xC2,0x05,0x66,0x80,0xA1,0xCB,0xE6,0x33,
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int create_http_post(const char    *host,
                            const char    *path,
                            char          *buffer,
                            size_t        buffer_size)
{
  const char *post_data = "Spresense!";
  const char http_post_request[] = "POST %s HTTP/1.1\r\n"
                                   "HOST: %s\r\n"
                                   "Connection: close\r\n"
                                   "Content-Length: %d\r\n"
                                   "\r\n"
                                   "%s";

  return snprintf(buffer, buffer_size,
                  http_post_request,
                  path,
                  host,
                  strlen(post_data),
                  post_data);
}

static void print_http_status_code(const unsigned char *buffer)
{
  unsigned char status_code[APP_HTTP_STATUS_CODE_LEN + 1] = {0};

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
 * lte_tls_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  FAR static const char *pers = "mbedtls";
  FAR DIR *dirp = NULL;  /* Directory for TLS certification files */
  FAR struct dirent *cert_info = NULL;
  char *url    = APP_POST_URL;
  struct url_s parsed_url;
  char scheme[APP_SCHEME_LEN]  = {0};
  char port_char[APP_PORT_LEN] = {0};
  unsigned char *buf_ptr;
  size_t        request_len;

  /* This application is a sample that
   *    parse URL and get hostname, filename, port number (1.)
   *    connect to the LTE network,                       (2.)
   *    establish TLS connection to server,               (3. - 7.)
   *    send a some data with HTTP POST method,           (8.)
   *    and disconnect from the LTE network.              (9. - 11.)
   * mbed TLS usage in this example is base on public example by ARM.
   *   Git:           git@github.com:ARMmbed/mbedtls.git
   *   program path:  programs/ssl/mini_client.c and ssl_client1.c 
   *
   * The URL is specified by the second argument.
   * If URL is not specified, use the default URL.
   * The URL starts with "https://"
   * (eg, https://example.com/index.html, or https://192.0.2.1:80/index.html)
   */

  if (argc > 1)
    {
      url = argv[1];
    }

  /* 1. Parse input URL */

  memset(&parsed_url, 0, sizeof(parsed_url));
  parsed_url.scheme    = scheme;
  parsed_url.schemelen = APP_SCHEME_LEN;
  parsed_url.host      = g_hostname;
  parsed_url.hostlen   = APP_HOSTNAME_LEN;
  parsed_url.path      = g_filename;
  parsed_url.pathlen   = APP_FILENAME_LEN;

  netlib_parseurl(url, &parsed_url);

  /* Check if url start from "https://" */

  if (strncmp(scheme, APP_SCHEME, APP_SCHEME_LEN) != 0)
    {
      /* Because this example want to use mbed TLS,
       * support only "https://" format URL.
       */

      printf("URL is not https\n");
      return ERROR;
    }

  if (parsed_url.port == 0)
    {
      /* parsed_url.port = 0 means that port number is abbreviated.
       * In such a case, use well-known port for HTTPS.
       */

      parsed_url.port = APP_HTTPS_WELLKNOWN_PORT;
    }

  /* Because mbedTLS need the port number with character string format
   *  in mbedtls_net_connect() API, transform int -> char.
   */
 
  snprintf(port_char, APP_PORT_LEN, "%d", parsed_url.port);

  /* 2. Connect to LTE network.
   *    Please refer to lte_connection.c.
   *    (The contents is the same as lte_http_get example.)
   */

  if (app_lte_tls_connect_to_lte()) 
    {
      return ERROR;
    }

  /* 3. Initialize mbedTLS stuff */

  mbedtls_ctr_drbg_init(&g_ctr_drbg);
  mbedtls_net_init(&g_server_fd);
  mbedtls_ssl_init(&g_ssl);
  mbedtls_ssl_config_init(&g_conf);
  mbedtls_x509_crt_init(&g_ca);
  mbedtls_entropy_init(&g_entropy);

  /* 4. Setup certificates.
   *    In this examples, read the certificates files from the directory
   *     which is defined by Kconfig.
   */

  dirp = opendir(CONFIG_EXAMPLES_LTE_TLS_CERTS_PATH);
  if (dirp == NULL)
    {
      cert_info = NULL;
    }
  else
    {
      cert_info = readdir(dirp);
    }

  if (cert_info == NULL)
    {
      /* In no certification files case, get certification data from memory */

      printf("No certificate files found. Use default certificate.\n");

      if (mbedtls_x509_crt_parse_der
            (&g_ca,
             GeoTrustGlobalCA_certificate,
             sizeof(GeoTrustGlobalCA_certificate)) != 0)
        {
          printf("mbedtls_x509_crt_parse_der() fail\n");
          goto exit;
        }
    }
  else
    {
      do
        {
          snprintf(g_tls_cert_filename, APP_TLS_CERT_FILENAME_LEN,
                   "%s/%s",
                   CONFIG_EXAMPLES_LTE_TLS_CERTS_PATH,
                   cert_info->d_name);
          if (mbedtls_x509_crt_parse_file(&g_ca, g_tls_cert_filename) == 0)
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

  mbedtls_ssl_conf_ca_chain(&g_conf, &g_ca, NULL);
  mbedtls_ssl_conf_authmode(&g_conf, MBEDTLS_SSL_VERIFY_REQUIRED);

  /* 5. Setup mbedTLS stuff */

  if ((ret = mbedtls_ctr_drbg_seed(&g_ctr_drbg, mbedtls_entropy_func, &g_entropy,
                            (const unsigned char *) pers, strlen(pers))) != 0)
    {
      printf("mbedtls_ctr_drbg_seed() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  if ((ret = mbedtls_ssl_config_defaults(&g_conf,
                                  MBEDTLS_SSL_IS_CLIENT,
                                  MBEDTLS_SSL_TRANSPORT_STREAM,
                                  MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
    {
      printf("mbedtls_ssl_config_defaults() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  mbedtls_ssl_conf_rng(&g_conf, mbedtls_ctr_drbg_random, &g_ctr_drbg);

  if ((ret = mbedtls_ssl_setup(&g_ssl, &g_conf)) != 0)
    {
      printf("mbedtls_ssl_setup() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  if ((ret = mbedtls_ssl_set_hostname(&g_ssl, g_hostname)) != 0)
    {
      printf("mbedtls_ssl_set_hostname() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  /* 6. Start the connection.
   *    mbedtls_net_connect execute address resolution, socket create,
   *    and connect.
   */

  if ((ret = mbedtls_net_connect(&g_server_fd,
                          g_hostname,
                          port_char,    /* The type is not integer, but string. */
                          MBEDTLS_NET_PROTO_TCP)) != 0)
    {
      printf("mbedtls_net_connect() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  mbedtls_ssl_set_bio(&g_ssl,
                      &g_server_fd,
                      mbedtls_net_send,
                      mbedtls_net_recv,
                      NULL);

  /* 7. Do SSL handshake */

  if ((ret = mbedtls_ssl_handshake(&g_ssl)) != 0)
    {
      printf("mbedtls_ssl_handshake() fail ret = -0x%X\n", -ret);
      goto exit;
    }

  /* 8. Write the POST request and receive HTTP response.
   *    In this example, the fixed HTTP body "Spresense!" is sent.
   */

  request_len = create_http_post(g_hostname,
                                 g_filename,
                                 (char *)g_iobuffer,
                                 APP_IOBUFFER_LEN);
  buf_ptr     = g_iobuffer;
  do
    {
      ret = mbedtls_ssl_write(&g_ssl,
                              (const unsigned char *) buf_ptr,
                              request_len);

      /* Return value means actual written size or error_code */

      if (ret > 0)
        {
          /* Successful case */

          if (ret < request_len)
            {
              /* Written size may be smaller than requested size.
               * In such case, for examples, shift address and retry */

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

              printf("mbedtls_ssl_write() fail: ret = %d\n", ret);
              break;
            }
        }
    }
  while (1);

  /* Read received data from TLS server.
   * In this example, receive HTTP response.
   */

  buf_ptr     = g_iobuffer;
  request_len = APP_IOBUFFER_LEN;
  do
    {
      ret = mbedtls_ssl_read(&g_ssl, buf_ptr, request_len);

      /* Return value means actual read size or error_code */

      if (ret > 0)
        {
          /* Successful case */

          if (ret < request_len)
            {
              /* Read size may be smaller than requested size.
               * In such case, for examples, shift address and retry */

              buf_ptr     += ret;
              request_len -= ret;
              continue;
            }

          /* In this example,
           * print only HTTP status code for simplicity.
           */

          print_http_status_code((const unsigned char *)g_iobuffer);
          break;
        }
      else if (ret == 0)
        {
          /* read size = 0 means communication end */

          break;
        }
      else
        {
          /* Unsuccessful case. */

          if ((ret == MBEDTLS_ERR_SSL_WANT_READ) ||
              (ret == MBEDTLS_ERR_SSL_WANT_WRITE))
            {
              /* Please retry */

              continue;
            }
          else if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
            {
              /* Peer's disconnection */

              break;
            }
          else
            {
              /* Error case */

              printf("mbedtls_ssl_read() fail: ret = %d\n", ret); 
              break;
            } 
        }
    }
  while (1);

  /* In this example, ignore the HTTP message except for status code. */

  /* 9. Close the connection */

  mbedtls_ssl_close_notify(&g_ssl);

exit:

  /* 10. Free mbedTLS stuff */

  mbedtls_net_free(&g_server_fd );
  mbedtls_ssl_free(&g_ssl);
  mbedtls_ssl_config_free(&g_conf);
  mbedtls_ctr_drbg_free(&g_ctr_drbg);
  mbedtls_entropy_free(&g_entropy);
  mbedtls_x509_crt_free(&g_ca);

  /* 11. Disconnect from LTE network.
   *     Please refer to lte_connection.c.
   *     (The contents is the same as lte_http_get examples.)
   */

  app_lte_tls_disconnect_from_lte();

  return OK;
}
