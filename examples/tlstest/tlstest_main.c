/****************************************************************************
 * examples/tlstest/tlstest_main.c
 *
 *   Copyright 2018 Sony Corporation
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
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>

#include "lte_sub.h"
#include "ssl_sub.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TLSTEST_ENABLE_CAFILE		/* Use parse_file command */
#define TLSTEST_FLAG_FAIL	0	/* Need to fail test */
#define TLSTEST_FLAG_SUCCESS	1	/* Need to success test */
#define TLSTEST_MAX_SOCKET      4	/* Use parse_file command */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef TLSTEST_ENABLE_CAFILE
static const char* tlstest_calist[] =
{
  "rootca_digi.crt",
  "rootca_digihigh.crt",
  "rootca_digig2.crt",
//  "rootca_comodo.crt",
  NULL
};
#endif
static int g_status =  0;
struct tlstest_t
{
  char* code;
  int   loop;
  char* port;
  char* path;
  int   flag;
  char* host;
};
static struct tlstest_t tlstest_site[] = 
{
  { "00", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "www.example.com" },
  { "01", 1, "443", "/", TLSTEST_FLAG_SUCCESS,
    "www.example.com" },
  { "02", 10, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "www.example.com" },
  { "03", 10, "443", "/", TLSTEST_FLAG_SUCCESS,
    "www.example.com" },
  { "04", 1, "443", "/", TLSTEST_MAX_SOCKET,
    "www.example.com" },
  { "10", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "badssl.com" },
  { "11", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "expired.badssl.com" },
  { "12", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "wrong.host.badssl.com" },
  { "13", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "self-signed.badssl.com" },
  { "14", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "untrusted-root.badssl.com" },
  { "15", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "revoked.badssl.com" },
  { "16", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "pinning-test.badssl.com" },
  { "17", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "no-common-name.badssl.com" },
  { "18", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "no-subject.badssl.com" },
  { "19", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "incomplete-chain.badssl.com" },
  { "20", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "sha1-intermediate.badssl.com" },
  { "21", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "sha256.badssl.com" },
  { "22", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "sha384.badssl.com" },
  { "23", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "sha512.badssl.com" },
  { "24", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "1000-sans.badssl.com" },
  { "25", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "10000-sans.badssl.com" },
  { "26", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "ecc256.badssl.com" },
  { "27", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "ecc384.badssl.com" },
  { "28", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "rsa2048.badssl.com" },
  { "29", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "rsa8192.badssl.com" },
  { "30", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "client.badssl.com" },
  { "31", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "client-cert-missing.badssl.com" },
  { "32", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "cbc.badssl.com" },
  { "33", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "rc4-md5.badssl.com" },
  { "34", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "rc4.badssl.com" },
  { "35", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "3des.badssl.com" },
  { "36", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "null.badssl.com" },
  { "37", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "mozilla-old.badssl.com" },
  { "38", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "mozilla-intermediate.badssl.com" },
  { "39", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "mozilla-modern.badssl.com" },
  { "40", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "dh480.badssl.com" },
  { "41", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "dh512.badssl.com" },
  { "42", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "dh1024.badssl.com" },
  { "43", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "dh2048.badssl.com" },
  { "44", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "dh-small-subgroup.badssl.com" },
  { "45", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "dh-composite.badssl.com" },
  { "46", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "static-rsa.badssl.com" },
  { "47", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "tls-v1-0.badssl.com" },
  { "48", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "tls-v1-1.badssl.com" },
  { "50", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "invalid-expected-sct.badssl.com" },
  { "51", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "hsts.badssl.com" },
  { "52", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "upgrade.badssl.com" },
  { "53", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "preloaded-hsts.badssl.com" },
  { "54", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "subdomain.preloaded-hsts.badssl.com" },
  { "55", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "https-everywhere.badssl.com" },
  { "56", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "spoofed-favicon.badssl.com" },
  { "57", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "long-extended-subdomain-name-containing-many-letters-and-dashes.badssl.com" },
  { "58", 1, "443", NULL, TLSTEST_FLAG_SUCCESS,
    "longextendedsubdomainnamewithoutdashesinordertotestwordwrapping.badssl.com" },
  { "60", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "superfish.badssl.com" },
  { "61", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "edellroot.badssl.com" },
  { "62", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "dsdtestprovider.badssl.com" },
  { "63", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "preact-cli.badssl.com" },
  { "64", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "webpack-dev-server.badssl.com" },
  { "65", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "captive-portal.badssl.com" },
  { "66", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "mitm-software.badssl.com" },
  { "67", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "sha1-2016.badssl.com" },
  { "68", 1, "443", NULL, TLSTEST_FLAG_FAIL,
    "sha1-2017.badssl.com" },
  { "70", 1, "10445", NULL, TLSTEST_FLAG_FAIL,
    "www.ssllabs.com" },
  { "71", 1, "10444", NULL, TLSTEST_FLAG_FAIL,
    "www.ssllabs.com" },
  { "72", 1, "10443", NULL, TLSTEST_FLAG_FAIL,
    "www.ssllabs.com" },
  { "73", 1, "10200", NULL, TLSTEST_FLAG_FAIL,
    "www.ssllabs.com" },
  { "74", 1, "10300", NULL, TLSTEST_FLAG_FAIL,
    "www.ssllabs.com" },
  { "75", 1, "10301", NULL, TLSTEST_FLAG_FAIL,
    "www.ssllabs.com" },
  { "76", 1, "10302", NULL, TLSTEST_FLAG_FAIL,
    "www.ssllabs.com" },
  { "77", 1, "10303", NULL, TLSTEST_FLAG_SUCCESS,
    "www.ssllabs.com" },
  { NULL, 0, NULL, NULL, TLSTEST_FLAG_SUCCESS, NULL },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Common function : INIT -> CONNECT x N -> (GET) -> DISCONNECT x N -> FIN
 ****************************************************************************/
static void tlstest_multi(char *host, char *port, char *path, int count)
{
  int sockfd[TLSTEST_MAX_SOCKET];
  int ret;
  int cnt;

  if (count > TLSTEST_MAX_SOCKET)
    {
      printf("Test case error : count = %d, max = %d\n", count, TLSTEST_MAX_SOCKET);
      return;
    }
  memset(sockfd, -1, TLSTEST_MAX_SOCKET);

#ifdef TLSTEST_ENABLE_CAFILE
  ssl_init(tlstest_calist, TLSTEST_SSL_VERIFY_REQUIRED);
#else
  ssl_init(NULL, TLSTEST_SSL_VERIFY_REQUIRED);
#endif

  for (cnt=0; cnt<count; cnt++)
    {
      /* SSL connect */
      printf("Start to SSL connect (ch:%d) : [%s:%s]\n", cnt+1, host, port);
      ret = ssl_connect(TLSTEST_SSL_TCP, host, port, &sockfd[cnt]);
      if (ret < 0)
        {
          printf("[ERR] ssl_conenct error (ch:%d) : %X\n", cnt+1, ret);
          goto ssl_disconnect;
        }
    }

  if (path != NULL)
    {
      for (cnt=0; cnt<count; cnt++)
        {
          /* HTTPS GET */
          ret = ssl_sendrecv(sockfd[cnt], TLSTEST_SSL_GET, host, path, NULL);
          if (ret < 0)
            {
              printf("[ERR] ssl_sendrecv error (ch:%d) : %X\n", cnt+1, ret);
              goto ssl_disconnect;
            }
        }
    }

ssl_disconnect:
  for (cnt=0; cnt<count; cnt++)
    {
      if (sockfd[cnt] != -1)
        {
          /* SSL disconnect */
          ssl_disconnect(sockfd[cnt]);
        }
    }

  ssl_fin();
  return;
}

/****************************************************************************
 * Common function : INIT -> CONNECT -> (GET) -> DISCONNECT -> FIN
 ****************************************************************************/
static void tlstest_common(char *host, char *port, char *path, int flag)
{
  int sockfd;
  int ret;

  /* Multi channel test */
  if ((flag != TLSTEST_FLAG_SUCCESS) && (flag != TLSTEST_FLAG_FAIL))
    {
      return tlstest_multi(host, port, path, flag);
    }

#ifdef TLSTEST_ENABLE_CAFILE
  ssl_init(tlstest_calist, TLSTEST_SSL_VERIFY_REQUIRED);
#else
  ssl_init(NULL, TLSTEST_SSL_VERIFY_REQUIRED);
#endif

  /* SSL connect */
  printf("Start to SSL connect : [%s:%s]\n", host, port);
  ret = ssl_connect(TLSTEST_SSL_TCP, host, port, &sockfd);
  if (ret < 0)
    {
      if (flag == TLSTEST_FLAG_SUCCESS)
        {
          printf("[ERR] ssl_conenct error : %X\n", ret);
        }
      else
        {
          printf("[OK] ssl_conenct error : %X\n", ret);
        }
      goto ssl_fin;
    }
  else if (flag == TLSTEST_FLAG_FAIL)
    {
      printf("[ERR] ssl_conenct illegal success\n");
      goto ssl_disconnect;
    }

  if (path != NULL)
    {
      /* HTTPS GET */
      ret = ssl_sendrecv(sockfd, TLSTEST_SSL_GET, host, path, NULL);
      if (ret < 0)
        {
          printf("[ERR] ssl_sendrecv error : %X\n", ret);
          goto ssl_disconnect;
        }
    }

ssl_disconnect:
  /* SSL disconnect */
  ssl_disconnect(sockfd);

ssl_fin:
  ssl_fin();
  return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int tlstest_main(int argc, char *argv[])
#endif
{
  int ret;
  int cnt;
  int exe = 0;
  struct tlstest_t *site;

  if (g_status == 0)
    {
      /* LTE connect */ 
      printf("Start to LTE connect\n");
      ret = lte_init();
      if (ret < 0)
        {
          return -1;
        }
      g_status = 1;
    }

  site = &tlstest_site[0];
  while (site->code)
    {
      /* Main TLS tests */
      if ((argc > 1) && (strncmp(argv[1], site->code, 2) == 0))
        {
          for (cnt=0; cnt<site->loop; cnt++)
            {
              tlstest_common(site->host, site->port, site->path, site->flag);
              exe++;
            }
        }
      site++;
    }
 
  if (exe == 0)
    {
      printf("Not exist test funcion : %s\n", argv[1]);
      printf("Funcion list :\n");
      printf("[Library]     : 00 - 04\n");
      printf("[Certificate] : 10 - 29\n");
      printf("[ClientCert]  : 30 - 31\n");
      printf("[CipherSuite] : 32 - 39\n");
      printf("[KeyExchange] : 40 - 46\n");
      printf("[Protocol]    : 47 - 48\n");
      printf("[Transparent] : 50 \n");
      printf("[Upgrade]     : 51 - 55\n");
      printf("[UI]          : 56 - 58\n");
      printf("[KnownBad]    : 60 - 68\n");
      printf("[SSL LABS]    : 70 - 77\n");
      printf("(not satisfy) : 15, 16, 19, 24, 25, 31, 44, 45\n"); 
    }

  return 0;
}
