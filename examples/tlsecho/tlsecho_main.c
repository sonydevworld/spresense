/****************************************************************************
 * examples/tlsecho/tlsecho_main.c
 *
 *   Copyright 2025 Sony Semiconductor Solutions Corporation
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

#include <sslutils/sslutil.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUFSIZE (1024)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sslutil_tls_context g_ctx;
static char g_sbuf[BUFSIZE];
static char g_rbuf[BUFSIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void print_usage(const char *app)
{
  printf("Usage: nsh> %s <IP addr or hostname> <port num>\n", app);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret = 0;
  struct webclient_tls_ops *ops;
  struct webclient_tls_connection *connp;

  if (argc != 3)
    {
      print_usage(argv[0]);
      return -1;
    }

  /* Initialize sslutil */

  SSLUTIL_CTX_INIT(&g_ctx);
  ops = sslutil_webclient_tlsops();
  if (ops)
    {
      /* Connect TLS to the host */

      if (ops->connect(&g_ctx, argv[1], argv[2], -1, &connp) == OK)
        {
          while (1)
            {
              /* Wait input from terminal */

              if (fgets(g_sbuf, sizeof(g_sbuf), stdin) == NULL)
                {
                  printf("Failed to input!\n");
                  break;
                }
              else
                {
                  ret = strlen(g_sbuf);

                  if (ret > 0 && g_sbuf[ret - 1] == '\n')
                    {
                      g_sbuf[ret - 1] = '\0';  /* remove a LF('\n') by fgets. */
                    }
                }

              if (ret == 0 || ret == 1)
                {
                  /* No character only by Enter input. */

                  continue;
                }

              /* Send the data to the server */

              ops->send(&g_ctx, connp, g_sbuf, strlen(g_sbuf));

              /* Receive echo data from the server */

              ret = ops->recv(&g_ctx, connp, g_rbuf, BUFSIZE);
              g_rbuf[ret] = '\0';

              printf("From Server >>> %s <<<\n", g_rbuf);

              /* Simple termination way: if server send 'q' on top of the data,
               * Shutdown the connection, and finish this app.
               */

              if (g_rbuf[0] == 'q')
                {
                  break;
                }
            }

          ops->close(&g_ctx, connp);
        }
      else
        {
          printf("Error tls connection: Host:%s  Port:%s\n", argv[1], argv[2]);
        }
    }
  else
    {
      printf("Error sslutil_webclient_tlsops()\n");
    }

  printf("End\n");

  return 0;
}
