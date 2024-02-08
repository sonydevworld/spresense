/****************************************************************************
 * examples/audiolite_rec2net/host/storage_server.c
 *
 *   Copyright 2024 Sony Semiconductor Solutions Corporation
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

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SERVER_PORT (12345)     /* Server Port number */
#define END_MARK    "12345678"  /* Data end marker */

#define DATA_MAX  (8192)

/****************************************************************************
 * Privete Data
 ****************************************************************************/

static char g_rcvdata[DATA_MAX];

/****************************************************************************
 * Privete Functions
 ****************************************************************************/

/* open_datafile(): Open a file to store received audio data */

static FILE *open_datafile(int idx)
{
  FILE *fp;
  char filename[256];

  sprintf(filename, "idx%04d.raw", idx);
  fp = fopen(filename, "wb");
  printf("Open : %s\n", filename);

  return fp;
}

/* dump_top_data(): Dump top 4 samples on a terminal */

static void dump_top_data(void)
{
  short *data = (short *)g_rcvdata;
  printf("%8d, %8d, %8d, %8d\n",
         data[0], data[1], data[2], data[3]);
}

/* recv_data(): Receive data from Spresense */

static void recv_data(int sock, int idx)
{
  int ttl_size = 0;
  int sz = 0;
  FILE *fp;

  fp = open_datafile(idx); /* Open a file to store receiving data */
  if (fp != NULL)
    {
      while (sz >= 0)
        {
          sz = recv(sock, g_rcvdata, DATA_MAX, 0);

          if (sz < 0)
            {
              printf("failed to recv data(errno=%d:%s)\n",
                    errno, strerror(errno));
              break;
            }
          else if (!strncmp(g_rcvdata, END_MARK, strlen(END_MARK)))
            {
              break;
            }

          dump_top_data();
          fwrite(g_rcvdata, 1, sz, fp);
          ttl_size += sz;
        }
    }

  close(sock);
  fclose(fp);

  printf("Saved :%d bytes\n", ttl_size);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  int ret;
  int idx;
  int yes = 1;

  int srv_sock = -1;
  int clnt_sock = -1;

  struct sockaddr_in srvadr, clntadr;
  socklen_t socklen;

  /* Clear socket address */

  memset(&srvadr, 0, sizeof(srvadr));
  memset(&clntadr, 0, sizeof(clntadr));
  socklen = sizeof(clntadr);

  /* Create server socket */

  srv_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (srv_sock == -1)
    {
      printf("failed to socket(errno=%d:%s)\n", errno, strerror(errno));
      return -1;
    }

  /* Set socket option for reusing port number */

  setsockopt(srv_sock, SOL_SOCKET, SO_REUSEADDR,
            (const char *)&yes, sizeof(yes));

  /* Bind socket to the port */

  srvadr.sin_family = AF_INET;
  srvadr.sin_addr.s_addr = INADDR_ANY;
  srvadr.sin_port = htons(SERVER_PORT);

  ret = bind(srv_sock, (const struct sockaddr *)&srvadr, sizeof(srvadr));
  if (ret == -1)
    {
      printf("failed to bind(errno=%d:%s)\n", errno, strerror(errno));
      close(srv_sock);
      return -1;
    }

  /* Listen the socket */

  ret = listen(srv_sock, 1);
  if (ret == -1)
    {
      printf("failed to listen(errno=%d:%s)\n", errno, strerror(errno));
      close(srv_sock);
      return -1;
    }

  idx = 0;  /* Initialize file index number */

  /* Main loop */

  while(1)
    {
      /* Wait for connection from a client */

      printf("accept wating...\n");
      clnt_sock = accept(srv_sock, (struct sockaddr *)&clntadr, &socklen);
      if (clnt_sock == -1)
        {
          printf("failed to accept(errno=%d:%s)\n", errno, strerror(errno));
          continue;
        }

      /* Client has connected */

      printf("Accepted\n");

      /* Receiving data form the client until receiving END_MARK */

      recv_data(clnt_sock, idx);

      /* Close client socket */

      close(clnt_sock);
      clnt_sock = -1;
      idx++;
    }

  return 0;
}
