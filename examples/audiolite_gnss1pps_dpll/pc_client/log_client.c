/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/pc_tool/log_client.c
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

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <poll.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SERVER_PORT_NUM (12345)     /* Server Port number */

#define SAMPLES  (1024)
#define CHNUM    (2)
#define DATASIZE (2)
#define SAMPLE_CH (SAMPLES * DATASIZE)
#define DATA_MAX  (SAMPLE_CH * CHNUM)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lr_data_s
{
  short l;
  short r;
};

union sample_data_u
{
  char data[DATA_MAX];
  short samp[CHNUM][SAMPLES];
  struct lr_data_s samp_lr[SAMPLES];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static union sample_data_u g_rcvdata;
static int en_print = 1;
static int planner_lr = 1;

static char data_csv[1024];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void dump_top_data(int sz, FILE *fp)
{
  int i;
  short *data1 = g_rcvdata.samp[0];
  short *data2 = g_rcvdata.samp[1];

  for (i = 0; i < SAMPLES; i++)
    {
      sprintf(data_csv, "%8d, %8d\n", data1[i], data2[i]);
      if (en_print) printf(data_csv);
      fprintf(fp, data_csv);
    }
}

static void dump_top_data_lr(int sz, FILE *fp)
{
  int i;
  struct lr_data_s *data = g_rcvdata.samp_lr;

  for (i = 0; i < SAMPLES; i++)
    {
      sprintf(data_csv, "%8d, %8d\n", data[i].l, data[i].r);
      if (en_print) printf(data_csv);
      fprintf(fp, data_csv);
    }
}

static int receive_all(int sock, char *data, int len)
{
  int ret;
  int sz = 0;

  while (len > 0)
    {
      ret = recv(sock, &data[sz], len, 0);
      if (ret < 0)
        {
          printf("failed to recv data(errno=%d:%s)\n",
                 errno, strerror(errno));
          return -errno;
        }

      sz += ret;
      len -= ret;
    }

  return sz;
}

static int check_cmd(void)
{ 
  struct pollfd pfd;
  int key;

  /* Check the key-board for existing the loop */
  pfd.fd = fileno(stdin);
  pfd.events = POLLIN;
  poll(&pfd, 1, 0);
  if (pfd.revents & POLLIN)
    {
      switch (getchar())
        {
          case 'q': return 1; 
        }
    }

  return 0;
}

static void recv_and_save(int sock, FILE *fp)
{
  int ttl_size = 0;
  int sz = 0;

  while (sz >= 0 && !check_cmd())
    {
      check_cmd();
      sz = receive_all(sock, g_rcvdata.data, DATA_MAX);
      if (sz < 0)
        {
          printf("Error receiving data... quit\n");
          break;
        }

      if (planner_lr) dump_top_data(sz, fp);
      else dump_top_data_lr(sz, fp);

      ttl_size += sz;
    }

  printf("Saved :%d bytes\n", ttl_size);
}


static int connect_server(const char *ip)
{
  int sock;
  struct sockaddr_in saddr;

  sock = socket(PF_INET, SOCK_STREAM, 0);
  if (sock >= 0)
    {
      memset(&saddr, 0, sizeof(saddr));
      saddr.sin_family = PF_INET;
      saddr.sin_addr.s_addr = inet_addr(ip);
      saddr.sin_port = htons(SERVER_PORT_NUM);

      if (connect(sock, (struct sockaddr *)&saddr, sizeof(saddr)) != 0)
        {
          close(sock);
          sock = -1;
        }
    }

  return sock;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  FILE *fp;
  int sock;

  if (argc != 3 && argc != 4)
    {
      printf("Usage: $ %s <ipaddr> <filepath to save> (-s)\n", argv[0]);
      return -1;
    }

  fp = fopen(argv[2], "wb");
  printf("Open : %s\n", argv[2]);
  if (!fp)
    {
      printf("    Failed...\n");
      return -1;
    }

  sock = connect_server(argv[1]);
  if (sock < 0)
    {
      printf("Could not connect to %s\n", argv[1]);
      fclose(fp);
      return -1;
    }

  if (argc == 4) planner_lr = 0;

  recv_and_save(sock, fp);

  close(sock);
  fclose(fp);

  return 0;
}
