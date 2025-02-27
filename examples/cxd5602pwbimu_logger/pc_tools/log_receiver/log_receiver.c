/****************************************************************************
 * examples/cxd5602pwbimu_logger/pc_tool/log_receiver/log_receiver.c
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


/*****************************************************************************
 * Include Files
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "../../server_conf.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* #define STORE_FORMAT_BIN */
/* #define FLOAT_DECODED */

#define FS (19200000) /* Frequency of timestamp */

/*****************************************************************************
 * Private Data Types
 *****************************************************************************/

union data_u
{
  struct senddata_block_s pkt;
  uint8_t stream[sizeof(struct senddata_block_s)];
};

union conv_f2u_u
{
  float    f;
  uint32_t u;
};
typedef union conv_f2u_u conv_f2u_t;


/*****************************************************************************
 * Private Data
 *****************************************************************************/

static uint32_t delim_data = DELIMITER_CODE;

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

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

static int receive_size(int sock, char *buf, int sz)
{
  int r, ttl;
  ttl = 0;

  while (ttl < sz)
    {
      r = recv(sock, &buf[ttl], sz - ttl, 0);
      if (r < 0)
        {
          return -1;
        }

      ttl += r;
    }

  return ttl;
}

static int is_sync_header(char *s)
{
  char *delim_str = (char *)&delim_data;

  return s[0] == delim_str[0] &&
         s[1] == delim_str[1] &&
         s[2] == delim_str[2] &&
         s[3] == delim_str[3];
}

static int receive_data(int sock, struct senddata_block_s *data)
{
  char delim[4];

  if (receive_size(sock, delim, 4) != 4)
    {
      return -1;
    }

  while (!is_sync_header(delim))
    {
      memmove(&delim[0], &delim[1], 3);
      if (receive_size(sock, &delim[3], 1) != 1)
        {
          return -1;
        }
    }

  data->delimiter = delim_data;
  if (receive_size(sock, (char *)&data->timestamp,
                   sizeof(struct senddata_block_s) - 4) ==
                   sizeof(struct senddata_block_s) - 4)
    {
      return sizeof(struct senddata_block_s);
    }

  return -1;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

int main(int argc, char **argv)
{
  int ret;
  int sockfd;
  unsigned int not_dump_data = 1;
  char dump_str[1024];
#ifdef FLOAT_DECODED
  bool valid_first_ts = false;
  uint32_t last_ts;
  float offset_ts;
#endif

  FILE *fp;
  struct senddata_block_s data;

  if (argc < 3)
    {
      printf("Usage : %s <ipaddr> <filename> (-d)\n", argv[0]);
      return -1;
    }

  if (argc >= 4)
    {
      not_dump_data = 0;
    }

  fp = fopen(argv[2], "w");
  if (!fp)
    {
      printf("Could not open %s\n", argv[2]);
      return -1;
    }

  sockfd = connect_server(argv[1]);
  if (sockfd < 0)
    {
      printf("Connection error : %s(%d)\n", argv[1], SERVER_PORT_NUM);
      fclose(fp);
      return -1;
    }

  while (1)
    {
      while (sockfd < 0)
        {
          sockfd = connect_server(argv[1]);
        }

      while (receive_data(sockfd, &data) > 0)
        {
#ifdef FLOAT_DECODED
          if (valid_first_ts)
            {
              /* For avoiding wrap round 32bit value */

              offset_ts = offset_ts +
                          ((float)(data.timestamp - last_ts) / FS);
              last_ts = data.timestamp;
            }
          else
            {
              last_ts   = data.timestamp;
              offset_ts = 0.;
              valid_first_ts = true;
            }

          snprintf(dump_str, sizeof(dump_str),
                   "%8.5f,%8.5f,%8.5f,%8.5f,%8.5f,%8.5f,%8.5f,%8.5f\n",
                   offset_ts, data.temp,
                   data.accel[0], data.accel[1], data.accel[2],
                   data.gyro[0], data.gyro[1], data.gyro[2]);
#else
          snprintf(dump_str, sizeof(dump_str),
                   "%08x,%08x,%08x,%08x,"
                   "%08x,%08x,%08x,%08x\n",
                   data.timestamp,
                   ((conv_f2u_t)data.temp).u,
                   ((conv_f2u_t)data.gyro[0]).u,
                   ((conv_f2u_t)data.gyro[1]).u,
                   ((conv_f2u_t)data.gyro[2]).u,
                   ((conv_f2u_t)data.accel[0]).u,
                   ((conv_f2u_t)data.accel[1]).u,
                   ((conv_f2u_t)data.accel[2]).u);
#endif

          if (!not_dump_data)
            {
              /* CSV Style dump */

              printf(dump_str);
            }
          else
            {
              not_dump_data++;

              if (not_dump_data == 0)
                {
                  not_dump_data = 1;
                }

              if ((not_dump_data & (0x1FFF)) == 0)
                {
                  printf("Num: %d TS: 0x%08x\n",
                         not_dump_data, data.timestamp);
                  printf(dump_str);
                }
            }

#ifdef STORE_FORMAT_BIN
          fwrite(&data.timestamp, 1,
                 (sizeof(struct senddata_block_s) - sizeof(uint32_t)), fp);
#else
          fprintf(fp, dump_str);
#endif
        }

      close(sockfd);
      sockfd = -1;
    }

  return 0;
}
