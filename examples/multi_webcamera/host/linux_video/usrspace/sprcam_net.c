// SPDX-License-Identifier: GPL-2.0
/*
 *      sprcam_net.c   --  JPEG Receiver and image injection to
 *                         spr_camera driver.
 *
 *      Copyright (c) 2024
 *          Sony Semiconductor Solutions Corporation
 */

/*****************************************************************************
 * Include Files
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
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

#include "../driver/spr_camera.h"

/*****************************************************************************
 * Preprocessor Definitions
 *****************************************************************************/

#define DEV_PATH  "/dev/" SPRCAM_DRVNAME
#define PORT_NUM  (10080)

#define BUFF_SIZE (1024 * 1024)

#define DEBUGGING

/*****************************************************************************
 * Private Data
 *****************************************************************************/

static unsigned char jpgdata[BUFF_SIZE];
static const char delim_str[4] = { 'S', 'Z', ':', ' ' };

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

static int init_video_device(void)
{
  int fd;

  fd = open(DEV_PATH, O_RDWR);
  if (fd < 0)
    {
      printf("Device File <%s> open error\n", DEV_PATH);
    }

  return fd;
}

static int inject_jpgdata(int fd, char *jpg, size_t len)
{
  int ret;
  ret = write(fd, jpg, len);
  return ret;
}

static void finish_video_device(int fd)
{
  close(fd);
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
      saddr.sin_port = htons(PORT_NUM);

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
  return s[0] == delim_str[0] &&
         s[1] == delim_str[1] &&
         s[2] == delim_str[2] &&
         s[3] == delim_str[3];
}

static int receive_jpgdata(int sock, char *buf, uint32_t len)
{
  char delim[4];
  uint32_t jpglen;

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

  if (receive_size(sock, (char *)&jpglen, sizeof(jpglen)) == sizeof(jpglen))
    {
      if (jpglen > len)
        {
          printf("Receiving size is too long (%d)\n", jpglen);
          return -1;
        }

      if (receive_size(sock, buf, (int)jpglen) != (int)jpglen)
        {
          return -1;
        }
    }

  return (int)jpglen;
}

#ifdef DEBUGGING
static void dump_top(unsigned char *dat, int sz)
{
  int i;

  for (i = 0; i < sz; i++)
    {
      printf("0x%02x", *dat++);
      if (i % 16 == 15) printf("\n");
      else printf(", ");
    }

  if (i % 16 != 15) printf("\n");
}
#else
static void dump_top(unsigned char *dat, int sz)
{
}
#endif

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

int main(int argc, char **argv)
{
  int ret;
  int vdevfd;
  int sockfd;
  int jpglen;

  if (argc != 2)
    {
      printf("Usage : %s <ipaddr>\n", argv[0]);
      return -1;
    }

  vdevfd = init_video_device();
  if (vdevfd < 0)
    {
      return -1;
    }

  sockfd = connect_server(argv[1]);
  if (sockfd < 0)
    {
      printf("Connection error : %s(%d)\n", argv[1], PORT_NUM);
      finish_video_device(vdevfd);
      return -1;
    }

  while (1)
    {
      while (sockfd < 0)
        {
          sockfd = connect_server(argv[1]);
        }

      while ((jpglen = receive_jpgdata(sockfd, jpgdata, BUFF_SIZE)) > 0)
        {
          dump_top(jpgdata, 16);
          inject_jpgdata(vdevfd, jpgdata, jpglen);
        }

      close(sockfd);
      sockfd = -1;
    }

  return 0;
}
