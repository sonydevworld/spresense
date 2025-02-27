/****************************************************************************
 * examples/cxd5602pwbimu_logger/log_server.c
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
 * Include Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <nuttx/sensors/cxd5602pwbimu.h>
#include "server_conf.h"

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct connection_info_s
{
  int s_sock;
  int sock;
  pthread_mutex_t lock;
  pthread_t thd;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct senddata_block_s g_blk;
static struct connection_info_s g_conn;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int get_clientsock(struct connection_info_s *info)
{
  int ret;
  pthread_mutex_lock(&info->lock);
  ret = info->sock;
  pthread_mutex_unlock(&info->lock);
  return ret;
}

static void set_clientsock(struct connection_info_s *info, int sock)
{
  pthread_mutex_lock(&info->lock);
  info->sock = sock;
  pthread_mutex_unlock(&info->lock);
}

static int wait_connection(int s_sock, struct sockaddr_in *client)
{
  int c_sock;
  socklen_t len = sizeof(struct sockaddr_in);

  /* accept TCP connection from client */

  c_sock = accept(s_sock, (struct sockaddr *)client, &len);

  return c_sock;
}

static void *server_daemon(void *param)
{
  int sock;
  struct connection_info_s *info = (struct connection_info_s *)param;
  struct sockaddr_in client;

  while (1)
    {
      sock = get_clientsock(info);
      if (sock >= 0)
        {
          /* Client is already exists. Just wait */

          sleep(1);
        }
      else
        {
          printf("Waiting for connection from a client\n");
          sock = wait_connection(info->s_sock, &client);
          if (sock < 0)
            {
              break;
            }
          else
            {
              printf("Connected\n");
              set_clientsock(info, sock);
            }
        }
    }

  return NULL;
}

static pthread_t start_server(struct connection_info_s *info)
{
  pthread_attr_t attr;
  struct sched_param sparam;

  info->thd = -1;
  info->sock = -1;
  pthread_mutex_init(&info->lock, NULL);
  
  pthread_attr_init(&attr);
  sparam.sched_priority = 110;
  pthread_attr_setschedparam(&attr, &sparam);
  pthread_create(&info->thd, &attr, server_daemon, (void *)info);

  return info->thd;
}

static int send_binary(int s, const char *data, int len)
{
  int ret;
  int sending_len;
  int sent_len = 0;

  while (sent_len < len)
    {
      sending_len = len - sent_len;
      ret = write(s, data, sending_len);

      if (ret == 0)
        {
          usleep(1);
        }

      if (ret < 0)
        {
          return ret;
        }

      sent_len += ret;
      data += ret;
    }

  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int logsvr_initserver(int port_num)
{
  int ret;
  struct sockaddr_in addr;

  /* make socket */

  g_conn.s_sock = -1;
  g_conn.s_sock = socket(AF_INET, SOCK_STREAM, 0);
 
  if (g_conn.s_sock < 0)
    {
      printf("Error. Cannot make socket\n");
      return -1;
    }
    
  /* socket setting */

  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(port_num);
  addr.sin_addr.s_addr = INADDR_ANY;

  /* binding socket */    

  ret = bind(g_conn.s_sock, (struct sockaddr *)&addr, sizeof(addr));
    
  if (ret < 0)
    {
      printf("Error. Cannot bind socket\n");
      close(g_conn.s_sock);
      return -1;
    }
 
  /* listen socket */

  listen(g_conn.s_sock, 1);

  if (start_server(&g_conn) < 0)
    {
      close(g_conn.s_sock);
      g_conn.s_sock = -1;
    }

  return g_conn.s_sock;
}

int logsvr_sendimudata(cxd5602pwbimu_data_t *dat, int num)
{
  int i;
  int sock;

  g_blk.delimiter  = DELIMITER_CODE;

  sock = get_clientsock(&g_conn);

  if (sock >= 0)
    {
      for (i = 0; i < num; i++)
        {
          g_blk.timestamp = dat[i].timestamp;
          g_blk.temp      = dat[i].temp;
          g_blk.gyro[0]   = dat[i].gx;
          g_blk.gyro[1]   = dat[i].gy;
          g_blk.gyro[2]   = dat[i].gz;
          g_blk.accel[0]  = dat[i].ax;
          g_blk.accel[1]  = dat[i].ay;
          g_blk.accel[2]  = dat[i].az;

          if (send_binary(sock, (const char *)&g_blk, sizeof(g_blk)) < 0)
            {
              set_clientsock(&g_conn, -1);
            }
        }
    }

  return OK;
}

void logsvr_shutdown(void)
{
  if (g_conn.s_sock >= 0)
    {
      /* Clean-up resources on server daemon */

      close(g_conn.s_sock);
      g_conn.s_sock = -1;

      pthread_mutex_lock(&g_conn.lock);

      if (g_conn.sock >= 0)
        {
          close(g_conn.sock);
          g_conn.sock = -1;
        }

      pthread_mutex_unlock(&g_conn.lock);

      /* Kill server thread */

      pthread_kill(g_conn.thd, SIGKILL);

      /* Make sure server is killed */

      pthread_join(g_conn.thd, NULL);
      g_conn.thd = -1;
    }
}
