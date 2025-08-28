/****************************************************************************
 * examples/ahrs_pwbimu/ahrs_receiver_main.c
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
#include <errno.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "server_port.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int init_server(int port_num)
{
  int ret;
  int s_sock;
  struct sockaddr_in addr;

  /* make socket */

  s_sock = socket(AF_INET, SOCK_STREAM, 0);

  if (s_sock < 0)
    {
      printf("Error. Cannot make socket\n");
      return -1;
    }

  /* socket setting */

  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(port_num);
  addr.sin_addr.s_addr = INADDR_ANY;

  /* binding socket */

  ret = bind(s_sock, (struct sockaddr *)&addr, sizeof(addr));

  if (ret < 0)
    {
      printf("Error. Cannot bind socket\n");
      close(s_sock);
      return -1;
    }

  /* listen socket */

  listen(s_sock, 5);

  return s_sock;
}

static int wait_connection(int s_sock, struct sockaddr_in *client)
{
  socklen_t len = sizeof(struct sockaddr_in);

  /* accept TCP connection from client */

  return accept(s_sock, (struct sockaddr *)client, &len);
}

static int receive_data(int sock, char *buf, int sz)
{
  int r, ttl;
  ttl = 0;

  while (ttl < sz)
    {
      r = recv(sock, &buf[ttl], sz - ttl, 0);
      if (r < 0)
        {
          printf("Recv Error %d : %02x\n", r, errno);
          return -1;
        }

      ttl += r;
    }

  return ttl;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  int ssock;
  int csock;
  struct sockaddr_in client;
  float q[4];

  ssock = init_server(PORT_AHRS_RCVSERVER);

  csock = -1;
  while (1)
    {
      printf("Wait connection\n");
      csock = wait_connection(ssock, &client);
      printf("Connected!\n");
      if (csock >= 0)
        {
          while (receive_data(csock, (char *)q, sizeof(q)) == sizeof(q))
            {
              printf("%08x,%08x,%08x,%08x\n", *(unsigned int *)&q[0],
                                              *(unsigned int *)&q[1],
                                              *(unsigned int *)&q[2],
                                              *(unsigned int *)&q[3]);
            }

          sleep(1);
          close(csock);
          csock = -1;
        }
    }

  return 0;
}
