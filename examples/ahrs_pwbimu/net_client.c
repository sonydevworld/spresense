/****************************************************************************
 * examples/ahrs_pwbimu/net_client.c
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
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "server_port.h"
#include "net_client.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ahrs_connect_server(const char *ip)
{
  int sock;
  struct sockaddr_in saddr;

  sock = socket(PF_INET, SOCK_STREAM, 0);
  if (sock >= 0)
    {
      memset(&saddr, 0, sizeof(saddr));
      saddr.sin_family = PF_INET;
      saddr.sin_addr.s_addr = inet_addr(ip);
      saddr.sin_port = htons(PORT_AHRS_RCVSERVER);

      if (connect(sock, (struct sockaddr *)&saddr, sizeof(saddr)) != 0)
        {
          close(sock);
          sock = -1;
        }
    }

  return sock;
}

int ahrs_send_binary(int s, const char *data, int len)
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
          printf("Send 0 byte. Maybe poor reception..\n");
        }

      if (ret < 0)
        {
          printf("Error!! : %d : %d\n", ret, errno);
          return ret;
        }
      sent_len += ret;
      data += ret;
    }

  return len;
}
