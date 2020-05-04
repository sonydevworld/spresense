/****************************************************************************
 * examples/multi_webcamera/multiwebcam_server.c
 *
 *   Copyright 2019, 2020 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "multiwebcam_server.h"
 
#define MAX_SENDING_LEN (1500)

static int send_binary(int s, const char *data, int len)
{
  int ret;
  int sending_len;
  int sent_len = 0;

  while (sent_len < len)
    {
      sending_len = len - sent_len;
      sending_len = ( sending_len > MAX_SENDING_LEN ) ? MAX_SENDING_LEN : sending_len;
      ret = write(s, data, sending_len);
      if (ret < 0)
        {
          return ret;
        }
      sent_len += ret;
      data += ret;
    }

  return len;
}

static int send_string(int s, const char *msg)
{
  int len = strlen(msg);

  return send_binary(s, msg, len);
}

int multiwebcam_initserver(int port_num)
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

int multiwebcam_waitconnection(int s_sock, struct sockaddr_in *client)
{
  int c_sock;
  socklen_t len = sizeof(struct sockaddr_in);

  /* accept TCP connection from client */

  c_sock = accept(s_sock, (struct sockaddr *)client, &len);

  return c_sock;
}

#ifdef CONFIG_EXAMPLES_MULTIWEBCAM_USE_HTTPMJPEG

/* If you set USE_HTTP_MJPEG config,
 * Use image send protocol as Motion JPEG over HTTP.
 */

#define CRLF "\r\n"

#define HTTP_MJPEG_HEADER "HTTP/1.1 200 OK" CRLF \
  "Content-Type: multipart/x-mixed-replace; boundary=--MOBOTIX_Fast_Serverpush" \
  CRLF CRLF

#define HTTP_MJPEG_PART_HEADER "--MOBOTIX_Fast_Serverpush" CRLF \
  "Content-Length: "

#define HTTP_MJPEG_PART_HEADER2 CRLF "Content-type: image/jpeg" CRLF CRLF

int multiwabcam_sendheader(int c_sock)
{
  return send_string(c_sock, HTTP_MJPEG_HEADER);
}

static int send_midheader(int s, int sz)
{
  char data_size[16];
  sprintf(data_size, "%d", sz);

  if (send_string(s, HTTP_MJPEG_PART_HEADER) < 0)
    {
      return -1;
    }

  if (send_string(s, data_size) < 0)
    {
      return -1;
    }

  if (send_string(s, HTTP_MJPEG_PART_HEADER2) < 0)
    {
      return -1;
    }

  return 0;
}

int multiwebcam_sendframe(int c_sock, char *jpg, int jpg_len)
{
  if (send_midheader(c_sock, jpg_len) < 0)
    {
      return -1;
    }

  if (send_binary(c_sock, jpg, jpg_len) < 0)
    {
      return -1;
    }

  if (send_string(c_sock, CRLF) < 0)
    {
      return -1;
    }

  return 0;
}

#else   /* CONFIG_EXAMPLES_MULTIWEBCAM_USE_HTTPMJPEG */

/* If you DIDN'T set USE_HTTP_MJPEG config,
 * Use original simple protocol to send image.
 */

int multiwabcam_sendheader(int c_sock)
{

  /* Do nothing for this protocol. */

  return 0;
}

static int send_midheader(int s, int sz)
{
  if (send_string(s, "SZ: ") < 0)
    {
      return -1;
    }

  if (send_binary(s, (char *)&sz, 4) < 0)
    {
      return -1;
    }

  return 0;
}

int multiwebcam_sendframe(int c_sock, char *jpg, int jpg_len)
{
  if (send_midheader(c_sock, jpg_len) < 0)
    {
      return -1;
    }

  if (send_binary(c_sock, jpg, jpg_len) < 0)
    {
      return -1;
    }

  return 0;
}

#endif  /* CONFIG_EXAMPLES_MULTIWEBCAM_USE_HTTPMJPEG */
