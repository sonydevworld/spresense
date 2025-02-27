/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/netserver.cxx
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
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <signal.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <audiolite/audiolite.h>
#include <netserver.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void *connection_server(void *param)
{
  int sock;
  netserver *srv = (netserver *)param;
  struct sockaddr_in client;

  while (1)
    {
      sock = srv->get_clientsock();
      if (sock >= 0)
        {
          /* Client is already exists. */

          srv->sleep_until_lostconnect();
        }
      else
        {
          printf("Wait for connection..\n");
          sock = srv->wait_connection(&client);
          if (sock < 0)
            {
              break;
            }
          else
            {
              printf("Connected from %s:%d\n",
                     inet_ntoa(client.sin_addr),
                     ntohs(client.sin_port));
              srv->set_clientsock(sock);
            }
        }
    }

  printf("EXIT netserver\n");
  return NULL;
}

/****************************************************************************
 * netserver Class Methods
 ****************************************************************************/

int netserver::send_data(uint8_t *data, uint32_t len)
{
  int sock;
  uint32_t sent_sz = 0;
  int sz;

  sock = get_clientsock();

  if  (sock >= 0)
    {
      while (len > sent_sz)
        {
          sz = send(sock, &data[sent_sz], len - sent_sz, 0);
          if (sz < 0)
            {
              set_clientsock(-1);
              return -errno;
            }

          sent_sz += sz;
        }
    }

  return sent_sz;
}

int netserver::get_clientsock(void)
{
  int ret;
  pthread_mutex_lock(&c_lock);
  ret = c_sock;
  pthread_mutex_unlock(&c_lock);
  return ret;
}

void netserver::set_clientsock(int sock)
{
  pthread_mutex_lock(&c_lock);
  c_sock = sock;
  if (c_sock < 0)
    {
      pthread_cond_signal(&c_cond);
    }
  pthread_mutex_unlock(&c_lock);
}

void netserver::sleep_until_lostconnect(void)
{
  pthread_mutex_lock(&c_lock);
  while (c_sock >= 0) // Sleep until client sock is available
    {
      pthread_cond_wait(&c_cond, &c_lock);
    }
  pthread_mutex_unlock(&c_lock);
}

int netserver::wait_connection(struct sockaddr_in *client)
{
  int sock;
  socklen_t len = sizeof(struct sockaddr_in);

  /* accept TCP connection from client */

  sock = accept(s_sock, (struct sockaddr *)client, &len);

  return sock;
}

pthread_t netserver::init_server(int port_num)
{
  int ret;
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

  ret = ::bind(s_sock, (struct sockaddr *)&addr, sizeof(addr));
    
  if (ret < 0)
    {
      printf("Error. Cannot bind socket\n");
      close(s_sock);
      return -1;
    }
 
  /* listen socket */

  listen(s_sock, 5);

  return start_server();
}

pthread_t netserver::start_server(void)
{
  pthread_t thd;
  pthread_attr_t attr;
  struct sched_param sparam;

  pthread_attr_init(&attr);
  sparam.sched_priority = 110;
  pthread_attr_setschedparam(&attr, &sparam);
  pthread_create(&thd, &attr, connection_server, (void *)this);

  return thd;
}

void netserver::on_data(void)
{
  audiolite_memapbuf *mem = (audiolite_memapbuf *)pop_data();
  if (mem)
    {
      send_data((uint8_t *)mem->get_data(), mem->get_storedsize());
      mem->release();
    }
}

netserver::netserver(int port) : audiolite_component(),
                                 s_sock(-1), c_sock(-1), s_thd(-1)
{
  pthread_mutex_init(&c_lock, NULL);
  pthread_cond_init(&c_cond, NULL);

  s_thd = init_server(port);
}

netserver::~netserver()
{
}
