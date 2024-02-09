/****************************************************************************
 * examples/audiolite_rec2net/audiolite_rec2net_main.cxx
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

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <audiolite/audiolite.h>

#include "event_str.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SERVER_PORT (12345)
#define END_MARK  "12345678"

/****************************************************************************
 * Privete Class
 ****************************************************************************/

/* For Event receiving */

class my_netsendlistener : public audiolite_eventlistener
{
  public:
    void on_event(int evt, audiolite_component *cmp,
                  unsigned long arg)
    {
      printf("Event %s is happened : %d\n", convert_evtid(evt),
                                            (int)arg);
    }
};

/* For sending audio data after receiving from mic */

class netsender : public audiolite_component
{
  public:
    int sock;
    netsender(const char *ip = NULL);

    int connect_to_server(const char *ip);
    int send_data(uint8_t *data, uint32_t len);
    void disconnect();

    /* Inherited member functions from audiolite_component */

    virtual void on_data();
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Class Methods : netsender
 ****************************************************************************/

/* Constructor */

netsender::netsender(const char *ip)
      : audiolite_component(1,0), sock(-1)
{
  if (ip != NULL)
    {
      printf("Connecting to server:%s\n", ip);
      sock = connect_to_server(ip);
      if (sock < 0)
        {
          printf("Could not connect to server : reason(%s)\n",
                 strerror(-sock));
        }
    }
}

/* Member method: connect_to_server */

int netsender::connect_to_server(const char *ip)
{
  int skt;

  struct sockaddr_in sin;
  memset(&sin, 0, sizeof(sin));

  skt = socket(AF_INET, SOCK_STREAM, 0);
  if (skt == -1)
    {
      return -errno;
    }

  sin.sin_family = AF_INET;
  sin.sin_addr.s_addr = inet_addr(ip);
  sin.sin_port = htons(SERVER_PORT);

  if (connect(skt, (const struct sockaddr *)&sin, sizeof(sin)) < 0)
    {
      close(skt);
      return -errno;
    }

  return skt;
}

/* Member method: send_data */

int netsender::send_data(uint8_t *data, uint32_t len)
{
  uint32_t sent_sz = 0;
  int sz;

  if  (sock >= 0)
    {
      while (len > sent_sz)
        {
          sz = send(sock, &data[sent_sz], len - sent_sz, 0);
          if (sz < 0)
            {
              return -errno;
            }

          sent_sz += sz;
        }
    }

  return sent_sz;
}

/* Inherited Member method: on_data */

void netsender::on_data()
{
  /* Receive data from MIC device */

   audiolite_memapbuf *mem = (audiolite_memapbuf *)pop_data();
   if (mem)
     {
       send_data((uint8_t *)mem->get_data(), mem->get_storedsize());
       mem->release();
     }
}

void netsender::disconnect()
{
  if (sock >= 0)
    {
      usleep(5 * 1000);
      printf("Disconnectiong..\n");
      send_data((uint8_t *)END_MARK, strlen(END_MARK));
      usleep(5 * 1000);
      close(sock);
      sock = -1;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C"
int main(int argc, FAR char *argv[])
{
  int ret;
  my_netsendlistener lsn;

  if (argc != 2)
    {
      printf("Usage nsh> %s <IP address to send>\n", argv[0]);
      return -1;
    }

  /* To Create below structure.
   *
   * +-------------------------------------+
   * | my_netsendlistener to listen events |
   * +-------------------------------------+
   *     ^            ^
   *     |            |
   * +-------+    +--------+                               +----+
   * | Audio |    | Net    |                               | PC |
   * | Input | -> | Sender | ------ TCP/IP Network ------> |    |
   * +-------+    +--------+                               +----+
   *     ^                                                   |
   *     |                                                   V
   *  +------+                                         Save to File
   *  |Memroy|                                      PC's server program is
   *  | Pool |                                      in 'host' directory.
   *  +------+
   */

  audiolite_mempoolapbuf *mempool = new audiolite_mempoolapbuf;
  audiolite_inputcomp *aindev = new audiolite_inputcomp;
  netsender *sender = new netsender(argv[1]);

  /* Setup system parameter as
   *   Output Sampling rate : 192K
   *   Output bitwidth for each sample : 16 bits
   *   Output channels : 4 channel
   */

  audiolite_set_systemparam(192000, 16, 4);

  /* Set listener to listen system events */

  audiolite_set_evtlistener(&lsn);

  /* Setup memory pool to receive audio data from the Input device
   * as 8192bytes x 16blocks.
   * Now because of HW limitation, one memory block can be acceptable
   * 1024 samples data. 8912byte = 1024 * 4ch * 2byte/sample
   */

  mempool->create_instance(8192, 16);

  /* Audio Input device setting */

  /* Set memory pool */

  aindev->set_mempool(mempool);

  /* Connect Audio input device to sender
   *     aindev -> sender
   */

  aindev->bind(sender);

  /* Let's Record */

  printf("Start Recording 10 sec\n");
  ret = aindev->start();
  if (ret != OK)
    {
      printf("Start error..: %d\n", ret);
      goto app_error;
    }

  /* If recording time is 10 sec,
   * Let's wait 10 sec.
   */

  for (int i = 0; i < 10; i++)
    {
      printf("."); fflush(stdout);
      sleep(1);
    }

  printf("\n");

  /* Disconnect from the server */

  printf("Disconnect to server\n");
  sender->disconnect();

  /* Stop Recording */

  printf("Stop record\n");
  aindev->stop();

  printf("Clean up\n");
app_error:

  /* Clean up */

  aindev->unbindall();

  printf("Delete instances\n");

  delete aindev;
  delete sender;
  delete mempool;

  printf("Delete system event handler\n");

  audiolite_eventdestroy();

  return 0;
}
