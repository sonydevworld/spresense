/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/netserver.h
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

#ifndef __EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_NETSERVER_H
#define __EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_NETSERVER_H

#include <pthread.h>
#include <audiolite/audiolite.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SERVER_PORT (12345)

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

class netserver : public audiolite_component
{
  int s_sock;
  int c_sock;
  pthread_t s_thd;
  pthread_mutex_t c_lock;
  pthread_cond_t c_cond;

  protected:
    pthread_t init_server(int port_num);
    pthread_t start_server(void);
    int send_data(uint8_t *data, uint32_t len);

  public:
    netserver(int port = SERVER_PORT);
    virtual ~netserver();

    int get_clientsock(void);
    void set_clientsock(int sock);
    int wait_connection(struct sockaddr_in *client);
    void sleep_until_lostconnect();

    virtual void on_data();
};

#endif /* __EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_NETSERVER_H */
