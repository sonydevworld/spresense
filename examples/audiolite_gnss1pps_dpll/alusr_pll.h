/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/alusr_pll.h
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

#ifndef __INCLUDE_AUDIOLITE_USER_PLL_H
#define __INCLUDE_AUDIOLITE_USER_PLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <pthread.h>
#include <semaphore.h>

#include <audiolite/al_workercomp.h>
#include <audiolite/al_workercmd.h>
#include <pll/pll_worker_main.h>

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

class alusr_pll : public audiolite_workercomp
{
  protected:
    class pll_msglistener : public audiolite_stdworker_msglistener
    {
      public:
        virtual ~pll_msglistener(){};
        void bootup(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                    int version, void *d);
        void usermsg(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                     al_comm_msghdr_t hdr, al_comm_msgopt_t *opt);
    };

    pll_msglistener _msglsnr;
    audiolite_mempoolapbuf *_outmempool;

    sem_t _sem_wcom;
    pthread_mutex_t _pllcmd_lock;
    pthread_cond_t _pllcmd_cond;
    int _pllcmd_reply;

    bool sendto_worker(al_comm_msgopt_t *opt);

  public:
    alusr_pll();
    virtual ~alusr_pll();
    void start_pll();
    void stop_pll();

    bool send_xferdata(uint8_t *data, unsigned int len);
    bool enable_xfer();
    bool cancel_xfer();
    void deliverr_reply(int code);
    void xfer_done();
};

#endif /* __INCLUDE_AUDIOLITE_USER_PLL_H */
