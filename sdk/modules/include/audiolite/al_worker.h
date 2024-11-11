/****************************************************************************
 * modules/include/audiolite/al_worker.h
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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

#ifndef __INCLUDE_AUDIOLITE_WORKER_H
#define __INCLUDE_AUDIOLITE_WORKER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/queue.h>
#include <mossfw/mossfw_lock.h>
#include <audiolite/al_memalloc.h>
#include <audiolite/alworker_comm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CHECK_HDR(hdr, g, c) \
    (((hdr).grp == AL_COMM_MESSAGE_##g) && \
     ((hdr).code == AL_COMM_MSGCODE##c))

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

typedef int (*alwkr_msghandler_t)(al_comm_msghdr_t hdr,
                                  al_comm_msgopt_t *opt, void *arg);

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_workermemq
 ****************************************************************************/

class audiolite_workermemq
{
  private:
    int _max_qsz;
    volatile bool _en;
    mossfw_lock_t _lock;
    mossfw_condition_t _cond;
    struct dq_queue_s _mem_proc;

  public:
    audiolite_workermemq(int maxsz);
    ~audiolite_workermemq();

    void enable();
    void disable();

    void push(audiolite_memapbuf *mem);
    audiolite_memapbuf *pop();
    audiolite_memapbuf *pop(unsigned char *addr);
    int get_qsize() { return _max_qsz; };
    int current_sz() { return dq_count(&_mem_proc); };
};

/****************************************************************************
 * class: audiolite_worker
 ****************************************************************************/

class audiolite_worker
{
  private:
    mossfw_thread_t _tid;
    mossfw_lock_t _lock;
    void *_hdlr_arg;
    alwkr_msghandler_t _hdlr;
    bool _running;
    al_wtask_t _wtask;

    static void *msg_receiver(void *arg);

    bool is_running();
    void mod_running(bool running);
    int wait_bootmsg();

  public:
    audiolite_worker();
    ~audiolite_worker();

    int bringup_worker(const char *dspname, bool is_spk = false,
                       const char *rcvname = "alworker",
                       int prio = CONFIG_AUDIOLITE_WSVR_DEFPRIO,
                       int stksz = CONFIG_AUDIOLITE_WSVR_DEFSTACK);
    void terminate_worker();
    void set_msghandler(alwkr_msghandler_t hdr, void *arg);
    
    al_wtask_t *getwtask() { return &_wtask; };
};

#endif /* __INCLUDE_AUDIOLITE_WORKER_H */
