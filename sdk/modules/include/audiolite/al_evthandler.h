/****************************************************************************
 * modules/include/audiolite/al_evthandler.h
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

#ifndef __INCLUDE_AUDIOLITE_EVTHANDLER_H
#define __INCLUDE_AUDIOLITE_EVTHANDLER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/queue.h>
#include <audiolite/al_singleton.h>
#include <audiolite/al_eventlistener.h>
#include <audiolite/al_memalloc.h>

/****************************************************************************
 * Class Pre-definitions
 ****************************************************************************/

class audiolite_component;

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

/****************************************************************************
 * class: audiolite_evthandler
 ****************************************************************************/

class audiolite_evthandler
{
  private:
    SINGLETON_MEMBER(audiolite_evthandler);

    int _fs;
    int _chnum;
    int _bitwidth;
    audiolite_eventlistener *_listen;
    audiolite_mempoolsysmsg *_pool;
    mossfw_input_t *_receiver;
    mossfw_output_t *_accepter;
    mossfw_callback_op_t *_op;

    audiolite_evthandler(int memnum = 8);
    ~audiolite_evthandler();

    static int event_handler(mossfw_callback_op_t *op, unsigned long arg);

  public:
    SINGLETON_METHODS(audiolite_evthandler);

    int publish_event(int evtid, audiolite_component *issuer, unsigned long arg);
    int set_systemparam(int fs, int bitwidth, int chnum);

    void set_evtlistener(audiolite_eventlistener *l) { _listen = l; };
    int get_fs(){ return _fs; };
    int get_chnum(){ return _chnum; };
    int get_bitwidth(){ return _bitwidth; };
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void audiolite_set_evtlistener(audiolite_eventlistener *l);
int audiolite_set_systemparam(int fs, int chnum, int bitwidth);
void audiolite_eventdestroy();

#endif /* __INCLUDE_AUDIOLITE_EVTHANDLER_H */
