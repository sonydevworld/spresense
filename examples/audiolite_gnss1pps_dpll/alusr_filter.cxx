/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/alusr_filter.cxx
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

#include <audiolite/audiolite.h>
#include <alusr_filter.h>

/****************************************************************************
 * alusr_filter Class Methods
 ****************************************************************************/

alusr_filter::alusr_filter() :
  audiolite_workercomp("filter", 8, 8), _outmempool(NULL),
  _mode(MODE_NORMAL)
{
  set_msglistener(&_msglsnr);
  _outmempool = new audiolite_mempoolapbuf;
  _outmempool->create_instance(1024 * 2 * 2, 8);
  set_mempool(_outmempool);
}

alusr_filter::~alusr_filter()
{
  set_mempool(NULL);
  if (_outmempool)
    delete _outmempool;
}

/****************************************************************************
 * Message Listener Class Methods
 ****************************************************************************/

void alusr_filter::filter_msglistener::bootup(
     audiolite_workercomp *wcomp, al_wtask_t *wtask, int version, void *d)
{
  al_comm_msgopt_t opt;
  // This method is called just after receiving boot-up
  // message from the worker

  if (version == FILTER_WORKER_VERSION)
    {
      opt.usr[0] = ((alusr_filter *)wcomp)->_mode;
      alworker_send_systemparam(wtask, wcomp->channels(),
                                       wcomp->samplingrate(),
                                       wcomp->samplebitwidth());
      alworker_send_start(wtask, &opt);
    }
  else
    {
      wcomp->publish_event(AL_EVENT_WRONGVERSION, version);
    }
}
