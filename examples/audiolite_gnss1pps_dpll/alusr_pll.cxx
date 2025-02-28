/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/alusr_pll.cxx
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
#include <alusr_pll.h>

/****************************************************************************
 * alusr_pll Class Methods
 ****************************************************************************/

alusr_pll::alusr_pll() :
  audiolite_workercomp("pll", 8, 8), _outmempool(NULL)
{
  sem_init(&_sem_wcom, 0, 1);
  pthread_mutex_init(&_pllcmd_lock, NULL);
  pthread_cond_init(&_pllcmd_cond, NULL);

  set_msglistener(&_msglsnr);
  _outmempool = new audiolite_mempoolapbuf;
  _outmempool->create_instance(4096, 8);
  set_mempool(_outmempool);
}

alusr_pll::~alusr_pll()
{
  set_mempool(NULL);
  if (_outmempool)
    delete _outmempool;
}

void alusr_pll::start_pll()
{
  al_comm_msgopt_t opt = {0};

  opt.usr[0] = PLLCMD_START;
  sem_wait(&_sem_wcom);
  alworker_send_usrcmd(_worker.getwtask(), &opt);
  sem_post(&_sem_wcom);
}

void alusr_pll::stop_pll()
{
  al_comm_msgopt_t opt = {0};

  opt.usr[0] = PLLCMD_STOP;
  sem_wait(&_sem_wcom);
  alworker_send_usrcmd(_worker.getwtask(), &opt);
  sem_post(&_sem_wcom);
}

void alusr_pll::deliverr_reply(int code)
{
  pthread_mutex_lock(&_pllcmd_lock);
  if (_pllcmd_reply == -1)
    {
      _pllcmd_reply = code;
      pthread_cond_signal(&_pllcmd_cond);
    }
  pthread_mutex_unlock(&_pllcmd_lock);
}

bool alusr_pll::sendto_worker(al_comm_msgopt_t *opt)
{
  int ret;

  pthread_mutex_lock(&_pllcmd_lock);
  _pllcmd_reply = -1;

  alworker_send_usrcmd(_worker.getwtask(), opt);
  while (_pllcmd_reply == -1)
    {
      pthread_cond_wait(&_pllcmd_cond, &_pllcmd_lock);
    }

  ret = _pllcmd_reply;
  pthread_mutex_unlock(&_pllcmd_lock);

  return ret == PLLCMD_RETCODE_OK;
}

bool alusr_pll::send_xferdata(uint8_t *data, unsigned int len)
{
  int i;
  int slen;
  int ofst = 0;
  uint8_t *sdat;
  al_comm_msgopt_t opt = {0};

  if (len != XFER_DATA_BYTES)
    {
      return false;
    }

  sdat = (uint8_t *)&opt.usr[1];
  sem_wait(&_sem_wcom);

  while (len)
    {
      slen = len > sizeof(uint32_t) * 3 ? sizeof(uint32_t) * 3 : len;
      for (i = 0; i < slen; i++)
        {
          sdat[i] = data[i];
        }

      opt.usr[0] = PLLXFERDATACMD(ofst,slen);

      if (!sendto_worker(&opt))
        {
          return false;
        }

      ofst += slen;
      len -= slen;
      data += slen;
    }

  sem_post(&_sem_wcom);
  return true;
}

bool alusr_pll::enable_xfer()
{
  bool ret;
  al_comm_msgopt_t opt = {0};
  opt.usr[0] = PLLCMD_ENABLEXFER;
  sem_wait(&_sem_wcom);
  ret = sendto_worker(&opt);
  sem_post(&_sem_wcom);
  return ret;
}

bool alusr_pll::cancel_xfer()
{
  al_comm_msgopt_t opt = {0};
  opt.usr[0] = PLLCMD_CANCELXFER;
  sem_wait(&_sem_wcom);
  alworker_send_usrcmd(_worker.getwtask(), &opt);
  sem_post(&_sem_wcom);
  return true;
}

void alusr_pll::xfer_done()
{
  printf("XFER Done\n");
}

/****************************************************************************
 * Message Listener Class Methods
 ****************************************************************************/

void alusr_pll::pll_msglistener::bootup(
     audiolite_workercomp *wcomp, al_wtask_t *wtask, int version, void *d)
{
  al_comm_msgopt_t opt = { 0 };

  // This method is called just after receiving boot-up
  // message from the worker

  if (version == PLL_WORKER_VERSION)
    {
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

void alusr_pll::pll_msglistener::usermsg(
              audiolite_workercomp *wcomp, al_wtask_t *wtask,
              al_comm_msghdr_t hdr, al_comm_msgopt_t *opt)
{
  alusr_pll *pll = (alusr_pll *)wcomp;

  if (hdr.type == AL_COMM_MSGTYPE_RESP)
    {
      pll->deliverr_reply((int)hdr.code);
    }
  else if (opt->usr[0] == PLLCMD_DONE_XFER)
    {
      pll->xfer_done();
    }
}
