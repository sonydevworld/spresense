#include <audiolite/audiolite.h>
#include <alusr_fftworker.h>

/****************************************************************************
 * alusr_fftworker Class Methods
 ****************************************************************************/

alusr_fftworker::alusr_fftworker() :
  audiolite_workercomp("fftworker", 8, 8), _outmempool(NULL)
{
  set_msglistener(&_msglsnr);
  _outmempool = new audiolite_mempoolapbuf;
  _outmempool->create_instance(FFT_TAPSHALF, 8);
  set_mempool(_outmempool);
}

alusr_fftworker::~alusr_fftworker()
{
  set_mempool(NULL);
  if (_outmempool)
    delete _outmempool;
}

/****************************************************************************
 * Message Listener Class Methods
 ****************************************************************************/

void alusr_fftworker::fftworker_msglistener::bootup(
     audiolite_workercomp *wcomp, al_wtask_t *wtask, int version, void *d)
{
  // This method is called just after receiving boot-up
  // message from the worker

  if (version == FFTWORKER_WORKER_VERSION)
    {
      alworker_send_systemparam(wtask, wcomp->channels(),
                                       wcomp->samplingrate(),
                                       wcomp->samplebitwidth());
      alworker_send_start(wtask);
    }
  else
    {
      wcomp->publish_event(AL_EVENT_WRONGVERSION, version);
    }
}
