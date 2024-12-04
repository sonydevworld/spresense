#ifndef __INCLUDE_AUDIOLITE_USER_FFTWORKER_H
#define __INCLUDE_AUDIOLITE_USER_FFTWORKER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <audiolite/al_workercomp.h>
#include <audiolite/al_workercmd.h>
#include <fftworker/fftworker_worker_main.h>

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

class alusr_fftworker : public audiolite_workercomp
{
  protected:
    class fftworker_msglistener : public audiolite_stdworker_msglistener
    {
      public:
        virtual ~fftworker_msglistener(){};
        void bootup(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                    int version, void *d);
    };

    fftworker_msglistener _msglsnr;
    audiolite_mempoolapbuf *_outmempool;

  public:
    alusr_fftworker();
    virtual ~alusr_fftworker();
};

#endif /* __INCLUDE_AUDIOLITE_USER_FFTWORKER_H */
