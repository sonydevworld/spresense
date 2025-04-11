/****************************************************************************
 * examples/fft_pwbimu/alusr_imufft.h
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

#ifndef __INCLUDE_AUDIOLITE_USER_IMUFFT_H
#define __INCLUDE_AUDIOLITE_USER_IMUFFT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <audiolite/al_workercomp.h>
#include <audiolite/al_workercmd.h>
#include <imufft/imufft_worker_main.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*fftresult_cb_t)(float *topdata, int sz, int taps, int chs);

/****************************************************************************
 * Class Definitions
 ****************************************************************************/

class alusr_imufft : public audiolite_workercomp
{
  protected:
    class imufft_msglistener : public audiolite_stdworker_msglistener
    {
      fftresult_cb_t _cb;
      public:
        virtual ~imufft_msglistener(){};
        void bootup(audiolite_workercomp *wcomp, al_wtask_t *wtask,
                    int version, void *d);
        virtual audiolite_memapbuf *release_inmem(audiolite_workercomp *wcomp,
                                                  al_wtask_t *wtask,
                                                  audiolite_memapbuf *mem,
                                                  int size);

      friend alusr_imufft;
    };

    int _taps;
    int _chs;
    imufft_msglistener _msglsnr;
    audiolite_mempoolapbuf *_outmempool;

  public:
    alusr_imufft(int taps = 512, int chs = 6);
    virtual ~alusr_imufft();

    void set_resultcb(fftresult_cb_t cb) { _msglsnr._cb = cb; };

    int taps() { return _taps; };
    int chs()  { return _chs;  };
};

#endif /* __INCLUDE_AUDIOLITE_USER_IMUFFT_H */
