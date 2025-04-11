/****************************************************************************
 * examples/fft_pwbimu/alusr_imufft.cxx
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
#include <alusr_imufft.h>

/****************************************************************************
 * alusr_imufft Class Methods
 ****************************************************************************/

alusr_imufft::alusr_imufft(int taps, int chs) :
  audiolite_workercomp("imufft", 8, 0), _outmempool(NULL)
{
  int t;

  if (chs <= 0)
    {
      chs = 1;
    }
  else if (chs > 3)
    {
      chs = 3;
    }

  _chs = chs;

  _taps = 0;
  for (t = FFT_MINTAPS; t <= FFT_MAXTAPS; t <<= 1)
    {
      if (t == taps)
        {
          _taps = taps;
          break;
        }
    }

  if (_taps == 0)
    {
      _taps = FFT_MINTAPS;
    }

  set_msglistener(&_msglsnr);
}

alusr_imufft::~alusr_imufft()
{
}

/****************************************************************************
 * Message Listener Class Methods
 ****************************************************************************/

void alusr_imufft::imufft_msglistener::bootup(
     audiolite_workercomp *wcomp, al_wtask_t *wtask, int version, void *d)
{
  al_comm_msgopt_t opt;

  if (version == IMUFFT_WORKER_VERSION)
    {
      opt.usr[0] = ((alusr_imufft *)wcomp)->taps();
      opt.usr[1] = ((alusr_imufft *)wcomp)->chs();

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

audiolite_memapbuf
*alusr_imufft::imufft_msglistener::release_inmem(audiolite_workercomp *wcomp,
                                                 al_wtask_t *wtask,
                                                 audiolite_memapbuf *mem,
                                                 int size)
{
  /* This function calls when the input memory is released back.
   * And in the memory, FFT results are stored.
   */

  _cb((float *)mem->get_data(), size / 2, ((alusr_imufft *)wcomp)->taps(),
                                          ((alusr_imufft *)wcomp)->chs());
  return NULL;
}
