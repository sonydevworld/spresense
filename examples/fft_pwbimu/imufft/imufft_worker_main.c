/****************************************************************************
 * examples/fft_pwbimu/imufft/imufft_worker_main.c
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

#include <asmp/stdio.h>
#include <alworker_commfw.h>
#include "imufft_worker_main.h"

#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

typedef arm_rfft_fast_instance_f32 fftinst_t;

struct my_worker_instance_s
{
  /* ALWORKERCOMMFW_INSTANCE should be on top of your instance */

  ALWORKERCOMMFW_INSTANCE;

  /* Add specific items for your application here */

  uint32_t fft_taps;
  uint32_t fft_chs;

  memblk_t  input[FFT_CHANNELS];

  fftinst_t rfft;

  float fft_input[FFT_CHANNELS][FFT_MAXTAPS];
  float fft_workmem[FFT_MAXTAPS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct my_worker_instance_s g_instance;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void split_memory(struct my_worker_instance_s *inst,
                         memblk_t *imem, memblk_t *owork)
{
  int i, j;
  float dat;

  /* Re-use input mem for output */

  float *memfloat = (float *)imem->addr;

  for (i = 0; i < inst->fft_chs; i++)
    {
      memblk_reset(&inst->input[i]);

      /* Slice input memory for each FFT results */

      memblk_init(&owork[i], (char *)memfloat, 
                  inst->fft_taps * sizeof(float) / 2);
      memfloat += inst->fft_taps / 2;
    }

  /* Split and copy input data from embedded to planner */

  for (i = 0; i < inst->fft_taps; i++)
    {
      for (j = 0; j < inst->fft_chs; j++)
        {
          dat = memblk_pop_float(imem);
          memblk_push_float(&inst->input[j], dat);
        }
    }
}

static void exec_fft(fftinst_t *inst, int taps, memblk_t *in,
                     memblk_t *out, float *work)
{
  float *indata  = memblk_dataptrfloat(in);
  float *outdata = memblk_fillptrfloat(out);

  /* No window function to aviod collapse information */

  arm_rfft_fast_f32(inst, indata, work, 0);
  arm_cmplx_mag_f32(&work[2], &outdata[1], (taps / 2) - 1);
  outdata[0] = work[0] < 0 ? -work[0] : work[0];

  memblk_commitfloat(out, taps / 2);
}

static void fft_work(struct my_worker_instance_s *inst, memblk_t *imem)
{
  int i;
  memblk_t out[FFT_CHANNELS];

  split_memory(inst, imem, out);

  memblk_reset(imem);

  for (i = 0; i < inst->fft_chs; i++)
    {
      exec_fft(&inst->rfft, inst->fft_taps, &inst->input[i],
               &out[i], inst->fft_workmem);
    }

  memblk_commitfloat(imem, inst->fft_taps * inst->fft_chs / 2);
}

/* on_process():
 *  This is called when the state is in PROCESS.
 *
 *   Return code controls state change.
 *     AL_COMMFW_RET_OK     : Stay the state to process next work.
 *     AL_COMMFW_RET_NOIMEM : Change to WAIT_IMEM for waiting next input.
 *     AL_COMMFW_RET_NOOMEM : Change to WAIT_OMEM for waiting next output.
 */

static int on_process(void *arg)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  memblk_t *imem = PEEK_IMEM(inst);

  if (inst->fft_taps == 0)
    {
      /* No TAPs means failed initialze */

      if (imem)
        {
          TAKE_IMEM(inst);
          FREE_MEMBLK(imem, inst);
        }
    }
  else if (imem)
    {
      if (memblk_remainfloat(imem) == (inst->fft_taps * inst->fft_chs))
        {
          fft_work(inst, imem);
        }

      /* Release back memory */

      TAKE_IMEM(inst);
      FREE_MEMBLK(imem, inst);
    }

  return AL_COMMFW_RET_OK;
}

/* on_playmsg():
 *  This is called when AL_COMM_MSGCODESYS_PLAY is received from a host.
 *
 *  Return AL_COMMFW_RET_OK, to go to STARTING state,
 *  Non AL_COMMFW_RET_OK makes return a message with a returned code
 *  to the host.
 */

static int on_playmsg(int state, void *arg, al_comm_msgopt_t *opt)
{
  uint32_t tap;

  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  inst->fft_taps  = 0;
  inst->fft_chs   = 0;

  for (tap = FFT_MINTAPS; tap <= FFT_MAXTAPS; tap <<= 1)
    {
      if (tap == opt->usr[0])
        {
          inst->fft_taps = tap;
          break;
        }
    }

  if (inst->fft_taps)
    {
      if (opt->usr[1] > 0 && opt->usr[1] <= FFT_CHANNELS)
        {
          inst->fft_chs = opt->usr[1];
          arm_rfft_fast_init_f32(&inst->rfft, inst->fft_taps);
        }
      else
        {
          inst->fft_taps = 0;
        }
    }

  return AL_COMMFW_RET_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  int i;
  alcommfw_cbs_t *cbs = alworker_commfw_get_cbtable();

  if (alworker_commfw_initialize((alworker_insthead_t *)&g_instance) != OK)
    {
      return 0;
    }

  for (i = 0; i < FFT_CHANNELS; i++)
    {
      memblk_init(&g_instance.input[i],
                  (char *)g_instance.fft_input[i],
                  FFT_MAXTAPS * sizeof(float));
    }

  /* Set callbacks to handle host message and
   * state processing
   */

  SET_PROCESS(cbs, on_process);
  SET_PLAYMSG(cbs, on_playmsg);

  /* Send boot message to host to notice this worker is ready */

  alworker_send_bootmsg(IMUFFT_WORKER_VERSION, NULL);

  /* Start message loop.
   * This function returns receive SYSTERM message from a host.
   */

  alworker_commfw_msgloop((alworker_insthead_t *)&g_instance);

  return 0;
}
