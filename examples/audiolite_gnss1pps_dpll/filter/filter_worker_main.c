/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/filter/filter_worker_main.c
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

#include <nuttx/config.h>
#include <asmp/stdio.h>
#include <alworker_commfw.h>
#include "filter_worker_main.h"
#include "../pll/pll_worker_main.h"
#include "iir_filter.h"

#include <arm_math.h>

#define CUTTOFF_FREQ  (400)
#define SQRT0_5 (0.707106781186547572737f)

#define OFFSET_IIR_FILTER (110)
#define DELAY_BUFSZ       (BLK_SAMPLES - OFFSET_IIR_FILTER)

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct my_worker_instance_s
{
  /* ALWORKERCOMMFW_INSTANCE should be on top of your instance */

  ALWORKERCOMMFW_INSTANCE;

  /* Add specific items for your application here */

  iir_filter_t iir;

  memblk_t inputf;
  memblk_t outputf;
  memblk_t remain;

  memblk_t *outbuf[2];

  float mem_inputf[BLK_SAMPLES];
  float mem_outputf[BLK_SAMPLES];

  int mode;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct my_worker_instance_s g_instance;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void signal_filter(struct my_worker_instance_s *inst)
{
  memblk_reset(&inst->outputf);
  execute_iir_filter(&inst->iir,
                     memblk_dataptrfloat(&inst->inputf),
                     memblk_fillptrfloat(&inst->outputf), BLK_SAMPLES);
  memblk_commit(&inst->outputf, BLK_SAMPLES * sizeof(float));
}

/* on_process():
 *  This is called when the state is in PROCESS.
 */

static int on_process(void *arg)
{
  memblk_t *imem;
  memblk_t onepps;
  memblk_t tmp;
  int half_size;
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  imem            = PEEK_IMEM(inst);
  inst->outbuf[1] = PEEK_OMEM(inst);

  if (imem && inst->outbuf[1])
    {
      TAKE_OMEM(inst);  // Drop the memory block from the queue

      half_size = imem->size / SAMPLE_CHS;
      memblk_init(&onepps, &inst->outbuf[1]->addr[0], half_size);

      /* Bit tricky but use buttom half memory of outbuf[1] as
       * temporary buffer for split L/R data.
       */

      memblk_init(&tmp, &inst->outbuf[1]->addr[half_size], half_size);

      /* Split data from input to output top half and temp buffer */

#ifdef CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_MICASSIGN_ONEPPS_MICB
      memblk_split_lr16(&tmp, &onepps, imem);
#else
      memblk_split_lr16(&onepps, &tmp, imem);
#endif

      /* Convert temp buffer to float for calculating filter */

      memblk_reset(&inst->inputf);
      memblk_conv_pcm16tofloat(&inst->inputf, &tmp);

      /* Then execute filter */

      signal_filter(inst);

      if (inst->outbuf[0])  /* Check existance of previouse output buffer */
        {
          /* Store remaining space in previouse output memory */

          memblk_conv_floattopcm16(&inst->remain, &inst->outputf, 1.f);
          memblk_commituint8(inst->outbuf[0], inst->outbuf[0]->size);
          FREE_MEMBLK(inst->outbuf[0], inst);
        }
      else
        {
          /* No previouse output buffer, drop first data for delay.
           * This statement is only at first time of this call.
           */

          memblk_dropfloat(&inst->outputf, OFFSET_IIR_FILTER);
        }

      /* Reuse remain for next outbuf, and save remain filtered data */

      if (inst->mode == MODE_NORMAL)
        {
          memblk_init(&inst->remain, &inst->outbuf[1]->addr[half_size],
                      half_size);
        }
      else if (inst->mode == MODE_FILTER_CHECK)
        {
          memblk_init(&inst->remain, &inst->outbuf[1]->addr[0],
                      half_size);
        }

      memblk_conv_floattopcm16(&inst->remain, &inst->outputf, 1.f);

      inst->outbuf[0] = inst->outbuf[1];  // Shift output buffer

      TAKE_IMEM(inst);
      FREE_MEMBLK(imem, inst);
    }

  return AL_COMMFW_RET_OK;
}

static int on_playmsg(int state, void *arg, al_comm_msgopt_t *opt)
{
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  inst->mode = opt->usr[0];

  return AL_COMMFW_RET_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  alcommfw_cbs_t *cbs = alworker_commfw_get_cbtable();

  if (alworker_commfw_initialize((alworker_insthead_t *)&g_instance) != OK)
    {
      return 0;
    }

  memblk_init(&g_instance.inputf, g_instance.mem_inputf,
              BLK_SAMPLES * sizeof(float));
  memblk_init(&g_instance.outputf, g_instance.mem_outputf,
              BLK_SAMPLES * sizeof(float));

  initialize_iir_filter(&g_instance.iir, FILTER_TYPE_LPF,
                        SAMPLE_FS, CUTTOFF_FREQ, SQRT0_5);

  g_instance.outbuf[0] = NULL;
  g_instance.outbuf[1] = NULL;

  /* Set callbacks to handle host message and
   * state processing
   */

  SET_PROCESS(cbs, on_process);
  SET_PLAYMSG(cbs, on_playmsg);

  /* Send boot message to host to notice this worker is ready */

  alworker_send_bootmsg(FILTER_WORKER_VERSION, NULL);

  /* Start message loop.
   * This function returns receive SYSTERM message from a host.
   */

  alworker_commfw_msgloop((alworker_insthead_t *)&g_instance);

  return 0;
}
