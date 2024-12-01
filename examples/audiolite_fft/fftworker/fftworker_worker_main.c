/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <asmp/stdio.h>
#include <alworker_commfw.h>
#include "fftworker_worker_main.h"

#include "arm_math.h"

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct my_worker_instance_s
{
  /* ALWORKERCOMMFW_INSTANCE should be on top of your instance */

  ALWORKERCOMMFW_INSTANCE;

  /* Add specific items for your application here */

  memblk_t *pcm_in;

  memblk_t pcm_cache;
  memblk_t pcm_float;

  arm_rfft_fast_instance_f32 rfft;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct my_worker_instance_s g_instance;

static short pcm_cache_mem[FFT_TAPSHALF];
static float pcm_float_mem[FFT_TAPS];

static float fft_input[FFT_TAPS];
static float fft_window[FFT_TAPS];
static float fft_tmp[FFT_TAPS];
static float fft_power[FFT_TAPSHALF];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void init_hanningw(float *dat, int len)
{
  int i;
  float weight;

  for (i = 0; i < len / 2; i++)
    {
      weight = 0.5f *
               (1.0f - arm_cos_f32(2 * PI * (float)i / len));
      dat[i] = weight;
      dat[len - (i + 1)] = weight;
    }
}

static void calc_fft(struct my_worker_instance_s *inst, memblk_t *out)
{
  uint32_t i;
  float max;
  float *pcm = (float *)memblk_dataptr(&inst->pcm_float);

  /* Calculate FFT with log */

  arm_mult_f32(pcm, fft_window, fft_input, FFT_TAPS);
  arm_rfft_fast_f32(&inst->rfft, fft_input, fft_tmp, 0);
  arm_cmplx_mag_f32(fft_tmp, fft_power, FFT_TAPSHALF);

  /* Normalize */

  arm_max_f32(fft_power, FFT_TAPSHALF, &max, &i);
  arm_scale_f32(fft_power, 255.f/max, fft_power, FFT_TAPSHALF);

  /* Store result */

  memblk_reset(out);
  for (i = 0; i < FFT_TAPSHALF && !memblk_is_full(out); i++)
    {
      memblk_push_uint8(out, (uint8_t)fft_power[i]);
    }
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
  memblk_t *fft_out;
  struct my_worker_instance_s *inst =
    (struct my_worker_instance_s *)arg;

  while(!memblk_is_full(&inst->pcm_cache))
    {
      if (inst->pcm_in == NULL)
        {
          inst->pcm_in = PEEK_IMEM(inst);
          if (inst->pcm_in == NULL)
            {
              return AL_COMMFW_RET_OK;  /* Wait for PCM data */
            }
        }

      memblk_single16(&inst->pcm_cache, inst->pcm_in);
      if (memblk_is_empty(inst->pcm_in))
        {
          inst->pcm_in = TAKE_IMEM(inst);
          FREE_MEMBLK(inst->pcm_in, inst);
          inst->pcm_in = NULL;
        }
    }

  fft_out = TAKE_OMEM(inst);
  if (fft_out == NULL)
    {
      return AL_COMMFW_RET_OK;  /* Wait for memory to output */
    }

  memblk_shift_drop(&inst->pcm_float, sizeof(float) * FFT_TAPSHALF);
  memblk_conv_pcm16tofloat(&inst->pcm_float, &inst->pcm_cache);
  calc_fft(inst, fft_out);

  FREE_MEMBLK(fft_out, inst);
  memblk_reset(&inst->pcm_cache);

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

  g_instance.pcm_in = NULL;

  memblk_init(&g_instance.pcm_cache, (char *)pcm_cache_mem, sizeof(pcm_cache_mem));
  memblk_init(&g_instance.pcm_float, (char *)pcm_float_mem, sizeof(pcm_float_mem));
  arm_rfft_fast_init_f32(&g_instance.rfft, FFT_TAPS);
  init_hanningw(fft_window, FFT_TAPS);

  /* Set callbacks to handle host message and
   * state processing
   */

  SET_PROCESS(cbs, on_process);

  /* Send boot message to host to notice this worker is ready */

  alworker_send_bootmsg(FFTWORKER_WORKER_VERSION, NULL);

  /* Start message loop.
   * This function returns receive SYSTERM message from a host.
   */

  alworker_commfw_msgloop((alworker_insthead_t *)&g_instance);

  return 0;
}
