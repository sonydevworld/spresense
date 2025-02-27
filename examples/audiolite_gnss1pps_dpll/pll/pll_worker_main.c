/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/pll/pll_worker_main.c
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
#include <asmp/gpio.h>
#include <alworker_commfw.h>
#include "pll_worker_main.h"

#include <arm_math.h>

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#define PLLSTATE_READY                (0)
#define PLLSTATE_1STEDGEDETECT        (1)
#define PLLSTATE_2NDEDGEDETECT        (2)
#define PLLSTATE_INTERVAL             (3)
#define PLLSTATE_PHASESHIFT           (4)
#define PLLSTATE_PHASEUPDATED         (5)
#define PLLSTATE_ADJUSTDELAY          (6)
#define PLLSTATE_ADJUSTPHASE          (7)
#define PLLSTATE_ADJUSTOMEGA          (8)
#define PLLSTATE_PHASELOCKED          (9)
#define PLLSTATE_NUMKINDS             (10)

#define M_2PI48         ((int64_t)0x0000800000000000)
#define M_2PI48_MASK    ((int64_t)0x00007fffffffffff)
#define WRAP_PI48(p)    (((p) >= 0) ? (p) & M_2PI48_MASK : \
                                     -((-p) & M_2PI48_MASK))
#define M_2PI16_MASK    ((int16_t)0x7fff)
#define PHASE48to16(p)  ((p) >= 0) ? \
                        (((int16_t)((p) / 0x100000000)) & M_2PI16_MASK) : \
                        (-(((int16_t)((-p) / 0x100000000)) & M_2PI16_MASK))

#define ONEPPS_RISEEDGE_THRESH  (25000)

#ifdef SINWAVE_IS_NEGATIVE
#  define REFSIG_TRIGGER_LEVEL  (-10000)
#  define CHECK_TRIGGER_LEVEL(l)  ((l) < REFSIG_TRIGGER_LEVEL)
#  define ZEROCROSS(l, c)  ((l) < 0 && (c) >= 0)
#else
#  define REFSIG_TRIGGER_LEVEL  (10000)
#  define CHECK_TRIGGER_LEVEL(l)  ((l) > REFSIG_TRIGGER_LEVEL)
#  define ZEROCROSS(l, c)  ((l) >= 0 && (c) < 0)
#endif

#define ATTENATE_REFSIG(s)   ((s) * 1 / 4)

#define EDGEDETECT_INTERVAL_SEC   (2)
#define PHASEUPDATE_INTERVAL_SEC  (2)

#define OMEGA_ADJUST_FROM   (1)
#define OMEGA_ADJUST_STABLE (OMEGA_ADJUST_FROM + 5)
#define OMEGA_ADJUST_RANGE  (50)
#define OMEGA_ADJUST_DELTAP(w)  ((w)->omega + \
                                 ((M_2PI48 / SAMPLE_FS / 100) * (w)->freq))
#define OMEGA_ADJUST_DELTAM(w)  ((w)->omega - \
                                 ((M_2PI48 / SAMPLE_FS / 100) * (w)->freq))

#define ADJUSTPHASE_INTERVAL (2)
#define OMEGA_ADJUST_AVARAGE_SIZE (20)

#define BLK_SAMPLES_BOUNDARY (10)

#ifdef CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_PLL_DEBUG
#  define dprintf printf
#else
#  define dprintf(...)
#endif

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct wave_param_s
{
  int64_t delta;
  int64_t theta;
  int64_t omega;
  int freq;         /* (Hz) */
  int wavcnt;       /* wave number counter: 0 to freq -1 */
  int edge_wavcnt;  /* wave number count on the 1-PPS edge */
};
typedef struct wave_param_s wave_param_t;

struct adjust_omega_s
{
  int pps_count;  /* One PPS Edge counter */
  int period_diff_sum;   /* Sum of period from 1PPS edge to Zero-cross point */
  int last_diff;  /* Last period */
};
typedef struct adjust_omega_s adjust_omega_t;

struct my_worker_instance_s
{
  /* ALWORKERCOMMFW_INSTANCE should be on top of your instance */

  ALWORKERCOMMFW_INSTANCE;

  int pll_state;        /* State of PLL phase */

  memblk_t onepps;      /* 1-PPS signal buffer from audio mic input */
  memblk_t refsig;      /* Reference signal loopbacked from audio mic */
  memblk_t refsig_last; /* To store one sample to detect across blocks */
  memblk_t refval_last; /* To store last sample of refsig on the 1-PPS edge */

  int onepps_period;    /* Counter of samples edge-to-edge */
  int no_rise_detect;   /* Paralysis period after detecting an edge */

  int one_hz_period;    /* Samples in 1 secound. Theoretical value is Fs */
  int zerox_period;     /* Sample counter after an 1-PPS edge */
  bool zerox_detection; /* Indicates zerocross detection area */
  int interval_cnt;

  wave_param_t refwave;
  wave_param_t carwave;

  int16_t last_refsig;  /* Actual memory for refsig_last */
  int16_t last_refval;  /* Actual memory for refval_last */

  adjust_omega_t omega_adjust;

  uint8_t *sending_data;
  uint8_t sdata_mem[DATA_BITS / 8];
  int current_bit;

  int mode;
#ifdef DEBUG_DISPLAY_ONEPPSPERIOD
  int debug_period;
#endif
};
typedef struct my_worker_instance_s worker_inst_t;

typedef int (*state_process)(worker_inst_t *inst, int epos, int zerox);

struct lr_signal_s
{
  int16_t l;
  int16_t r;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void update_freq(worker_inst_t *inst);
static int state_ready(worker_inst_t *inst, int epos, int zerox);
static int state_1stedgedetect(worker_inst_t *inst, int epos, int zerox);
static int state_2ndedgedetect(worker_inst_t *inst, int epos, int zerox);
static int state_interval(worker_inst_t *inst, int epos, int zerox);
static int state_phaseshift(worker_inst_t *inst, int epos, int zerox);
static int state_phaseupdated(worker_inst_t *inst, int epos, int zerox);
static int state_adjustdelay(worker_inst_t *inst, int epos, int zerox);
static int state_adjustphase(worker_inst_t *inst, int epos, int zerox);
static int state_adjustomega(worker_inst_t *inst, int epos, int zerox);
static int state_phaselocked(worker_inst_t *inst, int epos, int zerox);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static worker_inst_t g_instance;
static state_process state_func[PLLSTATE_NUMKINDS] =
{
  [PLLSTATE_READY]          = state_ready,
  [PLLSTATE_1STEDGEDETECT]  = state_1stedgedetect,
  [PLLSTATE_2NDEDGEDETECT]  = state_2ndedgedetect,
  [PLLSTATE_INTERVAL]       = state_interval,
  [PLLSTATE_PHASESHIFT]     = state_phaseshift,
  [PLLSTATE_PHASEUPDATED]   = state_phaseupdated,
  [PLLSTATE_ADJUSTDELAY]    = state_adjustdelay,
  [PLLSTATE_ADJUSTPHASE]    = state_adjustphase,
  [PLLSTATE_ADJUSTOMEGA]    = state_adjustomega,
  [PLLSTATE_PHASELOCKED]    = state_phaselocked,
};

static uint32_t state_ledptn[PLLSTATE_NUMKINDS] =
{
  [PLLSTATE_READY]          = 0,
  [PLLSTATE_1STEDGEDETECT]  = 1,
  [PLLSTATE_2NDEDGEDETECT]  = 1,
  [PLLSTATE_INTERVAL]       = 2,
  [PLLSTATE_PHASESHIFT]     = 2,
  [PLLSTATE_PHASEUPDATED]   = 2,
  [PLLSTATE_ADJUSTDELAY]    = 3,
  [PLLSTATE_ADJUSTPHASE]    = 4,
  [PLLSTATE_ADJUSTOMEGA]    = 5,
  [PLLSTATE_PHASELOCKED]    = 7,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_PLL_DEBUG
#  define STATE_NAME(s) case PLLSTATE_##s: return #s
static const char *state_name(int state)
{
  switch (state)
    {
      STATE_NAME(READY);
      STATE_NAME(1STEDGEDETECT);
      STATE_NAME(2NDEDGEDETECT);
      STATE_NAME(INTERVAL);
      STATE_NAME(PHASESHIFT);
      STATE_NAME(PHASEUPDATED);
      STATE_NAME(ADJUSTDELAY);
      STATE_NAME(ADJUSTPHASE);
      STATE_NAME(ADJUSTOMEGA);
      STATE_NAME(PHASELOCKED);
    }

  return "Unknown";
}

/**
 * display_statechange()
 * Debug print when internal state is changed.
 */

static void display_statechange(int cur, int next)
{
  if (cur != next)
    {
      dprintf("S-CHG %s -> %s\n", state_name(cur), state_name(next));
    }
}
#else
#  define display_statechange(...)
#endif /* CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_PLL_DEBUG */

/**
 * initialize_instance()
 * Reset the instance to back to begining
 */

static void initialize_instance(worker_inst_t *inst)
{
  inst->onepps_period  = SAMPLE_FS;
  inst->no_rise_detect = 200; /* 200 blocks of input memory is avoided */

  inst->zerox_detection = false;
  inst->zerox_period = 0;

  inst->refwave.freq = DEFAULT_REFFREQ;
  inst->carwave.freq = DEFAULT_CARRIERFREQ;
  update_freq(inst);
  inst->refwave.theta = M_2PI48 / 2;
  inst->refwave.delta = 0;
  inst->refwave.wavcnt = 0;
  inst->refwave.edge_wavcnt = 0;
  inst->carwave.theta = 0;
  inst->carwave.delta = 0;
  inst->carwave.wavcnt = 0;
  inst->carwave.edge_wavcnt = 0;

  inst->sending_data = NULL;
  inst->current_bit  = 0;
}

/**
 * state_led()
 * Display internal status on LEDs.
 */

static void state_led(uint32_t led)
{
  board_gpio_write(PIN_LED0, (led & 0x01) ? 1 : 0);
  board_gpio_write(PIN_LED1, (led & 0x02) ? 1 : 0);
  board_gpio_write(PIN_LED2, (led & 0x04) ? 1 : 0);
}

static void init_adjustomega(adjust_omega_t *ao)
{
  ao->pps_count = 0;
  ao->period_diff_sum = 0;
  ao->last_diff = 0;
}

static void update_adjustomega(adjust_omega_t *ao, int epos, int zerox)
{
  int diff = zerox - epos;

  if (ao->pps_count != 0)
    {
      if (diff != ao->last_diff)
        {
          ao->period_diff_sum += diff - ao->last_diff;
        }
    }

  ao->last_diff = diff;
}

static bool adjust_omega(adjust_omega_t *ao,
                         wave_param_t *ref, wave_param_t *car, int one_hz)
{
  bool ret = false;

#ifdef CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_PLL_DEBUG
  uint32_t *dd = (uint32_t *)&ref->omega;
#endif

  if (ao->pps_count == OMEGA_ADJUST_AVARAGE_SIZE)
    {
      dprintf("Update omega..from %08x%08x\n",
              (unsigned int)dd[1], (unsigned int)dd[0]);
      ref->omega =
        M_2PI48 * OMEGA_ADJUST_AVARAGE_SIZE /
        ((int64_t)one_hz * OMEGA_ADJUST_AVARAGE_SIZE -
        (int64_t)ao->period_diff_sum) * ref->freq;
      car->omega =
        M_2PI48 * OMEGA_ADJUST_AVARAGE_SIZE /
        ((int64_t)one_hz * OMEGA_ADJUST_AVARAGE_SIZE -
        (int64_t)ao->period_diff_sum) * car->freq;
      dprintf("                to %08x%08x\n",
              (unsigned int)dd[1], (unsigned int)dd[0]);
      ret = true;
    }

  ao->pps_count++;

  return ret;
}

/**
 * update_freq()
 * Update the sample number of one second.
 * This value is the base value for the output sampling frequency. 
 */

static void update_freq(worker_inst_t *inst)
{
  dprintf("Change Edge Period from %d to %d\n",
          inst->one_hz_period, inst->onepps_period);

  inst->one_hz_period = inst->onepps_period;
  inst->refwave.omega = M_2PI48 / (int64_t)inst->one_hz_period
                                * (int64_t)inst->refwave.freq;
  inst->carwave.omega = M_2PI48 / (int64_t)inst->one_hz_period
                                * (int64_t)inst->carwave.freq;
}

/**
 * update_wave_count()
 */

static void update_wave_count(wave_param_t *wave, int count)
{
  wave->delta = wave->delta - ((int64_t)count * wave->omega);

  wave->wavcnt = (wave->wavcnt >= wave->edge_wavcnt) ?
                    wave->wavcnt - wave->edge_wavcnt :
                    wave->wavcnt + wave->freq - wave->edge_wavcnt;
}

/**
 * update_phase()
 * Update the sample time counter to fit the edge of 1PPS.
 */

static void update_phase(worker_inst_t *inst)
{
  update_wave_count(&inst->refwave, inst->zerox_period);
  update_wave_count(&inst->carwave, inst->zerox_period);
}

/**
 * adjust_output_delay()
 * Adjust wave counter for output delay from SPK output to MIC input
 */

static void adjust_output_delay(wave_param_t *wave)
{
  int diff = wave->wavcnt >= wave->edge_wavcnt ?
             wave->wavcnt - wave->edge_wavcnt :
             wave->wavcnt - (wave->edge_wavcnt - wave->freq);
  wave->wavcnt += diff;
}

static void adjust_phase_1st(wave_param_t *wave, int cnt)
{
  wave->delta = wave->delta - ((int64_t)cnt * wave->omega);
}

/**
 * adjust_phase()
 * Adjust the phase to shift delta based on the value of reference.
 */

static void adjust_phase(wave_param_t *ref, wave_param_t *car, int delta)
{
  ref->theta = ref->theta + (ref->omega * delta);
  car->theta = car->theta + (car->omega * delta);
}

/**
 * onepps_riseedge_detect()
 * Detect 1 PPS signal rising edge
 */

static int onepps_riseedge_detect(worker_inst_t *inst,
                                  memblk_t *input)
{
  int16_t *adat = memblk_dataptrint16(input);
  int adat_samples = memblk_remainint16(input);
  int pos;

  for (pos = 0; pos < adat_samples; pos++)
    {
      if (adat[pos] >= ONEPPS_RISEEDGE_THRESH)
        {
          if (inst->no_rise_detect == 0)
            {
              inst->no_rise_detect = 8;
              goto out_func;
            }
        }
    }

  /* No edge detected... */

  pos = -1;

out_func:

  if (inst->no_rise_detect > 0) inst->no_rise_detect--;

  return pos;
}

/**
 * detect_zerox()
 * Detect actual zero-cross data
 */

static bool detect_zerox(int16_t last, int16_t cur, bool *detect_area)
{
  if (*detect_area)
    {
      if (ZEROCROSS(last, cur))
        {
          *detect_area = false;
          return true;
        }
    }
  else if (CHECK_TRIGGER_LEVEL(last))
    {
      *detect_area = true;
    }

  return false;
}

/**
 * zerocross_detect()
 * Detect zero-cross data by using last sample and current sample
 */

static int zerocross_detect(worker_inst_t *inst,
                            memblk_t *refsig, memblk_t *last)
{
  int16_t last_dat;
  int16_t *adat = memblk_dataptrint16(refsig);
  int adat_sample = memblk_remainint16(refsig);
  int pos = 0;

  if (memblk_remain(last))
    {
      last_dat = memblk_pop_int16(last);
      if (detect_zerox(last_dat, adat[0], &inst->zerox_detection))
        {
          goto out_func;
        }
    }

  for (pos = 1; pos < adat_sample; pos++)
    {
      if (detect_zerox(adat[pos - 1], adat[pos], &inst->zerox_detection))
        {
          goto out_func;
        }
    }

  pos = -1;

  memblk_reset(last);
  memblk_push_int16(last, adat[adat_sample - 1]);

out_func:

  return pos;
}

/****************************************************************************
 * Private Functions for each state processing
 ****************************************************************************/

/** state_ready() : Process in the state of READY
 * Just waiting for instructions to start
 */

static int state_ready(worker_inst_t *inst, int epos, int zerox)
{
  /* Do nothing and stay this state
   * State change is happened on start pll command from main CPU.
   */

  return PLLSTATE_READY;
}

/** state_1stedgedetect() : Process in the sate of 1STEDGEDETECT.
 * Search first rising edge on 1-PPS signal
 */

static int state_1stedgedetect(worker_inst_t *inst, int epos, int zerox)
{
  int next_state = PLLSTATE_1STEDGEDETECT;
  if (epos >= 0)
    {
      dprintf("Detect Edge%c\n", ' ');
      inst->onepps_period =
        memblk_remainint16(&inst->onepps) - epos;
      next_state = PLLSTATE_2NDEDGEDETECT;
    }

  return next_state;
}

/** state_2ndedgedetect() : Process in the state of 2NDEDGEDETECT.
 * Search 2nd rising edge for measureing the period of 1-PPS.
 */

static int state_2ndedgedetect(worker_inst_t *inst, int epos, int zerox)
{
  int next_state = PLLSTATE_2NDEDGEDETECT;
  if (epos >= 0)
    {
      inst->onepps_period += epos;
      update_freq(inst);
      inst->interval_cnt = 0;
      next_state = PLLSTATE_INTERVAL;
    }
  else
    {
      inst->onepps_period += memblk_remainint16(&inst->onepps);
    }

  return next_state;
}

/** state_interval() : Process in the state of INTERVAL.
 * Wait for 1-PPS edge for stabilizing reference signal
 */ 

static int state_interval(worker_inst_t *inst, int epos, int zerox)
{
  int next_state = PLLSTATE_INTERVAL;

  if (epos >= 0)
    {
      inst->interval_cnt++;

      if (inst->interval_cnt >= EDGEDETECT_INTERVAL_SEC)
        {
          /* Check zerocross point for phase lock here
           * because the case should be cared that the zerocross
           * point on reference wave signal is close to the edge.
           */

          inst->refwave.edge_wavcnt = inst->refwave.wavcnt;
          inst->carwave.edge_wavcnt = inst->carwave.wavcnt;

          zerox -= epos;
          if (zerox >= 0)
            {
              inst->zerox_period = zerox;
              update_phase(inst);
              inst->interval_cnt = 0;
              next_state = PLLSTATE_PHASEUPDATED;
            }
          else
            {
              inst->zerox_period =
                  memblk_remainint16(&inst->refsig) - epos;
              inst->interval_cnt = 0;
              next_state = PLLSTATE_PHASESHIFT;
            }
        }
    }

  return next_state;
}

/** state_phaseshift() : Process in the state of PHASESHIFT.
 * Detecting Zero cross point on reference signal
 */

static int state_phaseshift(worker_inst_t *inst, int epos, int zerox)
{
  int next_state = PLLSTATE_PHASESHIFT;

  if (zerox >= 0)
    {
      inst->zerox_period += zerox;
      update_phase(inst);
      inst->interval_cnt = 0;
      next_state = PLLSTATE_PHASEUPDATED;
    }
  else
    {
      inst->zerox_period += memblk_remainint16(&inst->refsig);
    }

  return next_state;
}

/** state_phaseupdated() : Process in the state of PHASEUPDATED.
 * This state is just wait 1-PPS edge signal for stabilizing refrerence
 * after updateed of the phase.
 */

static int state_phaseupdated(worker_inst_t *inst, int epos, int zerox)
{
  int next_state = PLLSTATE_PHASEUPDATED;

  if (epos >= 0)
    {
      inst->interval_cnt++;

      if (inst->interval_cnt >= PHASEUPDATE_INTERVAL_SEC)
        {
          inst->refwave.edge_wavcnt = inst->refwave.wavcnt;
          inst->carwave.edge_wavcnt = inst->carwave.wavcnt;

          if (zerox >= 0)
            {
              zerox -= epos;
              inst->zerox_period = zerox;
              adjust_output_delay(&inst->refwave);
              adjust_output_delay(&inst->carwave);
              memblk_reset(&inst->refval_last);
              inst->interval_cnt = 0;
              next_state = PLLSTATE_ADJUSTPHASE;
            }
          else
            {
              inst->zerox_period = memblk_remainint16(&inst->refsig) - epos;
              next_state = PLLSTATE_ADJUSTDELAY;
            }

          inst->interval_cnt = 0;
        }
    }

  return next_state;
}

static int state_adjustdelay(worker_inst_t *inst, int epos, int zerox)
{
  int next_state = PLLSTATE_ADJUSTDELAY;

  if (zerox >= 0)
    {
      inst->zerox_period += zerox;
      adjust_output_delay(&inst->refwave);
      adjust_output_delay(&inst->carwave);
      memblk_reset(&inst->refval_last);
      inst->interval_cnt = 0;
      next_state = PLLSTATE_ADJUSTPHASE;
    }
  else
    {
      inst->zerox_period += memblk_remainint16(&inst->refsig);
    }

  return next_state;
}

static int state_adjustphase(worker_inst_t *inst, int epos, int zerox)
{
  int next_state = PLLSTATE_ADJUSTPHASE;
  int diff;

  if (epos >= 0)
    {
      inst->interval_cnt++;
      if (inst->interval_cnt >= ADJUSTPHASE_INTERVAL)
        {
          if (zerox >= 0)
            {
              diff = zerox - epos;
              dprintf("Adjusting phase zerox[%d] - epos[%d] = %d\n",
                     zerox, epos, diff);
              if (diff != 0)
                {
                  adjust_phase_1st(&inst->refwave, diff);
                  adjust_phase_1st(&inst->carwave, diff);
                }

              init_adjustomega(&inst->omega_adjust);
              next_state = PLLSTATE_ADJUSTOMEGA;
            }
          else
            {
              dprintf("Not in same block..%c\n", ' ');
              inst->interval_cnt = 0;
              next_state = PLLSTATE_PHASEUPDATED;
            }
        }
    }

  return next_state;
}

static int state_adjustomega(worker_inst_t *inst, int epos, int zerox)
{
  int next_state = PLLSTATE_ADJUSTOMEGA;

  if (epos >= 0)
    {
      if (zerox >= 0)
        {
          update_adjustomega(&inst->omega_adjust, epos, zerox);

          dprintf("ADJUSTOMG zerox[%d] - epos[%d] = %d : cnt%d sum[%d]\n",
                  zerox, epos, zerox - epos,
                  inst->omega_adjust.pps_count,
                  inst->omega_adjust.period_diff_sum);

          if (adjust_omega(&inst->omega_adjust,
                           &inst->refwave, &inst->carwave,
                           inst->one_hz_period))
            {
              next_state = PLLSTATE_PHASELOCKED;
            }
        }
      else
        {
          dprintf("ADJUSTOMG Not same block... epos[%d]\n", epos);
          if (epos < (BLK_SAMPLES - 10) && epos > 10)
            {
              inst->interval_cnt = 0;
              next_state = PLLSTATE_PHASEUPDATED;
            }
        }
    }

  return next_state;
}

/** state_phaselocked() : Process in the state of PHASELOCKED.
 * On this state, adjust phase from the refsig value on the edge of 1-PPS
 * signal on every edge.
 */

static int state_phaselocked(worker_inst_t *inst, int epos, int zerox)
{
  int next_state = PLLSTATE_PHASELOCKED;
  int delta;
  int16_t data;

  if (epos >= 0)
    {
      if (zerox >= 0)
        {
          delta = zerox - epos;
          memblk_dropint16(&inst->refsig, epos);
          data = memblk_pop_int16(&inst->refsig);
          dprintf("PHASELOCKED zerox[%d] - epos[%d] = %d dat:%d\n",
                  zerox, epos, zerox - epos, data);
          if (delta != 0)
            {
              /* Case of the difference between eops and zerox
               * is more than eq 1 sample
               */

              adjust_phase(&inst->refwave, &inst->carwave, delta);
            }
          else
            {
              // adjust_phase(inst, data);
            }
        }
      else  /* Case of no zerox point in the same sample block */
        {
          /* Check if sample block boundary */

          if (epos < (BLK_SAMPLES - BLK_SAMPLES_BOUNDARY) &&
              epos > BLK_SAMPLES_BOUNDARY)
            {
              /* Over the boundary */

              dprintf("Over boundary epos[%d]\n", epos);

              inst->interval_cnt = 0;
              next_state = PLLSTATE_PHASEUPDATED;
            }
          else
            {
              dprintf("PHASELOCKED NO zerox epos[%d]\n", epos);
            }
        }
    }

  return next_state;
}

/****************************************************************************
 * Private Functions for signal generation
 ****************************************************************************/

/** generate_refsig()
 * Generate reference signal.
 * Reference signal is generated just one cycle on top of timer counter.
 */

static void update_theta(wave_param_t *wave, int64_t bound)
{
  int64_t next_theta = wave->theta + wave->omega;
  int64_t next_wrap  = WRAP_PI48(next_theta + wave->delta);
  int64_t cur_wrap   = WRAP_PI48(wave->theta + wave->delta);

  /* Update wave counter */

  if (cur_wrap > next_wrap) /* In case of wrap round 2PI */
    {
      wave->wavcnt++;
      if (wave->wavcnt >= wave->freq)
        {
          wave->wavcnt = 0;
        }
    }

  /* Just in case */

  else if (wave->wavcnt >= wave->freq)
    {
      wave->wavcnt = 0;
    }

  /* Update theta */

  if (next_theta >= bound)
    {
      wave->theta = WRAP_PI48(next_theta);
    }
  else
    {
      wave->theta = next_theta;
    }
}

static int16_t generate_refsig(wave_param_t *wave)
{
  int16_t ret = 0;
  int16_t phase = PHASE48to16(wave->theta + wave->delta);

  if (wave->wavcnt == 0)
    {
      ret = ATTENATE_REFSIG(arm_sin_q15(phase));
    }

  update_theta(wave, wave->omega * wave->freq);

  return ret;
}

/** modulate_carrier()
 * Modulate carrier signal by sending data.
 */

static int16_t modulate_carrier(wave_param_t *wave)
{
  int16_t ret;
  int16_t phase = PHASE48to16(wave->theta + wave->delta);

  if (wave->wavcnt < 128 * 10)
    {
      ret = arm_sin_q15(phase);
    }
  else
    {
      ret = 0;
    }

  /* Update wave counter */

  update_theta(wave, M_2PI48);

  /* TODO: Modulation */

  return ret;
}

/** generate_signals()
 * Generate and store calculated signals to audio output buffer.
 */

static void generate_signals(worker_inst_t *inst, memblk_t *out)
{
  int i;
  int sample_num = memblk_spaceint16(out) / 2;
  struct lr_signal_s *sig = (struct lr_signal_s *)memblk_fillptrint16(out);

  for (i = 0; i < sample_num; i++)
    {
#ifdef CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_OUTPUT_REFSIG_ASSIGN_R
      sig[i].l = modulate_carrier(&inst->carwave);
      sig[i].r = generate_refsig(&inst->refwave);
#else
      sig[i].l = generate_refsig(&inst->refwave);
      sig[i].r = modulate_carrier(&inst->carwave);
#endif
    }

  memblk_commitint16(out, sample_num * 2);
}

/****************************************************************************
 * Private Functions of AudioLite worker framework
 ****************************************************************************/

/* on_playmsg():
 *  Received start message from Host (Main Core)
 */

static int on_playmsg(int state, void *arg, al_comm_msgopt_t *opt)
{
  worker_inst_t *inst = (worker_inst_t *)arg;

  inst->mode = opt->usr[0];

  return AL_COMMFW_RET_OK;
}

/* on_usrmsg():
 *  Received PLL Worker specific command.
 *  On this case, start / stop message for PLL
 */

static int on_usrmsg(int state, void *arg,
                     al_comm_msghdr_t hdr, al_comm_msgopt_t *opt)
{
  worker_inst_t *inst = (worker_inst_t *)arg;

  if (inst->pll_state == PLLSTATE_READY && opt->usr[0] == PLLCMD_START)
    {
      dprintf("[PLLWorker] Start PLL%c\n", ' ');
      inst->pll_state = PLLSTATE_1STEDGEDETECT;
    }
  else if (inst->pll_state != PLLSTATE_READY && opt->usr[0] == PLLCMD_STOP)
    {
      dprintf("[PLLWorker] Stop PLL%c\n", ' ');
      inst->pll_state = PLLSTATE_READY;
      initialize_instance(inst);
    }

  return AL_COMMFW_RET_OK;
}

/* on_process():
 *  This is called when the state is in PROCESS.
 */

static int on_process(void *arg)
{
  int edge_pos;
  int zerox;
  int next_state;
  memblk_t *memblk;
  worker_inst_t *inst = (worker_inst_t *)arg;

  /* Process Digital PLL */

  memblk = TAKE_IMEM(inst);
  if (memblk)
    {
      /* Signal has already split in the filter block.
       * So the upper half in input memory holds 1-PPS signal
       * and lower half does Reference 200Hz signal
       */

      memblk_init(&inst->onepps, &memblk->addr[0], memblk->size / 2);
      memblk_init(&inst->refsig, &memblk->addr[memblk->size / 2],
                                 memblk->size / 2);
      memblk_commit(&inst->onepps, inst->onepps.size);
      memblk_commit(&inst->refsig, inst->refsig.size);

      edge_pos = onepps_riseedge_detect(inst, &inst->onepps);
      zerox = zerocross_detect(inst, &inst->refsig, &inst->refsig_last);

      /* Process a function of current state */

      next_state = state_func[inst->pll_state](inst, edge_pos, zerox);
      display_statechange(inst->pll_state, next_state);
      inst->pll_state = next_state;

      state_led(state_ledptn[inst->pll_state]);

      FREE_MEMBLK(memblk, inst);
    }

  /* Process signal Ossilator */

  memblk = TAKE_OMEM(inst);
  if (memblk)
    {
      generate_signals(inst, memblk);
      FREE_MEMBLK(memblk, inst);
    }

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

  g_instance.pll_state = PLLSTATE_READY;

  memblk_init(&g_instance.refsig_last, &g_instance.last_refsig,
              sizeof(int16_t));
  memblk_init(&g_instance.refval_last, &g_instance.last_refval,
              sizeof(int16_t));

  initialize_instance(&g_instance);

  g_instance.mode = PLL_MODE_NORMAL;
#ifdef DEBUG_DISPLAY_ONEPPSPERIOD
  g_instance.debug_period = 0;
#endif

  /* Set callbacks to handle host message and
   * state processing
   */

  SET_PROCESS(cbs, on_process);
  SET_PLAYMSG(cbs, on_playmsg);
  SET_USRMSG(cbs, on_usrmsg);

  /* Send boot message to host to notice this worker is ready */

  alworker_send_bootmsg(PLL_WORKER_VERSION, NULL);

  /* Start message loop.
   * This function returns receive SYSTERM message from a host.
   */

  alworker_commfw_msgloop((alworker_insthead_t *)&g_instance);

  return 0;
}
