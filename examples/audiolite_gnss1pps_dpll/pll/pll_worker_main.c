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

/* #define MODULATION_TEST */

#define PLLSTATE_READY          (0)
#define PLLSTATE_1STEDGEDETECT  (1)
#define PLLSTATE_FREQMEASURE    (2)
#define PLLSTATE_INTERVAL       (3)
#define PLLSTATE_PHASESHIFT     (4)
#define PLLSTATE_PHASEADJUST    (5)
#define PLLSTATE_PHASELOCKED    (6)
#define PLLSTATE_NUMKINDS       (7)

#define IS_PLLLOCKED(i) ((i)->pll_state == PLLSTATE_PHASELOCKED)

#define M_2PI48         ((int64_t)0x0000800000000000)
#define M_2PI48_MASK    ((int64_t)0x00007fffffffffff)
#define M_PI48          ((int64_t)0x0000400000000000)
#define WRAP_PI48(p)    (((p) >= 0) ? (p) & M_2PI48_MASK : \
                                     -((-p) & M_2PI48_MASK))
#define M_2PI16_MASK    ((int16_t)0x7fff)
#define PHASE48to16(p)  ((p) >= 0) ? \
                        (((int16_t)((p) / 0x100000000)) & M_2PI16_MASK) : \
                        (-(((int16_t)((-p) / 0x100000000)) & M_2PI16_MASK))

#define DETECT_FREQ_COUNT   (10)
#define OMEGA_ADJUST_FACTOR (SAMPLE_FS * DETECT_FREQ_COUNT)
#define THETA_ADJUST_FACTOR (SAMPLE_FS * DETECT_FREQ_COUNT * 10)

#define ONEPPS_RISEEDGE_THRESH  (25000)

#ifdef SINWAVE_IS_NEGATIVE
#  define REFSIG_TRIGGER_LEVEL  (-10000)
#  define CHECK_TRIGGER_LEVEL(l)  ((l) < REFSIG_TRIGGER_LEVEL)
#  define ZEROCROSS(l, c)  ((l) < 0 && (c) >= 0)
#else
#  define REFSIG_TRIGGER_LEVEL  (10000)
#  define CHECK_TRIGGER_LEVEL(l)  ((l) > REFSIG_TRIGGER_LEVEL)
#  define ZEROCROSS(l, c)  ((l) > 0 && (c) <= 0)
#endif

#define ATTENATE_REFSIG(s)   ((s) * 1 / 4)

#define EDGEDETECT_INTERVAL_SEC   (2)

#define BLK_SAMPLES_BOUNDARY (10)

#ifdef CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_PLL_DEBUG
#  define dprintf printf
#else
#  define dprintf(...)
#endif

#define MODULATION_PHASE_OFST (M_PI48 / 2)
#define MODULATION_ONEBIT_LEN (M_2PI48 * 2)

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct wave_param_s
{
  int64_t theta;
  int64_t omega;
  int freq;         /* (Hz) */
  int wavcnt;       /* wave number counter: 0 to freq -1 */
  int edge_wavcnt;  /* wave number count on the 1-PPS edge */
};
typedef struct wave_param_s wave_param_t;

struct xfer_data_s
{
  int64_t next_angle;
  int64_t cur_angle;
  int current_bit;
  uint8_t *data_inuse;
  uint8_t data_mem[XFER_DATA_BYTES];
};
typedef struct xfer_data_s xfer_data_t;

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

  int16_t ref_skew;

  xfer_data_t xfer_data;
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

static void update_freq(worker_inst_t *inst, int sec);
static int state_ready(worker_inst_t *inst, int epos, int zerox);
static int state_1stedgedetect(worker_inst_t *inst, int epos, int zerox);
static int state_freqmeasure(worker_inst_t *inst, int epos, int zerox);
static int state_interval(worker_inst_t *inst, int epos, int zerox);
static int state_phaseshift(worker_inst_t *inst, int epos, int zerox);
static int state_phaseadjust(worker_inst_t *inst, int epos, int zerox);
static int state_phaselocked(worker_inst_t *inst, int epos, int zerox);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static worker_inst_t g_instance;
static state_process state_func[PLLSTATE_NUMKINDS] =
{
  [PLLSTATE_READY]          = state_ready,
  [PLLSTATE_1STEDGEDETECT]  = state_1stedgedetect,
  [PLLSTATE_FREQMEASURE]    = state_freqmeasure,
  [PLLSTATE_INTERVAL]       = state_interval,
  [PLLSTATE_PHASESHIFT]     = state_phaseshift,
  [PLLSTATE_PHASEADJUST]    = state_phaseadjust,
  [PLLSTATE_PHASELOCKED]    = state_phaselocked,
};

static uint32_t state_ledptn[PLLSTATE_NUMKINDS] =
{
  [PLLSTATE_READY]          = 0,
  [PLLSTATE_1STEDGEDETECT]  = 1,
  [PLLSTATE_FREQMEASURE]    = 2,
  [PLLSTATE_INTERVAL]       = 3,
  [PLLSTATE_PHASESHIFT]     = 4,
  [PLLSTATE_PHASEADJUST]    = 5,
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
      STATE_NAME(FREQMEASURE);
      STATE_NAME(INTERVAL);
      STATE_NAME(PHASESHIFT);
      STATE_NAME(PHASEADJUST);
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
 * init_xfer_data()
 * Initialize xfer data
 */

static void init_xfer_data(xfer_data_t *xf)
{
  xf->data_inuse = NULL;
  xf->current_bit = -1;
  xf->cur_angle = 0;

#ifdef MODULATION_TEST
  {
    int i;
    for (i = 0; i < XFER_DATA_BYTES; i++)
      {
        xf->data_mem[i] = 0x55;
      }
  }
#endif
}

/**
 * set_xferdata()
 * Set xfer data for sending.
 */

static bool set_xferdata(xfer_data_t *xf, int ofst, uint8_t *d, int len)
{
  int i;
  bool ret = false;

  if (xf->data_inuse == NULL && len + ofst <= XFER_DATA_BYTES)
    {
      ret = true;
      len = len + ofst;
      for (i = ofst; i < len; i++)
        {
          xf->data_mem[i] = d[i - ofst];
        }
    }

  return ret;
}

/**
 * enable_xferdata()
 * Request xfer data. Not xfer started yet, because xfer should start
 * on the edge of 1PPS.
 */

static bool enable_xferdata(xfer_data_t *xf)
{
  if (xf->data_inuse == NULL)
    {
      init_xfer_data(xf);
      xf->data_inuse = xf->data_mem;
      xf->current_bit = -1;
      xf->cur_angle = 0;
      return true;
    }

  return false;
}

/**
 * cancel_xferdata()
 * Cancel current xfer data.
 */

static void cancel_xferdata(xfer_data_t *xf)
{
  if (xf->data_inuse != NULL)
    {
      init_xfer_data(xf);
    }
}

/**
 * modulate_psk()
 * Modulate the input signal 'sig' according to xter data.
 * Return true means all data is done to xfer.
 */

static bool modulate_psk(xfer_data_t *xf, int16_t *sig, int64_t car_omega)
{
  int bytepos;
  int bitpos;
  bool done = false;

  if (xf->data_inuse != NULL && xf->current_bit >= 0)
    {
      bytepos = xf->current_bit / 8;
      bitpos  = xf->current_bit % 8;
      *sig = xf->data_inuse[bytepos] & (1 << bitpos) ? *sig : -*sig;

      xf->cur_angle += car_omega;
      if (xf->cur_angle >= MODULATION_ONEBIT_LEN)
        {
          xf->cur_angle = xf->cur_angle - MODULATION_ONEBIT_LEN;
          xf->current_bit++;
          if (xf->current_bit >= XFER_DATA_BITS)
            {
              done = true;
              init_xfer_data(xf);
            }
        }
    }
  else
    {
      *sig = 0;
    }

  return done;
}

/**
 * start_xfer()
 * Start xfer data.
 * This function is called the start point on edge of 1PPS.
 * Return true if this is accepted. false is already in xfer.
 */

static bool start_xfer(xfer_data_t *xf)
{
  bool ret = false;

  if (xf->data_inuse != NULL && xf->current_bit == -1)
    {
      xf->current_bit = 0;
      ret = true;
    }

  return ret;
}

/**
 * is_xfering()
 * Check if xfer data is already started.
 */

static bool is_xfering(xfer_data_t *xf)
{
  return xf->data_inuse != NULL && xf->current_bit >= 0;
}

/**
 * initialize_instance()
 * reset the instance to back to begining
 */

static void initialize_instance(worker_inst_t *inst)
{
  inst->onepps_period  = SAMPLE_FS;
  inst->no_rise_detect = 200; /* 200 blocks of input memory is avoided */

  inst->zerox_detection = false;
  inst->zerox_period = 0;

  inst->refwave.freq = DEFAULT_REFFREQ;
  inst->carwave.freq = DEFAULT_CARRIERFREQ;
  update_freq(inst, 1);
  inst->refwave.theta = M_PI48;
  inst->refwave.wavcnt = 0;
  inst->refwave.edge_wavcnt = 0;
  inst->carwave.theta = 0;
  inst->carwave.wavcnt = 0;
  inst->carwave.edge_wavcnt = 0;
  inst->ref_skew = 0;

  init_xfer_data(&inst->xfer_data);
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

/**
 * update_freq()
 * Update the sample number of one second.
 * This value is the base value for the output sampling frequency. 
 */

static void update_freq(worker_inst_t *inst, int sec)
{
  inst->one_hz_period = inst->onepps_period / sec;
  inst->refwave.omega = M_2PI48 / (int64_t)inst->onepps_period * sec
                                * (int64_t)inst->refwave.freq;
}

/**
 * update_phase()
 * Update the sample time counter to fit the edge of 1PPS.
 */

static void update_phase(worker_inst_t *inst)
{
  int64_t dphase;
  int64_t dwcnt;
  int64_t dtheta;

  dphase = inst->refwave.omega * inst->zerox_period;
  dwcnt = (dphase + (M_2PI48_MASK)) / M_2PI48;
  dtheta = (dphase & M_2PI48_MASK) - M_PI48; /* Refsig has offset PI */

  inst->refwave.wavcnt += (int)dwcnt;
  inst->refwave.theta -= dtheta;
  while (inst->refwave.wavcnt >= inst->refwave.freq)
    {
      inst->refwave.wavcnt -= inst->refwave.freq;
    }
}

/**
 * adjust_theta()
 * Adjusting theta value roughly.
 */

static void adjust_theta(wave_param_t *wave, int cnt)
{
  if (cnt == 1 && cnt == -1)
    {
      wave->theta = wave->theta + ((int64_t)cnt * wave->omega * 3 / 10);
    }
  else
    {
      wave->theta = wave->theta + ((int64_t)cnt * wave->omega);
    }
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

/**
 * adjust_refwave()
 * According to refwave data on 1PPS edge,
 * adjust refsig freq and position (theta).
 */

static void adjust_refwave(wave_param_t *refwav, int one_hz,
                           int16_t data, int16_t diff, int skew)
{
  /* If 'diff' is positive, ref wave is slightly slow, decrease omega value.
   * If it is negative, ref wave is slightly fast, increase omega value.
   */

  if (diff != 0)
    {
      refwav->omega += (refwav->omega  * diff) / (int64_t)skew * DETECT_FREQ_COUNT / one_hz;
    }

  /* 'data' indicates current position,
   * for positive, it is before 1PPS, and for negative, it is behind 1PPS.
   */

  if (data != 0)
    {
      refwav->theta += (refwav->omega * data) / (int64_t)skew;
    }
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
      inst->interval_cnt = 0;
      next_state = PLLSTATE_FREQMEASURE;
    }

  return next_state;
}

/** state_freqmeasure() : Process in the state of FREQMEASURE.
 * Search 2nd rising edge for measureing the period of 1-PPS.
 */

static int state_freqmeasure(worker_inst_t *inst, int epos, int zerox)
{
  int next_state = PLLSTATE_FREQMEASURE;
  if (epos >= 0)
    {
      inst->interval_cnt++;
      dprintf("Edge Period : %d:%d\n", inst->interval_cnt,
                                       inst->onepps_period + epos);
      if (inst->interval_cnt == DETECT_FREQ_COUNT)
        {
          inst->onepps_period += epos;
          update_freq(inst, DETECT_FREQ_COUNT);
          inst->interval_cnt = 0;
          next_state = PLLSTATE_INTERVAL;
        }
      else
        {
          inst->onepps_period += memblk_remainint16(&inst->onepps);
        }
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

          zerox -= epos;
          if (zerox >= 0)
            {
              inst->zerox_period = zerox;
              update_phase(inst);
              inst->interval_cnt = 0;
              inst->zerox_period = -1;
              next_state = PLLSTATE_PHASEADJUST;
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
      inst->zerox_period = -1;
      next_state = PLLSTATE_PHASEADJUST;
    }
  else
    {
      inst->zerox_period += memblk_remainint16(&inst->refsig);
    }

  return next_state;
}

/** state_phaseadjust() : Process in the state of PHASEADJUST.
 * This state is just wait 1-PPS edge signal for stabilizing refrerence
 * after updateed of the phase.
 */

static int state_phaseadjust(worker_inst_t *inst, int epos, int zerox)
{
  int diff;
  int next_state = PLLSTATE_PHASEADJUST;

  /* Measure distance between 1pps and zero-cross point of ref-signal
   * And adjust the phase of ref-signal.
   */

  if (inst->zerox_period < 0) /* Not detected 1PPS edge yet */
    {
      if (epos >= 0)  /* Detect 1PPS edge */
        { 
          if (zerox >= 0) /* Zero-cross point is in the same block */
            {
              diff = zerox - epos;
              dprintf("Adjusting phase zerox[%d] - epos[%d] = %d\n",
                    zerox, epos, diff);
              if (diff != 0)  /* Need rough adjustment */
                {
                  adjust_theta(&inst->refwave, diff);
                }
              else
                {
                  /* 1PPS position and zero-cross position are the same */

                  memblk_reset(&inst->refval_last);
                  next_state = PLLSTATE_PHASELOCKED;
                }
            }
          else /* No zero-cross point in the same block */
            {
              inst->zerox_period =
                memblk_remainint16(&inst->refsig) - epos;
            }
        }
    }
  else  /* 1PPS edge has been already detected */
    {
      if (zerox >= 0) /* Zero-cross is detected */
        {
          inst->zerox_period += zerox;

          /* Check nerest boundary of 1PPS edge */

          if ((inst->one_hz_period / 2 - inst->zerox_period) < 0)
            {
              inst->zerox_period =
                inst->zerox_period - inst->one_hz_period;
            }

          dprintf("Adjusting zerox period %d\n", inst->zerox_period);
          adjust_theta(&inst->refwave, inst->zerox_period);
          inst->zerox_period = -1;
        }
      else  /* No zero-cross point */
        {
          inst->zerox_period +=memblk_remainint16(&inst->refsig);
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

  int16_t last;
  int16_t data;
  int16_t diff;

  /* Detecting skew */

  if (zerox >= 0)
    {
      int16_t *dat = memblk_dataptrint16(&inst->refsig);
      if (zerox >= memblk_remainint16(&inst->refsig) - 1)
        {
          inst->ref_skew = dat[zerox - 1] - dat[zerox];
        }
      else
        {
          inst->ref_skew = dat[zerox] - dat[zerox + 1];
        }
    }

  /* Adjust refwave */

  if (epos >= 0)
    {
      /* Get the reference signal value on the same position of 1PPS Edge */

      memblk_dropint16(&inst->refsig, epos);
      data = memblk_pop_int16(&inst->refsig);

      if (memblk_remainint16(&inst->refval_last))
        {
          /* If the last reference signal is stored,
           * compare current and last to detect the direction in whilch
           * the reference signal shifts with respect to the 1PPS signal.
           */

          last = memblk_pop_int16(&inst->refval_last);
          diff = data - last;
          dprintf("Skew:%d Cur:%d - Last:%d = %d\n", inst->ref_skew, data, last, diff);
          adjust_refwave(&inst->refwave, inst->onepps_period, data, diff, inst->ref_skew);

          /* Just reset memblock (make it empty)
           * Because adjusted results are reflected in the timing of 
           * the next signal generation.
           */

          memblk_reset(&inst->refval_last);
        }
      else
        {
          /* Store the current data as last data */

          memblk_push_int16(&inst->refval_last, data);
        }

      if (zerox >= 0)
        {
          int sampdiff = zerox - epos;
          dprintf("LOCKED phase zerox[%d] - epos[%d] = %d\n",
                zerox, epos, sampdiff);
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

static int update_theta(wave_param_t *wave, int64_t bound)
{
  int ret = 0;
  int64_t next_theta = wave->theta + wave->omega;
  int64_t next_wrap  = WRAP_PI48(next_theta);
  int64_t cur_wrap   = WRAP_PI48(wave->theta);

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

  /* Trigger for starting carrier signal */

  cur_wrap  = WRAP_PI48(wave->theta);
  next_wrap = WRAP_PI48(next_theta);
  if (wave->wavcnt == 0 && cur_wrap < M_PI48 && next_wrap >= M_PI48)
    {
      ret = 1;
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

  return ret;
}

static int16_t generate_refsig(wave_param_t *wave, int *reset_carrier)
{
  int16_t ret = 0;
  int16_t phase = PHASE48to16(wave->theta);

  if (wave->wavcnt == 0)
    {
      ret = ATTENATE_REFSIG(arm_sin_q15(phase));
    }

  *reset_carrier = update_theta(wave, wave->omega * wave->freq);

  return ret;
}

/** modulate_carrier()
 * Modulate carrier signal by sending data.
 */

static int16_t modulate_carrier(wave_param_t *wave, xfer_data_t *xf,
                                wave_param_t *ref, int start)
{
  int16_t ret;
  int16_t phase;
  al_comm_msgopt_t opt = { 0 };

  if (start)
    {
      wave->wavcnt = 0;
      wave->theta = 0;
      wave->omega = ref->omega * wave->freq / ref->freq;
#ifdef MODULATION_TEST
      enable_xferdata(xf);
      if (start_xfer(xf))
        {
          dprintf("Start Xfer%c\n", ' ');
        }
#else
      start_xfer(xf);
#endif
    }

  if (is_xfering(xf))
    {
      phase = PHASE48to16(wave->theta + MODULATION_PHASE_OFST);
      ret = arm_sin_q15(phase);
      if (modulate_psk(xf, &ret, wave->omega))
        {
          opt.usr[0] = PLLCMD_DONE_XFER;
          alworker_send_usrcmd(&opt);
        }
    }
  else
    {
      ret = 0;
    }

  /* Update wave counter */

  update_theta(wave, M_2PI48);

  return ret;
}

/** generate_signals()
 * Generate and store calculated signals to audio output buffer.
 */

static void generate_signals(worker_inst_t *inst, memblk_t *out)
{
  int i;
  int start_carrier;
  int sample_num = memblk_spaceint16(out) / 2;
  struct lr_signal_s *sig = (struct lr_signal_s *)memblk_fillptrint16(out);

  for (i = 0; i < sample_num; i++)
    {
#ifdef CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_OUTPUT_REFSIG_ASSIGN_R
      sig[i].r = generate_refsig(&inst->refwave, &start_carrier);
      sig[i].l = 0;
      if (IS_PLLLOCKED(inst))
        {
          sig[i].l = modulate_carrier(&inst->carwave, &inst->xfer_data,
                                      &inst->refwave, start_carrier);
        }
#else
      sig[i].l = generate_refsig(&inst->refwave, &start_carrier);
      sig[i].r = 0;
      if (IS_PLLLOCKED(inst))
        {
          sig[i].r = modulate_carrier(&inst->carwave, &inst->xfer_data,
                                      &inst->refwave, start_carrier);
        }
#endif
    }

  memblk_commitint16(out, sample_num * 2);
}

/****************************************************************************
 * Private Functions of AudioLite worker framework
 ****************************************************************************/

/* on_usrmsg():
 *  Receive and handle PLL Worker specific command from main core.
 */

static int on_usrmsg(int state, void *arg,
                     al_comm_msghdr_t hdr, al_comm_msgopt_t *opt)
{
  int dofst, dlen;
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
  else if ((opt->usr[0] & PLLXFERDATACMDMASK) == PLLCMD_XFERDATA)
    {
      dofst = PLLXFERDATACMD_OFST(opt->usr[0]);
      dlen = PLLXFERDATACMD_LEN(opt->usr[0]);
      if (dlen > 12)
        {
          alworker_resp_usrcmd(PLLCMD_RETCODE_NG, opt);
        }
      else if (set_xferdata(&inst->xfer_data, dofst,
                            (uint8_t *)&opt->usr[1], dlen))
        {
          alworker_resp_usrcmd(PLLCMD_RETCODE_OK, opt);
        }
      else
        {
          alworker_resp_usrcmd(PLLCMD_RETCODE_NG, opt);
        }
    }
  else if (opt->usr[0] == PLLCMD_ENABLEXFER)
    {
      if (enable_xferdata(&inst->xfer_data))
        {
          alworker_resp_usrcmd(PLLCMD_RETCODE_OK, opt);
        }
      else
        {
          alworker_resp_usrcmd(PLLCMD_RETCODE_NG, opt);
        }
    }
  else if (opt->usr[0] == PLLCMD_CANCELXFER)
    {
      cancel_xferdata(&inst->xfer_data);
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

  /* Process signal Oscillator */

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

  /* Set callbacks to handle host message and
   * state processing
   */

  SET_PROCESS(cbs, on_process);
  SET_USRMSG(cbs, on_usrmsg);

  /* Send boot message to host to notice this worker is ready */

  alworker_send_bootmsg(PLL_WORKER_VERSION, NULL);

  /* Start message loop.
   * This function returns receive SYSTERM message from a host.
   */

  alworker_commfw_msgloop((alworker_insthead_t *)&g_instance);

  return 0;
}
