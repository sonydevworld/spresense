/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/filter/iir_filter.c
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

#include <math.h>
#include <arm_math.h>
#include "iir_filter.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONST_LOG2  (0.693147180559945286227f)

#define itemsof(a) (sizeof(a)/sizeof(a[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*iir_coef_func_t)(iir_filter_t *iir, float q, float w);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void calc_lpf_coef(iir_filter_t *iir, float q, float w);
static void calc_hpf_coef(iir_filter_t *iir, float q, float w);
static void calc_bpf_coef(iir_filter_t *iir, float q, float w);
static void calc_bef_coef(iir_filter_t *iir, float q, float w);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const iir_coef_func_t coef_funcs[] =
{
  calc_lpf_coef,
  calc_hpf_coef,
  calc_bpf_coef,
  calc_bef_coef,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void create_iir(iir_filter_t *iir, float a[3], float b[3])
{
  iir->coef[0] =  b[0] / a[0];
  iir->coef[1] =  b[1] / a[0];
  iir->coef[2] =  b[2] / a[0];
  iir->coef[3] = -a[1] / a[0];
  iir->coef[4] = -a[2] / a[0];

  arm_biquad_cascade_df2T_init_f32(&iir->iir, 1, iir->coef, iir->state_buf);
}

static void calc_lpf_coef(iir_filter_t *iir, float q, float w)
{
  float k;
  float a[3];
  float b[3];

  k = arm_sin_f32(w) / (2.0f * q);
  a[0] =  1.0f + k;
  a[1] = -2.0f * arm_cos_f32(w);
  a[2] =  1.0f - k;

  k = 1.0f - arm_cos_f32(w);
  b[0] = k / 2.0f;
  b[1] = k;
  b[2] = k / 2.0f;

  create_iir(iir, a, b);
}

static void calc_hpf_coef(iir_filter_t *iir, float q, float w)
{
  float k;
  float a[3];
  float b[3];

  k = arm_sin_f32(w) / (2.0f * q);
  a[0] =  1.0f + k;
  a[1] = -2.0f * arm_cos_f32(w);
  a[2] =  1.0f - k;

  k = 1.0f + arm_cos_f32(w);
  b[0] =  k / 2.0f;
  b[1] = -k;
  b[2] =  k / 2.0f;

  create_iir(iir, a, b);
}

static void calc_bpf_coef(iir_filter_t *iir, float q, float w)
{
  float k;
  float a[3];
  float b[3];

  k = arm_sin_f32(w) * sinhf(CONST_LOG2 / 2.0 * q * w / arm_sin_f32(w));
  a[0] =  1.0f + k;
  a[1] = -2.0f * arm_cos_f32(w);
  a[2] =  1.0f - k;

  b[0] =  k;
  b[1] =  0.0f;
  b[2] = -k;

  create_iir(iir, a, b);
}

static void calc_bef_coef(iir_filter_t *iir, float q, float w)
{
  float k;
  float a[3];
  float b[3];

  k = arm_sin_f32(w) * sinhf(CONST_LOG2 / 2.0 * q * w / arm_sin_f32(w));
  a[0] =  1.0f + k;
  a[1] = -2.0f * arm_cos_f32(w);
  a[2] =  1.0f - k;

  b[0] =  1.0f;
  b[1] = -2.0f * arm_cos_f32(w);
  b[2] =  1.0f;

  create_iir(iir, a, b);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int initialize_iir_filter(iir_filter_t *iir,
                           int type, int fs, int cutoff, float q)
{
  float w;

  if (type < itemsof(coef_funcs))
    {
      w = 2.0f * M_PI * cutoff / fs;

      coef_funcs[type](iir, q, w);
      return 0;
    }

  return -1;
}

void execute_iir_filter(iir_filter_t *iir, float *in, float *out, int sz)
{
  arm_biquad_cascade_df2T_f32(&iir->iir, in, out, sz);
}
