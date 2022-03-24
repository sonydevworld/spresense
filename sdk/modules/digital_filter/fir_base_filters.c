/****************************************************************************
 * modules/digital_filter/fir_base_filters.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <arm_math.h>

#include <digital_filter/fir_filter.h>
#include <digital_filter/fir_decimator.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int tap_number(int fs, int tr_width)
{
  int taps;

  taps = (int)roundf(3.1 / ((float)tr_width / (float)fs));
  taps += (taps & 0x01);

  return taps + 1;
}

static void hanning_window(float *coeffs, int taps)
{
  int i;

  for (i = 0; i < taps; i++)
    {
      coeffs[i] *= 0.5 - 0.5 * cos(2.0 * M_PI * (i + 0.5) / taps);
    }
}

static float sinc(float n)
{
  if (n == 0)
    {
      return 1.0;
    }

  return sin(n) / n;
}

static arm_fir_instance_f32 * prepare_fir(int taps, float *coeffs, int blocksz)
{
  arm_fir_instance_f32 *S;
  float *state;

  S = (arm_fir_instance_f32 *)malloc(sizeof(arm_fir_instance_f32));
  if (S == NULL)
    {
      return NULL;
    }

  state = (float *)malloc(sizeof(float) * (taps + blocksz - 1));
  if (state == NULL)
    {
      free(S);
      return NULL;
    }

  arm_fir_init_f32(S, taps, coeffs, state, blocksz);

  return S;
}

static decimator_instancef_t * prepare_decimator(int dec_factor, int taps,
                                                 float *coeffs, int blocksz)
{
  decimator_instancef_t *S;
  float *state;

  S = (decimator_instancef_t *)
    malloc(sizeof(decimator_instancef_t));
  if (S == NULL)
    {
      return NULL;
    }

  state = (float *)malloc(sizeof(float) * (taps + blocksz - 1));
  if (state == NULL)
    {
      free(S);
      return NULL;
    }

  arm_fir_decimate_init_f32(&S->inst, taps, dec_factor, coeffs, state, blocksz);

  return S;
}

static float * fir_coeffs_lpf(float fe, int taps)
{
  int i;
  int half = (taps - 1) / 2;
  float *coeffs;

  coeffs = (float *)malloc(sizeof(float) * taps);
  if (coeffs == NULL)
    {
      return NULL;
    }

  for (i = -half; i <= half; i++)
    {
      coeffs[half + i] = 2.0 * fe * sinc(2.0 * M_PI * fe * i);
    }

  hanning_window(coeffs, taps);

  return coeffs;
}

static float * fir_coeffs_hpf(float fe, int taps)
{
  int i;
  int half = (taps - 1) / 2;
  float *coeffs;

  coeffs = (float *)malloc(sizeof(float) * taps);
  if (coeffs == NULL)
    {
      return NULL;
    }

  for (i = -half; i <= half; i++)
    {
      coeffs[half + i] = sinc(M_PI * i) - (2.0 * fe * sinc(2.0 * M_PI * fe * i));
    }

  hanning_window(coeffs, taps);

  return coeffs;
}

static float * fir_coeffs_bpf(float fe1, float fe2, int taps)
{
  int i;
  int half = (taps - 1) / 2;
  float *coeffs;

  coeffs = (float *)malloc(sizeof(float) * taps);
  if (coeffs == NULL)
    {
      return NULL;
    }

  for (i = -half; i <= half; i++)
    {
      coeffs[half + i] = (2 * fe2 * sinc(2 * M_PI * fe2 * i))
        - (2.0 * fe1 * sinc(2.0 * M_PI * fe1 * i));
    }

  hanning_window(coeffs, taps);

  return coeffs;
}

static float * fir_coeffs_bef(float fe1, float fe2, int taps)
{
  int i;
  int half = (taps - 1) / 2;
  float *coeffs;

  coeffs = (float *)malloc(sizeof(float) * taps);
  if (coeffs == NULL)
    {
      return NULL;
    }

  for (i = -half; i <= half; i++)
    {
      coeffs[half + i] = sinc(M_PI * i) - (2 * fe2 * sinc(2 * M_PI * fe2 * i))
        + (2.0 * fe1 * sinc(2.0 * M_PI * fe1 * i));
    }

  hanning_window(coeffs, taps);

  return coeffs;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * FIR Filter
 ****************************************************************************/

#ifdef CONFIG_DIGITAL_FILTER_FIR

/** fir_create_lpff() */

fir_instancef_t * fir_create_lpff(int fs, int cutoff_freq, int tr_width,
    int blocksz)
{
  if ((tr_width <= 0) || (fs <= 0) || (blocksz <= 0))
    {
      return NULL;
    }

  return fir_create_lpff_tap(fs, cutoff_freq,
                             tap_number(fs, tr_width), blocksz);
}

/** fir_create_lpff_tap() */

fir_instancef_t * fir_create_lpff_tap(int fs, int cutoff_freq, int taps,
    int blocksz)
{
  fir_instancef_t *S;
  float *coeffs;

  if ((taps <= 0) || (fs <= 0) || (blocksz <= 0))
    {
      return NULL;
    }

  taps = (taps & 0x01) ? taps : taps + 1;

  coeffs = fir_coeffs_lpf((float)cutoff_freq / (float)fs, taps);
  if (coeffs == NULL)
    {
      return NULL;
    }

  S = prepare_fir(taps, coeffs, blocksz);
  if (S == NULL)
    {
      free(coeffs);
    }

  return S;
}

/** fir_create_hpff() */

fir_instancef_t * fir_create_hpff(int fs, int cutoff_freq, int tr_width,
    int blocksz)
{
  if ((tr_width <= 0) || (fs <= 0) || (blocksz <= 0))
    {
      return NULL;
    }

  return fir_create_hpff_tap(fs, cutoff_freq,
                             tap_number(fs, tr_width), blocksz);
}

/** fir_create_hpff_tap() */

fir_instancef_t * fir_create_hpff_tap(int fs, int cutoff_freq, int taps,
    int blocksz)
{
  fir_instancef_t *S;
  float *coeffs;

  if ((taps <= 0) || (fs <= 0) || (blocksz <= 0))
    {
      return NULL;
    }

  taps = (taps & 0x01) ? taps : taps + 1;

  coeffs = fir_coeffs_hpf((float)cutoff_freq / (float)fs, taps);
  if (coeffs == NULL)
    {
      return NULL;
    }

  S = prepare_fir(taps, coeffs, blocksz);
  if (S == NULL)
    {
      free(coeffs);
    }

  return S;
}

/** fir_create_bpff() */

fir_instancef_t * fir_create_bpff(int fs, int lower_cutfreq,
    int higher_cutfreq, int tr_width, int blocksz)
{
  if (tr_width <= 0 || fs <= 0 || blocksz <= 0)
    {
      return NULL;
    }

  return fir_create_bpff_tap(fs, lower_cutfreq, higher_cutfreq,
                             tap_number(fs, tr_width), blocksz);
}

/** fir_create_bpff_tap() */

fir_instancef_t * fir_create_bpff_tap(int fs, int lower_cutfreq,
    int higher_cutfreq, int taps, int blocksz)
{
  fir_instancef_t *S;
  float *coeffs;

  if (taps <= 0 || fs <= 0 || blocksz <= 0)
    {
      return NULL;
    }

  taps = (taps & 0x01) ? taps : taps + 1;

  coeffs = fir_coeffs_bpf((float)lower_cutfreq / (float)fs,
                          (float)higher_cutfreq / (float)fs, taps);
  if (coeffs == NULL)
    {
      return NULL;
    }

  S = prepare_fir(taps, coeffs, blocksz);
  if (S == NULL)
    {
      free(coeffs);
    }

  return S;
}

/** fir_create_beff() */

fir_instancef_t * fir_create_beff(int fs, int lower_cutfreq,
    int higher_cutfreq, int tr_width, int blocksz)
{
  if (tr_width <= 0 || fs <= 0 || blocksz <= 0)
    {
      return NULL;
    }

  return fir_create_beff_tap(fs, lower_cutfreq, higher_cutfreq,
                             tap_number(fs, tr_width), blocksz);
}

/** fir_create_beff_tap() */

fir_instancef_t * fir_create_beff_tap(int fs, int lower_cutfreq,
    int higher_cutfreq, int taps, int blocksz)
{
  fir_instancef_t *S;
  float *coeffs;

  if (taps <= 0 || fs <= 0 || blocksz <= 0)
    {
      return NULL;
    }

  taps = (taps & 0x01) ? taps : taps + 1;

  coeffs = fir_coeffs_bef((float)lower_cutfreq / (float)fs,
                          (float)higher_cutfreq / (float)fs, taps);
  if (coeffs == NULL)
    {
      return NULL;
    }

  S = prepare_fir(taps, coeffs, blocksz);
  if (S == NULL)
    {
      free(coeffs);
    }

  return S;
}

/** fir_get_tapnumf() */

int fir_get_tapnumf(fir_instancef_t *fir)
{
  return fir->numTaps;
}

/** fir_executef() */

void fir_executef(fir_instancef_t *fir, float *input, float *output, int len)
{
  arm_fir_f32(fir, input, output, len);
}

/** firabs_executef() */

void firabs_executef(fir_instancef_t *fir, float *input, float *output, int len)
{
  arm_fir_f32(fir, input, output, len);
  arm_abs_f32(output, output, len);
}

/** fir_calc_tapnumber() */

int fir_calc_tapnumber(int fs, int tr_width)
{
  return tap_number(fs, tr_width);
}

/** fir_delete() */

void fir_deletef(fir_instancef_t *fir)
{
  if (fir)
    {
      free(fir->pState);
      free((void *)fir->pCoeffs);
      free(fir);
    }
}

#endif  /* CONFIG_DIGITAL_FILTER_FIR */

/****************************************************************************
 * Decimation Filter
 ****************************************************************************/

#ifdef CONFIG_DIGITAL_FILTER_DECIMATOR

/** create_decimatorf() */

decimator_instancef_t *create_decimatorf(int fs, int dec_factor,
    int tr_width, int blocksz)
{
  decimator_instancef_t *S;

  if ((dec_factor <= 0) || (fs <= 0) || (blocksz <= 0))
    {
      return NULL;
    }

  S = create_decimatorf_tap(fs, dec_factor, tap_number(fs, tr_width), blocksz);

  return S;
}

/** create_decimatorf_tap() */

decimator_instancef_t *create_decimatorf_tap(int fs, int dec_factor,
    int taps, int blocksz)
{
  decimator_instancef_t *S;
  float *coeffs;

  if ((dec_factor <= 0) || (fs <= 0) || (blocksz <= 0))
    {
      return NULL;
    }

  if (taps > 0)
    {
      taps = (taps & 0x01) ? taps : taps + 1;

      coeffs = fir_coeffs_lpf(1.f / (float)(2 * dec_factor), taps);
      if (coeffs == NULL)
        {
          return NULL;
        }

      S = prepare_decimator(dec_factor, taps, coeffs, blocksz);
    }
  else
    {
      S = (decimator_instancef_t *)malloc(sizeof(decimator_instancef_t));
      if (S)
        {
          S->inst.numTaps = 0;
          S->inst.M = dec_factor;
        }
    }

  return S;
}

/** decimator_execute() */

int decimator_executef(decimator_instancef_t *dec, float *input, int input_len,
    float *output, int output_len)
{
  int i;
  int output_sz = input_len / dec->inst.M;

  if (output_len < output_sz)
    {
      return -1;
    }

  if (dec->inst.numTaps)
    {
      arm_fir_decimate_f32(&dec->inst, input, output, input_len);
    }
  else
    {
      /* Do without filter */

      for (i = 0; i < output_sz; i++)
        {
          output[i] = input[i * dec->inst.M];
        }
    }

  return output_sz;
}

/** decimator_tapnumf() */

int decimator_tapnumf(decimator_instancef_t *dec)
{
  return dec->inst.numTaps;
}

/** decimator_deletef() */

void decimator_deletef(decimator_instancef_t *dec)
{
  if (dec)
    {
      if (dec->inst.numTaps)
        {
          free(dec->inst.pState);
          free((void *)dec->inst.pCoeffs);
        }

      free(dec);
    }
}

#endif  /* CONFIG_DIGITAL_FILTER_DECIMATOR */
