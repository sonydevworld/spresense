/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/filter/iir_filter.h
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

#ifndef __EXAMPLES_AUDIOLITE_DPLL_FILTER_IIR_FILTER_H
#define __EXAMPLES_AUDIOLITE_DPLL_FILTER_IIR_FILTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arm_math.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FILTER_TYPE_LPF (0)
#define FILTER_TYPE_HPF (1)
#define FILTER_TYPE_BPF (2)
#define FILTER_TYPE_BEF (3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct iir_filter_s
{
  float coef[5];
  float state_buf[2];
  arm_biquad_cascade_df2T_instance_f32 iir;
};
typedef struct iir_filter_s iir_filter_t; 

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int initialize_iir_filter(iir_filter_t *iir, int type,
                          int fs, int cutoff, float q);
void execute_iir_filter(iir_filter_t *iir, float *in, float *out, int sz);

#endif /* __EXAMPLES_AUDIOLITE_DPLL_FILTER_IIR_FILTER_H */
