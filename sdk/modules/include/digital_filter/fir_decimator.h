/****************************************************************************
 * modules/include/digital_filter/fir_decimator.h
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

/**
 * @file fir_decimator.h
 */

#ifndef __INCLUDE_FILTERS_FIR_DECIMATOR_H
#define __INCLUDE_FILTERS_FIR_DECIMATOR_H

/**
 * @defgroup fir_decimator FIR Decimation Filter
 * @{
 *
 * FIR Decimation filters using CMSIS DSP
 */

#include <nuttx/config.h>

#ifdef CONFIG_EXTERNALS_CMSIS_DSP
#include <arm_math.h>

/**
 * @defgroup fir_decimator_datatype Data Types
 * @{
 */

typedef struct
{
  arm_fir_decimate_instance_f32 inst;
} decimator_instancef_t;

/** @} fir_decimator_datatype */

#else
#error "FIR filter needs CMSIS DSP library"
#endif

#  ifdef __cplusplus
#    define EXTERN extern "C"
extern "C"
{
#  else
#    define EXTERN extern
#  endif

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

/**
 * @defgroup fir_decimator_funcs Functions
 * @{
 */

/**
 * Create FIR Decimator intance
 *
 * @param [in] fs: Sampling rate of input signals.
 * @param [in] dec_factor: Cut off frequency. (Hz)
 * @param [in] tr_width: Transition frequency band width of Low Pass Filter(Hz)
 * @param [in] blocksz: Block size to execute filter calcuation in one time. (samples)
 *
 * @return decimator_instancef_t instance is returned on success, otherwise returns NULL.
 *
 * @note This function allocate memory for instance of fir_instancef_t, so you
 *       free when you finished to use the filter by decimator_delete().
 */
decimator_instancef_t *create_decimatorf(int fs, int dec_factor,
    int tr_width, int blocksz);

/**
 * Create FIR Decimator intance with Tap size
 *
 * @param [in] fs: Sampling rate of input signals.
 * @param [in] dec_factor: Cut off frequency. (Hz)
 * @param [in] taps: Tap size of FIR filter.
 * @param [in] blocksz: Block size to execute filter calcuation in one time. (samples)
 *
 * @return decimator_instancef_t instance is returned on success, otherwise returns NULL.
 *
 * @note This function allocate memory for instance of fir_instancef_t, so you
 *       free when you finished to use the filter by decimator_delete().
 */
decimator_instancef_t *create_decimatorf_tap(int fs, int dec_factor,
    int taps, int blocksz);

/**
 * Execute Decimation
 *
 * @param [in] dec: Instance of decimation_instancef_t.
 * @param [in] input: float array of input data.
 * @param [in] input_len: Length of input array. This size must be multiple of
 *                  blocksz set by create_decimatorf().
 * @param [out] output: float array of output data.
 * @param [in] output_len: Length of output array.
 *
 * @return Decimated data length, means size of output data.
 */
int decimator_executef(decimator_instancef_t *dec, float *input, int input_len,
    float *output, int output_len);

/**
 * Delete Decimator instance
 *
 * @param [in] dec: Instance of decimator_instancef_t.
 *
 * @return Tap number of the FIR filter.
 */
int decimator_tapnumf(decimator_instancef_t *dec);

/**
 * Delete Decimator instance
 *
 * @param [in] dec: Instance of decimator_instancef_t.
 */
void decimator_deletef(decimator_instancef_t *dec);

/** @} fir_decimator_funcs */

#  undef EXTERN
#  ifdef __cplusplus
}
#  endif

/** @} fir_decimator */

#endif  /* __INCLUDE_FILTERS_FIR_DECIMATOR_H */
