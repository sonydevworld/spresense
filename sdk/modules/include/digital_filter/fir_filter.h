/****************************************************************************
 * modules/include/digital_filter/fir_filter.h
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
 * @file fir_filter.h
 */

#ifndef __INCLUDE_FILTERS_FIR_FILTER_H
#define __INCLUDE_FILTERS_FIR_FILTER_H

/**
 * @defgroup fir_filter FIR Filter
 * @{
 *
 * FIR filters using CMSIS DSP
 */

#include <nuttx/config.h>

#ifdef CONFIG_EXTERNALS_CMSIS_DSP
#include <arm_math.h>

/**
 * @defgroup fir_datatype Data Types
 * @{
 */

/**
 * @typedef fir_instancef_t
 * Wrap type of arm_fir_instance_f32
 */
typedef arm_fir_instance_f32 fir_instancef_t;

/** @} fir_datatype */

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
 * @defgroup fir_funcs Functions
 * @{
 */

/**
 * Calculate FIR tap number from sampling frequency and transition band width
 *
 * @param [in] fs: Sampling rate of target signals.
 * @param [in] tr_width: Transition frequency band width. (Hz)
 *
 * @return tap number for the transision band width.
 */
int fir_calc_tapnumber(int fs, int tr_width);

/**
 * Create FIR Low Pass Filter coefficients
 *
 * @param [in] fs: Sampling rate of target signals.
 * @param [in] cuttoff_freq: Cut off frequency. (Hz)
 * @param [in] tr_width: Transition frequency band width. (Hz)
 * @param [in] blocksz: Block size to execute filter calcuation in one time. (samples)
 *
 * @return fir_instancef_t instance is returned on success, otherwise returns NULL.
 *
 * @note This function allocate memory for instance of fir_instancef_t, so you
 *       free when you finished to use the filter by fir_delete().
 */
fir_instancef_t * fir_create_lpff(int fs, int cutoff_freq,
    int tr_width, int blocksz);

/**
 * Create FIR Low Pass Filter coefficients with Tap size
 *
 * @param [in] fs: Sampling rate of target signals.
 * @param [in] cuttoff_freq: Cut off frequency. (Hz)
 * @param [in] taps: Tap size
 * @param [in] blocksz: Block size to execute filter calcuation in one time. (samples)
 *
 * @return fir_instancef_t instance is returned on success, otherwise returns NULL.
 *
 * @note This function allocate memory for instance of fir_instancef_t, so you
 *       free when you finished to use the filter by fir_delete().
 */
fir_instancef_t * fir_create_lpff_tap(int fs, int cutoff_freq,
    int taps, int blocksz);

/**
 * Create FIR High Pass Filter coefficients
 *
 * @param [in] fs: Sampling rate of target signals.
 * @param [in] cuttoff_freq: Cut off frequency. (Hz)
 * @param [in] tr_width: Transition frequency band width. (Hz)
 * @param [in] blocksz: Block size to execute filter calcuation in one time. (samples)
 *
 * @return fir_instancef_t instance is returned on success, otherwise returns NULL.
 *
 * @note This function allocate memory for instance of fir_instancef_t, so you
 *       free when you finished to use the filter by fir_delete().
 */
fir_instancef_t * fir_create_hpff(int fs, int cutoff_freq,
    int tr_width, int blocksz);

/**
 * Create FIR High Pass Filter coefficients with Tap size
 *
 * @param [in] fs: Sampling rate of target signals.
 * @param [in] cuttoff_freq: Cut off frequency. (Hz)
 * @param [in] taps: Tap size
 * @param [in] blocksz: Block size to execute filter calcuation in one time. (samples)
 *
 * @return fir_instancef_t instance is returned on success, otherwise returns NULL.
 *
 * @note This function allocate memory for instance of fir_instancef_t, so you
 *       free when you finished to use the filter by fir_delete().
 */
fir_instancef_t * fir_create_hpff_tap(int fs, int cutoff_freq,
    int taps, int blocksz);

/**
 * Create FIR Band Pass Filter coefficients
 *
 * @param [in] fs: Sampling rate of target signals.
 * @param [in] cuttoff_freq: Cut off frequency. (Hz)
 * @param [in] tr_width: Transition frequency band width. (Hz)
 * @param [in] blocksz: Block size to execute filter calcuation in one time. (samples)
 *
 * @return fir_instancef_t instance is returned on success, otherwise returns NULL.
 *
 * @note This function allocate memory for instance of fir_instancef_t, so you
 *       free when you finished to use the filter by fir_delete().
 */
fir_instancef_t * fir_create_bpff(int fs, int lower_cutfreq, int higher_cutfreq,
    int tr_width, int blocksz);

/**
 * Create FIR Band Pass Filter coefficients with Tap size
 *
 * @param [in] fs: Sampling rate of target signals.
 * @param [in] cuttoff_freq: Cut off frequency. (Hz)
 * @param [in] taps: Tap size
 * @param [in] blocksz: Block size to execute filter calcuation in one time. (samples)
 *
 * @return fir_instancef_t instance is returned on success, otherwise returns NULL.
 *
 * @note This function allocate memory for instance of fir_instancef_t, so you
 *       free when you finished to use the filter by fir_delete().
 */
fir_instancef_t * fir_create_bpff_tap(int fs, int lower_cutfreq, int higher_cutfreq,
    int taps, int blocksz);

/**
 * Create FIR Band Elimination Filter coefficients
 *
 * @param [in] fs: Sampling rate of target signals.
 * @param [in] cuttoff_freq: Cut off frequency. (Hz)
 * @param [in] tr_width: Transition frequency band width. (Hz)
 * @param [in] blocksz: Block size to execute filter calcuation in one time. (samples)
 *
 * @return fir_instancef_t instance is returned on success, otherwise returns NULL.
 *
 * @note This function allocate memory for instance of fir_instancef_t, so you
 *       free when you finished to use the filter by fir_delete().
 */
fir_instancef_t * fir_create_beff(int fs, int lower_cutfreq, int higher_cutfreq,
    int tr_width, int blocksz);

/**
 * Create FIR Band Elimination Filter coefficients with Tap size
 *
 * @param [in] fs: Sampling rate of target signals.
 * @param [in] cuttoff_freq: Cut off frequency. (Hz)
 * @param [in] taps: Tap size
 * @param [in] blocksz: Block size to execute filter calcuation in one time. (samples)
 *
 * @return fir_instancef_t instance is returned on success, otherwise returns NULL.
 *
 * @note This function allocate memory for instance of fir_instancef_t, so you
 *       free when you finished to use the filter by fir_delete().
 */
fir_instancef_t * fir_create_beff_tap(int fs, int lower_cutfreq, int higher_cutfreq,
    int taps, int blocksz);

/**
 * Get tap number of created FIR filter instance
 *
 * @param [in] fir: Target instance of fir_instancef_t to get tap number from.
 *
 * @return Tap number.
 */
int fir_get_tapnumf(fir_instancef_t *fir);

/**
 * Execute FIR filter
 *
 * @param [in] fir: Instance of fir_instancef_t.
 * @param [in] input: float array of input data.
 * @param [out] output: float array of output data.
 * @param [in] len: Length of arrays. This size must be multiple of blocksz set
 *                  by fir_create_xxx().
 */
void fir_executef(fir_instancef_t *fir, float *input, float *output, int len);

/**
 * Execute FIR filter and calculate the avsolute value
 *
 * @param [in] fir: Instance of fir_instancef_t.
 * @param [in] input: float array of input data.
 * @param [out] output: float array of output data.
 * @param [in] len: Length of arrays. This size must be multiple of blocksz set
 *                  by fir_create_xxx().
 */
void firabs_executef(fir_instancef_t *fir, float *input, float *output, int len);

/**
 * Delete FIR instance
 *
 * @param [in] fir: Instance of fir_instancef_t.
 */
void fir_deletef(fir_instancef_t *fir);

/** @} fir_funcs */

#  undef EXTERN
#  ifdef __cplusplus
}
#  endif

/** @} fir_filter */

#endif  /* __INCLUDE_FILTERS_FIR_FILTER_H */
