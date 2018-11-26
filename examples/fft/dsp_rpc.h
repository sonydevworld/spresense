/****************************************************************************
 * examples/fft/dsp_rpc.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef __EXAMPLES_FFT_DSP_RPC_H
#define __EXAMPLES_FFT_DSP_RPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "resource.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: fft
 *
 * Description:
 *   Execute a single FFT calculation. This is a bloking API, and wait until
 *   a FFT calculation is completed on the DSP.
 *
 * Input Parameters:
 *   pSrc   - Input buffer to the array of float data
 *            If CONFIG_EXAMPLES_FFT_REAL_DATA=y, data have only real number.
 *            If CONFIG_EXAMPLES_FFT_COMPLEX_DATA=y, data have complex number.
 *   pDst   - Output buffer to a computed FFT spectrum
 *   fftLen - The number of FFT samples,
 *            16, 32, 64, 128, 256, 1024 or 2048
 *
 ****************************************************************************/

void fft(void *pSrc, void *pDst, uint32_t fftLen);

/****************************************************************************
 * Name: fft_request
 *
 * Description:
 *   Request the multiple FFT calculations. This is the non-blocking API,
 *   and is returned before FFT calculation is completed. It's possible
 *   to get the result of FFT calculation using fft_wait_response().
 *
 * Input Parameters:
 *   pDesc - a pointer to the fft_desc_t descriptor
 *            struct _fft_desc
 *            {
 *              fft_desc_t  *pNext;    // the pointer to next descriptor
 *              void        *pSrc;     // input buffer
 *              void        *pDst;     // output buffer
 *              uint32_t    fftLen;    // FFT samples
 *              uint32_t    attribute; // FFT attribute
 *            };
 *           When FFT_ATTR_NOTIFY is set to the attribute, FFT completion of
 *           the descriptor is notified.
 *
 ****************************************************************************/

void fft_request(fft_desc_t *pDesc);

/****************************************************************************
 * Name: fft_wait_response
 *
 * Description:
 *   Wait for the notification of FFT completion.
 *
 * Input Parameters:
 *   ms    - timeout value in millisecondes. If ms is 0, wait the response
 *           infinitely without timeout.
 *   paddr - address of FFT calculated result
 *
 * Returned Value:
 *   DSP return code.
 *   If this is a negative value, it means the error code.
 *
 ****************************************************************************/

int fft_wait_response(uint32_t ms, void *paddr);

/****************************************************************************
 * Name: fft_stream_init
 *
 * Description:
 *   Initialize FFT stream funcion. This function must be called before
 *   fft_stream_input() function is executed.
 *
 * Input Parameters:
 *   fftLen - The number of FFT samples,
 *            16, 32, 64, 128, 256, 1024 or 2048
 *   fftShift - offset of the next FFT samples after a FFT is completed.
 *              If fftShift is 0, fftShift treat fftShift as equivalent to
 *              fftLen.
 *   type - type of the real data. TYPE_FLOAT or TYPE_SHORT
 *
 ****************************************************************************/

void fft_stream_init(uint32_t fftLen, uint32_t fftShift, fft_stream_type_t type);

/****************************************************************************
 * Name: fft_stream_input
 *
 * Description:
 *   Input FFT stream data.
 *
 * Input Parameters:
 *   pBuf - pointer to input data
 *   size - size of input data
 *
 ****************************************************************************/

void fft_stream_input(void *pBuf, size_t size);

/****************************************************************************
 * Name: fft_stream_output
 *
 * Description:
 *   Get the output of FFT calculated stream
 *
 * Input Parameters:
 *   ms    - timeout value in millisecondes. If ms is 0, wait the response
 *           infinitely without timeout.
 *   paddr - address of FFT calculated result
 *
 * Returned Value:
 *   DSP return code.
 *   If this is a negative value, it means the error code.
 *
 ****************************************************************************/

int fft_stream_output(uint32_t ms, void *paddr);

/****************************************************************************
 * Name: fft_window
 *
 * Description:
 *   Set the FFT window
 *
 * Input Parameters:
 *   type - FFT_WIN_RECTANGLE uses the rectangle window.
 *          FFT_WIN_HAMMING uses the hamming window.
 *          FFT_WIN_HANNING uses the hanning window.
 *
 ****************************************************************************/

void fft_window(int type);

/****************************************************************************
 * Name: fft_peak
 *
 * Description:
 *   Set the buffer to restore the peak frequency of FFT spectrum.
 *
 * Input Parameters:
 *   pPeakBuf - the float array of the peak frequency
 *   bufLen   - the length of pPeakBuf
 *   fs       - the sampling frequency
 *
 ****************************************************************************/

void fft_peak(float *pPeakBuf, int bufLen, int fs);

/****************************************************************************
 * Name: load_library
 *
 * Description:
 *   Load a DSP ELF program
 *
 * Input Parameters:
 *   filename - the absolute path of the loaded DSP program
 *
 ****************************************************************************/

int load_library(const char *filename);

/****************************************************************************
 * Name: unload_library
 *
 * Description:
 *   Unload the loaded DSP ELF program
 *
 ****************************************************************************/

void unload_library(void);

#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLES_FFT_DSP_RPC_H */
