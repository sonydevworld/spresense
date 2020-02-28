/****************************************************************************
 * modules/audio/dma_controller/audio_dma_buffer.h
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

#ifndef __MODULES_AUDIO_DMA_CONTROLLER_AUDIO_DMA_BUFFER_H
#define __MODULES_AUDIO_DMA_CONTROLLER_AUDIO_DMA_BUFFER_H

#include <arch/chip/audio.h>

#define AS_DMA_I2SO_CH 2

#define AS_DMA_I2SO_SAMPLE_STEP 4
#define AS_DMA_ACI_CH 8
#define AS_DMA_I2SI_CH 2
#define AS_DMA_I2SI2_CH 2
#define AS_DMA_I2SO2_CH 2

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief         Read data from DMA(aci) buffer.
 * @param[in]     sample Number of 16bit samples per channel
 * @param[in]     channel Number of DMA I/O channels
 * @param[in]     dma_addr is the DMA buffer address to be read
 * @param[out]    *p_in_buff points to the input data buffer
 * @return        none
 */
void AS_AudioDrvDmaGetMicInput(uint16_t sample,
                               uint8_t channel,
                               uint32_t dma_addr,
                               void *p_in_buff);

/**
 * @brief         change Lch data and Rch data
 * @param[in]     dma_addr is the DMA buffer address
 * @param[in]     sample is sampling number
 * @return        none
 */
void AS_AudioDrvDmaGetSwapData(uint32_t dma_addr, uint16_t sample);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#endif /* __MODULES_AUDIO_DMA_CONTROLLER_AUDIO_DMA_BUFFER_H */

