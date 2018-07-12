/****************************************************************************
 * modules/audio/dma_controller/audio_dma_buffer.cpp
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

#include "audio_dma_buffer.h"

/*--------------------------------------------------------------------*/
/* AS_AudioDrvDmaGetMicInput is used
 * to skip invalid data when 16bit odd channel DMA.
 */

void AS_AudioDrvDmaGetMicInput(uint16_t sample,
                               uint8_t channel,
                               uint32_t dma_addr,
                               void *p_in_buff)
{
  uint32_t i = 0;
  uint8_t ch = 0;
  uint16_t *p_from = (uint16_t *)dma_addr;
  uint16_t *p_to = (uint16_t *)p_in_buff;

  for (i = 0; i < sample; i++)
    {
      for (ch = 0; ch < channel; ch++)
        {
          *p_to = *p_from;
          p_to++;
          p_from++;
        }

      p_from++;
    }
}

/*--------------------------------------------------------------------*/
/* AS_AudioDrvDmaGetSwapData is used
 * to swap Lch/Rch data when 16bit DMA.
 */

void AS_AudioDrvDmaGetSwapData(uint32_t dma_addr, uint16_t sample)
{
  uint32_t i = 0;
  uint16_t tmp_buffer;
  uint16_t *p_lch = (uint16_t *)dma_addr;
  uint16_t *p_rch = p_lch + 1;

  for (i = 0; i < sample; i++)
    {
      tmp_buffer = *p_lch;
      *p_lch = *p_rch;
      *p_rch = tmp_buffer;

      p_lch += 2;
      p_rch += 2;
    }
}

