/****************************************************************************
 * modules/audio/dma_controller/audio_bb_drv.h
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

#ifndef __MODULES_AUDIO_DMA_CONTROLLER_AUDIO_BB_DRV_H
#define __MODULES_AUDIO_DMA_CONTROLLER_AUDIO_BB_DRV_H

#include "dma_controller/audio_dma_drv_api.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef enum AudioDrvDmaStopMode_
{
  AudioDrvDmaStopNormal = 0,
  AudioDrvDmaStopImmediate,
} AudioDrvDmaStopMode;


typedef struct AudioDrvDmaInitParam_
{
  cxd56_audio_dma_t dmac_id;
  cxd56_audio_samp_fmt_t format;
  AS_ErrorCb        p_error_func;
  AS_DmaDoneCb      p_dmadone_func;
  uint8_t           dma_byte_len;
  uint8_t           ch_num;
  bool              fade_en;
} AudioDrvDmaInitParam;

typedef struct AudioDrvDmaRunParam_
{
  asReadDmacParam run_dmac_param;
  uint32_t     split_addr;
  uint32_t     addr_dest;
  uint16_t     split_size;
  uint8_t      overlap_cnt;
  AS_DmaDoneCb p_dmadone_func;
} AudioDrvDmaRunParam;

typedef struct AudioDrvDmaStopParam_
{
  cxd56_audio_dma_t   dmac_id;
  AudioDrvDmaStopMode stop_mode;
} AudioDrvDmaStopParam;

typedef struct AudioDrvDmaInfo_
{
  cxd56_audio_dma_t dmac_id;
  uint32_t    running_wait;
  uint32_t    running_empty;
  uint32_t    ready_wait;
  uint32_t    ready_empty;
  asDmaState  state;
  E_AS_BB     result;
} AudioDrvDmaInfo;

typedef struct AudioDrvResultParam_
{
  cxd56_audio_dma_t dmac_id;
  E_AS_BB result;
} AudioDrvResultParam;


E_AS_BB AS_AudioDrvDmaInit(AudioDrvDmaInitParam*);
E_AS_BB AS_AudioDrvDmaRun(asReadDmacParam*);
E_AS_BB AS_AudioDrvDmaStop(AudioDrvDmaStopParam*);
E_AS_BB AS_AudioDrvDmaGetInfo(cxd56_audio_dma_t, AudioDrvDmaInfo*);
E_AS_BB AS_AudioDrvDmaStart(cxd56_audio_dma_t);
E_AS_BB AS_AudioDrvDmaNofifyCmplt(cxd56_audio_dma_t, CXD56_AUDIO_ECODE);

E_AS_BB dmaDrvTaskActive(cxd56_audio_dma_t dmacId);
E_AS_BB dmaDrvTaskDeactive(cxd56_audio_dma_t dmacId);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#endif /* __MODULES_AUDIO_DMA_CONTROLLER_AUDIO_BB_DRV_H */
