/****************************************************************************
 * modules/audio/dma_controller/audio_dma_drv_api.cpp
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

#include <sys/types.h>

#include "audio_bb_drv.h"
#include "audio_dma_drv_api.h"
#include "memutils/common_utils/common_assert.h"
#include <debug.h>
#include "debug/dbg_log.h"

static uint16_t dmacMinimumSize[5] = {0};

/*--------------------------------------------------------------------*/
static E_AS initDmac(asInitDmacParam *pInitDmacParam)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;
  AudioDrvDmaInitParam param;

  if (pInitDmacParam->p_dmadone_func != NULL)
    {
      dmacMinimumSize[pInitDmacParam->dmacId] = DMAC_MIN_SIZE_INT;
      param.p_dmadone_func = pInitDmacParam->p_dmadone_func;
    }
  else
    {
      dmacMinimumSize[pInitDmacParam->dmacId] = DMAC_MIN_SIZE_POL;
      param.p_dmadone_func = NULL;
    }

  param.dmac_id      = pInitDmacParam->dmacId;
  param.ch_num       = pInitDmacParam->ch_num;
  param.format       = pInitDmacParam->format;
  param.p_error_func = pInitDmacParam->p_error_func;

  if (pInitDmacParam->format == CXD56_AUDIO_SAMP_FMT_24)
    {
      param.dma_byte_len = AS_DMAC_BYTE_WT_24BIT;
    }
  else
    {
      param.dma_byte_len = AS_DMAC_BYTE_WT_16BIT;
    }

  param.fade_en = pInitDmacParam->fade_en;

  rtCodeBB = AS_AudioDrvDmaInit(&param);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      _err("ERR: dma(%d) er(%d)\n", pInitDmacParam->dmacId, rtCodeBB);
      rtCode = E_AS_DMAC_MSG_SEND_ERR;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_InitDmac(asInitDmacParam *pInitDmacParam)
{
  if (pInitDmacParam == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_INITDMAC_NULL;
    }

  E_AS rtCode = initDmac(pInitDmacParam);

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_StartDmac(cxd56_audio_dma_t dmacId)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;

  rtCodeBB = AS_AudioDrvDmaStart(dmacId);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      return E_AS_DMAC_MSG_SEND_ERR;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_ReadDmac(asReadDmacParam *pReadDmacParam)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;

  if (pReadDmacParam == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_WRITEDMAC_NULL;
    }

  switch (pReadDmacParam->dmacId)
    {
      case CXD56_AUDIO_DMAC_MIC:
      case CXD56_AUDIO_DMAC_I2S0_UP:
      case CXD56_AUDIO_DMAC_I2S1_UP:
          break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
      case CXD56_AUDIO_DMAC_I2S1_DOWN:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          _err("ERR: dma(%d) ID error\n", pReadDmacParam->dmacId);
          return E_AS_DMAC_ID_PARAM;

      default:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          return E_AS_DMAC_ID_PARAM;
    }

  /* Error check */

  if ((pReadDmacParam->size + pReadDmacParam->size2) > DMAC_MAX_SIZE)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size(%d,%d)\n",
             pReadDmacParam->dmacId, pReadDmacParam->size,
             pReadDmacParam->size2);

      return E_AS_DMAC_SIZE_MAX_ERR;
    }

  if (pReadDmacParam->size < dmacMinimumSize[pReadDmacParam->dmacId])
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size(%d) min(%d)\n",
             pReadDmacParam->dmacId, pReadDmacParam->size,
             dmacMinimumSize[pReadDmacParam->dmacId]);

      return E_AS_DMAC_SIZE_MIN_ERR;
    }

  if ((uint32_t*)(pReadDmacParam->addr) == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) addr(0x%lx)\n",
             pReadDmacParam->dmacId, pReadDmacParam->addr);

      return E_AS_DMAC_TRANS_ADDR_NULL;
    }

  if ((0 < pReadDmacParam->size2)
   && (pReadDmacParam->size2 < dmacMinimumSize[pReadDmacParam->dmacId]))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size2(%d) min(%d)\n",
             pReadDmacParam->dmacId, pReadDmacParam->size2,
             dmacMinimumSize[pReadDmacParam->dmacId]);

      return E_AS_DMAC_SIZE_MIN_ERR;
    }

  if ((pReadDmacParam->size2 > 0)
   && ((uint32_t*)(pReadDmacParam->addr2) == NULL))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size2(%d) addr2(0x%lx)\n",
             pReadDmacParam->dmacId, pReadDmacParam->size2,
             pReadDmacParam->addr2);

      return E_AS_DMAC_TRANS_ADDR_NULL;
    }

  rtCodeBB = AS_AudioDrvDmaRun(pReadDmacParam);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      return E_AS_DMAC_MSG_SEND_ERR;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_WriteDmac(asWriteDmacParam *pWriteDmacParam)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;

  if (pWriteDmacParam == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_WRITEDMAC_NULL;
    }

  switch (pWriteDmacParam->dmacId)
    {
      case CXD56_AUDIO_DMAC_MIC:
      case CXD56_AUDIO_DMAC_I2S0_UP:
      case CXD56_AUDIO_DMAC_I2S1_UP:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          _err("ERR: dma(%d) ID error\n", pWriteDmacParam->dmacId);
          return E_AS_DMAC_ID_PARAM;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
      case CXD56_AUDIO_DMAC_I2S1_DOWN:
          break;

      default:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          return E_AS_DMAC_ID_PARAM;
    }

  /* Error check */

  if ((pWriteDmacParam->size + pWriteDmacParam->size2) > DMAC_MAX_SIZE)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size(%d,%d)\n",
             pWriteDmacParam->dmacId, pWriteDmacParam->size,
             pWriteDmacParam->size2);

      return E_AS_DMAC_SIZE_MAX_ERR;
    }

  if (pWriteDmacParam->size < dmacMinimumSize[pWriteDmacParam->dmacId])
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size(%d) min(%d)\n",
             pWriteDmacParam->dmacId, pWriteDmacParam->size,
             dmacMinimumSize[pWriteDmacParam->dmacId]);

      return E_AS_DMAC_SIZE_MIN_ERR;
    }

  if ((uint32_t*)(pWriteDmacParam->addr) == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) addr(0x%lx)\n",
             pWriteDmacParam->dmacId, pWriteDmacParam->addr);

      return E_AS_DMAC_TRANS_ADDR_NULL;
    }

  if ((0 < pWriteDmacParam->size2)
   && (pWriteDmacParam->size2 < dmacMinimumSize[pWriteDmacParam->dmacId]))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size2(%d) min(%d)\n",
             pWriteDmacParam->dmacId, pWriteDmacParam->size2,
             dmacMinimumSize[pWriteDmacParam->dmacId]);

      return E_AS_DMAC_SIZE_MIN_ERR;
    }

  if ((pWriteDmacParam->size2 > 0)
   && ((uint32_t*)(pWriteDmacParam->addr2) == NULL))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size2(%d) addr2(0x%lx)\n",
             pWriteDmacParam->dmacId, pWriteDmacParam->size2,
             pWriteDmacParam->addr2);

      return E_AS_DMAC_TRANS_ADDR_NULL;
    }

  rtCodeBB = AS_AudioDrvDmaRun((asReadDmacParam *)pWriteDmacParam);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      return E_AS_DMAC_MSG_SEND_ERR;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_StopDmac(cxd56_audio_dma_t dmacId, asDmacStopMode stopMode)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;
  AudioDrvDmaStopParam param;

  param.dmac_id   = dmacId;
  param.stop_mode = (AudioDrvDmaStopMode)stopMode;

  rtCodeBB = AS_AudioDrvDmaStop(&param);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      return E_AS_DMAC_MSG_SEND_ERR;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_GetReadyCmdNumDmac(cxd56_audio_dma_t dmacId, uint32_t *pResult)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;
  AudioDrvDmaInfo dmaInfo;

  if (pResult == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_GETREADYCMD_RESULT_NULL;
    }

  rtCodeBB = AS_AudioDrvDmaGetInfo(dmacId, &dmaInfo);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      return E_AS_DMAC_MSG_SEND_ERR;
    }

  *pResult = dmaInfo.ready_empty + dmaInfo.running_empty;

  _info("dma(%d:%d,%ld)\n", dmacId, dmaInfo.state, *pResult);

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_RegistDmaIntCb(cxd56_audio_dma_t dmacId,
                       cxd56_audio_dma_cb_t p_dmaIntCb)
{
  E_AS rtCode = E_AS_OK;
  CXD56_AUDIO_ECODE drv_ret = CXD56_AUDIO_ECODE_OK;

  if (p_dmaIntCb == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_GETREADYCMD_RESULT_NULL;
    }

  drv_ret = cxd56_audio_set_dmacb(dmacId, p_dmaIntCb);
  if (CXD56_AUDIO_ECODE_OK != drv_ret)
    {
      _err("cxd56_audio_set_dmacb() is failer. err = 0x%x\n", drv_ret);
      rtCode = E_AS_DMAC_ID_PARAM;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_NotifyDmaCmplt(cxd56_audio_dma_t dmacId, E_AS_DMA_INT code)
{
  E_AS rtCode = E_AS_OK;

  if (E_AS_BB_DMA_OK != AS_AudioDrvDmaNofifyCmplt(dmacId, code))
    {
      return E_AS_DMAC_ID_PARAM;
    }

  return rtCode;
}

