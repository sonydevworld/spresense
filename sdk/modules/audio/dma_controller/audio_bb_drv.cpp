/****************************************************************************
 * modules/audio/dma_controller/audio_bb_drv.cpp
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

/****************************************************************************
    Include
 ****************************************************************************/

#include "audio_bb_drv.h"
#include "audio_dma_drv.h"

#include <debug.h>

/****************************************************************************
    Definitions
 ****************************************************************************/

#define DMA_INST_NUM  (CXD56_AUDIO_DMAC_I2S1_DOWN + 1)

/****************************************************************************/
extern "C" {

static AsDmaDrv *s_dma_drv_instance[DMA_INST_NUM];

/****************************************************************************/
E_AS dmaDrvActive(cxd56_audio_dma_t dmacId)
{
  s_dma_drv_instance[dmacId] = new AsDmaDrv(dmacId);

  if (s_dma_drv_instance[dmacId] == NULL)
    {
      return E_AS_DMAC_ACTIVATED_ERR;
    }

  return E_AS_OK;
}


/****************************************************************************/
E_AS dmaDrvDeactive(cxd56_audio_dma_t dmacId)
{
  if (s_dma_drv_instance[dmacId] == NULL)
    {
      return E_AS_DMAC_DEACTIVATED_ERR;
    }

  delete s_dma_drv_instance[dmacId];

  return E_AS_OK;
}

/****************************************************************************/

static bool activateDmac[DMA_INST_NUM] =
{
  false, false, false, false, false
};

/*--------------------------------------------------------------------*/
E_AS AS_ActivateDmac(cxd56_audio_dma_t dmacId)
{
  E_AS rtCode = E_AS_OK;

  /* Get resource for using dmac. */

  if (!activateDmac[dmacId])
    {
      rtCode = dmaDrvActive(dmacId);

      if(rtCode == E_AS_OK)
        {
          activateDmac[dmacId] = true;
        }
    }
  else
    {
      rtCode = E_AS_DMAC_ACTIVATED_ERR;
    }

  _info("dma(%d) er(0x%x)\n", dmacId, rtCode);

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_DeactivateDmac(cxd56_audio_dma_t dmacId)
{
  E_AS rtCode = E_AS_OK;

  /* Release resource for using dmac. */

  if (activateDmac[dmacId])
    {
      rtCode = dmaDrvDeactive(dmacId);

      if(rtCode == E_AS_OK)
        {
          activateDmac[dmacId] = false;
        }
    }
  else
    {
      rtCode = E_AS_DMAC_DEACTIVATED_ERR;
    }

  _info("dma(%d) er(0x%x)\n", dmacId, rtCode);

  return rtCode;
}

} /* extern "C" */

/****************************************************************************/
/* API      */

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaInit(AudioDrvDmaInitParam *pParam)
{
  if (s_dma_drv_instance[pParam->dmac_id]->parse(AsDmaDrv::EvtInit,
                                                 (void *)pParam))
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaRun(asReadDmacParam *pParam)
{
  if (s_dma_drv_instance[pParam->dmacId]->parse(AsDmaDrv::EvtRun,
                                                (void *)pParam))
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaStart(cxd56_audio_dma_t dmac_id)
{
  if (s_dma_drv_instance[dmac_id]->parse(AsDmaDrv::EvtStart,
                                         (void *)&dmac_id))
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaStop(AudioDrvDmaStopParam *pParam)
{
  if (s_dma_drv_instance[pParam->dmac_id]->parse(AsDmaDrv::EvtStop,
                                                 (void *)pParam))
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaGetInfo(cxd56_audio_dma_t dmac_id,
                                      FAR AudioDrvDmaInfo *pDmaInfo)
{
  if (s_dma_drv_instance[dmac_id]->parse(AsDmaDrv::EvtGetInfo,
                                         (void *)pDmaInfo))
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaNofifyCmplt(cxd56_audio_dma_t dmacId, CXD56_AUDIO_ECODE code)
{
  bool result = false;

  /* Notify dma excution result. */

  switch(code)
    {
      case E_AS_DMA_INT_CMPLT:
        result = s_dma_drv_instance[dmacId]->parse(AsDmaDrv::EvtCmplt, NULL);
        break;

      case E_AS_DMA_INT_ERR:
        result = s_dma_drv_instance[dmacId]->parse(AsDmaDrv::EvtDmaErr, NULL);
        break;

      case E_AS_DMA_INT_ERR_BUS:
        result = s_dma_drv_instance[dmacId]->parse(AsDmaDrv::EvtBusErr, NULL);
        break;

      default:
        break;
    }

  if (result)
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

