/****************************************************************************
 * modules/audio/dma_controller/audio_dma_drv.cpp
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

#include <nuttx/kmalloc.h>
#include <debug.h>

#include "memutils/os_utils/chateau_osal.h"
#include "audio/audio_high_level_api.h"
#include "debug/dbg_log.h"
#include "audio_dma_drv.h"
#include "audio_dma_buffer.h"

/* Currently, a timing to start fade is when set setting of previous frame.
 * Because, the volume will going down about 6db/ms, therefore after about
 * 5ms, the volume is very small which is hard to hear.
 * However, if you'd like to take fade time strictly,
 * enable "FADECTRL_BY_FADETERM".
 */
/* #define FADECTRL_BY_FADETERM */

#define CHECK_DMA_CH_SHIFT

#define FADE_QUEUE_COUNT 2
#define STOP_QUEUE_COUNT 1
#define LAST_QUEUE_COUNT 1

#define DMA_BUFFER_MAX_SIZE 1024
#define DMA_BUFFER_POOL_SEG_SIZE 0x00002000

#define TIMEOUT_CNT 10000    /* >160MHz/48kHz */
#define RETRY_CNT 10

extern "C" uint32_t cxd56_get_cpu_baseclk(void);

#ifdef CONFIG_AUDIOUTILS_RENDERER_UNDERFLOW
/* Underflow insertion data
 *  (saize: sample[UNDERFLOW_DATA_SAMPLE] * bitleng[4] * channel[2])
 */

static char under_data[UNDERFLOW_INSERT_SAMPLE * AS_DMAC_BYTE_WT_24BIT * AS_DMA_I2SO_CH] = {0};
#endif  /* CONFIG_AUDIOUTILS_RENDERER_UNDERFLOW */

AsDmaDrv::dmaDrvFuncTbl AsDmaDrv::m_func_tbl[] =
{
  {
    EvtInit,

    {                           /* DmaController status:  */
      &AsDmaDrv::init,          /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::init,          /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::illegal,       /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::illegal,       /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::illegal,       /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::illegal,       /*   AS_DMA_STATE_FLUSH   */
      &AsDmaDrv::illegal,       /*   AS_DMA_STATE_ERROR   */
      &AsDmaDrv::init           /*   AS_DMA_STATE_TERMINATE */
    }
  },

  {
    EvtRun,

    {                             /* DmaController status:  */
      &AsDmaDrv::illegal,         /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::runDmaOnStop,    /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::runDmaOnReady,   /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::runDmaOnPrepare, /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::runDma,          /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::illegal,         /*   AS_DMA_STATE_FLUSH   */
      &AsDmaDrv::illegal,         /*   AS_DMA_STATE_ERROR   */
      &AsDmaDrv::illegal          /*   AS_DMA_STATE_TERMINATE */
    }
  },

  {
    EvtStop,

    {                         /* DmaController status:  */
      &AsDmaDrv::illegal,     /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::ignore,      /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::stop,        /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::stop,        /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::stopOnRun,   /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::illegal,     /*   AS_DMA_STATE_FLUSH   */
      &AsDmaDrv::ignore,      /*   AS_DMA_STATE_ERROR   */
      &AsDmaDrv::stop         /*   AS_DMA_STATE_TERMINATE */
    }
  },

  {
    EvtCmplt,

    {                             /* DmaController status:  */
      &AsDmaDrv::illegalDmaCmplt, /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::illegalDmaCmplt, /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::illegalDmaCmplt, /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::illegalDmaCmplt, /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::dmaCmpltOnRun,   /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::dmaCmpltOnFlush, /*   AS_DMA_STATE_FLUSH   */
      &AsDmaDrv::dmaCmpltOnError, /*   AS_DMA_STATE_ERROR   */
      &AsDmaDrv::illegalDmaCmplt  /*   AS_DMA_STATE_TERMINATE */
    }
  },

  {
    EvtGetInfo,

    {                            /* DmaController status:  */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::getInfo,        /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::getInfo,        /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::getInfo,        /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::getInfo,        /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::getInfo,        /*   AS_DMA_STATE_FLUSH   */
      &AsDmaDrv::getInfo,        /*   AS_DMA_STATE_ERROR   */
      &AsDmaDrv::getInfo         /*   AS_DMA_STATE_TERMINATE */
    }
  },

  {
    EvtDmaErr,

    {                            /* DmaController status:  */
      &AsDmaDrv::dmaErrInt,      /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::dmaErrInt,      /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::dmaErrInt,      /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::dmaErrInt,      /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::dmaErrIntOnRun, /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::dmaErrInt,      /*   AS_DMA_STATE_FLUSH   */
      &AsDmaDrv::dmaErrInt,      /*   AS_DMA_STATE_ERROR   */
      &AsDmaDrv::dmaErrInt       /*   AS_DMA_STATE_TERMINATE */
    }
  },

  {
    EvtBusErr,

    {                            /* DmaController status:  */
      &AsDmaDrv::dmaErrBus,      /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::dmaErrBus,      /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::dmaErrBus,      /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::dmaErrBus,      /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::dmaErrBusOnRun, /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::dmaErrBus,      /*   AS_DMA_STATE_FLUSH   */
      &AsDmaDrv::dmaErrBus,      /*   AS_DMA_STATE_ERROR   */
      &AsDmaDrv::dmaErrBus       /*   AS_DMA_STATE_TERMINATE */
    }
  },

  {
    EvtStart,

    {                            /* DmaController status:  */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::startDma,       /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_FLUSH   */
      &AsDmaDrv::ignore,         /*   AS_DMA_STATE_ERROR   */
      &AsDmaDrv::ignore          /*   AS_DMA_STATE_TERMINATE */
    }
  }
};

uint32_t AsDmaDrv::m_funcTblNum = sizeof(m_func_tbl) / sizeof(m_func_tbl[0]);

/*--------------------------------------------------------------------*/
void AsDmaDrv::readyQuePush(const AudioDrvDmaRunParam &dmaParam)
{
  if (!m_ready_que.push(dmaParam))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
    }
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::readyQuePop()
{
  if (!m_ready_que.pop())
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
    }
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::runningQuePush(const AudioDrvDmaRunParam &dmaParam)
{
  if (!m_running_que.push(dmaParam))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
    }
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::runningQuePop()
{
  if (!m_running_que.pop())
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
    }
}

/*--------------------------------------------------------------------*/
AsDmaDrv::dmaDrvFuncTbl* AsDmaDrv::searchFuncTbl(ExternalEvent event)
{
  dmaDrvFuncTbl *p_tbl = NULL;

  for (uint32_t i=0 ; i<m_funcTblNum ; i++)
    {
      if ((m_func_tbl + i)->event == event)
        {
          p_tbl = m_func_tbl + i;
          break;
        }
    }

  F_ASSERT(p_tbl);

  return p_tbl;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::parse(ExternalEvent event, void *p_param)
{
  dmaDrvFuncTbl *p_tbl = searchFuncTbl(event);

  return (this->*(p_tbl->p_func[m_state.get()]))(p_param);
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::illegal(void *p_param)
{
  DMAC_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);

  return false;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::ignore(void *p_param)
{
  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::init(void *p_param)
{
  CXD56_AUDIO_ECODE drv_ret = CXD56_AUDIO_ECODE_OK;
  AudioDrvDmaInitParam *initParam =
    reinterpret_cast<AudioDrvDmaInitParam*>(p_param);

  m_ready_que.clear();
  m_running_que.clear();
  m_error_func = initParam->p_error_func;
  m_dma_byte_len = initParam->dma_byte_len;
  m_ch_num = initParam->ch_num;
  m_dmadone_func = initParam->p_dmadone_func;

  drv_ret = cxd56_audio_init_dma(initParam->dmac_id,
                                 initParam->format,
                                 &m_ch_num);
  if (CXD56_AUDIO_ECODE_OK != drv_ret)
    {
      _err("AsDmaDrv::init cxd56_audio_init_dma() failer. err = 0x%x\n",
            drv_ret);
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return false;
    }

  if (m_dmadone_func == NULL)
    {
      m_min_size = DMAC_MIN_SIZE_POL;
    }
  else
    {
      m_min_size = DMAC_MIN_SIZE_INT;
    }

  if (!m_level_ctrl.init(initParam->dmac_id, true, true))
    {
      return false;
    }

  drv_ret = cxd56_audio_en_dmaint();
  if (CXD56_AUDIO_ECODE_OK != drv_ret)
    {
      _err("AsDmaDrv::init cxd56_audio_en_dmaint() failer. err = 0x%x\n",
            drv_ret);
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return false;
    }

  m_state = AS_DMA_STATE_READY;

  _info("READY(%d)\n", initParam->dmac_id);

  return true;
}

/*--------------------------------------------------------------------*/
E_AS AsDmaDrv::setDmaCmd(cxd56_audio_dma_t dmac_id,
                         uint32_t addr,
                         uint32_t size,
                         bool nointr,
                         bool dma_run_1st)
{
  E_AS rtCode = E_AS_OK;
  CXD56_AUDIO_ECODE drv_ret = CXD56_AUDIO_ECODE_OK;
#ifndef CHECK_DMA_CH_SHIFT
  dma_run_1st = false;
#endif

  drv_ret = cxd56_audio_start_dma(dmac_id, addr, size);

  if (drv_ret == CXD56_AUDIO_ECODE_DMA_BUSY)
    {
      rtCode = E_AS_DMAC_BUSY;
      dmaErrCb(E_AS_BB_DMA_ERR_REQUEST);
    }
  else if (drv_ret != CXD56_AUDIO_ECODE_OK)
    {
      rtCode = E_AS_DMAC_ERR_START;
      dmaErrCb(E_AS_BB_DMA_ERR_START);
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::runDmaSplitRequest(AudioDrvDmaRunParam *pDmaParam,
                                  uint32_t addr,
                                  uint16_t size)
{
  uint32_t sizeCalc = 0;

  sizeCalc = size;

  while (sizeCalc != 0)
    {
      if ((m_dmac_id == CXD56_AUDIO_DMAC_MIC)
       && ((m_ch_num % 2) == 1)
       && (m_dma_byte_len == AS_DMAC_BYTE_WT_16BIT))
        {
          pDmaParam->split_addr = (uint32_t)m_dma_buffer[m_dma_buf_cnt];

          if (m_dma_buf_cnt == 0)
            {
              m_dma_buf_cnt = 1;
            }
          else
            {
              m_dma_buf_cnt = 0;
            }

          pDmaParam->addr_dest =
            addr + ((size - sizeCalc) * m_ch_num * m_dma_byte_len);
        }
      else
        {
          pDmaParam->split_addr =
            addr + ((size - sizeCalc) * m_ch_num * m_dma_byte_len);
        }

      /* If sample num is over 1024(DMA_BUFFER_MAX_SIZE), transfer unit is
       * divided into every 1024 samples. However, the sample num is.
       * 1024 < x <= 2048, transfer unit will be half of it. This is to
       * prevent too fewer transfer unit will be.
       */

      if (sizeCalc > DMA_BUFFER_MAX_SIZE * 2)
        {
          pDmaParam->split_size = DMA_BUFFER_MAX_SIZE;
        }
      else if ((DMA_BUFFER_MAX_SIZE < sizeCalc)
            && (sizeCalc <= DMA_BUFFER_MAX_SIZE * 2))
        {
          pDmaParam->split_size = sizeCalc / 2;
        }
      else
        {
          pDmaParam->split_size = sizeCalc;
        }

      sizeCalc -= pDmaParam->split_size;
      pDmaParam->overlap_cnt -= 1;

      readyQuePush(*pDmaParam);
    }
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::runDmaOnPrepare(void *p_param)
{
  return pushRequest(p_param, true);
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::pushRequest(void *p_param, bool use_callback)
{
  AudioDrvDmaRunParam dmaParam;

  dmaParam.run_dmac_param = *(reinterpret_cast<asReadDmacParam*>(p_param));
  if (use_callback)
    {
      dmaParam.p_dmadone_func = m_dmadone_func;
    }
  else
    {
      dmaParam.p_dmadone_func = NULL;
    }

  uint32_t size1_cnt = 0;
  uint32_t size2_cnt = 0;

  if ((cxd56_audio_get_dmafmt() == CXD56_AUDIO_DMA_FMT_RL)
   && (m_dma_byte_len == AS_DMAC_BYTE_WT_16BIT))
    {
      switch (m_dmac_id)
        {
          case CXD56_AUDIO_DMAC_I2S0_DOWN:
          case CXD56_AUDIO_DMAC_I2S1_DOWN:
              AS_AudioDrvDmaGetSwapData(dmaParam.run_dmac_param.addr,
                                        dmaParam.run_dmac_param.size);

              if (dmaParam.run_dmac_param.size2 != 0)
                {
                  AS_AudioDrvDmaGetSwapData(dmaParam.run_dmac_param.addr2,
                                            dmaParam.run_dmac_param.size2);
                }
              break;

          default:
              break;
        }
    }

  /* Process of DMA request */

  size1_cnt = dmaParam.run_dmac_param.size / DMA_BUFFER_MAX_SIZE;
  if (dmaParam.run_dmac_param.size % DMA_BUFFER_MAX_SIZE)
    {
      size1_cnt += 1;
    }

  size2_cnt = dmaParam.run_dmac_param.size2 / DMA_BUFFER_MAX_SIZE;
  if (dmaParam.run_dmac_param.size2 % DMA_BUFFER_MAX_SIZE)
    {
      size2_cnt += 1;
    }

  dmaParam.overlap_cnt = size1_cnt + size2_cnt;

  if ((m_ready_que.size() + dmaParam.overlap_cnt) > READY_QUEUE_NUM)
    {
      _info("OVERFLOW(%d) rdy(%d)\n", m_dmac_id, m_ready_que.size());
      dmaErrCb(E_AS_BB_DMA_OVERFLOW);
    }
  else
    {
      runDmaSplitRequest(&dmaParam,
                         dmaParam.run_dmac_param.addr,
                         dmaParam.run_dmac_param.size);

      runDmaSplitRequest(&dmaParam,
                         dmaParam.run_dmac_param.addr2,
                         dmaParam.run_dmac_param.size2);
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::runDmaOnStop(void *p_param)
{
  m_ready_que.clear();
  m_running_que.clear();

  pushRequest(p_param, true);

  m_state = AS_DMA_STATE_PREPARE;

  _info("PREPARE(%d)\n", m_dmac_id);

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::runDmaOnReady(void *p_param)
{
  if (m_level_ctrl.setFadeRamp(&m_fade_required_sample) != true)
    {
      return false;
    }

  pushRequest(p_param, true);

  m_state = AS_DMA_STATE_PREPARE;

  _info("PREPARE(%d)\n", m_dmac_id);

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::runDma(void *p_param)
{
  pushRequest(p_param, true);

  while ((m_ready_que.size() > 0)
      && (m_running_que.size() < RUNNING_QUEUE_NUM))
    {
      const AudioDrvDmaRunParam& reqParam = m_ready_que.top();

      runningQuePush(reqParam);

      setDmaCmd(reqParam.run_dmac_param.dmacId,
                reqParam.split_addr,
                reqParam.split_size,
                false,
                false);

      readyQuePop();
    }

  /* Fade control. */

  fadeControl();

  return true;
}


/*--------------------------------------------------------------------*/
bool AsDmaDrv::startDma(void *p_param)
{
  bool run_1st = true;
  E_AS rtCode = E_AS_OK;
  m_dmac_id = *(reinterpret_cast<cxd56_audio_dma_t*>(p_param));

  /* Move request from ready queue to running queue (= DMA transfer queue). */

  while ((m_ready_que.size() > 0)
      && (m_running_que.size() < RUNNING_QUEUE_NUM))
    {
      const AudioDrvDmaRunParam& reqParam = m_ready_que.top();

      runningQuePush(reqParam);

      rtCode = setDmaCmd(reqParam.run_dmac_param.dmacId,
                         reqParam.split_addr,
                         reqParam.split_size,
                         false,
                         run_1st);

      run_1st = false;

      readyQuePop();

      if (rtCode == E_AS_DMAC_ERR_START)
        {
          break;
        }
    }

  if (rtCode != E_AS_DMAC_ERR_START)
    {
      cxd56_audio_clear_dmaerrint(m_dmac_id);

      cxd56_audio_unmask_dmaerrint(m_dmac_id);

      m_state = AS_DMA_STATE_RUN;

      _info("RUN(%d)\n", m_dmac_id);
    }

  /* Fade control */

  fadeControl();

  return true;
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::dmaCmplt(void)
{
  const AudioDrvDmaRunParam& dmaParam = m_running_que.top();

  if (((m_ch_num % 2) == 1) && (m_dma_byte_len == AS_DMAC_BYTE_WT_16BIT))
    {
      AS_AudioDrvDmaGetMicInput(dmaParam.split_size,
                                m_ch_num,
                                dmaParam.split_addr,
                                (void *)dmaParam.addr_dest);
    }

  if ((cxd56_audio_get_dmafmt() == CXD56_AUDIO_DMA_FMT_RL)
      && (m_dma_byte_len == AS_DMAC_BYTE_WT_16BIT) )
    {
      switch (m_dmac_id)
        {
          case CXD56_AUDIO_DMAC_I2S0_UP:
          case CXD56_AUDIO_DMAC_I2S1_UP:
              AS_AudioDrvDmaGetSwapData(dmaParam.split_addr,
                                        dmaParam.split_size);
              break;

          default:
              break;
        }
    }

  if (dmaParam.overlap_cnt == 0)
    {
      if (dmaParam.p_dmadone_func != NULL)
        {
          AudioDrvDmaResult resultParam;

          resultParam.result  = E_AS_BB_DMA_OK;
          resultParam.dmac_id = dmaParam.run_dmac_param.dmacId;
          resultParam.addr1   = dmaParam.run_dmac_param.addr;
          resultParam.size1   = dmaParam.run_dmac_param.size;
          resultParam.addr2   = dmaParam.run_dmac_param.addr2;
          resultParam.size2   = dmaParam.run_dmac_param.size2;
          resultParam.endflg  = false;

          if (m_state == AS_DMA_STATE_FLUSH)
            {
              if ((m_running_que.size() <= LAST_QUEUE_COUNT)
               && (m_ready_que.size() == 0))
                {
                  resultParam.endflg  = true;
                }
            }

          if (m_state == AS_DMA_STATE_ERROR)
            {
              resultParam.endflg  = true;
            }

          (*m_dmadone_func)(&resultParam);
        }
    }

  runningQuePop();

  while ((m_ready_que.size() > 0)
      && (m_running_que.size() < RUNNING_QUEUE_NUM))
    {
      const AudioDrvDmaRunParam& reqParam = m_ready_que.top();

      runningQuePush(reqParam);

      setDmaCmd(reqParam.run_dmac_param.dmacId,
                reqParam.split_addr,
                reqParam.split_size,
                false,
                false);

      readyQuePop();
    }
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaCmpltOnRun(void *p_param)
{
  dmaCmplt();
  fadeControl();

#ifdef CONFIG_AUDIOUTILS_RENDERER_UNDERFLOW
  if ((m_running_que.size() == 1) &&
      (m_dmac_id == CXD56_AUDIO_DMAC_I2S0_DOWN ||
       m_dmac_id == CXD56_AUDIO_DMAC_I2S1_DOWN))
    {
      asWriteDmacParam dmac_param;
      dmac_param.dmacId = m_dmac_id;
      dmac_param.addr = (uint32_t)&under_data[0];
      dmac_param.size = UNDERFLOW_INSERT_SAMPLE;
      dmac_param.addr2 = 0;
      dmac_param.size2 = 0;
      dmac_param.validity = false;
      pushRequest((void*)&dmac_param, false);

      const AudioDrvDmaRunParam& reqParam = m_ready_que.top();
      runningQuePush(reqParam);

      setDmaCmd(reqParam.run_dmac_param.dmacId,
                    reqParam.split_addr,
                    reqParam.split_size,
                    false,
                    false);

      readyQuePop();

      dmaErrCb(E_AS_BB_DMA_UNDERFLOW);
    }
#endif  /* CONFIG_AUDIOUTILS_RENDERER_UNDERFLOW */

  if (m_running_que.size() <= 1)
    {
      cxd56_audio_stop_dma(m_dmac_id);

      dmaErrCb(E_AS_BB_DMA_UNDERFLOW);

      m_state = AS_DMA_STATE_ERROR;

      _info("PREPARE(%d)\n", m_dmac_id);
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaCmpltOnFlush(void *p_param)
{
  dmaCmplt();

  if (((m_ready_que.size() + m_running_que.size()) <  FADE_QUEUE_COUNT))
    {
      volumeCtrl(true, true);
    }
  else
    {
      fadeControl();
    }

  if (m_running_que.size() == STOP_QUEUE_COUNT)
    {
      cxd56_audio_stop_dma(m_dmac_id);
    }
  else if (m_running_que.size() == 0 )
    {
      m_state = AS_DMA_STATE_STOP;
      _info("STOP(%d)\n", m_dmac_id);
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaCmpltOnError(void *p_param)
{
  dmaCmplt();

  /* Current state is ERROR, it means this cmplt is last one
   * Because DMA was already stopped when error has occured.
   * Therefore, change state to STOP.
   */

  m_state = AS_DMA_STATE_TERMINATE;

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::illegalDmaCmplt(void *p_param)
{
  DMAC_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);

  /* Even if illegal completion, but need to reply to requester as usual.
   * Because, DMA completion means that the transfer request have came.
   */

  dmaCmplt();

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::stop(void *p_param)
{
  AudioDrvDmaStopParam *stopParam =
    reinterpret_cast<AudioDrvDmaStopParam*>(p_param);

  /* Populate requst of DMA stop */

  m_ready_que.clear();

  m_state = AS_DMA_STATE_STOP;

  _info("STOP(%d)\n", stopParam->dmac_id);

  m_dmac_id = stopParam->dmac_id;

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::stopOnRun(void *p_param)
{
  AudioDrvDmaStopParam *stopParam =
    reinterpret_cast<AudioDrvDmaStopParam*>(p_param);

  /* Populate requst of DMA stop */

  if (stopParam->stop_mode == AudioDrvDmaStopImmediate)
    {
      m_ready_que.clear();
    }

  if (((m_ready_que.size() + m_running_que.size()) <  FADE_QUEUE_COUNT))
    {
      volumeCtrl(true, true);
    }

  if ((m_ready_que.size() + m_running_que.size()) <=  STOP_QUEUE_COUNT)
    {
      cxd56_audio_stop_dma(m_dmac_id);
    }

  m_state = AS_DMA_STATE_FLUSH;

  _info("FLUSH(%d)\n", stopParam->dmac_id);

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaErrInt(void *p_param)
{
  cxd56_audio_stop_dma(m_dmac_id);

  dmaErrCb(E_AS_BB_DMA_ERR_INT);

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaErrIntOnRun(void *p_param)
{
  dmaErrInt(p_param);

  m_state = AS_DMA_STATE_TERMINATE;

  return true;
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::dmaErrCb(E_AS_BB err_code)
{
  /* If dma error, force mute. */

  volumeCtrl(false, false);

  if (m_error_func != NULL)
    {
      AudioDrvDmaError errorParam;

      errorParam.dmac_id = m_dmac_id;
      errorParam.status  = err_code;
      errorParam.state   = m_state.get();
#ifdef CONFIG_AUDIOUTILS_RENDERER_UNDERFLOW
    if (m_dmac_id != CXD56_AUDIO_DMAC_I2S0_DOWN &&
        m_dmac_id != CXD56_AUDIO_DMAC_I2S1_DOWN)
      {
        (*m_error_func)(&errorParam);
      }
    else if (err_code != E_AS_BB_DMA_UNDERFLOW)
      {
        (*m_error_func)(&errorParam);
      }
#else
      (*m_error_func)(&errorParam);
#endif
    }

  switch (err_code)
    {
      case E_AS_BB_DMA_UNDERFLOW:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_DMA_UNDERFLOW);
          break;

      case E_AS_BB_DMA_OVERFLOW:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_DMA_OVERFLOW);
          break;

      case E_AS_BB_DMA_ILLEGAL:
      case E_AS_BB_DMA_ERR_INT:
      case E_AS_BB_DMA_PARAM:
      case E_AS_BB_DMA_ERR_START:
      case E_AS_BB_DMA_ERR_REQUEST:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_DMA_ERROR);
          break;

      case E_AS_BB_DMA_ERR_BUS:
          DMAC_FATAL(AS_ATTENTION_SUB_CODE_DMA_ERROR);
          break;

      default:
          break;
    }
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaErrBus(void *p_param)
{
  dmaErrCb(E_AS_BB_DMA_ERR_BUS);

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaErrBusOnRun(void *p_param)
{
  dmaErrBus(p_param);

  m_state = AS_DMA_STATE_PREPARE;

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::getInfo(void *p_param)
{
  AudioDrvDmaInfo *dmaInfo = reinterpret_cast<AudioDrvDmaInfo*>(p_param);

  dmaInfo->result        = E_AS_BB_DMA_OK;
  dmaInfo->dmac_id       = m_dmac_id;
  dmaInfo->running_wait  = m_running_que.size();
  dmaInfo->running_empty = RUNNING_QUEUE_NUM - dmaInfo->running_wait;
  dmaInfo->ready_wait    = m_ready_que.size();
  dmaInfo->ready_empty   = READY_QUEUE_NUM - dmaInfo->ready_wait;
  dmaInfo->state         = m_state.get();

  return true;
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::allocDmaBuffer(cxd56_audio_dma_t dmac_id)
{
  if(dmac_id == CXD56_AUDIO_DMAC_MIC)
    {
      m_dma_buffer[0] = (FAR uint32_t *)kmm_malloc(DMA_BUFFER_POOL_SEG_SIZE);

      if (!m_dma_buffer[0])
        {
          F_ASSERT(0);
        }

      m_dma_buffer[1] = (FAR uint32_t *)kmm_malloc(DMA_BUFFER_POOL_SEG_SIZE);

      if (!m_dma_buffer[1])
        {
          F_ASSERT(0);
        }
    }
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::freeDmaBuffer(cxd56_audio_dma_t dmac_id)
{
  if(dmac_id == CXD56_AUDIO_DMAC_MIC)
    {
      kmm_free(m_dma_buffer[0]);
      kmm_free(m_dma_buffer[1]);
    }
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::fadeControl(void)
{
  /* Check frame validity within one transfer frame from current frame. */
  /* Is there a invalid frame within one frame (include current frame), */
  /* mute request will be publised.                                     */

  AudioDrvDmaRunParam queParam;
  uint8_t que_stored_num = m_running_que.size() + m_ready_que.size();
  bool validity = false;
#ifdef FADECTRL_BY_FADETERM
  uint32_t samples_to_transfer = 0;
#endif /* FADECTRL_BY_FADETERM */
  uint8_t cnt = 0;

  for (cnt = 0; cnt < que_stored_num; cnt++)
    {
      if (cnt < m_running_que.size())
        {
          queParam = m_running_que.at(cnt);
        }
      else
        {
          queParam = m_ready_que.at(cnt - m_running_que.size());
        }

      if (!queParam.run_dmac_param.validity)
        {
          validity = false;
          break;
        }
      else
        {
#ifdef FADECTRL_BY_FADETERM
          if (samples_to_transfer >= m_fade_required_sample)
#endif /* FADECTRL_BY_FADETERM */
          if (cnt >= 1)
            {
              validity = true;
              break;
            }
        }

#ifdef FADECTRL_BY_FADETERM
      samples_to_transfer += queParam.split_size;
#endif /* FADECTRL_BY_FADETERM */
    }

  /* If valid frame is not exist within term,
   * next frame is assume to invalide frame.
   */

  volumeCtrl(validity, false);
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::volumeCtrl(bool validity, bool is_last_frame)
{
  /* Mute/unmute a.s.a.p, therefore do not send inter task message. */

  LevelCtrl::LevelCtrlCmd cmd = LevelCtrl::CmdMuteOff;

  /* Check auto fade */

  if (m_level_ctrl.getAutoFade())
    {
      if(is_last_frame)
        {
          cmd = LevelCtrl::CmdMuteOn;
        }
    }

  /* Check validity of data */

  if (!validity)
    {
      cmd = LevelCtrl::CmdMuteOn;
    }

  m_level_ctrl.exec(cmd, false);
}
