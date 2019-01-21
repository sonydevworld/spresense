/****************************************************************************
 * modules/audio/dma_controller/audio_dma_drv.h
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

#ifndef __MODULES_AUDIO_DMA_CONTROLLER_AUDIO_DMA_DRV_H
#define __MODULES_AUDIO_DMA_CONTROLLER_AUDIO_DMA_DRV_H

#include "dma_controller/audio_dma_drv_api.h"
#include "dma_controller/audio_bb_drv.h"
#include "memutils/s_stl/queue.h"
#include "memutils/s_stl/s_stl_config.h"
#include "dma_controller/level_ctrl.h"
#include "audio_state.h"
#include <debug.h>

__USING_S_STL;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Activate DMAC
 *
 * @param[in] cxd56_audio_dma_t DMAC ID
 *
 * @retval E_AS return code
 */
E_AS AS_ActivateDmac(cxd56_audio_dma_t dmacId);

/**
 * @brief Deactivate DMAC
 *
 * @param[in] cxd56_audio_dma_t DMAC ID
 *
 * @retval E_AS return code
 */
E_AS AS_DeactivateDmac(cxd56_audio_dma_t dmacId);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#ifdef CONFIG_AUDIOUTILS_RENDERER_UNDERFLOW
/* Number of silent insertion samples at dmac underflow */

# define UNDERFLOW_INSERT_SAMPLE 1024  /* 21ms */
#endif  /* CONFIG_AUDIOUTILS_RENDERER_UNDERFLOW */

/* Definition of size of DMA ReadyQueue.
 * The number of queues is a value obtained by multiplying
 * the maximum number of segments by the number of forwarding of DMA.
 * Assume that the number of Segment is 10 maximum.
 * The maximum number of DMA transfers in 1 segment is 3.
 * (192000/882000 * 1024sample / 1024(DMA MAX) = 2.17)
 */

#define READY_QUEUE_NUM 30
#define RUNNING_QUEUE_NUM 2
#define PREPARE_SAVE_NUM RUNNING_QUEUE_NUM

typedef bool (* AS_DmaDrvFunc)(void*);

class AsDmaDrv
{
public:
  enum ExternalEvent
  {
    EvtInit = 0,
    EvtRun,
    EvtStop,
    EvtCmplt,
    EvtGetInfo,
    EvtDmaErr,
    EvtBusErr,
    EvtStart,
    ExternalEventNum
  };

  AsDmaDrv(cxd56_audio_dma_t dmac_id)
      : m_dmac_id(dmac_id)
      , m_state(AS_MODULE_ID_AUDIO_DRIVER, "", AS_DMA_STATE_BOOTED)
      , m_error_func(dma_err_callback)
      , m_dma_buf_cnt(0)
      , m_min_size(0)
      , m_fade_required_sample(0)
  {
    m_ready_que.clear();
    m_running_que.clear();
    allocDmaBuffer(dmac_id);
  }

  virtual ~AsDmaDrv()
  {
    freeDmaBuffer(m_dmac_id);
  }

  void run(void);
  bool parse(ExternalEvent event, void *p_param);

  LevelCtrl m_level_ctrl;

private:
  typedef bool (AsDmaDrv::*AS_DmaDrvFunc)(void *);

  struct dmaDrvFuncTbl
  {
    ExternalEvent event;
    AS_DmaDrvFunc p_func[AS_DMA_STATE_MAX_ENTRY];
  };

  cxd56_audio_dma_t m_dmac_id;

  AudioState<asDmaState>  m_state;

  AS_ErrorCb   m_error_func;
  AS_DmaDoneCb m_dmadone_func;

  uint8_t     m_dma_byte_len;
  uint8_t     m_ch_num;
  uint32_t    m_context;

  FAR uint32_t *m_dma_buffer[2];
  uint8_t      m_dma_buf_cnt;

  uint32_t    m_min_size;
  uint32_t    m_fade_required_sample;

  Queue<AudioDrvDmaRunParam, READY_QUEUE_NUM> m_ready_que;
  Queue<AudioDrvDmaRunParam, RUNNING_QUEUE_NUM> m_running_que;

  static dmaDrvFuncTbl  m_func_tbl[];

  static uint32_t     m_funcTblNum;

  void readyQuePush(const AudioDrvDmaRunParam &dmaParam);
  void readyQuePop();
  void runningQuePush(const AudioDrvDmaRunParam &dmaParam);
  void runningQuePop();

  dmaDrvFuncTbl* searchFuncTbl(ExternalEvent);

  E_AS setDmaCmd(cxd56_audio_dma_t, uint32_t, uint32_t, bool, bool);

  bool illegal(void*);
  void muteSdinVol(bool);
  void unMuteSdinVol(bool);
  bool init(void*);
  void runDmaSplitRequest(AudioDrvDmaRunParam*, uint32_t, uint16_t);
  bool runDmaOnPrepare(void*);
  bool runDmaOnStop(void*);
  bool runDmaOnReady(void*);
  bool runDma(void*);
  bool startDma(void*);
  void dmaCmplt(void);
  bool dmaCmpltOnRun(void*);
  bool dmaCmpltOnFlush(void*);
  bool dmaCmpltOnError(void*);
  bool illegalDmaCmplt(void*);
  bool dmaErrInt(void*);
  bool dmaErrIntOnRun(void*);
  bool dmaErrBus(void*);
  bool dmaErrBusOnRun(void*);
  bool stop(void*);
  bool stopOnRun(void*);
  bool getInfo(void*);
  void dmaErrCb(E_AS_BB);
  void allocDmaBuffer(cxd56_audio_dma_t);
  void freeDmaBuffer(cxd56_audio_dma_t);
  void fadeControl(void);
  void volumeCtrl(bool validity, bool is_last_frame);
  bool pushRequest(void*, bool);
  bool ignore(void*);


  static void dma_err_callback(AudioDrvDmaError *pParam)
  {
    printf("dma(%d) state(%d) err(%d)\n",
           pParam->dmac_id, pParam->state, pParam->status);
  }
};

#endif /* __MODULES_AUDIO_DMA_CONTROLLER_AUDIO_DMA_DRV_H */

