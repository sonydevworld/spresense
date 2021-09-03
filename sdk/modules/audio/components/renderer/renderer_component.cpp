/****************************************************************************
 * modules/audio/components/renderer/renderer_component.cpp
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

#include "renderer_component.h"

#include "dma_controller/audio_dma_drv.h"

#define DBG_MODULE DBG_MODULE_AS

__WIEN2_BEGIN_NAMESPACE

#ifndef CONFIG_AUDIOUTILS_RENDERER_CH_NUM
#define CONFIG_AUDIOUTILS_RENDERER_CH_NUM 2
#endif
/* Equals to the Max. number of available DMAC resources. */
#define MAX_RENDER_COMP_INSTANCE_NUM  CONFIG_AUDIOUTILS_RENDERER_CH_NUM

class RenderCompFactory
{
public:

  bool getRenderCompHandler(RenderComponentHandler *p_handle)
  {
    for (int i = 0 ; i < MAX_RENDER_COMP_INSTANCE_NUM ; i++)
      {
        if (m_render_comp_instance[i].is_available)
          {
            *(p_handle) = i;
            m_render_comp_instance[i].is_available = false;
            return true;
          }
      }

    RENDERER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
    return false;
  }

  bool destroyRenderCompHandler(RenderComponentHandler handle)
  {
    if (MAX_RENDER_COMP_INSTANCE_NUM > handle)
      {
        m_render_comp_instance[handle].is_available = true;
        return true;
      }

    RENDERER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
    return false;
  }

  bool isEmptyRenderCompHandler(void)
  {
    for (int i = 0 ; i < MAX_RENDER_COMP_INSTANCE_NUM ; i++)
      {
        if (m_render_comp_instance[i].is_available != true)
          {
            return false;
          }
      }

    return true;
  }

  RendererComponent* getRenderCompInstance(RenderComponentHandler handle)
  {
    if (MAX_RENDER_COMP_INSTANCE_NUM > handle)
      {
        return &(m_render_comp_instance[handle].render_comp_instance);
      }

    return NULL;
  }

  RenderComponentHandler getRenderHandleByDmacId(cxd56_audio_dma_t dmac_id)
  {
    for (int i = 0 ; i < MAX_RENDER_COMP_INSTANCE_NUM ; i++)
      {
        if (m_render_comp_instance[i].render_comp_instance.m_dmac_id ==
              dmac_id)
          {
            return static_cast<RenderComponentHandler>(i);
          }
      }

    return MAX_RENDER_COMP_INSTANCE_NUM;
  }

  RenderCompFactory() {}
  ~RenderCompFactory() {}

  bool parse(RenderComponentHandler handle,
             int event,
             const RendererComponent::RendererComponentParam& param);

private:

  struct InstanceParam
  {
    RendererComponent render_comp_instance;
    bool              is_available;

    InstanceParam():
      is_available(true)
      {}
  };

  InstanceParam m_render_comp_instance[MAX_RENDER_COMP_INSTANCE_NUM];
};

static RenderCompFactory *s_pFactory=NULL;

static pthread_t s_render_pid[MAX_RENDER_COMP_INSTANCE_NUM];
static MsgQueId s_self_dtq[MAX_RENDER_COMP_INSTANCE_NUM];
static MsgQueId s_self_sync_dtq[MAX_RENDER_COMP_INSTANCE_NUM];

/*--------------------------------------------------------------------
    C Interface
  --------------------------------------------------------------------*/
extern "C" {

static void AS_RendererNotifyDmaDoneDev(AudioDrvDmaResult *param)
{
  RenderComponentHandler handle =
    s_pFactory->getRenderHandleByDmacId(param->dmac_id);
  RendererComponent *instance = s_pFactory->getRenderCompInstance(handle);
  instance->m_callback(param, static_cast<void*>(instance->m_p_requester));
}

/*--------------------------------------------------------------------*/
static void AS_RendererNotifyDmaError(AudioDrvDmaError *p_param)
{
  switch (p_param->status)
    {
      case E_AS_BB_DMA_OK:
        D_ASSERT(0);
        break;

      case E_AS_BB_DMA_ILLEGAL:
      case E_AS_BB_DMA_ERR_BUS:
      case E_AS_BB_DMA_ERR_INT:
      case E_AS_BB_DMA_ERR_START:
      case E_AS_BB_DMA_ERR_REQUEST:
        break;

      case E_AS_BB_DMA_UNDERFLOW:
        {
          RenderComponentHandler handle =
            s_pFactory->getRenderHandleByDmacId(p_param->dmac_id);
          RendererComponent *instance =
            s_pFactory->getRenderCompInstance(handle);
          instance->m_err_callback(p_param,
                                   static_cast<void*>(instance->m_p_requester));
        }
        break;
      case E_AS_BB_DMA_OVERFLOW:
      case E_AS_BB_DMA_PARAM:
      default:
        break;
    }
}


/*--------------------------------------------------------------------*/
FAR void AS_RendererCmpEntryDev0(FAR void *arg)
{
  RendererComponent *instance = s_pFactory->getRenderCompInstance(0);

  instance->create(AS_RendererNotifyDmaDoneDev,
                   AS_RendererNotifyDmaError,
                   s_self_dtq[0],
                   s_self_sync_dtq[0]);
}

/*--------------------------------------------------------------------*/
#if MAX_RENDER_COMP_INSTANCE_NUM > 1
FAR void AS_RendererCmpEntryDev1(FAR void *arg)
{
  RendererComponent *instance = s_pFactory->getRenderCompInstance(1);

  instance->create(AS_RendererNotifyDmaDoneDev,
                   AS_RendererNotifyDmaError,
                   s_self_dtq[1],
                   s_self_sync_dtq[1]);
}
#endif

/*--------------------------------------------------------------------*/
bool AS_CreateRenderer(FAR AsCreateRendererParam_t *param)
{
  MsgQueId dev0_self_dtq      = param->msgq_id.dev0_req;
  MsgQueId dev0_self_sync_dtq = param->msgq_id.dev0_sync;
  MsgQueId dev1_self_dtq      = param->msgq_id.dev1_req;
  MsgQueId dev1_self_sync_dtq = param->msgq_id.dev1_sync;

  if ((dev0_self_dtq == 0xFF || dev0_self_sync_dtq == 0xFF)
    || dev0_self_dtq == dev0_self_sync_dtq)
    {
      RENDERER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  if (s_pFactory != NULL)
    {
      RENDERER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  s_pFactory = new RenderCompFactory();

  if (s_pFactory == NULL)
    {
      RENDERER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  s_render_pid[0] = INVALID_PROCESS_ID;
#if MAX_RENDER_COMP_INSTANCE_NUM > 1
  s_render_pid[1] = INVALID_PROCESS_ID;
#endif

  /* dev0 setting */

  s_self_dtq[0]      = dev0_self_dtq;
  s_self_sync_dtq[0] = dev0_self_sync_dtq;

  /* Reset Message queue. */

  FAR MsgQueBlock *que;
  err_t err_code = MsgLib::referMsgQueBlock(dev0_self_dtq, &que);
  F_ASSERT(err_code == ERR_OK);
  que->reset();

  /* Init pthread attributes object. */

  pthread_attr_t attr;

  pthread_attr_init(&attr);

  /* Set pthread scheduling parameter. */

  struct sched_param sch_param;

  sch_param.sched_priority = 200;
  attr.stacksize           = 1024 * 2;

  pthread_attr_setschedparam(&attr, &sch_param);

  /* Create thread. */

  int ret = pthread_create(&s_render_pid[0],
                           &attr,
                           (pthread_startroutine_t)AS_RendererCmpEntryDev0,
                           (pthread_addr_t)NULL);
  if (ret < 0)
    {
      RENDERER_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  pthread_setname_np(s_render_pid[0], "renderer0");

  /* dev1 setting */

  if (dev1_self_dtq != 0xFF || dev1_self_sync_dtq != 0xFF)
    {
#if MAX_RENDER_COMP_INSTANCE_NUM > 1
      if ((dev1_self_dtq == 0xFF || dev1_self_sync_dtq == 0xFF)
        || dev1_self_dtq == dev1_self_sync_dtq
        || dev0_self_dtq == dev1_self_dtq
        || dev0_self_sync_dtq == dev1_self_sync_dtq)
        {
          RENDERER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          pthread_cancel(s_render_pid[0]);
          pthread_join(s_render_pid[0], NULL);
          s_render_pid[0] = INVALID_PROCESS_ID;
          delete s_pFactory;
          s_pFactory = NULL;
          return false;
        }

      s_self_dtq[1]      = dev1_self_dtq;
      s_self_sync_dtq[1] = dev1_self_sync_dtq;

      /* Reset Message queue. */

      err_code = MsgLib::referMsgQueBlock(dev1_self_dtq, &que);
      F_ASSERT(err_code == ERR_OK);
      que->reset();

      /* Create thread. */
      /* Attributes are as same as capture dev0. */
    
      ret = pthread_create(&s_render_pid[1],
                           &attr,
                           (pthread_startroutine_t)AS_RendererCmpEntryDev1,
                           (pthread_addr_t)NULL);
      if (ret < 0)
        {
          RENDERER_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
          return false;
        }

      pthread_setname_np(s_render_pid[1], "renderer1");
#else
      RENDERER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
#endif
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AS_DeleteRenderer(void)
{
  if (s_pFactory != NULL)
    {
      if (s_pFactory->isEmptyRenderCompHandler())
        {
          pthread_cancel(s_render_pid[0]);
          pthread_join(s_render_pid[0], NULL);
          s_render_pid[0] = INVALID_PROCESS_ID;

#if MAX_RENDER_COMP_INSTANCE_NUM > 1
          if (s_render_pid[1] != INVALID_PROCESS_ID)
            {
              pthread_cancel(s_render_pid[1]);
              pthread_join(s_render_pid[1], NULL);
              s_render_pid[1] = INVALID_PROCESS_ID;
            }
#endif
          delete s_pFactory;
          s_pFactory = NULL;

          return true;
        }
    }

  RENDERER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
  return false;
}

} /* extern "C" */

/*--------------------------------------------------------------------*/
bool RenderCompFactory::parse
               (RenderComponentHandler handle,
                int event,
                const RendererComponent::RendererComponentParam& param)
{
  if (MAX_RENDER_COMP_INSTANCE_NUM > handle
   && !m_render_comp_instance[handle].is_available)
    {
      err_t err =
        MsgLib::send<RendererComponent::RendererComponentParam>
                                               (s_self_dtq[handle],
                                                MsgPriNormal,
                                                event,
                                                s_self_sync_dtq[handle],
                                                param);

      F_ASSERT(err == ERR_OK);
      return true;
    }

  RENDERER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
  return false;
}

/*--------------------------------------------------------------------*/
static bool rcv_result(int ch)
{
  err_t        err_code;
  MsgQueBlock  *que;
  MsgPacket    *msg;

  err_code = MsgLib::referMsgQueBlock(s_self_sync_dtq[ch], &que);
  F_ASSERT(err_code == ERR_OK);

  err_code = que->recv(TIME_FOREVER, &msg);
  F_ASSERT(err_code == ERR_OK);
  F_ASSERT(msg->getType() == MSG_AUD_BB_RST);

  bool result = msg->moveParam<bool>();
  err_code = que->pop();
  F_ASSERT(err_code == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool AS_get_render_comp_handler(RenderComponentHandler *p_handle,
                                RenderDevice device_type)
{
  if (!s_pFactory->getRenderCompHandler(p_handle))
    {
      return false;
    }

  RendererComponent::RendererComponentParam param;

  if (*p_handle == 0)
    {
      param.act_render_param.dma_path_id = CXD56_AUDIO_DMA_PATH_MEM_TO_BUSIF1;
      param.act_render_param.sig_id      = CXD56_AUDIO_SIG_BUSIF1;
      param.act_render_param.sel_info.au_dat_sel1 = true;
      param.act_render_param.sel_info.au_dat_sel2 = false;
      param.act_render_param.sel_info.cod_insel2  = true;
      param.act_render_param.sel_info.cod_insel3  = false;
      param.act_render_param.sel_info.src1in_sel  = false;
      param.act_render_param.sel_info.src2in_sel  = false;
    }
  else
    {
      param.act_render_param.dma_path_id = CXD56_AUDIO_DMA_PATH_MEM_TO_BUSIF2;
      param.act_render_param.sig_id      = CXD56_AUDIO_SIG_BUSIF2;
      param.act_render_param.sel_info.au_dat_sel1 = false;
      param.act_render_param.sel_info.au_dat_sel2 = true;
      param.act_render_param.sel_info.cod_insel2  = false;
      param.act_render_param.sel_info.cod_insel3  = true;
      param.act_render_param.sel_info.src1in_sel  = false;
      param.act_render_param.sel_info.src2in_sel  = false;
    }

  if (!s_pFactory->parse(*p_handle, MSG_AUD_BB_CMD_ACT, param))
    {
      return false;
    }

  if (!rcv_result(*p_handle))
    {
      RENDERER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AS_release_render_comp_handler(RenderComponentHandler handle)
{
  RendererComponent::RendererComponentParam param;

  if (!s_pFactory->parse(handle, MSG_AUD_BB_CMD_DEACT, param))
    {
      return false;
    }

  if (!rcv_result(handle))
    {
      RENDERER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  if (!s_pFactory->destroyRenderCompHandler(handle))
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AS_init_renderer(RenderComponentHandler handle,
                      RenderDoneCB callback,
                      RenderErrorCB err_callback,
                      void *p_requester,
                      uint8_t bit_length)
{
  RendererComponent::RendererComponentParam param;

  param.init_render_param.format = ((bit_length == AS_BITLENGTH_16) ?
                                    CXD56_AUDIO_SAMP_FMT_16 :
                                    CXD56_AUDIO_SAMP_FMT_24);
  param.init_render_param.callback    = callback;
  param.init_render_param.p_requester = p_requester;
  param.init_render_param.err_callback = err_callback;

  if (!s_pFactory->parse(handle, MSG_AUD_BB_CMD_INIT, param))
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AS_exec_renderer(RenderComponentHandler handle,
                      void *addr,
                      uint32_t sample,
                      bool is_valid)
{
  RendererComponent::RendererComponentParam param;

  param.exec_render_param.addr     = addr;
  param.exec_render_param.sample   = sample;
  param.exec_render_param.is_valid = is_valid;

  if (!s_pFactory->parse(handle, MSG_AUD_BB_CMD_RUN, param))
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AS_stop_renderer(RenderComponentHandler handle, asDmacStopMode mode)
{
  RendererComponent::RendererComponentParam param;

  param.stop_render_param.mode = mode;

  if (!s_pFactory->parse(handle, MSG_AUD_BB_CMD_STOP, param))
    {
      return false;
    }

  return rcv_result(handle);
}

/*--------------------------------------------------------------------*/
static void dma_notify_cmplt_int(cxd56_audio_dma_t dmacId, uint32_t dma_result)
{
  E_AS_DMA_INT code;
  RendererComponent::RendererComponentParam param;

  switch (dma_result)
    {
      case CXD56_AUDIO_ECODE_DMA_CMPLT:
        code = E_AS_DMA_INT_CMPLT;
        break;

      case CXD56_AUDIO_ECODE_DMA_CMB:
        code = E_AS_DMA_INT_ERR_BUS;
        break;

      default:
        code = E_AS_DMA_INT_ERR;
        break;
    }

  RenderComponentHandler handle = s_pFactory->getRenderHandleByDmacId(dmacId);

  param.notify_render_param.type = RendererComponent::NtfDmaCmplt;
  param.notify_render_param.code = code;

  err_t err= MsgLib::sendIsr<RendererComponent::RendererComponentParam>
                                                   (s_self_dtq[handle],
                                                   MsgPriNormal,
                                                   MSG_AUD_BB_CMD_CMPLT,
                                                   s_self_sync_dtq[handle],
                                                   param);
  F_ASSERT(err == ERR_OK);
}

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
RendererComponent::EvtProc RendererComponent::EvetProcTbl[AUD_BB_MSG_NUM][StateNum] =
{
  /* Message type: MSG_AUD_BB_CMD_ACT */
  {                                   /* Renderer status: */
    &RendererComponent::act,          /*   Booted         */
    &RendererComponent::illegal,      /*   Ready          */
    &RendererComponent::illegal,      /*   PreAct         */
    &RendererComponent::illegal       /*   Act            */
  },

  /* Message type: MSG_AUD_BB_CMD_DEACT */
  {                                   /* Renderer status: */
    &RendererComponent::illegal,      /*   Booted         */
    &RendererComponent::deact,        /*   Ready          */
    &RendererComponent::illegal,      /*   PreAct         */
    &RendererComponent::illegal       /*   Act            */
  },

  /* Message type: MSG_AUD_BB_CMD_INIT */
  {                                   /* Renderer status: */
    &RendererComponent::illegal,      /*   Booted         */
    &RendererComponent::init,         /*   Ready          */
    &RendererComponent::illegal,      /*   PreAct         */
    &RendererComponent::illegal       /*   Act            */
  },

  /* Message type: MSG_AUD_BB_CMD_RUN */
  {                                   /* Renderer status: */
    &RendererComponent::illegal,      /*   Booted         */
    &RendererComponent::runOnRdy,     /*   Ready          */
    &RendererComponent::runOnPreAct,  /*   PreAct         */
    &RendererComponent::runOnAct      /*   Act            */
  },

  /* Message type: MSG_AUD_BB_CMD_STOP */
  {                                   /* Renderer status: */
    &RendererComponent::illegal,      /*   Booted         */
    &RendererComponent::illegal,      /*   Ready          */
    &RendererComponent::stopOnPreAct, /*   PreAct         */
    &RendererComponent::stopOnAct     /*   Act            */
  },

  /* Message type: MSG_AUD_BB_CMD_CMPLT */
  {                                   /* Renderer status: */
    &RendererComponent::illegal,      /*   Booted         */
    &RendererComponent::notify,       /*   Ready          */
    &RendererComponent::notify,       /*   PreAct         */
    &RendererComponent::notify        /*   Act            */
  },
};


/*--------------------------------------------------------------------*/
void RendererComponent::create(AS_DmaDoneCb dma_done_cb,
                               AS_ErrorCb dma_err_cb,
                               MsgQueId self_dtq,
                               MsgQueId self_sync_dtq)
{
  this->m_dma_done_cb   = dma_done_cb;
  this->m_dma_err_cb    = dma_err_cb;
  this->m_self_dtq      = self_dtq;
  this->m_self_sync_dtq = self_sync_dtq;

  this->run();
}

/*--------------------------------------------------------------------*/
void RendererComponent::run()
{
  err_t        err_code;
  MsgQueBlock  *que;
  MsgPacket    *msg;

  err_code = MsgLib::referMsgQueBlock(m_self_dtq, &que);
  F_ASSERT(err_code == ERR_OK);

  while (1)
    {
      err_code = que->recv(TIME_FOREVER, &msg);
      F_ASSERT(err_code == ERR_OK);

      parse(msg);

      err_code = que->pop();
      F_ASSERT(err_code == ERR_OK);
    }
}

/*--------------------------------------------------------------------*/
bool RendererComponent::parse(MsgPacket *msg)
{
  uint8_t event = MSG_GET_SUBTYPE(msg->getType());

  RendererComponentParam param = msg->moveParam<RendererComponentParam>();
  return (this->*EvetProcTbl[event][m_state.get()])(param);
}

/*--------------------------------------------------------------------*/
bool RendererComponent::illegal(const RendererComponentParam& param)
{
  RENDERER_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
  return false;
}

/*--------------------------------------------------------------------*/
bool RendererComponent::act(const RendererComponentParam& param)
{
  bool result = true;

  RENDERER_DBG("ACT: path %d, signal %d\n",
               param.act_render_param.dma_path_id,
               param.act_render_param.sig_id);
  if (result &&
      CXD56_AUDIO_ECODE_OK != cxd56_audio_get_dmahandle(
        param.act_render_param.dma_path_id,
        &m_dmac_id))
    {
      result = false;
    }

  if (result &&
      CXD56_AUDIO_ECODE_OK != cxd56_audio_set_datapath(
        param.act_render_param.sig_id,
        param.act_render_param.sel_info))
    {
      result = false;
    }

  if (result && E_AS_OK != AS_ActivateDmac(m_dmac_id))
    {
      result = false;
    }

  if (result)
    {
      m_state = Ready;
    }

  err_t err = MsgLib::send<bool>(m_self_sync_dtq,
                                 MsgPriNormal,
                                 MSG_AUD_BB_RST,
                                 NULL,
                                 result);
  F_ASSERT(err == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool RendererComponent::deact(const RendererComponentParam& param)
{
  bool result = true;

  RENDERER_DBG("DEACT:\n");

  if (result && E_AS_OK != AS_DeactivateDmac(m_dmac_id))
    {
      result = false;
    }

  if (result &&
      CXD56_AUDIO_ECODE_OK != cxd56_audio_free_dmahandle(m_dmac_id))
    {
      result = false;
    }

  if (result)
    {
      m_state = Booted;
    }

  err_t err = MsgLib::send<bool>(m_self_sync_dtq,
                                 MsgPriNormal,
                                 MSG_AUD_BB_RST,
                                 NULL,
                                 result);
  F_ASSERT(err == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool RendererComponent::init(const RendererComponentParam& param)
{
  asInitDmacParam init_param;

  RENDERER_DBG("INIT: fmt %d, cb %p, req %p\n",
               param.init_render_param.format,
               param.init_render_param.callback,
               param.init_render_param.p_requester);

  init_param.dmacId         = m_dmac_id;
  init_param.format         = param.init_render_param.format;
  init_param.p_error_func   = m_dma_err_cb;
  init_param.fade_en        = true;
  init_param.p_dmadone_func = m_dma_done_cb;

  m_callback    = param.init_render_param.callback;
  m_p_requester = param.init_render_param.p_requester;
  m_err_callback = param.init_render_param.err_callback;

  if (E_AS_OK != AS_InitDmac(&init_param))
    {
      return false;
    }

  if (E_AS_OK != AS_RegistDmaIntCb(m_dmac_id, dma_notify_cmplt_int))
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool RendererComponent::runOnRdy(const RendererComponentParam& param)
{
  asWriteDmacParam write_dmac_param;

  write_dmac_param.dmacId   = m_dmac_id;
  write_dmac_param.addr     = (uint32_t)param.exec_render_param.addr;
  write_dmac_param.size     = param.exec_render_param.sample;
  write_dmac_param.addr2    = 0;
  write_dmac_param.size2    = 0;
  write_dmac_param.validity = param.exec_render_param.is_valid;

  /* At first, transfer request is only pushed into queue.
   * Not start yet, DMAC need more than one requests at its start.
   */

  if (!m_write_dmac_cmd_que.push(write_dmac_param))
    {
      RENDERER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  m_state = PreAct;

  return true;
}

/*--------------------------------------------------------------------*/
bool RendererComponent::runOnPreAct(const RendererComponentParam& param)
{
  asWriteDmacParam write_dmac_param;

  write_dmac_param.dmacId   = m_dmac_id;
  write_dmac_param.addr     = (uint32_t)param.exec_render_param.addr;
  write_dmac_param.size     = param.exec_render_param.sample;
  write_dmac_param.addr2    = 0;
  write_dmac_param.size2    = 0;
  write_dmac_param.validity = param.exec_render_param.is_valid;

  if (!m_write_dmac_cmd_que.push(write_dmac_param))
    {
      RENDERER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  /* When predefined number of requests are ready,
   * set request to DMAC and start
   */

  if (m_write_dmac_cmd_que.full())
    {
      while (!m_write_dmac_cmd_que.empty())
        {
          asWriteDmacParam cur_cmd = m_write_dmac_cmd_que.top();

          if (E_AS_OK != AS_WriteDmac(&cur_cmd))
            {
              return false;
            }

          if (!m_write_dmac_cmd_que.pop())
            {
              RENDERER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
              break;
            }
        }

      if (E_AS_OK != AS_StartDmac(m_dmac_id))
        {
          return false;
        }

      m_state = Act;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool RendererComponent::runOnAct(const RendererComponentParam& param)
{
  asWriteDmacParam write_dmac_param;

  write_dmac_param.dmacId   = m_dmac_id;
  write_dmac_param.addr     = (uint32_t)param.exec_render_param.addr;
  write_dmac_param.size     = param.exec_render_param.sample;
  write_dmac_param.addr2    = 0;
  write_dmac_param.size2    = 0;
  write_dmac_param.validity = param.exec_render_param.is_valid;

  if (E_AS_OK != AS_WriteDmac(&write_dmac_param))
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool RendererComponent::stopOnPreAct(const RendererComponentParam& param)
{
  bool result = true;

  RENDERER_DBG("STOP: mode %d\n", param.stop_render_param.mode);

  err_t err = MsgLib::send<bool>(m_self_sync_dtq,
                                 MsgPriNormal,
                                 MSG_AUD_BB_RST,
                                 NULL,
                                 result);
  F_ASSERT(err == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool RendererComponent::stopOnAct(const RendererComponentParam& param)
{
  bool result = true;

  RENDERER_DBG("STOP: mode %d\n", param.stop_render_param.mode);

  if (E_AS_OK != AS_StopDmac(m_dmac_id, param.stop_render_param.mode))
    {
      result = false;
    }

  if (result)
    {
      m_state = Ready;
    }

  err_t err = MsgLib::send<bool>(m_self_sync_dtq,
                                 MsgPriNormal,
                                 MSG_AUD_BB_RST,
                                 NULL,
                                 result);
  F_ASSERT(err == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool RendererComponent::notify(const RendererComponentParam& param)
{
  bool result = false;

  switch(param.notify_render_param.type)
    {
      case NtfDmaCmplt:
        {
          if (E_AS_OK != AS_NotifyDmaCmplt(m_dmac_id,
                                           param.notify_render_param.code))
            {
              return false;
            }
          result = true;
        }
        break;

      default:
        break;
    }

  return result;
}


__WIEN2_END_NAMESPACE

