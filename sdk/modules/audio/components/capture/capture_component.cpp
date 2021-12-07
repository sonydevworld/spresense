/****************************************************************************
 * modules/audio/components/capture/capture_component.cpp
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

#include "capture_component.h"

#include "dma_controller/audio_dma_drv.h"

#define DBG_MODULE DBG_MODULE_AS

__WIEN2_BEGIN_NAMESPACE

class CaptureCompFactory
{
public:

  bool getCaptureCompHandler(CaptureComponentHandler *p_handle)
  {
    for (int i = 0; i < MAX_CAPTURE_COMP_INSTANCE_NUM; i++)
      {
        if (m_capture_comp_instance[i].is_available)
          {
            *(p_handle) = i;
            m_capture_comp_instance[i].is_available = false;
            return true;
          }
      }

    CAPTURE_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
    return false;
  }

  bool destroyCaptureCompHandler(CaptureComponentHandler handle)
  {
    if (MAX_CAPTURE_COMP_INSTANCE_NUM > handle)
      {
        m_capture_comp_instance[handle].is_available = true;
        return true;
      }

    CAPTURE_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
    return false;
  }

  bool isEmptyCaptureCompHandler(void)
  {
    for (int i = 0; i < MAX_CAPTURE_COMP_INSTANCE_NUM; i++)
      {
        if (m_capture_comp_instance[i].is_available != true)
          {
            return false;
          }
      }

    return true;
  }

  CaptureComponent* getCaptureCompInstance(CaptureComponentHandler handle)
  {
    if (MAX_CAPTURE_COMP_INSTANCE_NUM > handle)
      {
        return &(m_capture_comp_instance[handle].capture_comp_instance);
      }

    return NULL;
  }

  CaptureComponentHandler getCaptureHandleByDmacId(cxd56_audio_dma_t dmac_id)
  {
    for (int i = 0; i < MAX_CAPTURE_COMP_INSTANCE_NUM; i++)
      {
        if (m_capture_comp_instance[i].capture_comp_instance.m_dmac_id ==
              dmac_id)
          {
            return static_cast<CaptureComponentHandler>(i);
          }
      }

    return MAX_CAPTURE_COMP_INSTANCE_NUM;
  }

  CaptureCompFactory() {}
  ~CaptureCompFactory() {}

  bool parse(CaptureComponentHandler handle,
             int event,
             const CaptureComponentParam& param);

private:

  /* Instance of capture_component can be created
   * up to MAX_CAPTURE_COMP_INSTANCE_NUM.
   * A one instance have relation with a one device.
   */

  struct InstanceParam
  {
    CaptureComponent capture_comp_instance;
    bool             is_available;

    InstanceParam()
      : is_available(true)
      {}
  };

  InstanceParam m_capture_comp_instance[MAX_CAPTURE_COMP_INSTANCE_NUM];
};

static CaptureCompFactory *s_pFactory = NULL;

static pthread_t s_capture_pid[MAX_CAPTURE_COMP_INSTANCE_NUM];
static MsgQueId s_self_dtq[MAX_CAPTURE_COMP_INSTANCE_NUM];
static MsgQueId s_self_sync_dtq[MAX_CAPTURE_COMP_INSTANCE_NUM];

/*--------------------------------------------------------------------
    C Interface
  --------------------------------------------------------------------*/
extern "C" {
static void AS_CaptureNotifyDmaDoneDev0(AudioDrvDmaResult *p_param)
{
  CaptureComponent *instance = s_pFactory->getCaptureCompInstance(0);

  /* Check DMA requests */

  if (instance->m_req_data_que.empty())
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      return;
    }

  /* Construct result parameters */

  CaptureDataParam result;

  result.output_device = instance->m_output_device;
  result.end_flag      = p_param->endflg;
  result.buf           = instance->m_req_data_que.top();

  /* Notify to requester (use callback) */

  instance->m_callback(result);

  /* Discard request parameter */

  if (!instance->m_req_data_que.pop())
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return;
    }

  /* Notify invalid frames which couldn't read DMA. */

  while (!instance->m_req_data_que.empty())
    {
      if (instance->m_req_data_que.top().validity)
        {
          break;
        }

      result.output_device = instance->m_output_device;
      result.end_flag      = false;
      result.buf           = instance->m_req_data_que.top();

      instance->m_callback(result);

      if (!instance->m_req_data_que.pop())
        {
          CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
          return;
        }
    }
}

/*--------------------------------------------------------------------*/
#if MAX_CAPTURE_COMP_INSTANCE_NUM > 1
static void AS_CaptureNotifyDmaDoneDev1(AudioDrvDmaResult *p_param)
{
  CaptureComponent *instance = s_pFactory->getCaptureCompInstance(1);

  if (instance->m_req_data_que.empty())
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      return;
    }

  CaptureDataParam result;

  result.output_device = instance->m_output_device;
  result.end_flag      = p_param->endflg;
  result.buf           = instance->m_req_data_que.top();

  instance->m_callback(result);

  if (!instance->m_req_data_que.pop())
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return;
    }

  while (!instance->m_req_data_que.empty())
    {
      if (instance->m_req_data_que.top().validity)
        {
          break;
        }

      result.output_device = instance->m_output_device;
      result.end_flag      = false;
      result.buf           = instance->m_req_data_que.top();

      instance->m_callback(result);

      if (!instance->m_req_data_que.pop())
        {
          CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
          return;
        }
    }
}
#endif

/*--------------------------------------------------------------------*/
static void AS_CaptureNotifyDmaError(AudioDrvDmaError *p_param)
{
  CaptureComponentHandler handle =
    s_pFactory->getCaptureHandleByDmacId(p_param->dmac_id);

  CaptureComponent *instance =
    s_pFactory->getCaptureCompInstance(handle);

  CaptureErrorParam errparam;

  switch (p_param->status)
    {
      case E_AS_BB_DMA_OK:
          D_ASSERT(0);
          break;

      case E_AS_BB_DMA_ILLEGAL:
      case E_AS_BB_DMA_ERR_BUS:
      case E_AS_BB_DMA_ERR_START:
      case E_AS_BB_DMA_ERR_REQUEST:
          F_ASSERT(0);
          break;

      case E_AS_BB_DMA_ERR_INT:
          errparam.error_type = CaptureErrorErrInt;
          instance->m_err_callback(errparam);
          break;

      case E_AS_BB_DMA_UNDERFLOW:
      case E_AS_BB_DMA_OVERFLOW:
      case E_AS_BB_DMA_PARAM:
          errparam.error_type = CaptureErrorDMAunder;
          instance->m_err_callback(errparam);
          break;

      default:
          break;
    }
}

/*--------------------------------------------------------------------*/
FAR void AS_CaptureCmpEntryDev0(FAR void *arg)
{
  CaptureComponent *instance = s_pFactory->getCaptureCompInstance(0);

  instance->create(AS_CaptureNotifyDmaDoneDev0,
                   AS_CaptureNotifyDmaError,
                   s_self_dtq[0],
                   s_self_sync_dtq[0]);
}

/*--------------------------------------------------------------------*/
#if MAX_CAPTURE_COMP_INSTANCE_NUM > 1
FAR void AS_CaptureCmpEntryDev1(FAR void *arg)
{
  CaptureComponent *instance = s_pFactory->getCaptureCompInstance(1);

  instance->create(AS_CaptureNotifyDmaDoneDev1,
                   AS_CaptureNotifyDmaError,
                   s_self_dtq[1],
                   s_self_sync_dtq[1]);
}
#endif

/*--------------------------------------------------------------------*/
bool AS_CreateCapture(FAR AsCreateCaptureParam_t *param)
{
  /* Parameter check */

  if (param == NULL)
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Create */

  MsgQueId dev0_self_dtq      = param->msgq_id.dev0_req;
  MsgQueId dev0_self_sync_dtq = param->msgq_id.dev0_sync;
  MsgQueId dev1_self_dtq      = param->msgq_id.dev1_req;
  MsgQueId dev1_self_sync_dtq = param->msgq_id.dev1_sync;

  if ((dev0_self_dtq == 0xFF || dev0_self_sync_dtq == 0xFF)
    || dev0_self_dtq == dev0_self_sync_dtq)
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  if (s_pFactory != NULL)
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  s_pFactory = new CaptureCompFactory();

  if (s_pFactory == NULL)
    {
      RENDERER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  s_capture_pid[0] = INVALID_PROCESS_ID;
#if MAX_CAPTURE_COMP_INSTANCE_NUM > 1
  s_capture_pid[1] = INVALID_PROCESS_ID;
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

  int ret = pthread_create(&s_capture_pid[0],
                           &attr,
                           (pthread_startroutine_t)AS_CaptureCmpEntryDev0,
                           (pthread_addr_t)NULL);
  if (ret < 0)
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  pthread_setname_np(s_capture_pid[0], "capture0");

  /* dev1 setting */

  if (dev1_self_dtq != 0xFF || dev1_self_sync_dtq != 0xFF)
    {
#if MAX_CAPTURE_COMP_INSTANCE_NUM > 1
      if ((dev1_self_dtq == 0xFF || dev1_self_sync_dtq == 0xFF)
        || dev1_self_dtq == dev1_self_sync_dtq
        || dev0_self_dtq == dev1_self_dtq
        || dev0_self_sync_dtq == dev1_self_sync_dtq)
        {
          CAPTURE_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          pthread_cancel(s_capture_pid[0]);
          pthread_join(s_capture_pid[0], NULL);
          s_capture_pid[0] = INVALID_PROCESS_ID;
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

      ret = pthread_create(&s_capture_pid[1],
                           &attr,
                           (pthread_startroutine_t)AS_CaptureCmpEntryDev1,
                           (pthread_addr_t)NULL);
      if (ret < 0)
        {
          CAPTURE_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
          return false;
        }

      pthread_setname_np(s_capture_pid[1], "capture1");
#else
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
#endif
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AS_DeleteCapture(void)
{
  if (s_pFactory != NULL)
    {
      if (s_pFactory->isEmptyCaptureCompHandler())
        {
          pthread_cancel(s_capture_pid[0]);
          pthread_join(s_capture_pid[0], NULL);
          s_capture_pid[0] = INVALID_PROCESS_ID;

#if MAX_CAPTURE_COMP_INSTANCE_NUM > 1
          if (s_capture_pid[1] != INVALID_PROCESS_ID)
            {
              pthread_cancel(s_capture_pid[1]);
              pthread_join(s_capture_pid[1], NULL);
              s_capture_pid[1] = INVALID_PROCESS_ID;
            }
#endif
          delete s_pFactory;
          s_pFactory = NULL;

          return true;
        }
    }

  CAPTURE_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
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
  F_ASSERT(msg->getType() == MSG_AUD_CAP_RST);

  bool result = msg->moveParam<bool>();
  err_code = que->pop();
  F_ASSERT(err_code == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool AS_get_capture_comp_handler(CaptureComponentHandler *p_handle,
                                 CaptureDevice device_type,
                                 MemMgrLite::PoolId mem_pool_id)
{
  if (!s_pFactory->getCaptureCompHandler(p_handle))
    {
      return false;
    }

  CaptureComponentParam param;

  switch (device_type)
    {
      case CaptureDeviceAnalogMic:
      case CaptureDeviceDigitalMic:
          param.act_param.dma_path_id = CXD56_AUDIO_DMA_PATH_MIC_TO_MEM;
          break;

      case CaptureDeviceI2S:
          param.act_param.dma_path_id = CXD56_AUDIO_DMA_PATH_I2S0_TO_MEM;
          break;

      default:
          CAPTURE_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          return false;
    }

  param.act_param.output_device = device_type;
  param.act_param.mem_pool_id   = mem_pool_id;

  if (!s_pFactory->parse(*p_handle, MSG_AUD_CAP_CMD_ACT, param))
    {
      return false;
    }

  /* Wait response of ACTIVATE */

  return rcv_result(*p_handle);
}

/*--------------------------------------------------------------------*/
bool AS_release_capture_comp_handler(CaptureComponentHandler handle)
{
  CaptureComponentParam param;

  s_pFactory->parse(handle, MSG_AUD_CAP_CMD_DEACT, param);

  /* Wait response of DEACTIVATE */

  if (!rcv_result(handle))
    {
      return false;
    }

  if (!s_pFactory->destroyCaptureCompHandler(handle))
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AS_init_capture(const CaptureComponentParam *param)
{
  /* Parameter check */

  if (param == NULL)
    {
      return false;
    }

  /* Init */

  if (!s_pFactory->parse(param->handle, MSG_AUD_CAP_CMD_INIT, *param))
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AS_exec_capture(const CaptureComponentParam *param)
{
  /* Parameter check */

  if (param == NULL)
    {
      return false;
    }

  /* Execute */

  if (!s_pFactory->parse(param->handle, MSG_AUD_CAP_CMD_RUN, *param))
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AS_stop_capture(const CaptureComponentParam *param)
{
  /* Parameter check */

  if (param == NULL)
    {
      return false;
    }

  /* Stop */

  if (!s_pFactory->parse(param->handle, MSG_AUD_CAP_CMD_STOP, *param))
    {
      return false;
    }

  /* Wait response of STOP */

  return rcv_result(param->handle);
}

/*--------------------------------------------------------------------*/
bool AS_set_micgain_capture(const CaptureComponentParam *param)
{
  /* Parameter check */

  if (param == NULL)
    {
      return false;
    }

  /* Set Mic Gain */

  if (!s_pFactory->parse(param->handle, MSG_AUD_CAP_CMD_SETMICGAIN, *param))
    {
      return false;
    }

  /* Wait response of Set Mic Gain */

  return rcv_result(param->handle);
}

/*--------------------------------------------------------------------*/
static void dma_notify_cmplt_int(cxd56_audio_dma_t dmacId, uint32_t dma_result)
{
  E_AS_DMA_INT code;
  CaptureComponentParam param;

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

  CaptureComponentHandler handle =
    s_pFactory->getCaptureHandleByDmacId(dmacId);

  param.notify_param.type = NtfDmaCmplt;
  param.notify_param.code = code;

  err_t err = MsgLib::sendIsr<CaptureComponentParam>(s_self_dtq[handle],
                                                     MsgPriNormal,
                                                     MSG_AUD_CAP_CMD_CMPLT,
                                                     s_self_sync_dtq[handle],
                                                     param);
  F_ASSERT(err == ERR_OK);
}

} /* extern "C" */

/*--------------------------------------------------------------------*/
bool CaptureCompFactory::parse(CaptureComponentHandler handle,
                               int event,
                               const CaptureComponentParam& param)
{
  if (MAX_CAPTURE_COMP_INSTANCE_NUM > handle
   && !m_capture_comp_instance[handle].is_available)
    {
      err_t err = MsgLib::send<CaptureComponentParam>(s_self_dtq[handle],
                                                      MsgPriNormal,
                                                      event,
                                                      s_self_sync_dtq[handle],
                                                      param);
      F_ASSERT(err == ERR_OK);
      return true;
    }

  CAPTURE_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
  return false;
}

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
CaptureComponent::EvtProc CaptureComponent::EvetProcTbl[AUD_CAP_MSG_NUM][StateNum] =
{
  /* Message type: MSG_AUD_CAP_CMD_ACT */
  {                                  /* Capture status: */
    &CaptureComponent::act,          /*   Booted        */
    &CaptureComponent::illegal,      /*   Ready         */
    &CaptureComponent::illegal,      /*   PreAct        */
    &CaptureComponent::illegal,      /*   Act           */
    &CaptureComponent::illegal       /*   Error         */
  },

  /* Message type: MSG_AUD_CAP_CMD_DEACT */
  {                                  /* Capture status: */
    &CaptureComponent::illegal,      /*   Booted        */
    &CaptureComponent::deact,        /*   Ready         */
    &CaptureComponent::illegal,      /*   PreAct        */
    &CaptureComponent::illegal,      /*   Act           */
    &CaptureComponent::illegal       /*   Error         */
  },

  /* Message type: MSG_AUD_CAP_CMD_INIT */
  {                                  /* Capture status: */
    &CaptureComponent::illegal,      /*   Booted        */
    &CaptureComponent::init,         /*   Ready         */
    &CaptureComponent::illegal,      /*   PreAct        */
    &CaptureComponent::illegal,      /*   Act           */
    &CaptureComponent::illegal       /*   Error         */
  },

  /* Message type: MSG_AUD_CAP_CMD_RUN */
  {                                  /* Capture status: */
    &CaptureComponent::illegal,      /*   Booted        */
    &CaptureComponent::execOnRdy,    /*   Ready         */
    &CaptureComponent::execOnPreAct, /*   PreAct        */
    &CaptureComponent::execOnAct,    /*   Act           */
    &CaptureComponent::execOnError   /*   Error         */
  },

  /* Message type: MSG_AUD_CAP_CMD_STOP */
  {                                  /* Capture status: */
    &CaptureComponent::illegal,      /*   Booted        */
    &CaptureComponent::stopOnReady,  /*   Ready         */
    &CaptureComponent::stopOnPreAct, /*   PreAct        */
    &CaptureComponent::stopOnAct,    /*   Act           */
    &CaptureComponent::stopOnError   /*   Error         */
  },

  /* Message type: MSG_AUD_CAP_CMD_SETMICGAIN */
  {                                  /* Capture status: */
    &CaptureComponent::illegal,      /*   Booted        */
    &CaptureComponent::setMicGain,   /*   Ready         */
    &CaptureComponent::setMicGain,   /*   PreAct        */
    &CaptureComponent::setMicGain,   /*   Act           */
    &CaptureComponent::setMicGain    /*   Error         */
  },

  /* Message type: MSG_AUD_CAP_CMD_CMPLT */
  {                                  /* Capture status: */
    &CaptureComponent::illegal,      /*   Booted        */
    &CaptureComponent::notify,       /*   Ready         */
    &CaptureComponent::notify,       /*   PreAct        */
    &CaptureComponent::notify,       /*   Act           */
    &CaptureComponent::notify        /*   Error         */
  },
};

/*--------------------------------------------------------------------*/
void CaptureComponent::create(AS_DmaDoneCb dma_done_cb,
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
void CaptureComponent::run()
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
bool CaptureComponent::parse(MsgPacket *msg)
{
  uint8_t event = MSG_GET_SUBTYPE(msg->getType());

  CaptureComponentParam param = msg->moveParam<CaptureComponentParam>();
  return (this->*EvetProcTbl[event][m_state.get()])(param);
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::illegal(const CaptureComponentParam& param)
{
    CAPTURE_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
    return false;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::act(const CaptureComponentParam& param)
{
  bool result = true;

  CAPTURE_DBG("ACT: path %d, outdev %d\n",
              param.act_param.dma_path_id,
              param.act_param.output_device);

  m_output_device  = param.act_param.output_device;
  m_mem_pool_id    = param.act_param.mem_pool_id;

  if (result &&
      CXD56_AUDIO_ECODE_OK != cxd56_audio_get_dmahandle(
        param.act_param.dma_path_id,
        &m_dmac_id))
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
                                 MSG_AUD_CAP_RST,
                                 NULL,
                                 result);
  F_ASSERT(err == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::deact(const CaptureComponentParam& param)
{
  bool result = true;

  CAPTURE_DBG("DEACT\n");

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
                                 MSG_AUD_CAP_RST,
                                 NULL,
                                 result);
  F_ASSERT(err == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::init(const CaptureComponentParam& param)
{
  bool result = true;

  CAPTURE_DBG("INIT: ch num %d, bit len %d, cb %p\n",
              param.init_param.capture_ch_num,
              param.init_param.capture_bit_width,
              param.init_param.callback);

  asInitDmacParam init_param;

  init_param.dmacId = m_dmac_id;
  init_param.ch_num = param.init_param.capture_ch_num;

  switch (param.init_param.capture_bit_width)
    {
      case AS_BITLENGTH_16:
          init_param.format = CXD56_AUDIO_SAMP_FMT_16;
          break;

      case AS_BITLENGTH_24:
      case AS_BITLENGTH_32:
          init_param.format = CXD56_AUDIO_SAMP_FMT_24;
          break;

      default:
          result = false;
          break;
    }

  init_param.fade_en        = false;
  init_param.p_error_func   = m_dma_err_cb;
  init_param.p_dmadone_func = m_dma_done_cb;

  m_callback = param.init_param.callback;
  m_err_callback = param.init_param.err_callback;

  if (result && E_AS_OK != AS_InitDmac(&init_param))
    {
      result = false;
    }

  if (result && E_AS_OK != AS_RegistDmaIntCb(m_dmac_id, dma_notify_cmplt_int))
    {
      result = false;
    }

  m_ch_num = param.init_param.capture_ch_num;

  if ((param.init_param.preset_num > PRE_REQ_QUE_NUM)
   || (param.init_param.preset_num > MemMgrLite::Manager::getPoolNumAvailSegs(m_mem_pool_id)))
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      result = false;
    }

  m_preset_num = param.init_param.preset_num;

  return result;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::execOnRdy(const CaptureComponentParam& param)
{
  if (!m_cap_pre_que.push(param))
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  /* At first, caputre request is only pushed into queue.
   * Not start yet, DMAC needs more than one request at its start.
   */

  m_state = PreAct;

  return true;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::execOnPreAct(const CaptureComponentParam& param)
{
  if (!m_cap_pre_que.push(param))
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  /* When predefined number of requests are ready,
   * send read request to DMAC and start.
   */

  if (m_preset_num <= m_cap_pre_que.size())
    {
      while (!m_cap_pre_que.empty())
        {
          asReadDmacParam dmac_param;
          CaptureComponentParam pre = m_cap_pre_que.top();

          CaptureBuffer capbuf = getCapBuf(pre.exec_param.pcm_sample);

          dmac_param.dmacId   = m_dmac_id;
          dmac_param.addr     = (uint32_t)((capbuf.cap_mh.isNull()) ? NULL : capbuf.cap_mh.getPa());
          dmac_param.size     = pre.exec_param.pcm_sample;
          dmac_param.addr2    = 0;
          dmac_param.size2    = 0;
          dmac_param.validity = true;

          if (E_AS_OK != AS_ReadDmac(&dmac_param))
            {
              capbuf.validity = false;
            }
          else
            {
              capbuf.validity = true;
            }

          enqueDmaReqQue(capbuf);

          if (!m_cap_pre_que.pop())
            {
              CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
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
bool CaptureComponent::execOnAct(const CaptureComponentParam& param)
{
  asReadDmacParam dmac_param;

  CaptureBuffer capbuf = getCapBuf(param.exec_param.pcm_sample);

  dmac_param.dmacId   = m_dmac_id;
  dmac_param.addr     = (uint32_t)((capbuf.cap_mh.isNull()) ? NULL : capbuf.cap_mh.getPa());
  dmac_param.size     = param.exec_param.pcm_sample;
  dmac_param.addr2    = 0;
  dmac_param.size2    = 0;
  dmac_param.validity = true;

  if (E_AS_OK != AS_ReadDmac(&dmac_param))
    {
      capbuf.validity = false;
    }
  else
    {
      capbuf.validity = true;
    }

  enqueDmaReqQue(capbuf);

  return true;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::execOnError(const CaptureComponentParam& param)
{
  CaptureBuffer invalid_req;

  /* Hold as invalid capture request. */

  invalid_req.sample = param.exec_param.pcm_sample;
  invalid_req.validity = false;

  enqueDmaReqQue(invalid_req);

  return true;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::stopOnReady(const CaptureComponentParam& param)
{
  return stopOnPreAct(param);
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::stopOnPreAct(const CaptureComponentParam& param)
{
  bool result = true;

  CAPTURE_DBG("STOP:\n");

  err_t err = MsgLib::send<bool>(m_self_sync_dtq,
                                 MsgPriNormal,
                                 MSG_AUD_CAP_RST,
                                 NULL,
                                 result);
  F_ASSERT(err == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::stopOnAct(const CaptureComponentParam& param)
{
  bool result = true;

  CAPTURE_DBG("STOP:\n");

  if (E_AS_OK != AS_StopDmac(m_dmac_id, param.stop_param.mode))
    {
      result = false;
    }

  if (result)
    {
      m_state = Ready;
    }

  err_t err = MsgLib::send<bool>(m_self_sync_dtq,
                                 MsgPriNormal,
                                 MSG_AUD_CAP_RST,
                                 NULL,
                                 result);
  F_ASSERT(err == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::stopOnError(const CaptureComponentParam& param)
{
  return stopOnAct(param);
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::setMicGain(const CaptureComponentParam& param)
{
  bool result = true;

  CAPTURE_DBG("SETMICGAIN:\n");

  cxd56_audio_mic_gain_t cxd56_mic_gain;

  for (int i = 0; i < CXD56_AUDIO_MIC_CH_MAX; i++)
    {
      if (i < MAX_CAPTURE_MIC_CH)
        {
          cxd56_mic_gain.gain[i] = param.set_micgain_param->mic_gain[i];
        }
      else
        {
          cxd56_mic_gain.gain[i] = 0;
        }
    }

  if (CXD56_AUDIO_ECODE_OK != cxd56_audio_set_micgain(&cxd56_mic_gain))
    {
      result = false;
    }

  err_t err = MsgLib::send<bool>(m_self_sync_dtq,
                                 MsgPriNormal,
                                 MSG_AUD_CAP_RST,
                                 NULL,
                                 result);
  F_ASSERT(err == ERR_OK);

  return result;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::notify(const CaptureComponentParam& param)
{
  bool result = false;

  switch(param.notify_param.type)
    {
      case NtfDmaCmplt:
        {
          if (param.notify_param.code == E_AS_DMA_INT_ERR)
            {
              if (m_state != Ready)
                {
                  m_state = Error;
                }

              /* If DMA ERRINT occured, following capture data
               * will not come, so reply to all of request here.  
               */

              while (!m_req_data_que.empty())
                {
                  CaptureDataParam capresult;

                  capresult.output_device = m_output_device;
                  capresult.end_flag      = false;
                  capresult.buf           = m_req_data_que.top();
                  capresult.buf.sample    = 0;
                  capresult.buf.validity  = false;

                  m_callback(capresult);

                  if (!m_req_data_que.pop())
                    {
                      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
                      return false;
                    }
                }
            }

          if (E_AS_OK != AS_NotifyDmaCmplt
                                  (m_dmac_id,
                                   param.notify_param.code))
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

/*--------------------------------------------------------------------*/
CaptureBuffer CaptureComponent::getCapBuf(uint32_t cap_sample)
{
  /* Allocate memory for capture, and push to request que */

  CaptureBuffer buf;


  if (buf.cap_mh.allocSeg(m_mem_pool_id,
                          cap_sample * m_ch_num * 2)
      != ERR_OK)
    {
      CAPTURE_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
    }

  buf.sample = cap_sample;

  return buf;
}

/*--------------------------------------------------------------------*/
bool CaptureComponent::enqueDmaReqQue(CaptureBuffer buf)
{
  if (!m_req_data_que.push(buf))
    {
      CAPTURE_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  return true;
}

__WIEN2_END_NAMESPACE

