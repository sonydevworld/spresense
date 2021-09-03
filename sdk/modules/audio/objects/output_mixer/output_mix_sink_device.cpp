/****************************************************************************
 * modules/audio/objects/output_mixer/output_mix_sink_device.cpp
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
 * Included Files
 ****************************************************************************/

#include "output_mix_sink_device.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BYTE_SIZE_PER_SAMPLE_HIGHRES 8    /* Number of bytes per sample that
                                           * case of high res.
                                           * 8 == (sizeof(int32_t)x2ch)
                                           */
#define BYTE_SIZE_PER_SAMPLE         4    /* Number of bytes per sample
                                           * 4 == (sizeof(int16_t)x2ch)
                                           */
#define DMA_MIN_SAMPLE               240  /* DMA minimum Samples. */
#define DMA_MAX_SAMPLE               1024 /* DMA maximum Samples. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void send_renderer(RenderComponentHandler handle,
                          void *p_addr,
                          uint32_t byte_size,
                          int8_t adjust,
                          bool is_valid,
                          uint8_t bit_length);
static bool check_sample(AsPcmDataParam* data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static bool postfilter_done_callback(ComponentCbParam *p_param,
                                     void* p_requester)
{
  err_t er;
  OutputMixObjParam outmix_param;
  outmix_param.handle =
    (static_cast<OutputMixToHPI2S*>(p_requester))->m_self_handle;
  outmix_param.postfilterdone_param.event_type  = p_param->event_type;
  outmix_param.postfilterdone_param.result      = p_param->result;

  er = MsgLib::send<OutputMixObjParam>((static_cast<OutputMixToHPI2S*>
                                        (p_requester))->m_self_msgq_id,
                                       MsgPriNormal,
                                       MSG_AUD_MIX_RST_PSTFLT_DONE,
                                       NULL,
                                       outmix_param);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
static void render_done_callback(FAR AudioDrvDmaResult *p_param,
                                 void* p_requester)
{
  err_t er;
  OutputMixObjParam outmix_param;
  outmix_param.handle =
    (static_cast<OutputMixToHPI2S*>(p_requester))->m_self_handle;
  outmix_param.renderdone_param.end_flag = p_param->endflg;
  outmix_param.renderdone_param.error_flag = false;

  er = MsgLib::send<OutputMixObjParam>((static_cast<OutputMixToHPI2S*>
                                        (p_requester))->m_self_msgq_id,
                                       MsgPriNormal,
                                       MSG_AUD_MIX_RST_RENDER_DONE,
                                       NULL,
                                       outmix_param);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
static void render_err_callback(FAR AudioDrvDmaError *p_param,
                                void* p_requester)
{
  err_t er;
  OutputMixObjParam outmix_param;
  outmix_param.handle =
    (static_cast<OutputMixToHPI2S*>(p_requester))->m_self_handle;
  outmix_param.renderdone_param.end_flag = false;
  outmix_param.renderdone_param.error_flag = true;

  er = MsgLib::send<OutputMixObjParam>((static_cast<OutputMixToHPI2S*>
                                        (p_requester))->m_self_msgq_id,
                                       MsgPriNormal,
                                       MSG_AUD_MIX_RST_RENDER_DONE,
                                       NULL,
                                       outmix_param);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
/* Methods of OutputMixToHPI2S class */
/*--------------------------------------------------------------------------*/
OutputMixToHPI2S::MsgProc OutputMixToHPI2S::MsgProcTbl[AUD_MIX_MSG_NUM][StateNum] =
{
  /* Message type: ACT   */

  {                                           /* OutputMixToHPI2S State: */
    &OutputMixToHPI2S::act,                   /*   Booted                */
    &OutputMixToHPI2S::illegal,               /*   Ready                 */
    &OutputMixToHPI2S::illegal,               /*   Active                */
    &OutputMixToHPI2S::illegal,               /*   Stopping              */
    &OutputMixToHPI2S::illegal                /*   Underflow             */
  },

  /* Message type: DEACT */

  {                                           /* OutputMixToHPI2S State: */
    &OutputMixToHPI2S::illegal,               /*  Booted                 */
    &OutputMixToHPI2S::deact,                 /*  Ready                  */
    &OutputMixToHPI2S::illegal,               /*  Active                 */
    &OutputMixToHPI2S::illegal,               /*  Stopping               */
    &OutputMixToHPI2S::illegal                /*  Underflow              */
  },

  /* Message type: INIT  */

  {                                           /* OutputMixToHPI2S State: */
    &OutputMixToHPI2S::illegal,               /*  Booted                 */
    &OutputMixToHPI2S::init,                  /*  Ready                  */
    &OutputMixToHPI2S::illegal,               /*  Active                 */
    &OutputMixToHPI2S::illegal,               /*  Stopping               */
    &OutputMixToHPI2S::illegal                /*  Underflow              */
  },

  /* Message type: Data  */

  {                                           /* OutputMixToHPI2S State: */
    &OutputMixToHPI2S::illegal,               /*  Booted                 */
    &OutputMixToHPI2S::input_data_on_ready,   /*  Ready                  */
    &OutputMixToHPI2S::input_data_on_active,  /*  Active                 */
    &OutputMixToHPI2S::illegal,               /*  Stopping               */
    &OutputMixToHPI2S::input_data_on_under    /*  Underflow              */
  },


  /* Message type: CLKRECOVERY */
  {                                           /* OutputMixToHPI2S State: */
    &OutputMixToHPI2S::illegal,               /*  Booted                 */
    &OutputMixToHPI2S::clock_recovery,        /*  Ready                  */
    &OutputMixToHPI2S::clock_recovery,        /*  Active                 */
    &OutputMixToHPI2S::clock_recovery,        /*  Stopping               */
    &OutputMixToHPI2S::clock_recovery         /*  Underflow              */
  },

  /* Message type: INIT PostProc */
  {                                           /* OutputMixToHPI2S State: */
    &OutputMixToHPI2S::illegal,               /*  Booted                 */
    &OutputMixToHPI2S::init_postproc,         /*  Ready                  */
    &OutputMixToHPI2S::init_postproc,         /*  Active                 */
    &OutputMixToHPI2S::init_postproc,         /*  Stopping               */
    &OutputMixToHPI2S::init_postproc,         /*  Underflow              */
  },

  /* Message type: SET PostProc */
  {                                           /* OutputMixToHPI2S State: */
    &OutputMixToHPI2S::illegal,               /*  Booted                 */
    &OutputMixToHPI2S::set_postproc,          /*  Ready                  */
    &OutputMixToHPI2S::set_postproc,          /*  Active                 */
    &OutputMixToHPI2S::set_postproc,          /*  Stopping               */
    &OutputMixToHPI2S::set_postproc,          /*  Underflow              */
  },
};

OutputMixToHPI2S::MsgProc OutputMixToHPI2S::MsgRsltTbl[AUD_MIX_RST_MSG_NUM][StateNum] =
{
  /* Message type: PFDONE */

  {                                           /* OutputMixToHPI2S State: */
    &OutputMixToHPI2S::illegal_done,          /*  Booted                 */
    &OutputMixToHPI2S::postdone_on_ready,     /*  Ready                  */
    &OutputMixToHPI2S::postdone_on_active,    /*  Active                 */
    &OutputMixToHPI2S::postdone_on_stopping,  /*  Stopping               */
    &OutputMixToHPI2S::postdone_on_underflow  /*  Underflow              */
  },

  /* Message type: DONE */

  {                                           /* OutputMixToHPI2S State: */
    &OutputMixToHPI2S::illegal_done,          /*  Booted                 */
    &OutputMixToHPI2S::illegal_done,          /*  Ready                  */
    &OutputMixToHPI2S::done_on_active,        /*  Active                 */
    &OutputMixToHPI2S::done_on_stopping,      /*  Stopping               */
    &OutputMixToHPI2S::illegal_done           /*  Underflow              */
  },
};


/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::parse(MsgPacket* msg)
{
  uint event = MSG_GET_SUBTYPE(msg->getType());
  if (MSG_IS_REQUEST(msg->getType()) == 0)
    {
       F_ASSERT(event < AUD_MIX_RST_MSG_NUM);
       (this->*MsgRsltTbl[event][m_state.get()])(msg);
    }
  else
    {
       F_ASSERT(event < AUD_MIX_MSG_NUM);
       (this->*MsgProcTbl[event][m_state.get()])(msg);
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::reply(MsgQueId requester_msgq_id,
                             MsgType msg_type,
                             AsOutputMixDoneParam *done_param)
{
  if (m_callback != NULL)
    {
      m_callback(requester_msgq_id, msg_type, done_param);
    }
  else if (m_requester_msgq_id != MSG_QUE_NULL)
    {
      AudioObjReply cmplt((uint32_t)msg_type,
                           AS_OBJ_REPLY_TYPE_REQ,
                           AS_MODULE_ID_OUTPUT_MIX_OBJ,
                           done_param->ecode);
      err_t er = MsgLib::send<AudioObjReply>(requester_msgq_id,
                                             MsgPriNormal,
                                             MSG_TYPE_AUD_RES,
                                             NULL,
                                             cmplt);
      if (ERR_OK != er)
        {
          F_ASSERT(0);
        }
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::illegal(MsgPacket* msg)
{
  OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);

  uint event = msg->getType();

  /* Extract and abandon message data
   * ! Each message size is not same !
   */

  switch (event)
    {
      case MSG_AUD_MIX_CMD_ACT:
      case MSG_AUD_MIX_CMD_INIT:
      case MSG_AUD_MIX_CMD_DEACT:
      case MSG_AUD_MIX_CMD_CLKRECOVERY:
      case MSG_AUD_MIX_CMD_INITMPP:
      case MSG_AUD_MIX_CMD_SETMPP:
        msg->moveParam<OutputMixerCommand>();
        break;

      case MSG_AUD_MIX_RST_RENDER_DONE:
        msg->moveParam<OutputMixObjParam>();
        break;

      case MSG_AUD_MIX_CMD_DATA:
        msg->moveParam<AsPcmDataParam>();

      default:
        break;
    }

  return;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::act(MsgPacket* msg)
{
  AsOutputMixDoneParam done_param;
  OutputMixerCommand cmd = msg->moveParam<OutputMixerCommand>();

  m_requester_msgq_id = msg->getReply();

  /* Initialize reply value */

  done_param.handle    = cmd.handle;
  done_param.done_type = OutputMixActDone;
  done_param.result    = true;
  done_param.ecode     = AS_ECODE_OK;

  OUTPUT_MIX_DBG("ACT: dev %d, type %d\n",
                 cmd.act_param.output_device,
                 cmd.act_param.mixer_type);

  if (!checkMemPool())
    {
      done_param.result = false;
      done_param.ecode  = AS_ECODE_CHECK_MEMORY_POOL_ERROR;

      reply(m_requester_msgq_id, MSG_AUD_MIX_CMD_ACT, &done_param);
      return;
    }

  switch(cmd.act_param.mixer_type)
    {
      case MainOnly:
        /* Create render component for rendering main sound. */

        {
          RenderDevice render_device;
          if (cmd.act_param.output_device == HPOutputDevice)
            {
              render_device = RenderDeviceHPSP;
            }
          else
            {
              render_device = RenderDeviceI2S;
            }

          if (!AS_get_render_comp_handler(&m_render_comp_handler,
                                          render_device))
            {
              OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
              return;
            }
        }
        break;

      default:
        OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return;
    }

  /* Set handle */

  m_self_handle = cmd.handle;

  /* Set callback function */

  m_callback = cmd.act_param.cb;

  /* Set error callback function */

  m_error_callback = cmd.act_param.error_cb;

  /* Activation of component */

  char filepath[64];
  snprintf(filepath, sizeof(filepath), "%s/POSTPROC", CONFIG_AUDIOUTILS_DSP_MOUNTPT);

  uint32_t ret =loadComponent((cmd.act_param.post_enable == PostFilterEnable)
                                ? AsPostprocTypeUserCustom : AsPostprocTypeThrough,
                              filepath);

  if(ret != AS_ECODE_OK)
    {
      done_param.result = false;
      done_param.ecode  = ret;

      reply(m_requester_msgq_id, MSG_AUD_MIX_CMD_ACT, &done_param);
      return;
    }

  /* Reply "Success" */

  reply(m_requester_msgq_id, MSG_AUD_MIX_CMD_ACT, &done_param);

  m_state = Ready;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::init(MsgPacket* msg)
{
  AsOutputMixDoneParam done_param;
  OutputMixerCommand cmd = msg->moveParam<OutputMixerCommand>();

  OUTPUT_MIX_DBG("INIT: post_type %d, dsp %s\n",
                 cmd.init_param.postproc_type,
                 cmd.init_param.dsp_path);

  /* Initialize reply value */

  done_param.handle    = cmd.handle;
  done_param.done_type = OutputMixInitDone;
  done_param.result    = true;
  done_param.ecode     = AS_ECODE_OK;

  /* Check component load */

  if ((m_postproc_type != cmd.init_param.postproc_type)
   || (strncmp(m_dsp_path, cmd.init_param.dsp_path, sizeof(m_dsp_path))))
    {
      uint32_t ret = unloadComponent();

      if (ret != AS_ECODE_OK)
        {
          /* Error reply */

          done_param.result    = false;
          done_param.ecode     = ret;

          reply(m_requester_msgq_id, MSG_AUD_MIX_CMD_INIT, &done_param);
          return;
        }

      ret = loadComponent(static_cast<AsPostprocType>(cmd.init_param.postproc_type),
                          cmd.init_param.dsp_path);

      if(ret != AS_ECODE_OK)
        {
          /* Error reply */

          done_param.result    = false;
          done_param.ecode     = ret;

          reply(m_requester_msgq_id, MSG_AUD_MIX_CMD_INIT, &done_param);
          return;
        }
    }

  /* Reply "Success" */

  reply(m_requester_msgq_id, MSG_AUD_MIX_CMD_INIT, &done_param);
}

/*--------------------------------------------------------------------------*/
uint32_t OutputMixToHPI2S::loadComponent(AsPostprocType type, char *dsp_path)
{
  /* Set component */

  switch (type)
    {
      case AsPostprocTypeUserCustom:
        m_p_postfliter_instance = &m_usercstm_instance;
        break;

      default:
        m_p_postfliter_instance = &m_thruproc_instance;
        break;
    }

  /* If component cannot be set, reply error. */

  if (m_p_postfliter_instance == NULL)
    {
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  /* Activate component */

  uint32_t dsp_inf = 0;

  uint32_t ret = m_p_postfliter_instance->activate(postfilter_done_callback,
                                                   dsp_path,
                                                   static_cast<void *>(this),
                                                   &dsp_inf);

  if (ret != AS_ECODE_OK)
    {
      m_p_postfliter_instance = NULL;

      return ret;
    }

  /* Hold postproc type */

  m_postproc_type = type;

  /* Hold dsp_path */

  strncpy(m_dsp_path, dsp_path, sizeof(m_dsp_path) - 1);
  m_dsp_path[sizeof(m_dsp_path) -1] = '\0';

  /* return success */

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t OutputMixToHPI2S::unloadComponent(void)
{
  /* If there is no component to deactivate, return OK. */

  if (m_p_postfliter_instance == NULL)
    {
      return AS_ECODE_OK;
    }

  /* Deactivate post proc */

  bool ret = m_p_postfliter_instance->deactivate();

  if (!ret)
    {
      return AS_ECODE_DSP_UNLOAD_ERROR;
    }

  m_p_postfliter_instance = NULL;

  /* When unload complete successfully, set invalid type and dsp_name. */

  m_postproc_type = AsPostprocTypeInvalid;

  memset(m_dsp_path, 0, sizeof(m_dsp_path));

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::deact(MsgPacket* msg)
{
  AsOutputMixDoneParam done_param;
  uint8_t handle = msg->moveParam<OutputMixerCommand>().handle;

  done_param.handle    = handle;
  done_param.done_type = OutputMixDeactDone;
  done_param.result    = true;
  done_param.ecode     = AS_ECODE_OK;

  OUTPUT_MIX_DBG("DEACT:\n");

  if (!AS_release_render_comp_handler(m_render_comp_handler))
    {
      return;
    }

  uint32_t ret = unloadComponent();

  if (ret != AS_ECODE_OK)
    {
      done_param.result = false;
      done_param.ecode  = ret;
    }

  /* Replay */

  reply(m_requester_msgq_id, MSG_AUD_MIX_CMD_DEACT, &done_param);

  m_self_handle = 0;
  m_state       = Booted;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::input_data_on_ready(MsgPacket* msg)
{
  AsPcmDataParam input =
    msg->moveParam<AsPcmDataParam>();

  /* Exec postfilter */

  ExecComponentParam exec;

  exec.input     = input;
  exec.output    = input.mh;

  if (!m_p_postfliter_instance->exec(exec))
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  /* Init renderer */

  if (!AS_init_renderer(m_render_comp_handler,
                        &render_done_callback,
                        &render_err_callback,
                        static_cast<void*>(this),
                        input.bit_length))
    {
      return;
    }

  m_state = Active;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::input_data_on_active(MsgPacket* msg)
{
  AsPcmDataParam input =
    msg->moveParam<AsPcmDataParam>();

  /* Exec postfilter */

  ExecComponentParam exec;

  exec.input     = input;
  exec.output    = input.mh;

  if (!m_p_postfliter_instance->exec(exec))
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  /* If last frame, send flush command */

  if (input.is_end)
    {
      FlushComponentParam flush_param;

      if (ERR_OK != flush_param.output.allocSeg(m_pcm_pool_id, m_max_pcm_buff_size))
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
        }

      if (!m_p_postfliter_instance->flush(flush_param))
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
          return;
        }
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::postdone_on_ready(MsgPacket* msg)
{
  /* On Ready state, reply of SET command may come. */

  postdone_on_active(msg);
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::postdone_on_active(MsgPacket* msg)
{
  OutputMixObjPostfilterDoneCmd post_done =
    msg->moveParam<OutputMixObjParam>().postfilterdone_param;

  /* If it is not return of Exec of Flush, no need to rendering. */

  if (!(post_done.event_type == ComponentExec)
   && !(post_done.event_type == ComponentFlush))
    {
      m_p_postfliter_instance->recv_done();
      return;
    }

  /* Get postfilter result */

  ComponentCmpltParam cmplt;

  m_p_postfliter_instance->recv_done(&cmplt);

  /* Check minimum trans size and send filtered data to renderer */

  if (check_sample(&cmplt.output) && cmplt.result)
    {
      send_renderer(m_render_comp_handler,
                    cmplt.output.mh.getPa(),
                    cmplt.output.size,
                    get_period_adjustment(),
                    cmplt.output.is_valid,
                    cmplt.output.bit_length);

      if (!m_render_data_queue.push(cmplt.output))
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return;
        }
    }

  /* If flust event done, stop renderer */

  if (post_done.event_type == ComponentFlush)
    {
      if (!AS_stop_renderer(m_render_comp_handler, AS_DMASTOPMODE_NORMAL))
        {
          return;
        }

      m_state = Stopping;
    }

  return;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::postdone_on_stopping(MsgPacket* msg)
{
  postdone_on_active(msg);
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::postdone_on_underflow(MsgPacket* msg)
{
  postdone_on_active(msg);
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::input_data_on_under(MsgPacket* msg)
{
  AsPcmDataParam input =
    msg->moveParam<AsPcmDataParam>();

  /* If end-data, publish render stop */

  if (input.is_end)
    {
      if (!m_render_data_queue.push(input))
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return;
        }

      AS_stop_renderer(m_render_comp_handler, AS_DMASTOPMODE_NORMAL);

      OutputMixObjParam param;
      param.renderdone_param.end_flag = true;
      m_render_data_queue.top().callback(m_self_handle,
                                         param.renderdone_param.end_flag);

      if (!m_render_data_queue.pop())
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
          return;
        }

      m_state = Ready;
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::illegal_done(MsgPacket* msg)
{
  msg->moveParam<OutputMixObjParam>();
  return;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::done_on_active(MsgPacket* msg)
{
  OutputMixObjParam param = msg->moveParam<OutputMixObjParam>();

  if (param.renderdone_param.error_flag)
    {
      m_error_callback(m_self_handle);
      m_state = Underflow;
      return;
    }

  if (param.renderdone_param.end_flag)
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }

  if (m_render_data_queue.empty())
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      return;
    }

  /* Reply */

  m_render_data_queue.top().callback(m_self_handle,
                                     param.renderdone_param.end_flag);

  if (!m_render_data_queue.pop())
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return;
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::done_on_stopping(MsgPacket* msg)
{
  OutputMixObjParam param = msg->moveParam<OutputMixObjParam>();

  if (m_render_data_queue.empty())
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      return;
    }

  /* Reply */

  m_render_data_queue.top().callback(m_self_handle,
                                     param.renderdone_param.end_flag);

  if (!m_render_data_queue.pop())
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return;
    }

  if (m_render_data_queue.empty())
    {
      if (param.renderdone_param.end_flag)
        {
          m_state = Ready;
        }
      else
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        }
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::clock_recovery(MsgPacket* msg)
{
  OutputMixerCommand cmd =
    msg->moveParam<OutputMixerCommand>();

  OUTPUT_MIX_DBG("CLOCK RECOVERY: dir %d, times %ld\n",
                 cmd.fterm_param.direction, cmd.fterm_param.times);

  /* Check Paramete. */

  if (cmd.fterm_param.direction < OutputMixAdvance
   || OutputMixDelay < cmd.fterm_param.direction)
    {
      return;
    }

  /* Set recovery parameters. */

  m_adjust_direction = cmd.fterm_param.direction;
  m_adjustment_times = cmd.fterm_param.times;

  AsOutputMixDoneParam done_param;

  done_param.handle    = cmd.handle;
  done_param.done_type = OutputMixSetClkRcvDone;
  done_param.result    = true;
  done_param.ecode     = AS_ECODE_OK;

  reply(m_requester_msgq_id, MSG_AUD_MIX_CMD_CLKRECOVERY, &done_param);

  return;
}

/*--------------------------------------------------------------------------*/
int8_t OutputMixToHPI2S::get_period_adjustment(void)
{
  bool do_adjust = false;
  int8_t adjust_sample = 0;

  /* Determine adjust or not. */

  if (m_adjustment_times > 0)
    {
      do_adjust = true;
      m_adjustment_times--;
    }
  else if (m_adjustment_times < 0)
    {
      do_adjust = true;
    }
  else
    {
      do_adjust = false;
    }

  /* Set adjust samples. */

  if (do_adjust)
    {
      if (m_adjust_direction > 0)
        {
          adjust_sample = 1;
        }
      else if (m_adjust_direction < 0)
        {
          adjust_sample = -1;
        }
      else
        {
          /* No adjust work. */
        }
    }

  return adjust_sample;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::init_postproc(MsgPacket* msg)
{
  OUTPUT_MIX_DBG("INIT POSTPROC:\n");

  OutputMixerCommand cmd =
    msg->moveParam<OutputMixerCommand>();

  InitComponentParam param;

  param.is_userdraw = true;
  param.custom.addr = cmd.initpp_param.addr;
  param.custom.size = cmd.initpp_param.size;

  /* Init Postproc (Copy packet to MH internally, and wait return from DSP) */

  uint32_t send_result = m_p_postfliter_instance->init(param);

  ComponentCmpltParam cmplt;
  m_p_postfliter_instance->recv_done(&cmplt);

  /* Reply */

  AsOutputMixDoneParam done_param;

  done_param.handle    = cmd.handle;
  done_param.done_type = OutputMixInitPostDone;
  done_param.result    = (send_result == AS_ECODE_OK) ? true : false;
  done_param.ecode     = send_result;

  reply(m_requester_msgq_id, MSG_AUD_MIX_CMD_INITMPP, &done_param);
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::set_postproc(MsgPacket *msg)
{
  OUTPUT_MIX_DBG("SET POSTPROC:\n");

  OutputMixerCommand cmd =
    msg->moveParam<OutputMixerCommand>();

  SetComponentParam param;

  param.is_userdraw = true;
  param.custom.addr = cmd.setpp_param.addr;
  param.custom.size = cmd.setpp_param.size;

  /* Set Postproc (Copy packet to MH internally) */

  bool send_result = m_p_postfliter_instance->set(param);

  /* Reply (Don't wait reply from DSP because it will take long time) */

  AsOutputMixDoneParam done_param;

  done_param.handle    = cmd.handle;
  done_param.done_type = OutputMixSetPostDone;
  done_param.result    = send_result;
  done_param.ecode     = send_result ? AS_ECODE_OK : AS_ECODE_DSP_SET_ERROR;

  reply(m_requester_msgq_id, MSG_AUD_MIX_CMD_SETMPP, &done_param);
}

/*--------------------------------------------------------------------------*/
bool OutputMixToHPI2S::checkMemPool(void)
{
  if (!MemMgrLite::Manager::isPoolAvailable(m_pcm_pool_id))
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  m_max_pcm_buff_size =
      (MemMgrLite::Manager::getPoolSize(m_pcm_pool_id)) /
      (MemMgrLite::Manager::getPoolNumSegs(m_pcm_pool_id));

  if (!MemMgrLite::Manager::isPoolAvailable(m_apu_pool_id))
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  m_apucmd_pcm_buff_size =
      (MemMgrLite::Manager::getPoolSize(m_apu_pool_id)) /
      (MemMgrLite::Manager::getPoolNumSegs(m_apu_pool_id));

  return true;
}

/*--------------------------------------------------------------------------*/
static void send_renderer(RenderComponentHandler handle,
                          void *p_addr,
                          uint32_t byte_size,
                          int8_t adjust,
                          bool is_valid,
                          uint8_t bit_length)
{
  uint32_t byte_size_per_sample = ((bit_length == AS_BITLENGTH_16) ?
                                   BYTE_SIZE_PER_SAMPLE :
                                   BYTE_SIZE_PER_SAMPLE_HIGHRES);

  /* Insert dummy data. */

  if (adjust > 0)
    {
      memcpy((char*)p_addr + byte_size,
             (char*)p_addr + byte_size - (byte_size_per_sample * adjust),
             byte_size_per_sample * adjust);
    }

  /* Adjust sound period.
   * (Adjust transfer samples of DMA out for clock recovery.)
   */

  byte_size += (adjust * byte_size_per_sample);

  /* Send to renderer. */

  if (!AS_exec_renderer(handle,
                        p_addr,
                        byte_size / byte_size_per_sample,
                        is_valid))
    {
      return;
    }
}

/*--------------------------------------------------------------------------*/
static bool check_sample(AsPcmDataParam *data)
{
  bool res = true;
  uint32_t byte_size_per_sample = ((data->bit_length == AS_BITLENGTH_16) ?
                                   BYTE_SIZE_PER_SAMPLE :
                                   BYTE_SIZE_PER_SAMPLE_HIGHRES);

  if (data->size < DMA_MIN_SAMPLE * byte_size_per_sample)
    {
      res = false;
    }

  return res;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__WIEN2_END_NAMESPACE

