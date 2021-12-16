/****************************************************************************
 * modules/audio/objects/front_end/front_end_obj.cpp
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

#include <stdlib.h>
#include <nuttx/arch.h>
#include <stdlib.h>
#include "front_end_obj.h"
#include "debug/dbg_log.h"

__USING_WIEN2
using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void capture_done_callback(CaptureDataParam param)
{
  err_t er;

  er = MsgLib::send<CaptureDataParam>(MicFrontEndObject::get_msgq_id(),
                                      MsgPriNormal,
                                      MSG_AUD_MFE_RST_CAPTURE_DONE,
                                      NULL,
                                      param);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
static void capture_error_callback(CaptureErrorParam param)
{
  err_t er;

  er = MsgLib::send<CaptureErrorParam>(MicFrontEndObject::get_msgq_id(),
                                       MsgPriNormal,
                                       MSG_AUD_MFE_RST_CAPTURE_ERR,
                                       NULL,
                                       param);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
static bool preproc_done_callback(ComponentCbParam *cmplt, void* p_requester)
{
  MicFrontendObjPreProcDoneCmd param;

  param.event_type = cmplt->event_type;
  param.result     = cmplt->result;

  err_t er = MsgLib::send<MicFrontendObjPreProcDoneCmd>(MicFrontEndObject::get_msgq_id(),
                                                        MsgPriNormal,
                                                        MSG_AUD_MFE_RST_PREPROC,
                                                        NULL,
                                                        param);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
static void pcm_send_done_callback(int32_t identifier, bool is_end)
{
}

/*--------------------------------------------------------------------------*/
FAR void AS_MicFrontendObjEntry(FAR void *arg)
{
  MicFrontEndObject::create((AsObjectParams_t *)arg);
}

/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
MicFrontEndObject::MsgProc
  MicFrontEndObject::MsgParamTbl[AUD_MFE_PRM_NUM][StateNum] =
{
  /* Message Type: MSG_AUD_MFE_CMD_INITPREPROC */
  {                                   /* MicFrontend status: */
    &MicFrontEndObject::illegal,         /*   Booted.        */
    &MicFrontEndObject::initPreproc,     /*   Ready.         */
    &MicFrontEndObject::illegal,         /*   PreActive.     */
    &MicFrontEndObject::illegal,         /*   Active.        */
    &MicFrontEndObject::illegal,         /*   Stopping.      */
    &MicFrontEndObject::illegal,         /*   ErrorStopping. */
    &MicFrontEndObject::illegal          /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_MFE_CMD_SETPREPROC */

  {                                   /* MicFrontend status: */
    &MicFrontEndObject::illegal,         /*   Booted.        */
    &MicFrontEndObject::setPreproc,      /*   Ready.         */
    &MicFrontEndObject::illegal,         /*   PreActive.     */
    &MicFrontEndObject::setPreproc,      /*   Active.        */
    &MicFrontEndObject::illegal,         /*   Stopping.      */
    &MicFrontEndObject::illegal,         /*   ErrorStopping. */
    &MicFrontEndObject::illegal          /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_MFE_CMD_SET_MICGAIN */

  {                                   /* MicFrontend status: */
    &MicFrontEndObject::illegal,         /*   Booted.        */
    &MicFrontEndObject::setMicGain,      /*   Ready.         */
    &MicFrontEndObject::illegal,         /*   PreActive.     */
    &MicFrontEndObject::setMicGain,      /*   Active.        */
    &MicFrontEndObject::illegal,         /*   Stopping.      */
    &MicFrontEndObject::illegal,         /*   ErrorStopping. */
    &MicFrontEndObject::illegal          /*   WaitStop.      */
  }
};

/*--------------------------------------------------------------------------*/
MicFrontEndObject::MsgProc
  MicFrontEndObject::RsltProcTbl[AUD_MFE_RST_MSG_NUM][StateNum] =
{
  /* Message Type: MSG_AUD_MFE_RST_CAPTURE_DONE. */

  {                                          /* MicFrontend status: */
    &MicFrontEndObject::illegalCaptureDone,     /*   Booted.        */
    &MicFrontEndObject::illegalCaptureDone,     /*   Ready.         */
    &MicFrontEndObject::illegalCaptureDone,     /*   PreActive.     */
    &MicFrontEndObject::captureDoneOnActive,    /*   Active.        */
    &MicFrontEndObject::captureDoneOnStop,      /*   Stopping.      */
    &MicFrontEndObject::captureDoneOnErrorStop, /*   ErrorStopping. */
    &MicFrontEndObject::captureDoneOnWaitStop   /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_MFE_RST_CAPTURE_ERR. */

  {                                           /* MicFrontend status: */
    &MicFrontEndObject::illegalCaptureError,     /*   Booted.        */
    &MicFrontEndObject::illegalCaptureError,     /*   Ready.         */
    &MicFrontEndObject::illegalCaptureError,     /*   PreActive      */
    &MicFrontEndObject::captureErrorOnActive,    /*   Active.        */
    &MicFrontEndObject::captureErrorOnStop,      /*   Stopping.      */
    &MicFrontEndObject::captureErrorOnErrorStop, /*   ErrorStopping. */
    &MicFrontEndObject::captureErrorOnWaitStop   /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_MFE_RST_PREPROC. */

  {                                           /* MicFrontend status: */
    &MicFrontEndObject::illegalPreprocDone,      /*   Booted.        */
    &MicFrontEndObject::illegalPreprocDone,      /*   Ready.         */
    &MicFrontEndObject::illegalPreprocDone,      /*   PreActive.     */
    &MicFrontEndObject::preprocDoneOnActive,     /*   Active.        */
    &MicFrontEndObject::preprocDoneOnStop,       /*   Stopping.      */
    &MicFrontEndObject::preprocDoneOnErrorStop,  /*   ErrorStopping. */
    &MicFrontEndObject::preprocDoneOnWaitStop    /*   WaitStop.      */
  }
};

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::parseResult(MsgPacket *msg)
{
  uint32_t event = MSG_GET_SUBTYPE(msg->getType());
  F_ASSERT((event < AUD_MFE_RST_MSG_NUM));

  (this->*RsltProcTbl[event][m_state.get()])(msg);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::set(MsgPacket *msg)
{
  uint32_t event;
  event = MSG_GET_PARAM(msg->getType());
  F_ASSERT((event < AUD_MFE_PRM_NUM));

  (this->*MsgParamTbl[event][m_state.get()])(msg);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::reply(AsMicFrontendEvent evtype,
                              MsgType msg_type,
                              uint32_t result)
{
  if (m_callback != NULL)
    {
      m_callback(evtype, result, 0);
    }
  else if (m_msgq_id.from != MSG_QUE_NULL)
    {
      AudioObjReply cmplt((uint32_t)msg_type,
                           AS_OBJ_REPLY_TYPE_REQ,
                           AS_MODULE_ID_MIC_FRONTEND_OBJ,
                           result);
      err_t er = MsgLib::send<AudioObjReply>(m_msgq_id.from,
                                             MsgPriNormal,
                                             MSG_TYPE_AUD_RES,
                                             m_msgq_id.self,
                                             cmplt);
      if (ERR_OK != er)
        {
          F_ASSERT(0);
        }
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::illegal(MsgPacket *msg)
{
  uint msgtype = msg->getType();
  msg->moveParam<MicFrontendCommand>();

  uint32_t idx = msgtype - MSG_AUD_MFE_CMD_ACT;

  AsMicFrontendEvent table[] =
  {
    AsMicFrontendEventAct,
    AsMicFrontendEventDeact,
    AsMicFrontendEventInit,
    AsMicFrontendEventStart,
    AsMicFrontendEventStop,
    AsMicFrontendEventInitPreProc,
    AsMicFrontendEventSetPreProc,
    AsMicFrontendEventSetMicGain
  };

  reply(table[idx], msg->getType(), AS_ECODE_STATE_VIOLATION);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::activateOnBooted(MsgPacket *msg)
{
  uint32_t rst;
  AsActivateMicFrontend act = msg->moveParam<MicFrontendCommand>().act_param;

  MIC_FRONTEND_DBG("ACT: indev %d\n", act.param.input_device);

  /* Set event callback */

  m_callback = act.cb;

  /* Parameter check */

  if (!checkAndSetMemPool())
    {
      reply(AsMicFrontendEventAct,
            msg->getType(),
            AS_ECODE_CHECK_MEMORY_POOL_ERROR);
      return;
    }

  rst = activateParamCheck(act.param);

  if (rst != AS_ECODE_OK)
    {
      reply(AsMicFrontendEventAct, msg->getType(), rst);
      return;
    }

  /* State transit */

  m_state = Ready;

  /* Reply */

  reply(AsMicFrontendEventAct, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::deactivateOnReady(MsgPacket *msg)
{
  msg->moveParam<MicFrontendCommand>();

  MIC_FRONTEND_DBG("DEACT:\n");

  if (!delInputDeviceHdlr())
    {
      reply(AsMicFrontendEventDeact,
            msg->getType(),
            AS_ECODE_CLEAR_AUDIO_DATA_PATH_ERROR);
      return;
    }

  /* Deactivate PreProcess */

  uint32_t ret = unloadComponent();
  if (ret != AS_ECODE_OK)
    {
      /* Error reply */

      reply(AsMicFrontendEventDeact, msg->getType(), ret);

      return;
    }

  /* State transit */

  m_state = Booted;

  /* Reply */

  reply(AsMicFrontendEventDeact, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::initOnReady(MsgPacket *msg)
{
  uint32_t rst = AS_ECODE_OK;
  MicFrontendCommand cmd = msg->moveParam<MicFrontendCommand>();

  MIC_FRONTEND_DBG("INIT: ch num %d, bit len %d, samples/frame %d\n",
                 cmd.init_param.channel_number,
                 cmd.init_param.bit_length,
                 cmd.init_param.samples_per_frame);

  /* Check parameters */

  rst = initParamCheck(cmd);
  if (rst != AS_ECODE_OK)
    {
      reply(AsMicFrontendEventInit, msg->getType(), rst);
      return;
    }

  /* Hold parameters */

  m_channel_num       = cmd.init_param.channel_number;
  m_pcm_bit_width     = cmd.init_param.bit_length;
  m_cap_bytes         = ((m_pcm_bit_width == AS_BITLENGTH_16) ? 2 : 4);
  m_samples_per_frame = cmd.init_param.samples_per_frame;
  m_pcm_data_path     = static_cast<AsMicFrontendDataPath>(cmd.init_param.data_path);
  m_pcm_data_dest     = cmd.init_param.dest;

  /* Release and get capture (Input device) handler */

  if (!delInputDeviceHdlr())
    {
      reply(AsMicFrontendEventInit, msg->getType(), AS_ECODE_CLEAR_AUDIO_DATA_PATH_ERROR);
      return;
    }

  if (!getInputDeviceHdlr())
    {
      reply(AsMicFrontendEventInit, msg->getType(), AS_ECODE_SET_AUDIO_DATA_PATH_ERROR);
      return;
    }

  /* Init capture */

  CaptureComponentParam cap_comp_param;

  cap_comp_param.init_param.capture_ch_num    = m_channel_num;
  cap_comp_param.init_param.capture_bit_width = m_pcm_bit_width;
  cap_comp_param.init_param.preset_num        = CAPTURE_PRESET_NUM;
  cap_comp_param.init_param.callback          = capture_done_callback;
  cap_comp_param.init_param.err_callback      = capture_error_callback;
  cap_comp_param.handle                       = m_capture_hdlr;

  if (!AS_init_capture(&cap_comp_param))
    {
      reply(AsMicFrontendEventStart, msg->getType(), AS_ECODE_DMAC_INITIALIZE_ERROR);
      return;
    }

  /* Check type and dsp name. If differ from current of them, do reload. */

  if ((m_preproc_type != static_cast<AsMicFrontendPreProcType>(cmd.init_param.preproc_type))
   || (strncmp(m_dsp_path, cmd.init_param.dsp_path, sizeof(m_dsp_path))))
    {
      uint32_t ret = unloadComponent();

      if (ret != AS_ECODE_OK)
        {
          /* Error reply */

          reply(AsMicFrontendEventInit, msg->getType(), ret);
          return;
        }

      ret = loadComponent(static_cast<AsMicFrontendPreProcType>(cmd.init_param.preproc_type),
                          cmd.init_param.dsp_path);

      if(ret != AS_ECODE_OK)
        {
          /* Error reply */

          reply(AsMicFrontendEventInit, msg->getType(), ret);
          return;
        }
    }

  /* Init Samplint Rate Converter. */

  if (m_preproc_type == AsMicFrontendPreProcSrc)
    {
      InitComponentParam init_src_param;

      init_src_param.common.samples       = cmd.init_param.samples_per_frame;
      init_src_param.common.in_fs         =
        (cxd56_audio_get_clkmode() == CXD56_AUDIO_CLKMODE_HIRES)
          ? AS_SAMPLINGRATE_192000 : AS_SAMPLINGRATE_48000;
      init_src_param.common.out_fs        = cmd.init_param.out_fs;
      init_src_param.common.in_bitlength  = cmd.init_param.bit_length;
      init_src_param.common.out_bitlength = cmd.init_param.bit_length;
      init_src_param.common.ch_num        = cmd.init_param.channel_number;

      m_p_preproc_instance->init(init_src_param);
      m_p_preproc_instance->recv_done();
    }

  /* Reply */

  reply(AsMicFrontendEventInit, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
uint32_t MicFrontEndObject::loadComponent(AsMicFrontendPreProcType type, char *dsp_path)
{
  /* Create component */

  switch (type)
    {
      case AsMicFrontendPreProcUserCustom:
        m_p_preproc_instance = new UserCustomComponent(m_pool_id.cmp,
                                                       m_msgq_id.cmp);
        break;

      case AsMicFrontendPreProcSrc:
        m_p_preproc_instance = new SRCComponent(m_pool_id.cmp,
                                                m_msgq_id.cmp);
        break;

      default:
        m_p_preproc_instance = new ThruProcComponent();
        break;
    }

  if (m_p_preproc_instance == NULL)
    {
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  /* Activate proprocess proc */

  uint32_t dsp_inf = 0;

  uint32_t ret = m_p_preproc_instance->activate(preproc_done_callback,
                                                dsp_path,
                                                static_cast<void *>(this),
                                                &dsp_inf);
  if (ret != AS_ECODE_OK)
    {
      delete m_p_preproc_instance;

      m_p_preproc_instance = NULL;

      return ret;
    }

  /* Hold preproc type */

  m_preproc_type = type;

  strncpy(m_dsp_path, dsp_path, sizeof(m_dsp_path) - 1);
  m_dsp_path[sizeof(m_dsp_path) -1] = '\0';

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t MicFrontEndObject::unloadComponent(void)
{
  if (m_p_preproc_instance == NULL)
    {
      return AS_ECODE_OK;
    }

  /* Deactivate recognition proc */

  bool ret = m_p_preproc_instance->deactivate();

  if (!ret)
    {
      return AS_ECODE_DSP_UNLOAD_ERROR;
    }

  delete m_p_preproc_instance;

  m_p_preproc_instance = NULL;

  /* When unload complete successfully, set invalid type and dsp_name. */

  m_preproc_type = AsMicFrontendPreProcInvalid;

  memset(m_dsp_path, 0, sizeof(m_dsp_path));

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::startOnReady(MsgPacket *msg)
{
  msg->moveParam<MicFrontendCommand>();

  MIC_FRONTEND_DBG("START:\n");

  /* Start capture */

  if (!startCapture())
    {
      reply(AsMicFrontendEventStart, msg->getType(), AS_ECODE_DMAC_READ_ERROR);
      return;
    }

  /* State transit */

  m_state = Active;

  /* Reply*/

  reply(AsMicFrontendEventStart, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::stopOnActive(MsgPacket *msg)
{
  msg->moveParam<MicFrontendCommand>();

  MIC_FRONTEND_DBG("STOP:\n");

  if (!setExternalCmd(AsMicFrontendEventStop))
    {
      reply(AsMicFrontendEventStop, msg->getType(), AS_ECODE_QUEUE_OPERATION_ERROR);
      return;
    }

  /* Stop DMA transfer. */

  CaptureComponentParam cap_comp_param;

  cap_comp_param.handle          = m_capture_hdlr;
  cap_comp_param.stop_param.mode = AS_DMASTOPMODE_NORMAL;

  AS_stop_capture(&cap_comp_param);

  /* State transit */

  m_state = Stopping;
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::stopOnErrorStopping(MsgPacket *msg)
{
  msg->moveParam<MicFrontendCommand>();

  MIC_FRONTEND_DBG("STOP:\n");

  /* If STOP command received during internal error stop sequence, enque command.
   * It will be trigger of transition to Ready when all of preproc is done.
   */

   if (!setExternalCmd(AsMicFrontendEventStop))
     {
       reply(AsMicFrontendEventStop, msg->getType(), AS_ECODE_QUEUE_OPERATION_ERROR);
       return;
     }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::stopOnWaitStop(MsgPacket *msg)
{
  msg->moveParam<MicFrontendCommand>();

  MIC_FRONTEND_DBG("STOP:\n");

  if (!m_capture_req && !m_preproc_req)
    {
      /* If all of capture and preproc request was returned,
       * reply and transit to Ready state.
       */

      reply(AsMicFrontendEventStop, msg->getType(), AS_ECODE_OK);

      m_state = Ready;
    }
  else
    {
      /* If capture or preproc request remains, hold command. */

      if (!setExternalCmd(AsMicFrontendEventStop))
        {
          reply(AsMicFrontendEventStop, msg->getType(), AS_ECODE_QUEUE_OPERATION_ERROR);
          return;
        }
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::initPreproc(MsgPacket *msg)
{
  AsInitPreProcParam initparam =
    msg->moveParam<MicFrontendCommand>().initpreproc_param;

  MIC_FRONTEND_DBG("Init Pre Proc:\n");

  if (m_preproc_type == AsMicFrontendPreProcSrc)
    {
      reply(AsMicFrontendEventInitPreProc,
            msg->getType(),
            AS_ECODE_OK);
      return;
    }

  if (m_p_preproc_instance == NULL)
    {
      reply(AsMicFrontendEventInitPreProc,
            msg->getType(),
            AS_ECODE_DSP_SET_ERROR);
      return;
    }

  InitComponentParam param;

  param.custom.addr = initparam.packet_addr;
  param.custom.size = initparam.packet_size;

  /* Init Preproc (Copy packet to MH internally, and wait return from DSP) */

  bool send_result = m_p_preproc_instance->init(param);

  ComponentCmpltParam cmplt;
  m_p_preproc_instance->recv_done(&cmplt);

  /* Reply */

  reply(AsMicFrontendEventInitPreProc, msg->getType(), send_result);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::setPreproc(MsgPacket *msg)
{
  AsSetPreProcParam setparam =
    msg->moveParam<MicFrontendCommand>().setpreproc_param;

  MIC_FRONTEND_DBG("Set Pre Proc:\n");

  if (m_p_preproc_instance == NULL)
    {
      reply(AsMicFrontendEventInitPreProc,
            msg->getType(),
            AS_ECODE_DSP_SET_ERROR);
      return;
    }

  SetComponentParam param;

  param.custom.addr = setparam.packet_addr;
  param.custom.size = setparam.packet_size;

  /* Set Preproc (Copy packet to MH internally) */

  bool send_result = m_p_preproc_instance->set(param);

  /* Reply (Don't wait reply from DSP because it will take long time) */

  reply(AsMicFrontendEventSetPreProc,
        msg->getType(),
        (send_result) ? AS_ECODE_OK : AS_ECODE_DSP_SET_ERROR);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::setMicGain(MsgPacket *msg)
{
  AsMicFrontendMicGainParam recv_micgain =
    msg->moveParam<MicFrontendCommand>().mic_gain_param;

  MIC_FRONTEND_DBG("Set Mic Gain:\n");

  SetMicGainCaptureComponentParam set_micgain;

  for (int i = 0; i < MAX_CAPTURE_MIC_CH; i++)
    {
      if (i < AS_MIC_CHANNEL_MAX)
        {
          set_micgain.mic_gain[i] = recv_micgain.mic_gain[i];
        }
      else
        {
          set_micgain.mic_gain[i] = 0;
        }
    }

  CaptureComponentParam cap_comp_param;
  cap_comp_param.handle = m_capture_hdlr;
  cap_comp_param.set_micgain_param = &set_micgain;

  if (!AS_set_micgain_capture(&cap_comp_param))
    {
      reply(AsMicFrontendEventSetMicGain, msg->getType(), AS_ECODE_SET_MIC_GAIN_ERROR);
    }

  reply(AsMicFrontendEventSetMicGain, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::illegalPreprocDone(MsgPacket *msg)
{
  msg->moveParam<MicFrontendObjPreProcDoneCmd>();

  /* Even if illegal reply, but need to do post handling same as usual.
   * Because allocated areas and queue for encodeing should be free.
   */

  if (m_p_preproc_instance)
    {
      m_p_preproc_instance->recv_done();
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::preprocDoneOnActive(MsgPacket *msg)
{
  MicFrontendObjPreProcDoneCmd preproc_result =
    msg->moveParam<MicFrontendObjPreProcDoneCmd>();

  /* Check PreProcess instance exsits */

  if (m_p_preproc_instance == NULL)
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  /* If it is not return of Exec of Flush, no need to proprocess. */

  if (!(preproc_result.event_type == ComponentExec)
   && !(preproc_result.event_type == ComponentFlush))
    {
      m_p_preproc_instance->recv_done();
      return;
    }

  /* Get prefilter result */

  ComponentCmpltParam cmplt;

  m_p_preproc_instance->recv_done(&cmplt);

  /* Send to dest */

  sendData(cmplt.output);

  /* Decrement proproc request num */

  m_preproc_req--;
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::preprocDoneOnStop(MsgPacket *msg)
{
  MicFrontendObjPreProcDoneCmd preproc_result =
    msg->moveParam<MicFrontendObjPreProcDoneCmd>();

  /* Check PreProcess instance exsits */

  if (m_p_preproc_instance == NULL)
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  /* If it is not return of Exec of Flush, no need to rendering. */

  if (!(preproc_result.event_type == ComponentExec)
   && !(preproc_result.event_type == ComponentFlush))
    {
      m_p_preproc_instance->recv_done();
      return;
    }

  /* Decrement proproc request num */

  m_preproc_req--;

  /* Get prefilter result */

  ComponentCmpltParam cmplt;

  m_p_preproc_instance->recv_done(&cmplt);

  if (preproc_result.event_type == ComponentExec)
    {
      if (cmplt.result && (cmplt.output.size > 0))
        {
          sendData(cmplt.output);
        }
    }
  else if (preproc_result.event_type == ComponentFlush)
    {
      if (cmplt.result)
        {
          /* Add EndFrame mark to flush result. */

          cmplt.output.is_end = true;

          sendData(cmplt.output);

          /* Check external command */

          if (!m_capture_req && !m_preproc_req)
            {
              /* If all of capture and preproc request was returned,
               * Check exeternal command and transit to Ready if exist.
               */

              if (checkExternalCmd())
                {
                  AsMicFrontendEvent ext_cmd = getExternalCmd();

                  reply(ext_cmd, MSG_AUD_MFE_CMD_STOP, AS_ECODE_OK);

                  m_state = Ready;
                }
              else
                {
                  m_state = WaitStop;
                }
            }
          else
            {
              /* If capture or preproc request remains,
               * transit to WaitStop to avoid leak of MH.
               */

              m_state = WaitStop;
            }
        }
    }
  else
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::preprocDoneOnErrorStop(MsgPacket *msg)
{
  preprocDoneOnStop(msg);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::preprocDoneOnWaitStop(MsgPacket *msg)
{
  MicFrontendObjPreProcDoneCmd preproc_result =
    msg->moveParam<MicFrontendObjPreProcDoneCmd>();

  /* Check PreProcess instance exsits */

  if (m_p_preproc_instance == NULL)
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  /* If it is not return of Exec of Flush, no need to rendering. */

  if (!(preproc_result.event_type == ComponentExec)
   && !(preproc_result.event_type == ComponentFlush))
    {
      m_p_preproc_instance->recv_done();
      return;
    }

  /* Decrement proproc request num */

  m_preproc_req--;

  /* Get prefilter result */

  m_p_preproc_instance->recv_done();

  /* If capture or preproc request remains,
   * don't ransit to Ready to avoid leak of MH.
   */

  if (!m_capture_req && !m_preproc_req)
    {
      if (checkExternalCmd())
        {
          AsMicFrontendEvent ext_cmd = getExternalCmd();
          reply(ext_cmd, MSG_AUD_MFE_CMD_STOP, AS_ECODE_OK);

          m_state = Ready;
        }
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::illegalCaptureDone(MsgPacket *msg)
{
  msg->moveParam<CaptureDataParam>();
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::captureDoneOnActive(MsgPacket *msg)
{
  CaptureDataParam cap_rslt = msg->moveParam<CaptureDataParam>();
  CaptureComponentParam cap_comp_param;

  /* Decrement capture request num */

  m_capture_req--;

  /* Request next capture */

  cap_comp_param.handle                = m_capture_hdlr;
  cap_comp_param.exec_param.pcm_sample = m_samples_per_frame;

  bool result = AS_exec_capture(&cap_comp_param);
  if (result)
    {
      /* Increment capture request num */

      m_capture_req++;
    }

  /* Exec PreProcess */

  bool exec_result = (!cap_rslt.buf.validity)
                       ? false : execPreProc(cap_rslt.buf.cap_mh,
                                             cap_rslt.buf.sample);

  /* If exec failed, stop capturing */

  if (!exec_result)
    {
      cap_comp_param.handle          = m_capture_hdlr;
      cap_comp_param.stop_param.mode = AS_DMASTOPMODE_NORMAL;

      AS_stop_capture(&cap_comp_param);

      m_state = ErrorStopping;
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::captureDoneOnStop(MsgPacket *msg)
{
  CaptureDataParam cap_rslt = msg->moveParam<CaptureDataParam>();

  /* Decrement capture request num */

  m_capture_req--;

  /* Exec PreProcess */

  bool exec_result = (!cap_rslt.buf.validity)
                       ? false : execPreProc(cap_rslt.buf.cap_mh,
                                             cap_rslt.buf.sample);

  /* If exec failed, transit to ErrorStopping.
   * Here, capture stop request was alreay issued.
   */

  if (!exec_result)
    {
      m_state = ErrorStopping;
    }

  /* Check endframe of capture */

  if (cap_rslt.end_flag)
    {
      bool stop_result = flushPreProc();

      /* If flush failed, reply of flush request will not be notified.
       * It means that cannot detect last frame of preproc.
       * So, here, transit to Ready state and dispose follow proproc data.
       */

      if (!stop_result)
        {
          /* Send dummy end frame */

          sendDummyEndData();

          if (!m_capture_req && !m_preproc_req)
            {
              /* Reply */

              AsMicFrontendEvent ext_cmd = getExternalCmd();
              reply(ext_cmd, MSG_AUD_MFE_CMD_STOP, AS_ECODE_OK);

              /* Transit to Ready */

              m_state = Ready;
            }
          else
            {
              /* If capture or preproc request remains,
               * transit to WaitStop to avoid leak of MH.
               */

              m_state = WaitStop;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::captureDoneOnErrorStop(MsgPacket *msg)
{
  CaptureDataParam cap_rslt = msg->moveParam<CaptureDataParam>();

  /* Decrement capture request num */

  m_capture_req--;

  /* In error stopping sequence, do not pre process because
   * it won't be sequencial data and cause noise at end of data.
   */

  /* Check end of capture */

  if (cap_rslt.end_flag)
    {
      bool stop_result = flushPreProc();

      /* If stop failed, reply of flush request will not be notified.
       * It means that cannot detect last frame of preproc.
       * So, here, transit to WaitStop state and dispose follow proproc data.
       */

      if (!stop_result)
        {
          /* Send dummy end frame */

          sendDummyEndData();

          /* Check exeternal command que, and reply if command exeits. */

          if (!m_capture_req && !m_preproc_req)
            {
              if (checkExternalCmd())
                {
                  AsMicFrontendEvent ext_cmd = getExternalCmd();
                  reply(ext_cmd, MSG_AUD_MFE_CMD_STOP, AS_ECODE_OK);

                  m_state = Ready;
                }
              else
                {
                  m_state = WaitStop;
                }
            }
          else
            {
              /* If capture or preproc request remains,
               * transit to WaitStop to avoid leak of MH.
               */

              m_state = WaitStop;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::captureDoneOnWaitStop(MsgPacket *msg)
{
  msg->moveParam<CaptureDataParam>();

  /* Decrement capture request num */

  m_capture_req--;

  if (!m_capture_req && !m_preproc_req)
    {
      if (checkExternalCmd())
        {
          AsMicFrontendEvent ext_cmd = getExternalCmd();
          reply(ext_cmd, MSG_AUD_MFE_CMD_STOP, AS_ECODE_OK);

          m_state = Ready;
        }
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::illegalCaptureError(MsgPacket *msg)
{
  msg->moveParam<CaptureErrorParam>();
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::captureErrorOnActive(MsgPacket *msg)
{
  CaptureErrorParam param = msg->moveParam<CaptureErrorParam>();

  /* Stop capture input */

  CaptureComponentParam cap_comp_param;
  cap_comp_param.handle          = m_capture_hdlr;
  cap_comp_param.stop_param.mode = AS_DMASTOPMODE_NORMAL;

  AS_stop_capture(&cap_comp_param);

  if (param.error_type == CaptureErrorErrInt)
    {
      /* Flush PreProcess.
       * Because, the continuity of captured audio data will be lost when
       * ERRINT was occured. Therefore, do not excute subsequent encodings.
       * Instead of them, flush(stop) encoding right now.
       */

      bool stop_result = flushPreProc();

      if (!stop_result)
        {
          /* If flush failed, return from preprocess never returns.
           * So, Send dummy end frame, and tantist WaitStop state.
           */

          sendDummyEndData();

          m_state = WaitStop;
        }
      else
        {
          m_state = ErrorStopping;
        }
    }
  else
    {
      /* Transit to ErrorStopping state. */

      m_state = ErrorStopping;
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::captureErrorOnStop(MsgPacket *msg)
{
  CaptureErrorParam param = msg->moveParam<CaptureErrorParam>();

  if (param.error_type == CaptureErrorErrInt)
    {
      /* Occurrence of ERRINT means that the end flame has not arrived yet
       * and it will never arrive. Therefore, flush preproc is not executed
       * yet and there is no trigger to execute in future.
       * Request flush preproc here.
       */

      bool stop_result = flushPreProc();

      if (!stop_result)
        {
          /* If flush failed, return from preprocess never returns.
           * So, Send dummy end frame, and tantist WaitStop or Ready state.
           */

          sendDummyEndData();

          if (!m_capture_req && !m_preproc_req)
            {
              if (checkExternalCmd())
                {
                  AsMicFrontendEvent ext_cmd = getExternalCmd();
                  reply(ext_cmd, MSG_AUD_MFE_CMD_STOP, AS_ECODE_OK);

                  m_state = Ready;
                }
              else
                {
                  m_state = WaitStop;
                }
            }
          else
            {
              /* If capture or preproc request remains,
               * transit to WaitStop to avoid leak of MH.
               */

              m_state = WaitStop;
            }
        }
      else
        {
          m_state = ErrorStopping;
        }
    }
  else
    {
      /* Transit to ErrorStopping state. */

      m_state = ErrorStopping;
    }
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::captureErrorOnErrorStop(MsgPacket *msg)
{
  /* Same as case of Stopping. */

  captureErrorOnStop(msg);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::captureErrorOnWaitStop(MsgPacket *msg)
{
  msg->moveParam<CaptureErrorParam>();

  /* If already in wait stop sequence, there are nothing to do */
}

/*--------------------------------------------------------------------------*/
bool MicFrontEndObject::startCapture()
{
  bool result = true;

  /* Pre stock capture request to avoid underflow. */

  for (int i = 0; i < CAPTURE_PRESET_NUM; i++)
    {
      CaptureComponentParam cap_comp_param;

      cap_comp_param.handle                = m_capture_hdlr;
      cap_comp_param.exec_param.pcm_sample = m_samples_per_frame;

      result = AS_exec_capture(&cap_comp_param);
      if (!result)
        {
          break;
        }

      /* Increment capture request num */

      m_capture_req++;
    }

  return result;
}

/*--------------------------------------------------------------------------*/
bool MicFrontEndObject::setExternalCmd(AsMicFrontendEvent ext_event)
{
  if (!m_external_cmd_que.push(ext_event))
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
AsMicFrontendEvent MicFrontEndObject::getExternalCmd(void)
{
  AsMicFrontendEvent ext_cmd = AsMicFrontendEventAct;

  if (m_external_cmd_que.empty())
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
    }
  else
    {
      ext_cmd = m_external_cmd_que.top();

      if (!m_external_cmd_que.pop())
        {
          MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }
    }

  return ext_cmd;
}

/*--------------------------------------------------------------------------*/
uint32_t MicFrontEndObject::checkExternalCmd(void)
{
  return m_external_cmd_que.size();
}

/*--------------------------------------------------------------------------*/
uint32_t MicFrontEndObject::activateParamCheck(
  const AsActivateFrontendParam &param)
{
  switch (param.input_device)
    {
      case AsMicFrontendDeviceMic:
        {
          cxd56_audio_micdev_t micdev = cxd56_audio_get_micdev();
          if (micdev == CXD56_AUDIO_MIC_DEV_ANALOG)
            {
              m_input_device = CaptureDeviceAnalogMic;
            }
          else
            {
              m_input_device = CaptureDeviceDigitalMic;
            }
        }
        break;

      case AsMicFrontendDeviceI2S:
        m_input_device = CaptureDeviceI2S;
        break;

      default:
        MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_INPUT_DEVICE;
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t MicFrontEndObject::initParamCheck(const MicFrontendCommand& cmd)
{
  uint32_t rst = AS_ECODE_OK;

  /* Check number of channels */

  switch(cmd.init_param.channel_number)
    {
      case AS_CHANNEL_STEREO:
          break;

      case AS_CHANNEL_MONO:
      case AS_CHANNEL_4CH:
        if (m_input_device == CaptureDeviceI2S)
          {
            MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
            return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
          }
          break;

      case AS_CHANNEL_6CH:
      case AS_CHANNEL_8CH:
        if (m_input_device != CaptureDeviceDigitalMic)
          {
            MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
            return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
          }
          break;

      default:
        MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
    }

  /* Check bit length */

  switch(cmd.init_param.bit_length)
    {
      case AS_BITLENGTH_16:
      case AS_BITLENGTH_24:
      case AS_BITLENGTH_32:
        break;

      default:
        MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_BIT_LENGTH;
    }

  return rst;
}

/*--------------------------------------------------------------------------*/
bool MicFrontEndObject::execPreProc(MemMgrLite::MemHandle inmh, uint32_t sample)
{
  if (m_p_preproc_instance == NULL)
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  ExecComponentParam exec;

  exec.input.identifier = 0;
  exec.input.callback   = NULL;
  exec.input.mh         = inmh;
  exec.input.sample     = sample;
  exec.input.size       = m_channel_num * m_cap_bytes * sample;
  exec.input.is_end     = false;
  exec.input.is_valid   = true;
  exec.input.bit_length = m_pcm_bit_width;

  /* If preprocess is not active, don't alloc output area. */

  if (m_preproc_type != AsMicFrontendPreProcThrough)
    {

      if (m_pool_id.output == MemMgrLite::NullPoolId)
        {
          /* For compatibility.
           * If null pool id is set, use same area as input.
           */

          exec.output = exec.input.mh;
        }
      else
        {
          if (ERR_OK != exec.output.allocSeg(m_pool_id.output,
                                             m_max_output_size))
            {
              MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
              return false;
            }
        }
    }

  /* Exec PreProcess */

  if (!m_p_preproc_instance->exec(exec))
    {
      return false;
    }

  /* Increment proproc request num */

  m_preproc_req++;

  return true;
}

/*--------------------------------------------------------------------------*/
bool MicFrontEndObject::flushPreProc(void)
{
  if (m_p_preproc_instance == NULL)
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  FlushComponentParam flush;

  /* Set preprocess flush output area */

  if (m_preproc_type != AsMicFrontendPreProcThrough)
    {
      if (m_pool_id.output == MemMgrLite::NullPoolId)
        {
          /* For compatibility.
           * If null pool id is set, allocate on input area.
           */

          if (ERR_OK != flush.output.allocSeg(m_pool_id.input,
                                              m_max_capture_size))
            {
              MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
              return false;
            }
        }
      else
        {
          if (ERR_OK != flush.output.allocSeg(m_pool_id.output,
                                              m_max_capture_size))
            {
              MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
              return false;
            }
        }
    }

  if (!m_p_preproc_instance->flush(flush))
    {
      return false;
    }

  /* Increment proproc request num */

  m_preproc_req++;

  return true;
}

/*--------------------------------------------------------------------------*/
bool MicFrontEndObject::sendData(AsPcmDataParam& data)
{
  data.identifier = 0;
  data.callback   = pcm_send_done_callback;

  if (m_pcm_data_path == AsDataPathCallback)
    {
      /* Call callback function for PCM data notify */

      m_pcm_data_dest.cb(data);
    }
  else if (m_pcm_data_path == AsDataPathSimpleFIFO)
    {
      /* Get into the FIFO for PCM data notify */

      if (data.size == 0)
        {
          return true;
        }

      if (CMN_SimpleFifoGetVacantSize(m_pcm_data_dest.simple_fifo_handler) < data.size)
        {
          MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_OVERFLOW);
          return false;
        }

      if (CMN_SimpleFifoOffer(m_pcm_data_dest.simple_fifo_handler,
                              static_cast<const void*>(data.mh.getVa()),
                              data.size) != data.size)
        {
          MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_OVERFLOW);
          return false;
        }
    }
  else
    {
      /* Send message for PCM data notify */

      err_t er = MsgLib::send<AsPcmDataParam>(m_pcm_data_dest.msg.msgqid,
                                              MsgPriNormal,
                                              m_pcm_data_dest.msg.msgtype,
                                              m_msgq_id.self,
                                              data);
      F_ASSERT(er == ERR_OK);
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool MicFrontEndObject::sendDummyEndData(void)
{
  /* Send dummy end frame */

  AsPcmDataParam dmypcm = { 0 };
  dmypcm.size   = 0;
  dmypcm.is_end = true;

  return sendData(dmypcm);
}

/*--------------------------------------------------------------------------*/
bool MicFrontEndObject::getInputDeviceHdlr(void)
{
  if (m_capture_hdlr != MAX_CAPTURE_COMP_INSTANCE_NUM)
    {
      return false;
    }

  if (!AS_get_capture_comp_handler(&m_capture_hdlr,
                                   m_input_device,
                                   m_pool_id.input))
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool MicFrontEndObject::delInputDeviceHdlr(void)
{
  if (m_capture_hdlr != MAX_CAPTURE_COMP_INSTANCE_NUM)
    {
      if(!AS_release_capture_comp_handler(m_capture_hdlr))
        {
          return false;
        }

      m_capture_hdlr = MAX_CAPTURE_COMP_INSTANCE_NUM;
    }
  return true;
}

/*--------------------------------------------------------------------------*/
bool MicFrontEndObject::checkAndSetMemPool(void)
{
  /* Check capture in buffer pool */

  if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.input))
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  m_max_capture_size = (MemMgrLite::Manager::getPoolSize(m_pool_id.input)) /
                       (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.input));

  /* check output buffer pool */

  if (m_pool_id.output != MemMgrLite::NullPoolId)
    {
      if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.output))
        {
          MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
          return false;
        }

      m_max_output_size = (MemMgrLite::Manager::getPoolSize(m_pool_id.output)) /
        (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.output));
    }

  /* check DSP command buffer pool */

  if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.cmp))
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  if ((int)(sizeof(Apu::Wien2ApuCmd)) >
      (MemMgrLite::Manager::getPoolSize(m_pool_id.cmp))/
      (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.cmp)))
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*--------------------------------------------------------------------------*/
static bool CreateFrontend(AsObjectParams_t params, AudioAttentionCb attcb)
{
  /* Register attention callback */

  MIC_FRONTEND_REG_ATTCB(attcb);


  /* Reset Message queue. */

  FAR MsgQueBlock *que;
  err_t err_code = MsgLib::referMsgQueBlock(params.msgq_id.self, &que);
  F_ASSERT(err_code == ERR_OK);
  que->reset();

  /* Init pthread attributes object. */

  pthread_attr_t attr;

  pthread_attr_init(&attr);

  pthread_t pid;

  /* Set pthread scheduling parameter. */

  struct sched_param sch_param;

  sch_param.sched_priority = 150;
  attr.stacksize           = 1024 * 2;

  pthread_attr_setschedparam(&attr, &sch_param);

  /* Create thread. */

  int ret = pthread_create(&pid,
                           &attr,
                           (pthread_startroutine_t)AS_MicFrontendObjEntry,
                           (pthread_addr_t) &params);
  if (ret < 0)
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  pthread_setname_np(pid, "front_end");

  MicFrontEndObject::set_pid(pid);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_CreateMicFrontend(FAR AsCreateMicFrontendParams_t *param, AudioAttentionCb attcb)
{
  AsObjectParams_t params;

  params.msgq_id.self = param->msgq_id.micfrontend;
  params.msgq_id.from = param->msgq_id.mng;
  params.msgq_id.cmp  = param->msgq_id.dsp;

  params.pool_id.input  = param->pool_id.input;
  params.pool_id.output = param->pool_id.output;
  params.pool_id.cmp    = param->pool_id.dsp;;

  return CreateFrontend(params, attcb);
}

/*--------------------------------------------------------------------------*/
bool AS_CreateMicFrontend(FAR AsObjectParams_t params, AudioAttentionCb attcb)
{
  return CreateFrontend(params, attcb);

}

/*--------------------------------------------------------------------------*/
bool AS_ActivateMicFrontend(FAR AsActivateMicFrontend *actparam)
{
  /* Parameter check */

  if ( (actparam == NULL) || !AS_checkAvailabilityMicFrontend() )
    {
      return false;
    }

  /* Activate */

  MicFrontendCommand cmd;

  cmd.act_param = *actparam;

  err_t er = MsgLib::send<MicFrontendCommand>(MicFrontEndObject::get_msgq_id(),
                                              MsgPriNormal,
                                              MSG_AUD_MFE_CMD_ACT,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_InitMicFrontend(FAR AsInitMicFrontendParam *initparam)
{
  /* Parameter check */

  if ( (initparam == NULL) || !AS_checkAvailabilityMicFrontend() )
    {
      return false;
    }

  /* Init */

  MicFrontendCommand cmd;

  cmd.init_param = *initparam;

  err_t er = MsgLib::send<MicFrontendCommand>(MicFrontEndObject::get_msgq_id(),
                                              MsgPriNormal,
                                              MSG_AUD_MFE_CMD_INIT,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StartMicFrontend(FAR AsStartMicFrontendParam *startparam)
{
  /* Parameter check */

  if ( (startparam == NULL) || !AS_checkAvailabilityMicFrontend() )
    {
      return false;
    }

  /* Start */

  MicFrontendCommand cmd;

  cmd.start_param = *startparam;

  err_t er = MsgLib::send<MicFrontendCommand>(MicFrontEndObject::get_msgq_id(),
                                              MsgPriNormal,
                                              MSG_AUD_MFE_CMD_START,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StopMicFrontend(FAR AsStopMicFrontendParam *stopparam)
{
  /* Parameter check */

  if (stopparam == NULL)
    {
      return false;
    }

  /* Stop */

  MicFrontendCommand cmd;

  cmd.stop_param = *stopparam;

  err_t er = MsgLib::send<MicFrontendCommand>(MicFrontEndObject::get_msgq_id(),
                                              MsgPriNormal,
                                              MSG_AUD_MFE_CMD_STOP,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_InitPreprocFrontend(FAR AsInitPreProcParam *initpreparam)
{
  /* Parameter check */

  if ( (initpreparam == NULL) || !AS_checkAvailabilityMicFrontend() )
    {
      return false;
    }

  /* Init PreProcess */

  MicFrontendCommand cmd;

  cmd.initpreproc_param = *initpreparam;

  err_t er = MsgLib::send<MicFrontendCommand>(MicFrontEndObject::get_msgq_id(),
                                              MsgPriNormal,
                                              MSG_AUD_MFE_CMD_INITPREPROC,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_SetPreprocMicFrontend(FAR AsSetPreProcParam *setpreparam)
{
  /* Parameter check */

  if ( (setpreparam == NULL) || !AS_checkAvailabilityMicFrontend() )
    {
      return false;
    }

  /* Set PreProcess */

  MicFrontendCommand cmd;

  cmd.setpreproc_param = *setpreparam;

  err_t er = MsgLib::send<MicFrontendCommand>(MicFrontEndObject::get_msgq_id(),
                                              MsgPriNormal,
                                              MSG_AUD_MFE_CMD_SETPREPROC,
                                              NULL,
                                              cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_SetMicGainMicFrontend(FAR AsMicFrontendMicGainParam *micgain_param)
{
  if ( (micgain_param == NULL) || !AS_checkAvailabilityMicFrontend() )
    {
      return false;
    }

  MicFrontendCommand cmd;

  cmd.mic_gain_param = *micgain_param;

  err_t er = MsgLib::send<MicFrontendCommand>(MicFrontEndObject::get_msgq_id(),
                                              MsgPriNormal,
                                              MSG_AUD_MFE_CMD_SETMICGAIN,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivateMicFrontend(FAR AsDeactivateMicFrontendParam *deactparam)
{
  if ( !AS_checkAvailabilityMicFrontend() )
    {
      return false;
    }

  MicFrontendCommand cmd;

  err_t er = MsgLib::send<MicFrontendCommand>(MicFrontEndObject::get_msgq_id(),
                                              MsgPriNormal,
                                              MSG_AUD_MFE_CMD_DEACT,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeleteMicFrontend(void)
{
  pid_t pid = MicFrontEndObject::get_pid();
  MicFrontEndObject::destory();

  if (pid == INVALID_PROCESS_ID)
    {
      return false;
    }

  pthread_cancel(pid);
  pthread_join(pid, NULL);

  MicFrontEndObject::set_pid(INVALID_PROCESS_ID);
  /* Unregister attention callback */

  MIC_FRONTEND_UNREG_ATTCB();

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_checkAvailabilityMicFrontend(void)
{
  return (MicFrontEndObject::get_instance() != NULL);
}

/*--------------------------------------------------------------------------*/
void MicFrontEndObject::create(AsObjectParams_t* params)
{
  MicFrontEndObject* inst = new(MicFrontEndObject::get_adr()) MicFrontEndObject(params->msgq_id,params->pool_id);
  if (inst != NULL)
    {
      inst->run();
    }
  else
    {
      MIC_FRONTEND_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
    }
}
