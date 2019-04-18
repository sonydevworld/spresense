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
#include <arch/chip/cxd56_audio.h>
#include "memutils/common_utils/common_assert.h"
#include "front_end_obj.h"
#include "components/encoder/encoder_component.h"
#include "components/filter/filter_api.h"
#include "dsp_driver/include/dsp_drv.h"
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

static pid_t s_fed_pid;
static AsFrontendMsgQueId_t s_msgq_id;
static AsFrontendPoolId_t   s_pool_id;

static FrontEndObject *s_fed_obj = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void capture_done_callback(CaptureDataParam param)
{
  err_t er;

  er = MsgLib::send<CaptureDataParam>(s_msgq_id.frontend,
                                      MsgPriNormal,
                                      MSG_AUD_FED_RST_CAPTURE_DONE,
                                      NULL,
                                      param);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
static void capture_error_callback(CaptureErrorParam param)
{
  err_t er;

  er = MsgLib::send<CaptureErrorParam>(s_msgq_id.frontend,
                                       MsgPriNormal,
                                       MSG_AUD_FED_RST_CAPTURE_ERR,
                                       NULL,
                                       param);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
static bool preproc_done_callback(PreprocCbParam *cmplt, void* p_requester)
{
  FRONT_END_VDBG("flt sz %d\n", cmplt->out_buffer.size);

  FrontendObjPreProcDoneCmd param;

  param.event_type = cmplt->event_type;
  param.result     = cmplt->result;

  err_t er = MsgLib::send<FrontendObjPreProcDoneCmd>(s_msgq_id.frontend,
                                                     MsgPriNormal,
                                                     MSG_AUD_FED_RST_PREPROC,
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
int AS_FrontendObjEntry(int argc, char *argv[])
{
  FrontEndObject::create(s_msgq_id,
                         s_pool_id);
  return 0;
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::run(void)
{
  err_t        err_code;
  MsgQueBlock* que;
  MsgPacket*   msg;

  err_code = MsgLib::referMsgQueBlock(m_msgq_id.frontend, &que);
  F_ASSERT(err_code == ERR_OK);

  while(1)
    {
      err_code = que->recv(TIME_FOREVER, &msg);
      F_ASSERT(err_code == ERR_OK);

      parse(msg);

      err_code = que->pop();
      F_ASSERT(err_code == ERR_OK);
    }
}

/*--------------------------------------------------------------------------*/
FrontEndObject::MsgProc
  FrontEndObject::MsgProcTbl[AUD_FED_MSG_NUM][FrontendStateNum] =
{
  /* Message Type: MSG_AUD_FED_CMD_ACT */

  {                                  /* Frontend status: */
    &FrontEndObject::activate,       /*   Inactive.      */
    &FrontEndObject::illegal,        /*   Ready.         */
    &FrontEndObject::illegal,        /*   Active.        */
    &FrontEndObject::illegal,        /*   Stopping.      */
    &FrontEndObject::illegal,        /*   ErrorStopping. */
    &FrontEndObject::illegal         /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_FED_CMD_DEACT */

  {                                  /* Frontend status: */
    &FrontEndObject::illegal,        /*   Inactive.      */
    &FrontEndObject::deactivate,     /*   Ready.         */
    &FrontEndObject::illegal,        /*   Active.        */
    &FrontEndObject::illegal,        /*   Stopping.      */
    &FrontEndObject::illegal,        /*   ErrorStopping. */
    &FrontEndObject::illegal         /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_FED_CMD_INIT */

  {                                  /* Frontend status: */
    &FrontEndObject::illegal,        /*   Inactive.      */
    &FrontEndObject::init,           /*   Ready.         */
    &FrontEndObject::illegal,        /*   Active.        */
    &FrontEndObject::illegal,        /*   Stopping.      */
    &FrontEndObject::illegal,        /*   ErrorStopping. */
    &FrontEndObject::illegal         /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_FED_CMD_START */

  {                                  /* Frontend status: */
    &FrontEndObject::illegal,        /*   Inactive.      */
    &FrontEndObject::startOnReady,   /*   Ready.         */
    &FrontEndObject::illegal,        /*   Active.        */
    &FrontEndObject::illegal,        /*   Stopping.      */
    &FrontEndObject::illegal,        /*   ErrorStopping. */
    &FrontEndObject::illegal         /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_FED_CMD_STOP */

  {                                   /* Frontend status: */
    &FrontEndObject::illegal,         /*   Inactive.      */
    &FrontEndObject::illegal,         /*   Ready.         */
    &FrontEndObject::stopOnActive,    /*   Active.        */
    &FrontEndObject::illegal,         /*   Stopping.      */
    &FrontEndObject::stopOnErrorStop, /*   ErrorStopping. */
    &FrontEndObject::stopOnWait       /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_FED_CMD_INITPREPROC */

  {                                   /* Frontend status: */
    &FrontEndObject::illegal,         /*   Inactive.      */
    &FrontEndObject::initPreproc,     /*   Ready.         */
    &FrontEndObject::illegal,         /*   Active.        */
    &FrontEndObject::illegal,         /*   Stopping.      */
    &FrontEndObject::illegal,         /*   ErrorStopping. */
    &FrontEndObject::illegal          /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_FED_CMD_SETPREPROC */

  {                                   /* Frontend status: */
    &FrontEndObject::illegal,         /*   Inactive.      */
    &FrontEndObject::setPreproc,      /*   Ready.         */
    &FrontEndObject::setPreproc,      /*   Active.        */
    &FrontEndObject::illegal,         /*   Stopping.      */
    &FrontEndObject::illegal,         /*   ErrorStopping. */
    &FrontEndObject::illegal          /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_FED_CMD_SET_MICGAIN. */

  {                                   /* Frontend status: */
    &FrontEndObject::illegal,         /*   Inactive.      */
    &FrontEndObject::setMicGain,      /*   Ready.         */
    &FrontEndObject::setMicGain,      /*   Active.        */
    &FrontEndObject::illegal,         /*   Stopping.      */
    &FrontEndObject::illegal,         /*   ErrorStopping. */
    &FrontEndObject::illegal          /*   WaitStop.      */
  }
};

/*--------------------------------------------------------------------------*/
FrontEndObject::MsgProc
  FrontEndObject::RsltProcTbl[AUD_FED_RST_MSG_NUM][FrontendStateNum] =
{
  /* Message Type: MSG_AUD_FED_RST_CAPTURE_DONE. */

  {                                          /* Frontend status: */
    &FrontEndObject::illegalCaptureDone,     /*   Inactive.      */
    &FrontEndObject::illegalCaptureDone,     /*   Ready.         */
    &FrontEndObject::captureDoneOnActive,    /*   Active.        */
    &FrontEndObject::captureDoneOnStop,      /*   Stopping.      */
    &FrontEndObject::captureDoneOnErrorStop, /*   ErrorStopping. */
    &FrontEndObject::captureDoneOnWaitStop   /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_FED_RST_CAPTURE_ERR. */

  {                                           /* Frontend status: */
    &FrontEndObject::illegalCaptureError,     /*   Inactive.      */
    &FrontEndObject::illegalCaptureError,     /*   Ready.         */
    &FrontEndObject::captureErrorOnActive,    /*   Active.        */
    &FrontEndObject::captureErrorOnStop,      /*   Stopping.      */
    &FrontEndObject::captureErrorOnErrorStop, /*   ErrorStopping. */
    &FrontEndObject::captureErrorOnWaitStop   /*   WaitStop.      */
  },

  /* Message Type: MSG_AUD_FED_RST_PREPROC. */

  {                                           /* Frontend status: */
    &FrontEndObject::illegalPreprocDone,      /*   Inactive.      */
    &FrontEndObject::illegalPreprocDone,      /*   Ready.         */
    &FrontEndObject::preprocDoneOnActive,     /*   Active.        */
    &FrontEndObject::preprocDoneOnStop,       /*   Stopping.      */
    &FrontEndObject::preprocDoneOnErrorStop,  /*   ErrorStopping. */
    &FrontEndObject::preprocDoneOnWaitStop    /*   WaitStop.      */
  }
};

/*--------------------------------------------------------------------------*/
void FrontEndObject::parse(MsgPacket *msg)
{
  uint32_t event;

  if (MSG_IS_REQUEST(msg->getType()) == 0)
    {
      event = MSG_GET_SUBTYPE(msg->getType());
      F_ASSERT((event < AUD_FED_RST_MSG_NUM));

      (this->*RsltProcTbl[event][m_state.get()])(msg);
    }
  else
    {
      event = MSG_GET_SUBTYPE(msg->getType());
      F_ASSERT((event < AUD_FED_MSG_NUM));

      (this->*MsgProcTbl[event][m_state.get()])(msg);
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::reply(AsFrontendEvent evtype,
                           MsgType msg_type,
                           uint32_t result)
{
  if (m_callback != NULL)
    {
      m_callback(evtype, result, 0);
    }
  else if (m_msgq_id.mng != MSG_QUE_NULL)
    {
      AudioObjReply cmplt((uint32_t)msg_type,
                           AS_OBJ_REPLY_TYPE_REQ,
                           AS_MODULE_ID_FRONT_END_OBJ,
                           result);
      err_t er = MsgLib::send<AudioObjReply>(m_msgq_id.mng,
                                             MsgPriNormal,
                                             MSG_TYPE_AUD_RES,
                                             m_msgq_id.frontend,
                                             cmplt);
      if (ERR_OK != er)
        {
          F_ASSERT(0);
        }
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::illegal(MsgPacket *msg)
{
  uint msgtype = msg->getType();
  msg->moveParam<FrontendCommand>();

  uint32_t idx = msgtype - MSG_AUD_FED_CMD_ACT;

  AsFrontendEvent table[] =
  {
    AsFrontendEventAct,
    AsFrontendEventDeact,
    AsFrontendEventInit,
    AsFrontendEventStart,
    AsFrontendEventStop,
    AsFrontendEventInitPreProc,
    AsFrontendEventSetPreProc,
    AsFrontendEventSetMicGain
  };

  m_callback(table[idx], AS_ECODE_STATE_VIOLATION, 0);
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::activate(MsgPacket *msg)
{
  uint32_t rst;
  AsActivateFrontend act = msg->moveParam<FrontendCommand>().act_param;

  FRONT_END_DBG("ACT: indev %d\n", act.param.input_device);

  /* Set event callback */

  m_callback = act.cb;

  /* Parameter check */

  if (!checkAndSetMemPool())
    {
      m_callback(AsFrontendEventAct,
                 AS_ECODE_CHECK_MEMORY_POOL_ERROR, 0);
      return;
    }

  rst = activateParamCheck(act.param);

  if (rst != AS_ECODE_OK)
    {
      m_callback(AsFrontendEventAct, rst, 0);
      return;
    }

  /* Activate PreProcess */

  uint32_t dsp_inf = 0;

  AS_preproc_activate(&m_p_preproc_instance,
                       m_pool_id.dspcmd,
                       m_msgq_id.dsp,
                       preproc_done_callback,
                       "PREPROC",
                       static_cast<void *>(this),
                       &dsp_inf,
                       (act.param.pre_enable == AsFrontendPreProcDisable));

  /* State transit */

  m_state = FrontendStateReady;

  /* Reply */

  m_callback(AsFrontendEventAct, AS_ECODE_OK, 0);
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::deactivate(MsgPacket *msg)
{
  msg->moveParam<FrontendCommand>();

  FRONT_END_DBG("DEACT:\n");

  if (!delInputDeviceHdlr())
    {
      m_callback(AsFrontendEventDeact,
                 AS_ECODE_CLEAR_AUDIO_DATA_PATH_ERROR, 0);
      return;
    }

  /* Deactivate PreProcess */

  AS_preproc_deactivate(m_p_preproc_instance);

  /* State transit */

  m_state = FrontendStateInactive;

  /* Reply */

  m_callback(AsFrontendEventDeact, AS_ECODE_OK, 0);
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::init(MsgPacket *msg)
{
  uint32_t rst = AS_ECODE_OK;
  FrontendCommand cmd = msg->moveParam<FrontendCommand>();

  FRONT_END_DBG("INIT: ch num %d, bit len %d, samples/frame %d\n",
                 cmd.init_param.channel_number,
                 cmd.init_param.bit_length,
                 cmd.init_param.samples_per_frame);

  /* Check parameters */

  rst = initParamCheck(cmd);
  if (rst != AS_ECODE_OK)
    {
      m_callback(AsFrontendEventInit, rst, 0);
      return;
    }

  /* Hold parameters */

  m_channel_num       = cmd.init_param.channel_number;
  m_pcm_bit_width     =
    ((cmd.init_param.bit_length == AS_BITLENGTH_16)
      ? AudPcm16Bit : (cmd.init_param.bit_length == AS_BITLENGTH_24)
                        ? AudPcm24Bit : AudPcm32Bit);
  m_cap_bytes         = ((m_pcm_bit_width == AudPcm16Bit) ? 2 : 4);
  m_samples_per_frame = cmd.init_param.samples_per_frame;
  m_pcm_data_path     = static_cast<AsFrontendDataPath>(cmd.init_param.data_path);
  m_pcm_data_dest     = cmd.init_param.dest;

  /* Release and get capture (Input device) handler */

  if (!delInputDeviceHdlr())
    {
      m_callback(AsFrontendEventInit, AS_ECODE_CLEAR_AUDIO_DATA_PATH_ERROR, 0);
      return;
    }

  if (!getInputDeviceHdlr())
    {
      m_callback(AsFrontendEventInit, AS_ECODE_SET_AUDIO_DATA_PATH_ERROR, 0);
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
      m_callback(AsFrontendEventStart, AS_ECODE_DMAC_INITIALIZE_ERROR, 0);
      return;
    }

  /* Reply */

  m_callback(AsFrontendEventInit, AS_ECODE_OK, 0);
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::startOnReady(MsgPacket *msg)
{
  msg->moveParam<FrontendCommand>();

  FRONT_END_DBG("START:\n");

  /* Start capture */

  if (!startCapture())
    {
      m_callback(AsFrontendEventStart, AS_ECODE_DMAC_READ_ERROR, 0);
      return;
    }

  /* State transit */

  m_state = FrontendStateActive;

  /* Reply*/

  m_callback(AsFrontendEventStart, AS_ECODE_OK, 0);
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::stopOnActive(MsgPacket *msg)
{
  msg->moveParam<FrontendCommand>();

  FRONT_END_DBG("STOP:\n");

  if (!setExternalCmd(AsFrontendEventStop))
    {
      m_callback(AsFrontendEventStop, AS_ECODE_QUEUE_OPERATION_ERROR, 0);
      return;
    }

  /* Stop DMA transfer. */

  CaptureComponentParam cap_comp_param;

  cap_comp_param.handle          = m_capture_hdlr;
  cap_comp_param.stop_param.mode = AS_DMASTOPMODE_NORMAL;

  AS_stop_capture(&cap_comp_param);

  /* State transit */

  m_state = FrontendStateStopping;
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::stopOnErrorStop(MsgPacket *msg)
{
  msg->moveParam<FrontendCommand>();

  FRONT_END_DBG("STOP:\n");

  /* If STOP command received during internal error stop sequence, enque command.
   * It will be trigger of transition to Ready when all of preproc is done.
   */

   if (!setExternalCmd(AsFrontendEventStop))
     {
       m_callback(AsFrontendEventStop, AS_ECODE_QUEUE_OPERATION_ERROR, 0);
       return;
     }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::stopOnWait(MsgPacket *msg)
{
  msg->moveParam<FrontendCommand>();

  FRONT_END_DBG("STOP:\n");

  if (!m_capture_req && !m_preproc_req)
    {
      /* If all of capture and preproc request was returned,
       * reply and transit to Ready state.
       */

      m_callback(AsFrontendEventStop, AS_ECODE_OK, 0);

      m_state = FrontendStateReady;
    }
  else
    {
      /* If capture or preproc request remains, hold command. */

      if (!setExternalCmd(AsFrontendEventStop))
        {
          m_callback(AsFrontendEventStop, AS_ECODE_QUEUE_OPERATION_ERROR, 0);
          return;
        }
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::initPreproc(MsgPacket *msg)
{
  AsInitPreProcParam initparam =
    msg->moveParam<FrontendCommand>().initpreproc_param;

  FRONT_END_DBG("Init Pre Proc:\n");

  InitPreprocParam param;

  param.is_userdraw = true;
  param.packet.addr = initparam.packet_addr;
  param.packet.size = initparam.packet_size;

  /* Init Preproc (Copy packet to MH internally, and wait return from DSP) */

  bool send_result = AS_preproc_init(&param, m_p_preproc_instance);

  PreprocCmpltParam cmplt;
  AS_preproc_recv_done(m_p_preproc_instance, &cmplt);

  /* Reply */

  m_callback(AsFrontendEventInitPreProc, send_result, 0);
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::setPreproc(MsgPacket *msg)
{
  AsSetPreProcParam setparam =
    msg->moveParam<FrontendCommand>().setpreproc_param;

  FRONT_END_DBG("Set Pre Proc:\n");

  SetPreprocParam param;

  param.is_userdraw = true;
  param.packet.addr = setparam.packet_addr;
  param.packet.size = setparam.packet_size;

  /* Set Preproc (Copy packet to MH internally) */

  bool send_result = AS_preproc_setparam(&param, m_p_preproc_instance);

  /* Reply (Don't wait reply from DSP because it will take long time) */

  m_callback(AsFrontendEventSetPreProc,
             (send_result) ? AS_ECODE_OK : AS_ECODE_DSP_SET_ERROR, 0);
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::setMicGain(MsgPacket *msg)
{
  AsFrontendMicGainParam recv_micgain =
    msg->moveParam<FrontendCommand>().mic_gain_param;

  FRONT_END_DBG("Set Mic Gain:\n");

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
      m_callback(AsFrontendEventSetMicGain, AS_ECODE_SET_MIC_GAIN_ERROR, 0);
    }

  m_callback(AsFrontendEventSetMicGain, AS_ECODE_OK, 0);
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::illegalPreprocDone(MsgPacket *msg)
{
  msg->moveParam<FrontendObjPreProcDoneCmd>();

  /* Even if illegal reply, but need to do post handling same as usual.
   * Because allocated areas and queue for encodeing should be free.
   */

  if (m_p_preproc_instance)
    {
      AS_preproc_recv_done(m_p_preproc_instance, NULL);
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::preprocDoneOnActive(MsgPacket *msg)
{
  FrontendObjPreProcDoneCmd preproc_result =
    msg->moveParam<FrontendObjPreProcDoneCmd>();

  /* Check PreProcess instance exsits */

  if (!m_p_preproc_instance)
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  /* If it is not return of Exec of Flush, no need to proprocess. */

  if (!(preproc_result.event_type == PreprocExec)
   && !(preproc_result.event_type == PreprocFlush))
    {
      AS_preproc_recv_done(m_p_preproc_instance, NULL);
      return;
    }

  /* Get prefilter result */

  PreprocCmpltParam cmplt;

  AS_preproc_recv_done(m_p_preproc_instance, &cmplt);

  /* Send to dest */

  sendData(cmplt.output);

  /* Decrement proproc request num */

  m_preproc_req--;
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::preprocDoneOnStop(MsgPacket *msg)
{
  FrontendObjPreProcDoneCmd preproc_result =
    msg->moveParam<FrontendObjPreProcDoneCmd>();

  /* Check PreProcess instance exsits */

  if (!m_p_preproc_instance)
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  /* If it is not return of Exec of Flush, no need to rendering. */

  if (!(preproc_result.event_type == PreprocExec)
   && !(preproc_result.event_type == PreprocFlush))
    {
      AS_preproc_recv_done(m_p_preproc_instance, NULL);
      return;
    }

  /* Decrement proproc request num */

  m_preproc_req--;

  /* Get prefilter result */

  PreprocCmpltParam cmplt;

  AS_preproc_recv_done(m_p_preproc_instance, &cmplt);

  if (preproc_result.event_type == PreprocExec)
    {
      if (cmplt.result && (cmplt.output.size > 0))
        {
          sendData(cmplt.output);
        }
    }
  else if (preproc_result.event_type == PreprocFlush)
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
                  AsFrontendEvent ext_cmd = getExternalCmd();

                  m_callback(ext_cmd, AS_ECODE_OK, 0);

                  m_state = FrontendStateReady;
                }
              else
                {
                  m_state = FrontendStateWaitStop;
                }
            }
          else
            {
              /* If capture or preproc request remains,
               * transit to WaitStop to avoid leak of MH.
               */

              m_state = FrontendStateWaitStop;
            }
        }
    }
  else
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::preprocDoneOnErrorStop(MsgPacket *msg)
{
  preprocDoneOnStop(msg);
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::preprocDoneOnWaitStop(MsgPacket *msg)
{
  FrontendObjPreProcDoneCmd preproc_result =
    msg->moveParam<FrontendObjPreProcDoneCmd>();

  /* Check PreProcess instance exsits */

  if (!m_p_preproc_instance)
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  /* If it is not return of Exec of Flush, no need to rendering. */

  if (!(preproc_result.event_type == PreprocExec)
   && !(preproc_result.event_type == PreprocFlush))
    {
      AS_preproc_recv_done(m_p_preproc_instance, NULL);
      return;
    }

  /* Decrement proproc request num */

  m_preproc_req--;

  /* Get prefilter result */

  AS_preproc_recv_done(m_p_preproc_instance, NULL);

  /* If capture or preproc request remains,
   * don't ransit to Ready to avoid leak of MH.
   */

  if (!m_capture_req && !m_preproc_req)
    {
      if (checkExternalCmd())
        {
          AsFrontendEvent ext_cmd = getExternalCmd();
          m_callback(ext_cmd, AS_ECODE_OK, 0);

          m_state = FrontendStateReady;
        }
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::illegalCaptureDone(MsgPacket *msg)
{
  msg->moveParam<CaptureDataParam>();
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::captureDoneOnActive(MsgPacket *msg)
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

  bool exec_result = execPreProc(cap_rslt.buf.cap_mh,
                                 cap_rslt.buf.sample);

  /* If exec failed, stop capturing */

  if (!exec_result)
    {
      cap_comp_param.handle          = m_capture_hdlr;
      cap_comp_param.stop_param.mode = AS_DMASTOPMODE_NORMAL;

      AS_stop_capture(&cap_comp_param);

      m_state = FrontendStateErrorStopping;
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::captureDoneOnStop(MsgPacket *msg)
{
  CaptureDataParam cap_rslt = msg->moveParam<CaptureDataParam>();

  /* Decrement capture request num */

  m_capture_req--;

  /* Exec PreProcess */

  bool exec_result = execPreProc(cap_rslt.buf.cap_mh,
                                 cap_rslt.buf.sample);

  /* If exec failed, transit to ErrorStopping.
   * Here, capture stop request was alreay issued.
   */

  if (!exec_result)
    {
      m_state = FrontendStateErrorStopping;
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

              AsFrontendEvent ext_cmd = getExternalCmd();
              m_callback(ext_cmd, AS_ECODE_OK, 0);

              /* Transit to Ready */

              m_state = FrontendStateReady;
            }
          else
            {
              /* If capture or preproc request remains,
               * transit to WaitStop to avoid leak of MH.
               */

              m_state = FrontendStateWaitStop;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::captureDoneOnErrorStop(MsgPacket *msg)
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
                  AsFrontendEvent ext_cmd = getExternalCmd();
                  m_callback(ext_cmd, AS_ECODE_OK, 0);

                  m_state = FrontendStateReady;
                }
              else
                {
                  m_state = FrontendStateWaitStop;
                }
            }
          else
            {
              /* If capture or preproc request remains,
               * transit to WaitStop to avoid leak of MH.
               */

              m_state = FrontendStateWaitStop;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::captureDoneOnWaitStop(MsgPacket *msg)
{
  msg->moveParam<CaptureDataParam>();

  /* Decrement capture request num */

  m_capture_req--;

  if (!m_capture_req && !m_preproc_req)
    {
      if (checkExternalCmd())
        {
          AsFrontendEvent ext_cmd = getExternalCmd();
          m_callback(ext_cmd, AS_ECODE_OK, 0);

          m_state = FrontendStateReady;
        }
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::illegalCaptureError(MsgPacket *msg)
{
  msg->moveParam<CaptureErrorParam>();
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::captureErrorOnActive(MsgPacket *msg)
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

          m_state = FrontendStateWaitStop;
        }
      else
        {
          m_state = FrontendStateErrorStopping;
        }
    }
  else
    {
      /* Transit to ErrorStopping state. */

      m_state = FrontendStateErrorStopping;
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::captureErrorOnStop(MsgPacket *msg)
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
                  AsFrontendEvent ext_cmd = getExternalCmd();
                  m_callback(ext_cmd, AS_ECODE_OK, 0);

                  m_state = FrontendStateReady;
                }
              else
                {
                  m_state = FrontendStateWaitStop;
                }
            }
          else
            {
              /* If capture or preproc request remains,
               * transit to WaitStop to avoid leak of MH.
               */

              m_state = FrontendStateWaitStop;
            }
        }
      else
        {
          m_state = FrontendStateErrorStopping;
        }
    }
  else
    {
      /* Transit to ErrorStopping state. */

      m_state = FrontendStateErrorStopping;
    }
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::captureErrorOnErrorStop(MsgPacket *msg)
{
  /* Same as case of Stopping. */

  captureErrorOnStop(msg);
}

/*--------------------------------------------------------------------------*/
void FrontEndObject::captureErrorOnWaitStop(MsgPacket *msg)
{
  msg->moveParam<CaptureErrorParam>();

  /* If already in wait stop sequence, there are nothing to do */
}

/*--------------------------------------------------------------------------*/
bool FrontEndObject::startCapture()
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
bool FrontEndObject::setExternalCmd(AsFrontendEvent ext_event)
{
  if (!m_external_cmd_que.push(ext_event))
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
AsFrontendEvent FrontEndObject::getExternalCmd(void)
{
  AsFrontendEvent ext_cmd = AsFrontendEventAct;

  if (m_external_cmd_que.empty())
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
    }
  else
    {
      ext_cmd = m_external_cmd_que.top();

      if (!m_external_cmd_que.pop())
        {
          FRONT_END_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }
    }

  return ext_cmd;
}

/*--------------------------------------------------------------------------*/
uint32_t FrontEndObject::checkExternalCmd(void)
{
  return m_external_cmd_que.size();
}

/*--------------------------------------------------------------------------*/
MemMgrLite::MemHandle FrontEndObject::getOutputBufAddr()
{
  MemMgrLite::MemHandle mh;
  if (mh.allocSeg(m_pool_id.output, m_max_output_size) != ERR_OK)
    {
      FRONT_END_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
    }

  return mh;
}

/*--------------------------------------------------------------------------*/
uint32_t FrontEndObject::activateParamCheck(
  const AsActivateFrontendParam &param)
{
  switch (param.input_device)
    {
      case AsFrontendDeviceMic:
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

      case AsFrontendDeviceI2s:
        m_input_device = CaptureDeviceI2S;
        break;

      default:
        FRONT_END_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_INPUT_DEVICE;
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t FrontEndObject::initParamCheck(const FrontendCommand& cmd)
{
  uint32_t rst = AS_ECODE_OK;

  /* Check number of channels */

  switch(cmd.init_param.channel_number)
    {
      case AS_CHANNEL_MONO:
      case AS_CHANNEL_4CH:
        if (m_input_device == CaptureDeviceI2S)
          {
            FRONT_END_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
            return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
          }
        break;

      case AS_CHANNEL_STEREO:
          break;

      case AS_CHANNEL_6CH:
      case AS_CHANNEL_8CH:
        if (m_input_device != CaptureDeviceDigitalMic)
          {
            FRONT_END_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
            return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
          }
          break;

      default:
        FRONT_END_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
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
        FRONT_END_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_BIT_LENGTH;
    }

  return rst;
}

/*--------------------------------------------------------------------------*/
bool FrontEndObject::execPreProc(MemMgrLite::MemHandle inmh, uint32_t sample)
{
  ExecPreprocParam exec;

  exec.input.identifier = 0;
  exec.input.callback   = NULL;
  exec.input.mh         = inmh;
  exec.input.sample     = sample;
  exec.input.size       = m_channel_num * m_cap_bytes * sample;
  exec.input.is_end     = false;
  exec.input.is_valid   = true;

  /* If preprocess is not active, don't alloc output area. */

  if (AS_preproc_is_enable(m_p_preproc_instance))
    {

      if (m_pool_id.output == MemMgrLite::NullPoolId)
        {
          /* For compatibility.
           * If null pool id is set, use same area as input.
           */

          exec.output_mh = exec.input.mh;
        }
      else
        {
          if (ERR_OK != exec.output_mh.allocSeg(m_pool_id.output,
                                                m_max_output_size))
            {
              FRONT_END_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
              return false;
            }
        }
    }

  /* Exec PreProcess */

  if (!AS_preproc_exec(&exec, m_p_preproc_instance))
    {
      return false;
    }

  /* Increment proproc request num */

  m_preproc_req++;

  return true;
}

/*--------------------------------------------------------------------------*/
bool FrontEndObject::flushPreProc(void)
{
  FlushPreprocParam flush;

  /* Set preprocess flush output area */

  if (AS_preproc_is_enable(m_p_preproc_instance))
    {
      if (m_pool_id.output == MemMgrLite::NullPoolId)
        {
          /* For compatibility.
           * If null pool id is set, allocate on input area.
           */

          if (ERR_OK != flush.output_mh.allocSeg(m_pool_id.capin,
                                                 m_max_capture_size))
            {
              FRONT_END_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
              return false;
            }
        }
      else
        {
          if (ERR_OK != flush.output_mh.allocSeg(m_pool_id.output,
                                                 m_max_capture_size))
            {
              FRONT_END_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
              return false;
            }
        }
    }

  if (!AS_preproc_flush(&flush, m_p_preproc_instance))
    {
      return false;
    }

  /* Increment proproc request num */

  m_preproc_req++;

  return true;
}

/*--------------------------------------------------------------------------*/
bool FrontEndObject::sendData(AsPcmDataParam& data)
{
  data.identifier = 0;
  data.callback   = pcm_send_done_callback;

  if (m_pcm_data_path == AsDataPathCallback)
    {
      /* Call callback function for PCM data notify */

      m_pcm_data_dest.cb(data);
    }
  else
    {
      /* Send message for PCM data notify */

      err_t er = MsgLib::send<AsPcmDataParam>(m_pcm_data_dest.msg.msgqid,
                                              MsgPriNormal,
                                              m_pcm_data_dest.msg.msgtype,
                                              m_msgq_id.frontend,
                                              data);
      F_ASSERT(er == ERR_OK);
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool FrontEndObject::sendDummyEndData(void)
{
  /* Send dummy end frame */

  AsPcmDataParam dmypcm = { 0 };
  dmypcm.size   = 0;
  dmypcm.is_end = true;

  return sendData(dmypcm);
}

/*--------------------------------------------------------------------------*/
bool FrontEndObject::getInputDeviceHdlr(void)
{
  if (m_capture_hdlr != MAX_CAPTURE_COMP_INSTANCE_NUM)
    {
      return false;
    }

  if (!AS_get_capture_comp_handler(&m_capture_hdlr,
                                   m_input_device,
                                   m_pool_id.capin))
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool FrontEndObject::delInputDeviceHdlr(void)
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
bool FrontEndObject::checkAndSetMemPool(void)
{
  /* Check capture in buffer pool */

  if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.capin))
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  m_max_capture_size = (MemMgrLite::Manager::getPoolSize(m_pool_id.capin)) /
                       (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.capin));

  /* check output buffer pool */

  if (m_pool_id.output != MemMgrLite::NullPoolId)
    {
      if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.output))
        {
          FRONT_END_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
          return false;
        }

      m_max_output_size = (MemMgrLite::Manager::getPoolSize(m_pool_id.output)) /
        (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.output));
    }

  /* check DSP command buffer pool */

  if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.dspcmd))
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  if ((int)(sizeof(Apu::Wien2ApuCmd)) >
      (MemMgrLite::Manager::getPoolSize(m_pool_id.dspcmd))/
      (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.dspcmd)))
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*--------------------------------------------------------------------------*/
bool AS_CreateFrontend(FAR AsCreateFrontendParam_t *param, AudioAttentionCb attcb)
{
  /* Register attention callback */

  FRONT_END_REG_ATTCB(attcb);

  /* Parameter check */

  if (param == NULL)
    {
      return false;
    }

  /* Create */

  s_msgq_id = param->msgq_id;
  s_pool_id = param->pool_id;

  /* Reset Message queue. */

  FAR MsgQueBlock *que;
  err_t err_code = MsgLib::referMsgQueBlock(s_msgq_id.frontend, &que);
  F_ASSERT(err_code == ERR_OK);
  que->reset();

  s_fed_pid = task_create("FED_OBJ",
                          150, 1024 * 2,
                          AS_FrontendObjEntry,
                          NULL);
  if (s_fed_pid < 0)
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_ActivateFrontend(FAR AsActivateFrontend *actparam)
{
  /* Parameter check */

  if (actparam == NULL)
    {
      return false;
    }

  /* Activate */

  FrontendCommand cmd;

  cmd.act_param = *actparam;

  err_t er = MsgLib::send<FrontendCommand>(s_msgq_id.frontend,
                                           MsgPriNormal,
                                           MSG_AUD_FED_CMD_ACT,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_InitFrontend(FAR AsInitFrontendParam *initparam)
{
  /* Parameter check */

  if (initparam == NULL)
    {
      return false;
    }

  /* Init */

  FrontendCommand cmd;

  cmd.init_param = *initparam;

  err_t er = MsgLib::send<FrontendCommand>(s_msgq_id.frontend,
                                           MsgPriNormal,
                                           MSG_AUD_FED_CMD_INIT,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StartFrontend(FAR AsStartFrontendParam *startparam)
{
  /* Parameter check */

  if (startparam == NULL)
    {
      return false;
    }

  /* Start */

  FrontendCommand cmd;

  cmd.start_param = *startparam;

  err_t er = MsgLib::send<FrontendCommand>(s_msgq_id.frontend,
                                           MsgPriNormal,
                                           MSG_AUD_FED_CMD_START,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StopFrontend(FAR AsStopFrontendParam *stopparam)
{
  /* Parameter check */

  if (stopparam == NULL)
    {
      return false;
    }

  /* Stop */

  FrontendCommand cmd;

  cmd.stop_param = *stopparam;

  err_t er = MsgLib::send<FrontendCommand>(s_msgq_id.frontend,
                                           MsgPriNormal,
                                           MSG_AUD_FED_CMD_STOP,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_InitPreprocFrontend(FAR AsInitPreProcParam *initpreparam)
{
  /* Parameter check */

  if (initpreparam == NULL)
    {
      return false;
    }

  /* Init PreProcess */

  FrontendCommand cmd;

  cmd.initpreproc_param = *initpreparam;

  err_t er = MsgLib::send<FrontendCommand>(s_msgq_id.frontend,
                                           MsgPriNormal,
                                           MSG_AUD_FED_CMD_INITPREPROC,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_SetPreprocFrontend(FAR AsSetPreProcParam *setpreparam)
{
  /* Parameter check */

  if (setpreparam == NULL)
    {
      return false;
    }

  /* Set PreProcess */

  FrontendCommand cmd;

  cmd.setpreproc_param = *setpreparam;

  err_t er = MsgLib::send<FrontendCommand>(s_msgq_id.frontend,
                                           MsgPriNormal,
                                           MSG_AUD_FED_CMD_SETPREPROC,
                                           NULL,
                                           cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_SetMicGainFrontend(FAR AsFrontendMicGainParam *micgain_param)
{
  if (micgain_param == NULL)
    {
      return false;
    }

  FrontendCommand cmd;

  cmd.mic_gain_param = *micgain_param;

  err_t er = MsgLib::send<FrontendCommand>(s_msgq_id.frontend,
                                           MsgPriNormal,
                                           MSG_AUD_FED_CMD_SETMICGAIN,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivateFrontend(FAR AsDeactivateFrontendParam *deactparam)
{
  FrontendCommand cmd;

  err_t er = MsgLib::send<FrontendCommand>(s_msgq_id.frontend,
                                           MsgPriNormal,
                                           MSG_AUD_FED_CMD_DEACT,
                                           NULL,
                                           cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeleteFrontend(void)
{
  if (s_fed_obj == NULL)
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  task_delete(s_fed_pid);
  delete s_fed_obj;
  s_fed_obj = NULL;

  /* Unregister attention callback */

  FRONT_END_UNREG_ATTCB();

  return true;
}


/*--------------------------------------------------------------------------*/
void FrontEndObject::create(AsFrontendMsgQueId_t msgq_id,
                            AsFrontendPoolId_t pool_id)
{
  if (s_fed_obj == NULL)
    {
      s_fed_obj = new FrontEndObject(msgq_id,
                                     pool_id);
      s_fed_obj->run();
    }
  else
    {
      FRONT_END_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }
}

