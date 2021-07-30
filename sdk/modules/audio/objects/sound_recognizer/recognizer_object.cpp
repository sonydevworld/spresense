/****************************************************************************
 * modules/audio/objects/sound_recognizer/recognizer_object.cpp
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "memutils/message/Message.h"
#include "recognizer_object.h"
#include "memutils/os_utils/chateau_osal.h"
#include "debug/dbg_log.h"

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
static bool recognition_done_callback(ComponentCbParam *cmplt, void* p_requester)
{
  RecognizerObject::RecognitionDoneCmd result_param;

  result_param.event_type = cmplt->event_type;
  result_param.result     = cmplt->result;

  err_t er = MsgLib::send<RecognizerObject::RecognitionDoneCmd>
                                                (RecognizerObject::get_msgq_id(),
                                                 MsgPriNormal,
                                                 MSG_AUD_RCG_RCG_CMPLT,
                                                 NULL,
                                                 result_param);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
RecognizerObject::MsgProc
  RecognizerObject::MsgParamTbl[AUD_RCG_PRM_NUM][StateNum] =
{
  /* Message Type: MSG_AUD_RCG_INITRCGPROC */
  {                                  /* Recognizer status: */
    &RecognizerObject::illegal,      /*   Booted.          */
    &RecognizerObject::initRcgproc,  /*   Ready.           */
    &RecognizerObject::illegal,      /*   PreActive.       */
    &RecognizerObject::illegal,      /*   Active.          */
    &RecognizerObject::illegal,      /*   Stopping.        */
    &RecognizerObject::illegal,      /*   ErrorStopping.   */
    &RecognizerObject::illegal,      /*   WaitStop.        */
  },

  /* Message Type: MSG_AUD_RCG_SETRCGPROC */

  {                                  /* MicFrontend status: */
    &RecognizerObject::illegal,      /*   Booted.          */
    &RecognizerObject::setRcgproc,   /*   Ready.           */
    &RecognizerObject::illegal,      /*   PreActive.       */
    &RecognizerObject::setRcgproc,   /*   Active.          */
    &RecognizerObject::illegal,      /*   Stopping.        */
    &RecognizerObject::setRcgproc,   /*   ErrorStopping.   */
    &RecognizerObject::illegal,      /*   WaitStop.        */
  }
};

/*--------------------------------------------------------------------------*/
RecognizerObject::MsgProc
  RecognizerObject::RsltProcTbl[AUD_RCG_RES_MSG_NUM][StateNum] =
{
  /* Message Type: MSG_AUD_RCG_RCG_CMPLT */

  {                                              /* Recognizer status: */
    &RecognizerObject::illegalRecognizeDone,     /*   Booted.          */
    &RecognizerObject::recognizeDoneOnReady,     /*   Ready.           */
    &RecognizerObject::illegalRecognizeDone,     /*   PreActive.       */
    &RecognizerObject::recognizeDoneOnActive,    /*   Active.          */
    &RecognizerObject::recognizeDoneOnStopping,  /*   Stopping.        */
    &RecognizerObject::illegalRecognizeDone,     /*   ErrorStopping.   */
    &RecognizerObject::illegalRecognizeDone,     /*   WaitStop.        */
  },
};

/*--------------------------------------------------------------------------*/
void RecognizerObject::illegal(MsgPacket *msg)
{
  uint msgtype = msg->getType();
  msg->moveParam<RecognizerCommand>();

  /* Reply */

  uint32_t idx = msgtype - MSG_AUD_RCG_ACT;

  AsRecognizerEvent table[] =
  {
    AsRecognizerEventAct,
    AsRecognizerEventDeact,
    AsRecognizerEventInit,
    AsRecognizerEventStart,
    AsRecognizerEventExec,
    AsRecognizerEventStop,
    AsRecognizerEventInitRecognizerProc,
    AsRecognizerEventSetRecognizerProc,
  };

  reply(table[idx], msgtype, AS_ECODE_STATE_VIOLATION);
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::activateOnBooted(MsgPacket *msg)
{
  RECOGNIZER_OBJ_DBG("ACT:\n");

  AsActivateRecognizerParam act_param =
    msg->moveParam<RecognizerCommand>().act_param;

  /* Set callback */

  m_callback = act_param.cb;

  /* Set state */

  m_state = Ready;

  /* Reply */

  reply(AsRecognizerEventAct, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::deactivateOnReady(MsgPacket *msg)
{
  RECOGNIZER_OBJ_DBG("DEACT:\n");

  msg->moveParam<RecognizerCommand>();

  /* Deactivate recognition proc */

  uint32_t ret = unloadComponent();
  if (ret != AS_ECODE_OK)
    {
      /* Error reply */

      reply(AsRecognizerEventDeact, msg->getType(), ret);

      return;
    }

  m_state = Booted;

  reply(AsRecognizerEventDeact, msg->getType(), ret);
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::initOnReady(MsgPacket *msg)
{
  RECOGNIZER_OBJ_DBG("INIT:\n");

  AsInitRecognizerParam init_param =
    msg->moveParam<RecognizerCommand>().init_param;

  m_notify_path = init_param.notify_path;
  m_notify_dest = init_param.dest;

  /* Check type and dsp name. If differ from current of them, do reload. */

  if ((m_recognizer_type != static_cast<AsRecognizerType>(init_param.type))
   || (strncmp(m_dsp_path, init_param.dsp_path, sizeof(m_dsp_path))))
    {
      uint32_t ret = unloadComponent();

      if (ret != AS_ECODE_OK)
        {
          /* Error reply */

          reply(AsRecognizerEventInit, msg->getType(), ret);
          return;
        }

      ret = loadComponent(static_cast<AsRecognizerType>(init_param.type),
                          init_param.dsp_path);

      if(ret != AS_ECODE_OK)
        {
          /* Error reply */

          reply(AsRecognizerEventInit, msg->getType(), ret);
          return;
        }
    }

  /* Reply */

  reply(AsRecognizerEventInit, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
uint32_t RecognizerObject::loadComponent(AsRecognizerType type, char *dsp_path)
{
  /* Create component */

  switch (type)
    {
      case AsRecognizerTypeUserCustom:
         m_p_rcgproc_instance = new UserCustomComponent(m_pool_id.cmp,
                                                        m_msgq_id.cmp);
         break;

      default:
         break;
    }

  if (m_p_rcgproc_instance == NULL)
    {
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  /* Activate recognition proc */

  uint32_t dsp_inf = 0;

  uint32_t ret = m_p_rcgproc_instance->activate(recognition_done_callback,
                                                dsp_path,
                                                static_cast<void *>(this),
                                                &dsp_inf);
  if (ret != AS_ECODE_OK)
    {
      delete m_p_rcgproc_instance;

      m_p_rcgproc_instance = NULL;

      return ret;
    }

  /* When load complete successfully, set type and dsp name. */

  m_recognizer_type = type;

  strncpy(m_dsp_path, dsp_path, sizeof(m_dsp_path) - 1);
  m_dsp_path[sizeof(m_dsp_path) -1] = '\0';

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t RecognizerObject::unloadComponent(void)
{
  if (m_p_rcgproc_instance == NULL)
    {
      return AS_ECODE_OK;
    }

  /* Deactivate recognition proc */

  bool ret = m_p_rcgproc_instance->deactivate();

  if (!ret)
    {
      return AS_ECODE_DSP_UNLOAD_ERROR;
    }

  delete m_p_rcgproc_instance;

  m_p_rcgproc_instance = NULL;

  /* When unload complete successfully, set invalid type and dsp_name. */

  m_recognizer_type = AsRecognizerTypeInvalid;

  memset(m_dsp_path, 0, sizeof(m_dsp_path));

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::startOnReady(MsgPacket *msg)
{
  RECOGNIZER_OBJ_DBG("START:\n");

  msg->moveParam<RecognizerCommand>();

  /* Initialize vad control parameters. */

  m_state = Active;

  /* Reply */

  reply(AsRecognizerEventStart, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::stopOnActive(MsgPacket *msg)
{
  RECOGNIZER_OBJ_DBG("STOP:\n");

  msg->moveParam<RecognizerCommand>();

  if (m_p_rcgproc_instance == NULL)
    {
      reply(AsRecognizerEventStop, msg->getType(), AS_ECODE_DSP_STOP_ERROR);
      return;
    }

  FlushComponentParam flush;

  /* Allocate output buffer */

  if (ERR_OK != flush.output.allocSeg(m_pool_id.output, VAD_IN_DATA_SIZE))
    {
      reply(AsRecognizerEventStop, msg->getType(), AS_ECODE_CHECK_MEMORY_POOL_ERROR);
      return;
    }

  /* Flush recognition proc */

  if (!m_p_rcgproc_instance->flush(flush))
    {
      reply(AsRecognizerEventStop, msg->getType(), AS_ECODE_DSP_STOP_ERROR);
      return;
    }

  m_state = Stopping;
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::initRcgproc(MsgPacket *msg)
{
  AsInitRecognizerProcParam initparam =
    msg->moveParam<RecognizerCommand>().initrcgproc_param;

  RECOGNIZER_OBJ_DBG("Init Recognizer Proc:\n");

  if (m_p_rcgproc_instance == NULL)
    {
      reply(AsRecognizerEventInitRecognizerProc,
            msg->getType(),
            AS_ECODE_DSP_SET_ERROR);
      return;
    }

  InitComponentParam param;

  param.is_userdraw = true;
  param.custom.addr = initparam.packet_addr;
  param.custom.size = initparam.packet_size;

  /* Init recognition proc (Copy packet to MH internally, and wait return from DSP) */

  uint32_t send_result = m_p_rcgproc_instance->init(param);

  m_p_rcgproc_instance->recv_done();

  /* Reply */

  reply(AsRecognizerEventInitRecognizerProc, msg->getType(), send_result);
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::setRcgproc(MsgPacket *msg)
{
  AsSetRecognizerProcParam setparam =
    msg->moveParam<RecognizerCommand>().setrcgproc_param;

  RECOGNIZER_OBJ_DBG("Set Recognizer Proc:\n");

  if (m_p_rcgproc_instance == NULL)
    {
      reply(AsRecognizerEventInitRecognizerProc,
            msg->getType(),
            AS_ECODE_DSP_SET_ERROR);
      return;
    }

  SetComponentParam param;

  param.is_userdraw = true;
  param.custom.addr = setparam.packet_addr;
  param.custom.size = setparam.packet_size;

  /* Set recognition proc (Copy packet to MH internally.) */

  bool send_result = m_p_rcgproc_instance->set(param);

  /* Reply */

  reply(AsRecognizerEventSetRecognizerProc,
        msg->getType(),
        (send_result) ? AS_ECODE_OK : AS_ECODE_DSP_SET_ERROR);
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::execOnActive(MsgPacket *msg)
{
  ExecComponentParam exec;

  exec.input = msg->moveParam<AsPcmDataParam>();

  if (m_p_rcgproc_instance == NULL)
    {
      RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  /* Allocate output buffer */

  if (ERR_OK != exec.output.allocSeg(m_pool_id.output, exec.input.size))
    {
      RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return;
    }

  /* Exec recognition proc */

  if (!m_p_rcgproc_instance->exec(exec))
    {
      RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
      return;
    }
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::ignore(MsgPacket *msg)
{
  /* Ignore, dispose PCM data. */

  msg->moveParam<AsPcmDataParam>();
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::recognizeDoneOnReady(MsgPacket *msg)
{
  RecognizerObject::RecognitionDoneCmd recog_result =
    msg->moveParam<RecognizerObject::RecognitionDoneCmd>();

  if (m_p_rcgproc_instance == NULL)
    {
      RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  switch(recog_result.event_type)
    {
      case ComponentInit:
      case ComponentSet:
        {
          m_p_rcgproc_instance->recv_done();
        }
        break;

      default:
        RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::recognizeDoneOnActive(MsgPacket *msg)
{
  RecognizerObject::RecognitionDoneCmd recog_result =
    msg->moveParam<RecognizerObject::RecognitionDoneCmd>();

  if (m_p_rcgproc_instance == NULL)
    {
      RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  switch(recog_result.event_type)
    {
      case ComponentInit:
      case ComponentSet:
        {
          m_p_rcgproc_instance->recv_done();
        }
        break;

      case ComponentExec:
        {
          /* Receive result as recognition notify. */

          ComponentInformParam info;
          m_p_rcgproc_instance->recv_done(&info);

          if (true == recog_result.result)
            {
              /* If inform request, notify to parent. */

              if (info.inform_req)
                {
                  notify(info.inform_data);
                }
            }
        }
        break;

      default:
        RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::recognizeDoneOnStopping(MsgPacket *msg)
{
  RecognizerObject::RecognitionDoneCmd recog_result =
    msg->moveParam<RecognizerObject::RecognitionDoneCmd>();

  if (m_p_rcgproc_instance == NULL)
    {
      RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  switch(recog_result.event_type)
    {
      case ComponentInit:
      case ComponentSet:
        {
          m_p_rcgproc_instance->recv_done();
        }
        break;

      case ComponentExec:
        {
          /* Receive result as recognition notify. */

          ComponentInformParam info;
          m_p_rcgproc_instance->recv_done(&info);

          if (true == recog_result.result)
            {
              /* If inform request, notify to parent. */

              if (info.inform_req)
                {
                  notify(info.inform_data);
                }
            }
        }
        break;

      case ComponentFlush:
        {
          m_p_rcgproc_instance->recv_done();

          m_state = Ready;
          reply(AsRecognizerEventStop, MSG_AUD_RCG_STOP, AS_ECODE_OK);
        }
        break;

      default:
        RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::illegalRecognizeDone(MsgPacket *msg)
{
  msg->moveParam<RecognizerObject::RecognitionDoneCmd>();

  if (m_p_rcgproc_instance)
    {
      m_p_rcgproc_instance->recv_done();
    }
}

/*--------------------------------------------------------------------------*/
bool RecognizerObject::notify(AsRecognitionInfo info)
{
  if(m_notify_path == AsNotifyPathCallback)
    {
      /* Call callback function for notify recognition result */

      if (m_notify_dest.cb != NULL)
        {
          m_notify_dest.cb(info);
        }
    }
  else
    {
      /* Send message for notify recognition result */

      if (m_notify_dest.msg.msgqid != MSG_QUE_NULL)
        {
          err_t er = MsgLib::send<AsRecognitionInfo>(m_notify_dest.msg.msgqid,
                                                     MsgPriNormal,
                                                     m_notify_dest.msg.msgtype,
                                                     m_msgq_id.self,
                                                     info);
          F_ASSERT(er == ERR_OK);
        }
    }

  return true;
}
/*--------------------------------------------------------------------------*/
void RecognizerObject::parseResult(MsgPacket *msg)
{
  uint32_t event = MSG_GET_SUBTYPE(msg->getType());
  F_ASSERT((event < AUD_RCG_RES_MSG_NUM));

  (this->*RsltProcTbl[event][m_state.get()])(msg);
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::set(MsgPacket *msg)
{
  uint32_t event;
  event = MSG_GET_PARAM(msg->getType());
  F_ASSERT((event < AUD_RCG_PRM_NUM));

  (this->*MsgParamTbl[event][m_state.get()])(msg);
}


/*--------------------------------------------------------------------------*/
void RecognizerObject::reply(AsRecognizerEvent event, uint32_t command_id, uint32_t result)
{
  RecognizerResult result_param;

  result_param.header.result_code = result;
  result_param.header.command_id  = command_id;
  result_param.header.event       = event;

  /* Reply */

  if (m_callback != NULL)
    {
      m_callback(&result_param);
    }
  else if (m_msgq_id.from != MSG_QUE_NULL)
    {
      err_t er = MsgLib::send<RecognizerResult>(m_msgq_id.from,
                                                MsgPriNormal,
                                                MSG_TYPE_AUD_RES,
                                                m_msgq_id.self,
                                                result_param);
      if (ERR_OK != er)
        {
          F_ASSERT(0);
        }
    }
  else
    {
      /* Cannot send reply */
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void AS_RecognizerObjEntry(FAR void *arg)
{
  RecognizerObject::create((AsObjectParams_t *)arg);
}

/*--------------------------------------------------------------------------*/
bool AS_CreateRecognizer(AsObjectParams_t params, AudioAttentionCb attcb)
{
  /* Register attention callback */

  RECOGNIZER_OBJ_REG_ATTCB(attcb);

  /* Reset Message queue. */

  FAR MsgQueBlock *que;
  err_t err_code = MsgLib::referMsgQueBlock(params.msgq_id.self, &que);
  F_ASSERT(err_code == ERR_OK);
  que->reset();

  /* Init pthread attributes object. */

  pthread_attr_t attr;

  pthread_attr_init(&attr);

  /* Set pthread scheduling parameter. */

  struct sched_param sch_param;

  pthread_t pid;

  sch_param.sched_priority = 150;
  attr.stacksize           = 2048;

  pthread_attr_setschedparam(&attr, &sch_param);

  /* Create thread. */

  int ret = pthread_create(&pid,
                           &attr,
                           (pthread_startroutine_t)AS_RecognizerObjEntry,
                           (pthread_addr_t) &params);

  if (ret < 0)
    {
      RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  pthread_setname_np(pid, "recognizer");

  RecognizerObject::set_pid(pid);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_CreateRecognizer(FAR AsCreateRecognizerParam_t *rcg_param, AudioAttentionCb attcb)
{
  AsObjectParams_t params;

  params.msgq_id.self   = rcg_param->msgq_id.recognizer;
  params.msgq_id.from   = rcg_param->msgq_id.mng;
  params.msgq_id.cmp    = rcg_param->msgq_id.dsp;
  params.pool_id.output = rcg_param->pool_id.out;
  params.pool_id.cmp    = rcg_param->pool_id.dsp;

  return AS_CreateRecognizer(params, attcb);
}

/*--------------------------------------------------------------------------*/
bool AS_CreateRecognizer(FAR AsCreateRecognizerParam_t *param)
{
  return AS_CreateRecognizer(param, NULL);
}

/*--------------------------------------------------------------------------*/
bool AS_DeleteRecognizer(void)
{
  pid_t pid = RecognizerObject::get_pid();
  RecognizerObject::destory();

  if (pid == INVALID_PROCESS_ID)
    {
      return false;
    }

  pthread_cancel(pid);
  pthread_join(pid, NULL);

  RecognizerObject::set_pid(INVALID_PROCESS_ID);

  /* Unregister attention callback */

  RECOGNIZER_OBJ_UNREG_ATTCB();
  usleep(10 * 1000); /* Wait for next creation */

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_checkAvailabilityRecognizer(void)
{
  return (RecognizerObject::get_instance() != NULL);
}

/*--------------------------------------------------------------------------*/
void RecognizerObject::create(AsObjectParams_t* params)
{
  RecognizerObject* inst = new(RecognizerObject::get_adr()) RecognizerObject(params->msgq_id,params->pool_id);
  if (inst != NULL)
    {
      inst->run();
    }
  else
    {
      RECOGNIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
    }
}

