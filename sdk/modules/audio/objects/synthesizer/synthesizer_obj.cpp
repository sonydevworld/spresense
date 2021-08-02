/****************************************************************************
 * modules/audio/objects/synthesizer/synthesizer_obj.cpp
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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
#include "memutils/common_utils/common_assert.h"
#include "synthesizer_obj.h"
#include "components/oscillator/oscillator_component.h"
#include "debug/dbg_log.h"

__USING_WIEN2
using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Minimum number of samples (bytes) */

#define MIN_SAMPLE_SIZE   240

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool osc_done_callback(OscCmpltParam *param, void *instance)
{
  SynthesizerObject *obj = (SynthesizerObject *)instance;

  SynthesizerCommand cmd;
  MsgType            type = MSG_AUD_SYN_CMD_NEXT_REQ;

  if (param->event_type == Apu::FlushEvent)
    {
      cmd.comp_param.is_end = true;
    }
  else if (param->event_type == Apu::SetParamEvent)
    {
      type = MSG_AUD_SYN_CMD_SET_DONE;
    }
  else if (param->event_type == Apu::ExecEvent)
    {
      cmd.comp_param.addr   = param->exec_osc_param.buffer.p_buffer;
      cmd.comp_param.size   = param->exec_osc_param.buffer.size;
      cmd.comp_param.is_end = false;
    }
  else
    {
      cmd.comp_param.addr   = NULL;
      cmd.comp_param.size   = 0;
      cmd.comp_param.is_end = false;
    }

  err_t er = obj->send(type, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/* ------------------------------------------------------------------------ */
static void pcm_proc_done_callback(int32_t identifier, bool is_end)
{
  SynthesizerObject*  obj = SynthesizerObject::get_instance();

  SynthesizerCommand cmd;

  cmd.comp_param.addr   = NULL;
  cmd.comp_param.size   = 0;
  cmd.comp_param.is_end = is_end;

  err_t er = obj->send(MSG_AUD_SYN_CMD_DONE, cmd);

  F_ASSERT(er == ERR_OK);
}

/* ------------------------------------------------------------------------ */
static void AS_SynthesizerObjectEntry(FAR void *arg)
{
  SynthesizerObject::create((AsObjectParams_t *)arg);
}

/* ------------------------------------------------------------------------ */
static bool CreateSynthesizer(AsObjectParams_t params, AudioAttentionCb attcb)
{
  /* Register attention callback */

  SYNTHESIZER_REG_ATTCB(attcb);

  /* Clear */

  FAR MsgQueBlock *que;

  F_ASSERT(MsgLib::referMsgQueBlock(params.msgq_id.self, &que) == ERR_OK);

  que->reset();

  /* Init pthread attributes object. */

  pthread_attr_t attr;

  pthread_attr_init(&attr);

  /* Set pthread scheduling parameter. */

  struct sched_param sch_param;

  pthread_t pid;

  sch_param.sched_priority = 150;
  attr.stacksize           = 1024 * 2;

  pthread_attr_setschedparam(&attr, &sch_param);

  /* Create thread. */

  int ret = pthread_create(&pid,
                           &attr,
                           (pthread_startroutine_t)AS_SynthesizerObjectEntry,
                           (pthread_addr_t) &params);
  if (ret < 0)
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  pthread_setname_np(pid, "synthesizer");

  SynthesizerObject::set_pid(pid);

  return true;
}

/* ------------------------------------------------------------------------ */
static bool CreateSynthesizer(AsSynthesizerMsgQueId_t syn_msgq_id, AsSynthesizerPoolId_t syn_pool_id, AudioAttentionCb attcb)
{
  AsObjectParams_t params;

  params.msgq_id.self   = syn_msgq_id.synthesizer;
  params.msgq_id.from   = syn_msgq_id.mng;
  params.msgq_id.cmp    = syn_msgq_id.dsp;
  params.pool_id.output = syn_pool_id.output;
  params.pool_id.cmp    = syn_pool_id.dsp;

  return CreateSynthesizer(params, attcb);
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::create(AsObjectParams_t* params)
{
  SynthesizerObject* inst = new(SynthesizerObject::get_adr()) SynthesizerObject(params->msgq_id,params->pool_id);
  if (inst != NULL)
    {
      inst->run();
    }
  else
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
    }
}

/* ------------------------------------------------------------------------ */
SynthesizerObject::MsgProc SynthesizerObject::MsgResultTbl[AUD_SYN_RST_MSG_NUM][StateNum] =
{
  /* Message type: MSG_AUD_SYN_CMD_NEXT_REQ. */

  {
    &SynthesizerObject::illegalCompDone,     /*   Booted            */
    &SynthesizerObject::illegalCompDone,     /*   Ready             */
    &SynthesizerObject::nextReqOnExec,       /*   PreActive         */
    &SynthesizerObject::cmpDoneOnExec,       /*   Active            */
    &SynthesizerObject::nextReqOnStopping,   /*   Stopping          */
    &SynthesizerObject::nextReqOnStopping,   /*   ErrorStopping     */
    &SynthesizerObject::illegalCompDone      /*   WaitStop          */

  },

  /* Message type: MSG_AUD_SYN_CMD_DONE. */

  {
    &SynthesizerObject::illegalCompDone,     /*   Booted            */
    &SynthesizerObject::illegalCompDone,     /*   Ready             */
    &SynthesizerObject::illegalCompDone,     /*   PreActive         */
    &SynthesizerObject::cmpDoneOnExec,       /*   Active            */
    &SynthesizerObject::cmpDoneOnStopping,   /*   Stopping          */
    &SynthesizerObject::cmpDoneOnErrorStop,  /*   ErrorStopping     */
    &SynthesizerObject::illegalCompDone      /*   WaitStop          */
  },

  /* Message type: MSG_AUD_SYN_CMD_SET_DONE. */

  {
    &SynthesizerObject::illegalCompDone,     /*   Booted            */
    &SynthesizerObject::cmpDoneOnSet,        /*   Ready             */
    &SynthesizerObject::cmpDoneOnSet,        /*   PreActive         */
    &SynthesizerObject::cmpDoneOnSet,        /*   Active            */
    &SynthesizerObject::illegalCompDone,     /*   Stopping          */
    &SynthesizerObject::illegalCompDone,     /*   ErrorStopping     */
    &SynthesizerObject::illegalCompDone      /*   WaitStop          */
  },
};

/*--------------------------------------------------------------------------*/
void SynthesizerObject::parseResult(MsgPacket *msg)
{
  uint32_t event = MSG_GET_SUBTYPE(msg->getType());
  F_ASSERT((event < AUD_SYN_RST_MSG_NUM));

  (this->*MsgResultTbl[event][m_state.get()])(msg);
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::reply(AsSynthesizerEvent evtype, MsgType msg_type, uint32_t result)
{
  if (m_callback != NULL)
    {
      m_callback(evtype, result, m_param);
    }
  else if (m_msgq_id.from != MSG_QUE_NULL)
    {
      AudioObjReply cmplt((uint32_t)msg_type,
                           AS_OBJ_REPLY_TYPE_REQ,
                           AS_MODULE_ID_SYNTHESIZER_OBJ,
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

/* ------------------------------------------------------------------------ */
void SynthesizerObject::illegal(MsgPacket *msg)
{
  uint msgtype = msg->getType();

  msg->moveParam<SynthesizerCommand>();

  /* Extract and abandon message data */

  uint32_t idx = msgtype - MSG_AUD_SYN_CMD_ACT;

  AsSynthesizerEvent table[] =
  {
    AsSynthesizerEventAct,
    AsSynthesizerEventInit,
    AsSynthesizerEventStart,
    AsSynthesizerEventStop,
    AsSynthesizerEventDeact,
    AsSynthesizerEventSet
  };

  reply(table[idx], (MsgType)msgtype, AS_ECODE_STATE_VIOLATION);
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::activateOnBooted(MsgPacket *msg)
{
  AsActivateSynthesizer act    = msg->moveParam<SynthesizerCommand>().act_param;
  uint32_t              result = AS_ECODE_OK;

  /* Set event callback */

  m_callback = act.cb;
  m_param    = act.param;

  /* Active */

  reply(AsSynthesizerEventAct, msg->getType(), result);

  m_state = Ready;
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::deactivateOnReady(MsgPacket *msg)
{
  uint32_t  result = AS_ECODE_OK;

  msg->moveParam<SynthesizerCommand>();

  SYNTHESIZER_OBJ_DBG("DEACT:\n");

  if (m_dsp_path[0] == '\0')
    {
      /* Does nothing when DSP is not loaded */

      result = AS_ECODE_STATE_VIOLATION;
    }
  else
    {
      if (!m_oscillator.deactivate())
        {
          result = AS_ECODE_DSP_UNLOAD_ERROR;
        }

      memset(m_dsp_path, 0, sizeof(m_dsp_path));

      m_state = Booted;
    }

  reply(AsSynthesizerEventDeact, msg->getType(), result);
}

/* ------------------------------------------------------------------------ */
uint32_t SynthesizerObject::check_parameter(AsInitSynthesizerParam *p)
{
  uint32_t  result = AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST;

  /* Waveform check */

  if (p->type >= AsSynthesizerWaveModeNum)
    {
      SYNTHESIZER_OBJ_ERR(result);
    }

  /* Number of channels checking */

  else if (p->channel_num != 1 &&
           p->channel_num != 2 &&
           p->channel_num != 4 &&
           p->channel_num != 6 &&
           p->channel_num != 8)
    {
      SYNTHESIZER_OBJ_ERR(result);
    }

  /* Bit depth checking */

  else if (p->bit_width != AS_BITLENGTH_16 &&
           p->bit_width != AS_BITLENGTH_24)
    {
      SYNTHESIZER_OBJ_ERR(result);
    }

  /* Sampling rate checking */

  else if (p->sampling_rate != AS_SAMPLINGRATE_16000 &&
           p->sampling_rate != AS_SAMPLINGRATE_48000 &&
           p->sampling_rate != AS_SAMPLINGRATE_96000 &&
           p->sampling_rate != AS_SAMPLINGRATE_192000)
    {
      SYNTHESIZER_OBJ_ERR(result);
    }
  /* Effect checking */

  else if (p->attack  == 0  ||
           p->decay   == 0  ||
           p->sustain > 100 ||
           p->release == 0)
    {
      SYNTHESIZER_OBJ_ERR(result);
    }
  else
    {
      /* Sample size checking */

      uint32_t  max_sample = (Manager::getPoolSize(m_pool_id.output)) /
                             (Manager::getPoolNumSegs(m_pool_id.output));

      max_sample /= (m_bit_length / 8);
      max_sample /= (p->channel_num == 1 ? 2 : p->channel_num);

      if (p->sample_size == 0)
        {
          /* Specify the minimum value as default */

          p->sample_size = MIN_SAMPLE_SIZE;
        }
      else if (MIN_SAMPLE_SIZE > p->sample_size ||
               p->sample_size  > max_sample)
        {
          /* Warn and set default value */

          SYNTHESIZER_OBJ_WARN(result);

          p->sample_size = MIN_SAMPLE_SIZE;
        }

      result = AS_ECODE_OK;
    }

  return result;
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::initOnReady(MsgPacket *msg)
{
  AsInitSynthesizerParam param   = msg->moveParam<SynthesizerCommand>().init_param;
  uint32_t               result  = AS_ECODE_OK;
  uint32_t               dsp_inf = 0;

  SYNTHESIZER_OBJ_DBG(
        "INIT: type %d, ch num %d, bit len %d, sampling_rate %ld sample %d\n",
        param.type,
        param.channel_num,
        param.bit_width,
        param.sampling_rate,
        param.sample_size);

  if (strcmp(m_dsp_path, param.dsp_path) != 0)
    {
      if (m_dsp_path[0] != '\0')
        {
          m_oscillator.deactivate();
        }

      result = m_oscillator.activate(m_msgq_id.cmp,
                                     m_pool_id.cmp,
                                     param.dsp_path,
                                     &dsp_inf);
    }

  if (result != AS_ECODE_OK)
    {
      SYNTHESIZER_OBJ_ERR(result);
    }
  else if ((result = check_parameter(&param)) == AS_ECODE_OK)
    {
      /* Set parameter */

      strncpy(m_dsp_path, param.dsp_path, sizeof(m_dsp_path));

      m_sample_size = param.sample_size;

      m_bit_length = param.bit_width;

      /* Extended to 2ch for Mixer convenience */

      m_pcm_buff_size = param.sample_size  *
                        (m_bit_length / 8) *
                        (param.channel_num == 1 ? 2 : param.channel_num);

      /* PCM data output destination setting */

      m_data_path = param.data_path;
      m_dest      = param.dest;

      /* Callback, parameter setting */

      InitOscParam  osc_init;

      osc_init.type          = (Wien2::WaveMode)param.type;
      osc_init.sampling_rate = param.sampling_rate;
      osc_init.channel_num   = param.channel_num;
      osc_init.bit_length    = m_bit_length;
      osc_init.callback      = osc_done_callback;
      osc_init.instance      = this;
      osc_init.env.attack    = param.attack;
      osc_init.env.decay     = param.decay;
      osc_init.env.sustain   = param.sustain;
      osc_init.env.release   = param.release;

      /* Set oscillator initialize */

      m_oscillator.init(osc_init, &dsp_inf);
    }

  reply(AsSynthesizerEventInit, msg->getType(), result);
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::startOnReady(MsgPacket *msg)
{
  uint32_t  result = AS_ECODE_OK;

  msg->moveParam<SynthesizerCommand>();

  if (m_dsp_path[0] == '\0')
    {
      /* Does nothing when DSP is not loaded */

      result = AS_ECODE_STATE_VIOLATION;
    }
  else
    {
      /* First oscillator request */

      if (!oscillator_exec())
        {
          result = AS_ECODE_DSP_EXEC_ERROR;
        }
      else
        {
          m_state = PreActive;
        }
    }

  reply(AsSynthesizerEventStart, msg->getType(), result);
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::stopOnActive(MsgPacket *msg)
{
  uint32_t  result = AS_ECODE_OK;

  msg->moveParam<SynthesizerCommand>();

  MemHandle mh;

  if (mh.allocSeg(m_pool_id.output, m_pcm_buff_size) != ERR_OK)
    {
      result = AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR;
    }
  else if (!m_pcm_buf_mh_que.push(mh))
    {
      result = AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR;
    }
  else if (!m_oscillator.flush())
    {
      result = AS_ECODE_DSP_STOP_ERROR;
    }

  if (result == AS_ECODE_OK)
    {
      m_state = Stopping;
    }
  else
    {
      reply(AsSynthesizerEventStop, msg->getType(), result);
    }
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::stopOnErrorStopping(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  /* If a command is received here,
   * no response is returned until the output is completed.
   */

  m_stock_event = AsSynthesizerEventStop;
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::stopOnWait(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  reply(AsSynthesizerEventStop, msg->getType(), AS_ECODE_OK);

  m_state = Ready;
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::set(MsgPacket *msg)
{
  SetOscParam param = msg->moveParam<SynthesizerCommand>().set_param;

  if (m_oscillator.set(param) == false)
    {
      /* Returns a response immediately when a command cannot be sent */

      reply(AsSynthesizerEventStart, msg->getType(), AS_ECODE_STATE_VIOLATION);
    }
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::illegalCompDone(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();
  SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::nextReqOnExec(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  if (m_pcm_buf_mh_que.size() < MAX_OUT_BUFF_NUM)
    {
      /* Next oscillator request */

      if (!oscillator_exec())
        {
          m_state = ErrorStopping;
        }
    }
  else
    {
      while (!m_pcm_buf_mh_que.empty())
        {
          /* Next mixer request */

          sendPcmToOwner(m_pcm_buf_mh_que.top(), false);

          m_pcm_buf_mh_que.pop();
        }

      /* Next request */

      if (oscillator_exec())
        {
          m_state = Active;
        }
      else
        {
          m_state = ErrorStopping;
        }
    }
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::nextReqOnStopping(MsgPacket *msg)
{
  OscCompComplete param = msg->moveParam<SynthesizerCommand>().comp_param;

  /* End mixer request */

  sendPcmToOwner(m_pcm_buf_mh_que.top(), param.is_end);

  m_pcm_buf_mh_que.pop();
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::cmpDoneOnExec(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  while (!m_pcm_buf_mh_que.empty())
    {
      sendPcmToOwner(m_pcm_buf_mh_que.top(), false);

      m_pcm_buf_mh_que.pop();
    }

  /* Next request. Free one pool for terminal processing */

  if (Manager::getPoolNumAvailSegs(m_pool_id.output) > 1)
    {
      if (!oscillator_exec())
        {
          m_state = ErrorStopping;
        }
    }
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::cmpDoneOnStopping(MsgPacket *msg)
{
  OscCompComplete param = msg->moveParam<SynthesizerCommand>().comp_param;

  if (param.is_end)
    {
      reply(AsSynthesizerEventStop, MSG_AUD_SYN_CMD_STOP, AS_ECODE_OK);

      m_state = Ready;
    }
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::cmpDoneOnSet(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  reply(AsSynthesizerEventSet, MSG_AUD_SYN_CMD_SET, AS_ECODE_OK);
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::cmpDoneOnErrorStop(MsgPacket *msg)
{
  OscCompComplete param = msg->moveParam<SynthesizerCommand>().comp_param;

  if (!param.is_end)
    {
      return;
    }

  if (m_stock_event == AsSynthesizerEventStop)
    {
      /* When the stop command is called in ErrorStopping state */

      m_stock_event = AsSynthesizerEventNum;

      reply(AsSynthesizerEventStop, MSG_AUD_SYN_CMD_STOP, AS_ECODE_OK);

      m_state = Ready;
    }
  else
    {
      m_state = WaitStop;
    }
}

/* ------------------------------------------------------------------------ */
void SynthesizerObject::sendPcmToOwner(MemHandle mh, bool is_end)
{
  /* Send message for PCM data notify */

  AsPcmDataParam  data;

  data.identifier = 0;
  data.callback   = pcm_proc_done_callback;
  data.sample     = m_sample_size;
  data.bit_length = m_bit_length;
  data.is_valid   = 1;
  data.size       = m_pcm_buff_size;
  data.mh         = mh;
  data.is_end     = is_end;

  if (m_data_path == AsSynthesizerDataPathCallback)
    {
      /* Call callback function for PCM data notify */

      F_ASSERT(m_dest.cb);

      m_dest.cb(data);
    }
  else if (m_data_path == AsSynthesizerDataPathMessage)
    {
      /* Send message for PCM data notify */

      data.identifier = m_dest.msg.identifier;

      err_t err = MsgLib::send<AsPcmDataParam>(m_dest.msg.id,
                                               MsgPriNormal,
                                               MSG_AUD_MIX_CMD_DATA,
                                               m_msgq_id.self,
                                               data);
      F_ASSERT(err == ERR_OK);
    }
}

/* ------------------------------------------------------------------------ */
bool SynthesizerObject::oscillator_exec(void)
{
  ExecOscParam  param;
  MemHandle     mh;

  if (mh.allocSeg(m_pool_id.output, m_pcm_buff_size) != ERR_OK)
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  if (!m_pcm_buf_mh_que.push(mh))
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  param.buffer.size     = m_pcm_buff_size;
  param.buffer.p_buffer = (unsigned long *)mh.getPa();

  if (!m_oscillator.exec(param))
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* ------------------------------------------------------------------------ */
bool AS_CreateMediaSynthesizer(FAR AsCreateSynthesizerParam_t *param,
                                   AudioAttentionCb            attcb)
{
  /* Parameter check */

  if (param == NULL)
    {
      return false;
    }

  return CreateSynthesizer(param->msgq_id, param->pool_id, attcb);
}

/* ------------------------------------------------------------------------ */
bool AS_CreateMediaSynthesizer(FAR AsCreateSynthesizerParam_t *param)
{
  return AS_CreateMediaSynthesizer(param, NULL);
}

/* ------------------------------------------------------------------------ */
bool AS_ActivateMediaSynthesizer(FAR AsActivateSynthesizer *actparam)
{
  /* Parameter check */

  if (actparam == NULL)
    {
      return false;
    }

  /* Activate */

  SynthesizerObject *obj = SynthesizerObject::get_instance();
  if (obj == NULL)
    {
      return false;
    }

  SynthesizerCommand cmd;

  cmd.act_param = *actparam;

  err_t er = obj->send(MSG_AUD_SYN_CMD_ACT, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/* ------------------------------------------------------------------------ */
bool AS_InitMediaSynthesizer(FAR AsInitSynthesizerParam *initparam)
{
  /* Parameter check */

  if (initparam == NULL)
    {
      return false;
    }

  /* Init */

  SynthesizerObject *obj = SynthesizerObject::get_instance();
  if (obj == NULL)
    {
      return false;
    }

  SynthesizerCommand cmd;

  cmd.init_param = *initparam;

  err_t er = obj->send(MSG_AUD_SYN_CMD_INIT, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/* ------------------------------------------------------------------------ */
bool AS_StartMediaSynthesizer(void)
{
  /* Start */

  SynthesizerObject *obj = SynthesizerObject::get_instance();
  if (obj == NULL)
    {
      return false;
    }

  SynthesizerCommand cmd;

  err_t er = obj->send(MSG_AUD_SYN_CMD_START, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/* ------------------------------------------------------------------------ */
bool AS_SetMediaSynthesizer(FAR AsSetSynthesizer *set_param)
{
  /* Set */

  SynthesizerObject *obj = SynthesizerObject::get_instance();
  if (obj == NULL)
    {
      return false;
    }

  SynthesizerCommand cmd;

  cmd.set_param.channel_no  = set_param->channel_no;
  cmd.set_param.type        = Apu::OscTypeFrequency | Apu::OscTypeEnvelope;
  cmd.set_param.frequency   = set_param->frequency;
  cmd.set_param.env.attack  = set_param->attack;
  cmd.set_param.env.decay   = set_param->decay;
  cmd.set_param.env.sustain = set_param->sustain;
  cmd.set_param.env.release = set_param->release;

  err_t er = obj->send(MSG_AUD_SYN_CMD_SET, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/* ------------------------------------------------------------------------ */
bool AS_ReleaseMediaSynthesizer(bool is_end)
{
  /* Discard PCM data */

  SynthesizerObject*  obj = SynthesizerObject::get_instance();
  if (obj == NULL)
    {
      return false;
    }

  SynthesizerCommand cmd;

  cmd.comp_param.addr   = NULL;
  cmd.comp_param.size   = 0;
  cmd.comp_param.is_end = is_end;

  err_t er = obj->send(MSG_AUD_SYN_CMD_DONE, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/* ------------------------------------------------------------------------ */
bool AS_StopMediaSynthesizer(void)
{
  /* Stop */

  SynthesizerObject *obj = SynthesizerObject::get_instance();
  if (obj == NULL)
    {
      return false;
    }

  SynthesizerCommand cmd;

  err_t er = obj->send(MSG_AUD_SYN_CMD_STOP, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/* ------------------------------------------------------------------------ */
bool AS_DeactivateMediaSynthesizer(void)
{
  SynthesizerObject *obj = SynthesizerObject::get_instance();
  if (obj == NULL)
    {
      return false;
    }

  SynthesizerCommand cmd;

  err_t er = obj->send(MSG_AUD_SYN_CMD_DEACT, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/* ------------------------------------------------------------------------ */
bool AS_DeleteMediaSynthesizer(void)
{
  pid_t pid = SynthesizerObject::get_pid();
  SynthesizerObject::destory();

  if (pid == INVALID_PROCESS_ID)
    {
      return false;
    }

  pthread_cancel(pid);
  pthread_join(pid, NULL);

  SynthesizerObject::set_pid(INVALID_PROCESS_ID);

  SYNTHESIZER_OBJ_UNREG_ATTCB();

  return true;
}

/* ------------------------------------------------------------------------ */
bool AS_SetFrequencyMediaSynthesizer(FAR AsSetSynthesizer *set_param)
{
  /* Set frequency */

  SynthesizerObject *obj = SynthesizerObject::get_instance();
  if (obj == NULL)
    {
      return false;
    }

  SynthesizerCommand cmd;

  cmd.set_param.channel_no  = set_param->channel_no;
  cmd.set_param.type        = Apu::OscTypeFrequency;
  cmd.set_param.frequency   = set_param->frequency;

  err_t er = obj->send(MSG_AUD_SYN_CMD_SET, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/* ------------------------------------------------------------------------ */
bool AS_SetEnvelopeMediaSynthesizer(FAR AsSetSynthesizer *set_param)
{
  /* Set envelope */

  SynthesizerObject *obj = SynthesizerObject::get_instance();
  if (obj == NULL)
    {
      return false;
    }

  SynthesizerCommand cmd;

  cmd.set_param.channel_no  = set_param->channel_no;
  cmd.set_param.type        = Apu::OscTypeEnvelope;
  cmd.set_param.env.attack  = set_param->attack;
  cmd.set_param.env.decay   = set_param->decay;
  cmd.set_param.env.sustain = set_param->sustain;
  cmd.set_param.env.release = set_param->release;

  err_t er = obj->send(MSG_AUD_SYN_CMD_SET, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}
