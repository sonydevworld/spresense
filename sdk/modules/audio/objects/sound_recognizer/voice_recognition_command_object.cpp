/****************************************************************************
 * modules/audio/objects/sound_recognizer/voice_recognition_command_object.cpp
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
#include "voice_recognition_command_object.h"
#include "components/recognition/voice_recognition_command_component.h"
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

static MsgQueId s_self_dtq;
static MsgQueId s_manager_dtq;
static MsgQueId s_dsp_dtq;
static PoolId   s_in_pool_id;
static pid_t    s_recognizer_pid;

/* TODO: Hide in class. */

static VoiceRecognitionCommandObject *s_rcg_obj = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void VoiceRecognitionCommandObject::act(void)
{
  uint32_t dsp_inf = 0;

  RECOGNITION_OBJ_DBG("ACT:\n");

  if (AS_voiceCmdCmpActivate(s_self_dtq, s_dsp_dtq, &dsp_inf) != 0)
    {
      /* Do nothing. */
    }
}

void VoiceRecognitionCommandObject::deact(void)
{
  RECOGNITION_OBJ_DBG("DEACT:\n");

  if (AS_voiceCmdCmpDeactivate() != 0)
    {
      /* Do nothing. */
    }
}

void VoiceRecognitionCommandObject::start(void)
{
  RECOGNITION_OBJ_DBG("START:\n");

  /* Initialize VAD/WuwSrlibrarys. */

  InitVoiceCmdCompReqParam_t init_param;
  init_param.vad_only    = m_vad_only;
  init_param.p_vad_param = m_vad_param;
  AS_voiceCmdCmpInit(&init_param);

  /* Initialize vad control parameters. */

  m_vad_previous_status = false;
  m_vad_buf_index       = 0;
}

void VoiceRecognitionCommandObject::stop(void)
{
  RECOGNITION_OBJ_DBG("STOP:\n");

  AS_voiceCmdCmpFlush();

  if (m_vad_buf_index != 0)
    {
      freeVadInBuf();
    }
}

VoiceRecognitionCommandObject::state_table_param_t
  VoiceRecognitionCommandObject::stateTbl[StateMax][StateMax] =
{
  /* Current status: stop. */

  {
    /* Setting status: stop. */

    {
      false, /* Can not transit. */
      NULL   /* Null pointer.    */
    },

    /* Setting status: start. */

    {
      true,                                 /* Can transit.      */
      &VoiceRecognitionCommandObject::start /* Function pointer. */
    },
  },

  /* Current status: start. */

  {
    /* Setting status: stop. */

    {
      true,                                /* Can transit.      */
      &VoiceRecognitionCommandObject::stop /* Function pointer. */
    },

    /* Setting status: start. */

    {
      false, /* Can not transit. */
      NULL   /* Null pointer.    */
    },
  }
};

void VoiceRecognitionCommandObject::setStatus(state_e status)
{
  if (stateTbl[m_status.get()][status].transition)
    {
      if (stateTbl[m_status.get()][status].p_func != NULL)
        {
          (this->*stateTbl[m_status.get()][status].p_func)();
        }

      m_status = status;
    }
}

void VoiceRecognitionCommandObject::setCommandParam(MsgPacket *msg)
{
  VoiceRecognitionCommandObject::CommandSetParam_t command_param =
    msg->moveParam<VoiceRecognitionCommandObject::CommandSetParam_t>();

  RECOGNITION_OBJ_DBG("SET CMD PARAM: vad only %d, keyword %d\n",
                      command_param.vad_only, command_param.key_word);

  m_vad_only  = command_param.vad_only;
  m_key_word  = command_param.key_word;
  m_vad_param = command_param.p_vad_param;
}

void VoiceRecognitionCommandObject::exec(MsgPacket *msg)
{
  VoiceRecognitionCommandObject::CommandExecParam_t exec_param =
    msg->moveParam<VoiceRecognitionCommandObject::CommandExecParam_t>();
  if (m_status.get() == StateStart)
    {

      if (m_vad_buf_index == 0)
        {
          m_vad_buf_pointer = reinterpret_cast<uint8_t*>(allocVadInBuf());
        }
      D_ASSERT(exec_param.size == MFE_OUTPUT_DATA_SIZE);

      /* Store voice data to local buffer, because it is not enough data size
       * for processing in one notification.
       */

      memcpy(reinterpret_cast<void *>
        (m_vad_buf_pointer + (exec_param.size * m_vad_buf_index)),
        reinterpret_cast<void *>(exec_param.address),
        exec_param.size);

      if (++m_vad_buf_index == VAD_FILL_CNT)
        {
          m_vad_buf_index = 0;

          /* Execute when data reaches the size of one processing. */

          ExecVoiceCmdCompReqParam_t req_param;
          req_param.address = reinterpret_cast<uint32_t>(m_vad_buf_pointer);
          req_param.sample_num = VAD_IN_SAMPLE_SIZE;
          AS_voiceCmdCmpExec(&req_param);
      }
    }
}

void VoiceRecognitionCommandObject::notify(MsgPacket *msg)
{
  VoiceRecognitionCommandObject::CommandResultParam_t result_param =
    msg->moveParam<VoiceRecognitionCommandObject::CommandResultParam_t>();

  switch(result_param.type)
    {
      case EventCmpltInit:
        sendAudioCmdCmplt(AUDCMD_STARTVOICECOMMAND, AS_ECODE_OK);
        break;

      case EventCmpltFlush:
        sendAudioCmdCmplt(AUDCMD_STOPVOICECOMMAND, AS_ECODE_OK);
        break;

      case EventCmpltExec:
        if (m_status.get() == StateStart)
          {
            if (0 == result_param.result)
              {
                AudioFindCommandInfo info;
                info.keyword = m_key_word;
                if ((m_vad_only == 0) && (result_param.is_wuwsr_found))
                  {
                    /* Notify that keword has been recognized. */

                    info.status = 2;
                    notifyAudioCommandInfo(info);
                  }
                else if (result_param.is_vad_found)
                  {
                    if (!m_vad_previous_status)
                      {
                        /* Notify that VAD has been recognized. */

                        info.status = 1;
                        notifyAudioCommandInfo(info);
                      }
                  }
                else
                  {
                    if (m_vad_previous_status == true)
                      {
                        /* Notify that VAD can not be recognized. */

                        info.status = 0;
                        notifyAudioCommandInfo(info);
                      }
                  }

                /* VAD result is stored and used as the next reference. */

                m_vad_previous_status = result_param.is_vad_found;
              }
          }
          freeVadInBuf();
          break;

      default:
        RECOGNITION_OBJ_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}

void VoiceRecognitionCommandObject::parse(MsgPacket *msg)
{
  switch(msg->getType())
    {
      case MSG_AUD_RCG_ACT:
        act();
        break;

      case MSG_AUD_RCG_DEACT:
        deact();
        break;

      case MSG_AUD_RCG_START:
        setCommandParam(msg);
        setStatus(StateStart);
        break;

      case MSG_AUD_RCG_STOP:
        setStatus(StateStop);
        break;

      case MSG_AUD_RCG_EXEC:
        exec(msg);
        break;

      case MSG_AUD_RCG_VOICE_CMPLT:
        notify(msg);
        break;

      default:
        RECOGNITION_OBJ_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        break;
    }
}

void VoiceRecognitionCommandObject::run()
{
  err_t        err_code;
  MsgQueBlock *que;
  MsgPacket   *msg;

  err_code = MsgLib::referMsgQueBlock(m_self, &que);
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

void *VoiceRecognitionCommandObject::allocVadInBuf()
{
  MemMgrLite::MemHandle mh;
  if (mh.allocSeg(s_in_pool_id, VAD_IN_DATA_SIZE) != ERR_OK)
    {
      RECOGNITION_OBJ_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return NULL;
    }

  vad_in_data_s data;
  data.mh = mh;
  if (!m_vad_in_buf_mh_que.push(data))
    {
      RECOGNITION_OBJ_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return NULL;
    }
  return mh.getPa();
}

void VoiceRecognitionCommandObject::freeVadInBuf()
{
  if (!m_vad_in_buf_mh_que.empty())
    {
      if (!m_vad_in_buf_mh_que.pop())
        {
          RECOGNITION_OBJ_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int AS_VoiceCmdObjEntry(int argc, char *argv[])
{
  VoiceRecognitionCommandObject::create(s_self_dtq, s_manager_dtq);
  return 0;
}

bool AS_CreateRecognizer(FAR AsCreateRecognizerParam_t *param)
{
  return AS_CreateRecognizer(param, NULL);
}

bool AS_CreateRecognizer(FAR AsCreateRecognizerParam_t *param, AudioAttentionCb attcb)
{
  /* Register attention callback */ 

  RECOGNITION_OBJ_REG_ATTCB(attcb);

  /* Parameter check */

  if (param == NULL)
    {
      RECOGNITION_OBJ_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Create */

  s_self_dtq    = param->msgq_id.recognizer;
  s_manager_dtq = param->msgq_id.mng;
  s_dsp_dtq     = param->msgq_id.dsp;
  s_in_pool_id  = param->pool_id.wuwsr_in;

  s_recognizer_pid = task_create("VCMD_OBJ",
                                 150,
                                 2048,
                                 AS_VoiceCmdObjEntry,
                                 NULL);
  if (s_recognizer_pid < 0)
    {
      RECOGNITION_OBJ_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  return true;
}

bool AS_DeleteRecognizer(void)
{
  task_delete(s_recognizer_pid);
  delete s_rcg_obj;
  s_rcg_obj = NULL;

  /* Unregister attention callback */ 

  RECOGNITION_OBJ_UNREG_ATTCB();

  return true;
}

void VoiceRecognitionCommandObject::create(MsgQueId msgq_id,
                                           MsgQueId manager_msg_id)
{
  if (s_rcg_obj == NULL)
    {
      s_rcg_obj = new VoiceRecognitionCommandObject(msgq_id,
                                                    manager_msg_id);
      s_rcg_obj->run();
    }
  else
    {
      RECOGNITION_OBJ_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
    }
}
