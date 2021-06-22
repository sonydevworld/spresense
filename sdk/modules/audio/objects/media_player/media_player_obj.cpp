/****************************************************************************
 * modules/audio/objects/media_player/media_player_obj.cpp
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
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <string.h>
#include <stdlib.h>
#include "memutils/os_utils/os_wrapper.h"
#include "memutils/common_utils/common_assert.h"
#include "media_player_obj.h"
#include "components/decoder/decoder_component.h"
#include "dsp_driver/include/dsp_drv.h"
#include "debug/dbg_log.h"

__USING_WIEN2
using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef SUPPORT_SBC_PLAYER
#  define NUM_OF_AU 8
#else
/* Note: Codectypes other than SBC do not adopt the concept of super AU. */
#  define NUM_OF_AU 1
#endif /* SUPPORT_SBC_PLAYER */

/* Definition when not using Memory pool. */

#define SRC_WORK_BUF_SIZE 8192 /* 1024sample * 2ch * 4bytes */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_t s_ply_pid = INVALID_PROCESS_ID;
static pthread_t s_sub_ply_pid = INVALID_PROCESS_ID;

static AsPlayerMsgQueId_t s_msgq_id;
static AsPlayerMsgQueId_t s_sub_msgq_id;
static AsPlayerPoolId_t   s_pool_id;
static AsPlayerPoolId_t   s_sub_pool_id;
static AsPlayerId         s_player_id;
static AsPlayerId         s_sub_player_id;

static void *s_play_obj = NULL;
static void *s_sub_play_obj = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*--------------------------------------------------------------------------*/
static bool decoder_comp_done_callback(void *p_response, FAR void *p_requester)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;
  DecCmpltParam cmplt;

  if (DSP_COM_DATA_TYPE_STRUCT_ADDRESS != p_param->type)
    {
      MEDIA_PLAYERS_ERR((static_cast<FAR PlayerObj *>(p_requester))->get_playerId(),
                        AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
      return false;
    }

  Apu::Wien2ApuCmd *packet = reinterpret_cast<Apu::Wien2ApuCmd *>
    (p_param->data.pParam);

  cmplt.event_type = static_cast<Wien2::Apu::ApuEventType>
    (packet->header.event_type);

  switch (packet->header.event_type)
    {
      case Apu::ExecEvent:
        {
          cmplt.exec_dec_cmplt.input_buffer   = packet->exec_dec_cmd.input_buffer;
          cmplt.exec_dec_cmplt.output_buffer  = packet->exec_dec_cmd.output_buffer;
          cmplt.exec_dec_cmplt.is_valid_frame =
            ((packet->result.exec_result == Apu::ApuExecOK) ? true : false);

          err_t er = MsgLib::send<DecCmpltParam>((static_cast<FAR PlayerObj *>
                                                  (p_requester))->get_selfId(),
                                                 MsgPriNormal,
                                                 MSG_AUD_PLY_CMD_DEC_DONE,
                                                 NULL,
                                                 cmplt);
          F_ASSERT(er == ERR_OK);
        }
        break;

    case Apu::FlushEvent:
      {
        cmplt.stop_dec_cmplt.output_buffer  = packet->flush_dec_cmd.output_buffer;
        cmplt.stop_dec_cmplt.is_valid_frame =
          ((packet->result.exec_result == Apu::ApuExecOK) ? true : false);

        MEDIA_PLAYER_VDBG("FlsDec s %ld v %d\n",
                          cmplt.stop_dec_cmplt.output_buffer.size,
                          cmplt.stop_dec_cmplt.is_valid_frame);

        err_t er = MsgLib::send<DecCmpltParam>((static_cast<FAR PlayerObj *>
                                                (p_requester))->get_selfId(),
                                               MsgPriNormal,
                                               MSG_AUD_PLY_CMD_DEC_DONE,
                                               NULL,
                                               cmplt);
        F_ASSERT(er == ERR_OK);
      }
      break;

    case Apu::SetParamEvent:
      {
        cmplt.setparam_dec_cmplt.l_gain = packet->setparam_dec_cmd.l_gain;
        cmplt.setparam_dec_cmplt.r_gain = packet->setparam_dec_cmd.l_gain;

        MEDIA_PLAYER_VDBG("SetPrm Lg %d Rg %d\n",
                          cmplt.setparam_dec_cmplt.l_gain,
                          cmplt.setparam_dec_cmplt.r_gain);

        err_t er = MsgLib::send<DecCmpltParam>((static_cast<FAR PlayerObj *>
                                                (p_requester))->get_selfId(),
                                               MsgPriNormal,
                                               MSG_AUD_PLY_CMD_DEC_SET_DONE,
                                               NULL,
                                               cmplt);
        F_ASSERT(er == ERR_OK);
      }
      break;

    default:
      MEDIA_PLAYERS_ERR((static_cast<FAR PlayerObj *>(p_requester))->get_playerId(),
                        AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
      return false;
  }
  return true;
}

/*--------------------------------------------------------------------------*/
static void pcm_send_done_callback(int32_t identifier, bool is_end)
{
  MsgQueId msgq_id =
    (identifier == OutputMixer0) ? s_msgq_id.player : s_sub_msgq_id.player;

  PlayerCommand cmd;

  cmd.req_next_param.type =
    (is_end != true) ? AsNextNormalRequest : AsNextStopResRequest;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_NEXT_REQ,
                                         NULL,
                                         cmd);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
PlayerObj::PlayerObj(AsPlayerMsgQueId_t msgq_id, AsPlayerPoolId_t pool_id, AsPlayerId player_id):
  m_msgq_id(msgq_id),
  m_pool_id(pool_id),
  m_state(AS_MODULE_ID_PLAYER_OBJ, "main", BootedState),
  m_sub_state(AS_MODULE_ID_PLAYER_OBJ, "sub", InvalidSubState),
  m_player_id(player_id),
  m_input_device_handler(NULL),
  m_codec_type(InvalidCodecType),
  m_src_work_buf(NULL),
  m_callback(NULL),
  m_pcm_path(AsPcmDataReply)
{
}

/*--------------------------------------------------------------------------*/
void PlayerObj::run(void)
{
  err_t        err_code;
  MsgQueBlock *que;
  MsgPacket   *msg;

  err_code = MsgLib::referMsgQueBlock(m_msgq_id.player, &que);
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
PlayerObj::MsgProc PlayerObj::MsgProcTbl[AUD_PLY_MSG_NUM][PlayerStateNum] =
{
  /* Message type: MSG_AUD_PLY_CMD_ACT. */

  {                                  /* Player status:        */
    &PlayerObj::activate,            /*   BootedState.        */
    &PlayerObj::illegalEvt,          /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::illegalEvt,          /*   PlayState.          */
    &PlayerObj::illegalEvt,          /*   StoppingState.      */
    &PlayerObj::illegalEvt,          /*   WaitEsEndState.     */
    &PlayerObj::illegalEvt,          /*   UnderflowState.     */
    &PlayerObj::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_INIT. */

  {                                  /* Player status:        */
    &PlayerObj::illegalEvt,          /*   BootedState.        */
    &PlayerObj::init,                /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::illegalEvt,          /*   PlayState.          */
    &PlayerObj::illegalEvt,          /*   StoppingState.      */
    &PlayerObj::illegalEvt,          /*   WaitEsEndState.     */
    &PlayerObj::illegalEvt,          /*   UnderflowState.     */
    &PlayerObj::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type:  MSG_AUD_PLY_CMD_PLAY. */

  {                                  /* Player status:        */
    &PlayerObj::illegalEvt,          /*   BootedState.        */
    &PlayerObj::playOnReady,         /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::illegalEvt,          /*   PlayState.          */
    &PlayerObj::illegalEvt,          /*   StoppingState.      */
    &PlayerObj::illegalEvt,          /*   WaitEsEndState.     */
    &PlayerObj::illegalEvt,          /*   UnderflowState.     */
    &PlayerObj::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_STOP. */

  {                                  /* Player status:        */
    &PlayerObj::illegalEvt,          /*   BootedState.        */
    &PlayerObj::illegalEvt,          /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::stopOnPlay,          /*   PlayState.          */
    &PlayerObj::illegalEvt,          /*   StoppingState.      */
    &PlayerObj::stopOnWaitEsEnd,     /*   WaitEsEndState.     */
    &PlayerObj::stopOnUnderflow,     /*   UnderflowState.     */
    &PlayerObj::stopOnWait           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_DEACT. */

  {                                  /* Player status:        */
    &PlayerObj::illegalEvt,          /*   BootedState.        */
    &PlayerObj::deactivate,          /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::illegalEvt,          /*   PlayState.          */
    &PlayerObj::illegalEvt,          /*   StoppingState.      */
    &PlayerObj::illegalEvt,          /*   WaitEsEndState.     */
    &PlayerObj::illegalEvt,          /*   UnderflowState.     */
    &PlayerObj::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_SETGAIN */
  {                                  /* Player status:        */
    &PlayerObj::illegalEvt,          /*   BootedState.        */
    &PlayerObj::setGain,             /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::setGain,             /*   PlayState.          */
    &PlayerObj::setGain,             /*   StoppingState.      */
    &PlayerObj::setGain,             /*   WaitEsEndState.     */
    &PlayerObj::setGain,             /*   UnderflowState.     */
    &PlayerObj::setGain,             /*   WaitStopState.      */
  }
};

/*--------------------------------------------------------------------------*/
PlayerObj::MsgProc PlayerObj::PlayerSubStateTbl[AUD_PLY_MSG_NUM][SubStateNum] =
{
  /* Message type: MSG_AUD_PLY_CMD_ACT. */

  {                                        /* Player sub status:          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlay.          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_INIT. */

  {                                        /* Player sub status:          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlay.          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayUnderflow. */
  },

  /* Message type:  MSG_AUD_PLY_CMD_PLAY. */

  {                                        /* Player sub status:          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlay.          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_STOP. */

  {                                        /* Player sub status:          */
    &PlayerObj::stopOnPrePlay,             /*   SubStatePrePlay.          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::stopOnPrePlayWaitEsEnd,    /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::stopOnPrePlayUnderflow,    /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_DEACT. */

  {                                        /* Player sub status:          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlay.          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_SETGAIN. */

  {                                        /* Player sub status:          */
    &PlayerObj::setGain,                   /*   SubStatePrePlay.          */
    &PlayerObj::setGain,                   /*   SubStatePrePlayStopping.  */
    &PlayerObj::setGain,                   /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::setGain,                   /*   SubStatePrePlayUnderflow. */
  }
};

/*--------------------------------------------------------------------------*/
PlayerObj::MsgProc PlayerObj::PlayerResultTbl[AUD_PLY_RST_MSG_NUM][PlayerStateNum] =
{
  /* Message type: MSG_AUD_PLY_CMD_NEXT_REQ. */

  {
    &PlayerObj::illegalSinkDone,     /*   BootedState.        */
    &PlayerObj::illegalSinkDone,     /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::nextReqOnPlay,       /*   PlayState.          */
    &PlayerObj::nextReqOnStopping,   /*   StoppingState.      */
    &PlayerObj::nextReqOnWaitEsEnd,  /*   WaitEsEndState.     */
    &PlayerObj::nextReqOnUnderflow,  /*   UnderflowState.     */
    &PlayerObj::illegalSinkDone      /*   WaitStopState.      */

  },

  /* Message type: MSG_AUD_PLY_CMD_DEC_DONE. */

  {
    &PlayerObj::illegalDecDone,      /*   BootedState.        */
    &PlayerObj::illegalDecDone,      /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::decDoneOnPlay,       /*   PlayState.          */
    &PlayerObj::decDoneOnWaitStop,   /*   StoppingState.      */
    &PlayerObj::decDoneOnWaitEsEnd,  /*   WaitEsEndState.     */
    &PlayerObj::decDoneOnWaitStop,   /*   UnderflowState.     */
    &PlayerObj::illegalDecDone       /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_DEC_SET_DONE. */

  {
    &PlayerObj::illegalDecDone,      /*   BootedState.        */
    &PlayerObj::decSetDone,          /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::decSetDone,          /*   PlayState.          */
    &PlayerObj::decSetDone,          /*   StoppingState.      */
    &PlayerObj::decSetDone,          /*   WaitEsEndState.     */
    &PlayerObj::decSetDone,          /*   UnderflowState.     */
    &PlayerObj::decSetDone           /*   WaitStopState.      */
  }
};

/*--------------------------------------------------------------------------*/
PlayerObj::MsgProc PlayerObj::PlayerResultSubTbl[AUD_PLY_RST_MSG_NUM][SubStateNum] =
{
  /* Message type: MSG_AUD_PLY_CMD_NEXT_REQ. */

  {                                        /* Player sub status:          */
    &PlayerObj::illegalSinkDone,           /*   SubStatePrePlay.          */
    &PlayerObj::illegalSinkDone,           /*   SubStatePrePlayStopping.  */
    &PlayerObj::illegalSinkDone,           /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::illegalSinkDone,           /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_DEC_DONE. */

  {                                        /* Player sub status:          */
    &PlayerObj::decDoneOnPrePlay,          /*   SubStatePrePlay.          */
    &PlayerObj::decDoneOnPrePlayStopping,  /*   SubStatePrePlayStopping.  */
    &PlayerObj::decDoneOnPrePlayWaitEsEnd, /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::decDoneOnPrePlayUnderflow, /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_DEC_SET_DONE. */

  {                                        /* Player sub status:          */
    &PlayerObj::decSetDone,                /*   SubStatePrePlay.          */
    &PlayerObj::decSetDone,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::decSetDone,                /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::decSetDone,                /*   SubStatePrePlayUnderflow. */
  }
};

/*--------------------------------------------------------------------------*/
void PlayerObj::parse(MsgPacket *msg)
{
  uint event = MSG_GET_SUBTYPE(msg->getType());

  if (MSG_IS_REQUEST(msg->getType()) != 0)
    {
      F_ASSERT(event < AUD_PLY_MSG_NUM);
      (this->*MsgProcTbl[event][m_state.get()])(msg);
    }
  else
    {
      F_ASSERT(event < AUD_PLY_RST_MSG_NUM);
      (this->*PlayerResultTbl[event][m_state.get()])(msg);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::reply(AsPlayerEvent evtype, MsgType msg_type, uint32_t result)
{
  if (m_callback != NULL)
    {
      m_callback(evtype, result, 0);
    }
  else if (m_msgq_id.mng != MSG_QUE_NULL)
    {
      AudioObjReply cmplt((uint32_t)msg_type,
                           AS_OBJ_REPLY_TYPE_REQ,
                           AS_MODULE_ID_PLAYER_OBJ,
                           result);
      err_t er = MsgLib::send<AudioObjReply>(m_msgq_id.mng,
                                             MsgPriNormal,
                                             MSG_TYPE_AUD_RES,
                                             m_msgq_id.player,
                                             cmplt);
      if (ERR_OK != er)
        {
          F_ASSERT(0);
        }
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::illegalEvt(MsgPacket *msg)
{
  uint msgtype = msg->getType();
  msg->moveParam<PlayerCommand>();

  /* Extract and abandon message data */

  uint32_t idx = msgtype - MSG_AUD_PLY_CMD_ACT;

  AsPlayerEvent table[] =
  {
    AsPlayerEventAct,
    AsPlayerEventInit,
    AsPlayerEventPlay,
    AsPlayerEventStop,
    AsPlayerEventDeact,
    AsPlayerEventSetGain
  };

  reply(table[idx], (MsgType)msgtype, AS_ECODE_STATE_VIOLATION);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::activate(MsgPacket *msg)
{
  AsActivatePlayer act = msg->moveParam<PlayerCommand>().act_param;
  PlayerInputDeviceHandler::PlayerInHandle in_device_handle;
  bool result;

  MEDIA_PLAYER_DBG("ACT: indev %d, outdev %d\n",
                   act.param.input_device,
                   act.param.output_device);

  /* Set event callback */

  m_callback = act.cb;

  if (!checkAndSetMemPool())
    {
      reply(AsPlayerEventAct,
            msg->getType(),
            AS_ECODE_CHECK_MEMORY_POOL_ERROR);
      return;
    }

  switch (act.param.input_device)
    {
      case AS_SETPLAYER_INPUTDEVICE_RAM:
        {
          m_input_device_handler = &m_in_ram_device_handler;
          in_device_handle.p_ram_device_handle =
            act.param.ram_handler;
        }
        break;

    default:
      reply(AsPlayerEventAct,
            msg->getType(),
            AS_ECODE_COMMAND_PARAM_INPUT_DEVICE);
      return;
  }

  result = m_input_device_handler->initialize(&in_device_handle);
  if (result)
    {
      reply(AsPlayerEventAct, msg->getType(), AS_ECODE_OK);
      m_state = ReadyState;
    }
  else
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      reply(AsPlayerEventAct,
            msg->getType(),
            AS_ECODE_COMMAND_PARAM_INPUT_HANDLER);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::deactivate(MsgPacket *msg)
{
  msg->moveParam<PlayerCommand>();

  MEDIA_PLAYER_DBG("DEACT:\n");

  if (AS_ECODE_OK != unloadCodec())
    {
      reply(AsPlayerEventDeact, msg->getType(), AS_ECODE_DSP_UNLOAD_ERROR);
      return;
    }

  reply(AsPlayerEventDeact, msg->getType(), AS_ECODE_OK);
  m_state = BootedState;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::init(MsgPacket *msg)
{
  AsInitPlayerParam param = msg->moveParam<PlayerCommand>().init_param;
  uint8_t result;
  uint32_t dsp_inf;

  MEDIA_PLAYER_DBG("INIT: ch num %d, bit len %d, codec %d(%s), fs %ld\n",
                   param.channel_number,
                   param.bit_length,
                   param.codec_type,
                   param.dsp_path,
                   param.sampling_rate);

  /* Compare existence of multi-core between this time and last time. */

  bool is_multi_core_prev = judgeMultiCore(
                            m_input_device_handler->getSamplingRate(),
                            m_input_device_handler->getBitLen());

  bool is_multi_core_cur = judgeMultiCore(param.sampling_rate,
                                         param.bit_length);

  result = m_input_device_handler->setParam(param);

  if (result == AS_ECODE_OK)
    {
      /* Update codec accordingt to audio data type. */

      /* NOTE : Codec loading process was Moved to here from play()
       *        for reducing memory usage. Exclude Gapless once.
       */

      AudioCodec next_codec = m_input_device_handler->getCodecType();

      if ((m_codec_type != next_codec) ||
          (is_multi_core_prev != is_multi_core_cur))
        {
          result = unloadCodec();

          if(result == AS_ECODE_OK)
            {
              result = loadCodec(next_codec, &param, &dsp_inf);
            }
        }
    }
  else
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
    }

  reply(AsPlayerEventInit, msg->getType(), result);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::playOnReady(MsgPacket *msg)
{
  AsPlayPlayerParam param = msg->moveParam<PlayerCommand>().play_param;

  uint32_t dsp_inf = 0;
  uint32_t rst     = AS_ECODE_OK;

  MEDIA_PLAYER_DBG("PLAY:\n");

  if ((rst = startPlay(&dsp_inf)) == AS_ECODE_OK)
    {
      /* Set PCM data path */

      m_pcm_path = static_cast<AsPcmDataPath>(param.pcm_path);

      if (m_pcm_path == AsPcmDataReply)
        {
          /* Set callback for send decoded pcm */

          m_pcm_dest.callback = param.pcm_dest.callback;
        }
      else
        {
          /* Set message destination and indentifier */

          m_pcm_dest.msg.id         = param.pcm_dest.msg.id;
          m_pcm_dest.msg.identifier = param.pcm_dest.msg.identifier;
        }

      m_sub_state = SubStatePrePlay;
      m_state = PrePlayParentState;

      reply(AsPlayerEventPlay, msg->getType(), AS_ECODE_OK);
    }
  else
    {
      reply(AsPlayerEventPlay, msg->getType(), rst);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnPlay(MsgPacket *msg)
{
  AsStopPlayerParam param = msg->moveParam<PlayerCommand>().stop_param;

  MEDIA_PLAYER_DBG("STOP:\n");

  /* Note: Since the completion of current command will be notified
   * by other event trigger, queue external command.
   */

  if (param.stop_mode == AS_STOPPLAYER_FORCIBLY)
    {
      stopPlay();
      m_state = UnderflowState;
      return;
    }

  if (!m_external_cmd_que.push(AsPlayerEventStop))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      reply(AsPlayerEventStop,
            msg->getType(),
            AS_ECODE_QUEUE_OPERATION_ERROR);
      return;
    }

  if (param.stop_mode == AS_STOPPLAYER_NORMAL)
    {
      stopPlay();
      m_state = StoppingState;
    }
  else
    {
      m_state = WaitEsEndState;
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnWait(MsgPacket *msg)
{
  msg->moveParam<PlayerCommand>();

  MEDIA_PLAYER_DBG("STOP:\n");

  reply(AsPlayerEventStop, msg->getType(), AS_ECODE_OK);

  freeSrcWorkBuf();

  m_state = ReadyState;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnWaitEsEnd(MsgPacket *msg)
{
  stopOnPlay(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnUnderflow(MsgPacket *msg)
{
  AsStopPlayerParam param = msg->moveParam<PlayerCommand>().stop_param;;

  MEDIA_PLAYER_DBG("STOP:\n");

  if (param.stop_mode == AS_STOPPLAYER_FORCIBLY)
    {
      return;
    }

  if (!m_external_cmd_que.push(AsPlayerEventStop))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      reply(AsPlayerEventStop,
            msg->getType(),
            AS_ECODE_QUEUE_OPERATION_ERROR);
      return;
    }

  m_state = StoppingState;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnPrePlay(MsgPacket *msg)
{
  AsStopPlayerParam param = msg->moveParam<PlayerCommand>().stop_param;

  MEDIA_PLAYER_DBG("STOP:\n");

  /* Note: Since the completion of current command will be notified
   * by other event trigger, queue external command.
   */

  if (!m_external_cmd_que.push(AsPlayerEventStop))
    {
      reply(AsPlayerEventStop,
            msg->getType(),
            AS_ECODE_QUEUE_OPERATION_ERROR);
      return;
    }

  if (param.stop_mode == AS_STOPPLAYER_NORMAL)
    {
      stopPlay();
      m_sub_state = SubStatePrePlayStopping;
    }
  else
    {
      m_sub_state = SubStatePrePlayWaitEsEnd;
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnPrePlayWaitEsEnd(MsgPacket *msg)
{
  stopOnPrePlay(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnPrePlayUnderflow(MsgPacket *msg)
{
  msg->moveParam<PlayerCommand>();

  MEDIA_PLAYER_DBG("STOP:\n");

  /* Note: Since the completion of current command will be notified
   * by other event trigger, queue external command.
   */

  if (!m_external_cmd_que.push(AsPlayerEventStop))
    {
      reply(AsPlayerEventStop,
            msg->getType(),
            AS_ECODE_QUEUE_OPERATION_ERROR);
      return;
    }

  m_sub_state = SubStatePrePlayStopping;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::illegalSinkDone(MsgPacket *msg)
{
  msg->moveParam<PlayerCommand>();
  MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::nextReqOnPlay(MsgPacket *msg)
{
  AsRequestNextParam param = msg->moveParam<PlayerCommand>().req_next_param;
  if (AsNextNormalRequest != param.type)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }

  if ((MemMgrLite::Manager::getPoolNumAvailSegs(m_pool_id.es) > 0) &&
        (MemMgrLite::Manager::getPoolNumAvailSegs(m_pool_id.pcm) > 1))
    {
      /* Do next decoding process. */

      uint32_t es_size = m_max_es_buff_size;
      void* es_addr = getEs(&es_size);

      if (es_addr != NULL)
        {
          decode(es_addr, es_size);
        }
      else
        {
          stopPlay();
          if (m_state == PlayState)
            {
              m_state = UnderflowState;
              MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW);
            }
          else
            {
              m_state = StoppingState;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::nextReqOnStopping(MsgPacket *msg)
{
  AsRequestNextParam param = msg->moveParam<PlayerCommand>().req_next_param;

  if (AsNextStopResRequest == param.type)
    {
      finalize();

      if (m_external_cmd_que.empty())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
          return;
        }

      AsPlayerEvent ext_cmd_code = m_external_cmd_que.top();
      if (!m_external_cmd_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }
      reply(ext_cmd_code, MSG_AUD_PLY_CMD_STOP, AS_ECODE_OK);

      freeSrcWorkBuf();

      m_state = ReadyState;
    }
  else if (AsNextNormalRequest == param.type)
    {
      /* Nothing to do */
    }
  else
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::nextReqOnWaitEsEnd(MsgPacket *msg)
{
  nextReqOnPlay(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::nextReqOnUnderflow(MsgPacket *msg)
{
  AsRequestNextParam param = msg->moveParam<PlayerCommand>().req_next_param;

  if (AsNextStopResRequest == param.type)
    {
      finalize();

      m_state = WaitStopState;
    }
  else if (AsNextNormalRequest == param.type)
    {
      /* Nothing to do */
    }
  else
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::illegalDecDone(MsgPacket *msg)
{
  msg->moveParam<DecCmpltParam>();

  MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnPlay(MsgPacket *msg)
{
  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();

  AS_decode_recv_done(m_p_dec_instance);
  freeEsBuf();

  AsPcmDataParam data;

  data.mh        = m_pcm_buf_mh_que.top();
  data.size      = cmplt.exec_dec_cmplt.output_buffer.size;
  data.is_end    = false;
  data.is_valid  = ((data.size == 0) ?
                    false : cmplt.exec_dec_cmplt.is_valid_frame);

  sendPcmToOwner(data);

  freePcmBuf();

  if ((MemMgrLite::Manager::getPoolNumAvailSegs(m_pool_id.es) > 0) &&
        (MemMgrLite::Manager::getPoolNumAvailSegs(m_pool_id.pcm) > 1))
    {
      /* Do next decoding process. */

      uint32_t es_size = m_max_es_buff_size;
      void    *es_addr = getEs(&es_size);

    if (es_addr != NULL)
      {
        decode(es_addr, es_size);
      }
    else
      {
        /* There is no stream data. */

        stopPlay();
        if (m_state == PlayState)
          {
            m_state = UnderflowState;
            MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW);
          }
        else
          {
            m_state = StoppingState;
          }
      }
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnWaitEsEnd(MsgPacket *msg)
{
  decDoneOnPlay(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnWaitStop(MsgPacket *msg)
{
  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();

  AS_decode_recv_done(m_p_dec_instance);

  AsPcmDataParam data;

  data.mh     = m_pcm_buf_mh_que.top();

  if (Apu::ExecEvent == cmplt.event_type)
    {
      freeEsBuf();

      data.size      = cmplt.exec_dec_cmplt.output_buffer.size;
      data.is_valid  = ((data.size == 0) ?
                       false : cmplt.exec_dec_cmplt.is_valid_frame);
      data.is_end = false;
    }
  else if (Apu::FlushEvent == cmplt.event_type)
    {
      data.size      = cmplt.stop_dec_cmplt.output_buffer.size;
      data.is_valid  = cmplt.stop_dec_cmplt.is_valid_frame;
      data.is_end = true;
    }

  sendPcmToOwner(data);

  freePcmBuf();
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnPrePlay(MsgPacket *msg)
{
  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();

  AS_decode_recv_done(m_p_dec_instance);
  freeEsBuf();

  AsPcmDataParam data;

  data.mh        = m_pcm_buf_mh_que.top();
  data.size      = cmplt.exec_dec_cmplt.output_buffer.size;
  data.is_end = false;
  data.is_valid  = ((data.size == 0) ?
                   false : cmplt.exec_dec_cmplt.is_valid_frame);

  if (!m_decoded_pcm_mh_que.push(data))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return;
    }

  freePcmBuf();

  if (m_decoded_pcm_mh_que.size() <= MAX_EXEC_COUNT)
    {
      uint32_t es_size = m_max_es_buff_size;
      void    *es_addr = getEs(&es_size);

    if (es_addr != NULL)
      {
        decode(es_addr, es_size);
      }
    else
      {
        stopPlay();
        if (m_sub_state == SubStatePrePlay)
          {
            m_sub_state = SubStatePrePlayUnderflow;
            MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW);
          }
        else
          {
            m_sub_state = SubStatePrePlayStopping;
          }
      }
    }
  else
    {
      while(!m_decoded_pcm_mh_que.empty())
        {
          sendPcmToOwner(const_cast<AsPcmDataParam&>(m_decoded_pcm_mh_que.top()));
          if (!m_decoded_pcm_mh_que.pop())
            {
              MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
              break;
            }
        }

      uint32_t es_size = m_max_es_buff_size;
      void    *es_addr = getEs(&es_size);

      if (es_addr != NULL)
        {
          decode(es_addr, es_size);

          if (m_sub_state == SubStatePrePlay)
            {
              m_state = PlayState;
            }
          else
            {
              m_state = WaitEsEndState;
            }

            m_sub_state = InvalidSubState;
        }
      else
        {
          stopPlay();
          if (m_sub_state == SubStatePrePlay)
            {
              m_state = UnderflowState;
              MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW);
            }
          else
            {
              m_state = StoppingState;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnPrePlayStopping(MsgPacket *msg)
{
  /* Free all resource and send stop completion. */

  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();
  AS_decode_recv_done(m_p_dec_instance);
  freePcmBuf();

  if (Apu::ExecEvent == cmplt.event_type)
    {
      freeEsBuf();
    }
  else if (Apu::FlushEvent == cmplt.event_type)
    {
      /* Free all decoded pcm MHandle. */

      while (!m_decoded_pcm_mh_que.empty())
        {
          if (!m_decoded_pcm_mh_que.pop())
            {
              MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
              break;
            }
        }

      finalize();

      if (m_external_cmd_que.empty())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
          return;
        }

      AsPlayerEvent ext_cmd_code = m_external_cmd_que.top();
      if (!m_external_cmd_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }
      reply(ext_cmd_code, msg->getType(), AS_ECODE_OK);

      freeSrcWorkBuf();

      m_sub_state = InvalidSubState;
      m_state = ReadyState;
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnPrePlayWaitEsEnd(MsgPacket *msg)
{
  decDoneOnPrePlay(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnPrePlayUnderflow(MsgPacket *msg)
{
  /* Free all resource. */

  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();
  AS_decode_recv_done(m_p_dec_instance);
  freePcmBuf();

  if (Apu::ExecEvent == cmplt.event_type)
    {
      freeEsBuf();
    }
  else if (Apu::FlushEvent == cmplt.event_type)
    {
      /* Free all decoded pcm MHandle. */

      while (!m_decoded_pcm_mh_que.empty())
        {
          if (!m_decoded_pcm_mh_que.pop())
            {
              MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
              break;
            }
        }
      finalize();

      m_sub_state = InvalidSubState;
      m_state = WaitStopState;
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decSetDone(MsgPacket *msg)
{
  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();
  AS_decode_recv_done(m_p_dec_instance);

   if (Apu::SetParamEvent != cmplt.event_type)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }

  if (m_external_cmd_que.empty())
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      return;
    }

  AsPlayerEvent ext_cmd_code = m_external_cmd_que.top();
  if (!m_external_cmd_que.pop())
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
    }

  /* Send result. */

  reply(ext_cmd_code, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::setGain(MsgPacket *msg)
{
  AsSetGainParam param = msg->moveParam<PlayerCommand>().set_gain_param;

  if (!m_external_cmd_que.push(AsPlayerEventSetGain))
    {
      reply(AsPlayerEventSetGain,
            msg->getType(),
            AS_ECODE_QUEUE_OPERATION_ERROR);
      return;
    }

  /* Set paramter to Decoder */

  SetDecCompParam setparam;

  setparam.l_gain = param.l_gain;
  setparam.r_gain = param.r_gain;

  if (AS_decode_setparam(&setparam, m_p_dec_instance) == false)
    {
      m_external_cmd_que.pop();

      reply(AsPlayerEventSetGain, msg->getType(), AS_ECODE_DSP_SET_ERROR);
      return;
    }

  /* Response is sent after decoder_component done */
}

/*--------------------------------------------------------------------------*/
void PlayerObj::parseSubState(MsgPacket *msg)
{
  uint32_t event = MSG_GET_SUBTYPE(msg->getType());

  if (PlayerObj::InvalidSubState == m_sub_state.get())
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_INTERNAL_STATE_ERROR);
      return;
    }

  if (MSG_IS_REQUEST(msg->getType()) != 0)
    {
      (this->*PlayerSubStateTbl[event][m_sub_state.get()])(msg);
    }
  else
    {
      (this->*PlayerResultSubTbl[event][m_sub_state.get()])(msg);
    }
}

/*--------------------------------------------------------------------------*/
uint32_t PlayerObj::loadCodec(AudioCodec codec,
                              AsInitPlayerParam *param,
                              uint32_t* dsp_inf)
{
  uint32_t rst;

  switch  (codec)
    {
      case AudCodecMP3:
      case AudCodecLPCM:
      case AudCodecAAC:
      case AudCodecOPUS:
        break;
      default:
        return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }

  if (m_codec_type != InvalidCodecType)
    {
      return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }

  ActDecCompParam act_param;
  act_param.codec          = codec;
  act_param.path           = (param->dsp_path != NULL) ? param->dsp_path :
                              (char *)CONFIG_AUDIOUTILS_DSP_MOUNTPT;
  act_param.p_instance     = &m_p_dec_instance;
  act_param.apu_pool_id    = m_pool_id.dsp;
  act_param.apu_mid        = m_msgq_id.dsp;
  act_param.dsp_inf        = dsp_inf;
  act_param.dsp_multi_core = judgeMultiCore(param->sampling_rate,
                                            param->bit_length);
  rst = AS_decode_activate(&act_param);
  if (rst != AS_ECODE_OK)
    {
      return rst;
    }

  m_codec_type = codec;

  return rst;
}

/*--------------------------------------------------------------------------*/
uint32_t PlayerObj::unloadCodec(void)
{
  /* Only unload when codec is loaded. */

  if (m_codec_type != InvalidCodecType)
    {
      if (!AS_decode_deactivate(m_p_dec_instance))
        {
          return AS_ECODE_DSP_UNLOAD_ERROR;
        }
      m_codec_type = InvalidCodecType;
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t PlayerObj::startPlay(uint32_t* dsp_inf)
{
  uint32_t   rst = AS_ECODE_OK;
  InitDecCompParam init_dec_comp_param;

  rst = m_input_device_handler->start();
  if (rst != AS_ECODE_OK)
    {
      if (rst == AS_ECODE_SIMPLE_FIFO_UNDERFLOW)
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW);
        }
      else
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_STREAM_PARSER_ERROR);
        }

      return rst;
    }

  init_dec_comp_param.codec_type          = m_codec_type;
  init_dec_comp_param.input_sampling_rate =
      m_input_device_handler->getSamplingRate();
  init_dec_comp_param.channel_num         =
      ((m_input_device_handler->getChannelNum() == AS_CHANNEL_STEREO) ?
      (TwoChannels) : (MonoChannels));
  init_dec_comp_param.frame_sample_num    =
      m_input_device_handler->getSampleNumPerFrame();
  init_dec_comp_param.callback            = &decoder_comp_done_callback;
  init_dec_comp_param.p_requester         = static_cast<void*>(this);
  init_dec_comp_param.bit_width =
    ((m_input_device_handler->getBitLen() == AS_BITLENGTH_16) ?
     AudPcmFormatInt16 : AudPcmFormatInt24);
  init_dec_comp_param.work_buffer.p_buffer = reinterpret_cast<unsigned long*>
      (allocSrcWorkBuf(m_max_src_work_buff_size));
  init_dec_comp_param.work_buffer.size     = m_max_src_work_buff_size;
  init_dec_comp_param.dsp_multi_core       =
    judgeMultiCore(m_input_device_handler->getSamplingRate(),
                   m_input_device_handler->getBitLen());
  rst = AS_decode_init(&init_dec_comp_param, m_p_dec_instance, dsp_inf);
  if (rst != AS_ECODE_OK)
    {
      freeSrcWorkBuf();
      return rst;
    }

  if (!AS_decode_recv_done(m_p_dec_instance))
    {
      freeSrcWorkBuf();
      return AS_ECODE_QUEUE_OPERATION_ERROR;
    }

  /* Get ES data. */

  uint32_t es_size = m_max_es_buff_size;
  void    *es_addr = getEs(&es_size);

  if (es_addr == NULL)
    {
      freeSrcWorkBuf();
      return  AS_ECODE_SIMPLE_FIFO_UNDERFLOW;
    }

  decode(es_addr, es_size);

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopPlay(void)
{
  StopDecCompParam param;
  param.output_buffer.size = m_max_pcm_buff_size;
  param.output_buffer.p_buffer = reinterpret_cast<unsigned long*>
    (allocPcmBuf(m_max_pcm_buff_size));

  if (param.output_buffer.p_buffer != NULL)
    {
      if (AS_decode_stop(&param, m_p_dec_instance) == false)
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
        }
    }

  m_input_device_handler->stop();
}

/*--------------------------------------------------------------------------*/
void PlayerObj::sendPcmToOwner(AsPcmDataParam& data)
{
  data.bit_length = m_input_device_handler->getBitLen();
  if (m_pcm_path == AsPcmDataReply)
    {
      /* Call callback function for PCM data notify */

      m_pcm_dest.callback(data);
    }
  else
    {
      /* Send message for PCM data notify */

      data.identifier = m_pcm_dest.msg.identifier;
      data.callback   = pcm_send_done_callback;

      err_t er = MsgLib::send<AsPcmDataParam>(m_pcm_dest.msg.id,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_DATA,
                                              s_msgq_id.player,
                                              data);
      F_ASSERT(er == ERR_OK);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decode(void* p_es, uint32_t es_size)
{
  void *p_pcm = allocPcmBuf(m_max_pcm_buff_size);
  ExecDecCompParam param;

  if (p_pcm == NULL)
    {
      return;
    }

  param.input_buffer.p_buffer  = reinterpret_cast<unsigned long *>(p_es);
  param.input_buffer.size      = es_size;
  param.output_buffer.p_buffer = reinterpret_cast<unsigned long *>(p_pcm);
  param.output_buffer.size     = m_max_pcm_buff_size;
  param.num_of_au              = NUM_OF_AU;

  if (AS_decode_exec(&param, m_p_dec_instance) == false)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }
}

/*--------------------------------------------------------------------------*/
void* PlayerObj::allocPcmBuf(uint32_t size)
{
  MemMgrLite::MemHandle mh;
  if (mh.allocSeg(m_pool_id.pcm, size) != ERR_OK)
    {
      MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return NULL;
    }
  if (!m_pcm_buf_mh_que.push(mh))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return NULL;
    }

  return mh.getPa();
}

/*--------------------------------------------------------------------------*/
void* PlayerObj::getEs(uint32_t* size)
{
  MemMgrLite::MemHandle mh;

  if (mh.allocSeg(m_pool_id.es, *size) != ERR_OK)
    {
      MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return NULL;
    }

  if (m_input_device_handler->getEs(mh.getVa(), size))
    {
      if (!m_es_buf_mh_que.push(mh))
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return NULL;
        }
    return mh.getPa();
  }

  return NULL;
}

/*--------------------------------------------------------------------------*/
void* PlayerObj::allocSrcWorkBuf(uint32_t size)
{
  void *work_buf_addr;

  if (m_pool_id.src_work.pool != NullPoolId.pool)
    {
      MemMgrLite::MemHandle mh;
      if (mh.allocSeg(m_pool_id.src_work, size) != ERR_OK)
        {
          MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
          return NULL;
        }
      if (!m_src_work_buf_mh_que.push(mh))
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return NULL;
        }
      work_buf_addr = mh.getPa();
    }
  else
    {
      /* Allocate work memory area for SRC. */

      MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_ALLOC_HEAP_MEMORY);
      m_src_work_buf = kmm_malloc(SRC_WORK_BUF_SIZE);
      work_buf_addr = m_src_work_buf;
    }

  return work_buf_addr;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::finalize()
{
  /* Note:
   *   This queues should be EMPTY. If not, there exist any bug.
   *   (error debug log will be showed as following)
   *   If it happens, clear the queues to prevent system crash.
   */

  while (!m_decoded_pcm_mh_que.empty())
    {
      MEDIA_PLAYER_INF(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      if (!m_decoded_pcm_mh_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
          break;
        }
    }

  while (!m_es_buf_mh_que.empty())
    {
      MEDIA_PLAYER_INF(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      if (!m_es_buf_mh_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
          break;
        }
    }

  while (!m_pcm_buf_mh_que.empty())
    {
      MEDIA_PLAYER_INF(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      if (!m_pcm_buf_mh_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
          break;
        }
    }
}

/*--------------------------------------------------------------------------*/
bool PlayerObj::checkAndSetMemPool()
{
  if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.es))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  m_max_es_buff_size = (MemMgrLite::Manager::getPoolSize(m_pool_id.es)) /
    (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.es));

  if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.pcm))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  m_max_pcm_buff_size = (MemMgrLite::Manager::getPoolSize(m_pool_id.pcm)) /
    (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.pcm));

  if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.dsp))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  if ((int)(sizeof(Apu::Wien2ApuCmd)) >
      (MemMgrLite::Manager::getPoolSize(m_pool_id.dsp))/
      (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.dsp)))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }

  if (m_pool_id.src_work.pool != NullPoolId.pool)
    {
      if (!MemMgrLite::Manager::isPoolAvailable(m_pool_id.src_work))
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
          return false;
        }
      m_max_src_work_buff_size = (MemMgrLite::Manager::getPoolSize(m_pool_id.src_work)) /
        (MemMgrLite::Manager::getPoolNumSegs(m_pool_id.src_work));
    }
  return true;
}

/*--------------------------------------------------------------------------*/
bool PlayerObj::judgeMultiCore(uint32_t sampling_rate, uint8_t bit_length)
{
  /* If the sampling rate is Hi-Res and bit_length is 24 bits,
   * load the SRC slave DSP.
   */

  if ((sampling_rate > AS_SAMPLINGRATE_48000) &&
      (sampling_rate != AS_SAMPLINGRATE_192000) &&
      (bit_length > AS_BITLENGTH_16))
    {
      /* Multi core. */

      return true;
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*--------------------------------------------------------------------------*/
FAR void AS_PlayerObjEntry(FAR void *arg)
{
  PlayerObj::create(&s_play_obj,
                    s_msgq_id,
                    s_pool_id,
                    s_player_id);
}

/*--------------------------------------------------------------------------*/
FAR void AS_SubPlayerObjEntry(FAR void *arg)
{
  PlayerObj::create(&s_sub_play_obj,
                    s_sub_msgq_id,
                    s_sub_pool_id,
                    s_sub_player_id);
}

/*--------------------------------------------------------------------------*/
static bool CreatePlayerMulti(AsPlayerId id, AsPlayerMsgQueId_t msgq_id, AsPlayerPoolId_t pool_id, AudioAttentionCb attcb)
{
  /* Register attention callback */

  MEDIA_PLAYER_REG_ATTCB(attcb);

  /* Parameter check */

  /* Create */

  if (id == AS_PLAYER_ID_0)
    {
      s_msgq_id = msgq_id;
      s_pool_id = pool_id;
      s_player_id = id;

      /* Reset Message queue. */

      FAR MsgQueBlock *que;
      err_t err_code = MsgLib::referMsgQueBlock(s_msgq_id.player, &que);
      F_ASSERT(err_code == ERR_OK);
      que->reset();

      /* Init pthread attributes object. */

      pthread_attr_t attr;
      pthread_attr_init(&attr);

      /* Set pthread scheduling parameter. */

      struct sched_param sch_param;

      sch_param.sched_priority = 150;
      attr.stacksize           = 1024 * 3;

      pthread_attr_setschedparam(&attr, &sch_param);

      /* Create thread. */

      int ret = pthread_create(&s_ply_pid,
                               &attr,
                               (pthread_startroutine_t)AS_PlayerObjEntry,
                               (pthread_addr_t)NULL);
      if (ret < 0)
        {
          MEDIA_PLAYERS_ERR(id, AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
          return false;
        }

      pthread_setname_np(s_ply_pid, "media_player0");

    }
  else
    {
      s_sub_msgq_id = msgq_id;
      s_sub_pool_id = pool_id;
      s_sub_player_id = id;

      /* Reset Message queue. */

      FAR MsgQueBlock *que;
      err_t err_code = MsgLib::referMsgQueBlock(s_sub_msgq_id.player, &que);
      F_ASSERT(err_code == ERR_OK);
      que->reset();

      /* Init pthread attributes object. */

      pthread_attr_t attr;
      pthread_attr_init(&attr);

      /* Set pthread scheduling parameter. */

      struct sched_param sch_param;

      sch_param.sched_priority = 150;
      attr.stacksize           = 1024 * 3;

      pthread_attr_setschedparam(&attr, &sch_param);

      /* Create thread. */

      int ret = pthread_create(&s_sub_ply_pid,
                               &attr,
                               (pthread_startroutine_t)AS_SubPlayerObjEntry,
                               (pthread_addr_t)NULL);
      if (ret < 0)
        {
          MEDIA_PLAYERS_ERR(id, AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
          return false;
        }

      pthread_setname_np(s_sub_ply_pid, "media_player1");

    }

  return true;
}

/*
 * The following tree functions are Old functions for compatibility.
 */

static bool CreatePlayerMulti(AsPlayerId id, AsPlayerMsgQueId_t msgq_id,AsPlayerPoolId_old_t pool_id, AudioAttentionCb attcb)
{
  AsPlayerPoolId_t tmp;
  tmp.es.sec = tmp.pcm.sec = tmp.dsp.sec = tmp.src_work.sec = 0;
  tmp.es.pool = pool_id.es;
  tmp.pcm.pool = pool_id.pcm;
  tmp.dsp.pool = pool_id.dsp;
  tmp.src_work.pool = pool_id.src_work;

  return CreatePlayerMulti(id, msgq_id, tmp, attcb);

}

/*--------------------------------------------------------------------------*/
bool AS_CreatePlayerMulti(AsPlayerId id, FAR AsCreatePlayerParam_t *param, AudioAttentionCb attcb)
{
  return CreatePlayerMulti(id, param->msgq_id, param->pool_id, attcb);
}

/*--------------------------------------------------------------------------*/
bool AS_CreatePlayerMulti(AsPlayerId id, FAR AsCreatePlayerParam_t *param)
{
  return AS_CreatePlayerMulti(id, param, NULL);
}

/*
 * The following two functions are New functions for multi-section memory layout.
 */

bool AS_CreatePlayerMulti(AsPlayerId id, FAR AsCreatePlayerParams_t *param, AudioAttentionCb attcb)
{
  return CreatePlayerMulti(id, param->msgq_id, param->pool_id, attcb);
}

/*--------------------------------------------------------------------------*/
bool AS_CreatePlayerMulti(AsPlayerId id, FAR AsCreatePlayerParams_t *param)
{
  return AS_CreatePlayerMulti(id, param, NULL);
}

/*--------------------------------------------------------------------------*/
static bool CreatePlayer(AsPlayerId id, AsPlayerMsgQueId_t msgq_id, AsPlayerPoolId_old_t pool_id)
{
  /* Do not use memory pool for work area of src.
   * Set invalid value in memory pool.
   */

  AsPlayerPoolId_t tmp;
  tmp.es.sec = tmp.pcm.sec = tmp.dsp.sec = 0;
  tmp.es.pool = pool_id.es;
  tmp.pcm.pool = pool_id.pcm;
  tmp.dsp.pool = pool_id.dsp;
  tmp.src_work = NullPoolId;

  return CreatePlayerMulti(id, msgq_id, tmp, NULL);

}

/*--------------------------------------------------------------------------*/
bool AS_CreatePlayer(AsPlayerId id, FAR AsCreatePlayerParam_t *param)
{
  if (param == NULL)
    {
      MEDIA_PLAYERS_ERR(id, AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  return CreatePlayer(id, param->msgq_id, param->pool_id);

}

/*--------------------------------------------------------------------------*/
bool AS_ActivatePlayer(AsPlayerId id, FAR AsActivatePlayer *actparam)
{
  /* Parameter check */

  if (actparam == NULL)
    {
      return false;
    }

  /* Activate */

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_msgq_id.player : s_sub_msgq_id.player;

  PlayerCommand cmd;

  cmd.player_id = id;
  cmd.act_param = *actparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_ACT,
                                         s_msgq_id.mng,
                                         cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_InitPlayer(AsPlayerId id, FAR AsInitPlayerParam *initparam)
{
  /* Parameter check */

  if (initparam == NULL)
    {
      return false;
    }

  /* Init */

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_msgq_id.player : s_sub_msgq_id.player;

  PlayerCommand cmd;

  cmd.player_id  = id;
  cmd.init_param = *initparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_INIT,
                                         s_msgq_id.mng,
                                         cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_PlayPlayer(AsPlayerId id, FAR AsPlayPlayerParam *playparam)
{
  /* Parameter check */

  if (playparam == NULL)
    {
      return false;
    }

  /* Play */

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_msgq_id.player : s_sub_msgq_id.player;

  PlayerCommand cmd;

  cmd.player_id  = id;
  cmd.play_param = *playparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_PLAY,
                                         s_msgq_id.mng,
                                         cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StopPlayer(AsPlayerId id, FAR AsStopPlayerParam *stopparam)
{
  /* Parameter check */

  if (stopparam == NULL)
    {
      return false;
    }

  /* Stop */

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_msgq_id.player : s_sub_msgq_id.player;

  PlayerCommand cmd;

  cmd.player_id  = id;
  cmd.stop_param = *stopparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_STOP,
                                         s_msgq_id.mng,
                                         cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_SetPlayerGain(AsPlayerId id, FAR AsSetGainParam *gainparam)
{
  /* Parameter check */

  if (gainparam == NULL)
    {
      return false;
    }

  /* Set gain */

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_msgq_id.player : s_sub_msgq_id.player;

  PlayerCommand cmd;

  cmd.player_id      = id;
  cmd.set_gain_param = *gainparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_SETGAIN,
                                         s_msgq_id.mng,
                                         cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_RequestNextPlayerProcess(AsPlayerId id, FAR AsRequestNextParam *nextparam)
{
  /* Parameter check */

  if (nextparam == NULL)
    {
      return false;
    }

  /* Next request */

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_msgq_id.player : s_sub_msgq_id.player;

  PlayerCommand cmd;

  cmd.player_id      = id;
  cmd.req_next_param = *nextparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_NEXT_REQ,
                                         s_msgq_id.mng,
                                         cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivatePlayer(AsPlayerId id, FAR AsDeactivatePlayer *deactparam)
{
  /* Parameter check */

  if (deactparam == NULL)
    {
      return false;
    }

  /* Deactivate */

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_msgq_id.player : s_sub_msgq_id.player;

  PlayerCommand cmd;

  cmd.player_id   = id;
  cmd.deact_param = *deactparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_DEACT,
                                         s_msgq_id.mng,
                                         cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeletePlayer(AsPlayerId id)
{
  if (id == AS_PLAYER_ID_0)
    {
      if (s_ply_pid == INVALID_PROCESS_ID)
        {
          MEDIA_PLAYERS_ERR(id, AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
          return false;
        }

      pthread_cancel(s_ply_pid);
      pthread_join(s_ply_pid, NULL);

      s_ply_pid = INVALID_PROCESS_ID;

      if (s_play_obj != NULL)
        {
          delete ((PlayerObj *)s_play_obj);
          s_play_obj = NULL;
        }
    }
  else
    {
      if (s_sub_ply_pid == INVALID_PROCESS_ID)
        {
          MEDIA_PLAYERS_ERR(id, AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
          return false;
        }

      pthread_cancel(s_sub_ply_pid);
      pthread_join(s_sub_ply_pid, NULL);

      s_sub_ply_pid = INVALID_PROCESS_ID;

      if (s_sub_play_obj != NULL)
        {
          delete ((PlayerObj *)s_sub_play_obj);
          s_sub_play_obj = NULL;
        }
    }

  /* Unregister attention callback */

  if (s_play_obj == NULL && s_sub_play_obj == NULL)
    {
      MEDIA_PLAYER_UNREG_ATTCB();
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_checkAvailabilityMediaPlayer(AsPlayerId id)
{
  if (id == AS_PLAYER_ID_0)
    {
      return (s_play_obj != NULL);
    }
  else
    {
      return (s_sub_play_obj != NULL);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::create(FAR void **obj,
                       AsPlayerMsgQueId_t msgq_id,
                       AsPlayerPoolId_t pool_id,
                       AsPlayerId player_id)
{
  FAR PlayerObj *player_obj = new PlayerObj(msgq_id, pool_id, player_id);

  if (player_obj != NULL)
    {
      *obj = reinterpret_cast<void*>(player_obj);
      player_obj->run();
    }
  else
    {
      MEDIA_PLAYERS_ERR(player_id, AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }
}
