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
#include <string.h>
#include <stdlib.h>
#include <arch/chip/cxd56_audio.h>
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

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pid_t    s_ply_pid = -1;
static MsgQueId s_self_dtq;
static MsgQueId s_dsp_dtq;
static PoolId   s_es_pool_id;
static PoolId   s_pcm_pool_id;
static PoolId   s_apu_pool_id;
static pid_t    s_sub_ply_pid;
static MsgQueId s_sub_self_dtq;
static MsgQueId s_sub_dsp_dtq;
static PoolId   s_sub_es_pool_id;
static PoolId   s_sub_pcm_pool_id;
static PoolId   s_sub_apu_pool_id;

static void *s_play_obj = NULL;
static void *s_sub_play_obj = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool decoder_comp_done_callback(void *p_response, FAR void *p_requester)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;
  DecCmpltParam cmplt;

  if (DSP_COM_DATA_TYPE_STRUCT_ADDRESS != p_param->type)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
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

          MEDIA_PLAYER_VDBG("Dec s %d v %d\n",
                            cmplt.exec_dec_cmplt.output_buffer.size,
                            cmplt.exec_dec_cmplt.is_valid_frame);

          err_t er = MsgLib::send<DecCmpltParam>((static_cast<FAR PlayerObj *>
                                                  (p_requester))->get_selfId(),
                                                 MsgPriNormal,
                                                 MSG_AUD_PLY_CMD_DEC_DONE,
                                                 (static_cast<FAR PlayerObj *>
                                                  (p_requester))->get_selfId(),
                                                 cmplt);
          F_ASSERT(er == ERR_OK);
        }
        break;

    case Apu::FlushEvent:
      {
        cmplt.stop_dec_cmplt.output_buffer  = packet->flush_dec_cmd.output_buffer;
        cmplt.stop_dec_cmplt.is_valid_frame =
          ((packet->result.exec_result == Apu::ApuExecOK) ? true : false);

        MEDIA_PLAYER_VDBG("FlsDec s %d v %d\n",
                          cmplt.stop_dec_cmplt.output_buffer.size,
                          cmplt.stop_dec_cmplt.is_valid_frame);

        err_t er = MsgLib::send<DecCmpltParam>((static_cast<FAR PlayerObj *>
                                                (p_requester))->get_selfId(),
                                               MsgPriNormal,
                                               MSG_AUD_PLY_CMD_DEC_DONE,
                                               (static_cast<FAR PlayerObj *>
                                                (p_requester))->get_selfId(),
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
                                               (static_cast<FAR PlayerObj *>
                                                (p_requester))->get_selfId(),
                                               cmplt);
        F_ASSERT(er == ERR_OK);
      }
      break;

    default:
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
      return false;
  }
  return true;
}

/*--------------------------------------------------------------------------*/
static void pcm_send_done_callback(int32_t identifier, bool is_end)
{
  MsgQueId msgq_id =
    (identifier == OutputMixer0) ? s_self_dtq : s_sub_self_dtq;

  PlayerCommand cmd;

  cmd.req_next_param.type =
    (is_end != true) ? AsNextNormalRequest : AsNextStopResRequest;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_NEXT_REQ,
                                         msgq_id,
                                         cmd);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
PlayerObj::PlayerObj(MsgQueId self_dtq,
                     MsgQueId apu_dtq,
                     MemMgrLite::PoolId es_pool_id,
                     MemMgrLite::PoolId pcm_pool_id,
                     MemMgrLite::PoolId apu_pool_id):
  m_self_dtq(self_dtq),
  m_apu_dtq(apu_dtq),
  m_es_pool_id(es_pool_id),
  m_pcm_pool_id(pcm_pool_id),
  m_apu_pool_id(apu_pool_id),
  m_state(AS_MODULE_ID_PLAYER_OBJ, "main", BootedState),
  m_sub_state(AS_MODULE_ID_PLAYER_OBJ, "sub", InvalidSubState),
  m_input_device_handler(NULL),
  m_codec_type(InvalidCodecType),
  m_pcm_path(AsPcmDataReply)
{
}

/*--------------------------------------------------------------------------*/
void PlayerObj::run(void)
{
  err_t        err_code;
  MsgQueBlock *que;
  MsgPacket   *msg;

  err_code = MsgLib::referMsgQueBlock(m_self_dtq, &que);
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

  m_callback(table[idx],
             AS_ECODE_STATE_VIOLATION, 0);
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
      m_callback(AsPlayerEventAct,
                 AS_ECODE_CHECK_MEMORY_POOL_ERROR, 0);
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
      m_callback(AsPlayerEventAct,
                 AS_ECODE_COMMAND_PARAM_INPUT_DEVICE, 0);
      return;
  }

  result = m_input_device_handler->initialize(&in_device_handle);
  if (result)
    {
      m_callback(AsPlayerEventAct, AS_ECODE_OK, 0);
      m_state = ReadyState;
    }
  else
    {
      m_callback(AsPlayerEventAct,
                 AS_ECODE_COMMAND_PARAM_INPUT_HANDLER, 0);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::deactivate(MsgPacket *msg)
{
  msg->moveParam<PlayerCommand>();

  MEDIA_PLAYER_DBG("DEACT:\n");

  if (AS_ECODE_OK != unloadCodec())
    {
      m_callback(AsPlayerEventDeact,
                 AS_ECODE_DSP_UNLOAD_ERROR, 0);
      return;
    }

  m_callback(AsPlayerEventDeact, AS_ECODE_OK, 0);
  m_state = BootedState;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::init(MsgPacket *msg)
{
  AsInitPlayerParam param = msg->moveParam<PlayerCommand>().init_param;
  uint8_t result;
  uint32_t dsp_inf;

  MEDIA_PLAYER_DBG("INIT: ch num %d, bit len %d, codec %d(%s), fs %d\n",
                   param.channel_number,
                   param.bit_length,
                   param.codec_type,
                   param.dsp_path,
                   param.sampling_rate);

  result = m_input_device_handler->setParam(param);

  if (result == AS_ECODE_OK)
    {
      /* Update codec accordingt to audio data type. */

      /* NOTE : Codec loading process was Moved to here from play()
       *        for reducing memory usage. Exclude Gapless once.
       */

      AudioCodec next_codec = m_input_device_handler->getCodecType();

      if (m_codec_type != next_codec)
        {
          result = unloadCodec();

          if(result == AS_ECODE_OK)
            {
              result = loadCodec(next_codec, param.dsp_path, &dsp_inf);
            }
        }
    }

  m_callback(AsPlayerEventInit, result, dsp_inf);
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

      m_callback(AsPlayerEventPlay, AS_ECODE_OK, 0);
    }
  else
    {
      m_callback(AsPlayerEventPlay, rst, dsp_inf);
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
      m_callback(AsPlayerEventStop, AS_ECODE_QUEUE_OPERATION_ERROR, 0);
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

  m_callback(AsPlayerEventStop, AS_ECODE_OK, 0);

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
      m_callback(AsPlayerEventStop, AS_ECODE_QUEUE_OPERATION_ERROR, 0);
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
      m_callback(AsPlayerEventStop, AS_ECODE_QUEUE_OPERATION_ERROR, 0);
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
      m_callback(AsPlayerEventStop, AS_ECODE_QUEUE_OPERATION_ERROR, 0);
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

  if ((MemMgrLite::Manager::getPoolNumAvailSegs(m_es_pool_id) > 0) &&
        (MemMgrLite::Manager::getPoolNumAvailSegs(m_pcm_pool_id) > 1))
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
      m_callback(ext_cmd_code, AS_ECODE_OK, 0);

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

  if ((MemMgrLite::Manager::getPoolNumAvailSegs(m_es_pool_id) > 0) &&
        (MemMgrLite::Manager::getPoolNumAvailSegs(m_pcm_pool_id) > 1))
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

  if (m_decoded_pcm_mh_que.size() < MAX_EXEC_COUNT)
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
      m_callback(ext_cmd_code, AS_ECODE_OK, 0);

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

  m_callback(ext_cmd_code, AS_ECODE_OK, 0);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::setGain(MsgPacket *msg)
{
  AsSetGainParam param = msg->moveParam<PlayerCommand>().set_gain_param;

  if (!m_external_cmd_que.push(AsPlayerEventSetGain))
    {
      m_callback(AsPlayerEventSetGain,
                 AS_ECODE_QUEUE_OPERATION_ERROR, 0);
      return;
    }

  /* Set paramter to Decoder */

  SetDecCompParam setparam;

  setparam.l_gain = param.l_gain;
  setparam.r_gain = param.r_gain;

  if (AS_decode_setparam(&setparam, m_p_dec_instance) == false)
    {
      m_external_cmd_que.pop();

      m_callback(AsPlayerEventSetGain,
                 AS_ECODE_DSP_SET_ERROR, 0);
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
uint32_t PlayerObj::loadCodec(AudioCodec codec, char *path, uint32_t* dsp_inf)
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

  rst = AS_decode_activate(codec,
                           (path) ? path : CONFIG_AUDIOUTILS_DSP_MOUNTPT,
                           &m_p_dec_instance,
                           m_apu_pool_id,
                           m_apu_dtq,
                           dsp_inf);
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
     AudPcm16Bit : AudPcm24Bit);

  rst = AS_decode_init(&init_dec_comp_param, m_p_dec_instance, dsp_inf);
  if (rst != AS_ECODE_OK)
    {
      return rst;
    }

  if (!AS_decode_recv_done(m_p_dec_instance))
    {
      return AS_ECODE_QUEUE_OPERATION_ERROR;
    }

  for (int i=0; i < MAX_EXEC_COUNT; i++)
    {
      /* Get ES data. */

      uint32_t es_size = m_max_es_buff_size;
      void    *es_addr = getEs(&es_size);

      if (es_addr == NULL)
        {
          return  AS_ECODE_SIMPLE_FIFO_UNDERFLOW;
        }

    decode(es_addr, es_size);
  }
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
                                              s_self_dtq,
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
  if (mh.allocSeg(m_pcm_pool_id, size) != ERR_OK)
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

  if (mh.allocSeg(m_es_pool_id, *size) != ERR_OK)
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
  if (!MemMgrLite::Manager::isPoolAvailable(m_es_pool_id))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  m_max_es_buff_size = (MemMgrLite::Manager::getPoolSize(m_es_pool_id)) /
    (MemMgrLite::Manager::getPoolNumSegs(m_es_pool_id));

  if (!MemMgrLite::Manager::isPoolAvailable(m_pcm_pool_id))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  m_max_pcm_buff_size = (MemMgrLite::Manager::getPoolSize(m_pcm_pool_id)) /
    (MemMgrLite::Manager::getPoolNumSegs(m_pcm_pool_id));

  if (!MemMgrLite::Manager::isPoolAvailable(m_apu_pool_id))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  if ((int)(sizeof(Apu::Wien2ApuCmd)) >
      (MemMgrLite::Manager::getPoolSize(m_apu_pool_id))/
      (MemMgrLite::Manager::getPoolNumSegs(m_apu_pool_id)))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C"
{
/*--------------------------------------------------------------------------*/
int AS_PlayerObjEntry(int argc, char *argv[])
{
  PlayerObj::create(&s_play_obj,
                    s_self_dtq,
                    s_dsp_dtq,
                    s_es_pool_id,
                    s_pcm_pool_id,
                    s_apu_pool_id);
  return 0;
}

/*--------------------------------------------------------------------------*/
int AS_SubPlayerObjEntry(int argc, char *argv[])
{
  PlayerObj::create(&s_sub_play_obj,
                    s_sub_self_dtq,
                    s_sub_dsp_dtq,
                    s_sub_es_pool_id,
                    s_sub_pcm_pool_id,
                    s_sub_apu_pool_id);
  return 0;
}

/*--------------------------------------------------------------------------*/
bool AS_CreatePlayer(AsPlayerId id, FAR AsCreatePlayerParam_t *param)
{
  /* Parameter check */

  if (param == NULL)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Create */

  if (id == AS_PLAYER_ID_0)
    {
      s_self_dtq    = param->msgq_id.player;
      s_dsp_dtq     = param->msgq_id.dsp;
      s_es_pool_id  = param->pool_id.es;
      s_pcm_pool_id = param->pool_id.pcm;
      s_apu_pool_id = param->pool_id.dsp;

      s_ply_pid = task_create("PLY_OBJ",
                              150, 1024 * 3,
                              AS_PlayerObjEntry,
                              NULL);
      if (s_ply_pid < 0)
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
          return false;
        }
    }
  else
    {
      s_sub_self_dtq    = param->msgq_id.player;
      s_sub_dsp_dtq     = param->msgq_id.dsp;
      s_sub_es_pool_id  = param->pool_id.es;
      s_sub_pcm_pool_id = param->pool_id.pcm;
      s_sub_apu_pool_id = param->pool_id.dsp;
    
      s_sub_ply_pid = task_create("SUB_PLY_OBJ",
                                  150, 1024 * 3,
                                  AS_SubPlayerObjEntry,
                                  NULL);
      if (s_sub_ply_pid < 0)
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
          return false;
        }
    }

  return true;
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

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_self_dtq : s_sub_self_dtq;

  PlayerCommand cmd;

  cmd.player_id = id;
  cmd.act_param = *actparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_ACT,
                                         s_self_dtq,
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

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_self_dtq : s_sub_self_dtq;

  PlayerCommand cmd;

  cmd.player_id  = id;
  cmd.init_param = *initparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_INIT,
                                         s_self_dtq,
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

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_self_dtq : s_sub_self_dtq;

  PlayerCommand cmd;

  cmd.player_id  = id;
  cmd.play_param = *playparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_PLAY,
                                         s_self_dtq,
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

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_self_dtq : s_sub_self_dtq;

  PlayerCommand cmd;

  cmd.player_id  = id;
  cmd.stop_param = *stopparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_STOP,
                                         s_self_dtq,
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

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_self_dtq : s_sub_self_dtq;

  PlayerCommand cmd;

  cmd.player_id      = id;
  cmd.set_gain_param = *gainparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_SETGAIN,
                                         s_self_dtq,
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

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_self_dtq : s_sub_self_dtq;

  PlayerCommand cmd;

  cmd.player_id      = id;
  cmd.req_next_param = *nextparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_NEXT_REQ,
                                         s_self_dtq,
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

  MsgQueId msgq_id = (id == AS_PLAYER_ID_0) ? s_self_dtq : s_sub_self_dtq;

  PlayerCommand cmd;

  cmd.player_id   = id;
  cmd.deact_param = *deactparam;

  err_t er = MsgLib::send<PlayerCommand>(msgq_id,
                                         MsgPriNormal,
                                         MSG_AUD_PLY_CMD_DEACT,
                                         s_self_dtq,
                                         cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeletePlayer(AsPlayerId id)
{
  if (id == AS_PLAYER_ID_0)
    {
      if (s_ply_pid < 0)
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
          return false;
        }

      task_delete(s_ply_pid);

      if (s_play_obj != NULL)
        {
          delete ((PlayerObj *)s_play_obj);
          s_play_obj = NULL;
        }
    }
  else
    {
      if (s_sub_ply_pid < 0)
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
          return false;
        }
    
      task_delete(s_sub_ply_pid);

      if (s_sub_play_obj != NULL)
        {
          delete ((PlayerObj *)s_sub_play_obj);
          s_sub_play_obj = NULL;
        }
    }

  return true;
}
} /* extern "C" */

void PlayerObj::create(FAR void **obj,
                       MsgQueId self_dtq,
                       MsgQueId apu_dtq,
                       MemMgrLite::PoolId es_pool_id,
                       MemMgrLite::PoolId pcm_pool_id,
                       MemMgrLite::PoolId apu_pool_id)
{
  FAR PlayerObj *player_obj = new PlayerObj(self_dtq,
                                            apu_dtq,
                                            es_pool_id,
                                            pcm_pool_id,
                                            apu_pool_id);

  if (player_obj != NULL)
    {
      *obj = reinterpret_cast<void*>(player_obj);
      player_obj->run();
    }
  else
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }
}
