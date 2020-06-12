/****************************************************************************
 * modules/audio/objects/sound_effector/sound_effect_object.cpp
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

#include "sound_effect_object.h"

#include "memutils/common_utils/common_assert.h"
#ifdef CONFIG_AUDIOUTILS_SOUND_RECOGNIZER
#include "objects/sound_recognizer/voice_recognition_command_object.h"
#endif

__USING_WIEN2

using namespace MemMgrLite;
static MsgQueId s_self_dtq;
static MsgQueId s_manager_dtq;
static MsgQueId s_recognizer_dtq;
static MsgQueId s_dsp_dtq;
static PoolId   s_mic_in_pool_id;
static PoolId   s_i2s_in_pool_id;
static PoolId   s_hp_out_pool_id;
static PoolId   s_i2s_out_pool_id;
static PoolId   s_mfe_out_pool_id;
static pid_t    s_effector_pid = -1;
/* TODO: Hide to Class */

SoundEffectObject* s_effec_obj = NULL;

#define DBG_MODULE DBG_MODULE_AS

/* TODO: Is it better to be configurable ? */
#define MAX_CAPTURE_SAMPLE_NUM  (240)

/* TODO: Now 16bit fixed. It have to be configurable. */
#define AC_IN_BYTE_LEN  (2)

/* TODO: Now 16bit fixec. It have to be configurable. */
#define I2S_IN_BYTE_LEN (2)

/* Note: I2S only supports 2-ch output */
/* TODO: Limitation of HW, This definition
 * should be defined in baseband driver layer.
 */
#define MAX_I2S_OUT_PCM_BUF_SIZE \
  (MAX_CAPTURE_SAMPLE_NUM * AC_IN_BYTE_LEN * 2)

#define MAX_HP_OUT_PCM_BUF_SIZE \
  (MAX_CAPTURE_SAMPLE_NUM * I2S_IN_BYTE_LEN * MAX_I2S_IN_CH_NUM)

/* TODO: This value depend on Library,
 * therefore it's not good that define in object layer.
 */
#define MFE_OUT_SAMPLE_NUM (80)

/* TODO: Now 16bit fixed. It have to be configurable. */
#define MFE_OUT_BYTE_LEN (2)

/* TODO: Why not use macro created by memory library? */
#define MAX_MFE_OUT_PCM_BUF_SIZE (MFE_OUT_SAMPLE_NUM * MFE_OUT_BYTE_LEN)

/*--------------------------------------------------------------------*/
int AS_SoundEffectObjEntry(int argc, char *argv[])
{
  SoundEffectObject::create(s_self_dtq, s_manager_dtq, s_mfe_out_pool_id);
  return 0;
}

/*--------------------------------------------------------------------*/
bool AS_CreateEffector(FAR AsCreateEffectorParam_t *param)
{
  return AS_CreateEffector(param, NULL);
}

/*--------------------------------------------------------------------*/
bool AS_CreateEffector(FAR AsCreateEffectorParam_t *param, AudioAttentionCb attcb)
{
  /* Register attention callback */

  SOUNDFX_REG_ATTCB(attcb);

  /* Paramter check */

  if (param == NULL)
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Create */

  s_self_dtq        = param->msgq_id.effector;
  s_manager_dtq     = param->msgq_id.mng;
  s_recognizer_dtq  = param->msgq_id.recognizer;
  s_dsp_dtq         = param->msgq_id.dsp;
  s_mic_in_pool_id  = param->pool_id.mic_in;
  s_i2s_in_pool_id  = param->pool_id.i2s_in;
  s_hp_out_pool_id  = param->pool_id.sphp_out;
  s_i2s_out_pool_id = param->pool_id.i2s_out;
  s_mfe_out_pool_id = param->pool_id.mfe_out;

  s_effector_pid = task_create("SEFFECT_OBJ",
                               150,
                               2048,
                               AS_SoundEffectObjEntry,
                               NULL);

  if (s_effector_pid < 0)
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AS_DeleteEffector(void)
{
  if (s_effector_pid < 0)
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  task_delete(s_effector_pid);

  if (s_effec_obj != NULL)
    {
      delete s_effec_obj;
      s_effec_obj = NULL;

      /* Unregister attention callback */

      SOUNDFX_UNREG_ATTCB();
    }

  return true;
}

extern "C" {

/*--------------------------------------------------------------------*/
static void capture_done_callback(CaptureDataParam param)
{
  err_t er = MsgLib::send<CaptureDataParam>(s_self_dtq,
                                            MsgPriNormal,
                                            MSG_AUD_SEF_CMD_INPUT,
                                            s_self_dtq,
                                            param);

  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------*/
static void render_done_callback(AudioDrvDmaResult *pParam, void *p_requester)
{
  err_t er = MsgLib::send<cxd56_audio_dma_t>(s_self_dtq,
                                       MsgPriNormal,
                                       MSG_AUD_SEF_CMD_DMA_OUT_DONE,
                                       NULL,
                                       pParam->dmac_id);

  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------*/
static void render_error_callback(AudioDrvDmaError *pParam, void *p_requester)
{
}

/*--------------------------------------------------------------------*/
static bool handle_mfe_done_notification(MfeCmpltParam *p_cmplt)
{
  switch (p_cmplt->event_type)
    {
      case InitEvent:
        break;
      case ExecEvent:
#ifdef CONFIG_AUDIOUTILS_SOUND_RECOGNIZER
        {
          VoiceRecognitionCommandObject::CommandExecParam_t exec_param;

          exec_param.address = (uint32_t)p_cmplt->filtered_buffer.p_buffer;
          exec_param.size    = p_cmplt->filtered_buffer.size;

          err_t er =
            MsgLib::send<VoiceRecognitionCommandObject::CommandExecParam_t>
                                                        (s_recognizer_dtq,
                                                         MsgPriNormal,
                                                         MSG_AUD_RCG_EXEC,
                                                         s_self_dtq,
                                                         exec_param);
          F_ASSERT(er == ERR_OK);
        }
#endif /* #ifdef CONFIG_AUDIOUTILS_SOUND_RECOGNIZER */
        break;

      case StopEvent:
        {
          err_t er =
            MsgLib::send<FilterCompCmpltParam>(s_self_dtq,
                                               MsgPriNormal,
                                               MSG_AUD_SEF_CMD_CMPLT,
                                               NULL,
                                               *(p_cmplt));

          F_ASSERT(er == ERR_OK);
        }
        break;

      case SetParamEvent:
      case TuningEvent:
        break;
    }
  return true;
}

/*--------------------------------------------------------------------*/
static bool handle_mfe_post_notification(SrcCmpltParam *p_cmplt)
{
  switch (p_cmplt->event_type)
    {
      case ExecEvent:
        {
          /* TODO: Refactor data manage processing after SRC done. */

          err_t er =
            MsgLib::send<FilterCompCmpltParam>(s_self_dtq,
                                               MsgPriNormal,
                                               MSG_AUD_SEF_CMD_FILTER_DATA,
                                               NULL,
                                               *(p_cmplt));

          F_ASSERT(er == ERR_OK);
        }
        break;

      case InitEvent:
      case StopEvent:
        SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
        return false;

      case SetParamEvent:
      case TuningEvent:
        break;
    }

  return true;
}

/*--------------------------------------------------------------------*/
static bool mfe_done_callback(FilterCompCmpltParam *cmplt)
{
  if (cmplt->filter_type == MicFrontEnd)
    {
      /* 16kHz data for voice recognition
       *  | MIC-in 48kHz
       *  | -> SRC 1/3 (16kHz)
       *  | -> MFE(16kHz data is required)
       *  | -> here
       */

      return handle_mfe_done_notification(static_cast<MfeCmpltParam *>(cmplt));
    }
  else if (cmplt->filter_type == SampleRateConv)
    {
      /* 48kHz data for I2S-out
       *  | MIC-in 48kHz
       *  | -> SRC 1/3 (16kHz)
       *  | -> MFE(16kHz data is required)
       *  | -> SRC x3 (48kHz))
       *  | -> here
       */

      return handle_mfe_post_notification(static_cast<SrcCmpltParam *>(cmplt));
    }

  SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);

  return false;
}

/*--------------------------------------------------------------------*/
static bool xloud_done_callback(FilterCompCmpltParam *p_cmplt)
{
  switch (p_cmplt->event_type)
    {
      case InitEvent:
        break;
      case ExecEvent:
        {
          err_t er =
            MsgLib::send<FilterCompCmpltParam>(s_self_dtq,
                                               MsgPriNormal,
                                               MSG_AUD_SEF_CMD_FILTER_DATA,
                                               NULL,
                                               *(p_cmplt));

          F_ASSERT(er == ERR_OK);
        }
        break;

      case StopEvent:
        {
          err_t er = MsgLib::send<FilterCompCmpltParam>(s_self_dtq,
                                                        MsgPriNormal,
                                                        MSG_AUD_SEF_CMD_CMPLT,
                                                        NULL,
                                                        *(p_cmplt));

          F_ASSERT(er == ERR_OK);
        }
        break;

      case SetParamEvent:
      case TuningEvent:
        break;
    }

  return true;
}

} /* extern "C" */

/*--------------------------------------------------------------------*/
void SoundEffectObject::create(MsgQueId self_dtq,
                               MsgQueId manager_dtq,
                               MsgQueId voice_recognition_dtq)
{
  if (s_effec_obj == NULL)
    {
      s_effec_obj = new SoundEffectObject(self_dtq,
                                          manager_dtq,
                                          voice_recognition_dtq);

      if (s_effec_obj == NULL)
        {
          SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
          return;
        }

      s_effec_obj->run();
    }
  else
    {
      F_ASSERT(0);
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::run()
{
  err_t        err_code;
  MsgQueBlock* que;
  MsgPacket*   msg;

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

/*--------------------------------------------------------------------*/
void SoundEffectObject::parse(MsgPacket* msg)
{
    uint32_t event  = MSG_GET_SUBTYPE(msg->getType());
    F_ASSERT((event < AUD_SEF_MSG_NUM));

    (this->*MsgProcTbl[event][m_state.get()])(msg);
}

/*--------------------------------------------------------------------*/
SoundEffectObject::MsgProc SoundEffectObject::MsgProcTbl[AUD_SEF_MSG_NUM][SoundFXStateNum] =
{
  /* ACT  */
  {                                               /* SoundEffector Status:  */
    &SoundEffectObject::act,                      /*   SoundFXBootedState   */
    &SoundEffectObject::illegal,                  /*   SoundFXReadyState    */
    &SoundEffectObject::illegal,                  /*   SoundFXRunState      */
    &SoundEffectObject::illegal                   /*   SoundFXStoppingState */
  },

  /* DEACT */
  {                                               /* SoundEffector Status:  */
    &SoundEffectObject::illegal,                  /*   SoundFXBootedState   */
    &SoundEffectObject::deact,                    /*   SoundFXReadyState    */
    &SoundEffectObject::illegal,                  /*   SoundFXRunState      */
    &SoundEffectObject::illegal                   /*   SoundFXStoppingState */
  },

  /* INIT  */
  {                                               /* SoundEffector Status:  */
    &SoundEffectObject::illegal,                  /*   SoundFXBootedState   */
    &SoundEffectObject::init,                     /*   SoundFXReadyState    */
    &SoundEffectObject::illegal,                  /*   SoundFXRunState      */
    &SoundEffectObject::illegal                   /*   SoundFXStoppingState */
  },

  /* START */
  {                                               /* SoundEffector Status:  */
    &SoundEffectObject::illegal,                  /*   SoundFXBootedState   */
    &SoundEffectObject::startOnReady,             /*   SoundFXReadyState    */
    &SoundEffectObject::illegal,                  /*   SoundFXRunState      */
    &SoundEffectObject::illegal                   /*   SoundFXStoppingState */
  },

  /* STOP */
  {                                               /* SoundEffector Status:  */
    &SoundEffectObject::illegal,                  /*   SoundFXBootedState   */
    &SoundEffectObject::illegal,                  /*   SoundFXReadyState    */
    &SoundEffectObject::stopOnActive,             /*   SoundFXRunState      */
    &SoundEffectObject::illegal                   /*   SoundFXStoppingState */
  },

  /* INPUT */
  {                                               /* SoundEffector Status:  */
    &SoundEffectObject::illegal,                  /*   SoundFXBootedState   */
    &SoundEffectObject::illegalInput,             /*   SoundFXReadyState    */
    &SoundEffectObject::inputOnActive,            /*   SoundFXRunState      */
    &SoundEffectObject::inputOnStopping           /*   SoundFXStoppingState */
  },

  /* FILTER_DATA */
  {                                               /* SoundEffector Status:  */
    &SoundEffectObject::illegal,                  /*   SoundFXBootedState   */
    &SoundEffectObject::illegal,                  /*   SoundFXReadyState    */
    &SoundEffectObject::filterRstOnActive,        /*   SoundFXRunState      */
    &SoundEffectObject::filterRstOnStopping       /*   SoundFXStoppingState */
  },

  /* DMA_OUT_DONE */
  {                                               /* SoundEffector Status:  */
    &SoundEffectObject::illegal,                  /*   SoundFXBootedState   */
    &SoundEffectObject::illegal,                  /*   SoundFXReadyState    */
    &SoundEffectObject::dmaOutDoneCmpltOnActive,  /*   SoundFXRunState      */
    &SoundEffectObject::dmaOutDoneCmpltOnStopping /*   SoundFXStoppingState */
  },

  /* SET_PARAM */
  {                                               /* SoundEffector Status:  */
    &SoundEffectObject::illegal,                  /*   SoundFXBootedState   */
    &SoundEffectObject::setParam,                 /*   SoundFXReadyState    */
    &SoundEffectObject::setParam,                 /*   SoundFXRunState      */
    &SoundEffectObject::illegal                   /*   SoundFXStoppingState */
  },

  /* CMPLT */
  {                                               /* SoundEffector Status:  */
    &SoundEffectObject::illegal,                  /*   SoundFXBootedState   */
    &SoundEffectObject::illegal,                  /*   SoundFXReadyState    */
    &SoundEffectObject::illegal,                  /*   SoundFXRunState      */
    &SoundEffectObject::filterDoneCmplt           /*   SoundFXStoppingState */
  },
};
/*--------------------------------------------------------------------*/
void SoundEffectObject::illegal(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();
  sendAudioCmdCmplt(cmd, AS_ECODE_STATE_VIOLATION);
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::act(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  SOUNDFX_DBG("ACT: with <MPP %d/Voice Command %d/MFE %d>, "
              "indev 0x%04x, outdev 0x%04x\n",
              cmd.set_baseband_status_param.with_MPP,
              cmd.set_baseband_status_param.with_Voice_Command,
              cmd.set_baseband_status_param.with_MFE,
              cmd.set_baseband_status_param.input_device,
              cmd.set_baseband_status_param.output_device);

  /* Check IN/OUT device first. Currently,
   * it only supports the following settings.
   */

  if (AS_INPUT_DEVICE_AMIC1CH_I2S2CH !=
      cmd.set_baseband_status_param.input_device)
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_INPUT_DEVICE);
      return;
    }
  if (AS_OUTPUT_DEVICE_SP2CH_I2S2CH !=
      cmd.set_baseband_status_param.output_device)
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_OUTPUT_DEVICE);
      return;
    }

  if (!AS_get_capture_comp_handler(&m_capture_from_mic_hdlr,
                                   CaptureDeviceAnalogMic,
                                   s_mic_in_pool_id)
   || !AS_get_capture_comp_handler(&m_capture_from_i2s_hdlr,
                                   CaptureDeviceI2S,
                                   s_i2s_in_pool_id))
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_SET_AUDIO_DATA_PATH_ERROR);
      return;
    }

  if (!AS_get_render_comp_handler(&m_hp_render_comp_handler,
                                  RenderDeviceHPSP)
   || !AS_get_render_comp_handler(&m_i2s_render_comp_handler,
                                  RenderDeviceI2S))
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_SET_AUDIO_DATA_PATH_ERROR);
      return;
    }

  if (!AS_init_renderer(m_hp_render_comp_handler,
                        &render_done_callback,
                        &render_error_callback,
                        static_cast<void*>(this),
                        AS_BITLENGTH_16)
   || !AS_init_renderer(m_i2s_render_comp_handler,
                        &render_done_callback,
                        &render_error_callback,
                        static_cast<void*>(this),
                        AS_BITLENGTH_16))
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_DMAC_INITIALIZE_ERROR);
      return;
    }

  /* set filter mode */
  if ((cmd.set_baseband_status_param.with_MFE != AS_SET_BBSTS_WITH_MFE_NONE &&
      cmd.set_baseband_status_param.with_MFE != AS_SET_BBSTS_WITH_MFE_ACTIVE))
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_WITH_MFE);
      return;
    }

  if ((cmd.set_baseband_status_param.with_MPP != AS_SET_BBSTS_WITH_MPP_NONE &&
      cmd.set_baseband_status_param.with_MPP != AS_SET_BBSTS_WITH_MPP_ACTIVE))
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_WITH_MPP);
      return;
    }

  m_filter_mode = FILTER_MODE_THROUGH;

  if (AS_SET_BBSTS_WITH_MFE_ACTIVE == cmd.set_baseband_status_param.with_MFE)
    {
      m_filter_mode |= FILTER_MODE_MFE;
    }

  if (AS_SET_BBSTS_WITH_MPP_ACTIVE == cmd.set_baseband_status_param.with_MPP)
    {
      m_filter_mode |= FILTER_MODE_MPPEAX;
    }

  if (m_filter_mode != FILTER_MODE_THROUGH)
    {
      uint32_t rst = AS_ECODE_OK;
      uint32_t dsp_inf = 0;
      
      if (m_filter_mode == FILTER_MODE_MPPEAX)
        {
          /* MPPeax Only */

          /* T.B.D. MPP EAX only is not supported */

          sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_WITH_MPP);
          return;
        }
      else
        {
          /*
           * Case MFE
           *  * Load/Unload DSP is done by MFEComp
           *
           * +--------+ MicIn +-------+      +========+
           * | SfxObj |--+--->|MFEcomp|----->|        |
           * +--------+  |    +-------+      | MFESRC |
           *             |    +-------+      |  DSP   |
           *             +--->|MPPcomp|----->|        |
           *            I2SIn +-------+      +========+
           */

          /*
           * Case MFE + MPPEAX
           *
           * +--------+ MicIn +-------+      +========+
           * | SfxObj |--+--->|MFEcomp|----->| MFESRC |
           * +--------+  |    +-------+      +========+
           *             |    +-------+      +========+
           *             +--->|MPPcomp|----->| MPPEAX |
           *            I2SIn +-------+      +========+
           *
           */

          FilterComponentType mpp_acttype = FilterComponentTypeNum;

          if (m_filter_mode & FILTER_MODE_MFE)
            {
              rst = AS_filter_activate(MicFrontEnd,
                                       cmd.set_baseband_status_param.dsp_path,
                                       s_dsp_dtq,
                                       0,
                                       &dsp_inf,
                                       mfe_done_callback,
                                       &m_mfe_instance);

              /* If MFE is active, use MPP component too. */

              mpp_acttype = MediaPlayerPostAsSub;
            }

          if (m_filter_mode == (FILTER_MODE_MFE + FILTER_MODE_MPPEAX))
            {
              mpp_acttype = MediaPlayerPost;
            }

          if (mpp_acttype != FilterComponentTypeNum)
            {
              rst = AS_filter_activate(mpp_acttype,
                                       cmd.set_baseband_status_param.dsp_path,
                                       s_dsp_dtq,
                                       0,
                                       &dsp_inf,
                                       xloud_done_callback,
                                       &m_mpp_instance);
            }
        }

      if (rst != AS_ECODE_OK)
        {
          sendAudioCmdCmplt(cmd, rst, dsp_inf);
          return;
        }
    }

  m_state = SoundFXReadyState;

  sendAudioCmdCmplt(cmd, AS_ECODE_OK);
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::deact(MsgPacket* msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  SOUNDFX_DBG("DEACT:\n");

  AS_release_capture_comp_handler(m_capture_from_mic_hdlr);
  AS_release_capture_comp_handler(m_capture_from_i2s_hdlr);

  AS_release_render_comp_handler(m_hp_render_comp_handler);
  AS_release_render_comp_handler(m_i2s_render_comp_handler);

  if (m_filter_mode != FILTER_MODE_THROUGH)
    {
      FilterComponentType mpp_acttype = FilterComponentTypeNum;

      if (m_mfe_instance)
        {
          if (!AS_filter_deactivate(m_mfe_instance, MicFrontEnd))
            {
              sendAudioCmdCmplt(cmd, AS_ECODE_DSP_UNLOAD_ERROR);
              return;
            }

          m_mfe_instance = NULL;

          mpp_acttype = MediaPlayerPostAsSub;
        }

      if (m_mpp_instance)
        {
          if (m_filter_mode == (FILTER_MODE_MFE + FILTER_MODE_MPPEAX))
            {
              mpp_acttype = MediaPlayerPost;
            }

          if (!AS_filter_deactivate(m_mpp_instance, mpp_acttype))
            {
              sendAudioCmdCmplt(cmd, AS_ECODE_DSP_UNLOAD_ERROR);
              return;
            }

          m_mpp_instance = NULL;
        }
    }

  m_state = SoundFXBootedState;

  sendAudioCmdCmplt(cmd, AS_ECODE_OK);
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::init(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();
  uint32_t result = AS_ECODE_OK;

  switch(cmd.header.command_code)
    {
      case AUDCMD_INITMFE:
        result = initMfe(cmd);
        break;

      case AUDCMD_INITMPP:
        result = initMpp(cmd);
        break;

      default:
        SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        result = AS_ECODE_COMMAND_CODE_ERROR;
        break;
    }

  sendAudioCmdCmplt(cmd, result);
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::startOnReady(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  SOUNDFX_DBG("START: outdev 0x%04x, indev 0x%04x, "
              "out mic %d, i2s out %d, sp out %d\n",
              cmd.start_bb_param.output_device,
              cmd.start_bb_param.input_device,
              cmd.start_bb_param.select_output_mic,
              cmd.start_bb_param.I2S_output_data,
              cmd.start_bb_param.SP_output_data);

  /* check fixed parameter */
  if (AS_INPUT_DEVICE_AMIC1CH_I2S2CH != cmd.start_bb_param.input_device)
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_INPUT_DEVICE);
      return;
    }

  if (AS_OUTPUT_DEVICE_SP2CH_I2S2CH != cmd.start_bb_param.output_device)
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_OUTPUT_DEVICE);
      return;
    }

  if (AS_MPP_OUTPUT_I2SIN != cmd.start_bb_param.SP_output_data)
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_OUTPUT_DATE);
      return;
    }

  /* check validity of MFE output */
  if (AS_MFE_OUTPUT_MICSIN == cmd.start_bb_param.I2S_output_data)
    {
      if (m_filter_mode == FILTER_MODE_THROUGH)
        {
          sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_OUTPUT_DATE);
          return;
        }
    }
  else if (AS_MIC_THROUGH == cmd.start_bb_param.I2S_output_data)
    {
      if (AS_SELECT_MIC1_OR_MIC2 == cmd.start_bb_param.select_output_mic
       || AS_SELECT_MIC0_OR_MIC3 == cmd.start_bb_param.select_output_mic)
        {
          m_select_output_mic =
            (AsSelectOutputMic)cmd.start_bb_param.select_output_mic;
        }
      else
        {
          sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_SELECT_MIC);
          return;
        }
    }
  else
    {
      sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_PARAM_OUTPUT_DATE);
      return;
    }

  m_I2S_output_data = (AsI2sOutputData)cmd.start_bb_param.I2S_output_data;

  uint32_t rst = AS_ECODE_OK;
  uint32_t dsp_inf = 0;

  if (m_filter_mode != FILTER_MODE_THROUGH)
    {
      SOUNDFX_DBG("[[ %08x, %08x ]]\n", ((MFEComponent *)m_mfe_instance)->m_dsp_handler, ((MPPComponent *)m_mpp_instance)->m_dsp_handler);
      if ((rst = AS_filter_init(&m_init_mfe_param, &dsp_inf, m_mfe_instance))
          != AS_ECODE_OK)
        {
          sendAudioCmdCmplt(cmd, rst, dsp_inf);
          return;
        }

      if ((rst = AS_filter_init(&m_init_xloud_param, &dsp_inf, m_mpp_instance))
          != AS_ECODE_OK)
        {
          sendAudioCmdCmplt(cmd, rst, dsp_inf);
          return;
        }
    }

  /* Multiple capture commands are issued to I2S-in and Mic-in respectively */

  for (int i = 0; i < CAPTURE_DELAY_STAGE_NUM; i++)
    {
      CaptureComponentParam cap_comp_param;

      cap_comp_param.handle                = m_capture_from_mic_hdlr;
      cap_comp_param.exec_param.pcm_sample = MAX_CAPTURE_SAMPLE_NUM;

      if (!AS_exec_capture(&cap_comp_param))
        {
          sendAudioCmdCmplt(cmd, AS_ECODE_DMAC_READ_ERROR);
          return;
        }

      cap_comp_param.handle                = m_capture_from_i2s_hdlr;
      cap_comp_param.exec_param.pcm_sample = MAX_CAPTURE_SAMPLE_NUM;

      if (!AS_exec_capture(&cap_comp_param))
        {
          sendAudioCmdCmplt(cmd, AS_ECODE_DMAC_READ_ERROR);
          return;
        }
    }

  /* initialize sync counter of mic and i2s */

  m_mic_in_sync_cnt = 0;
  m_i2s_in_sync_cnt = 0;
  m_capt_sync_wait_flg = true;

  m_state = SoundFXRunState;

  sendAudioCmdCmplt(cmd, AS_ECODE_OK);
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::stopOnActive(MsgPacket* msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  SOUNDFX_DBG("STOP:\n");

  if (!m_external_cmd_que.push(cmd))
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      sendAudioCmdCmplt(cmd, AS_ECODE_QUEUE_OPERATION_ERROR);
      return;
    }

  m_state = SoundFXStoppingState;
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::illegalInput(MsgPacket *msg)
{
  msg->popParam<CaptureDataParam>();
  SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::selectCh4to2(uint16_t *p_src, uint16_t *p_dst, uint32_t sample_num)
{
  /* 4ch -> 2ch */

  if (m_select_output_mic == AS_SELECT_MIC1_OR_MIC2)
    {
      /* MIC1 and MIC2 */
      for(uint32_t i = 0; i < sample_num; i++)
        {
          p_dst[0] = p_src[1];
          p_dst[1] = p_src[2];
          p_src += 4;
          p_dst += 2;
        }
    }
  else
    {
      /* MIC0 and MIC3 */
      for(uint32_t i = 0; i < sample_num; i++)
        {
          p_dst[0] = p_src[0];
          p_dst[1] = p_src[3];
          p_src += 4;
          p_dst += 2;
        }
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::convertCh1to2(uint16_t *p_src, uint16_t *p_dst, uint32_t sample_num)
{
  /* 1ch -> 2ch */

  for (uint32_t i = 0; i < sample_num; i++)
    {
      p_dst[0] = p_src[0];
      p_dst[1] = p_src[0];
      p_src += 1;
      p_dst += 2;
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::input(CaptureDataParam& param)
{
  if (CaptureDeviceAnalogMic == param.output_device)
    {
      /* case of MFE is active */
      if ((m_filter_mode & FILTER_MODE_MFE) != 0)
        {
          /* TODO: Hide the name in Pre-.
           * It may be recognized as a Front-end process.
           */

          execMfe(param.buf.cap_mh, param.buf.sample, param.end_flag);
          m_mic_in_sync_cnt++;
        }
      else
        {
          FilterCompCmpltParam render_param;

          render_param.out_buffer.p_buffer =
            reinterpret_cast<unsigned long*>(allocI2SOutBuf());

          render_param.out_buffer.size =
            param.buf.sample * m_i2s_out_ch_num * AC_IN_BYTE_LEN;

          /* case of MFE is through */
          if (m_mic_in_ch_num == 4)
            {
              selectCh4to2
              ((uint16_t *)param.buf.cap_mh.getPa(),
               (uint16_t *)render_param.out_buffer.p_buffer,
               (uint32_t)param.buf.sample);
            }
          else
            {
              convertCh1to2
              ((uint16_t *)param.buf.cap_mh.getPa(),
               (uint16_t *)render_param.out_buffer.p_buffer,
               (uint32_t)param.buf.sample);
            }

          execI2SOutRender(&render_param);

          /* stop process */
          if (m_state.get() != SoundFXRunState)
            {
              if (!param.end_flag)
                {
                  /* The number of commands issued to DMA is equal to
                   * m_mic_in_buf_mh_que.size().
                   */

                  return;
                }

              if (!AS_stop_renderer(m_i2s_render_comp_handler,
                                    AS_DMASTOPMODE_NORMAL))
                {
                  return;
                }
            }
        }
    }
  else if (CaptureDeviceI2S == param.output_device)
    {
      /* case of MFE is active */
      if((m_filter_mode & FILTER_MODE_MFE) != 0)
        {
          execMpp(param.buf.cap_mh, param.buf.sample, param.end_flag);
          m_i2s_in_sync_cnt++;
        }
      else
        {
          /* case of MFE is through */
          FilterCompCmpltParam render_param;

          render_param.out_buffer.p_buffer =
            reinterpret_cast<unsigned long*>(allocHpOutBuf());

          render_param.out_buffer.size =
            param.buf.sample * MAX_I2S_IN_CH_NUM * I2S_IN_BYTE_LEN;

          if (render_param.out_buffer.size > MAX_HP_OUT_PCM_BUF_SIZE)
            {
              /* It is impossible to enter here, but it becomes an Assertion
               * if you carelessly change the code
               */

              SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
              return;
            }

          memcpy(render_param.out_buffer.p_buffer,
                 param.buf.cap_mh.getPa(),
                 render_param.out_buffer.size);

          execHpSpOutRender(&render_param);

          /* stop process */

          if (m_state.get() != SoundFXRunState)
            {
              if (!param.end_flag)
                {
                  return;
                }

              if (!AS_stop_renderer(m_hp_render_comp_handler,
                                    AS_DMASTOPMODE_NORMAL))
                {
                  return;
                }
            }
        }
    }
  else
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::inputOnActive(MsgPacket *msg)
{
  CaptureDataParam param = msg->moveParam<CaptureDataParam>();

  /* Send input data to filter. */

  input(param);

  /* Set next command. */

  if (CaptureDeviceAnalogMic == param.output_device)
    {
      /* set next command */

      CaptureComponentParam cap_comp_param;

      cap_comp_param.handle                = m_capture_from_mic_hdlr;
      cap_comp_param.exec_param.pcm_sample = MAX_CAPTURE_SAMPLE_NUM;

      AS_exec_capture(&cap_comp_param);
    }
  else if (CaptureDeviceI2S == param.output_device)
    {
      /* set next command */

      CaptureComponentParam cap_comp_param;

      cap_comp_param.handle                = m_capture_from_i2s_hdlr;
      cap_comp_param.exec_param.pcm_sample = MAX_CAPTURE_SAMPLE_NUM;

      AS_exec_capture(&cap_comp_param);
    }
  else
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::inputOnStopping(MsgPacket *msg)
{
  CaptureDataParam param = msg->peekParam<CaptureDataParam>();

  if (m_capt_sync_wait_flg)
    {
      if (m_mic_in_sync_cnt == m_i2s_in_sync_cnt)
        {
          /* Stop capture of mic-in and i2s-in */

          CaptureComponentParam cap_comp_param;

          cap_comp_param.handle          = m_capture_from_mic_hdlr;
          cap_comp_param.stop_param.mode = AS_DMASTOPMODE_NORMAL;

          AS_stop_capture(&cap_comp_param);

          cap_comp_param.handle = m_capture_from_i2s_hdlr;

          AS_stop_capture(&cap_comp_param);

          m_capt_sync_wait_flg = false;
        }
      else
        {
          /* Capture remaining data. Because mic-in and
           * i2s-in are out of synchronization.
           */

          inputOnActive(msg);
          return;
        }
    }

  /* Send input data to filter */

  input(param);

  /* pop message parameter */

  msg->popParam<CaptureDataParam>();
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::filterRstOnActive(MsgPacket *msg)
{
  FilterCompCmpltParam param = msg->moveParam<FilterCompCmpltParam>();

  if (param.filter_type == SampleRateConv)
    {
      /* Send MFE processed data to I2S-out and voice recognition object. */

      freeMfeInBuf();

      execI2SOutRender(&param);
    }
  else if (param.filter_type == MediaPlayerPost)
    {
      /* Send MPP processed data to HP. */

      freeMppInBuf();

      execHpSpOutRender(&param);
    }
  else
    {
      printf("ft!!  %0x\n", param.filter_type);
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::filterRstOnStopping(MsgPacket *msg)
{
  FilterCompCmpltParam param = msg->moveParam<FilterCompCmpltParam>();

  /* TODO: Refactor process sequence.
   * It has potential of using freed memory in APU.
   */

  if (param.filter_type == SampleRateConv)
    {
      execI2SOutRender(&param);

      if (m_mfe_in_buf_mh_que.top().is_end)
        {
          /* There is no wait for MIC-capture-in or MFE-filter-proc. */

          if (!AS_stop_renderer(m_i2s_render_comp_handler,
                                AS_DMASTOPMODE_NORMAL))
            {
              return;
            }

          StopMFEParam stopParam;

          AS_filter_stop(&stopParam, m_mfe_instance);
        }

      freeMfeInBuf();
    }
  else if (param.filter_type == MediaPlayerPost)
    {
      execHpSpOutRender(&param);

      if (m_mpp_in_buf_mh_que.top().is_end)
        {
          /* There is no wait for I2S-capture-in or MPP-filter-proc. */

          if (!AS_stop_renderer(m_hp_render_comp_handler,
                                AS_DMASTOPMODE_NORMAL))
            {
              return;
            }

          StopXLOUDParam stopParam;

          AS_filter_stop(&stopParam, m_mpp_instance);
        }

      freeMppInBuf();
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::filterDoneCmplt(MsgPacket *msg)
{
  /* If state becames "Sttopping", last data will discarded. */

  FilterCompCmpltParam param= msg->moveParam<FilterCompCmpltParam>();
  (void)param;
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::dmaOutDoneCmpltOnActive(MsgPacket *msg)
{
  cxd56_audio_dma_t param= msg->moveParam<cxd56_audio_dma_t>();

  freeOutBuf(param);
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::dmaOutDoneCmpltOnStopping(MsgPacket *msg)
{
  cxd56_audio_dma_t param = msg->moveParam<cxd56_audio_dma_t>();

  freeOutBuf(param);

  /* TODO: It's better to check with end-flag(component). */

  if (m_i2s_out_buf_mh_que.empty() && m_hp_out_buf_mh_que.empty())
    {
      if (m_external_cmd_que.empty())
        {
          SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
          return;
        }

      AudioCommand ext_cmd = m_external_cmd_que.top();

      if (!m_external_cmd_que.pop())
        {
          SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }

      m_state = SoundFXReadyState;
      sendAudioCmdCmplt(ext_cmd, AS_ECODE_OK);
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::freeOutBuf(cxd56_audio_dma_t dmac_select_id)
{
  switch(dmac_select_id)
    {
      case CXD56_AUDIO_DMAC_I2S1_DOWN:
        freeI2SOutBuf();
        if((m_filter_mode & FILTER_MODE_MFE) != 0)
          {
            freeMfeOutBuf();
          }
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        freeHpOutBuf();
        break;

      default:
          break;
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::setParam(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  bool result = false;

  SOUNDFX_DBG("SET PARAM: xloud vol %d\n",
              cmd.set_mpp_param.mpp_xloud_set.xloud_vol);

  switch(cmd.header.sub_code)
    {
      case SUB_SETMPP_COMMON:
        {
          SetXLOUDParam filter_param;
          filter_param.param_idx = cmd.header.sub_code;

          result = AS_filter_setparam(&filter_param, m_mfe_instance);

          if (!result)
            {
              sendAudioCmdCmplt(cmd, AS_ECODE_QUEUE_OPERATION_ERROR);
              return;
            }

          break;
        }

      case SUB_SETMPP_XLOUD:
        {
          SetXLOUDParam filter_param;
          filter_param.param_idx = cmd.header.sub_code;
          filter_param.xloud_vol = cmd.set_mpp_param.mpp_xloud_set.xloud_vol;

          result = AS_filter_setparam(&filter_param, m_mpp_instance);

          if (!result)
            {
              sendAudioCmdCmplt(cmd, AS_ECODE_QUEUE_OPERATION_ERROR);
              return;
            }
          break;
        }

      default:
        sendAudioCmdCmplt(cmd, AS_ECODE_COMMAND_CODE_ERROR);
        return;
    }

  /* TODO: This is not synchronous command. */

  sendAudioCmdCmplt(cmd, AS_ECODE_OK);
}

/*--------------------------------------------------------------------*/
uint32_t SoundEffectObject::initMfe(const AudioCommand& cmd)
{
  SOUNDFX_DBG("INIT MFE: infs %d, ch num <ref %d/mic %d>, "
              "echocan <enable %d/include %d>, mode %d\n",
              cmd.init_mfe_param.input_fs,
              cmd.init_mfe_param.ref_channel_num,
              cmd.init_mfe_param.mic_channel_num,
              cmd.init_mfe_param.enable_echocancel,
              cmd.init_mfe_param.include_echocancel,
              cmd.init_mfe_param.mfe_mode);

  if (AS_CHANNEL_MONO != cmd.init_mfe_param.mic_channel_num)
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
    }

  if (cmd.init_mfe_param.input_fs != AS_SAMPLINGRATE_16000)
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
    }

  if (AS_CHANNEL_STEREO != cmd.init_mfe_param.ref_channel_num)
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
    }

  if (AS_SAMPLINGRATE_16000 != cmd.init_mfe_param.input_fs)
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
    }

  m_init_mfe_param.proc_mode = cmd.init_mfe_param.mfe_mode;
  m_init_mfe_param.ch_num    = cmd.init_mfe_param.mic_channel_num;
  m_init_mfe_param.ref_channel_num = cmd.init_mfe_param.ref_channel_num;
  m_init_mfe_param.in_fs = cmd.init_mfe_param.input_fs;
  m_init_mfe_param.sample_per_frame =
    MAX_CAPTURE_SAMPLE_NUM
    * m_init_mfe_param.in_fs
    / AudioFs2ApuValue[AudFs_48000];

  m_init_mfe_param.use_aec =
    (cmd.init_mfe_param.include_echocancel == 1) ? true : false;

  m_init_mfe_param.enable_mfe_aec =
    (cmd.init_mfe_param.enable_echocancel == 1) ? true : false;

  m_init_mfe_param.config_table = cmd.init_mfe_param.config_table;

  SOUNDFX_INF(0);

  m_mic_in_ch_num = cmd.init_mfe_param.mic_channel_num;
  m_i2s_in_ch_num = cmd.init_mfe_param.ref_channel_num;
  SOUNDFX_INF(0);

  /* Initialize capture component for MIC-in. */

  CaptureComponentParam cap_comp_param;

  cap_comp_param.init_param.capture_ch_num    = m_mic_in_ch_num;
  /* TODO: Fixed valuse */
  cap_comp_param.init_param.capture_bit_width = AudPcmFormatInt16;
  cap_comp_param.init_param.callback          = capture_done_callback;
  cap_comp_param.handle                       = m_capture_from_mic_hdlr;

  if (!AS_init_capture(&cap_comp_param))
    {
      return AS_ECODE_DMAC_INITIALIZE_ERROR;
    }

  /* Initialize capture component for I2S-in. */
  /* TODO: There is a case that do not capture I2S,
   *       Therefor should be configurable to disable.
   */
  cap_comp_param.init_param.capture_ch_num    = m_i2s_in_ch_num;
  /* TODO: Fixed value */
  cap_comp_param.init_param.capture_bit_width = AudPcmFormatInt16;
  cap_comp_param.init_param.callback          = capture_done_callback;
  cap_comp_param.handle                       = m_capture_from_i2s_hdlr;

  if (!AS_init_capture(&cap_comp_param))
    {
      return AS_ECODE_DMAC_INITIALIZE_ERROR;
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
uint32_t SoundEffectObject::initMpp(const AudioCommand& cmd)
{
  SOUNDFX_DBG("INIT MPP: outfs %d, out ch num %d, "
              "mode <mpp %d/eax %d/xloud %d>, coef mode %d\n",
              cmd.init_mpp_param.output_fs,
              cmd.init_mpp_param.output_channel_num,
              cmd.init_mpp_param.mpp_mode,
              cmd.init_mpp_param.eax_mode,
              cmd.init_mpp_param.xloud_mode,
              cmd.init_mpp_param.coef_mode);

  m_init_xloud_param.ch_num = cmd.init_mpp_param.output_channel_num;
  m_init_xloud_param.sample_per_frame = MAX_CAPTURE_SAMPLE_NUM;

  m_i2s_out_ch_num = cmd.init_mpp_param.output_channel_num;

  switch (cmd.init_mpp_param.output_fs)
    {
      case AS_SAMPLINGRATE_48000:
        /* fixed value */
        m_init_xloud_param.in_fs =
          cmd.init_mpp_param.output_fs;
        break;

      default:
        SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
    }

  m_init_xloud_param.mode =
    static_cast<Apu::AudioXloudMode>(cmd.init_mpp_param.xloud_mode);

  m_init_xloud_param.in_bytelength = 2;
  m_init_xloud_param.out_bytelength = 2;
  m_init_xloud_param.p_xloud_coef_image =
    reinterpret_cast<void *>(cmd.init_mpp_param.xloud_coef_table);

  m_init_xloud_param.xloud_coef_size =
    (m_init_xloud_param.p_xloud_coef_image == NULL) ? 0 : 512;

  m_init_xloud_param.p_eax_coef_image =
    reinterpret_cast<void *>(cmd.init_mpp_param.eax_coef_table);

  m_init_xloud_param.eax_coef_size =
    (m_init_xloud_param.p_eax_coef_image == NULL) ? 0 : 512;

  m_init_xloud_param.p_sel_out_param = NULL;

  SOUNDFX_INF(0);

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::execI2SOutRender(FilterCompCmpltParam *param)
{
  if (param->out_buffer.p_buffer == NULL)
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }

   /* Note: I2S-out is support max 2ch. */
  if (!AS_exec_renderer
        (m_i2s_render_comp_handler,
        (void*)(param->out_buffer.p_buffer),
        param->out_buffer.size 
          / m_i2s_out_ch_num 
          / AC_IN_BYTE_LEN,
        true))
    {
      return;
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::execHpSpOutRender(FilterCompCmpltParam *param)
{
  if (param->out_buffer.p_buffer == NULL)
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }

  /* Note: HPSP is 2ch out. */

  if (!AS_exec_renderer
        (m_hp_render_comp_handler,
        (void*)(param->out_buffer.p_buffer),
        param->out_buffer.size / 2 / I2S_IN_BYTE_LEN,
        true))
    {
      return;
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::execMfe(MemMgrLite::MemHandle mh,
                                int32_t sample,
                                bool is_end)
{
  ExecMFEParam param;

  param.in_buffer.p_buffer =
    reinterpret_cast<unsigned long*>(mh.getPa());

  param.in_buffer.size =
    sample * m_mic_in_ch_num * AC_IN_BYTE_LEN;

  param.out_buffer.p_buffer =
    reinterpret_cast<unsigned long*>(allocI2SOutBuf());

  param.out_buffer.size = 0;

  param.notification_buffer.p_buffer =
    reinterpret_cast<unsigned long*>(allocMfeOutBuf());

  param.notification_buffer.size = 0;

  AS_filter_exec(&param, m_mfe_instance);

  /* Copy MH from MIC-captured data to mfe-in data. */

  SoundFxBufParam micin;
  micin.mh     = mh;
  micin.sample = sample;
  micin.is_end = is_end;

  if (!m_mfe_in_buf_mh_que.push(micin))
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
    }
}

/*--------------------------------------------------------------------*/
void SoundEffectObject::execMpp(MemMgrLite::MemHandle mh,
                                int32_t sample,
                                bool is_end)
{
  /* First, Send I2S-in data to xLOUD(MPP) filter component.
   * Next, next request I2S-in caputure command.
   */

  ExecXLOUDParam param;

  param.in_buffer.p_buffer =
    reinterpret_cast<unsigned long*>(mh.getPa());

  param.in_buffer.size =
    sample * m_i2s_in_ch_num * I2S_IN_BYTE_LEN;

  param.out_buffer.p_buffer =
    reinterpret_cast<unsigned long*>(allocHpOutBuf());

  param.out_buffer.size = 0;

  AS_filter_exec(&param, m_mpp_instance);

  /* Copy MH from I2S-captured data to mpp-in data. */

  SoundFxBufParam i2sin;
  i2sin.mh     = mh;
  i2sin.sample = sample;
  i2sin.is_end = is_end;

  if (!m_mpp_in_buf_mh_que.push(i2sin))
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
    }
}

/*--------------------------------------------------------------------*/
void* SoundEffectObject::allocHpOutBuf()
{
  MemMgrLite::MemHandle mh;

  if (mh.allocSeg(s_hp_out_pool_id, MAX_HP_OUT_PCM_BUF_SIZE) != ERR_OK)
    {
      SOUNDFX_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return NULL;
    }

  SoundFxBufParam data;

  data.mh = mh;
  data.sample = MAX_CAPTURE_SAMPLE_NUM;
  data.is_end = false;

  if (!m_hp_out_buf_mh_que.push(data))
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return NULL;
    }

  return mh.getPa();
}

/*--------------------------------------------------------------------*/
void* SoundEffectObject::allocI2SOutBuf()
{
  MemMgrLite::MemHandle mh;

  if (mh.allocSeg(s_i2s_out_pool_id, MAX_I2S_OUT_PCM_BUF_SIZE) != ERR_OK)
    {
      SOUNDFX_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return NULL;
    }

  SoundFxBufParam data;

  data.mh = mh;
  data.sample = MAX_CAPTURE_SAMPLE_NUM;
  data.is_end = false;

  if (!m_i2s_out_buf_mh_que.push(data))
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return NULL;
    }

  return mh.getPa();
}

/*--------------------------------------------------------------------*/
void* SoundEffectObject::allocMfeOutBuf()
{
  MemMgrLite::MemHandle mh;

  if (mh.allocSeg(s_mfe_out_pool_id, MAX_MFE_OUT_PCM_BUF_SIZE) != ERR_OK)
    {
      SOUNDFX_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return NULL;
    }

  SoundFxBufParam data;

  data.mh = mh;
  data.sample = MFE_OUT_SAMPLE_NUM;
  data.is_end = false;

  if (!m_mfe_out_buf_mh_que.push(data))
    {
      SOUNDFX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
    }

  return mh.getPa();
}

