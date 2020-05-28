/****************************************************************************
 * modules/audio/objects/output_mixer/output_mix_obj.cpp
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

#include <string.h>
#include <stdlib.h>
#include <nuttx/arch.h>
#include "output_mix_obj.h"
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

static pthread_t s_omix_pid = -1;
static AsOutputMixMsgQueId_t s_msgq_id;
static AsOutputMixPoolId_t   s_pool_id;
static OutputMixObjectTask *s_omix_ojb = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

OutputMixObjectTask::OutputMixObjectTask(AsOutputMixMsgQueId_t msgq_id,
                                         AsOutputMixPoolId_t pool_id)
  : m_msgq_id(msgq_id)
  , m_output_mix_to_hpi2s_0(msgq_id.mixer,
                            msgq_id.render_path0_filter_dsp,
                            pool_id.render_path0_filter_dsp,
                            pool_id.render_path0_filter_pcm)
  , m_output_mix_to_hpi2s_1(msgq_id.mixer,
                            msgq_id.render_path1_filter_dsp,
                            pool_id.render_path1_filter_dsp,
                            pool_id.render_path1_filter_pcm)
  , m_output_device(HPOutputDevice)
{}

/*--------------------------------------------------------------------------*/
int OutputMixObjectTask::getHandle(MsgPacket* msg)
{
  int handle = 0;
  MsgType msgtype = msg->getType();

  switch (msgtype)
    {
      case MSG_AUD_MIX_CMD_ACT:
      case MSG_AUD_MIX_CMD_INIT:
      case MSG_AUD_MIX_CMD_DEACT:
      case MSG_AUD_MIX_CMD_CLKRECOVERY:
      case MSG_AUD_MIX_CMD_INITMPP:
      case MSG_AUD_MIX_CMD_SETMPP:
        handle = msg->peekParam<OutputMixerCommand>().handle;
        break;

      case MSG_AUD_MIX_CMD_DATA:
        handle = msg->peekParam<AsPcmDataParam>().identifier;
        break;

      default:
        handle = msg->peekParam<OutputMixObjParam>().handle;
        break;
    }

  return handle;
}

/*--------------------------------------------------------------------------*/
void OutputMixObjectTask::enableOutputDevice()
{
  CXD56_AUDIO_ECODE error_code;

  switch (m_output_device)
    {
      case A2dpSrcOutputDevice:
        cxd56_audio_dis_i2s_io();
        error_code = cxd56_audio_dis_output();
        break;

      case HPOutputDevice:
        cxd56_audio_dis_i2s_io();
        cxd56_audio_set_spout(true);
        error_code = cxd56_audio_en_output();
        break;

      case I2SOutputDevice:
        {
          cxd56_audio_en_i2s_io();
          cxd56_audio_set_spout(false);
          error_code = cxd56_audio_en_output();
          if (error_code != CXD56_AUDIO_ECODE_OK)
            {
              break;
            }

          /* Set Path, Mixer to I2S0 */

          cxd56_audio_signal_t sig_id = CXD56_AUDIO_SIG_MIX;
          cxd56_audio_sel_t    sel_info;
          sel_info.au_dat_sel1 = false;
          sel_info.au_dat_sel2 = false;
          sel_info.cod_insel2  = false;
          sel_info.cod_insel3  = false;
          sel_info.src1in_sel  = true;
          sel_info.src2in_sel  = false;

          error_code = cxd56_audio_set_datapath(sig_id, sel_info);
        }
        break;

      default:
        OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return;;
    }

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_CODE_FATAL);
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixObjectTask::disableOutputDevice()
{
  CXD56_AUDIO_ECODE error_code;

  switch (m_output_device)
    {
      case A2dpSrcOutputDevice:
        /* Do nothing */

        error_code = CXD56_AUDIO_ECODE_OK;
        break;

      case HPOutputDevice:
        cxd56_audio_set_spout(false);
        error_code = cxd56_audio_dis_output();
        break;

      case I2SOutputDevice:
        {
          cxd56_audio_dis_i2s_io();
          error_code = cxd56_audio_dis_output();
          if (error_code != CXD56_AUDIO_ECODE_OK)
            {
              break;
            }
        }
        break;

      default:
        OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return;
    }

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_CODE_FATAL);
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixObjectTask::run()
{
  err_t        err_code;
  MsgQueBlock* que;
  MsgPacket*   msg;

  err_code = MsgLib::referMsgQueBlock(m_msgq_id.mixer, &que);
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

/*--------------------------------------------------------------------------*/
void OutputMixObjectTask::parse(MsgPacket* msg)
{
  MsgType msg_type = msg->getType();

  if (MSG_GET_CATEGORY(msg_type) == MSG_CAT_AUD_MIX)
    {
      switch(msg_type)
        {
          case MSG_AUD_MIX_CMD_ACT:
            {
              m_output_device = static_cast<AsOutputMixDevice>
                (msg->peekParam<OutputMixerCommand>().act_param.output_device);

              /* Enable output device */

              enableOutputDevice();
            }
            break;

          case MSG_AUD_MIX_CMD_DEACT:
            {
              /* Disable output device */

              disableOutputDevice();
            }
            break;

          default:
            /* Do nothing */

            break;
        }

      switch(m_output_device)
        {
          case HPOutputDevice:
          case I2SOutputDevice:
            if (getHandle(msg) == 0)
              {
                m_output_mix_to_hpi2s_0.parse(msg);
              }
            else
              {
                m_output_mix_to_hpi2s_1.parse(msg);
              }
            break;

          case A2dpSrcOutputDevice:
            break;

          default:
            OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
            break;
        }
    }
  else
    {
      F_ASSERT(0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*--------------------------------------------------------------------------*/
FAR void AS_OutputMixObjEntry(FAR void *arg)
{
  OutputMixObjectTask::create(s_msgq_id, s_pool_id);
}

/*--------------------------------------------------------------------------*/
static bool CreateOutputMixer(AsOutputMixMsgQueId_t msgq_id, AsOutputMixPoolId_t pool_id, AudioAttentionCb attcb)
{
  /* Register attention callback */

  OUTPUT_MIX_REG_ATTCB(attcb);

  /* Create */

  s_msgq_id = msgq_id;
  s_pool_id = pool_id;

  /* Reset Message queue. */

  FAR MsgQueBlock *que;
  err_t err_code = MsgLib::referMsgQueBlock(s_msgq_id.mixer, &que);
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

  int ret = pthread_create(&s_omix_pid,
                           &attr,
                           (pthread_startroutine_t)AS_OutputMixObjEntry,
                           (pthread_addr_t)NULL);
  if (ret < 0)
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  pthread_setname_np(s_omix_pid, "out_mixer");

  return true;
}

/*
 * The following tree functions are Old functions for compatibility.
 */

static bool CreateOutputMixer(AsOutputMixMsgQueId_t msgq_id, AsOutputMixPoolId_old_t pool_id, AudioAttentionCb attcb)
{
  AsOutputMixPoolId_t tmp;
  tmp.render_path0_filter_pcm.sec = tmp.render_path1_filter_pcm.sec = 0;
  tmp.render_path0_filter_dsp.sec = tmp.render_path1_filter_dsp.sec = 0;
  tmp.render_path0_filter_pcm.pool = pool_id.render_path0_filter_pcm;
  tmp.render_path1_filter_pcm.pool = pool_id.render_path1_filter_pcm;
  tmp.render_path0_filter_dsp.pool = pool_id.render_path0_filter_dsp;
  tmp.render_path1_filter_dsp.pool = pool_id.render_path1_filter_dsp;

  return CreateOutputMixer(msgq_id, tmp, attcb);
}

/*--------------------------------------------------------------------------*/
bool AS_CreateOutputMixer(FAR AsCreateOutputMixParam_t *param)
{
  return AS_CreateOutputMixer(param, NULL);
}

/*--------------------------------------------------------------------------*/
bool AS_CreateOutputMixer(FAR AsCreateOutputMixParam_t *param, AudioAttentionCb attcb)
{
  /* Parameter check */

  if (param == NULL)
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  return CreateOutputMixer(param->msgq_id, param->pool_id, attcb);
}

/*
 * The following two functions are New functions for multi-section memory layout.
 */

bool AS_CreateOutputMixer(FAR AsCreateOutputMixParams_t *param)
{
  return AS_CreateOutputMixer(param, NULL);
}

/*--------------------------------------------------------------------------*/
bool AS_CreateOutputMixer(FAR AsCreateOutputMixParams_t *param, AudioAttentionCb attcb)
{
  /* Parameter check */

  if (param == NULL)
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  return CreateOutputMixer(param->msgq_id, param->pool_id, attcb);
}
  
/*--------------------------------------------------------------------------*/
bool AS_ActivateOutputMixer(uint8_t handle, FAR AsActivateOutputMixer *actparam)
{
  /* Parameter check */

  if (actparam == NULL)
    {
      return false;
    }

  /* Activate */

  OutputMixerCommand cmd;

  cmd.handle    = handle;
  cmd.act_param = *actparam;

  err_t er = MsgLib::send<OutputMixerCommand>(s_msgq_id.mixer,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_ACT,
                                              s_msgq_id.mng,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_InitOutputMixer(uint8_t handle, FAR AsInitOutputMixer *initparam)
{
  /* Parameter check */

  if (initparam == NULL)
    {
      return false;
    }

  /* Set init param */

  OutputMixerCommand cmd;

  cmd.handle     = handle;
  cmd.init_param = *initparam;

  err_t er = MsgLib::send<OutputMixerCommand>(s_msgq_id.mixer,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_INIT,
                                              s_msgq_id.mng,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_SendDataOutputMixer(FAR AsSendDataOutputMixer *sendparam)
{
  /* Parameter check */

  if (sendparam == NULL)
    {
      return false;
    }

  /* Send data, reload parameters */

  sendparam->pcm.identifier = sendparam->handle;
  sendparam->pcm.callback   = sendparam->callback;

  err_t er = MsgLib::send<AsPcmDataParam>(s_msgq_id.mixer,
                                          MsgPriNormal,
                                          MSG_AUD_MIX_CMD_DATA,
                                          s_msgq_id.mng,
                                          sendparam->pcm);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_FrameTermFineControlOutputMixer(uint8_t handle, FAR AsFrameTermFineControl *ftermparam)
{
  /* Parameter check */

  if (ftermparam == NULL)
    {
      return false;
    }

  /* Set frame term */

  OutputMixerCommand cmd;

  cmd.handle      = handle;
  cmd.fterm_param = *ftermparam;

  err_t er = MsgLib::send<OutputMixerCommand>(s_msgq_id.mixer,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_CLKRECOVERY,
                                              s_msgq_id.mng,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_InitPostprocOutputMixer(uint8_t handle, FAR AsInitPostProc *initppparam)
{
  /* Parameter check */

  if (initppparam == NULL)
    {
      return false;
    }

  /* Set Postfilter command param */

  OutputMixerCommand cmd;

  cmd.handle       = handle;
  cmd.initpp_param = *initppparam;

  err_t er = MsgLib::send<OutputMixerCommand>(s_msgq_id.mixer,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_INITMPP,
                                              s_msgq_id.mng,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_SetPostprocOutputMixer(uint8_t handle, FAR AsSetPostProc *setppparam)
{
  /* Parameter check */

  if (setppparam == NULL)
    {
      return false;
    }

  /* Set Postfilter command param */

  OutputMixerCommand cmd;

  cmd.handle       = handle;
  cmd.setpp_param = *setppparam;

  err_t er = MsgLib::send<OutputMixerCommand>(s_msgq_id.mixer,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_SETMPP,
                                              s_msgq_id.mng,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivateOutputMixer(uint8_t handle, FAR AsDeactivateOutputMixer *deactparam)
{
  /* Parameter check */

  if (deactparam == NULL)
    {
      return false;
    }

  /* Deactivate */

  OutputMixerCommand cmd;

  cmd.handle      = handle;
  cmd.deact_param = *deactparam;

  err_t er = MsgLib::send<OutputMixerCommand>(s_msgq_id.mixer,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_DEACT,
                                              s_msgq_id.mng,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeleteOutputMix(void)
{
  if (s_omix_pid == INVALID_PROCESS_ID)
    {
      return false;
    }

  pthread_cancel(s_omix_pid);
  pthread_join(s_omix_pid, NULL);

  s_omix_pid = INVALID_PROCESS_ID;

  if (s_omix_ojb != NULL)
    {
      delete s_omix_ojb;
      s_omix_ojb = NULL;

      /* Unregister attention callback */

      OUTPUT_MIX_UNREG_ATTCB();
    }
  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_checkAvailabilityOutputMixer(void)
{
  return (s_omix_ojb != NULL);
}

/*--------------------------------------------------------------------------*/
void OutputMixObjectTask::create(AsOutputMixMsgQueId_t msgq_id,
                                 AsOutputMixPoolId_t pool_id)
{
  if (s_omix_ojb == NULL)
    {
      s_omix_ojb = new OutputMixObjectTask(msgq_id, pool_id);
      if (s_omix_ojb == NULL)
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
          return;
        }
      s_omix_ojb->run();
    }
  else
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
    }
}
