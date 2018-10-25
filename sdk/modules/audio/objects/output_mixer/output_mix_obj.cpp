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

static pid_t    s_omix_pid = -1;
static MsgQueId s_self_dtq;
static MsgQueId s_render_path0_filter_apu_dtq;
static MsgQueId s_render_path1_filter_apu_dtq;
static PoolId   s_render_path0_filter_pcm_pool_id;
static PoolId   s_render_path1_filter_pcm_pool_id;
static PoolId   s_render_path0_filter_apu_pool_id;
static PoolId   s_render_path1_filter_apu_pool_id;
static OutputMixObjectTask *s_omix_ojb = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

OutputMixObjectTask::OutputMixObjectTask(MsgQueId self_dtq,
                                         MsgQueId render_path0_filter_apu_dtq,
                                         MsgQueId render_path1_filter_apu_dtq,
                                         PoolId render_path0_filter_pcm_pool_id,
                                         PoolId render_path1_filter_pcm_pool_id,
                                         PoolId render_path0_filter_apu_pool_id,
                                         PoolId render_path1_filter_apu_pool_id) :
  m_self_dtq(self_dtq),
  m_output_device(HPOutputDevice)
  {
    for (int i = 0; i < HPI2SoutChNum; i++)
      {
        m_output_mix_to_hpi2s[i].set_self_dtq(self_dtq);
      }

    m_output_mix_to_hpi2s[0].set_apu_dtq(render_path0_filter_apu_dtq);
    m_output_mix_to_hpi2s[1].set_apu_dtq(render_path1_filter_apu_dtq);

    m_output_mix_to_hpi2s[0].set_pcm_pool_id(render_path0_filter_pcm_pool_id);
    m_output_mix_to_hpi2s[1].set_pcm_pool_id(render_path1_filter_pcm_pool_id);

    m_output_mix_to_hpi2s[0].set_apu_pool_id(render_path0_filter_apu_pool_id);
    m_output_mix_to_hpi2s[1].set_apu_pool_id(render_path1_filter_apu_pool_id);
  }

/*--------------------------------------------------------------------------*/
int OutputMixObjectTask::getHandle(MsgPacket* msg)
{
  int handle = 0;
  MsgType msgtype = msg->getType();

  switch (msgtype)
    {
      case MSG_AUD_MIX_CMD_ACT:
      case MSG_AUD_MIX_CMD_DEACT:
      case MSG_AUD_MIX_CMD_CLKRECOVERY:
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
void OutputMixObjectTask::run()
{
  err_t        err_code;
  MsgQueBlock* que;
  MsgPacket*   msg;

  err_code = MsgLib::referMsgQueBlock(m_self_dtq, &que);
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
  if (MSG_GET_CATEGORY(msg->getType()) == MSG_CAT_AUD_MIX)
    {
      if (msg->getType() == MSG_AUD_MIX_CMD_ACT)
        {
          m_output_device =
            static_cast<AsOutputMixDevice>
              (msg->peekParam<OutputMixerCommand>().act_param.output_device);
        }

      switch(m_output_device)
        {
          case HPOutputDevice:
          case I2SOutputDevice:
            m_output_mix_to_hpi2s[getHandle(msg)].parse(msg);
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
int AS_OutputMixObjEntry(int argc, char *argv[])
{
  OutputMixObjectTask::create(s_self_dtq,
                              s_render_path0_filter_apu_dtq,
                              s_render_path1_filter_apu_dtq,
                              s_render_path0_filter_pcm_pool_id,
                              s_render_path1_filter_pcm_pool_id,
                              s_render_path0_filter_apu_pool_id,
                              s_render_path1_filter_apu_pool_id);
  return 0;
}

/*--------------------------------------------------------------------------*/
bool AS_CreateOutputMixer(FAR AsCreateOutputMixParam_t *param)
{
  return AS_CreateOutputMixer(param, NULL);
}

/*--------------------------------------------------------------------------*/
bool AS_CreateOutputMixer(FAR AsCreateOutputMixParam_t *param, AudioAttentionCb attcb)
{
  /* Register attention callback */

  OUTPUT_MIX_REG_ATTCB(attcb);

  /* Parameter check */

  if (param == NULL)
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Create */

  s_self_dtq    = param->msgq_id.mixer;
  s_render_path0_filter_apu_dtq     = param->msgq_id.render_path0_filter_dsp;
  s_render_path1_filter_apu_dtq     = param->msgq_id.render_path1_filter_dsp;
  s_render_path0_filter_pcm_pool_id = param->pool_id.render_path0_filter_pcm;
  s_render_path1_filter_pcm_pool_id = param->pool_id.render_path1_filter_pcm;
  s_render_path0_filter_apu_pool_id = param->pool_id.render_path0_filter_dsp;
  s_render_path1_filter_apu_pool_id = param->pool_id.render_path1_filter_dsp;

  s_omix_pid = task_create("OMIX_OBJ",
                           150, 1024 * 3,
                           AS_OutputMixObjEntry,
                           NULL);
  if (s_omix_pid < 0)
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  return true;
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

  err_t er = MsgLib::send<OutputMixerCommand>(s_self_dtq,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_ACT,
                                              s_self_dtq,
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

  err_t er = MsgLib::send<AsPcmDataParam>(s_self_dtq,
                                          MsgPriNormal,
                                          MSG_AUD_MIX_CMD_DATA,
                                          s_self_dtq,
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

  err_t er = MsgLib::send<OutputMixerCommand>(s_self_dtq,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_CLKRECOVERY,
                                              s_self_dtq,
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

  err_t er = MsgLib::send<OutputMixerCommand>(s_self_dtq,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_DEACT,
                                              s_self_dtq,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeleteOutputMix(void)
{
  if (s_omix_pid < 0)
    {
      return false;
    }

  task_delete(s_omix_pid);

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
void OutputMixObjectTask::create(MsgQueId self_dtq,
                                 MsgQueId render_path0_filter_apu_dtq,
                                 MsgQueId render_path1_filter_apu_dtq,
                                 PoolId render_path0_filter_pcm_pool_id,
                                 PoolId render_path1_filter_pcm_pool_id,
                                 PoolId render_path0_filter_apu_pool_id,
                                 PoolId render_path1_filter_apu_pool_id)
{
  if (s_omix_ojb == NULL)
    {
      s_omix_ojb = new OutputMixObjectTask(self_dtq,
                                           render_path0_filter_apu_dtq,
                                           render_path1_filter_apu_dtq,
                                           render_path0_filter_pcm_pool_id,
                                           render_path1_filter_pcm_pool_id,
                                           render_path0_filter_apu_pool_id,
                                           render_path1_filter_apu_pool_id);
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


