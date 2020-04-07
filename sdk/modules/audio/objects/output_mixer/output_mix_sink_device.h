/****************************************************************************
 * modules/audio/objects/output_mixer/output_mix_sink_device.h
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

#ifndef __MODULES_AUDIO_OBJECTS_OUTPUT_OUTPUT_MIXER_OUTPUT_MIX_SINK_DEVICE_H
#define __MODULES_AUDIO_OBJECTS_OUTPUT_OUTPUT_MIXER_OUTPUT_MIX_SINK_DEVICE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/s_stl/queue.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/common_utils/common_assert.h"
#include "audio/audio_message_types.h"
#include "audio/audio_outputmix_api.h"
#include "audio/audio_object_common_api.h"
#include "wien2_internal_packet.h"
#include "wien2_common_defs.h"
#include "components/renderer/renderer_component.h"
#include "components/customproc/usercustom_component.h"
#include "components/customproc/thruproc_component.h"
#include "objects/stream_parser/ram_lpcm_data_source.h"
#include "objects/stream_parser/mp3_stream_mng.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**< Parameters for post done notify to output-mix object. */

struct OutputMixObjPostfilterDoneCmd
{
  ComponentEventType event_type;
  bool                result;
};

/**< Parameters for render done notify to output-mix object. */

struct OutputMixObjRenderDoneCmd
{
  bool end_flag;
  bool error_flag;
};

/**< OutputMixer self internal message structure */

struct OutputMixObjParam
{
  int handle;

  union
  {
    OutputMixObjPostfilterDoneCmd postfilterdone_param;
    OutputMixObjRenderDoneCmd     renderdone_param;
  };
};

class OutputMixToHPI2S
{
public:
  OutputMixToHPI2S(MsgQueId self_msgq_id,
                   MsgQueId dsp_msgq_id,
                   MemMgrLite::PoolId cmd_pool,
                   MemMgrLite::PoolId pcm_pool)
    : m_self_msgq_id(self_msgq_id)
    , m_apu_msgq_id(dsp_msgq_id)
    , m_apu_pool_id(cmd_pool)
    , m_pcm_pool_id(pcm_pool)
    , m_state(AS_MODULE_ID_OUTPUT_MIX_OBJ, "", Booted)
    , m_p_postfliter_instance(NULL)
    , m_usercstm_instance(cmd_pool, dsp_msgq_id)
    , m_thruproc_instance()
    , m_postproc_type(AsPostprocTypeInvalid)
    , m_callback(NULL)
    , m_adjust_direction(OutputMixNoAdjust)
    , m_adjustment_times(0)
  {
    memset(m_dsp_path, 0, sizeof(m_dsp_path));
  }

  ~OutputMixToHPI2S()
  {}

    MsgQueId m_self_msgq_id, m_requester_msgq_id, m_apu_msgq_id;
    MemMgrLite::PoolId m_apu_pool_id;
    MemMgrLite::PoolId m_pcm_pool_id;
    int m_self_handle;

  void parse(MsgPacket* msg);

  bool is_active(void)
    {
      return ((m_state.get() != Booted) ? true : false);
    }

private:
  enum State
  {
    Booted = 0,
    Ready,
    Active,
    Stopping,
    Underflow,
    StateNum
  };

  AudioState<State> m_state;

  typedef void (OutputMixToHPI2S::*MsgProc)(MsgPacket *);
  static MsgProc MsgProcTbl[AUD_MIX_MSG_NUM][StateNum];
  static MsgProc MsgRsltTbl[AUD_MIX_RST_MSG_NUM][StateNum];

  typedef s_std::Queue<AsPcmDataParam, 10> RenderDataQueue;
  RenderDataQueue m_render_data_queue;

  ComponentBase *m_p_postfliter_instance;
  UserCustomComponent m_usercstm_instance;
  ThruProcComponent m_thruproc_instance;
  AsPostprocType m_postproc_type;
  char m_dsp_path[AS_POSTPROC_FILE_PATH_LEN];

  RenderComponentHandler m_render_comp_handler;

  OutputMixerCallback m_callback;
  OutputMixerErrorCallback m_error_callback;

  int8_t m_adjust_direction;
  int32_t m_adjustment_times;

  uint32_t m_max_pcm_buff_size;
  uint32_t m_apucmd_pcm_buff_size;

  void reply(MsgQueId requester_msgq_id,
             MsgType msg_type,
             AsOutputMixDoneParam *done_param);

  void illegal(MsgPacket *msg);
  void act(MsgPacket *msg);
  void init(MsgPacket *msg);
  uint32_t loadComponent(AsPostprocType type, char *dsp_path);
  uint32_t unloadComponent(void);
  void deact(MsgPacket *msg);

  void input_data_on_ready(MsgPacket *msg);
  void input_data_on_active(MsgPacket *msg);
  void input_data_on_under(MsgPacket *msg);

  void postdone_on_ready(MsgPacket *msg);
  void postdone_on_active(MsgPacket *msg);
  void postdone_on_stopping(MsgPacket *msg);
  void postdone_on_underflow(MsgPacket *msg);

  void illegal_done(MsgPacket *msg);
  void done_on_active(MsgPacket *msg);
  void done_on_stopping(MsgPacket *msg);

  void clock_recovery(MsgPacket *msg);

  void init_postproc(MsgPacket* msg);
  void set_postproc(MsgPacket* msg);

  void parseOutputMixRst(MsgPacket *msg);

  int8_t get_period_adjustment(void);
  bool checkMemPool(void);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_OUTPUT_OUTPUT_MIXER_OUTPUT_MIX_SINK_DEVICE_H */
