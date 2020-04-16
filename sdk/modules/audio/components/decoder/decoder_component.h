/****************************************************************************
 * modules/audio/components/decoder/decoder_component.h
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

#ifndef _DECODER_COMPONENT_H_
#define _DECODER_COMPONENT_H_

#include "wien2_common_defs.h"
#include "apus/apu_cmd.h"
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "debug/dbg_log.h"
#include "components/component_base.h"

__WIEN2_BEGIN_NAMESPACE

#define FALT_HANDLE_ID 0xFF

typedef bool (*DecCompCallback)(void*, void*);

struct ActDecCompParam
{
  AudioCodec         codec;
  FAR char           *path;
  FAR void           **p_instance;
  MemMgrLite::PoolId apu_pool_id;
  MsgQueId           apu_mid;
  FAR uint32_t       *dsp_inf;
  bool               dsp_multi_core;
};

struct InitDecCompParam
{
  AudioCodec       codec_type;
  uint32_t         input_sampling_rate;
  AudioChannelNum  channel_num;
  AudioPcmFormat   bit_width;
  uint32_t         frame_sample_num;
  DecCompCallback  callback;
  void             *p_requester;
  BufferHeader     work_buffer;
  bool             dsp_multi_core;
};

struct ExecDecCompParam
{
  BufferHeader input_buffer;
  BufferHeader output_buffer;
  uint8_t      num_of_au;
  bool         is_valid_frame;
};

struct StopDecCompParam
{
  BufferHeader output_buffer;
  bool         is_valid_frame;
};

struct SetDecCompParam
{
  uint8_t l_gain; /**< audio level gain of L ch (0 - 200%) */
  uint8_t r_gain; /**< audio level gain of R ch (0 - 200%) */
};

struct DecCmpltParam
{
  Apu::ApuEventType event_type;

  union
  {
    InitDecCompParam init_dec_cmplt;
    ExecDecCompParam exec_dec_cmplt;
    StopDecCompParam stop_dec_cmplt;
    SetDecCompParam  setparam_dec_cmplt;
  };
};

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
struct DecDebugLogInfo
{
  DebugLogInfo info;
  uint32_t     namemap_ps;
};
#endif

extern "C" {

uint32_t AS_decode_init(const InitDecCompParam *param,
                        void *p_instance,
                        uint32_t *dsp_inf);

bool AS_decode_exec(const ExecDecCompParam *param, void *p_instance);

bool AS_decode_stop(const StopDecCompParam *param, void *p_instance);

bool AS_decode_setparam(const SetDecCompParam *param, void *p_instance);

bool AS_decode_recv_apu(void *p_param, void *p_instance);

bool AS_decode_recv_done(void *p_instance);

uint32_t AS_decode_activate(FAR ActDecCompParam *param);

bool AS_decode_deactivate(void *p_instance);

} /* extern "C" */


class DecoderComponent : public ComponentBase
{
public:
  DecoderComponent(MemMgrLite::PoolId apu_pool_id,MsgQueId apu_mid)
  {
    m_apu_pool_id = apu_pool_id;
    m_apu_mid = apu_mid;
    m_dsp_slave_handler = NULL;
  }
  ~DecoderComponent() {}

  uint32_t init_apu(const InitDecCompParam& param, uint32_t *dsp_inf);
  bool exec_apu(const ExecDecCompParam& param);
  bool flush_apu(const StopDecCompParam& param);
  bool setparam_apu(const SetDecCompParam& param);
  bool recv_apu(void *p_param);
  bool recv_done(void) { return freeApuCmdBuf(); };
  uint32_t activate(FAR ActDecCompParam *param);
  bool deactivate();
  MsgQueId get_apu_mid(void) { return m_apu_mid; };

  void *m_dsp_handler;
  void *m_dsp_slave_handler;

private:
  MemMgrLite::PoolId m_apu_pool_id;

  MsgQueId m_apu_mid;

  DecCompCallback m_callback;

  #define APU_DEC_QUEUE_SIZE 5

  typedef s_std::Queue<MemMgrLite::MemHandle, APU_DEC_QUEUE_SIZE> ApuCmdMhQue;
  ApuCmdMhQue m_apu_cmd_mh_que;

  void *m_p_requester;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  DecDebugLogInfo m_debug_log_info;
#endif

  uint8_t m_slave_cpu_id;

  void send_apu(Apu::Wien2ApuCmd*);

  void* getApuCmdBuf()
  {
    MemMgrLite::MemHandle mh;

    if (mh.allocSeg(m_apu_pool_id, sizeof(Apu::Wien2ApuCmd)) != ERR_OK)
      {
        DECODER_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
        return NULL;
      }

    if (!m_apu_cmd_mh_que.push(mh))
      {
        DECODER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
        return NULL;
      }

    return mh.getPa();
  }

  bool freeApuCmdBuf()
  {
    if (!m_apu_cmd_mh_que.pop())
      {
        DECODER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }

    return true;
  }
};

__WIEN2_END_NAMESPACE

#endif /* _DECODER_COMPONENT_H_ */

