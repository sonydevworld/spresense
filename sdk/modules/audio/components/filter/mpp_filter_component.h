/****************************************************************************
 * modules/audio/components/filter/mpp_filter_component.h
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

#ifndef MPP_FILTER_COMPONENT_H
#define MPP_FILTER_COMPONENT_H

#include "memutils/os_utils/chateau_osal.h"

#include "wien2_common_defs.h"
#include "memutils/s_stl/queue.h"
#include "apus/apu_cmd.h"

#include "dsp_driver/include/dsp_drv.h"
#include "memutils/message/Message.h"
#include "components/common/component_common.h"

__WIEN2_BEGIN_NAMESPACE

/*--------------------------------------------------------------------*/
struct InitXLOUDParam
{
  uint8_t                 channel_num;
  uint32_t                sample;
  uint32_t                input_sampling_rate;
  Apu::AudioXloudMode     mode;
  Apu::AudioPcmFormatType in_pcm_bit_len;
  Apu::AudioPcmFormatType out_pcm_bit_len;
  void                    *p_xloud_coef_image;
  uint32_t                xloud_coef_size;
  void                    *p_eax_coef_image;
  uint32_t                eax_coef_size;
  void                    *p_sel_out_param;
};

/*--------------------------------------------------------------------*/
struct ExecXLOUDParam
{
  BufferHeader input_buffer;
  BufferHeader output_buffer;
};

/*--------------------------------------------------------------------*/
class MPPComponent : public ComponentCommon
{
private:
  /* Hold (push to queue) command which is processing in API. 
   * When APU process ends and receive reply, free (pop from queue) command.
   */

  s_std::Queue<Apu::Wien2ApuCmd*, APU_COMMAND_QUEUE_SIZE> m_exec_queue;
  int m_buf_idx;
  Apu::Wien2ApuCmd m_apu_cmd_buf[APU_COMMAND_QUEUE_SIZE];

  typedef bool (*MppCompCallback)(DspDrvComPrm_t*);
  MppCompCallback m_callback;

  MsgQueId m_apu_dtq;

public:
  MPPComponent(MsgQueId apu_dtq)
    : m_apu_dtq(apu_dtq)
    , m_dsp_handler(NULL)
  {
    m_exec_queue.clear();
  }
  ~MPPComponent() {}

  uint32_t activate_apu(MPPComponent *p_component, const char *path, uint32_t* dsp_inf);
  bool deactivate_apu();
  uint32_t init_apu(InitXLOUDParam param, uint32_t* dsp_inf);
  bool exec_apu(ExecXLOUDParam param);
  bool flush_apu();
  bool setparam_apu(Apu::ApuSetParamFilterCmd param);
  bool tuning_apu(Apu::ApuTuningFilterCmd param);

  bool setCallBack(MppCompCallback func) { m_callback = func; return true; };
  bool recv_apu(DspDrvComPrm_t*);
  MsgQueId get_apu_mid(void) { return m_apu_dtq; };

  void* m_dsp_handler;

private:
  void send_apu(Apu::Wien2ApuCmd& cmd);
};

__WIEN2_END_NAMESPACE

#endif /* MPP_FILTER_COMPONENT_H */

