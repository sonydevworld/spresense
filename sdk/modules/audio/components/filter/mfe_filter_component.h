/****************************************************************************
 * modules/audio/components/filter/mfe_filter_component.h
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

#ifndef MFE_FILTER_COMPONENT_H
#define MFE_FILTER_COMPONENT_H

#include "memutils/os_utils/chateau_osal.h"

#include "wien2_common_defs.h"
#include "memutils/s_stl/queue.h"
#include "apus/apu_cmd.h"

#include "dsp_driver/include/dsp_drv.h"

#include "components/filter/mpp_filter_component.h" /* for MFE/SRC 1core */
#include "memutils/message/Message.h"
#include "components/common/component_common.h"

__WIEN2_BEGIN_NAMESPACE

#define MFE_COMMAND_QUEUE_SIZE (APU_COMMAND_QUEUE_SIZE*2)
/*--------------------------------------------------------------------*/
struct InitMFEParam
{
  int32_t  sample_num;      /* Sample number per processing unit */
  int32_t  proc_mode;       /* excution mode */
  int32_t  mic_channel_num; /* Channel number of input data */
  int32_t  ref_channel_num; /* Channel number of reference data */
  int32_t  sampling_rate;   /* sampling rate of input data */
  bool     use_aec;         /* Activate Echo Cancellr */
  bool     enable_mfe_aec;  /* Enable Echo Cancellr */
  uint32_t config_table;    /* MFE configuration table */
};

struct ExecMFEParam
{
  BufferHeader input_buffer;
  BufferHeader output_buffer;
  BufferHeader notification_buffer;
};

class MFEComponent : public ComponentCommon
{
private:
  /* Hold (push to queue) command which is processing in API. 
   * When APU process ends and receive reply, free (pop from queue) command.
   */

  s_std::Queue<Apu::Wien2ApuCmd*, MFE_COMMAND_QUEUE_SIZE> m_exec_queue;
  void send_apu(Apu::Wien2ApuCmd& cmd);

  int m_buf_idx;
  Apu::Wien2ApuCmd m_apu_cmd_buf[MFE_COMMAND_QUEUE_SIZE];
  void increment_buf_idx()
    {
      m_buf_idx = (m_buf_idx + 1) % MFE_COMMAND_QUEUE_SIZE;
    }

  typedef bool (*MFECompCallback)(DspDrvComPrm_t*);
  MFECompCallback m_callback;

  MsgQueId m_apu_dtq;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  DebugLogInfo m_debug_log_info;
#endif

public:
  MFEComponent(MsgQueId apu_dtq) : m_apu_dtq(apu_dtq), m_dsp_handler(NULL)
    {
      m_exec_queue.clear();
    }
  ~MFEComponent() {}

  uint32_t activate_apu(MFEComponent *p_component, uint32_t* dsp_inf);
  uint32_t activate_apu(MFEComponent *p_mfe_component,
                        MPPComponent *p_mpp_component,
                        uint32_t *dsp_inf);
  bool deactivate_apu();
  uint32_t init_apu(InitMFEParam param, uint32_t* dsp_inf);
  bool exec_apu(ExecMFEParam param);
  bool flush_apu();
  bool setparam_apu(Apu::ApuSetParamFilterCmd param);
  bool tuning_apu(Apu::ApuTuningFilterCmd param);

  bool setCallBack(MFECompCallback func) { m_callback = func; return true; };
  bool recv_apu(DspDrvComPrm_t*);

  void dsp_done_callback(DspDrvComPrm_t *p_param);
  MsgQueId get_apu_mid(void) { return m_apu_dtq; };

  void *m_dsp_handler;
};

__WIEN2_END_NAMESPACE

#endif /* MFE_FILTER_COMPONENT_H */

