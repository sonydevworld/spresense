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
#include "memutils/message/Message.h"
#include "dsp_driver/include/dsp_drv.h"
#include "components/common/component_common.h"

#include "filter_component.h"

__WIEN2_BEGIN_NAMESPACE

/*--------------------------------------------------------------------*/
/* Data structure definitions                                         */
/*--------------------------------------------------------------------*/

/* Init MPP(XLOUD) Parameters */

struct InitXLOUDParam : public InitFilterParam
{
  uint32_t                sample;
  Apu::AudioXloudMode     mode;
  void                    *p_xloud_coef_image;
  uint32_t                xloud_coef_size;
  void                    *p_eax_coef_image;
  uint32_t                eax_coef_size;
  void                    *p_sel_out_param;
};

/* Exec MPP(XLOUD) Parameters */

struct ExecXLOUDParam : public ExecFilterParam
{
};

/* Stop MPP(XLOUD) Parameters */

struct StopXLOUDParam : public StopFilterParam
{
};

/* Set MPP(XLOUD) Parameters */

struct SetXLOUDParam : public SetFilterParam
{
  int16_t  param_idx;  /**< Index of parameter */
  int16_t  xloud_vol;  /**< XLOUD volume */
};

/* Tuning MPP(XLOUD) Parameters */

struct TuningXLOUDParam : TuningFilterParam
{
  int16_t   param_idx;  /**< Index of parameter */
  uint32_t  xloud_config_table;  /**< XLOUD configuration address */
  uint32_t  xloud_param_table;   /**< XLOUD parameter table address */
  uint32_t  eax_config_table;    /**< EAX configuration address */
  uint32_t  eax_param_table;     /**< EAX parameter table address */
};

/* MPP(XLOUD) Complete Reply Parameters */

struct xLoudCmpltParam : public FilterCompCmpltParam
{
};

/*--------------------------------------------------------------------*/
/* Class definitions                                                  */
/*--------------------------------------------------------------------*/

class MPPComponent : public FilterComponent,
                     public ComponentCommon<Apu::InternalResult>
{
private:
  /* Hold (push to queue) command which is processing in API. 
   * When APU process ends and receive reply, free (pop from queue) command.
   */

  s_std::Queue<Apu::Wien2ApuCmd*, APU_COMMAND_QUEUE_SIZE> m_exec_queue;
  Apu::Wien2ApuCmd m_apu_cmd_buf[APU_COMMAND_QUEUE_SIZE];

  int m_buf_idx;
  MsgQueId m_apu_dtq;

  uint32_t init_apu(InitXLOUDParam *param, uint32_t* dsp_inf);
  bool exec_apu(ExecXLOUDParam *param);
  bool flush_apu();
  bool setparam_apu(SetXLOUDParam *param);
  bool tuning_apu(TuningXLOUDParam *param);

  void send_apu(Apu::Wien2ApuCmd& cmd);

public:
  MPPComponent(MsgQueId apu_dtq)
    : m_apu_dtq(apu_dtq)
    , m_dsp_handler(NULL)
  {
    m_exec_queue.clear();
  }
  ~MPPComponent() {}

  virtual uint32_t activate_apu(const char *path, uint32_t* dsp_inf);
  virtual bool deactivate_apu();

  virtual uint32_t init_apu(InitFilterParam *param, uint32_t* dsp_inf)
  {
    return init_apu(static_cast<InitXLOUDParam *>(param), dsp_inf);
  }

  virtual bool exec_apu(ExecFilterParam *param)
  {
    return exec_apu(static_cast<ExecXLOUDParam *>(param));
  }

  virtual bool flush_apu(StopFilterParam *param)
  {
    return flush_apu();
  }

  virtual bool setparam_apu(SetFilterParam *param)
  {
    return setparam_apu(static_cast<SetXLOUDParam *>(param));
  }

  virtual bool tuning_apu(TuningFilterParam *param)
  {
    return tuning_apu(static_cast<TuningXLOUDParam *>(param));
  }

  virtual bool recv_done(void) { return true; }

  bool recv_apu(DspDrvComPrm_t*);
  MsgQueId get_apu_mid(void) { return m_apu_dtq; };

  void *m_dsp_handler;
};

__WIEN2_END_NAMESPACE

#endif /* MPP_FILTER_COMPONENT_H */

