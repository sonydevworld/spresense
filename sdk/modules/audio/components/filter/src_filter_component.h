/****************************************************************************
 * modules/audio/components/filter/src_filter_component.h
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

#ifndef SRC_FILTER_COMPONENT_H
#define SRC_FILTER_COMPONENT_H

#include "memutils/os_utils/chateau_osal.h"
#include "wien2_common_defs.h"
#include "memutils/s_stl/queue.h"
#include "apus/apu_cmd.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "dsp_driver/include/dsp_drv.h"
#include "components/common/component_common.h"
#include "debug/dbg_log.h"

#include "filter_component.h"


__WIEN2_BEGIN_NAMESPACE
using namespace MemMgrLite;

/*--------------------------------------------------------------------*/
/* Data structure definitions                                         */
/*--------------------------------------------------------------------*/

/* Init SRC Parameters */

struct InitSRCParam : public InitFilterParam
{
};

/* Exec SRC Parameters */

struct ExecSRCParam : public ExecFilterParam
{
};

/* Stop SRC Parameters */

struct StopSRCParam : public StopFilterParam
{
};

/* SRC Complete Reply Parameters */

struct SrcCmpltParam : public FilterCompCmpltParam
{
};

/*--------------------------------------------------------------------*/
/* Class definitions                                                  */
/*--------------------------------------------------------------------*/

class SRCComponent : public FilterComponent,
                     public ComponentCommon
{
private:

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  DebugLogInfo m_debug_log_info;
#endif

  s_std::Queue<MemMgrLite::MemHandle, APU_COMMAND_QUEUE_SIZE> m_apu_cmd_mh_que;

  MsgQueId m_apu_dtq;
  PoolId m_apu_pool_id;

  uint32_t init_apu(InitSRCParam *param, uint32_t *dsp_inf);
  bool exec_apu(ExecSRCParam *param);
  bool flush_apu(StopSRCParam *param);

  void send_apu(Apu::Wien2ApuCmd*);

  void* getApuCmdBuf()
  {
    MemMgrLite::MemHandle mh;

    if (mh.allocSeg(m_apu_pool_id, sizeof(Apu::Wien2ApuCmd)) != ERR_OK)
      {
        FILTER_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
        return NULL;
      }

    if (!m_apu_cmd_mh_que.push(mh))
      {
        FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
        return NULL;
      }

    return mh.getPa();
  }

  bool freeApuCmdBuf()
  {
    if (!m_apu_cmd_mh_que.pop())
      {
        FILTER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }

    return true;
  }

public:
  SRCComponent(MsgQueId apu_dtq,PoolId apu_pool_id)
    : m_apu_dtq(apu_dtq)
    , m_apu_pool_id(apu_pool_id)
    , m_dsp_handler(NULL) {}
  ~SRCComponent() {}

  virtual uint32_t activate_apu(const char *path, uint32_t *dsp_inf);
  virtual bool deactivate_apu();

  virtual uint32_t init_apu(InitFilterParam *param, uint32_t *dsp_inf)
  {
    return init_apu(static_cast<InitSRCParam *>(param), dsp_inf);
  }

  virtual bool exec_apu(ExecFilterParam *param)
  {
    return exec_apu(static_cast<ExecSRCParam *>(param));
  }

  virtual bool flush_apu(StopFilterParam *param)
  {
    return flush_apu(static_cast<StopSRCParam *>(param));
  }

  virtual bool setparam_apu(SetFilterParam *param) { return true; }
  virtual bool tuning_apu(TuningFilterParam *param) { return true; }

  virtual bool recv_done(void) { return freeApuCmdBuf(); };

  bool recv_apu(DspDrvComPrm_t*);
  MsgQueId get_apu_mid(void) { return m_apu_dtq; };

  void *m_dsp_handler;
};

__WIEN2_END_NAMESPACE

#endif /* SRC_FILTER_COMPONENT_H */

