/****************************************************************************
 * modules/audio/components/capture/capture_component.h
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

#ifndef CAPTURE_COMPONENT_H
#define CAPTURE_COMPONENT_H

#include <nuttx/arch.h>

#include "wien2_common_defs.h"
#include "wien2_internal_packet.h"
#include "audio_state.h"

#include "memutils/os_utils/chateau_osal.h"

#include "memutils/s_stl/queue.h"

#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"

#include "audio/audio_capture_api.h"
#include "audio/audio_message_types.h"

#include "dma_controller/audio_dma_drv_api.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

#define PRE_REQ_QUE_NUM     (8)
#define MAX_CAPTURE_QUE_NUM (9)
#define MAX_AC_IN_CH_NUM    (4)
#define MAX_I2S_IN_CH_NUM   (2)

#ifndef CONFIG_AUDIOUTILS_CAPTURE_CH_NUM
#define CONFIG_AUDIOUTILS_CAPTURE_CH_NUM 2
#endif
/* Equals to Max number of DMAC resource */
#define MAX_CAPTURE_COMP_INSTANCE_NUM  CONFIG_AUDIOUTILS_CAPTURE_CH_NUM

#define MAX_CAPTURE_MIC_CH  CXD56_AUDIO_MIC_CH_MAX

/* General types */

struct CaptureBuffer
{
  MemMgrLite::MemHandle cap_mh;
  uint32_t              sample; 
  bool                  validity;
};

enum CaptureDevice
{
  CaptureDeviceAnalogMic = 0,
  CaptureDeviceDigitalMic,
  CaptureDeviceI2S,
  CaptureDeviceTypeNum
};

enum CaptureError
{
  CaptureErrorDMAunder = 0,
  CaptureErrorErrInt,
  CaptureErrorBusErr,
  CaptureErrorTypeNum
};

struct CaptureDataParam
{
  CaptureDevice output_device;
  CaptureBuffer buf;
  bool          end_flag;
};

typedef void (* CaptureDoneCB)(CaptureDataParam p_param);

struct CaptureErrorParam
{
  CaptureError error_type;
};

typedef void (* CaptureErrorCB)(CaptureErrorParam p_param);

/* API paramters */

struct ExecCaptureComponentParam
{
  int32_t pcm_sample;
};

struct InitCaptureComponentParam
{
  uint8_t          capture_ch_num;
  uint8_t          capture_bit_width;
  uint8_t          preset_num;
  CaptureDoneCB    callback;
  CaptureErrorCB   err_callback;
};

struct ActCaptureComponentParam
{
  cxd56_audio_dma_path_t  dma_path_id;
  CaptureDevice           output_device;
  MemMgrLite::PoolId      mem_pool_id;
};

struct StopCaptureComponentParam
{
  asDmacStopMode mode;
};

typedef uint32_t CaptureComponentHandler;

enum NotifyType
{
  NtfDmaCmplt = 0,
  NotifyTypeNum
};

struct NotifyCaptureComponentParam
{
  NotifyType   type;
  E_AS_DMA_INT code;
};

struct SetMicGainCaptureComponentParam
{
  int16_t mic_gain[CXD56_AUDIO_MIC_CH_MAX];
};

struct CaptureComponentParam
{
  CaptureComponentHandler handle;

  union
  {
    ActCaptureComponentParam    act_param;
    InitCaptureComponentParam   init_param;
    ExecCaptureComponentParam   exec_param;
    StopCaptureComponentParam   stop_param;
    NotifyCaptureComponentParam notify_param;
    FAR SetMicGainCaptureComponentParam *set_micgain_param;
  };
};

/* API definitions */

extern "C" {

bool AS_get_capture_comp_handler(CaptureComponentHandler *p_handle,
                                 CaptureDevice device_type,
                                 MemMgrLite::PoolId mem_pool_id);

bool AS_release_capture_comp_handler(CaptureComponentHandler p_handle);

bool AS_init_capture(const CaptureComponentParam *param);

bool AS_exec_capture(const CaptureComponentParam *param);

bool AS_stop_capture(const CaptureComponentParam *param);

bool AS_set_micgain_capture(FAR const CaptureComponentParam *param);
} /* extern "C" */

class CaptureComponent
{
public:
  CaptureComponent()
    : m_dmac_id(CXD56_AUDIO_DMAC_MIC)
    , m_output_device(CaptureDeviceTypeNum)
    , m_ch_num(CONFIG_AUDIOUTILS_CAPTURE_CH_NUM)
    , m_preset_num(0)
    , m_state(AS_MODULE_ID_CAPTURE_CMP, "", Booted)
  {}

  ~CaptureComponent() {}

  cxd56_audio_dma_t m_dmac_id;

  CaptureDoneCB m_callback;
  CaptureErrorCB m_err_callback;

  CaptureDevice m_output_device;

  typedef s_std::Queue<CaptureBuffer, MAX_CAPTURE_QUE_NUM> CapDataQue;
  CapDataQue m_req_data_que;

  void create(AS_DmaDoneCb, AS_ErrorCb, MsgQueId, MsgQueId);

private:
  enum State
  {
    Booted = 0,
    Ready,
    PreAct,
    Act,
    Error,
    StateNum
  };

  AS_ErrorCb   m_dma_err_cb;
  AS_DmaDoneCb m_dma_done_cb;

  MsgQueId m_self_dtq;
  MsgQueId m_self_sync_dtq;

  MemMgrLite::PoolId m_mem_pool_id;

  uint8_t m_ch_num;

  uint8_t m_preset_num;

  AudioState<State> m_state;

  typedef s_std::Queue<CaptureComponentParam, PRE_REQ_QUE_NUM> ReadDmacCmdQue;
  ReadDmacCmdQue m_cap_pre_que;

  typedef bool (CaptureComponent::*EvtProc)(const CaptureComponentParam&);
  static EvtProc EvetProcTbl[AUD_CAP_MSG_NUM][StateNum];

  void run();
  bool parse(MsgPacket *msg);

  bool act(const CaptureComponentParam& param);
  bool deact(const CaptureComponentParam& param);
  bool illegal(const CaptureComponentParam& param);
  bool init(const CaptureComponentParam& param);
  bool execOnRdy(const CaptureComponentParam& param);
  bool execOnPreAct(const CaptureComponentParam& param);
  bool execOnAct(const CaptureComponentParam& param);
  bool execOnError(const CaptureComponentParam& param);
  bool stopOnReady(const CaptureComponentParam& param);
  bool stopOnPreAct(const CaptureComponentParam& param);
  bool stopOnAct(const CaptureComponentParam& param);
  bool stopOnError(const CaptureComponentParam& param);
  bool setMicGain(const CaptureComponentParam& param);
  bool notify(const CaptureComponentParam& param);

  CaptureBuffer getCapBuf(uint32_t cap_sample);
  bool enqueDmaReqQue(CaptureBuffer buf);
};


__WIEN2_END_NAMESPACE

#endif /* CAPTURE_COMPONENT_H */

