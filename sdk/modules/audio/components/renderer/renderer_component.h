/****************************************************************************
 * modules/audio/components/renderer/renderer_component.h
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

#ifndef RENDERER_COMPONENT_H
#define RENDERER_COMPONENT_H

#include "wien2_common_defs.h"
#include "audio_state.h"

#include "dma_controller/audio_dma_drv_api.h"
#include "memutils/s_stl/queue.h"
#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"

#include "debug/dbg_log.h"
#include "memutils/common_utils/common_assert.h"
#include "audio/audio_renderer_api.h"
#include "audio/audio_message_types.h"

__WIEN2_BEGIN_NAMESPACE


typedef uint32_t RenderComponentHandler;
typedef void (*RenderDoneCB)(AudioDrvDmaResult *p_param, void *p_requester);
typedef void (*RenderErrorCB)(AudioDrvDmaError *p_param, void *p_requester);

enum RenderDevice
{
  RenderDeviceHPSP = 0,
  RenderDeviceHPSPSoundEffect,
  RenderDeviceI2S,
  RenderDeviceTypeNum
};

bool AS_get_render_comp_handler(RenderComponentHandler *p_handle,
                                RenderDevice device_type);

bool AS_release_render_comp_handler(RenderComponentHandler handle);

bool AS_init_renderer(RenderComponentHandler handle,
                      RenderDoneCB callback,
                      RenderErrorCB err_callback,
                      void *p_requester,
                      uint8_t bit_length);

bool AS_exec_renderer(RenderComponentHandler handle,
                      void *addr,
                      uint32_t sample,
                      bool is_valid);

bool AS_stop_renderer(RenderComponentHandler handle,
                      asDmacStopMode mode);

/*--------------------------------------------------------------------*/
class RendererComponent
{
public:
  RendererComponent()
    : m_dmac_id(CXD56_AUDIO_DMAC_I2S0_DOWN)
    , m_state(AS_MODULE_ID_RENDERER_CMP, "", Booted)
  {}

  ~RendererComponent() {}

  enum NotifyType
  {
    NtfDmaCmplt = 0,
    NotifyTypeNum
  };

  struct ActRenderCompParam
  {
    cxd56_audio_dma_path_t  dma_path_id;
    cxd56_audio_signal_t    sig_id;
    cxd56_audio_sel_t       sel_info;
  };

  struct InitRenderCompParam
  {
    cxd56_audio_samp_fmt_t format;
    RenderDoneCB           callback;
    RenderErrorCB err_callback;
    void                   *p_requester;
  };

  struct ExecRenderCompParam
  {
    void     *addr;
    uint32_t sample;
    bool     is_valid;
  };

  struct StopRenderCompParam
  {
    asDmacStopMode mode;
  };

  struct NotifyRenderCompParam
  {
    NotifyType   type;
    E_AS_DMA_INT code;
  };

  struct RendererComponentParam
  {
    union
    {
      ActRenderCompParam    act_render_param;
      InitRenderCompParam   init_render_param;
      ExecRenderCompParam   exec_render_param;
      StopRenderCompParam   stop_render_param;
      NotifyRenderCompParam notify_render_param;
    };
  };

  cxd56_audio_dma_t m_dmac_id;

  void *m_p_requester;

  typedef void (*RendererDoneCb)(AudioDrvDmaResult *pParam,
                                 void *p_requester);
  RendererDoneCb m_callback;

  typedef void (*RendererErrorCb)(AudioDrvDmaError *pParam,
                                 void *p_requester);
  RendererErrorCb m_err_callback;

  void create(AS_DmaDoneCb, AS_ErrorCb, MsgQueId, MsgQueId);

private:
  enum State
  {
    Booted = 0,
    Ready,
    PreAct,
    Act,
    StateNum
  };

  AS_ErrorCb   m_dma_err_cb;
  AS_DmaDoneCb m_dma_done_cb;

  MsgQueId m_self_dtq;
  MsgQueId m_self_sync_dtq;

  AudioState<State> m_state;

  typedef s_std::Queue<asWriteDmacParam, 2> WriteDmacCmdQue;
  WriteDmacCmdQue m_write_dmac_cmd_que;

  typedef bool (RendererComponent::*EvtProc)(const RendererComponentParam&);
  static EvtProc EvetProcTbl[AUD_BB_MSG_NUM][StateNum];

  void run();
  bool parse(MsgPacket *msg);
  bool illegal(const RendererComponentParam& param);

  bool act(const RendererComponentParam& param);
  bool deact(const RendererComponentParam& param);
  bool init(const RendererComponentParam& param);

  bool runOnRdy(const RendererComponentParam& param);
  bool runOnPreAct(const RendererComponentParam& param);
  bool runOnAct(const RendererComponentParam& param);

  bool stopOnPreAct(const RendererComponentParam& param);
  bool stopOnAct(const RendererComponentParam& param);

  bool notify(const RendererComponentParam& param);
};

__WIEN2_END_NAMESPACE

#endif /* RENDERER_COMPONENT_H */

