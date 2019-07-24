/****************************************************************************
 * modules/audio/components/customproc/srcwrap_component.cpp
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

#include "srcwrap_component.h"

/* Hols instance pointer to call method from callback function. */

static SrcWrapComponent *s_p_self_instance;

/*--------------------------------------------------------------------*/
static bool src_done_callback(Wien2::FilterCompCmpltParam *param)
{
  s_p_self_instance->recv_apu(param);

  return true;
}

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
uint32_t SrcWrapComponent::init(const InitCustomProcParam& param)
{
  /* This function will not call method of SRCComponent. 
   * Because command format of SRC DSP is not opened to application.
   * Therefore, it will done by SDK internally.
   */

  /* Hold dummy cmplt */

  CustomProcCmpltParam cmplt;

  m_cmplt_que.push(cmplt);

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
uint32_t SrcWrapComponent::init_src(InitSrcParam& init_src_param)
{
  /* This function initializes SRC DSP via SRCComponent */

  uint32_t dsp_inf = 0;

  m_p_src_instance->init_apu(&(init_src_param.param), &dsp_inf);
  m_p_src_instance->recv_done();

  return 0;
}

/*--------------------------------------------------------------------*/
bool SrcWrapComponent::exec(const ExecCustomProcParam& param)
{
  /* Hold request information */

  ApuReqData req;

  req.input     = param.input;
  req.output_mh = param.output_mh;

  m_req_que.push(req);

  /* Exec SRC DSP via SRCComponent. */

  Wien2::ExecSRCParam exe_src_parm;

  exe_src_parm.in_buffer.size      = param.input.size;
  exe_src_parm.in_buffer.p_buffer  =
    static_cast<unsigned long *>(param.input.mh.getPa());
  exe_src_parm.out_buffer.size     = param.output_mh.getSize();
  exe_src_parm.out_buffer.p_buffer =
    static_cast<unsigned long *>(param.output_mh.getPa());

  bool rtcd = m_p_src_instance->exec_apu(
                static_cast<Wien2::ExecFilterParam *>(&exe_src_parm));
  
  return rtcd;
}

/*--------------------------------------------------------------------*/
bool SrcWrapComponent::flush(const FlushCustomProcParam& param)
{
  /* Hold request information */

  ApuReqData req;

  req.output_mh = param.output_mh;

  m_req_que.push(req);

  /* Flush SRC DSP vis SRCComponent. */

  Wien2::StopSRCParam flush_src_param;

  flush_src_param.out_buffer.size     = param.output_mh.getSize();
  flush_src_param.out_buffer.p_buffer =
    static_cast<unsigned long *>(param.output_mh.getPa());
  
  bool rtcd = m_p_src_instance->flush_apu(
                static_cast<Wien2::StopFilterParam *>(&flush_src_param));

  return rtcd;
}

/*--------------------------------------------------------------------*/
bool SrcWrapComponent::set(const SetCustomProcParam& param)
{
  /* This function will not call method of SRCComponent. 
   * Because proprietary SRC DSP doesn't have "Set" function.
   */

  /* Hold dummy cmplt */

  CustomProcCmpltParam cmplt;

  m_cmplt_que.push(cmplt);

  /* Call reply callback function */

  CustomProcCbParam cbpram;

  cbpram.event_type = CustomProcSet;

  m_callback(&cbpram, m_p_requester);

  return true;
}

/*--------------------------------------------------------------------*/
bool SrcWrapComponent::recv_apu(Wien2::FilterCompCmpltParam *srccmplt)
{
  /* SRCComponent replys output data addr and size now,
   * but this class cannot return to owner at this timing.
   * Therefor, store addr and size information to "cmplt que"
   * and return them when recv_done() of this class is called.
   */

  /* Enqueue cmplt queue and dequeue request que. */

  CustomProcCmpltParam cmplt;

  cmplt.output          = m_req_que.top().input;
  cmplt.output.mh       = m_req_que.top().output_mh;
  cmplt.output.size     = srccmplt->out_buffer.size;
  cmplt.output.is_valid = (srccmplt->result) ? true : false;
  cmplt.result          = srccmplt->result;

  m_cmplt_que.push(cmplt);

  m_req_que.pop();

  /* Notify done to SRC */

  m_p_src_instance->recv_done();

  /* Call reply callback function */

  CustomProcCbParam cbparam;

  switch (srccmplt->event_type)
    {
      case Wien2::InitEvent:
        cbparam.event_type = CustomProcInit;
        break;

      case Wien2::ExecEvent:
        cbparam.event_type = CustomProcExec;
        break;

      case Wien2::StopEvent:
        cbparam.event_type = CustomProcFlush;
        break;

      case Wien2::SetParamEvent:
        cbparam.event_type = CustomProcSet;
        break;

      default:
        break;
    }

  cbparam.result = srccmplt->result;

  return m_callback(&cbparam, m_p_requester);
}

/*--------------------------------------------------------------------*/
bool SrcWrapComponent::recv_done(CustomProcCmpltParam *cmplt)
{
  /* Reply stored completion information. */

  *cmplt = m_cmplt_que.top();

  m_cmplt_que.pop();

  return true;
}

/*--------------------------------------------------------------------*/
bool SrcWrapComponent::recv_done(CustomProcInformParam *info)
{
  memset(info, 0, sizeof(CustomProcInformParam));

  m_cmplt_que.pop();

  return true;
}


/*--------------------------------------------------------------------*/
bool SrcWrapComponent::recv_done(void)
{
  m_cmplt_que.pop();

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t SrcWrapComponent::activate(CustomProcCallback callback,
                                   const char *dsp_name,
                                   void *p_requester,
                                   uint32_t *dsp_inf)
{
  /* Set callback function to SRC */

  m_p_src_instance->setCallBack(src_done_callback);

  /* Activate SRC with full path (include file name). */

  m_p_src_instance->activate_apu(dsp_name, dsp_inf, true);

  m_p_requester = p_requester;
  m_callback = callback;

  /* Hold self instance */

  s_p_self_instance = this;

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool SrcWrapComponent::deactivate(void)
{
  m_p_src_instance->deactivate_apu();

  return true;
}

