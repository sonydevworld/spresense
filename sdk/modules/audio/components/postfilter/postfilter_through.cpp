/****************************************************************************
 * modules/audio/components/postfilter/postfilter_through.cpp
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

#include "postfilter_through.h"

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
uint32_t PostfilterThrough::init_apu(const InitPostfilterParam& param,
                                     uint32_t *dsp_inf)
{
  m_callback = param.callback;
  m_p_requester = param.p_requester;

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool PostfilterThrough::exec_apu(const ExecPostfilterParam& param)
{
  m_req_que.push(param.input);

  PostfilterCbParam cbpram;

  cbpram.event_type = Wien2::Apu::ExecEvent;

  m_callback(&cbpram, m_p_requester);

  return true;
}

/*--------------------------------------------------------------------*/
bool PostfilterThrough::flush_apu(const FlushPostfilterParam& param)
{
  AsPcmDataParam fls = { 0 };

  fls.mh       = param.output_mh;
  fls.is_valid = true;

  m_req_que.push(fls);

  PostfilterCbParam cbpram;

  cbpram.event_type = Wien2::Apu::FlushEvent;

  m_callback(&cbpram, m_p_requester);

  return true;
}

/*--------------------------------------------------------------------*/
bool PostfilterThrough::recv_done(PostfilterCmpltParam *cmplt)
{
  cmplt->output = m_req_que.top();
  cmplt->result = Wien2::Apu::ApuExecOK;

  m_req_que.pop();

  return true;
};

/*--------------------------------------------------------------------*/
uint32_t PostfilterThrough::activate(uint32_t *dsp_inf)
{
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool PostfilterThrough::deactivate(void)
{
  return true;
}

