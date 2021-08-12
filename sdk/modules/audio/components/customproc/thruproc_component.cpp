/****************************************************************************
 * modules/audio/components/customproc/thruproc_component.cpp
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

#include "thruproc_component.h"

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
uint32_t ThruProcComponent::init(const InitComponentParam& param)
{
  m_req_que.alloc();
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool ThruProcComponent::exec(const ExecComponentParam& param)
{
  m_req_que.alloc(param.input, param.input.mh);

  ComponentCbParam cbpram;
  cbpram.event_type = ComponentExec;
  cbpram.result     = true;

  m_callback(&cbpram, m_p_requester);

  return true;
}

/*--------------------------------------------------------------------*/
bool ThruProcComponent::flush(const FlushComponentParam& param)
{
  AsPcmDataParam fls = { 0 };

  fls.mh       = param.output;
  fls.is_valid = false;

  m_req_que.alloc(fls);

  ComponentCbParam cbpram;
  cbpram.event_type = ComponentFlush;
  cbpram.result     = true;

  m_callback(&cbpram, m_p_requester);

  return true;
}

/*--------------------------------------------------------------------*/
bool ThruProcComponent::set(const SetComponentParam& param)
{
  m_req_que.alloc();

  ComponentCbParam cbpram;
  cbpram.result     = true;
  cbpram.event_type = ComponentSet;

  m_callback(&cbpram, m_p_requester);

  return true;
}

/*--------------------------------------------------------------------*/
bool ThruProcComponent::recv_done(ComponentCmpltParam *cmplt)
{
  cmplt->output = m_req_que.top_input();
  cmplt->result = true;

  return m_req_que.free();
}

/*--------------------------------------------------------------------*/
bool ThruProcComponent::recv_done(ComponentInformParam *info)
{
  memset(info, 0, sizeof(ComponentInformParam));

  return m_req_que.free();
}

/*--------------------------------------------------------------------*/
bool ThruProcComponent::recv_done(void)
{
  return m_req_que.free();
}

/*--------------------------------------------------------------------*/
uint32_t ThruProcComponent::activate(ComponentCallback callback,
                                   const char *dsp_name,
                                   void *p_requester,
                                   uint32_t *dsp_inf)
{
  m_p_requester = p_requester;
  m_callback = callback;

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool ThruProcComponent::deactivate(void)
{
  return true;
}

