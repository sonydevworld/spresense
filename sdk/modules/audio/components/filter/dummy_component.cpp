/****************************************************************************
 * modules/audio/components/filter/dummy_component.cpp
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

#include "components/filter/dummy_component.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

/*--------------------------------------------------------------------*/
/* Methods of DummyComponent class */
/*--------------------------------------------------------------------*/
uint32_t DummyComponent::activate_apu(const char *path, uint32_t *dsp_inf)
{
  FILTER_DBG("ACT DUMMY:\n");

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool DummyComponent::deactivate_apu(void)
{
  FILTER_DBG("DEACT DUMMY:\n");

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t DummyComponent::init_apu(InitDummyParam *param, uint32_t *dsp_inf)
{
  FILTER_DBG("INIT DUMMY:\n");

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool DummyComponent::exec_apu(ExecDummyParam *param)
{
  memcpy(param->out_buffer.p_buffer,
         param->in_buffer.p_buffer,
         param->in_buffer.size);

  param->out_buffer.size = param->in_buffer.size;

  send_resp(ExecEvent, true, param->out_buffer);

  return true;
}

/*--------------------------------------------------------------------*/
bool DummyComponent::flush_apu(StopDummyParam *param)
{
  FILTER_DBG("FLUSH DUMMY:\n");

  param->out_buffer.size = 0;

  send_resp(StopEvent, true, param->out_buffer);

  return true;
}

/*--------------------------------------------------------------------*/
void DummyComponent::send_resp(FilterComponentEvent evt, bool result, BufferHeader outbuf)
{
  DummyCmpltParam cmplt;

  cmplt.filter_type = Dummy;
  cmplt.event_type  = evt;
  cmplt.result      = result;
  cmplt.out_buffer  = outbuf;

  m_callback(&cmplt);
}


__WIEN2_END_NAMESPACE

