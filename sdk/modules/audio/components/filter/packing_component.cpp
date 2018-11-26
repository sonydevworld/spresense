/****************************************************************************
 * modules/audio/components/filter/packing_component.cpp
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

#include "components/filter/packing_component.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

static PackingComponent *sp_packing_component = NULL;

/*--------------------------------------------------------------------*/
/* Methods of PackingComponent class */
/*--------------------------------------------------------------------*/
uint32_t PackingComponent::activate_apu(PackingComponent *p_component)
{
  FILTER_DBG("ACT BITCNV:\n");

  sp_packing_component = p_component;

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool PackingComponent::deactivate_apu(void)
{
  FILTER_DBG("DEACT BITCNV:\n");

  sp_packing_component = NULL;

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t PackingComponent::init_apu(InitPackingParam param)
{
  FILTER_DBG("INIT BITCNV: in bitwidth %d, out bitwidth %d\n",
             param.in_bitwidth, param.out_bitwidth);

  m_in_bitwidth = param.in_bitwidth;
  m_out_bitwidth = param.out_bitwidth;

  //notify_reply(Apu::InitEvent, true);

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool PackingComponent::exec_apu(ExecPackingParam param)
{
  void (PackingComponent::*convfunc)(uint32_t, int8_t *, int8_t *);
  uint32_t outsize = 0;
  bool result = false;

  if ((m_in_bitwidth == BitWidth32bit) && (m_out_bitwidth == BitWidth24bit))
    {
      convfunc = &PackingComponent::cnv32to24;
      outsize  = param.in_buffer.size * BitWidth24bit / BitWidth32bit;
    }
  else if ((m_in_bitwidth == BitWidth24bit) && (m_out_bitwidth == BitWidth32bit))
    {
      convfunc = &PackingComponent::cnv24to32;
      outsize  = param.in_buffer.size * BitWidth32bit / BitWidth24bit;
    }
  else
    {
      return false;
    }

  /* Excec convert */

  if (outsize <= param.out_buffer.size)
    {
      (this->*convfunc)(param.in_buffer.size / (m_in_bitwidth / 8),
                        reinterpret_cast<int8_t *>(param.in_buffer.p_buffer),
                        reinterpret_cast<int8_t *>(param.out_buffer.p_buffer));

      param.out_buffer.size = outsize;

      result = true;
    }
 
  notify_reply(Apu::ExecEvent, result, param.out_buffer);

  return true;
}

/*--------------------------------------------------------------------*/
bool PackingComponent::flush_apu(StopPackingParam param)
{
  FILTER_DBG("FLUSH BITCNV:\n");

  param.out_buffer.size = 0;

  notify_reply(Apu::FlushEvent, true, param.out_buffer);

  return true;
}

/*--------------------------------------------------------------------*/
void PackingComponent::cnv32to24(uint32_t samples, int8_t *in, int8_t *out)
{
  /** For Little endian data ! **/

  uint32_t *p_in  = (uint32_t *)in;
  uint32_t *p_out = (uint32_t *)out;

  for (uint32_t cnt = 0; cnt < samples / 4; cnt++)
    {
      *(p_out+0) = (uint32_t)(((*(p_in+0) & 0xFFFFFF00) >> 8 ) + ((*(p_in+1) & 0x0000FF00) << 16));
      *(p_out+1) = (uint32_t)(((*(p_in+1) & 0xFFFF0000) >> 16) + ((*(p_in+2) & 0x00FFFF00) << 8 ));
      *(p_out+2) = (uint32_t)(((*(p_in+2) & 0xFF000000) >> 24) + ((*(p_in+3) & 0xFFFFFF00) >> 0 ));

      p_out +=3;
      p_in  +=4;
    }
}

/*--------------------------------------------------------------------*/
void PackingComponent::cnv24to32(uint32_t samples, int8_t *in, int8_t *out)
{
  /** For Little endian data ! **/

  uint32_t *p_in  = (uint32_t *)in;
  uint32_t *p_out = (uint32_t *)out;

  for (uint32_t cnt = 0; cnt < samples / 4; cnt++)
    {
      *(p_out+0) = (uint32_t)( (*(p_in+0) & 0x00FFFFFF) << 8 );
      *(p_out+1) = (uint32_t)(((*(p_in+0) & 0xFF000000) >> 16) + ((*(p_in+1) & 0x0000FFFF) << 16));
      *(p_out+2) = (uint32_t)(((*(p_in+1) & 0xFFFF0000) >> 8 ) + ((*(p_in+2) & 0x000000FF) << 24));
      *(p_out+3) = (uint32_t)(  *(p_in+2) & 0xFFFFFF00);

      p_out +=4;
      p_in  +=3;
    }
}

/*--------------------------------------------------------------------*/
void PackingComponent::notify_reply(uint8_t evt, bool result, BufferHeader outbuf)
{
  Apu::Wien2ApuCmd packet;

  packet.header.event_type  = evt;
  packet.result.exec_result = (result) ? Apu::ApuExecOK : Apu::ApuExecError;

  if (evt == Apu::ExecEvent)
    {
      packet.exec_filter_cmd.output_buffer = outbuf;
    }
  else
    {
      packet.flush_filter_cmd.flush_src_cmd.output_buffer = outbuf;
    }

  DspDrvComPrm_t param;

  param.process_mode = Apu::FilterMode;
  param.event_type   = evt;
  param.type         = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  param.data.pParam  = static_cast<void *>(&packet);

  m_callback(&param);
}


__WIEN2_END_NAMESPACE

