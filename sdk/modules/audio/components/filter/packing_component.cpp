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

__WIEN2_BEGIN_NAMESPACE

/*--------------------------------------------------------------------*/
/* Methods of PackingComponent class */
/*--------------------------------------------------------------------*/
uint32_t PackingComponent::activate(ComponentCallback callback,
                                    const char *image_name,
                                    void *p_requester,
                                    uint32_t *dsp_inf)
{
  FILTER_DBG("ACT BITCNV:\n");

  m_p_requester = p_requester;
  m_callback = callback;

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool PackingComponent::deactivate(void)
{
  FILTER_DBG("DEACT BITCNV:\n");

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t PackingComponent::init(const InitComponentParam& param)
{
  FILTER_DBG("INIT BITCNV: in bytewidth %d, out bytewidth %d\n",
             param.common.in_bitlength, param.common.out_bitlength);

  m_in_bitwidth  = param.common.in_bitlength;
  m_out_bitwidth = param.common.out_bitlength;

  /* Hold dummy. */

  AsPcmDataParam dummy;

  if (!m_req_que.push(dummy))
    {
      return AS_ECODE_QUEUE_OPERATION_ERROR;
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool PackingComponent::set(const SetComponentParam& param)
{
  /* Hold dummy */

  AsPcmDataParam dummy;

  if (!m_req_que.push(dummy))
    {
      return false;
    }

  /* Call reply callback function */
  
  send_resp(ComponentSet, true);

  return true;
}

/*--------------------------------------------------------------------*/
bool PackingComponent::exec(const ExecComponentParam& param)
{
  void (PackingComponent::*convfunc)(uint32_t, int8_t *, int8_t *);
  uint32_t outsize = 0;
  bool result = false;

  /* Filter data area check */

  if ((param.input.mh.getPa() == NULL)
   || (param.output.getPa() == NULL))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return false;
    }

  /* Execute packing */

  if ((m_in_bitwidth == BitWidth32bit) && (m_out_bitwidth == BitWidth24bit))
    {
      convfunc = &PackingComponent::cnv32to24;
      outsize  = param.input.size * BitWidth24bit / BitWidth32bit;
    }
  else if ((m_in_bitwidth == BitWidth24bit) && (m_out_bitwidth == BitWidth32bit))
    {
      convfunc = &PackingComponent::cnv24to32;
      outsize  = param.input.size * BitWidth32bit / BitWidth24bit;
    }
  else
    {
      return false;
    }

  /* Excec convert */

  if (outsize <= param.output.getSize())
    {
      (this->*convfunc)(param.input.size / (m_in_bitwidth / 8),
                        reinterpret_cast<int8_t *>(param.input.mh.getPa()),
                        reinterpret_cast<int8_t *>(param.output.getPa()));
      result = true;
    }
 
  /* Hold result */

  AsPcmDataParam output = param.input;

  output.mh       = param.output;
  output.size     = outsize;
  output.is_valid = result;

  if (!m_req_que.push(output))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  /* Send response */

  send_resp(ComponentExec, result);

  return true;
}

/*--------------------------------------------------------------------*/
bool PackingComponent::flush(const FlushComponentParam& param)
{
  FILTER_DBG("FLUSH BITCNV:\n");

  /* Hold result */

  AsPcmDataParam output;

  output.mh       = param.output;
  output.size     = 0;
  output.is_valid = true;

  if (!m_req_que.push(output))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return false;
    }

  /* Send response */

  send_resp(ComponentFlush, true);

  return true;
}

/*--------------------------------------------------------------------*/
bool PackingComponent::recv_done(ComponentCmpltParam *cmplt)
{
  /* Set output pcm parameters (even if is not there) */

  cmplt->output          = m_req_que.top();
  cmplt->output.is_valid = true;

  /* Set result */

  cmplt->result = cmplt->output.is_valid;

  if (!m_req_que.pop())
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return false;
    }

  return true;
}


/*--------------------------------------------------------------------*/
bool PackingComponent::recv_done(ComponentInformParam *info)
{
  return recv_done();
}

/*--------------------------------------------------------------------*/
bool PackingComponent::recv_done(void)
{
  if (!m_req_que.pop())
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return false;
    }

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
void PackingComponent::send_resp(ComponentEventType evt, bool result)
{
  ComponentCbParam cbpram;

  cbpram.event_type = evt;
  cbpram.result     = result;

  m_callback(&cbpram, m_p_requester);
}


__WIEN2_END_NAMESPACE

