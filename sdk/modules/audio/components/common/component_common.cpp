/****************************************************************************
 * modules/audio/components/common/component_common.cpp
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

#include "component_common.h"

#define DBG_MODULE DBG_MODULE_AS

__WIEN2_BEGIN_NAMESPACE

/*--------------------------------------------------------------------*/
bool ComponentCommon::dsp_boot_check(MsgQueId dsp_dtq,
                                     uint32_t *dsp_inf)
{
  err_t        err_code;
  MsgQueBlock  *que;
  MsgPacket    *msg;

  err_code = MsgLib::referMsgQueBlock(dsp_dtq, &que);
  F_ASSERT(err_code == ERR_OK);

  err_code = que->recv(TIME_FOREVER, &msg);
  F_ASSERT(err_code == ERR_OK);
  F_ASSERT(msg->getType() == MSG_ISR_APU0);

  uint32_t dsp_version = msg->moveParam<uint32_t>();
  err_code = que->pop();
  F_ASSERT(err_code == ERR_OK);

  /* Reply DSP version */

  *dsp_inf = dsp_version;

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t ComponentCommon::dsp_init_check(MsgQueId dsp_dtq, uint32_t *dsp_inf)
{
  err_t        err_code;
  MsgQueBlock  *que;
  MsgPacket    *msg;

  err_code = MsgLib::referMsgQueBlock(dsp_dtq, &que);
  F_ASSERT(err_code == ERR_OK);

  err_code = que->recv(TIME_FOREVER, &msg);
  F_ASSERT(err_code == ERR_OK);
  F_ASSERT(msg->getType() == MSG_ISR_APU0);

  DspResult rst = msg->moveParam<DspResult>();
  err_code = que->pop();
  F_ASSERT(err_code == ERR_OK);

  if (rst.exec_result != Apu::ApuExecOK)
    {
      if (rst.internal_result.res_src == Apu::FromLib)
        {
          *dsp_inf = rst.internal_result.value;
          return AS_ECODE_DECODER_LIB_INITIALIZE_ERROR;
        }
      else
        {
          return rst.internal_result.code;
        }
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
void ComponentCommon::dsp_init_complete(MsgQueId dsp_dtq,
                                        Apu::Wien2ApuCmd *packet)
{
  DspResult dsp_result;

  dsp_result.exec_result     = packet->result.exec_result;
  dsp_result.internal_result = packet->result.internal_result[0];

  err_t er = MsgLib::send<DspResult>(dsp_dtq,
                                     MsgPriNormal,
                                     MSG_ISR_APU0,
                                     0,
                                     dsp_result);
  F_ASSERT(er == ERR_OK);
}

__WIEN2_END_NAMESPACE

