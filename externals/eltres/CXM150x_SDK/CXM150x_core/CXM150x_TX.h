// ==========================================================================
/*!
* @file     CXM150x_TX.h
* @brief    CXM150x control API (TX group command)
* @date     2021/12/27
*
* Copyright 2021, 2022 Sony Semiconductor Solutions Corporation
* 
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
* 
* 3. Neither the name of Sony Semiconductor Solutions Corporation nor the names of
* its contributors may be used to endorse or promote products derived from this
* software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
// =========================================================================

#ifndef __CXM150x_TX_H
#define __CXM150x_TX_H

#include "CXM150x_APITypeDef.h"

CXM150x_return_code set_CXM150x_LPWA_payload(uint8_t *payload,CmdResSetCXM150xLPWAPayload *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_LPWA_payload(void *param,CmdResGetCXM150xLPWAPayload *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_prev_LPWA_tx_data(void *param,CmdResGetCXM150xPrevLPWATxData *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_tx_state_event(CXM150xTxState *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_tx_state_event(uint32_t on_off,CmdResSetCXM150xTxStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_tx_state_event(void *param,CmdResGetCXM150xTxStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_prev_LPWA_tx_time(void *param,CmdResGetCXM150xPrevLPWATxTime *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_next_LPWA_tx_time(void *param,CmdResGetCXM150xNextLPWATxTime *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_tx_profile(uint32_t param, CmdResSetCXM150xTxProfile *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_tx_profile(void* param, CmdResGetCXM150xTxProfile *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150xTxState conv_tx_stt_message_to_code(uint8_t *msg);
CXM150x_return_code set_CXM150x_tx_PoC_enable(void *param, CmdResSetCXM150xTxPoCEnable *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_tx_PoC_enable_message_event(CXM150xTxPoCEnableMessage *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_tx_payload_event(CXM150xTxPayloadInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_tx_duty_event(uint32_t on_off, CmdResSetCXM150xTxDutyEvent *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_tx_duty_event(void *param,CmdResGetCXM150xTxDutyEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_tx_duty_event(CXM150xTxDutyEventInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);

#endif // __CXM150x_TX_H
