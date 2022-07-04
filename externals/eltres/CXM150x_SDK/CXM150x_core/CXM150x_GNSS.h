// ==========================================================================
/*!
* @file     CXM150x_GNSS.h
* @brief    CXM150x control API (GNSS group command)
* @date     2021/08/16
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

#ifndef __CXM150x_GNSS_H
#define __CXM150x_GNSS_H

#include "CXM150x_APITypeDef.h"

CXM150x_return_code get_CXM150x_GNSS_firmware_version(void *param,CmdResGetCXM150xGNSSFirmwareVersion *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_GNSS_state_event(CXM150xGNSSState *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_GNSS_state_event(uint32_t on_off,CmdResSetCXM150xGNSSStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_GNSS_state_event(void *param,CmdResGetCXM150xGNSSStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_GNSS_state_event_info(void *param, CmdResGetCXM150xGnssStateEventInfo *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAGGA_event(CXM150xNMEAGGAInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAGLL_event(CXM150xNMEAGLLInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAGNS_event(CXM150xNMEAGNSInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAGSA_event(CXM150xNMEAGSAInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAGSV_event(CXM150xNMEAGSVInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEARMC_event(CXM150xNMEARMCInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAVTG_event(CXM150xNMEAVTGInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAZDA_event(CXM150xNMEAZDAInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAPSGES_event(CXM150xNMEAPSGESInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_NMEAPSLES_event(CXM150xNMEAPSLESInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_NMEA_event(uint32_t param, CmdResSetCXM150xNMEAEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_NMEA_event(void* param, CmdResGetCXM150xNMEAEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150xGNSSState conv_gnss_stt_message_to_code(uint8_t *msg);
CXM150x_return_code set_CXM150x_GNSS_position (CXM150xGNSSPositionSetData*param, CmdResSetCXM150xGNSSPosition *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_GNSS_datetime (uint8_t *param, CmdResSetCXM150xGNSSDateTime *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);

#endif // __CXM150x_GNSS_H
