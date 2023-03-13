// ==========================================================================
/*!
* @file     CXM150x_SYS.h
* @brief    API for CXM150x control (SYS group command)
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

#ifndef __CXM150x_SYS_H
#define __CXM150x_SYS_H

#include "CXM150x_APITypeDef.h"

CXM150x_return_code get_CXM150x_firmware_version(void *param,CmdResGetCXM150xFirmwareVersion *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_power(uint32_t param, CmdResSetCXM150xPower *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_power(void *param,CmdResGetCXM150xPower *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_mode(uint32_t mode,CmdResSetCXM150xMode *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_mode(void *param,CmdResGetCXM150xMode *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_uart_start_interrupt(void *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_sys_state_event(CXM150xSysState *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_sys_state_event(uint32_t on_off,CmdResSetCXM150xSysStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_sys_state_event(void *param,CmdResGetCXM150xSysStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_sys_state_event_info(void *param,CmdResGetCXM150xSysStateEventInfo *info,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_fetching_time(void *param, CmdResSetCXM150xFetchingTime *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_wait_fetching_time(void *param, CmdResSetCXM150xWaitFetchingTime *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code reset_CXM150x(void *param,CmdResResetCXM150x *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_reset_event_info(void *param, CmdResGetCXM150xResetEventInfo *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_EEPROM_data(uint32_t param, CmdResGetCXM150xEEPROMData *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_EEPROM_data(CXM150xEEPROMSetData*  param, CmdResSetCXM150xEEPROMData *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_Bootloader_version(void *param,CmdResGetCXM150xBootloaderVersion *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_FATAL_message_event(CXM150xFATALMessage *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_event_buffer_overflow(CXM150xEventBufferOverflow *info,CXM150x_CALLBACK_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_api_version(void *param,CmdResGetCXM150xAPIVersion *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_GNSS_backup(void *param, CmdResSetCXM150xGNSSBackup *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_EEPROM_data_sequential(void* param, CmdResGetCXM150xEEPROMDataSequential *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_sys_to_deep_sleep (uint32_t param, CmdResSetCXM150xSysToDeepSleep *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_sys_to_deep_sleep_event(CXM150xSysToDeepSleepInfo *info,CXM150x_CALLBACK_FUNC_POINTER func);

CXM150xSysState conv_sys_stt_message_to_code(uint8_t *msg);
void analyse_CXM150x_Rx(void);

#endif // __CXM150x_SYS_H
