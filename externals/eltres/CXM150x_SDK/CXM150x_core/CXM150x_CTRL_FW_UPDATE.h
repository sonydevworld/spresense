// ==========================================================================
/*!
* @file     CXM150x_CTRL_FW_UPDATE.h
* @brief    CXM150x control API (for CONTROL FW UPDATE mode)
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

#ifndef __CXM150x_CTRL_FW_UPDATE_H
#define __CXM150x_CTRL_FW_UPDATE_H

#include "CXM150x_APITypeDef.h"

#if CXM150x_CTRL_FW_UPDATE_API_USE

CXM150x_return_code get_CXM150x_ctrl_fw_update_routine_name(void *param, CmdResGetCtrlFWUpdateRoutineName *res);
CXM150x_return_code get_CXM150x_ctrl_fw_update_version(void* param, CmdResGetCtrlFWUpdateVersion *res);
CXM150x_return_code get_CXM150x_ctrl_fw_update_company_name(void* param, CmdResGetCtrlFWUpdateCompanyName *res);
CXM150x_return_code get_CXM150x_ctrl_fw_update_device_name(void* param, CmdResGetCtrlFWUpdateDeviceName *res);
CXM150x_return_code get_CXM150x_ctrl_fw_update_state(uint32_t param, CmdResGetCtrlFWUpdateState *res);
CXM150x_return_code set_CXM150x_ctrl_fw_update_end_request(CXM150xFWUpdateType param, CmdResSetCtrlFWUpdateEndRequest *res);
CXM150x_return_code set_CXM150x_ctrl_fw_update_data(CXM150xFWUpdateSetData* param, CmdResSetCtrlFWUpdateData *res);
CXM150x_return_code set_CXM150x_ctrl_fw_update_power (CXM150xFWUpdateSetPower *param, CmdResSetCtrlFWUpdatePower *res);

#endif // CXM150x_CTRL_FW_UPDATE_API_USE


#endif // __CXM150x_CTRL_FW_UPDATE_H
