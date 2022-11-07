// ==========================================================================
/*!
* @file     CXM150x_TEST.h
* @brief    CXM150x control API (TEST group command)
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

#ifndef __CXM150x_TEST_H
#define __CXM150x_TEST_H

#include "CXM150x_APITypeDef.h"

// Compile only if certain symbols are defined in CXM150x_APITypedef.h
#if CXM150x_TEST_MODE_API_USE

CXM150x_return_code set_CXM150x_test_tx_ch(uint32_t param, CmdResSetCXM150xTestTxCh *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_test_tx_ch(void* param, CmdResGetCXM150xTestTxCh *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_test_tx_mode(uint32_t mode, CmdResSetCXM150xTestTxMode *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_test_tx_mode(void* param, CmdResGetCXM150xTestTxMode *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_test_tx_run(uint32_t on_off, CmdResSetCXM150xTestTxRun *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_test_tx_run(void* param, CmdResGetCXM150xTestTxRun *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_test_gpi_state(uint32_t param, CmdResGetCXM150xTestGPIState *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code set_CXM150x_test_gpo_state(CXM150xSetGPOState *param, CmdResSetCXM150xTestGPOState *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);
CXM150x_return_code get_CXM150x_test_gpo_state(uint32_t param, CmdResGetCXM150xTestGPOState *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func);

#endif  // CXM150x_TEST_MODE_API_USE

#endif // __CXM150x_TEST_H
