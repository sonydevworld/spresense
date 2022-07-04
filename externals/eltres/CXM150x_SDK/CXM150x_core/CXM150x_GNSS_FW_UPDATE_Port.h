// ==========================================================================
/*!
* @file     CXM150x_GNSS_FW_UPDATE_Port.h
* @brief    Define HAL wrapper functions for GNSS FW UPDATE
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

#ifndef __CXM150x_GNSS_FW_UPDATE_PORT_H
#define __CXM150x_GNSS_FW_UPDATE_PORT_H

#include "CXM150x_APITypeDef.h"

// Compile only if certain symbols are defined in CXM150x_APITypedef.h
#if CXM150x_GNSS_FW_UPDATE_API_USE



//// transmission timeout time definition /////

// GNSS FW UPDATE mode UART communication timeout time
#define GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT              (1000)

// GNSS FW UPDATE mode Wait timeout time after mode change
#define GNSS_FW_UPDATE_MODE_CHANGE_TIME_OUT_TICK_COUNT      (10000)

// GNSS FW UPDATE mode Mode confirmation wait timeout time
#define GNSS_FW_UPDATE_MODE_CHECK_TIME_OUT_TICK_COUNT       (2000)

// GNSS FW UPDATE mode Response waiting timeout time after file transmission
#define GNSS_FW_UPDATE_SEND_FILE_TIME_OUT_TICK_COUNT        (60000)

// GNSS FW UPDATE mode Update result confirmation wait timeout time
#define GNSS_FW_UPDATE_RESULT_TIME_OUT_TICK_COUNT           (90000)

// GNSS block FW update mode header file transmission command
#define CXM150x_GNSS_FW_UPDATE_MODE_SEND_HEADER_CMD       "LOADH\r"
// GNSS block FW update mode code file transmission command
#define CXM150x_GNSS_FW_UPDATE_MODE_SEND_CODE_CMD         "LOADC\r"
// GNSS block FW update mode update execution command
#define CXM150x_GNSS_FW_UPDATE_MODE_EXEC_CMD              "EXEC\r"
// GNSS block FW update mode header file send command length
#define CXM150x_GNSS_FW_UPDATE_MODE_SEND_HEADER_CMD_LEN   (6)
// GNSS block FW update mode code file transmission command length
#define CXM150x_GNSS_FW_UPDATE_MODE_SEND_CODE_CMD_LEN     (6)
// GNSS block FW update mode update execution command length
#define CXM150x_GNSS_FW_UPDATE_MODE_EXEC_CMD_LEN          (5)

// GNSS block FW update mode header file send command length
#define CXM150x_GNSS_FW_UPDATE_MODE_CHECK_MESSAGE             "H>"

// GNSS block FW update mode header file send command response message
#define CXM150x_GNSS_FW_UPDATE_MODE_SEND_HEADER_CMD_RESPONSE  "LOADH"

// GNSS block FW update mode code file transmission command response message
#define CXM150x_GNSS_FW_UPDATE_MODE_SEND_CODE_CMD_RESPONSE    "LOADC"

// GNSS firmware update data transmission response message
#define CXM150x_GNSS_FW_UPDATE_MODE_DATA_SEND_RESPONSE        "OK"

// GNSS firmware update completion message
#define CXM150x_GNSS_FW_UPDATE_MODE_UPDATE_COMPLETE_MESSAGE   "success"

// ON / OFF definition of flag
#define CXM150x_GNSS_FW_UPDATE_API_FLAG_ON         (1)
#define CXM150x_GNSS_FW_UPDATE_API_FLAG_OFF        (0)

// prototype declaration
CXM150x_return_code wrapper_CXM150x_GNSS_fw_update_rx_message(uint8_t *rx_buf,uint32_t wait_cnt);
CXM150x_return_code wrapper_CXM150x_GNSS_fw_update_tx_message(uint8_t *snd_buf,uint32_t snd_cnt,uint32_t wait_cnt);
CXM150x_return_code wrapper_CXM150x_GNSS_fw_update_uart_abort_IT(void);

#endif // CXM150x_GNSS_FW_UPDATE_API_USE

#endif // __CXM150x_GNSS_FW_UPDATE_PORT_H

