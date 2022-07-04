// ==========================================================================
/*!
* @file     CXM150x_CTRL_FW_UPDATE_Port.h
* @brief    Define HAL wrapper functions for CONTROL FW UPDATE
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

#ifndef __CXM150x_CTRL_FW_UPDATE_PORT_H
#define __CXM150x_CTRL_FW_UPDATE_PORT_H

#include "CXM150x_APITypeDef.h"

// Compile only if certain symbols are defined in CXM150x_APITypedef.h
#if CXM150x_CTRL_FW_UPDATE_API_USE



//// transmission timeout time definition /////

// CONTROL FW UPDATE mode UART communication timeout time
#define CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT                    (1000)

// CONTROL FW UPDATE mode Firmware binary image data transmission response wait timeout time
#define CTRL_FW_UPDATE_SEND_FW_DATA_MAX_TIME_OUT_TICK_COUNT       (1000)

// CONTROL FW UPDATE mode Current status acquisition response wait timeout time
#define CTRL_FW_UPDATE_GET_UPDATE_STATE_MAX_TIME_OUT_TICK_COUNT   (100)

// CONTROL FW UPDATE mode After the Firmware Update End Request, "| SYS RESET POR_PIN" or "| SYS UPDATE OK" wait timeout time
#define CTRL_FW_UPDATE_GET_UPDATE_SYS_RESET_TIME_OUT_TICK_COUNT   (15000)


//// Command transmission interval definition /////

// Interval from timeout after retrieving current status of firmware update routine to start retry
#define CTRL_FW_UPDATE_GET_UPDATE_STATE_WAIT_TICK_COUNT           (5000)


// message format definition
// Offset to each information
#define CXM150x_CTRL_FW_UPDATE_API_POS_SD0         (0)
#define CXM150x_CTRL_FW_UPDATE_API_POS_LE0         (1)
#define CXM150x_CTRL_FW_UPDATE_API_POS_LE1         (2)
#define CXM150x_CTRL_FW_UPDATE_API_POS_SD1         (3)
#define CXM150x_CTRL_FW_UPDATE_API_POS_RB          (4)
#define CXM150x_CTRL_FW_UPDATE_API_POS_NUL         (5)
#define CXM150x_CTRL_FW_UPDATE_API_POS_CMD         (6)
#define CXM150x_CTRL_FW_UPDATE_API_POS_DATA        (7)

// header 7 bytes + sum value 1 byte + terminator 1 byte
#define CXM150x_CTRL_FW_UPDATE_API_MSG_INF_LEN     (9)

// Data start offset of first firmware binary image data transmission D_SD (1 byte) + D_LE [0: 3] (4 bytes)
#define CXM150x_CTRL_FW_UPDATE_API_FIRST_DATA_OFFSET   (5)

// Command transmission retry count definition
#define CXM150x_CTRL_FW_UPDATE_API_STATE_RETRY_NUM (1)
#define CXM150x_CTRL_FW_UPDATE_API_DATA_RETRY_NUM  (1)

// message constant definition
// Constants defined in the specification
#define CXM150x_CTRL_FW_UPDATE_API_SD                  (0x68)
#define CXM150x_CTRL_FW_UPDATE_API_RB_FROM_HOST        (0x00)
#define CXM150x_CTRL_FW_UPDATE_API_RB_FROM_CXM150x   (0x80)
#define CXM150x_CTRL_FW_UPDATE_API_NUL                 (0x00)
#define CXM150x_CTRL_FW_UPDATE_API_ED                  (0x16)
#define CXM150x_CTRL_FW_UPDATE_API_D_SD                (0x03)
#define CXM150x_CTRL_FW_UPDATE_API_D_ACK               (0x90)

// command constant
// FW data transmission or other command (constant set in CMD section)
#define CXM150x_CTRL_FW_UPDATE_API_CMD_SEND_CMD        (0x40)
#define CXM150x_CTRL_FW_UPDATE_API_CMD_SEND_FW_DATA    (0x41)

// size of DATA section of command
#define CXM150x_CTRL_FW_UPDATE_API_CMD_DATA_SIZE       (1)

// Definition of request data type
#define CXM150x_CTRL_FW_UPDATE_API_DATA_ROUTINE_NAME   (0x04)
#define CXM150x_CTRL_FW_UPDATE_API_DATA_VERSION        (0x05)
#define CXM150x_CTRL_FW_UPDATE_API_DATA_COMPANY_NAME   (0x06)
#define CXM150x_CTRL_FW_UPDATE_API_DATA_DEVICE_NAME    (0x07)
#define CXM150x_CTRL_FW_UPDATE_API_DATA_STATE          (0x08)
#define CXM150x_CTRL_FW_UPDATE_API_DATA_END_REQUEST    (0xFF)

// Restart message after issuing firmware update end notification (Side A)
#define CXM150x_CTRL_FW_UPDATE_API_POWER_ON_MESSAGE_TYPE_A  "| SYS UPDATE"

// Restart message after issuing firmware update completion notice (Side B)
#define CXM150x_CTRL_FW_UPDATE_API_POWER_ON_MESSAGE_TYPE_B  "| SYS RESET POR_PIN"

// ON / OFF definition of flag
#define CXM150x_CTRL_FW_UPDATE_API_FLAG_ON         (1)
#define CXM150x_CTRL_FW_UPDATE_API_FLAG_OFF        (0)

// Time to hold the ENABLE pin to LOW to enter CONTROL FW UPDATE mode
#define CXM150x_CTRL_FW_UPDATE_API_ENABLE_WAIT_COUNT   (1000)

// prototype declaration
CXM150x_return_code wrapper_CXM150x_ctrl_fw_update_rx_message(uint8_t *rx_buf,uint32_t wait_cnt);
CXM150x_return_code wrapper_CXM150x_ctrl_fw_update_tx_message(uint8_t *snd_buf,uint8_t snd_cnt,uint32_t wait_cnt);
CXM150x_return_code wrapper_CXM150x_ctrl_fw_update_uart_abort_IT(void);

#endif // CXM150x_CTRL_FW_UPDATE_API_USE

#endif // __CXM150x_CTRL_FW_UPDATE_PORT_H

