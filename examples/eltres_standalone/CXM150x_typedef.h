// ==========================================================================
/*!
* @file     CXM150x_typedef.h
* @brief    Define type of Application
* @date     2021/12/27
*
* Copyright 2021 Sony Semiconductor Solutions Corporation
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CXM150x_TYPEDEF
#define __CXM150x_TYPEDEF
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
/* Includes ------------------------------------------------------------------*/

// CXM150x power ON / OFF state constant
typedef enum {
    CXM150x_POWER_OFF = 0,
    CXM150x_POWER_ON
}CXM150x_power_state;

// CXM150x WAKEUP High / Low state constant
typedef enum {
    CXM150x_WAKEUP_L = 0,
    CXM150x_WAKEUP_H
}CXM150x_wakeup_state;

// Define the command response
typedef enum {
    RETURN_NG = 0,
    RETURN_OK,
    RETURN_TIMEOUT,
    RETURN_BUSY
} return_code;

// OK or NG in the command response message from CXM150x
typedef enum {
    CXM150x_RESPONSE_NG = 0,
    CXM150x_RESPONSE_OK
}CXM150x_response_ok_ng;

// Flag ON / OFF definition
typedef enum {
    FLAG_OFF = 0,
    FLAG_ON
}FlagOnOff;

typedef enum {
    HOST_NORMAL_MODE = 0,
    HOST_STOP_MODE,
}HOST_Mode;

typedef enum {
    CXM150x_EVENT_NONE = 0,
    CXM150x_EVENT_SYS_RESET_POWER_PIN,
    CXM150x_EVENT_TX_PLD,
    CXM150x_EVENT_SYS_TO_DSLP,
    CXM150x_EVENT_SYS_RESET_DSLP,
    CXM150x_EVENT_POC_EN,
    CXM150x_EVENT_GNSS_TIMER_TOUT,
    CXM150x_EVENT_SYS_RESET_CMD,
    CXM150x_EVENT_TX_DUTY,
    CXM150x_EVENT_UNKNOWN_EVENT
}CXM150x_Event;

// Maximum message length from CXM150x
#define RECEIVE_BUF_SIZE    (0x80)

// Communication wait time with CXM (unit is msec)
#define MAX_TIME_OUT_TICK_COUNT     (5000)

// Timeout period from receiving the first character of UART communication to receiving CR + LF
#define MAX_UART_LINE_TIME_OUT_TICK_COUNT     (3000)

// Timeout period for CXM150x power ON message wait
#define MAX_POWER_ON_TIME_OUT_TICK_COUNT     (10000)

// Mode setting timeout time
#define MAX_SET_MODE_TIME_OUT_TICK_COUNT     (10000)

/* Command string definition */
#define CXM150x_COMMAND_PREFIX_CHAR      '<'
#define CXM150x_RESPONSE_PREFIX_CHAR     '>'
#define CXM150x_EVENT_PREFIX_CHAR        '|'


#define CXM150x_PAYLOAD_SET_COMMAND      "TX PLD SET"
#define CXM150x_PAYLOAD_SET_RESPONSE     "> TX PLD SET"
#define CXM150x_PAYLOAD_EVENT_MESSAGE    "| TX PLD"
#define CXM150x_POWER_ON_EVENT_MESSAGE   "| SYS RESET POR_PIN"
#define CXM150x_TX_POC_EVENT_MESSAGE     "| TX POC_EN"
#define CXM150x_SYS_TO_DSLP_EVENT_MESSAGE     "| SYS TO_DSLP"
#define CXM150x_POWER_DSLP_EVENT_MESSAGE   "| SYS RESET DSLP"
#define CXM150x_SYS_MODE_SET_COMMAND       "SYS MODE SET"
#define CXM150x_SYS_MODE_SET_RESPONSE      "> SYS MODE SET"
#define CXM150x_TX_POC_EN_SET_COMMAND      "TX POC_EN SET"
#define CXM150x_POC_EN_ON                  "ON"
#define CXM150x_TX_POC_EN_SET_RESPONSE      "> TX POC_EN SET"
#define CXM150x_TX_DUTY_SET_EVT_COMMAND      "TX DUTY SET_EVT"
#define CXM150x_TX_DUTY_SET_EVT_ON                  "ON"
#define CXM150x_TX_DUTY_SET_EVT_RESPONSE      "> TX DUTY SET_EVT"
#define CXM150x_TX_DUTY_EVENT_MESSAGE     "| TX DUTY"

#define CXM150x_NORMAL_MODE                (0)
#define CXM150x_GNSS_TIMER_TOUT_EVENT_MESSAGE "| GNSS TIMER TOUT"
#define CXM150x_SYS_RESET_CMD_EVENT_MESSAGE   "| SYS RESET CMD"

// Payload length (16 bytes)
#define CXM150x_PAYLOAD_LEN   (16)

#endif /* __CXM150x_TYPEDEF */




