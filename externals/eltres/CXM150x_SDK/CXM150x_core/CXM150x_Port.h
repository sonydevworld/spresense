// ==========================================================================
/*!
* @file     CXM150x_Port.h
* @brief    HAL wrapper functions
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

#ifndef __CXM150x_PORT_H
#define __CXM150x_PORT_H

#include <nuttx/config.h>
#include <debug.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>
#include "CXM150x_APITypeDef.h"

/* Debug print */

#ifdef CONFIG_EXTERNALS_ELTRES_DEBUG_ERROR
#  define printf_err(format, ...)   printf(format, ##__VA_ARGS__)
#else
#  define printf_err(format, ...)
#endif

#ifdef CONFIG_EXTERNALS_ELTRES_DEBUG_INFO
#  define printf_info(format, ...)  printf(format, ##__VA_ARGS__)
#else
#  define printf_info(format, ...)
#endif

/* Board-specific pin assignment */

#if defined(CONFIG_EXTERNALS_ELTRES_SPEXEL)
#  define ELTRES_PIN_DDC2V        PMIC_GPO(2)
#  define ELTRES_PIN_ENABLE       PMIC_GPO(3)
#  define ELTRES_PIN_WAKEUP       PIN_HIF_GPIO0
#  define ELTRES_PIN_INT_OUT1     PIN_PWM3
#  define ELTRES_PIN_INT_OUT2     PIN_PWM2
#elif defined(CONFIG_EXTERNALS_ELTRES_ADDON)
#  define ELTRES_PIN_ENABLE       PIN_I2S0_BCK
#  define ELTRES_PIN_WAKEUP       PIN_I2S0_LRCK
#  define ELTRES_PIN_INT_OUT1     PIN_I2S0_DATA_IN
#  define ELTRES_PIN_INT_OUT2     PIN_I2S0_DATA_OUT
#elif defined(CONFIG_EXTERNALS_ELTRES_ORIGINAL)
#  warning "TODO: Define the pin assignments for your original board."
#  define ELTRES_PIN_ENABLE       PIN_EMMC_DATA2
#  define ELTRES_PIN_WAKEUP       PIN_EMMC_DATA3
#  define ELTRES_PIN_INT_OUT1     PIN_HIF_IRQ_OUT
#  define ELTRES_PIN_INT_OUT2     PIN_GNSS_1PPS_OUT
#endif

#define WRAPPER_UART_RX_FROM_CXM150x    (1)
#define WRAPPER_UART_RX_FROM_PC           (2)

void wrapper_CXM150x_set_power(CXM150x_power_state on_off);
uint32_t wrapper_CXM150x_get_power(void);
void wrapper_CXM150x_set_Wakeup_pin(CXM150x_power_state on_off);
void wrapper_CXM150x_set_wakeup_pin(CXM150x_wakeup_state high_low);
void wrapper_CXM150x_set_uart_Hiz(void);
int32_t wrapper_CXM150x_set_uart_rx_buf(uint8_t *rcv_char_buf);
void wrapper_CXM150x_set_uart_rx_callback(void (*callback)(uint32_t,uint32_t));
CXM150x_return_code wrapper_CXM150x_uart_transmit(uint8_t *data,uint16_t len,uint32_t wait_count);
uint32_t wrapper_CXM150x_get_tick(void);
void wrapper_CXM150x_delay(uint32_t tick);
void wrapper_CXM150x_int_out1(void);
void wrapper_CXM150x_int_out2(void);

void wrapper_CXM150x_enter_stop_mode(void);
void wrapper_CXM150x_resume_stop_mode(void);
void wrapper_CXM150x_system_reset(void);

#endif // __CXM150x_PORT_H
