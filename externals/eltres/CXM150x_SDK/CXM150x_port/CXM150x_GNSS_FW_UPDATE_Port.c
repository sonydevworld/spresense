// ==========================================================================
/*!
* @file     CXM150x_GNSS_FW_UPDATE_Port.c
* @brief    Define HAL wrapper functions for GNSS FW UPDATE
* @date     2021/08/16
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

#include <stdio.h>
#include "CXM150x_APITypeDef.h"
#include "CXM150x_GNSS_FW_UPDATE_Port.h"
#include "CXM150x_Port.h"
#include "stm32l0xx_hal.h"
#include "usart.h"

#define FOR_STM32_HAL_DRIVER_GNSS_FW_UPDATE

// Compile only if certain symbols are defined in CXM150x_APITypedef.h
#if CXM150x_GNSS_FW_UPDATE_API_USE


// ===========================================================================
//! Perform UART reception in GNSS FW UPDATE mode
/*!
 *
 * @param [in] wait_cnt: Maximum message reception wait count (specified in milliseconds)
 * @param [out] rx_buf: Receive buffer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return receive result
*/
// ===========================================================================
return_code wrapper_CXM150x_GNSS_fw_update_rx_message(uint8_t *rx_buf,uint32_t wait_cnt){
    uint8_t c = 0;
    uint32_t rcv_cnt = 0;
    int32_t set_result;
    uint32_t st_tick = wrapper_CXM150x_get_tick();
    uint32_t c_tick;
    
    while(1){
#ifdef FOR_STM32_HAL_DRIVER_GNSS_FW_UPDATE
        set_result = HAL_UART_Receive(&huart1,&c,1,wait_cnt);
        if(set_result == HAL_OK){
            if(rcv_cnt >= CXM150x_GNSS_FW_UPDATE_RECEIVE_MAX_LEN - 1){     // Subtract one because '\0' requires 1 byte
                rx_buf[rcv_cnt] = '\0';
                return RETURN_NG;
            }
            rx_buf[rcv_cnt++] = c;
            if(c == '\n'){
                rx_buf[rcv_cnt] = '\0';
                printf("rcv:%s",rx_buf);
                return RETURN_OK;
            }
        }
#endif
        // timeout check
        c_tick = wrapper_CXM150x_get_tick();
        if(c_tick - st_tick > wait_cnt){
            return RETURN_TIMEOUT;
        }
    }
}

// ===========================================================================
//! Perform UART transmission in GNSS FW UPDATE mode
/*!
 *
 * @param [in] snd_buf: Send message buffer
 * @param [in] snd_cnt: Number of transmit bytes
 * @param [in] wait_cnt: Maximum message reception wait count (specified in milliseconds)
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return transmission result
*/
// ===========================================================================
return_code wrapper_CXM150x_GNSS_fw_update_tx_message(uint8_t *snd_buf,uint32_t snd_cnt,uint32_t wait_cnt){

    // display sent message
    printf("snd: ");
    for(uint32_t i=0;i<snd_cnt;i++){
        printf("%02X ",snd_buf[i]);
    }
    printf("\r\n");

#ifdef FOR_STM32_HAL_DRIVER_GNSS_FW_UPDATE
    HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart1,snd_buf,snd_cnt,wait_cnt);
    if(ret == HAL_TIMEOUT){
        printf("transmit timeout.\r\n");
        return RETURN_TIMEOUT;
    } else if(ret == HAL_BUSY || ret == HAL_ERROR){
        printf("transmit error.(%d)\r\n",ret);
        return RETURN_NG;
    }
#endif

    return RETURN_OK;
}

// ===========================================================================
//! Abort UART IT
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
return_code wrapper_CXM150x_GNSS_fw_update_uart_abort_IT(void){
#ifdef FOR_STM32_HAL_DRIVER_GNSS_FW_UPDATE
    HAL_StatusTypeDef ret = HAL_UART_Abort_IT(&huart1);
    if(ret == HAL_TIMEOUT){
        printf("abort timeout.\r\n");
        return RETURN_TIMEOUT;
    } else if(ret == HAL_BUSY || ret == HAL_ERROR){
        printf("abort error.(%d)\r\n",ret);
        return RETURN_NG;
    }
#endif
    return RETURN_OK;
}


#endif  //CXM150x_GNSS_FW_UPDATE_API_USE
