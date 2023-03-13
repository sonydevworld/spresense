// ==========================================================================
/*!
* @file     CXM150x_TIME.c
* @brief    CXM150x control API (TIME group command)
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "CXM150x_TIME.h"
#include "CXM150x_LIB.h"
#include "CXM150x_Port.h"
#include "CXM150x_Utility.h"


extern CXM150x_CALLBACK_FUNC_POINTER g_LPWA_start_interrupt_callback_func_p;
extern CXM150x_CALLBACK_FUNC_POINTER g_uart_start_interrupt_callback_func_p;
extern CXM150xTimeAlarm *g_time_alarm_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_time_alarm_callback_func_p;

// ===========================================================================
//! Registration of INT_OUT1 callback function
/*!
 *
 * @param [in] func: Event notification callback function pointer
 * @param [in] info: NULL fixed
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_LPWA_start_interrupt_callback_func_p: INT_OUT1 interrupt callback function pointer
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_LPWA_start_interrupt(void *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_LPWA_start_interrupt_callback_func_p;
    
    g_LPWA_start_interrupt_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Registration of LPWA Transmit data update time limit UART event callback function
/*!
 *
 * @param [in] info: NULL fixed
 * @param [in] func: Event notification callback function
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_time_alarm_callback_func_p: INT_OUT1 event callback function pointer
 *        [out] g_time_alarm_info: Pointer to LPWA Transmit data update time limit UART event information structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_tx_start_message_event(CXM150xTimeAlarm *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_time_alarm_callback_func_p;
    
    g_time_alarm_callback_func_p = func;
    g_time_alarm_info = info;
    
    return prev_func;
}

// ===========================================================================
//! Parse remaining time before next startup time of CXM150x
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_get_next_LPWA_tx_CXM150x_power_on_limit_time(uint8_t *response,void *res_buf){
    CmdResGetCXM150xNextLPWATxPowerOnLimitTime *res = (CmdResGetCXM150xNextLPWATxPowerOnLimitTime*)res_buf;
    uint8_t buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint32_t prem_time = 0;

    // Parse CXM150x response message
    if(res != NULL){
        //Error checking
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            // Get the last word out of the message
            if(CXM150x_get_last_word(response,buf) == CXM150x_RESPONSE_OK){
                if(sscanf((char*)buf, "%ld", &prem_time)){
                    res->m_num = prem_time;
                } else {
                    res->m_num = CXM150x_RESPONSE_ERROR;
                }
            } else {
                res->m_num = CXM150x_RESPONSE_ERROR;
            }
        } else {
            res->m_num = CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Get remaining time before next startup time of CXM150x
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_next_LPWA_tx_power_on_limit_time(void *param,CmdResGetCXM150xNextLPWATxPowerOnLimitTime *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TIME PREM GET
    //> TIME PREM GET Decimal data (second)
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TIME_PREM,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_next_LPWA_tx_CXM150x_power_on_limit_time,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_next_LPWA_tx_CXM150x_power_on_limit_time(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse remaining time before update time limit for next LPWA transmit data
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_get_next_LPWA_tx_renew_limit_time(uint8_t *response,void *res_buf){
    CmdResGetCXM150xNextLPWATxRenewLimitTime *res = (CmdResGetCXM150xNextLPWATxRenewLimitTime*)res_buf;
    uint8_t buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint32_t drem_time = 0;

    // Parse CXM150x response message
    if(res != NULL){
        //Error checking
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            // Get the last word out of the message
            if(CXM150x_get_last_word(response,buf) == CXM150x_RESPONSE_OK){
                if(sscanf((char*)buf, "%ld", &drem_time)){
                    res->m_num = drem_time;
                } else {
                    res->m_num = CXM150x_RESPONSE_ERROR;
                }
            } else {
                res->m_num = CXM150x_RESPONSE_ERROR;
            }
        } else {
            res->m_num = CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Acquisition of the remaining time before update time limit for next LPWA transmit data
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_next_LPWA_tx_renew_limit_time(void *param,CmdResGetCXM150xNextLPWATxRenewLimitTime *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TIME DREM GET
    //> TIME DREM GET Decimal data (second)
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TIME_DREM,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_next_LPWA_tx_renew_limit_time,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_next_LPWA_tx_renew_limit_time(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse remaining time before next LPWA transmission
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_get_next_LPWA_tx_term(uint8_t *response,void *res_buf){
    CmdResGetCXM150xNextLPWATxTerm *res = (CmdResGetCXM150xNextLPWATxTerm*)res_buf;
    uint8_t buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint32_t trem_time = 0;

    // Parse CXM150x response message
    if(res != NULL){
        //Error checking
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            // Get the last word out of the message
            if(CXM150x_get_last_word(response,buf) == CXM150x_RESPONSE_OK){
                if(sscanf((char*)buf, "%ld", &trem_time)){
                    res->m_num = trem_time;
                } else {
                    res->m_num = CXM150x_RESPONSE_ERROR;
                }
            } else {
                res->m_num = CXM150x_RESPONSE_ERROR;
            }
        } else {
            res->m_num = CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Get remaining time before next LPWA transmission
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_next_LPWA_tx_term(void *param,CmdResGetCXM150xNextLPWATxTerm *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TIME TREM GET
    //> TIME TREM GET Decimal data (second)
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TIME_TREM,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_next_LPWA_tx_term,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_next_LPWA_tx_term(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the current GPS time acquisition result
/*!
 *
 * @param [in] response: Response string from CXM150x
 * @param [out] res_buf: Parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_get_current_GNSS_time(uint8_t *response,void *res_buf){
    CmdResGetCXM150xCurrentGNSSTime *res = (CmdResGetCXM150xCurrentGNSSTime*)res_buf;
    uint8_t buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    
    // Parse CXM150x response message
    if(res != NULL){
        //Error checking
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            // Get the last word out of the message
            if(CXM150x_get_last_word(response,buf) == CXM150x_RESPONSE_OK){
                // buf [2]-> Since "0x" is prefixed, parse from the third character
                strncpy((char*)res->m_str, (char*)&buf[2], CXM150x_RECEIVE_BUF_GNSSTIME_SIZE);
                res->m_str[CXM150x_RECEIVE_BUF_GNSSTIME_SIZE] = '\0';
            } else {
                res->m_str[0] = '\0';
            }
        } else {
            // Set NULL for NG
            res->m_str[0] = '\0';
        }
    }

}

// ===========================================================================
//! Get current GPS time
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_current_GNSS_time(void *param,CmdResGetCXM150xCurrentGNSSTime *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TIME CTIME GET
    //> TIME CTIME GET 0x7D044C9E
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TIME_CTIME,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_current_GNSS_time,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_current_GNSS_time(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Callback function called when INT1 interrupt is received
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_LPWA_start_interrupt_callback_func_p: INT1 callback function pointer
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
void CXM150x_on_int1(void){
    if(g_LPWA_start_interrupt_callback_func_p != NULL){
        g_LPWA_start_interrupt_callback_func_p(NULL,CXM150x_EVENT_CALLBACK_ID_LPWA_START_INTERRUPT);
    }
}

// ===========================================================================
//! Callback function called when INT2 interrupt occurs
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_uart_start_interrupt_callback_func_p: INT2 callback function pointer
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
void CXM150x_on_int2(void){
    if(g_uart_start_interrupt_callback_func_p != NULL){
        g_uart_start_interrupt_callback_func_p(NULL,CXM150x_EVENT_CALLBACK_ID_CXM150x_UART_START_INTERRUPT);
    }
}
