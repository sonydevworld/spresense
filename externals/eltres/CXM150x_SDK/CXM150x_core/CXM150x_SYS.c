// ==========================================================================
/*!
* @file     CXM150x_SYS.c
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "CXM150x_SYS.h"
#include "CXM150x_LIB.h"
#include "CXM150x_Port.h"
#include "CXM150x_Utility.h"

// Mode setting timeout time
#define MAX_SET_MODE_TIME_OUT_TICK_COUNT     (10000)

extern CXM150xSysState *g_sys_state_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_sys_state_callback_func_p;
extern CXM150x_CALLBACK_FUNC_POINTER g_uart_start_interrupt_callback_func_p;
extern CXM150xFATALMessage    *g_FATAL_message_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_FATAL_message_callback_func_p;
extern CXM150xEventBufferOverflow *g_eventbuffer_overflow_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_event_buffer_overflow_func_p;
extern CXM150xSysToDeepSleepInfo *g_sys_to_deepsleep_event_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_sys_to_deepsleep_callback_func_p;

// ===========================================================================
//! Parse CXM150x message
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
void analyse_CXM150x_Rx(void){
    CXM150x_trigger_analyse();
}

// ===========================================================================
//! Parse startup message
/*! Tentatively, set OK
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
static void res_check_set_CXM150x_power(uint8_t *response,void *res_buf){
    CmdResSetCXM150xPower *res = (CmdResSetCXM150xPower*)res_buf;
    res->m_result = CXM150x_RESPONSE_OK;
}

// ===========================================================================
//! Set the power supply status
/*! Wait for startup message after CXM150x power ON
 *
 * @param [in] param: Power supply status setting
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_power(uint32_t param, CmdResSetCXM150xPower *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    if(param == CXM150x_POWER_ON){
        // CXM150x normal power ON
        printf_info("CXM150x power on\r\n");
        CXM150x_init_uart_driver();

        /* Already poweron */
        if (CXM150x_POWER_ON == wrapper_CXM150x_get_power()){
            printf_info("Already power on\r\n");
            if(res != NULL){
                res->m_result = CXM150x_RESPONSE_OK;
            }
            if (func != NULL){
                func(RETURN_OK,res);
            }
            return RETURN_OK;
        }
        wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_ON);
        wrapper_CXM150x_set_power(CXM150x_POWER_ON);
        
        if(func == NULL){
            CXM150x_return_code ret = CXM150x_wait_power_on_message();
            if(res != NULL){
                res->m_result = CXM150x_RESPONSE_OK;
            }
            return ret;
        } else {
            return CXM150x_prep_wait_power_on_message(func,res_check_set_CXM150x_power,res);
        }
    } else if(param == CXM150x_POWER_OFF){
        // Power off
        wrapper_CXM150x_set_power(CXM150x_POWER_OFF);
        wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_OFF);
        
        printf_info("CXM150x power off\r\n");
        
        if(res != NULL){
            res->m_result = CXM150x_RESPONSE_OK;
        }
        
        if(func != NULL){
            func(RETURN_OK,res);
        }

    } else if(param == CXM150x_POWER_CONTROL_FW_UPDATE){
        // CONTROL_FW update mode
        printf_info("CXM150x power CONTROL_FW_UPDATE mode\r\n");

        wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_OFF);
        wrapper_CXM150x_set_power(CXM150x_POWER_ON);
        wrapper_CXM150x_set_uart_Hiz();

        // always OK because no communication is involved
        if(res != NULL){
            res->m_result = CXM150x_RESPONSE_OK;
        }

        if(func != NULL){
            func(RETURN_OK,res);
        }
    } else if(param == CXM150x_POWER_GNSS_FW_UPDATE){
        // GNSS block FW update mode
        printf_info("CXM150x power GNSS_FW_UPDATE mode\r\n");

        wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_ON);
        wrapper_CXM150x_set_power(CXM150x_POWER_ON);
        wrapper_CXM150x_set_uart_Hiz();
        
        // always OK because no communication is involved
        if(res != NULL){
            res->m_result = CXM150x_RESPONSE_OK;
        }

        if(func != NULL){
            func(RETURN_OK,res);
        }
    } else {
        printf_err("set_CXM150x_power param invalid\r\n");
    }
    
    
    return RETURN_OK;
}

// ===========================================================================
//! Get power supply status
/*! 
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return on_off: Get power supply status
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_power(void *param,CmdResGetCXM150xPower *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){

    if(res != NULL){
        res->m_num = wrapper_CXM150x_get_power();
    }
    
    if(func != NULL){
        func(RETURN_OK,res);
    }
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the setting result of system mode
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
static void res_check_set_CXM150x_mode(uint8_t *response,void *res_buf){
    CmdResSetCXM150xMode *res = (CmdResSetCXM150xMode*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        // Whether the message ends in 'OK' or not
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_check_last_ok_ng(response) == CXM150x_RESPONSE_OK){
                res->m_result = CXM150x_RESPONSE_OK;
            } else {
                res->m_result = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! System mode setting
/*!
 *
 * @param [in] mode: System mode setting value
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_mode(uint32_t mode,CmdResSetCXM150xMode *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS MODE SET 00
    //> SYS MODE SET OK
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t mode_name[3] = "";
    
    snprintf((char*)mode_name,sizeof(mode_name),"%02ld",mode);
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_MODE,CXM150x_COMMAND_SET,mode_name);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback_long_wait(command,func,res_check_set_CXM150x_mode,res,MAX_SET_MODE_TIME_OUT_TICK_COUNT);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response_long_wait(command,response,MAX_SET_MODE_TIME_OUT_TICK_COUNT);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_mode(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the acquisition result of system mode
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
static void res_check_get_CXM150x_mode(uint8_t *response,void *res_buf){
    CmdResGetCXM150xMode *res = (CmdResGetCXM150xMode*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        uint8_t mode_str[CXM150x_MAX_COMMAND_LEN] = "";
         if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_get_last_word(response,mode_str) == CXM150x_RESPONSE_OK){
                if(sscanf((char*)mode_str,"%ld",&res->m_num) == 0){
                    res->m_num = CXM150x_RESPONSE_ERROR;
                }
            } else {
                // Acquisition failed
                res->m_num = CXM150x_RESPONSE_ERROR;
            }
        } else {
            // Acquisition failed
            res->m_num = CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Get system mode
/*!
 *
 * @param [in] param: System mode setting value
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_mode(void *param,CmdResGetCXM150xMode *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS MODE GET
    //> SYS MODE GET 00
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_MODE,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_mode,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_mode(response,res);

    
    return RETURN_OK;
}

// ===========================================================================
//! Register interrupt callback function before UART transmission start
/*!
 *
 * @param [in] info: Event data storage location
 * @param [in] func: Interrupt callback function
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_uart_start_interrupt_callback_func_p: Interrupt callback function pointer before UART transmission start
 *
 * @return Callback function set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_uart_start_interrupt(void *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_uart_start_interrupt_callback_func_p;
    
    g_uart_start_interrupt_callback_func_p = func;
    
    return prev_func;
}

// ===========================================================================
//! Register system state event notification callback function
/*!
 *
 * @param [in] info: Event data storage location
 * @param [in] func: Event notification callback function
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_sys_state_callback_func_p: system state event notification callback function pointer
 *        [out] g_sys_state_info: pointer to system state event notification structure
 *
 * @return Callback function set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_sys_state_event(CXM150xSysState *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_sys_state_callback_func_p;
    
    g_sys_state_callback_func_p = func;
    g_sys_state_info = info;
    
    return prev_func;
}

// ===========================================================================
//! Parse the setting result of the system state event notification
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
static void res_check_set_sys_state_event(uint8_t *response,void *res_buf){
    CmdResSetCXM150xSysStateEvent *res = (CmdResSetCXM150xSysStateEvent*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        // Whether the message ends in 'OK' or not
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_check_last_ok_ng(response) == CXM150x_RESPONSE_OK){
                res->m_result = CXM150x_RESPONSE_OK;
            } else {
                res->m_result = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! System state event notification settings
/*!
 *
 * @param [in] on_off: Event notification setting
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_sys_state_event(uint32_t on_off,CmdResSetCXM150xSysStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS STT SET_EVT ON
    //> SYS STT SET_EVT OK
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    if(on_off == EVENT_ON){
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_STT,CXM150x_COMMAND_SET_EVT,CXM150x_COMMAND_ON);
    } else {
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_STT,CXM150x_COMMAND_SET_EVT,CXM150x_COMMAND_OFF);
    }
    
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_sys_state_event,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_sys_state_event(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the result of getting the system state event notification
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
static void res_check_get_sys_state_event(uint8_t *response,void *res_buf){
    CmdResGetCXM150xSysStateEvent *res = (CmdResGetCXM150xSysStateEvent*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            res->m_num = CXM150x_check_last_on_off(response);
        } else {
            res->m_num = CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Get system state event notification settings
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_sys_state_event(void *param,CmdResGetCXM150xSysStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS STT GET_EVT
    //> SYS STT SET_EVT OK
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_STT,CXM150x_COMMAND_GET_EVT);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_sys_state_event,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_sys_state_event(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Analyze system state message
/*!
 *
 * @param [in] msg: CXM150x message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return system state information
*/
// ===========================================================================
CXM150xSysState conv_sys_stt_message_to_code(uint8_t *msg){
    if(msg == NULL){
        return SYS_STT_PARSE_ERROR;
    }
    
    // Note: Do not delete the space leading to "FETCHING_TIME" to distinguish it from "WAIT_FETCHING_TIME"
    // Note: "GNSS_BACKUP_DONE" should be judged before "GNSS_BACKUP" (because of partial match)
    if(strstr((char*)msg,"IDLE")){
        return SYS_STT_IDLE;
    } else if(strstr((char*)msg," FETCHING_TIME")){
        return SYS_STT_FETCHING_TIME;
    } else if(strstr((char*)msg,"WAIT_FETCHING_TIME")){
        return SYS_STT_WAIT_FETCHING_TIME;
    } else if(strstr((char*)msg,"EPM_FILL")){
        return SYS_STT_EPM_FILL;
    } else if(strstr((char*)msg,"WAIT_TX_PREPARE")){
        return SYS_STT_WAIT_TX_PREPARE;
    } else if(strstr((char*)msg,"AF_TX_PREPARE")){
        return SYS_STT_AF_TX_PREPARE;
    } else if(strstr((char*)msg,"AF_WAIT_TX_START")){
        return SYS_STT_AF_WAIT_TX_START;
    } else if(strstr((char*)msg,"AF_TX_PROGRESS")){
        return SYS_STT_AF_TX_PROGRESS;
    } else if(strstr((char*)msg,"DF_TX_PREPARE")){
        return SYS_STT_DF_TX_PREPARE;
    } else if(strstr((char*)msg,"DF_WAIT_TX_START")){
        return SYS_STT_DF_WAIT_TX_START;
    } else if(strstr((char*)msg,"DF_TX_PROGRESS")){
        return SYS_STT_DF_TX_PROGRESS;
    } else if(strstr((char*)msg,"EV_TX_COMPLETE")){
        return SYS_STT_EV_TX_COMPLETE;
    } else if(strstr((char*)msg,"GNSS_BACKUP_DONE")){
        return SYS_STT_GNSS_BACKUP_DONE;
    } else if(strstr((char*)msg,"GNSS_BACKUP")){
        return SYS_STT_GNSS_BACKUP;
    } else {
        // Error if none of the above
        printf_err("conv_sys_stt_message_to_code decode error:%s\r\n",msg);
    }
    
    return SYS_STT_PARSE_ERROR;

}

// ===========================================================================
//! Parse the result of obtaining the firmware version
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
static void res_check_get_CXM150x_firmware_version(uint8_t *response,void *res_buf){
    CmdResGetCXM150xFirmwareVersion *res = (CmdResGetCXM150xFirmwareVersion*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        uint8_t *find_start_pos = &response[SYS_GET_VER_FORMAT_HARDWARE_POS];
        uint8_t *space_pos = NULL;

        // Hardware ID
        space_pos = (uint8_t*)strchr((char*)find_start_pos,' ');
        if(space_pos != NULL && (space_pos - find_start_pos) ==  SYS_GET_VER_FORMAT_HARDWARE_LEN){
            CXM150x_ascii_to_bin(find_start_pos,res->m_id,SYS_GET_VER_FORMAT_HARDWARE_LEN);
        } else {
            return;
        }
        
        // FW version
        find_start_pos = space_pos + 1;
        space_pos = (uint8_t*)strchr((char*)find_start_pos,' ');
        // Version names can be 5 or 6 characters in length
        uint32_t fw_ver_len = (space_pos - find_start_pos);
        if(space_pos != NULL && fw_ver_len <=  SYS_GET_VER_FORMAT_VER_NAME_LEN){
            strncpy((char*)res->m_version,(char*)find_start_pos,fw_ver_len);
            res->m_version[fw_ver_len] = '\0';
        } else {
            return;
        }
        
        // commit ID
        find_start_pos = space_pos + 1;
        space_pos = (uint8_t*)strchr((char*)find_start_pos,' ');
        if(space_pos != NULL && (space_pos - find_start_pos) ==  SYS_GET_VER_FORMAT_COMMITID_LEN){
            uint8_t commit_id_buf[SYS_GET_VER_FORMAT_COMMITID_LEN+1] = "";
            memcpy(commit_id_buf,find_start_pos,SYS_GET_VER_FORMAT_COMMITID_LEN);
            if(sscanf((char*)commit_id_buf,"%lX",&res->m_commit_id) == 0){
                res->m_commit_id = 0;
            }
        } else {
            return;
        }
        
        // build date
        find_start_pos = space_pos + 1;
        strncpy((char*)res->m_build_date,(char*)find_start_pos,SYS_GET_VER_FORMAT_BUILD_DT_LEN);
        res->m_build_date[SYS_GET_VER_FORMAT_BUILD_DT_LEN] = '\0';
    }
}

// ===========================================================================
//! Get firmware version
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_firmware_version(void *param,CmdResGetCXM150xFirmwareVersion *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS VER GET
    //> SYS VER GET 002F00413833343104473533 LF00D 01234567 Feb  7 2018 17:51:49
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    memset(res,0,sizeof(CmdResGetCXM150xFirmwareVersion));
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_VER,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_firmware_version,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_firmware_version(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the system state acquisition result
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
static void res_check_get_sys_state_event_info(uint8_t *response,void *res_buf){
    CmdResGetCXM150xSysStateEventInfo *info = (CmdResGetCXM150xSysStateEventInfo*)res_buf;
    // Parse CXM150x response message
    if(info != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            info->m_num = conv_sys_stt_message_to_code(response);
        } else {
            info->m_num = CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Get system state event
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] info: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_sys_state_event_info(void *param,CmdResGetCXM150xSysStateEventInfo *info,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS STT GET
    //> SYS STT GET FETCHING_TIME
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_STT,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_sys_state_event_info,info);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_sys_state_event_info(response,info);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the FETCHING_TIME transition command result
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
static void res_check_set_CXM150x_fetching_time(uint8_t *response,void *res_buf){
    CmdResSetCXM150xFetchingTime *info = (CmdResSetCXM150xFetchingTime*)res_buf;
    // Parse CXM150x response message
    if(info != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            info->m_result = CXM150x_RESPONSE_OK;
        } else {
            info->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! Force the CXM150x to transit to the FETCHING_TIME state
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_fetching_time(void *param, CmdResSetCXM150xFetchingTime *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS TO_FETCHING SET ON
    //> SYS TO_FETCHING SET OK
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    memset(res,0,sizeof(CmdResSetCXM150xFetchingTime));
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_TO_FETCHING,CXM150x_COMMAND_SET,CXM150x_COMMAND_ON);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_fetching_time,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_fetching_time(response,res);

    return RETURN_OK;
}


// ===========================================================================
//! Parse the WAIT_FETCHING_TIME transition command result
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
static void res_check_set_CXM150x_wait_fetching_time(uint8_t *response,void *res_buf){
    CmdResSetCXM150xWaitFetchingTime *info = (CmdResSetCXM150xWaitFetchingTime*)res_buf;
    // Parse CXM150x response message
    if(info != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            info->m_result = CXM150x_RESPONSE_OK;
        } else {
            info->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! Force the CXM150x to transit to the WAIT_FETCHING_TIME state
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_wait_fetching_time(void *param, CmdResSetCXM150xWaitFetchingTime *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS TO_WAIT_FETCHING SET ON
    //> SYS TO_WAIT_FETCHING SET OK
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    memset(res,0,sizeof(CmdResSetCXM150xWaitFetchingTime));
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_TO_WAIT_FETCHING,CXM150x_COMMAND_SET,CXM150x_COMMAND_ON);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_wait_fetching_time,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_wait_fetching_time(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse reset command result
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
static void res_check_reset_CXM150x(uint8_t *response,void *res_buf){
    CmdResResetCXM150x *info = (CmdResResetCXM150x*)res_buf;
    // Parse CXM150x response message
    if(info != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            info->m_result = CXM150x_RESPONSE_OK;
        } else {
            info->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! Send reset command
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code reset_CXM150x(void *param,CmdResResetCXM150x *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS RESET SET ON
    //> SYS RESET SET OK
    //| SYS RESET CMD
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_RESET,CXM150x_COMMAND_SET,CXM150x_COMMAND_ON);
    
    if(func != NULL){
        // Send command
        // After receiving command response OK, start waiting for CXM150x power ON message (implemented in check_command_response function of CXM150x_LIB.c)
        ret = CXM150x_send_and_register_callback(command,func,res_check_reset_CXM150x,res);
        return ret;
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
        if(ret == RETURN_OK){
            // Parse the contents of the response, and if the response is OK, start waiting for the CXM150x power ON message
            // Use a temporary variable because NULL may be specified in the response structure
            CmdResResetCXM150x resCmdResResetCXM150x;
            res_check_reset_CXM150x(response,&resCmdResResetCXM150x);
            if(resCmdResResetCXM150x.m_result == CXM150x_RESPONSE_OK){
                ret = CXM150x_wait_power_on_message();
                if(ret != RETURN_OK){
                    return ret;
                }
            }
        }
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    // Parse and set the message content in the response structure specified by the API caller
    res_check_reset_CXM150x(response,res);

    return RETURN_OK;
}


// ===========================================================================
//! Parse reset factor
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
static void res_check_get_CXM150x_reset_event_info(uint8_t *response,void *res_buf){
    CmdResGetCXM150xResetEventInfo *res = (CmdResGetCXM150xResetEventInfo*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        uint8_t mode_str[CXM150x_MAX_COMMAND_LEN] = "";
         if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_get_last_word(response,mode_str) == CXM150x_RESPONSE_OK){
                if(strstr((char*)mode_str,"0x0C")){
                    res->m_num = SYS_RESET_POWER;
                } else if(strstr((char*)mode_str,"0x14")){
                    res->m_num = SYS_RESET_COMMAND;
                } else {
                    res->m_num = CXM150x_RESPONSE_ERROR;
                }

            } else {
                //Acquisition failure
                res->m_num = CXM150x_RESPONSE_ERROR;
            }
        } else {
            //Acquisition failure
            res->m_num = CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Get reset factor
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_reset_event_info(void *param, CmdResGetCXM150xResetEventInfo *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS RESET GET
    //> SYS RESET GET 0x14
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_RESET,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_reset_event_info,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_reset_event_info(response,res);
    
    return RETURN_OK;
}


// ===========================================================================
//! Parse the EEPROM data acquisition result
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
static void res_check_get_CXM150x_EEPROM_data(uint8_t *response,void *res_buf){
    CmdResGetCXM150xEEPROMData *res = (CmdResGetCXM150xEEPROMData*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            uint8_t val_str[CXM150x_MAX_COMMAND_LEN] = "";
            if(CXM150x_get_last_word(response,val_str)){
                uint32_t val = 0;
                if(sscanf((char*)val_str,"0x%08lX",&val) == 0){
                    res->m_num = (uint32_t)CXM150x_RESPONSE_ERROR;
                } else {
                    res->m_num = val;
                }
            } else {
                res->m_num = (uint32_t)CXM150x_RESPONSE_ERROR;
            }
        } else {
            res->m_num = (uint32_t)CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Get EEPROM data
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_EEPROM_data(uint32_t param, CmdResGetCXM150xEEPROMData *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS EEPROM GET 0x0004
    //> SYS EEPROM GET 0x1D1D1D1D
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    uint8_t offset_address_str[7] = "";
    snprintf((char*)offset_address_str,sizeof(offset_address_str),"0x%04lX",param);
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_EEPROM,CXM150x_COMMAND_GET,offset_address_str);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_EEPROM_data,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_EEPROM_data(response,res);
    
    return RETURN_OK;
}


// ===========================================================================
//! Parse the EEPROM data setting result
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
static void res_check_set_CXM150x_EEPROM_data(uint8_t *response,void *res_buf){
    CmdResSetCXM150xEEPROMData *res = (CmdResSetCXM150xEEPROMData*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        // Whether the message ends in 'OK' or not
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_check_last_ok_ng(response) == CXM150x_RESPONSE_OK){
                res->m_result = CXM150x_RESPONSE_OK;
            } else {
                res->m_result = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! EEPROM data setting
/*!
 *
 * @param [in] param: EEPROM setting information
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_EEPROM_data(CXM150xEEPROMSetData*  param, CmdResSetCXM150xEEPROMData *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS EEPROM SET 0x0224,0x00000001
    //> SYS EEPROM SET OK
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";

    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s 0x%04lX,0x%08lX\r\n",
             CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_EEPROM,CXM150x_COMMAND_SET,
             param->m_offset_address,param->m_val);

    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_EEPROM_data,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_EEPROM_data(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the result of getting the boot loader version
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
static void res_check_get_CXM150x_Bootloader_version(uint8_t *response,void *res_buf){
    CmdResGetCXM150xBootloaderVersion *res = (CmdResGetCXM150xBootloaderVersion*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        uint8_t *find_start_pos = &response[SYS_GET_BTVER_FORMAT_VER_NAME_POS];
        uint8_t *space_pos = NULL;
        
        space_pos = (uint8_t*)strchr((char*)find_start_pos,' ');
        
        // Version names can be 5 or 6 characters in length
        int32_t btver_len = (space_pos - find_start_pos);
        
        if(space_pos != NULL && (btver_len == SYS_GET_BTVER_FORMAT_VER_NAME_LEN || btver_len == SYS_GET_BTVER_FORMAT_VER_NAME_LEN - 1)){
            // FW version
            memcpy(res->m_version,&response[SYS_GET_BTVER_FORMAT_VER_NAME_POS],btver_len);
            res->m_version[btver_len] = '\0';

        } else {
            return;
        }
        
        find_start_pos = space_pos + 1;
        // build date
        memcpy(res->m_build_date,find_start_pos,SYS_GET_BTVER_FORMAT_BUILD_DT_LEN);
        res->m_build_date[SYS_GET_BTVER_FORMAT_BUILD_DT_LEN] = '\0';
    }
}

// ===========================================================================
//! Get boot loader version
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_Bootloader_version(void *param,CmdResGetCXM150xBootloaderVersion *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS BTVER GET
    //> SYS BTVER GET DZ001 Dec 27 2017 16:09:53
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    memset(res,0,sizeof(CmdResGetCXM150xBootloaderVersion));
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_BTVER,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_Bootloader_version,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_Bootloader_version(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Register FATAL message event notification callback function
/*!
 *
 * @param [in] info: Event data storage location
 * @param [in] func: Event notification callback function
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_FATAL_message_callback_func_p: FATAL message event notification callback function pointer
 *        [out] g_FATAL_message_info: Pointer to FATAL message event notification structure
 *
 * @return Callback function set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_FATAL_message_event(CXM150xFATALMessage *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_FATAL_message_callback_func_p;
    
    g_FATAL_message_callback_func_p = func;
    g_FATAL_message_info = info;
    
    return prev_func;
}

// ===========================================================================
//! Register event buffer overflow occurrence callback function
/*!
 *
 * @param [in] info: Overflow information storage location
 * @param [in] func: Event buffer overflow occurrence callback function
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_event_buffer_overflow_func_p: Event buffer overflow occurrence callback function pointer
 *        [out] g_eventbuffer_overflow_info: Pointer to overflow information
 *
 * @return Callback function set immediately before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_event_buffer_overflow(CXM150xEventBufferOverflow *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_event_buffer_overflow_func_p;
    
    g_event_buffer_overflow_func_p = func;
    g_eventbuffer_overflow_info = info;
    
    return prev_func;
}

// ===========================================================================
//! Get API version
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return OK fixed
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_api_version(void *param,CmdResGetCXM150xAPIVersion *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    
    if(res != NULL){
        // Set the version defined in CXM150x_APITypeDef.h in the response data structure
        strncpy((char*)res->m_version,CXM150x_API_VERSION,sizeof(CmdResGetCXM150xAPIVersion));
        res->m_version[CXM150x_API_VERSION_MAX_LEN] = '\0';
    }
    
    if(func != NULL){
        func(RETURN_OK,res);
    }
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse backup result of GNSS information
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
static void res_check_set_CXM150x_GNSS_backup(uint8_t *response,void *res_buf){
    CmdResSetCXM150xGNSSBackup *res = (CmdResSetCXM150xGNSSBackup*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        // Whether the message ends in 'OK' or not
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_check_last_ok_ng(response) == CXM150x_RESPONSE_OK){
                res->m_result = CXM150x_RESPONSE_OK;
            } else {
                res->m_result = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! Backup GNSS information
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_GNSS_backup(void *param, CmdResSetCXM150xGNSSBackup *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS TO_GNSS_BACKUP SET ON
    //> SYS TO_GNSS_BACKUP SET OK
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_TO_GNSS_BACKUP,CXM150x_COMMAND_SET,CXM150x_COMMAND_ON);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_GNSS_backup,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_GNSS_backup(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the EEPROM data sequentially acquisition result
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
static void res_get_CXM150x_EEPROM_data_sequential(uint8_t *response,void *res_buf){
    CmdResGetCXM150xEEPROMDataSequential *res = (CmdResGetCXM150xEEPROMDataSequential*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            uint8_t val_str[CXM150x_MAX_COMMAND_LEN] = "";
            if(CXM150x_get_last_word(response,val_str)){
                uint32_t val = 0;
                if(sscanf((char*)val_str,"0x%08lX",&val) == 0){
                    res->m_num = (uint32_t)CXM150x_RESPONSE_ERROR;
                } else {
                    res->m_num = val;
                }
            } else {
                res->m_num = (uint32_t)CXM150x_RESPONSE_ERROR;
            }
        } else {
            res->m_num = (uint32_t)CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Get EEPROM data sequentially
/*!
 *
 * @param [in] param: NULL fixed
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_EEPROM_data_sequential(void* param, CmdResGetCXM150xEEPROMDataSequential *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS EEPROM GET
    //> SYS EEPROM GET 0x1D1D1D1D
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_EEPROM,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_get_CXM150x_EEPROM_data_sequential,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_get_CXM150x_EEPROM_data_sequential(response,res);
    
    return RETURN_OK;
}


// ===========================================================================
//! Parse to deepsleep response information
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
static void res_set_CXM150x_sys_to_deep_sleep(uint8_t *response,void *res_buf){
    CmdResSetCXM150xSysToDeepSleep *res = (CmdResSetCXM150xSysToDeepSleep*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        // Whether the message ends in 'OK' or not
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_check_last_ok_ng(response) == CXM150x_RESPONSE_OK){
                res->m_result = CXM150x_RESPONSE_OK;
            } else {
                res->m_result = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! set CXM150x to deepsleep state
/*!
 *
 * @param [in] param: Number of seconds to release
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_sys_to_deep_sleep (uint32_t param, CmdResSetCXM150xSysToDeepSleep *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< SYS TO_DSLP SET 600
    //> SYS TO_DSLP SET OK

    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %ld\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_SYS_TO_DSLP,CXM150x_COMMAND_SET,param);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_set_CXM150x_sys_to_deep_sleep,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_set_CXM150x_sys_to_deep_sleep(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Register to deepsleep event callback function
/*!
 *
 * @param [in] info: to deepsleep event information storage location
 * @param [in] func: to deepsleep event callback function
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_sys_to_deepsleep_callback_func_p: To deeepsleep event occurrence callback function pointer
 *        [out] g_sys_to_deepsleep_event_info: Pointer to deepsleep event information
 *
 * @return Callback function set immediately before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_sys_to_deep_sleep_event(CXM150xSysToDeepSleepInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_func = g_sys_to_deepsleep_callback_func_p;
    
    g_sys_to_deepsleep_callback_func_p = func;
    g_sys_to_deepsleep_event_info = info;
    
    return prev_func;
}



