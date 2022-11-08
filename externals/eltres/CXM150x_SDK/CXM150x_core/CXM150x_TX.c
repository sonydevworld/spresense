// ==========================================================================
/*!
* @file     CXM150x_TX.c
* @brief    CXM150x control API (TX group command)
* @date     2021/12/27
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

#include "CXM150x_TX.h"
#include "CXM150x_LIB.h"
#include "CXM150x_Port.h"
#include "CXM150x_Utility.h"

// Profile changing timeout time
#define MAX_SET_PROFILE_TIME_OUT_TICK_COUNT     (50000)

extern CXM150xTxState *g_tx_state_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_tx_prev_stt_callback_func_p;
extern CXM150xTxPoCEnableMessage *g_tx_poc_enable_message_event_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_tx_poc_enable_message_callback_func_p;
extern CXM150xTxPayloadInfo *g_tx_payload_event_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_tx_payload_event_callback_func_p;
extern CXM150xTxDutyEventInfo *g_tx_duty_event_info;
extern CXM150x_CALLBACK_FUNC_POINTER g_tx_duty_event_callback_func_p;

// ===========================================================================
//! Parse the result of setting the transmission data to be transmitted next
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
static void res_check_set_LPWA_payload(uint8_t *response,void *res_buf){
    CmdResSetCXM150xLPWAPayload *res = (CmdResSetCXM150xLPWAPayload*)res_buf;
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
//! Set payload data
/*!
 *
 * @param [in] paload: LPWA transmission data
 * @param [in] func: Response callback function
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_LPWA_payload(uint8_t *payload,CmdResSetCXM150xLPWAPayload *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX PLD SET ABCDEF00000000000000000000000000
    //> TX PLD SET OK
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    memset(res,0,sizeof(CmdResSetCXM150xLPWAPayload));
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s ",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_PLD,CXM150x_COMMAND_SET);
    
    // byte array to ASCII conversion
    for(uint32_t i=0;i<CXM150x_PAYLOAD_LEN;i++){
        uint8_t dt = *payload++;
        uint8_t add_ch[3];
        snprintf((char*)add_ch,3,"%X%X",dt>>4,dt&0x0F);
        strncat((char*)command,(char*)add_ch,3);
    }

    strncat((char*)command,"\r\n",3);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_LPWA_payload,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_LPWA_payload(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the result of getting the last sent data
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
static void res_check_get_prev_LPWA_tx_data(uint8_t *response,void *res_buf){
    CmdResGetCXM150xPrevLPWATxData *res = (CmdResGetCXM150xPrevLPWATxData*)res_buf;
    if(res != NULL){
        // Check the response message error from CXM (Error; Invalid Param, etc.)
        if(CXM150x_chk_response_error(response) != CXM150x_RESPONSE_OK){
            res->m_str[0] = '\0';
            return;
        }

        memset(res,'\0',sizeof(CmdResGetCXM150xPrevLPWATxData));
        
        // ASCII-> byte array conversion
        uint8_t rcv_str_last_word[CXM150x_RECEIVE_BUF_SIZE] = "";
        if(CXM150x_get_last_word(response,rcv_str_last_word) == CXM150x_RESPONSE_OK){
            CXM150x_ascii_to_bin(rcv_str_last_word,res->m_str,CXM150x_PAYLOAD_LEN*2);
        } else {
            res->m_str[0] = '\0';
        }
    }
}

// ===========================================================================
//! Get the last sent data
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
CXM150x_return_code get_CXM150x_prev_LPWA_tx_data(void *param,CmdResGetCXM150xPrevLPWATxData *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX PREV_PLD GET
    //> TX PREV_PLD GET 333535332E33353831333934372E3331

    CXM150x_return_code ret;
    uint8_t command[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    memset(res,0,sizeof(CmdResGetCXM150xPrevLPWATxData));
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_PREV_PLD,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_prev_LPWA_tx_data,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    res_check_get_prev_LPWA_tx_data(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the result of acquiring the transmission data to be transmitted next
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
static void res_check_get_LPWA_payload(uint8_t *response,void *res_buf){
    CmdResGetCXM150xLPWAPayload *res = (CmdResGetCXM150xLPWAPayload*)res_buf;
    if(res != NULL){
        // Check the response message error from CXM (Error; Invalid Param, etc.)
        if(CXM150x_chk_response_error(response) != CXM150x_RESPONSE_OK){
            res->m_str[0] = '\0';
            return;
        }

        memset(res,'\0',sizeof(CmdResGetCXM150xLPWAPayload));
        
        // ASCII-> byte array conversion
        uint8_t rcv_str_last_word[CXM150x_RECEIVE_BUF_SIZE] = "";
        if(CXM150x_get_last_word(response,rcv_str_last_word) == CXM150x_RESPONSE_OK){
            CXM150x_ascii_to_bin(rcv_str_last_word,res->m_str,CXM150x_PAYLOAD_LEN*2);
        } else {
            res->m_str[0] = '\0';
        }
    }
}

// ===========================================================================
//! Acquisition of transmission data to be transmitted next
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
CXM150x_return_code get_CXM150x_LPWA_payload(void *param,CmdResGetCXM150xLPWAPayload *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX PLD GET
    //> TX PLD GET 30303030303030303030303030303030

    CXM150x_return_code ret;
    uint8_t command[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    memset(res,0,sizeof(CmdResGetCXM150xLPWAPayload));
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_PLD,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_LPWA_payload,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_LPWA_payload(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Registration of Tx previous state event notification callback function
/*!
 *
 * @param [in] info: Event data structure
 * @param [in] func: Event notification callback function
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_tx_prev_stt_callback_func_p: Tx previous state event notification callback function pointer
 *        [out] g_tx_state_info: Pointer to Tx previous state evetn information structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_tx_state_event(CXM150xTxState *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_p = g_tx_prev_stt_callback_func_p;
    g_tx_prev_stt_callback_func_p = func;
    g_tx_state_info = info;
    
    return prev_p;
}

// ===========================================================================
//! Parse the setting result of Tx previuous state event notification
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
static void res_check_set_tx_state_event(uint8_t *response,void *res_buf){
    CmdResSetCXM150xTxStateEvent *res = (CmdResSetCXM150xTxStateEvent*)res_buf;
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
//! Setting of Tx prevoious state event notification
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
CXM150x_return_code set_CXM150x_tx_state_event(uint32_t on_off,CmdResSetCXM150xTxStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX PREV_STT SET_EVT ON
    //> TX PREV_STT SET_EVT OK

    CXM150x_return_code ret;
    uint8_t command[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    memset(res,0,sizeof(CmdResSetCXM150xTxStateEvent));
    
    // Create command string
    if(on_off == EVENT_ON){
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_PREV_STT,CXM150x_COMMAND_SET_EVT,CXM150x_COMMAND_ON);
    } else {
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_PREV_STT,CXM150x_COMMAND_SET_EVT,CXM150x_COMMAND_OFF);
    }
    
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_tx_state_event,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_tx_state_event(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the Tx previous state event notification setting result
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
static void res_check_get_tx_state_event(uint8_t *response,void *res_buf){
    CmdResGetCXM150xTxStateEvent *res = (CmdResGetCXM150xTxStateEvent*)res_buf;
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
//! Acquisition of Tx previous state event notification settings
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
CXM150x_return_code get_CXM150x_tx_state_event(void *param,CmdResGetCXM150xTxStateEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX PREV_STT GET_EVT
    //> TX PREV_STT GET_EVT ON
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_PREV_STT,CXM150x_COMMAND_GET_EVT);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_tx_state_event,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_tx_state_event(response,res);
    
    return RETURN_OK;
}


// ===========================================================================
//! Analysis of transmission result message
/*!
 *
 * @param [in] msg: CXM150x message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return transmission result
*/
// ===========================================================================
CXM150xTxState conv_tx_stt_message_to_code(uint8_t *msg){
    uint8_t last_word[CXM150x_MAX_COMMAND_LEN] = "";
    CXM150x_get_last_word(msg,(uint8_t*)last_word);
    // At first, check the message if the last word is "CS_*"
    if(strstr((char*)last_word,"CS_OK")){
        return TX_PREV_STT_CS_OK;
    } else if(strstr((char*)last_word,"CS_NG")){
        return TX_PREV_STT_CS_NG;
    } else if(strstr((char*)last_word,"OK")){
        return TX_PREV_STT_OK;
    } else if(strstr((char*)last_word,"NG")){
        return TX_PREV_STT_NG;
    } else {
        // If none of the above apply, return NG. (temporary implementation)
        return TX_PREV_STT_NG;
    }
}

// ===========================================================================
//! Parse the result of the previous transmission start time
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
static void res_check_get_prev_LPWA_tx_time(uint8_t *response,void *res_buf){
    CmdResGetCXM150xPrevLPWATxTime *res = (CmdResGetCXM150xPrevLPWATxTime*)res_buf;
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
//! Get last transmission start time
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
CXM150x_return_code get_CXM150x_prev_LPWA_tx_time(void *param,CmdResGetCXM150xPrevLPWATxTime *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX PREV_TIME GET
    //> TX PREV_TIME GET 0x7D044AFC

    CXM150x_return_code ret;
    uint8_t command[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    memset(res,0,sizeof(CmdResGetCXM150xPrevLPWATxTime));
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_PREV_TIME,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_prev_LPWA_tx_time,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_prev_LPWA_tx_time(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the result of acquiring the next transmission start time
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
static void res_check_get_next_LPWA_tx_time(uint8_t *response,void *res_buf){
    CmdResGetCXM150xNextLPWATxTime *res = (CmdResGetCXM150xNextLPWATxTime*)res_buf;
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
//! Get next transmission start time
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
CXM150x_return_code get_CXM150x_next_LPWA_tx_time(void *param,CmdResGetCXM150xNextLPWATxTime *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX NEXT_TIME GET
    //> TX NEXT_TIME GET 0x7D045204

    CXM150x_return_code ret;
    uint8_t command[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    memset(res,0,sizeof(CmdResGetCXM150xNextLPWATxTime));
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_NEXT_TIME,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_next_LPWA_tx_time,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_next_LPWA_tx_time(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the control profile setting result
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
static void res_check_set_CXM150x_tx_profile(uint8_t *response,void *res_buf){
    CmdResSetCXM150xTxProfile *res = (CmdResSetCXM150xTxProfile*)res_buf;
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
//! Set the control profile
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
CXM150x_return_code set_CXM150x_tx_profile(uint32_t param, CmdResSetCXM150xTxProfile *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX CUR_FRM_TYPE SET P,1
    //> TX CUR_FRM_TYPE SET OK

    CXM150x_return_code ret;
    uint8_t command[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    uint8_t *mode = NULL;
    switch(param){
        case TX_CUR_FRM_TYPE_PERIODIC_1:        //periodic1
            mode = (uint8_t*)CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P1;
            break;
        case TX_CUR_FRM_TYPE_PERIODIC_2:        //periodic2
            mode = (uint8_t*)CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P2;
            break;
        case TX_CUR_FRM_TYPE_EVENT:             //event
            mode = (uint8_t*)CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_E;
            break;
        case TX_CUR_FRM_TYPE_PERIODIC:          //periodic
            mode = (uint8_t*)CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P;
            break;
        default:
            // Returns NG if none of the above apply
            return RETURN_NG;
    }
    
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_CUR_FRM_TYPE,CXM150x_COMMAND_SET,mode);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback_long_wait(command,func,res_check_set_CXM150x_tx_profile,res,MAX_SET_PROFILE_TIME_OUT_TICK_COUNT);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response_long_wait(command,response,MAX_SET_PROFILE_TIME_OUT_TICK_COUNT);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_tx_profile(response,res);

    return RETURN_OK;
}




// ===========================================================================
//! Parse the control profile acquisition result
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
static void res_check_get_CXM150x_tx_profile(uint8_t *response,void *res_buf){
    CmdResGetCXM150xTxProfile *res = (CmdResGetCXM150xTxProfile*)res_buf;
    if(res != NULL){
        // Check the response message error from CXM (Error; Invalid Param, etc.)
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            
            // Determine response mesage
            uint8_t last_word[CXM150x_MAX_COMMAND_LEN] = "";
            if(CXM150x_get_last_word(response,last_word) == CXM150x_RESPONSE_NG){
                res->m_num = CXM150x_RESPONSE_ERROR;
            } else {
                // Note: "CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P1" , "CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P2"
                // should be judged before "CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P" (because of partial match)
                if(strstr((char*)last_word,CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P1)){
                    res->m_num = TX_CUR_FRM_TYPE_PERIODIC_1;
                } else if(strstr((char*)last_word,CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P2)){
                    res->m_num = TX_CUR_FRM_TYPE_PERIODIC_2;
                } else if(strstr((char*)last_word,CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_E)){
                    res->m_num = TX_CUR_FRM_TYPE_EVENT;
                } else if(strstr((char*)last_word,CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P)){
                    res->m_num = TX_CUR_FRM_TYPE_PERIODIC;
                } else {
                    res->m_num = CXM150x_RESPONSE_ERROR;
                }
            }
        } else {
            res->m_num = CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Get control profile
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
CXM150x_return_code get_CXM150x_tx_profile(void* param, CmdResGetCXM150xTxProfile *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX CUR_FRM_TYPE GET
    //> TX CUR_FRM_TYPE GET P,1

    CXM150x_return_code ret;
    uint8_t command[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_CUR_FRM_TYPE,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_tx_profile,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_tx_profile(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse PoC format transmission setting result
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
static void res_check_set_CXM150x_tx_PoC_enable(uint8_t *response,void *res_buf){
    CmdResSetCXM150xTxPoCEnable *res = (CmdResSetCXM150xTxPoCEnable*)res_buf;
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
//! Set PoC format transmission
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
CXM150x_return_code set_CXM150x_tx_PoC_enable(void *param, CmdResSetCXM150xTxPoCEnable *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX POC_EN SET ON
    //> TX POC_EN SET OK
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_RECEIVE_BUF_SIZE] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_POC_EN,CXM150x_COMMAND_SET,CXM150x_COMMAND_ON);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_tx_PoC_enable,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_tx_PoC_enable(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse tx duty set event result
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
static void res_check_set_CXM150x_tx_duty_event(uint8_t *response,void *res_buf){
    CmdResSetCXM150xTxDutyEvent *res = (CmdResSetCXM150xTxDutyEvent*)res_buf;
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
//! Set tx duty set event transmission
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
CXM150x_return_code set_CXM150x_tx_duty_event(uint32_t on_off, CmdResSetCXM150xTxDutyEvent *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX DUTY SET_EVT ON
    //> TX DUTY SET_EVT OK
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_RECEIVE_BUF_SIZE] = "";
    
    
    // Create command string
    if(on_off == EVENT_ON){
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_DUTY,CXM150x_COMMAND_SET_EVT,CXM150x_COMMAND_ON);
    } else {
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_DUTY,CXM150x_COMMAND_SET_EVT,CXM150x_COMMAND_OFF);
    }

    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_tx_duty_event,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }

    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_tx_duty_event(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the tx duty event notification setting result
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
static void res_check_get_tx_duty_event(uint8_t *response,void *res_buf){
    CmdResGetCXM150xTxDutyEvent *res = (CmdResGetCXM150xTxDutyEvent*)res_buf;
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
//! Acquisition of Tx duty event notification settings
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
CXM150x_return_code get_CXM150x_tx_duty_event(void *param,CmdResGetCXM150xTxDutyEvent *res,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TX DUTY GET_EVT
    //> TX DUTY GET_EVT ON
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_RECEIVE_BUF_SIZE] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TX_DUTY,CXM150x_COMMAND_GET_EVT);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_tx_duty_event,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_tx_duty_event(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Registration of tx poc enable message callback function
/*!
 *
 * @param [in] info: Event data structure
 * @param [in] func: PoC enable message event callback function
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_tx_poc_enable_message_callback_func_p: PoC enable message event callback function pointer
 *        [out] g_tx_poc_enable_message_event_info: Pointer to Tx PoC enable message evetn information structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_tx_PoC_enable_message_event(CXM150xTxPoCEnableMessage *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_p = g_tx_poc_enable_message_callback_func_p;
    g_tx_poc_enable_message_callback_func_p = func;
    g_tx_poc_enable_message_event_info = info;
    
    return prev_p;
}

// ===========================================================================
//! Registration of payload event callback function
/*!
 *
 * @param [in] info: Event data structure
 * @param [in] func: Payload event callback function
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_tx_payload_event_callback_func_p: TX payload event callback function pointer
 *        [out] g_tx_payload_event_info: Pointer to TX payload evetn information structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_tx_payload_event(CXM150xTxPayloadInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_p = g_tx_payload_event_callback_func_p;
    g_tx_payload_event_callback_func_p = func;
    g_tx_payload_event_info = info;
    
    return prev_p;
}

// ===========================================================================
//! Registration of tx duty event callback function
/*!
 *
 * @param [in] info: Event data structure
 * @param [in] func: Duty event callback function
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_tx_duty_event_callback_func_p:TX Duty event callback function pointer
 *        [out] g_tx_duty_event_info: Pointer to Tx Duty event information structure
 *
 * @return Callback function pointer set before
*/
// ===========================================================================
CXM150x_CALLBACK_FUNC_POINTER register_CXM150x_tx_duty_event(CXM150xTxDutyEventInfo *info,CXM150x_CALLBACK_FUNC_POINTER func){
    CXM150x_CALLBACK_FUNC_POINTER prev_p = g_tx_duty_event_callback_func_p;
    g_tx_duty_event_callback_func_p = func;
    g_tx_duty_event_info = info;
    
    return prev_p;
}
