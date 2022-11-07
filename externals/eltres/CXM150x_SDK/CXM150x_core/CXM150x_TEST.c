// ==========================================================================
/*!
* @file     CXM150x_TEST.c
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "CXM150x_TEST.h"
#include "CXM150x_LIB.h"
#include "CXM150x_Utility.h"

// Compile only if certain symbols are defined in CXM150x_APITypedef.h
#if CXM150x_TEST_MODE_API_USE

// ===========================================================================
//! Parse the transmission test channel setting result
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
static void res_check_set_CXM150x_test_tx_ch(uint8_t *response,void *res_buf){
    CmdResSetCXM150xTestTxCh *res = (CmdResSetCXM150xTestTxCh*)res_buf;
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
//! Set the channel used in transmission test mode
/*! 
 *
 * @param [in] param: channel
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_test_tx_ch(uint32_t param, CmdResSetCXM150xTestTxCh *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TEST TX_CH SET 27
    //> TEST TX_CH SET OK
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %02lX\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_TX_CH,CXM150x_COMMAND_SET,param);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_test_tx_ch,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_test_tx_ch(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the transmission test channel acquisition result
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
static void res_check_get_CXM150x_test_tx_ch(uint8_t *response,void *res_buf){
    CmdResGetCXM150xTestTxCh *res = (CmdResGetCXM150xTestTxCh*)res_buf;
    uint8_t ch_str[CXM150x_MAX_COMMAND_LEN] = "";
    // Parse CXM150x response message
    if(res != NULL){
        // Whether the message ends in 'OK' or not
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_get_last_word(response,ch_str) == CXM150x_RESPONSE_OK){
                if(sscanf((char*)ch_str,"%02lX",&res->m_num) == 0){
                    res->m_num = CXM150x_RESPONSE_ERROR;
                }
            } else {
                //Acquisition failure
                res->m_num = CXM150x_RESPONSE_ERROR;
            }
        } else {
            res->m_num = CXM150x_RESPONSE_ERROR;
        }
    }
}

// ===========================================================================
//! Get the channel used in transmission test mode
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
CXM150x_return_code get_CXM150x_test_tx_ch(void* param, CmdResGetCXM150xTestTxCh *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TEST TX_CH GET
    //> TEST TX_CH GET 01
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_TX_CH,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_test_tx_ch,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_test_tx_ch(response,res);
    
    return RETURN_OK;
}


// ===========================================================================
//! Parse the transmission test operation mode setting result
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
static void res_check_set_CXM150x_test_tx_mode(uint8_t *response,void *res_buf){
    CmdResSetCXM150xTestTxMode *res = (CmdResSetCXM150xTestTxMode*)res_buf;
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
//! Set the transmission test operation mode
/*! 
 *
 * @param [in] mode: Mode
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_test_tx_mode(uint32_t mode, CmdResSetCXM150xTestTxMode *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TEST TX_MODE SET 00
    //> TEST TX_MODE SET OK
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %02lX\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_TX_MODE,CXM150x_COMMAND_SET,mode);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_test_tx_mode,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_test_tx_mode(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! Parse the transmission test operation mode acquisition result
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
static void res_check_get_CXM150x_test_tx_mode(uint8_t *response,void *res_buf){
    CmdResGetCXM150xTestTxMode *res = (CmdResGetCXM150xTestTxMode*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        uint8_t mode_str[CXM150x_MAX_COMMAND_LEN] = "";
         if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_get_last_word(response,mode_str) == CXM150x_RESPONSE_OK){
                if(sscanf((char*)mode_str,"%02lX",&res->m_num) == 0){
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
//! Get the transmission test operation mode
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
CXM150x_return_code get_CXM150x_test_tx_mode(void* param, CmdResGetCXM150xTestTxMode *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TEST TX_MODE GET
    //> TEST TX_MODE GET 00
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_TX_MODE,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_test_tx_mode,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_test_tx_mode(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the result of setting the transmission test operation status
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
static void res_check_set_CXM150x_test_tx_run(uint8_t *response,void *res_buf){
    CmdResSetCXM150xTestTxRun *res = (CmdResSetCXM150xTestTxRun*)res_buf;
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
//! Set the transmission test operation status
/*! 
 *
 * @param [in] on_off: Operation status
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_test_tx_run(uint32_t on_off, CmdResSetCXM150xTestTxRun *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TEST TX_RUN SET ON
    //> TEST TX_RUN SET OK
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    if(on_off == CXM150x_TX_TEST_RUN_ON){
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_TX_RUN,CXM150x_COMMAND_SET,CXM150x_COMMAND_ON);
    } else {
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_TX_RUN,CXM150x_COMMAND_SET,CXM150x_COMMAND_OFF);
    }
    
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_test_tx_run,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_test_tx_run(response,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the transmission test operation status acquisition result
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
static void res_check_get_CXM150x_test_tx_run(uint8_t *response,void *res_buf){
    CmdResGetCXM150xTestTxRun *res = (CmdResGetCXM150xTestTxRun*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            if(CXM150x_check_last_on_off(response) == EVENT_ON){
                res->m_num = CXM150x_TX_TEST_RUN_ON;
            } else {
                res->m_num = CXM150x_TX_TEST_RUN_OFF;
            }
        } else {
            res->m_num = CXM150x_RESPONSE_ERROR;
        }
    }
}


// ===========================================================================
//! Get the transmission test operation status
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
CXM150x_return_code get_CXM150x_test_tx_run(void* param, CmdResGetCXM150xTestTxRun *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TEST TX_RUN GET
    //> TEST TX_RUN GET ON
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_TX_RUN,CXM150x_COMMAND_GET);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_test_tx_run,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_test_tx_run(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! GPI port status acquisition result parse
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
static void res_check_get_CXM150x_test_gpi_state(uint8_t *response,void *res_buf){
    CmdResGetCXM150xTestGPIState *res = (CmdResGetCXM150xTestGPIState*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            uint8_t last_word[CXM150x_MAX_COMMAND_LEN] = "";
            if(CXM150x_get_last_word(response,last_word) == CXM150x_RESPONSE_OK){
                if(last_word[0] == 'H'){
                    res->m_num = CXM150x_GPIO_PORT_STATE_H;
                } else if(last_word[0] == 'L'){
                    res->m_num = CXM150x_GPIO_PORT_STATE_L;
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
//! Get GPI port status
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
CXM150x_return_code get_CXM150x_test_gpi_state(uint32_t param, CmdResGetCXM150xTestGPIState *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TEST GPI GET 0
    //> TEST GPI GET L
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %ld\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_GPI,CXM150x_COMMAND_GET,param);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_test_gpi_state,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_test_gpi_state(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! GPO port status setting result parse
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
static void res_check_set_CXM150x_test_gpo_state(uint8_t *response,void *res_buf){
    CmdResSetCXM150xTestGPOState *res = (CmdResSetCXM150xTestGPOState*)res_buf;
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
//! Set GPO port status
/*! 
 *
 * @param [in] param: Pointer to GPO port setting structure
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_test_gpo_state(CXM150xSetGPOState *param, CmdResSetCXM150xTestGPOState *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TEST GPO SET 1,L
    //> TEST GPO SET OK
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    if(param == NULL){
        return RETURN_NG;
    }
    
    // Create command string
    if(param->m_port_state == CXM150x_GPIO_PORT_STATE_H){
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %ld,%s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_GPO,CXM150x_COMMAND_SET,param->m_port_type,CXM150x_COMMAND_GPIO_PORT_H);
    } else {
        snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %ld,%s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_GPO,CXM150x_COMMAND_SET,param->m_port_type,CXM150x_COMMAND_GPIO_PORT_L);
    }
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_set_CXM150x_test_gpo_state,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_set_CXM150x_test_gpo_state(response,res);
    
    return RETURN_OK;
}

// ===========================================================================
//! GPO port status acquisition result parsing
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
static void res_check_get_CXM150x_test_gpo_state(uint8_t *response,void *res_buf){
    CmdResGetCXM150xTestGPOState *res = (CmdResGetCXM150xTestGPOState*)res_buf;
    // Parse CXM150x response message
    if(res != NULL){
        if(CXM150x_chk_response_error(response) == CXM150x_RESPONSE_OK){
            uint8_t last_word[CXM150x_MAX_COMMAND_LEN] = "";
            if(CXM150x_get_last_word(response,last_word) == CXM150x_RESPONSE_OK){
                if(last_word[0] == 'H'){
                    res->m_num = CXM150x_GPIO_PORT_STATE_H;
                } else if(last_word[0] == 'L'){
                    res->m_num = CXM150x_GPIO_PORT_STATE_L;
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
//! Get GPO port status
/*! 
 *
 * @param [in] param: Port type
 * @param [in] func: Response callback function pointer
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_test_gpo_state(uint32_t param, CmdResGetCXM150xTestGPOState *res, CXM150x_CALLBACK_RESPONSE_FUNC_POINTER func){
    //< TEST GPO GET 1
    //> TEST GPO GET L
    
    CXM150x_return_code ret;
    uint8_t command[CXM150x_MAX_COMMAND_LEN] = "";
    uint8_t response[CXM150x_MAX_COMMAND_LEN] = "";
    
    // Create command string
    snprintf((char*)command,CXM150x_MAX_COMMAND_LEN,"%s %s %s %ld\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_COMMAND_TEST_GPO,CXM150x_COMMAND_GET,param);
    
    if(func != NULL){
        return CXM150x_send_and_register_callback(command,func,res_check_get_CXM150x_test_gpo_state,res);
    } else {
        // Send command and wait for response
        ret = CXM150x_send_and_wait_command_response(command,response);
    }
    
    if(ret != RETURN_OK){
        return ret;         // In case of BUSY or timeout, return here
    }
    
    res_check_get_CXM150x_test_gpo_state(response,res);
    
    return RETURN_OK;
}

#endif  // CXM150x_TEST_MODE_API_USE

