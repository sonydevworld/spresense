// ==========================================================================
/*!
* @file     CXM150x_GNSS_FW_UPDATE.c
* @brief    CXM150x control API (for GNSS FW UPDATE mode)
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

#include "CXM150x_APITypeDef.h"
#include "CXM150x_GNSS_FW_UPDATE.h"
#include "CXM150x_Port.h"
#include "CXM150x_GNSS_FW_UPDATE_Port.h"

// Compile only if certain symbols are defined in CXM150x_APITypedef.h
#if CXM150x_GNSS_FW_UPDATE_API_USE


// ===========================================================================
//! Parse the result of confirming that GNSS block FW update mode is activate
/*!
 *
 * @param [in] recv_buf: Received message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static CXM150x_return_code res_check_get_CXM150x_GNSS_fw_update_mode_check(uint8_t *recv_buf){
    if(strstr((char*)recv_buf,CXM150x_GNSS_FW_UPDATE_MODE_CHECK_MESSAGE)){
        return RETURN_OK;
    } else {
        return RETURN_NG;
    }
}

// ===========================================================================
//! Check if the CXM150x is in GNSS block FW update mode
/*!
 *
 * @param [in] param: NULL fixed
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_GNSS_fw_update_mode_check(void *param, CmdResGetGNSSFWUpdateModeCheck *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_GNSS_FW_UPDATE_SEND_MAX_LEN + 1] = "\r";
    uint8_t recv_buf[CXM150x_GNSS_FW_UPDATE_RECEIVE_MAX_LEN + 1] = "";
    uint8_t send_len = 1;

    // Communication success flag
    uint8_t timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_ON;
    uint32_t st_tick;
    uint32_t c_tick;
    
    // Disable the UART interrupt
    wrapper_CXM150x_GNSS_fw_update_uart_abort_IT();

    // wait for reception until timeout
    st_tick = wrapper_CXM150x_get_tick();
    c_tick = wrapper_CXM150x_get_tick();
    while((c_tick - st_tick) < GNSS_FW_UPDATE_MODE_CHANGE_TIME_OUT_TICK_COUNT){
        ret = wrapper_CXM150x_GNSS_fw_update_rx_message(recv_buf,GNSS_FW_UPDATE_MODE_CHECK_TIME_OUT_TICK_COUNT);
        if(ret == RETURN_OK){
            ret = res_check_get_CXM150x_GNSS_fw_update_mode_check(recv_buf);
            if(ret == RETURN_OK){
                timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_OFF;
                break;
            }
        }
        c_tick = wrapper_CXM150x_get_tick();
    }

    // Time out if "H>" could not be received.
    if(timeout_flag == CXM150x_GNSS_FW_UPDATE_API_FLAG_ON){
        printf_err("rx_message timeout .\r\n");
        return RETURN_TIMEOUT;
    }

    // send command
    ret = wrapper_CXM150x_GNSS_fw_update_tx_message(send_buf,send_len,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    if(ret != RETURN_OK){
        return ret;
    }
    
    // timeout if command transmission failed
    if(ret != RETURN_OK){
        return ret;
    }

    timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_ON;
    // wait for reception until timeout
    st_tick = wrapper_CXM150x_get_tick();
    c_tick = wrapper_CXM150x_get_tick();
    while((c_tick - st_tick) < GNSS_FW_UPDATE_MODE_CHECK_TIME_OUT_TICK_COUNT){
        ret = wrapper_CXM150x_GNSS_fw_update_rx_message(recv_buf,GNSS_FW_UPDATE_MODE_CHECK_TIME_OUT_TICK_COUNT);
        if(ret == RETURN_OK){
            ret = res_check_get_CXM150x_GNSS_fw_update_mode_check(recv_buf);
            if(ret == RETURN_OK){
                res->m_result = CXM150x_RESPONSE_OK;
                timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_OFF;
                break;
            }else{
                res->m_result = CXM150x_RESPONSE_NG;
            }
        }
        c_tick = wrapper_CXM150x_get_tick();
    }

    // Time out if "H>" could not be received.
    if(timeout_flag == CXM150x_GNSS_FW_UPDATE_API_FLAG_ON){
        printf_err("rx_message timeout .\r\n");
        return RETURN_TIMEOUT;
    }

    return RETURN_OK;
}


// ===========================================================================
//! Response check of GNSS firmware update header file command
/*!
 *
 * @param [in] recv_buf: Received message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return confirmation result
*/
// ===========================================================================
static CXM150x_return_code res_check_set_CXM150x_GNSS_fw_update_header_data(uint8_t *recv_buf){
    if(strstr((char*)recv_buf,CXM150x_GNSS_FW_UPDATE_MODE_SEND_HEADER_CMD_RESPONSE)){
        return RETURN_OK;
    } else {
        return RETURN_NG;
    }
}

// ===========================================================================
//! Confirm response of GNSS firmware update data transmission
/*!
 *
 * @param [in] recv_buf: Received message
 * @param[out]   
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return confirmation result
*/
// ===========================================================================
static CXM150x_return_code res_check_set_CXM150x_GNSS_fw_update_data_send(uint8_t *recv_buf){
    if(strstr((char*)recv_buf,CXM150x_GNSS_FW_UPDATE_MODE_DATA_SEND_RESPONSE)){
        return RETURN_OK;
    } else {
        return RETURN_NG;
    }
}

// ===========================================================================
//! Send GNSS firmware update header data
/*!
 *
 * @param [in] param: Firmware header file binary image
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_GNSS_fw_update_header_data(uint8_t* param, CmdResSetGNSSFWUpdateHeaderData *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_GNSS_FW_UPDATE_SEND_MAX_LEN + 1] = "";
    uint8_t recv_buf[CXM150x_GNSS_FW_UPDATE_RECEIVE_MAX_LEN + 1] = "";
    
    uint32_t send_len = 0;
    
    // Communication success flag
    uint8_t timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_ON;
    uint32_t st_tick;
    uint32_t c_tick;
    
    // Send command
    strncpy((char *)send_buf, CXM150x_GNSS_FW_UPDATE_MODE_SEND_HEADER_CMD, CXM150x_GNSS_FW_UPDATE_MODE_SEND_HEADER_CMD_LEN);
    send_buf[CXM150x_GNSS_FW_UPDATE_MODE_SEND_HEADER_CMD_LEN] = '\0';
    send_len = CXM150x_GNSS_FW_UPDATE_MODE_SEND_HEADER_CMD_LEN;
    ret = wrapper_CXM150x_GNSS_fw_update_tx_message(send_buf,send_len,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);

    // timeout if command transmission failed
    if(ret != RETURN_OK){
        return ret;
    }

    // wait for reception until timeout
    st_tick = wrapper_CXM150x_get_tick();
    c_tick = wrapper_CXM150x_get_tick();
    while((c_tick - st_tick) < GNSS_FW_UPDATE_MODE_CHECK_TIME_OUT_TICK_COUNT){
        ret = wrapper_CXM150x_GNSS_fw_update_rx_message(recv_buf,GNSS_FW_UPDATE_MODE_CHECK_TIME_OUT_TICK_COUNT);
        if(ret == RETURN_OK){
            ret = res_check_set_CXM150x_GNSS_fw_update_header_data(recv_buf);
            if(ret == RETURN_OK){
                timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_OFF;
                break;
            }
        }
        c_tick = wrapper_CXM150x_get_tick();
    }

    // Timeout if "LOADH" could not be received
    if(timeout_flag == CXM150x_GNSS_FW_UPDATE_API_FLAG_ON){
        printf_err("rx_message timeout .\r\n");
        return RETURN_TIMEOUT;
    }
    
    // Send data
    send_len = CXM150x_GNSS_FW_UPDATE_SEND_HEADER_LEN;
    memcpy(send_buf,param,send_len);
    
    ret = wrapper_CXM150x_GNSS_fw_update_tx_message(send_buf,send_len,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    
    // timeout if data transmission failed
    if(ret != RETURN_OK){
        return ret;
    }
    // wait for reception until timeout
    timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_ON;
    st_tick = wrapper_CXM150x_get_tick();
    c_tick = wrapper_CXM150x_get_tick();
    while((c_tick - st_tick) < GNSS_FW_UPDATE_SEND_FILE_TIME_OUT_TICK_COUNT){
        ret = wrapper_CXM150x_GNSS_fw_update_rx_message(recv_buf,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
        if(ret == RETURN_OK){
            ret = res_check_set_CXM150x_GNSS_fw_update_data_send(recv_buf);
            if(res != NULL){
                if(ret == RETURN_OK){
                    res->m_result = CXM150x_RESPONSE_OK;
                    timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_OFF;
                    break;
                } else {
                    res->m_result = CXM150x_RESPONSE_NG;
                }
            }
        }
        c_tick = wrapper_CXM150x_get_tick();
    }
    
    // time out if OK was not received
    if(timeout_flag == CXM150x_GNSS_FW_UPDATE_API_FLAG_ON){
        printf_err("rx_message timeout .\r\n");
        return RETURN_TIMEOUT;
    }
    
    return RETURN_OK;
}

// ===========================================================================
//! Confirm response to GNSS firmware update code transmission command
/*!
 *
 * @param [in] recv_buf: Received message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return confirmation result
*/
// ===========================================================================
static CXM150x_return_code res_check_set_CXM150x_GNSS_fw_update_code_data(uint8_t *recv_buf){
    if(strstr((char*)recv_buf,CXM150x_GNSS_FW_UPDATE_MODE_SEND_CODE_CMD_RESPONSE)){
        return RETURN_OK;
    } else {
        return RETURN_NG;
    }
}

// ===========================================================================
//! GNSS firmware update code data transmission
/*!
 *
 * @param [in] param: GNSS firmware binary image structure
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_GNSS_fw_update_code_data(CXM150xGNSSFWUpdateCodeData* param, CmdResSetGNSSFWUpdateCodeData *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_GNSS_FW_UPDATE_SEND_MAX_LEN + 1] = "";
    uint8_t recv_buf[CXM150x_GNSS_FW_UPDATE_RECEIVE_MAX_LEN + 1] = "";
    
    uint32_t send_len = 0;
    
    uint8_t timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_ON;
    uint32_t st_tick;
    uint32_t c_tick;
    
    // Communication success flag
    timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_ON;
    
    // length check
    if(param->m_data_len == 0 || param->m_data_len > CXM150x_GNSS_FW_UPDATE_SEND_MAX_LEN){
        return RETURN_NG;
    }
    
    if(param->m_data_pos_flag == CODE_DATA_FIRST){
        // Send command
        strncpy((char *)send_buf, CXM150x_GNSS_FW_UPDATE_MODE_SEND_CODE_CMD, CXM150x_GNSS_FW_UPDATE_MODE_SEND_CODE_CMD_LEN);
        send_buf[CXM150x_GNSS_FW_UPDATE_MODE_SEND_CODE_CMD_LEN] = '\0';
        send_len = CXM150x_GNSS_FW_UPDATE_MODE_SEND_CODE_CMD_LEN;
        ret = wrapper_CXM150x_GNSS_fw_update_tx_message(send_buf,send_len,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);

        // timeout if command transmission failed
        if(ret != RETURN_OK){
            return ret;
        }

        // wait for reception until timeout
        st_tick = wrapper_CXM150x_get_tick();
        c_tick = wrapper_CXM150x_get_tick();
        while((c_tick - st_tick) < GNSS_FW_UPDATE_MODE_CHECK_TIME_OUT_TICK_COUNT){
            ret = wrapper_CXM150x_GNSS_fw_update_rx_message(recv_buf,GNSS_FW_UPDATE_MODE_CHECK_TIME_OUT_TICK_COUNT);
            if(ret == RETURN_OK){
                ret = res_check_set_CXM150x_GNSS_fw_update_code_data(recv_buf);
                if(ret == RETURN_OK){
                    timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_OFF;
                    break;
                }
            }
            c_tick = wrapper_CXM150x_get_tick();
        }

        // Time out if "LOADC" could not be received
        if(timeout_flag == CXM150x_GNSS_FW_UPDATE_API_FLAG_ON){
            printf_err("rx_message timeout .\r\n");
            return RETURN_TIMEOUT;
        }

        // Send data
        send_len = param->m_data_len;
        memcpy(send_buf,param->m_data,send_len);
        
        ret = wrapper_CXM150x_GNSS_fw_update_tx_message(send_buf,send_len,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
        
        // timeout if data transmission failed
        if(ret != RETURN_OK){
            return ret;
        }
        
        res->m_result = CXM150x_RESPONSE_OK;
    }else if(param->m_data_pos_flag == CODE_DATA_MID){
        // Send data
        send_len = param->m_data_len;
        memcpy(send_buf,param->m_data,send_len);
        
        ret = wrapper_CXM150x_GNSS_fw_update_tx_message(send_buf,send_len,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
        
        // timeout if data transmission failed
        if(ret != RETURN_OK){
            return ret;
        }
        
        res->m_result = CXM150x_RESPONSE_OK;
    }else if(param->m_data_pos_flag == CODE_DATA_LAST){
        // Send data
        send_len = param->m_data_len;
        memcpy(send_buf,param->m_data,send_len);
        
        ret = wrapper_CXM150x_GNSS_fw_update_tx_message(send_buf,send_len,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
        
        // timeout if data transmission failed
        if(ret != RETURN_OK){
            return ret;
        }
        
        // wait for reception until timeout
        st_tick = wrapper_CXM150x_get_tick();
        c_tick = wrapper_CXM150x_get_tick();
        while((c_tick - st_tick) < GNSS_FW_UPDATE_SEND_FILE_TIME_OUT_TICK_COUNT){
            ret = wrapper_CXM150x_GNSS_fw_update_rx_message(recv_buf,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
            if(ret == RETURN_OK){
                ret = res_check_set_CXM150x_GNSS_fw_update_data_send(recv_buf);
                if(res != NULL){
                    if(ret == RETURN_OK){
                        res->m_result = CXM150x_RESPONSE_OK;
                        timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_OFF;
                        break;
                    } else {
                        res->m_result = CXM150x_RESPONSE_NG;
                    }
                }
            }
            c_tick = wrapper_CXM150x_get_tick();
        }
    
        // time out if OK was not received
        if(timeout_flag == CXM150x_GNSS_FW_UPDATE_API_FLAG_ON){
            printf_err("rx_message timeout .\r\n");
            return RETURN_TIMEOUT;
        }
    }else{
        return RETURN_NG;
    }
    return RETURN_OK;
}

// ===========================================================================
//! Parse the update result of GNSS firmware update
/*!
 *
 * @param [in] recv_buf: Received message
 * @param [out] res: parse result
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void res_check_get_CXM150x_GNSS_fw_update_result(uint8_t *recv_buf,CmdResGetGNSSFWUpdateResult *res){
    if(res != NULL){
        if(strncmp((char*)recv_buf,CXM150x_GNSS_FW_UPDATE_MODE_UPDATE_COMPLETE_MESSAGE,strlen(CXM150x_GNSS_FW_UPDATE_MODE_UPDATE_COMPLETE_MESSAGE)) == 0){
            res->m_result = CXM150x_RESPONSE_OK;
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! Check the update result of GNSS firmware update
/*!
 *
 * @param [in] param: NULL fixed
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_GNSS_fw_update_result(void* param, CmdResGetGNSSFWUpdateResult *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_GNSS_FW_UPDATE_SEND_MAX_LEN + 1] = "";
    uint8_t recv_buf[CXM150x_GNSS_FW_UPDATE_RECEIVE_MAX_LEN + 1] = "";
    
    uint8_t send_len = 0;
    
    // Communication success flag
    uint8_t timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_ON;
    uint32_t st_tick;
    uint32_t c_tick;
    
    // Send command
    strncpy((char *)send_buf, CXM150x_GNSS_FW_UPDATE_MODE_EXEC_CMD, CXM150x_GNSS_FW_UPDATE_MODE_EXEC_CMD_LEN);
    send_buf[CXM150x_GNSS_FW_UPDATE_MODE_EXEC_CMD_LEN] = '\0';
    send_len = CXM150x_GNSS_FW_UPDATE_MODE_EXEC_CMD_LEN;
    ret = wrapper_CXM150x_GNSS_fw_update_tx_message(send_buf,send_len,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);

    // timeout if command transmission failed
    if(ret != RETURN_OK){
        return ret;
    }

    // wait for reception until timeout
    st_tick = wrapper_CXM150x_get_tick();
    c_tick = wrapper_CXM150x_get_tick();
    while((c_tick - st_tick) <  GNSS_FW_UPDATE_RESULT_TIME_OUT_TICK_COUNT){
        ret = wrapper_CXM150x_GNSS_fw_update_rx_message(recv_buf,GNSS_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
        if(ret == RETURN_OK){
            res_check_get_CXM150x_GNSS_fw_update_result(recv_buf,res);
            if(res->m_result == CXM150x_RESPONSE_OK){
                timeout_flag = CXM150x_GNSS_FW_UPDATE_API_FLAG_OFF;
                break;
            }
        }
        c_tick = wrapper_CXM150x_get_tick();
    }
    
    // time out if no success was received
    if(timeout_flag == CXM150x_GNSS_FW_UPDATE_API_FLAG_ON){
        printf_err("rx_message timeout .\r\n");
        return RETURN_TIMEOUT;
    }
    
    return RETURN_OK;
}

#endif // CXM150x_GNSS_FW_UPDATE_API_USE

