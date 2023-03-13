// ==========================================================================
/*!
* @file     CXM150x_CTRL_FW_UPDATE.c
* @brief    CXM150x control API (for CONTROL FW UPDATE mode)
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
#include "CXM150x_CTRL_FW_UPDATE.h"
#include "CXM150x_Port.h"
#include "CXM150x_CTRL_FW_UPDATE_Port.h"
#include "CXM150x_Utility.h"

// Compile only if certain symbols are defined in CXM150x_APITypedef.h
#if CXM150x_CTRL_FW_UPDATE_API_USE


// ===========================================================================
//! Create transmission message in CONTROL FW UPDATE mode
/*!
 *
 * @param [in] cmd: Command
 * @param [in] data: Data content of DATA section
 * @param [in] data_len: Number of bytes in DATA section
 * @param [out] send_buf: Buffer to store created send command
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return message length
*/
// ===========================================================================
static uint8_t create_tx_data(uint8_t cmd,uint8_t *data,uint8_t data_len,uint8_t *send_buf){
    uint32_t sum = 0;
    uint8_t msg_len;
    send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_SD0] = CXM150x_CTRL_FW_UPDATE_API_SD;
    send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_LE0] = data_len;
    send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_LE1] = data_len;
    send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_SD1] = CXM150x_CTRL_FW_UPDATE_API_SD;
    send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_RB] = CXM150x_CTRL_FW_UPDATE_API_RB_FROM_HOST;
    send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_NUL] = CXM150x_CTRL_FW_UPDATE_API_NUL;
    send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_CMD] = cmd;
    
    // Checksum calculation
    sum = send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_RB]
         + send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_NUL]
         + send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_CMD];
    
    // Set DATA section and calculate checksum
    for(uint32_t i=0;i<data_len;i++){
        send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA+i] = data[i];
        sum += data[i];
    }
    
    // Set lower 8 bits of RB + NUL + CMD + DATA
    send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA + data_len ] = (uint8_t)(sum & 0x000000FF);
    send_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA + data_len + 1] = CXM150x_CTRL_FW_UPDATE_API_ED;
    
    msg_len = CXM150x_CTRL_FW_UPDATE_API_MSG_INF_LEN + data_len;
    
    // return message length
    return msg_len;

}

// ===========================================================================
//! Convert the byte array received in little endian format to uint32_t format
/*!
 *
 * @param [in] little_endian_bytes: Little endian byte array to be converted
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return Value converted to uint32_t
*/
// ===========================================================================
static uint32_t conv_uint32_big_endian(uint8_t *little_endian_bytes){
    return (little_endian_bytes[3] << 24) + (little_endian_bytes[2] << 16) + (little_endian_bytes[1] << 8) + little_endian_bytes[0];
}

// ===========================================================================
//! Convert numeric value in uint32_t format to byte array in little endian format
/*!
 *
 * @param [in] big_endian_val: uint32_t value to be converted
 * @param [out] little_endian_bytes: byte array after conversion to little endian format
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void conv_uint32_little_endian(uint32_t big_endian_val,uint8_t *little_endian_bytes){
    little_endian_bytes[0] = (big_endian_val & 0x000000FF);
    little_endian_bytes[1] = (big_endian_val & 0x0000FF00) >> 8;
    little_endian_bytes[2] = (big_endian_val & 0x00FF0000) >> 16;
    little_endian_bytes[3] = (big_endian_val & 0xFF000000) >> 24;
}

// ===========================================================================
//! Retrieve the character string stored in the DATA section
/*!
 *
 * @param [in] recv_buf: Received message
 * @param [out] data_str: Buffer that stores the extracted character string
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void get_res_data_str(uint8_t *recv_buf,uint8_t *data_str){
    uint8_t *st_p;
    // Convert the string length in the first 4 bytes of the DATA section in little endian format
    uint8_t str_len = conv_uint32_big_endian(&recv_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA]);
    
    // Check if the DATA length of the header data matches the contents of the character string length in the DATA section
    uint8_t recv_buf_len = recv_buf[CXM150x_CTRL_FW_UPDATE_API_POS_LE0]  - 4;     // Compare with the value obtained by subtracting 4 bytes for the character string length at the beginning of the DATA section
    if(str_len != recv_buf_len){
        // acquisition failed because character string length is invalid, set '\0' to the first character
        data_str[0] = '\0';
        return;
    }
    
    // string length check
    if(str_len <= 0 || str_len > CTRL_FW_UPDATE_RECEIVE_STR_MAX){
        // acquisition failed because character string length is invalid, set '\0' to the first character
        data_str[0] = '\0';
        return;
    }
    
    // Copying character string length data from the 4th byte or later
    st_p = &recv_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA+4];
    strncpy((char*)data_str,(char*)st_p,str_len);
    data_str[str_len] = '\0';
}

// ===========================================================================
//! ACK message check
/*!
 *
 * @param [in] recv_buf: Received message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return ACK receive result
*/
// ===========================================================================
static uint32_t check_ack(uint8_t *recv_buf){
    
    // Send message format check (whether the first byte starts with 0x68)
    if(recv_buf[CXM150x_CTRL_FW_UPDATE_API_POS_SD0] != CXM150x_CTRL_FW_UPDATE_API_SD){
        printf_info("str:%s\r\n",recv_buf);
        return CXM150x_RESPONSE_NG;
    }
    
    // If the DATA section is 0x90, return OK because it is ACK
    if(recv_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA] == CXM150x_CTRL_FW_UPDATE_API_D_ACK){
        return CXM150x_RESPONSE_OK;
    } else {
        return CXM150x_RESPONSE_NG;
    }
}

// ===========================================================================
//! Parse the character string acquisition result in the firmware update routine
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
static void res_check_get_CXM150x_ctrl_fw_update_routine_name(uint8_t *recv_buf,CmdResGetCtrlFWUpdateRoutineName *res){
    // Set the character string stored in the DATA section as a response
    if(res != NULL){
        get_res_data_str(recv_buf,res->m_str);
    }
}

// ===========================================================================
//! Get character string in firmware update routine
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
CXM150x_return_code get_CXM150x_ctrl_fw_update_routine_name(void *param, CmdResGetCtrlFWUpdateRoutineName *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    uint8_t recv_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    
    // create command
    uint8_t dt = CXM150x_CTRL_FW_UPDATE_API_DATA_ROUTINE_NAME;
    uint8_t send_len = create_tx_data(CXM150x_CTRL_FW_UPDATE_API_CMD_SEND_CMD,&dt,CXM150x_CTRL_FW_UPDATE_API_CMD_DATA_SIZE,send_buf);
    
    // send command
    ret = wrapper_CXM150x_ctrl_fw_update_tx_message(send_buf,send_len,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    if(ret != RETURN_OK){
        return ret;
    }
    
    // Wait for command response
    ret = wrapper_CXM150x_ctrl_fw_update_rx_message(recv_buf,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    if(ret != RETURN_OK){
        return ret;
    }
    
    // Create response data
    res_check_get_CXM150x_ctrl_fw_update_routine_name(recv_buf,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the result of obtaining firmware version information in CXM150x
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
static void res_check_get_CXM150x_ctrl_fw_update_version(uint8_t *recv_buf,CmdResGetCtrlFWUpdateVersion *res){
    // Endian conversion of version number stored in DATA section and set as response
    if(res != NULL){
        res->m_num = conv_uint32_big_endian(&recv_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA]);
    }
}

// ===========================================================================
//! Get firmware version number in CXM150x
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
CXM150x_return_code get_CXM150x_ctrl_fw_update_version(void* param, CmdResGetCtrlFWUpdateVersion *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    uint8_t recv_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    
    // create command
    uint8_t dt = CXM150x_CTRL_FW_UPDATE_API_DATA_VERSION;
    uint8_t send_len = create_tx_data(CXM150x_CTRL_FW_UPDATE_API_CMD_SEND_CMD,&dt,CXM150x_CTRL_FW_UPDATE_API_CMD_DATA_SIZE,send_buf);
    
    // send command
    ret = wrapper_CXM150x_ctrl_fw_update_tx_message(send_buf,send_len,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    if(ret != RETURN_OK){
        return ret;
    }
    
    // Wait for command response
    ret = wrapper_CXM150x_ctrl_fw_update_rx_message(recv_buf,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    if(ret != RETURN_OK){
        return ret;
    }
    
    // Create response data
    res_check_get_CXM150x_ctrl_fw_update_version(recv_buf,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the company name acquisition result in the firmware update routine
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
static void res_check_get_CXM150x_ctrl_fw_update_company_name(uint8_t *recv_buf,CmdResGetCtrlFWUpdateCompanyName *res){
    // Set the character string stored in the DATA section as a response
    if(res != NULL){
        get_res_data_str(recv_buf,res->m_str);
    }
}

// ===========================================================================
//! Get company name in firmware update routine
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
CXM150x_return_code get_CXM150x_ctrl_fw_update_company_name(void* param, CmdResGetCtrlFWUpdateCompanyName *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    uint8_t recv_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    
    // create command
    uint8_t dt = CXM150x_CTRL_FW_UPDATE_API_DATA_COMPANY_NAME;
    uint8_t send_len = create_tx_data(CXM150x_CTRL_FW_UPDATE_API_CMD_SEND_CMD,&dt,CXM150x_CTRL_FW_UPDATE_API_CMD_DATA_SIZE,send_buf);
    
    // send command
    ret = wrapper_CXM150x_ctrl_fw_update_tx_message(send_buf,send_len,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    if(ret != RETURN_OK){
        return ret;
    }
    
    // Wait for command response
    ret = wrapper_CXM150x_ctrl_fw_update_rx_message(recv_buf,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    if(ret != RETURN_OK){
        return ret;
    }
    
    // Create response data
    res_check_get_CXM150x_ctrl_fw_update_company_name(recv_buf,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the device string acquisition result in the firmware update routine
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
static void res_check_get_CXM150x_ctrl_fw_update_device_name(uint8_t *recv_buf,CmdResGetCtrlFWUpdateDeviceName *res){
    // Set the character string stored in the DATA section as a response
    if(res != NULL){
        get_res_data_str(recv_buf,res->m_str);
    }
}

// ===========================================================================
//! Get device string in firmware update routine
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
CXM150x_return_code get_CXM150x_ctrl_fw_update_device_name(void* param, CmdResGetCtrlFWUpdateDeviceName *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    uint8_t recv_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    
    // create command
    uint8_t dt = CXM150x_CTRL_FW_UPDATE_API_DATA_DEVICE_NAME;
    uint8_t send_len = create_tx_data(CXM150x_CTRL_FW_UPDATE_API_CMD_SEND_CMD,&dt,CXM150x_CTRL_FW_UPDATE_API_CMD_DATA_SIZE,send_buf);
    
    // send command
    ret = wrapper_CXM150x_ctrl_fw_update_tx_message(send_buf,send_len,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    if(ret != RETURN_OK){
        return ret;
    }
    
    // Wait for command response
    ret = wrapper_CXM150x_ctrl_fw_update_rx_message(recv_buf,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    if(ret != RETURN_OK){
        return ret;
    }
    
    // Create response data
    res_check_get_CXM150x_ctrl_fw_update_device_name(recv_buf,res);

    return RETURN_OK;
}

// ===========================================================================
//! Parse the current status acquisition result of the firmware update routine
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
static void res_check_get_CXM150x_ctrl_fw_update_state(uint8_t *recv_buf,CmdResGetCtrlFWUpdateState *res){
    if(res != NULL){
        uint32_t recv_val = conv_uint32_big_endian(&recv_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA]);
        switch(recv_val){

            // Initialization completion
            case CXM150x_CTRL_FW_UPDATE_STATE_INIT_END:
                res->m_num = CXM150x_CTRL_FW_UPDATE_STATE_INIT_END;
                break;
            // Firmware update
            case CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATEING:
                res->m_num = CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATEING;
                break;
            // Firmware update successful
            case CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATE_OK:
                res->m_num = CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATE_OK;
                break;
            // Firmware update failed
            case CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATE_NG:
                res->m_num = CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATE_NG;
                break;
            // undefined value
            default:
                res->m_num = CXM150x_CTRL_FW_UPDATE_STATE_FW_UPDATE_INVALID;
                break;
        }
    }
}

// ===========================================================================
//! Get current status of firmware update routine
/*!
 *
 * @param [in] param: Wait time for retry
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code get_CXM150x_ctrl_fw_update_state(uint32_t param, CmdResGetCtrlFWUpdateState *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    uint8_t recv_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    
    // create command
    uint8_t dt = CXM150x_CTRL_FW_UPDATE_API_DATA_STATE;
    uint8_t send_len = create_tx_data(CXM150x_CTRL_FW_UPDATE_API_CMD_SEND_CMD,&dt,CXM150x_CTRL_FW_UPDATE_API_CMD_DATA_SIZE,send_buf);
    uint32_t retry_wait_cnt = param;
    
    // counter for retry
    uint8_t try_cnt = 0;
    
    // Communication success flag
    uint8_t timeout_flag = CXM150x_CTRL_FW_UPDATE_API_FLAG_ON;

    // Perform retry processing when communication error occurs
    while(try_cnt <= CXM150x_CTRL_FW_UPDATE_API_STATE_RETRY_NUM){
        try_cnt++;
        // send command
        ret = wrapper_CXM150x_ctrl_fw_update_tx_message(send_buf,send_len,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
        if(ret == RETURN_OK){
            // Wait for command response
            ret = wrapper_CXM150x_ctrl_fw_update_rx_message(recv_buf,CTRL_FW_UPDATE_GET_UPDATE_STATE_MAX_TIME_OUT_TICK_COUNT);
            if(ret == RETURN_OK){
                timeout_flag = CXM150x_CTRL_FW_UPDATE_API_FLAG_OFF;
                break;
            }
            
            if(try_cnt <= CXM150x_CTRL_FW_UPDATE_API_STATE_RETRY_NUM){
                // put wait before retry
                wrapper_CXM150x_delay(retry_wait_cnt);
            }
        }
    }
    
    // Timeout if communication is not completed even after retry
    if(timeout_flag == CXM150x_CTRL_FW_UPDATE_API_FLAG_ON){
        return RETURN_TIMEOUT;
    }
    // Create response data
    res_check_get_CXM150x_ctrl_fw_update_state(recv_buf,res);

    return RETURN_OK;
}

// ===========================================================================
//! Firmware Update End Request result parsing (Side A)
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
static void res_check_set_CXM150x_ctrl_fw_update_end_request_type_A(uint8_t *recv_buf,CmdResSetCtrlFWUpdateEndRequest *res){
    if(res != NULL){
        if(recv_buf[CXM150x_CTRL_FW_UPDATE_API_POS_SD0] != CXM150x_CTRL_FW_UPDATE_API_SD){
            // After the update is completed, if "| SYS UPDATE OK" message is received, it ends normally
            if(strstr((char*)recv_buf,CXM150x_CTRL_FW_UPDATE_API_POWER_ON_MESSAGE_TYPE_A)){
                if(CXM150x_check_last_ok_ng(recv_buf) == CXM150x_RESPONSE_OK){
                    res->m_result = CXM150x_RESPONSE_OK;
                } else {
                    res->m_result = CXM150x_RESPONSE_NG;
                }
            } else {
                res->m_result = CXM150x_RESPONSE_NG;
            }
        } else {
            res->m_result = CXM150x_RESPONSE_NG;
        }
    }
}

// ===========================================================================
//! Firmware Update End Request result parsing (Side B)
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
static void res_check_set_CXM150x_ctrl_fw_update_end_request_type_B(uint8_t *recv_buf,CmdResSetCtrlFWUpdateEndRequest *res){
    if(res != NULL){
        if(recv_buf[CXM150x_CTRL_FW_UPDATE_API_POS_SD0] != CXM150x_CTRL_FW_UPDATE_API_SD){
            // After the update is completed, if the "| SYS RESET POR_PIN" message can be received, the process ends normally.
            if(strstr((char*)recv_buf,CXM150x_CTRL_FW_UPDATE_API_POWER_ON_MESSAGE_TYPE_B)){
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
//! Firmware Update End Request
/*!
 *
 * @param [in] param: Whether the transffered FW data is side A or side B
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_ctrl_fw_update_end_request(CXM150xFWUpdateType param, CmdResSetCtrlFWUpdateEndRequest *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    uint8_t recv_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    uint32_t ack_check_result;
    
    // create command
    uint8_t dt = CXM150x_CTRL_FW_UPDATE_API_DATA_END_REQUEST;
    uint8_t send_len = create_tx_data(CXM150x_CTRL_FW_UPDATE_API_CMD_SEND_CMD,&dt,CXM150x_CTRL_FW_UPDATE_API_CMD_DATA_SIZE,send_buf);
    
    // send command
    ret = wrapper_CXM150x_ctrl_fw_update_tx_message(send_buf,send_len,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    if(ret != RETURN_OK){
        return ret;
    }
    
    // Wait for command response (ACK)
    ret = wrapper_CXM150x_ctrl_fw_update_rx_message(recv_buf,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
    // Determine if received message is ACK
    
    if(ret == RETURN_OK){
        // Check if received message is ACK
        ack_check_result = check_ack(recv_buf);
        if(ack_check_result == CXM150x_RESPONSE_OK){
            // After receiving ACK, wait for "| SYS RESET POR_PIN" or "|SYS UPDATE OK" message
            ret = wrapper_CXM150x_ctrl_fw_update_rx_message(recv_buf,CTRL_FW_UPDATE_GET_UPDATE_SYS_RESET_TIME_OUT_TICK_COUNT);
            if(ret != RETURN_OK){
                // Receive NG
                return ret;
            }
        } else {
            // Receive non-ACK message
            if(res != NULL){
                res->m_result = CXM150x_RESPONSE_NG;
            }
            return ret;
        }
    } else {
        // message reception failed
        return ret;
    }
    
    // Create response data
    if(param == CXM150x_CTRL_FW_UPDATE_END_REQUEST_TYPE_A){
        res_check_set_CXM150x_ctrl_fw_update_end_request_type_A(recv_buf,res);
    } else {
        res_check_set_CXM150x_ctrl_fw_update_end_request_type_B(recv_buf,res);
    }

    return RETURN_OK;
}

// ===========================================================================
//! Parse the result of firmware update binary data transmission
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
static void res_check_set_CXM150x_ctrl_fw_update_data(uint8_t *recv_buf,CmdResSetCtrlFWUpdateData *res){
    if(res != NULL){
        // Normal termination if ACK can be received
         res->m_result = check_ack(recv_buf);
    }
}

// ===========================================================================
//! Transmit firmware update binary data
/*!
 *
 * @param [in] param: Firmware binary image structure
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_ctrl_fw_update_data(CXM150xFWUpdateSetData* param, CmdResSetCtrlFWUpdateData *res){
    CXM150x_return_code ret;
    uint8_t send_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    uint8_t recv_buf[CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN] = "";
    
    uint8_t data_buf[CTRL_FW_BINARY_IMAGE_MAX_LEN] = "";
    uint8_t data_len = 0;
    uint8_t send_len;
    
    // counter for retry
    uint8_t try_cnt = 0;
    
    // Communication success flag
    uint8_t timeout_flag = CXM150x_CTRL_FW_UPDATE_API_FLAG_ON;

    // Create DATA section
    if(param->m_total_image_len != 0){
        // Store the firmware image size at the beginning of the DATA section because it is the first send
        
        // Check firmware image data length (1 byte minimum, 235 bytes maximum)
        if(param->m_data_len > CTRL_FW_FIRST_BINARY_IMAGE_MAX_LEN || param->m_data_len < 1){
            return RETURN_NG;
        }
        
        // The beginning of the DATA section should be set as 0x03
        data_buf[0] = CXM150x_CTRL_FW_UPDATE_API_D_SD;
        
        // Store firmware image size in little endian format from 2nd to 5th byte
        conv_uint32_little_endian(param->m_total_image_len,&data_buf[1]);
        
        // Store firmware image data after 6th byte
        memcpy(&data_buf[CXM150x_CTRL_FW_UPDATE_API_FIRST_DATA_OFFSET],param->m_data,param->m_data_len);

        // Add D_SD + firmware image data length (5 bytes) to the specified length
        data_len = param->m_data_len + CXM150x_CTRL_FW_UPDATE_API_FIRST_DATA_OFFSET;
    } else {
        // Check FW image data length (Min. 1byte, Max. 240 bytes for second transaction or later)
        if(param->m_data_len > CTRL_FW_BINARY_IMAGE_MAX_LEN || param->m_data_len < 1){
            return RETURN_NG;
        }

        // After the second time, only firmware image data is stored
        memcpy(data_buf,param->m_data,param->m_data_len);
        
        // Second transaction or later, there is no additional information inside the API, so the length of the DATA section remains the specified length
        data_len = param->m_data_len;
    }
    
    // create command
    send_len = create_tx_data(CXM150x_CTRL_FW_UPDATE_API_CMD_SEND_FW_DATA,data_buf,data_len,send_buf);
    
    
    // Retry the specified number of times
    while(try_cnt <= CXM150x_CTRL_FW_UPDATE_API_DATA_RETRY_NUM){
        try_cnt++;
        // send command
        ret = wrapper_CXM150x_ctrl_fw_update_tx_message(send_buf,send_len,CTRL_FW_UPDATE_MAX_TIME_OUT_TICK_COUNT);
        if(ret == RETURN_OK){
            // Wait for command response
            ret = wrapper_CXM150x_ctrl_fw_update_rx_message(recv_buf,CTRL_FW_UPDATE_SEND_FW_DATA_MAX_TIME_OUT_TICK_COUNT);
            if(ret == RETURN_OK){
                timeout_flag = CXM150x_CTRL_FW_UPDATE_API_FLAG_OFF;
                break;
            }
        }
    }
    
    // Timeout if communication is not completed even after retry
    if(timeout_flag == CXM150x_CTRL_FW_UPDATE_API_FLAG_ON){
        return RETURN_TIMEOUT;
    }
    
    // Create response data
    res_check_set_CXM150x_ctrl_fw_update_data(recv_buf,res);

    return RETURN_OK;
}

// ===========================================================================
//! FW update mode power supply setting
/*!
 *
 * @param [in] on_off: Power supply status setting
 * @param [out] res: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code set_CXM150x_ctrl_fw_update_power (CXM150xFWUpdateSetPower *param, CmdResSetCtrlFWUpdatePower *res){
    if(param == NULL){
        return RETURN_NG;
    }
    
    if(param->m_on_off == CXM150x_POWER_ON){
        printf_info("CXM150x control fw update power on\r\n");
        
        // The CXM150x doesn't transit to CONTROL FW UPDATE mode correctly if LOW state duration of ENABLE pin is too short, so wait for a while after ENABLE pin state is set as LOW.
        wrapper_CXM150x_set_power(CXM150x_POWER_OFF);
        wrapper_CXM150x_delay(CXM150x_CTRL_FW_UPDATE_API_ENABLE_WAIT_COUNT);
        
        // Start CXM150x in CONTROL FW UPDATE mode
        wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_OFF);
        wrapper_CXM150x_set_power(CXM150x_POWER_ON);
        
        // Disable the UART interrupt
        wrapper_CXM150x_ctrl_fw_update_uart_abort_IT();
    } else {
        if(param->m_update_type == CXM150x_CTRL_FW_UPDATE_END_REQUEST_TYPE_A){
            // Do not change the WAKEUP pin to start transfer of side B after side A update
            printf_info("CXM150x control fw update power off type A\r\n");
            
            wrapper_CXM150x_set_power(CXM150x_POWER_OFF);
        } else {
            printf_info("CXM150x control fw update power off type B\r\n");
            
            wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_ON);
            wrapper_CXM150x_set_power(CXM150x_POWER_OFF);
        }
    }
    
    if(res != NULL){
        // always OK because no communication is involved
        res->m_result = CXM150x_RESPONSE_OK;
    }
    
    // always OK because no communication is involved
    return RETURN_OK;
}

#endif // CXM150x_CTRL_FW_UPDATE_API_USE

