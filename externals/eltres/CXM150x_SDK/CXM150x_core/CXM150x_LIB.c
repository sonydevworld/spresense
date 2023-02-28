// ==========================================================================
/*!
* @file     CXM150x_LIB.c
* @brief    UART communication related driver
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
#include <nuttx/compiler.h>
#include <string.h>
#include <stdio.h>
#include "CXM150x_APITypeDef.h"
#include "CXM150x_LIB.h"
#include "CXM150x_Utility.h"
#include "CXM150x_Port.h"
#include "CXM150x_SYS.h"
#include "CXM150x_TX.h"
#include "CXM150x_GNSS.h"

// Communication wait time with CXM (unit is msec)
#define MAX_TIME_OUT_TICK_COUNT     (5000)

// Timeout period for CXM150x power ON message wait
#define MAX_POWER_ON_TIME_OUT_TICK_COUNT     (10000)

// Command receive result
#define COMMAND_RESULT_RESPONSE_WAIT        (0)
#define COMMAND_RESULT_OK                   (1)
#define COMMAND_RESULT_ERROR                (2)
#define COMMAND_RESULT_BUSY                 (3)
#define COMMAND_RESULT_TIME_OUT             (4)
#define COMMAND_RESULT_COMMAND_INVALID      (5)

// Command transmission result
#define COMMAND_SEND_RESULT_OK              (0)
#define COMMAND_SEND_RESULT_BUSY            (1)
#define COMMAND_SEND_RESULT_NG              (2)
#define COMMAND_SEND_RESULT_TIME_OUT        (3)

// Response data structure
typedef struct {
    uint32_t m_result_code;                        // OK, NG (NG in response from CXM150x), timeout, BUSY
    int32_t m_option_num;                         // Numeric response (XX seconds, etc.)
    uint8_t m_option_str[CXM150x_RECEIVE_BUF_SIZE];      // On / Off, version information, previous payload, GPS time, etc.
}CommandResponseInfo;

// Buffer for received data from CXM, and variables for buffer control used in interrupt context
static uint8_t g_rcv_buf[CXM150x_RECEIVE_BUF_SIZE] = "";   // Receive buffer

// Response message buffer
static uint8_t g_response_buf[CXM150x_RECEIVE_BUF_SIZE] = "";   // Unprocessed command response message buffer
static CommandResponseInfo g_command_response_info;
static CXM150x_CALLBACK_RESPONSE_FUNC_POINTER g_command_response_callback = NULL;
static CXM150x_RES_PARSE_CALLBACK_FUNC_POINTER g_command_response_parse_func = NULL;
static void* g_response_parse_struct = NULL;

// Send command buffer
static uint32_t g_command_response_wait_flag = 0;
static uint32_t g_command_send_tick_count = 0;


static uint32_t volatile g_rcv_event_buf_rcv_index = 0;       // Index counter on addtional side
static uint32_t g_rcv_event_buf_proc_index = 0;      // Index counter on analysis processing side
static uint8_t g_event_buf[CXM150x_RECEIVE_EVENT_BUF_SIZE][CXM150x_RECEIVE_BUF_SIZE];      // unprocessed event message buffer

// Flag for reception status of CXM150x power ON message
static uint32_t g_rcv_power_on_message_wait = CXM150x_UART_DRIVER_FLAG_OFF;

// Wait message count
static uint32_t g_analyse_wait_message_cnt_res = 0;
static uint32_t g_analyse_wait_message_cnt_evt = 0;

// Response timeout time
static uint32_t g_max_time_out_tick_count = MAX_TIME_OUT_TICK_COUNT;

// event buffer overflow flag
static uint32_t g_event_buffer_overflow_flag = CXM150x_UART_DRIVER_FLAG_OFF;

// variables for callback when event occurs
CXM150xSysState *g_sys_state_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_sys_state_callback_func_p = NULL;
CXM150xGNSSState *g_gnss_state_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_GNSS_state_callback_func_p = NULL;
CXM150xNMEAGGAInfo *g_nmea_gga_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_NMEAGGA_callback_func_p = NULL;
CXM150xNMEAGLLInfo *g_nmea_gll_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_NMEAGLL_callback_func_p = NULL;
CXM150xNMEAGNSInfo *g_nmea_gns_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_NMEAGNS_callback_func_p = NULL;
CXM150xNMEAGSAInfo *g_nmea_gsa_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_NMEAGSA_callback_func_p = NULL;
CXM150xNMEAGSVInfo *g_nmea_gsv_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_NMEAGSV_callback_func_p = NULL;
CXM150xNMEARMCInfo *g_nmea_rmc_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_NMEARMC_callback_func_p = NULL;
CXM150xNMEAVTGInfo *g_nmea_vtg_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_NMEAVTG_callback_func_p = NULL;
CXM150xNMEAZDAInfo *g_nmea_zda_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_NMEAZDA_callback_func_p = NULL;
CXM150xTimeAlarm *g_time_alarm_info = NULL;
CXM150xNMEAPSGESInfo *g_nmea_psges_info;
CXM150x_CALLBACK_FUNC_POINTER g_NMEAPSGES_callback_func_p;
CXM150xNMEAPSLESInfo *g_nmea_psles_info;
CXM150x_CALLBACK_FUNC_POINTER g_NMEAPSLES_callback_func_p;
CXM150x_CALLBACK_FUNC_POINTER g_time_alarm_callback_func_p = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_LPWA_start_interrupt_callback_func_p = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_uart_start_interrupt_callback_func_p = NULL;
CXM150xTxState *g_tx_state_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_tx_prev_stt_callback_func_p = NULL;
CXM150xFATALMessage *g_FATAL_message_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_FATAL_message_callback_func_p = NULL;
CXM150xEventBufferOverflow *g_eventbuffer_overflow_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_event_buffer_overflow_func_p = NULL;
CXM150xTxPoCEnableMessage *g_tx_poc_enable_message_event_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_tx_poc_enable_message_callback_func_p = NULL;
CXM150xSysToDeepSleepInfo *g_sys_to_deepsleep_event_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_sys_to_deepsleep_callback_func_p = NULL;
CXM150xTxPayloadInfo *g_tx_payload_event_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_tx_payload_event_callback_func_p = NULL;
CXM150xTxDutyEventInfo *g_tx_duty_event_info = NULL;
CXM150x_CALLBACK_FUNC_POINTER g_tx_duty_event_callback_func_p = NULL;

extern uint8_t g_uart_error_flg;
// ===========================================================================
//! Receive interrupt buffer setting
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_rcv_buf: Receive buffer
 *
 * @return processing result
*/
// ===========================================================================
uint32_t CXM150x_init_uart_driver(void){
    return wrapper_CXM150x_set_uart_rx_buf(g_rcv_buf);
}

// ===========================================================================
//! Add unprocessed response message to buffer
/*!
 *
 * @param [in] msg: Response message
 * @param [in] msg_len: Response message length
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_response_buf: Response message buffer
 *
 * @return none
*/
// ===========================================================================
static void add_response_message_buf(uint8_t *msg,uint8_t msg_len){
    if(msg_len > CXM150x_RECEIVE_BUF_SIZE || msg_len == 0){
        printf_err("add_response_message_buf length error[%dbyte]\r\n",msg_len);
        return;
    }
    strncpy((char*)g_response_buf,(char*)msg,msg_len);
    g_response_buf[msg_len - 1] = '\0';
}

// ===========================================================================
//! Add unprocessed event message to buffer
/*!
 *
 * @param [in] msg: Event message
 * @param [in] msg_len: Event message length
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_event_buf: Unprocessed event message buffer
 *        [out] g_rcv_event_buf_rcv_index: Index counter of additional side
 *        [out] g_event_buffer_overflow_flag: Event buffer overflow flag
 *        [out] g_analyse_wait_message_cnt_evt: Processing wait event counter
* @return none
*/
// ===========================================================================
static void add_event_message_buf(uint8_t *msg,uint8_t msg_len){
    if(msg_len > CXM150x_RECEIVE_BUF_SIZE || msg_len == 0){
        printf_err("add_event_message_buf length error[%dbyte]\r\n",msg_len);
        return;
    }
    
    // Check if there is an unprocessed event in the copy destination buffer
    if(g_event_buf[g_rcv_event_buf_rcv_index][0] != '\0'){
        // If overwriting occurs

        // Flag ON because event buffer overflow occurred
        g_event_buffer_overflow_flag = CXM150x_UART_DRIVER_FLAG_ON;
        
        // In case of overflow, return without overwriting the event message
        return;
    }else {
        // If overwriting does not occur
        // Increment unprocessed message counter
        if(g_analyse_wait_message_cnt_evt < CXM150x_RECEIVE_EVENT_BUF_SIZE){
            g_analyse_wait_message_cnt_evt++;
        }
    }
    strncpy((char*)g_event_buf[g_rcv_event_buf_rcv_index],(char*)msg,msg_len);
    g_event_buf[g_rcv_event_buf_rcv_index][msg_len-1] = '\0';
    g_rcv_event_buf_rcv_index++;
    if(g_rcv_event_buf_rcv_index >= CXM150x_RECEIVE_EVENT_BUF_SIZE){
        g_rcv_event_buf_rcv_index = 0;
    }
}

// ===========================================================================
//! Add to receive message buffer
/*!
 *
 * @param [in] rx_message: Received message
 * @param [in] rcv_cnt: Number of received characters
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_analyse_wait_message_cnt_res: Processing response message counter
 * @return none
*/
// ===========================================================================
static void add_rx_buffer(uint8_t *rx_message,uint8_t rcv_cnt){
    if(rx_message[0] == '|'){
        // receive event message
        add_event_message_buf(rx_message,rcv_cnt);
    } else if(rx_message[0] == '>'){
        // Receive command response
        add_response_message_buf(rx_message,rcv_cnt);
        g_analyse_wait_message_cnt_res++;
    } else {
        printf_err("rcv message error:%s",rx_message);
    }
}

// ===========================================================================
//! Process when a line feed is received in receive interrupt
/*!
 *
 * @param [in] type_from: UART communication type
 * @param [in] rcv_cnt: Number of received characters
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_rcv_buf: Receive buffer
 *
 * @return none
*/
// ===========================================================================
void CXM150x_uart_receive_to_buffer_callback(uint32_t type_from,uint32_t rcv_cnt)
{
    if(type_from==WRAPPER_UART_RX_FROM_CXM150x){
        add_rx_buffer((uint8_t*)g_rcv_buf,rcv_cnt);
        g_rcv_buf[0] = '\0';
    }
}

// ===========================================================================
//! Send command
/*!
 *
 * @param [in] command: Command data
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_command_response_wait_flag: Command response wait flag
 *        [out] g_response_buf: Response message buffer
 *        [out] g_command_response_info: Response data structure
 *        [out] g_command_send_tick_count: Tick count at the start of transmission
 *
 * @return processing result
*/
// ===========================================================================
static CXM150x_return_code send_command(uint8_t *command){
    
    // Check if there is a command that has been sent and is waiting for a response (if any, end with BUSY)
    if(g_command_response_wait_flag == CXM150x_UART_DRIVER_FLAG_ON){
        printf_err("BUSY:%s",command);
        g_command_response_info.m_result_code = COMMAND_SEND_RESULT_BUSY;
        return RETURN_BUSY;
    }
    
    // Turn on the reception wait flag and start measuring the timeout
    g_command_response_wait_flag = CXM150x_UART_DRIVER_FLAG_ON;
    g_command_send_tick_count = wrapper_CXM150x_get_tick();
    g_command_response_info.m_result_code = COMMAND_RESULT_RESPONSE_WAIT;
    memset(g_command_response_info.m_option_str,'\0',sizeof(g_command_response_info.m_option_str));
    g_response_buf[0] = '\0';
    
    // Send processing
    CXM150x_return_code ret = wrapper_CXM150x_uart_transmit(command,strlen((char*)command),MAX_TIME_OUT_TICK_COUNT);
    printf_info("snd:%s",command);
    if(ret != RETURN_OK){
        // Cancel command response wait
        g_command_response_wait_flag = CXM150x_UART_DRIVER_FLAG_OFF;
        return ret;
    }
    
    return RETURN_OK;
}

// ===========================================================================
//! Set command response data
/*!
 *
 * @param [in] result: Command response data setting destination
 * @param [in] msg: Command response message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static void set_command_response_data(CommandResponseInfo *result,uint8_t *msg){
    result->m_result_code = RETURN_OK;
    strncpy((char*)result->m_option_str,(char*)msg,strlen((char*)msg));
    result->m_option_str[strlen((char*)msg)] = '\0';
}

// ===========================================================================
//! Check if there is an unprocessed command response
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_response_buf: Response message buffer
 *        [in] g_command_send_tick_count: Tick count at the start of transmission
 *        [in] g_max_time_out_tick_count: Timeout Tick count
 *        [out] g_command_response_wait_flag: Command response wait flag
 *        [out] g_command_response_info: Response parse result
 *        [out] g_command_response_callback: Response callback function
 *        [out] g_response_parse_struct: Response data structure
 * @return none
*/
// ===========================================================================
static void check_command_response(void){
    
    // Check if there is an unprocessed response
    if(g_response_buf[0] != '\0'){
        g_command_response_wait_flag = CXM150x_UART_DRIVER_FLAG_OFF;
        printf_info("rcv:%s",g_response_buf);
        set_command_response_data(&g_command_response_info,g_response_buf);
        g_response_buf[0] = '\0';
        g_analyse_wait_message_cnt_res--;
        
        //Non-blocking
        if(g_command_response_callback != NULL){
            
            // In case of reset command, do not call back here to wait for CXM150x power ON message
            if(strstr((char*)g_command_response_info.m_option_str,"> SYS RESET SET")){
                // Parses the contents of the response, and if the response is OK, starts waiting for the power ON message
                // Use a temporary variable because NULL may be set to the response structure
                CmdResResetCXM150x resCmdResResetCXM150x;
                g_command_response_parse_func((uint8_t*)g_command_response_info.m_option_str,&resCmdResResetCXM150x);
                if(resCmdResResetCXM150x.m_result == CXM150x_RESPONSE_OK){
                    g_command_response_wait_flag = CXM150x_UART_DRIVER_FLAG_ON;
                    CXM150x_prep_wait_power_on_message_reset();
                } else {
                    CXM150x_CALLBACK_RESPONSE_FUNC_POINTER tmp_func_p = g_command_response_callback;
                    g_command_response_callback = NULL;
                    g_command_response_parse_func((uint8_t*)g_command_response_info.m_option_str,g_response_parse_struct);
                    g_command_response_parse_func = NULL;
                    tmp_func_p(RETURN_OK,g_response_parse_struct);
                }
                return;
            }
            
            CXM150x_CALLBACK_RESPONSE_FUNC_POINTER tmp_func_p = g_command_response_callback;
            g_command_response_callback = NULL;
            g_command_response_parse_func((uint8_t*)g_command_response_info.m_option_str,g_response_parse_struct);
            g_command_response_parse_func = NULL;
            tmp_func_p(RETURN_OK,g_response_parse_struct);
        }
        return;
    }
    
    // response timeout check
    if(g_command_response_wait_flag == CXM150x_UART_DRIVER_FLAG_ON || g_rcv_power_on_message_wait == CXM150x_UART_DRIVER_FLAG_ON){
        uint32_t current_tick = wrapper_CXM150x_get_tick();
        if(current_tick - g_command_send_tick_count > g_max_time_out_tick_count){
            // timed out
            printf_err("timeout command.(%ldmsec)\r\n",g_max_time_out_tick_count);
            g_command_response_info.m_result_code = COMMAND_RESULT_TIME_OUT;
            g_command_response_wait_flag = CXM150x_UART_DRIVER_FLAG_OFF;
            if(g_rcv_power_on_message_wait == CXM150x_UART_DRIVER_FLAG_ON){
                g_rcv_power_on_message_wait = CXM150x_UART_DRIVER_FLAG_OFF;
            }
            if(g_command_response_callback != NULL){
                CXM150x_CALLBACK_RESPONSE_FUNC_POINTER tmp_func_p = g_command_response_callback;
                g_command_response_callback = NULL;
                g_command_response_parse_func = NULL;
                //tmp_func_p(&g_command_response_info,RETURN_TIMEOUT);
                tmp_func_p(RETURN_TIMEOUT,g_response_parse_struct);
            }
            return;
        }
    }
}

// ===========================================================================
//! Check event notifications
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_sys_state_info: system state event information
 *        [in] g_sys_state_callback_func_p: system state event callback function
 *        [in] g_gnss_state_info: GNSS state event information
 *        [in] g_GNSS_state_callback_func_p: GNSS state event callback function
 *        [in] g_nmea_gga_info: GGA event information
 *        [in] g_NMEAGGA_callback_func_p: GGA event callback function
 *        [in] g_nmea_gll_info: GLL event information
 *        [in] g_NMEAGLL_callback_func_p: GLL event callback function
 *        [in] g_nmea_gns_info: GNS event information
 *        [in] g_NMEAGNS_callback_func_p: GNS event callback function
 *        [in] g_nmea_gsa_info: GSA event information
 *        [in] g_NMEAGSA_callback_func_p: GSA event callback function
 *        [in] g_nmea_gsv_info: GSV event information
 *        [in] g_NMEAGSV_callback_func_p: GSV event callback function
 *        [in] g_nmea_rmc_info: RMC event information
 *        [in] g_NMEARMC_callback_func_p: RMC event callback function
 *        [in] g_nmea_vtg_info: VTG event information
 *        [in] g_NMEAVTG_callback_func_p: VTG event callback function
 *        [in] g_nmea_zda_info: ZDA event information
 *        [in] g_NMEAZDA_callback_func_p: ZDA event callback function
 *        [in] g_nmea_psges_info: PSGES event information
 *        [in] g_NMEAPSGES_callback_func_p: PSGES event callback function
 *        [in] g_nmea_psles_info: PSLES event information
 *        [in] g_NMEAPSLES_callback_func_p: PSLES event callback function
 *        [in] g_time_alarm_info: LPWA Transmit data update time limit UART event information
 *        [in] g_time_alarm_callback_func_p: INT_OUT1 event callback function
 *        [in] g_LPWA_start_interrupt_callback_func_p: INT_OUT1 callback function
 *        [in] g_uart_start_interrupt_callback_func_p: Callback function for interrupt notification before UART transmission time
 *        [in] g_tx_state_info: TxState event information
 *        [in] g_tx_prev_stt_callback_func_p: TxState event callback function
 *        [in] g_FATAL_message_info: Abnormal event information
 *        [in] g_FATAL_message_callback_func_p: Abnormal event callback function
 *        [in] g_event_buffer_overflow_flag: Event buffer overflow flag
 *        [in] g_tx_prev_stt_callback_func_p: Event buffer overflow occurrence callback function
 *        [in] g_eventbuffer_overflow_info: Overflow information pointer
 *        [out] g_rcv_event_buf_proc_index: Index counter on the analysis processing side
 *        [out] g_event_buf: Unprocessed event message buffer
 *        [out] g_rcv_power_on_message_wait: Receive flag for waiting for CXM150x power ON message
 *        [out] g_analyse_wait_message_cnt_evt: Processing wait event counter
 *        [out] g_command_response_parse_func: Result parse callback function when response is received
 *        [out] g_response_parse_struct: Response data structure
 *
 * @return none
*/
// ===========================================================================
static void check_event(void){
    
    if(g_rcv_event_buf_proc_index >= CXM150x_RECEIVE_EVENT_BUF_SIZE){
        printf_err("check_event g_rcv_event_buf_proc_index count over error\r\n");
        return;
    }
    
    uint8_t *proc_event_msg = g_event_buf[g_rcv_event_buf_proc_index];
    if(proc_event_msg == NULL){
        printf_err("check_event proc_event_msg is null\r\n");
        return;
    }
    
    // If an event buffer overflow occurs, call the registered callback function
    if(g_event_buffer_overflow_flag == CXM150x_UART_DRIVER_FLAG_ON){
        if(g_event_buffer_overflow_func_p != NULL){
            g_event_buffer_overflow_flag = CXM150x_UART_DRIVER_FLAG_OFF;
            if(g_eventbuffer_overflow_info != NULL){
                *g_eventbuffer_overflow_info = EVENT_BUFFER_OVERFLOW;
            }
            g_event_buffer_overflow_func_p(g_eventbuffer_overflow_info,CXM150x_EVENT_CALLBACK_ID_EVENT_BUFFER_OVERFLOW);
        } else {
            printf_err("check_event event buffer overflow\r\n");
        }
    }
    
    if(proc_event_msg[0] != '\0'){          // Check if there are unprocessed events
        
        //error
        if(proc_event_msg[0] != '|' || proc_event_msg[1] != ' '){
            // Mark the event message as processed
           //uint32_t len = strlen((char*)proc_event_msg);
           printf_err("error evt[%ld]:%s\r\n",g_rcv_event_buf_proc_index,(char*)proc_event_msg);
           memset(&g_event_buf[g_rcv_event_buf_proc_index],'\0',CXM150x_RECEIVE_BUF_SIZE);
            
           return;
        }
        
        printf_info("evt:%s",(char*)proc_event_msg);
        if(strstr((char*)proc_event_msg,"| SYS RESET DSLP") != NULL){
            
            // When resuming from deep sleep, set WAKEUP to "L" to save power.
            wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_OFF);
            
            if(g_sys_to_deepsleep_event_info != NULL){
                g_sys_to_deepsleep_event_info->m_type = TO_DEEPSLEEP_EVENT_TYPE_RESET_DSLP;
                g_sys_to_deepsleep_event_info->m_sleep_time = 0;
                if(g_sys_to_deepsleep_callback_func_p != NULL){
                    g_sys_to_deepsleep_callback_func_p(g_sys_to_deepsleep_event_info,CXM150x_EVENT_CALLBACK_ID_SYS_TO_DEEP_SLEEP_EVENT);
                }
            }
        }
        else if(strstr((char*)proc_event_msg,"| SYS RESET") != NULL){
            
            // If a message is received other than when waiting for the CXM150x power ON message, perform FATAL message event callback
            if(g_rcv_power_on_message_wait == CXM150x_UART_DRIVER_FLAG_ON){
                // In case of waiting for CXM150x power ON message
                g_rcv_power_on_message_wait = CXM150x_UART_DRIVER_FLAG_OFF;
                if(g_command_response_callback != NULL){
                    g_command_response_wait_flag = CXM150x_UART_DRIVER_FLAG_OFF;
                    // Callback processing when waiting for non-block CXM150x power ON message reception
                    CXM150x_CALLBACK_RESPONSE_FUNC_POINTER tmp_func_p = g_command_response_callback;
                    g_command_response_callback = NULL;
                    // Parse and set the message content in the response structure specified by the API caller
                    g_command_response_parse_func((uint8_t*)g_command_response_info.m_option_str,g_response_parse_struct);
                    g_command_response_parse_func = NULL;
                    tmp_func_p(RETURN_OK,g_response_parse_struct);
                }
            } else {
                // When not waiting for CXM150x power ON message
                if(g_FATAL_message_info != NULL){
                    memcpy(g_FATAL_message_info->m_str,&proc_event_msg[0],CXM150x_RECEIVE_BUF_SIZE);
                    g_FATAL_message_info->m_str[CXM150x_RECEIVE_BUF_SIZE - 1] = '\0';
                    proc_event_msg[0] = '\0';
                    if(g_FATAL_message_callback_func_p != NULL){
                        g_FATAL_message_callback_func_p(g_FATAL_message_info,CXM150x_EVENT_CALLBACK_ID_FATAL_MESSAGE_EVENT);
                    }
                }
            }
        } else if(strstr((char*)proc_event_msg,"| SYS STT")){
            if(g_sys_state_info != NULL){
                *g_sys_state_info = conv_sys_stt_message_to_code((uint8_t*)proc_event_msg);
                g_sys_state_callback_func_p(g_sys_state_info,CXM150x_EVENT_CALLBACK_ID_SYS_STATE_EVENT);
            }
        } else if(strstr((char*)proc_event_msg,"| GNSS STT")){
            if(g_gnss_state_info != NULL){
                *g_gnss_state_info = conv_gnss_stt_message_to_code((uint8_t*)proc_event_msg);
                if(g_GNSS_state_callback_func_p != NULL) {
                    g_GNSS_state_callback_func_p(g_gnss_state_info,CXM150x_EVENT_CALLBACK_ID_GNSS_STATE_EVENT);
                }
            }
        } else if(strstr((char*)proc_event_msg,"| $")){
            if(strstr((char*)proc_event_msg,"GGA")){
                if(g_nmea_gga_info != NULL){
                    parse_nmea_sentence_gga((uint8_t*)proc_event_msg,g_nmea_gga_info);
                    proc_event_msg[0] = '\0';
                    if(g_NMEAGGA_callback_func_p != NULL){
                        g_NMEAGGA_callback_func_p(g_nmea_gga_info,CXM150x_EVENT_CALLBACK_ID_NMEAGGA_EVENT);
                    }
                }
            } else if (strstr((char*)proc_event_msg,"GLL")) {
                if(g_nmea_gll_info != NULL){
                    parse_nmea_sentence_gll((uint8_t*)proc_event_msg,g_nmea_gll_info);
                    proc_event_msg[0] = '\0';
                    if(g_NMEAGLL_callback_func_p != NULL){
                        g_NMEAGLL_callback_func_p(g_nmea_gll_info,CXM150x_EVENT_CALLBACK_ID_NMEAGLL_EVENT);
                    }
                }
            } else if(strstr((char*)proc_event_msg,"GNS")){
                if(g_nmea_gns_info != NULL){
                    parse_nmea_sentence_gns((uint8_t*)proc_event_msg,g_nmea_gns_info);
                    proc_event_msg[0] = '\0';
                    if(g_NMEAGNS_callback_func_p != NULL){
                        g_NMEAGNS_callback_func_p(g_nmea_gns_info,CXM150x_EVENT_CALLBACK_ID_NMEAGNS_EVENT);
                    }
                }
            } else if(strstr((char*)proc_event_msg,"GSA")){
                if(g_nmea_gsa_info != NULL){
                    parse_nmea_sentence_gsa((uint8_t*)proc_event_msg,g_nmea_gsa_info);
                    proc_event_msg[0] = '\0';
                    if(g_NMEAGSA_callback_func_p != NULL){
                        g_NMEAGSA_callback_func_p(g_nmea_gsa_info,CXM150x_EVENT_CALLBACK_ID_NMEAGSA_EVENT);
                    }
                }
            } else if(strstr((char*)proc_event_msg,"GSV")){
                if(g_nmea_gsv_info != NULL){
                    parse_nmea_sentence_gsv((uint8_t*)proc_event_msg,g_nmea_gsv_info);
                    proc_event_msg[0] = '\0';
                    if(g_NMEAGSV_callback_func_p != NULL){
                        g_NMEAGSV_callback_func_p(g_nmea_gsv_info,CXM150x_EVENT_CALLBACK_ID_NMEAGSV_EVENT);
                    }
                }
            } else if(strstr((char*)proc_event_msg,"RMC")){
                if(g_nmea_rmc_info != NULL){
                    parse_nmea_sentence_rmc((uint8_t*)proc_event_msg,g_nmea_rmc_info);
                    proc_event_msg[0] = '\0';
                    if(g_NMEARMC_callback_func_p != NULL){
                        g_NMEARMC_callback_func_p(g_nmea_rmc_info,CXM150x_EVENT_CALLBACK_ID_NMEARMC_EVENT);
                    }
                }
            } else if(strstr((char*)proc_event_msg,"VTG")){
                if(g_nmea_vtg_info != NULL){
                    parse_nmea_sentence_vtg((uint8_t*)proc_event_msg,g_nmea_vtg_info);
                    proc_event_msg[0] = '\0';
                    if(g_NMEAVTG_callback_func_p != NULL){
                        g_NMEAVTG_callback_func_p(g_nmea_vtg_info,CXM150x_EVENT_CALLBACK_ID_NMEAVTG_EVENT);
                    }
                }
            } else if(strstr((char*)proc_event_msg,"ZDA")){
                if(g_nmea_zda_info != NULL){
                    parse_nmea_sentence_zda((uint8_t*)proc_event_msg,g_nmea_zda_info);
                    proc_event_msg[0] = '\0';
                    if(g_NMEAZDA_callback_func_p != NULL){
                        g_NMEAZDA_callback_func_p(g_nmea_zda_info,CXM150x_EVENT_CALLBACK_ID_NMEAZDA_EVENT);
                    }
                }
            } else if(strstr((char*)proc_event_msg,"PSGES")){
                if(g_nmea_psges_info != NULL){
                    parse_nmea_sentence_psges((uint8_t*)proc_event_msg,g_nmea_psges_info);
                    proc_event_msg[0] = '\0';
                    if(g_NMEAPSGES_callback_func_p != NULL){
                        g_NMEAPSGES_callback_func_p(g_nmea_psges_info,CXM150x_EVENT_CALLBACK_ID_NMEAPSGES_EVENT);
                    }
                }
            } else if(strstr((char*)proc_event_msg,"PSLES")){
                if(g_nmea_psles_info != NULL){
                    parse_nmea_sentence_psles((uint8_t*)proc_event_msg,g_nmea_psles_info);
                    proc_event_msg[0] = '\0';
                    if(g_NMEAPSLES_callback_func_p != NULL){
                        g_NMEAPSLES_callback_func_p(g_nmea_psles_info,CXM150x_EVENT_CALLBACK_ID_NMEAPSLES_EVENT);
                    }
                }
            } else {
                printf_err("check_event err:%s",proc_event_msg);
            }
        } else if(strstr((char*)proc_event_msg,"| TIME ALARM")){
            if(g_time_alarm_info != NULL){
                proc_event_msg[0] = '\0';
                *g_time_alarm_info = TIME_ALARM;
                if(g_time_alarm_callback_func_p != NULL){
                    g_time_alarm_callback_func_p(g_time_alarm_info,CXM150x_EVENT_CALLBACK_ID_TX_START_MESSAGE_EVENT);
                }
            }
        } else if(strstr((char*)proc_event_msg,"| TX PREV_STT")) {
            if(g_tx_state_info != NULL){
                *g_tx_state_info = conv_tx_stt_message_to_code((uint8_t*)proc_event_msg);
                proc_event_msg[0] = '\0';
                if(g_tx_prev_stt_callback_func_p != NULL){
                    g_tx_prev_stt_callback_func_p(g_tx_state_info,CXM150x_EVENT_CALLBACK_ID_TX_STATE_EVENT);
                }
            }
        } else if(strstr((char*)proc_event_msg,"| TX POC_EN")){
            if(g_tx_poc_enable_message_event_info != NULL){
                proc_event_msg[0] = '\0';
                *g_tx_poc_enable_message_event_info = TX_POC_ENABLE_MESSAGE;
                if(g_tx_poc_enable_message_callback_func_p != NULL){
                    g_tx_poc_enable_message_callback_func_p(g_tx_poc_enable_message_event_info,CXM150x_EVENT_CALLBACK_ID_POC_ENABLE_MESSAGE_EVENT);
                }
            }
        } else if(strstr((char*)proc_event_msg,"| SYS TO_DSLP") != NULL){
            // If WAKEUP is set to "L", CXM150x will start up in FW update mode when it resume from DeepSleep.
            wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_ON);
            
            if(g_sys_to_deepsleep_event_info != NULL){
                g_sys_to_deepsleep_event_info->m_type = TO_DEEPSLEEP_EVENT_TYPE_TO_DSLP;
                g_sys_to_deepsleep_event_info->m_sleep_time = CXM150x_get_last_uint32(proc_event_msg);
                if(g_sys_to_deepsleep_callback_func_p != NULL){
                    g_sys_to_deepsleep_callback_func_p(g_sys_to_deepsleep_event_info,CXM150x_EVENT_CALLBACK_ID_SYS_TO_DEEP_SLEEP_EVENT);
                }
            }
        } else if(strstr((char*)proc_event_msg,"| TX PLD") != NULL){
            if(g_tx_payload_event_info != NULL){
                uint8_t rcv_str_last_word[CXM150x_RECEIVE_BUF_SIZE] = "";
                if(CXM150x_get_last_word(proc_event_msg,rcv_str_last_word) == CXM150x_RESPONSE_OK){
                    CXM150x_ascii_to_bin(rcv_str_last_word,g_tx_payload_event_info->m_payload_data,CXM150x_PAYLOAD_LEN*2);
                } else {
                    g_tx_payload_event_info->m_payload_data[0] = '\0';
                }
                if(g_tx_payload_event_callback_func_p != NULL){
                    g_tx_payload_event_callback_func_p(g_tx_payload_event_info,CXM150x_EVENT_CALLBACK_ID_TX_PLD);
                }
            }
        } else if(strstr((char*)proc_event_msg,"| TX DUTY") != NULL){
            if(g_tx_duty_event_info != NULL){
                uint8_t rcv_str_last_word[CXM150x_RECEIVE_BUF_SIZE] = "";
                if(CXM150x_get_last_word(proc_event_msg,rcv_str_last_word) == CXM150x_RESPONSE_OK){
                    if(strstr((char*)rcv_str_last_word,"OK")){
                        g_tx_duty_event_info->m_result = CXM150x_RESPONSE_OK;
                        strncpy((char*)g_tx_duty_event_info->m_str, (char*)&rcv_str_last_word[5], CXM150x_RECEIVE_BUF_GNSSTIME_SIZE);
                        g_tx_duty_event_info->m_str[CXM150x_RECEIVE_BUF_GNSSTIME_SIZE] = '\0';
                    }else if(strstr((char*)rcv_str_last_word,"NG")){
                        g_tx_duty_event_info->m_result = CXM150x_RESPONSE_NG;
                        strncpy((char*)g_tx_duty_event_info->m_str, (char*)&rcv_str_last_word[5], CXM150x_RECEIVE_BUF_GNSSTIME_SIZE);
                        g_tx_duty_event_info->m_str[CXM150x_RECEIVE_BUF_GNSSTIME_SIZE] = '\0';
                    }else{
                        g_tx_duty_event_info->m_result = CXM150x_RESPONSE_NG;
                        g_tx_duty_event_info->m_str[0] = '\0';
                    }
                } else {
                        g_tx_duty_event_info->m_result = CXM150x_RESPONSE_NG;
                        g_tx_duty_event_info->m_str[0] = '\0';
                }
                if(g_tx_duty_event_callback_func_p != NULL){
                    g_tx_duty_event_callback_func_p(g_tx_duty_event_info,CXM150x_EVENT_CALLBACK_ID_TX_DUTY);
                }
            }
        } else {
            if(g_FATAL_message_info != NULL){
                memcpy(g_FATAL_message_info->m_str,&proc_event_msg[0],CXM150x_RECEIVE_BUF_SIZE);
                g_FATAL_message_info->m_str[CXM150x_RECEIVE_BUF_SIZE - 1] = '\0';
                proc_event_msg[0] = '\0';
                if(g_FATAL_message_callback_func_p != NULL){
                    g_FATAL_message_callback_func_p(g_FATAL_message_info,CXM150x_EVENT_CALLBACK_ID_FATAL_MESSAGE_EVENT);
                }
            }
        }
        
        // Mark the event message as processed
        proc_event_msg[0] = '\0';
        memset(&g_event_buf[g_rcv_event_buf_proc_index],'\0',CXM150x_RECEIVE_BUF_SIZE);
        g_rcv_event_buf_proc_index++;
        if(g_rcv_event_buf_proc_index >= CXM150x_RECEIVE_EVENT_BUF_SIZE){
            g_rcv_event_buf_proc_index = 0;
        }
        g_analyse_wait_message_cnt_evt--;
    }
}

// ===========================================================================
//! Parse received message
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_uart_error_flg: UART error flag
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
void CXM150x_trigger_analyse(void){
    // Perform recovery processing if a UART error has occurred
    if(g_uart_error_flg == CXM150x_UART_DRIVER_FLAG_ON){
        printf_err("uart1 error recovery\r\n");
        CXM150x_init_uart_driver();
        g_uart_error_flg = CXM150x_UART_DRIVER_FLAG_OFF;
    }

    // Monitor command response and judge timeout
    check_command_response();
    
    // Monitor and parse event messages
    check_event();

}

// ===========================================================================
//! Send command and wait for response
/*!
 *
 * @param [in] cmd: Send command data
 * @param [out] respons_message: Receive message storage buffer
 * @par Global variable
 *        [in] g_command_response_wait_flag: Command response wait flag
 *        [in] g_command_response_info: Response data structure
 *        [out] g_max_time_out_tick_count: Timeout Tick count
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code CXM150x_send_and_wait_command_response(uint8_t *cmd,uint8_t *respons_message){
    g_max_time_out_tick_count = MAX_TIME_OUT_TICK_COUNT;
    CXM150x_return_code ret = send_command(cmd);
    if(ret != RETURN_OK){
        return ret;
    }
    
    while(g_command_response_wait_flag == CXM150x_UART_DRIVER_FLAG_ON){
        CXM150x_trigger_analyse();
    }
    
    if(g_command_response_info.m_result_code == COMMAND_RESULT_TIME_OUT){
        return RETURN_TIMEOUT;
    }
    
    strncpy((char*)respons_message,(char*)g_command_response_info.m_option_str,CXM150x_RECEIVE_BUF_SIZE);
    respons_message[strlen((char*)g_command_response_info.m_option_str)] = '\0';
    
    return RETURN_OK;
}

// ===========================================================================
//! Command transmission and response wait (response wait time specified)
/*!
 *
 * @param [in] cmd: Send command data
 * @param [out] respons_message: Receive message storage buffer
 * @par Global variable
 *        [in] g_command_response_wait_flag: Command response wait flag
 *        [in] g_command_response_info: Response data structure
 *        [out] g_max_time_out_tick_count: Response waiting count
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code CXM150x_send_and_wait_command_response_long_wait(uint8_t *cmd,uint8_t *respons_message,uint32_t max_wait){
    g_max_time_out_tick_count = max_wait;
    CXM150x_return_code ret = send_command(cmd);
    if(ret != RETURN_OK){
        return ret;
    }
    
    while(g_command_response_wait_flag == CXM150x_UART_DRIVER_FLAG_ON){
        CXM150x_trigger_analyse();
    }
    
    if(g_command_response_info.m_result_code == COMMAND_RESULT_TIME_OUT){
        return RETURN_TIMEOUT;
    }
    
    strncpy((char*)respons_message,(char*)g_command_response_info.m_option_str,CXM150x_RECEIVE_BUF_SIZE);
    respons_message[strlen((char*)g_command_response_info.m_option_str)] = '\0';
    
    return RETURN_OK;
}

// ===========================================================================
//! Command transmission for non-blocking processing
/*!
 *
 * @param [in] cmd: Send command data
 * @param [in] res_func: Callback function when response is received
 * @param [in] parse_func: Result parse callback function when response is received
 * @param [in] ret_struct: Result parse result structure when receiving a response
 * @param [out] respons_message: Receive message storage buffer
 * @par Global variable
 *        [in] none
 *        [out] g_max_time_out_tick_count: Timeout Tick count
 *        [out] g_command_response_callback: Response callback function
 *        [out] g_command_response_parse_func: Result parse callback function when response is received
 *        [out] g_response_parse_struct: Response data structure
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code CXM150x_send_and_register_callback(uint8_t *cmd,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER res_func,CXM150x_RES_PARSE_CALLBACK_FUNC_POINTER parse_func,void *ret_struct){
    g_max_time_out_tick_count = MAX_TIME_OUT_TICK_COUNT;
    CXM150x_return_code ret = send_command(cmd);
    if(ret != RETURN_OK){
        return ret;
    }
    g_command_response_callback = res_func;
    g_command_response_parse_func = parse_func;
    g_response_parse_struct = ret_struct;
    
    return RETURN_OK;
}

// ===========================================================================
//! Command transmission for non-blocking processing (response wait time specified)
/*!
 *
 * @param [in] cmd: Send command data
 * @param [in] res_func: Callback function when response is received
 * @param [in] parse_func: Result parse callback function when response is received
 * @param [in] ret_struct: Receive message storage buffer
 * @param [in] max_wait: Response timeout time
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_max_time_out_tick_count: Timeout Tick count
 *        [out] g_command_response_callback: Response callback function
 *        [out] g_command_response_parse_func: Result parse callback function when response is received
 *        [out] g_response_parse_struct: Response data structure
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code CXM150x_send_and_register_callback_long_wait(uint8_t *cmd,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER res_func,CXM150x_RES_PARSE_CALLBACK_FUNC_POINTER parse_func,void *ret_struct,uint32_t max_wait){
    g_max_time_out_tick_count = max_wait;
    CXM150x_return_code ret = send_command(cmd);
    if(ret != RETURN_OK){
        return ret;
    }
    g_command_response_callback = res_func;
    g_command_response_parse_func = parse_func;
    g_response_parse_struct = ret_struct;
    
    return RETURN_OK;
}

// ===========================================================================
//! Wait for CXM150x power ON message
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_rcv_power_on_message_wait: Receive flag for waiting for CXM150x power ON message
 *        [out] g_max_time_out_tick_count: Timeout Tick count
 *        [out] g_command_send_tick_count: Tick count at the start of transmission
 *
 * @return command transmission result
*/
// ===========================================================================
CXM150x_return_code CXM150x_wait_power_on_message(void){
    g_rcv_power_on_message_wait = CXM150x_UART_DRIVER_FLAG_ON;
    g_max_time_out_tick_count = MAX_POWER_ON_TIME_OUT_TICK_COUNT;
    g_command_send_tick_count = wrapper_CXM150x_get_tick();
    
    uint32_t start_tm = wrapper_CXM150x_get_tick();
    while(1){
        if(g_rcv_power_on_message_wait == CXM150x_UART_DRIVER_FLAG_OFF){
            break;
        }
        CXM150x_trigger_analyse();
        // Check for timeout
        if(wrapper_CXM150x_get_tick() - start_tm > MAX_POWER_ON_TIME_OUT_TICK_COUNT){
            printf_err("CXM150x_wait_power_on_message timeout\r\n");
            return RETURN_TIMEOUT;
        }
    }
    
    return RETURN_OK;
}

// ===========================================================================
//! Start waiting for a power ON message by non-blocking
/*!
 *
 * @param [in] res_func: Response callback function pointer
 * @param [in] parse_func: Response parse callback function pointer
 * @param [out] ret_struct: Response data structure
 * @par Global variable
 *        [in] none
 *        [out] g_command_response_wait_flag: Command response wait flag
 *        [out] g_max_time_out_tick_count: Timeout Tick count
 *        [out] g_rcv_power_on_message_wait: Receive flag for waiting for CXM150x power ON message
 *        [out] g_command_response_callback: Response callback function
 *        [out] g_command_response_parse_func: Result parse callback function when response is received
 *        [out] g_response_parse_struct: Response data structure
 *        [out] g_command_send_tick_count: Tick count at the start of transmission
 * @return processing result
*/
// ===========================================================================
CXM150x_return_code CXM150x_prep_wait_power_on_message(CXM150x_CALLBACK_RESPONSE_FUNC_POINTER res_func,CXM150x_RES_PARSE_CALLBACK_FUNC_POINTER parse_func,void *ret_struct){
    if(g_command_response_wait_flag == CXM150x_UART_DRIVER_FLAG_ON){
        printf_err("BUSY:POWER ON\r\n");
        return RETURN_BUSY;
    }
    
    g_max_time_out_tick_count = MAX_POWER_ON_TIME_OUT_TICK_COUNT;

    g_rcv_power_on_message_wait = CXM150x_UART_DRIVER_FLAG_ON;
    g_command_response_callback = res_func;
    g_command_response_parse_func = parse_func;
    g_response_parse_struct = ret_struct;
    g_command_send_tick_count = wrapper_CXM150x_get_tick();
    
    return RETURN_OK;
}

// ===========================================================================
//! Start waiting for CXM150x power ON message after non-blocking reset command
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_max_time_out_tick_count: Timeout Tick count
 *        [out] g_rcv_power_on_message_wait: Receive flag for waiting for CXM150x power ON message
 * @return processing result
*/
// ===========================================================================
CXM150x_return_code CXM150x_prep_wait_power_on_message_reset(void){
    g_rcv_power_on_message_wait = CXM150x_UART_DRIVER_FLAG_ON;
    g_max_time_out_tick_count = MAX_POWER_ON_TIME_OUT_TICK_COUNT;
    
    return RETURN_OK;
}

// ===========================================================================
//! Get the number of unprocessed messages
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_uart_error_flg: UART error flag
 *        [out] g_analyse_wait_message_cnt_res: Processing response message counter
 *        [out] g_analyse_wait_message_cnt_evt: Processing wait event counter
 * @return number of unprocessed messages
*/
// ===========================================================================
uint32_t get_CXM150x_Rx_message_count(void){
    
    // Perform recovery processing if a UART error has occurred
    if(g_uart_error_flg == CXM150x_UART_DRIVER_FLAG_ON){
        printf_err("uart1 error recovery\r\n");
        CXM150x_init_uart_driver();
        g_uart_error_flg = CXM150x_UART_DRIVER_FLAG_OFF;
    }
    
    return (g_analyse_wait_message_cnt_res + g_analyse_wait_message_cnt_evt);
}

