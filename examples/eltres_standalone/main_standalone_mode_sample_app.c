// ==========================================================================
/*!
* @file     main_standalone_mode_sample_app.c
* @brief    Stand alone mode application
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

#include <nuttx/config.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>
#include "main_standalone_mode_sample_app.h"
#include "CXM150x_SYS.h"
#include "CXM150x_Utility.h"
#include "CXM150x_LIB.h"
#include "CXM150x_Port.h"

// Sample application name definition
#define SAMPLE_APP_NAME   "main_standalone_mode_sample_app"
// Sample application version definition
#define SAMPLE_APP_VER    "1.0.3"

// For standalone mode definitions

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

// Communication wait time with CXM (unit is msec)
#define MAX_TIME_OUT_TICK_COUNT     (5000)

// Timeout period from receiving the first character of UART communication to receiving CR + LF
#define MAX_UART_LINE_TIME_OUT_TICK_COUNT     (3000)

// Timeout period for CXM150x power ON message wait
#define MAX_POWER_ON_TIME_OUT_TICK_COUNT     (10000)

// Mode setting timeout time
#define MAX_SET_MODE_TIME_OUT_TICK_COUNT     (10000)

/* Command string definition */
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

// Flag ON / OFF definition
typedef enum {
    FLAG_OFF = 0,
    FLAG_ON
}FlagOnOff;

// Buffer for UART RX (used in the receive interrupt)
static uint8_t g_rcv_buf[CXM150x_RECEIVE_BUF_SIZE] = "";

// Buffer for event message from CXM150x
static uint8_t g_event_buf[CXM150x_RECEIVE_BUF_SIZE] = "";

// Buffer for response message from CXM150x
static uint8_t g_response_buf[CXM150x_RECEIVE_BUF_SIZE] = "";

// Buffer for LPWA payload data
static uint8_t g_payload_buf[CXM150x_PAYLOAD_LEN] = "";

// Flag whether the host microcontroller is staying in STOP mode or not.
static HOST_Mode g_host_mode = HOST_NORMAL_MODE;

// Flag ON when INT_OUT2 interrupt occurs, OFF when UART is received
static FlagOnOff g_INT_OUT2_flag = FLAG_OFF;

// Additional payload value set position definition
#define ADDITIONAL_PAYLOAD_VAL_START_BIT    (117)
#define ADDITIONAL_PAYLOAD_VAL_BIT_LEN      (11)

// Set to a value other than 0 to enable PoC format send(unencrypted message)
#define TX_POC_USE                          (0)

// Set to a value other than 0 to enable TX Duty event
#define TX_DUTY_USE                          (0)

static CXM150x_return_code send_TX_PLD_SET_command(void);
static uint32_t get_additional_val(void);
static void set_additional_payload_data(uint32_t additional_val);

// ===========================================================================
//! Callback function of UART receive interrupt occurs
/*!
 *
 * @param [in] type_from: UART Sender(fixed value)
 * @param [in] rcv_cnt: UART RX length
 * @param [out] none
 * @par Global variable
 *        [in] g_rcv_buf: UART RX message
 *        [out] g_event_buf: event buffer
 *        [out] g_response_buf: response buffer
 *        [out] g_INT_OUT2_flag: INT_OUT2 flag
 * @return none
*/
// ===========================================================================
static void uart_receive_to_buffer_callback(uint32_t type_from,uint32_t rcv_cnt){
    g_INT_OUT2_flag = FLAG_OFF;
    
    // Check the first byte to determine if it is an event or a response.
    if(g_rcv_buf[0] == CXM150x_EVENT_PREFIX_CHAR){
        // receive event message
        strncpy((char*)g_event_buf,(char*)g_rcv_buf,rcv_cnt);
        g_event_buf[rcv_cnt] = '\0';
    } else if(g_rcv_buf[0] == CXM150x_RESPONSE_PREFIX_CHAR){
        // Receive command response
        strncpy((char*)g_response_buf,(char*)g_rcv_buf,rcv_cnt);
        g_response_buf[rcv_cnt] = '\0';
    } else {
        printf("rcv message error:%s",g_rcv_buf);
    }
}

// ===========================================================================
//! Callback function of INT_OUT2 interrupt occurs
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_INT_OUT2_flag : INT_OUT2 flag
 *        [out] g_host_mode : host stop flag
 * @return none
*/
// ===========================================================================
static void int2_callback(void* msg,uint32_t id){
    if(g_host_mode == HOST_STOP_MODE){
        wrapper_CXM150x_resume_stop_mode();
        g_host_mode = HOST_NORMAL_MODE;
        printf("host resume:int2_callback\r\n");
    }
    g_INT_OUT2_flag = FLAG_ON;
    printf("int2_callback\r\n");
}

// ===========================================================================
//! Functions for setting payload data
/*!
 *
 * @param[in]   st_bit  : Specifies the number of bits to be set in the payload
 * @param[in]   bit_len : Bit length to be specified
 * @param[in]   dt      : Data to be set
 * @param[out]  payload : The destination payload data buffer
 * @par Global variable
 *       [in]   none
 *       [out]  none
 *
 * @return  none
*/
// ===========================================================================
static void set_payload_bits(uint32_t st_bit,uint32_t bit_len,uint32_t dt,uint8_t *payload){
    uint32_t payload_offset_bytes = st_bit / 8;
    uint32_t payload_offset_bits = st_bit % 8;
    uint32_t dt_offset_bit = bit_len - 1;
    
    for(uint32_t i=0;i<bit_len;i++){
        uint8_t dt_bit = ((1 << dt_offset_bit) & dt) > 0 ? 1 : 0;
        uint8_t *tgt_payload_byte = payload + payload_offset_bytes;
        
        uint8_t shift = (7 - payload_offset_bits);
        
        uint8_t msk = 0x01 << shift;
        msk = ~msk;
        *tgt_payload_byte &= msk;
        *tgt_payload_byte |= (dt_bit << shift);
        
        dt_offset_bit--;
        if(payload_offset_bits < 7){
            payload_offset_bits++;
        } else {
            payload_offset_bits = 0;
            payload_offset_bytes++;
        }
    }
}

// ===========================================================================
//! parse payload event message
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_event_buf : event message data from CXM150x
 *        [out] g_payload_buf : store parsed payload data in binary format
 *
 * @return none
*/
// ===========================================================================
static void parse_payload_event_data(void){
    uint8_t payload_data_str[CXM150x_RECEIVE_BUF_SIZE] = "";
    if(CXM150x_get_last_word(g_event_buf,payload_data_str) == CXM150x_RESPONSE_OK){
        CXM150x_ascii_to_bin(payload_data_str,g_payload_buf,CXM150x_PAYLOAD_LEN*2);
    } else {
        printf("payload event parse error");
    }
}

// ===========================================================================
//! check event message data
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_event_buf : event message data from CXM150x
 *        [out] none
 *
 * @return event type
*/
// ===========================================================================
static CXM150x_Event check_event(void){
    if(g_event_buf[0] != '\0'){
        CXM150x_Event ret = CXM150x_EVENT_UNKNOWN_EVENT;
        printf("evt:%s",g_event_buf);
        
        if(strstr((char*)g_event_buf,CXM150x_POWER_ON_EVENT_MESSAGE) != NULL){
            //g_power_on_message_flag = FLAG_ON;
            ret = CXM150x_EVENT_SYS_RESET_POWER_PIN;
        } else if(strstr((char*)g_event_buf,CXM150x_PAYLOAD_EVENT_MESSAGE) != NULL){
            ret = CXM150x_EVENT_TX_PLD;
        } else if(strstr((char*)g_event_buf,CXM150x_SYS_TO_DSLP_EVENT_MESSAGE) != NULL){
            ret = CXM150x_EVENT_SYS_TO_DSLP;
        } else if(strstr((char*)g_event_buf,CXM150x_POWER_DSLP_EVENT_MESSAGE) != NULL){
            ret = CXM150x_EVENT_SYS_RESET_DSLP;
        } else if(strstr((char*)g_event_buf,CXM150x_TX_POC_EVENT_MESSAGE) != NULL){
            ret = CXM150x_EVENT_POC_EN;
        } else if(strstr((char*)g_event_buf,CXM150x_GNSS_TIMER_TOUT_EVENT_MESSAGE) != NULL){
            ret = CXM150x_EVENT_GNSS_TIMER_TOUT;
        } else if(strstr((char*)g_event_buf,CXM150x_SYS_RESET_CMD_EVENT_MESSAGE) != NULL){
            ret = CXM150x_EVENT_SYS_RESET_CMD;
        } else if(strstr((char*)g_event_buf,CXM150x_TX_DUTY_EVENT_MESSAGE) != NULL){
            ret = CXM150x_EVENT_TX_DUTY;
        }else {
            ret = CXM150x_EVENT_UNKNOWN_EVENT;
        }
        return ret;
    } else {
        return CXM150x_EVENT_NONE;
    }
}

// ===========================================================================
//! send command and wait response
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_payload_buf : store parsed payload data in binary format
 *        [in] g_response_buf : buffer for response message from CXM150x
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
static CXM150x_return_code send_and_wait_response(uint8_t *command_str,uint8_t *command_response_str,uint32_t response_wait_tick_count){
    CXM150x_return_code ret = RETURN_NG;
    
    g_response_buf[0] = '\0';
    
    // transmit command
    printf("snd:%s",command_str);
    CXM150x_return_code tx_result = wrapper_CXM150x_uart_transmit(command_str,strlen((char*)command_str),MAX_TIME_OUT_TICK_COUNT);
    if(tx_result != RETURN_OK){
        printf("TX NG(%d)\r\n",tx_result);
        return RETURN_NG;
    }
    
    // wait response
    uint32_t start_tick = wrapper_CXM150x_get_tick();
    while(1){
        if(g_response_buf[0] != '\0'){
            printf("rcv:%s",g_response_buf);
            
            if(strstr((char*)g_response_buf,(char*)command_response_str) != NULL){
                if(CXM150x_check_last_ok_ng(g_response_buf) == CXM150x_RESPONSE_OK){
                    printf("response OK\r\n");
                    ret = RETURN_OK;
                } else {
                    printf("response NG\r\n");
                    ret = RETURN_NG;
                }
                g_response_buf[0] = '\0';
                
                break;
            } else {
                printf("ERROR:%s",g_response_buf);
                g_response_buf[0] = '\0';
            }
        } else if(g_event_buf[0] != '\0'){
            //In some cases, event messages are received from the CXM150x while waiting for a response, so perform an event check.
            CXM150x_Event event = check_event();
            if(event == CXM150x_EVENT_POC_EN){
#if TX_POC_USE
                printf("TX PoC enable message\r\n");
#else
                printf("FATAL error :An unencrypted message\r\n");
                wrapper_CXM150x_system_reset();
#endif
            } else if(event == CXM150x_EVENT_GNSS_TIMER_TOUT){
                printf("GNSS TIMER TOUT event\r\n");
            } else if(event == CXM150x_EVENT_SYS_RESET_CMD){
                // When the "| SYS RESET CMD" event occurs, it is necessary to process the "| SYS RESET CMD" event with priority.
                // "| SYS RESET CMD" event handler is implemented in the main loop, then exit function.
                printf("SYS RESET CMD event\r\n");
                return RETURN_NG;
            } else if(event == CXM150x_EVENT_TX_DUTY){
                printf("TX DUTY event\r\n");
            } else {
                printf("ERROR:%s",g_event_buf);
            }
            g_event_buf[0] = '\0';
        }
        
        // timeout check
        uint32_t current_tick = wrapper_CXM150x_get_tick();
        if(current_tick - start_tick > response_wait_tick_count){
            printf("response timeout\r\n");
            ret = RETURN_TIMEOUT;
            break;
        }

    }
    
    return ret;
}

// ===========================================================================
//! send "TX PLD SET" command and wait response
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
static CXM150x_return_code send_TX_PLD_SET_command(){
    CXM150x_return_code ret = RETURN_NG;
    uint8_t command_buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    
    // creating a command transmission string
    snprintf((char*)command_buf,CXM150x_RECEIVE_BUF_SIZE,"%s %s ",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_PAYLOAD_SET_COMMAND);
    // byte array to ASCII conversion
    for(uint32_t i=0;i<CXM150x_PAYLOAD_LEN;i++){
        uint8_t dt = g_payload_buf[i];
        uint8_t add_ch[3] = "";
        snprintf((char*)add_ch,3,"%02X",dt);
        strncat((char*)command_buf,(char*)add_ch,3);
    }
    strncat((char*)command_buf,"\r\n\0",3);
    
    ret = send_and_wait_response(command_buf,(uint8_t*)CXM150x_PAYLOAD_SET_RESPONSE,MAX_TIME_OUT_TICK_COUNT);
    return ret;
}


// ===========================================================================
//! send "SYS MODE SET" command and wait response
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
static CXM150x_return_code send_SYS_MODE_SET_command(void){
    CXM150x_return_code ret = RETURN_NG;
    uint8_t command_buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    
    // creating a command transmission string
    snprintf((char*)command_buf,CXM150x_RECEIVE_BUF_SIZE,"%s %s %02d\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_SYS_MODE_SET_COMMAND,CXM150x_NORMAL_MODE);

    ret = send_and_wait_response(command_buf,(uint8_t*)CXM150x_SYS_MODE_SET_RESPONSE,MAX_SET_MODE_TIME_OUT_TICK_COUNT);
    return ret;
}

#if TX_POC_USE
// ===========================================================================
//! send "TX POC_EN SET" command and wait response
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
static CXM150x_return_code send_TX_POC_EN_SET_command(){
    CXM150x_return_code ret = RETURN_NG;
    uint8_t command_buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    
    // creating a command transmission string
    snprintf((char*)command_buf,CXM150x_RECEIVE_BUF_SIZE,"%c %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_TX_POC_EN_SET_COMMAND,CXM150x_POC_EN_ON);

    ret = send_and_wait_response(command_buf,(uint8_t*)CXM150x_TX_POC_EN_SET_RESPONSE,MAX_TIME_OUT_TICK_COUNT);
    return ret;
}
#endif

#if TX_DUTY_USE
// ===========================================================================
//! send "TX DUTY SET_EVT" command and wait response
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
static CXM150x_return_code send_TX_DUTY_SET_EVT_command(){
    CXM150x_return_code ret = RETURN_NG;
    uint8_t command_buf[CXM150x_RECEIVE_BUF_SIZE] = "";
    
    // creating a command transmission string
    snprintf((char*)command_buf,CXM150x_RECEIVE_BUF_SIZE,"%c %s %s\r\n",CXM150x_COMMAND_PREFIX_CHAR,CXM150x_TX_DUTY_SET_EVT_COMMAND,CXM150x_TX_DUTY_SET_EVT_ON);

    ret = send_and_wait_response(command_buf,(uint8_t*)CXM150x_TX_DUTY_SET_EVT_RESPONSE,MAX_TIME_OUT_TICK_COUNT);
    return ret;
}
#endif
// ===========================================================================
//! Check unprocessed messages
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_INT_OUT2_flag : INT_OUT2 interrupt occurs
 *        [in] g_event_buf : buffer for event message from CXM150x
 *        [in] g_response_buf : buffer for response message from CXM150x
 *        [out] none
 *
 * @return FLAG_ON if unprocessed messages are exist
*/
// ===========================================================================
static FlagOnOff check_unprosessed_messages(void){
    if(g_INT_OUT2_flag == FLAG_ON || g_response_buf[0] != '\0' || g_event_buf[0] != '\0'){
        return FLAG_ON;
    } else {
        return FLAG_OFF;
    }
}

// ===========================================================================
//! get additional value
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return additional value
*/
// ===========================================================================
static uint32_t get_additional_val(){
    // Get data to be appended to the payload created by the CXM150x
    // Here, as a sample, we set the lower 11 bits of the tick count as the payload.
    uint32_t additional_val = wrapper_CXM150x_get_tick() & 0x7FF;
    return additional_val;
}

// ===========================================================================
//! set additional payload data
/*!
 *
 * @param [in] additional_val : additional data to be appended 
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return additional value
*/
// ===========================================================================
static void set_additional_payload_data(uint32_t additional_val){
    // Overwrite the content of the payload with the value of the argument
    // Here, as a sample, we have overwritten the 11-bit value from the 117th bit of the payload
    set_payload_bits(ADDITIONAL_PAYLOAD_VAL_START_BIT,ADDITIONAL_PAYLOAD_VAL_BIT_LEN,additional_val,g_payload_buf);
}

// ===========================================================================
//! standalone_mode_sample_app main function
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_payload_buf : store parsed payload data in binary format
 *        [in] g_response_buf : buffer for response message from CXM150x
 *        [in] g_host_mode : host stop flag
 *        [out] none
 *
 * @return exit code
*/
// ===========================================================================
int main_standalone_mode_sample_app(void){
    
    // Display version information
    printf("%s:Ver.%s\r\n",SAMPLE_APP_NAME,SAMPLE_APP_VER);
    
    // set UART RX callback
    wrapper_CXM150x_set_uart_rx_callback(uart_receive_to_buffer_callback);

    // set UART RX buffer
    wrapper_CXM150x_set_uart_rx_buf(g_rcv_buf);
    
    // Power ON and set normal mode
    board_gpio_intconfig(ELTRES_PIN_INT_OUT2, INT_RISING_EDGE, false, (xcpt_t)wrapper_CXM150x_int_out2);
    board_gpio_int(ELTRES_PIN_INT_OUT2, true);

    // int2 interrupt callback setting
    register_CXM150x_uart_start_interrupt(NULL,int2_callback);

    // CXM150x power ON
    wrapper_CXM150x_set_wakeup_pin(CXM150x_WAKEUP_H);
    wrapper_CXM150x_set_power(CXM150x_POWER_ON);

    // wait power ON message
    uint32_t start_tick = wrapper_CXM150x_get_tick();
    while(1){
        if(g_event_buf[0] != '\0'){
            if(check_event() == CXM150x_EVENT_SYS_RESET_POWER_PIN){
                g_event_buf[0] = '\0';
                break;
            } else {
                printf("ERROR FATAL:%s",g_event_buf);
                g_event_buf[0] = '\0';
            }
        }
        
        // timeout check
        uint32_t current_tick = wrapper_CXM150x_get_tick();
        if(current_tick - start_tick > MAX_POWER_ON_TIME_OUT_TICK_COUNT){
            printf("power on timeout\r\n");
            break;
        }
    }
#if TX_POC_USE
    send_TX_POC_EN_SET_command();
    if(check_event() != CXM150x_EVENT_SYS_RESET_CMD){
        send_SYS_MODE_SET_command();
    }
#else
    send_SYS_MODE_SET_command();
#endif
    
#if TX_DUTY_USE
    send_TX_DUTY_SET_EVT_command();
#endif
    
    // Infinite loop
    while(1){
        // enter stop mode if unporosessed messages are not exist
        if(check_unprosessed_messages() == FLAG_OFF){
            g_host_mode = HOST_STOP_MODE;
            wrapper_CXM150x_enter_stop_mode();
            // TBD: Implement sleep function
            sleep(1);
            continue;
        }
        
        if(g_event_buf[0] != '\0'){
            // process event message
            CXM150x_Event ret = check_event();
            if(ret == CXM150x_EVENT_TX_PLD){
                parse_payload_event_data();
                g_event_buf[0] = '\0';
                uint32_t additional_val = get_additional_val();
                set_additional_payload_data(additional_val);
                send_TX_PLD_SET_command();
            } else if(ret == CXM150x_EVENT_SYS_TO_DSLP){
                g_event_buf[0] = '\0';
                printf("SYS TO DSLP event\r\n");
                wrapper_CXM150x_set_wakeup_pin(CXM150x_WAKEUP_H);
            } else if(ret == CXM150x_EVENT_SYS_RESET_DSLP){
                g_event_buf[0] = '\0';
                printf("SYS RESET DSLP event\r\n");
#if TX_POC_USE
                send_TX_POC_EN_SET_command();
                if(check_event() != CXM150x_EVENT_SYS_RESET_CMD){
                    send_SYS_MODE_SET_command();
                }
#else
                send_SYS_MODE_SET_command();
#endif
#if TX_DUTY_USE
                send_TX_DUTY_SET_EVT_command();
#endif
            } else if(ret == CXM150x_EVENT_POC_EN){
#if TX_POC_USE
                g_event_buf[0] = '\0';
                printf("TX PoC enable message\r\n");
#else
                printf("FATAL error :An unencrypted message\r\n");
                wrapper_CXM150x_system_reset();
#endif
            } else if(ret == CXM150x_EVENT_GNSS_TIMER_TOUT){
                g_event_buf[0] = '\0';
                printf("GNSS TIMER TOUT event\r\n");
            } else if(ret == CXM150x_EVENT_SYS_RESET_CMD){
                g_event_buf[0] = '\0';
                printf("SYS RESET CMD event\r\n");
#if TX_POC_USE
                send_TX_POC_EN_SET_command();
                if(check_event() != CXM150x_EVENT_SYS_RESET_CMD){
                    send_SYS_MODE_SET_command();
                }
#else
                send_SYS_MODE_SET_command();
#endif
#if TX_DUTY_USE
                send_TX_DUTY_SET_EVT_command();
#endif
            } else if(ret == CXM150x_EVENT_TX_DUTY){
                g_event_buf[0] = '\0';
                printf("TX DUTY event\r\n");
            } else {
                printf("ERROR:%s",g_event_buf);
                g_event_buf[0] = '\0';
            }
            
        } else if(g_response_buf[0] != '\0'){
            printf("rcv:%s",g_response_buf);
            g_response_buf[0] = '\0';
        }
    }
}

