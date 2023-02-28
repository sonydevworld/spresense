// ==========================================================================
/*!
* @file     CXM150x_Port.c
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

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>

#ifdef CONFIG_ARCH_BOARD_SPRESENSE
#include <unistd.h>
#include <fcntl.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

#include <pthread.h>
#include <arch/chip/uart0.h>
#endif

#include "CXM150x_Port.h"
#include "CXM150x_LIB.h"
#include "CXM150x_GNSS.h"
#include "CXM150x_SYS.h"
#include "CXM150x_TIME.h"
#include "CXM150x_TX.h"

// Timeout period from receiving the first character of UART communication to receiving CR + LF
#define MAX_UART_LINE_TIME_OUT_TICK_COUNT     (3000)

#define UART_RECEIVE_TASK_PRIORITY  110

uint8_t g_uart_error_flg = CXM150x_UART_DRIVER_FLAG_OFF;

static uint32_t g_rcv_cnt = 0;
static uint8_t *g_rcv_buf = NULL;
static uint8_t g_rcv_char = '\0';

#ifdef CONFIG_ARCH_BOARD_SPRESENSE
int g_uart_fd;
int g_fw_updating;
static pthread_t g_uart_recv_thread;
static void uart_recv_main(void);
static bool g_stop_thread = false;
static void (*g_CXM150x_uart_rx_callback)(uint32_t, uint32_t) = NULL;
#endif

// ===========================================================================
//! UART communication buffer specification
/*! Call HAL's HAL_UART_Receive_IT function
 * Specify the receive buffer for UART communication with CXM150x and enable the UART receive interrupt
 * Buffer size must be prepared by the caller for 128 bytes
 *
 * @param [in] * rcv_char_buf: Pointer to receive buffer
 * @param [out] none
 * @par Global variable
 *        [in] huart1: UART handler
 *        [out] g_rcv_buf: Receive interrupt buffer
 *
 * @return interrupt setting result (OK, ERROR, BUSY, TIMEOUT)
*/
// ===========================================================================
int32_t wrapper_CXM150x_set_uart_rx_buf(uint8_t *rcv_char_buf){
    g_rcv_buf = rcv_char_buf;
#ifdef CONFIG_EXTERNALS_ELTRES_ADDON
    if (CXM150x_POWER_OFF == wrapper_CXM150x_get_power()){
        board_gpio_config(ELTRES_PIN_ENABLE, 0, false, false, PIN_FLOAT);
        board_gpio_write(ELTRES_PIN_ENABLE, 0);
    }
#endif
    if (g_uart_recv_thread == (pthread_t)0){
        /* Create UART reception thread */
        struct sched_param param;
        pthread_attr_t attr;
        g_fw_updating = false;
        g_stop_thread = false;
        pthread_attr_init(&attr);
        param.sched_priority = UART_RECEIVE_TASK_PRIORITY;
        pthread_attr_setschedparam(&attr, &param);
        pthread_create(&g_uart_recv_thread,
                       &attr,
                       (pthread_startroutine_t)uart_recv_main,
                       NULL);
        /* Wait a little for the next command to be sent. */
        usleep(1);
    }
    return OK;
}

// ===========================================================================
//! CXM150x power control
/*! Set the power supply state by controlling the ENABLE pin with the HAL_GPIO_WritePin function of HAL
 *
 * @param [in] on_off: 1 for ON, 0 for OFF
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] D9 pin
 *
 * @return none
*/
// ===========================================================================
void wrapper_CXM150x_set_power(CXM150x_power_state on_off){
    pthread_addr_t result;
    if(on_off == CXM150x_POWER_ON){
        // CXM150x power ON
#ifdef ELTRES_PIN_DDC2V
        board_power_control(ELTRES_PIN_DDC2V, true);
        board_unset_reset_gpo(ELTRES_PIN_DDC2V);
#endif

#if defined(CONFIG_EXTERNALS_ELTRES_SPEXEL)
        board_power_control_tristate(ELTRES_PIN_ENABLE, 0);
        board_power_control_tristate(ELTRES_PIN_ENABLE, -1); // Hi-Z
        board_unset_reset_gpo(ELTRES_PIN_ENABLE);
#else
        board_gpio_config(ELTRES_PIN_ENABLE, 0, false, false, PIN_FLOAT);
        board_gpio_write(ELTRES_PIN_ENABLE, 0);
        board_gpio_write(ELTRES_PIN_ENABLE, 1);
#endif
    } else {
        // Power OFF operation
#if defined(CONFIG_EXTERNALS_ELTRES_SPEXEL)
        board_set_reset_gpo(ELTRES_PIN_ENABLE);
        board_power_control(ELTRES_PIN_ENABLE, false);
#else
        board_gpio_write(ELTRES_PIN_ENABLE, 0);
#endif
        // Wait for reception thread exit
        g_stop_thread = true;
        pthread_join(g_uart_recv_thread, &result);
        g_uart_recv_thread = (pthread_t)0;
#ifdef ELTRES_PIN_DDC2V
        board_set_reset_gpo(ELTRES_PIN_DDC2V);
        board_power_control(ELTRES_PIN_DDC2V, false);
#endif
    }
}

// ===========================================================================
//! CXM150x wake-up pin control
/*! Control WAKEUP pin with HAL_GPIO_WritePin function of HAL
 *
 * @param [in] on_off: 1 for ON, 0 for OFF
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] D12 pin
 *
 * @return none
*/
// ===========================================================================

// TBD: This is an obsolete function to keep compatibility.
// We should use wrapper_CXM150x_set_wakeup_pin instead of it.
void wrapper_CXM150x_set_Wakeup_pin(CXM150x_power_state on_off){
  wrapper_CXM150x_set_wakeup_pin(on_off);
}

void wrapper_CXM150x_set_wakeup_pin(CXM150x_wakeup_state high_low){
    
    if(high_low == CXM150x_WAKEUP_H){
        board_gpio_write(ELTRES_PIN_WAKEUP, 1);
    } else {
        board_gpio_write(ELTRES_PIN_WAKEUP, 0);
    }
}

// ===========================================================================
//! CXM150x UART Hi-Z control
/*! Set UART pin to high impedance using HAL_GPIO_Init and HAL_GPIO_WritePin of HAL
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] UART pin
 *
 * @return none
*/
// ===========================================================================
void wrapper_CXM150x_set_uart_Hiz(void){
    // TODO: unnecessary for SPRESENSE??
}

// ===========================================================================
//! Get CXM150x power supply status
/*! Call HAL_GPIO_ReadPin function of HAL to get ENABLE signal status
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] D9 pin
 *        [out] none
 *
 * @return signal On / Off status: 1 for ON, 0 for OFF
 
*/
// ===========================================================================
uint32_t wrapper_CXM150x_get_power(void){
#if defined(CONFIG_EXTERNALS_ELTRES_SPEXEL)
    if ((board_power_monitor_tristate(ELTRES_PIN_ENABLE) == -1) &&
        (board_power_monitor(ELTRES_PIN_DDC2V) == true))
    {
        return CXM150x_POWER_ON;
    } else {
        return CXM150x_POWER_OFF;
    }
#else
    if (board_gpio_read(ELTRES_PIN_ENABLE) == 1){
        return CXM150x_POWER_ON;
    } else {
        return CXM150x_POWER_OFF;
    }
#endif
}

// ===========================================================================
//! Set UART receive callback
/*! This function is mainly used in standalone mode. The callback function is
 * CXM150x_uart_receive_to_buffer_callback by default, but the function can
 * be set by the original function in standalone mode.
 *
 * @param [in] callback: Function pointer to UART receive callback
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_CXM150x_uart_rx_callback: UART receive callback
 *
 * @return none
*/
// ===========================================================================
void wrapper_CXM150x_set_uart_rx_callback(void (*callback)(uint32_t, uint32_t)){
    g_CXM150x_uart_rx_callback = callback;
}

// ===========================================================================
//! UART receive handler
/*! Buffers characters received by UART from CXM150x and continues UART reception until line feed code is received (timeout is fixed at 5 seconds)
 * Call the CXM150x_uart_receive_to_buffer_callback function of the library core when a line feed code is received
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] huart1: UART handler
 *        [in] g_rcv_char: UART receive character
 *        [out] g_rcv_buf: Receive buffer
 *        [out] g_rcv_cnt: Number of characters received
 *        [out] g_uart_error_flg: UART error flag
 *
 * @return none
*/
// ===========================================================================
void wrapper_CXM150x_uart_rx_callback(void){
    int ret;
    // Receive UART communication from CXM150x
    uint32_t st_tick = wrapper_CXM150x_get_tick();
    while(1){
        if(g_rcv_cnt < CXM150x_RECEIVE_BUF_SIZE - 1){       // Subtract one because '\0' requires 1 byte

            if(g_rcv_char == '\n'){ // receive line feed code
                // Set in receive buffer and inclement buffer counter
                g_rcv_buf[g_rcv_cnt++] = g_rcv_char;
                // Receive line feed code
                g_rcv_buf[g_rcv_cnt++] = '\0';
                if (g_CXM150x_uart_rx_callback){
                    g_CXM150x_uart_rx_callback(WRAPPER_UART_RX_FROM_CXM150x,g_rcv_cnt);
                } else {
                    CXM150x_uart_receive_to_buffer_callback(WRAPPER_UART_RX_FROM_CXM150x,g_rcv_cnt);
                }
                g_rcv_cnt = 0;
                g_rcv_buf[0] = '\0';
                break;
            } else {    // Receive characters other than line feed code
                // Set in receive buffer and inclement buffer counter
                g_rcv_buf[g_rcv_cnt++] = g_rcv_char;
                if (g_rcv_char == '\0') {
                  // Ignore if it receives invalid characters.
                  g_rcv_cnt = 0;
                  g_rcv_buf[0] = '\0';
                  break;
                }
                // Make settings to receive the next character
                while (1){
                    ret = read(g_uart_fd, &g_rcv_char, 1);
                    if (ret > 0) break;

                    if ((ret < 0) && (errno != EAGAIN)){
                        // Clear errno
                        set_errno(0);

                        // Turn on error flag and finish reading one line forcibly.
                        g_uart_error_flg = CXM150x_UART_DRIVER_FLAG_ON;
                        g_rcv_char = '\n';
                        break;
                    }
                }

                if (ret < 0)
                {
                  // Turn on error flag and finish reading one line forcibly.
                  g_uart_error_flg = CXM150x_UART_DRIVER_FLAG_ON;
                  g_rcv_char = '\n';
                }
            }
        } else {
            // Number of received bytes exceeded
            g_rcv_cnt = 0;
            g_rcv_buf[0] = '\0';
            printf_err("UART_Receive g_rcv_cnt over\r\n");
            return;
        }
        
         // timeout check
        uint32_t c_time = wrapper_CXM150x_get_tick();
        if((c_time - st_tick) > MAX_UART_LINE_TIME_OUT_TICK_COUNT){
            printf_err("wrapper_CXM150x_uart_rx_callback time out.\r\n");
            // advance the counter because you can get one character
            g_rcv_cnt++;
            return;
        }
    }
}

#ifdef CONFIG_ARCH_BOARD_SPRESENSE
static void uart_recv_main(void)
{
    int ret;

#if defined(CONFIG_EXTERNALS_ELTRES_ADDON)
    g_uart_fd = open("/dev/ttyS2", O_RDWR | O_NONBLOCK);
#else
    cxd56_uart0initialize("/dev/uart0");
    g_uart_fd = open("/dev/uart0", O_RDWR | O_NONBLOCK);
#endif

    while (g_stop_thread == false) {
        if (g_fw_updating) {
            // In FW update, do not read in this thread.
            usleep(1);
            continue;
        }

        ret = read(g_uart_fd, &g_rcv_char, 1);
        if (ret > 0) {
            wrapper_CXM150x_uart_rx_callback();
        } else if (ret < 0) {
            if ((errno != EAGAIN) && (errno != EIO)) {
                printf_err("Error: UART read %d\r\n", errno);
                break;
            }
        }
        usleep(1);
    }

    close(g_uart_fd);
#if !defined(CONFIG_EXTERNALS_ELTRES_ADDON)
    cxd56_uart0uninitialize("/dev/uart0");
#endif
}
#endif

// ===========================================================================
//! UART transmission
/*! Call HAL_UART_Transmit function of HAL and send UART to CXM150x
 *
 * @param [in] * data: Pointer to transmission data buffer
 * @param [in] len: Transmission data length
 * @param [in] wait_count: Timeout time (unit is msec)
 * @param [out] none
 * @par Global variable
 *        [in] huart1: UART handler
 *        [out] none
 *
 * @return transmission result (OK, ERROR, BUSY, TIMEOUT)
*/
// ===========================================================================
CXM150x_return_code wrapper_CXM150x_uart_transmit(uint8_t *data,uint16_t len,uint32_t wait_count){
    size_t  remain_size = len;
    uint8_t *send_addr  = data;
    ssize_t write_size;

    wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_ON);

    while (remain_size > 0){
        write_size = write(g_uart_fd, send_addr, remain_size);
        if (write_size < 0) {
            printf_err("Error: UART write %d\r\n", errno);
            wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_OFF);
            return RETURN_NG;
        }
        send_addr   += write_size;
        remain_size -= write_size;
    }

    // Do not turn off the Wakeup pin when issuing a reset command
    if(strstr((char*)data,"SYS RESET SET") == NULL){
        wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_OFF);
    }
    return RETURN_OK;
}

// ===========================================================================
//! Get tick count
/*! Call HAL_GetTick function of HAL to get tick count
 * Tick count is incremented every 1msec
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return tick count value
*/
// ===========================================================================
uint32_t wrapper_CXM150x_get_tick(void){
    struct   timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / (1000 * 1000));
}

// ===========================================================================
//! Wait processing
/*! Call HAL_Delay function of HAL and perform wait processing for specified msec
 *
 * @param [in] tick: Wait time (unit is msec)
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
void wrapper_CXM150x_delay(uint32_t tick){
    usleep(tick * 1000);
}


// ===========================================================================
//! Enter the microcomputer to the stop mode.
/*! Call HAL functions
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
void wrapper_CXM150x_enter_stop_mode(void){
}

// ===========================================================================
//! Enter the microcomputer to the normal mode.
/*! Call HAL functions
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
void wrapper_CXM150x_resume_stop_mode(void){
}

// ===========================================================================
//! INT_OUT1 handler
/*! Non-weak implementation of the same name function of startup_stm32l073xx.s, call the wrapper_int_out1 function of the library core
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
void wrapper_CXM150x_int_out1(void){
    CXM150x_on_int1();
}

// ===========================================================================
//! INT_OUT2 handler
/*! Non-weak implementation of the same name function of startup_stm32l073xx.s, call the wrapper_int_out2 function of the library core
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
void wrapper_CXM150x_int_out2(void){
    CXM150x_on_int2();
}

// ===========================================================================
//! Reset the microcomputer.
/*! Call HAL functions
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
void wrapper_CXM150x_system_reset(void){
    //NVIC_SystemReset();
}
