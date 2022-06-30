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

/* Board-specific power control */

#define POWER_ELTRES_DDC2V  PMIC_GPO(2)
#define POWER_ELTRES_ENABLE PMIC_GPO(3)
#define POWER_ELTRES_SDCARD PMIC_GPO(5)

#define PIN_WAKEUP          PIN_HIF_GPIO0

uint8_t g_uart_error_flg = UART_DRIVER_FLAG_OFF;

static uint32_t g_rcv_cnt = 0;
static uint8_t *g_rcv_buf = NULL;
static uint8_t g_rcv_char = '\0';

#ifdef CONFIG_ARCH_BOARD_SPRESENSE
int g_uart0_fd;
int g_fw_updating;
static pthread_t g_uart0_recv_thid;
static void uart0_recv_main(void);
#endif

// ===========================================================================
//! Description to switch output destination of printf to PC
/*!
 *
 * @param [in] ch: Output data
 * @param [out] none
 * @par Global variable
 *        [in] huart2: UART2 handler
 *        [out] none
 *
 * @return output data
*/
// ===========================================================================
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int32_t __io_putchar(int32_t ch)
#else
#define PUTCHAR_PROTOTYPE int32_t fputc(int32_t ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
#ifdef FOR_STM32_HAL_DRIVER
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
#endif
  return ch;
}

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
#ifdef FOR_STM32_HAL_DRIVER
    return HAL_UART_Receive_IT(&huart1, &g_rcv_char, 1);
#elif defined CONFIG_ARCH_BOARD_SPRESENSE
    return OK;
#endif
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
#ifdef FOR_STM32_HAL_DRIVER
        HAL_GPIO_WritePin(D9_GPO_RST_GPIO_Port, D9_GPO_RST_Pin, GPIO_PIN_SET);
#elif defined CONFIG_ARCH_BOARD_SPRESENSE
        board_power_control_tristate(POWER_ELTRES_ENABLE, -1); // Hi-Z
        board_power_control(POWER_ELTRES_DDC2V, true);

        /* Create UART reception thread */
        struct sched_param param;
        pthread_attr_t attr;

        pthread_attr_init(&attr);
        param.sched_priority = 110; // larger than application task priority
        pthread_attr_setschedparam(&attr, &param);
        pthread_create(&g_uart0_recv_thid,
                       &attr,
                       (pthread_startroutine_t)uart0_recv_main,
                       NULL);
#endif
    } else {
        // Power OFF operation
#ifdef FOR_STM32_HAL_DRIVER
        HAL_GPIO_WritePin(D9_GPO_RST_GPIO_Port, D9_GPO_RST_Pin, GPIO_PIN_RESET);
#elif defined CONFIG_ARCH_BOARD_SPRESENSE
        board_power_control(POWER_ELTRES_ENABLE, false);
        board_power_control(POWER_ELTRES_DDC2V, false);

        // Wait for reception thread exit
        pthread_join(g_uart0_recv_thid, &result);
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
        board_gpio_write(PIN_WAKEUP, true);
    } else {
        board_gpio_write(PIN_WAKEUP, false);
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
#ifdef FOR_STM32_HAL_DRIVER
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
#elif defined CONFIG_ARCH_BOARD_SPRESENSE
    // TODO: unnecessary for SPRESENSE??
#endif
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
#ifdef FOR_STM32_HAL_DRIVER
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(D9_GPO_RST_GPIO_Port, D9_GPO_RST_Pin);
    if(pin_state == GPIO_PIN_SET){
        return CXM150x_POWER_ON;
    } else {
        return CXM150x_POWER_OFF;
    }
#elif defined CONFIG_ARCH_BOARD_SPRESENSE
    if ((board_power_monitor_tristate(POWER_ELTRES_ENABLE) == -1) &&
        (board_power_monitor(POWER_ELTRES_DDC2V) == true)){
        return CXM150x_POWER_ON;
    } else {
        return CXM150x_POWER_OFF;
    }
#endif
}

// ===========================================================================
//! UART receive handler
/*! Buffers characters received by UART from CXM150x and continues UART reception until line feed code is received (timeout is fixed at 5 seconds)
 * Call the uart_receive_to_buffer_callback function of the library core when a line feed code is received
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
        if(g_rcv_cnt < RECEIVE_BUF_SIZE - 1){       // Subtract one because '\0' requires 1 byte

            if(g_rcv_char == '\n'){ // receive line feed code
                // Set in receive buffer and inclement buffer counter
                g_rcv_buf[g_rcv_cnt++] = g_rcv_char;
                // Receive line feed code
                g_rcv_buf[g_rcv_cnt++] = '\0';
                uart_receive_to_buffer_callback(WRAPPER_UART_RX_FROM_CXM150x,g_rcv_cnt);
                g_rcv_cnt = 0;
                g_rcv_buf[0] = '\0';
#ifdef FOR_STM32_HAL_DRIVER
                HAL_UART_Receive_IT(&huart1, &g_rcv_char, 1);
#endif
                break;
            } else {    // Receive characters other than line feed code
                // Set in receive buffer and inclement buffer counter
                g_rcv_buf[g_rcv_cnt++] = g_rcv_char;

                // Make settings to receive the next character
#ifdef FOR_STM32_HAL_DRIVER
                int32_t set_result = HAL_UART_Receive(&huart1,&g_rcv_char,1,MAX_TIME_OUT_TICK_COUNT);

                // Setting error check
                if(set_result != 0){
                    if(g_rcv_cnt > 0){
                        g_rcv_cnt--;
                    }
                    g_uart_error_flg = UART_DRIVER_FLAG_ON;
                }
#elif defined CONFIG_ARCH_BOARD_SPRESENSE
                while (1){
                    ret = read(g_uart0_fd, &g_rcv_char, 1);
                    if (ret > 0) break;

                    if (errno != 0){
                        // Clear errno
                        set_errno(0);

                        // Turn on error flag and finish reading one line forcibly.
                        g_uart_error_flg = UART_DRIVER_FLAG_ON;
                        g_rcv_char = '\n';
                        break;
                    }
                }

                if (ret < 0)
                {
                  // Turn on error flag and finish reading one line forcibly.
                  g_uart_error_flg = UART_DRIVER_FLAG_ON;
                  g_rcv_char = '\n';
                }
#endif
            }
        } else {
            // Number of received bytes exceeded
            g_rcv_cnt = 0;
            g_rcv_buf[0] = '\0';
#ifdef FOR_STM32_HAL_DRIVER
            HAL_UART_Receive_IT(&huart1, &g_rcv_char, 1);
#endif
            printf("UART_Receive g_rcv_cnt over\r\n");
            return;
        }
        
         // timeout check
        uint32_t c_time = wrapper_CXM150x_get_tick();
        if((c_time - st_tick) > MAX_UART_LINE_TIME_OUT_TICK_COUNT){
            printf("wrapper_CXM150x_uart_rx_callback time out.\r\n");
            // advance the counter because you can get one character
            g_rcv_cnt++;
            return;
        }
    }
}

#ifdef CONFIG_ARCH_BOARD_SPRESENSE
static void uart0_recv_main(void)
{
  int ret;

  while (wrapper_CXM150x_get_power() == CXM150x_POWER_ON)
    {
      g_fw_updating = false;
      cxd56_uart0initialize("/dev/uart0");
      g_uart0_fd = open("/dev/uart0", O_RDWR | O_NONBLOCK);
      while ((g_uart_error_flg == UART_DRIVER_FLAG_OFF) &&
             (wrapper_CXM150x_get_power() == CXM150x_POWER_ON))
        {
          if (g_fw_updating)
            {
              // In FW update, do not read in this thread.
              usleep(1);
              continue;
            }

          ret = read(g_uart0_fd, &g_rcv_char, 1);
          if (ret > 0)
            {
              wrapper_CXM150x_uart_rx_callback();
            }
          else
            {
              if (errno != 0)
                {
                  // Clear errno
                  set_errno(0);

                  // Break to recover UART0
                  break;
                }
            }
          usleep(1);
        }
      usleep(1);
      close(g_uart0_fd);
      cxd56_uart0uninitialize("/dev/uart0");
    }
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
return_code wrapper_CXM150x_uart_transmit(uint8_t *data,uint16_t len,uint32_t wait_count){
#ifdef FOR_STM32_HAL_DRIVER	
    int32_t     ret;
    char    uartState;
    uint32_t st_tick = wrapper_CXM150x_get_tick();

    wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_ON);

    ret = HAL_UART_Transmit_IT(&huart1, data, len);

    if(ret == HAL_OK){
        while(1){
            uartState = HAL_UART_GetState(&huart1);
            //b0     Tx state
            //   0  : Ready (no Tx operation ongoing)
            //   1  : Busy (Tx operation ongoing)
            if((uartState & 0x01)==0x00){
                break;
            }
            
            // timeout check
            uint32_t c_time = wrapper_CXM150x_get_tick();
            if(c_time - st_tick > wait_count){
                printf("wrapper_CXM150x_uart_transmit time out.\r\n");
                wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_OFF);
                HAL_UART_AbortTransmit_IT(&huart1);
                return RETURN_TIMEOUT;
            }
        }
    }else{
        printf("Error: HAL_UART_Transmit %d\r\n",ret);
        wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_OFF);
        HAL_UART_AbortTransmit_IT(&huart1);
        return RETURN_NG;
    }
    
    // Do not turn off the Wakeup pin when issuing a reset command
    if(strstr((char*)data,"SYS RESET SET") == NULL){
        wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_OFF);
    }

    return RETURN_OK;	
#elif defined CONFIG_ARCH_BOARD_SPRESENSE
    size_t  remain_size = len;
    uint8_t *send_addr  = data;
    ssize_t write_size;

    wrapper_CXM150x_set_Wakeup_pin(CXM150x_POWER_ON);

    while (remain_size > 0){
        write_size  =  write(g_uart0_fd, send_addr, remain_size);
        if (write_size < 0) {
            printf("Error: UART0 write %d\r\n", errno);
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
#endif
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
#ifdef FOR_STM32_HAL_DRIVER
    return HAL_GetTick();
#else
    struct   timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / (1000 * 1000));
#endif
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
#ifdef FOR_STM32_HAL_DRIVER
    HAL_Delay(tick);
#else
    usleep(tick * 1000);
#endif
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
void wrapper_enter_stop_mode(void){
#ifdef FOR_STM32_HAL_DRIVER
    HAL_SuspendTick();
    HAL_PWR_EnableSleepOnExit();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
#endif
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
void wrapper_resume_stop_mode(void){
#ifdef FOR_STM32_HAL_DRIVER
    SystemClock_Config();
    HAL_ResumeTick();
    HAL_PWR_DisableSleepOnExit();
#endif
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
    on_int1();
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
    on_int2();
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
void wrapper_system_reset(void){
    //NVIC_SystemReset();
}
