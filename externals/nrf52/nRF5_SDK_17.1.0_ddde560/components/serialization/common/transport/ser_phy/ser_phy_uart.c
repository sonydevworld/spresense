/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 * Copyright 2022 Sony Semiconductor Solutions Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <sched.h>
#include <poll.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/tioctl.h>
#include "sdk/config.h"


#include "ser_phy.h"
#include "ser_config.h"
#ifdef SER_CONNECTIVITY
    #include "ser_phy_config_conn.h"
#else
//    #include "ser_phy_config_app.h"
#endif
//#include "nrf_drv_uart.h"
#include "app_error.h"
#include "app_util.h"
#include "app_util_platform.h"
#include "nrf_error.h"
#include "sdk_config.h"

#define NRF_LOG_MODULE_NAME ser_phy_uart
#if SER_PHY_UART_CONFIG_LOG_ENABLED
    #define NRF_LOG_LEVEL SER_PHY_UART_CONFIG_LOG_LEVEL
#else
    #define NRF_LOG_LEVEL 0
#endif
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define BLE_UART_FILE "/dev/ttyS2"

#define FD_SET_UART 0
#define FD_SET_CTRL 1

#define NRF52_UARTTASK_NAME "nrf52_uart_task"
#define NRF52_UARTTASK_STACKSIZE (2048)

typedef enum {
  CTL_IN = 0,
  CTL_OUT,
  CTL_MAX
} CTL_FD;

typedef enum {
  CTL_CMD_EXIT = 0,
  CTL_CMD_MAX
} CTL_CMD;

typedef struct
{
  int uart_fd;
  int ctrl_fd[CTL_MAX];
} UART_CONTEXT;

static UART_CONTEXT g_ctx;
static pid_t   m_rx_tid;
static uint8_t m_rx_loop = 0;

static uint8_t * mp_tx_stream; /**< Pointer to Tx data */
static uint16_t  m_tx_stream_length; /**< Length of Tx data including SER_PHY header */
static uint16_t  m_tx_stream_index; /**< Byte index in Tx data */
static uint8_t   m_tx_length_buf[SER_PHY_HEADER_SIZE]; /**< Buffer for header of Tx packet */

static uint8_t * mp_rx_stream; /**< Pointer to Rx buffer */
static uint16_t  m_rx_stream_length; /**< Length of Rx data including SER_PHY header*/
static uint16_t  m_rx_stream_index; /**< Byte index in Rx data */
static uint8_t   m_rx_length_buf[SER_PHY_HEADER_SIZE]; /**< Buffer for header of Rx packet */
static uint8_t   m_rx_drop_buf[1]; /**< 1-byte buffer used to trash incoming data */
static uint8_t   m_rx_byte; /**< Rx byte passed from low-level driver */

static ser_phy_events_handler_t m_ser_phy_event_handler; /**< Event handler for upper layer */
static ser_phy_evt_t            m_ser_phy_rx_event; /**< Rx event for upper layer notification */
static ser_phy_evt_t            m_ser_phy_tx_event; /**< Tx event for upper layer notification */

/**
 *@brief Callback for requesting from upper layer memory for an incomming packet.
 */
static __INLINE void callback_mem_request(void)
{
  m_rx_stream_length = uint16_decode(m_rx_length_buf) + SER_PHY_HEADER_SIZE;
  m_ser_phy_rx_event.evt_type = SER_PHY_EVT_RX_BUF_REQUEST;
  m_ser_phy_rx_event.evt_params.rx_buf_request.num_of_bytes =
    m_rx_stream_length - SER_PHY_HEADER_SIZE;

  if (m_ser_phy_event_handler != NULL)
  {
    m_ser_phy_event_handler(m_ser_phy_rx_event);
  }
}

/**
 *@brief Callback for notifying upper layer that either a packet was succesfully received or it was
 * dropped.
 */
static __INLINE void callback_packet_received(void)
{
//  NRF_LOG_DEBUG("callback_packet_received\n");
  if (mp_rx_stream == m_rx_drop_buf)
  {
    m_ser_phy_rx_event.evt_type = SER_PHY_EVT_RX_PKT_DROPPED;
    NRF_LOG_DEBUG("callback_packet_received : dropped\n");
  }
  else
  {
    m_ser_phy_rx_event.evt_type = SER_PHY_EVT_RX_PKT_RECEIVED;
    m_ser_phy_rx_event.evt_params.rx_pkt_received.num_of_bytes =
      m_rx_stream_index - SER_PHY_HEADER_SIZE;
    m_ser_phy_rx_event.evt_params.rx_pkt_received.p_buffer =
      mp_rx_stream;
  }

  mp_rx_stream = NULL;
  m_rx_stream_length = 0;
  m_rx_stream_index  = 0;

  if (m_ser_phy_event_handler != NULL)
  {
    m_ser_phy_event_handler(m_ser_phy_rx_event);
  }
  //NRF_LOG_DEBUG("callback_packet_received END\n");
}

/**
 *@brief Function for handling Rx procedure.
 */
static int ser_phy_uart_rx(uint8_t rx_byte)
{
  int ret = 0;
//  NRF_LOG_DEBUG("ser_phy_uart_rx\n");

  if (mp_rx_stream == NULL )
  {
    //Receive length value and request rx buffer from higher layer
    if (m_rx_stream_index < SER_PHY_HEADER_SIZE)
    {
      m_rx_length_buf[m_rx_stream_index++] = rx_byte;

      if (m_rx_stream_index == SER_PHY_HEADER_SIZE)
      {
        //Request rx buffer from upper layer
        callback_mem_request();
      }
    }
  }
  else if (m_rx_stream_index < m_rx_stream_length)
  {
    //Receive or drop payload
    if (mp_rx_stream == m_rx_drop_buf)
    {
      //Drop incoming data to the one-element drop buffer
      *mp_rx_stream = rx_byte;
      m_rx_stream_index++;
    }
    else
    {
      mp_rx_stream[m_rx_stream_index - SER_PHY_HEADER_SIZE] = rx_byte;
      m_rx_stream_index++;
    }
  }

  //Process RX packet, notify higher layer
  if (m_rx_stream_index == m_rx_stream_length)
  {
    callback_packet_received();
  }
//  NRF_LOG_DEBUG("ser_phy_uart_rx END\n");
  return ret;
}

/**
 *@brief Poll uart data
 */
static int ser_phy_poll_uart_receive(UART_CONTEXT *ctx)
{
  int errcode = 0;
  struct pollfd fdSet[] = {
    {.fd = ctx->uart_fd,         .events = POLLIN, .revents = 0},
      {.fd = ctx->ctrl_fd[CTL_IN], .events = POLLIN, .revents = 0}
    };

  do {
    errcode = 0;
    if (poll(fdSet, (sizeof(fdSet) / sizeof(fdSet[0])), 10) < 0) {
      errcode = errno;
      NRF_LOG_DEBUG("ser_phy_poll_uart_receive: poll error: %d\n", errcode);
    }

    if (fdSet[FD_SET_CTRL].revents & POLLIN) {
      NRF_LOG_DEBUG("ctrl event : called\n");
      return 0;
    }
    if (fdSet[FD_SET_UART].revents & POLLIN) {
      //NRF_LOG_DEBUG("event uart\n");
      return 1;
    }

    /* Need sleep because this thread has higher priority */
    usleep(5000);

  } while ((EINTR == errcode) || (0 == errcode));

  return -1;
}

static int ser_phy_get_ctrl_cmd(UART_CONTEXT *ctx, CTL_CMD *cmd)
{
  ssize_t rx_len = 0;
  *cmd = CTL_CMD_MAX;
  rx_len = read(ctx->ctrl_fd[CTL_IN], cmd, sizeof(CTL_CMD));
  if (rx_len <= 0)
  {
    NRF_LOG_DEBUG("ser_phy_get_ctrl_cmd: rx_len %d errno %d\n", rx_len, -errno);
    return -1;
  }
  NRF_LOG_DEBUG("read exit value = %d\n", *cmd);
  return NRF_SUCCESS;
}

static void ser_phy_null_read(UART_CONTEXT *ctx)
{
  struct pollfd fdSet[] = {
    {.fd = ctx->uart_fd, .events = POLLIN, .revents = 0}
  };

  while (poll(fdSet, (sizeof(fdSet) / sizeof(fdSet[0])), 10) > 0) {
    if (fdSet[0].revents & POLLIN) {
      read(ctx->uart_fd, &m_rx_byte, 1);
      NRF_LOG_DEBUG("ser_phy_null_read: %02X\n", m_rx_byte);
      continue;
    }
  }
}

static int ser_phy_even_parity_set(int fd)
{
  int ret;
  struct termios prm;

  ret = ioctl(fd, TCGETS, &prm);
  if (ret != 0)
    {
      return ret;
    }

  prm.c_cflag |= PARENB;
  prm.c_cflag &= ~PARODD;

  ret = ioctl(fd, TCSETS, &prm);
  if (ret != 0)
    {
      return ret;
    }

  ret = ioctl(fd, TCFLSH, NULL);
  return ret;
}

static void *ser_phy_uart_receive(void *param)
{
  ssize_t rx_len = 0;
  UART_CONTEXT *ctx = &g_ctx;
  int ret = 0;
  CTL_CMD cmd;

  NRF_LOG_DEBUG("ser_phy_uart_receive: start\n");
  while (m_rx_loop)
  {
    ret = ser_phy_poll_uart_receive(ctx);
    if (ret == 1)
    {
      rx_len = read(ctx->uart_fd, &m_rx_byte, 1);
      if (rx_len <= 0)

      {
        NRF_LOG_DEBUG("ser_phy_uart_receive: rx_len %d errno %d\n", rx_len, errno);
      }
      else
      {
        ser_phy_uart_rx(m_rx_byte);
      }
    }
    else if (ret == 0) {
      ret = ser_phy_get_ctrl_cmd(ctx, &cmd);
      if ((ret == NRF_SUCCESS) && (CTL_CMD_EXIT == cmd))
      {
        NRF_LOG_DEBUG("close uart : %d\n", ctx->uart_fd);
        ret = close(ctx->uart_fd);
        if (ret) {
          NRF_LOG_DEBUG("close uart failed\n");
        }
        NRF_LOG_DEBUG("close pipe : %d/%d\n", ctx->ctrl_fd[CTL_IN], ctx->ctrl_fd[CTL_OUT]);
        ret = close(ctx->ctrl_fd[CTL_IN]);
        if (ret) {
          NRF_LOG_DEBUG("close pipe ctrl_in failed\n");
        }
        ret = close(ctx->ctrl_fd[CTL_OUT]);
        if (ret) {
          NRF_LOG_DEBUG("close pipe ctrl_out failed\n");
        }
        m_ser_phy_event_handler = NULL;
        m_rx_loop = 0;
        memset(ctx, 0, sizeof(UART_CONTEXT));
        NRF_LOG_DEBUG("uart task exit\n");
        break;
      }
      else
      {
        NRF_LOG_DEBUG("uart task exit failed\n");
      }
    }
    else
    {
      NRF_LOG_DEBUG("ser_phy_uart_receive: poll error\n");
    }
  }

  return NULL;
}


/** API FUNCTIONS */

uint32_t ser_phy_open(ser_phy_events_handler_t events_handler)
{
  int errcode = 0;
  UART_CONTEXT *ctx = &g_ctx;
  int ret = 0;
  struct sched_param ser_param;
  int prio;

  if (events_handler == NULL)
  {
    return NRF_ERROR_NULL;
  }

  //Check if function was not called before
  if (m_ser_phy_event_handler != NULL)
  {
    return NRF_ERROR_INVALID_STATE;
  }

  //Configure UART and register handler
  //uart_evt_handler is used to handle events produced by low-level uart driver
  m_ser_phy_event_handler = events_handler;

  mp_tx_stream = NULL;
  m_tx_stream_length = 0;
  m_tx_stream_index  = 0;

  mp_rx_stream = NULL;
  m_rx_stream_length = 0;
  m_rx_stream_index  = 0;

  NRF_LOG_DEBUG("ser_phy_open: %s\n", BLE_UART_FILE);
  ctx->uart_fd = open(BLE_UART_FILE, O_RDWR);
  if (ctx->uart_fd < 0)
  {
    NRF_LOG_DEBUG("ser_phy_open: open err %d\n", ctx->uart_fd);
    return ctx->uart_fd;
  }
  ret = ser_phy_even_parity_set(ctx->uart_fd);
  if (ret != 0)
  {
    errcode = errno;
    NRF_LOG_ERROR("ser_phy_open: UART even parity set err %d\n", errcode);
    (void)close(ctx->uart_fd);
    return -errno;
  }

  ser_phy_null_read(ctx);

  ret = pipe(ctx->ctrl_fd);
  if (ret)
  {
    errcode = errno;
    NRF_LOG_DEBUG("ser_phy_open: pipe err %d\n", errcode);
    (void)close(ctx->uart_fd);
    return -errcode;
  }

  m_rx_tid = ERROR;
  m_rx_loop = 1;

  ret = sched_getparam(0, &ser_param);
  if (!ret) {
#if defined(CONFIG_BLUETOOTH_NRF52_UART_PRIORITY)
    prio = CONFIG_BLUETOOTH_NRF52_UART_PRIORITY;
#else
    if (ser_param.sched_priority < sched_get_priority_max(SCHED_FIFO)) {
      prio = ser_param.sched_priority + 1;
    }
    else {
      prio = ser_param.sched_priority;
    }
#endif
    NRF_LOG_DEBUG("ser_phy_open: priority : %d->%d\n", ser_param.sched_priority, prio);

    m_rx_tid = task_create(NRF52_UARTTASK_NAME,
                           prio,
                           NRF52_UARTTASK_STACKSIZE,
                           (main_t)ser_phy_uart_receive,
                           NULL);
  }

  if (m_rx_tid == ERROR)
  {
    errcode = errno;
    NRF_LOG_DEBUG("ser_phy_open: task_create err %d\n", errcode);
    (void)close(ctx->uart_fd);
    (void)close(ctx->ctrl_fd[CTL_IN]);
    (void)close(ctx->ctrl_fd[CTL_OUT]);
    return -errcode;
  }

  NRF_LOG_DEBUG("ser_phy_open: success\n");
  return NRF_SUCCESS;
}

/**
 *@brief Callback for notifying upper layer that a packet was succesfully transmitted
 */
static __INLINE void callback_packet_sent(void)
{
  mp_tx_stream = NULL;
  m_tx_stream_length = 0;
  m_tx_stream_index  = 0;

  m_ser_phy_tx_event.evt_type = SER_PHY_EVT_TX_PKT_SENT;

  if (m_ser_phy_event_handler != NULL)
  {
    m_ser_phy_event_handler(m_ser_phy_tx_event);
  }
}

static int ser_phy_uart_send(void* buf, int size)
{
  int ret = 0;
  ssize_t sz;
  ssize_t rem_sz = size;
  uint8_t *addr = buf;
  NRF_LOG_DEBUG("ser_phy_uart_send size %d\n", size);

  while (rem_sz > 0)
    {
      sz = write(g_ctx.uart_fd, addr, rem_sz);
      if (sz < 0)
        {
          ret = errno;
          NRF_LOG_DEBUG("ser_phy_uart_send: write:err %d\n", ret);
          return -ret;
        }

      addr   += sz;
      rem_sz -= sz;
    }

  return NRF_SUCCESS;
}

uint32_t ser_phy_tx_pkt_send(const uint8_t * p_buffer, uint16_t num_of_bytes)
{
//  NRF_LOG_DEBUG("ser_phy_tx_pkt_send num %d\n", num_of_bytes);
  if (p_buffer == NULL)
  {
    return NRF_ERROR_NULL;
  }
  else if (num_of_bytes == 0)
  {
    return NRF_ERROR_INVALID_PARAM;
  }

  //Check if there is no ongoing transmission at the moment
  if ((mp_tx_stream == NULL) && (m_tx_stream_length == 0) && (m_tx_stream_index == 0))
  {
    uint32_t ret = 0;
    (void) uint16_encode(num_of_bytes, m_tx_length_buf);
    mp_tx_stream = (uint8_t *)p_buffer;
    m_tx_stream_length = num_of_bytes + SER_PHY_HEADER_SIZE;

    //Call tx procedure to start transmission of a packet
    ret = ser_phy_uart_send(&m_tx_length_buf[0], SER_PHY_HEADER_SIZE);
    if (ret) return ret;
    ret = ser_phy_uart_send(&mp_tx_stream[0], num_of_bytes);
    if (ret) return ret;
    callback_packet_sent();
//    NRF_LOG_DEBUG("ser_phy_tx_pkt_send : finished\n");
  }
  else
  {
    return NRF_ERROR_BUSY;
  }

  return NRF_SUCCESS;
}

uint32_t ser_phy_rx_buf_set(uint8_t * p_buffer)
{
  if (m_ser_phy_rx_event.evt_type != SER_PHY_EVT_RX_BUF_REQUEST)
  {
    return NRF_ERROR_INVALID_STATE;
  }

  if (p_buffer != NULL)
  {
    mp_rx_stream = p_buffer;
  }
  else
  {
    mp_rx_stream = m_rx_drop_buf;
  }

  return NRF_SUCCESS;
}

void ser_phy_close(void)
{
  UART_CONTEXT *ctx = &g_ctx;
  CTL_CMD cmd_exit = CTL_CMD_EXIT;
  ssize_t sent_len = 0;
  int ret = 0;

  NRF_LOG_DEBUG("ser_phy_close: called\n");
  do
  {
    sent_len = write(ctx->ctrl_fd[CTL_OUT], &cmd_exit, sizeof(CTL_CMD));
  } while (sent_len == 0);
  if (sent_len < 0)
  {
    NRF_LOG_DEBUG("ser_phy_close: writed err: %d\n", errno);
  }

  if (m_rx_tid != ERROR) {
    ret = waitpid(m_rx_tid, NULL, WEXITED);
    if (ret)
    {
      NRF_LOG_DEBUG("ser_phy_close: waitpid err: %d\n", errno);
    }
  }
  else {
    NRF_LOG_DEBUG("ser_phy_close: task not created\n");
  }

  NRF_LOG_DEBUG("ser_phy_close: end\n");
}

void ser_phy_interrupts_enable(void)
{
  /* not needed */
}

void ser_phy_interrupts_disable(void)
{
  /* not needed */
}
