/**
 * examples/rs422/rs422_main.c
 *
 *  Copyright (C) 2022, Ixy Design Studio, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_RS422_RE_PIN_I2S_DIN
#define PIN_RS422_NEG_RDEN PIN_I2S0_DATA_IN 
#else
#define PIN_RS422_NEG_RDEN PIN_EMMC_DATA3
#endif

#ifdef CONFIG_EXAMPLES_RS422_DE_PIN_I2S_DOUT
#define PIN_RS422_POS_WREN PIN_I2S0_DATA_OUT 
#else
#define PIN_RS422_POS_WREN PIN_EMMC_DATA2
#endif

#define LED0  PIN_I2S1_DATA_OUT
#define LED1  PIN_I2S1_DATA_IN
#define LED2  PIN_I2S1_LRCK
#define LED3  PIN_I2S1_BCK 

#define RS422_TTY_NAME      "/dev/ttyS2"
#define RS422_TTY_SPEED     (B115200)
#define BUFSIZE_TTY_READ    (CONFIG_UART2_RXBUFSIZE)

#define APP_MODE_LOOPBACK   (0)
#define APP_MODE_MASTER     (1)
#define APP_MODE_SLAVE      (2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct _cmd_reqest_type
{
  uint8_t cmd[4];
  uint32_t offset;
  uint32_t len;
  uint8_t payload[128];
} cmd_reqest_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void gpio_setup_direction(int id, bool input);
static void gpio_set(int id);
static void gpio_clear(int id);
static void init_pins(void);
static int open_and_setup_ttys(void);

static int tty_send_data(int comm_tty, uint8_t *pbuf,
int offset, int remain);
static int tty_recv_data(int comm_tty, uint8_t *pbuf,
int offset, int remain);
static int tty_send_cmd(int comm_tty, cmd_reqest_t *cmd);
static int tty_recv_cmd(int comm_tty, cmd_reqest_t *cmd);
static int handshake_master_cmd(int comm_tty, const char *msg,
const int len, const int offset, cmd_reqest_t *pget_cmd);
static int handshake_master_data(int comm_tty, const int len, uint8_t *pbuf);

static int exec_loopbackmode(int comm_tty);
static int exec_slavemode(int comm_tty);
static int exec_mastermode(int comm_tty);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief Initialize I/O pins
 *
 */

static void
init_pins
(void)
{
  gpio_setup_direction(LED0, false);
  gpio_setup_direction(LED1, false);
  gpio_setup_direction(LED2, false);
  gpio_setup_direction(LED3, false);

  gpio_setup_direction(PIN_RS422_NEG_RDEN, false);
  gpio_setup_direction(PIN_RS422_POS_WREN, false);
}

/**
 * @brief Open /dev/ttySx for RS422 Communicate Devices
 *
 * @return int file descriptor of ttySx
 */

static int
open_and_setup_ttys
(void)
{
  int ret = -1;
  int comm_tty = -1;
  struct termios tio;

  /* Open and Setup RS422_TTY_NAME */

  comm_tty = open(RS422_TTY_NAME, O_RDWR | O_NOCTTY);
  ret = errno;
  if (ret != 0)
    {
      printf("could not open %s\n", RS422_TTY_NAME);
      return -1;
    }

  tcgetattr(comm_tty, &tio);

  tio.c_cflag |= CREAD;   /* Enable receive */
  tio.c_cflag |= CLOCAL;  /* Local line, no modem control */
  tio.c_cflag &= ~CSIZE;  /* Clean the bits */
  tio.c_cflag |= CS8;     /* Data bit 8bit */
  tio.c_cflag &= ~CSTOPB; /* Stop bit 1bit */
  tio.c_cflag &= ~PARENB; /* Paritiy none */
  cfsetspeed(&tio, RS422_TTY_SPEED);

  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN] = 1;

  tcflush(comm_tty, TCIOFLUSH);
  tcsetattr(comm_tty, TCSANOW, &tio);

  fcntl(comm_tty, F_SETFL, O_NONBLOCK);

  return comm_tty;
}

/**
 * @brief GPIO Setup
 *
 * @param id pin number
 * @param input input=true, output=false
 */

static void
gpio_setup_direction
(int id, bool input)
{
  int _gabege_value = 0;
  if (input == true)
    {
      /* Input Pin Config */

      board_gpio_config(id, 0, true, false, PIN_PULLUP);
      _gabege_value = board_gpio_read(id);
    }
  else
    {
      /* Output Pin Config */

      board_gpio_config(id, 0, false, true, 0);
      board_gpio_write(id, _gabege_value);
    }

  return;
}

/**
 * @brief GPIO Write = 1
 *
 * @param id pin number
 */

static void
gpio_set
(int id)
{
  board_gpio_write(id, 1);
}

/**
 * @brief GPIO Write = 0
 *
 * @param id pin number
 */

static void
gpio_clear
(int id)
{
  board_gpio_write(id, 0);
}

/**
 * @brief Data Write to ttySx
 *
 * @param comm_tty file descriptor of ttySx
 * @param pbuf write data
 * @param offset position from head
 * @param remain size of buffer
 * @return int success = 0
 */

static int
tty_send_data
(int comm_tty, uint8_t *pbuf, int offset, int remain)
{
  int ret;
  int done = 0;
  while (remain > 0)
    {
      ret = write(comm_tty, pbuf + offset, remain);
      offset += ret;
      remain -= ret;
      done += ret;
    }

  printf("-> pushed [%d byte]\n", done);
  return 0;
}

/**
 * @brief Data Read from ttySx
 *
 * @param comm_tty file descriptor of ttySx
 * @param pbuf read data
 * @param offset position from head
 * @param remain size of buffer
 * @return int success = 0
 */

static int
tty_recv_data
(int comm_tty, uint8_t *pbuf, int offset, int remain)
{
  int ret;
  struct pollfd fds[1];
  int nfds = 1;
  int done = 0;
  fds[0].fd = comm_tty;
  fds[0].events = POLLIN | POLLRDNORM;

  while (remain > 0)
    {
      ret = poll(fds, nfds, 500);
      while ((ret = poll(fds, nfds, 0)) > 0)
        {
          if (ret > 0)
            {
              if ((fds[0].revents & (POLLIN | POLLRDNORM)) != 0)
                {
                  ret = read(comm_tty, pbuf + offset, remain);
                  offset += ret;
                  remain -= ret;
                  done += ret;
                }
            }
        }
    }

  printf("<- fetched [%d byte]\n", done);
  return 0;
}

/**
 * @brief Command Write from ttySx
 *
 * @param comm_tty file descriptor of ttySx
 * @param cmd write command
 * @return int success = 0
 */

static int
tty_send_cmd
(int comm_tty, cmd_reqest_t *cmd)
{
  return tty_send_data(comm_tty, (uint8_t *)cmd, 0, sizeof(cmd_reqest_t));
}

/**
 * @brief Command Read from ttySx
 *
 * @param comm_tty file descriptor of ttySx
 * @param cmd read command
 * @return int success = 0
 */

static int
tty_recv_cmd
(int comm_tty, cmd_reqest_t *cmd)
{
  return tty_recv_data(comm_tty, (uint8_t *)cmd, 0, sizeof(cmd_reqest_t));
}

/**
 * @brief exec slave main loop
 *
 * @param comm_tty file descriptor of ttySx
 * @return int success = 0
 */

static int
exec_slavemode
(int comm_tty)
{
  int ret;
  cmd_reqest_t recv_req;
  cmd_reqest_t send_rsp;
  uint8_t *data_buffer = NULL;
  int pos;

  do
    {
      memset(&recv_req, 0, sizeof(cmd_reqest_t));
      ret = tty_recv_cmd(comm_tty, &recv_req);
      if (ret < 0)
        {
          printf("could not recv command on [%s message]\n", recv_req.cmd);
          return -1;
        }

      printf("recv cmd[%s]\n", recv_req.cmd);
      if (strcmp((const char *)recv_req.cmd, "ini") == 0)
        {
          memset(&send_rsp, 0, sizeof(cmd_reqest_t));

          if (data_buffer == NULL)
            {
              data_buffer = (uint8_t *)malloc(CONFIG_UART2_RXBUFSIZE);
            }

          if (data_buffer == NULL)
            {
              printf("could not allocate memory"
              " (%dbyte)\n", CONFIG_UART2_RXBUFSIZE);
              strcpy((char *)send_rsp.cmd, "nak");
            }
          else
            {
              for (pos = 0; pos < CONFIG_UART2_RXBUFSIZE; pos++)
                {
                    data_buffer[pos] = (pos & 0xff);
                    printf("... senddata[%d](0x%02x)\n",
                    pos, data_buffer[pos]);
                }

              strcpy((char *)send_rsp.cmd, "ack");
            }

          ret = tty_send_cmd(comm_tty, &send_rsp);
          if (ret < 0)
            {
              printf("could not send command on"
              " [%s message]\n", send_rsp.cmd);
              return -1;
            }
        }
      else if (strcmp((const char *)recv_req.cmd, "dat") == 0)
        {
          ret = tty_send_data(comm_tty, data_buffer,
          0, CONFIG_UART2_RXBUFSIZE);
          if (ret < 0)
            {
              printf("could not send data"
              " (%dbyte)\n", CONFIG_UART2_RXBUFSIZE);
              return -1;
            }
        }
      else if (strcmp((const char *)recv_req.cmd, "fin") == 0)
        {
          memset(&send_rsp, 0, sizeof(cmd_reqest_t));
          if (data_buffer != NULL)
            {
              free(data_buffer);
            }

          strcpy((char *)send_rsp.cmd, "ack");
          ret = tty_send_cmd(comm_tty, &send_rsp);
          if (ret < 0)
            {
              printf("could not send command on"
              " [%s message]\n", send_rsp.cmd);
              return -1;
            }
        }
      else
        {
          memset(&send_rsp, 0, sizeof(cmd_reqest_t));
          strcpy((char *)send_rsp.cmd, "nak");
          ret = tty_send_cmd(comm_tty, &send_rsp);
          if (ret < 0)
            {
              printf("could not send command on"
              " [%s message]\n", send_rsp.cmd);
              return -1;
            }
        }
    }
  while (true);

  if (data_buffer != NULL)
    {
        free(data_buffer);
    }

  return 0;
}

/**
 * @brief exec master main loop
 *
 * @param comm_tty file descriptor of ttySx
 * @return int success = 0
 */

static int
exec_mastermode
(int comm_tty)
{
  int ret;
  cmd_reqest_t send_req;
  cmd_reqest_t recv_rsp;
  uint8_t *data_buffer;

  data_buffer = (uint8_t *)malloc(CONFIG_UART2_RXBUFSIZE);
  if (data_buffer == NULL)
    {
      printf("Error : could not allocate memory for recv\n");
      return -1;
    }

  /* ini */

  memset(&send_req, 0, sizeof(cmd_reqest_t));
  strcpy((char *)send_req.cmd, "ini");
  ret = handshake_master_cmd(comm_tty, (const char *)&send_req,
  sizeof(cmd_reqest_t), 0, &recv_rsp);
  if (ret < 0)
    {
      printf("could not send/recv command on [cmd=%s]\n", send_req.cmd);
      free(data_buffer);
      return -1;
    }

  printf("resnponse command is %s\n", recv_rsp.cmd);

  /* dat */

  ret = handshake_master_data(comm_tty, CONFIG_UART2_RXBUFSIZE, data_buffer);
  if (ret < 0)
    {
      printf("could not send/recv command on [cmd=%s]\n", "dat");
      return -1;
    }

  for (int i = 0; i < CONFIG_UART2_RXBUFSIZE; i++)
    {
      if (data_buffer[i] != (i & 0xff))
        {
          printf("data missmatch pos[%d](0x%02x)\n", i, data_buffer[i]);
        }
    }

  free(data_buffer);

  /* fin */

  memset(&send_req, 0, sizeof(cmd_reqest_t));
  strcpy((char *)send_req.cmd, "fin");
  ret = handshake_master_cmd(comm_tty, (const char *)&send_req,
  sizeof(cmd_reqest_t), 0, &recv_rsp);
  if (ret < 0)
    {
      printf("could not send/recv command on [cmd=%s]\n", send_req.cmd);
      return -1;
    }

  printf("resnponse command is %s\n", recv_rsp.cmd);
  return 0;
}

/**
 * @brief exec loopback main loop
 *
 * @param comm_tty file descriptor of ttySx
 * @return int success = 0
 */

static int
exec_loopbackmode
(int comm_tty)
{
  int ret;
  cmd_reqest_t send_req;
  cmd_reqest_t recv_rsp;

  memset(&send_req, 0, sizeof(cmd_reqest_t));
  strcpy((char *)send_req.cmd, "ini");
  ret = handshake_master_cmd(comm_tty, (const char *)&send_req,
  sizeof(cmd_reqest_t), 0, &recv_rsp);
  if (ret < 0)
    {
      printf("could not send/recv command on [cmd=%s]\n", send_req.cmd);
      return -1;
    }

  printf("resnponse command is %s\n", recv_rsp.cmd);

  memset(&send_req, 0, sizeof(cmd_reqest_t));
  strcpy((char *)send_req.cmd, "fin");
  ret = handshake_master_cmd(comm_tty, (const char *)&send_req,
  sizeof(cmd_reqest_t), 0, &recv_rsp);
  if (ret < 0)
    {
      printf("could not send/recv command on [cmd=%s]\n", send_req.cmd);
      return -1;
    }

  printf("resnponse command is %s\n", recv_rsp.cmd);
  return 0;
}

/**
 * @brief RS-422 Command Message Handshake
 *
 * @param comm_tty file descriptor of ttySx
 * @param msg send message
 * @param len length of message
 * @param offset position from head
 * @param pget_cmd received command
 * @return int success = 0
 */

static int
handshake_master_cmd
(int comm_tty, const char *msg, const int len,
const int offset, cmd_reqest_t *pget_cmd)
{
  int ret;
  cmd_reqest_t put_cmd;

  memset(&put_cmd, 0, sizeof(cmd_reqest_t));
  strcpy((char *)put_cmd.cmd, msg);
  put_cmd.len = len;
  put_cmd.offset = offset;
  ret = tty_send_cmd(comm_tty, &put_cmd);
  if (ret < 0)
    {
      printf("could not send command on [%s message]\n", msg);
      return -1;
    }

  ret = tty_recv_cmd(comm_tty, pget_cmd);
  if (ret < 0)
    {
      printf("could not recv command on [%s message]\n", msg);
      return -1;
    }

  return 0;
}

/**
 * @brief RS-422 Data Message Handshake
 *
 * @param comm_tty file descriptor of ttySx
 * @param len length of message
 * @param pbuf received message
 * @return int success = 0
 */

static int
handshake_master_data
(int comm_tty, const int len, uint8_t *pbuf)
{
  int ret;
  cmd_reqest_t put_cmd;
  int remain = len;
  int offset = 0;

  while (remain > 0)
    {
      int reqsize = (remain > CONFIG_UART2_RXBUFSIZE)?
        CONFIG_UART2_RXBUFSIZE : remain;

      memset(&put_cmd, 0, sizeof(cmd_reqest_t));
      strcpy((char *)put_cmd.cmd, "dat");
      put_cmd.len = reqsize;
      put_cmd.offset = offset;
      ret = tty_send_cmd(comm_tty, &put_cmd);
      if (ret < 0)
        {
          printf("could not send command on [%s message]\n", put_cmd.cmd);
          return -1;
        }

      ret = tty_recv_data(comm_tty, pbuf, offset, reqsize);
      if (ret < 0)
        {
          printf("could not recv command on [%s message]\n", put_cmd.cmd);
          return -1;
        }

      remain -= reqsize;
      offset += reqsize;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief RS-422 Communication Examples
 *
 * @param argc please set 1 argment.
 * @param argv please set "-m" master, "-s" slave, "-l" loopback
 * @return int success = 0
 */

int
main
(int argc, char *argv[])
{
  int ret;
  int comm_tty = -1;
  mode_t mode = APP_MODE_LOOPBACK;

  printf("RS-422 <Test>\n");
  printf("UART2 Buffer Size = %dbyte\n", CONFIG_UART2_RXBUFSIZE);
  printf("Communicate Data Set Size = %dbyre\n", sizeof(cmd_reqest_t));

  /* init */

  init_pins();
  gpio_clear(LED0);
  gpio_clear(LED1);
  gpio_clear(LED2);
  gpio_clear(LED3);

  comm_tty = open_and_setup_ttys();
  if (comm_tty < 0)
    {
      printf("could not open (%s)\n", RS422_TTY_NAME);
      return -1;
    }

  /* Check Argument */

  if (argc >= 2)
    {
      if (strcmp(argv[1], "-l") == 0)
        {
          mode = APP_MODE_LOOPBACK;
          gpio_set(LED2);
          gpio_set(LED3);
        }
      else if (strcmp(argv[1], "-m") == 0)
        {
          mode = APP_MODE_MASTER;
          gpio_set(LED0);
        }
      else if (strcmp(argv[1], "-s") == 0)
        {
          mode = APP_MODE_SLAVE;
          gpio_set(LED1);
        }
      else
        {
          printf("Error : %s is not supported.\n", argv[1]);
          goto error_on_parsing_arg;
        }
    }
  else
    {
      mode = APP_MODE_LOOPBACK;
    }

  /* Enable RS-422 */

  gpio_clear(PIN_RS422_NEG_RDEN);
  gpio_set(PIN_RS422_POS_WREN);

  switch (mode)
    {
      case APP_MODE_LOOPBACK:
        printf("Loop Back Mode\n");
        ret = exec_loopbackmode(comm_tty);
        break;
      case APP_MODE_MASTER:
        printf("Master Node\n");
        ret = exec_mastermode(comm_tty);
        break;
      case APP_MODE_SLAVE:
        printf("Slave Node\n");
        ret = exec_slavemode(comm_tty);
        break;
      default:
        printf("Error : mode(=%d) is not support mode.\n", mode);
        goto error_on_parsing_arg;
        break;
    }

  if (ret < 0)
    {
      printf("Error: on operating mode(%d).\n", mode);
    }

error_on_parsing_arg:

  /* fini */

  if (comm_tty >= 0)
    {
      close(comm_tty);
    }

  return 0;
}
