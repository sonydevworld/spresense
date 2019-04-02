/****************************************************************************
 * modules/bluetooth/hal/bcm20706/manager/bt_uart_manager.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <arch/board/board.h>
#include <debug.h>
#include <semaphore.h>

#include "manager/bt_uart_manager.h"
#include "manager/bt_freq_lock.h"
#include "bt_util.h"
#include "bt_debug.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DBG_LOG_ERROR btdbg

#ifdef CONFIG_BCM20706_UART_DEV_PATH
#define BT_UART_FILE CONFIG_BCM20706_UART_DEV_PATH
#else
#define BT_UART_FILE "/dev/ttyS2"
#endif

/* One BT HCI packet header include packet type(1 byte),
 * opcode code(1 byte), group code(1 byte),
 * packet length(2 bytes)
 */

#define BT_PACKET_HEADER_LEN      5
#define BT_BUF_MAX_LEN            BT_EVT_DATA_LEN + BT_PACKET_HEADER_LEN

#define PKT_TYPE_IDX 0

#define HCI_PRE_RECV_BYTES 3
#define CTL_PRE_RECV_BYTES 2

#define PKT_HCI_DATA_LEN_IDX 2
#define PKT_HCI_HEAD_LEN 2

#define PKT_CTL_DATA_LEN_L_IDX 3
#define PKT_CTL_DATA_LEN_H_IDX 4

#define PKT_CTL_HEAD_LEN 2

#define FD_SET_UART 0
#define FD_SET_CTRL 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
  CTL_IN = 0,
  CTL_OUT,
  CTL_MAX
} CTL_FD;

typedef enum
{
  CTL_CMD_EXIT = 0,
  CTL_CMD_MAX
} CTL_CMD;

typedef struct
{
  uint16_t dataLen;
  uint8_t buff[BT_BUF_MAX_LEN];
} UART_BUFF;

typedef struct
{
  UART_BUFF inputData;
  int uartFd;
  int ctrlFd[CTL_MAX];
  sem_t uartTxSem;
} UART_MGR_CONTEXT;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static UART_MGR_CONTEXT gCtx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void btUartRecvBytes(UART_BUFF *inputData, uint16_t len)
{
  UART_MGR_CONTEXT *ctx = &gCtx;
  uint8_t *buff = inputData->buff + inputData->dataLen;
  ssize_t readLen = 0;
  uint16_t recvLen = 0;

  while (recvLen != len)
    {
      readLen = read(ctx->uartFd, buff, 1);
      if (readLen < 0)
        {
          DBG_LOG_ERROR("read %s error: %d\n", BT_UART_FILE, errno);
          break;
        }
      else
        {
          recvLen += (uint16_t)readLen;
          ++buff;
        }
    }

  inputData->dataLen += len;
}

static void btUartRecvPacket(UART_BUFF *inputData)
{
  uint8_t *buff = inputData->buff;
  uint16_t rxLen = HCI_PRE_RECV_BYTES;

  btUartRecvBytes(inputData, rxLen);

  switch (buff[PKT_TYPE_IDX])
    {
      case PACKET_HCI:
        rxLen = buff[PKT_HCI_DATA_LEN_IDX];
        btUartRecvBytes(inputData, rxLen);
        break;

      case PACKET_MEDIA:
      case PACKET_CONTROL:
        btUartRecvBytes(inputData, CTL_PRE_RECV_BYTES);
        rxLen = (buff[PKT_CTL_DATA_LEN_L_IDX] | (buff[PKT_CTL_DATA_LEN_H_IDX] << 8));
        btUartRecvBytes(inputData, rxLen);
        break;

      default:
        DBG_LOG_ERROR("unknown packet, type: %02x.\n", buff[PKT_TYPE_IDX]);
        break;

    }
}

static int btIsUartDataReady(UART_MGR_CONTEXT *ctx)
{
  int errcode = 0;
  struct pollfd fdSet[] =
  {
    {.fd = ctx->uartFd,         .events = POLLIN, .revents = 0},
    {.fd = ctx->ctrlFd[CTL_IN], .events = POLLIN, .revents = 0}
  };

  do
    {
      errcode = 0;
      if (poll(fdSet, (sizeof(fdSet) / sizeof(fdSet[0])), -1) < 0)
        {
          errcode = errno;
          DBG_LOG_ERROR("poll error: %d\n", errno);
        }
    }
  while (EINTR == errcode);

  if (fdSet[FD_SET_CTRL].revents & POLLIN)
    {
      return 0;
    }

  if (fdSet[FD_SET_UART].revents & POLLIN)
    {
      return 1;
    }

  return -1;
}

static CTL_CMD btGetCtrlCmd(UART_MGR_CONTEXT *ctx)
{
  CTL_CMD ctrlCmd = CTL_CMD_MAX;
  (void)read(ctx->ctrlFd[CTL_IN], &ctrlCmd, sizeof(ctrlCmd));

  return ctrlCmd;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btUartInitialization
 *
 * Description:
 *   UART initialize for Bluetooth drive.
 *   Open a UART port and store descriptor into context.
 *
 ****************************************************************************/

int btUartInitialization(void)
{
  int ret = 0;
  UART_MGR_CONTEXT *ctx = &gCtx;
  ret = sem_init(&ctx->uartTxSem, 0, 1);
  if (ret)
    {
      btdbg("uart tx semaphore init failed\n");
      return -ENOENT;
    }
  ctx->uartFd = open(BT_UART_FILE, O_RDWR);
  if (ctx->uartFd < 0)
    {
      btdbg("uart open failed\n");
      return -ENOENT;
    }

  ret = pipe(ctx->ctrlFd);
  if (ret < 0)
    {
      ret = -ENOENT;
      (void)close(ctx->uartFd);
      btdbg("init pipe failed\n");
    }

  return ret;
}

/****************************************************************************
 * Name: btUartInitialization
 *
 * Description:
 *   UART initialize for Bluetooth drive.
 *   Open a UART port and store descriptor into context.
 *
 ****************************************************************************/

int btUartFinalization(void)
{
  int ret = 0;
  CTL_CMD cmdExit = CTL_CMD_EXIT;
  UART_MGR_CONTEXT *ctx = &gCtx;
  if (write(ctx->ctrlFd[CTL_OUT], &cmdExit, sizeof(CTL_CMD_EXIT)) < 0)
    {
      btdbg("send uart poll exit cmd error: %d\n", errno);
    }

  return ret;
}

int btWaitingForTxComplete(void)
{
  UART_MGR_CONTEXT *ctx = &gCtx;
  uint32_t txBufLen     = 0;
  int ret               = 0;

  do
    {
      ret = ioctl(ctx->uartFd, FIONSPACE, (uintptr_t)(&txBufLen));
    }
  while ((txBufLen != (CONFIG_UART2_TXBUFSIZE - 1)) && (0 == ret));

  ret = ioctl(ctx->uartFd, TCFLSH, (uintptr_t)NULL);

  return ret;
}

int btWaitTxSem(void)
{
  int ret = 0;
  UART_MGR_CONTEXT *ctx = &gCtx;
  ret = sem_wait(&ctx->uartTxSem);
  if (ret)
    {
      btdbg("wait uart TX semaphore failed\n");
    }
  return ret;
}

int btPostTxSem(void)
{
  int ret = 0;
  UART_MGR_CONTEXT *ctx = &gCtx;
  ret = sem_post(&ctx->uartTxSem);
  if (ret)
    {
      btdbg("Post uart TX semaphore failed\n");
    }
  return ret;
}

int btUartSendData(uint8_t *p, uint16_t len)
{
  int ret = 0;
  ret = btWaitTxSem();
  if (ret)
    {
      btdbg("wait uart TX semaphore failed\n");
      return ret;
    }
  board_bluetooth_enable_sleep(false);

  ret = btUartSendRawData(p, len);
  if (ret)
    {
      btdbg("uart send data failed\n");
      return btPostTxSem();
    }
  board_bluetooth_enable_sleep(true);
  ret = btPostTxSem();
  if (ret)
    {
      btdbg("post uart TX semaphore failed\n");
      return ret;
    }
  return ret;
}

int btUartSendRawData(uint8_t *p, uint16_t len)
{
  UART_MGR_CONTEXT *ctx = &gCtx;
  ssize_t writtenLen    = 0;
  uint16_t sentLen      = 0;
  int ret               = 0;

  do
    {
      writtenLen = write(ctx->uartFd, p, 1);
      if (writtenLen < 0)
        {
          ret = writtenLen;
          DBG_LOG_ERROR("write %s error: %d\n", BT_UART_FILE, errno);
          break;
        }
      else
        {
          sentLen += (uint16_t)writtenLen;
          ++p;
        }
    }
  while (sentLen != len);

  if (ret)
    {
      ret = -EIO;
      return ret;
    }

  ret = btWaitingForTxComplete();
  if (ret)
    {
      ret = -EIO;
      return ret;
    }

  return ret;
}

uint8_t *btUartGetCompleteBuff(uint16_t *len)
{
  UART_MGR_CONTEXT *ctx = &gCtx;
  UART_BUFF *inputData = &ctx->inputData;
  uint8_t *buff = NULL;
  int ret = btIsUartDataReady(ctx);

  if (ret > 0)
    {
      btUartRecvPacket(inputData);
      buff = inputData->buff;
      *len = inputData->dataLen;
    }
  else if (0 == ret)
    {
      if (CTL_CMD_EXIT == btGetCtrlCmd(ctx))
        {
          btWaitTxSem();
          btSetUartBaudrate(BT_LOCK_ROSC_UART_BAUD_RATE);
          btChangeFreLock(BT_LOCK_ROSC_UART_BAUD_RATE);

          buff = NULL;
          ret = close(ctx->uartFd);
          if (ret)
            {
              btdbg("close uart failed\n");
            }
          ret = close(ctx->ctrlFd[CTL_IN]);
          if (ret)
            {
              btdbg("close pipe ctrl_in failed\n");
            }
          ret = close(ctx->ctrlFd[CTL_OUT]);
          if (ret)
            {
              btdbg("close pipe ctrl_out failed\n");
            }
          btPostTxSem();
          ret = sem_destroy(&ctx->uartTxSem);
          if (ret)
            {
              btdbg("destroy uart tx semaphore failed\n");
            }
          memset(ctx, 0, sizeof(UART_MGR_CONTEXT));
        }
    }

  return buff;
}

void btUartReleaseCompleteBuff(void)
{
  gCtx.inputData.dataLen = 0;
}

uint8_t *btUartGetCompleteBuffSingle(uint16_t *len)
{
  uint8_t *buff = btUartGetCompleteBuff(len);
  btUartReleaseCompleteBuff();
  return buff;
}

int btSetUartBaudrate(int baudRate)
{
  UART_MGR_CONTEXT *ctx = &gCtx;
  struct termios tio = {0};
  int ret = 0;

  ret = tcgetattr(ctx->uartFd, &tio);
  ret |= cfsetspeed(&tio, baudRate);
  ret |= tcsetattr(ctx->uartFd, TCSANOW, &tio);
  if (ret)
    {
      DBG_LOG_ERROR("set baud rate failed, error: %d\n", errno);
      ret = -ENODEV;
    }
  return ret;
}

int btUartStartRx(bool isStart)
{
  int ret = 0;
  UART_MGR_CONTEXT *ctx = &gCtx;
  const int bits = TIOCM_RTS;

  if (isStart)
    {
      ret = ioctl(ctx->uartFd, TIOCMBIS, (unsigned long)&bits);
      if (ret)
        {
          DBG_LOG_ERROR("set rts bit, error: %d\n", errno);
          ret = -ENODEV;
        }
    }
  else
    {
      ret = ioctl(ctx->uartFd, TIOCMBIC, (unsigned long)&bits);
      if (ret)
        {
          DBG_LOG_ERROR("clear rts bit, error: %d\n", errno);
          ret = -ENODEV;
        }
    }

  return ret;
}
