/****************************************************************************
 * modules/bluetooth/hal/bcm20706/manager/bt_hci_manager.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <arch/board/board.h>

#include "manager/bt_uart_manager.h"
#include "manager/bt_freq_lock.h"
#include "bt_debug.h"
#include "bt_util.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DBG_LOG_DEBUG btdbg
#define DBG_LOG_ERROR btdbg

#ifndef CONFIG_BCM20706_FIRMWARE_PATH
#define CONFIG_BCM20706_FIRMWARE_PATH "/mnt/spif/bcm20706fw"
#endif /* CONFIG_BCM20706_FIRMWARE_PATH */

#define PACKAGE_SIZE_OF_FILE 40
#define CONTOL_PACKET_GROUP_INDEX 2
#define CONTOL_PACKET_OPCODE_INDEX 1

#define HCI_RESET_CMD {0x01, 0x03, 0x0C, 0x00}
#define HCI_RESET_RES {0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00}
#define HCI_SWITCH_BAUD_RATE_CMD { 0x01, 0x18, 0xFC, 0x06, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA }
#define HCI_SWITCH_BAUD_RATE_RES  { 0x04, 0x0E, 0x04, 0x01, 0x18, 0xFC, 0x00 }
#define HCI_DOWNLOAD_MINIDRIVER_CMD { 0x01, 0x2E, 0xFC, 0x00 }
#define HCI_DOWNLOAD_MINIDRIVER_RES { 0x04, 0x0E, 0x04, 0x01, 0x2E, 0xFC, 0x00 }
#define HCI_HCD_RECORD_HEAD { 0x01, 0x4C, 0xFC, 0x00 }
#define HCI_HCD_RECORD_RES { 0x04, 0x0E, 0x04, 0x01, 0x4C, 0xFC, 0x00 }
#define HCI_LAUNCH_RAM_CMD { 0x01, 0x4E, 0xFC, 0x04, 0xFF, 0xFF, 0xFF, 0xFF }
#define HCI_LAUNCH_RAM_RES  { 0x04, 0x0E, 0x04, 0x01, 0x4E, 0xFC, 0x00 }

#define HCI_AR_RECV_CMD_INDEX 0x00
#define HCI_AR_RECV_LEN_INDEX 0x02
#define HCI_HCD_LAUNCH_CMD    0x4E
#define HCI_AR_RECV_HEAD_LEN  0x03
#define HCI_AR_RECV_ADDR_LEN  0x04

#define HCI_AR_RECV_ADDR_LEN 0x04
#define ADDR_1BYTE 0
#define ADDR_2BYTE 1
#define ADDR_3BYTE 2
#define ADDR_4BYTE 3

#define BT_HCD_BUFF_LEN 256

#define BAUD_RATE_1BYTE 6
#define BAUD_RATE_2BYTE 7
#define BAUD_RATE_3BYTE 8
#define BAUD_RATE_4BYTE 9

#define HCI_AR_HCD_PACKET_MAX_LEN 261
#define HCI_AR_HCD_PACKET_HEAD_LEN 4
#define HCI_AR_HCD_PACKET_ADDR_LEN 4
#define HCI_AR_HCD_PACKET_SIZE_INDEX 3
#define HCI_AR_HCD_PACKET_ADDR_1BYTE 4
#define HCI_AR_HCD_PACKET_ADDR_2BYTE 5
#define HCI_AR_HCD_PACKET_ADDR_3BYTE 6
#define HCI_AR_HCD_PACKET_ADDR_4BYTE 7

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
#ifdef CONFIG_BCM20706_FIRMWARE_IN_FILESYSTEM
  int fd;
#else
  /* nop */
#endif
} FIRMWARE_INFO;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t* bt_read_new_hci_packet(void);
static int bt_send_hci_reset(void);
static int bt_send_update_baud_rate(int newBaudRate);
static int bt_rend_download_minidriver(void);
static int bt_send_hcd_record(uint32_t nAddr, uint32_t nHCDRecSize, uint8_t* arHCDDataBuffer);
static int bt_send_launch_ram(void);
static int bt_hci_trace(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t *gPacketbuff;
static uint32_t hciRecvEvtLen;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bt_firmware_read(uint8_t *buff, uint8_t size, FIRMWARE_INFO frimware)
{
  int ret = 0;
#ifdef CONFIG_BCM20706_FIRMWARE_IN_FILESYSTEM
  uint8_t read_size = 0;

  read_size = read(frimware.fd, buff, size);

  if (read_size != size)
    {
      DBG_LOG_DEBUG("%s: Firmware read failed.\n", __func__);
      ret = -1;
    }
#else
  /* nop */
#endif
  return ret;
}

static int bt_read_hcd_head(int* bIsLaunch, uint32_t* nHCDRecSize, FIRMWARE_INFO frimware)
{
  uint8_t   arRecHeader[HCI_AR_RECV_HEAD_LEN] = {0};

  if (bt_firmware_read(arRecHeader, HCI_AR_RECV_HEAD_LEN, frimware))
    {
      DBG_LOG_DEBUG("%s: Header read failed.\n", __func__);
      return -1;
    }

  *nHCDRecSize = arRecHeader[HCI_AR_RECV_LEN_INDEX] - HCI_AR_RECV_ADDR_LEN;
  *bIsLaunch = (arRecHeader[HCI_AR_RECV_CMD_INDEX] == HCI_HCD_LAUNCH_CMD);
  return 0;
}

static int bt_read_hcd_addr(uint32_t* nAddr, FIRMWARE_INFO frimware)
{
  uint8_t arAddress[HCI_AR_RECV_ADDR_LEN] = {0};

  if (bt_firmware_read(arAddress, HCI_AR_RECV_ADDR_LEN, frimware))
    {
      DBG_LOG_DEBUG("%s: Address read failed.\n", __func__);
      return -1;
    }

  *nAddr = arAddress[ADDR_1BYTE] + (arAddress[ADDR_2BYTE] << BITS8) +
    (arAddress[ADDR_3BYTE] << BITS16) + (arAddress[ADDR_4BYTE] << BITS24);
  return 0;
}

static int bt_read_hcd_data(uint8_t* arHCDDataBuffer, uint32_t* nHCDRecSize, FIRMWARE_INFO frimware)
{
  if (*nHCDRecSize > 0)
    {
      if (bt_firmware_read(arHCDDataBuffer, *nHCDRecSize, frimware))
        {
          DBG_LOG_DEBUG("%s: Firmware data read failed.\n", __func__);
          return -1;
        }
    }
  return 0;
}

static int bt_download_firmware(void)
{
  FIRMWARE_INFO frimware;
  uint32_t nAddr = 0;
  uint32_t nHCDRecSize = 0;
  uint8_t arHCDDataBuffer[BT_HCD_BUFF_LEN] = {0};
  int bIsLaunch = 0;
  int ret = 0;

#ifdef CONFIG_BCM20706_FIRMWARE_IN_FILESYSTEM
  frimware.fd = open(CONFIG_BCM20706_FIRMWARE_PATH, O_RDONLY);
  if (frimware.fd == -1)
    {
      DBG_LOG_DEBUG("%s: Firmware open failed.\n", __func__);
      return -1;
    }
#else
  /* nop */
#endif

  while (true)
    {
      if (bt_read_hcd_head(&bIsLaunch, &nHCDRecSize, frimware))
        {
          DBG_LOG_DEBUG("bt_read_hcd_head failed\n");
          ret = -1;
          goto error;
        }

      if (bt_read_hcd_addr(&nAddr, frimware))
        {
          DBG_LOG_DEBUG("bt_read_hcd_addr failed\n");
          ret = -1;
          goto error;
        }

      if (bt_read_hcd_data(arHCDDataBuffer, &nHCDRecSize, frimware))
        {
          DBG_LOG_DEBUG("bt_read_hcd_data failed\n");
          ret = -1;
          goto error;
        }

      if (bt_send_hcd_record(nAddr, nHCDRecSize, arHCDDataBuffer))
        {
          ret = -1;
          goto error;
        }

      if (bIsLaunch)
        {
          DBG_LOG_DEBUG("launch finish\n");
          break;
        }
    }

error:
#ifdef CONFIG_BCM20706_FIRMWARE_IN_FILESYSTEM
  close(frimware.fd);
#else
  /* nop */
#endif
  return ret;
}

static int bt_hci_trace(void)
{
  uint8_t buff[BT_SHORT_COMMAND_LEN] = {0};
  uint8_t *p = buff;
  UINT8_TO_STREAM(p,PACKET_CONTROL);
  UINT16_TO_STREAM(p, BT_CONTROL_COMMAND_TRACE_ENABLE);
  UINT16_TO_STREAM(p, 2);
  UINT8_TO_STREAM(p, 1);
  UINT8_TO_STREAM(p, 4);
  return btUartSendData(buff, p - buff);
}

static uint8_t* bt_read_new_hci_packet(void)
{
  uint16_t packet_len = 0;
  uint8_t *p = btUartGetCompleteBuffSingle(&packet_len);
  hciRecvEvtLen = (uint32_t) packet_len;
  return p;
}

static int bt_send_hci_reset(void)
{
  uint8_t arHciCommandTx[] = HCI_RESET_CMD;
  uint8_t arBytesExpectedRx[] = HCI_RESET_RES;
  hciRecvEvtLen = 0;
  btUartSendData(arHciCommandTx, sizeof(arHciCommandTx));
  gPacketbuff = bt_read_new_hci_packet();
  if (hciRecvEvtLen == sizeof(arBytesExpectedRx))
    {
      if (memcmp(gPacketbuff, arBytesExpectedRx, hciRecvEvtLen) == 0)
        {
          return 0;
        }
    }
  return -1;
}

static int bt_send_update_baud_rate(int newBaudRate)
{
  uint8_t arHciCommandTx[] = HCI_SWITCH_BAUD_RATE_CMD;
  uint8_t arBytesExpectedRx[] = HCI_SWITCH_BAUD_RATE_RES;

  arHciCommandTx[BAUD_RATE_1BYTE] = newBaudRate & 0xff;
  arHciCommandTx[BAUD_RATE_2BYTE] = (newBaudRate >> BITS8) & 0xff;
  arHciCommandTx[BAUD_RATE_3BYTE] = (newBaudRate >> BITS16) & 0xff;
  arHciCommandTx[BAUD_RATE_4BYTE] = (newBaudRate >> BITS24) & 0xff;

  hciRecvEvtLen = 0;
  btUartSendData(arHciCommandTx, sizeof(arHciCommandTx));
  gPacketbuff = bt_read_new_hci_packet();
  if (hciRecvEvtLen == sizeof(arBytesExpectedRx))
    {
      if (memcmp(gPacketbuff, arBytesExpectedRx, hciRecvEvtLen) == 0)
        {
          return 0;
        }
    }
  return -1;
}

static int bt_rend_download_minidriver(void)
{
  uint8_t arHciCommandTx[] = HCI_DOWNLOAD_MINIDRIVER_CMD;
  uint8_t arBytesExpectedRx[] = HCI_DOWNLOAD_MINIDRIVER_RES;

  hciRecvEvtLen = 0;
  btUartSendData(arHciCommandTx, sizeof(arHciCommandTx));
  gPacketbuff = bt_read_new_hci_packet();
  if (hciRecvEvtLen == sizeof(arBytesExpectedRx))
    {
      if (memcmp(gPacketbuff, arBytesExpectedRx, hciRecvEvtLen) == 0)
        {
          return 0;
        }
    }
  return -1;
}

static int bt_send_hcd_record(uint32_t nAddr, uint32_t nHCDRecSize, uint8_t * arHCDDataBuffer)
{
  uint8_t arHciCommandTx[HCI_AR_HCD_PACKET_MAX_LEN] = HCI_HCD_RECORD_HEAD;
  uint8_t arBytesExpectedRx[] = HCI_HCD_RECORD_RES;

  arHciCommandTx[HCI_AR_HCD_PACKET_SIZE_INDEX] = (int8_t)(HCI_AR_HCD_PACKET_HEAD_LEN + nHCDRecSize);
  arHciCommandTx[HCI_AR_HCD_PACKET_ADDR_1BYTE] = (nAddr & 0xff);
  arHciCommandTx[HCI_AR_HCD_PACKET_ADDR_2BYTE] = (nAddr >> BITS8) & 0xff;
  arHciCommandTx[HCI_AR_HCD_PACKET_ADDR_3BYTE] = (nAddr >> BITS16) & 0xff;
  arHciCommandTx[HCI_AR_HCD_PACKET_ADDR_4BYTE] = (nAddr >> BITS24) & 0xff;
  memcpy(&arHciCommandTx[8], arHCDDataBuffer, nHCDRecSize);
  hciRecvEvtLen = 0;
  btUartSendData(arHciCommandTx, HCI_AR_HCD_PACKET_HEAD_LEN + HCI_AR_HCD_PACKET_ADDR_LEN + nHCDRecSize);
  gPacketbuff = bt_read_new_hci_packet();
  if (hciRecvEvtLen == sizeof(arBytesExpectedRx))
    {
      if (memcmp(gPacketbuff, arBytesExpectedRx, hciRecvEvtLen) == 0)
        {
          return 0;
        }
      else
        {
          DBG_LOG_ERROR("Wrong bytes in the event\n");
        }
    }
  else
    {
      DBG_LOG_DEBUG("Response timeout\n");
    }
  return -1;
}

static int bt_send_launch_ram(void)
{
  uint8_t arHciCommandTx[] = HCI_LAUNCH_RAM_CMD;
  uint8_t arBytesExpectedRx[] = HCI_LAUNCH_RAM_RES;
  hciRecvEvtLen = 0;
  btUartSendData(arHciCommandTx, sizeof(arHciCommandTx));
  gPacketbuff = bt_read_new_hci_packet();
  if (hciRecvEvtLen == sizeof(arBytesExpectedRx)) {
    if (memcmp(gPacketbuff, arBytesExpectedRx, hciRecvEvtLen) == 0) {
      return 0;
    }
  }
  return -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_hci_start_boot
 *
 * Description:
 *   Bluetooth HCI Boot start
 *   Start a bluetooth boot via HCI.
 *
 ****************************************************************************/

int bt_hci_start_boot(void)
{
  int ret = 0;

  if (BT_UART_BAUD_RATE_APP > BT_LOCK_LV_UART_BAUD_RATE)
    {
      btAcquireHighFrequency();
      btReleaseLowFrequency();
    }
  else if (BT_UART_BAUD_RATE_APP > BT_LOCK_ROSC_UART_BAUD_RATE)
    {
      btAcquireLowFrequency();
      btReleaseHighFrequency();
    }

  if (bt_send_hci_reset())
    {
      DBG_LOG_ERROR("Failed to HCI Reset\n");
      return -1;
    }

  if (bt_send_update_baud_rate(BT_UART_BAUD_RATE_APP))
    {
      DBG_LOG_ERROR("Failed to send update baud rate\n");
      return -1;
    }

  ret = btSetUartBaudrate(BT_UART_BAUD_RATE_APP);
  if (ret)
    {
      DBG_LOG_ERROR("Failed to switch baud rate %d\n",ret);
      return -1;
    }

  if (bt_rend_download_minidriver())
    {
      DBG_LOG_ERROR("Failed to send download minidriver\n");
      return -1;
    }

  if (bt_download_firmware())
    {
      DBG_LOG_ERROR("Failed to read firmware from flash\n");
      return -1;
    }

  if (bt_send_launch_ram())
    {
      DBG_LOG_ERROR("Failed to send launch RAM\n");
      return -1;
    }

  gPacketbuff = bt_read_new_hci_packet();
  ret = gPacketbuff[CONTOL_PACKET_OPCODE_INDEX] | (gPacketbuff[CONTOL_PACKET_GROUP_INDEX]<<8);

  if (ret == BT_CONTROL_EVENT_DEVICE_STARTED)
    {
      DBG_LOG_DEBUG("boot finish.\n");
    }
  bt_hci_trace();
  return 0;
}

