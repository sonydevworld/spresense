/****************************************************************************
 * hostif/host_i2c_main.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C settings */

#define I2C_SLAVE_ADDR          0x24
#define I2C_SPEED               I2C_SPEED_FAST

/* ICMD defnitions */

#define ICMD_AVAILABLE_SIZE(n)  (0x10 + (n))
#define ICMD_FIXLEN_TRANS(n)    (0x80 + (n))
#define ICMD_VARLEN_TRANS(n)    (0xa0 + (n))

/* User defined buffer id */

#define WRITE_BUFFER            0
#define READ_BUFFER             1
#define VERSION_BUFFER          2
#define TIMESTAMP_BUFFER        3

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct timestamp_s
{
  uint32_t sec;
  uint32_t nsec;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_bufsize
 ****************************************************************************/

static size_t get_bufsize(int fd, int bufid)
{
  struct i2c_msg_s      msg[2];
  struct i2c_transfer_s xfer;
  int                   ret;
  uint8_t               cmd = ICMD_AVAILABLE_SIZE(bufid);
  uint8_t               response[3];

  /* I2C packets */

  msg[0].frequency = I2C_SPEED;
  msg[0].addr      = I2C_SLAVE_ADDR;
  msg[0].buffer    = &cmd;
  msg[0].length    = sizeof(cmd);
  msg[0].flags     = I2C_M_NOSTOP;

  msg[1].frequency = I2C_SPEED;
  msg[1].addr      = I2C_SLAVE_ADDR;
  msg[1].buffer    = response;
  msg[1].length    = sizeof(response);
  msg[1].flags     = I2C_M_READ;

  xfer.msgv = msg;
  xfer.msgc = 2;

  ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)((uintptr_t)&xfer));

  if ((ret < 0) || (response[0] != 0))
    {
      printf("ERROR: failed to get the size of buffer%d: %d\n", bufid, ret);
      return 0;
    }

  return response[1] | (response[2] << 8);
}

/****************************************************************************
 * Name: host_receive
 ****************************************************************************/

static int host_receive(int fd, int bufid, uint8_t *buffer, size_t len,
                        bool lock)
{
  struct i2c_msg_s      msg[2];
  struct i2c_transfer_s xfer;
  int                   ret;
  uint8_t               cmd[3];
  size_t                sz = len - 1;

  if (lock)
    {
      cmd[0] = ICMD_VARLEN_TRANS(bufid);
      cmd[1] = sz & 0xff;
      cmd[2] = ((sz >> 8) & 0x3f) | 0x40;
    }
  else
    {
      cmd[0] = ICMD_FIXLEN_TRANS(bufid);
    }

  /* I2C packets */

  msg[0].frequency = I2C_SPEED;
  msg[0].addr      = I2C_SLAVE_ADDR;
  msg[0].buffer    = cmd;
  msg[0].length    = (lock) ? 3 : 1;
  msg[0].flags     = I2C_M_NOSTOP;

  msg[1].frequency = I2C_SPEED;
  msg[1].addr      = I2C_SLAVE_ADDR;
  msg[1].buffer    = buffer;
  msg[1].length    = len;
  msg[1].flags     = I2C_M_READ;

  xfer.msgv = msg;
  xfer.msgc = 2;

  ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)((uintptr_t)&xfer));

  if ((ret < 0) || (buffer[0] != 0))
    {
      printf("ERROR: failed to get the buffer%d: %d\n", bufid, ret);
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: host_send
 ****************************************************************************/

static int host_send(int fd, int bufid, uint8_t *buffer, size_t len)
{
  struct i2c_msg_s      msg[2];
  struct i2c_transfer_s xfer;
  int                   ret;
  uint8_t               cmd[3];
  size_t                sz = len - 1;

  cmd[0] = ICMD_VARLEN_TRANS(bufid);
  cmd[1] = sz & 0xff;
  cmd[2] = ((sz >> 8) & 0x3f);

  /* I2C packets */

  msg[0].frequency = I2C_SPEED;
  msg[0].addr      = I2C_SLAVE_ADDR;
  msg[0].buffer    = cmd;
  msg[0].length    = 3;
  msg[0].flags     = I2C_M_NOSTOP;

  msg[1].frequency = I2C_SPEED;
  msg[1].addr      = I2C_SLAVE_ADDR;
  msg[1].buffer    = buffer;
  msg[1].length    = len;
  msg[1].flags     = I2C_M_NOSTART;

  xfer.msgv = msg;
  xfer.msgc = 2;

  ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)((uintptr_t)&xfer));

  if (ret < 0)
    {
      printf("ERROR: failed to set the buffer%d: %d\n", bufid, ret);
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: receive_data
 ****************************************************************************/

static void receive_data(int fd)
{
  size_t    len;
  size_t    sz;
  uint8_t  *buffer;
  int       ret;
  int       i;

  /* Get available memory size */

  sz = get_bufsize(fd, READ_BUFFER);

  if (sz <= 0)
    {
      printf("ERROR: failed to get the size of buffer\n");
      return;
    }

  /* Allocate memory with 1 byte of status */

  len = sz + 1;

  buffer = (uint8_t *)malloc(len);

  if (!buffer)
    {
      printf("ERROR: failed to allocate memory\n");
      return;
    }

  /* Receive data from slave */

  ret = host_receive(fd, READ_BUFFER, buffer, len, false);

  if (ret == OK)
    {
      /* Print if to get the data is successful */

      for (i = 1; i < len; i++)
        {
          printf(" %02x", buffer[i]);
        }

      printf("\n");
    }

  free(buffer);
}

/****************************************************************************
 * Name: get_version
 ****************************************************************************/

static void get_version(int fd)
{
  char     *version;
  size_t    len;
  size_t    sz;
  uint8_t  *buffer;
  int       ret;

  /* Get available memory size */

  sz = get_bufsize(fd, VERSION_BUFFER);

  if (sz <= 0)
    {
      printf("ERROR: failed to get the size of buffer\n");
      return;
    }

  /* Allocate memory with 1 byte of status */

  len = sz + 1;

  buffer = (uint8_t *)malloc(len);

  if (!buffer)
    {
      printf("ERROR: failed to allocate memory\n");
      return;
    }

  /* Receive data from slave
   * To get the constant data, add a lock flag for the buffer.
   */

  ret = host_receive(fd, VERSION_BUFFER, buffer, len, true);

  if (ret == OK)
    {
      /* Print if to get the version is successful */

      version = (char *)&buffer[1];

      printf("version=%s (sz=%d)\n", version, sz);
    }

  free(buffer);
}

/****************************************************************************
 * Name: get_timestamp
 ****************************************************************************/

static void get_timestamp(int fd)
{
  struct timestamp_s *ts;
  size_t    len;
  size_t    sz;
  uint8_t  *buffer;
  int       ret;

  /* Get available memory size */

  sz = get_bufsize(fd, TIMESTAMP_BUFFER);

  if (sz <= 0)
    {
      printf("ERROR: failed to get the size of buffer\n");
      return;
    }

  /* Allocate memory with 1 byte of status */

  len = sz + 1;

  buffer = (uint8_t *)malloc(len);

  if (!buffer)
    {
      printf("ERROR: failed to allocate memory\n");
      return;
    }

  /* Receive data from slave */

  ret = host_receive(fd, TIMESTAMP_BUFFER, buffer, len, false);

  if (ret == OK)
    {
      /* Print if to get the timestamp is successful */

      ts = (struct timestamp_s *)&buffer[1];

      printf("sec=%ld nsec=%ld (sz=%d)\n", ts->sec, ts->nsec, sz);
    }

  free(buffer);
}

/****************************************************************************
 * Name: send_data
 ****************************************************************************/

static void send_data(int fd)
{
  static int k = 0;
  size_t    len;
  size_t    sz;
  uint8_t  *buffer;
  int       ret;
  int       i;

  /* Get available memory size */

  sz = get_bufsize(fd, WRITE_BUFFER);

  if (sz <= 0)
    {
      printf("ERROR: failed to get the size of buffer\n");
      return;
    }

  sz = 16;

  /* Allocate memory with 1 byte of dummy */

  len = sz + 1;

  buffer = (uint8_t *)malloc(len);

  if (!buffer)
    {
      printf("ERROR: failed to allocate memory\n");
      return;
    }

  /* Send data to slave */

  for (i = 0; i < sz; i++)
    {
      buffer[i] = (i + k) & 0xff;
    }

  ret = host_send(fd, WRITE_BUFFER, buffer, len);

  if (ret == OK)
    {
      /* Print if to send the data is successful */

      printf("Send done.\n");
    }

  free(buffer);

  /* Change data to send in the next execution */

  k++;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * host_i2c_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int i;
  int fd;

  /* Open the i2c driver */

  fd = open("/dev/i2c0", O_WRONLY);
  if (fd < 0)
    {
      printf("ERROR: failed to open /dev/i2c0: %d\n", fd);
      return ERROR;
    }

  /* Get the version information from slave */

  get_version(fd);

  /* Loopback */

  for (i = 0; i < 10; i++)
    {
      /* Send incremental data to slave */

      send_data(fd);

      /* Wait a moment */

      usleep(10 * 1000);

      /* Receive looped-back data */

      receive_data(fd);
    }

  for (i = 0; i < 10; i++)
    {
      /* Get the timestamp from slave */

      get_timestamp(fd);

      /* Wait a second until the timestamp is updated */

      sleep(1);
    }

  /* Close the i2c driver */

  close(fd);

  return 0;
}
