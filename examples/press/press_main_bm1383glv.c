/****************************************************************************
 * press/press_main_bm1383glv.c
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

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/sensors/bm1383glv.h>

#ifdef CONFIG_CXD56_SCU
#include <arch/chip/scu.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_PRESS_DEVNAME
#  define CONFIG_EXAMPLES_PRESS_DEVNAME "/dev/press0"
#endif

#ifndef CONFIG_EXAMPLES_PRESS_SIGNO
#  define CONFIG_EXAMPLES_PRESS_SIGNO 13
#endif

struct three_bytes_meas_s
{
  uint8_t msb;    /** meas value MSB */
  uint8_t lsb;    /** meas value LSB */
  uint8_t xlsb;   /** meas value XLSB */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_data;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void dump_data(struct scutimestamp_s *ts, void *data, int nbytes)
{
  static struct scutimestamp_s prev = { 0, 0 };
  struct scutimestamp_s delta;
  int nrsamples = nbytes / sizeof(struct three_bytes_meas_s);
  int i;
  struct three_bytes_meas_s* meas_press = (struct three_bytes_meas_s*)data;

  for (i = 0; i < nrsamples; i++)
    {
      uint32_t raw_press = (((uint32_t)meas_press[i].msb << 16) |
                            ((uint32_t)meas_press[i].lsb << 8) |
                            ((uint32_t)meas_press[i].xlsb & 0xfc)) >> 2;
      uint32_t press = (raw_press << 8) / 2048;
      uint32_t deci = 0;
      uint32_t temp = 390625;
      int j;
      for(j = 0; j<8; j++)
        {
          if((press & 0xff) & (1 << j))
            {
              deci += temp;
            }
          temp *= 2;
        }
      printf("[%3d] %u.%u raw[%02x %02x %02x]\n", i, press >> 8, deci / 1000000,
                                                  meas_press[i].msb, meas_press[i].lsb, meas_press[i].xlsb);
    }

  delta.sec = ts->sec - prev.sec;
  if (ts->tick < prev.tick)
    {
      /* Borrow from sec */

      delta.tick = 32768 - (prev.tick - ts->tick);
      delta.sec--;
    }
  else
    {
      delta.tick = ts->tick - prev.tick;
    }

  printf("SCU Timestamp: %ld.%05d (%ld.%05d)\n",
         ts->sec, ts->tick, delta.sec, delta.tick);

  prev.sec = ts->sec;
  prev.tick = ts->tick;
}

static int sensing_main(int fd)
{
  sigset_t       set;
  struct siginfo value;
  struct scufifo_wm_s wm;
  struct scutimestamp_s ts;
  struct timespec timeout;
  int ret;

  /* Set FIFO size to 3 bytes * 128 Hz = 384 */

  ret = ioctl(fd, SCUIOC_SETFIFO, sizeof(struct three_bytes_meas_s) * 128);
  if (ret < 0)
    {
      printf("SETFIFO failed. %d\n", ret);
      return ret;
    }

  /* Set sequencer sampling rate 128 Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 2) = 128
   */

  ret = ioctl(fd, SCUIOC_SETSAMPLE, 2);
  if (ret < 0)
    {
      printf("SETSAMPLE failed. %d\n", ret);
      return ret;
    }

  wm.signo = CONFIG_EXAMPLES_PRESS_SIGNO;
  wm.ts = &ts;
  wm.watermark = 128;

  ret = ioctl(fd, SCUIOC_SETWATERMARK, (unsigned long)(uintptr_t)&wm);
  if (ret < 0)
    {
      printf("SETWATERMARK failed. %d\n", ret);
      return ret;
    }

  /* Start sequencer */

  ret = ioctl(fd, SCUIOC_START, 0);
  ASSERT(ret == 0);

  /* Set timeout 2 seconds, SCU may send signal every 1 second. */

  timeout.tv_sec  = 2;
  timeout.tv_nsec = 0;

  for (;;)
    {
      sigemptyset(&set);
      sigaddset(&set, CONFIG_EXAMPLES_PRESS_SIGNO);
      ret = sigtimedwait(&set, &value, &timeout);
      if (ret < 0)
        {
          int errcode = errno;

          if (errcode != EAGAIN)
            {
              fprintf(stderr, "ERROR: sigtimedwait() failed: %d\n", errcode);
              return -errcode;
            }

          printf("Timeout!\n");
          break;
        }

      ret = read(fd, g_data, sizeof(struct three_bytes_meas_s) * 128);
      if (ret < 0)
        {
          int errcode = errno;
          fprintf(stderr, "ERROR: read() failed: %d\n", errcode);
          return -errcode;
        }
      else if (ret != sizeof(struct three_bytes_meas_s) * 128)
        {
          fprintf(stderr, "ERROR: read() unexpected size: %ld vs %d\n",
                  (long)ret, sizeof(struct three_bytes_meas_s) * 128);
          return -EIO;
        }

      dump_data(value.si_value.sival_ptr, g_data, ret);
    }

  ret = ioctl(fd, SCUIOC_STOP, 0);
  ASSERT(ret == 0);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * sensor_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;

  printf("Sensing start...\n");

  g_data = (char *)malloc(1024);
  if (!g_data)
    {
      printf("Memory allocation failure.\n");
      return -1;
    }
  memset(g_data, 0, 1024);

  fd = open(CONFIG_EXAMPLES_PRESS_DEVNAME, O_RDONLY);
  if (fd < 0)
    {
      printf("Device %s open failure. %d\n",
             CONFIG_EXAMPLES_PRESS_DEVNAME, fd);
      free(g_data);
      return -1;
    }

  sensing_main(fd);

  close(fd);

  free(g_data);

  return 0;
}
