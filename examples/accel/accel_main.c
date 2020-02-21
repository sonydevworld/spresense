/****************************************************************************
 * accel/accel_main.c
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

#include <arch/chip/scu.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACC_DEVPATH      "/dev/accel0"

#define MY_TIMER_SIGNAL 17

struct three_axis_s
{
  int16_t x;
  int16_t y;
  int16_t z;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int scu_setup(int fd)
{
  int ret;

  /* Set FIFO size to 6 (bytes) * 128 (Hz).
   * This size is enough to store 0.5 second data without overwrite if task has
   * been delayed.
   */

  ret = ioctl(fd, SCUIOC_SETFIFO, 768);
  if (ret < 0)
    {
      fprintf(stderr, "SETFIFO failed. %d\n", ret);
      return ret;
    }

  /* Set sequencer sampling rate 128 Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 2) = 128
   */

  ret = ioctl(fd, SCUIOC_SETSAMPLE, 2);
  if (ret < 0)
    {
      fprintf(stderr, "SETSAMPLE failed. %d\n", ret);
      return ret;
    }

  return OK;
}

static void dump_data(void *buffer, int size)
{
  struct three_axis_s *ta = buffer;
  int i;

  for (i = 0; i < size / sizeof(struct three_axis_s); i++, ta++)
    {
      printf("[%3d] xyz = %6d, %6d, %6d\n", i, ta->x, ta->y, ta->z);
    }
}

static void sensing_main(int fd)
{
  sigset_t           set;
  struct sigevent    sev;
  struct itimerspec  timer;
  timer_t            timerid;
  void              *buffer;
  int                size;
  int                ret;
  int                i;

  /* Allocate memory for picking up sensing data */

  buffer = malloc(768);
  if (!buffer)
    {
      fprintf(stderr, "Memory allocation failure.\n");
      return;
    }
  memset(buffer, 0, 768);

  ret = scu_setup(fd);
  if (ret < 0)
    {
      free(buffer);
      return;
    }

  /* Setup timer to picking up in every second */

  sev.sigev_notify            = SIGEV_SIGNAL;
  sev.sigev_signo             = MY_TIMER_SIGNAL;
  ret = timer_create(CLOCK_REALTIME, &sev, &timerid);
  if (ret != OK)
    {
      fprintf(stderr, "timer_create failed. %d\n", ret);
      free(buffer);
      return;
    }

  timer.it_value.tv_sec     = 0;
  timer.it_value.tv_nsec    = 500 * 1000 * 1000;
  timer.it_interval.tv_sec  = timer.it_value.tv_sec;
  timer.it_interval.tv_nsec = timer.it_value.tv_nsec;

  ret = timer_settime(timerid, 0, &timer, NULL);

  sigemptyset(&set);
  sigaddset(&set, MY_TIMER_SIGNAL);

  /* Start sequencer */

  ret = ioctl(fd, SCUIOC_START, 0);
  if (ret != 0)
    {
      fprintf(stderr, "Sequencer start failed. %d\n", errno);
      free(buffer);
      return;
    }

  size = 0;
  for (i = 0; i < 10; i++)
    {
      /* Waiting for timer signal */

      ret = sigwaitinfo(&set, NULL);
      if (ret < 0)
        {
          int errcode = errno;
          if (errcode == EINTR)
            {
              continue;
            }
          fprintf(stderr, "sigwaitinfo() failed. %d\n", errcode);
          break;
        }

      /* Read sensing data from sequencer FIFO.
       * 768 bytes is samping data size for 1 second.
       * This size is depends on sequencer sampling rate, see ioctl command
       * SCUIOC_SETSAMPLE.
       */

      ret = read(fd, buffer, 768);
      if (ret < 0)
        {
          fprintf(stderr, "Read error %d.\n", errno);
        }
      else
        {
          printf("%d samples read.\n", ret / sizeof(struct three_axis_s));
          size = ret;
        }
    }

  timer_delete(timerid);

  /* Stop sequencer */

  (void) ioctl(fd, SCUIOC_STOP, 0);

  dump_data(buffer, size);
  free(buffer);

  return;
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

  fd = open(ACC_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      fprintf(stderr, "Device %s open failure. %d\n", ACC_DEVPATH, fd);
      return -1;
    }

  sensing_main(fd);

  close(fd);

  return 0;
}
