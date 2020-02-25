/****************************************************************************
 * proximity/proximity_main_apds9930_int.c
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

#include <nuttx/sensors/apds9930.h>

#ifdef CONFIG_CXD56_SCU
#include <arch/chip/scu.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_PROXIMITY_DEVNAME
#  define CONFIG_EXAMPLES_PROXIMITY_DEVNAME "/dev/proximity0"
#endif

#ifndef CONFIG_EXAMPLES_PROXIMITY_SIGNO
#  define CONFIG_EXAMPLES_PROXIMITY_SIGNO 13
#endif

#define MY_TIMER_SIGNAL 17

struct proximity_data_s
{
  int16_t ps;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_sensing_data(int fd)
{
  int ret;
  uint16_t ps;
  uint8_t status = 0;

  ret = read(fd, &ps, 2);
  if (ret < 0)
    {
      printf("Read error %d\n", ret);
    }
  else
    {
      printf("PS:%04x  ", ps);
    }

  ret = ioctl(fd, SNIOC_GETINTSTATUS, (unsigned long)((uintptr_t)&status));
  if (ret < 0)
    {
      printf("GETINTSTATUS failed. %d\n", ret);
    }
  else
    {
      printf("STATUS:%02x  ", status);
      if (status & 0x20)
        {
          printf("INT Active!!\n");
          ret = ioctl(fd, SNIOC_CLEARPSINT, 0);
          if (ret < 0)
            {
              printf("CLEARINT failed. %d\n", ret);
            }
        }
      else
        {
          printf("\n");
        }
    }
}

static int sensing_main(int fd)
{
  sigset_t           set;
  struct sigevent    sev;
  struct itimerspec  timer;
  timer_t timerid;
  int ret;

  sev.sigev_notify            = SIGEV_SIGNAL;
  sev.sigev_signo             = MY_TIMER_SIGNAL;
  ret = timer_create(CLOCK_REALTIME, &sev, &timerid);
  if (ret != OK)
    {
      printf("timer_create failed. %d\n", ret);
      return 0;
    }

  timer.it_value.tv_sec     = 0;
  timer.it_value.tv_nsec    = 250 * 1000 * 1000;
  timer.it_interval.tv_sec  = timer.it_value.tv_sec;
  timer.it_interval.tv_nsec = timer.it_value.tv_nsec;
  timer_settime(timerid, 0, &timer, NULL);

  sigemptyset(&set);
  sigaddset(&set, MY_TIMER_SIGNAL);

  for (;;)
    {
      ret = sigwaitinfo(&set, NULL);
      if (ret < 0)
        {
          int errcode = errno;
          if (errcode == EINTR)
            {
              continue;
            }
          fprintf(stderr, "sigwaitinfo() failed. %d\n", errcode);
          return -1;
        }

      show_sensing_data(fd);
    }

  return 0;
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
  int ret;

  printf("Sensing start...\n");

  fd = open(CONFIG_EXAMPLES_PROXIMITY_DEVNAME, O_RDONLY);
  if (fd < 0)
    {
      printf("Device %s open failure. %d\n",
             CONFIG_EXAMPLES_PROXIMITY_DEVNAME, fd);
      return -1;
    }

  ret = ioctl(fd, SNIOC_SETPSLTHRESHOLD, 0x0000);
  if (ret < 0)
    {
      printf("SETTHRESHOLD failed. %d\n", ret);
      return -1;
    }

  ret = ioctl(fd, SNIOC_SETPSHTHRESHOLD, 0x0200);
  if (ret < 0)
    {
      printf("SETTHRESHOLD failed. %d\n", ret);
      return -1;
    }

  ret = ioctl(fd, SNIOC_SETPSPERSISTENCE, 0x02);
  if (ret < 0)
    {
      printf("SETPERSISTENCE failed. %d\n", ret);
      return -1;
    }

  ret = ioctl(fd, SNIOC_STARTPSMEASUREMENT, 0);
  if (ret < 0)
    {
      printf("STARTMEASUREMENT failed. %d\n", ret);
      return -1;
    }

  sensing_main(fd);

  close(fd);

  return 0;
}
