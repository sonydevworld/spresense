/****************************************************************************
 * decimator/decimator_main.c
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
#include <pthread.h>

#ifdef CONFIG_CXD56_SCU
#include <arch/chip/scu.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_DECIMATOR0_DEVNAME
#  define DEC0_DEVPATH CONFIG_EXAMPLES_DECIMATOR0_DEVNAME
#else
#  define DEC0_DEVPATH "/dev/accel0"
#endif
#ifdef CONFIG_EXAMPLES_DECIMATOR1_DEVNAME
#  define DEC1_DEVPATH CONFIG_EXAMPLES_DECIMATOR1_DEVNAME
#else
#  define DEC1_DEVPATH "/dev/accel1"
#endif

#ifdef CONFIG_EXAMPLES_DECIMATOR0_SIGNO
#  define DEC0_SIGNO CONFIG_EXAMPLES_DECIMATOR0_SIGNO
#else
#  define DEC0_SIGNO 13
#endif
#ifdef CONFIG_EXAMPLES_DECIMATOR1_SIGNO
#  define DEC1_SIGNO CONFIG_EXAMPLES_DECIMATOR1_SIGNO
#else
#  define DEC1_SIGNO 14
#endif

struct three_axis_s
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct deci_s
{
  struct scutimestamp_s ts;
  int fd;
  int signo;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void *g_buffer;
static struct deci_s g_data[2];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int setup_as_decimated(FAR struct deci_s *d)
{
  struct decimation_s   dec;
  struct scufifo_wm_s   wm;
  int                   ret;

  ret = ioctl(d->fd, SCUIOC_SETFIFO, 384);
  if (ret != 0)
    {
      printf("SETFIFO failed. %d\n", errno);
      return ret;
    }

  ret = ioctl(d->fd, SCUIOC_SETSAMPLE, 4);
  if (ret != 0)
    {
      printf("SETSAMPLE failed. %d\n", errno);
      return ret;
    }

  /*
   * ratio parameter is 1 / (2 ^ ratio), so 1 is (1 / (2 ^ 1)) = 1 / 2.
   * As a result, the number of sampling data per seconds of this decimation
   * setting will be sampling rate / 2.
   * In this case, 16 samples per seconds can be read (sampling rate 32 Hz / 2).
   */

  dec.ratio = 1;
  dec.leveladj = SCU_LEVELADJ_X1;
  dec.forcethrough = 0;

  ret = ioctl(d->fd, SCUIOC_SETDECIMATION, (unsigned long)(uintptr_t)&dec);
  if (ret != 0)
    {
      printf("SETDECIMATION failed. %d\n", errno);
      return ret;
    }

  wm.signo = d->signo;
  wm.ts = &d->ts;
  wm.watermark = 16;

  ret = ioctl(d->fd, SCUIOC_SETWATERMARK, (unsigned long)(uintptr_t)&wm);
  if (ret != 0)
    {
      fprintf(stderr, "SETWATERMARK failed. %d\n", errno);
    }

  return ret;
}

/*
 * This thread is also decimator, but default is "forcethrough" setting,
 * so it acts as normal sequencer.
 */

static int setup_as_normal(FAR struct deci_s *d)
{
  struct scufifo_wm_s wm;
  int ret;

  ret = ioctl(d->fd, SCUIOC_SETFIFO, 384);
  if (ret != 0)
    {
      printf("SETFIFO failed. %d\n", errno);
      return ret;
    }

  ret = ioctl(d->fd, SCUIOC_SETSAMPLE, 4);
  if (ret != 0)
    {
      printf("SETSAMPLE failed. %d\n", errno);
      return ret;
    }

  wm.signo = d->signo;
  wm.ts = &d->ts;
  wm.watermark = 32;

  ret = ioctl(d->fd, SCUIOC_SETWATERMARK, (unsigned long)(uintptr_t)&wm);
  if (ret != 0)
    {
      printf("SETWATERMARK failed. %d\n", errno);
    }

  return ret;
}

static void dump_sample(void *buffer, int nr_samples)
{
  struct three_axis_s *ta = buffer;
  int i;

  for (i = 0; i < nr_samples; i++, ta++)
    {
      printf("[%3d] xyz = %6d, %6d, %6d\n", i, ta->x, ta->y, ta->z);
    }
  printf("\n");
}

static int sensing_main(FAR struct deci_s *d0, FAR struct deci_s *d1)
{
  sigset_t  set;
  siginfo_t info;
  FAR struct deci_s *d;
  int ret;

  ret = setup_as_normal(d0);
  if (ret < 0)
    {
      return ret;
    }

  ret = setup_as_decimated(d1);
  if (ret < 0)
    {
      return ret;
    }

  sigemptyset(&set);
  sigaddset(&set, d0->signo);
  sigaddset(&set, d1->signo);

  /* Each signals must be blocked while processing a signal. */

  sigprocmask(SIG_BLOCK, &set, NULL);

  ret = ioctl(d0->fd, SCUIOC_START, 0);
  if (ret < 0)
    {
      fprintf(stderr, "Sequencer start failed. %d\n", errno);
      return ret;
    }

  ret = ioctl(d1->fd, SCUIOC_START, 0);
  if (ret < 0)
    {
      fprintf(stderr, "Sequencer start failed. %d\n", errno);
      return ret;
    }

  for (;;)
    {
      ret = sigwaitinfo(&set, &info);
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

      if (ret == d0->signo)
        {
          printf("Decimator 0:\n");
          d = d0;
        }
      else if (ret == d1->signo)
        {
          printf("Decimator 1:\n");
          d = d1;
        }
      else
        {
          fprintf(stderr, "Unknown signal number. %d\n", ret);
          break;
        }

      ret = read(d->fd, g_buffer, sizeof(struct three_axis_s) * 32);
      if (ret >= 0)
        {
          dump_sample(g_buffer, ret / sizeof(struct three_axis_s));
        }
    }

  ioctl(d0->fd, SCUIOC_STOP, 0);
  ioctl(d1->fd, SCUIOC_STOP, 0);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * decimator_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct deci_s *d0, *d1;
  int fd;

  d0 = &g_data[0];
  d1 = &g_data[1];

  fd = open(DEC0_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      fprintf(stderr, "%s open failed.\n", DEC0_DEVPATH, errno);
      return 0;
    }
  d0->fd = fd;
  d0->signo = DEC0_SIGNO;

  fd = open(DEC1_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      fprintf(stderr, "%s open failed.\n", DEC1_DEVPATH, errno);
      close(d0->fd);
      return 0;
    }
  d1->fd = fd;
  d1->signo = DEC1_SIGNO;

  g_buffer = malloc(sizeof(struct three_axis_s) * 32);
  if (g_buffer == NULL)
    {
      fprintf(stderr, "Memory allocation failure.\n");
      return 0;
    }

  sensing_main(d0, d1);

  close(d0->fd);
  close(d1->fd);

  free(g_buffer);

  return 0;
}
