/****************************************************************************
 * tilt/tilt_main.c
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

#ifdef CONFIG_CXD56_SCU
#include <arch/chip/scu.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef ABS
#define ABS(a) ((a) > 0 ? (a) : -(a))
#endif

#define ACC_DEVPATH      "/dev/accel0"

#define EVENT_SIGNAL 17
#define WM_SIGNAL    18

struct sample_s
{
  int16_t x;
  int16_t y;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_data;
static struct scuev_arg_s g_evarg;

/*
 * Equal multiplication setting for each IIR filters.
 */

static const struct math_filter_s g_filter =
{
  .pos = FILTER_POS_AA,
  .filter[0] = {
    .ishift = 0,
    .oshift = 0,
    .coeff[0].h = 0x20000000,
    .coeff[0].l = 0,
    .coeff[1].h = 0,
    .coeff[1].l = 0,
    .coeff[2].h = 0,
    .coeff[2].l = 0,
    .coeff[3].h = 0,
    .coeff[3].l = 0,
    .coeff[4].h = 0,
    .coeff[4].l = 0,
  },
  .filter[1] = {
    .ishift = 0,
    .oshift = 0,
    .coeff[0].h = 0x20000000,
    .coeff[0].l = 0,
    .coeff[1].h = 0,
    .coeff[1].l = 0,
    .coeff[2].h = 0,
    .coeff[2].l = 0,
    .coeff[3].h = 0,
    .coeff[3].l = 0,
    .coeff[4].h = 0,
    .coeff[4].l = 0,
  },
};

/*
 * SCU event notifier control for tilt detection
 *
 * Both of rise event and fall event are enabled, always output sensing data
 * to SCU FIFO.
 *
 * SCU event detection function takes NORM processed data as follows:
 *
 * For 1 axis (X axis):
 *  norm = abs(x)
 *
 * For 2 axis (X and Y axis):
 *  norm = max(abs(x), abs(y)) * 123 / 128 + min(abs(x), abs(y)) * 51 / 128
 *
 * For 3 axis (X, Y, Z):
 *  L1   = max(abs(x), abs(y)) * 120 / 128 + min(abs(x), abs(y)) * 49 / 128
 *  norm = max(L1, abs(z)) + min(L1, abs(z)) * 44 / 128
 *
 * In this example, BMI160 output NORMed value about 300 - 16000, so set
 * 10000 to threshold to detect tilting.
 *
 * rise and fall members are "Low to High" and "High to Low" settings,
 * respectively. count0 and count1 are number of counts to detect when value
 * is high/low against threshold, count0 is pre-counting, count1 is actual
 * counting, therefore, event raised when input count0 + count1 values
 * continuously. rise and fall setting should be set if you use only rise
 * event because hardware reset its counts when reached to fall.threshold,
 * also vise-versa.
 *
 * delaysamples member is set delay samples for raising event signal.
 */

static const struct scuev_notify_s g_notify =
{
  .signo = EVENT_SIGNAL,

  /* If you want to see about actual data, please change SCU_EV_NOTOUT to
   * SCU_EV_OUTALWAYS */

  .ctrl = SCU_EV_RISE_EN | SCU_EV_NOTOUT,
  .rise = {
    .threshold = 10000,
    .count0 = 5,
    .count1 = 10,
    .delaysamples = 0,
  },
  .fall = {
    .threshold = 10000,
    .count0 = 5,
    .count1 = 10,
    .delaysamples = 0,
  },
  .arg = &g_evarg,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void dump_data(int fd)
{
  struct sample_s *sample;
  int              i;
  int              size;

  size = read(fd, g_data, sizeof(struct sample_s) * 128);
  if (size < 0)
    {
      fprintf(stderr, "read() failed. %d\n", errno);
      return;
    }

  for (i = 0, sample = (struct sample_s *)g_data;
       i < size/sizeof(struct sample_s); i++, sample++)
    {
      uint16_t norm;

      norm = MAX(ABS(sample->x), ABS(sample->y)) * 123 / 128;
      norm += MIN(ABS(sample->x), ABS(sample->y)) * 51 / 128;

      printf("[%3d] xy = (%6d, %6d), NORM = %d\n", i, sample->x, sample->y, norm);
    }
}

static int set_event_detection(int fd)
{
  int ret;

  /*
   * Set IIR filter
   *
   * This operation is mandatory because event notifier takes filtered sampling
   * data. If you don't need to IIR filter, then set filter to be equal
   * multiplication (x1).
   */

  ret = ioctl(fd, SCUIOC_SETFILTER, (unsigned long)(uintptr_t)&g_filter);
  if (ret < 0)
    {
      fprintf(stderr, "Set filter failed: %d\n", errno);
      return ret;
    }

  /* Use X and Y from sampling data */

  ioctl(fd, SCUIOC_SETELEMENTS, 2);

  /*
   * Set event notifier configuration
   *
   * See g_notify declaration for configuration details.
   */

  ret = ioctl(fd, SCUIOC_SETNOTIFY, (unsigned long)(uintptr_t)&g_notify);
  if (ret < 0)
    {
      fprintf(stderr, "Set notify failed: %d\n", errno);
      return ret;
    }

  return OK;
}

static int set_watermark(int fd)
{
  struct scufifo_wm_s wm;
  int                 ret;

  /* Set watermark at 128 samples */

  wm.signo = WM_SIGNAL;
  wm.ts = NULL;
  wm.watermark = 128;

  ret = ioctl(fd, SCUIOC_SETWATERMARK, (unsigned long)(uintptr_t)&wm);
  if (ret < 0)
    {
      fprintf(stderr, "SETWATERMARK failed. %d\n", errno);
      return ret;
    }

  return OK;
}

static int sensing_main(int fd)
{
  sigset_t  set;
  siginfo_t info;
  int ret;

  /* Set FIFO size to 6 bytes * 128 Hz = 768 */

  ret = ioctl(fd, SCUIOC_SETFIFO, sizeof(struct sample_s) * 128);
  if (ret < 0)
    {
      fprintf(stderr, "SETFIFO failed. %d\n", errno);
      return ret;
    }

  /* Set sequencer sampling rate 128 Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 2) = 128
   */

  ret = ioctl(fd, SCUIOC_SETSAMPLE, 2);
  if (ret < 0)
    {
      fprintf(stderr, "SETSAMPLE failed. %d\n", errno);
      return ret;
    }

  set_event_detection(fd);
  set_watermark(fd);

  sigemptyset(&set);
  sigaddset(&set, EVENT_SIGNAL);
  sigaddset(&set, WM_SIGNAL);
  sigprocmask(SIG_BLOCK, &set, NULL);

  /* Start sequencer */

  ret = ioctl(fd, SCUIOC_START, 0);
  if (ret != 0)
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
      else if (ret == EVENT_SIGNAL)
        {
          struct scuev_arg_s *arg = (struct scuev_arg_s *)info.si_value.sival_ptr;

          printf("Tilt detected! (%s) %u\n",
                 arg->type == SCU_EV_RISE ? "Rise" : "Fall",
                 arg->ts.sec);
        }
      else if (ret == WM_SIGNAL)
        {
          dump_data(fd);
        }
    }

  /* Stop sequencer */

  (void) ioctl(fd, SCUIOC_STOP, 0);

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

  printf("Sensing start...\n");

  g_data = (char *)malloc(1536);
  if (!g_data)
    {
      printf("Memory allocation failure.\n");
      return -1;
    }
  memset(g_data, 0, 1536);

  fd = open(ACC_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("Device %s open failure. %d\n", ACC_DEVPATH, fd);
      return -1;
    }

  sensing_main(fd);

  close(fd);

  free(g_data);

  return 0;
}
