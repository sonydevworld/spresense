/****************************************************************************
 * lte_lwm2m/system_adc.c
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
#include <nuttx/arch.h>
#include <arch/cxd56xx/scu.h>
#include <arch/cxd56xx/adc.h>

#include "lwm2mclient.h"
#include "liblwm2m.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_NUMS                  6
#define MY_ADC_SIGKILL            19

#define ADC_RANGE_MIN             0.0f
#define ADC_RANGE_MAX             1.0f

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pid_t adc_pid[ADC_NUMS];

static const char *devfile[] =
{
  "/dev/hpadc0",
  "/dev/hpadc1",
  "/dev/lpadc0",
  "/dev/lpadc1",
  "/dev/lpadc2",
  "/dev/lpadc3",
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static float adc_map(int16_t x, int16_t in_min, int16_t in_max,
                     float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


static int adc_task(int argc, char **argv)
{
  int      fd;
  int      ret;
  int      ch;
  int16_t  raw;
  ssize_t  nbytes;
  sigset_t mask;
  struct timespec timeout;
  float    value;

  /* Program start. */

  ch = atoi(argv[1]);

  printf(">%s(%d)\n", __FUNCTION__, ch);

  /* Get file descriptor to control GNSS. */

  fd = open(devfile[ch], O_RDONLY);
  if (fd < 0)
    {
      printf("open error:%d,%d\n", fd, errno);
      return -ENODEV;
    }

  /* SCU FIFO overwrite */

  ret = ioctl(fd, SCUIOC_SETFIFOMODE, 1);
  if (ret < 0)
    {
      printf("ioctl(SETFIFOMODE) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* Set ADC FIFO size */

  if (ioctl(fd, ANIOC_CXD56_FIFOSIZE, 2) < 0)
    {
      printf("ioctl(FIFOSIZE) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* Start A/D conversion */

  ret = ioctl(fd, ANIOC_CXD56_START, 0);
  if (ret < 0)
    {
      printf("ioctl(START) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* Warm up ADC */

  up_mdelay(1);


  sigemptyset(&mask);
  sigaddset(&mask, MY_ADC_SIGKILL);

  timeout.tv_sec = 1;
  timeout.tv_nsec = 0;

  while (1)
    {
      /* Wait for signal */

      ret = sigtimedwait(&mask, NULL, &timeout);
      if (ret == MY_ADC_SIGKILL)
        {
          printf("sigtimedwait error: %d\n", ret);
          break;
        }

      /* read data */

      do
        {
          nbytes = read(fd, &raw, sizeof(int16_t));
          if (nbytes < 0)
            {
              printf("ERROR: Failed to read ADC: %d\n", nbytes);
              goto errout_with_start;
            }
        }
      while (nbytes == 0);

      /* ADC value */

      value = adc_map(raw, INT16_MIN, INT16_MAX, ADC_RANGE_MIN, ADC_RANGE_MAX);
      analog_input_setValue((uint16_t)ch, value, ADC_RANGE_MIN, ADC_RANGE_MAX);
      //printf("v:%d (%.3f)\n", raw, value);
    }

errout_with_start:

  /* Stop A/D conversion */

  ret = ioctl(fd, ANIOC_CXD56_STOP, 0);
  if (ret < 0)
    {
      printf("ioctl(STOP) failed: %d\n", errno);
    }

errout_with_dev:

  /* Close ADC file descriptor */

  ret = close(fd);
  if (ret < 0)
    {
      printf("close error: %d\n", errno);
    }

  printf("<%s(%d)\n", __FUNCTION__, ch);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int adc_start(int ch)
{
  const char *argv[2];
  char str[8];

  if ((ch < 0) || (ADC_NUMS <= ch))
    {
      return -EINVAL;
    }

  snprintf(str, 8, "%d", ch);

  argv[0] = str;
  argv[1] = NULL;

  adc_pid[ch] = task_create("adc", 100, CONFIG_DEFAULT_TASK_STACKSIZE,
                            adc_task, (char * const *)argv);
  return 0;
}

int adc_stop(int ch)
{
  int ret;

  ret = kill(adc_pid[ch], MY_ADC_SIGKILL);

  /* wait until adc task is terminated */

  sleep(2);

  return ret;
}
