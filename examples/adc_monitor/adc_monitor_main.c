/****************************************************************************
 * adc_monitor/adc_monitor_main.c
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#include <sdk/config.h>
#include <nuttx/arch.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/boardctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_CXD56_ADC
#include <arch/chip/scu.h>
#include <arch/chip/adc.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_ADC_MONITOR_DEVPATH
#  define CONFIG_EXAMPLES_ADC_MONITOR_DEVPATH "/dev/lpadc0"
#endif

#ifndef CONFIG_EXAMPLES_ADC_MONITOR_READCOUNT
#  define CONFIG_EXAMPLES_ADC_MONITOR_READCOUNT 0
#endif

#ifndef CONFIG_EXAMPLES_ADC_MONITOR_BUFSIZE
#  define CONFIG_EXAMPLES_ADC_MONITOR_BUFSIZE 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct adc_state_s
{
  bool      initialized;
  FAR char *devpath;
  int       count;
  int       bufsize;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct adc_state_s g_adcstate;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_devpath
 ****************************************************************************/

static void adc_devpath(FAR struct adc_state_s *adc,
                        FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (adc->devpath)
    {
      free(adc->devpath);
    }

  /* Then set-up the new device path by copying the string */

  adc->devpath = strdup(devpath);
}

/****************************************************************************
 * Name: adc_help
 ****************************************************************************/

static void adc_help(FAR struct adc_state_s *adc)
{
  printf("Usage: adc_monitor [OPTIONS]\n");
  printf("\nArguments are \"sticky\". For example, once the ADC device is\n");
  printf("specified, that device will be re-used until it is changed.\n");
  printf("\n\"sticky\" OPTIONS include:\n");
  printf("  [-p devpath] selects the ADC device.\n"
         "   /dev/lpadc{0,1,2,3}, /dev/hpadc{0,1} Current: %s\n",
         adc->devpath ? adc->devpath : "NONE");
  printf("  [-n count] set the number of reads."
         "Current: %d\n", adc->count);
  printf("  [-h] shows this message and exits\n");
}

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(FAR struct adc_state_s *adc, int argc,
                       FAR char **argv)
{
  FAR char *ptr;
  FAR char *str;
  long value;
  int index;
  int nargs;

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          printf("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          case 'n':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0)
              {
                printf("Count must be non-negative: %ld\n", value);
                exit(1);
              }

            adc->count = (uint32_t)value;
            index += nargs;
            break;

          case 'p':
            nargs = arg_string(&argv[index], &str);
            adc_devpath(adc, str);
            index += nargs;
            break;

          case 'h':
            adc_help(adc);
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            adc_help(adc);
            exit(1);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_monitor_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  int errval = 0;
  int fd;
  char *buftop = NULL;
  uint32_t loopcnt = 0;
  ssize_t nbytes;

  /* Check if we have initialized */

  if (!g_adcstate.initialized)
    {
      /* Set the default values */

      adc_devpath(&g_adcstate, CONFIG_EXAMPLES_ADC_MONITOR_DEVPATH);
      g_adcstate.count = CONFIG_EXAMPLES_ADC_MONITOR_READCOUNT;
      if (g_adcstate.count == 0)
        {
          g_adcstate.count = 10;
        }
      g_adcstate.bufsize = CONFIG_EXAMPLES_ADC_MONITOR_BUFSIZE;
      if (g_adcstate.bufsize == 0)
        {
          g_adcstate.bufsize = 16;
        }
      g_adcstate.initialized = true;
    }

  /* Parse the command line */

  parse_args(&g_adcstate, argc, argv);

  loopcnt = g_adcstate.count;

  buftop = (char *)malloc(g_adcstate.bufsize);
  if (!buftop)
    {
      printf("malloc failed. size:%d\n", g_adcstate.bufsize);
      errval = 3;
      goto errout;
    }

  printf("ADC monitor example - Name:%s bufsize:%d\n",
         g_adcstate.devpath, g_adcstate.bufsize);

  fd = open(g_adcstate.devpath, O_RDONLY);
  if (fd < 0)
    {
      printf("open %s failed: %d\n", g_adcstate.devpath, errno);
      errval = 4;
      goto errout;
    }

  /* SCU FIFO overwrite */

  ret = ioctl(fd, SCUIOC_SETFIFOMODE, 1);
  if (ret < 0)
    {
      errval = errno;
      printf("ioctl(SETFIFOMODE) failed: %d\n", errval);
      goto errout_with_dev;
    }

  /* Start A/D conversion */

  ret = ioctl(fd, ANIOC_CXD56_START, 0);
  if (ret < 0)
    {
      errval = errno;
      printf("ioctl(START) failed: %d\n", errval);
      goto errout_with_dev;
    }

  for (; loopcnt > 0; loopcnt--)
    {
      fflush(stdout);

      /* wait */

      up_mdelay(1000);

      /* read data */

      nbytes = read(fd, buftop, g_adcstate.bufsize);

      if (nbytes < 0)
        {
          errval = errno;
          printf("read failed:%d\n", errval);
          goto errout_with_dev;
        }
      else if (nbytes == 0)
        {
          printf("read data size = 0\n");
        }
      else
        {
          if (nbytes & 1)
            {
              printf("read size=%ld is not a multiple of sample size=%d\n",
                     (long)nbytes, sizeof(uint16_t));
            }
          else
            {
              int32_t count = 0;
              char *start = buftop;
              char *end = buftop + g_adcstate.bufsize;
              int16_t data = 0, min = 0, max = 0;
              int32_t sum = 0;

              if (g_adcstate.bufsize > nbytes)
                {
                  end = buftop + nbytes;
                }

              while (1)
                {
                  data = (int16_t)(*(uint16_t *)(start));
                  min = ((min == 0) || (data < min)) ? data : min;
                  max = ((max == 0) || (data > max)) ? data : max;
                  sum += (int32_t)data;
                  count++;
                  start += sizeof(uint16_t);
                  if (start >= end)
                    {
                      break;
                    }
                }

              if (count > 0)
                {
                  printf("Ave:%d Min:%d Max:%d Cnt:%d\n", sum / count, min, max, count);
                }
            }
        }
    }

  /* Stop A/D conversion */

  ret = ioctl(fd, ANIOC_CXD56_STOP, 0);
  if (ret < 0)
    {
      int errcode = errno;
      printf("ioctl(STOP) failed: %d\n", errcode);
    }

  close(fd);

  if (buftop)
    {
      free(buftop);
      buftop = NULL;
    }

  printf("ADC monitor example end\n");

  return OK;

  /* Error exits */

errout_with_dev:
  close(fd);

errout:
  printf("ADC monitor example terminating!\n");
  if (buftop)
    {
      free(buftop);
      buftop = NULL;
    }
  fflush(stdout);
  return errval;
}
