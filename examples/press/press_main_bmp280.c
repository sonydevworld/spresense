/****************************************************************************
 * press/press_main_bmp280.c
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

#include <nuttx/arch.h>
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
#include <signal.h>

#include <nuttx/sensors/bmp280.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_PRESS_DEVNAME
#  define CONFIG_EXAMPLES_PRESS_DEVNAME "/dev/press0"
#endif

#ifndef CONFIG_EXAMPLES_PRESS_NTIMES
#  define CONFIG_EXAMPLES_PRESS_NTIMES 10
#endif

#ifndef CONFIG_EXAMPLES_PRESS_INTERVAL
#  define CONFIG_EXAMPLES_PRESS_INTERVAL 100
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * sensor_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  uint32_t press;
  int fd;
  int ret;
  int i;

  fd = open(CONFIG_EXAMPLES_PRESS_DEVNAME, O_RDONLY);
  if (fd < 0)
    {
      printf("Device %s open failure. %d\n",
             CONFIG_EXAMPLES_PRESS_DEVNAME, fd);
      return -1;
    }

  for (i = 0; i < CONFIG_EXAMPLES_PRESS_NTIMES; i++)
    {
      /* sleep in micro-seconds. */

      usleep(CONFIG_EXAMPLES_PRESS_INTERVAL * 1000);

      ret = read(fd, &press, sizeof(uint32_t));
      if (ret < 0)
        {
          fprintf(stderr, "Reading sensor failed.\n");
          break;
        }

      printf("%d\n", press);
      fflush(stdout);
    }

  close(fd);

  return 0;
}
