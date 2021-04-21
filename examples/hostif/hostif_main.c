/****************************************************************************
 * hostif/hostif_main.c
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
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <debug.h>
#include <arch/chip/hostif.h>

#include <pthread.h>
#include <sys/utsname.h>
#include <time.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_HOSTIF_I2C
static struct hostif_i2cconf_s conf =
#else
static struct hostif_spiconf_s conf =
#endif
{
#ifdef CONFIG_EXAMPLES_HOSTIF_I2C
  .address = 0x24, /* own slave address */
#endif
  .buff[0] =
    {
      /* BUFFER0: receive buffer from host */

      0x100,
      HOSTIF_BUFF_ATTR_READ  | HOSTIF_BUFF_ATTR_VARLEN
    },

  .buff[1] =
    {
      /* BUFFER1: send buffer to host */

      0x100,
      HOSTIF_BUFF_ATTR_WRITE | HOSTIF_BUFF_ATTR_VARLEN
    },

  .buff[2] =
    {
      /* BUFFER2: send the constant version information */

      VERSION_NAMELEN,
      HOSTIF_BUFF_ATTR_WRITE | HOSTIF_BUFF_ATTR_FIXLEN
    },

  .buff[3] =
    {
      /* BUFFER3: send the variable timestamp information */

      sizeof(struct timespec),
      HOSTIF_BUFF_ATTR_WRITE | HOSTIF_BUFF_ATTR_FIXLEN
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hostif_loopback
 ****************************************************************************/

static void *hostif_loopback(void *arg)
{
  size_t  size;
  int     wfd;
  int     rfd;
  uint8_t buffer[0x100];
  int     i;

  printf("Start loopback: loopback the received data\n");

  /* Open read device of BUFFER0 */

  rfd = open("/dev/hostifr0", O_RDONLY);

  if (rfd < 0)
    {
      printf("ERROR: failed to open /dev/hostifr0: %d\n", rfd);
      return NULL;
    }

  /* Open write device of BUFFER1 */

  wfd = open("/dev/hostifw1", O_WRONLY);

  if (wfd < 0)
    {
      printf("ERROR: failed to open /dev/hostifw1: %d\n", wfd);
      close(rfd);
      return NULL;
    }

  while (1)
    {
      /* blocking read */

      size = read(rfd, buffer, sizeof(buffer));

      if (size < 0)
        {
          printf("ERROR: failed to read: %d\n", size);
        }

      /* dump read data */

      for (i = 0; i < size; i++)
        {
          printf(" %02x", buffer[i]);
        }

      printf("\n");

      /* blocking write */

      size = write(wfd, buffer, size);

      if (size < 0)
        {
          printf("ERROR: failed to write: %d\n", size);
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: hostif_updater
 ****************************************************************************/

static void *hostif_updater(void *arg)
{
  int     ret;
  int     wfd2;
  int     wfd3;

  printf("Start updater: update the information periodically\n");

  /* Open write device of BUFFER2 */

  wfd2 = open("/dev/hostifw2", O_WRONLY | O_NONBLOCK);

  if (wfd2 < 0)
    {
      printf("ERROR: failed to open /dev/hostifw2: %d\n", wfd2);
      return NULL;
    }

  /* Open write device of BUFFER3 */

  wfd3 = open("/dev/hostifw3", O_WRONLY | O_NONBLOCK);

  if (wfd3 < 0)
    {
      printf("ERROR: failed to open /dev/hostifw3: %d\n", wfd3);
      close(wfd2);
      return NULL;
    }

  /* Write-once the constant software version to BUFFER2 */

  struct utsname name;

  uname(&name);

  ret = write(wfd2, &name.version, VERSION_NAMELEN);

  if (ret < 0)
    {
      printf("ERROR: failed to write /dev/hostifw2: %d\n", ret);
      close(wfd3);
      close(wfd2);
      return NULL;
    }

  printf("version: %s\n", name.version);

  while (1)
    {
      /* Write the current timestamp to BUFFER3 */

      struct timespec ts;

      clock_gettime(CLOCK_REALTIME, &ts);

      ret = write(wfd3, &ts, sizeof(ts));

      if (ret < 0)
        {
          printf("ERROR: failed to write /dev/hostifw3: %d\n", ret);
        }

      /* Run periodically */

      sleep(1);
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hostif_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int                 ret;
  pthread_t           loopback;
  pthread_t           updater;
  pthread_attr_t      attr;
  pthread_addr_t      result;
  struct sched_param  param;

  /* Initialize the hostif driver */

#ifdef CONFIG_EXAMPLES_HOSTIF_I2C
  ret = hostif_i2cinitialize(&conf);
#else
  ret = hostif_spiinitialize(&conf);
#endif
  if (ret != OK)
    {
      printf("ERROR: failed to initialize hostif: %d\n", ret);
      return ERROR;
    }

  /* Start the updater thread which runs periodically */

  /* Set the higher priority than the loopback thread */

  pthread_attr_init(&attr);
  pthread_attr_getschedparam(&attr, &param);
  param.sched_priority++;
  pthread_attr_setschedparam(&attr, &param);

  pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_HOSTIF_STACKSIZE);

  ret = pthread_create(&updater, &attr, hostif_updater, NULL);
  if (ret != OK)
    {
      printf("ERROR: failed to start updater thread: %d\n", ret);
      return ERROR;
    }

  pthread_setname_np(updater, "updater");

  /* Start the loopback thread which runs event driven */

  ret = pthread_create(&loopback, NULL, hostif_loopback, NULL);
  if (ret != OK)
    {
      printf("ERROR: failed to start loopback thread: %d\n", ret);
      return ERROR;
    }

  pthread_setname_np(loopback, "loopback");

  /* Waiting for each thread */

  pthread_join(loopback, &result);
  pthread_join(updater, &result);

  /* Finalize the hostif driver */

  hostif_uninitialize();

  return 0;
}
