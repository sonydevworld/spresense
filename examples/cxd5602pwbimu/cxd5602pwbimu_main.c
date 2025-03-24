/****************************************************************************
 * examples/cxd5602pwbimu/cxd5602pwbimu_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <time.h>
#include <inttypes.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/sensors/cxd5602pwbimu.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD5602PWBIMU_DEVPATH      "/dev/imu0"

#define itemsof(a) (sizeof(a)/sizeof(a[0]))

/****************************************************************************
 * Private values
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int start_sensing(int fd, int rate)
{
  int ret;

  ret = ioctl(fd, SNIOC_SSAMPRATE, rate);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return 1;
    }

  ret = ioctl(fd, SNIOC_ENABLE, 1);
  if (ret)
    {
      printf("ERROR: Enable failed. %d\n", errno);
      return 1;
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
  struct pollfd fds[1];
  struct timespec start, now, delta;
  cxd5602pwbimu_data_t *outbuf = NULL;
  cxd5602pwbimu_data_t *p = NULL;
  cxd5602pwbimu_data_t *last;
  const int samplerate = 1920;

  fd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: Device %s open failure. %d\n", CXD5602PWBIMU_DEVPATH, errno);
      return 1;
    }

  outbuf = (cxd5602pwbimu_data_t *)malloc(sizeof(cxd5602pwbimu_data_t) * samplerate);
  if (outbuf == NULL)
    {
      printf("ERROR: Output buffer allocation failed.\n");
      return 1;
    }
  last = outbuf + samplerate;

  fds[0].fd = fd;
  fds[0].events = POLLIN;

  ret = start_sensing(fd, samplerate);
  if (ret)
    {
      close(fd);
      return ret;
    }

  memset(&now, 0, sizeof(now));

  for (p = outbuf; p < last; p++)
    {
      ret = poll(fds, 1, 1000);
      if (ret < 0)
        {
          if (errno != EINTR)
            {
              printf("ERROR: poll failed. %d\n", errno);
            }
          break;
        }
      if (ret == 0)
        {
          printf("Timeout!\n");
        }
      if (p == outbuf)
        {
          /* To remove first sensing delay, start time measurement from
           * the first captured data.
           */

          clock_gettime(CLOCK_MONOTONIC, &start);
        }

      if (fds[0].revents & POLLIN)
        {
          ret = read(fd, p, sizeof(*p));
          if (ret != sizeof(*p))
            {
              printf("ERROR: read size mismatch! %d\n", ret);
            }
        }

      clock_gettime(CLOCK_MONOTONIC, &now);
      clock_timespec_subtract(&now, &start, &delta);
      if (delta.tv_sec >= 1)
        {
          break;
        }
    }

  /* Save the latest written position */

  last = p;

  close(fd);

  /* Output buffered sensing data */

  for (p = outbuf; p < last; p++)
    {
      printf("%.6f,%.6f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
             p->timestamp / 19200000.0f,
             p->temp,
             p->gx, p->gy, p->gz,
             p->ax, p->ay, p->az);
    }
  free(outbuf);

  clock_timespec_subtract(&now, &start, &delta);
  printf("Elapsed %ld.%09ld seconds\n", delta.tv_sec, delta.tv_nsec);
  printf("%d samples captured\n", last - outbuf);
  printf("Finished.\n");

  return 0;
}
