/****************************************************************************
 * examples/cxd5602pwbimu/cxd5602pwbimu_main.c
 *
 *   Copyright 2025 Sony Semiconductor Solutions Corporation
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

static int start_sensing(int fd, int rate, int adrange, int gdrange,
                         int nfifos)
{
  cxd5602pwbimu_range_t range;
  int ret;

  /*
   * Set sampling rate. Available values (Hz) are below.
   *
   * 15 (default), 30, 60, 120, 240, 480, 960, 1920
   */

  ret = ioctl(fd, SNIOC_SSAMPRATE, rate);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return 1;
    }

  /*
   * Set dynamic ranges for accelerometer and gyroscope.
   * Available values are below.
   *
   * accel: 2 (default), 4, 8, 16
   * gyro: 125 (default), 250, 500, 1000, 2000, 4000
   */

  range.accel = adrange;
  range.gyro = gdrange;
  ret = ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  if (ret)
    {
      printf("ERROR: Set dynamic range failed. %d\n", errno);
      return 1;
    }

  /*
   * Set hardware FIFO threshold.
   * Increasing this value will reduce the frequency with which data is
   * received.
   */

  ret = ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return 1;
    }

  /*
   * Start sensing, user can not change the all of configurations.
   */

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

  /* Sensing parameters, see start sensing function. */

  const int samplerate = 1920;
  const int adrange = 2;
  const int gdrange = 125;
  const int nfifos = 1;

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

  ret = start_sensing(fd, samplerate, adrange, gdrange, nfifos);
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

  clock_timespec_subtract(&now, &start, &delta);
  printf("Elapsed %ld.%09ld seconds\n", delta.tv_sec, delta.tv_nsec);
  printf("%d samples captured\n", last - outbuf);
  printf("Finished.\n");

  free(outbuf);

  return 0;
}
