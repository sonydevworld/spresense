/****************************************************************************
 * examples/gyrocompass_pwbimu/imu_utils.c
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

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>

#include <nuttx/sensors/cxd5602pwbimu.h>

#include "imu_utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD5602PWBIMU_DEVPATH   "/dev/imu0"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/** start_imusensing()
 * @rate    [in]: 15, 30, 60, 120, 240, 480, 960, 1920
 * @adrange [in]: 2, 4, 8, 16
 * @gdrange [in]: 125, 250, 500, 1000, 2000, 4000
 * @nfifos  [in]: 1, 2, 3, 4
 */

int pwbimu_start_sensing(int rate, int adrange, int gdrange, int nfifos)
{
  cxd5602pwbimu_range_t range;
  int ret;
  int fd;

  fd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: Device %s open failure. %d\n",
             CXD5602PWBIMU_DEVPATH, errno);
      return -errno;
    }


  ret = ioctl(fd, SNIOC_SSAMPRATE, rate);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return -errno;
    }

  range.accel = adrange;
  range.gyro = gdrange;
  ret = ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  if (ret)
    {
      printf("ERROR: Set dynamic range failed. %d\n", errno);
      return -errno;
    }

  ret = ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return -errno;
    }

  ret = ioctl(fd, SNIOC_ENABLE, 1);
  if (ret)
    {
      printf("ERROR: Enable failed. %d\n", errno);
      return -errno;
    }

  return fd;
}

int pwbimu_read_imudata(int fd, cxd5602pwbimu_data_t *imudata)
{
  char c;
  int ret;
  struct pollfd fds[2];
  int keep_trying = 1;

  fds[0].fd     = fd;
  fds[0].events = POLLIN;
  fds[1].fd     = fileno(stdin);
  fds[1].events = POLLIN;

  while (keep_trying)
    {
      ret = poll(fds, 2, -1);
      if (ret <= 0)
        {
          printf("Poll error\n");
          keep_trying = 0;
          ret = 0;
        }
      else
        {
          if (fds[0].revents & POLLIN)
            {
              ret = read(fd, imudata, sizeof(*imudata));
              if (ret == sizeof(*imudata))
                {
                  keep_trying = 0;
                  ret = 1;
                }
            }

          if (fds[1].revents & POLLIN)
            {
              read(fds[1].fd, &c, 1);
              if (c == 'q')
                {
                  keep_trying = 0;
                  ret = 0;
                }
            }
        }
    }

  return ret;
}

int pwbimu_drop_data50ms(int fd, int samprate, cxd5602pwbimu_data_t *imu)
{
  int cnt = samprate / 20; /* data size of 50ms */

  if (cnt == 0) cnt = 1;

  while (cnt)
    {
      pwbimu_read_imudata(fd, imu);
      cnt--;
    }

  return 0;
}

int pwbimu_terminate(int fd)
{
  int ret;

  ret = ioctl(fd, SNIOC_ENABLE, 0);
  if (ret)
    {
      printf("ERROR: Enable failed. %d\n", errno);
      return -errno;
    }

  close(fd);

  return 0;
}
