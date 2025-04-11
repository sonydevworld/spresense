/****************************************************************************
 * examples/fft_pwbimu/fft_pwbimu_main.cxx
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
#include <inttypes.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/sensors/cxd5602pwbimu.h>
#include <audiolite/audiolite.h>  // For signal processing with worker

#include "alusr_imufft.h"
#include "imu_hydrant.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD5602PWBIMU_DEVPATH   "/dev/imu0"

#define DEFAULT_SAMPLERATE      (1920)
#define DEFAULT_GYROSCOPERANGE  (1000)
#define DEFAULT_ACCELRANGE      (4)

#define DEFAULT_TAPS  (4096)
#define TARGET_AXIS (IMUHYDRANT_GX | IMUHYDRANT_GY | IMUHYDRANT_GZ | \
                     IMUHYDRANT_AX | IMUHYDRANT_AY | IMUHYDRANT_AZ)

#define itemsof(t)  (sizeof(t)/sizeof(t[0]))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile int disp_ch = 0;
static const int g_rate_tbl[] =
{
  1920, 960, 480, 240, 120, 60, 30, 15
};

static const int g_arange_tbl[] =
{
  16, 8, 4, 2
};

static const int g_grange_tbl[] =
{
  125, 250, 500, 1000, 2000, 4000
};

static const int g_tap_tbl[] =
{
  4096, 2048, 1024, 512, 256, 128, 64, 32
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int check_value(int val, const int *tbl, int sz)
{
  int i;
  for (i = 0; i < sz; i++)
    {
      if (val == tbl[i]) return 0;
    }

  return -1;
}

static int parse_appargs(int argc, char **argv, int *rate,
                         int *a, int *g, int *t)
{
  int opt;

  *rate = DEFAULT_SAMPLERATE;
  *a    = DEFAULT_ACCELRANGE;
  *g    = DEFAULT_GYROSCOPERANGE;
  *t    = DEFAULT_TAPS;

  while ((opt = getopt(argc, argv, "hr:a:g:t:")) != ERROR)
    {
      switch (opt)
        {
          case 'r':
            *rate = atoi(optarg);
            if (check_value(*rate, g_rate_tbl, itemsof(g_rate_tbl)))
              {
                printf("Sample rate is out of range\n");
                return -1;
              }
            break;
          case 'a':
            *a = atoi(optarg);
            if (check_value(*a, g_arange_tbl, itemsof(g_arange_tbl)))
              {
                printf("Accel range is out of range\n");
                return -1;
              }
            break;
          case 'g':
            *g = atoi(optarg);
            if (check_value(*g, g_grange_tbl, itemsof(g_grange_tbl)))
              {
                printf("Gyro range is out of range\n");
                return -1;
              }
            break;
          case 't':
            *t = atoi(optarg);
            if (check_value(*t, g_tap_tbl, itemsof(g_tap_tbl)))
              {
                printf("FFT Taps is out of range\n");
                return -1;
              }
            break;
          case 'h':
          default:
            {
              printf("Usage: nsh> fft_pwbimu (-r <sample rate>) (-a <acc range>) "
                     "(-g <gyro range>) (-t <Tap num>)\n");
              printf("       -r <sample rate>\n"
                     "          1920, 960, 480, 240, 120, 60, 30, 15\n"
                     "       -a <acc range>\n"
                     "          16, 8, 4, 2\n"
                     "       -g <gyro range>\n"
                     "          125, 250, 500, 1000, 2000, 4000\n"
                     "       -t <FFT taps>\n"
                     "          4096, 2048, 1024, 512, 256, 128, 64, 32\n");
              return -1;
            }
        }
    }

  return 0;
}

static int start_sensing(int rate, int adrange, int gdrange, int nfifos)
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

static int drop_50msdata(int fd, int samprate, cxd5602pwbimu_data_t *imu)
{
  int cnt = samprate / 20; /* data size of 50ms */

  if (cnt == 0) cnt = 1;

  while (cnt)
    {
      read(fd, imu, sizeof(imu[0]));
      cnt--;
    }

  return 0;
}

static int read_imudata(int fd, cxd5602pwbimu_data_t *imudata, int chs)
{
  char c;
  int ret;
  struct pollfd fds[2];
  int keep_trying = 1;

  fds[0].fd     = fileno(stdin);
  fds[0].events = POLLIN;
  fds[1].fd     = fd;
  fds[1].events = POLLIN;

  while (keep_trying)
    {
      ret = poll(fds, 2, 1000);
      if (ret <= 0)
        {
          printf("Poll error\n");
          keep_trying = 0;
          ret = 0;
        }
      else
        {
          if (fds[1].revents & POLLIN)
            {
              ret = read(fd, imudata, sizeof(*imudata));
              if (ret == sizeof(*imudata))
                {
                  keep_trying = 0;
                  ret = 1;
                }
            }

          if (fds[0].revents & POLLIN)
            {
              read(fds[0].fd, &c, 1);
              if (c == 'q')
                {
                  keep_trying = 0;
                  ret = 0;
                }
              else if (c >= '1' && c <= '6')
                {
                  c = c - '1';
                  if (c >= chs)
                    {
                      c = chs - 1;
                    }

                  disp_ch = c;
                }
            }
        }
    }

  return ret;
}

static void fftresult_callback(float *topdata, int sz, int taps, int chs)
{
  int i;
  float *data = &topdata[taps * disp_ch];

  printf("Top 10 result data : ");

  for (i = 0; i < 10; i++)
    {
      printf("%+10.5f, ", data[i]);
    }

  printf("\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  int fd;
  cxd5602pwbimu_data_t imu;
  imu_hydrant *hydrant = NULL;
  alusr_imufft *imufft = NULL;
  int srate, arange, grange, taps;

  if (parse_appargs(argc, argv, &srate, &arange, &grange, &taps))
    {
      return -1;
    }

  printf("%s is starting with : "
         "SampleRate(%d) AccRange(%d) GyrRange(%d) FFTTap(%d)\n",
         argv[0], srate, arange, grange, taps);

  fd = start_sensing(srate, arange, grange, 1);
  if (fd < 0)
    {
      printf("IMU Sendor initialization error\n");
      return -1;
    }

  drop_50msdata(fd, srate, &imu);

  hydrant = new imu_hydrant(TARGET_AXIS, taps);
  imufft  = new alusr_imufft(hydrant->taps(), hydrant->chs());

  imufft->set_resultcb(fftresult_callback);
  hydrant->bind(imufft);
  hydrant->start();

  while (read_imudata(fd, &imu, hydrant->chs()))
    {
      hydrant->inject(&imu);
    }

  hydrant->stop();
  hydrant->unbindall();
  audiolite_eventdestroy(); // Delete signal processing system

  delete imufft;
  delete hydrant;

  close(fd);

  return 0;
}
