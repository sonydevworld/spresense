/****************************************************************************
 * examples/ahrs_pwbimu/ahrs_pwbimu_main.c
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
#include <float.h>
#include <math.h>

#include <nuttx/sensors/cxd5602pwbimu.h>
#include <MadgwickAHRS.h>

#include "net_client.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD5602PWBIMU_DEVPATH   "/dev/imu0"

#define IMU_COUNTER_FREQ        (19200000.f)
#define DEFAULT_SAMPLERATE      (1920)
#define DEFAULT_GYROSCOPERANGE  (1000)
#define DEFAULT_ACCELRANGE      (4)

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

#define OUTPUT_MODE_EULER      (0)
#define OUTPUT_MODE_QUATERNION (1)
#define OUTPUT_MODE_NET        (2)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t s_prev_time;
static uint64_t s_prev_unroll_time;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/** start_sensing()
 * @rate    [in]: 15, 30, 60, 120, 240, 480, 960, 1920
 * @adrange [in]: 2, 4, 8, 16
 * @gdrange [in]: 125, 250, 500, 1000, 2000, 4000
 * @nfifos  [in]: 1, 2, 3, 4
 */

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

static int read_imudata(int fd, cxd5602pwbimu_data_t *imudata)
{
  int ret = 0;

#ifndef CONFIG_SYSTEM_STARTUP_SCRIPT
  char c;
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
            }
        }
    }
#else /* #ifdef CONFIG_SYSTEM_STARTUP_SCRIPT */
  /* This code is a workaround for an issue where IMU data cannot be obtained
   * when poll() is executed with stdin while the program task is running in the background.
   * When the startup_script is enabled, the situation will happen more often.
   * Incidentally, it is planed to fix the above issues at a later date.
   */

  ret = read(fd, imudata, sizeof(*imudata));
  return (ret == sizeof(*imudata)) ? 1 : 0;
#endif /* #ifdef CONFIG_SYSTEM_STARTUP_SCRIPT */

  return ret;
}

static int drop_50msdata(int fd, int samprate, cxd5602pwbimu_data_t *imu)
{
  int cnt = samprate / 20; /* data size of 50ms */

  if (cnt == 0) cnt = 1;

  while (cnt)
    {
      read_imudata(fd, imu);
      cnt--;
    }

  return 0;
}

#ifdef CONFIG_EXAMPLES_AHRS_PWBIMU_EXEC_GYROBIAS_ESTIMATION 
static void update_maxmin(float val, float* min_val, float* max_val)
{
  *min_val = val < *min_val ? val : *min_val;
  *max_val = val > *max_val ? val : *max_val;
}

static int gyrobias_estimation(int fd, int samprate, int estimation_time,
                               float* bias_out, cxd5602pwbimu_data_t *imu)
{
  float min_gx = FLT_MAX, max_gx = -FLT_MAX;
  float min_gy = FLT_MAX, max_gy = -FLT_MAX;
  float min_gz = FLT_MAX, max_gz = -FLT_MAX;
  double gyrobias[3];
  int i = 0;

  estimation_time *= samprate; /* data num of estimation time */
  
  printf("Start gyro-bias estimation.\n");

  for (i = 0; i < estimation_time; i++)
    {
      read_imudata(fd, imu);

      gyrobias[0] += imu->gx;
      gyrobias[1] += imu->gy;
      gyrobias[2] += imu->gz;

      update_maxmin(imu->gx, &min_gx, &max_gx);
      update_maxmin(imu->gy, &min_gy, &max_gy);
      update_maxmin(imu->gz, &min_gz, &max_gz);

      if ((i % samprate) == (samprate - 1))
        {
          printf("Elapsed time : %03d seconds.\n", i / samprate + 1);
        }
    }

  printf("Finish gyro-bias estimation.\n");

  for(i = 0; i < 3; i++)
    {
      gyrobias[i] /= (double)estimation_time;
    }

  printf("gx: ave=%12.3e, min=%12.3e, max=%12.3e\n", gyrobias[0],
                                                     min_gx, max_gx);
  printf("gy: ave=%12.3e, min=%12.3e, max=%12.3e\n", gyrobias[1],
                                                     min_gy, max_gy);
  printf("gz: ave=%12.3e, min=%12.3e, max=%12.3e\n", gyrobias[2],
                                                     min_gz, max_gz);

  return 0;
}
#endif

static float convert_timesec(uint32_t curr_time)
{
  s_prev_unroll_time += (uint64_t)(curr_time - s_prev_time);
  s_prev_time = curr_time;
  return (float)s_prev_unroll_time / IMU_COUNTER_FREQ;
}

static void set_initial_posture(int fd, struct ahrs_out_s *inst,
                                cxd5602pwbimu_data_t *imu)
{
  read_imudata(fd, imu);
  setPostureByAccel(inst, imu->ax, imu->ay, imu->az, 0.f);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  struct ahrs_out_s ahrs;
  cxd5602pwbimu_data_t imu;
  float e[3];
  double gyrobias[3] = {0.0};
  unsigned int cnt = 0;
  int outmode = OUTPUT_MODE_EULER;
  int sock = -1;
  uint32_t last_ts;
  float    diff_ts;

  /* Parse arguments */

  if (argc == 2 && argv[1][0] == 'h')
    {
      outmode = OUTPUT_MODE_QUATERNION;
    }
  else if (argc == 3 && !strncmp(argv[1], "net", 4))
    {
      outmode = OUTPUT_MODE_NET;
      sock = ahrs_connect_server(argv[2]);
      if (sock < 0)
        {
          printf("Could not connect Receiver %s: errno:%d\n", argv[2], errno);
          return -1;
        }
      printf("Connected to server\n");
    }

  /* Initialize time stamp */

  s_prev_time = 0;
  s_prev_unroll_time = 0;

  /* Initialize AHRS instance */

  INIT_AHRS(&ahrs, 0.5f, (float)DEFAULT_SAMPLERATE);

  /* Start IMU Driver */

  printf("Starting Driver\n");
  fd = start_sensing(DEFAULT_SAMPLERATE,
                     DEFAULT_ACCELRANGE,
                     DEFAULT_GYROSCOPERANGE, 1);
  printf("Drop 50ms data for stability\n");
  drop_50msdata(fd, DEFAULT_SAMPLERATE, &imu);

  /* Note: Add logic to remove gyro bias, othewise DC offset will be
   *       on the gyro sensor data.
   *       A commonly used method to remove the DC offset is, for example,
   *       to take the average of the gyro sensor data while it is
   *       stationary for several seconds, use that value as the BIAS
   *       value, and subtract that value from each gyro sample data.
   */

#ifdef CONFIG_EXAMPLES_AHRS_PWBIMU_EXEC_GYROBIAS_ESTIMATION 

  /* This is an example of gyro bias estimation.
   * This function estimate the gyro bias and the angular velocity
   * caused by the Earth's rotation together as offset values.
   * Please maintain a stationary state as much as possible
   * during the estimation.
   */

  gyrobias_estimation(fd, DEFAULT_SAMPLERATE, 
                      CONFIG_EXAMPLES_AHRS_PWBIMU_GYROBIAS_ESTIMATION_TIME,
                      gyrobias, &imu);
#endif
  last_ts = imu.timestamp;

  set_initial_posture(fd, &ahrs, &imu);

  printf("Start loop\n");
  while (read_imudata(fd, &imu))
    {
      diff_ts = ((float)(imu.timestamp - last_ts) / IMU_COUNTER_FREQ);
      last_ts = imu.timestamp;

      MadgwickAHRSupdateIMU(&ahrs,
                            imu.gx - (float)gyrobias[0],
                            imu.gy - (float)gyrobias[1],
                            imu.gz - (float)gyrobias[2],
                            imu.ax, imu.ay, imu.az, diff_ts);

      quaternion2euler(ahrs.q, e);

      /* Output Data */

      if (++cnt >= 20)  /* Decimate data 1920Hz / 20 = 96Hz */
        {
          switch (outmode)
            {
              case OUTPUT_MODE_EULER:
                printf("T:%0.2f, R:%0.2f, P:%0.2f, Y:%0.2f\n",
                       convert_timesec(imu.timestamp),
                       RAD2DEG(e[0]), RAD2DEG(e[1]), RAD2DEG(e[2]));
                break;

              case OUTPUT_MODE_QUATERNION:
                printf("%08x,%08x,%08x,%08x\n",
                       *(unsigned int *)&ahrs.q[0],
                       *(unsigned int *)&ahrs.q[1],
                       *(unsigned int *)&ahrs.q[2],
                       *(unsigned int *)&ahrs.q[3]);
                break;

              case OUTPUT_MODE_NET:
                if (ahrs_send_binary(sock, (const char *)&ahrs.q[0],
                                           sizeof(ahrs.q)) < 0)
                  {
                    printf("Send error.\n");
                    close(sock);
                    close(fd);
                    return -1;
                  }

                break;
            }

          cnt = 0;
        }
    }

  if (sock >= 0)
    {
      close(sock);
    }

  close(fd);

  return 0;
}
