/****************************************************************************
 * examples/ahrs/ahrs_pwbimu.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD5602PWBIMU_DEVPATH   "/dev/imu0"

#define DEFAULT_SAMPLERATE      (1920)
#define DEFAULT_GYROSCOPERANGE  (1000)
#define DEFAULT_ACCELRANGE      (4)

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t s_prev_time;
static uint64_t s_prev_unroll_time;

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
            }
        }
    }

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
static void max_min_check(float val, float* min_val, float* max_val)
{
  *min_val = val < *min_val ? val : *min_val;
  *max_val = val > *max_val ? val : *max_val;
}

static int gyrobias_estimation(int fd, int samprate, int estimation_time,
                               double* gyrobias, cxd5602pwbimu_data_t *imu)
{
  int cnt = estimation_time * samprate; /* data size of estimation time */
  int cnt_dwn = cnt;
  int elp_cnt = 0;
  float min_gx = FLT_MAX, max_gx = -FLT_MAX;
  float min_gy = FLT_MAX, max_gy = -FLT_MAX;
  float min_gz = FLT_MAX, max_gz = -FLT_MAX;
  float min_ax = FLT_MAX, max_ax = -FLT_MAX;
  float min_ay = FLT_MAX, max_ay = -FLT_MAX;
  float min_az = FLT_MAX, max_az = -FLT_MAX;
  double accelave[3] = {0.0};
  int i = 0;
  
  printf("Start gyro-bias estimation.\n");

  while (cnt_dwn)
    {
      read_imudata(fd, imu);

      gyrobias[0] += imu->gx;
      gyrobias[1] += imu->gy;
      gyrobias[2] += imu->gz;

      accelave[0] += imu->ax;
      accelave[1] += imu->ay;
      accelave[2] += imu->az;

      max_min_check(imu->gx, &min_gx, &max_gx);
      max_min_check(imu->gy, &min_gy, &max_gy);
      max_min_check(imu->gz, &min_gz, &max_gz);

      max_min_check(imu->ax, &min_ax, &max_ax);
      max_min_check(imu->ay, &min_ay, &max_ay);
      max_min_check(imu->az, &min_az, &max_az);

      cnt_dwn--;

      if (!(cnt_dwn % samprate))
        {
          printf("Elapsed time : %03d seconds.\n", ++elp_cnt);
        }
    }

  printf("Finish gyro-bias estimation.\n");

  for(i = 0; i < 3; i++)
    {
      gyrobias[i] /= (double)cnt;
      accelave[i] /= (double)cnt;
    }

  printf("gx: ave=%12.3e, min=%12.3e, max=%12.3e\n", gyrobias[0], min_gx, max_gx);
  printf("gy: ave=%12.3e, min=%12.3e, max=%12.3e\n", gyrobias[1], min_gy, max_gy);
  printf("gz: ave=%12.3e, min=%12.3e, max=%12.3e\n", gyrobias[2], min_gz, max_gz);

  printf("ax: ave=%12.3e, min=%12.3e, max=%12.3e\n", accelave[0], min_ax, max_ax);
  printf("ay: ave=%12.3e, min=%12.3e, max=%12.3e\n", accelave[1], min_ay, max_ay);
  printf("az: ave=%12.3e, min=%12.3e, max=%12.3e\n", accelave[2], min_az, max_az);

  return 0;
}
#endif

static uint64_t generate_unroll_timestamp(uint32_t curr_time)
{
  s_prev_unroll_time += (uint64_t)(curr_time - s_prev_time);
  s_prev_time = curr_time;
  return s_prev_unroll_time;
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
  int print_hex = 0;

  s_prev_time = 0;
  s_prev_unroll_time = 0;

  if (argc == 2 && argv[1][0] == 'h')
    {
      print_hex = 1;
    }

  INIT_AHRS(&ahrs, 0.5f, (float)DEFAULT_SAMPLERATE);

  fd = start_sensing(DEFAULT_SAMPLERATE,
                     DEFAULT_ACCELRANGE,
                     DEFAULT_GYROSCOPERANGE, 1);
  drop_50msdata(fd, DEFAULT_SAMPLERATE, &imu);

  /* Note: Add logic to remove gyro bias, othewise DC offset will be
   *       on the gyro sensor data.
   *       A commonly used method to remove the DC offset is, for example,
   *       to take the average of the gyro sensor data while it is
   *       stationary for several seconds, use that value as the BIAS
   *       value, and subtract that value from each gyro sample data.
   */
#ifdef CONFIG_EXAMPLES_AHRS_PWBIMU_EXEC_GYROBIAS_ESTIMATION 
  /* This is a example of gyro bias estimation.
   * This function estimate the gyro bias and the angular velocity
   * caused by the Earth's rotation together as offset values.
   * Please maintain a stationary state as much as possible
   * during the estimation.
   */
  gyrobias_estimation(fd, DEFAULT_SAMPLERATE, 
                          CONFIG_EXAMPLES_AHRS_PWBIMU_GYROBIAS_ESTIMATION_TIME,
                          gyrobias, &imu);
#endif

  set_initial_posture(fd, &ahrs, &imu);

  while (read_imudata(fd, &imu))
    {
        MadgwickAHRSupdateIMU(&ahrs,
                              imu.gx - (float)gyrobias[0],
                              imu.gy - (float)gyrobias[1],
                              imu.gz - (float)gyrobias[2],
                              imu.ax, imu.ay, imu.az);

        quaternion2euler(ahrs.q, e);

        if (++cnt >= 20)  /* Decimate data 1920Hz / 20 = 96Hz */
          {
            if (print_hex)
              {
                printf("%08x,%08x,%08x,%08x\n", *(unsigned int *)&ahrs.q[0],
                                                *(unsigned int *)&ahrs.q[1],
                                                *(unsigned int *)&ahrs.q[2],
                                                *(unsigned int *)&ahrs.q[3]);
              }
            else
              {
                uint64_t unroll_time = generate_unroll_timestamp(imu.timestamp);
                printf("T:%0.2f, R:%0.2f, P:%0.2f, Y:%0.2f\n", (float)(unroll_time/19200000.0f), RAD2DEG(e[0]), RAD2DEG(e[1]), RAD2DEG(e[2]));
              }

            cnt = 0;
          }
    }

  close(fd);

  return 0;
}
