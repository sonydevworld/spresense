/****************************************************************************
 * examples/pospredict_pwbimu/simple_pospredict_pwbimu_main.c
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
#include <string.h>

#include <nuttx/sensors/cxd5602pwbimu.h>
#include <MadgwickAHRS.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD5602PWBIMU_DEVPATH   "/dev/imu0"
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

#define TS_FREQ     (19200000)
#define SAMPLERATE  (1920)
#define ARANGE      (16)
#define GRANGE      (1000)

#define DRIFT_THREASH  (5.f)

#define GH(a) (gyro[(a)] / 2.f)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pospredict_s
{
  uint32_t last_ts; /* Last IMU timestamp */
  float posquat[4]; /* Posture Quaternion (order w,x,y,z) */
  float vel[3];     /* Velocity (m/sec) */
  float pos[3];     /* Position in global coordinate system (m) */
  float gravity;    /* Gravity (m/sec^2) */
  float gbias[3];   /* Gyro Bias */

  float calibtime;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pospredict_s g_inst;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
      printf("ERROR: Set FIFO threshold failed. %d\n", errno);
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

  fds[0].fd     = fileno(stdin);
  fds[0].events = POLLIN;
  fds[1].fd     = fd;
  fds[1].events = POLLIN;

  ret = poll(fds, 2, 1000);
  if (ret > 0)
    {
      if (fds[0].revents & POLLIN)
        {
          read(fds[0].fd, &c, 1);
          switch (c)
            {
              case 'z': return 2;
              case 'q': return 0;
              default : break;
            }
        }

      if (fds[1].revents & POLLIN)
        {
          ret = read(fd, imudata, sizeof(*imudata));
          if (ret == sizeof(*imudata))
            {
              return 1;
            }
        }
    }

  return 0;
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

static void init_pospredict(struct pospredict_s *inst)
{
  memset(inst, 0, sizeof(*inst));
  inst->posquat[0] = 1.f;
}

static void update_posture(float *q, float *gyro, float dt)
{
  float mat[4][4];
  float outq[4];
  float norm;

  gyro[0] = gyro[0] * dt; /* X axis */
  gyro[1] = gyro[1] * dt; /* Y axis */
  gyro[2] = gyro[2] * dt; /* Z axis */

  /* Skew symmetric Matrix for delta posture */

  memset(mat, 0, sizeof(float) * 4 * 4);
                      mat[1][0] =  GH(0); mat[2][0] =  GH(1); mat[3][0] =  GH(2);
  mat[0][1] = -GH(0);                     mat[2][1] = -GH(2); mat[3][1] =  GH(1);
  mat[0][2] = -GH(1); mat[1][2] =  GH(2);                     mat[3][2] = -GH(0);
  mat[0][3] = -GH(2); mat[1][3] = -GH(1); mat[2][3] =  GH(0);

  /* Calculate new posture in global coordinate system */

  memcpy(outq, q, sizeof(float) * 4);
  outq[0] += mat[0][0] * q[0] + mat[0][1] * q[1] + mat[0][2] * q[2] + mat[0][3] * q[3];
  outq[1] += mat[1][0] * q[0] + mat[1][1] * q[1] + mat[1][2] * q[2] + mat[1][3] * q[3];
  outq[2] += mat[2][0] * q[0] + mat[2][1] * q[1] + mat[2][2] * q[2] + mat[2][3] * q[3];
  outq[3] += mat[3][0] * q[0] + mat[3][1] * q[1] + mat[3][2] * q[2] + mat[3][3] * q[3];

  /* Normalize the quaternion */

  norm = sqrtf(outq[0]*outq[0] + outq[1]*outq[1] + outq[2]*outq[2] + outq[3]*outq[3]);
  q[0] = outq[0] / norm;
  q[1] = outq[1] / norm;
  q[2] = outq[2] / norm;
  q[3] = outq[3] / norm;
}

static void calibration(int fd, cxd5602pwbimu_data_t *imu, float *posquat,
                        float *gravity, float *gbias)
{
  int i, j;
  int cnt;
  float accav[3];
  memset(gbias, 0, sizeof(float) * 3);
  memset(accav, 0, sizeof(float) * 3);

  printf("Calibration will start in 3sec..\n T: ");
  for (i = 0; i < 3; i++)
    {
      printf("%d ", i - 3); fflush(stdout);
      for (j = 0; j < SAMPLERATE; j++)
        {
          read_imudata(fd, imu);
        }
    }

  printf("\n Calibration for 10sec..\n");
  cnt = 0;
  for (j = 0; j < 10; j++)
    {
      for (i = 0; i < SAMPLERATE; i++)
        {
          if (read_imudata(fd, imu) == 1)
            {
              cnt++;
              accav[0] += imu->ax;
              accav[1] += imu->ay;
              accav[2] += imu->az;
              gbias[0] += imu->gx;
              gbias[1] += imu->gy;
              gbias[2] += imu->gz;
            }
        }
      printf("%d", 9 - j); fflush(stdout);
    }

  accav[0] /= (float)cnt;
  accav[1] /= (float)cnt;
  accav[2] /= (float)cnt;
  postureByAccel(posquat, accav[0], accav[1], accav[2], 0.f);
  *gravity = sqrtf(accav[0] * accav[0] + accav[1] * accav[1] + accav[2] * accav[2]);

  gbias[0] /= (float)cnt;
  gbias[1] /= (float)cnt;
  gbias[2] /= (float)cnt;

  printf("\nG:%1.2f GB:(%2.2f, %2.2f, %2.2f)\n", *gravity, gbias[0], gbias[1], gbias[2]);
}

static void quat_to_rotmat(float *q, float (*rot)[3])
{
  rot[0][0] = 1 - 2 * (q[2]*q[2] + q[3]*q[3]);
  rot[0][1] =     2 * (q[1]*q[2] - q[0]*q[3]);
  rot[0][2] =     2 * (q[1]*q[3] + q[0]*q[2]);
  rot[1][0] =     2 * (q[1]*q[2] + q[0]*q[3]);
  rot[1][1] = 1 - 2 * (q[1]*q[1] + q[3]*q[3]);
  rot[1][2] =     2 * (q[2]*q[3] - q[0]*q[1]);
  rot[2][0] =     2 * (q[1]*q[3] - q[0]*q[2]);
  rot[2][1] =     2 * (q[2]*q[3] + q[0]*q[1]);
  rot[2][2] = 1 - 2 * (q[1]*q[1] + q[2]*q[2]);
}

static void simple_prediction(struct pospredict_s *inst, cxd5602pwbimu_data_t *imu)
{
  int i;
  float tsdiff;
  float adj_acc[3];
  float rotM[3][3];

  /* Delta time */

  tsdiff = ((float)(imu->timestamp - inst->last_ts)) / ((float)TS_FREQ);
  inst->last_ts = imu->timestamp;

  inst->calibtime += tsdiff;

  /* Adjust bias */

  imu->gx -= inst->gbias[0];
  imu->gy -= inst->gbias[1];
  imu->gz -= inst->gbias[2];

  /* Update posture */

  update_posture(inst->posquat, &imu->gx, tsdiff);

  /* Rotate acceleration elements to world coordinates */

  quat_to_rotmat(inst->posquat, rotM);

  memset(adj_acc, 0, sizeof(adj_acc));
  for (i = 0; i < 3; i++)
    {
      adj_acc[i] += rotM[i][0] * imu->ax;
      adj_acc[i] += rotM[i][1] * imu->ay;
      adj_acc[i] += rotM[i][2] * imu->az;
    }
  adj_acc[2] -= inst->gravity;  // Avoid gravity

  /* Update velocity */

  inst->vel[0] += adj_acc[0] * tsdiff;
  inst->vel[1] += adj_acc[1] * tsdiff;
  inst->vel[2] += adj_acc[2] * tsdiff;

  /* Update position */

  inst->pos[0] += inst->vel[0] * tsdiff;
  inst->pos[1] += inst->vel[1] * tsdiff;
  inst->pos[2] += inst->vel[2] * tsdiff;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int i;
  int fd;
  cxd5602pwbimu_data_t imu;
  int dispcnt;
  float e[3];

  init_pospredict(&g_inst);

  fd = start_sensing(SAMPLERATE, ARANGE, GRANGE, 1);
  drop_50msdata(fd, SAMPLERATE, &imu);
  calibration(fd, &imu, g_inst.posquat, &g_inst.gravity, g_inst.gbias);
  sleep(1);

  /* Dummy Read for timestamp synchro */

  for (i = 0; i < 10; i++)
    {
      read_imudata(fd, &imu);
    }
  g_inst.last_ts = imu.timestamp;

  dispcnt = 0;
  while (1)
    {
      switch (read_imudata(fd, &imu))
        {
          case 1:
            simple_prediction(&g_inst, &imu);
            if (dispcnt >= 20)  /* Decimation for display */
              {
                quaternion2euler(g_inst.posquat, e);
                printf("%sX:%2.2f Y:%2.2f Z:%2.2f R:%2.2f Y:%2.2f P:%2.2f\n",
                        (g_inst.calibtime > DRIFT_THREASH) ? "Drift may have begun, press 'z' " : "",
                        g_inst.pos[0], g_inst.pos[1], g_inst.pos[2],
                        RAD2DEG(e[0]), RAD2DEG(e[1]), RAD2DEG(e[2]));
                dispcnt = 0;
              }
            break;

          case 2:
            printf("==== Clear velocity as zero ====\n");
            memset(g_inst.vel, 0, sizeof(g_inst.vel));  /* Reset velocity as zero */
            g_inst.calibtime = 0.f;                     /* Clear calibration time */
            break;

          default:
            printf("Quit this app\n");
            goto endapp;
        }

      dispcnt++;
    }

endapp:

  close(fd);

  return 0;
}
