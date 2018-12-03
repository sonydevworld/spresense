/****************************************************************************
 * step_counter/accel_sensor.cxx
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

#include <sdk/config.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <nuttx/sensors/bmi160.h>
#include <arch/chip/cxd56_scu.h>

#include "accel_sensor.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STEP_COUNTER_ACCEL_DEVNAME "/dev/accel0"

#ifndef CONFIG_EXAMPLES_SENSOR_STEP_COUNTER_ACCEL_WM_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_STEP_COUNTER_ACCEL_WM_SIGNO 14
#endif

#define err(format, ...)        fprintf(stderr, format, ##__VA_ARGS__)

#define CHECK_TRUE_GOTO(expr, failed)                                   \
      do {                                                              \
        if (!(expr)) {                                                  \
          err("check failed. %s, %d\n", __FUNCTION__, __LINE__);        \
          goto failed;                                                  \
        }                                                               \
      } while(0)


#define CHECK_NULL_RET(expr)                                            \
  do {                                                                  \
    if (expr == NULL) {                                                 \
      err("check failed. %s, %d\n", __FUNCTION__, __LINE__);            \
      return -1;                                                        \
    }                                                                   \
  } while(0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct accel_t three_axis_s;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool accel_sensing_stop = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int AccelSensorScuSetup(FAR AccelSensor *sensor)
{

  int                   ret;
  struct scufifo_wm_s   wm;

  /* Set FIFO size */

  ret = ioctl(sensor->fd,
              SCUIOC_SETFIFO,
              sizeof(three_axis_s) * ACCEL_WATERMARK_NUM * 2);
  if (ret < 0)
    {
      err("Accel set FIFO size error %d\n", ret);
      return ret;
    }

  /* Set sequencer sampling rate 32Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 4) = 32
   */

  ret = ioctl(sensor->fd, SCUIOC_SETSAMPLE, 4);
  if (ret < 0)
    {
      err("Accel set sequencer sampling rate error %d\n", ret);
      return ret;
    }

  /* Set water mark */

  wm.signo     = CONFIG_EXAMPLES_SENSOR_STEP_COUNTER_ACCEL_WM_SIGNO;
  wm.ts        = &sensor->wm_ts;
  wm.watermark = ACCEL_WATERMARK_NUM;

  ret = ioctl(sensor->fd,
              SCUIOC_SETWATERMARK,
              (unsigned long)(uintptr_t)&wm);
  if (ret < 0)
    {
      err("Accel set water mark error %d\n", ret);
      return ret;
    }

  return OK;
}

/*--------------------------------------------------------------------------*/
static void AccelConvertData(FAR three_axis_s *p_src,
                             FAR AccelDOF *p_dst,
                             int sample_num)
{
  for (int i = 0; i < sample_num; ++i)
    {
      p_dst->accel_x = (float)p_src->x * 2 / 32768;
      p_dst->accel_y = (float)p_src->y * 2 / 32768;
      p_dst->accel_z = (float)p_src->z * 2 / 32768;

      p_src++;
      p_dst++;
    }
}

/*--------------------------------------------------------------------------*/
static void AccelNotifyData(FAR AccelSensor* sensor,
                            MemMgrLite::MemHandle &mh_dst)
{
  if ((sensor->handler != NULL) && (sensor->stopped != true))
    {
      sensor->handler(sensor->context, mh_dst);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int AccelSensorCreate(FAR AccelSensor** sensor)
{
  CHECK_NULL_RET(sensor);

  *sensor = (AccelSensor *)malloc(sizeof(AccelSensor));
  memset(*sensor, 0 , sizeof(AccelSensor));

  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorRegisterHandler(FAR AccelSensor* sensor,
                               AccelEventHandler handler,
                               uint32_t context)
{
  sensor->handler = handler;
  sensor->context = context;

  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorStartSensing(FAR AccelSensor *sensor)
{
  int ret;
  MemMgrLite::MemHandle mh_dst;
  MemMgrLite::MemHandle mh_src;
  sigset_t              set;
  char                 *p_src;
  struct siginfo        value;
  struct timespec       timeout;

  sensor->fd = open(STEP_COUNTER_ACCEL_DEVNAME, O_RDONLY);
  if (sensor->fd <= 0)
    {
      err("Accel device open error %d\n", sensor->fd);
      return -1;
    }

  /* Set timeout 2 seconds, SCU may send signal every 1 second. */
  
  timeout.tv_sec  = 2;
  timeout.tv_nsec = 0;
  
  sigemptyset(&set);
  sigaddset(&set, CONFIG_EXAMPLES_SENSOR_STEP_COUNTER_ACCEL_WM_SIGNO);

  ret = AccelSensorScuSetup(sensor);
  if (ret < 0)
    {
      return ret;
    }

  /* Start sequencer */

  ret = ioctl(sensor->fd, SCUIOC_START, 0);
  ASSERT(ret == 0);

  sensor->stopped = false;
  while(!sensor->stopped)
    {
      ret = sigtimedwait(&set, &value, &timeout);
      if (ret < 0)
        {
          continue;
        }

      /* get MemHandle */

      if (ERR_OK != mh_src.allocSeg(
                      ACCEL_DATA_BUF_POOL,
                      (sizeof(three_axis_s) * ACCEL_WATERMARK_NUM)))
        {
          ASSERT(0);
        }
      p_src = reinterpret_cast<char *>(mh_src.getPa());

      if (ERR_OK != mh_dst.allocSeg(
                      ACCEL_DATA_BUF_POOL,
                      (sizeof(AccelDOF) * ACCEL_WATERMARK_NUM)))
        {
          ASSERT(0);
        }

      /* read accel data from driver */

      ret = read(sensor->fd,
                 p_src,
                 sizeof(three_axis_s) * ACCEL_WATERMARK_NUM);

      CHECK_TRUE_GOTO(ret == (sizeof(three_axis_s) * ACCEL_WATERMARK_NUM),
                      failed);

      AccelConvertData(reinterpret_cast<three_axis_s*>(p_src),
                       reinterpret_cast<AccelDOF*>(mh_dst.getPa()),
                       ACCEL_WATERMARK_NUM);
      AccelNotifyData(sensor, mh_dst);

      mh_src.freeSeg();
      mh_dst.freeSeg();
    }

  if (sensor->fd >= 0)
    {
      ret = ioctl(sensor->fd, SCUIOC_STOP, 0);
    }

failed:
  if (sensor->fd >= 0)
    {
      close(sensor->fd);
    }

  accel_sensing_stop = true;

  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorDestroy(AccelSensor* sensor)
{
  free(sensor);
  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorStopSensing(AccelSensor* sensor)
{
  CHECK_NULL_RET(sensor);
  sensor->stopped = true;

  while(!accel_sensing_stop)
    {
      sleep(1);
    }

  accel_sensing_stop = false;

  return 0;
}
