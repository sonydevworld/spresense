/****************************************************************************
 * transport_mode/magnetometer_sensor.cxx
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
#include <nuttx/sensors/ak09912.h>

#include "sensing/logical_sensor/transport_mode.h"
#include "include/mem_layout.h"
#include "magnetometer_sensor.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRAM_MAG_DEVNAME "/dev/mag0"

#ifndef CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO 12
#endif

/* Task priority */

#define SENSING_TASK_PRIORITY  150

/* Error message */

#define err(format, ...)        fprintf(stderr, format, ##__VA_ARGS__)

/* Value check macros */

#define CHECK_FUNC_RET(func)                                            \
  do {                                                                  \
    if ((func) < 0) {                                                   \
      err("return error, %s, %d\n", __FUNCTION__, __LINE__);            \
      return -1;                                                        \
    }                                                                   \
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

typedef struct mag_data_s three_axis_s;

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define AK09912_SENSITIVITY      128
#define AK09912_SENSITIVITY_DIV  256
#define AK9912_DECIMAL_MAX       32752.0F
#define AK9912_PHISICAL_MAX      4912.0F

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int MagnetometerSensorSetupScu(FAR MagnetmeterSensor *sensor)
{
  struct scufifo_wm_s wm;
  struct ak09912_sensadj_s sensadj;

  /* Set adjust value */

  CHECK_FUNC_RET(ioctl(sensor->fd,
                       SNIOC_GETADJ,
                       (unsigned long)(uintptr_t)&sensadj));

  sensor->sens_adj[0] = sensadj.x + AK09912_SENSITIVITY;
  sensor->sens_adj[1] = sensadj.y + AK09912_SENSITIVITY;
  sensor->sens_adj[2] = sensadj.z + AK09912_SENSITIVITY;

  /* Set FIFO size to 6 bytes * 8 Hz = 48 */

  CHECK_FUNC_RET(ioctl(sensor->fd,
                       SCUIOC_SETFIFO,
                       sizeof(three_axis_s) * MAG_WATERMARK_NUM));

  /* Set sequencer sampling rate 8 Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 6) = 8
   */

  CHECK_FUNC_RET(ioctl(sensor->fd, SCUIOC_SETSAMPLE, 6));

  /* Set watermark */

  wm.signo = CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO;
  wm.ts = &sensor->wm_ts;
  wm.watermark = MAG_WATERMARK_NUM;

  CHECK_FUNC_RET(ioctl(sensor->fd,
                       SCUIOC_SETWATERMARK,
                       (unsigned long)(uintptr_t)&wm));

  return 0;
}

/*--------------------------------------------------------------------------*/
static inline float MagnetometerGetCompValue(FAR MagnetmeterSensor *sensor,
                                             int16_t val,
                                             uint32_t axis)
{
  return (float)(((val * (int32_t)sensor->sens_adj[axis]) >> 8) *
                    AK9912_PHISICAL_MAX / AK9912_DECIMAL_MAX);
}

/*--------------------------------------------------------------------------*/
static int MagnetmeterNotifyData(FAR MagnetmeterSensor *sensor,
                                 MemMgrLite::MemHandle &mh_src,
                                 MemMgrLite::MemHandle &mh_dst)
{
  FAR three_axis_s *p_src =
    reinterpret_cast<FAR three_axis_s *>(mh_src.getVa());
  FAR MagnetometerDOF *p_dst =
    reinterpret_cast<FAR MagnetometerDOF *>(mh_dst.getVa());

  for (int i = 0; i < MAG_WATERMARK_NUM; i++)
    {
      p_dst->x = MagnetometerGetCompValue(sensor, p_src->x, 0);
      p_dst->y = MagnetometerGetCompValue(sensor, p_src->y, 1);
      p_dst->z = MagnetometerGetCompValue(sensor, p_src->z, 2);

      p_src++;
      p_dst++;
    }

  if ((sensor->handler != NULL) && !sensor->stopped)
    {
      sensor->handler(sensor->context, mh_dst);
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
extern "C" void *MagnetmeterSensorReceivingThread(FAR void *arg)
{
  int                   ret;
  FAR char              *p_src;
  struct siginfo        value;
  struct timespec       timeout;
  FAR MagnetmeterSensor *sensor;

  sensor = reinterpret_cast<FAR MagnetmeterSensor *>(arg);

  /* Setup scu */

  ret = MagnetometerSensorSetupScu(sensor);
  if (ret != 0)
    {
      err("Setup mag failed. error = %d\n", ret);
      ASSERT(0);
    }

  /* Start sequencer */

  ret = ioctl(sensor->fd, SCUIOC_START, 0);
  if (ret != 0)
    {
      err("Start magnetometer sensor failed. error = %d\n", ret);
      ASSERT(0);
    }

  /* Set timeout 6 seconds, SCU may send signal every 5 second. */

  timeout.tv_sec  = 6;
  timeout.tv_nsec = 0;

  while(!sensor->stopped)
    {
      ret = sigtimedwait(&sensor->sig_set, &value, &timeout);
      if (ret < 0)
        {
          continue;
        }

      /* Get MemHandle */

      MemMgrLite::MemHandle mh_src;
      MemMgrLite::MemHandle mh_dst;
      if (mh_src.allocSeg(
            MAG_DATA_BUF_POOL,
            (sizeof(three_axis_s) * MAG_WATERMARK_NUM)) != ERR_OK)
        {
          err("Get MemHandle of src failed.\n");
          ASSERT(0);
        }

      /* Set physical address.
       * (The addess specified for the SCU must be a physical address)
       */

      p_src = reinterpret_cast<FAR char *>(mh_src.getPa());

      if (mh_dst.allocSeg(
            MAG_DATA_BUF_POOL,
            (sizeof(MagnetometerDOF) * MAG_WATERMARK_NUM)) != ERR_OK)
        {
          err("Get MemHandle of dst failed.\n");
          ASSERT(0);
        }

      ret = read(sensor->fd, p_src, sizeof(three_axis_s) * MAG_WATERMARK_NUM);
      if (ret != (sizeof(three_axis_s) * MAG_WATERMARK_NUM))
        {
          err("Read data failed.\n");
          ASSERT(0);
        }

      MagnetmeterNotifyData(sensor, mh_src, mh_dst);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int MagnetmeterSensorCreate(FAR MagnetmeterSensor **sensor)
{
  CHECK_NULL_RET(sensor);

  *sensor = (FAR MagnetmeterSensor *)malloc(sizeof(MagnetmeterSensor));
  memset(*sensor, 0 , sizeof(MagnetmeterSensor));

  return 0;
}

/*--------------------------------------------------------------------------*/
int MagnetmeterSensorRegisterHandler(FAR MagnetmeterSensor *sensor,
                                     MagnetometerEventHandler handler,
                                     uint32_t context)
{
  CHECK_NULL_RET(sensor);

  sensor->handler = handler;
  sensor->context = context;

  return 0;
}

/*--------------------------------------------------------------------------*/
int MagnetmeterSensorStartSensing(FAR MagnetmeterSensor *sensor)
{
  pthread_attr_t attr;
  struct sched_param sch_param;

  CHECK_NULL_RET(sensor);

  /* Open driver */

  sensor->fd = open(TRAM_MAG_DEVNAME, O_RDONLY);
  if (sensor->fd <= 0)
    {
      err("Mag device open error %d\n", sensor->fd);
      return -1;
    }

  /* Set status */

  sensor->stopped = false;

  /* Add signal */

  sigemptyset(&sensor->sig_set);
  sigaddset(&sensor->sig_set, CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO);

  /* Create receive thread */

  (void)pthread_attr_init(&attr);
  sch_param.sched_priority = SENSING_TASK_PRIORITY;
  CHECK_FUNC_RET(pthread_attr_setschedparam(&attr, &sch_param));

  CHECK_FUNC_RET(pthread_create(&sensor->thread_id,
                                &attr,
                                MagnetmeterSensorReceivingThread,
                                (pthread_addr_t)sensor));

  return 0;
}

/*--------------------------------------------------------------------------*/
int MagnetmeterSensorStopSensing(FAR MagnetmeterSensor *sensor)
{
  FAR void *value;

  CHECK_NULL_RET(sensor);

  if (sensor->fd == 0)
    {
      /* Already stopped */

      return 0;
    }

  /* Set status */

  sensor->stopped = true;

  /* Cancel thread */

  pthread_cancel(sensor->thread_id);
  pthread_join(sensor->thread_id, &value);

  /* Delete signal */

  sigdelset(&sensor->sig_set, CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO);

  /* Close driver */

  close(sensor->fd);

  return 0;
}

/*--------------------------------------------------------------------------*/
int MagnetmeterSensorDestroy(FAR MagnetmeterSensor *sensor)
{
  CHECK_NULL_RET(sensor);

  free(sensor);
  return 0;
}
