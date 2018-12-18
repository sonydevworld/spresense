/****************************************************************************
 * transport_mode/accel_sensor.cxx
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

#include "accel_sensor.h"
#include "sensing/logical_sensor/transport_mode.h"
#include "include/mem_layout.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRAM_ACCEL_DEVNAME "/dev/accel0"

#ifndef CONFIG_EXAMPLES_SENSOR_TRAM_CHANGE_SCU_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_TRAM_CHANGE_SCU_SIGNO 16
#endif
#ifndef CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO 15
#endif
#ifndef CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO 14
#endif
#ifndef CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO 13
#endif

#define err(format, ...)        fprintf(stderr, format, ##__VA_ARGS__)

/* task priority */

#define SENSING_TASK_PRIORITY  150

/* value check macros */

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

typedef struct accel_t three_axis_s;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct ScuSettings* s_scu_settings;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int createOneshotTimer(FAR timer_t *timerid)
{
  struct sigevent notify;

  notify.sigev_notify            = SIGEV_SIGNAL;
  notify.sigev_signo             = CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO;
  notify.sigev_value.sival_int   = 0;

  return timer_create(CLOCK_REALTIME, &notify, timerid);
}

/*--------------------------------------------------------------------------*/
static int startOneshotTimer(timer_t timerid, uint32_t milliseconds)
{
  struct itimerspec timer;

  timer.it_value.tv_sec     = milliseconds / 1000;
  timer.it_value.tv_nsec    = milliseconds % 1000 * 1000 * 1000;
  timer.it_interval.tv_sec  = 0;
  timer.it_interval.tv_nsec = 0;

  return timer_settime(timerid, 0, &timer, NULL);
}

/*--------------------------------------------------------------------------*/
static int deleteOneshotTimer(timer_t timerid)
{
  return timer_delete(timerid);
}

/*--------------------------------------------------------------------------*/
static int setupAccel(FAR AccelSensor *sensor,
                      FAR struct ScuSettings *settings)
{
  uint32_t scu_sampling_rate;
  uint32_t power_mode;
  uint32_t output_data_rate;

  scu_sampling_rate =
    GET_SCU_ACCEL_SAMPLING_FREQUENCY(settings->samplingrate);

  switch (scu_sampling_rate)
    {
      case ACCEL_SAMPLING_FREQUENCY_MS:
        power_mode       = BMI160_PM_LOWPOWER;
        output_data_rate = BMI160_ACCEL_ODR_25HZ;
        break;

      case ACCEL_SAMPLING_FREQUENCY_CMD:
        power_mode       = BMI160_PM_NORMAL;
        output_data_rate = BMI160_ACCEL_ODR_100HZ;
        break;

      default:
        return -1;
    }

  /* Set power mode. */

  CHECK_FUNC_RET(ioctl(sensor->fd, SNIOC_SETACCPM, power_mode));

  /* Set output data rate. */

  CHECK_FUNC_RET(ioctl(sensor->fd, SNIOC_SETACCODR, output_data_rate));

  return 0;
}

/*--------------------------------------------------------------------------*/
static int setupScu(FAR AccelSensor *sensor,
                    FAR struct ScuSettings *settings)
{
  /* Set FIFO size */

  CHECK_FUNC_RET(ioctl(sensor->fd,
                       SCUIOC_SETFIFO,
                       sizeof(three_axis_s) * settings->fifosize));

  /* Set sampling rate */

  CHECK_FUNC_RET(ioctl(sensor->fd,
                       SCUIOC_SETSAMPLE,
                       settings->samplingrate));

  /* Set elements */

  if (settings->elements)
    {
      CHECK_FUNC_RET(ioctl(sensor->fd,
                           SCUIOC_SETELEMENTS,
                           settings->elements));
    }

  /* Set MathFunction filter */

  if (settings->mf)
    {
      CHECK_FUNC_RET(ioctl(sensor->fd,
                           SCUIOC_SETFILTER,
                           (unsigned long)(uintptr_t)settings->mf));
    }

  /* Set event */

  if (settings->ev)
    {
      settings->ev->signo = CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO;
      settings->ev->arg   = &sensor->ev_arg;

      CHECK_FUNC_RET(ioctl(sensor->fd,
                           SCUIOC_SETNOTIFY,
                           (unsigned long)(uintptr_t)settings->ev));
    }

  /* Set water mark */

  if (settings->wm)
    {
      settings->wm->signo = CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO;
      settings->wm->ts    = &sensor->wm_ts;
      CHECK_FUNC_RET(ioctl(sensor->fd,
                           SCUIOC_SETWATERMARK,
                           (unsigned long)(uintptr_t)settings->wm));
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
static int notifyData(FAR AccelSensor *sensor,
                      MemMgrLite::MemHandle &mh_src,
                      MemMgrLite::MemHandle &mh_dst)
{
  FAR three_axis_s *p_src =
    reinterpret_cast<FAR three_axis_s *>(mh_src.getVa());
  FAR AccelDOF     *p_dst =
    reinterpret_cast<FAR AccelDOF *>(mh_dst.getVa());

  for (int i = 0; i < ACCEL_WATERMARK_NUM; ++i)
    {
      p_dst->accel_x = (float)p_src->x * 2 / 32768;
      p_dst->accel_y = (float)p_src->y * 2 / 32768;
      p_dst->accel_z = (float)p_src->z * 2 / 32768;

      p_src++;
      p_dst++;
    }

  if ((sensor->handler != NULL) && !sensor->stopped)
    {
      sensor->handler(sensor->context, ACCEL_EV_WM, mh_dst);
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
static int restartScu(FAR AccelSensor *sensor)
{
  /* Stop SCU */

  CHECK_FUNC_RET(ioctl(sensor->fd, SCUIOC_STOP, 0));

  /* Free SCU FIFO */

  CHECK_FUNC_RET(ioctl(sensor->fd, SCUIOC_FREEFIFO, 0));

  /* Set up accel physical sensor */

  CHECK_FUNC_RET(setupAccel(sensor, s_scu_settings));

  /* Set up SCU */

  CHECK_FUNC_RET(setupScu(sensor, s_scu_settings));

  /* Restart SCU */

  CHECK_FUNC_RET(ioctl(sensor->fd, SCUIOC_START, 0));

  return 0;
}

/*--------------------------------------------------------------------------*/
extern "C" void *AccelSensorReceivingThread(FAR void *arg)
{
  int              ret;
  FAR char         *p_src;
  struct siginfo   value;
  struct timespec  timeout;
  FAR AccelSensor  *sensor;
  timer_t timerid;
  bool is_first_ev = true;
  bool is_rise = false;

  sensor = reinterpret_cast<FAR AccelSensor *>(arg);

  /* Set up accel physical sensor */

  ret = setupAccel(sensor, s_scu_settings);
  if (ret != 0)
    {
      err("Setup accel failed. error = %d\n", ret);
      ASSERT(0);
    }

  /* Setup scu */

  ret = setupScu(sensor, s_scu_settings);
  if (ret != 0)
    {
      err("Setup scu failed. error = %d\n", ret);
      ASSERT(0);
    }

  /* Create oneshot timer */

  ret = createOneshotTimer(&timerid);
  if (ret != 0)
    {
      err("Create timer failed. error = %d\n", ret);
      ASSERT(0);
    }

  /* Start sequencer */

  ret = ioctl(sensor->fd, SCUIOC_START, 0);
  if (ret != 0)
    {
      err("Start accel sensor failed. error = %d\n", ret);
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
      else if (ret == CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO)
        {
          /* mathfunc */

          FAR struct scuev_arg_s *scuev =
            (FAR struct scuev_arg_s *)value.si_value.sival_ptr;

          if (scuev->type == SCU_EV_RISE)
            {
              is_rise = true;

              if (is_first_ev)
                {
                  /* Rise event occurs just after sensor starts
                   * because of the filter setting.
                   * So check later if it is moving or not.
                   */

                  ret = startOneshotTimer(timerid, 1000);
                  if (ret != 0)
                    {
                      err("Start timer failed.\n");
                      ASSERT(0);
                    }

                  is_first_ev = false;
                }
              else
                {
                  MemMgrLite::MemHandle dummy;
                  sensor->handler(sensor->context, ACCEL_EV_MF, dummy);
                }
            }
          else
            {
              is_rise = false;
            }

          continue;
        }
      else if (ret == CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO)
        {
          if (is_rise)
            {
              /* Still rise, then send rise event */

              MemMgrLite::MemHandle dummy;
              sensor->handler(sensor->context, ACCEL_EV_MF, dummy);
            }

          continue;
        }
      else if (ret == CONFIG_EXAMPLES_SENSOR_TRAM_CHANGE_SCU_SIGNO)
        {
          /* Reset status flags */

          is_first_ev = true;
          is_rise = false;

          /* Rester accel and scu */

          if (restartScu(sensor) != 0)
            {
              ASSERT(0);
            }

          continue;
        }
      else
        {
          /* Watermark */

          /* Do nothing */
        }

      /* Get MemHandle */

      MemMgrLite::MemHandle mh_src;
      MemMgrLite::MemHandle mh_dst;
      if (mh_src.allocSeg(
            ACCEL_DATA_BUF_POOL,
            (sizeof(three_axis_s) * ACCEL_WATERMARK_NUM)) != ERR_OK)
        {
          err("Get MemHandle failed.\n");
          ASSERT(0);
        }

      /* Set physical address.
       * (The addess specified for the SCU must be a physical address)
       */

      p_src = reinterpret_cast<FAR char *>(mh_src.getPa());

      if (mh_dst.allocSeg(
            ACCEL_DATA_BUF_POOL,
            (sizeof(AccelDOF)*ACCEL_WATERMARK_NUM)) != ERR_OK)
        {
          ASSERT(0);
        }

      /* Read accel data from driver */

      ret = read(sensor->fd,
                 p_src,
                 sizeof(three_axis_s) * ACCEL_WATERMARK_NUM);
      if (ret != (sizeof(three_axis_s) * ACCEL_WATERMARK_NUM))
        {
          ASSERT(0);
        }

      notifyData(sensor, mh_src, mh_dst);
    }

  deleteOneshotTimer(timerid);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int AccelSensorCreate(FAR AccelSensor **sensor)
{
  CHECK_NULL_RET(sensor);

  *sensor = (AccelSensor *)malloc(sizeof(AccelSensor));
  memset(*sensor, 0 , sizeof(AccelSensor));

  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorRegisterHandler(FAR AccelSensor *sensor,
                               AccelEventHandler handler,
                               uint32_t context)
{
  CHECK_NULL_RET(sensor);

  sensor->handler = handler;
  sensor->context = context;

  return 0;
}


/*--------------------------------------------------------------------------*/
int AccelSensorStartSensing(FAR AccelSensor *sensor,
                            FAR struct ScuSettings *settings)
{
  pthread_attr_t attr;
  struct sched_param sch_param;

  CHECK_NULL_RET(sensor);
  CHECK_NULL_RET(settings);

  /* Open accel driver */

  sensor->fd = open(TRAM_ACCEL_DEVNAME, O_RDONLY);
  if (sensor->fd <= 0)
    {
      err("Accel device open error %d\n", sensor->fd);
      return -1;
    }

  /* Set status */

  sensor->stopped = false;

  /* Add signal */

  sigemptyset(&sensor->sig_set);
  sigaddset(&sensor->sig_set, CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO);
  sigaddset(&sensor->sig_set, CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO);
  sigaddset(&sensor->sig_set, CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO);
  sigaddset(&sensor->sig_set, CONFIG_EXAMPLES_SENSOR_TRAM_CHANGE_SCU_SIGNO);

  /* Set SCU settings */

  s_scu_settings = settings;

  /* Create receive thread */

  (void)pthread_attr_init(&attr);
  sch_param.sched_priority = SENSING_TASK_PRIORITY;
  CHECK_FUNC_RET(pthread_attr_setschedparam(&attr, &sch_param));

  CHECK_FUNC_RET(pthread_create(&sensor->thread_id,
                 &attr,
                 AccelSensorReceivingThread,
                 (pthread_addr_t)sensor));

  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorStopSensing(FAR AccelSensor *sensor)
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

  CHECK_FUNC_RET(ioctl(sensor->fd, SCUIOC_STOP, 0));

  /* Delete signal */

  sigdelset(&sensor->sig_set, CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO);
  sigdelset(&sensor->sig_set, CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO);
  sigdelset(&sensor->sig_set, CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO);
  sigdelset(&sensor->sig_set, CONFIG_EXAMPLES_SENSOR_TRAM_CHANGE_SCU_SIGNO);

  /* Close driver */

  CHECK_FUNC_RET(close(sensor->fd));

  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorDestroy(FAR AccelSensor *sensor)
{
  CHECK_NULL_RET(sensor);

  free(sensor);
  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorChangeScuSetting(FAR AccelSensor *sensor,
                                FAR struct ScuSettings *settings)
{
  CHECK_NULL_RET(sensor);
  CHECK_NULL_RET(settings);

  if (settings == s_scu_settings)
    {
      /* No need to change */

      return 0;
    }

  s_scu_settings = settings;

#ifdef CONFIG_CAN_PASS_STRUCTS
  union sigval value;
  value.sival_ptr = NULL;
  sigqueue(sensor->thread_id,
           CONFIG_EXAMPLES_SENSOR_TRAM_CHANGE_SCU_SIGNO,
           value);
#else
  sigqueue(sensor->thread_id,
           CONFIG_EXAMPLES_SENSOR_TRAM_CHANGE_SCU_SIGNO,
           0);
#endif

  return 0;
}
