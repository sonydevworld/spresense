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

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <nuttx/sensors/bmi160.h>

#include "accel_sensor.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For physical sensor. */

#define STEP_COUNTER_ACCEL_DEVNAME "/dev/accel0"

#ifndef CONFIG_EXAMPLES_SENSOR_STEP_COUNTER_ACCEL_WM_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_STEP_COUNTER_ACCEL_WM_SIGNO 14
#endif

/* For error */

#define err(format, ...)    fprintf(stderr, format, ##__VA_ARGS__)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR void *accel_sensor_entry(pthread_addr_t arg)
{
  static int s_sensor_entry_result = PHYSICAL_SENSOR_ERR_CODE_OK;
  FAR physical_sensor_t *sensor =
    reinterpret_cast<FAR physical_sensor_t *>(arg);

  /* Create instanse of AccelSensorClass. */

  AccelSensorClass *instance = new AccelSensorClass(sensor);

  /* Set sensor signal number. */

  instance->add_signal(CONFIG_EXAMPLES_SENSOR_STEP_COUNTER_ACCEL_WM_SIGNO);

  /* Start accel sensor process. */

  instance->run();

  /* Delete sensor signal number. */

  instance->delete_signal(CONFIG_EXAMPLES_SENSOR_STEP_COUNTER_ACCEL_WM_SIGNO);

  /* Free instance of AccelSensorClass. */

  free(instance);

  return (void *)&s_sensor_entry_result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR physical_sensor_t *AccelSensorCreate(pysical_event_handler_t handler)
{
  return PhysicalSensorCreate(handler,
                              (void *)accel_sensor_entry,
                              "accel_sensor");
}

/*--------------------------------------------------------------------------*/
int AccelSensorOpen(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorOpen(sensor, NULL);
}

/*--------------------------------------------------------------------------*/
int AccelSensorStart(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorStart(sensor);
}

/*--------------------------------------------------------------------------*/
int AccelSensorStop(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorStop(sensor);
}

/*--------------------------------------------------------------------------*/
int AccelSensorClose(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorClose(sensor);
}

/*--------------------------------------------------------------------------*/
int AccelSensorDestroy(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorDestroy(sensor);
}

/****************************************************************************
 * AccelSensorClass
 ****************************************************************************/

int AccelSensorClass::open_sensor()
{
  m_fd = open(STEP_COUNTER_ACCEL_DEVNAME, O_RDONLY);
  if (m_fd <= 0)
    {
      return -1;
    }
  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::close_sensor()
{
  return close(m_fd);
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::start_sensor()
{
  return ioctl(m_fd, SCUIOC_START, 0);
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::stop_sensor()
{
  return ioctl(m_fd, SCUIOC_STOP, 0);
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::setup_sensor(FAR void *param)
{
  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::setup_scu(FAR void *param)
{
  /* Free FIFO. */

  int ret = ioctl(m_fd, SCUIOC_FREEFIFO, 0);
  if (ret < 0)
    {
      err("Accel free FIFO error %d\n", ret);
      return ret;
    }

  /* Set FIFO size. */

  ret = ioctl(m_fd,
              SCUIOC_SETFIFO,
              sizeof(struct accel_t) * ACCEL_WATERMARK_NUM * 2);
  if (ret < 0)
    {
      err("Accel set FIFO size error %d\n", ret);
      return ret;
    }

  /* Set sequencer sampling rate 32Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 4) = 32
   */

  ret = ioctl(m_fd, SCUIOC_SETSAMPLE, 4);
  if (ret < 0)
    {
      err("Accel set sequencer sampling rate error %d\n", ret);
      return ret;
    }

  /* Set water mark */

  struct scufifo_wm_s wm;
  wm.signo     = CONFIG_EXAMPLES_SENSOR_STEP_COUNTER_ACCEL_WM_SIGNO;
  wm.ts        = &m_wm_ts;
  wm.watermark = ACCEL_WATERMARK_NUM;

  ret = ioctl(m_fd,
              SCUIOC_SETWATERMARK,
              static_cast<unsigned long>((uintptr_t)&wm));
  if (ret < 0)
    {
      err("Accel set water mark error %d\n", ret);
      return ret;
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::receive_signal(int sig_no, FAR siginfo_t *sig_info)
{
  int ret = -1;

  switch (sig_no)
    {
      case CONFIG_EXAMPLES_SENSOR_STEP_COUNTER_ACCEL_WM_SIGNO:
        {
          ret = receive_scu_wm_ev();
        }
        break;

      default:
        break;
    }
  return ret;
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::receive_scu_wm_ev()
{
  MemMgrLite::MemHandle mh_dst;
  MemMgrLite::MemHandle mh_src;
  FAR char *p_src;
  FAR char *p_dst;

  /* Get segment of memory handle. */

  if (ERR_OK != mh_src.allocSeg(
                  S0_ACCEL_DATA_BUF_POOL,
                  (sizeof(struct accel_t) * ACCEL_WATERMARK_NUM)))
    {
      /* Fatal error occured. */

      err("Fail to allocate segment of memory handle.\n");
      ASSERT(0);
    }
  p_src = reinterpret_cast<char *>(mh_src.getPa());

  if (ERR_OK != mh_dst.allocSeg(
                  S0_ACCEL_DATA_BUF_POOL,
                  (sizeof(accel_float_t) * ACCEL_WATERMARK_NUM)))
    {
      /* Fatal error occured. */

      err("Fail to allocate segment of memory handle.\n");
      ASSERT(0);
    }
  p_dst = reinterpret_cast<char *>(mh_dst.getPa());

  /* Read accelerometer data from driver. */

  read(m_fd, p_src, sizeof(struct accel_t) * ACCEL_WATERMARK_NUM);

  this->convert_data(reinterpret_cast<FAR struct accel_t *>(p_src),
                     reinterpret_cast<FAR accel_float_t *>(p_dst),
                     ACCEL_WATERMARK_NUM);

  /* Notify accelerometer data to sensor manager. */

  this->notify_data(mh_dst);

  /* Free segment. */

  mh_src.freeSeg();
  mh_dst.freeSeg();

  return 0;
}

/*--------------------------------------------------------------------------*/
void AccelSensorClass::convert_data(FAR accel_t *p_src,
                                    FAR accel_float_t *p_dst,
                                    int sample_num)
{
  for (int i = 0; i < sample_num; i++, p_src++, p_dst++)
    {
      p_dst->x = (float)p_src->x * 2 / 32768;
      p_dst->y = (float)p_src->y * 2 / 32768;
      p_dst->z = (float)p_src->z * 2 / 32768;
    }
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::notify_data(MemMgrLite::MemHandle &mh_dst)
{
  uint32_t timestamp = get_timestamp();

  return m_handler(0, timestamp, mh_dst);
};
