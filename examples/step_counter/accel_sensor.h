/****************************************************************************
 * step_counter/accel_sensor.h
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

#ifndef _STEP_COUNTER_ACCEL_SENSOR_H
#define _STEP_COUNTER_ACCEL_SENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/scu.h>

#include "physical_sensor.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACCEL_SAMPLING_FREQUENCY 32  /* 32Hz */
#define ACCEL_WATERMARK_NUM      32  /* 32samples */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

FAR physical_sensor_t *AccelSensorCreate(pysical_event_handler_t handler);
int AccelSensorOpen(FAR physical_sensor_t *sensor);
int AccelSensorStart(FAR physical_sensor_t *sensor);
int AccelSensorStop(FAR physical_sensor_t *sensor);
int AccelSensorClose(FAR physical_sensor_t *sensor);
int AccelSensorDestroy(FAR physical_sensor_t *sensor);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

/****************************************************************************
 * Class
 ****************************************************************************/

#ifdef __cplusplus

class AccelSensorClass : public PhysicalSensorClass
{
public:

  AccelSensorClass(FAR physical_sensor_t *sensor) :
    PhysicalSensorClass(sensor)
    {
      create();
    };

  ~AccelSensorClass(){};

private:

  struct accel_float_s
    {
      float x;  /* X axis standard gravity acceleration.[G] */
      float y;  /* Y axis standard gravity acceleration.[G] */
      float z;  /* Z axis standard gravity acceleration.[G] */
    };
  typedef struct accel_float_s accel_float_t;

  /* Override method */

  int open_sensor();
  int close_sensor();
  int start_sensor();
  int stop_sensor();

  int setup_sensor(FAR void *param);
  int setup_scu(FAR void *param);
  int receive_signal(int sig_no, FAR siginfo_t *sig_info);

  /* Local method */

  int receive_scu_wm_ev();
  void convert_data(FAR struct accel_t *p_src,
                    FAR accel_float_t *p_dst,
                    int sample_num);
  int notify_data(MemMgrLite::MemHandle &mh_dst);

  /* Inline method */

  uint32_t get_timestamp()
    {
      /* Get timestamp in millisecond. Tick in 32768 Hz  */

      return 1000 * m_wm_ts.sec + ((1000 * m_wm_ts.tick) >> 15);
    }

  int m_fd;
  struct scutimestamp_s m_wm_ts;
};

#endif /* __cplusplus */
#endif /* _EXAMPLES_STEP_COUNTER_ACCEL_SENSOR_H */
