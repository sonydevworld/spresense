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

#include <sdk/config.h>
#include <arch/chip/cxd56_scu.h>

#include "memutils/memory_manager/MemHandle.h"
#include "include/mem_layout.h"
#include "include/msgq_id.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACCEL_SAMPLING_FREQUENCY 32  /* 32Hz */
#define ACCEL_WATERMARK_NUM      32  /* 32samples */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
  float accel_x;  /* X axis standard gravity acceleration.[G] */
  float accel_y;  /* Y axis standard gravity acceleration.[G] */
  float accel_z;  /* Z axis standard gravity acceleration.[G] */
} AccelDOF;

typedef int (*AccelEventHandler)(uint32_t context,
                                 MemMgrLite::MemHandle &mh);

struct accel_sensor_s
{
  /* Indicates the event handler of the acceleration sensor. */

  AccelEventHandler     handler;

  /* Indicates the context of the acceleration sensor. */

  uint32_t              context; 

  /* Status flag of accel process. */

  bool                  stopped;

  /* Indicates the file discriptor of driver. */
  
  int                   fd;

  /* Indicates the time stamp information of water mark. */

  struct scutimestamp_s wm_ts;
};

typedef struct accel_sensor_s AccelSensor;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int AccelSensorCreate(FAR AccelSensor **sensor);
int AccelSensorRegisterHandler(FAR AccelSensor *sensor,
                               AccelEventHandler handler,
                               uint32_t context);
int AccelSensorStartSensing(FAR AccelSensor *sensor);
int AccelSensorDestroy(FAR AccelSensor* sensor);
int AccelSensorStopSensing(FAR AccelSensor* sensor);

#endif /* _EXAMPLES_STEP_COUNTER_ACCEL_SENSOR_H */
