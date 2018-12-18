/****************************************************************************
 * transport_mode/temperature_sensor.h
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

#ifndef __TRAM_TEMPERATURE_SENSOR_H
#define __TRAM_TEMPERATURE_SENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/chip/cxd56_scu.h>
#include <nuttx/sensors/bmp280.h>

#include "memutils/memory_manager/MemHandle.h"
#include "sensing/logical_sensor/barometer.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef int (*TemperatureEventHandler) (uint32_t context,
                                        MemMgrLite::MemHandle &mh);

typedef struct
{
  TemperatureEventHandler  handler;    /* Event handler of the temperature sensor.  */
  uint32_t                 context;    /* Context of the temperature sensor.        */
  bool                     stopped;    /* Status flag of pressure process.          */
  pthread_t                thread_id;  /* ID of receiving thread.                   */
  int                      fd;         /* File discriptor of driver.                */
  sigset_t                 sig_set;    /* Information of signal from driver.        */
  struct scutimestamp_s    wm_ts;      /* Time stamp information of water mark.     */
  struct bmp280_temp_adj_s sens_adj;   /* Sensitivity adjustment value.             */
  FAR BarometerClass       *owner;     /* Owner class of Barometer                  */
} TemperatureSensor;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int TemperatureSensorCreate(FAR TemperatureSensor **sensor);
int TemperatureSensorRegisterHandler(FAR TemperatureSensor *sensor,
                                     TemperatureEventHandler handler,
                                     uint32_t context);
int TemperatureSensorStartSensing(FAR TemperatureSensor *sensor);
int TemperatureSensorDestroy(FAR TemperatureSensor *sensor);
int TemperatureSensorStopSensing(FAR TemperatureSensor *sensor);

#endif /* __TRAM_TEMPERATURE_SENSOR_H */
