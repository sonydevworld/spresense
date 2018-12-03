/****************************************************************************
 * modules/include/sensing/step_counter_command.h
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

#ifndef __INCLUDE_SENSING_STEP_COUNTER_COMMAND_H
#define __INCLUDE_SENSING_STEP_COUNTER_COMMAND_H

/**
 * @defgroup logical_step_counter STEP_COUNTER API
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

#include "sensing/logical_sensor/physical_command.h"
#include "sensing/logical_sensor/step_counter.h"

/**
 * @file step_counter_command.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * @def STEP_COUNTER_SAMPLING_MAX
 * @brief Indicates the maximum number of samples.
 */
#define STEP_COUNTER_SAMPLING_MAX         (32)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @enum StepCounterMovementType
 * @brief Activity Class
 */
typedef enum
{
  STEP_COUNTER_MOVEMENT_TYPE_OTHER = 0, /**< It can not be recognized. */
  STEP_COUNTER_MOVEMENT_TYPE_STILL,     /**< It is stopped.            */
  STEP_COUNTER_MOVEMENT_TYPE_WALK,      /**< It is walking.            */
  STEP_COUNTER_MOVEMENT_TYPE_RUN,       /**< It is running.            */
} StepCounterMovementType;

/*--------------------------------------------------------------------------*/
/**
 * @enum StepCounterCmdType
 * @brief Publishers
 */
typedef enum
{
  STEP_COUNTER_CMD_UPDATE_ACCELERATION = 0 , /**< Acceleration sensor data
                                      *   is used to update acceleration.
                                      */
  STEP_COUNTER_CMD_UPDATE_GPS ,              /**< Update GPS information
                                      *   using GPS sensor data.
                                      */
  STEP_COUNTER_CMD_STEP_SET                  /**< Set user step setting. */
} StepCounterCmdType;

/*--------------------------------------------------------------------------*/
/**
 * Debug Dump Information
 */
struct StepCounterDebugDumpInfo
{
  void    *addr;  /**< debug dump area address. */
  size_t  size;   /**< debug dump area size.    */
};

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorInitStepCounter
 * @brief the structure of Accelstep setting for initilaize.
 */
typedef struct
{
  StepCounterSetting        setting; /**< Step user setting structure. */
  StepCounterDebugDumpInfo  debug_dump_info; /**< debug dump information. */
} SensorInitStepCounter;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorExecStepCounter
 * @brief the command of STEP_COUNTER execute by a frame(a few sample).
 */
typedef struct
{
  StepCounterCmdType cmd_type; /**< Command type of step counter. */

  union
    {
      ThreeAxisSampleData update_acc;  /**< Acceleration update command.    */
      GnssSampleData      update_gps;  /**< GPS information update command. */
      StepCounterSetting  setting;     /**< Step user setting command.      */
    };
} SensorExecStepCounter;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorFlushStepCounter
 * @brief the command of STEP_COUNTER terminate.
 */
typedef struct
{

} SensorFlushStepCounter;

/*--------------------------------------------------------------------------*/
/**
 * @struct StepCounterStepInfo
 * @brief the structure of STEP_COUNTER results.
 */
typedef struct {

  float    tempo;     /**< Indicates tempo of walking / jogging calculated
                       *   from the input acceleration data.
                       *   The unit is [Hz].
                       */
  float    stride;    /**< Indicates stride calculated
                       *   from input acceleration data.
                       *   The unit is [cm].
                       */
  float    speed;     /**< Indicates speed of walking / jogging calculated
                       *   from the input acceleration data.
                       *   The unit is [m/s].
                       */
  float    distance;  /**< Indicates cumulative travel distance calculated
                       *   from the input acceleration data.
                       *   The unit is [m].
                       */
  uint32_t step;      /**< Indicates the number of steps calculated
                       *   from the input acceleration data.
                       * The unit is [step].
                       */
  StepCounterMovementType  movement_type; /**<
                                    * Indicates the walking type calculated
                                    * from the input acceleration data.
                                    */
  uint64_t          time_stamp;    /**< Indicates latest timestamp of the
                                    *   acceleration sensor data used.
                                    */
} StepCounterStepInfo;

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /*  __INCLUDE_SENSING_STEP_COUNTER_COMMAND_H */

