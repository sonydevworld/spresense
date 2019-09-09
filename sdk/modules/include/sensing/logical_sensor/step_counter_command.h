/****************************************************************************
 * modules/include/sensing/logical_sensor/step_counter_command.h
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
 * @defgroup logical_stepcounter Step Counter API 
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

#include "sensing/logical_sensor/sensor_command.h"
#include "sensing/logical_sensor/physical_command.h"

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
 * @enum StepCounterStepMode
 * @brief Step modes
 */
typedef enum
{
  STEP_COUNTER_MODE_FIXED_LENGTH = 0, /**<
                                       * Using fixed length
                                       * (not using existing stride table).
                                       */
  STEP_COUNTER_MODE_NEW_TABLE,        /**<
                                       * Create a new stride table
                                       * with stepLength, and use the table.
                                       * (not supported)
                                       */
  STEP_COUNTER_MODE_STEP_TABLE        /**<
                                       * Using existing stride table
                                       * (not using fixed length).
                                       * (not supported)
                                       */
} StepCounterStepMode;

/*--------------------------------------------------------------------------*/
/**
 * @struct StepCounterSetParam
 * @brief the structure of step setting for initialize.
 */
struct step_counter_param_s
{
  int32_t              step_length; /**<
                                     * Step stride setting value.
                                     * Min 1[cm], Max 249[cm].
                                     */
  StepCounterStepMode  step_mode;   /**< Setting of using the stride table. */
};

typedef struct step_counter_param_s StepCounterSetParam;

/*--------------------------------------------------------------------------*/
/**
 * @struct StepCounterSetting
 * @brief the structure of Accelstep user setting.
 */
struct step_counter_setting_s
{
  StepCounterSetParam  walking;      /**< User Setting for walking. */
  StepCounterSetParam  running;      /**< User Setting for running. */
};

typedef struct step_counter_setting_s StepCounterSetting;

/*--------------------------------------------------------------------------*/
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

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorResultStepCounter
 * @brief the structure of sensor result on step counter commands.
 */
typedef struct
{
  SensorExecResult exec_result; /**< Execute resule.  */

  union
    {
      StepCounterStepInfo steps;        /**< Step count.         */
      SensorAssertionInfo assert_info;  /**< Assert information. */
    };
} SensorResultStepCounter;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorCmdStepCounter
 * @brief the structure of step counter commands.
 */

typedef struct
{
  SensorCmdHeader header;  /**< Sensor command header. */

  union
    {
      SensorInitStepCounter  init_cmd;  /**< Initialization command. */
      SensorExecStepCounter  exec_cmd;  /**< Execution command.      */
      SensorFlushStepCounter flush_cmd; /**< Termination command.    */
    };

  SensorResultStepCounter result; /**< Result information. */
} SensorCmdStepCounter;

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /*  __INCLUDE_SENSING_STEP_COUNTER_COMMAND_H */

