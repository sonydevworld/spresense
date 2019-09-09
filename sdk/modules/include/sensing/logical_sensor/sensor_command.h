/****************************************************************************
 * modules/include/sensing/logical_sensor/sensor_command.h
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

#ifndef __INCLUDE_SENSING_SENSOR_COMMAND_H
#define __INCLUDE_SENSING_SENSOR_COMMAND_H

/**
 * @defgroup logical_sensor Logical sensors definition
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/**
 * @file sensor_command.h
 */

#include "sensing/logical_sensor/sensor_assertion.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @enum SensorEventType
 * @brief codes of command events.
 */
typedef enum
{
  InvalidSensorEvent = 0xFF,              /**< Illegal event type.   */
  InitEvent = 0,                          /**< Initialization event. */
  ExecEvent,                              /**< Execution event.      */
  FlushEvent,                             /**< Terminal event.       */
  SensorEventTypeNum                      /**< Number of events.     */
} SensorEventType;

/*--------------------------------------------------------------------------*/
/**
 * @enum SensorProcessMode
 * @brief codes of command logical sensor modes.
 */
typedef enum
{
  InvalidSensorProcessMode = 0xFF,        /**< Illegal process mode. */
  CommonMode = 0,                         /**< Common mode.          */
  StepCounterMode,                        /**< StepCounter mode.     */
  GestureProcMode,                        /**< Gesture mode.         */
  CompassProcMode,                        /**< Compass mode.         */
  TramProcMode,                           /**< Tram mode.            */
  TramliteProcMode                        /**< Tramlite mode.        */
} SensorProcessMode;

/*--------------------------------------------------------------------------*/
/**
 * @enum SensorType
 * @brief codes of command logical sensor types.
 */
typedef enum
{
  InvalidSensorType = 0xFF,               /**< Illegal sensor type.      */
  StepCounter = 0,                        /**< Step counter sensor type. */
  ArmGesture,                             /**< Arm Gesture sensor type.  */
  Compass,                                /**< Compass sensor type.      */
  TransportationMode,                     /**< Tram sensor type.         */
  TransportationModeLite                  /**< Tramlite sensor type.     */
} SensorType;

/*--------------------------------------------------------------------------*/
/**
 * @enum SensorExecResult
 * @brief results.
 */
typedef enum {
  SensorOK,          /**< Command execution succeeded.                    */
  SensorError,       /**< Command execution failed.                       */
  SensorWarning,     /**< Warning is issued when the command is executed. */
  SensorResultData   /**< Result Event from logical sensor.               */
} SensorExecResult;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorCmdHeader
 * @brief the header of sensor commands.
 */
typedef struct
{
  uint8_t    context_id;
  uint8_t    event_type;    /**<
                             * Indicates the event type of the command.
                             * See SensorEventType in sensor_cmd_defs.h.
                             */
  uint8_t    sensor_type;   /**<
                             * Indicates the sensor type of the command.
                             * See SensorType in sensor_cmd_defs.h.
                             */
  uint8_t    reserved;
} SensorCmdHeader;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

inline bool is_async_msg(uint32_t data)
{
  return ((data & 0x80000000) != 0) ? true : false;
}

inline uint8_t get_async_msgtype(uint32_t param)
{
  return static_cast<uint8_t>((param & 0x0000FF00) >> 8);
}

inline uint8_t get_async_msgparam(uint32_t param)
{
  return static_cast<uint8_t>(param & 0x000000FF);
}

inline uint32_t make_async_msg(uint8_t type, uint8_t param)
{
  return (type << 8) | param;
}

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /* __INCLUDE_SENSING_SENSOR_COMMAND_H */
