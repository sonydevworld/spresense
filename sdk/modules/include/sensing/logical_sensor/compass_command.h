/****************************************************************************
 * modules/include/sensing/logical_sensor/compass_command.h
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

#ifndef __INCLUDE_SENSING_COMPASS_COMMAND_H
#define __INCLUDE_SENSING_COMPASS_COMMAND_H

/**
 * @defgroup logical_compass Compass API
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
 * @file compass_command.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * @def COMPASS_ACCEL_SAMPLING_MAX
 * @brief Acceleration sensor Indicates the maximum number of data samples.
 */
#define COMPASS_ACCEL_SAMPLING_MAX 16
/**
 * @def COMPASS_MAG_SAMPLING_MAX
 * @brief Magnetometer sensor Indicates the maximum number of data samples.
 */
#define COMPASS_MAG_SAMPLING_MAX   8

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @enum CompassCmdType
 * @brief Publishers
 */
typedef enum {
  COMPASS_CMD_UPDATE_ACCEL = 0 ,  /**<
                                   * Used to update acceleration sensor data.
                                   */
  COMPASS_CMD_UPDATE_MAG ,        /**<
                                   * Used to update magnetometer sensor data.
                                   */
  COMPASS_CMD_SET_SOFTIRON_PARAM, /**<
                                   * Used to adjust magnetometer sensor data.
                                   */
  COMPASS_CMD_SET_WALKSTATE,      /**<
                                   * Used to update magnetometer sensor state.
                                   */
  COMPASS_CMD_RESET_CALIB,        /**<
                                   * Used to reset calibration info of
                                   * magnetometer sensor.
                                   */
} CompassCmdType;

/**
 * @enum CompassWalkStateType
 * @brief Sensor state specified by COMPASS_CMD_SET_WALKSTATE
 */
typedef enum {
  COMPASS_WALK_STATE_DISABLED = -1, /**< Disable state information. */
  COMPASS_WALK_STATE_STOPPING,      /**< Stopping state setting.    */
  COMPASS_WALK_STATE_WALKING        /**< Walking state setting.     */
} CompassWalkStateType;

/*--------------------------------------------------------------------------
 * Command Structures
 *--------------------------------------------------------------------------
 */
/**
 * Debug Dump Information
 */
struct CompassDebugDumpInfo
{
  void   *addr;  /**< debug dump area address */
  size_t size;   /**< debug dump area size */
};

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorInitCompass
 * @brief the structure of Accelstep setting for initilaize.
 */
typedef struct {

  CompassDebugDumpInfo   debug_dump_info;  /**< debug dump information */

} SensorInitCompass;

/*--------------------------------------------------------------------------*/
/**
 * @struct CompassSetSoftIron
 * @brief Adjust parameter of magnetometer sensor.
 */
typedef struct {
  float tilt_yaw;       /**< Ellipse yaw tilt[deg]                  */
  float tilt_pitch;     /**< Ellipse pitch tilt[deg]                */
  float rot_dist_yaw;   /**< Rotational distortion yaw tilt[deg]    */
  float rot_dist_pitch; /**< Rotational distortion pitch tilt[deg]  */
  float gain_x;         /**< gainX[deg]                             */
  float gain_y;         /**< gainY[deg]                             */
  float gain_z;         /**< gainZ[deg]                             */
} CompassSetSoftIron;

/**
 * @struct SensorExecCompass
 * @brief the command of Compass execute by a frame(a few sample).
 */
typedef struct {

  CompassCmdType cmd_type; /**< Indicates the Compass command type. */

  union {
    ThreeAxisSampleData  update_three_axis;  /**< Frame of 3 axis data. */
    CompassSetSoftIron   soft_iron;          /**<
                                              * Adjust parameter of
                                              * magnetometer sensor.
                                              */
    CompassWalkStateType walk_state; /**< Used to update sensor status. */
  };

} SensorExecCompass;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorFlushCompass
 * @brief the command of compass terminate.
 */
typedef struct {

} SensorFlushCompass;

/*--------------------------------------------------------------------------*/
/**
 * @struct CompassData
 * @brief the structure of Compass results.
 *        the result is returned only when COMPASS_CMD_UPDATE_MAG is
 *        specified as the command type.
 */
typedef struct{

  float azimuth;     /**<
                      * Indicates calculation of azimuth from input sensor data.
                      * The unit is[rad].
                      * (*)No deviation correction.
                      */
  float pitch;       /**<
                      * Indicates calculation of pitch attitude from input sensor data.
                      * The unit is[rad].
                      */
  float roll;        /**<
                      * Indicates calculation of roll attitude from input sensor data.
                      * The unit is[rad].
                      */
  float decl;        /**<
                      * Argument angle.
                      * The unit is[rad].
                      * (*)Use Azimuth - Decl
                      */
  int   acc_azimuth; /**<
                      * Return the azimuth accuracy level. [0 - 3]
                      */
  int   calib_lv;    /**<
                      * Return the calibration accuracy level. [0 - 3]
                      */

} CompassData;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorResultCompass
 * @brief the structure of sensor result on compass commands.
 */
typedef struct
{
  SensorExecResult exec_result; /**< Execute resule.  */

  union
    {
      CompassData         compass;      /**< Acquired azimuth angle.     */
      SensorAssertionInfo assert_info;  /**< Assert information. */
    };
} SensorResultCompass;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorCmdCompass
 * @brief the structure of compass commands.
 */

typedef struct
{
  SensorCmdHeader header;  /**< Sensor command header. */

  union
    {
      SensorInitCompass  init_cmd;  /**< Initialization command. */
      SensorExecCompass  exec_cmd;  /**< Execution command.      */
      SensorFlushCompass flush_cmd; /**< Termination command.    */
    };

  SensorResultCompass result; /**< Result information. */
} SensorCmdCompass;

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /*  __INCLUDE_SENSING_COMPASS_COMMAND_H */

