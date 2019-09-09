/****************************************************************************
 * modules/include/sensing/logical_sensor/transport_mode_command.h
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

#ifndef __INCLUDE_SENSING_TRAM_COMMAD_H
#define __INCLUDE_SENSING_TRAM_COMMAD_H

/**
 * @defgroup logical_tramsport TRAnsport Mode rcognition API
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
 * @file transport_mode_command.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * Accelerometer sensor sampling frequency[Hz]
 */
#define TRAM_ACC_SAMPLING  64

/**
 * Magnetmeter sensor sampling frequency[Hz]
 */
#define TRAM_MAG_SAMPLING  8

/**
 * Barometer sensor sampling frequency[Hz]
 */
#define TRAM_BAR_SAMPLING  8

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * Command type
 */
typedef enum {
  TramCmdTypeResult = 0, /**< Result of command*/
  TramCmdTypeTrans,      /**< Notification of state transition */
  TramCmdTypeNum
} TramCmdType;
    
/**
 * TRAM state
 */
typedef enum {
  TramStateMs = 0, /**< MS state */
  TramStateCmd,    /**< CMD state */
  TramStateTmi,    /**< TMI state */
  TramStateNum
} TramState;


/**
 * Sensor type
 */
typedef enum {
  TramSensorAcc = 0 , /**< Accelerometer */
  TramSensorMag ,     /**< Magnetmeter */
  TramSensorBar ,     /**< Barometer */
} TramSensorType;

/**
 * Result of transportation mode inference
 */
enum
{
  TRAM_CLASS_UNDETERMINED = 0, /**< Undetermined */
  TRAM_CLASS_STAY,             /**< Staying */
  TRAM_CLASS_WALK,             /**< Walking */
  TRAM_CLASS_RUN,              /**< Running */
  TRAM_CLASS_UPSTAIRS,         /**< Ascending stairs */
  TRAM_CLASS_DOWNSTAIRS,       /**< Descending stairs */
  TRAM_CLASS_ESCUP,            /**< Going up on escalator */
  TRAM_CLASS_ESCDOWN,          /**< Going down on escalator */
  TRAM_CLASS_ELEVUP,           /**< Going up in elevator */
  TRAM_CLASS_ELEVDOWN,         /**< Going down in elevator */
  TRAM_CLASS_TRAIN,            /**< Getting on train */
  TRAM_CLASS_BUS,              /**< Getting on bus */
  TRAM_CLASS_CAR,              /**< Getting in car */
  TRAM_CLASS_BICYCLE,          /**< Riding bicycle */
};


/* --------------------------------------------------------------------------
 * Command Structures
 * --------------------------------------------------------------------------
 */
/**
 * Debug Dump Information
 */
struct TramDebugDumpInfo
{
  void   *addr;  /**< debug dump area address */
  size_t size;   /**< debug dump area size */
};

/*--------------------------------------------------------------------------*/
/**
 * Initialization command
 */
typedef struct
{

  FAR float *likelihood;
  TramDebugDumpInfo    debug_dump_info;  /**< debug dump information */
} SensorInitTram;

/*--------------------------------------------------------------------------*/
/**
 * Execution command
 */
typedef struct
{
  TramSensorType         type;  /**< Sensor type  */

  union
    {
      ThreeAxisSampleData  acc_data; /**< Accelerometer data */
      ThreeAxisSampleData  mag_data; /**< Magnetmeter data  */
      BarSampleData        bar_data; /**< Barometer data  */
  };
} SensorExecTram;

/*--------------------------------------------------------------------------*/
/**
 * Finalization command
 */

typedef struct {

} SensorFlushTram;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorResultTram
 * @brief the structure of sensor result on transport mode commands.
 */
typedef struct
{
  SensorExecResult exec_result;     /**< Execute resule.  */
  SensorAssertionInfo assert_info;  /**< Assert information. */
 
} SensorResultTram;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorCmdTram
 * @brief the structure of transport mode commands.
 */

typedef struct
{
  SensorCmdHeader header;  /**< Sensor command header. */

  union
    {
      SensorInitTram  init_cmd;  /**< Initialization command. */
      SensorExecTram  exec_cmd;  /**< Execution command.      */
      SensorFlushTram flush_cmd; /**< Termination command.    */
    };

  SensorResultTram result; /**< Result information. */
} SensorCmdTram;

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /*  __INCLUDE_SENSING_TRAM_COMMAD_H */
