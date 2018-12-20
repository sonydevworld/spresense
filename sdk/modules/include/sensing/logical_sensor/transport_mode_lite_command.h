/****************************************************************************
 * modules/include/sensing/tramlite_command.h
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

#ifndef __INCLUDE_SENSING_TRAMLITE_COMMAD_H
#define __INCLUDE_SENSING_TRAMLITE_COMMAD_H

/**
 * @defgroup logical_tramlite TRAMLITE API
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
 * @file tramlite_command.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * Accelerometer sensor sampling frequency
 */
#define TRAMLITE_ACC_SAMPLING  64

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * Command type
 */
typedef enum {
  TramliteCmdTypeResult = 0, /**< Result of command*/
  TramliteCmdTypeTrans,      /**< Notification of state transition */
  TramliteCmdTypeNum
} TramliteCmdType;
    
/**
 * TRAMLITE state
 */
typedef enum {
  TramliteStateMs = 0, /**< MS state */
  TramliteStateCmd,    /**< CMD state */
  TramliteStateTmi,    /**< TMI state */
  TramliteStateNum
} TramliteState;


/**
 * Sensor type
 */
typedef enum {
  TramliteSensorAcc = 0 ,  /**< Accelerometer */
} TramliteSensorType;

/**
 * Result of transportation mode inference
 */
enum
{
  TRAMLITE_CLASS_UNDETERMINED = 0, /**< Undetermined */
  TRAMLITE_CLASS_STAY,             /**< Staying */
  TRAMLITE_CLASS_WALK,             /**< Walking */
  TRAMLITE_CLASS_RUN,              /**< Running */
  TRAMLITE_CLASS_VEHICLE,          /**< Getting on vehicle */
  TRAMLITE_CLASS_BICYCLE,          /**< Riding bicycle */
};
  
/* --------------------------------------------------------------------------
 * Command Structures
 * --------------------------------------------------------------------------
 */
/**
 * Debug Dump Information
 */
struct TramliteDebugDumpInfo
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
  TramliteDebugDumpInfo debug_dump_info;  /**< debug dump information */

} SensorInitTramlite;


/*--------------------------------------------------------------------------*/
/**
 * Execution command
 */
typedef struct
{
  TramliteSensorType         type; /**< Sensor type  */

  union {
    ThreeAxisSampleData  acc_data; /**< Accelerometer data */
  };
} SensorExecTramlite;

/*--------------------------------------------------------------------------*/
/**
 * Finalization command
 */
typedef struct {

} SensorFlushTramlite;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorResultTramlite
 * @brief the structure of sensor result on transport mode(lite) commands.
 */
typedef struct
{
  SensorExecResult exec_result;     /**< Execute resule.  */
  SensorAssertionInfo assert_info;  /**< Assert information. */
 
} SensorResultTramlite;

/*--------------------------------------------------------------------------*/
/**
 * @struct SensorCmdTramlite
 * @brief the structure of transport mode(lite) commands.
 */

typedef struct
{
  SensorCmdHeader header;  /**< Sensor command header. */

  union
    {
      SensorInitTramlite  init_cmd;  /**< Initialization command. */
      SensorExecTramlite  exec_cmd;  /**< Execution command.      */
      SensorFlushTramlite flush_cmd; /**< Termination command.    */
    };

  SensorResultTramlite result; /**< Result information. */
} SensorCmdTramlite;

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /*  __INCLUDE_SENSING_TRAMLITE_COMMAD_H */

