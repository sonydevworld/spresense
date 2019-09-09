/****************************************************************************
 * modules/include/sensing/logical_sensor/sensor_assertion.h
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

#ifndef __INCLUDE_SENSING_SENSOR_ASSERTION_H
#define __INCLUDE_SENSING_SENSOR_ASSERTION_H

/**
 * @defgroup logical_sensor Logical sensors definition
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/**
 * @file sensor_assertion.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SENSOR_NOERROR                 0x00  /**< Success OK. */
#define SENSOR_ILLEGALSTATE            0x01  /**< State violation. */
#define SENSOR_DUMPINIT_ERROR          0x10  /**< Initialization error of dump function. */
#define SENSOR_INVALIDSENSORTYPE       0x11  /**< Sensor type value error. */
#define SENSOR_PROCESSMODE_ERROR       0x19  /**< Parameter process mode error */
#define SENSOR_INVALIDEVENTTYPE        0x1A  /**< Parameter event type error. */
#define SENSOR_QUEUEFULL_ERROR         0x90  /**< Queue is full. */
#define SENSOR_QUEUEEMPTY_ERROR        0x91  /**< Queue is empty. */
#define SENSOR_QUEUEPOP_ERROR          0x92  /**< Queue pop error. */
#define SENSOR_QUEUEPUSH_ERROR         0x93  /**< Queue push error. */
#define SENSOR_RESORCEBUSY_ERROR       0x94  /**< Resorce busy error. */
#define SENSOR_CPUFIFOSEND_ERROR       0x95  /**< CPU fifo send error. */
#define SENSOR_INVALIDARG              0xA0  /**< Illegal argument. */
#define SENSOR_INVALIDCOMMANDID        0xA1  /**< Invalid command ID. */
#define SENSOR_NOTUPDATE               0xA2  /**< No update. */
#define SENSOR_TIMESTAMPDISCONTINUOUS  0xA3  /**< Time stamps are not continuous. */
#define SENSOR_ASSERTIONFAIL           0xA4  /**< Abnormal occurrence. */

typedef unsigned char SensorErrorCode;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct SensorAssertionInfo
 * @brief Assert information structure
 */
typedef struct
{
  SensorErrorCode code;         /**< Error code */
  uint16_t        src_file_no;  /**< Serial number of the fault detection source file. */
  uint16_t        line;         /**< Line number of the fault detection source file. */
} SensorAssertionInfo;

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif/* __INCLUDE_SENSING_SENSOR_ASSERTION_H */
