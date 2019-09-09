/****************************************************************************
 * modules/include/sensing/sensor_ecode.h
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

#ifndef __INCLUDE_SENSING_SENSOR_ECODE_H
#define __INCLUDE_SENSING_SENSOR_ECODE_H

/* Error codes from sensorutils */

/** @defgroup error_code Sensor Manager error code */
/** @{ */

/*! \brief Response OK */

#define SS_ECODE_OK 0

/*! \brief DSP Load Error
 *  \details Tried to load DSP binary to sub core but it was failed.\n
 *           DSP binary may not be stored on dsp load path.
 */

#define SS_ECODE_DSP_LOAD_ERROR 1

/*! \brief DSP Boot Error
 *  \details Tried to boot DSP but it was failed.\n
 */

#define SS_ECODE_DSP_BOOT_ERROR 2

/*! \brief DSP Version Error
 *  \details Loaded DSP binary version is differ from expected.
 */

#define SS_ECODE_DSP_VERSION_ERROR 3

/*! \brief DSP initialization error
 *  \details DSP initialization failed.\n
 */

#define SS_ECODE_DSP_INIT_ERROR 4

/*! \brief DSP Exec Error
 *  \details The data or command which sent to DSP may not be correct format.
 */

#define SS_ECODE_DSP_EXEC_ERROR 5

/*! \brief DSP Unload Error
 *  \details Tried to unload DSP binary from sub core but it was failed.\n
 *           DSP binary may not loaded.
 */

#define SS_ECODE_DSP_UNLOAD_ERROR 5

/*! \brief Memory Handle Alloc Error
 *  \details All Memory handles may be used.\n
 *           Response from DSP may be delayed or get stacked up.
 */

#define SS_ECODE_MEMHANDLE_ALLOC_ERROR 6

/*! \brief Memory Handle Free Error
 *  \details Specified handle is already freed.
 */

#define SS_ECODE_MEMHANDLE_FREE_ERROR 7

/*! \brief Internal Queue Push Error
 *  \details The internal process queue is already full but tried to insertion.
 */

#define SS_ECODE_QUEUE_PUSH_ERROR 8

/*! \brief Internal Queue Pop Error
 *  \details The internal process queue is already empty but tried to extraction.
 */

#define SS_ECODE_QUEUE_POP_ERROR 9

/*! \brief Internal Queue Missing Error
 *  \details The queue became empty unexpectedly.
 */

#define SS_ECODE_QUEUE_MISSING_ERROR 10

/*! \brief Request is invalid
 *  \details The request is invalid because the sensor is not registered.
 */

#define SS_ECODE_REQUIRED_SENSOR_NOT_ACTIVE 11

/*! \brief Release error
 *  \details Unable to release because the sensor to subscribe is active.
 */

#define SS_ECODE_REQUIRED_SENSOR_STILL_ACTIVE 12

/*! \brief No destination
 *  \details There is no publish destination.
 */

#define SS_ECODE_NOTIFICATION_DST_UNDEFINED 13

/*! \brief Task Create Error
 *  \details Task context couldn't be created.\n
 *           Revise max task creation number on menu config.
 */

#define SS_ECODE_TASK_CREATE_ERROR 14

/*! \brief Unexpected Parameter
 *  \details The command parameter is wrong. Please confirm the command parameters.
 */

#define SS_ECODE_PARAM_ERROR 15

/*! \brief Internal State Error
 *  \details Internal State Error.
 */

#define SS_ECODE_STATE_ERROR 16

/** @} */

#endif /* __INCLUDE_SENSING_SENSOR_ECODE_H */
