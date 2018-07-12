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

#define SS_ECODE_OK 0
#define SS_ECODE_DSP_LOAD_ERROR 1
#define SS_ECODE_DSP_BOOT_ERROR 2
#define SS_ECODE_DSP_VERSION_ERROR 3
#define SS_ECODE_DSP_INIT_ERROR 4
#define SS_ECODE_DSP_EXEC_ERROR 5
#define SS_ECODE_DSP_UNLOAD_ERROR 5
#define SS_ECODE_MEMHANDLE_ALLOC_ERROR 6
#define SS_ECODE_MEMHANDLE_FREE_ERROR 7
#define SS_ECODE_QUEUE_PUSH_ERROR 8
#define SS_ECODE_QUEUE_POP_ERROR 9
#define SS_ECODE_QUEUE_MISSING_ERROR 10
#define SS_ECODE_REQUIRED_SENSOR_NOT_ACTIVE 11
#define SS_ECODE_REQUIRED_SENSOR_STILL_ACTIVE 12
#define SS_ECODE_NOTIFICATION_DST_UNDEFINED 13
#define SS_ECODE_TASK_CREATE_ERROR 14
#define SS_ECODE_PARAM_ERROR 15
#define SS_ECODE_STATE_ERROR 16

#endif /* __INCLUDE_SENSING_SENSOR_ECODE_H */

