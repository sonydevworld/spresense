/****************************************************************************
 * modules/include/audio/audio_object_common_api.h
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

#ifndef __MODULES_INCLUDE_AUDIO_AUDIO_OBJECT_COMMON_API_H
#define __MODULES_INCLUDE_AUDIO_AUDIO_OBJECT_COMMON_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_object_api Audio Object Layer API
 * @{
 *
 * @file       audio_object_common_api.h
 * @brief      CXD5602 Audio Object Layer API
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "memutils/message/MsgPacket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

enum AudioObjReplyType_e
{
  AS_OBJ_REPLY_TYPE_REQ = 0,
  AS_OBJ_REPLY_TYPE_EVT,
  AS_OBJ_REPLY_TYPE_FREE,
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct AudioObjReply
{
  uint32_t            id;         /**< command id */
  AudioObjReplyType_e type;       /**< id type */
  uint8_t             module_id;  /**< module id of Objects */
  uint32_t            result;     /**< result code */

  AudioObjReply():
    id(0xFFFFFFFF),
    type(AS_OBJ_REPLY_TYPE_FREE),
    module_id(0xFF),
    result(0xFFFFFFFF)
  {}

  AudioObjReply(uint32_t arg_id,
                AudioObjReplyType_e arg_type,
                uint8_t  arg_module_id,
                uint32_t arg_result):
    id(arg_id),
    type(arg_type),
    module_id(arg_module_id),
    result(arg_result)
  {}
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Wait until an asynchronous event is received from the object layer
 *
 * @param[in]  msgq_id : ID of message queue to receive
 * @param[out] reply   : information of replay message
 *
 * @retval     true  : success
 * @retval     false : failure
 */

template <typename T>
bool AS_ReceiveObjectReply(MsgQueId msgq_id,
                           T *reply);

/* Default template cannot use at C++99 */

bool AS_ReceiveObjectReply(MsgQueId msgq_id,
                           AudioObjReply *reply);

/**
 * @brief Wait for the specified time until an asynchronous event
 *        is received from the object layer
 *
 * @param[in]  msgq_id : ID of message queue to receive
 * @param[in]  ms      : Wait time(ms)
 * @param[out] reply   : information of replay message
 *
 * @retval     true  : success
 * @retval     false : failure
 */

template <typename T>
bool AS_ReceiveObjectReply(MsgQueId msgq_id,
                           uint32_t ms,
                           T *reply);

bool AS_ReceiveObjectReply(MsgQueId msgq_id,
                           uint32_t ms,
                           AudioObjReply *reply);

#endif  /* __MODULES_INCLUDE_AUDIO_AUDIO_OBJECT_COMMON_API_H */
/**
 * @}
 */

/**
 * @}
 */
