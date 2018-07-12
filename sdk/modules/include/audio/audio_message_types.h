/****************************************************************************
 * modules/include/audio/audio_message_types.h
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

#ifndef __MODULES_INCLUDE_AUDIO_AUDIO_MESSAGE_TYPE_H
#define __MODULES_INCLUDE_AUDIO_AUDIO_MESSAGE_TYPE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/message/Message.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/************************************************************************
 *
 *  request / response
 *
 ************************************************************************
 */

#define MSG_TYPE_AUD_RES  (MSG_TYPE_RESPONSE | MSG_TYPE_USER_AUDIO_UTIL)
#define MSG_TYPE_AUD_REQ  (MSG_TYPE_REQUEST  | MSG_TYPE_USER_AUDIO_UTIL)

/************************************************************************
 *
 *  category
 *
 ************************************************************************
 */

#define MSG_CAT_AUD_MNG           (MSG_SET_CATEGORY(0x0))
#define MSG_CAT_AUD_ISR           (MSG_SET_CATEGORY(0x1))
#define MSG_CAT_AUD_PLY           (MSG_SET_CATEGORY(0x2))
#define MSG_CAT_AUD_BB            (MSG_SET_CATEGORY(0x3))
#define MSG_CAT_AUD_RCG           (MSG_SET_CATEGORY(0x4))
#define MSG_CAT_AUD_VRC           (MSG_SET_CATEGORY(0x5))
#define MSG_CAT_AUD_SNK           (MSG_SET_CATEGORY(0x6))
#define MSG_CAT_AUD_MIX           (MSG_SET_CATEGORY(0x7))
#define MSG_CAT_AUD_MIX_SEF       (MSG_SET_CATEGORY(0x8))
#define MSG_CAT_AUD_SEF           (MSG_SET_CATEGORY(0x9))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_INCLUDE_AUDIO_AUDIO_MESSAGE_TYPE_H */
