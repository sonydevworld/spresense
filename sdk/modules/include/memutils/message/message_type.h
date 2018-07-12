/****************************************************************************
 * modules/include/memutils/message/message_type.h
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

#ifndef __APPS_MEMUTILS_MESSAGE_INCLUDE_MESSAGE_TYPE_H
#define __APPS_MEMUTILS_MESSAGE_INCLUDE_MESSAGE_TYPE_H

/***********************************************************************
 *
 *  [type]
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ| MSG_USER  | MSG_CATEGORY  | MSG_SUB_TYPE                  |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 *    D15: MSG_TYPE_REQUEST
 *         0: response
 *         1: request
 *         +-----------+ request message  +---------+
 *         | message   |----------------->| message |
 *         | initiator |<-----------------|  target |
 *         +-----------+ response message +---------+
 *
 *    D14-D12: MSG_USER
 *         0-5: (reserved)
 *         6: Audio Sub System
 *         7: Sensor Sub System
 *
 *    D11-D8: MSG_CATEGORY
 *         0-15: (reserved)  defined by each user
 *
 *    D7-D0: MSG_SUB_TYPE
 *         0-255: (reserved)  defined by each category
 *
 ***********************************************************************
 */
typedef unsigned short MSG_TYPE;

/************************************************************************
 *
 *  request / response
 *
 ************************************************************************
 */
 
#define MSG_TYPE_REQUEST_FIELD_SIZE    (0x1)
#define MSG_TYPE_REQUEST_FIELD_OFFSET  (15)

#define MSG_TYPE_RESPONSE (0x0 << MSG_TYPE_REQUEST_FIELD_OFFSET)
#define MSG_TYPE_REQUEST  (0x1 << MSG_TYPE_REQUEST_FIELD_OFFSET)

/************************************************************************
 *
 *  user
 *
 ************************************************************************
 */

#define MSG_TYPE_USER_FIELD_SIZE    (0x7)
#define MSG_TYPE_USER_FIELD_OFFSET  (12)

#define MSG_TYPE_USER          (MSG_TYPE_USER_FIELD_SIZE << MSG_TYPE_USER_FIELD_OFFSET)

#define MSG_TYPE_USER_AUDIO_UTIL   (0x6 << MSG_TYPE_USER_FIELD_OFFSET)
#define MSG_TYPE_USER_SENSOR_UTIL  (0x7 << MSG_TYPE_USER_FIELD_OFFSET)

/************************************************************************
 *
 *  category
 *
 ************************************************************************
 */

#define MSG_TYPE_CATEGORY_FIELD_SIZE    (0xf)
#define MSG_TYPE_CATEGORY_FIELD_OFFSET  (8)

#define MSG_TYPE_CATEGORY  (MSG_TYPE_CATEGORY_FIELD_SIZE << MSG_TYPE_CATEGORY_FIELD_OFFSET)

/************************************************************************
 *
 *  sub type
 *
 ************************************************************************
 */

#define MSG_TYPE_SUBTYPE_FIELD_SIZE     (0xff)
#define MSG_TYPE_SUBTYPE_FIELD_OFFSET   (0)

#define MSG_TYPE_SUBTYPE  (MSG_TYPE_SUBTYPE_FIELD_SIZE << MSG_TYPE_SUBTYPE_FIELD_OFFSET)

/************************************************************************
 *
 *  MACRO
 *
 ************************************************************************
 */
#define MSG_SET_CATEGORY(x)  (((x) & MSG_TYPE_CATEGORY_FIELD_SIZE) << MSG_TYPE_CATEGORY_FIELD_OFFSET)
#define MSG_SET_SUBTYPE(x)   (((x) & MSG_TYPE_SUBTYPE_FIELD_SIZE ) << MSG_TYPE_SUBTYPE_FIELD_OFFSET)
 
#define MSG_GET_USER(x)      (MSG_TYPE_USER     & (MSG_TYPE)x)
#define MSG_GET_CATEGORY(x)  (MSG_TYPE_CATEGORY & (MSG_TYPE)x)
#define MSG_GET_SUBTYPE(x)   (MSG_TYPE_SUBTYPE  & (MSG_TYPE)x)
#define MSG_IS_REQUEST(x)    (MSG_TYPE_REQUEST  & (MSG_TYPE)x)

#endif /* __APPS_MEMUTILS_MESSAGE_INCLUDE_MESSAGE_TYPE_H */
