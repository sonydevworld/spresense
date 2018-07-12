/****************************************************************************
 * modules/include/sensing/sensor_message_types.h
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

#ifndef __INCLUDE_SENSING_SENSOR_MESSAGE_TYPES_H
#define __INCLUDE_SENSING_SENSOR_MESSAGE_TYPES_H

#include "memutils/message/Message.h"

/************************************************************************
 *
 *  request / response
 *
 ************************************************************************
 */

#define MSG_TYPE_SEN_RES  (MSG_TYPE_RESPONSE | MSG_TYPE_USER_SENSOR_UTIL)
#define MSG_TYPE_SEN_REQ  (MSG_TYPE_REQUEST  | MSG_TYPE_USER_SENSOR_UTIL)

/************************************************************************
 *
 *  category
 *
 ************************************************************************
 */

#define MSG_CAT_SEN_MNG           (MSG_SET_CATEGORY(0x0))

/************************************************************************
 *
 *    MSG_CAT_SEN_MNG: Sensor Manager Command(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+----+---+---+---+---+---+---+---+
 *  |REQ|USER_SENSOR| MSG_CAT_SEN_MNG|   MSG_SUB_TYPE            |
 *  +---+---+---+---+---+---+---+----+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_SENSOR_MNG_REQ    (MSG_TYPE_SEN_REQ | MSG_CAT_SEN_MNG)
#define MSG_SENSOR_MNG_RES    (MSG_TYPE_SEN_RES | MSG_CAT_SEN_MNG)

#define MSG_SENSOR_MGR_CMD_REGISTER_CLIENT  (MSG_SENSOR_MNG_REQ | MSG_SET_SUBTYPE(0x00))
#define MSG_SENSOR_MGR_CMD_RELEASE_CLIENT   (MSG_SENSOR_MNG_REQ | MSG_SET_SUBTYPE(0x01))
#define MSG_SENSOR_MGR_CMD_CHG_SUBSCRIPTION (MSG_SENSOR_MNG_REQ | MSG_SET_SUBTYPE(0x02))
#define MSG_SENSOR_MGR_CMD_SET_POWER        (MSG_SENSOR_MNG_REQ | MSG_SET_SUBTYPE(0x03))
#define MSG_SENSOR_MGR_CMD_CLEAR_POWER      (MSG_SENSOR_MNG_REQ | MSG_SET_SUBTYPE(0x04))
#define MSG_SENSOR_MGR_CMD_SEND_DATA        (MSG_SENSOR_MNG_REQ | MSG_SET_SUBTYPE(0x05))
#define MSG_SENSOR_MGR_CMD_SEND_DATA_MH     (MSG_SENSOR_MNG_REQ | MSG_SET_SUBTYPE(0x06))
#define MSG_SENSOR_MGR_CMD_SEND_RESULT      (MSG_SENSOR_MNG_REQ | MSG_SET_SUBTYPE(0x07))
#define MSG_SENSOR_MGR_CMD_INVALID          (MSG_SENSOR_MNG_REQ | MSG_SET_SUBTYPE(0x08))

#define LAST_SENSOR_MNG_MSG                 (MSG_SENSOR_MGR_CMD_INVALID + 1)
#define SENSOR_MNG_MSG_NUM                  (LAST_SENSOR_MNG_MSG & MSG_TYPE_SUBTYPE)

#define MSG_SENSOR_MGR_RST                  (MSG_SENSOR_MNG_RES | MSG_SET_SUBTYPE(0x00))


#endif /* __INCLUDE_SENSING_SENSOR_MESSAGE_TYPES_H */
