/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/bt/bt_a2dp_src.h
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

/**
 * @file    bt_a2dp.h
 */


#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_A2DP_SRC_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_A2DP_SRC_H

/**
 * @defgroup BT
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <bt/bt_comm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * @defgroup bt_defs Defines
 * @{
 */

/**
 *@name A2dp audio codec type
 *@{
 */
#define BT_AUDIO_CODEC_TYPE_SBC                0x00
#define BT_AUDIO_CODEC_TYPE_AAC                0x01
#define BT_AUDIO_CODEC_TYPE_LDAC               0x02
/** @} */

/**
 * @name A2DP Group Event Opcode Type
 * @{
 */
#define BT_EVT_A2DP_SRC_STATUS             0x01    /**< Command status event for the requested operation */
#define BT_EVT_A2DP_SRC_CONNECT            0x02    /**< Audio connection opened */
#define BT_EVT_A2DP_SRC_SERVICE_NOT_FOUND  0x03    /**< SDP record with audio service not found */
#define BT_EVT_A2DP_SRC_CONNECT_FAIL       0x04    /**< Connection attempt failed  */
#define BT_EVT_A2DP_SRC_DISCONNECT         0x05    /**< Audio connection closed */
#define BT_EVT_A2DP_SRC_DATA_REQUEST       0x06    /**< Audio data request*/
#define BT_EVT_A2DP_SRC_STARTED            0x07    /**< Command for audio start succeeded */
#define BT_EVT_A2DP_SRC_STOPPED            0x08    /**< Command for audio stop completed */
/** @} */

/** @} bt_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/**@brief BT a2dp data request sructure
 */
typedef struct
{
  uint16_t bytePerPacket;
  uint8_t  numOfPackets;
  uint16_t numOfReq;
  uint16_t numOfRecv;
} AUDIO_DATA_REQ;

/**@brief a2dp src callback function
 */
typedef int (*a2dpSrcEvtCallBack)(BT_SESSION_EVT *);

/** @} bt_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup bt_funcs Functions
 * @{
 */

/**@brief   Create a2dp src connection to remote device
 *
 * @param[in]  addr: remote device address
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find nearby device
 * @retval     -ENOMEM: no more memory
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_A2dpSrcConnect(BT_ADDR *addr);

/**@brief   distroy a2dp src connecion to remote device
 *
 * @param[in]  addr: remote device address
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find connected device
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_A2dpSrcDisconnect(BT_ADDR *addr);

/**@brief   stop a2dp src play
 *
 * @param[in]  addr: remote device address
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find connected device
 * @retval     -EINVAL: a2dp stoped
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_A2dpSrcStopPlay(BT_ADDR *addr);

/**@brief   set a2dp src event callback
 *
 * @param[in]  addr: remote device
 * @param[in]  cb: a2dp rx callback
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find connected device
 * @retval     -EINVAL: invalid argument
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_A2dpSrcSetEvtCallBack(BT_ADDR *addr, a2dpSrcEvtCallBack cb);


/**@brief   send a2dp src audio data
 *
 * @param[in]  addr: remote device
 * @param[in]  data: point to audio data to be send
 * @param[in]  len: length of audio data to be send
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find connected device
 * @retval     -EINVAL: paramter inval
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_A2dpSrcSendAudio(BT_ADDR *addr, uint8_t *data, uint16_t len);

/** @} bt_funcs */

/** @} BT */

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_A2DP_SRC_H */
