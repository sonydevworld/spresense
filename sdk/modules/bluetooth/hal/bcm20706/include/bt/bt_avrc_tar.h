/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/manager/bt_storage_manager.h
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
 * @file      bt_avrc_tar.h
 */

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_AVRC_TAR_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_AVRC_TAR_H

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
 * @name AVRC Con Group Event Opcode Type
 * @{
 */
#define BT_EVT_AVRC_TARGET_CONNECT            0x01    /**< AVRC target connected event */
#define BT_EVT_AVRC_TARGET_DISCONNECT         0x02    /**< AVRC target disconnected event */
#define BT_EVT_AVRC_TARGET_PLAY               0x03    /**< Play command received event */
#define BT_EVT_AVRC_TARGET_STOP               0x04    /**< Stop command received event */
#define BT_EVT_AVRC_TARGET_PAUSE              0x05    /**< Pause command received event */
#define BT_EVT_AVRC_TARGET_NEXT_TRACK         0x06    /**< Next track command received event */
#define BT_EVT_AVRC_TARGET_PREVIOUS_TRACK     0x07    /**< Previous command received event */
#define BT_EVT_AVRC_TARGET_BEGIN_FAST_FORWARD 0x08    /**< Begin fast forward command received event */
#define BT_EVT_AVRC_TARGET_END_FAST_FORWARD   0x09    /**< End fast forward command received event */
#define BT_EVT_AVRC_TARGET_BEGIN_REWIND       0x0A    /**< Begin rewind command received event */
#define BT_EVT_AVRC_TARGET_END_REWIND         0x0B    /**< End rewind command received event */
#define BT_EVT_AVRC_TARGET_VOLUME_LEVEL       0x0C    /**< volume level received event */
/** @} */

/** @} bt_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/**@brief BT avrc play status type
 */
typedef enum avrcTargetPlayStatus
{
  BT_AVRC_TARGET_PLAY_STATUS_STOPPED = 0,
  BT_AVRC_TARGET_PLAY_STATUS_PLAYING,
  BT_AVRC_TARGET_PLAY_STATUS_PAUSED,
  BT_AVRC_TARGET_PLAY_STATUS_FORWARD_SEEK,
  BT_AVRC_TARGET_PLAY_STATUS_REVERSE_SEEK,
} BT_AVRC_TARGET_PLAY_STATUS;

/**@brief BT avrc play position type
*/
typedef struct avrcTargetVolumeLevel
{
  BT_ADDR addr;
  uint8_t volumeLevel;
} BT_AVRC_TARGET_VOLUME_LEVEL;

/**@brief avrc target callback function
 */
typedef int (*avrcTargetEvtCallBack)(BT_SESSION_EVT *);

/** @} bt_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup bt_funcs Functions
 * @{
 */

/**@brief   Create avrc target connection to remote device
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
int BT_AvrcTargetConnect(BT_ADDR *addr);

/**@brief   distroy avrc target connecion to remote device
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
int BT_AvrcTargetDisconnect(BT_ADDR *addr);

/**@brief  avrcp target set  volume up
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
int BT_AvrcTargetVolumUp(BT_ADDR *addr);

/**@brief  avrcp target set volume down
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
int BT_AvrcTargetVolumDown(BT_ADDR *addr);

/**@brief  avrcp target set  volume level
 *
 * @param[in]  addr: remote device address
 * @param[in]  volume: new volume level
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
int BT_AvrcTargetSetVolume(BT_ADDR *addr, uint8_t volume);

/**@brief send the player play state and track position information
 *
 * @param[in]  addr: remote device
 * @param[in]  status: play state.
 * @param[in]  len: length of the current track in milliseconds.
 * @param[in]  position: position in the current track in ms within length defined above.
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
int BT_AvrcTargetSendPlayerStatus(BT_ADDR *addr, BT_AVRC_TARGET_PLAY_STATUS status, uint32_t len, uint32_t position);

/**@brief   set avrc target event callback
 *
 * @param[in]  addr: remote device
 * @param[in]  cb: avrc_tar rx callback
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
int BT_AvrcTargetSetEvtCallBack(BT_ADDR *addr, avrcTargetEvtCallBack cb);

/** @} bt_funcs */

/** @} bt */

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_AVRC_TAR_H */
