/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/bt/bt_avrc_con.h
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
 * @file    bt_avrc_con.h
 */


#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_AVRC_CON_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_AVRC_CON_H

/**
 * @defgroup BT
 * @{
 */

/*-----------------------------------------------------------------------------
 * include files
 *---------------------------------------------------------------------------*/

#include <bt/bt_comm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup bt_defs Defines
 * @{
 */

/**
 *@name AVRC ATTR
 *@{
 */
#define BT_AVRC_ATTR_TITLE        0x01
#define BT_AVRC_ATTR_ARTIST       0x02
#define BT_AVRC_ATTR_ALBUM        0x04
#define BT_AVRC_ATTR_TRACK_NUM    0x08
#define BT_AVRC_ATTR_NUM_TRACKS   0x10
#define BT_AVRC_ATTR_GENRE        0x20
#define BT_AVRC_ATTR_PLAYING_TIME 0x40
/** @} */

/**
 * @name AVRC Con Group Event Opcode Type
 * @{
 */
#define BT_EVT_AVRC_CONTROLLER_CONNECT           0x01    /**< AVRC controller connected event*/
#define BT_EVT_AVRC_CONTROLLER_DISCONNECT        0x02    /**< AVRC controller disconnected event*/
#define BT_EVT_AVRC_CONTROLLER_TRACK_INFO        0x03    /**< AVRC controller track information event*/
#define BT_EVT_AVRC_CONTROLLER_PLAY_STATUS       0x04    /**< AVRC controller play status change event*/
#define BT_EVT_AVRC_CONTROLLER_PLAY_POSITION     0x05    /**< AVRC controller play position change event*/
#define BT_EVT_AVRC_CONTROLLER_SETTING_CHANGE    0x0A    /**< AVRC controller player setting changed*/
#define BT_EVT_AVRC_CONTROLLER_VOLUME_LEVEL      0x20    /**< AVRC controller player volume level*/
#define BT_EVT_AVRC_CONTROLLER_VOLUME_UP         0x21    /**< AVRC controller player volume up*/
#define BT_EVT_AVRC_CONTROLLER_VOLUME_DOWN       0x22    /**< AVRC controller player volume down*/
#define BT_EVT_AVRC_CONTROLLER_CMD_STATUS        0xFF    /**< Results status for AVRC commands*/
/** @} */

/**
 * @name AVRC Repeat type
 * @{
 */
#define BT_AVRC_REPEAT_OFF          0x01
#define BT_AVRC_REPEAT_SINGLE_TRACK 0x02
#define BT_AVRC_REPEAT_ALL_TRACK    0x03
#define BT_AVRC_REPEAT_GROUP        0x04
/** @} */

/**
 * @name AVRC Shuffle type
 * @{
 */
#define BT_AVRC_SHUFFLE_OFF         0x01
#define BT_AVRC_SHUFFLE_ALL_TRACK   0x02
#define BT_AVRC_SHUFFLE_GROUP       0x03
/** @} */

/** @} bt_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/**@brief BT avrc cmd type
 */
typedef enum avrcCommandType
{
  BT_AVRC_CMD_PLAY = 0x01,
  BT_AVRC_CMD_STOP,
  BT_AVRC_CMD_PAUSE,
  BT_AVRC_CMD_BEGIN_FAST_FORWARD,
  BT_AVRC_CMD_END_FAST_FORWARD,
  BT_AVRC_CMD_BEGIN_REWIND,
  BT_AVRC_CMD_END_REWIND,
  BT_AVRC_CMD_NEXT_TRACK,
  BT_AVRC_CMD_PREVIOUS_TRACK,
  BT_AVRC_CMD_VOLUME_UP,
  BT_AVRC_CMD_VOLUME_DOWN,
  BT_AVRC_CMD_SET_REPEAT_MODE,
  BT_AVRC_CMD_SET_SHUFFLE_MODE,
} BT_AVRC_CMD;

/**@brief BT avrc play status type
 */
typedef enum PlayStatus
{
  BT_AVRC_PLAY_STATUS_STOPPED = 0,
  BT_AVRC_PLAY_STATUS_PLAYING,
  BT_AVRC_PLAY_STATUS_PAUSED,
  BT_AVRC_PLAY_STATUS_FORWARD_SEEK,
  BT_AVRC_PLAY_STATUS_REVERSE_SEEK,
  BT_AVRC_PLAY_STATUS_ERROR = 0xFF,
} BT_PLAY_STATUS;

/**@brief BT avrc play mode type
 */
typedef enum avrcSettingId
{
  BT_AVRC_REPEAT_MODE = 0x02,
  BT_AVRC_SHUFFLE_MODE,
} BT_AVRC_SETTING_ID;

/**@brief BT avrc volume key press state
 */
typedef enum avrcVoluemKeyState
{
  BT_AVRC_PRESS_STATE = 0,
  BT_AVRC_RELEASE_STATE,
} BT_AVRC_VOLUME_KEY_STATE;

/**@brief BT avrc track info type
*/
typedef struct avrcTrackInfo
{
  BT_ADDR addr;
  uint8_t attrId;
  uint16_t attrLen;
  uint8_t attrValue[48];
} BT_AVRC_TRACK_INFO;

/**@brief BT avrc play status type
*/
typedef struct avrcPlayStatus
{
  BT_ADDR addr;
  BT_PLAY_STATUS playStatus;
} BT_AVRC_PLAY_STATUS;

/**@brief BT avrc play position type
*/
typedef struct avrcPlayPosition
{
  BT_ADDR addr;
  uint32_t position;
} BT_AVRC_PLAY_POSITION;

/**@brief BT avrc setting type
*/
typedef struct avrcSettingChange
{
  BT_ADDR addr;
  BT_AVRC_SETTING_ID id;
  uint8_t value;
} BT_AVRC_SETTING_CHANGE;

/**@brief BT avrc volume level
*/
typedef struct avrcVolumeLevel
{
  BT_ADDR addr;
  uint8_t volumeLevel;
} BT_AVRC_VOLUME_LEVEL;

/**@brief BT avrc volume changed state
*/
typedef struct avrcVolumeChangedState
{
  BT_ADDR addr;
  BT_AVRC_VOLUME_KEY_STATE keyPressState;
} BT_AVRC_VOLUME_CHANGED_STATE;

/**@brief avrc controller callback function
 */
typedef int (*avrcControllerEvtCallBack)(BT_SESSION_EVT *);

/** @} bt_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup bt_funcs Functions
 * @{
 */


/**@brief   Create avrc controller connection to remote device
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
int BT_AvrcControllerConnect(BT_ADDR *addr);

/**@brief   distroy avrc controller connecion to remote device
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
int BT_AvrcControllerDisconnect(BT_ADDR *addr);

/**@brief  get track information
 *
 * @param[in]  addr: address of remote device.
 * @param[in]  attr: each byte represents an attribute to retrieve.
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
int BT_AvrcControllerGetTrackInfo(BT_ADDR *addr, uint8_t attr);

/**@brief   set avrc controller event callback
 *
 * @param[in]  addr: remote device
 * @param[in]  cb: avrc_con rx callback
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
int BT_AvrcControllerSetEvtCallBack(BT_ADDR *addr, avrcControllerEvtCallBack cb);

/**@brief  avrcp controller set  volume level
 *
 * @param[in]  addr: remote device address
 * @param[in]  volume: new volume level, range 0 ~ 100, 0 corresponds 0%, 100 corresponds 100%
 *                     return -EINVAL if out of range
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
int BT_AvrcControllerSetVolume(BT_ADDR *addr, uint8_t volume);

/**@brief  send command to the target player
 *
 * @param[in]  addr: remote device
 * @param[in]  cmd: command type
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
int BT_AvrcControllerSendCommand(BT_ADDR *addr, BT_AVRC_CMD cmd);

/**@brief  notify volume change to the target player
 *
 * @param[in]  addr: remote device, if addr == NULL, init the value of notify volume
 * @param[in]  volume: volume value range 0 ~ 100,  0 corresponds 0%, 100 corresponds 100%,
 *                     return -EINVAL if out of range
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
int BT_AvrcControllerNotifyVolume(BT_ADDR *addr, uint8_t volume);

/**@brief  set supported notify event
 * @details This call should be called before the creating of AVRC CT connection.
 *          The information of supported event will be stored in the array of BT chip,
 *          so it is not needed to be called again before the second AVRC CT connection.
 *
 * @param[in]  support: supported event
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find the control block
 * @retval     -EINVAL: paramter inval
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_AvrcControllerSetSuppEvt(BT_AVRC_SUPPORT_NOTIFY_EVENT *support);

/** @} bt_funcs */

/** @} BT */

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_AVRC_CON_H */
