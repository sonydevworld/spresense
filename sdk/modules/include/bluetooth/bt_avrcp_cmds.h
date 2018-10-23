/****************************************************************************
 * modules/include/bluetooth/bt_avrcp_cmds.h
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
 * @file bt_avrcp_cmds.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief AVRCP command I/F.
 * @details This header file includes AVRCP related commands list.
 *           - AVRCP available commands
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_BT_AVRCP_CMDS_H
#define __MODULES_INCLUDE_BLUETOOTH_BT_AVRCP_CMDS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @enum BT_AVRCP_CMD_ID
 * @brief AVRCP command list
 */
typedef enum
{
  BT_AVRCP_CMD_SELECT     = 0x00, /**< 'Select' button */
  BT_AVRCP_CMD_UP,                /**< 'Up' button */
  BT_AVRCP_CMD_DOWN,              /**< 'Down' button */
  BT_AVRCP_CMD_LEFT,              /**< 'Left' button */
  BT_AVRCP_CMD_RIGHT,             /**< 'Right' button */
  BT_AVRCP_CMD_RIGHT_UP,          /**< 'Right Up' button */
  BT_AVRCP_CMD_RIGHT_DOWN,        /**< 'Right Down' button */
  BT_AVRCP_CMD_LEFT_UP,           /**< 'Left Up' button */
  BT_AVRCP_CMD_LEFT_DOWN,         /**< 'Left Down' button */
  BT_AVRCP_CMD_ROOT_MENU,         /**< 'Root menu' button */
  BT_AVRCP_CMD_SETUP_MENU,        /**< 'Setup menu' button */
  BT_AVRCP_CMD_CONT_MENU,         /**< 'Contents menu' button */
  BT_AVRCP_CMD_FAV_MENU,          /**< 'Favorite menu' button */
  BT_AVRCP_CMD_EXIT,              /**< 'Exit' button */
  BT_AVRCP_CMD_0          = 0x20, /**< '0' button */
  BT_AVRCP_CMD_1,                 /**< '1' button */
  BT_AVRCP_CMD_2,                 /**< '2' button */
  BT_AVRCP_CMD_3,                 /**< '3' button */
  BT_AVRCP_CMD_4,                 /**< '4' button */
  BT_AVRCP_CMD_5,                 /**< '5' button */
  BT_AVRCP_CMD_6,                 /**< '6' button */
  BT_AVRCP_CMD_7,                 /**< '7' button */
  BT_AVRCP_CMD_8,                 /**< '8' button */
  BT_AVRCP_CMD_9,                 /**< '9' button */
  BT_AVRCP_CMD_DOT,               /**< '.' button */
  BT_AVRCP_CMD_ENTER,             /**< 'Enter' button */
  BT_AVRCP_CMD_CLEAR,             /**< 'Clear' button */
  BT_AVRCP_CMD_CHAN_UP    = 0x30, /**< 'Channel Up' button */
  BT_AVRCP_CMD_CHAN_DOWN,         /**< 'Channel Down' button */
  BT_AVRCP_CMD_PREV_CHAN,         /**< 'Previous Channel' button */
  BT_AVRCP_CMD_SOUND_SEL,         /**< 'Sound Select' button */
  BT_AVRCP_CMD_INPUT_SEL,         /**< 'Input Select' button */
  BT_AVRCP_CMD_DISP_INFO,         /**< 'Display Information' button */
  BT_AVRCP_CMD_HELP,              /**< 'Help' button */
  BT_AVRCP_CMD_PAGE_UP,           /**< 'Page Up' button */
  BT_AVRCP_CMD_PAGE_DOWN,         /**< 'Page Down' button */
  BT_AVRCP_CMD_POWER      = 0x40, /**< 'Power' button */
  BT_AVRCP_CMD_VOL_UP,            /**< 'Volume Up' button */
  BT_AVRCP_CMD_VOL_DOWN,          /**< 'Volume Down' button */
  BT_AVRCP_CMD_MUTE,              /**< 'Mute' button */
  BT_AVRCP_CMD_PLAY,              /**< 'Play' button */
  BT_AVRCP_CMD_STOP,              /**< 'Stop' button */
  BT_AVRCP_CMD_PAUSE,             /**< 'Pause' button */
  BT_AVRCP_CMD_RECORD,            /**< 'Record' button */
  BT_AVRCP_CMD_REWIND,            /**< 'Rewind' button */
  BT_AVRCP_CMD_FAST_FOR,          /**< 'Fast Forward' button */
  BT_AVRCP_CMD_EJECT,             /**< 'Eject' button */
  BT_AVRCP_CMD_FORWARD,           /**< 'Forward' button */
  BT_AVRCP_CMD_BACKWARD,          /**< 'Backward' button */
  BT_AVRCP_CMD_ANGLE      = 0x50, /**< 'Angle' button */
  BT_AVRCP_CMD_SUBPICT,           /**< 'Subpicture' button */
  BT_AVRCP_CMD_F1         = 0x71, /**< 'F1' button */
  BT_AVRCP_CMD_F2,                /**< 'F2' button */
  BT_AVRCP_CMD_F3,                /**< 'F3' button */
  BT_AVRCP_CMD_F4,                /**< 'F4' button */
  BT_AVRCP_CMD_F5,                /**< 'F5' button */
  BT_AVRCP_CMD_VENDOR     = 0x7E  /**< Vendor unique button */
} BT_AVRCP_CMD_ID;

#endif /* __MODULES_INCLUDE_BLUETOOTH_BT_AVRCP_CMDS_H */
