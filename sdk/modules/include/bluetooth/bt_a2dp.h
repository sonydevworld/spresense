/****************************************************************************
 * modules/include/bluetooth/bt_a2dp.h
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
 * @file bt_a2dp.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief A2DP API.
 * @details This API is for using A2DP and includes Function and Callback
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_BT_A2DP_H
#define __MODULES_INCLUDE_BLUETOOTH_BT_A2DP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bt_a2dp_codecs.h>
#include <bluetooth/bt_common.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/**
 * @struct bt_a2dp_ops_s
 * @brief Bluetooth A2DP application callbacks
 */
struct bt_a2dp_ops_s
{
  void (*command_status)(BT_CMD_STATUS status);                                           /**< Command status */
  void (*connect)(struct bt_acl_state_s *bt_acl_state, BT_AUDIO_CODEC_INFO codecInfo);    /**< Connection status */
  void (*disconnect)(struct bt_acl_state_s *bt_acl_state);                                /**< Disconnection status */
  void (*receive_media_pkt)(struct bt_acl_state_s *bt_acl_state, uint8_t *data, int len); /**< Receive media data */
};

/**
 * @struct bt_a2dp_state_s
 * @brief Bluetooth A2DP context
 */
struct bt_a2dp_state_s
{
  BT_CONNECT_STATUS        bt_a2dp_connection; /**< Status of A2DP connection @ref BT_CONNECT_STATUS */
  uint16_t                 bt_a2dp_handle;     /**< Handle ID of A2DP connection */
  struct bt_acl_state_s    *bt_acl_state;      /**< Bluetooth ACL context @ref bt_acl_state_s */
  struct bt_hal_a2dp_ops_s *bt_hal_a2dp_ops;   /**< A2DP HAL interfaces @ref bt_hal_a2dp_ops_s */
  struct bt_a2dp_ops_s     *bt_a2dp_ops;       /**< A2DP connection callbacks @ref bt_a2dp_ops_s */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Get A2DP support or not support
 *
 * @retval Suppot or Not support
 */

bool bt_a2dp_is_supported(void);

/**
 * @brief Bluetooth A2DP connect
 *        Connect to peer device with A2DP.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 *
 * @retval error code
 */

int bt_a2dp_connect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth A2DP disconnect
 *        Disconnect to peer device with A2DP.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 *
 * @retval error code
 */

int bt_a2dp_disconnect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth A2DP set codec capability
 *        Set capability of audio codecs
 *
 * @param[in] codec_capabilities: BT_A2DP_CODEC_TYPE Codec type list(SBC/AAC) @ref BT_AUDIO_CODEC_INFO
 * @param[in] num: Length of codec capability
 *
 * @retval error code
 */

int bt_a2dp_set_codec_capability(BT_AUDIO_CODEC_INFO *codec_capabilities, uint8_t num);

/**
 * @brief Register A2DP event packet callback
 *
 * @param[in] bt_a2dp_ops: Callback function for event handler @ref bt_a2dp_ops_s
 *
 * @retval error code
 */

int bt_a2dp_register_callback(struct bt_a2dp_ops_s *bt_a2dp_ops);

#endif /* __MODULES_INCLUDE_BLUETOOTH_BT_A2DP_H */
