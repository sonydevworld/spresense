/****************************************************************************
 * modules/include/bluetooth/bt_hfp.h
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
 * @file bt_hfp.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief HFP API.
 * @details This API is for using HFP and includes Function and Callback
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_BT_HFP_H
#define __MODULES_INCLUDE_BLUETOOTH_BT_HFP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bt_hfp_features.h>
#include <bluetooth/bt_common.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct bt_hfp_ops_s
 * @brief Bluetooth HFP application callbacks
 */
struct bt_hfp_ops_s
{
  void (*command_status)(BT_CMD_STATUS status);                                            /**< Command status */
  void (*connect)(struct bt_acl_state_s *bt_acl_state, BT_PROFILE_TYPE btProfileType);     /**< HFP connection status */
  void (*disconnect)(struct bt_acl_state_s *bt_acl_state);                                 /**< HFP disconnection status */
  void (*audio_connect)(struct bt_acl_state_s *bt_acl_state);                              /**< HFP audio connection status */
  void (*audio_disconnect)(struct bt_acl_state_s *bt_acl_state);                           /**< HFP audio disconnection */
  void (*ag_feature)(struct bt_acl_state_s *bt_acl_state, BT_HFP_AG_FEATURE_FLAG feature); /**< HFP AG feature response */
  void (*hf_at_response)(struct bt_acl_state_s *bt_acl_state, char *at_resp);              /**< HFP AT command response */
};

/**
 * @struct bt_hfp_state_s
 * @brief Bluetooth HFP context
 */
struct bt_hfp_state_s
{
  BT_CONNECT_STATUS        bt_hfp_connection;       /**< Status of HFP connection @ref BT_CONNECT_STATUS */
  BT_CONNECT_STATUS        bt_hfp_audio_connection; /**< Status of HFP audio connection @ref BT_CONNECT_STATUS */
  uint16_t                 bt_hfp_handle;           /**< Handle ID of HFP connection */
  uint16_t                 bt_hfp_audio_handle;     /**< Handle ID of HFP audio connection */
  struct bt_acl_state_s    *bt_acl_state;           /**< Bluetooth ACL context @ref bt_acl_state_s */
  struct bt_hal_hfp_ops_s  *bt_hal_hfp_ops;         /**< HFP HAL interfaces @ref bt_hal_hfp_ops_s */
  struct bt_hfp_ops_s      *bt_hfp_ops;             /**< HFP connection callbacks @ref bt_hfp_ops_s */
  BT_HFP_HF_FEATURE_FLAG   bt_hfp_supported_feature;/**< HFP supported feature @ref BT_HFP_HF_FEATURE_FLAG */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Get HFP support or not support
 *
 * @retval Suppot or Not support
 */

bool bt_hfp_is_supported(void);

/**
 * @brief Bluetooth HFP connect
 *        Connect to peer device with HFP.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 *
 * @retval error code
 */

int bt_hfp_connect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth HFP disconnect
 *        Disconnect to peer device with HFP.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 *
 * @retval error code
 */

int bt_hfp_disconnect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth HFP Audio connect
 *        Connect to peer device with HFP Audio.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 *
 * @retval error code
 */

int bt_hfp_audio_connect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth HFP Audio disconnect
 *        Disconnect to peer device with HFP Audio.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 *
 * @retval error code
 */

int bt_hfp_audio_disconnect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth send HFP command
 *        Send HFP command.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 * @param[in] at_cmd_str: AT command string(Need to end with '\0')
 *
 * @retval error code
 */

int bt_hfp_send_at_command(struct bt_acl_state_s *bt_acl_state, char *at_cmd_str);

/**
 * @brief Bluetooth press button
 *        Press button.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 * @retval error code
 */

int bt_hfp_press_button(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth set feature
 *        Set feature.
 *
 * @param[in] flag: Bluetooth feature flag @ref BT_HFP_HF_FEATURE_FLAG
 * @retval error code
 */

int bt_hfp_set_feature(BT_HFP_HF_FEATURE_FLAG flag);

/**
 * @brief Bluetooth HFP Register callbacks
 *        Set callback about HFP.
 *
 * @param[in] bt_hfp_ops: HFP callbacks @ref bt_hfp_ops_s
 *
 * @retval error code
 */

int bt_hfp_register_cb(struct bt_hfp_ops_s *bt_hfp_ops);

#endif /* __MODULES_INCLUDE_BLUETOOTH_BT_HFP_H */
