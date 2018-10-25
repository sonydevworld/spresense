/****************************************************************************
 * modules/include/bluetooth/bt_spp.h
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
 * @file bt_spp.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief SPP API.
 * @details This API is for using SPP and includes Function and Callback
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_BT_SPP_H
#define __MODULES_INCLUDE_BLUETOOTH_BT_SPP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bt_common.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct bt_spp_ops_s
 * @brief Bluetooth SPP application callbacks
 */
struct bt_spp_ops_s
{
  void (*connect)(struct bt_acl_state_s *bt_acl_state);                                            /**< Connection status */
  void (*disconnect)(struct bt_acl_state_s *bt_acl_state);                                         /**< Disconnection status */
  void (*connection_fail)(struct bt_acl_state_s *bt_acl_state, BT_CONNECT_FAIL_REASON_ID fail_id); /**< Connection fail */
  void (*receive_data)(struct bt_acl_state_s *bt_acl_state, uint8_t *data, int len);               /**< Receive SPP data */
};

/**
 * @struct bt_spp_state_s
 * @brief Bluetooth SPP context
 */
struct bt_spp_state_s
{
  BT_CONNECT_STATUS        bt_spp_connection; /**< Status of SPP connection @ref BT_CONNECT_STATUS */
  uint16_t                 bt_spp_handle;     /**< Handle ID of SPP connection */
  struct bt_acl_state_s    *bt_acl_state;     /**< Bluetooth ACL context @ref bt_acl_state_s */
  struct bt_hal_spp_ops_s  *bt_hal_spp_ops;   /**< SPP HAL interfaces @ref bt_hal_spp_ops_s */
  struct bt_spp_ops_s      *bt_spp_ops;       /**< SPP connection callbacks @ref bt_spp_ops_s */
  BT_UUID                  spp_uuid;          /**< SPP UUID @ref BT_UUID */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Get SPP support or not support
 *
 * @retval Suppot or Not support
 */

bool bt_spp_is_supported(void);

/**
 * @brief Bluetooth SPP connect
 *        Connect to peer device with SPP.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 *
 * @retval error code
 */

int bt_spp_connect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth SPP disconnect
 *        Disconnect to peer device with SPP.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 *
 * @retval error code
 */

int bt_spp_disconnect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth SPP set UUID
 *
 * @param[in] uuid: UUID @ref BT_UUID
 *
 * @retval error code
 */

int bt_spp_set_uuid(BT_UUID *uuid);

/**
 * @brief Bluetooth SPP send Tx data
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 * @param[in] data: uint8_t * Tx data.
 * @param[in] len: int Tx data size.
 *
 * @retval error code
 */

int bt_spp_send_tx_data(struct bt_acl_state_s *bt_acl_state, uint8_t *data, int len);

/**
 * @brief Bluetooth SPP Register notification
 *        Set callback about SPP callbacks.
 *
 * @param[in] bt_spp_ops: Callback functions @ref bt_spp_ops_s
 *
 * @retval error code
 */

int bt_spp_register_cb(struct bt_spp_ops_s *bt_spp_ops);

#endif /* __MODULES_INCLUDE_BLUETOOTH_BT_SPP_H */
