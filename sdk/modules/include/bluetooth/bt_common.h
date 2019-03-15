/****************************************************************************
 * modules/include/bluetooth/bt_common.h
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
 * @file bt_common.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief Bluetooth generic API.
 * @details This API is generic functions for bluetooth operations
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_BT_COMMON_H
#define __MODULES_INCLUDE_BLUETOOTH_BT_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @enum BT_CONNECT_STATUS
 * @brief BT profile connection status
 */
typedef enum
{
	BT_DISCONNECTED  = 0, /**< Disconnected */
	BT_DISCONNECTING = 1, /**< Disconnect operation working */
	BT_CONNECTING    = 2, /**< Connect operation working */
	BT_CONNECTED     = 3  /**< Connected */
} BT_CONNECT_STATUS;

/**
 * @struct bt_common_state_s
 * @brief Bluetooth base context
 */
struct bt_common_state_s
{
  struct bt_hal_common_ops_s  *bt_hal_common_ops;    /**< BT common HAL interfaces @ref bt_hal_common_ops_s */
  struct ble_hal_common_ops_s *ble_hal_common_ops;   /**< BLE common HAL interfaces @ref ble_hal_common_ops_s */
  struct bt_common_ops_s      *bt_common_ops;        /**< BT status callbacks @ref bt_common_ops_s */
  struct ble_common_ops_s      *ble_common_ops;      /**< BLE status callbacks @ref ble_common_ops_s */
  BT_ADDR                     bt_addr;               /**< BT local device address @ref BT_ADDR */
  BT_ADDR                     ble_addr;              /**< BLE local device address @ref BT_ADDR */
  char                        bt_name[BT_NAME_LEN];  /**< BT local device name */
  char                        ble_name[BT_NAME_LEN]; /**< BLE local device name */
};

/**
 * @struct bt_acl_state_s
 * @brief Bluetooth ACL context
 */
struct bt_acl_state_s
{
  BT_CONNECT_STATUS          bt_acl_connection;           /**< Status of ACL connection @ref BT_CONNECT_STATUS */
  struct bt_common_state_s   *bt_common_state;            /**< BT base context @ref bt_common_state_s */
  BT_ADDR                    bt_target_addr;              /**< BT target device address @ref BT_ADDR */
  char                       bt_target_name[BT_NAME_LEN]; /**< BT target device name */
};

/**
 * @struct ble_state_s
 * @brief Bluetooth LE context
 */
struct ble_state_s
{
  BT_CONNECT_STATUS          ble_connection;              /**< Status of BLE connection @ref BT_CONNECT_STATUS */
  uint16_t                   ble_connect_handle;          /**< Handle ID for BLE connection */
  struct bt_common_state_s   *bt_common_state;            /**< BT base context @ref bt_common_state_s */
  BT_ADDR                    bt_target_addr;              /**< BT target device address @ref BT_ADDR */
  char                       bt_target_name[BT_NAME_LEN]; /**< BT target device name */
};

/**
 * @struct bt_common_ops_s
 * @brief Bluetooth Common application callbacks
 */
struct bt_common_ops_s
{
  void (*command_status)(BT_CMD_STATUS status);                                                    /**< Command status */
  void (*pairing_complete)(BT_ADDR addr, BT_PAIR_STATUS status);                                   /**< Pairing complete */
  void (*inquiry_result)(BT_ADDR addr, char *name);                                                /**< Inquiry data result */
  void (*inquiry_complete)(void);                                                                  /**< Coplete inquiry */
  void (*connect_status_changed)(struct bt_acl_state_s *bt_acl_state, bool connected, int status); /**< Connection status change */
  void (*connected_device_name)(const char *name);                                                 /**< Device name change */
  void (*bond_info)(BT_ADDR addr);                                                                 /**< Bonding information */
};

/**
 * @struct ble_common_ops_s
 * @brief Bluetooth LE Common application callbacks
 */
struct ble_common_ops_s
{
  void (*connect_status_changed)(struct ble_state_s *ble_state, bool connected);  /**< Connection status change */
  void (*connected_device_name_resp)(const char *name);                           /**< Device name change */
  void (*scan_result)(BT_ADDR addr, char *dev_name);                              /**< Result callback for scan */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Bluetooth module initialize
 *        For initialize a pin config, NV storage, UART, etc.
 *
 * @retval error code
 */

int bt_init(void);

/**
 * @brief Bluetooth module finalize
 *
 * @retval error code
 */

int bt_finalize(void);

/**
 * @brief Set Bluetooth module address
 *        This is Spresense side address and should be call before bt_enable.
 *
 * @param[in] addr: Bluetooth device address @ref BT_ADDR
 *
 * @retval error code
 */

int bt_set_address(BT_ADDR *addr);

/**
 * @brief Get Bluetooth module address
 *
 * @param[out] addr: Bluetooth device address @ref BT_ADDR
 *
 * @retval error code
 */

int bt_get_address(BT_ADDR *addr);

/**
 * @brief Set Bluetooth module name
 *        This name visible for other devices and should be call before bt_enable.
 *
 * @param[in] name: Bluetooth device name
 *
 * @retval error code
 */

int bt_set_name(char *name);

/**
 * @brief Get Bluetooth module name
 *
 * @param[out] name: Bluetooth device name
 *
 * @retval error code
 */

int bt_get_name(char *name);

/**
 * @brief Bluetooth module enable
 *        Bluetooth set power on(and download firmware, etc).
 *
 * @retval error code
 */

int bt_enable(void);

/**
 * @brief Bluetooth module disable
 *        Bluetooth set power off.
 *
 * @retval error code
 */

int bt_disable(void);

/**
 * @brief Bluetooth pairing enable
 *        Entering bluetooth pairing mode.
 *
 * @retval error code
 */

int bt_pairing_enable(void);

/**
 * @brief Bluetooth pairing disable
 *        Escaping bluetooth pairing mode.
 *
 * @retval error code
 */

int bt_paring_disable(void);

/**
 * @brief Bluetooth get bond device list
 *        Get bond devices list with BD_ADDR.
 *
 * @param[in] addr: Device address list @ref BT_ADDR
 * @param[in] num: Number of BD_ADDR
 *
 * @retval error code
 */

int bt_get_bond_list(BT_ADDR *addr, int *num);

/**
 * @brief Bluetooth unbond by BD_ADDR
 *        Unbond device by BD_ADDR.
 *
 * @param[out] addr: Unbond device BD_ADDR @ref BT_ADDR
 *
 * @retval error code
 */

int bt_unbond(BT_ADDR *addr);

/**
 * @brief Bluetooth set visible
 *        Visible this device from others.
 *
 * @param[in] visibility: Device visibility parameter @ref BT_VISIBILITY
 *
 * @retval error code
 */

int bt_set_visibility(BT_VISIBILITY visibility);

/**
 * @brief Bluetooth inquiry start
 *        Start to inquiry for connect peer device.
 *
 * @retval error code
 */

int bt_start_inquiry(void);

/**
 * @brief Bluetooth inquiry cancel
 *        Cancel to inquiry.
 *
 * @retval error code
 */

int bt_cancel_inquiry(void);

/**
 * @brief Bluetooth register common callbacks
 *        Register Connect/Pairing/Inquiry callback
 *
 * @param[in] bt_common_ops: Application callback @ref bt_common_ops_s
 *
 * @retval error code
 */

int bt_register_common_cb(struct bt_common_ops_s *bt_common_ops);

/**
 * @brief Set Bluetooth LE module address
 *        This is Spresense side address and should be call before bt_enable.
 *
 * @param[in] addr: Bluetooth LE device address @ref BT_ADDR
 *
 * @retval error code
 */

int ble_set_address(BT_ADDR *addr);

/**
 * @brief Get Bluetooth LE module address
 *
 * @param[out] addr: Bluetooth LE device address @ref BT_ADDR
 *
 * @retval error code
 */

int ble_get_address(BT_ADDR *addr);

/**
 * @brief Set Bluetooth LE module name
 *        This name visible for other devices and should be call before bt_enable.
 *
 * @param[in] name: Bluetooth LE device name
 *
 * @retval error code
 */

int ble_set_name(char *name);

/**
 * @brief Get Bluetooth LE module name
 *
 * @param[out] name: Bluetooth LE device name
 *
 * @retval error code
 */

int ble_get_name(char *name);

/**
 * @brief Bluetooth LE enable
 *
 * @retval error code
 */

int ble_enable(void);

/**
 * @brief Bluetooth LE disable
 *
 * @retval error code
 */

int ble_disable(void);

/**
 * @brief Bluetooth LE connect for Central
 *
 * @param[in] ble_state: Bluetooth context @ref ble_state_s
 *
 * @retval error code
 */

int ble_connect(struct ble_state_s *ble_state);

/**
 * @brief Bluetooth LE dicsonnect for Central
 *
 * @param[in] ble_state: Bluetooth context @ref ble_state_s
 *
 * @retval error code
 */

int ble_disconnect(struct ble_state_s *ble_state);

/**
 * @brief Bluetooth LE start advertise
 *        Start BLE advertise mode.
 *
 * @retval error code
 */

int ble_start_advertise(void);

/**
 * @brief Bluetooth LE cancel advertise
 *        Cancel BLE advertise mode.
 *
 * @retval error code
 */

int ble_cancel_advertise(void);

/**
 * @brief Bluetooth LE start scan
 *        Start BLE scan mode.
 *
 * @retval error code
 */

int ble_start_scan(void);

/**
 * @brief Bluetooth LE cancel scan
 *        Cancel BLE scan mode.
 *
 * @retval error code
 */

int ble_cancel_scan(void);

/**
 * @brief Bluetooth LE register common callbacks
 *        Register Connect/Advertise/Scan callback
 *
 * @param[in] ble_common_ops: Application callback @ref ble_common_ops_s
 *
 * @retval error code
 */

int ble_register_common_cb(struct ble_common_ops_s *ble_common_ops);

#endif /* __MODULES_INCLUDE_BLUETOOTH_BT_COMMON_H */
