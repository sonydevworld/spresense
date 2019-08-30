/****************************************************************************
 * modules/include/bluetooth/bt_avrcp.h
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
 * @file bt_avrcp.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief AVRCP API.
 * @details This API is for using AVRCP and includes Function and Callback
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_BT_AVRCP_H
#define __MODULES_INCLUDE_BLUETOOTH_BT_AVRCP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bt_avrcp_cmds.h>
#include <bluetooth/bt_common.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @enum BT_AVRCP_ROLE
 * @brief AVRCP role
 */
typedef enum
{
	BT_AVRCP_CONTROLLER  = 0, /**< Controller */
	BT_AVRCP_TARGET      = 1  /**< Target */
} BT_AVRCP_ROLE;

/**
 * @struct BT_AVRC_TRACK_INFO
 * @brief BT avrc track info type
 */
typedef struct avrcTrackInfo {
	BT_ADDR addr;           /**< Target device address */
	uint8_t attrId;         /**< Track information attribute ID */
	uint16_t attrLen;       /**< Attribute data length */
	uint8_t attrValue[48];  /**< Attribute data */
} BT_AVRC_TRACK_INFO;

/**
 * @struct bt_avrcp_notify_ops_s
 * @brief Bluetooth AVRCP application callbacks
 */
struct bt_avrcp_notify_ops_s
{
  void (*playStatusChange)(BT_AVRC_TRACK_INFO *trackInfo);  /**< Play status change */
  void (*trackChange)(BT_AVRC_TRACK_INFO *trackInfo);       /**< Track change */
  void (*trackReachedEnd)(uint8_t *pdata, int len);         /**< Track Reach End */
  void (*trackReachedStart)(uint8_t *pdata, int len);       /**< Track Reach Start */
  void (*playPosChanged)(uint8_t *pdata, int len);          /**< Play position change */
  void (*batteryStatusChange)(uint8_t *pdata, int len);     /**< Battery status change */
  void (*systemStatusChange)(uint8_t *pdata, int len);      /**< System status change */
  void (*appSettingChange)(uint8_t *pdata, int len);        /**< Application settings change */
  void (*nowPlayingChange)(uint8_t *pdata, int len);        /**< Content of Now Playing folder change */
  void (*avalPlayerChange)(uint8_t *pdata, int len);        /**< Available players change */
  void (*addrPlayChange)(uint8_t *pdata, int len);          /**< Addressed player change */
  void (*uidsChange)(uint8_t *pdata, int len);              /**< UIDs change */
  void (*volumeChange)(uint8_t *pdata, int len);            /**< Volume change on the target device */
};

/**
 * @struct bt_avrcp_ops_s
 * @brief  Bluetooth AVRCP application callbacks
 */
struct bt_avrcp_ops_s
{
  void (*command_status)(BT_CMD_STATUS status);                                /**< Command status notification */
  void (*connect)(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_ROLE role);    /**< Connection status notification */
  void (*disconnect)(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_ROLE role); /**< Disconnection status notification */
};

/**
 * @struct bt_avrcp_state_s
 * @brief  Bluetooth AVRCP context
 */
struct bt_avrcp_state_s
{
  BT_CONNECT_STATUS            bt_avrcc_connection;  /**< Status of AVRCP(controller) connection @ref BT_CONNECT_STATUS */
  BT_CONNECT_STATUS            bt_avrct_connection;  /**< Status of AVRCP(target) connection @ref BT_CONNECT_STATUS */
  uint16_t                     bt_avrcc_handle;      /**< Handle ID of AVRCP Controller connection */
  uint16_t                     bt_avrct_handle;      /**< Handle ID of AVRCP Target connection */
  struct bt_acl_state_s        *bt_acl_state;        /**< Bluetooth ACL context @ref bt_acl_state_s */
  struct bt_hal_avrcp_ops_s    *bt_hal_avrcp_ops;    /**< AVRCP HAL interfaces @ref bt_hal_avrcp_ops_s */
  struct bt_avrcp_ops_s        *bt_avrcp_ops;        /**< AVRCP connection callbacks @ref bt_avrcp_ops_s */
  struct bt_avrcp_notify_ops_s *bt_avrcp_notify_ops; /**< AVRCP Notification callbacks @refbt_avrcp_notify_ops_s  */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Get AVRCP support or not support
 *
 * @retval Suppot or Not support
 */

bool bt_avrcp_is_supported(void);

/**
 * @brief Bluetooth AVRCP connect
 *        Connect to peer device with AVRCP.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 *
 * @retval error code
 */

int bt_avrcp_connect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth AVRCP disconnect
 *        Disconnect to peer device with AVRCP.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 *
 * @retval error code
 */

int bt_avrcp_disconnect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth send AVRCP command
 *        Send command to target device.
 *
 * @param[in] bt_acl_state: Bluetooth context @ref bt_acl_state_s
 * @param[in] cmd_id: Command ID @ref BT_AVRCP_CMD_ID
 * @param[in] press: Button status (press/release)
 *
 * @retval error code
 */

int bt_avrcp_send_command(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_CMD_ID cmd_id, bool press);

/**
 * @brief Bluetooth AVRCP Register notification
 *        Set callback about AVRCP notification.
 *
 * @param[in] bt_avrcp_notify_ops: Notification callbacks @ref bt_avrcp_notify_ops_s
 *
 * @retval error code
 */

int bt_register_notification(struct bt_avrcp_notify_ops_s *bt_avrcp_notify_ops);

/**
 * @brief Bluetooth AVRCP Register connection status callback
 *        Set callback about AVRCP connection.
 *
 * @param[in] bt_avrcp_ops: connection callbacks @ref bt_avrcp_ops_s
 *
 * @retval error code
 */

int bt_avrcp_register_cb(struct bt_avrcp_ops_s *bt_avrcp_ops);

#endif /* __MODULES_INCLUDE_BLUETOOTH_BT_AVRCP_H */
