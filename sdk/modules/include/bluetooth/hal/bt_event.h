/****************************************************************************
 * modules/include/bluetooth/hal/bt_event.h
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
 * @file bt_event.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief Bluetooth HAL I/F for event handler.
 * @details This header file includes bluetooth event related definitions
 *          for HAL I/F.
 *          Event handler will be use this definition for switch events.
 *           - Event type
 *           - Event structure
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_HAL_BT_EVENT_H
#define __MODULES_INCLUDE_BLUETOOTH_HAL_BT_EVENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 *@name Max event data length
 *@{
 */
#define BT_MAX_EVENT_DATA_LEN 1024
/** @} */

/**
 *@name Max ble advertise data length
 *@{
 */
#define BLE_MAX_ADV_DATA_LEN 31
/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @enum BT_GROUP_ID
 * @brief Bluetooth profile and protocol ID
 */
typedef enum
{
	BT_GROUP_COMMON = 0,  /**< Common group */
	BT_GROUP_A2DP,        /**< A2DP */
	BT_GROUP_AVRCP,       /**< AVRCP */
	BT_GROUP_HFP,         /**< HFP */
	BT_GROUP_SPP,         /**< SPP */
	BT_GROUP_RFCOMM,      /**< RFCOMM */
	BLE_GROUP_COMMON,     /**< BLE Common */
	BLE_GROUP_GATT        /**< BLE GATT */
} BT_GROUP_ID;

/**
 * @enum BT_COMMON_EVENT_ID
 * @brief Bluetooth event ID for common function
 */
typedef enum
{
	BT_COMMON_EVENT_CMD_STATUS = 0,   /**< Command status event */
	BT_COMMON_EVENT_PAIRING_COMPLETE, /**< Pairing complete event */
	BT_COMMON_EVENT_INQUIRY_RESULT,   /**< Inquiry result event */
	BT_COMMON_EVENT_INQUIRY_COMPLETE, /**< Inquiry complete event */
	BT_COMMON_EVENT_CONN_STAT_CHANGE, /**< Connection status change event */
	BT_COMMON_EVENT_CONN_DEV_NAME,    /**< Connected device name event */
	BT_COMMON_EVENT_BOND_INFO,        /**< Bonding information event */
} BT_COMMON_EVENT_ID;

/**
 * @enum BT_A2DP_EVENT_ID
 * @brief Bluetooth event ID for A2DP
 */
typedef enum
{
	BT_A2DP_EVENT_CMD_STATUS = 0, /**< Command status event */
	BT_A2DP_EVENT_CONNECT,        /**< Connect event */
	BT_A2DP_EVENT_DISCONNECT,     /**< Disconnect event */
	BT_A2DP_EVENT_MEDIA_PACKET,   /**< Media packet receive event */
} BT_A2DP_EVENT_ID;

/**
 * @enum BT_AVRCP_EVENT_ID
 * @brief Bluetooth event ID for A2DP
 */
typedef enum
{
	BT_AVRCP_EVENT_CMD_STATUS = 0,     /**< Command status event */
	BT_AVRCC_EVENT_CONNECT,            /**< AVRCP controller connect event */
	BT_AVRCC_EVENT_DISCONNECT,         /**< AVRCP vontroller disconnect event */
	BT_AVRCT_EVENT_CONNECT,            /**< AVRCP target connect event */
	BT_AVRCT_EVENT_DISCONNECT,         /**< AVRCP target disconnect event */
	BT_AVRCP_EVENT_PLAY_STAT_CHANGE,   /**< Play status change event */
	BT_AVRCP_EVENT_TRACK_CHANGE,       /**< Play track change event */
	BT_AVRCP_EVENT_TRACK_REACH_END,    /**< Play track reach end event */
	BT_AVRCP_EVENT_TRACK_REACH_START,  /**< Play track reach start event */
	BT_AVRCP_EVENT_PLAY_POS_CHANGE,    /**< Play position change event */
	BT_AVRCP_EVENT_BATT_STAT_CHANGE,   /**< Battery status change event */
	BT_AVRCP_EVENT_SYS_STATUS_CHANGE,  /**< System status change event */
	BT_AVRCP_EVENT_APP_SETT_CHANGE,    /**< Application settings change event */
	BT_AVRCP_EVENT_NOW_PLAY_CHANGE,    /**< Content of Now Playing folder change event */
	BT_AVRCP_EVENT_AVAI_PLAYER_CHANGE, /**< Available players change event */
	BT_AVRCP_EVENT_ADDR_PLAYER_CHANGE, /**< Addressed player change event */
	BT_AVRCP_EVENT_UIDS_CHANGE,        /**< UIDs change event */
	BT_AVRCP_EVENT_VOLUME_CHANGE,      /**< Volume change on the target device event */
} BT_AVRCP_EVENT_ID;

/**
 * @enum BT_HFP_EVENT_ID
 * @brief Bluetooth event ID for HFP
 */
typedef enum
{
	BT_HFP_EVENT_CMD_STATUS = 0,   /**< Command status event */
	BT_HFP_EVENT_HF_CONNECT,       /**< HFP connect event */
	BT_HFP_EVENT_HF_DISCONNECT,    /**< HFP disconnect event */
	BT_HFP_EVENT_AUDIO_CONNECT,    /**< HFP audio connect event */
	BT_HFP_EVENT_AUDIO_DISCONNECT, /**< HFP audio disconnect event */
	BT_HFP_EVENT_AG_FEATURE_RESP,  /**< HFP AG feature response event */
	BT_HFP_EVENT_AT_CMD_RESP,      /**< HFP AT command response event */
} BT_HFP_EVENT_ID;

/**
 * @enum BT_SPP_EVENT_ID
 * @brief Bluetooth event ID for SPP
 */
typedef enum
{
	BT_SPP_EVENT_CONNECT = 0,  /**< Connect event */
	BT_SPP_EVENT_DISCONNECT,   /**< Disconnect event */
	BT_SPP_EVENT_CONNECT_FAIL, /**< Connection fail event */
	BT_SPP_EVENT_RX_DATA,      /**< Receive SPP data event */
} BT_SPP_EVENT_ID;

/**
 * @enum BLE_COMMON_EVENT_ID
 * @brief Bluetooth LE event ID for common function
 */
typedef enum
{
	BLE_COMMON_EVENT_CONN_STAT_CHANGE = 0, /**< Connection status change event */
	BLE_COMMON_EVENT_CONN_DEV_NAME,        /**< Device name receive event */
	BLE_COMMON_EVENT_SCAN_RESULT,          /**< Scan result event */
} BLE_COMMON_EVENT_ID;

/**
 * @enum BLE_COMMON_EVENT_ID
 * @brief Bluetooth LE event ID for common function
 */
typedef enum
{
	BLE_GATT_EVENT_WRITE_REQ = 0,           /**< GATT Characteristic write request event */
	BLE_GATT_EVENT_READ_REQ,                /**< GATT Characteristic read request event */
	BLE_GATT_EVENT_NOTIFY_REQ,              /**< GATT Characteristic notify request event */
	BLE_GATT_EVENT_WRITE_RESP,              /**< GATT Characteristic write response event */
	BLE_GATT_EVENT_READ_RESP,               /**< GATT Characteristic read response event */
	BLE_GATT_EVENT_NOTIFY_RESP,             /**< GATT Characteristic notify response event */
	BLE_GATT_EVENT_DB_DISCOVERY_COMPLETE,   /**< GATTC discovery requested by host completed */
} BLE_GATT_EVENT_ID;

/**
 * @struct bt_event_t
 * @brief Bluetooth parent event data type
 */
struct bt_event_t
{
  uint8_t group_id;                    /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                    /**< Event sub ID */
  uint8_t data[BT_MAX_EVENT_DATA_LEN]; /**< Event data */
};

/**
 * @struct bt_event_cmd_stat_t
 * @brief Bluetooth command status event data type
 */
struct bt_event_cmd_stat_t
{
  uint8_t group_id;         /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;         /**< Event sub ID */
  BT_CMD_STATUS cmd_status; /**< Command status @ref BT_CMD_STATUS */
};

/**
 * @struct bt_event_pair_cmplt_t
 * @brief Bluetooth pairing complete event data type
 */
struct bt_event_pair_cmplt_t
{
  uint8_t group_id;      /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;      /**< Event sub ID @ref BT_COMMON_EVENT_ID */
  BT_ADDR addr;          /**< Pairing target device address @ref BT_ADDR */
  BT_PAIR_STATUS status; /**< Pairing status @ref BT_PAIR_STATUS */
};

/**
 * @struct bt_event_inquiry_rslt_t
 * @brief Bluetooth inquiry result event data type
 */
struct bt_event_inquiry_rslt_t
{
  uint8_t group_id;       /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;       /**< Event sub ID @ref BT_COMMON_EVENT_ID */
  BT_ADDR addr;           /**< Scaned device address @ref BT_ADDR */
  char name[BT_NAME_LEN]; /**< Scaned device name */
};

/**
 * @struct bt_event_conn_stat_t
 * @brief Bluetooth connection status change event data type
 */
struct bt_event_conn_stat_t
{
  uint8_t group_id; /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id; /**< Event sub ID @ref BT_COMMON_EVENT_ID */
  BT_ADDR addr;     /**< Target address @ref BT_ADDR */
  bool connected;   /**< Connection status */
  uint8_t status;
};

/**
 * @struct bt_event_dev_name_t
 * @brief Bluetooth change device name event data type
 */
struct bt_event_dev_name_t
{
  uint8_t group_id;       /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;       /**< Event sub ID @ref BT_COMMON_EVENT_ID */
  char name[BT_NAME_LEN]; /**< Connected device name */
};

/**
 * @struct bt_event_bond_info_t
 * @brief Bluetooth new bonding information event data type
 */
struct bt_event_bond_info_t
{
  uint8_t group_id; /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id; /**< Event sub ID @ref BT_COMMON_EVENT_ID */
  BT_ADDR addr;     /**< Bond device address @ref BT_ADDR */
};

/**
 * @struct bt_a2dp_event_connect_t
 * @brief Bluetooth A2DP connection event data type
 */
struct bt_a2dp_event_connect_t
{
  uint8_t group_id; /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id; /**< Event sub ID @ref BT_A2DP_EVENT_ID */
  uint16_t handle;  /**< Handle ID for A2DP connection */
  BT_ADDR addr;     /**< Connected device address @ref BT_ADDR */
  BT_AUDIO_CODEC_INFO codecInfo;
};

/**
 * @struct bt_a2dp_event_recv_t
 * @brief Bluetooth A2DP receive data event data type
 */
struct bt_a2dp_event_recv_t
{
  uint8_t group_id;                    /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                    /**< Event sub ID @ref BT_A2DP_EVENT_ID */
  BT_ADDR addr;                        /**< Connected device address @ref BT_ADDR */
  int len;                             /**< Receive data length */
  uint8_t data[BT_MAX_EVENT_DATA_LEN]; /**< Receive data */
};

/**
 * @struct bt_avrcp_event_connect_t
 * @brief Bluetooth AVRCP connection event data type
 */
struct bt_avrcp_event_connect_t
{
  uint8_t group_id; /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id; /**< Event sub ID @ref BT_AVRCP_EVENT_ID */
  uint16_t handle;  /**< Handle ID for AVRCP connection */
  BT_ADDR addr;     /**< Connected device address @ref BT_ADDR */
};

/**
 * @struct bt_avrcp_event_play_position_t
 * @brief Bluetooth AVRCP play position event data type
 */
struct bt_avrcp_event_play_position_t
{
  uint8_t group_id; /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id; /**< Event sub ID @ref BT_AVRCP_EVENT_ID */
  uint16_t handle;  /**< Handle ID for AVRCP connection */
  uint32_t position;/**< Play Position */
};

/**
 * @struct bt_hfp_event_connect_t
 * @brief Bluetooth HFP connection event data type
 */
struct bt_hfp_event_connect_t
{
  uint8_t group_id;         /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;         /**< Event sub ID @ref BT_HFP_EVENT_ID */
  uint16_t handle;          /**< Handle ID for HFP connection */
  BT_ADDR addr;             /**< Connected device address @ref BT_ADDR */
  BT_PROFILE_TYPE hfp_type; /**< Connected profile type @ref BT_PROFILE_TYPE */
};

/**
 * @struct bt_hfp_event_ag_feature_t
 * @brief Bluetooth HFP ag feature event data type
 */
struct bt_hfp_event_ag_feature_t
{
  uint8_t group_id;               /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;               /**< Event sub ID @ref BT_HFP_EVENT_ID */
  BT_ADDR addr;                   /**< Connected device address @ref BT_ADDR */
  BT_HFP_AG_FEATURE_FLAG ag_flag; /**< Support AG feature flag @ref BT_HFP_AG_FEATURE_FLAG */
};

/**
 * @struct bt_hfp_event_at_cmd_t
 * @brief Bluetooth HFP at command event data type
 */
struct bt_hfp_event_at_cmd_t
{
  uint8_t group_id;                       /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                       /**< Event sub ID @ref BT_HFP_EVENT_ID */
  BT_ADDR addr;                           /**< Connected device address @ref BT_ADDR */
  char at_resp[BT_MAX_EVENT_DATA_LEN];    /**< AT command response */
};

/**
 * @struct bt_spp_event_connect_t
 * @brief Bluetooth SPP connection event data type
 */
struct bt_spp_event_connect_t
{
  uint8_t group_id;                 /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                 /**< Event sub ID @ref BT_SPP_EVENT_ID */
  uint16_t handle;                  /**< Handle ID for SPP connection */
  BT_ADDR addr;                     /**< Connected device address @ref BT_ADDR */
  BT_CONNECT_FAIL_REASON_ID reason; /**< Connection fail reason @ref BT_CONNECT_FAIL_REASON_ID */
};

/**
 * @struct bt_spp_event_recv_data_t
 * @brief Bluetooth SPP Rx event data type
 */
struct bt_spp_event_recv_data_t
{
  uint8_t group_id;                    /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                    /**< Event sub ID @ref BT_SPP_EVENT_ID */
  BT_ADDR addr;                        /**< Connected device address @ref BT_ADDR */
  int     len;                         /**< Receive data length */
  uint8_t data[BT_MAX_EVENT_DATA_LEN]; /**< Receive data */
};

/**
 * @struct ble_event_conn_stat_t
 * @brief Bluetooth LE connection status change event data type
 */
struct ble_event_conn_stat_t
{
  uint8_t group_id; /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id; /**< Event sub ID @ref BLE_COMMON_EVENT_ID */
  uint16_t handle;  /**< Handle ID for BLE connection */
  BT_ADDR addr;     /**< Target address @ref BT_ADDR */
  bool connected;   /**< Connection status */
};

/**
 * @struct ble_event_conn_stat_t
 * @brief Bluetooth LE connection status change event data type
 */
struct ble_event_dev_name_t
{
  uint8_t group_id;       /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;       /**< Event sub ID @ref BLE_COMMON_EVENT_ID */
  BT_ADDR addr;           /**< Target address @ref BT_ADDR */
  char name[BT_NAME_LEN]; /**< Connected device name */
};

/**
 * @struct ble_gatt_event_write_req_t
 * @brief Bluetooth LE GATT Write request event
 */
struct ble_gatt_event_write_req_t
{
  uint8_t group_id;                    /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                    /**< Event sub ID @ref BLE_GATT_EVENT_ID */
  uint16_t serv_handle;                /**< Service handle ID @ref ble_gatt_service_s */
  uint16_t char_handle;                /**< Characteristic handle ID @ref ble_gatt_char_s */
  uint16_t length;                     /**< Write data length */
  uint8_t data[BT_MAX_EVENT_DATA_LEN]; /**< Write data */
};

/**
 * @struct ble_gatt_event_read_req_t
 * @brief Bluetooth LE GATT Read request event
 */
struct ble_gatt_event_read_req_t
{
  uint8_t group_id;                    /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                    /**< Event sub ID @ref BLE_GATT_EVENT_ID */
  uint16_t serv_handle;                /**< Service handle ID @ref ble_gatt_service_s */
  uint16_t char_handle;                /**< Characteristic handle ID @ref ble_gatt_char_s */
};

/**
 * @struct ble_gatt_event_notify_req_t
 * @brief Bluetooth LE GATT Notify request event
 */
struct ble_gatt_event_notify_req_t
{
  uint8_t group_id;                    /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                    /**< Event sub ID @ref BLE_GATT_EVENT_ID */
  uint16_t serv_handle;                /**< Service handle ID @ref ble_gatt_service_s */
  uint16_t char_handle;                /**< Characteristic handle ID @ref ble_gatt_char_s */
  bool enable;                         /**< Notify enable/disable */
};

/**
 * @struct ble_gatt_event_write_rsp_t
 * @brief Bluetooth LE GATT Write response event
 */
struct ble_gatt_event_write_rsp_t
{
  uint8_t group_id;                    /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                    /**< Event sub ID @ref BLE_GATT_EVENT_ID */
  uint16_t serv_handle;                /**< Service handle ID @ref ble_gatt_service_s */
  uint16_t conn_handle;                /**< Connection handle ID */
  uint16_t char_handle;                /**< Characteristic handle ID @ref ble_gatt_char_s */
  uint8_t status;                      /**< Write status */
};

/**
 * @struct ble_gatt_event_read_rsp_t
 * @brief Bluetooth LE GATT Read response event
 */
struct ble_gatt_event_read_rsp_t
{
  uint8_t group_id;                    /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                    /**< Event sub ID @ref BLE_GATT_EVENT_ID */
  uint16_t serv_handle;                /**< Service handle ID @ref ble_gatt_service_s */
  uint16_t conn_handle;                /**< Connection handle ID */
  uint16_t char_handle;                /**< Characteristic handle ID @ref ble_gatt_char_s */
  uint16_t length;                     /**< Read data length */
  uint8_t data[BT_MAX_EVENT_DATA_LEN]; /**< Read data */
};

/**
 * @struct ble_event_adv_rept_t
 * @brief Bluetooth LE advertise report event
 */
struct ble_event_adv_rept_t
{
  uint8_t group_id;                    /**< Event group ID @ref BT_GROUP_ID */
  uint8_t event_id;                    /**< Event sub ID @ref BLE_GATT_EVENT_ID */
  int8_t  rssi;                        /**< Received Signal Strength Indication in dBm. */
  uint8_t scan_rsp;                    /**< If 1, the report corresponds to a scan response */
  uint8_t length;                      /**< Scan response data length */
  uint8_t data[BLE_MAX_ADV_DATA_LEN];  /**< Scan response data */
  BT_ADDR addr;                        /**< Advertising device address @ref BT_ADDR */
};

/**
 * @struct BT_AVRC_SUPPORT_NOTIFY_EVENT
 * @brief BT avrc supported notify event
*/
typedef struct {
	bool playStatusChange;          /**< Playback Status Changed */
	bool trackChange;               /**< Track Changed */
	bool trackReachedEnd;           /**< Track End Reached */
	bool trackReachedStart;         /**< Track Reached Start */
	bool playPosChanged;            /**< Playback position changed */
	bool batteryStatusChange;       /**< Battery status changed */
	bool systemStatusChange;        /**< System status changed */
	bool appSettingChange;          /**< Player application settings changed */
	bool nowPlayingChange;          /**< Now Playing Content Changed (AVRCP 1.4) */
	bool avalPlayerChange;          /**< Available Players Changed Notification (AVRCP 1.4) */
	bool addrPlayChange;            /**< Addressed Player Changed Notification (AVRCP 1.4) */
	bool uidsChange;                /**< UIDs Changed Notification (AVRCP 1.4) */
	bool volumeChange;              /**< Notify Volume Change (AVRCP 1.4) */
} BT_AVRC_SUPPORT_NOTIFY_EVENT;

#endif /* __MODULES_INCLUDE_BLUETOOTH_HAL_BT_EVENT_H */
