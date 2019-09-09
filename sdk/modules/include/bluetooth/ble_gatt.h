/****************************************************************************
 * modules/include/bluetooth/ble_gatt.h
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
 * @file ble_gatt.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief Bluetooth Low Energy GATT Server/Client API.
 * @details This API is for using BLE GATT and includes Function and Callback
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_BLE_GATT_H
#define __MODULES_INCLUDE_BLUETOOTH_BLE_GATT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bt_common.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 *@name Max number of services
 *@{
 */
#define BLE_MAX_SERVICES 1
/** @} */

/**
 *@name Max number of characteristics
 *@{
 */
#define BLE_MAX_CHARACTERISTICS 1
/** @} */

/**
 *@name Max size of characteristics value
 *@{
 */
#define BLE_MAX_CHAR_SIZE 20
/** @} */

/**
 *@name Invalid service handle ID
 *@{
 */
#define BLE_GATT_INVALID_SERVICE_HANDLE UINT16_MAX
/** @} */

/**
 *@name Support Max services
 *@{
 */
#define BLE_DB_DISCOVERY_MAX_SRV          3
/** @} */

/**
 *@name Support Max characteristics per service
 *@{
 */
#define BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV 4
/** @} */

/****************************************************************************
 * Public Types prototype
 ****************************************************************************/

struct ble_gatt_char_s;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @enum BLE_GATT_UUID_TYPE
 * @brief GATTS UUID type
 */
typedef enum
{
  BLE_UUID_TYPE_UUID128 = 0,         /**< UUID type 128-bit UUID */
  BLE_UUID_TYPE_BASEALIAS_BTSIG,     /**< UUID type base alias bluetooth SIG qualified */
  BLE_UUID_TYPE_BASEALIAS_VENDOR,    /**< UUID type base alias vendor */
} BLE_GATT_UUID_TYPE;

/**
 * @enum BLE_GATT_TYPE
 * @brief GATTS service type
 */
typedef enum
{
  BLE_GATTS_SRVTYP_PRIMARY = 1, /**< GATTS service type primary */
  BLE_GATTS_SRVTYP_SECONDARY,   /**< GATTS service type secondary */
} BLE_GATT_TYPE;

/**
 * @struct BLE_SEC_MODE
 * @brief BLE security mode security level.
 * @details According to ble core spec v4.1 Vol 3;Part C;10.2 LE security modes and Vol 3;Part F;3.2.5 attribute permissions.
 */
typedef enum{
  BLE_SEC_MODE_NO_ACCESS  = 0x00,    /**< No access rigths */
  BLE_SEC_MODE1LV1_NO_SEC,           /**< Security mode 1 level 1, no security open link */
  BLE_SEC_MODE1LV2_NO_MITM_ENC,      /**< Security mode 1 level 2, unauthenticated pairing with encryption */
  BLE_SEC_MODE1LV3_MITM_ENC,         /**< Security mode 1 level 3, authenticated pairing with encryption */
  BLE_SEC_MODE2LV1_NO_MITM_DATA_SGN, /**< Security mode 2 level 1, unauthenticated pairing with data signing, not supported now */
  BLE_SEC_MODE2LV2__MITM_DATA_SGN,   /**< Security mode 2 level 2, authenticated pairing with data signing, not supported now */
} BLE_SEC_MODE;

/**
 * @struct BLE_UUID128
 * @brief 128-bit UUID structure for BLE
 */
typedef struct
{
  uint8_t uuid128[BT_UUID128_LEN]; /**< 128-bit UUID */
} BLE_UUID128;

/**
 * @struct BLE_UUID_ALIAS
 * @brief UUID base + alias structure
 */
typedef struct
{
  BLE_UUID128 uuidBase;  /**< 128-bit UUID */
  uint16_t    uuidAlias; /**< UUID alias */
} BLE_UUID_ALIAS;

/**
 * @struct BLE_UUID
 * @brief BLE UUID structure
 */
typedef struct
{
  BLE_GATT_UUID_TYPE  type;      /**< UUID type @ref BLE_GATT_UUID_TYPE */
  union
  {
    BLE_UUID128       uuid128;   /**< 128-bit UUID @ref BLE_UUID128 */
    BLE_UUID_ALIAS    alias;     /**< UUID base alias @ref BLE_UUID_ALIAS */
  }                   value;     /**< value union */
} BLE_UUID;

/**
 * @struct BLE_ATTR_PERM
 * @brief Attribute permission structure
 */
typedef struct
{
  BLE_SEC_MODE  readPerm; /**< Read permissions. */
  BLE_SEC_MODE writePerm; /**< Write permissions. */
} BLE_ATTR_PERM;

/**
 * @struct BLE_CHAR_VALUE
 * @brief characteristic value attribute structure
 */
typedef struct
{
  BLE_ATTR_PERM             attrPerm; /**< Attribute permission structure */
  uint8_t                   *data;    /**< Pointer to attribute value(Need to allocate BLE_MAX_CHAR_SIZE size in application) */
  uint16_t                  length;   /**< Attribute value length */
} BLE_CHAR_VALUE;

/**
 * @struct BLE_CHAR_PROP
 * @brief Characteristic standard properties
 * @details According to ble core spec 4.1 vol3;Part G 3.3.1.1, Current version support read, write, notify properties.
 */
typedef struct
{
  uint8_t broadcast       :1; /**< Broadcasting of the value permitted. */
  uint8_t read            :1; /**< Reading the value permitted. */
  uint8_t writeWoResp     :1; /**< Writing the value with Write Command permitted. */
  uint8_t write           :1; /**< Writing the value with Write Request permitted. */
  uint8_t notify          :1; /**< Notification of the value permitted. */
  uint8_t indicate        :1; /**< Indication of the value permitted. */
  uint8_t authSignedWr    :1; /**< Writing the value with Signed Write Command permitted. */
  uint8_t reserve         :1; /**< Reserve */
} BLE_CHAR_PROP;

/**
 * @struct ble_gatt_peripheral_ops_s
 * @brief Bluetooth LE characteristic callbacks(for Peripheral)
 * @details If characteristic get event from target, this callback will
 *         call for request(Peripheral).
 */
struct ble_gatt_peripheral_ops_s
{
  void (*write)(struct ble_gatt_char_s *ble_gatt_char);               /**< Write request */
  void (*read)(struct ble_gatt_char_s *ble_gatt_char);                /**< Read request */
  void (*notify)(struct ble_gatt_char_s *ble_gatt_char, bool enable); /**< Notify request */
};

/**
 * @struct ble_gatt_char_s
 * @brief Bluetooth LE GATT characteristic context
 */
struct ble_gatt_char_s
{
  uint16_t              handle;        /**< Characteristic handle id */
  BLE_UUID              uuid;          /**< Characteristic UUID */
  BLE_CHAR_VALUE        value;         /**< Characteristic value */
  BLE_CHAR_PROP         property;      /**< Characteristic property */
  uint8_t               status;        /**< Characteristic write response status */
  union
  {
    struct ble_gatt_central_ops_s    *ble_gatt_central_ops;    /**< Central role application callbacks @ref ble_gatt_central_ops_s */
    struct ble_gatt_peripheral_ops_s *ble_gatt_peripheral_ops; /**< Peripheral role application callbacks @ref ble_gatt_peripheral_ops_s */
  };
};

/**
 * @struct ble_gatt_service_s
 * @brief Bluetooth LE GATT service context
 */
struct ble_gatt_service_s
{
  uint16_t               handle;                          /**< Service handle id */
  BLE_UUID               uuid;                            /**< Service UUID */
  BLE_GATT_TYPE          type;                            /**< Service type */
  uint8_t                num;                             /**< Number of characteristics */
  struct ble_gatt_char_s *chars[BLE_MAX_CHARACTERISTICS]; /**< Characteristic list @ref ble_gatt_char_s */
};

/**
 * @struct ble_gatt_state_s
 * @brief Bluetooth LE GATT context
 */
struct ble_gatt_state_s
{
  struct ble_state_s        *ble_state;                 /**< Bluetooth LE context @ref ble_state_s */
  struct ble_hal_gatt_ops_s *ble_hal_gatt_ops;          /**< BLE GATT HAL interfaces */
  uint8_t                   num;                        /**< Number of services */
  struct ble_gatt_service_s services[BLE_MAX_SERVICES]; /**< Service list @ref ble_gatt_service_s */
  /* TODO: temporary, needs to consider the design */
  struct ble_gatt_central_ops_s *ble_gatt_central_ops;  /**< Central role application callbacks @ref ble_gatt_central_ops_s */
};

/**
 * @struct ble_gattc_handle_range_s
 * @brief GATTC handle range structure
 */
struct ble_gattc_handle_range_s
{
  uint16_t start_handle; /**< Start handle */
  uint16_t end_handle;   /**< End handle */
};

/**
 * @struct ble_gattc_char_s
 * @brief GATTC characteristic structure
 */
struct ble_gattc_char_s
{
  BLE_CHAR_PROP char_prope;      /**< Characteristic property */
  uint16_t      char_valhandle;  /**< Characteristic value handle */
  uint16_t      char_declhandle; /**< Characteristic declaration handle */
  BLE_UUID      char_valuuid;    /**< Characteristic value uuid */
};

/**
 * @struct ble_gattc_db_disc_char_s
 * @brief GATTC discovered characteristic data structure
 */
struct ble_gattc_db_disc_char_s
{
  uint16_t                cccd_handle;    /**< Handle of client configuration characteristic descriptor in characteristic */
  struct ble_gattc_char_s characteristic; /**< Characteristic information */
};

/**
 * @struct ble_gattc_db_disc_srv_s
 * @brief GATTC discovered service data structure
 */
struct ble_gattc_db_disc_srv_s
{
  uint8_t                         char_count;                                         /**< Discovered characteristic count in service */
  struct ble_gattc_handle_range_s srv_handle_range;                                   /**< The handle range of this service */
  struct ble_gattc_db_disc_char_s characteristics[BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV]; /**< Discovered characteristics in this service */
  BLE_UUID                        srv_uuid;                                           /**< UUID of discovered service */
};

/**
 * @struct ble_gattc_db_discovery_s
 * @brief GATTC discovered attribute database data structure
 */
struct ble_gattc_db_discovery_s
{
  uint8_t                        srv_count;                           /**< Discovered services count */
  uint16_t                       conn_handle;                         /**< Connection handle */
  struct ble_gattc_db_disc_srv_s services[BLE_DB_DISCOVERY_MAX_SRV];  /**< Discovered services in attribute database */
};

/**
 * @struct ble_gattc_overrun_state_s
 * @brief GATTC discover overrun state
 */
struct ble_gatt_coverrun_state_s
{
  uint8_t  srv_count;  /**< the Max service count of peer device */
  uint16_t end_handle; /**< the end handle of the last discorver service */
};

/**
 * @struct ble_gatt_event_db_discovery_t
 * @brief GATTC attribute database discovery event structure
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GATTC_EVENT_DBDISCOVERY, and the member evtData is point to @ref BLE_EvtGattcDbDiscovery structure.
 */
struct ble_gatt_event_db_discovery_t
{
  uint8_t                           group_id;     /**< Event group ID @ref BT_GROUP_ID */
  uint8_t                           event_id;     /**< Event sub ID */
  uint8_t                           result;       /**< GATTC discovery result, @ref GATT Client DB Discovery Result Code */
  uint16_t                          conn_handle;  /**< Connection handle */
  struct ble_gatt_coverrun_state_s  state;        /**< When the service count of peer device is over the supporter MAX services */
  union{
    uint32_t                        reason;       /**< Indicate db discovery failed reason */
    struct ble_gattc_db_discovery_s db_discovery; /**< Discovered db information */
  } params;                                       /**< params union */
};

/**
 * @struct ble_gatt_central_ops_s
 * @brief Bluetooth LE characteristic callbacks(for Central)
 * @details If characteristic get event from target, this callback will
 *         call for response(Central).
 */
struct ble_gatt_central_ops_s
{
  void (*write)(struct ble_gatt_char_s *ble_gatt_char);                      /**< Write response */
  void (*read)(struct ble_gatt_char_s *ble_gatt_char);                       /**< Read response */
  void (*notify)(struct ble_gatt_char_s *ble_gatt_char);                     /**< Notify response */
  void (*database_discovery)(struct ble_gatt_event_db_discovery_t *db_disc); /**< Database discovery event */
};



/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Get Bluetooth Low Ennergy GATT support or not support
 *
 * @retval Suppot or Not support
 */

bool ble_gatt_is_supported(void);

/**
 * @brief BLE Create GATT Service
 *        Create GATT Service instance and return instance pointer via *service
 *
 * @param[out] service: Bluetooth LE service context @ref ble_gatt_service_s
 *
 * @retval error code
 */

int ble_create_service(struct ble_gatt_service_s **service);

/**
 * @brief BLE Register GATT Service
 *        Register GATT Service to HAL.
 *
 * @param[in] service: Bluetooth LE GATT service context @ref ble_gatt_service_s
 *
 * @retval error code
 */

int ble_register_servce(struct ble_gatt_service_s *service);

/**
 * @brief BLE add Characteristic to service
 *        Add characteristic to service
 *
 * @param[in] service: Bluetooth LE GATT service context @ref ble_gatt_service_s
 * @param[in] charc: Bluetooth LE GATT characteristic context @ref ble_gatt_char_s
 *
 * @retval error code
 */

int ble_add_characteristic(struct ble_gatt_service_s *service, struct ble_gatt_char_s *charc);

/**
 * @brief BLE Notify Characteristic value
 *        Notify characteristic value to Central (For Peripheral role)
 *
 * @param[in] charc: Target characteristic @ref ble_gatt_char_s
 * @param[in] data: Notify data
 * @param[in] len: Notify data length
 *
 * @retval error code
 */

int ble_characteristic_notify(struct ble_gatt_char_s *charc, uint8_t *data, int len);

/**
 * @brief BLE Read Characteristic value
 *        Send read characteristic request to peripheral (For Central role)
 *
 * @param[in] charc: Bluetooth LE GATT characteristic context @ref ble_gatt_char_s
 *
 * @retval error code
 */

int ble_characteristic_read(struct ble_gatt_char_s *charc);

/**
 * @brief BLE Write Characteristic value
 *        Send write characteristic request to peripheral (For Central role)
 *
 * @param[in] charc: Bluetooth LE GATT characteristic context @ref ble_gatt_char_s
 * @param[in] data: Write data
 * @param[in] len: Write data length
 *
 * @retval error code
 */

int ble_characteristic_write(struct ble_gatt_char_s *charc, uint8_t *data, int len);

/**
 * @brief BLE start database discovery
 *        Send database discovery request to peripheral (For Central role)
 *
 * @param[in] conn_handle: Bluetooth LE GATT connection handle
 *
 * @retval error code
 */

int ble_start_db_discovery(uint16_t conn_handle);

/**
 * @brief BLE continue database discovery
 *        Send continue database discovery request to peripheral (For Central role)
 *
 * @param[in] start_handle: Bluetooth LE GATT start handle
 * @param[in] conn_handle: Bluetooth LE GATT connection handle
 *
 * @retval error code
 */

int ble_continue_db_discovery(uint16_t start_handle, uint16_t conn_handle);

#endif /* __MODULES_INCLUDE_BLUETOOTH_BLE_GATT_H */
