/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/ble/ble_gap.h
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
 * @file       ble_gap.h
 */

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BLE_BLE_GAP_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BLE_BLE_GAP_H

/**
 * @defgroup BLE Bluetooth LE GAP
 *
 * #include <ble/ble_gap.h>
 *
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/


#include <stdint.h>

#include "ble_gatts.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup ble_defs Defines
 * @{
 */

#define BLE_GAP_ADDR_LENGTH             6 /**< BLE address length */
#define BLE_GAP_ADV_MAX_SIZE           31 /**< Maximum size of advertising data in octets */
#define BLE_GAP_SEC_KEY_LEN            16 /**< GAP Security Key Length */
#define BLE_GAP_SEC_RAND_LEN            8 /**< GAP Security Random Number Length. */
#define BLE_SAVE_BOND_DEVICE_MAX_NUM    8 /**< The maximum size of bonded devices. */
#define BLE_CONN_HANDLE_INVALID    0xFFFF /**< The invalid connection handle value. */

/**
 * @name Device Config Type
 * @{
 */
#define BLE_GAP_DEVICE_CONFIG_ADDR               1  /**< Config addr index, @ref BLE_GapAddr */
#define BLE_GAP_DEVICE_CONFIG_NAME               2  /**< Config name index, @ref BLE_GapName */
#define BLE_GAP_DEVICE_CONFIG_UUID               3  /**< Config UUID index, @ref BLE_Uuid128 */
#define BLE_GAP_DEVICE_CONFIG_SERV_CHNG_ENABLE   4  /**< Config service change enable index */
#define BLE_GAP_DEVICE_CONFIG_APPR_VALUE         5  /**< Config appearance value index, @ref BLE_GAP_APPEARANCE */
#define BLE_GAP_DEVICE_CONFIG_PPCP_VALUE         6  /**< Config PPCP value index, @ref BLE_GapConnParams*/
/** @} */

/**
 * @name Address Mode
 * @{
 */
#define BLE_GAP_ADDR_MODE_MANUAL 0 /**< Address manual mode */
#define BLE_GAP_ADDR_MODE_AUTO   1 /**< Address auto mode */
/** @} */

/**
 * @name Advertising Data Type
 * @{
 */
#define BLE_GAP_ADV_LE_LIMITED_DISC_MODE  0x01 /**< LE Limited Discoverable Mode */
#define BLE_GAP_ADV_LE_GENERAL_DISC_MODE  0x02 /**< LE General Discoverable Mode */
#define BLE_GAP_ADV_BR_EDR_NOT_SUPPORTED  0x04 /**< BR/EDR not supported */
#define BLE_GAP_ADV_LE_BR_EDR_CONTROLLER  0x08 /**< Simultaneous LE and BR/EDR, Controller */
#define BLE_GAP_ADV_LE_BR_EDR_HOST        0x10 /**< Simultaneous LE and BR/EDR, Host */
/** @} */

/**
 * @name Authentication Key Type
 * @{
 */
#define BLE_GAP_AUTH_KEY_TYPE_NONE       0x00 /**< No key */
#define BLE_GAP_AUTH_KEY_TYPE_PASSKEY    0x01 /**< 6-digit Passkey */
#define BLE_GAP_AUTH_KEY_TYPE_OOB        0x02 /**< Out Of Band data */
/** @} */

/**
 * @name Source of Timeout Event
 * @{
 */
#define BLE_GAP_TIMEOUT_ADVERTISING         0x00 /**< Advertising timeout */
#define BLE_GAP_TIMEOUT_SECURITY_REQUEST    0x01 /**< Security request timeout */
#define BLE_GAP_TIMEOUT_SCAN                0x02 /**< Scanning timeout */
#define BLE_GAP_TIMEOUT_CONN                0x03 /**< Connection timeout */
/** @} */

/**
 * @name Disconnction Reason
 * @{
 */
#define BLE_HCI_STATUS_CODE_SUCCESS                           0x00 /**< Success */
#define BLE_HCI_STATUS_CODE_UNKNOWN_BTLE_COMMAND              0x01 /**< Unknown BLE Command */
#define BLE_HCI_STATUS_CODE_UNKNOWN_CONNECTION_IDENTIFIER     0x02 /**< Unknown Connection Identifier */
#define BLE_HCI_AUTHENTICATION_FAILURE                        0x05 /**< Authentication Failure */
#define BLE_HCI_STATUS_CODE_PIN_OR_KEY_MISSING                0x06 /**< Pin or Key missing */
#define BLE_HCI_MEMORY_CAPACITY_EXCEEDED                      0x07 /**< Memory Capacity Exceeded */
#define BLE_HCI_CONNECTION_TIMEOUT                            0x08 /**< Connection Timeout */
#define BLE_HCI_STATUS_CODE_COMMAND_DISALLOWED                0x0C /**< Command Disallowed */
#define BLE_HCI_STATUS_CODE_INVALID_BTLE_COMMAND_PARAMETERS   0x12 /**< Invalid BLE Command Parameters */
#define BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION             0x13 /**< Remote User Terminated Connection */
#define BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES   0x14 /**< Remote Device Terminated Connection due to low resources */
#define BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF       0x15 /**< Remote Device Terminated Connection due to power off */
#define BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION              0x16 /**< Local Host Terminated Connection */
#define BLE_HCI_UNSUPPORTED_REMOTE_FEATURE                    0x1A /**< Unsupported Remote Feature */
#define BLE_HCI_STATUS_CODE_INVALID_LMP_PARAMETERS            0x1E /**< Invalid LMP Parameters */
#define BLE_HCI_STATUS_CODE_UNSPECIFIED_ERROR                 0x1F /**< Unspecified Error */
#define BLE_HCI_STATUS_CODE_LMP_RESPONSE_TIMEOUT              0x22 /**< LMP Response Timeout */
#define BLE_HCI_STATUS_CODE_LMP_PDU_NOT_ALLOWED               0x24 /**< LMP PDU Not Allowed */
#define BLE_HCI_INSTANT_PASSED                                0x28 /**< Instant Passed */
#define BLE_HCI_PAIRING_WITH_UNIT_KEY_UNSUPPORTED             0x29 /**< Pairing with Unit Key Unsupported */
#define BLE_HCI_DIFFERENT_TRANSACTION_COLLISION               0x2A /**< Different Transaction Collision */
#define BLE_HCI_CONTROLLER_BUSY                               0x3A /**< Controller Busy */
#define BLE_HCI_CONN_INTERVAL_UNACCEPTABLE                    0x3B /**< Connection Interval Unacceptable */
#define BLE_HCI_DIRECTED_ADVERTISER_TIMEOUT                   0x3C /**< Directed Adverisement Timeout */
#define BLE_HCI_CONN_TERMINATED_DUE_TO_MIC_FAILURE            0x3D /**< Connection Terminated due to MIC Failure */
#define BLE_HCI_CONN_FAILED_TO_BE_ESTABLISHED                 0x3E /**< Connection Failed to be Established */
/** @} */

/**
 * @name Pairing Result Status Code
 * @sa @ref BLE_GAP_EVENT_AUTH_STATUS
 * @{
 */
#define BLE_GAP_SM_STATUS_SUCCESS                0x00 /**< Procedure completed with success */
#define BLE_GAP_SM_STATUS_TIMEOUT                0x01 /**< Procedure timed out */
#define BLE_GAP_SM_STATUS_RESERVED               0x20 /**< Reserved */
#define BLE_GAP_SM_STATUS_PASSKEY_ENTRY_FAILED   0x21 /**< Passkey entry failed */
#define BLE_GAP_SM_STATUS_OOB_NOT_AVAILABLE      0x22 /**< Out of Band Key not available */
#define BLE_GAP_SM_STATUS_AUTH_REQ               0x23 /**< Authentication requirements not met */
#define BLE_GAP_SM_STATUS_CONFIRM_VALUE          0x24 /**< Confirm value failed */
#define BLE_GAP_SM_STATUS_PAIRING_NOT_SUPP       0x25 /**< Pairing not supported */
#define BLE_GAP_SM_STATUS_ENC_KEY_SIZE           0x26 /**< Encryption key size */
#define BLE_GAP_SM_STATUS_CMD_NOT_SUPPORTED      0x27 /**< Unsupported SMP command */
#define BLE_GAP_SM_STATUS_UNSPECIFIED_REASON     0x28 /**< Unspecified reason */
#define BLE_GAP_SM_STATUS_REPEATED_ATTEMPTS      0x29 /**< Too little time elapsed since last attempt */
#define BLE_GAP_SM_STATUS_INVALID_PARAMS         0x2A /**< Invalid parameters */
/** @} */

/**
 * @name Security Config Type Base
 * @{
 */
#define TYPE_SEC_CFG_BASE              0x60 /**< Define Security config type base */
/** @} */

/**
 * @name Encryption Key Size
 * @{
 */
#define BLE_GAP_MIN_KEY_SIZE            7 /**< Minimum encryption key size */
#define BLE_GAP_MAX_KEY_SIZE           16 /**< Maximum encryption key size */
/** @} */

/**
 * @name Passkey Length
 * @{
 */
#define BLE_GAP_PASSKEY_LEN            6 /**< Passkey length 6 digits */
/** @} */

/**
 * @name Passkey Length
 * @{
 */
#define BLE_DID_INFO_LEN               4 /**< BLE DID info length */
/** @} */

/**
 * @name Advertising Type
 * @{
 */
#define ADV_DATA_TYPE_FLAGS                         0x01 /**< Flags */
#define ADV_DATA_TYPE_COMPLETE_32_UUIDS             0x05 /**< Complete List of 32-bit Service Class UUIDs */
#define ADV_DATA_TYPE_COMPLETE_LOCAL_NAME           0x09 /**< Complete Local Name */
#define ADV_DATA_TYPE_TX_POWER                      0x0a /**< Tx power level */
#define ADV_DATA_TYPE_SERVICE_DATA                  0x16 /**< Service Data - 16-bit UUID */
#define ADV_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA    0xff /**< Manufacturer Specific Data */
/** @} */

/**
 * @name Tx power invalid value
 * @{
 */
#define ADV_DATA_TX_POWER_INVILID_VALUE                 127   /**< Invalid value of tx power  */
/** @} */

/**@defgroup BLE_GAP_ADDR_TYPES GAP Address types
 * @{ */
#define BLE_GAP_ADDR_TYPE_PUBLIC                        0x00 /**< Public address. */
#define BLE_GAP_ADDR_TYPE_RANDOM_STATIC                 0x01 /**< Random Static address. */
#define BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE     0x02 /**< Private Resolvable address. */
#define BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE 0x03 /**< Private Non-Resolvable address. */
/**@} */

/** @} ble_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup ble_datatypes Data types
 * @{
 */

/**@brief Passkey configure */
enum
{
  SEC_CFG_PASSKEY = TYPE_SEC_CFG_BASE, /**< Security configura type passkey */
};

/**@brief Out Of Band information*/
typedef enum
{
  BLE_GAP_OOB_AUTH_DATA_NOT_PRESENT = 0, /**< Out of band not present */
  BLE_GAP_OOB_AUTH_DATA_PRESENT,         /**< Out of band present */
} BLE_GapOOB;

/**@brief Authentication masks */
enum BLE_GapAuthMask
{
  BLE_GAP_AUTH_NONE = 0,        /**< Auth none */
  BLE_GAP_AUTH_BOND = (1 << 0), /**< Auth bond */
  BLE_GAP_AUTH_MITM = (1 << 1), /**< Auth mitm */
};

/**@brief Authentication requirements */
typedef enum
{
  BLE_GAP_AUTH_REQ_NO_MITM_NO_BOND = (BLE_GAP_AUTH_NONE), /**< No MITM No Bonding */
  BLE_GAP_AUTH_REQ_NO_MITM_BOND    = (BLE_GAP_AUTH_BOND), /**< No MITM Bonding */
  BLE_GAP_AUTH_REQ_MITM_NO_BOND    = (BLE_GAP_AUTH_MITM), /**< MITM No Bonding */
  BLE_GAP_AUTH_REQ_MITM_BOND       = (BLE_GAP_AUTH_MITM | BLE_GAP_AUTH_BOND), /**< MITM and Bonding */
} BLE_GapAuth;

/**@brief GATT appearance definitions */
typedef enum
{
  APPEARANCE_GENERIC_PHONE                    = 64,
  APPEARANCE_GENERIC_COMPUTER                 = 128,
  APPEARANCE_GENERIC_WATCH                    = 192,
  APPEARANCE_WATCH_SPORTS                     = 193,
  APPEARANCE_GENERIC_CLOCK                    = 256,
  APPEARANCE_GENERIC_DISPLAY                  = 320,
  APPEARANCE_GENERIC_REMOTE_CONTROL           = 384,
  APPEARANCE_GENERIC_EYE_GLASSES              = 448,
  APPEARANCE_GENERIC_TAG                      = 512,
  APPEARANCE_GENERIC_KEYRING                  = 576,
  APPEARANCE_GENERIC_MEDIA_PLAYER             = 640,
  APPEARANCE_GENERIC_BARCODE_SCANNER          = 704,
  APPEARANCE_GENERIC_THERMOMETER              = 768,
  APPEARANCE_THERMOMETER_EAR                  = 769,
  APPEARANCE_GENERIC_HEART_RATE_SENSOR        = 832,
  APPEARANCE_HEART_RATE_BELT                  = 833,
  APPEARANCE_GENERIC_BLOOD_PRESSURE           = 896,
  APPEARANCE_BLOOD_PRESSURE_ARM               = 897,
  APPEARANCE_BLOOD_PRESSURE_WRIST             = 898,
  APPEARANCE_GENERIC_HID_DEVICE               = 960,
  APPEARANCE_HID_KEYBOARD                     = 961,
  APPEARANCE_HID_MOUSE                        = 962,
  APPEARANCE_HID_JOYSTICK                     = 963,
  APPEARANCE_HID_GAMEPAD                      = 964,
  APPEARANCE_HID_DIGITIZER_TABLET             = 965,
  APPEARANCE_HID_CARD_READER                  = 966,
  APPEARANCE_HID_DIGITAL_PEN                  = 967,
  APPEARANCE_HID_BARCODE_SCANNER              = 968,
  APPEARANCE_GENERIC_GLUCOSE_METER            = 1024,
  APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR   = 1088,
  APPEARANCE_RUNNING_WALKING_SENSOR_IN_SHOE   = 1089,
  APPEARANCE_RUNNING_WALKING_SENSOR_ON_SHOE   = 1090,
  APPEARANCE_RUNNING_WALKING_SENSOR_ON_HIP    = 1091,
  APPEARANCE_GENERIC_CYCLING                  = 1152,
  APPEARANCE_CYCLING_COMPUTER                 = 1153,
  APPEARANCE_CYCLING_SPEED_SENSOR             = 1154,
  APPEARANCE_CYCLING_CADENCE_SENSOR           = 1155,
  APPEARANCE_CYCLING_POWER_SENSOR             = 1156,
  APPEARANCE_CYCLING_SPEED_AND_CADENCE_SENSOR = 1157,
} BLE_GAP_APPEARANCE;

/**@brief GAP set/get gap address paramters. */
typedef struct BLE_GapAddr_t
{
  uint8_t mode;                       /**< Address mode */
  uint8_t type;                       /**< Address type, See @ref BLE_GAP_ADDR_TYPES. */
  uint8_t reserve[2];                 /**< Reserve */
  uint8_t addr[BLE_GAP_ADDR_LENGTH];  /**< Address data, the size of the address is 48 bits */
} BLE_GapAddr;

/**@brief GAP set/get gap name parameters.*/
typedef struct BLE_GapName_t
{
  uint16_t size;      /**< The size of the gap name */
  uint8_t reserve[2]; /**< Reserve */
  uint8_t *name;      /**< Pointer to gap name data */
  BLE_SEC_MODE pNameWritePerm; /**< Permission of name writable  */
} BLE_GapName;

/**@brief GAP set/get device configuration information. */
typedef struct BLE_GapDeviceConfig_t
{
  uint8_t type;       /**< Types of configuration information, @ref Device Config Type */
  uint8_t reserve[3]; /**< Reserve */
  void *data;         /**< Configuration data, @ref Device Config Type */
} BLE_GapDeviceConfig;

/**@brief Connection handler */
typedef uint16_t BLE_GapConnHandle;

/**@brief Advertising structure */
typedef struct BLE_GapAdvStructure_t
{
  uint8_t    advLength;  /**< The size of the advertising structure */
  uint8_t    reserve[3]; /**< Reserve */
  uint8_t    *advData;   /**< advertising structure data */
} BLE_GapAdvStructure;

/**@brief Advertising data structure */
typedef struct BLE_GapAdvData_t
{
  uint8_t                flags;                    /**< Flags */
  int8_t                 txPower;                  /**< Range -127 ~ 127 dBm, the tx power isn't added into advertise data when value is ADV_DATA_TX_POWER_INVILID_VALUE  */
  uint32_t               complete32Uuid;           /**< Complete List of 32-bit Service Class UUIDs */
  BLE_GapAdvStructure    completeLocalName;        /**< Complete Local Name */
  BLE_GapAdvStructure    manufacturerSpecificData; /**< Manufacturer Specific Data */
  BLE_GapAdvStructure*   serviceData;              /**< Array of Service data structures. */
  uint8_t                serviceDataCount;         /**< Number of Service data structures. */
} BLE_GapAdvData;

/**@brief Advertising parameters */
typedef struct BLE_GapAdvParams_t
{
  uint8_t    reserve; /**< Reserve */
} BLE_GapAdvParams;

/**@brief Passkey configure, Config Type: @ref SEC_CFG_PASSKEY
 */
typedef struct
{
  char *passkey; /**< Pointer to passkey. The passkey is numeric value. Valid values are decimal 000000 - 999999.*/
} BLE_GapSecCfgPasskey;

/**@brief  Security configure type */
typedef struct
{
  uint32_t    type;  /**< The type of security configuration */
  void        *data; /**< Security configure data, the following data structure is supported now: @ref BLE_GapSecCfgPasskey */
} BLE_GapSecCfg;

/**@brief Pairing Feature structure */
typedef struct
{
  BLE_GAP_IO_CAP      ioCap; /**< IO capabilities */
  BLE_GapOOB            oob; /**< Out Of Band information */
  BLE_GapAuth       authReq; /**< Authentication requirements */
  uint8_t        minKeySize; /**< Minimum encryption key size in octets between 7 and 16 */
  uint8_t        maxKeySize; /**< Maximum encryption key size in octets between min_key_size and 16 */
} BLE_GapPairingFeature;

/**@brief Authentication key structure */
typedef struct
{
  BLE_GapConnHandle    handle;  /**< Connection handle */
  uint8_t              keyType; /**< Authentication key type, the following three GAP authentication key types are supported: @ref BLE_GAP_AUTH_KEY_TYPE_NONE, @ref BLE_GAP_AUTH_KEY_TYPE_PASSKEY, @ref BLE_GAP_AUTH_KEY_TYPE_OOB */
  uint8_t              keyLen;  /**< The size of authentication key */
  uint8_t              *key;    /**< Authentication key */
} BLE_GapAuthKey;

/**@brief Bond information. */
typedef struct
{
  uint8_t addrType;                    /**< See @ref BLE_GAP_ADDR_TYPES. */
  uint8_t addr[BLE_GAP_ADDR_LENGTH];   /**< 48-bit address, LSB format. */
} BLE_GapBondInfo;

/**@brief Bonded information list structure. */
typedef struct
{
  uint32_t bondNum;
  uint8_t bondInfoId[BLE_SAVE_BOND_DEVICE_MAX_NUM][BLE_GAP_ADDR_LENGTH];
  uint8_t didInfo[BLE_SAVE_BOND_DEVICE_MAX_NUM][BLE_DID_INFO_LEN];     /**< BLE DID information, please ref@ BT_DID_VendorInfo*/
} BLE_GapBondInfoList;

/**@brief Connection parameters */
typedef struct
{
  uint16_t minConnInterval; /**< Minimum Connection Interval in 1.25 ms units. (7.5ms - 4s) */
  uint16_t maxConnInterval; /**< Maximum Connection Interval in 1.25 ms units. (7.5ms - 4s) */
  uint16_t slaveLatency;    /**< Slave Latency in number of connection events. (max 499) */
  uint16_t connSupTimeout;  /**< Connection Supervision Timeout in 10 ms unit. (100ms - 32s) */
} BLE_GapConnParams;

/**@brief Connection established
 * @details  When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GAP_EVENT_CONNECTED, and the member evtData is point to @ref BLE_EvtGapConnected_t structure.
 */
typedef struct BLE_EvtGapConnected_t
{
  BLE_GapConnHandle    handle; /**< Connection handle */
  BLE_GapAddr          addr;   /**< Bluetooth address of the peer device */
  uint8_t              role;   /**< BLE role for this connection */
} BLE_EvtConnected;

/**@brief Connection parameters updated
 * @details  When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GAP_EVENT_CONN_PARAM_UPDATE, and the member evtData is point to @ref BLE_EvtConnParamUpdate structure.
 */
typedef struct
{
  uint8_t              status;     /**< Connection parameters update status */
  BLE_GapConnParams    connParams; /**< Connection parameters */
} BLE_EvtConnParamUpdate;

/**@brief peer device MTU event,
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GAP_EVENT_CONN_PEER_MTU, and the member evtData is point to @ref BLE_EvtPeerMtu structure.
 */
typedef struct
{
  BLE_GapConnHandle        handle;      /**< Connection handle */
  uint16_t                 mtu;         /**< BLE Maximum Transmission Unit */
} BLE_EvtPeerMtu;

/**@brief Disconnected from peer
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GAP_EVENT_DISCONNECTED, and the member evtData is point to @ref BLE_EvtGapDisconnected_t structure.
 */
typedef struct BLE_EvtGapDisconnected_t
{
  BLE_GapConnHandle    handle; /**< Connection handle */
  uint8_t              reason; /**< Disconnected reason */
} BLE_EvtDisconnected;

/**@brief Exchange pairing feature event, it contains peer device’s pairing feature.
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GAP_EVENT_EXCHANGE_FEATURE, and the member evtData is point to @ref BLE_EvtExchangeFeature structure.
 */
typedef struct
{
  BLE_GapConnHandle        handle;      /**< Connection handle */
  BLE_GapPairingFeature    peerFeature; /**< Pairing feature */
} BLE_EvtExchangeFeature;

/**@brief Pairing result event.
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GAP_EVENT_AUTH_STATUS, and the member evtData is point to @ref BLE_EvtAuthStatus structure.
 */
typedef struct
{
  BLE_GapConnHandle        handle;      /**< Connection handle */
  uint8_t                  status;      /**< Pairing result event */
  uint8_t                  reserve;     /**< Reserver */
  BLE_GapBondInfo          bondInfo;    /**< Bond information */
} BLE_EvtAuthStatus;

/**@brief Display passkey event structure
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GAP_EVENT_DISPLAY_PASSKEY, and the member evtData is point to @ref BLE_EvtDisplayPasskey structure.
 */
typedef struct
{
  BLE_GapConnHandle        handle;                         /**< Connection handle */
  uint8_t                  passkey[BLE_GAP_PASSKEY_LEN+1]; /**< 6-digit passkey ASCII 000000~999999 + 0x00 */
} BLE_EvtDisplayPasskey;

/**@brief Advertising report event structure
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GAP_EVENT_ADV_REPORT, and the member evtData is point to @ref BLE_EvtAdvReportData structure.
 */
typedef struct
{
  int8_t rssi;                        /**< Received Signal Strength Indication in dBm. */
  uint8_t dlen;                       /**< Scan response data length */
  uint8_t data[BLE_GAP_ADV_MAX_SIZE]; /**< Scan response data */
  BLE_GapAddr addr;                   /**< Bluetooth address of the peer device */
} BLE_EvtAdvReportData;

/**@brief Request to provide an authentication key event structure
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GAP_EVENT_AUTH_KEY_REQUEST, and the member evtData is point to @ref BLE_EvtAuthKey structure.
 */
typedef struct
{
  BLE_GapConnHandle        handle;      /**< Connection handle */
  uint8_t                  keyType;     /**< GAP authentication key types */
  uint8_t                  reserve;     /**< Reserver */
} BLE_EvtAuthKey;

/**@brief Event structure for @ref BLE_GAP_EVENT_TIMEOUT
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GAP_EVENT_TIMEOUT, and the member evtData is point to @ref BLE_EvtTimeout structure.
 */
typedef struct
{
  BLE_GapConnHandle        handle;      /**< Connection handle */
  uint8_t                  timeoutSrc;  /**< Timeout reason, The following four timeout reasons are supported: @ref BLE_GAP_TIMEOUT_ADVERTISING, @ref BLE_GAP_TIMEOUT_SECURITY_REQUEST, @ref BLE_GAP_TIMEOUT_SCAN, @ref BLE_GAP_TIMEOUT_CONN */
  uint8_t                  reserve;     /**< Reserver */
} BLE_EvtTimeout;

/** @} ble_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup ble_funcs Functions
 * @{
 */

/**@brief Get device configuration information
 * @details This call allows the application to get the device information.
 *
 * @param[out]  deviceConfig: Pointer to device config structure to be filled in, UUID type is not supported.
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapGetDeviceConfig(BLE_GapDeviceConfig *deviceConfig);

/**@brief Set device configuration information.
 * @details This call allows the application to set device configuration information.
 *
 * @param[in]  deviceConfig: Pointer to config structure
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapSetDeviceConfig(BLE_GapDeviceConfig *deviceConfig);

/**@brief  Set, clear or update advertising data
 * @details  This call allows the application to set, clear or update advertising data.
 *           Invalid data is not set or updated.
 *
 * @param[in]  advData: Raw data to be placed in advertising packet
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 * @note  This function should be called before @ref BLE_GapStartAdv().
 */
int BLE_GapSetAdvData(BLE_GapAdvData *advData);

/**@brief Start advertising
 * @details This call allows the application to start advertising, must be called after @ref BLE_GapSetAdvData(), when first call this function. The advertisement timeout interval is 180 seconds.
 *
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapStartAdv(void);

/**@brief Stop advertising
 * @details This call allows the application to stop advertising (GAP discoverable, connectable modes, broadcast procedure).
 *
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapStopAdv(void);

/**@brief Disconnect(GAP link termination)
 * @details This call initiates the disconnection procedure. The following events may be triggered: @ref BLE_GAP_EVENT_DISCONNECTED.
 * @param[in]  connHandle: Connection handle
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapDisconnectLink(BLE_GapConnHandle connHandle);

/**@brief Set Security Configure Parameter
 *
 * @param[in]  param: Security related configure parameter
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapSetSecParam(BLE_GapSecCfg *param);

/**@brief    Exchange pairing feature
 * @details This call exchanges the pairing feature between pairing initiator and responder.The following events may be triggered: @ref BLE_GAP_EVENT_DISPLAY_PASSKEY, @ref BLE_GAP_EVENT_AUTH_STATUS.
 *
 * @param[in]  connHandle: Connection handle
 * @param[in]  pairingFeature: Pairing feature exchanged in pairing phase 1
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapExchangePairingFeature(BLE_GapConnHandle connHandle, BLE_GapPairingFeature *pairingFeature);

/**@brief    Start scanning
 * @details This call allows the application to start scanning. The scanning timeout interval is 60 seconds.The following events may be triggered: @ref BLE_GAP_EVENT_TIMEOUT, @ref BLE_GAP_EVENT_ADV_REPORT.
 *
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapStartScan(void);

/**@brief    Stop scanning
 *
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapStopScan(void);

/**@brief    Create a connection
 * @details  This call allows the application to create a connection. The following events may be triggered: @ref BLE_GAP_EVENT_CONNECTED.
 * @param[in]  addr: Pointer to peer address
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapConnect(BLE_GapAddr *addr);

/**@brief    Update connection paramters
 * @details  This call allows to update connection parameters. The following events may be triggered: @ref BLE_GAP_EVENT_CONNECTED.
 * @param[in]  connHandle: Connection handle
 * @param[in]  connParam: Pointer to connection parameters
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapUpdateConnectionParams(BLE_GapConnHandle connHandle, BLE_GapConnParams *connParams);

/**@brief     Cancel a connection establishment
 *
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapCancelConnecting(void);

/**@brief Initiate the GAP authentication procedure
 * @details This call allows the application to send a SMP Pairing Request.The following events may be triggered: @ref BLE_GAP_EVENT_EXCHANGE_FEATURE.
 *
 * @param[in]  connHandle: Connection handle
 * @param[in]  pairingFeature: Pointer to authenticate parameters
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapAuthenticate(BLE_GapConnHandle connHandle, BLE_GapPairingFeature *pairingFeature);

/**@brief    Reply with GAP security parameters
 * @details This call allows the application to reply security parameters. The following events may be triggered: @ref BLE_GAP_EVENT_AUTH_KEY_REQUEST, @ref BLE_GAP_EVENT_CONN_SEC_UPDATE, @ref BLE_GAP_EVENT_AUTH_STATUS.
 *
 * @param[in]  connHandle: Connection handle
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapReplyPairingFeature(BLE_GapConnHandle connHandle);

/**@brief Reply with an authentication key
 * @details This call allows the application to reply with an authentication key.The following events may be triggered: @ref BLE_GAP_EVENT_AUTH_KEY_REQUEST, @ref BLE_GAP_EVENT_CONN_SEC_UPDATE, @ref BLE_GAP_EVENT_AUTH_STATUS. User must make sure that the value of keyType in param--authKey is the same with the value which is passed by event handler when event--BLE_GAP_EVNET_AUTH_KEY_REQUEST happens.
 *
 * @param[in]  authKey: Pointer to authentication key data
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapReplyAuthKey(BLE_GapAuthKey *authKey);

/**@brief Save the boned peer device information
 * @details This call allows the application to save the bonded information of peer device.
 *
 * @param[in]  info: Bonded information
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapSaveBondInfo(BLE_GapBondInfo *info);

/**@brief Clear the boned peer device information
 * @details This call allows the application to clear the bonded information of peer device.
 *
 * @param[in]  info: Bonded information, must be existing bond info.
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapClearBondInfo(BLE_GapBondInfo *info);

/**@brief Get bond information ID list
 * @details This call allows the application to get bond information ID list.
 *
 * @param[out]  bondInfo: Bonded information list structure
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapGetBondInfoIdList(BLE_GapBondInfoList *bondInfo);

/**@brief Encrypt connection link
 * @details This call allows the application to encrypt link.The following events may be triggered: @ref BLE_GAP_EVENT_CONN_SEC_UPDATE, @ref BLE_GAP_EVENT_DISCONNECTED. Before calling this API, user must make sure that the key value, which generated by BLE_GapSaveBondInfo, have been stored in flash.
 *
 * @param[in]  connHandle: Connection handle
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GapEncrypt(BLE_GapConnHandle connHandle);

/** @} ble_funcs */

/** @} ble_gap */
#endif  /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BLE_BLE_GAP_H */
