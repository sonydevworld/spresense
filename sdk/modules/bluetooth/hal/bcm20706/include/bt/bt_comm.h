/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/bt/bt_comm.h
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
 * @file       bt_comm.h
 */

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_COMM_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_COMM_H

/**
 * @defgroup BT
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup bt_defs Defines
 * @{
 */

/**
 *@name BT firmware
 *@{
 */
#define FIRMWARE_NAME_BT                  "bcm20706fw"     /**< Firmware name for A2DP SNK, AVRCP CT/TG, HFP HF/AG, SPP, BLE*/
#define FIRMWARE_NAME_BT_EXT              "bcm20706fw_src" /**< Firmware name for A2DP SRC, AVRCP CT/TG, HFP HF/AG, SPP, BLE*/
/** @} */

/**
 *@name BT DID information length
 *@{
 */
#define BT_DID_INFO_LEN   4
/** @} */

/**
 *@name BT save bond device max num
 *@{
 */
#define BT_SAVE_BOND_DEVICE_MAX_NUM 8
/** @} */

/**
 *@name BT Name Length
 *@{
 */
#define BT_MAX_NAME_LEN    28
/** @} */

/**
 *@name BT Class Length
 *@{
 */
#define BT_CLASS_LEN (3)
/** @} */

/**
 * @name Device Event Group Type
 * @{
 */
#define BT_CONTROL_GROUP_DEVICE                         0x00    /**< General control of BCM20706 management and Bluetooth functionality*/
#define BT_CONTROL_GROUP_LE                             0x01    /**< LE device-related commands*/
#define BT_CONTROL_GROUP_GATT                           0x02    /**< GATT commands*/
#define BT_CONTROL_GROUP_HF                             0x03    /**< Hands-free profile HF commands*/
#define BT_CONTROL_GROUP_SPP                            0x04    /**< Serial port profile commands*/
#define BT_CONTROL_GROUP_A2DP_SRC                       0x05    /**< Audio/video (AV) commands*/
#define BT_CONTROL_GROUP_HIDD                           0x06    /**< HID device (HIDD) commands*/
#define BT_CONTROL_GROUP_AVRC_TARGET                    0x07    /**< AV remote control (AVRC) target commands*/
#define BT_CONTROL_GROUP_AG                             0x0e    /**< Hands-free profile AG commands*/
#define BT_CONTROL_GROUP_AVRC_CONTROLLER                0x11    /**< AV remote control (AVRC) controller commands*/
#define BT_CONTROL_GROUP_A2DP_SINK                      0x13    /**< a2dp sink commands*/
#define BT_CONTROL_GROUP_AUDIO_SINK                     0x14    /**< audio sink commands*/
/** @} */

/**
 * @name Device Group Event Opcode Type
 * @{
 */
#define BT_EVT_DEV_COMMAND_STATUS                       0x01  /**< Command status event for the requested operation */
#define BT_EVT_DEV_BOND_INFO                            0x04  /**< Request to MCU to save bond info */
#define BT_EVT_DEV_INQUIRY_RESULT                       0x06  /**< Inquiry result */
#define BT_EVT_DEV_INQUIRY_COMPLETE                     0x07  /**< Inquiry completed event */
#define BT_EVT_DEV_PAIRING_COMPLETE                     0x08  /**< Pairing Completed */
#define BT_EVT_DEV_ENCRYPTION_CHANGED                   0x09  /**< Encryption changed event */
#define BT_EVT_DEV_CONNECTED_DEVICE_NAME                0x0A  /**< Remote connected device name */
#define BT_EVT_DEV_CONFIRMATION_REQUEST                 0x0B  /**< Confirmation request during pairing */
#define BT_EVT_DEV_PASSKEY_REQUEST                      0x17  /**< Passkey request  during pairing */
#define BT_EVT_DEV_PMGR_STATUS                          0x20  /**< power manager status and value */
#define BT_EVT_DEV_BAUDRATE_RESPONSE                    0x21  /**< baud rate response */
#define BT_EVT_DEV_I2S_ROLE_RESPONSE                    0x22  /**< i2s role response */
#define BT_EVT_DEV_REPLY_VENDORID                       0x23  /**< reply vendor id event */
#define BT_EVT_DEV_REPLY_BT_VERSION                     0x24  /**< reply the bt version */
#define BT_EVT_DEV_ACL_CONNECTION_STATUS                0x27  /**< Acl connection/disconnection status */
/** @} */

/**
 * @name BT/BLE transport type
 * @{
 */
#define BT_TRANSPORT_BR_EDR  1
#define BT_TRANSPORT_LE      2
/** @} */

/**
 * @name Acl disconnection type
 * @{
 */
#define BT_CONNECTION_TIMEOUT            0x08           /**< BT disconnection because of time out */
#define BT_REMOTE_USER_TERM_CONN         0x13           /**< Remote user terminate the connection */
#define BT_LOCAL_HOST_TERM_CONN          0x16           /**< Local host terminate the connection */
/** @} */

/**
 * @name Event Data length
 * @{
 */
#define BT_EVT_DATA_LEN 1000 /**< BT event data max length */
#define BT_EVT_EIR_LEN (BT_EVT_DATA_LEN - 30)
#define BT_EVT_AUDIO_DATA_LEN (BT_EVT_DATA_LEN - 10)
/** @} */

/** @} bt_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/**@brief BT initialize parameter
 */
typedef struct
{
  int reverse;
} BT_INIT_PARAM;

/**@brief BT Class structure
 */
typedef struct
{
  uint8_t btClass[BT_CLASS_LEN];
} BT_CLASS;

typedef enum
{
  BT_IO_NO_INPUT_NO_OUTPUT = 0,
  BT_IO_INPUT_ONLY,
  BT_IO_OUTPUT_ONLY,
  BT_INPUT_OUTPUT,
} BT_IO_CAP;

/**@brief BT visibility param structure
 */
typedef struct
{
  uint16_t discWindow;       /**<inquiry scan window, range 0x0012 ~ 0x1000, default 0x12, invalid if out of range*/
  uint16_t discInterval;     /**<inquiry scan internal, range 0x0012 ~ 0x1000, default 0x800, invalid if out of range*/
  uint16_t connWindow;       /**<page scan window, range 0x0012 ~ 0x1000, default 0x12, invalid if out of range */
  uint16_t connInterval;     /**<page scan internal, range 0x0012 ~ 0x1000, default 0x800, invalid if out of range*/
} BT_VISIBILITY_PARAM;

/**@brief BT sniff mode param structure
 */
typedef struct
{
  uint16_t minPeriod;     /**<min sniff mode internal, range 100 ~ 500, default 500, invalid if out of range, disable automatic sniff mode if value = 0*/
  uint16_t maxPeriod;     /**<max sniff mode internal, range 100 ~ 500, default 500, invalid if out of range, disable automatic sniff mode if value = 0*/
  uint16_t attempt;       /**<Number of attempts for switch to sniff mode, range 1 ~ 5, default 4, invalid if out of range , disable automatic sniff mode if value = 0*/
  uint16_t timeout;       /**<Timeout for attempting to switch to sniff mode, range 0 ~5, default 0, invalid if out of range*/
} BT_SNIFF_PARAM;

/**@brief BT bool type
 */
typedef enum
{
  BT_FALSE = 0,
  BT_TRUE = 1,
} BT_BOOL;

/**@brief Power Manager status codes
 */
typedef enum
{
  BT_POWER_STATE_ACTIVE      = 0,                            /**< Active */
  BT_POWER_STATE_SNIFF       = 2,                            /**< Sniff */
  BT_POWER_STATE_SSR         = BT_POWER_STATE_SNIFF + 2,     /**< Sniff subrating notification */
  BT_POWER_STATE_PENDING,                                    /**< Pending (waiting for status from controller) */
  BT_POWER_STATE_ERROR,                                      /**< Error (controller returned error) */
  BT_POWER_STATE_SMART_SNIFF                                 /**< Smart Sniff */
} BT_POWER_STATE;

/**@brief BT PCM I2S role
 */
typedef enum
{
  BT_PCM_I2S_MASTER = 0, /**< PCM I2S as MASTER */
  BT_PCM_I2S_SLAVE,      /**< PCM I2S as SLAVE */
}BT_PCM_I2S_ROLE;

/**@brief BT boot firmware information
 */
typedef struct
{
  uint8_t *bin;
  uint32_t binsize;
  const char *fileName;
} BT_FIRMWARE_INFO;

/**@brief BT extended inquiry data
 */
typedef struct
{
  int8_t txPower;   /**< range -127 ~ 127 dBm, invalid out of range*/
} BT_EXT_INQR_DATA;

/**@brief BT pair result
 */
typedef struct
{
  BT_PAIR_STATUS btPairStatus;
  BT_ADDR btAddr;
} BT_PAIR_RESULT;

/**@brief BT reply confirmation structure
 */
typedef struct
{
  BT_ADDR addr;       /**< BT addresss */
  BT_BOOL btAccept;   /**< BT_FALSE reject pairing, BT_TRUE accept pairing*/
} BT_REPLY_CONFIRM;

/**@brief BT reply passkey structure
 */
typedef struct
{
  BT_ADDR addr;       /**< BT addresss */
  BT_BOOL btAccept;   /**< BT_FALSE reject pairing, BT_TRUE accept pairing*/
  uint32_t passKey;   /**< BT passKey*/
} BT_REPLY_PASSKEY;

/**@brief BT event structure
 */
typedef struct
{
  uint8_t opcode;
  uint8_t group;
  uint8_t evtData[BT_EVT_DATA_LEN];
  void    *data; /**< Event callback data. This field is totally user defined. For example you can define data as your application state. */
} BT_EVT;

/**@brief BT Rf test event structure
 */
typedef struct
{
  uint8_t header;
  uint8_t len;
  uint8_t evtData[BT_EVT_DATA_LEN];
  void    *data; /**< Event callback data. This field is totally user defined. For example you can define data as your application state. */
} BT_RF_TEST_EVT;

/**@brief BT session event structure
 */
typedef struct
{
  uint8_t opcode;
  uint8_t reserved;
  uint8_t evtData[BT_EVT_DATA_LEN];
  void    *data; /**< Event callback data. This field is totally user defined. For example you can define data as your application state. */
} BT_SESSION_EVT;

/**@brief BT Event Inquiry Result structure
 */
typedef struct
{
  BT_ADDR addr;
  BT_CLASS btClass;
  int rssi;
  uint16_t len;
  uint8_t eir[BT_EVT_EIR_LEN];
} BT_EVT_INQUIRY_RESULT;

/**@brief BT Event passkey request
 */
typedef struct
{
  BT_ADDR addr;
}BT_EVT_PASSKEY_REQ;

/**@brief BT Event confirmation request
 */
typedef struct
{
  BT_ADDR addr;
  uint32_t numeric;
} BT_EVT_CONFIRMATION_REQ;

/**@brief BT Event acl connection/disconnection status
 */
typedef struct
{
  BT_ADDR addr;
  bool isConnected;        /**< true: connected, false: disconnected */
  uint8_t transport;       /**< transport type, please ref@ BT/BLE transport type */
  uint8_t disconReason;    /**< The reason of disconnection, please ref@ Acl disconnection type*/
} BT_EVT_ACL_CONN_STATUS;

/**@brief BT Event power manager status
 */
typedef struct
{
  BT_ADDR addr;
  BT_POWER_STATE status;
  uint16_t value;      /**< power mode value, such as sniff mode period and so on */
} BT_EVT_POWER_STATUS;

/**@brief BT Event spp connected
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_SPP_CONNECTED;


/**@brief BT Event a2dp src connected
 */
typedef struct
{
  BT_ADDR addr;
  uint8_t valCap;
  uint8_t featrue[4];
} BT_EVT_A2DP_SRC_CONNECTED;


/**@brief BT Event avrc controller connected
 */
typedef struct
{
  BT_ADDR addr;
  uint8_t status;
} BT_EVT_AVRC_CONTROLLER_CONNECTED;


/**@brief BT Event avrc target connected
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_AVRC_TARGET_CONNECTED;


/**@brief BT Event SPP disconnected
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_SPP_DISCONNECTED;


/**@brief BT Event a2dp src disconnected
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_A2DP_SRC_DISCONNECTED;


/**@brief BT Event avrc controller disconnected
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_AVRC_CONTROLLER_DISCONNECTED;


/**@brief BT Event avrc target disconnected
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_AVRC_TARGET_DISCONNECTED;


/**@brief BT Event a2dp started
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_A2DP_SRC_STARTED;

/**@brief BT Event a2dp stop
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_A2DP_SRC_STOP;

/**@brief BT Event Encryption State structure
 */
typedef struct
{
  uint8_t status; /**< 0 encryption is enable, else encryption is not enable */
  BT_ADDR addr;
} BT_EVT_ENC_STATE;

/**@brief BT Event remote connect device name structure
 */
typedef struct
{
  uint16_t status; /**< 0 get the connect device name success, other failed */
  BT_ADDR addr;
  char name[BT_MAX_NAME_LEN];
} BT_EVT_CONN_DEV_NAME;

/**@brief BT reply vendor information event
 */
typedef struct
{
    uint16_t vendor_source;
    uint16_t vendor_id;
} BT_DID_VendorInfo;

/**@brief BT bond info structure
 */
typedef struct
{
  BT_ADDR btAddr;
} BT_BondInfo;

/**@brief BT device bond list structure
 */
typedef struct
{
  uint32_t bondNum;
  uint8_t bondInfoId[BT_SAVE_BOND_DEVICE_MAX_NUM][BT_ADDR_LEN];
  uint8_t didInfo[BT_SAVE_BOND_DEVICE_MAX_NUM][BT_DID_INFO_LEN];     /**< BT DID information, please ref@ BT_DID_VendorInfo*/
} BT_BondInfoList;

/**@brief BT rf test command
 */
typedef struct
{
  uint16_t commandLen;
  uint8_t* command;
} BT_RfTestCommand;

/**@brief BT reply get version information event
 */
typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
    /* The branch number */
    uint8_t branch;
    /* The type or directory */
    uint8_t type;
    /* The build date, including month, day, and hour */
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    /* commit hash */
    uint32_t commitHash;
} BT_VersionInfo;

/**@brief Event callback function
 */
typedef void (*evtCallBack)(BT_EVT *pEvt);

/** @} bt_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup bt_funcs Functions
 * @{
 */

/**@brief   Initialize the BT hardware
 *
 * @param[in]  info: bt boot firmware information
 * @return     0 on success, otherwise error
 * @retval     -ENXIO: uart init fail
 * @retval     -EIO: can't receive from remote devices
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 * @note    This call initializes the bt hardware, no other bt related function can be called before this one.
 */
int BT_CommonInitializeWithBtBinary(BT_FIRMWARE_INFO info);

/**@brief   Initialize the BT hardware
 *
 * @return     0 on success, otherwise error
 * @retval     -ENXIO: uart init fail
 * @retval     -EIO: can't receive from remote devices
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 * @note    This call initializes the bt hardware, no other bt related function can be called before this one.
 */
int BT_CommonInitialize(void);

/**@brief Finalize the BT hardware
 *
 * @return     0 on success, otherwise error
 * @retval     -ENXIO: uart Finalize fail
 * @retval     -EIO: can't receive from remote devices
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_CommonFinalize(void);

/**@brief Start inquiry nearby devices
 *
 * @return     0 on success, otherwise error
 * @retval     -EBUSY: bt is inquiring
 * @retval     -EPERM: system error
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_StartInquiry(void);

/**@brief Set extended inquiry data
 * @details This call should be called before API BT_SetVisibility()
 *          The dafault Tx power in extended inquiry data is disable.
 *          Calling this API will enable TX power in extended inquiry data.
 *
 * @return     0 on success, otherwise error
 * @retval     -EINVAL: invalid argument
 * @retval     -EIO: failed to send cmd by uart
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetExtInquiryData(BT_EXT_INQR_DATA* inqrData);

/**@brief Cancel inquiry nearby devices
 *
 * @return     0 on success, otherwise error
 * @retval     -EBUSY: bt is idle
 * @retval     -EPERM: system error
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_CancelInquiry(void);

/**@brief Get device class
 *
 * @param[in]  btClass: Pointer to the device class to get
 * @return     0 on success, otherwise error
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_GetDeviceClass(BT_CLASS *btClass);

/**@brief Set device address
 *
 * @param[in]  addr: Pointer to the device address to set
 * @return     0 on success, otherwise error
 * @retval     -EPERM: system error
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetBtAddress(BT_ADDR *addr);

/**@brief get device address
 *
 * @param[out]  addr: Pointer to the device address to be filled in
 * @return     0: success
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
int BT_GetBtAddress(BT_ADDR *addr);

/**@brief Set device name
 *
 * @param[in]  name: Pointer to the device name to set
 * @return     0 on success, otherwise error
 * @retval     -EPERM: system error
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetBtName(char *name);

/**@brief get device name
 *
 * @param[in]  name: Pointer to the device name to be filled in, end with '\0'
 * @return     0: success
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
int BT_GetBtName(char *name);

/**@brief Set pairing enable
 *
 * @param[out]  isEnable: 1 enable 0 disable
 * @return      0 on success, otherwise error
 * @retval      -EPERM: bt is connected
 * @retval      -EINVAL: no such class to set
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetPairingEnable(BT_BOOL isEnable);

/**@brief Set device visibility
 *
 * @param[in]  isVisible: visibility of bt
 * @return     0 on success, otherwise error
 * @retval     -EPERM: bt has connected too much devices
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetVisibility(BT_VISIBILITY isVisible);

/**@brief Save the bonded peer device information
 *
 * @param[in]  btBondInfo: Bond information
 * @return     0: success
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
int BT_SaveBondInfo(BT_BondInfo *btBondInfo);

/**@brief Set pairing mode
 *
 * @param[in]  ioCapability: io capability of local device
 * @return     0 on success, otherwise error
 * @retval     -EPERM: system error
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetParingMode(BT_IO_CAP ioCapability);

/**@brief reply confirmation
 *
 * @param[in] replyConfirm: reply confirmation of local device
 * @return     0 on success, otherwise error
 * @retval     -EPERM: system error
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_ReplyConfirmation(BT_REPLY_CONFIRM replyConfirm);

/**@brief reply passkey
 *
 * @param[in] replyPasskey: reply passkey of local device
 * @return     0 on success, otherwise error
 * @retval     -EPERM: system error
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_ReplyPasskey(BT_REPLY_PASSKEY replyPasskey);

/**@brief Set event callback function
 *
 * @param[in]  cb: Pointer to event callback
 * @param[in]  evt: BT event context
 * @retval     -EINVAL: paramter inval
 * @retval     -EEXIST: buffer(evt) already allocated, the exiting buffer will be used instead of replacing it,
 *              cb is set successfully
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetEvtCallBack(evtCallBack cb,BT_EVT *evt);

/**@brief start bond function
 *
 * @param[in] addr: remote device address
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find nearby device
 * @retval     -EINVAL: paramter inval
 * @retval     -EPERM: system error
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_StartBond(BT_ADDR *addr);

/**@brief  unbond function
 *
 * @param[in] addr: remote device address
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: no such bonded device
 * @retval     -EINVAL: paramter inval
 * @retval     -EPERM: system error
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_Unbond(BT_ADDR *addr);

/**@brief  getBondInfo list function
 *
 * @param[in] bondInfo: bt bond list
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: get bond list failed
 * @retval     -EINVAL: paramter inval
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_GetBondInfoIdList(BT_BondInfoList *bondInfo);

/**@brief  set discover and connect param function
 *
 * @param[in] btVisilityParam: set discover and connect param, see structure BT_VISIBILITY_PARAM
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: set param failed
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetVisibilityParam(BT_VISIBILITY_PARAM *btVisilityParam);

/**@brief  set sniff mode param
 *
 * @param[in] btSniffParam: set sniff mode param,see structure BT_SNIFF_PARAM
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: set sniff param failed
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetSniffModeParam(BT_SNIFF_PARAM *btSniffParam);

/**@brief  set UART transportion rate.
 * @details This call allows the application to set UART baudrate and peer UART baudrate
 *          This API is not allowed to be called when UART is busy for transport or receive mass data.
 *          This API mustn't be called between sending of API BT_SppSendData() and receiving
 *          the event of BT_EVT_SPP_TX_COMPLETE.
 *
 * @param[in] baudrate: uart transportion rate, range 115200 ~ 1000000, default 921600
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: get bond list failed
 * @retval     -EINVAL: paramter inval
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetBaudrate(uint32_t baudrate);

/**@brief Set BT PCM I2S role.
 * @details This call allows the application to set BT device PCM I2S role
 *
 * @param[in]  role: PCM I2S role
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
int BT_SetPcmI2sRole(BT_PCM_I2S_ROLE role);

/**@brief Set Sniff mode timer timeout value.
 * @details This call allows the application to set sniff mode timer timeout value,
 *          this call is just used for debug.
 *
 * @param[in]  timeoutValue: timer timeout value
 * @return     0: success
 * @retval     -ENOENT: can't find the control block
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SetSniffTimerValue(uint8_t timeoutValue);

/**@brief Enable/Disable sniff mode.
 * @details This call allows the application to enable/disable sniff mode,
 *          this call can't be used when automatic sniff mode is open and is just used for debug.
 *
 * @param[in]  btIsEnbaleSniff: enable/disable sniff mode
 * @return     0: success
 * @retval     -ENOENT: can't find the control block
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_EnableSniffMode(BT_BOOL btIsEnbaleSniff);

/**@brief send RF test command.
 * @details This call allows the application to send RF test command.
 *
 * @param[in]  rfTestCommand: RF test command
 * @return     0: success
 * @retval     -ENOENT: can't find the control block
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_SendRfTestCommnd(BT_RfTestCommand* rfTestCommand);

/**@brief Get BT FW version.
 * @details This call request to get the BT version.
 *
 * @return     0: success
 * @retval     -ENOENT: can't find the control block
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_GetBTVersion(void);

/** @} bt_funcs */

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_COMM_H */
