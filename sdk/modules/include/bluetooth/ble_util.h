/****************************************************************************
 * sdk/modules/include/bluetooth/ble_util.h
 *
 *   Copyright 2023, 2024 Sony Semiconductor Solutions Corporation
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

#ifndef __SDK_MODULES_INCLUDE_BLUETOOTH_BLE_UTIL_H
#define __SDK_MODULES_INCLUDE_BLUETOOTH_BLE_UTIL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/hal/bt_if.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/* name: bleutil_convert_uuid2str
 *       Convert BLE_UUID to string.
 *
 *       uuid [in]  : UUID
 *       str  [out] : UUID string
 *                    In uuid->type = BLE_UUID_TYPE_UUID128 case,
 *                      VVVVVVVV-WWWW-XXXX-YYYY-ZZZZZZZZZZZZ.
 *                    otherwise XXXX
 *       len  [in]  : str buffer length.
 *                    In uuid->type = BLE_UUID_TYPE_UUID128 case,
 *                    len needs to be greater than BLE_UUID_128BIT_STRING_LENGTH.
 *                    othewise, len needs to be greater than BLE_UUID_16BIT_STRING_LENGTH.
 *
 *       Return : Positive integer  String length of str.
 *                -ENOMEM           len is too short.
 */

int bleutil_convert_uuid2str(BLE_UUID *uuid, char *str, int len);

/* name: bleutil_convert_str2uuid
 *       Convert UUID string to BLE_UUID structure.
 *
 *       str    [in]  : UUID string
 *       uuid   [out] : BLE_UUID structure
 *
 *       Return : OK convert success, ERROR invalid input string
 */

int bleutil_convert_str2uuid(char *str, BLE_UUID *uuid);

/* name: bleutil_uuidcmp
 *       Compare two UUIDs that are input.
 *
 *       uuid1 [in] : UUID1
 *       uuid2 [in] : UUID2
 *
 *       Return : 0  Two UUIDs are same. 0 Not
 */

int bleutil_uuidcmp(BLE_UUID *uuid1, BLE_UUID *uuid2);

/* name: bleutil_add_btaddr
 *       Add a BLE address in the ble_status.
 *
 *       status    [in/out] : Status instance.
 *       addr      [in]     : BLE address
 *       addr_type [in]     : BLE address type
 */

void bleutil_add_btaddr(struct ble_state_s *state, BT_ADDR *addr,
                        uint8_t addr_type);

/* name: bleutil_find_srvc_uuid
 *       Find specific UUID from advertising data.
 *
 *       uuid [in] : Target UUID to find.
 *       data [in] : Advertising data
 *       len  [in] : Advertising data length in bytes
 *
 *       Return : 1 Found the UUID.  0 Not
 */

int bleutil_find_srvc_uuid(BLE_UUID *uuid, uint8_t *data, uint8_t len);

/* name: bleutil_get_devicename
 *       Get device name from advertising data
 *
 *       data    [in]  : Advertising data
 *       len     [in]  : Advertising data length in bytes
 *       devname [out] : Device name
 *
 *       Return : 1 Device name is contained.  0 Not
 */

int bleutil_get_devicename(uint8_t *data, uint8_t len, char *devname);

/* name: bleutil_get_addrtype
 *       Get BLE address type from advertising data
 *
 *       data    [in]  : Advertising data
 *       len     [in]  : Advertising data length in bytes
 *
 *       Return : BLE address type. One of below
 *                   BLE_ADDRTYPE_PUBLIC
 *                   BLE_ADDRTYPE_RAND_STATIC
 *                   BLE_ADDRTYPE_RAND_PRIV_RESOLVABLE
 *                   BLE_ADDRTYPE_RAND_PRIV_NONRESOLVABLE
 */

BLE_ADDRESS_TYPE bleutil_get_addrtype(uint8_t *data, uint8_t len);

/* name: bleutil_get_rssi
 *       Get BLE RSSI value from advertising data
 *
 *       data    [in]  : Advertising data
 *       len     [in]  : Advertising data length in bytes
 *
 *       Return : BLE RSSI value for advertising
 */

int8_t bleutil_get_rssi(uint8_t *data, uint8_t len);

/* name: bleutil_get_advertising_flags
 *       Get advertising flags from advertising data
 *       This flags setting is defined in P.15 of
 *        https://www.bluetooth.org/DocMan/handlers/DownloadDoc.ashx?doc_id=556598
 *
 *       data    [in]  : Advertising data
 *       len     [in]  : Advertising data length in bytes
 *       flags   [out] : Advertising flags, which has the bit structure.
 *                       Each bits are defined as follows
 *                         BLE_ADV_FLAGS_LIMITED_DISC_MODE
 *                         BLE_ADV_FLAGS_GENERAL_DISC_MODE
 *                         BLE_ADV_FLAGS_BR_EDR_NOT_SUPPORTED
 *                         BLE_ADV_FLAGS_LE_BR_EDR_CONTROLLER
 *                         BLE_ADV_FLAGS_LE_BR_EDR_HOST
 *
 *       Return : 1 Advertising flags is contained.  0 Not
 */

int bleutil_get_advertising_flags(uint8_t *data, uint8_t len, uint8_t *flags);

/* name: bleutil_get_txpower
 *       Get advertise flags from advertising data
 *
 *       data    [in]  : Advertising data
 *       len     [in]  : Advertising data length in bytes
 *       txpower [out] : TX power(-127dBm to +127dBm)
 *
 *       Return : 1 TX power is contained.  0 Not
 */

int bleutil_get_txpower(uint8_t *data, uint8_t len, int8_t *txpower);

/* name: bleutil_get_connection_interval
 *       Get connection interval range from advertising data.
 *
 *       data      [in]  : Advertising data
 *       len       [in]  : Advertising data length in bytes
 *       min       [out] : The minimum value for the connection interval
 *                         The actual minimum value = min * 1.25msec
 *       max       [out] : The maximum value for the connection interval
 *                         The actual maximum value = max * 1.25msec
 *
 *       Return : 1 Connection interval is contained.  0 Not
 */
 
int bleutil_get_connection_interval(uint8_t *data,
                                    uint8_t len,
                                    uint16_t *min,
                                    uint16_t *max);

/* name: bleutil_judge_srvc_uuid_requested
 *       Judge if specific UUID is requested from advertising data.
 *
 *       uuid [in] : Target UUID, that own device provides
 *       data [in] : Advertising data
 *       len  [in] : Advertising data length in bytes
 *
 *       Return : 1 UUID is requested.  0 Not
 */

int bleutil_judge_srvc_uuid_requested(BLE_UUID *uuid,
                                      uint8_t *data,
                                      uint8_t len);

/* name: bleutil_get_appearance
 *       Get appearance from advertising data.
 *       Appearances are defined in P.27 - P.40
 *        of https://btprodspecificationrefs.blob.core.windows.net/assigned-numbers/Assigned%20Number%20Types/Assigned_Numbers.pdf
 *
 *       data       [in]  : Advertising data
 *       len        [in]  : Advertising data length in bytes
 *       appearance [out] : Apperance. One of below
 *                             BLE_APPEARANCE_GENERIC_PHONE
 *                             BLE_APPEARANCE_GENERIC_COMPUTER
 *                             BLE_APPEARANCE_GENERIC_WATCH
 *                             BLE_APPEARANCE_WATCH_SPORTS
 *                             BLE_APPEARANCE_GENERIC_CLOCK
 *                             BLE_APPEARANCE_GENERIC_DISPLAY
 *                             BLE_APPEARANCE_GENERIC_REMOTE_CONTROL
 *                             BLE_APPEARANCE_GENERIC_EYE_GLASSES
 *                             BLE_APPEARANCE_GENERIC_TAG
 *                             BLE_APPEARANCE_GENERIC_KEYRING
 *                             BLE_APPEARANCE_GENERIC_MEDIA_PLAYER
 *                             BLE_APPEARANCE_GENERIC_BARCODE_SCANNER
 *                             BLE_APPEARANCE_GENERIC_THERMOMETER
 *                             BLE_APPEARANCE_THERMOMETER_EAR
 *                             BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR
 *                             BLE_APPEARANCE_HEART_RATE_BELT
 *                             BLE_APPEARANCE_GENERIC_BLOOD_PRESSURE
 *                             BLE_APPEARANCE_BLOOD_PRESSURE_ARM
 *                             BLE_APPEARANCE_BLOOD_PRESSURE_WRIST
 *                             BLE_APPEARANCE_GENERIC_HID_DEVICE
 *                             BLE_APPEARANCE_HID_KEYBOARD
 *                             BLE_APPEARANCE_HID_MOUSE
 *                             BLE_APPEARANCE_HID_JOYSTICK
 *                             BLE_APPEARANCE_HID_GAMEPAD
 *                             BLE_APPEARANCE_HID_DIGITIZER_TABLET
 *                             BLE_APPEARANCE_HID_CARD_READER
 *                             BLE_APPEARANCE_HID_DIGITAL_PEN
 *                             BLE_APPEARANCE_HID_BARCODE_SCANNER
 *                             BLE_APPEARANCE_GENERIC_GLUCOSE_METER
 *                             BLE_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR
 *                             BLE_APPEARANCE_RUNNING_WALKING_SENSOR_IN_SHOE
 *                             BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_SHOE
 *                             BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_HIP
 *                             BLE_APPEARANCE_GENERIC_CYCLING
 *                             BLE_APPEARANCE_CYCLING_COMPUTER
 *                             BLE_APPEARANCE_CYCLING_SPEED_SENSOR
 *                             BLE_APPEARANCE_CYCLING_CADENCE_SENSOR
 *                             BLE_APPEARANCE_CYCLING_POWER_SENSOR
 *                             BLE_APPEARANCE_CYCLING_SPEED_AND_CADENCE_SENSOR
 *
 *       Return : 1 Apperance is contained.  0 Not
 */

int bleutil_get_appearance(uint8_t *data, uint8_t len, uint16_t *appearance);

/* name: bleutil_get_advertise_interval
 *       Get advertising interval range from advertising data.
 *
 *       data      [in]  : Advertising data
 *       len       [in]  : Advertising data length in bytes
 *       interval  [out] : The advertising interval
 *                         The actual value = interval * 0.625msec
 *
 *       Return : 1 Advertising interval is contained.  0 Not
 */
 
int bleutil_get_advertising_interval(uint8_t *data, uint8_t len, uint16_t *interval);

/* name: bleutil_get_manufacturer_specific_data
 *       Get manufacturer specific data from advertising data.
 *       The manufacturer specific data consists of company ID and arbitrary data.
 *       Company IDs are defined in P.216 - P.402
 *        of https://btprodspecificationrefs.blob.core.windows.net/assigned-numbers/Assigned%20Number%20Types/Assigned_Numbers.pdf
 *
 *       data             [in]  : Advertising data
 *       len              [in]  : Advertising data length in bytes
 *       company          [out] : company ID
 *       specific_data    [out] : Manufacturer specific data
 *       specific_datalen [out] : Manufacturer specific data length
 *
 *       Return : 1 Manufacturer specific data is contained.  0 Not
 */

int bleutil_get_manufacturer_specific_data(uint8_t *data,
                                           uint8_t len,
                                           uint8_t *company,
                                           uint8_t **specific_data,
                                           uint8_t *specific_datalen);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SDK_MODULES_INCLUDE_BLUETOOTH_BLE_UTIL_H */
