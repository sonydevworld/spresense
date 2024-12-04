/****************************************************************************
 * sdk/modules/bluetooth/bluetooth_le_util.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLE_AD_CONN_INTERVAL_OFFSET_MIN (0)
#define BLE_AD_CONN_INTERVAL_OFFSET_MAX (2)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* name: check_uuid128_list
 *       Check if the 128bit target uuid is contained in the list.
 *
 *       tgt_uuid  [in] : Target UUID to check.
 *       uuid_list [in] : List of UUIDs 16 bytes each.
 *       num       [in] : Size of UUIDs in the list.
 *
 *       Return : 1 Found the UUID.  0 Not
 */

static int check_uuid128_list(uint8_t *tgt_uuid, uint8_t uuid_list[][16],
                              int num)
{
  int i;

  for (i = 0; i < num; i++)
    {
      if (memcmp(tgt_uuid, uuid_list[i], 16) == 0)
        {
          return 1; /* Found */
        }
    }

  return 0;
}

/* name: check_uuid16_list
 *       Check if the 16bit target uuid is contained in the list.
 *
 *       tgt_uuid  [in] : Target short UUID to check.
 *       uuid_list [in] : List of short UUIDs 16 bytes each.
 *       num       [in] : Size of short UUIDs in the list.
 *
 *       Return : 1 Found the UUID.  0 Not
 */

static int check_uuid16_list(uint8_t *tgt_uuid, uint8_t uuid_list[][2],
                              int num)
{
  int i;

  for (i = 0; i < num; i++)
    {
      if (memcmp(tgt_uuid, uuid_list[i], 2) == 0)
        {
          return 1; /* Found */
        }
    }

  return 0;
}

/* name: find_uuid128
 *       Find 128bit target UUID in the advetising data
 *       from a peripheral device.
 *
 *       tgt_uuid [in] : Target UUID to check.
 *       data     [in] : Advertising data
 *       len      [in] : Advertising data length in bytes
 *
 *       Return : 1 Found the UUID.  0 Not
 */

static int find_uuid128(uint8_t *tgt_uuid, uint8_t *data, uint8_t len)
{
  int ret;
  struct bt_eir_s eir;
  uint8_t (*uuid_list)[16];
  int list_num;

  ret = ble_parse_advertising_data(
                    BLE_AD_TYPE_128BIT_SERVICE_UUID_INCOMPLETE,
                    data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      list_num = eir.len / BT_UUID128_LEN;
      uuid_list = (uint8_t (*)[BT_UUID128_LEN])eir.data;

      if (check_uuid128_list(tgt_uuid, uuid_list, list_num))
        {
          return 1;
        }
    }

  ret = ble_parse_advertising_data(
                    BLE_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE,
                    data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      list_num = eir.len / BT_UUID128_LEN;
      uuid_list = (uint8_t (*)[BT_UUID128_LEN])eir.data;

      if (check_uuid128_list(tgt_uuid, uuid_list, list_num))
        {
          return 1;
        }
    }

  return 0;
}

/* name: find_uuid16
 *       Find 16bit target UUID in the advetising data
 *       from a peripheral device.
 *
 *       tgt_uuid [in] : Target short UUID to check.
 *       data     [in] : Advertising data
 *       len      [in] : Advertising data length in bytes
 *
 *       Return : 1 Found the UUID.  0 Not
 */

static int find_uuid16(uint16_t tgt_uuid, uint8_t *data, uint8_t len)
{
  int ret;
  struct bt_eir_s eir;
  uint8_t (*uuid_list)[2];
  int list_num;

  ret = ble_parse_advertising_data(
                    BLE_AD_TYPE_16BIT_SERVICE_UUID_INCOMPLETE,
                    data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      list_num = eir.len / 2;
      uuid_list = (uint8_t (*)[2])eir.data;

      if (check_uuid16_list((uint8_t *)&tgt_uuid, uuid_list, list_num))
        {
          return 1;
        }
    }

  ret = ble_parse_advertising_data(
                    BLE_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                    data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      list_num = eir.len / 2;
      uuid_list = (uint8_t (*)[2])eir.data;

      if (check_uuid16_list((uint8_t *)&tgt_uuid, uuid_list, list_num))
        {
          return 1;
        }
    }

  return 0;
}

/* name: find_requested_uuid128
 *       Find 128bit target UUID in the advetising data
 *       from a peripheral device.
 *
 *       tgt_uuid [in] : Target UUID to check.
 *       data     [in] : Advertising data
 *       len      [in] : Advertising data length in bytes
 *
 *       Return : 1 The UUID is requested.  0 Not
 */

static int find_requested_uuid128(uint8_t *tgt_uuid, uint8_t *data, uint8_t len)
{
  int ret;
  struct bt_eir_s eir;
  uint8_t (*uuid_list)[BT_UUID128_LEN];
  int list_num;

  ret = ble_parse_advertising_data(
                    BLE_AD_TYPE_SOLICITED_SERVICE_UUIDS_128BIT,
                    data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      list_num = eir.len / BT_UUID128_LEN;
      uuid_list = (uint8_t (*)[BT_UUID128_LEN])eir.data;

      if (check_uuid128_list(tgt_uuid, uuid_list, list_num))
        {
          return 1;
        }
      else
        {
          return 0;
        }
    }

  return 1;
}

/* name: find_requested_uuid16
 *       Find 16bit target UUID in the advetising data
 *       from a peripheral device.
 *
 *       tgt_uuid [in] : Target short UUID to check.
 *       data     [in] : Advertising data
 *       len      [in] : Advertising data length in bytes
 *
 *       Return : 1 The UUID is requested.  0 Not
 */

static int find_requested_uuid16(uint16_t tgt_uuid, uint8_t *data, uint8_t len)
{
  int ret;
  struct bt_eir_s eir;
  uint8_t (*uuid_list)[2];
  int list_num;

  ret = ble_parse_advertising_data(
                    BLE_AD_TYPE_SOLICITED_SERVICE_UUIDS_16BIT,
                    data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      list_num = eir.len / 2;
      uuid_list = (uint8_t (*)[2])eir.data;

      if (check_uuid16_list((uint8_t *)&tgt_uuid, uuid_list, list_num))
        {
          return 1;
        }
      else
        {
          return 0;
        }
    }

  return 1;
}

/* name: get_character_wo_hyphen
 *       Get character without hyphen from input string.
 *
 *       in   [in]  : Pointer to input string.
 *       hex  [out] : The first character without hyphen in input string
 *
 *       Return : The next address of hex.
 */

static char *get_character_wo_hyphen(char *in, char *hex)
{
  char *ret = in;

  while (*ret == '-')
    {
      ret++;
    }

  *hex = *ret;
  ret++;

  return ret;
}

/****************************************************************************
 * Public Functions
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
 *       Return : Positive interger  String length of str.
 *                -ENOMEM            len is too short.
 */

int bleutil_convert_uuid2str(BLE_UUID *uuid, char *str, int len)
{
  int i;
  uint8_t *in;
  char *out = str;

  memset(str, 0, len);

  if (uuid->type == BLE_UUID_TYPE_UUID128)
    {
      if (len < BLE_UUID_128BIT_STRING_BUFSIZE)
        {
          return -ENOMEM;
        }

      in = &uuid->value.uuid128.uuid128[BT_UUID128_LEN - 1];

      for (i = 0; i < BT_UUID128_LEN; i++)
        {
          /* Convert one byte hexadecimal value to two characters. */

          snprintf(out, 3, "%02x", *in);
          out += 2;
          in--;

          if ((i == 3) || (i == 5) || (i == 7) || (i == 9))
            {
              *out = '-';
              out++;
            }
        }

      return strlen(str);
    }
  else
    {
      if (len < BLE_UUID_16BIT_STRING_BUFSIZE)
        {
          return -ENOMEM;
        }

      snprintf(out,
               BLE_UUID_16BIT_STRING_BUFSIZE,
               "%04x",
               uuid->value.alias.uuidAlias);

      return strlen(str);
    }
}

/* name: bleutil_convert_str2uuid
 *       Convert UUID string to BLE_UUID structure.
 *
 *       str    [in]  : UUID string.
 *                        VVVVVVVV-WWWW-XXXX-YYYY-ZZZZZZZZZZZZ
 *                        XXXX
 *                      V, W, X, Y are hexdecimal number
 *                      The hyphens are not necessarily needed.
 *       uuid   [out] : BLE_UUID structure
 *
 *       Return : BT_SUCCESS convert success, -EINVAL invalid input string
 */

int bleutil_convert_str2uuid(char *str, BLE_UUID *uuid)
{
  char *in = str;
  uint8_t *out;
  char hex[3] = {0};
  char *errptr;

  if (strlen(str) == BLE_UUID_16BIT_STRING_LENGTH)
    {
      /* string format : XXXX */

      uuid->type = BLE_UUID_TYPE_BASEALIAS_BTSIG;
      uuid->value.alias.uuidAlias = strtol(str, &errptr, 16);
      if (*errptr)
        {
          /* Invalid characters exist. */

          return -EINVAL;
        }
    }
  else
    {
      uuid->type = BLE_UUID_TYPE_UUID128;

      in = str;
      out = &uuid->value.uuid128.uuid128[BT_UUID128_LEN - 1];
      while ((*in != '\0') && (out >= uuid->value.uuid128.uuid128))
        {
          in = get_character_wo_hyphen(in, &hex[0]);
          in = get_character_wo_hyphen(in, &hex[1]);

          /* Error if NULL character exists in unexpected position. */

          if (strlen(hex) != sizeof(uint16_t))
            {
              return -EINVAL;
            }

          /* Convert two characters to one byte data. */

          *out = strtol(hex, &errptr, 16);
          if (*errptr)
            {
              /* Invalid characters exist. */

              return -EINVAL;
            }

          out--;
        }

      if ((*in != '\0') || (out >= uuid->value.uuid128.uuid128))
        {
          /* Input data length is not 128-bits. */

          return -EINVAL;
        }
    }

  return BT_SUCCESS;
}

/* name: bleutil_uuidcmp
 *       Compare two UUIDs that are input.
 *
 *       uuid1 [in] : UUID1
 *       uuid2 [in] : UUID2
 *
 *       Return : 0  Two UUIDs are same. 0 Not
 */

int bleutil_uuidcmp(BLE_UUID *uuid1, BLE_UUID *uuid2)
{
  if (uuid1->type != uuid2->type)
    {
      return 1;
    }

  if (uuid1->type == BLE_UUID_TYPE_UUID128)
    {
      return memcmp(uuid1->value.uuid128.uuid128,
                    uuid2->value.uuid128.uuid128,
                    BT_UUID128_LEN);
    }
  else
    {
      return (uuid1->value.alias.uuidAlias != uuid2->value.alias.uuidAlias);
    }
}

/* name: bleutil_add_btaddr
 *       Add a BLE address in the ble_status.
 *
 *       status    [in/out] : Status instance.
 *       addr      [in]     : BLE address
 *       addr_type [in]     : BLE address type
 */

void bleutil_add_btaddr(struct ble_state_s *state, BT_ADDR *addr,
                        uint8_t addr_type)
{
  memcpy(&state->bt_target_addr, addr, sizeof(state->bt_target_addr));
  state->bt_target_addr_type = addr_type;
}

/* name: bleutil_find_srvc_uuid
 *       Find specific UUID from advertising data.
 *
 *       uuid [in] : Target UUID to find.
 *       data [in] : Advertising data
 *       len  [in] : Advertising data length in bytes
 *
 *       Return : 1 Found the UUID.  0 Not
 */

int bleutil_find_srvc_uuid(BLE_UUID *uuid, uint8_t *data, uint8_t len)
{
  if (uuid->type == BLE_UUID_TYPE_UUID128)
    {
      return find_uuid128(uuid->value.uuid128.uuid128, data, len);
    }
  else
    {
      return find_uuid16(uuid->value.alias.uuidAlias, data, len);
    }
}

/* name: bleutil_get_devicename
 *       Get device name from advertising data
 *
 *       data    [in]  : Advertising data
 *       len     [in]  : Advertising data length in bytes
 *       devname [out] : Device name
 *
 *       Return : 1 Device name is contained.  0 Not
 */

int bleutil_get_devicename(uint8_t *data, uint8_t len, char *devname)
{
  struct bt_eir_s eir;

  if (ble_parse_advertising_data(BLE_AD_TYPE_COMPLETE_LOCAL_NAME,
                                 data, len, &eir) == BT_SUCCESS)
    {
      memcpy(devname, eir.data, eir.len);
      devname[eir.len] = '\0';
      return 1;
    }
  else if(ble_parse_advertising_data(BLE_AD_TYPE_SHORT_LOCAL_NAME,
                                     data, len, &eir) == BT_SUCCESS)
    {
      memcpy(devname, eir.data, eir.len);
      devname[eir.len] = '\0';
      return 1;
    }

  return 0;
}

/* name: bleutil_get_addrtype
 *       Get BLE address type from advertising data
 *
 *       data    [in]  : Advertising data
 *       len     [in]  : Advertising data length in bytes
 *
 *       Return : BLE address type. One of below
 *                   BLE_GAP_ADDR_TYPE_PUBLIC
 *                   BLE_GAP_ADDR_TYPE_RANDOM_STATIC
 *                   BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE
 *                   BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE
 */

uint8_t bleutil_get_addrtype(uint8_t *data, uint8_t len)
{
  return (BLE_ADDRESS_TYPE)data[0];
}

/* name: bleutil_get_rssi
 *       Get BLE RSSI value from advertising data
 *
 *       data    [in]  : Advertising data
 *       len     [in]  : Advertising data length in bytes
 *
 *       Return : BLE RSSI value for advertising
 */

int8_t bleutil_get_rssi(uint8_t *data, uint8_t len)
{
  return (int8_t)data[1];
}

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

int bleutil_get_advertising_flags(uint8_t *data, uint8_t len, uint8_t *flags)
{
  int ret;
  struct bt_eir_s eir;

  ret = ble_parse_advertising_data(BLE_AD_TYPE_FLAGS, data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      *flags = eir.data[0];
      return 1;
    }
  else
    {
      return 0;
    }
}

/* name: bleutil_get_txpower
 *       Get TX power from advertising data
 *
 *       data    [in]  : Advertising data
 *       len     [in]  : Advertising data length in bytes
 *       txpower [out] : TX power(-127dBm to +127dBm)
 *
 *       Return : 1 TX power is contained.  0 Not
 */

int bleutil_get_txpower(uint8_t *data, uint8_t len, int8_t *txpower)
{
  int ret;
  struct bt_eir_s eir;

  ret = ble_parse_advertising_data(BLE_AD_TYPE_TX_POWER_LEVEL, data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      *txpower = (int8_t)eir.data[0];
      return 1;
    }

  return 0;
}

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
                                    uint16_t *max)
{
  int ret;
  struct bt_eir_s eir;

  ret = ble_parse_advertising_data(BLE_AD_TYPE_CONNECTION_INTERVAL_RANGE,
                                    data, len, &eir);
  if (ret == BT_SUCCESS)
    {

      memcpy(min, &eir.data[BLE_AD_CONN_INTERVAL_OFFSET_MIN], sizeof(uint16_t));
      memcpy(max, &eir.data[BLE_AD_CONN_INTERVAL_OFFSET_MAX], sizeof(uint16_t));
      return 1;
    }

  return 0;
}

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
                                      uint8_t len)
{
  if (uuid->type == BLE_UUID_TYPE_UUID128)
    {
      return find_requested_uuid128(uuid->value.uuid128.uuid128, data, len);
    }
  else
    {
      return find_requested_uuid16(uuid->value.alias.uuidAlias, data, len);
    }
}

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

int bleutil_get_appearance(uint8_t *data, uint8_t len, uint16_t *appearance)
{
  int ret;
  struct bt_eir_s eir;

  ret = ble_parse_advertising_data(BLE_AD_TYPE_APPEARANCE, data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      memcpy(appearance, eir.data, sizeof(uint16_t));
      return 1;
    }

  return 0;
}

/* name: bleutil_get_advertising_interval
 *       Get advertising interval range from advertising data.
 *
 *       data      [in]  : Advertising data
 *       len       [in]  : Advertising data length in bytes
 *       interval  [out] : The advertising interval
 *                         The actual value = interval * 0.625msec
 *
 *       Return : 1 Advertising interval is contained.  0 Not
 */

int bleutil_get_advertising_interval(uint8_t *data, uint8_t len, uint16_t *interval)
{
  int ret;
  struct bt_eir_s eir;

  ret = ble_parse_advertising_data(BLE_AD_TYPE_ADVERTISING_INTERVAL,
                                    data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      memcpy(interval, eir.data, sizeof(uint16_t));
      return 1;
    }

  return 0;
}

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

int bleutil_get_manufacturer_specific_data(uint8_t  *data,
                                           uint8_t  len,
                                           uint16_t *company,
                                           uint8_t  **specific_data,
                                           uint8_t  *specific_datalen)
{
  int ret;
  static struct bt_eir_s eir;

  ret = ble_parse_advertising_data(BLE_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                                    data, len, &eir);
  if (ret == BT_SUCCESS)
    {
      /* Manufacturer specific data has the following structure.
       *   company ID     2bytes
       *   specific data  arbitrary length eir.data[2] ...
       */

      memcpy(company, eir.data, sizeof(uint16_t));
      *specific_datalen = eir.len - 2; /* Except company ID 2bytes. */
      *specific_data = &eir.data[2];
      return 1;
    }

  return 0;
}

