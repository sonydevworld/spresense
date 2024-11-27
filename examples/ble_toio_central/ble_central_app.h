/****************************************************************************
 * examples/ble_toio_central/ble_central_app.h
 *
 *   Copyright 2024 Sony Semiconductor Solutions Corporation
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

#ifndef __EXAMPLES_BLE_TOIO_CENTRAL_BLE_CENTRAL_APP_H
#define __EXAMPLES_BLE_TOIO_CENTRAL_BLE_CENTRAL_APP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <bluetooth/ble_gatt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLE_UUID16(_uuid) \
{ \
  .type = BLE_UUID_TYPE_BASEALIAS_BTSIG, \
  .value.alias.uuidAlias = _uuid, \
}

#define BLE_UUID128(_uuid) \
{ \
  .type = BLE_UUID_TYPE_UUID128, \
  .value.uuid128 = {_uuid}, \
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* BLE app event */

enum ble_app_event
{
  BLE_APP_CONNECTED,
  BLE_APP_DISCONNECTED,
};

/* Discovery callback */

typedef void (*discover_cb_t)(struct ble_gattc_db_disc_char_s *gatt_disc_char);

/* Notify callback */

typedef void (*notify_cb_t)(struct ble_gatt_char_s *gatt_char);

/****************************************************************************
 * Public Functions Prototype
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* BLE GATT synchronous API
 * These APIs can be called only from application threads.
 * They must not be called from notify callback threads.
 */

int write_gatt_descriptor(uint16_t handle, uint8_t *data, int len);
int read_gatt_descriptor(uint16_t handle, uint8_t *data, int *len);
int write_gatt_char(uint16_t handle, uint8_t *data, int len, bool response);
int read_gatt_char(uint16_t handle, uint8_t *data, int *len);

/* BLE Scan filter API */

int scan_filter_uuid(BLE_UUID *uuid);
int scan_filter_device_name(const char *name);

/* BLE Register discovery API */

int register_discover(BLE_UUID *uuid, discover_cb_t cb);
int unregister_discover(BLE_UUID *uuid);

/* BLE Notify API */

int start_notify(BLE_UUID *uuid, notify_cb_t cb);
int stop_notify(BLE_UUID *uuid);

/* BLE App API */

int ble_app_init(void);
enum ble_app_event ble_app_wait_event(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLES_BLE_TOIO_CENTRAL_BLE_CENTRAL_APP_H */
