/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bcm20706_hal.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Public Datas
 ****************************************************************************/

/* BT Common HAL I/F */

struct bt_hal_common_ops_s bt_hal_common_ops;

/* BT A2DP HAL I/F */

struct bt_hal_a2dp_ops_s bt_hal_a2dp_ops;

/* BT AVRCP HAL I/F */

struct bt_hal_avrcp_ops_s bt_hal_avrcp_ops;

/* BT HFP HAL I/F */

struct bt_hal_hfp_ops_s bt_hal_hfp_ops;

/* BT SPP HAL I/F */

struct bt_hal_spp_ops_s bt_hal_spp_ops;

/* BT SPP HAL I/F */

struct ble_hal_common_ops_s ble_hal_common_ops;

/* BT SPP HAL I/F */

struct ble_hal_gatt_ops_s ble_hal_gatt_ops;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bcm20706_probe(void)
{
  int ret = 0;

  /* Register BT common HAL */

  ret = bt_common_register_hal(&bt_hal_common_ops);

  /* Register BT A2DP HAL */

  ret = bt_a2dp_register_hal(&bt_hal_a2dp_ops);

  /* Register BT AVRCP HAL */

  ret = bt_avrcp_register_hal(&bt_hal_avrcp_ops);

  /* Register BT HFP HAL */

  ret = bt_hfp_register_hal(&bt_hal_hfp_ops);

  /* Register BT SPP HAL */

  ret = bt_spp_register_hal(&bt_hal_spp_ops);

  /* Register BLE common HAL */

  ret = ble_common_register_hal(&ble_hal_common_ops);

  /* Register BLE GATT HAL */

  ret = ble_gatt_register_hal(&ble_hal_gatt_ops);

  return ret;
}
