/****************************************************************************
 * examples/ble_mouse_central/ble_mouse_central_main.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/hal/bt_if.h>
#include <bluetooth/ble_util.h>
#ifdef CONFIG_EXAMPLES_BLE_MOUSE_INPUT_DEVICE
#include <nuttx/input/mouse.h>
#endif
#include "ble_central_app.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BLE HID Service UUID */

#define BLE_UUID_HID_SERVICE 0x1812

/* BLE Report Characteristic UUID */

#define BLE_UUID_REPORT_CHAR 0x2A4D

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Mouse device specific data */

begin_packed_struct struct mouse_data_t
{
  uint8_t buttons;
  int16_t x;
  int16_t y;
  int8_t  wheel;
  uint8_t pan;
} end_packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_BLE_MOUSE_INPUT_DEVICE
static struct mouse_lowerhalf_s g_mouselower;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mouse_discover(struct ble_gattc_db_disc_char_s *gatt_disc_char)
{
  printf("%s()\n", __func__);
}

static void mouse_notify(struct ble_gatt_char_s *gatt_char)
{
  /* Received notify from mouse device.
   * The mouse data structure depends on the mouse device
   * and should be customized as needed.
   */

  struct mouse_data_t *data = (struct mouse_data_t *)gatt_char->value.data;

#ifdef CONFIG_EXAMPLES_BLE_MOUSE_INPUT_DEVICE
  struct mouse_report_s sample;

  sample.buttons = data->buttons;
  sample.x       = data->x;
  sample.y       = data->y;
#  ifdef CONFIG_INPUT_MOUSE_WHEEL
  sample.wheel   = data->wheel;
#  endif
  mouse_event(g_mouselower.priv, &sample);
#else
  printf("button = %d (x, y) = (%4d, %4d) wheel= %3d\n",
         data->buttons, data->x, data->y, data->wheel);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  char myname[]  = "BLE_MOUSE_CENTRAL";
  BT_ADDR myaddr = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};

  BLE_UUID hid_uuid = BLE_UUID16(BLE_UUID_HID_SERVICE);
  BLE_UUID report_uuid = BLE_UUID16(BLE_UUID_REPORT_CHAR);

  /* Initialize BLE central application. */

  ble_app_init();

  ble_set_name(myname);
  ble_set_address(&myaddr);

  ret = bt_enable();
  if (ret != BT_SUCCESS)
    {
      printf("ERROR: bt_enable() ret=%d\n", ret);
      return ret;
    }

  ret = ble_enable();
  if (ret != BT_SUCCESS)
    {
      printf("ERROR: ble_enable() ret=%d\n", ret);
      goto error;
    }

  /* Set scan filter to connect if the specified UUID is found. */

  scan_filter_uuid(&hid_uuid);

  /* Discover the report characteristic */

  register_discover(&report_uuid, mouse_discover);

#ifdef CONFIG_EXAMPLES_BLE_MOUSE_INPUT_DEVICE
  ret = mouse_register(&g_mouselower, CONFIG_EXAMPLES_BLE_MOUSE_INPUT_DEVPATH, 1);
  if (ret < 0)
    {
      printf("[%s] mouse_register failed. ret = %d\n", __func__, ret);
      goto error;
    }
#endif

  ret = ble_start_scan(false);
  if (ret != BT_SUCCESS)
    {
      printf("[%s] ble_start_scan() failed. ret = %d\n", __func__, ret);
      goto error;
    }

  while (1)
    {
      switch (ble_app_wait_event())
        {
          case BLE_APP_CONNECTED:

            /* Enable HID report notification. */

            start_notify(&report_uuid, mouse_notify);
            break;

          case BLE_APP_DISCONNECTED:

            /* Disable HID report notification. */

            stop_notify(&report_uuid);

            /* Re-scan after disconnection. */

            ble_start_scan(false);
            break;
          default:
            break;
        }
    }

error:
  ble_disable();
  bt_disable();

  return 0;
}
