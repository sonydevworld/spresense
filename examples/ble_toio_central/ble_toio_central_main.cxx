/****************************************************************************
 * examples/ble_toio_central/ble_toio_central_main.cxx
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
#include <pthread.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/hal/bt_if.h>
#include <bluetooth/ble_util.h>
#include "ble_central_app.h"
#include "toio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* toio: Service UUID */

const char *SERVICE_UUID = "10B20100-5B3B-4571-9508-CF3EFCD7BBAE";

/* toio: Characteristic UUID */

const char *ID_UUID      = "10B20101-5B3B-4571-9508-CF3EFCD7BBAE";
const char *MOTOR_UUID   = "10B20102-5B3B-4571-9508-CF3EFCD7BBAE";
const char *LIGHT_UUID   = "10B20103-5B3B-4571-9508-CF3EFCD7BBAE";
const char *SOUND_UUID   = "10B20104-5B3B-4571-9508-CF3EFCD7BBAE";
const char *SENSOR_UUID  = "10B20106-5B3B-4571-9508-CF3EFCD7BBAE";
const char *BUTTON_UUID  = "10B20107-5B3B-4571-9508-CF3EFCD7BBAE";
const char *BATTERY_UUID = "10B20108-5B3B-4571-9508-CF3EFCD7BBAE";
const char *SETTING_UUID = "10B201FF-5B3B-4571-9508-CF3EFCD7BBAE";

/* Characteristic UUIDs */

static BLE_UUID g_id_uuid;
static BLE_UUID g_motor_uuid;
static BLE_UUID g_light_uuid;
static BLE_UUID g_sound_uuid;
static BLE_UUID g_sensor_uuid;
static BLE_UUID g_button_uuid;
static BLE_UUID g_battery_uuid;
static BLE_UUID g_setting_uuid;

/* Characteristic handles */

static uint16_t g_id_handle;
static uint16_t g_motor_handle;
static uint16_t g_light_handle;
static uint16_t g_sound_handle;
static uint16_t g_sensor_handle;
static uint16_t g_button_handle;
static uint16_t g_battery_handle;
static uint16_t g_setting_handle;

static volatile bool g_thread_finished;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Get characteristic handle of the discovered UUID */

static void char_discovered(struct ble_gattc_db_disc_char_s *gatt_disc_char)
{
  struct ble_gattc_char_s *ch = &gatt_disc_char->characteristic;

  printf("%s()\n", __func__);

  if (bleutil_uuidcmp(&g_id_uuid, &ch->char_valuuid) == 0)
    {
      g_id_handle = ch->char_valhandle;
    }

  if (bleutil_uuidcmp(&g_motor_uuid, &ch->char_valuuid) == 0)
    {
      g_motor_handle = ch->char_valhandle;
    }

  if (bleutil_uuidcmp(&g_light_uuid, &ch->char_valuuid) == 0)
    {
      g_light_handle = ch->char_valhandle;
    }

  if (bleutil_uuidcmp(&g_sound_uuid, &ch->char_valuuid) == 0)
    {
      g_sound_handle = ch->char_valhandle;
    }

  if (bleutil_uuidcmp(&g_sensor_uuid, &ch->char_valuuid) == 0)
    {
      g_sensor_handle = ch->char_valhandle;
    }

  if (bleutil_uuidcmp(&g_button_uuid, &ch->char_valuuid) == 0)
    {
      g_button_handle = ch->char_valhandle;
    }

  if (bleutil_uuidcmp(&g_battery_uuid, &ch->char_valuuid) == 0)
    {
      g_battery_handle = ch->char_valhandle;
    }

  if (bleutil_uuidcmp(&g_setting_uuid, &ch->char_valuuid) == 0)
    {
      g_setting_handle = ch->char_valhandle;
    }
}

/****************************************************************************
 * toio_thread_main
 ****************************************************************************/

static void *toio_thread_main(void *arg)
{
  Identification id(g_id_handle);
  Motor   motor(g_motor_handle);
  Light   light(g_light_handle);
  Sound   sound(g_sound_handle);
  Sensor  sensor(g_sensor_handle);
  Button  button(g_button_handle);
  Battery battery(g_battery_handle);
  Setting setting(g_setting_handle);

  bool mat = false;

  printf("Protocol version: %s\n", setting.version());
  printf("Battery level: %d %%\n", battery.level());

  printf("Play sound effect: Enter\n");
  sound.play_sound_effect(Sound::Enter);

  printf("Motor spin\n");
  motor.run(100, -100, 100);
  sleep(1);
  motor.run(-100, 100, 100);
  sleep(1);

  while (!g_thread_finished)
    {
      if (sensor.tilt_detected())
        {
          printf("Motion: Tilt detected!!\n");
          printf("Turn on light: Blue\n");
          light.on(0, 0, 255);
          printf("Play sound effect: Get1\n");
          sound.play_sound_effect(Sound::Get1);
        }

      if (sensor.collision_detected())
        {
          printf("Motion: Collision detected!!\n");
          printf("Turn on light: Red:\n");
          light.on(255, 0, 0);
          printf("Play sound effect: Get2\n");
          sound.play_sound_effect(Sound::Get2);
        }

      if (sensor.shake_detected())
        {
          printf("Motion: Shake detected!!\n");
          printf("Turn on light: Green\n");
          light.on(0, 255, 0);
          printf("Play sound effect: Get3\n");
          sound.play_sound_effect(Sound::Get3);
        }

      if (button.is_pressed())
        {
          printf("Battery level: %d %%\n", battery.level());
          printf("Turn off light\n");
          light.off();
        }

      if (!mat && (id.is_valid_position() || id.is_valid_standard()))
        {
          /* In the mat for the first time. */

          mat = true;
          printf("Play sound effect: MatIn\n");
          sound.play_sound_effect(Sound::MatIn);
          if (id.is_valid_position())
            {
              printf("(x, y, angle) = (%4d, %4d, %3d)\n", id.x(), id.y(), id.angle());
            }
          if (id.is_valid_standard())
            {
              printf("(stadard, angle) = (0x%08lx, %3d)\n", id.value(), id.angle());
            }
        }

      if (mat && (!id.is_valid_position() && !id.is_valid_standard()))
        {
          /* Out of the mat. */

          mat = false;
          printf("Play sound effect: MatOut\n");
          sound.play_sound_effect(Sound::MatOut);
        }

      /* Yield periodically to dispatch to other threads. */

      usleep(1);
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  int ret;
  char myname[]  = "BLE_TOIO_CENTRAL";
  BT_ADDR myaddr = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};
  BLE_UUID service_uuid;
  pthread_t toio_thread;

  /* Convert UUID strings to BLE_UUID value. */

  bleutil_convert_str2uuid(const_cast<char *>(SERVICE_UUID), &service_uuid);
  bleutil_convert_str2uuid(const_cast<char *>(ID_UUID),      &g_id_uuid);
  bleutil_convert_str2uuid(const_cast<char *>(MOTOR_UUID),   &g_motor_uuid);
  bleutil_convert_str2uuid(const_cast<char *>(LIGHT_UUID),   &g_light_uuid);
  bleutil_convert_str2uuid(const_cast<char *>(SOUND_UUID),   &g_sound_uuid);
  bleutil_convert_str2uuid(const_cast<char *>(SENSOR_UUID),  &g_sensor_uuid);
  bleutil_convert_str2uuid(const_cast<char *>(BUTTON_UUID),  &g_button_uuid);
  bleutil_convert_str2uuid(const_cast<char *>(BATTERY_UUID), &g_battery_uuid);
  bleutil_convert_str2uuid(const_cast<char *>(SETTING_UUID), &g_setting_uuid);

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

  /* Allow the vendor specific UUIDs to be discovered. */

  ble_set_vendor_uuid(&service_uuid);

  /* Set scan filter to connect if the specified UUID is found. */

  scan_filter_uuid(&service_uuid);

  /*scan_filter_device_name("toio-xxx");*/

  /* Discover characteristic to get the handle. */

  register_discover(&g_id_uuid,      char_discovered);
  register_discover(&g_motor_uuid,   char_discovered);
  register_discover(&g_light_uuid,   char_discovered);
  register_discover(&g_sound_uuid,   char_discovered);
  register_discover(&g_sensor_uuid,  char_discovered);
  register_discover(&g_button_uuid,  char_discovered);
  register_discover(&g_battery_uuid, char_discovered);
  register_discover(&g_setting_uuid, char_discovered);

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
            g_thread_finished = false;
            ret = pthread_create(&toio_thread, NULL, toio_thread_main, NULL);
            if (ret != 0)
              {
                printf("ERROR: pthread_create() failed: %d\n", ret);
                goto error;
              }
            break;
          case BLE_APP_DISCONNECTED:
            printf("Finish application.\n");
            g_thread_finished = true;
            pthread_join(toio_thread, NULL);
            goto error;
            break;
          default:
            break;
        }
    }

error:
  /* Unregister discover callbacks. */

  unregister_discover(&g_id_uuid);
  unregister_discover(&g_motor_uuid);
  unregister_discover(&g_light_uuid);
  unregister_discover(&g_sound_uuid);
  unregister_discover(&g_sensor_uuid);
  unregister_discover(&g_button_uuid);
  unregister_discover(&g_battery_uuid);
  unregister_discover(&g_setting_uuid);

  ble_disable();
  bt_disable();

  return ret;
}
