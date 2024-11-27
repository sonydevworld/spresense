/****************************************************************************
 * examples/ble_toio_central/toio.h
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

#ifndef __EXAMPLES_BLE_TOIO_CENTRAL_TOIO_H
#define __EXAMPLES_BLE_TOIO_CENTRAL_TOIO_H

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

/****************************************************************************
 * Public Classes
 ****************************************************************************/

/****************************************************************************
 * Identification
 ****************************************************************************/

class Identification
{
  public:
    Identification(uint16_t handle) : m_handle(handle)
    {
      start_notify(const_cast<BLE_UUID *>(&s_uuid), Identification::notify);
    }

    ~Identification()
    {
      stop_notify(const_cast<BLE_UUID *>(&s_uuid));
    }

    struct id_s
    {
      uint16_t x;
      uint16_t y;
      uint16_t angle;
      uint16_t sensor_x;
      uint16_t sensor_y;
      uint16_t sensor_angle;
      uint32_t value;
      bool     valid_position;
      bool     valid_standard;
    };

    static struct id_s s_id;

    static void notify(struct ble_gatt_char_s *gatt_char)
    {
      begin_packed_struct struct position_id_s
      {
        uint8_t  type;
        uint16_t x;
        uint16_t y;
        uint16_t angle;
        uint16_t sensor_x;
        uint16_t sensor_y;
        uint16_t sensor_angle;
      } end_packed_struct *pos_id;

      begin_packed_struct struct standard_id_s
      {
        uint8_t  type;
        uint32_t value;
        uint16_t angle;
      } end_packed_struct *std_id;

      uint8_t type;

      type = gatt_char->value.data[0];

      if (type == 0x01)
        {
          pos_id = (struct position_id_s *)gatt_char->value.data;
          s_id.x              = pos_id->x;
          s_id.y              = pos_id->y;
          s_id.angle          = pos_id->angle;
          s_id.sensor_x       = pos_id->sensor_x;
          s_id.sensor_y       = pos_id->sensor_y;
          s_id.sensor_angle   = pos_id->sensor_angle;
          s_id.value          = 0;
          s_id.valid_position = true;
          s_id.valid_standard = false;
        }
      else if (type == 0x02)
        {
          std_id = (struct standard_id_s *)gatt_char->value.data;
          s_id.x              = 0;
          s_id.y              = 0;
          s_id.angle          = std_id->angle;
          s_id.sensor_x       = 0;
          s_id.sensor_y       = 0;
          s_id.sensor_angle   = 0;
          s_id.value          = std_id->value;
          s_id.valid_position = false;
          s_id.valid_standard = true;
        }
      else if (type == 0x03)
        {
          //printf("Position ID missed\n");
          memset(&s_id, 0, sizeof(s_id));
          s_id.valid_position = false;
          s_id.valid_standard = false;
        }
      else if (type == 0x04)
        {
          //printf("Standard ID missed\n");
          memset(&s_id, 0, sizeof(s_id));
          s_id.valid_position = false;
          s_id.valid_standard = false;
        }
    }

    uint16_t x()             { return s_id.x; }
    uint16_t y()             { return s_id.y; }
    uint16_t angle()         { return s_id.angle; }
    uint16_t sensor_x()      { return s_id.sensor_x; }
    uint16_t sensor_y()      { return s_id.sensor_y; }
    uint16_t sensor_angle()  { return s_id.sensor_angle; }
    uint32_t value()         { return s_id.value; }
    bool is_valid_position() { return s_id.valid_position; }
    bool is_valid_standard() { return s_id.valid_standard; }

  private:
    static const BLE_UUID s_uuid;
    uint16_t m_handle;
};

struct Identification::id_s Identification::s_id = { 0 };
const BLE_UUID Identification::s_uuid = { BLE_UUID_TYPE_UUID128,
  { 0xae, 0xbb, 0xd7, 0xfc, 0x3e, 0xcf, 0x08, 0x95,
    0x71, 0x45, 0x3b, 0x5b, 0x01, 0x01, 0xb2, 0x10 }};

/****************************************************************************
 * Motor
 ****************************************************************************/

class Motor
{
  public:
    Motor(uint16_t handle) : m_handle(handle) {}
    ~Motor() {}

    int run(int8_t left_speed, int8_t right_speed, uint8_t duration = 0)
    {
      int ret;
      struct motor_control_t
      {
        uint8_t type;
        uint8_t left_id;
        uint8_t left_direction;
        uint8_t left_speed;
        uint8_t right_id;
        uint8_t right_direction;
        uint8_t right_speed;
        uint8_t duration; /* Actual period: duration x10 [msec] */
      } control =
      {
        .type            = 0x02,
        .left_id         = 0x01,
        .left_direction  = static_cast<uint8_t>((left_speed > 0) ? 1 : 2),
        .left_speed      = static_cast<uint8_t>(abs(left_speed)),
        .right_id        = 0x02,
        .right_direction = static_cast<uint8_t>((right_speed > 0) ? 1 : 2),
        .right_speed     = static_cast<uint8_t>(abs(right_speed)),
        .duration        = duration,
      };

      ret = write_gatt_char(m_handle, (uint8_t *)&control, sizeof(control), false);
      if (ret != 0)
        {
          printf("ERROR: %s() write ret=%d\n", __func__, ret);
        }

      return ret;
    }

    int stop()
    {
      return run(0, 0);
    }

  private:
    static const BLE_UUID s_uuid;
    uint16_t m_handle;
};

const BLE_UUID Motor::s_uuid = { BLE_UUID_TYPE_UUID128,
  { 0xae, 0xbb, 0xd7, 0xfc, 0x3e, 0xcf, 0x08, 0x95,
    0x71, 0x45, 0x3b, 0x5b, 0x02, 0x01, 0xb2, 0x10 }};

/****************************************************************************
 * Light
 ****************************************************************************/

class Light
{
  public:
    Light(uint16_t handle) : m_handle(handle) {}
    ~Light() {}

    int on(uint8_t red, uint8_t green, uint8_t blue, uint8_t duration = 0)
    {
      int ret;
      struct lamp_control_t
      {
        uint8_t type;
        uint8_t duration;
        uint8_t num;
        uint8_t id;
        uint8_t red;
        uint8_t green;
        uint8_t blue;
      } control =
      {
        .type     = 0x03,
        .duration = duration, /* Actual period: duration x10 [msec] */
        .num      = 0x01,
        .id       = 0x01,
        .red      = red,
        .green    = green,
        .blue     = blue,
      };

      ret = write_gatt_char(m_handle, (uint8_t *)&control, sizeof(control), true);
      if (ret != 0)
        {
          printf("ERROR: %s() write ret=%d\n", __func__, ret);
        }

      return ret;
    }

    int off(void)
    {
      int ret;
      uint8_t control = 0x01;

      ret = write_gatt_char(m_handle, (uint8_t *)&control, sizeof(control), true);
      if (ret != 0)
        {
          printf("ERROR: %s() write ret=%d\n", __func__, ret);
        }

      return ret;
    }

  private:
    static const BLE_UUID s_uuid;
    uint16_t m_handle;
};

const BLE_UUID Light::s_uuid = { BLE_UUID_TYPE_UUID128,
  { 0xae, 0xbb, 0xd7, 0xfc, 0x3e, 0xcf, 0x08, 0x95,
    0x71, 0x45, 0x3b, 0x5b, 0x03, 0x01, 0xb2, 0x10 }};

/****************************************************************************
 * Sound
 ****************************************************************************/

class Sound
{
  public:
    Sound(uint16_t handle) : m_handle(handle) {}
    ~Sound() {}

    enum EffectID
    {
      Enter = 0,
      Selected,
      Cancel,
      Cursor,
      MatIn,
      MatOut,
      Get1,
      Get2,
      Get3,
      Effect1,
      Effect2,
    };

    int play_sound_effect(EffectID id)
    {
      int ret;
      uint8_t control[] = { 0x02, static_cast<uint8_t>(id), 0x01 };

      ret = write_gatt_char(m_handle, (uint8_t *)&control, sizeof(control), true);
      if (ret != 0)
        {
          printf("ERROR: %s() write ret=%d\n", __func__, ret);
        }

      return ret;
    }

    int stop()
    {
      int ret;
      uint8_t control = 0x01;

      ret = write_gatt_char(m_handle, (uint8_t *)&control, sizeof(control), true);
      if (ret != 0)
        {
          printf("ERROR: %s() write ret=%d\n", __func__, ret);
        }

      return ret;
    }

  private:
    static const BLE_UUID s_uuid;
    uint16_t m_handle;
};

const BLE_UUID Sound::s_uuid = { BLE_UUID_TYPE_UUID128,
  { 0xae, 0xbb, 0xd7, 0xfc, 0x3e, 0xcf, 0x08, 0x95,
    0x71, 0x45, 0x3b, 0x5b, 0x04, 0x01, 0xb2, 0x10 }};

/****************************************************************************
 * Sensor
 ****************************************************************************/

class Sensor
{
  public:
    Sensor(uint16_t handle) : m_handle(handle)
    {
      start_notify(const_cast<BLE_UUID *>(&s_uuid), Sensor::notify);
    }

    ~Sensor()
    {
      stop_notify(const_cast<BLE_UUID *>(&s_uuid));
    }

    static uint8_t s_motion;

    static void notify(struct ble_gatt_char_s *gatt_char)
    {
      struct motion_data_t
      {
        uint8_t type;
        uint8_t tilt;
        uint8_t collision;
        uint8_t double_tap;
        uint8_t posture;
        uint8_t shake;
      } *data;

      data = (struct motion_data_t *)gatt_char->value.data;

      if (data->type == 0x01)
        {
          if (data->tilt == 0x00)
            {
              //printf("Motion: Tilt\n");
              s_motion |= FLAG_TILT;
            }
          if (data->collision == 0x01)
            {
              //printf("Motion: Collision\n");
              s_motion |= FLAG_COLLISTION;
            }
          if (data->double_tap == 0x01)
            {
              //printf("Motion: Double-Tap\n");
              s_motion |= FLAG_DOUBLE_TAP;
            }
          if (data->shake > 0x00)
            {
              //printf("Motion: Shake\n");
              s_motion |= FLAG_SHAKE;
            }
          s_motion &= ~FLAG_POSTURE;
          s_motion |= (data->posture & FLAG_POSTURE);
        }
    }

    bool tilt_detected()
    {
      bool detected = (s_motion & FLAG_TILT);
      s_motion &= ~FLAG_TILT;
      return detected;
    }

    bool collision_detected()
    {
      bool detected = (s_motion & FLAG_COLLISTION);
      s_motion &= ~FLAG_COLLISTION;
      return detected;
    }

    bool double_tap_detected()
    {
      bool detected = (s_motion & FLAG_DOUBLE_TAP);
      s_motion &= ~FLAG_DOUBLE_TAP;
      return detected;
    }

    bool shake_detected()
    {
      bool detected = (s_motion & FLAG_SHAKE);
      s_motion &= ~FLAG_SHAKE;
      return detected;
    }

    uint8_t posture()
    {
      return (s_motion & FLAG_POSTURE);
    }

  private:
    enum Flags
    {
      FLAG_POSTURE    = 0xf,
      FLAG_TILT       = 1 << 4,
      FLAG_COLLISTION = 1 << 5,
      FLAG_DOUBLE_TAP = 1 << 6,
      FLAG_SHAKE      = 1 << 7,
    };

  private:
    static const BLE_UUID s_uuid;
    uint16_t m_handle;
};

uint8_t Sensor::s_motion = 0;
const BLE_UUID Sensor::s_uuid = { BLE_UUID_TYPE_UUID128,
  { 0xae, 0xbb, 0xd7, 0xfc, 0x3e, 0xcf, 0x08, 0x95,
    0x71, 0x45, 0x3b, 0x5b, 0x06, 0x01, 0xb2, 0x10 }};

/****************************************************************************
 * Button
 ****************************************************************************/

class Button
{
  public:
    Button(uint16_t handle) : m_handle(handle)
    {
      start_notify(const_cast<BLE_UUID *>(&s_uuid), Button::notify);
    }

    ~Button()
    {
      stop_notify(const_cast<BLE_UUID *>(&s_uuid));
    }

    static uint8_t s_state;

    static void notify(struct ble_gatt_char_s *gatt_char)
    {
      struct button_data_t
      {
        uint8_t id;
        uint8_t state;
      } *data;

      data = (struct button_data_t *)gatt_char->value.data;

      if (data->id == 0x01)
        {
          //printf("Button: %s\n", (data->state == 0x80) ? "Pushed" : "Released");
          s_state = data->state;
        }
    }

    bool is_pressed()
    {
      return (s_state == 0x80);
    }

    bool is_released()
    {
      return (s_state != 0x80);
    }

  private:
    static const BLE_UUID s_uuid;
    uint16_t m_handle;
};

uint8_t Button::s_state = 0;
const BLE_UUID Button::s_uuid = { BLE_UUID_TYPE_UUID128,
  { 0xae, 0xbb, 0xd7, 0xfc, 0x3e, 0xcf, 0x08, 0x95,
    0x71, 0x45, 0x3b, 0x5b, 0x07, 0x01, 0xb2, 0x10 }};

/****************************************************************************
 * Battery
 ****************************************************************************/

class Battery
{
  public:
    Battery(uint16_t handle) : m_handle(handle) {}
    ~Battery() {}

    uint8_t level()
    {
      int ret;
      int len;
      uint8_t level = 0;

      ret = read_gatt_char(m_handle, &level, &len);
      if (ret != 0)
        {
          printf("ERROR: %s() read ret=%d\n", __func__, ret);
        }

      return level;
    }

  private:
    static const BLE_UUID s_uuid;
    uint16_t m_handle;
};

const BLE_UUID Battery::s_uuid = { BLE_UUID_TYPE_UUID128,
  { 0xae, 0xbb, 0xd7, 0xfc, 0x3e, 0xcf, 0x08, 0x95,
    0x71, 0x45, 0x3b, 0x5b, 0x08, 0x01, 0xb2, 0x10 }};

/****************************************************************************
 * Settings
 ****************************************************************************/

class Setting
{
  public:
    Setting(uint16_t handle) : m_handle(handle) {}
    ~Setting() {}

    const char *version()
    {
      int ret;
      int len;
      struct verinfo_s
      {
        uint8_t type;
        uint8_t reserved;
        char    version[6];
      } verinfo = {
        .type     = 0x01,
        .reserved = 0x00,
      };

      memset(verinfo.version, 0, sizeof(verinfo.version));

      ret = write_gatt_char(m_handle, (uint8_t *)&verinfo, 2, true);
      if (ret != 0)
        {
          printf("ERROR: %s() write ret=%d\n", __func__, ret);
        }

      ret = read_gatt_char(m_handle, (uint8_t *)&verinfo, &len);
      if (ret != 0)
        {
          printf("ERROR: %s() read ret=%d\n", __func__, ret);
        }

      strncpy(m_version, verinfo.version, sizeof(m_version));
      return m_version;
    }

  private:
    char m_version[6];
    static const BLE_UUID s_uuid;
    uint16_t m_handle;
};

const BLE_UUID Setting::s_uuid = { BLE_UUID_TYPE_UUID128,
  { 0xae, 0xbb, 0xd7, 0xfc, 0x3e, 0xcf, 0x08, 0x95,
    0x71, 0x45, 0x3b, 0x5b, 0xff, 0x01, 0xb2, 0x10 }};

#endif /* __EXAMPLES_BLE_TOIO_CENTRAL_TOIO_H */
