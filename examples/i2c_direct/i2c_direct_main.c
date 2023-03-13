/**
 * examples/i2c_direct/i2c_direct_main.c
 *
 *  Copyright (C) 2022, Ixy Design Studio, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

#include "debug_printf.h"
#include "i2c_common.h"
#include "i2c_bmi270.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LED0  PIN_I2S1_DATA_OUT
#define LED1  PIN_I2S1_DATA_IN
#define LED2  PIN_I2S1_LRCK
#define LED3  PIN_I2S1_BCK 

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void gpio_setup_direction(int id, bool input);
static void gpio_set(int id);
static void gpio_clear(int id);
static void init_pins(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief Initialize I/O pins
 *
 */

static void
init_pins
(void)
{
  gpio_setup_direction(LED0, false);
  gpio_setup_direction(LED1, false);
  gpio_setup_direction(LED2, false);
  gpio_setup_direction(LED3, false);
}

/**
 * @brief GPIO Setup
 *
 * @param id pin number
 * @param input input=true, output=false
 */

static void
gpio_setup_direction
(int id, bool input)
{
  int _gabege_value = 0;
  if (input == true)
    {
      /* Input Pin Config */

      board_gpio_config(id, 0, true, false, PIN_PULLUP);
      _gabege_value = board_gpio_read(id);
    }
  else
    {
      /* Output Pin Config */

      board_gpio_config(id, 0, false, true, 0);
      board_gpio_write(id, _gabege_value);
    }

  return;
}

/**
 * @brief GPIO Write = 1
 *
 * @param id pin number
 */

static void
gpio_set
(int id)
{
  board_gpio_write(id, 1);
}

/**
 * @brief GPIO Write = 0
 *
 * @param id pin number
 */

static void
gpio_clear
(int id)
{
  board_gpio_write(id, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief main - init BMI270 and measurement 512 times.
 *
 * @param argc UNUSED
 * @param argv UNUSED
 * @return int success == 0
 */

int
main
(int argc, char *argv[])
{
  int fd;
  int ret;
  int total = 512;
  struct timespec waittime;
  axis_t acc_data;
  axis_t gyr_data;
  i2c_bmi270_t bmi270 =
    {
      0
    };

#ifdef CONFIG_SCU_SENSORS
  printf("This sample could not work with CONFIG_SCU_SENSORS\n");
  return -1;
#endif

  /* I2C confiuguration */

  bmi270.i2c.fd = -1;
  bmi270.i2c.i2c_addr = BMI270_I2C_ADDRESS;
  bmi270.i2c.speed = I2C_SPEED;

  /** init LED */

  init_pins();

  /** open I2C bus */

  fd = open(I2C_DEVNAME_FOR_BMI270, O_WRONLY);
  if (fd < 0)
    {
      printf("ERROR: Failed to open %s: %d\n",
        I2C_DEVNAME_FOR_BMI270, errno);
      return -1;
    }

  bmi270.i2c.fd = fd;

  /** init bmi270 */

  ret = init_bmi270(&bmi270);
  if (ret < 0)
    {
      printf("ERROR: Failed to initialize: %d\n", ret);
      goto error_on_using_bmi270;
    }

  /* -- WAIT 250ms -- */

  waittime.tv_sec = 0;
  waittime.tv_nsec = 250 * 1000 * 1000;
  nanosleep(&waittime, NULL);

  /** mesurement */

  for (int i = 0; i < total; i++)
    {
      ret = exec_dequeue_fifo(&bmi270);
      if (ret < 0)
        {
          printf("ERROR: Failed to Dequeue: %d\n", ret);
          goto error_on_using_bmi270;
        }

      ret = get_latest_acc(&acc_data, &bmi270);
      ret = get_latest_gyr(&gyr_data, &bmi270);
      printf("(%3d/%3d) ACC|%6d,%6d,%6d || GYR|%6d,%6d,%6d\n",
        i, total - 1,
        acc_data.x, acc_data.y, acc_data.z,
        gyr_data.x, gyr_data.y, gyr_data.z);

      gpio_clear(LED0);
      gpio_clear(LED1);
      gpio_clear(LED2);
      gpio_clear(LED3);

      /* set LED */

      if (acc_data.x < -1500)
        {
          gpio_set(LED0);
        }
      else if (acc_data.x < -1000)
        {
          gpio_set(LED0);
          gpio_set(LED1);
        }
      else if (acc_data.x < -400)
        {
          gpio_set(LED1);
        }
      else if ((acc_data.x >= -400) && (acc_data.x <= 400))
        {
          gpio_set(LED1);
          gpio_set(LED2);
        }
      else if (acc_data.x > 1500)
        {
          gpio_set(LED3);
        }
      else if (acc_data.x > 1000)
        {
          gpio_set(LED3);
          gpio_set(LED2);
        }
      else
        {
          gpio_set(LED2);
        }

      /* -- WAIT 50ms -- */

      waittime.tv_sec = 0;
      waittime.tv_nsec = 50 * 1000 * 1000;
      nanosleep(&waittime, NULL);
    }

  ret = 0;

error_on_using_bmi270:

  /* clear LEDS */

  gpio_clear(LED0);
  gpio_clear(LED1);
  gpio_clear(LED2);
  gpio_clear(LED3);

  /** close I2C bus */

  close(fd);

  /** init bmi270 */

  fini_bmi270(&bmi270);

  return ret;
}
