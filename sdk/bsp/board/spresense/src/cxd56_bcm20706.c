/****************************************************************************
 * bsp/board/spresense/src/cxd56_bcm20706.c
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

#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <arch/board/board.h>

#include "cxd56_gpio.h"
#include "cxd56_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCM20707_RST_N    PIN_SEN_IRQ_IN
#define BCM20707_DEV_WAKE PIN_EMMC_DATA3
#define NUM_OF_PINS (sizeof(pin_cfg) / sizeof(pin_cfg[0]))

#define BCM20707_RST_DELAY  (50 * 1000)  /* ms */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct
{
  uint32_t pin;
  bool input_enable;
  bool init_val;
} pin_cfg[] =
{
  {BCM20707_RST_N,    false, false}, /* out, low */
  {BCM20707_DEV_WAKE, false, true},  /* out, high */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_bcm20706_pin_cfg
 *
 * Description:
 *   Setup pin configuration for bcm20706.
 *
 ****************************************************************************/

int board_bcm20706_pin_cfg(void)
{
  int ret = 0;
  int i = 0;
  for (i = 0; i < NUM_OF_PINS; ++i)
    {
      ret = cxd56_gpio_config(pin_cfg[i].pin, pin_cfg[i].input_enable);
      if (!pin_cfg[i].input_enable)
        {
          cxd56_gpio_write(pin_cfg[i].pin, pin_cfg[i].init_val);
        }
    }
  return ret;
}

/****************************************************************************
 * Name: board_bcm20706_uart_pin_cfg
 *
 * Description:
 *   Setup UART pin configuration for bcm20706.
 *
 ****************************************************************************/

int board_bcm20706_uart_pin_cfg(void)
{
  int ret = 0;

  /* Set float for UART2 CTS */

  ret = board_gpio_config(PIN_UART2_CTS, 1, 1, 0, 0);

  return ret;
}

/****************************************************************************
 * Name: board_bcm20706_reset
 *
 * Description:
 *   Reset BCM20706.
 *
 ****************************************************************************/

void board_bcm20706_reset(void)
{
  cxd56_gpio_write(BCM20707_RST_N, false);
  usleep(BCM20707_RST_DELAY);
  cxd56_gpio_write(BCM20707_RST_N, true);
  usleep(BCM20707_RST_DELAY);
}

/****************************************************************************
 * Name: board_bcm20706_power_control
 *
 * Description:
 *   Power ON/OFF BCM20706.
 *
 ****************************************************************************/

int board_bcm20706_power_control(bool en)
{
  int ret = 0;
  ret = board_power_control(POWER_BTBLE, en);
  return ret;
}

/****************************************************************************
 * Name: board_bcm20706_enable_sleep
 *
 * Description:
 *   Sleep mode ON/OFF BCM20706.
 *
 ****************************************************************************/

void board_bcm20706_enable_sleep(bool en)
{
  cxd56_gpio_write(BCM20707_DEV_WAKE, en);
}

