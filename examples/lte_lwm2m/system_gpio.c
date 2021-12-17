/****************************************************************************
 * lte_lwm2m/system_gpio.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

#include "lwm2mclient.h"
#include "liblwm2m.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gpio_handler(int irq, FAR void *context, FAR void *arg)
{
  uint32_t pin = (uint32_t)arg;

  digital_input_setCounter((uint16_t)pin);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gpio_input_config(uint16_t id, bool polarity)
{
  int pull;

  pull = (polarity) ? PIN_PULLUP : PIN_PULLDOWN;

  return board_gpio_config(id, 0, true, false, pull);
}

int gpio_output_config(uint16_t id)
{
  return board_gpio_config(id, 0, false, true, 0);
}

int gpio_read(uint16_t id, bool polarity)
{
  int value;

  value = board_gpio_read(id);

  if (polarity)
    {
      value ^= 1;
    }

  digital_input_setValue(id, (value) ? true : false);

  return value;
}

int gpio_interrupt(uint16_t id, int selection)
{
  int ret = 0;
  int mode = 0;
  bool filter = false;

  switch (selection)
    {
    case 1: mode = INT_FALLING_EDGE; break;
    case 2: mode = INT_RISING_EDGE; break;
    case 3: mode = INT_BOTH_EDGE; break;
    default: break;
    }

  if (mode != 0)
    {
      /* enable interrupt */

      ret = board_gpio_intconfig(id, mode, filter, gpio_handler);
      if (ret < 0)
        {
          return ret;
        }
      board_gpio_int(id, true);
    }
  else
    {
      /* disable interrupt */

      board_gpio_int(id, false);
    }

  return ret;
}

int gpio_write(uint16_t id, bool state, bool polarity)
{
  int value;

  if (state)
    {
      value = (polarity) ? 0 : 1;
    }
  else
    {
      value = (polarity) ? 1 : 0;
    }

  board_gpio_write(id, value);

  return 0;
}

