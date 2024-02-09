/****************************************************************************
 * examples/led_blink/led_blink_main.c
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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
#include <unistd.h>

#include <arch/board/board.h>
#include <arch/chip/pin.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin assignment to LEDs */

#define PIN_LED0 PIN_I2S1_BCK
#define PIN_LED1 PIN_I2S1_LRCK
#define PIN_LED2 PIN_I2S1_DATA_IN
#define PIN_LED3 PIN_I2S1_DATA_OUT

#define PATTERN_MASK  (0xf)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**** name: init_leds() */

static void init_leds(void)
{
  /* Set output to LED0 pin */

  board_gpio_write(PIN_LED0, -1);
  board_gpio_config(PIN_LED0, 0, false, true, PIN_FLOAT);

  /* Set output to LED1 pin */

  board_gpio_write(PIN_LED1, -1);
  board_gpio_config(PIN_LED1, 0, false, true, PIN_FLOAT);

  /* Set output to LED2 pin */

  board_gpio_write(PIN_LED2, -1);
  board_gpio_config(PIN_LED2, 0, false, true, PIN_FLOAT);

  /* Set output to LED3 pin */

  board_gpio_write(PIN_LED3, -1);
  board_gpio_config(PIN_LED3, 0, false, true, PIN_FLOAT);
}

/**** name: set_leds() */

static void set_leds(int ptn)
{
  board_gpio_write(PIN_LED0, (ptn & 0x01) ? 1 : 0);
  board_gpio_write(PIN_LED1, (ptn & 0x02) ? 1 : 0);
  board_gpio_write(PIN_LED2, (ptn & 0x04) ? 1 : 0);
  board_gpio_write(PIN_LED3, (ptn & 0x08) ? 1 : 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ptn;

  init_leds();

  ptn = 1;

  while (1)
    {
      set_leds(ptn);
      ptn = (ptn << 1) & PATTERN_MASK;

      if (ptn == 0)
        {
          ptn = 1;
        }

      usleep(100 * 1000);
    }

  return 0;
}
