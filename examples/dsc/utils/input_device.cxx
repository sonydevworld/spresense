/****************************************************************************
 * examples/dsc/utils/input_device.cxx
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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
#include <stdint.h>

#ifdef CONFIG_EXAMPLES_DSC_KEYBOARD_INPUT
# include <stdio.h>
# include <termios.h>
# include <fcntl.h>
#endif

#include <arch/board/board.h>
#include <arch/chip/pin.h>

#include "simple_menu.h"
#include "input_device.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define BUTTON_DECIDE PIN_I2S0_BCK
#define BUTTON_NEXT   PIN_I2S0_DATA_IN

#define BUTTON_STATUS_BIT_DECIDE  (0x01)
#define BUTTON_STATUS_BIT_NEXT    (0x02)

#define DECIDE_BTN_PRESSED(btn) ((btn & BUTTON_STATUS_BIT_DECIDE) == 0)
#define NEXT_BTN_PRESSED(btn)   ((btn & BUTTON_STATUS_BIT_NEXT)   == 0)

#define DECIDE_BTN_DOWN(btn) (btn & BUTTON_STATUS_BIT_DECIDE)
#define NEXT_BTN_DOWN(btn)   (btn & BUTTON_STATUS_BIT_NEXT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_DSC_KEYBOARD_INPUT
static struct termios original_setting;
#else
static uint32_t g_last_value;
#endif

#ifdef CONFIG_EXAMPLES_DSC_KEYBOARD_INPUT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int set_nonblocking(struct termios *saved)
{
	struct termios settings;

	tcgetattr(0, saved);
	settings = *saved;

	settings.c_lflag &= ~(ECHO|ICANON);
	settings.c_cc[VTIME] = 0;
	settings.c_cc[VMIN] = 1;
	tcsetattr(0,TCSANOW,&settings);
	fcntl(0,F_SETFL,O_NONBLOCK);

  return OK;
}

static void store_setting(struct termios *saved)
{
	tcsetattr(0,TCSANOW,saved);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern "C" int inputdev_initialize(void)
{
  return set_nonblocking(&original_setting);
}

extern "C" void inputdev_finalize(void)
{
  store_setting(&original_setting);
}

extern "C" int inputdev_update(void)
{
  int c;
  int key = MENU_KEY_NONE;

  c = getchar();
  if (c != EOF)
    {
      switch (c)
        {
          case 'h':
            key = MENU_KEY_LEFT;
            break;
          case 'k':
            key = MENU_KEY_UP;
            break;
          case 'j':
            key = MENU_KEY_DOWN;
            break;
          case ' ':
            key = MENU_KEY_SELECT;
            break;
        }
    }

  return key;
}

#else /* ifdef CONFIG_EXAMPLES_DSC_KEYBOARD_INPUT */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t button_status(void)
{
  return ((uint32_t)board_gpio_read(BUTTON_DECIDE) & 0x01) +
        (((uint32_t)board_gpio_read(BUTTON_NEXT)   & 0x01) << 1);
}

static uint32_t button_trigger(void)
{
  uint32_t ret;
  uint32_t current = button_status();
  
  ret = (~current) & (current ^ g_last_value);

  g_last_value = current;

  return ret;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern "C" int inputdev_initialize(void)
{
  board_gpio_config(BUTTON_DECIDE, 0, true, true, PIN_PULLUP);
  board_gpio_config(BUTTON_NEXT,   0, true, true, PIN_PULLUP);

  g_last_value = button_status();

  return OK;
}

extern "C" void inputdev_finalize(void)
{
}

extern "C" int inputdev_update(void)
{
  uint32_t btn;
  int key = MENU_KEY_NONE;

  btn = button_trigger();
  if (DECIDE_BTN_DOWN(btn))
    {
      key = MENU_KEY_SELECT;
    }
  else if (NEXT_BTN_DOWN(btn))
    {
      key = MENU_KEY_DOWN;
    }

  return key;
}

#endif
