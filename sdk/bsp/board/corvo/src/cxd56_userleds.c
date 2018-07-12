/****************************************************************************
 * bsp/board/corvo/src/cxd56_userleds.c
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

#include <sdk/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#ifdef CONFIG_USERLED_LOWER
#include <nuttx/leds/userled.h>
#endif

#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef BOARD_LED1_BIT
#  error "BOARD_LED1_BIT must be defined in board.h !!"
#endif
#ifndef BOARD_LED2_BIT
#  error "BOARD_LED2_BIT must be defined in board.h !!"
#endif

#define GPIO_LED1           (PIN_PWM0)
#define GPIO_LED2           (PIN_PWM1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

void board_userled_initialize(void)
{
  cxd56_gpio_config(GPIO_LED1, false);
  cxd56_gpio_config(GPIO_LED2, false);
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint32_t pin = (led == BOARD_LED1 ? GPIO_LED1 : GPIO_LED2);
  cxd56_gpio_write(pin, ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint8_t ledset)
{
  cxd56_gpio_write(GPIO_LED1, (bool)(ledset & BOARD_LED1_BIT));
  cxd56_gpio_write(GPIO_LED2, (bool)(ledset & BOARD_LED2_BIT));
}

#endif /* !CONFIG_ARCH_LEDS */
