/****************************************************************************
 * bsp/board/collet/src/cxd56_buttons.c
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
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"
#include "cxd56_pinconfig.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef BOARD_NUM_BUTTONS
#  error "BOARD_NUM_BUTTONS must be defined in board.h !!"
#endif

#define GPIO_BUT1           (PIN_SPI2_MOSI)
#define GPIO_BUT2           (PIN_SPI2_MISO)

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Pin configuration for each CXD56XX button.  This array is indexed
 * by the GPIO_* definitions.
 */

static const uint16_t g_buttoncfg[BOARD_NUM_BUTTONS] =
{
  GPIO_BUT1, GPIO_BUT2
};

/* This array defines all of the interrupt handlers current attached to
 * button events.
 */

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_CXD56_GPIO_IRQ)
static xcpt_t g_buttonisr[BOARD_NUM_BUTTONS];
#endif

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

void board_button_initialize(void)
{
  cxd56_gpio_config(GPIO_BUT1, true);
  cxd56_gpio_config(GPIO_BUT2, true);
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint8_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < BOARD_NUM_BUTTONS; i++)
    {
       /* A low value means that the key is pressed. */

       bool depressed = !cxd56_gpio_read(g_buttoncfg[i]);

       /* Accumulate the set of depressed (not released) keys */

       if (depressed)
         {
            ret |= (1 << i);
         }
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
#ifdef CONFIG_CXD56_GPIO_IRQ
  irqstate_t flags;

  /* Verify that the button ID is within range */

  if ((unsigned)id < BOARD_NUM_BUTTONS)
    {
      /* Return the current button handler and set the new interrupt handler */

      g_buttonisr[id] = irqhandler;

      /* Disable interrupts until we are done */

      flags = enter_critical_section();

      /* Configure the interrupt.  Either attach and enable the new
       * interrupt or disable and detach the old interrupt handler.
       */

      if (irqhandler)
        {
          /* Attach then enable the new interrupt handler */

          cxd56_gpioint_config(g_buttoncfg[id],
                               GPIOINT_TOGGLE_MODE_MASK |
                               GPIOINT_NOISE_FILTER_ENABLE |
                               GPIOINT_LEVEL_HIGH, irqhandler);

          cxd56_gpioint_enable(g_buttoncfg[id]);
        }
      else
        {
          /* Disable then detach the old interrupt handler */

          cxd56_gpioint_disable(g_buttoncfg[id]);
        }
      leave_critical_section(flags);
    }
#else
  _err("ERROR: Not found gpio interrupt driver.\n");
#endif
  return OK;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
