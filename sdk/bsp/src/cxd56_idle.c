/****************************************************************************
 * bsp/src/cxd56_idle.c
 *
 *   Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <arch/board/board.h>
#include <sdk/config.h>
#include <sdk/debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/power/pm.h>

#include <nuttx/irq.h>

#include "up_internal.h"
#if defined(CONFIG_CXD56_HOT_SLEEP)
#include "cxd56_powermgr.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Does the board support an IDLE LED to indicate that the board is in the
 * IDLE state?
 */

#if defined(CONFIG_ARCH_LEDS) && defined(LED_IDLE)
#  define BEGIN_IDLE() board_autoled_on(LED_IDLE)
#  define END_IDLE()   board_autoled_off(LED_IDLE)
#else
#  define BEGIN_IDLE()
#  define END_IDLE()
#endif

#if defined(CONFIG_CXD56_HOT_SLEEP)
#  ifndef CONFIG_CXD56_HOT_SLEEP_WAIT_COUNT
#    define CONFIG_CXD56_HOT_SLEEP_WAIT_COUNT (20)
#  endif
#  ifndef CONFIG_CXD56_HOT_SLEEP_THRESHOLD
#    define CONFIG_CXD56_HOT_SLEEP_THRESHOLD  (1000)
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_CXD56_HOT_SLEEP)
static systime_t g_hotsleep_count = 0;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_idlepm(void)
{
  static enum pm_state_e oldstate = PM_NORMAL;
  enum pm_state_e newstate;
  irqstate_t flags;
  int ret;

  /* Decide, which power saving level can be obtained */

  newstate = pm_checkstate();

  /* Check for state changes */

  if (newstate != oldstate)
    {
      flags = enter_critical_section();

      /* Perform board-specific, state-dependent logic here */

      logdebug("newstate= %d oldstate=%d\n", newstate, oldstate);

      /* Then force the global state change */

      ret = pm_changestate(newstate);
      if (ret < 0)
        {
          /* The new state change failed, revert to the preceding state */

          (void)pm_changestate(oldstate);
        }
      else
        {
          /* Save the new state */

          oldstate = newstate;
        }

      /* MCU-specific power management logic */

      switch (newstate)
        {
          case PM_NORMAL:
            break;

          case PM_IDLE:
            break;

          case PM_STANDBY:
            cxd56_pmstandby(true);
            break;

          case PM_SLEEP:
            (void)cxd56_pmsleep();
            break;

          default:
            break;
        }

      leave_critical_section(flags);
    }
}
#else
#  define up_idlepm()
#endif

#if defined(CONFIG_CXD56_HOT_SLEEP)
static void cxd56_idle_hotsleep(void)
{
  int idletime_tick;
  int idletime;

  idletime_tick = wd_get_expect_idletime();
  if (idletime_tick == 0)
    {
      idletime = 0x7fffffff;
    }
  else
    {
      idletime = (idletime_tick * CONFIG_USEC_PER_TICK) / USEC_PER_MSEC;
    }

  if (idletime >= CONFIG_CXD56_HOT_SLEEP_THRESHOLD)
    {
      cxd56_pm_hotsleep(idletime);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when their is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_CXD56_HOT_SLEEP)
  systime_t prev, current, elapsed_time;
#endif

#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  sched_process_timer();
#else

  /* Perform IDLE mode power management */

  up_idlepm();

  /* Sleep until an interrupt occurs to save power */

#  if defined(CONFIG_CXD56_HOT_SLEEP)
  prev = clock_systimer();
#  endif

  BEGIN_IDLE();
  asm("WFI");
  END_IDLE();

#  if defined(CONFIG_CXD56_HOT_SLEEP)
  current = clock_systimer();
  elapsed_time = current - prev;
  if (((elapsed_time * CONFIG_USEC_PER_TICK) / USEC_PER_MSEC) <
      (CONFIG_USEC_PER_TICK / USEC_PER_MSEC))
    {
      /* not SYSTICK interrupt */
      g_hotsleep_count = 0;
    }
  else
    {
      g_hotsleep_count = g_hotsleep_count + 1;
    }

  if (g_hotsleep_count >= CONFIG_CXD56_HOT_SLEEP_WAIT_COUNT)
    {
      g_hotsleep_count = 0;
      cxd56_idle_hotsleep();
    }
#  endif
#endif
}
