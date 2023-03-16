/****************************************************************************
 * modules/asmp/worker/arch/timer.c
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

#include <nuttx/arch.h>

#include <sys/types.h>

#include <inttypes.h>
#include <stdint.h>
#include <limits.h>
#include <assert.h>
#include <errno.h>

#include "arm_internal.h"
#include "hardware/cxd56_timer.h"

#include "timer.h"
#include "clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer divider definitions
 *
 * This definitions are not affected by NuttX configuration.
 * Please switch them manually which you want.
 */

#if defined(CONFIG_CXD56_TIMER_DIVIDER_1)
#define TIMER_DIVIDER       (1)
#define TIMERCTRL_DIV       (TIMERCTRL_DIV_1)
#elif defined(CONFIG_CXD56_TIMER_DIVIDER_16)
#define TIMER_DIVIDER       (16)
#define TIMERCTRL_DIV       (TIMERCTRL_DIV_16)
#elif defined(CONFIG_CXD56_TIMER_DIVIDER_256)
#define TIMER_DIVIDER       (256)
#define TIMERCTRL_DIV       (TIMERCTRL_DIV_256)
#else
#define TIMER_DIVIDER       (16)
#define TIMERCTRL_DIV       (TIMERCTRL_DIV_16)
#endif

/* e.g.) When the timer divider is 16, timer's max clock is about 10MHz
 * (Divide max 160MHz resolution by 16) and the timer has 32bit counter.
 * Therefore, the max counter is the following value to avoid counter
 * wrap around. Timer's base clock is dynamically changed with cpu clock.
 */

#define TIMER_MAXTIMEOUT    (ULONG_MAX / 160 / TIMER_DIVIDER)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t get_timer_base(int id)
{
  return id == 0 ? CXD56_TIMER0_BASE : CXD56_TIMER1_BASE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int timer_start(int id)
{
  if (id != 0 && id != 1)
    {
      return -EFAULT;
    }

  modreg32(TIMERCTRL_ENABLE, TIMERCTRL_ENABLE,
           get_timer_base(id) + CXD56_TIMER_CONTROL);

  return 0;
}

int timer_stop(int id)
{
  uint32_t base = get_timer_base(id);
  if (id != 0 && id != 1)
    {
      return -EFAULT;
    }

  putreg32(0, base + CXD56_TIMER_CONTROL);
  putreg32(0, base + CXD56_TIMER_LOAD);

  return 0;
}

int timer_settimeout(int id, uint32_t us)
{
  uint32_t clk;
  uint32_t load;
  uint32_t ctrl;
  uint32_t base = get_timer_base(id);

  if (id != 0 && id != 1)
    {
      return -EFAULT;
    }

  /* Can this timeout be represented? */

  if (us < 1 || us > TIMER_MAXTIMEOUT)
    {
      return -EINVAL;
    }

  ctrl = getreg32(base + CXD56_TIMER_CONTROL);
  if (ctrl & TIMERCTRL_ENABLE)
    {
      return -EBUSY;
    }

  /* Actual clock ticks */

  clk = clock_getcpubaseclock();

  load = (((uint64_t)us * clk) / TIMER_DIVIDER / 1000000);
  putreg32(load, base + CXD56_TIMER_LOAD);
  putreg32(TIMERCTRL_DIV | TIMERCTRL_SIZE_32BIT | TIMERCTRL_PERIODIC |
           TIMERCTRL_MODE_ONESHOT,
           get_timer_base(id) + CXD56_TIMER_CONTROL);

  return 0;
}

uint32_t timer_getvalue(int id)
{
  if (id != 0 && id != 1)
    {
      return 0;
    }

  return getreg32(get_timer_base(id) + CXD56_TIMER_VALUE);
}
