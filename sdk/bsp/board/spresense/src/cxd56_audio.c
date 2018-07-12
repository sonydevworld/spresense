/****************************************************************************
 * bsp/board/spresense/src/cxd56_audio.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "chip.h"
#include "up_arch.h"

#include <arch/board/board.h>
#include "cxd56_pmic.h"
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef CXD5247_XRST
#  error "CXD5247_XRST must be defined in board.h !!"
#endif
#ifndef CXD5247_AVDD
#  error "CXD5247_AVDD must be defined in board.h !!"
#endif
#ifndef CXD5247_DVDD
#  error "CXD5247_DVDD must be defined in board.h !!"
#endif

#define MUTE_OFF_DELAY  (1250 * 1000) /* ms */
#define MUTE_ON_DELAY   (150 * 1000) /* ms */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_aca_power_control
 *
 * Description:
 *   Power on/off the Aca device on the board.
 *
 ****************************************************************************/

int board_aca_power_control(int target, bool en)
{
  int ret = 0;
  static int first = 1;
  static bool avdd_on = false;
  static bool dvdd_on = false;

  if (first)
    {
      /* gpio configuration (output disabled yet) */

      cxd56_gpio_config(CXD5247_XRST, false);

      first = 0;
    }

  if (en)
    {
      if (!dvdd_on && (target & CXD5247_DVDD))
        {
          /* reset assert */
          cxd56_gpio_write(CXD5247_XRST, false);
        }

      /* power on */
      if (!avdd_on && (target & CXD5247_AVDD))
        {
          board_power_control(POWER_AUDIO_AVDD, true);
          avdd_on = true;
        }
      if (!dvdd_on && (target & CXD5247_DVDD))
        {
          board_power_control(POWER_AUDIO_DVDD, true);
          dvdd_on = true;

          /* reset release */
          cxd56_gpio_write(CXD5247_XRST, true);
        }
    }
  else
    {
      if (dvdd_on && (target & CXD5247_DVDD))
        {
          /* reset assert */
          cxd56_gpio_write(CXD5247_XRST, false);
        }

      /* power off */
      if (avdd_on && (target & CXD5247_AVDD))
        {
          board_power_control(POWER_AUDIO_AVDD, false);
          avdd_on = false;
        }
      if (dvdd_on && (target & CXD5247_DVDD))
        {
          board_power_control(POWER_AUDIO_DVDD, false);
          dvdd_on = false;
        }
    }
  return ret;
}

/****************************************************************************
 * Name: board_aca_power_monitor
 *
 * Description:
 *   Get status of Power on/off the Aca device on the board.
 *
 ****************************************************************************/

bool board_aca_power_monitor(int target)
{
  bool avdd_stat = true;
  bool dvdd_stat = true;

  if (target & CXD5247_AVDD)
    {
      avdd_stat = board_power_monitor(POWER_AUDIO_AVDD);
    }
  if (target & CXD5247_DVDD)
    {
      dvdd_stat = board_power_monitor(POWER_AUDIO_DVDD);
    }

  return avdd_stat && dvdd_stat;
}

/****************************************************************************
 * Name: board_external_amp_mute_control
 *
 * Description:
 *   External Amp. Mute on/off.
 *    true:  Mute on
 *    false: Mute off
 *
 ****************************************************************************/

int board_external_amp_mute_control(bool en)
{
  int ret = 0;

  if (en)
    {
      /* Mute ON */

      ret = board_power_control(POWER_AUDIO_MUTE, false);
      usleep(MUTE_ON_DELAY);
    }
  else
    {
      /* Mute OFF */

      usleep(MUTE_OFF_DELAY);
      ret = board_power_control(POWER_AUDIO_MUTE, true);
    }

  return ret;
}

/****************************************************************************
 * Name: board_external_amp_mute_monitor
 *
 * Description:
 *   Get External Amp. Mute status.
 *    true:  Mute on
 *    false: Mute off
 *
 ****************************************************************************/

bool board_external_amp_mute_monitor(void)
{
  bool mute = board_power_monitor(POWER_AUDIO_MUTE);
  return !mute;
}

