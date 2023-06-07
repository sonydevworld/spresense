/****************************************************************************
 * modules/asmp/worker/arch/clock.c
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

#include <stdint.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/cxd56_crg.h"
#include "hardware/cxd5602_backupmem.h"
#include "hardware/cxd5602_topreg.h"

/*
 * This file is partially imported from cxd56_clock.c
 */

enum clock_source
{
  RCOSC = 1,
  RTC,
  RCRTC,
  XOSC,
  SYSPLL,
};

static uint32_t rcosc_clock = 0;

static uint32_t cxd56_get_clock(enum clock_source cs)
{
  if (!rcosc_clock)
    {
      rcosc_clock = BKUP->rcosc_clock;
    }

  switch (cs)
    {
    case RCOSC:
      return rcosc_clock;
    case RTC:
      return 32768;
    case RCRTC:
      return rcosc_clock / 250;
    case XOSC:
      return CONFIG_CXD56_XOSC_CLOCK;
    case SYSPLL:
      {
        uint32_t ctrl;
        uint32_t rc;
        uint32_t fb;

        ctrl = getreg32(CXD56_TOPREG_SYS_PLL_CTRL2);
        rc = ctrl >> 30;
        fb = (ctrl >> 27) & 0x7;

        switch (rc)
          {
          case 0:
            rc = 1;
            break;
          case 1:
            rc = 2;
            break;
          case 3:
            rc = 4;
            break;
          }

        switch (fb)
          {
          case 0:
            fb = 10;
            break;
          case 1:
            fb = 12;
            break;
          case 2:
            fb = 15;
            break;
          }

        return CONFIG_CXD56_XOSC_CLOCK * fb / rc;
      }
    }

  return 0;
}

static uint32_t cxd56_get_appsmp_baseclock(void)
{
  uint32_t val = getreg32(CXD56_TOPREG_APP_CKSEL);

  switch ((val >> 8) & 0x3)
    {
    case 0:
      return cxd56_get_clock(RCOSC);

    case 1:
      {
        uint32_t div = ((val >> 10) & 0x3) + 1;

        if (div == 4 && (val & (1 << 7)))
          {
            div = 5;
          }

        return cxd56_get_clock(SYSPLL) / div;
      }

    case 2:
      return cxd56_get_clock(XOSC);

    case 3:
      return cxd56_get_clock(RTC);
    }

  return 0;
}

uint32_t clock_getcpubaseclock(void)
{
  uint32_t val;
  int n;
  int m;

  val = getreg32(CXD56_CRG_GEAR_AHB);
  n = (val >> 16) & 0x7f;
  m = val & 0x7f;

  if (n && m)
    {
      return cxd56_get_appsmp_baseclock() * n / m;
    }
  else
    {
      return 0;
    }
}
