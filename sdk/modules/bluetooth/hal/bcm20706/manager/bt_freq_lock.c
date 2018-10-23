/****************************************************************************
 * modules/bluetooth/hal/bcm20706/manager/bt_freq_lock.c
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

#include <stdio.h>
#include <stdint.h>
#include <arch/chip/pm.h>

#include "bt_debug.h"
#include "bt_util.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pm_cpu_freqlock_s g_hv_lock_0 =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('H','B', 0), PM_CPUFREQLOCK_FLAG_HV);
static struct pm_cpu_freqlock_s g_hv_lock_1 =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('H','B', 1), PM_CPUFREQLOCK_FLAG_HV);
static struct pm_cpu_freqlock_s g_lv_lock =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('L','B', 0), PM_CPUFREQLOCK_FLAG_LV);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void btAcquireLowFrequency(void)
{
  up_pm_acquire_freqlock(&g_lv_lock);
}

void btAcquireHighFrequency(void)
{
  up_pm_acquire_freqlock(&g_hv_lock_0);
}

/* This function is used for prevent frequency change during UART buadrate change */

void btLockFrequency(void)
{
  up_pm_acquire_freqlock(&g_hv_lock_1);
}

void btReleaseLowFrequency(void)
{
  up_pm_release_freqlock(&g_lv_lock);
}

void btReleaseHighFrequency(void)
{
  up_pm_release_freqlock(&g_hv_lock_0);
}

/* When baudrate change is finished, release the frequency lock */

void btReleaseFrequency(void)
{
  up_pm_release_freqlock(&g_hv_lock_1);
}

void btChangeFreLock(uint32_t baudrate)
{
  if (baudrate <= BT_LOCK_ROSC_UART_BAUD_RATE)
    {
      btReleaseLowFrequency();
      btReleaseHighFrequency();
      btdbg("bt release low and high frequency lock\n");
    }
  else if (baudrate <= BT_LOCK_LV_UART_BAUD_RATE)
    {
      btAcquireLowFrequency();
      btReleaseHighFrequency();
      btdbg("bt release high and acquire low frequency lock\n");
    }
  else
    {
      btAcquireHighFrequency();
      btReleaseLowFrequency();
      btdbg("bt acquire high frequency and release low frequency lock\n");
    }
}

