/****************************************************************************
 * mbedtls/spresense_porting.h
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#ifndef SPRESENSE_PORTING_H
#define SPRESENSE_PORTING_H

#include <sdk/config.h>

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
#include <arch/chip/pm.h>
  /* If mbedtls_ssl_handshake is running in low clock mode (RCOSC mode),
   * the handshake may fail. Therefore, it is necessary to execute
   * HV lock before handshake and switch to high clock mode (HV mode).
   */

#define PM_CPU_FREQLOCK_INSTANCE() static struct pm_cpu_freqlock_s hv_lock;

#define PM_CPU_FREQLOCK_ACQUIRE(mode) \
  do \
    { \
      hv_lock.flag = mode; \
      up_pm_acquire_freqlock(&hv_lock); \
    } \
  while(0)

#define PM_CPU_FREQLOCK_RELEASE() \
  do \
    { \
      up_pm_release_freqlock(&hv_lock); \
    } \
  while(0)

#else

#define PM_CPU_FREQLOCK_INSTANCE()
#define PM_CPU_FREQLOCK_ACQUIRE(mode)
#define PM_CPU_FREQLOCK_RELEASE()

#endif /* CONFIG_CPUFREQ_RELEASE_LOCK */

#endif /* SPRESENSE_PORTING_H */
