/****************************************************************************
 * modules/asmp/worker/arch/startup.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation.
 *
 *   Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sdk/config.h>
#include <arch/irq.h>
#include <asmp/mpmq.h>
#include <asmp/mpsignal.h>

#include "arm_arch.h"
#include "nvic.h"

#include "arch/intrinsics.h"
#include "arch/cpufifo.h"
#include "asmp.h"
#include "common.h"

extern uint32_t _sbss, _ebss;
extern int main(void);
extern void exception_common(void);
extern uint32_t __stack_pointer;
void __start(void);

unsigned __vectors[] __attribute__((section(".vectors"))) = {
  (unsigned)&__stack_pointer - 4,
  (unsigned)&__start,
  [2 ... (15 + 128)] = (unsigned)&exception_common,
};

#ifdef CONFIG_ARCH_FPU
#if defined(CONFIG_ARMV7M_CMNVECTOR) && !defined(CONFIG_ARMV7M_LAZYFPU)

static inline void fpuconfig(void)
{
  uint32_t regval;

  /* Set CONTROL.FPCA so that we always get the extended context frame
   * with the volatile FP registers stacked above the basic context.
   */

  regval = getcontrol();
  regval |= (1 << 2);
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to turn on CONTROL.FPCA for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~((1 << 31) | (1 << 30));
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= ((3 << (2*10)) | (3 << (2*11)));
  putreg32(regval, NVIC_CPACR);
}

#else

static inline void fpuconfig(void)
{
  uint32_t regval;

  /* Clear CONTROL.FPCA so that we do not get the extended context frame
   * with the volatile FP registers stacked in the saved context.
   */

  regval = getcontrol();
  regval &= ~(1 << 2);
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to keep CONTROL.FPCA off for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~((1 << 31) | (1 << 30));
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= ((3 << (2*10)) | (3 << (2*11)));
  putreg32(regval, NVIC_CPACR);
}

#endif

#else
#  define fpuconfig()
#endif

void __start(void)
{
  uint32_t *dest;

  /* Set PSP to IDLE stack */

  __asm__ __volatile__("\tmsr psp, %0\n" :
                       : "r" ((uint32_t)&_ebss+CONFIG_IDLETHREAD_STACKSIZE-4));

  up_irq_disable();

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  fpuconfig();

  _signal(2, MPSIGSTART, 0);

  up_irq_enable();

  wk_exit(main());
}

__attribute__((used))
void __debug_start(void)
{
  wfi();
  __start();
}
