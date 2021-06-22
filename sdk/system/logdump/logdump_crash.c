/****************************************************************************
 * system/logdump/logdump_crash.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>

#include <sys/stat.h>
#include <fcntl.h>

#include <arch/chip/backuplog.h>
#include <arch/chip/crashdump.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_TIME_STRING 80

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void stackdump(uint32_t *stack, int size)
{
  int i;

  for (i = 0; i < size; i++)
    {
      printf("%08lx ", stack[i]);
      if (7 == (i % 8))
        {
          printf("\n");
        }
    }
  printf("\n");
}

static void regdump(uint32_t *regs, int size)
{
  printf("S0:  %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
         regs[REG_S0], regs[REG_S1], regs[REG_S2], regs[REG_S3],
         regs[REG_S4], regs[REG_S5], regs[REG_S6], regs[REG_S7]);
  printf("S8:  %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
         regs[REG_S8], regs[REG_S9], regs[REG_S10], regs[REG_S11],
         regs[REG_S12], regs[REG_S13], regs[REG_S14], regs[REG_S15]);
  printf("S16: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
         regs[REG_S16], regs[REG_S17], regs[REG_S18], regs[REG_S19],
         regs[REG_S20], regs[REG_S21], regs[REG_S22], regs[REG_S23]);
  printf("S24: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
         regs[REG_S24], regs[REG_S25], regs[REG_S26], regs[REG_S27],
         regs[REG_S28], regs[REG_S29], regs[REG_S30], regs[REG_S31]);
  printf("FPSCR: %08lx\n", regs[REG_FPSCR]);
  printf("R0:  %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
         regs[REG_R0], regs[REG_R1], regs[REG_R2], regs[REG_R3],
         regs[REG_R4], regs[REG_R5], regs[REG_R6], regs[REG_R7]);
  printf("R8:  %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
         regs[REG_R8], regs[REG_R9], regs[REG_R10], regs[REG_R11],
         regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
#ifdef CONFIG_ARMV7M_USEBASEPRI
#  ifdef REG_EXC_RETURN
  printf("xPSR: %08lx BASEPRI: %08lx EXC_RETURN: %08lx\n",
         regs[REG_XPSR], regs[REG_BASEPRI], regs[REG_EXC_RETURN]);
#  else
  printf("xPSR: %08lx BASEPRI: %08lx\n",
         regs[REG_XPSR], regs[REG_BASEPRI]);
#  endif
#else
#  ifdef REG_EXC_RETURN
  printf("xPSR: %08lx PRIMASK: %08lx EXC_RETURN: %08lx\n",
         regs[REG_XPSR], regs[REG_PRIMASK], regs[REG_EXC_RETURN]);
#  else
  printf("xPSR: %08lx PRIMASK: %08lx\n",
         regs[REG_XPSR], regs[REG_PRIMASK]);
#  endif
#endif /* CONFIG_ARMV7M_USEBASEPRI */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int logdump_crash(void *buf, size_t size)
{
  fullcontext_t *pdump = buf;
#if defined(CONFIG_LIBC_LOCALTIME) || defined(CONFIG_TIME_EXTENDED)
  static const char format[] = "%a, %b %d %H:%M:%S %Y";
#else
  static const char format[] = "%b %d %H:%M:%S %Y";
#endif
  struct tm tm;
  char timbuf[MAX_TIME_STRING];
  int i;

  for (i = 0; i < size; i += CRASHLOG_SIZE, pdump++)
    {
      gmtime_r(&pdump->info.ts.tv_sec, &tm);
      strftime(timbuf, MAX_TIME_STRING, format, &tm);
      printf("date: %s\n", timbuf);
      printf("file: %s line: %d ", pdump->info.filename, pdump->info.lineno);
#if CONFIG_TASK_NAME_SIZE > 0
      printf("task: %s ", pdump->info.name);
#endif
      printf("pid: %d\n", pdump->info.pid);
      printf("user sp: %08lx\n", pdump->info.stacks.user.sp);
      printf("stack base: %08lx\n", pdump->info.stacks.user.top);
      printf("stack size: %08lx\n", pdump->info.stacks.user.size);
#if CONFIG_ARCH_INTERRUPTSTACK > 3
      printf("int sp: %08lx\n", pdump->info.stacks.interrupt.sp);
      printf("stack base: %08lx\n", pdump->info.stacks.interrupt.top);
      printf("stack size: %08lx\n", pdump->info.stacks.interrupt.size);
#endif
      printf("regs:\n");
      regdump(pdump->info.regs, XCPTCONTEXT_REGS);
#if CONFIG_ARCH_INTERRUPTSTACK > 3
      printf("interrupt stack:\n");
      stackdump(pdump->istack, CONFIG_ISTACK_SIZE);
#endif
      printf("user stack:\n");
      stackdump(pdump->ustack, CONFIG_USTACK_SIZE);
    }
  return 0;
}

