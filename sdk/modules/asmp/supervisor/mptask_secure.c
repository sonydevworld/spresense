/****************************************************************************
 * modules/asmp/supervisor/mptask_secure.c
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

#include <sys/stat.h>

#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <mm/tile.h>
#include <asmp/types.h>
#include <asmp/mptask.h>

#include <arch/chip/pm.h>

#include "up_arch.h"
#include "cxd56_clock.h"
#include "cxd56_sysctl.h"

#include "mptask.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOG2BASETILE 17

static int mptask_loadfw(mptask_t *task)
{
  sysctl_loadfw_t arg;

  arg.cpuid = mptask_getcpuid(task);
  arg.addr  = task->loadaddr;
  strncpy(arg.filename, task->filename, sizeof(arg.filename) - 1);
  arg.filename[sizeof(arg.filename) - 1] = '\0';

  return cxd56_sysctlcmd(SYSCTL_LOADFW, (uint32_t)(uintptr_t)&arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mptask_init_secure(mptask_t *task, const char *filename)
{
  sysctl_getfwsize_t arg;
  int i;
  int size;

  if (!task)
    {
      return -EINVAL;
    }
  if (strlen(filename) > 31)
    {
      return -EINVAL;
    }

  memset(task, 0, sizeof(mptask_t));

  /* Set all of CPUs to permitted */

  for (i = 1; i < NMPCPUS; i++)
    {
      CPU_SET(i, &task->attr.affinity);
    }

  /* Ask the firmware has been installed to system CPU. */

  strncpy(arg.filename, filename, sizeof(arg.filename) - 1);
  arg.filename[sizeof(arg.filename) - 1] = '\0';
  size = cxd56_sysctlcmd(SYSCTL_GETFWSIZE, (uint32_t)(uintptr_t)&arg);
  if (size <= 0)
    {
      return -ENOENT;
    }

  mpinfo("firmware size: %d\n", size);

  /* Secured firmware can't access from NuttX */

  strncpy(task->filename, filename, sizeof(task->filename) - 1);
  task->filename[sizeof(task->filename) - 1] = '\0';
  task->loadsize = (size + 0xffff) & ~0xffff;
  task_set_secure(task);

  sem_init(&task->wait, 0, 0);

  return OK;
}

int mptask_exec_secure(mptask_t *task)
{
  uintptr_t addr;

  addr = (uintptr_t)tile_alignalloc(task->loadsize, LOG2BASETILE);
  if (!addr)
    {
      mptask_cpu_free(task);
      return -ENOMEM;
    }
  task->loadaddr = addr;

  /* Map physical address for allocated CPU address map to based on zero */

  mptask_map(task);

  /* Load encrypted firmware by system CPU */

  mptask_loadfw(task);

  /* Suppress hot sleep */

  up_pm_acquire_wakelock(&g_mptask_wlock);

  cxd56_cpu_reset(task->cpuid);
  cxd56_cpu_clock_enable(task->cpuid);

  return OK;
}
