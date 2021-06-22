/****************************************************************************
 * modules/asmp/supervisor/mptask_destroy.c
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

#include "arm_arch.h"
#include "cxd56_clock.h"
#include "cxd56_sysctl.h"

#include "mptask.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mptask_unloadfw(mptask_t *task)
{
  sysctl_unloadfw_t arg;
  int ret;

  arg.cpuid = mptask_getcpuid(task);
  ret = cxd56_sysctlcmd(SYSCTL_UNLOADFW, (uint32_t)(uintptr_t)&arg);
  if (ret)
    {
      mperr("unload failed. %d\n", ret);
    }
}

static void mptask_unloadfwgp(mptask_t *task)
{
  sysctl_unloadfwgp_t arg;
  int ret;

  arg.groupid = task->groupid;
  ret = cxd56_sysctlcmd(SYSCTL_UNLOADFWGP, (uint32_t)(uintptr_t)&arg);
  if (ret)
    {
      mperr("unload group failed. %d\n", ret);
    }
}

static void mptask_clock_disable(mptask_t *task)
{
  int cpu;

  for (cpu = ACPU; cpu < NMPCPUS; cpu++)
    {
      if (CPU_ISSET(cpu, &task->cpuids))
        {
          cxd56_cpu_clock_disable(cpu);
        }
    }
}

static void mptask_unmap_unified(mptask_t *task)
{
  unified_binary_t *ub = &task->ubin;
  int cpu;
  int i;

  for (cpu = ACPU, i = 0; cpu < NMPCPUS; cpu++)
    {
      if (CPU_ISSET(cpu, &task->cpuids))
        {
          mptask_unmap(cpu, ub->size[i]);
          i++;
        }
    }
}

static void mptask_unmap_all(mptask_t *task)
{
  int cpu;

  for (cpu = ACPU; cpu < NMPCPUS; cpu++)
    {
      if (CPU_ISSET(cpu, &task->cpuids))
        {
          mptask_unmap(cpu, task->loadsize);
        }
    }
}

static void mptask_unmap_task(mptask_t *task)
{
  if (task_is_unified(task))
    {
      mptask_unmap_unified(task);
    }
  else
    {
      mptask_unmap_all(task);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mptask_destroy(mptask_t *task, bool force, int *exit_status)
{
  int i;

  /* If task is secure binary, then it can't wait for exit signal. */

  if (task_is_secure(task))
    {
      if (exit_status)
        {
          *exit_status = 0;
        }
      force = true;
    }

  if (!force)
    {
      mptask_join(task, exit_status);
    }

  mptask_clock_disable(task);

  /* Permit hot sleep */

  up_pm_release_wakelock(&g_mptask_wlock);

  /* If task is secure binary, then unload first */

  if (task_is_secure(task))
    {
      if (task->groupid > 0)
        {
          mptask_unloadfwgp(task);
        }
      else
        {
          mptask_unloadfw(task);
        }
    }

  mptask_unmap_task(task);

  mptask_cpu_free(task);

  if (task->loadaddr)
    {
      tile_free((FAR void *)task->loadaddr, task->loadsize);
    }
  if (task_is_cloned(task))
    {
      binary_info_t *bi = task->bin;
      int ncpus;

      ncpus = mptask_cpu_count(&task->cpuids);
      for (i = 0; i < ncpus; i++, bi++)
        {
          if (bi->loadaddr)
            {
              tile_free((FAR void *)(uintptr_t)bi->loadaddr, task->loadsize);
            }
        }
    }

  task_set_exit(task);

  return OK;
}
