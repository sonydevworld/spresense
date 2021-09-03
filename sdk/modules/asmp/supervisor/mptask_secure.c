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

#include <stdlib.h>
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

static int mptask_getfwoffsets(mptask_t *task, const char *filename)
{
  sysctl_getpgoffsets_t arg;
  unified_binary_t *ub = &task->ubin;
  int i;
  int ret;

  memset(&arg, 0, sizeof(sysctl_getpgoffsets_t));
  strncpy(arg.filename, filename, sizeof(arg.filename) - 1);
  arg.filename[sizeof(arg.filename) - 1] = '\0';
  ret = cxd56_sysctlcmd(SYSCTL_GETPGOFFSETS, (uint32_t)(uintptr_t)&arg);
  if (ret < 0)
    {
      return ret;
    }

  /* Save offset list to extra data */

  ub->nr_offs = arg.nr_offsets;
  for (i = 0; i < arg.nr_offsets; i++)
    {
      ub->offset[i] = arg.offset[i];
      ub->size[i] = arg.size[i];
    }

  /* Set unified binary attribute if multiple offsets returned */

  if (ub->nr_offs > 1)
    {
      task_set_unified(task);
      mpinfo("Unified binary detected. (%d binaries)\n", ub->nr_offs);
    }

  return OK;
}

static int mptask_loadfwunify(mptask_t *task)
{
  sysctl_loadfwgp_t arg;
  int ret;

  /*
   * CPU ID list in mptask context is in local app domain IDs,
   * Change them to 2 bit shift for global CPU ID.
   */

  arg.cpuids = task->cpuids << 2;
  arg.addr[0] = task->loadaddr;
  arg.nr_addrs = 1; /* always */
  strncpy(arg.filename, task->filename, sizeof(arg.filename) - 1);
  arg.filename[sizeof(arg.filename) - 1] = '\0';

  ret = cxd56_sysctlcmd(SYSCTL_LOADFWUNIFY, (uint32_t)(uintptr_t)&arg);
  if (ret > 0)
    {
      task->groupid = ret;
      return 0;
    }

  mperr("Unified binary couldn't be loaded. %d\n", ret);
  return ret;
}

static int mptask_loadfwclone(mptask_t *task)
{
  sysctl_loadfwgp_t arg;
  binary_info_t *bi = task->bin;
  int ncpus;
  int i;
  int ret;

  ncpus = mptask_cpu_count(&task->cpuids);

  /*
   * CPU ID list in mptask context is in local app domain IDs,
   * Change them to 2 bit shift for global CPU ID.
   */

  arg.cpuids = task->cpuids << 2;
  strncpy(arg.filename, task->filename, sizeof(arg.filename) - 1);
  arg.filename[sizeof(arg.filename) - 1] = '\0';
  arg.nr_addrs = ncpus;

  for (i = 0; i < ncpus; i++, bi++)
    {
      arg.addr[i] = bi->loadaddr;
    }

  ret = cxd56_sysctlcmd(SYSCTL_LOADFWCLONE, (uint32_t)(uintptr_t)&arg);
  if (ret > 0)
    {
      task->groupid = ret;
      return 0;
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mptask_init_secure(mptask_t *task, const char *filename)
{
  sysctl_getfwsize_t arg;
  int i;
  int size;
  int ret;

  if (!task)
    {
      return -EINVAL;
    }
  if (strlen(filename) > 31)
    {
      return -EINVAL;
    }

  memset(task, 0, sizeof(mptask_t));
  CPU_ZERO(&task->cpuids);

  /* Set all of CPUs to permitted */

  for (i = ACPU; i < NMPCPUS; i++)
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

  ret = mptask_getfwoffsets(task, filename);
  if (ret < 0)
    {
      return ret;
    }

  /* Secured firmware can't access from NuttX */

  strncpy(task->filename, filename, sizeof(task->filename) - 1);
  task->filename[sizeof(task->filename) - 1] = '\0';
  task->loadsize = (size + 0xffff) & ~0xffff;
  task_set_secure(task);

  sem_init(&task->wait, 0, 0);

  return OK;
}

static int mptask_exec_unified(mptask_t *task, int ncpus)
{
  uintptr_t addr;
  unified_binary_t *ub = &task->ubin;
  int cpu;
  int i;
  int ret;
  int firstcpu;

  /* Unified binary must be assign the same numbers with offsets (binaries) */

  if (ncpus != ub->nr_offs)
    {
      return -EFAULT;
    }

  addr = (uintptr_t)tile_alignalloc(task->loadsize, LOG2BASETILE);
  if (!addr)
    {
      mptask_cpu_free(task);
      return -ENOMEM;
    }
  task->loadaddr = addr;

  /* Load encrypted firmware by system CPU */

  ret = mptask_loadfwunify(task);
  if (ret < 0)
    {
      return ret;
    }

  firstcpu = 0;

  for (cpu = ACPU, i = 0; cpu < NMPCPUS && i < ub->nr_offs; cpu++)
    {
      if (CPU_ISSET(cpu, &task->cpuids))
        {
          uint32_t offs = ((uint32_t)ub->offset[i]) << 16;
          uint32_t sz = ((uint32_t)ub->size[i]) << 16;

          /*
           * LOADFWUNIFY command loads whole of unified binary and
           * set address mapping to first find CPU.
           * This logic is shrink these extra size mapping to necessary
           * range to first find CPU.
           */

          if (firstcpu == 0)
            {
              firstcpu = cpu;
              mptask_mapshrink(cpu, sz);
            }
          mptask_map(cpu, addr + offs, sz);
          i++;
        }
    }

  /* Suppress hot sleep */

  up_pm_acquire_wakelock(&g_mptask_wlock);

  cxd56_cpulist_reset(task->cpuids);
  cxd56_cpulist_clock_enable(task->cpuids);

  return OK;
}

static int mptask_exec_clone(mptask_t *task, int ncpus)
{
  uintptr_t addr;
  int cpu;
  binary_info_t *bi;
  int ret;

  for (cpu = ACPU, bi = task->bin; cpu < NMPCPUS; cpu++)
    {
      if (CPU_ISSET(cpu, &task->cpuids))
        {
          addr = (uintptr_t)tile_alignalloc(task->loadsize, LOG2BASETILE);
          if (!addr)
            {
              mptask_cpu_free(task);
              return -ENOMEM;
            }

          bi->loadaddr = addr;
          bi++;
        }
    }
  task_set_clone(task);

  /* Load encrypted firmware by system CPU */

  ret = mptask_loadfwclone(task);
  if (ret < 0)
    {
      return ret;
    }

  /* Map loaded memories. This logic must be after loaded. */

  for (cpu = ACPU, bi = task->bin; cpu < NMPCPUS; cpu++)
    {
      if (CPU_ISSET(cpu, &task->cpuids))
        {
          mptask_map(cpu, bi->loadaddr, task->loadsize);
          bi++;
        }
    }

  /* Suppress hot sleep */

  up_pm_acquire_wakelock(&g_mptask_wlock);

  cxd56_cpulist_reset(task->cpuids);
  cxd56_cpulist_clock_enable(task->cpuids);

  return OK;
}

int mptask_exec_secure(mptask_t *task)
{
  uintptr_t addr;
  int ncpus;
  int cpu;

  if (!task_is_secure(task))
    {
      return -EINVAL;
    }

  /* Secure binary must be assign CPUs before exec. */

  if (task->cpuids == 0)
    {
      return -ENOENT;
    }

  ncpus = mptask_cpu_count(&task->cpuids);
  if (ncpus > 1)
    {
      if (task_is_unified(task))
        {
          return mptask_exec_unified(task, ncpus);
        }
      else
        {
          return mptask_exec_clone(task, ncpus);
        }
    }

  /* Single core execution sequence */

  cpu = mptask_getcpuid(task) - 2;

  addr = (uintptr_t)tile_alignalloc(task->loadsize, LOG2BASETILE);
  if (!addr)
    {
      mptask_cpu_free(task);
      return -ENOMEM;
    }
  task->loadaddr = addr;

  /* Map physical address for allocated CPU address map to based on zero */

  mptask_map(cpu, addr, task->loadsize);

  /* Load encrypted firmware by system CPU */

  mptask_loadfw(task);

  /* Suppress hot sleep */

  up_pm_acquire_wakelock(&g_mptask_wlock);

  cxd56_cpu_reset(cpu);
  cxd56_cpu_clock_enable(cpu);

  return OK;
}
