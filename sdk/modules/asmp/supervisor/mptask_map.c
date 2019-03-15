/****************************************************************************
 * modules/asmp/supervisor/mptask_map.c
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

#include <asmp/mptask.h>

#include "cxd56_sysctl.h"
#include "mptask.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int do_unmap(int cpuid, uint32_t va, uint32_t size)
{
  sysctl_unmap_t arg;

  arg.cpuid = cpuid + 2;
  arg.virt  = va;
  arg.size  = size;

  return cxd56_sysctlcmd(SYSCTL_UNMAP, (uint32_t)(uintptr_t)&arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: mptask_map
 *
 * Description:
 *   Map allocated physical address to virtual address for affinity CPU.
 *   This function not affected in NuttX CPU.
 *
 ****************************************************************************/

int mptask_map(int cpuid, uint32_t pa, uint32_t size)
{
  sysctl_map_t arg;
  int ret;

  ASSERT(cpuid >= 0);

  /* Setup map arguments
   * For MP task mapping region is always starts with zero.
   * This is an architecture restriction.
   * And size is must be tile size aligned.
   */

  size = alignup(size, 1 << 16);

  arg.cpuid = cpuid + 2;
  arg.virt  = 0;
  arg.phys  = pa;
  arg.size  = size;

  ret = cxd56_sysctlcmd(SYSCTL_MAP, (uint32_t)(uintptr_t)&arg);
  if (ret)
    {
      mperr("MAP failed. %d\n", ret);
      return ret;
    }

  mpinfo("CPU%d Load at %08x (size: %x)\n", arg.cpuid, arg.phys, arg.size);

  return ret;
}

/****************************************************************************
 * Name: mptask_unmap
 *
 * Description:
 *   Unmap virtual address for affinity CPU.
 *   This function not affected in NuttX CPU.
 *
 ****************************************************************************/

void mptask_unmap(int cpuid, uint32_t size)
{
  int ret;

  ASSERT(cpuid >= 0);

  /* Set unmap arguments */

  size = alignup(size, 1 << 16);

  ret = do_unmap(cpuid, 0, size);
  if (ret)
    {
      mperr("Unmap failed. %d\n", ret);
    }
  (void) ret;
}

/****************************************************************************
 * Name: mptask_mapclear
 *
 * Description:
 *   Clear all of virtual address mappings for affinity CPU.
 *   This function not affected in NuttX CPU.
 *
 ****************************************************************************/

void mptask_mapclear(int cpuid)
{
  int ret;

  ASSERT(cpuid >= 0);

  /*
   * Address converter converts up to 1MB (0x100000).
   * Clear all of 1MB conversion table.
   */

  ret = do_unmap(cpuid, 0, 0x100000);
  if (ret)
    {
      mperr("Clear failed. %d\n", ret);
    }
  (void) ret;
}

/****************************************************************************
 * Name: mptask_mapshrink
 *
 * Description:
 *   Shrink mapped size to @a size.
 *
 ****************************************************************************/

void mptask_mapshrink(int cpuid, uint32_t size)
{
  int ret;
  int va;

  ASSERT(cpuid >= 0);

  /* Clear after size to 1MB region */

  va = size;
  size = 0x100000 - size;

  ret = do_unmap(cpuid, va, size);
  if (ret)
    {
      mperr("Shrink failed. %d\n", ret);
    }
  (void) ret;
}
