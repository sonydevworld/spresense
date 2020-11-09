/****************************************************************************
 * modules/asmp/worker/mpshm.c
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
#include <arch/irq.h>

#include <asmp/types.h>
#include <asmp/mpshm.h>
#include <asmp/mptask.h>

#include <errno.h>

#include "arm_arch.h"
#include "chip.h"

#include "asmp.h"
#include "common.h"
#include "arch/sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPSHM_BLOCK_SIZE_SHIFT 16
#define MPSHM_BLOCK_SIZE       (1 << MPSHM_BLOCK_SIZE_SHIFT)
#define ALIGNUP(v, a)          (((v) + ((a)-1)) & ~((a)-1))
#define BLOCKSIZEALIGNUP2(v)   ALIGNUP(v, MPSHM_BLOCK_SIZE * 2)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t mpshm_getactable(void)
{
  cpuid_t cpuid = asmp_getlocalcpuid();

  return CXD56_ADR_CONV_BASE + (cpuid * 0x20) + 4;
}

static uint32_t mpshm_gettag(void)
{
  int i;
  uint32_t reg;
  uint32_t ret;

  reg = mpshm_getactable();
  ret = 0;

  for (i = 0; i < 16; i += 2, reg += 4)
    {
      uint32_t val;
      val = getreg32(reg);
      if ((val & 0xffff) < 0x100)
        {
          ret |= 1 << i;
        }
      if ((val >> 16) < 0x100)
        {
          ret |= 1 << (i + 1);
        }
    }

  return ret;
}

static int8_t mpshm_findfreetag(size_t size)
{
  uint32_t tags;
  uint32_t mask;
  int bsize = size / MPSHM_BLOCK_SIZE;
  int i;

  tags = mpshm_gettag();
  mask = (1 << bsize) - 1;

  /* Search continuous memory for specified size */

  for (i = 0; i < 16 - bsize; i++)
    {
      if ((tags & mask) == mask)
        {
          return i;
        }
      mask <<= 1;
    }

  return -1;
}

static void *mpshm_map(int8_t tag, uintptr_t paddr, size_t size)
{
  sysctl_map_t arg;
  uint32_t va = (uint32_t)tag << 16;
  int ret;

  arg.cpuid = asmp_getglobalcpuid();
  arg.virt  = va;
  arg.phys  = paddr;
  arg.size  = size;

  /* Convert sysctl argument address from virtual to physical.
   * Because worker running on virtual address, but system CPU refers
   * physical address only.
   */

  ret = sysctl(SYSCTL_MAP, mpshm_virt2phys(NULL, &arg));
  if (ret)
    {
      return NULL;
    }

  return (void *)((uintptr_t)va);
}

static void _mpshm_unmap(int8_t tag, size_t size)
{
  sysctl_unmap_t arg;
  uint32_t va = (uint32_t)tag << 16;

  arg.cpuid = asmp_getglobalcpuid();
  arg.virt  = va;
  arg.size  = size;

  /* Convert sysctl argument address from virtual to physical.
   * Because worker running on virtual address, but system CPU refers
   * physical address only.
   */

  (void) sysctl(SYSCTL_UNMAP, mpshm_virt2phys(NULL, &arg));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * Initialize MP shared memory
 */

int mpshm_init(mpshm_t *shm, key_t key, size_t size)
{
  mpbindobj_t *obj;

  if (!shm)
    {
      return -EINVAL;
    }

  wk_memset(shm, 0, sizeof(mpshm_t));

  obj = asmp_findmpbindobj(MPOBJTYPE_SHM, key);
  if (!obj)
    {
      return -ENOENT;
    }

  /* Tile allocator least size is 128KB, so I adjust specified size */

  mpobj_init(shm, SHM, key);
  shm->paddr = obj->value;
  shm->size = BLOCKSIZEALIGNUP2(size);

  return OK;
}

/**
 * Destroy MP shared memory
 */

int mpshm_destroy(mpshm_t *shm)
{
  if (!shm)
    {
      return -EINVAL;
    }

  wk_memset(shm, 0, sizeof(mpshm_t));

  return OK;
}

/**
 * Attach MP shared memory
 */

void *mpshm_attach(mpshm_t *shm, int shmflg)
{
  irqstate_t flags;
  int tag;
  void *va;

  if (!shm)
    {
      return NULL;
    }

  flags = up_irq_save();

  tag = mpshm_findfreetag(shm->size);
  if (tag < 0)
    {
      return NULL;
    }
  va = mpshm_map(tag, shm->paddr, shm->size);
  if (va)
    {
      shm->tag = tag;
    }

  up_irq_restore(flags);

  return va;
}

/**
 * Detach MP shared memory
 */

int mpshm_detach(mpshm_t *shm)
{
  irqstate_t flags;

  if (!shm)
    {
      return -EINVAL;
    }

  flags = up_irq_save();
  _mpshm_unmap(shm->tag, shm->size);
  up_irq_restore(flags);

  return OK;
}

/**
 * MP shared memory control
 */

int mpshm_control(mpshm_t *shm, int cmd, void *buf)
{
  return -ENOSYS;
}

/**
 * Convert virtual address to physical address
 */

uintptr_t mpshm_virt2phys(mpshm_t *shm, void *vaddr)
{
  uintptr_t va, pa;
  uint32_t reg;
  int8_t tag;

  va = (uintptr_t)vaddr >> 16;
  if (va & 0xfff0)
    {
      return 0;
    }
  tag = va & 0xf;

  reg = mpshm_getactable();
  pa = getreg32(reg + (4 * (tag / 2)));

  if (!(tag & 1))
    {
      pa <<= 16;
    }
  pa = (pa & 0x01ff0000u) | ((pa & 0x06000000) << 1);

  return pa | ((uintptr_t)vaddr & 0xffff);
}

/**
 * Convert physical address to virtial address
 */

void *mpshm_phys2virt(mpshm_t *shm, uintptr_t paddr)
{
  uintptr_t va, pa;
  uint32_t reg;
  int i;

  reg = mpshm_getactable();
  pa = paddr >> 16;
  pa = (pa & 0x1ff) | ((pa >> 1) & 0x600);
  va = 0;

  for (i = 0; i < 16; i += 2, reg += 4)
    {
      uint32_t val = getreg32(reg);

      if ((val & 0x0000ffffu) == pa)
        {
          va = i << 16;
          break;
        }

      if ((val >> 16) == pa)
        {
          va = (i + 1) << 16;
          break;
        }
    }

  return (void *)(va | (paddr & 0xffff));
}
