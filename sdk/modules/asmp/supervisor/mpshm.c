/****************************************************************************
 * modules/asmp/supervisor/mpshm.c
 *
 *   Copyright 2018,2021 Sony Semiconductor Solutions Corporation
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
#include <assert.h>
#include <debug.h>

#include <mm/tile.h>
#include <asmp/types.h>
#include <asmp/mpshm.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <semaphore.h>

#include <arch/chip/chip.h>
#include <arch/chip/pm.h>
#include <mm/tile.h>

#include "cxd56_sysctl.h"
#include "arm_arch.h"
#include "chip.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* __stack is the end of RAM address, it would be set by linker. */

extern char __stack[];

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_ID  (CXD56_CPU_BASE + 0x40)

#define MPSHM_BLOCK_SIZE_SHIFT 16
#define MPSHM_BLOCK_SIZE       (1 << MPSHM_BLOCK_SIZE_SHIFT)
#define MPSHM_BLOCK_TILE_SHIFT 17
#define MPSHM_BLOCK_TILE       (1 << MPSHM_BLOCK_TILE_SHIFT)
#define ALIGNUP(v, a)          (((v) + ((a)-1)) & ~((a)-1))

#define MM_TILE_BASE ((uint32_t)&__stack)
#define MM_TILE_SIZE \
  (CONFIG_RAM_START + CONFIG_RAM_SIZE - CXD56_PHYSADDR(MM_TILE_BASE))

#ifdef CONFIG_ASMP_SMALL_BLOCK
#  define MPSHM_TILE_ALIGN    MPSHM_BLOCK_SIZE_SHIFT
#  define BLOCKALIGNUP(v)  ALIGNUP(v, MPSHM_BLOCK_SIZE)
#else
#  define MPSHM_TILE_ALIGN    MPSHM_BLOCK_TILE_SHIFT
#  define BLOCKALIGNUP(v)  ALIGNUP(v, MPSHM_BLOCK_TILE)
#endif

/* Address converter can be handled up to 1MB */

#define ADR_CONV_VSIZE         0x100000

#ifdef CONFIG_ASMP_DEBUG_ERROR
#  define mperr(fmt, ...)   _err(fmt, ## __VA_ARGS__)
#else
#  define mperr(fmt, ...)
#endif
#ifdef CONFIG_ASMP_DEBUG_WARN
#  define mpwarn(fmt, ...)  _warn(fmt, ## __VA_ARGS__)
#else
#  define mpwarn(fmt, ...)
#endif
#ifdef CONFIG_ASMP_DEBUG_INFO
#  define mpinfo(fmt, ...)  _info(fmt, ## __VA_ARGS__)
#else
#  define mpinfo(fmt, ...)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int mpshm_semtake(sem_t *id)
{
  while (sem_wait(id) != 0)
    {
      ASSERT(errno == EINTR);
    }
  return OK;
}

static inline void mpshm_semgive(sem_t *id)
{
  sem_post(id);
}

static inline uint32_t mpshm_getactable(void)
{
  uint32_t cpuid = getreg32(CPU_ID) - 2;

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

  /* Setup map arguments
   * This API map memory range for this (NuttX) CPU. So cpuid field is
   * constant 2.
   */

  arg.cpuid = 2;
  arg.virt = va;
  arg.phys = paddr;
  arg.size = size;

  ret = cxd56_sysctlcmd(SYSCTL_MAP, (uint32_t)(uintptr_t)&arg);
  if (ret)
    {
      return NULL;
    }

  return (void *)((uintptr_t)tag << 16);
}

static void _mpshm_unmap(int8_t tag, size_t size)
{
  sysctl_unmap_t arg;
  uint32_t va = (uint32_t)tag << 16;
  int ret;

  /* Setup map arguments
   * This API map memory range for this (NuttX) CPU. So cpuid field is
   * constant 2.
   */

  arg.cpuid = 2;
  arg.virt = va;
  arg.size = size;

  ret = cxd56_sysctlcmd(SYSCTL_UNMAP, (uint32_t)(uintptr_t)&arg);
  if (ret)
    {
      mperr("UNMAP failed.\n", ret);
    }
  (void) ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Initialize MP shared memory
 */

int mpshm_init(mpshm_t *shm, key_t key, size_t size)
{
  if (!shm && !key && !size)
    {
      return -EINVAL;
    }

  if (MM_TILE_SIZE <= 0)
    {
      return -ENOMEM;
    }

  memset(shm, 0, sizeof(mpshm_t));
  mpobj_init(shm, SHM, key);

  shm->paddr = (uintptr_t)tile_alloc(size);
  if (!shm->paddr)
    {
      mperr("Allocate tile memory failure.\n");
      return -ENOMEM;
    }

  /* Tile allocator least size is 64KB or 128KB, so I adjust specified size */

  shm->size = BLOCKALIGNUP(size);
  mpinfo("Allocate memory %08x (%x)\n", shm->paddr, shm->size);

  /* Initialize semaphore */

  sem_init(&shm->exc, 0, 1);

  return 0;
}

/*
 * Destroy MP shared memory
 */

int mpshm_destroy(mpshm_t *shm)
{
  if (!shm)
    {
      return -EINVAL;
    }

  if (shm->paddr && shm->size)
    {
      tile_free((FAR void *)shm->paddr, shm->size);
    }

  memset(shm, 0, sizeof(mpshm_t));
  return OK;
}

/*
 * Attach MP shared memory
 */

void *mpshm_attach(mpshm_t *shm, int shmflg)
{
  int tag;
  void *va;

  if (!shm)
    {
      return NULL;
    }

  mpshm_semtake(&shm->exc);

  tag = mpshm_findfreetag(shm->size);
  if (tag < 0)
    {
      mpshm_semgive(&shm->exc);
      mperr("Address convertion table is full.\n");
      return NULL;
    }

  mpinfo("Find from %d.\n", tag);
  va = mpshm_map(tag, shm->paddr, shm->size);
  if (va)
    {
      shm->tag = tag;
    }

  mpshm_semgive(&shm->exc);

  return va;
}

/*
 * Detach MP shared memory
 */

int mpshm_detach(mpshm_t *shm)
{
  if (!shm)
    {
      return -EINVAL;
    }

  mpshm_semtake(&shm->exc);
  _mpshm_unmap(shm->tag, shm->size);
  mpshm_semgive(&shm->exc);

  return OK;
}

/*
 * Remap MP shared memory to specified virtual address
 */

int mpshm_remap(mpshm_t *shm, void *vaddr)
{
  uint32_t tags, mask;
  int tag;
  uintptr_t va = (uintptr_t)vaddr;
  int ret = OK;

  if (!shm)
    {
      return -EINVAL;
    }

  /* Check virtual address is in address converter supported area */

  if (va >= 0x00100000 || (va + shm->size - 1) >= 0x00100000)
    {
      return -EINVAL;
    }

  mpshm_semtake(&shm->exc);

  /* Check request virtual address is in use */

  tags = mpshm_gettag();
  tag = va >> 16;
  mask = (1 << (shm->size / MPSHM_BLOCK_SIZE)) - 1;
  mask <<= tag;

  mpinfo("tag %d, tags %08x, mask %08x.\n", tag, tags, mask);

  if ((tags & mask) != mask)
    {
      mperr("Address convertion table is full.\n");
      ret = -ENOENT;
      goto leave;
    }

  /* OK, I'm going to map into the virtual space */

  vaddr = mpshm_map(tag, shm->paddr, shm->size);
  DEBUGASSERT(vaddr);
  shm->tag = tag;

 leave:
  mpshm_semgive(&shm->exc);

  return ret;
}

/*
 * MP shared memory control
 */

int mpshm_control(mpshm_t *shm, int cmd, void *buf)
{
  int ret = OK;

  if (!shm)
    {
      return -EINVAL;
    }

  /* Get exclusive access */

  mpshm_semtake(&shm->exc);

  switch (cmd)
    {
      /* Set shared memory power on. */

      case MPC_POWERON:
        {
          mpinfo("0x%08x-0x%08x -> on\n", shm->paddr, shm->paddr + shm->size - 1);
          up_pmramctrl(PMCMD_RAM_ON, shm->paddr, shm->size);
        }
        break;

      /* Set shared memory power off. User not need this command except
       * any special situations.
       */

      case MPC_POWEROFF:
        {
          mpinfo("0x%08x-0x%08x -> off\n", shm->paddr, shm->paddr + shm->size - 1);
          up_pmramctrl(PMCMD_RAM_OFF, shm->paddr, shm->size);
        }
        break;

      /* Set shared memory to retention state.
       * Retention state saves RAM power but data will be remained.
       */

      case MPC_RETENTION:
        {
          mpinfo("0x%08x-0x%08x -> retention\n", shm->paddr,
                 shm->paddr + shm->size - 1);
          up_pmramctrl(PMCMD_RAM_RET, shm->paddr, shm->size);
        }
        break;


      default:
        {
          ret = -ENOTTY;
        }
    }

  mpshm_semgive(&shm->exc);

  return ret;
}

/*
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

/*
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

void mpshm_initialize(void)
{
  int ret;

  /* MM_TILE_BASE must be aligned at 128Kbyte */

  ASSERT((MM_TILE_BASE & 0x1ffff) == 0);

  ret = tile_initialize((void *)MM_TILE_BASE, MM_TILE_SIZE, MPSHM_TILE_ALIGN);
  if (ret < 0)
    {
      mperr("Tile memory initialization failure.\n");
    }

  /* Clear virtual address mapping except first 64KB block.
   * Because first virtual address is necessary for wake up from hot sleep.
   */

  _mpshm_unmap(1, ADR_CONV_VSIZE - 65536);
}
