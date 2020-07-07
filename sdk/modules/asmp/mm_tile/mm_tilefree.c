/****************************************************************************
 * modules/asmp/mm_tile/mm_tilefree.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>
#include <sdk/debug.h>

#include <assert.h>

#include <mm/tile.h>

#include <arch/chip/pm.h>

#include "mm_tile/mm_tile.h"

#ifdef CONFIG_MM_TILE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: tile_common_free
 *
 * Description:
 *   Return memory to the tile heap.
 *
 * Input Parameters:
 *   handle - The handle previously returned by tile_initialize
 *   memory - A pointer to memory previoiusly allocated by tile_alloc.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void tile_common_free(FAR struct tile_s *priv, void *addr,
                                    size_t size)
{
  unsigned int idx;
  unsigned int ntiles;
  unsigned int mask;
  uintptr_t heapend;

  DEBUGASSERT(priv);

  tile_enter_critical(priv);

  if (addr == NULL || size == 0)
    {
      goto finish;
    }

  /* Check addr and size are in the heap */

  heapend = priv->heapstart + (priv->ntiles << priv->log2tile);
  if (heapend < ((uintptr_t)addr + size))
    {
      goto finish;
    }

  idx = ((uintptr_t)addr - priv->heapstart) >> priv->log2tile;
  ntiles = ALIGNUP(size, priv->log2tile) >> priv->log2tile;
  mask = (0xffffffff >> (32 - ntiles)) << idx;

  tinfo("free idx = %u, ntiles = %u, mask = %08x\n", idx, ntiles, mask);

  DEBUGASSERT((priv->at & mask) == mask);

  priv->at &= ~mask;

finish:
  tile_leave_critical(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tile_free
 *
 * Description:
 *   Return memory to the tile heap.
 *
 * Input Parameters:
 *   handle - The handle previously returned by tile_initialize
 *   memory - A pointer to memory previoiusly allocated by tile_alloc.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tile_free(FAR void *memory, size_t size)
{
  FAR struct tile_s *priv = g_tileinfo;
  size_t             tsize;

  if (!priv)
    {
      /* If the tile heap structure is none, do nothing */

      return;
    }

  tile_common_free(priv, memory, size);

  /* Power off free tiles */

  tsize = ALIGNUP(size, priv->log2tile);

  if (priv->log2tile == 17)
    {
      /* If block size is 128KB, just do power off */

      up_pmramctrl(PMCMD_RAM_OFF, (uintptr_t)memory, tsize);
    }
  else
    {
      uintptr_t    addr = (uintptr_t)memory;
      unsigned int idx;

      /* If 64KB block size, we need to check a same tile block is
       * still in used.
       */

      idx = ((addr - priv->heapstart) >> 17) * 2;
      if ((priv->at & (3 << idx)) == 0)
        {
          up_pmramctrl(PMCMD_RAM_OFF, (uintptr_t)memory, tsize);
        }
    }
}

#endif /* CONFIG_MM_TILE */
