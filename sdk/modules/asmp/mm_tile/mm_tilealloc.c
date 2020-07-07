/****************************************************************************
 * modules/asmp/mm_tile/mm_tilealloc.c
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tile_common_alloc
 *
 * Description:
 *   Allocate memory from the tile heap.
 *
 * Input Parameters:
 *   priv - The tile heap state structure.
 *   size - The size of the memory region to allocate.
 *
 * Returned Value:
 *   On success, a non-NULL pointer to the allocated memory is returned.
 *
 ****************************************************************************/

static FAR void *tile_common_alloc(FAR struct tile_s *priv, size_t size,
                                   int log2align)
{
  uintptr_t    addr;
  uint32_t     mask;
  unsigned int idx;
  unsigned int ntiles;
  unsigned int step;

  if (!priv)
    {
      /* If the tile heap structure is none, memory cannot be allocated */

      return NULL;
    }

  if (size == 0)
    {
      return NULL;
    }

  tile_enter_critical(priv);

  if (log2align == 0)
    {
      step = 1;
    }
  else
    {
      step = log2align - priv->log2tile + 1;
    }

  ntiles = ALIGNUP(size, priv->log2tile) >> priv->log2tile;
  mask = 0xffffffff >> (32 - ntiles);

  if (ntiles > priv->ntiles)
    {
      goto alloc_error;
    }

  tinfo("size = %u\n", size);
  tinfo("number of tiles = %d\n", ntiles);
  tinfo("mask = %08x\n", mask);

  addr = priv->heapstart;

  for (idx = 0; idx < (priv->ntiles - ntiles + 1); idx += step)
    {
      if ((priv->at & mask) == 0)
        {
          /* Found enough area to be assigned for requested size.
           * Mark bits and return assigned memory address.
           */

          priv->at |= mask;
          tile_leave_critical(priv);
          return (FAR void *)addr;
        }

      /* Step forward to next assign candidate address */

      mask <<= step;
      addr += 1 << (priv->log2tile + step - 1);
    }

  /* Memory couldn't assigned */

alloc_error:
  tile_leave_critical(priv);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tile_alloc
 *
 * Description:
 *   Allocate memory from the tile heap.
 *
 * Input Parameters:
 *   size   - The size of the memory region to allocate.
 *
 * Returned Value:
 *   On success, either a non-NULL pointer to the allocated memory is returned.
 *
 ****************************************************************************/

FAR void *tile_alloc(size_t size)
{
  FAR struct tile_s *priv = g_tileinfo;
  void              *addr;
  size_t             tsize;

  addr = tile_common_alloc(priv, size, 0);
  if (addr)
    {
      /* Tile power on if allocated successfully. */

      tsize = ALIGNUP(size, priv->log2tile);
      up_pmramctrl(PMCMD_RAM_ON, (uintptr_t)addr, tsize);
    }

  return addr;
}

/****************************************************************************
 * Name: tile_alignalloc
 *
 * Description:
 *   Allocate aligned memory from the tile heap.
 *
 *   NOTE: The current implementation also restricts the maximum allocation
 *   size to 32 tiles.  That restriction could be eliminated with some
 *   additional coding effort.
 *
 * Input Parameters:
 *   size      - The size of the memory region to allocate.
 *   log2align - Log base 2 of the alignment
 *
 * Returned Value:
 *   On success, either a non-NULL pointer to the allocated memory is returned.
 *
 ****************************************************************************/

FAR void *tile_alignalloc(size_t size, uint32_t log2align)
{
  FAR struct tile_s *priv = g_tileinfo;
  void              *addr;
  size_t             tsize;

  addr = tile_common_alloc(priv, size, log2align);
  if (addr)
    {
      /* Tile power on if allocated successfully. */

      tsize = ALIGNUP(size, priv->log2tile);
      up_pmramctrl(PMCMD_RAM_ON, (uintptr_t)addr, tsize);
    }

  return addr;
}

#endif /* CONFIG_MM_TILE */
