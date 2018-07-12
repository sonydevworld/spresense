/****************************************************************************
 * modules/asmp/mm_tile/mm_tilefree.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

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

static inline void tile_common_free(FAR struct tile_s *priv,
                                    FAR void *memory, size_t size)
{
  unsigned int tileno;
  unsigned int atidx;
  unsigned int atbit;
  unsigned int tilemask;
  unsigned int ntiles;
  unsigned int avail;
  uint32_t     atmask;

  DEBUGASSERT(priv && memory && size <= 32 * (1 << priv->log2tile));

  /* Get exclusive access to the AT */

  tile_enter_critical(priv);

  /* Determine the tile number of the first tile in the allocation */

  tileno = ((uintptr_t)memory - priv->heapstart) >> priv->log2tile;

  /* Determine the AT table index and bit number associated with the
   * allocation.
   */

  atidx = tileno >> 5;
  atbit = tileno & 31;

  /* Determine the number of tiles in the allocation */

  tilemask =  (1 << priv->log2tile) - 1;
  ntiles = (size + tilemask) >> priv->log2tile;

  /* Clear bits in the AT entry or entries */

  avail = 32 - atbit;
  if (ntiles > avail)
    {
      /* Clear bits in the first AT entry */

      atmask = (0xffffffff << atbit);
      DEBUGASSERT((priv->at[atidx] & atmask) == atmask);

      priv->at[atidx] &= ~atmask;
      ntiles -= avail;

      /* Clear bits in the second AT entry */

      atmask = 0xffffffff >> (32 - ntiles);
      DEBUGASSERT((priv->at[atidx+1] & atmask) == atmask);

      priv->at[atidx+1] &= ~atmask;
    }

  /* Handle the case where where all of the tiles came from one entry */

  else
    {
      /* Clear bits in a single AT entry */

      atmask   = 0xffffffff >> (32 - ntiles);
      atmask <<= atbit;
      DEBUGASSERT((priv->at[atidx] & atmask) == atmask);

      priv->at[atidx] &= ~atmask;
    }

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
  size_t tsize;

  tile_common_free(g_tileinfo, memory, size);

  /* Tile power off. */

  tsize = size + ((1 << g_tileinfo->log2tile) - 1);
  tsize &= ~((1 << g_tileinfo->log2tile) - 1);

  up_pmramctrl(PMCMD_RAM_OFF, (uintptr_t)memory, tsize);
}

#endif /* CONFIG_MM_TILE */
