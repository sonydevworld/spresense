/****************************************************************************
 * modules/asmp/mm_tile/mm_tilereserve.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include "mm_tile/mm_tile.h"

#ifdef CONFIG_MM_TILE

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tile_common_reserve
 *
 * Description:
 *   Reserve memory in the tile heap.  This will reserve the tiles
 *   that contain the start and end addresses plus all of the tiles
 *   in between.  This should be done early in the initialization sequence
 *   before any other allocations are made.
 *
 *   Reserved memory can never be allocated (it can be freed however which
 *   essentially unreserves the memory).
 *
 * Input Parameters:
 *   priv  - The tile heap state structure.
 *   start - The address of the beginning of the region to be reserved.
 *   size  - The size of the region to be reserved
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void tile_common_reserve(FAR struct tile_s *priv,
                                       uintptr_t start, size_t size)
{
  if (size > 0)
    {
      uintptr_t mask = (1 << priv->log2tile) - 1;
      uintptr_t end  = start + size - 1;
      unsigned int ntiles;

      /* Get the aligned (down) start address and the aligned (up) end
       * address
       */

      start &= ~mask;
      end = (end + mask) & ~mask;

      /* Calculate the new size in tiles */

      ntiles = ((end - start) >> priv->log2tile) + 1;

      /* And reserve the tiles */

      tile_mark_allocated(priv, start, ntiles);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tile_reserve
 *
 * Description:
 *   Reserve memory in the tile heap.  This will reserve the tiles
 *   that contain the start and end addresses plus all of the tiles
 *   in between.  This should be done early in the initialization sequence
 *   before any other allocations are made.
 *
 *   Reserved memory can never be allocated (it can be freed however which
 *   essentially unreserves the memory).
 *
 * Input Parameters:
 *   handle - The handle previously returned by tile_initialize
 *   start  - The address of the beginning of the region to be reserved.
 *   size   - The size of the region to be reserved
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tile_reserve(uintptr_t start, size_t size)
{
  return tile_common_reserve(g_tileinfo, start, size);
}

#endif /* CONFIG_MM_TILE */
