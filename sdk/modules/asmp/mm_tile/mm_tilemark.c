/****************************************************************************
 * modules/asmp/mm_tile/mm_tilemark.c
 *
 *   Copyright (C) 2012, 2014 Gregory Nutt. All rights reserved.
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tile_mark_allocated
 *
 * Description:
 *   Mark a range of tiles as allocated.
 *
 * Input Parameters:
 *   priv  - The tile heap state structure.
 *   alloc - The address of the allocation.
 *   ntiles - The number of tiles allocated
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tile_mark_allocated(FAR struct tile_s *priv, uintptr_t alloc,
                         unsigned int ntiles)
{
  unsigned int tileno;
  unsigned int atidx;
  unsigned int atbit;
  unsigned int avail;
  uint32_t     atmask;

  /* Determine the tile number of the allocation */

  tileno = (alloc - priv->heapstart) >> priv->log2tile;

  /* Determine the AT table index associated with the allocation */

  atidx = tileno >> 5;
  atbit = tileno & 31;

  /* Mark bits in the AT entry or entries */

  avail = 32 - atbit;
  if (ntiles > avail)
    {
      /* Mark bits in the first AT entry */

      atmask = 0xffffffff << atbit;
      DEBUGASSERT((priv->at[atidx] & atmask) == 0);

      priv->at[atidx] |= atmask;
      ntiles -= avail;

      /* Mark bits in the second AT entry */

      atmask = 0xffffffff >> (32 - ntiles);
      DEBUGASSERT((priv->at[atidx+1] & atmask) == 0);

      priv->at[atidx+1] |= atmask;
    }

  /* Handle the case where where all of the tiles come from one entry */

  else
    {
      /* Mark bits in a single AT entry */

      atmask   = 0xffffffff >> (32 - ntiles);
      atmask <<= atbit;
      DEBUGASSERT((priv->at[atidx] & atmask) == 0);

      priv->at[atidx] |= atmask;
      return;
    }
}

#endif /* CONFIG_MM_TILE */
