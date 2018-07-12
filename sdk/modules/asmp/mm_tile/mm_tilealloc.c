/****************************************************************************
 * modules/asmp/mm_tile/mm_tilealloc.c
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

static inline FAR void *tile_common_alloc(FAR struct tile_s *priv, size_t size)
{
  unsigned int ntiles;
  size_t       tmpmask;
  uintptr_t    alloc;
  uint32_t     curr;
  uint32_t     next;
  uint32_t     mask;
  int          tileidx;
  int          atidx;
  int          bitidx;
  int          shift;

  DEBUGASSERT(priv && size <= 32 * (1 << priv->log2tile));

  if (priv && size > 0)
    {
      /* Get exclusive access to the AT */

      tile_enter_critical(priv);

      /* How many contiguous tiles we we need to find? */

      tmpmask   = (1 << priv->log2tile) - 1;
      ntiles = (size + tmpmask) >> priv->log2tile;

      /* Then create mask for that number of tiles */

      DEBUGASSERT(ntiles <= 32);
      mask = 0xffffffff >> (32 - ntiles);

      /* Now search the tile allocation table for that number of contiguous */

      alloc = priv->heapstart;

      for (tileidx = 0; tileidx < priv->ntiles; tileidx += 32)
        {
          /* Get the AT index associated with the tile table entry */

          atidx = tileidx >> 5;
          curr = priv->at[atidx];

          /* Handle the case where there are no free tiles in the entry */

          if (curr == 0xffffffff)
            {
              alloc += (32 << priv->log2tile);
              continue;
            }

          /* Get the next entry from the AT to support a 64 bit shift */

          if (tileidx < priv->ntiles)
            {
              next = priv->at[atidx + 1];
            }

          /* Use all ones when are at the last entry in the AT (meaning
           * nothing can be allocated.
           */

          else
            {
              next = 0xffffffff;
            }

          /* Search through the allocations in the 'curr' AT entry
           * to see if we can satisfy the allocation starting in that
           * entry.
           *
           * This loop continues until either all of the bits have been
           * examined (bitidx >= 32), or until there are insufficient
           * tiles left to satisfy the allocation.
           */

          for (bitidx = 0;
               bitidx < 32 && (tileidx + bitidx + ntiles) <= priv->ntiles;
              )
            {
              /* Break out if there are no further free bits in 'curr'.
               * All of the zero bits might have gotten shifted out.
               */

              if (curr == 0xffffffff)
                {
                  break;
                }

              /* Check for the first zero bit in the lower or upper 16-bits.
               * From the test above, we know that at least one of the 32-
               * bits in 'curr' is zero.
               */

              else if ((curr & 0x0000ffff) == 0x0000ffff)
                {
                  /* Not in the lower 16 bits.  The first free bit must be
                   * in the upper 16 bits.
                   */

                  shift = 16;
                }

              /* We know that the first free bit is now within the lower 16
               * bits of 'curr'.  Is it in the upper or lower byte?
               */

              else if ((curr & 0x0000ff) == 0x000000ff)
                {
                  /* Not in the lower 8 bits.  The first free bit must be in
                   * the upper 8 bits.
                   */

                  shift = 8;
                }

              /* We know that the first free bit is now within the lower 4
               * bits of 'curr'.  Is it in the upper or lower nibble?
               */

              else if ((curr & 0x00000f) == 0x0000000f)
                {
                  /* Not in the lower 4 bits.  The first free bit must be in
                   * the upper 4 bits.
                   */

                  shift = 4;
                }

              /* We know that the first free bit is now within the lower 4 bits
               * of 'curr'.  Is it in the upper or lower pair?
               */

              else if ((curr & 0x000003) == 0x00000003)
                {
                  /* Not in the lower 2 bits.  The first free bit must be in
                   * the upper 2 bits.
                   */

                  shift = 2;
                }

              /* We know that the first free bit is now within the lower 4 bits
               * of 'curr'.  Check if we have the allocation at this bit position.
               */

              else if ((curr & mask) == 0)
                {
                  /* Yes.. mark these tiles allocated */

                  tile_mark_allocated(priv, alloc, ntiles);

                  /* And return the allocation address */

                  tile_leave_critical(priv);
                  return (FAR void *)alloc;
                }

              /* The free allocation does not start at this position */

              else
                {
                  shift = 1;
                }

              /* Set up for the next time through the loop.  Perform a 64
               * bit shift to move to the next tile position andi ncrement
               * to the next candidate allocation address.
               */

              alloc  += (shift << priv->log2tile);
              curr    = (curr >> shift) | (next << (32 - shift));
              next  >>= shift;
              bitidx += shift;
            }
        }

      tile_leave_critical(priv);
    }

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
 *   NOTE: The current implementation also restricts the maximum allocation
 *   size to 32 tiles.  That restriction could be eliminated with some
 *   additional coding effort.
 *
 * Input Parameters:
 *   handle - The handle previously returned by tile_initialize
 *   size   - The size of the memory region to allocate.
 *
 * Returned Value:
 *   On success, either a non-NULL pointer to the allocated memory is returned.
 *
 ****************************************************************************/

FAR void *tile_alloc(size_t size)
{
  void *addr;
  size_t tsize;

  addr = tile_common_alloc(g_tileinfo, size);
  if (addr)
    {
      /* Tile power on if allocated successfully. */

      tsize = size + ((1 << g_tileinfo->log2tile) - 1);
      tsize &= ~((1 << g_tileinfo->log2tile) - 1);

      up_pmramctrl(PMCMD_RAM_ON, (uintptr_t)addr, tsize);
    }

  return addr;
}

#endif /* CONFIG_MM_TILE */
