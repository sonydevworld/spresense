/****************************************************************************
 * modules/asmp/mm_tile/mm_tileinit.c
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
#include <errno.h>

#include <nuttx/kmalloc.h>

#include <mm/tile.h>

#include <arch/chip/pm.h>

#include "mm_tile/mm_tile.h"

#ifdef CONFIG_MM_TILE

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* State of the single TILE allocator */

FAR struct tile_s *g_tileinfo;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tile_common_initialize
 *
 * Description:
 *   Perform common TILE initialization.
 *
 * Input Parameters:
 *   heapstart - Start of the tile allocation heap
 *   heapsize  - Size of heap in bytes
 *   log2tile  - Log base 2 of the size of one tile.  16 -> 64KB, 17 -> 128KB.
 *               Currently, only 16 and 17 are supported.
 *
 * Returned Value:
 *   On success, a non-NULL info structure is returned that may be used with
 *   other tile allocator interfaces.
 *
 ****************************************************************************/

static inline FAR struct tile_s *
tile_common_initialize(FAR void *heapstart, size_t heapsize, uint8_t log2tile)
{
  FAR struct tile_s *priv;

  /* Check parameters if debug is on.  Note the size of a tile is
   * limited to 2**31 bytes and that the size of the tile must be greater
   * than or equal to the alignment size.
   */

  DEBUGASSERT(heapstart && heapsize >= 0 &&
              log2tile > 0 && log2tile < 32);

  if (log2tile != 16 && log2tile != 17)
    {
      terr("Tile allocator supported block size is 64KB or 128KB .\n");
      return NULL;
    }

  if (heapsize == 0)
    {
      terr("Tile heap area is empty.\n");
      return NULL;
    }

  /* Allocate exact size of the structure, tile allocator supports less than
   * or equal to 32 tiles for now.
   */

  priv = kmm_zalloc(sizeof(struct tile_s));
  if (priv)
    {
      priv->heapstart = (uintptr_t)heapstart;
      priv->log2tile = log2tile;
      priv->ntiles = ALIGNUP(heapsize, log2tile) / (1 << log2tile);
      sem_init(&priv->exclsem, 0, 1);
    }

  return priv;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tile_initialize
 *
 * Description:
 *   Set up one tile allocator instance.  Allocations will be aligned to
 *   the tile block (128KiB). Larger tiles will give better performance
 *   and less overhead but more losses of memory due to quantization waste.
 *   Additional memory waste can occur from alignment.
 *
 *   The actual memory allocates will be 64 byte (wasting 17 bytes) and
 *   will be aligned at least to (1 << log2align).
 *
 *   NOTE: The current implementation also restricts the maximum allocation
 *   size to 32 tiles.
 *
 * Input Parameters:
 *   heapstart - Start of the tile allocation heap
 *   heapsize  - Size of heap in bytes
 *   log2tile  - Log base 2 of the size of one tile.  16 -> 64KB, 17 -> 128KB.
 *               Currently, only 16 and 17 are supported.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned that may be used with other
 *   tile allocator interfaces.
 *
 ****************************************************************************/

int tile_initialize(FAR void *heapstart, size_t heapsize, uint8_t log2tile)
{
  g_tileinfo = tile_common_initialize(heapstart, heapsize, log2tile);
  if (!g_tileinfo)
    {
      return -ENOMEM;
    }

  /* Power off all of tile allocator management area RAM. */

  up_pmramctrl(PMCMD_RAM_OFF, (uintptr_t)heapstart, heapsize);

  return OK;
}

#endif /* CONFIG_MM_TILE */
