/****************************************************************************
 * modules/asmp/mm_tile/mm_tileinit.c
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
 *   log2tile  - Log base 2 of the size of one tile.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3-> 8 bytes, etc.
 *   log2align - Log base 2 of required alignment.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3-> 8 bytes, etc.  Note that
 *               log2tile must be greater than or equal to log2align
 *               so that all contiguous tiles in memory will meet
 *               the minimum alignment requirement. A value of zero
 *               would mean that no alignment is required.
 *
 * Returned Value:
 *   On success, a non-NULL info structure is returned that may be used with
 *   other tile allocator interfaces.
 *
 ****************************************************************************/

static inline FAR struct tile_s *
tile_common_initialize(FAR void *heapstart, size_t heapsize, uint8_t log2tile,
                       uint8_t log2align)
{
  FAR struct tile_s *priv;
  uintptr_t          heapend;
  uintptr_t          alignedstart;
  unsigned int       mask;
  unsigned int       alignedsize;
  unsigned int       ntiles;

  /* Check parameters if debug is on.  Note the size of a tile is
   * limited to 2**31 bytes and that the size of the tile must be greater
   * than or equal to the alignment size.
   */

  DEBUGASSERT(heapstart && heapsize > 0 &&
              log2tile > 0 && log2tile < 32 &&
              log2tile >= log2align);

  /* Get the aligned start of the heap */

  mask         = (1 << log2align) - 1;
  alignedstart = ((uintptr_t)heapstart + mask) & ~mask;

  /* Determine the number of tiles */

  mask         = (1 << log2tile) - 1;
  heapend      = (uintptr_t)heapstart + heapsize;
  alignedsize  = (heapend - alignedstart) & ~mask;
  ntiles    = alignedsize >> log2tile;

  /* Allocate the information structure with a tile table of the
   * correct size.
   */

  priv = (FAR struct tile_s *)kmm_zalloc(SIZEOF_TILE_S(ntiles));
  if (priv)
    {
      /* Initialize non-zero elements of the tiles heap info structure */

      priv->log2tile  = log2tile;
      priv->ntiles = ntiles;
      priv->heapstart = alignedstart;

      /* Initialize mutual exclusion support */

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
 *   the alignment size (log2align; allocations will be in units of the
 *   tile size (log2tile). Larger tiles will give better performance
 *   and less overhead but more losses of memory due to quantization waste.
 *   Additional memory waste can occur from alignment; log2align should be
 *   set to 0 unless you are using the tile allocator to manage DMA
 *   or page-aligned memory and your hardware has specific memory alignment
 *   requirements.
 *
 *   General Usage Summary.  This is an example using the GCC section
 *   attribute to position a DMA heap in memory (logic in the linker script
 *   would assign the section .dmaheap to the DMA memory.
 *
 *     FAR uint32_t g_dmaheap[DMAHEAP_SIZE] __attribute__((section(.dmaheap)));
 *
 *   The heap is created by calling tile_initialize().  Here the tile size
 *   is set to 64 bytes (2**6) and the alignment to 16 bytes (2**4):
 *
 *     TILE_HANDLE handle = tile_initialize(g_dmaheap, DMAHEAP_SIZE, 6, 4);
 *
 *   Then the TILE_HANDLE can be used to allocate memory:
 *
 *     FAR uint8_t *dma_memory = (FAR uint8_t *)tile_alloc(handle, 47);
 *
 *   The actual memory allocates will be 64 byte (wasting 17 bytes) and
 *   will be aligned at least to (1 << log2align).
 *
 *   NOTE: The current implementation also restricts the maximum allocation
 *   size to 32 tiles.  That restriction could be eliminated with some
 *   additional coding effort.
 *
 * Input Parameters:
 *   heapstart - Start of the tile allocation heap
 *   heapsize  - Size of heap in bytes
 *   log2tile  - Log base 2 of the size of one tile.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3->8 bytes, etc.
 *   log2align - Log base 2 of required alignment.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3->8 bytes, etc.  Note that
 *               log2tile must be greater than or equal to log2align
 *               so that all contiguous tiles in memory will meet
 *               the minimum alignment requirement. A value of zero
 *               would mean that no alignment is required.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned that may be used with other
 *   tile allocator interfaces.
 *
 ****************************************************************************/

int tile_initialize(FAR void *heapstart, size_t heapsize, uint8_t log2tile,
                    uint8_t log2align)
{
  g_tileinfo = tile_common_initialize(heapstart, heapsize, log2tile,
                                      log2align);
  if (!g_tileinfo)
    {
      return -ENOMEM;
    }

  /* Power off all of tile allocator management area RAM. */

  up_pmramctrl(PMCMD_RAM_OFF, (uintptr_t)heapstart, heapsize);

  return OK;
}

#endif /* CONFIG_MM_TILE */
