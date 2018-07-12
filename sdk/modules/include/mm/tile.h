/****************************************************************************
 * modules/include/mm/tile.h
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

#ifndef __INCLUDE_NUTTX_MM_MM_TILE_H
#define __INCLUDE_NUTTX_MM_MM_TILE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <sys/types.h>
#include <stdint.h>

#ifdef CONFIG_MM_TILE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_TILE - Enable tile allocator support
 * CONFIG_DEBUG_TILE - Just like CONFIG_DEBUG_MM, but only generates output
 *   from the tile allocation logic.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef CONFIG_TILE_SINGLE
typedef FAR void *TILE_HANDLE;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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
 *   The heap is created by calling tile_initialize.  Here the tile size
 *   is set to 64 bytes and the alignment to 16 bytes:
 *
 *     TILE_HANDLE handle = tile_initialize(g_dmaheap, DMAHEAP_SIZE, 6, 4);
 *
 *   Then the TILE_HANDLE can be used to allocate memory (There is no
 *   TILE_HANDLE if CONFIG_TILE_SINGLE=y):
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
                    uint8_t log2align);

/****************************************************************************
 * Name: tile_release
 *
 * Description:
 *   Uninitialize a tile memory allocator and release resources held by the
 *   allocator.
 *
 * Input Parameters:
 *   handle - The handle previously returned by tile_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tile_release(void);

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

void tile_reserve(uintptr_t start, size_t size);

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
 *   On success, either a non-NULL pointer to the allocated memory (if
 *   CONFIG_TILE_SINGLE) or zero  (if !CONFIG_TILE_SINGLE) is returned.
 *
 ****************************************************************************/

FAR void *tile_alloc(size_t size);

/****************************************************************************
 * Name: tile_free
 *
 * Description:
 *   Return memory to the tile heap.
 *
 * Input Parameters:
 *   handle - The handle previously returned by tile_initialize
 *   memory - A pointer to memory previously allocated by tile_alloc.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tile_free(FAR void *memory, size_t size);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MM_TILE */
#endif /* __INCLUDE_NUTTX_MM_MM_TILE_H */
