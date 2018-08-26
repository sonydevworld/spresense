/****************************************************************************
 * modules/include/mm/tile.h
 *
 *   Copyright (C) 2012, 2014 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_MM_TILE_H
#define __INCLUDE_MM_TILE_H

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
/* CONFIG_MM_TILE - Enable tile allocator support
 * CONFIG_DEBUG_TILE - Just like CONFIG_DEBUG_MM, but only generates output
 *   from the tile allocation logic.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 *   log2tile  - Log base 2 of the size of one tile.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3->8 bytes, etc.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned that may be used with other
 *   tile allocator interfaces.
 *
 ****************************************************************************/

int tile_initialize(FAR void *heapstart, size_t heapsize, uint8_t log2tile);

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
 * Name: tile_alloc
 *
 * Description:
 *   Allocate memory from the tile heap.
 *
 * Input Parameters:
 *   size   - The size of the memory region to allocate.
 *
 * Returned Value:
 *   On success, either a non-NULL pointer to the allocated memory or zero
 *   is returned.
 *
 ****************************************************************************/

FAR void *tile_alloc(size_t size);

/****************************************************************************
 * Name: tile_alignalloc
 *
 * Description:
 *   Allocate aligned memory from the tile heap.
 *
 * Input Parameters:
 *   size      - The size of the memory region to allocate.
 *   log2align - Log base 2 of the alignment
 *
 * Returned Value:
 *   On success, either a non-NULL pointer to the allocated memory or zero
 *   is returned.
 *
 ****************************************************************************/

FAR void *tile_alignalloc(size_t size, uint32_t log2align);

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
#endif /* __INCLUDE_MM_TILE_H */
