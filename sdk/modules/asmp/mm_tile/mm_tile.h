/****************************************************************************
 * modules/asmp/mm_tile/mm_tile.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright 2018,2021 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_ASMP_MM_MM_TILE_H
#define __MODULES_ASMP_MM_MM_TILE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>

#include <stdint.h>
#include <semaphore.h>

#include <arch/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug */

#ifdef CONFIG_MM_TILE_DEBUG
#  define tinfo(fmt, ...) _info(fmt, ## __VA_ARGS__)
#  define terr(fmt, ...)  _err(fmt, ## __VA_ARGS__)
#else
#  define tinfo(fmt, ...)
#  define terr(fmt, ...)
#endif

#define ALIGNUP(x, a)  (((x) + ((1 << (a)) - 1)) & ~((1 << (a)) - 1))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents the state of one tile allocation */

struct tile_s
{
  uint8_t    log2tile;  /* Log base 2 of the size of one tile */
  uint16_t   ntiles;    /* The total number of (aligned) tiles in the heap */
  sem_t      exclsem;   /* For exclusive access to the AT */
  uintptr_t  heapstart; /* The aligned start of the tile heap */
  uint32_t   at;        /* Tile allocation table */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* State of the single TILE allocator */

extern FAR struct tile_s *g_tileinfo;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tile_enter_critical and tile_leave_critical
 *
 * Description:
 *   Critical section management for the tile allocator.
 *
 * Input Parameters:
 *   priv - Pointer to the tile state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tile_enter_critical(FAR struct tile_s *priv);
void tile_leave_critical(FAR struct tile_s *priv);

#endif /* __MODULES_ASMP_MM_MM_TILE_H */
