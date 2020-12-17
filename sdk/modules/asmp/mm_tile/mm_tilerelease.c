/****************************************************************************
 * modules/asmp/mm_tile/mm_tilerelease.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include <mm/tile.h>
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
 * Name: tile_release_common
 *
 * Description:
 *   Perform common TILE initialization.
 *
 * Input Parameters:
 *   priv - Reference to the tile heap structure to be released.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void tile_release_common(FAR struct tile_s *priv)
{
  DEBUGASSERT(priv);
  sem_destroy(&priv->exclsem);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tile_release
 *
 * Description:
 *   Uninitialize a gram memory allocator and release resources held by the
 *   allocator.
 *
 * Input Parameters:
 *   handle - The handle previously returned by tile_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tile_release(void)
{
  if (g_tileinfo)
    {
      tile_release_common(g_tileinfo);
      g_tileinfo = NULL;
    }
}

#endif /* CONFIG_MM_TILE */
