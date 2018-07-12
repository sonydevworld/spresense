/****************************************************************************
 * camera/nximage.h
 *
 *   Copyright (C) 2011, 2015 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_EXAMPLES_NXIMAGE_NXIMAGE_H
#define __APPS_EXAMPLES_NXIMAGE_NXIMAGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct nximage_data_s
{
  /* The NX handles */

  NXHANDLE hnx;
  NXHANDLE hbkgd;

  /* The screen resolution */

  nxgl_coord_t xres;
  nxgl_coord_t yres;

  volatile bool havepos;
  sem_t sem;
  volatile int code;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* NXIMAGE state data */

extern struct nximage_data_s g_nximage;

/* NX callback vtables */

extern const struct nx_callback_s g_nximagecb;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Background window interfaces */

void nximage_image(NXWINDOW hwnd, FAR const void *image);

#endif /* __APPS_EXAMPLES_NXIMAGE_NXIMAGE_H */
