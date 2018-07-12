/****************************************************************************
 * camera/camera_bkgd.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "nximage.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nximage_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool more, FAR void *arg);
static void nximage_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg);
#ifdef CONFIG_NX_XYINPUT
static void nximage_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg);
#endif

#ifdef CONFIG_NX_KBD
static void nximage_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Background window call table */

const struct nx_callback_s g_nximagecb =
{
  nximage_redraw,   /* redraw */
  nximage_position  /* position */
#ifdef CONFIG_NX_XYINPUT
  , nximage_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , nximage_kbdin   /* my kbdin */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nximage_redraw
 *
 * Description:
 *   NX re-draw handler
 *
 ****************************************************************************/

static void nximage_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool more, FAR void *arg)
{
  ginfo("hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
         hwnd, rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
         more ? "true" : "false");
}

/****************************************************************************
 * Name: nximage_position
 *
 * Description:
 *   NX position change handler
 *
 ****************************************************************************/

static void nximage_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg)
{
  /* Report the position */

  ginfo("hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
        hwnd, size->w, size->h, pos->x, pos->y,
        bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

  /* Have we picked off the window bounds yet? */

  if (!g_nximage.havepos)
    {
      /* Save the background window handle */

      g_nximage.hbkgd = hwnd;

      /* Save the window limits */

      g_nximage.xres = bounds->pt2.x + 1;
      g_nximage.yres = bounds->pt2.y + 1;

      g_nximage.havepos = true;
      sem_post(&g_nximage.sem);
      ginfo("Have xres=%d yres=%d\n", g_nximage.xres, g_nximage.yres);
    }
}

/****************************************************************************
 * Name: nximage_mousein
 *
 * Description:
 *   NX mouse input handler
 *
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
static void nximage_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg)
{
  printf("nximage_mousein: hwnd=%p pos=(%d,%d) button=%02x\n",
         hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nximage_kbdin
 *
 * Description:
 *   NX keyboard input handler
 *
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nximage_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg)
{
  ginfo("hwnd=%p nch=%d\n", hwnd, nch);

   /* In this example, there is no keyboard so a keyboard event is not
    * expected.
    */

   printf("nximage_kbdin: Unexpected keyboard callback\n");
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nximage_image
 *
 * Description:
 *   Put the NuttX logo in the center of the display.
 *
 ****************************************************************************/

void nximage_image(NXWINDOW hwnd, FAR const void *image)
{
  FAR struct nxgl_point_s origin;
  FAR struct nxgl_rect_s dest;
  FAR const void *src[CONFIG_NX_NPLANES];
  int ret;

  origin.x = 0;
  origin.y = 0;

  /* Set up the destination to whole LCD screen */

  dest.pt1.x = 0;
  dest.pt1.y = 0;
  dest.pt2.x = g_nximage.xres - 1;
  dest.pt2.y = g_nximage.yres - 1;

  src[0] = image;

  ret = nx_bitmap((NXWINDOW)hwnd, &dest, src, &origin,
                  g_nximage.xres * sizeof(nxgl_mxpixel_t));
  if (ret < 0)
    {
      printf("nximage_image: nx_bitmapwindow failed: %d\n", errno);
    }
}
