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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <arch/board/cxd56_imageproc.h>

#include "camera_bkgd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56_IMAGEPROC
#error "CXD56_IMAGEPROC configuration must be selected for this applicatoin."
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nximage_data_s
{
  /* The NX handles */

  NXHANDLE hnx;
  NXHANDLE hbkgd;
  bool     connected;

  /* The screen resolution */

  nxgl_coord_t xres;
  nxgl_coord_t yres;

  volatile bool havepos;
  sem_t sem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nximage_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool more, FAR void *arg);
static void nximage_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Background window call table */

static const struct nx_callback_s g_nximagecb =
{
  nximage_redraw,   /* redraw */
  nximage_position  /* position */
#ifdef CONFIG_NX_XYINPUT
  , NULL            /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , NULL            /* my kbdin */
#endif
};


/* To handle nx context, below variable is defined for this application. */

static struct nximage_data_s g_nximage =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  false,         /* connected */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nximage_listener()
 *
 * Description:
 *   NX event listener for an event from NX server.
 ****************************************************************************/

FAR void *nximage_listener(FAR void *arg)
{
  int ret;

  /* Process events forever */

  for (;;)
    {
      /* Handle the next event.  If we were configured blocking, then
       * we will stay right here until the next event is received.  Since
       * we have dedicated a while thread to servicing events, it would
       * be most natural to also select CONFIG_NX_BLOCKING -- if not, the
       * following would be a tight infinite loop (unless we added addition
       * logic with nx_eventnotify and sigwait to pace it).
       */

      ret = nx_eventhandler(g_nximage.hnx);
      if (ret < 0)
        {
          /* An error occurred... assume that we have lost connection with
           * the server.
           */

          printf("nximage_listener: Lost server connection: %d\n", errno);
          exit(EXIT_FAILURE);
        }

      /* If we received a message, we must be connected */

      if (!g_nximage.connected)
        {
          g_nximage.connected = true;
          sem_post(&g_nximage.sem);
          printf("nximage_listener: Connected\n");
        }
    }
}

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nximage_initialize
 *
 * Description:
 *   Initialize NX graphics subsystem.
 *
 ****************************************************************************/

int nximage_initialize(void)
{
  nxgl_mxpixel_t color;
  pthread_t thread;
  int ret;

  /* Start the NX server kernel thread */

  ret = boardctl(BOARDIOC_NX_START, 0);
  if (ret < 0)
    {
      printf("nximage_initialize: Failed to start the NX server: %d\n", errno);
      return ERROR;
    }

  /* Connect to the server */

  g_nximage.hnx = nx_connect();
  if (!g_nximage.hnx)
    {
      printf("nximage_initialize: nx_connect failed: %d\n", errno);
      return ERROR;
    }

  /* Start a separate thread to listen for server events.
     For simplicity, use defaul thread attribute.
   */

  ret = pthread_create(&thread, NULL, nximage_listener, NULL);
  if (ret != 0)
    {
       printf("nximage_initialize: pthread_create failed: %d\n", ret);
       return ERROR;
    }

  /* Don't return until we are connected to the server */

  while (!g_nximage.connected)
    {
      /* Wait for the listener thread to wake us up when we really
       * are connected.
       */

      (void)sem_wait(&g_nximage.sem);
    }

  /* Set background color to black */

  color = 0;
  nx_setbgcolor(g_nximage.hnx, &color);
  ret = nx_requestbkgd(g_nximage.hnx, &g_nximagecb, NULL);
  if (ret < 0)
    {
      printf("nximage_initialize: nx_requestbkgd failed: %d\n", errno);
      nx_disconnect(g_nximage.hnx);
      return ERROR;
    }

  while (!g_nximage.havepos)
    {
      (void) sem_wait(&g_nximage.sem);
    }
  printf("nximage_initialize: Screen resolution (%d,%d)\n",
         g_nximage.xres, g_nximage.yres);

  /* Initialize imageproc for converting pixcel format from YUV to RGB */

  imageproc_initialize();

  return 0;
}

/****************************************************************************
 * Name: nximage_image
 *
 * Description:
 *   Put the NuttX logo in the center of the display.
 *
 ****************************************************************************/

void nximage_draw(FAR void *image, int w, int h)
{
  FAR struct nxgl_point_s origin;
  FAR struct nxgl_rect_s dest;
  FAR const void *src[CONFIG_NX_NPLANES];
  int ret;

  /* Convert YUV422 image to RGB565 image */

  imageproc_convert_yuv2rgb((void *)image, w, h);

  origin.x = 0;
  origin.y = 0;

  /* Set up the destination to whole LCD screen */

  dest.pt1.x = 0;
  dest.pt1.y = 0;
  dest.pt2.x = g_nximage.xres - 1;
  dest.pt2.y = g_nximage.yres - 1;

  src[0] = image;

  ret = nx_bitmap((NXWINDOW)g_nximage.hbkgd, &dest, src, &origin,
                  g_nximage.xres * sizeof(nxgl_mxpixel_t));
  if (ret < 0)
    {
      printf("nximage_image: nx_bitmapwindow failed: %d\n", errno);
    }
}

/****************************************************************************
 * Name: nximage_finalize()
 *
 * Description:
 *   Finalize NX server.
 ****************************************************************************/

void nximage_finalize(void)
{
  imageproc_finalize();
  nx_disconnect(g_nximage.hnx);
}

