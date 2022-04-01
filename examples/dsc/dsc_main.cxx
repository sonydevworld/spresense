/****************************************************************************
 * dsc/dsc_main.cxx
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "nx_backend.h"
#include "font_draw.h"
#include "camera_ctrl.h"
#include "input_device.h"

#include "dsc_file.h"
#include "dsc_appmenu.h"
#include "bitmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BITMAP_FILENAME  "/mnt/spif/lcd_bitmap.bmp"

#define APP_MODE_SHOOTER  (0)
#define APP_MODE_MENU     (1)

#define NOCARD_MSG  "NoCard"
#define NOCARD_SHOW_CYCLE   (15)
#define NOCARD_TOTAL_CYCLE  (25)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct app_param_t
{
  int vfd;
  unsigned char *fb;
  int fb_size;
  int lcd_w;
  int lcd_h;
  const char *sensorname;
  int app_mode;
  menu_system *menu;
  volatile bool is_card_inserted;
  int nocard_fcnt;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct app_param_t g_appinst;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * name: card_detection_cb
 ****************************************************************************/

static void card_detection_cb(bool injected)
{
  if (injected)
    {
      g_appinst.is_card_inserted = check_sdcard_content();
    }
  else
    {
      g_appinst.is_card_inserted = false;
    }
}

/****************************************************************************
 * name: draw_cardstatus
 ****************************************************************************/

static void draw_cardstatus(unsigned char *img)
{
  int msg_len;

  if (!g_appinst.is_card_inserted)
    {
      if (g_appinst.nocard_fcnt < NOCARD_SHOW_CYCLE)
        {
          msg_len = strlen(NOCARD_MSG);
          print_borderingfont(NOCARD_MSG, img, g_appinst.lcd_w, g_appinst.lcd_h,
                        g_appinst.lcd_w - (msg_len * get_drawfontwidth()),
                        g_appinst.lcd_h - get_drawfontheight(),
                        FONT_COLOR_RED, FONT_COLOR_WHITE);
        }
    }

  g_appinst.nocard_fcnt++;
  if (g_appinst.nocard_fcnt >= NOCARD_TOTAL_CYCLE)
    {
      g_appinst.nocard_fcnt = 0;
    }
}

/****************************************************************************
 * name: draw_topline
 ****************************************************************************/

static void draw_topline(unsigned char *img, const char *msg,
                           int pos, int fgc, int bgc)
{
  print_borderingfont(msg, img, g_appinst.lcd_w, g_appinst.lcd_h,
                pos, 0, fgc, bgc);
}

/****************************************************************************
 * name: draw_shooting_info
 ****************************************************************************/

static void draw_shooting_info(unsigned char *img)
{
  int fwidth = get_drawfontwidth();

  draw_topline(img, "HDR ", 0, FONT_COLOR_BLACK,
                 is_hdr_en() ? FONT_COLOR_WHITE : FONT_COLOR_DRKGRAY);

  draw_topline(img, "AW ", 4 * fwidth, FONT_COLOR_BLACK,
                 is_autowb_en() ? FONT_COLOR_WHITE : FONT_COLOR_DRKGRAY);

  draw_topline(img, "AISO ", 7 * fwidth, FONT_COLOR_BLACK,
                 is_autoiso_en() ? FONT_COLOR_WHITE : FONT_COLOR_DRKGRAY);

  draw_topline(img, jpeg_size(), 12 * fwidth, FONT_COLOR_BLACK,
                 FONT_COLOR_WHITE);
}

/****************************************************************************
 * name: taking_picture
 ****************************************************************************/

static void taking_picture(unsigned char **img)
{
  int jpg_size;
  unsigned char *jpg_buf = g_appinst.fb;

  // Update LCD display to indicate shutter action

  memset(*img, 0, g_appinst.lcd_w * g_appinst.lcd_h * 2);
  nximage_draw(*img);

  // Take actual picture

  printf("Taking picture size (%dx%d)\n", jpeg_width(), jpeg_height());
  jpg_size = take_pictureimage(g_appinst.vfd, &jpg_buf, g_appinst.fb_size,
                              jpeg_width(), jpeg_height());
  if (jpg_size > 0)
    {
      file_writeimage((uint8_t *)jpg_buf, (size_t)jpg_size);
    }

  // Get preview picture again

  start_streaming(g_appinst.vfd, g_appinst.fb, g_appinst.fb_size);
  get_previewimage(g_appinst.vfd, img);
}

/****************************************************************************
 * name: action_in_shootermode
 ****************************************************************************/

static int action_in_shootermode(unsigned char **img, int key)
{
  int ret = APP_MODE_SHOOTER;

  switch (key)
    {
      case MENU_KEY_SELECT:
        if (g_appinst.is_card_inserted)
          {
            taking_picture(img);
          }
        break;

      case MENU_KEY_DOWN:
        ret = APP_MODE_MENU;
        break;
    }

  draw_shooting_info(*img);

  return ret;
}

/****************************************************************************
 * name: action_in_menumode
 ****************************************************************************/

static int action_in_menumode(unsigned char *img, int key)
{
  int ret = APP_MODE_MENU;

  if (g_appinst.menu->input_key(key) == MENU_EXEC_END)
    {
      ret = APP_MODE_SHOOTER;
    }
  g_appinst.menu->set_canvas(img);
  g_appinst.menu->draw_menu();

  return ret;
}

/****************************************************************************
 * name: app_action
 ****************************************************************************/

static int app_action(unsigned char **img)
{
  int key;

  key = inputdev_update();

  switch (g_appinst.app_mode)
    {
      case APP_MODE_SHOOTER:
        g_appinst.app_mode = action_in_shootermode(img, key);
        break;

      case APP_MODE_MENU:
        g_appinst.app_mode = action_in_menumode(*img, key);
        break;

      default:
        g_appinst.app_mode = APP_MODE_SHOOTER;
        break;
    }

  return key;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * name: main
 ****************************************************************************/

extern "C" int main(void)
{
  int key;
  unsigned char *img;

  // Initial mode is SHOOTER mode

  g_appinst.app_mode = APP_MODE_SHOOTER;

  // Initialize card status and detection

  board_ioctl(BOARDIOC_SDCARD_SETNOTIFYCB, (uintptr_t)card_detection_cb);
  g_appinst.is_card_inserted = file_initialize();
  g_appinst.nocard_fcnt = 0;

  // Initialize NX graphics sub-system

  if (nximage_initialize(&g_appinst.lcd_w, &g_appinst.lcd_h) < 0)
    {
      printf("NX init failed\n");
      return -1;
    }

  // Initialize Spresense Camera

  g_appinst.sensorname = initialize_cameractrl(&g_appinst.vfd);
  if (g_appinst.sensorname == NULL)
    {
      nximage_finalize();
      printf("Couldn't init camera\n");
      return -1;
    }

  // Allocate enough frame buffer memory for this application

  g_appinst.fb = camera_framebuffer(&g_appinst.fb_size);
  if (g_appinst.fb == NULL)
    {
      nximage_finalize();
      printf("Couldn't Allocate camera memory\n");
      return -1;
    }

  // Check if the memory is really enough

  if (g_appinst.fb_size < (g_appinst.lcd_w * g_appinst.lcd_h * 2 * 2))
    {
      free(g_appinst.fb);
      finalize_cameractrl(g_appinst.vfd);
      nximage_finalize();
      return -1;
    }

  // Start camera preview image streaming

  start_streaming(g_appinst.vfd, g_appinst.fb, g_appinst.fb_size);

  // Input device and menu system initialization

  inputdev_initialize();
  g_appinst.menu = create_menu(g_appinst.vfd, g_appinst.sensorname,
                               g_appinst.lcd_w, g_appinst.lcd_h);

  // Main Loop ....

  while (1)
    {
      get_previewimage(g_appinst.vfd, &img);
      key = app_action(&img);
      draw_cardstatus(img);
      nximage_draw(img);

      if (key == MENU_KEY_LEFT)
        {
          /* Left menu key code is just used as a capture key
           * to capture and save drawing picture on the LCD display
           * to the file. This function can be used for debugging
           * or for getting a picture of the documentat of this app ;-)
           */

          write_bmp(BITMAP_FILENAME, (unsigned short *)img,
                                     g_appinst.lcd_h, g_appinst.lcd_w);
        }

      release_previewimage(g_appinst.vfd, img);
    }

  // Finalization

  inputdev_finalize();
  finalize_cameractrl(g_appinst.vfd);
  free(g_appinst.fb);
  nximage_finalize();

  file_finalize();
  board_ioctl(BOARDIOC_SDCARD_SETNOTIFYCB, 0);

  return 0;
}
