/****************************************************************************
 * dsc/dsc_appmenu.cxx
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
#include <nuttx/video/video.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "simple_menu.h"
#include "font_draw.h"

#include "camera_extctl.h"
#include "camera_jpgsize.h"

#include "dsc_appmenu.h"

//***************************************************************************
// Pre-processor Definitions
//***************************************************************************

#define MENU_MORE_MSG "more"
#define ITEM_DRAW_START_XPOS(fw)   ((fw) * 4)
#define MSG_DRAW_START_XPOS(fw)    ((fw) * 8)
#define ITEM_DRAW_START_YOFFSET    (3)
#define ITEM_DRAW_START_YPOS(l,fh) ((ITEM_DRAW_START_YOFFSET + (l)) * (fh))

//***************************************************************************
// Private Classes
//***************************************************************************

class lcd_drawer : public display_menu
{
  private:
    const char *sensor_name;
    unsigned char *canvas;
    int lcd_width;
    int lcd_height;

  public:
    lcd_drawer() : display_menu(5), canvas(NULL) {};
    virtual void draw_title(menu *m);
    virtual void draw_tail(menu *m);
    virtual void draw_menuitem(menu_item *itm, int line, bool sel);
    void set_sensorname(const char *name) { sensor_name = name; };
    void set_canvas(unsigned char *cvs) { canvas = cvs; };
    void set_canvassize(int w, int h)
    {
      lcd_width = w;
      lcd_height = h;
    }
};

class lcd_menu_system : public menu_system
{
  private:
    lcd_drawer *lcd_drw;

  public:
    lcd_menu_system() : menu_system(), lcd_drw(NULL) {};

    void set_drawer(lcd_drawer *drw)
      {
        lcd_drw = drw;
        menu_system::set_drawer(drw);
      }

    void set_canvas(void *canvas)
      {
        if (lcd_drw)
          {
            lcd_drw->set_canvas((unsigned char *)canvas);
          }
      }
};

//***************************************************************************
// Private Data
//***************************************************************************

static menu top_menu("SprCam");

static struct camera_param_name colorfx_param[] =
{
  { V4L2_COLORFX_NONE,          "NON" },
  { V4L2_COLORFX_BW,            "BW " },
  { V4L2_COLORFX_SEPIA,         "SPA" },
  { V4L2_COLORFX_NEGATIVE,      "NEG" },
  { V4L2_COLORFX_EMBOSS,        "EMB" },
  { V4L2_COLORFX_SKETCH,        "SKT" },
  { V4L2_COLORFX_SKY_BLUE,      "SKY" },
  { V4L2_COLORFX_GRASS_GREEN,   "GRN" },
  { V4L2_COLORFX_SKIN_WHITEN,   "SKN" },
  { V4L2_COLORFX_VIVID,         "VIV" },
  { V4L2_COLORFX_AQUA,          "AQA" },
  { V4L2_COLORFX_ART_FREEZE,    "ART" },
  { V4L2_COLORFX_SILHOUETTE,    "SIL" },
  { V4L2_COLORFX_SOLARIZATION,  "SOL" },
  { V4L2_COLORFX_ANTIQUE,       "ANT" },
  { V4L2_COLORFX_SET_CBCR,      "CBC" },
  { V4L2_COLORFX_PASTEL,        "PST" },
};
#define COLORFX_PARAMSIZE  (sizeof(colorfx_param)/sizeof(colorfx_param[0]))

static camera_extctl colorfx_item("ColorFX",
                                  V4L2_CTRL_CLASS_USER, V4L2_CID_COLORFX,
                                  colorfx_param, COLORFX_PARAMSIZE, 0);

static struct camera_param_name onoff_param[] =
{
  { 0, "Off" },
  { 1, "On"  },
};
#define ONOFF_PARAMSIZE  (sizeof(onoff_param)/sizeof(onoff_param[0]))

static camera_extctl hdr_item("MShotHDR",
                      V4L2_CTRL_CLASS_CAMERA, V4L2_CID_WIDE_DYNAMIC_RANGE,
                      onoff_param, ONOFF_PARAMSIZE, 0);

static struct camera_param_name quality_param[] =
{
  { 10, "10" },
  { 20, "20" },
  { 30, "30" },
  { 40, "40" },
  { 50, "50" },
  { 60, "60" },
  { 70, "70" },
  { 80, "80" },
  { 90, "90" },
  { 100, "100" },
};
#define QUALITY_PARAMSIZE  (sizeof(quality_param)/sizeof(quality_param[0]))

static camera_extctl quality_item("JPGQuality",
                      V4L2_CTRL_CLASS_JPEG, V4L2_CID_JPEG_COMPRESSION_QUALITY,
                      quality_param, QUALITY_PARAMSIZE, 7);

static camera_extctl autowb_item("AutoWB",
                      V4L2_CTRL_CLASS_USER, V4L2_CID_AUTO_WHITE_BALANCE,
                      onoff_param, ONOFF_PARAMSIZE, 0);

static struct camera_param_name wbmode_param[] =
{
  { V4L2_WHITE_BALANCE_MANUAL,        "Mual" },
  { V4L2_WHITE_BALANCE_AUTO,          "Auto" },
  { V4L2_WHITE_BALANCE_INCANDESCENT,  "INCA" },
  { V4L2_WHITE_BALANCE_FLUORESCENT,   "FLUO" },
  { V4L2_WHITE_BALANCE_FLUORESCENT_H, "FLUH" },
  { V4L2_WHITE_BALANCE_HORIZON,       "HORZ" },
  { V4L2_WHITE_BALANCE_DAYLIGHT,      "DAY " },
  { V4L2_WHITE_BALANCE_FLASH,         "FLSH" },
  { V4L2_WHITE_BALANCE_CLOUDY,        "CLDY" },
  { V4L2_WHITE_BALANCE_SHADE,         "SHAD" },
};
#define WBMODE_PARAMSIZE  (sizeof(wbmode_param)/sizeof(wbmode_param[0]))

static camera_extctl wbmode_item("WBMode",
                      V4L2_CTRL_CLASS_CAMERA, V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
                      wbmode_param, WBMODE_PARAMSIZE, 0);

static struct camera_param_name autoiso_param[] =
{
  { V4L2_ISO_SENSITIVITY_MANUAL, "Mual" },
  { V4L2_ISO_SENSITIVITY_AUTO,   "Auto" },
};
#define AUTOISO_PARAMSIZE  (sizeof(autoiso_param)/sizeof(autoiso_param[0]))

static camera_extctl autoiso_item("AutoISO",
                      V4L2_CTRL_CLASS_CAMERA, V4L2_CID_ISO_SENSITIVITY_AUTO,
                      autoiso_param, AUTOISO_PARAMSIZE, 0);

static struct camera_param_name isosense_param[] =
{
  { 100,   "100" },
  { 200,   "200" },
  { 400,   "400" },
  { 800,   "800" },
  { 1600,  "1k6" },
  { 3200,  "3k2" },
  { 6400,  "6k4" },
  { 12800, "12k8" },
};
#define ISOSENSE_PARAMSIZE  (sizeof(isosense_param)/sizeof(isosense_param[0]))

static camera_extctl isosense_item("ISO",
                      V4L2_CTRL_CLASS_CAMERA, V4L2_CID_ISO_SENSITIVITY,
                      isosense_param, ISOSENSE_PARAMSIZE, 0);

static struct camera_pair_param_name jpgsize_param[] =
{
  {   96,   64, "SML" },
  {  160,  120, "QQV" },
  {  320,  240, "QVG" },
  {  640,  480, "VGA" },
  { 1280,  720, "HD " },
  { 1280,  960, "QAD" },
  { 1920, 1080, "FHD" },
  { 2048, 1536, "3M " },
  { 2560, 1920, "5M " },
};
#define JPGSIZE_PARAMSIZE  (sizeof(jpgsize_param)/sizeof(jpgsize_param[0]))

static camera_jpgsize jpgsize_item(jpgsize_param, JPGSIZE_PARAMSIZE, 3);

static endmenu_item end_item("ExitMenu");

static lcd_drawer  drawer;
static lcd_menu_system sys;

//***************************************************************************
// Class Impementations
//***************************************************************************

void lcd_drawer::draw_title(menu *m)
{
  const char *title = m->get_menutitle();
  int fwidth  = get_drawfontwidth();
  int fheight = get_drawfontheight();

  if (canvas)
    {
      print_borderingfont(title, canvas, lcd_width, lcd_height,
                     0, 0, FONT_COLOR_BLACK, FONT_COLOR_WHITE);

      print_borderingfont("with", canvas, lcd_width, lcd_height,
                     fwidth * (strlen(title) + 1), 0,
                     FONT_COLOR_BLACK, FONT_COLOR_WHITE);

      print_borderingfont(sensor_name, canvas, lcd_width, lcd_height,
                     fwidth * (strlen(title) + 5), 0,
                     FONT_COLOR_RED, FONT_COLOR_WHITE);

      if (m->get_displaypos() != 0)
        {
          print_borderingfont(MENU_MORE_MSG, canvas, lcd_width, lcd_height,
                        MSG_DRAW_START_XPOS(fwidth),
                        ITEM_DRAW_START_YPOS(-1, fheight),
                        FONT_COLOR_BLACK, FONT_COLOR_WHITE);

        }
    }
}

void lcd_drawer::draw_menuitem(menu_item *itm, int line, bool sel)
{
  int fwidth;
  int fheight;
  uint16_t fgcolor;
  uint16_t bgcolor;

  if (canvas)
    {
      if (sel)
        {
          fgcolor = FONT_COLOR_YELLOW;
          bgcolor = FONT_COLOR_RED;
        }
      else
        {
          fgcolor = FONT_COLOR_BLACK;
          bgcolor = FONT_COLOR_WHITE;
        }

      fwidth = get_drawfontwidth();
      fheight = get_drawfontheight();

      print_borderingfont(itm->get_itemname(), canvas, lcd_width, lcd_height,
                     ITEM_DRAW_START_XPOS(fwidth),
                     ITEM_DRAW_START_YPOS(line, fheight),
                     fgcolor, bgcolor);

      if (itm->get_valuename())
        {
            print_borderingfont(itm->get_valuename(), canvas,
                           lcd_width, lcd_height,
                           fwidth * 15, ITEM_DRAW_START_YPOS(line, fheight),
                           fgcolor, bgcolor);
        }
    }
}

void lcd_drawer::draw_tail(menu *m)
{
  int fwidth;
  int fheight;

  if (m->get_displaypos() + lines() < m->get_menunum())
    {
      fwidth  = get_drawfontwidth();
      fheight = get_drawfontheight();

      print_borderingfont(MENU_MORE_MSG, canvas, lcd_width, lcd_height,
                    MSG_DRAW_START_XPOS(fwidth),
                    ITEM_DRAW_START_YPOS(lines(), fheight),
                    FONT_COLOR_BLACK, FONT_COLOR_WHITE);

    }
}

//***************************************************************************
// Public Functions
//***************************************************************************

//***************************************************************************
// name: create_menu
//***************************************************************************

menu_system *create_menu(int fd, const char *sensor_name,
                         int lcd_w, int lcd_h)
{
  cam_menuitem::set_camfd(fd);

  top_menu.set_menuitem(&colorfx_item);
  top_menu.set_menuitem(&hdr_item);
  top_menu.set_menuitem(&quality_item);
  top_menu.set_menuitem(&autowb_item);
  top_menu.set_menuitem(&wbmode_item);
  top_menu.set_menuitem(&autoiso_item);
  top_menu.set_menuitem(&isosense_item);
  top_menu.set_menuitem(&jpgsize_item);
  top_menu.set_menuitem(&end_item);

  drawer.set_sensorname(sensor_name);
  drawer.set_canvassize(lcd_w, lcd_h);
  sys.set_drawer(&drawer);
  sys.set_menu(&top_menu);

  return &sys;
}

//***************************************************************************
// name: is_hdr_en
//***************************************************************************

bool is_hdr_en()
{
  return hdr_item.get_value() == 1;
}

//***************************************************************************
// name: is_autowb_en
//***************************************************************************

bool is_autowb_en()
{
  return autowb_item.get_value() == 1;
}

//***************************************************************************
// name: jpg_quality
//***************************************************************************

const char *jpg_quality()
{
  return quality_item.get_valuename();
}

//***************************************************************************
// name: is_autoiso_en
//***************************************************************************

bool is_autoiso_en()
{
  return autoiso_item.get_value() == 1;
}

//***************************************************************************
// name: jpeg_width
//***************************************************************************

int jpeg_width()
{
  return jpgsize_item.jpg_width();
}

//***************************************************************************
// name: jpeg_height
//***************************************************************************

int jpeg_height()
{
  return jpgsize_item.jpg_height();
}

//***************************************************************************
// name: jpeg_size
//***************************************************************************

const char *jpeg_size()
{
  return jpgsize_item.get_valuename();
}

